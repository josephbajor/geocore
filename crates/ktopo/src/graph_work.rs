//! Shared operation-scope accounting for topology-owned graph queries.
//!
//! This is public only for reviewed lower-layer adapters such as `kxt`.
//! Application code should enter through the `kernel` facade.

use crate::store::Store;
use kcore::operation::{
    AccountingMode, BudgetPlan, ChildWorkLedger, LimitSnapshot, LimitSpec, OperationPolicyError,
    OperationScope, ResourceKind, SequentialWorkLedger, TOTAL_WORK_STAGE,
};
use kcore::tolerance::Tolerances;
use kgraph::{EvalBudgetProfile, EvalContext, EvalLimits, EvalResult, EvalUsage};

/// Run one strictly sequential graph query against a shared operation scope.
///
/// The evaluator retains the standalone 64-depth/4,096-visit local cap while
/// accepted usage streams immediately into the caller's aggregate and root
/// limits. Every local, aggregate, or root N+1 crossing is normalized to an
/// operation-policy snapshot; callers may adapt that typed boundary for a
/// legacy API after the complete operation report has been retained.
pub fn query_sequential<T>(
    scope: &mut OperationScope<'_, '_>,
    store: &Store,
    query: impl FnOnce(&mut EvalContext<'_>) -> EvalResult<T>,
) -> core::result::Result<EvalResult<T>, OperationPolicyError> {
    let defaults = EvalLimits::default();
    scope.ledger().require_limit(
        kgraph::eval_stage::NODE_VISITS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
    )?;
    let snapshots = scope.ledger().snapshots();
    let depth = snapshot(
        &snapshots,
        kgraph::eval_stage::DEPENDENCY_DEPTH,
        ResourceKind::Depth,
    )?;
    let max_node_visits_per_query = usize::try_from(maximum_admissible_graph_visits(
        scope,
        defaults.max_node_visits_per_query as u64,
    )?)
    .map_err(|_| OperationPolicyError::AccountingOverflow {
        stage: kgraph::eval_stage::NODE_VISITS,
        resource: ResourceKind::Work,
    })?;
    let max_dependency_depth =
        usize::try_from(depth.allowed.min(defaults.max_dependency_depth as u64)).map_err(|_| {
            OperationPolicyError::AccountingOverflow {
                stage: depth.stage,
                resource: depth.resource,
            }
        })?;
    let tolerances = scope.context().tolerances();
    let mut ledger = scope
        .ledger_mut()
        .sequential(EvalBudgetProfile::v1_defaults())?;
    let mut evaluator = EvalContext::new(
        store.geometry(),
        EvalLimits {
            max_dependency_depth,
            max_node_visits_per_query,
        },
        tolerances,
    );
    let lower = query(&mut evaluator);
    let crossing = account_sequential_query(
        &mut ledger,
        evaluator.last_query_usage(),
        lower.as_ref().err(),
    )?;
    if let Some(snapshot) = crossing {
        return Err(OperationPolicyError::LimitReached(snapshot));
    }
    Ok(lower)
}

/// Return the largest query-local visit charge the parent can currently admit.
///
/// Using the ledger's read-only preflight keeps active child reservations and
/// a root work ceiling in the evaluator allowance. A bounded binary search is
/// sufficient because the standalone graph cap is only 4,096 visits.
fn maximum_admissible_graph_visits(
    scope: &OperationScope<'_, '_>,
    upper: u64,
) -> core::result::Result<u64, OperationPolicyError> {
    let mut accepted = 0_u64;
    let mut rejected = upper.saturating_add(1);
    while accepted + 1 < rejected {
        let candidate = accepted + (rejected - accepted) / 2;
        match scope
            .ledger()
            .check_charge(kgraph::eval_stage::NODE_VISITS, candidate)
        {
            Ok(()) => accepted = candidate,
            Err(OperationPolicyError::LimitReached(_)) => rejected = candidate,
            Err(error) => return Err(error),
        }
    }
    Ok(accepted)
}

/// One deterministic child reservation shared by a sequence of graph queries.
pub struct GraphQueryWork {
    child: ChildWorkLedger,
    limits: EvalLimits,
    tolerances: Tolerances,
}

impl GraphQueryWork {
    /// Reserve the remaining graph allowance from `scope` for one stable child.
    pub fn reserve(
        scope: &mut OperationScope<'_, '_>,
        child_ordinal: u64,
    ) -> core::result::Result<Self, OperationPolicyError> {
        let (plan, limits) = child_plan(scope)?;
        let tolerances = scope.context().tolerances();
        let child = scope.ledger_mut().reserve_child(child_ordinal, plan)?;
        Ok(Self {
            child,
            limits,
            tolerances,
        })
    }

    /// Run and account one graph query using the operation's tolerances.
    pub fn query<T>(
        &mut self,
        store: &Store,
        query: impl FnOnce(&mut EvalContext<'_>) -> EvalResult<T>,
    ) -> core::result::Result<EvalResult<T>, OperationPolicyError> {
        self.query_with_tolerances(store, self.tolerances, query)
    }

    /// Run and account one graph query with an explicitly derived tolerance.
    pub fn query_with_tolerances<T>(
        &mut self,
        store: &Store,
        tolerances: Tolerances,
        query: impl FnOnce(&mut EvalContext<'_>) -> EvalResult<T>,
    ) -> core::result::Result<EvalResult<T>, OperationPolicyError> {
        let snapshots = self.child.ledger().snapshots();
        let node = snapshot(
            &snapshots,
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
        )?;
        let mut remaining = node.allowed.saturating_sub(node.consumed);
        if let Some(total) = snapshots
            .iter()
            .find(|entry| entry.stage == TOTAL_WORK_STAGE && entry.resource == ResourceKind::Work)
        {
            remaining = remaining.min(total.allowed.saturating_sub(total.consumed));
        }
        let limits = EvalLimits {
            max_dependency_depth: self.limits.max_dependency_depth,
            max_node_visits_per_query: usize::try_from(remaining).map_err(|_| {
                OperationPolicyError::AccountingOverflow {
                    stage: node.stage,
                    resource: node.resource,
                }
            })?,
        };
        let mut evaluator = EvalContext::new(store.geometry(), limits, tolerances);
        let lower = query(&mut evaluator);
        account_query(
            &mut self.child,
            evaluator.last_query_usage(),
            lower.as_ref().err(),
        )?;
        Ok(lower)
    }

    /// Merge the completed child into the caller ledger at the join point.
    pub fn merge(
        self,
        scope: &mut OperationScope<'_, '_>,
    ) -> core::result::Result<(), OperationPolicyError> {
        scope.ledger_mut().merge_children(vec![self.child])
    }
}

fn child_plan(
    scope: &OperationScope<'_, '_>,
) -> core::result::Result<(BudgetPlan, EvalLimits), OperationPolicyError> {
    let ledger = scope.ledger();
    ledger.require_limit(
        kgraph::eval_stage::NODE_VISITS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
    )?;
    ledger.require_limit(
        kgraph::eval_stage::DEPENDENCY_DEPTH,
        ResourceKind::Depth,
        AccountingMode::HighWater,
    )?;
    let snapshots = ledger.snapshots();
    let node = snapshot(
        &snapshots,
        kgraph::eval_stage::NODE_VISITS,
        ResourceKind::Work,
    )?;
    let depth = snapshot(
        &snapshots,
        kgraph::eval_stage::DEPENDENCY_DEPTH,
        ResourceKind::Depth,
    )?;
    let remaining = node.allowed.saturating_sub(node.consumed);
    let mut plan = BudgetPlan::new([
        LimitSpec::new(
            node.stage,
            node.resource,
            AccountingMode::Cumulative,
            remaining,
        ),
        LimitSpec::new(
            depth.stage,
            depth.resource,
            AccountingMode::HighWater,
            depth.allowed,
        ),
    ])?;
    if let Some(total) = snapshots
        .iter()
        .find(|entry| entry.stage == TOTAL_WORK_STAGE && entry.resource == ResourceKind::Work)
    {
        plan =
            plan.with_total_work_limit(total.allowed.saturating_sub(total.consumed).min(remaining));
    }
    let limits = EvalLimits {
        max_dependency_depth: usize::try_from(depth.allowed).map_err(|_| {
            OperationPolicyError::AccountingOverflow {
                stage: depth.stage,
                resource: depth.resource,
            }
        })?,
        max_node_visits_per_query: usize::try_from(remaining).map_err(|_| {
            OperationPolicyError::AccountingOverflow {
                stage: node.stage,
                resource: node.resource,
            }
        })?,
    };
    Ok((plan, limits))
}

fn snapshot(
    snapshots: &[LimitSnapshot],
    stage: kcore::operation::StageId,
    resource: ResourceKind,
) -> core::result::Result<LimitSnapshot, OperationPolicyError> {
    snapshots
        .iter()
        .copied()
        .find(|entry| entry.stage == stage && entry.resource == resource)
        .ok_or(OperationPolicyError::UnknownLimit { stage, resource })
}

fn account_query(
    child: &mut ChildWorkLedger,
    usage: EvalUsage,
    failure: Option<&kgraph::EvalError>,
) -> core::result::Result<(), OperationPolicyError> {
    let visits = u64::try_from(usage.node_visits()).map_err(|_| {
        OperationPolicyError::AccountingOverflow {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: ResourceKind::Work,
        }
    })?;
    let depth = u64::try_from(usage.dependency_depth()).map_err(|_| {
        OperationPolicyError::AccountingOverflow {
            stage: kgraph::eval_stage::DEPENDENCY_DEPTH,
            resource: ResourceKind::Depth,
        }
    })?;
    child
        .ledger_mut()
        .charge(kgraph::eval_stage::NODE_VISITS, visits)?;
    child.ledger_mut().observe(
        kgraph::eval_stage::DEPENDENCY_DEPTH,
        ResourceKind::Depth,
        depth,
    )?;

    let Some(snapshot) = failure.and_then(kgraph::EvalError::limit) else {
        return Ok(());
    };
    let crossing = match snapshot.resource {
        ResourceKind::Work => child.ledger_mut().charge_resource(
            snapshot.stage,
            snapshot.resource,
            snapshot.consumed.saturating_sub(visits),
        ),
        ResourceKind::Depth => {
            child
                .ledger_mut()
                .observe(snapshot.stage, snapshot.resource, snapshot.consumed)
        }
        _ => {
            return Err(OperationPolicyError::UnknownLimit {
                stage: snapshot.stage,
                resource: snapshot.resource,
            });
        }
    };
    match crossing {
        Err(OperationPolicyError::LimitReached(actual)) if actual == snapshot => Ok(()),
        Err(error) => Err(error),
        Ok(()) => Err(OperationPolicyError::UnknownLimit {
            stage: snapshot.stage,
            resource: snapshot.resource,
        }),
    }
}

fn account_sequential_query(
    ledger: &mut SequentialWorkLedger<'_>,
    usage: EvalUsage,
    failure: Option<&kgraph::EvalError>,
) -> core::result::Result<Option<LimitSnapshot>, OperationPolicyError> {
    let visits = u64::try_from(usage.node_visits()).map_err(|_| {
        OperationPolicyError::AccountingOverflow {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: ResourceKind::Work,
        }
    })?;
    let depth = u64::try_from(usage.dependency_depth()).map_err(|_| {
        OperationPolicyError::AccountingOverflow {
            stage: kgraph::eval_stage::DEPENDENCY_DEPTH,
            resource: ResourceKind::Depth,
        }
    })?;
    ledger.charge(kgraph::eval_stage::NODE_VISITS, visits)?;
    ledger.observe(
        kgraph::eval_stage::DEPENDENCY_DEPTH,
        ResourceKind::Depth,
        depth,
    )?;
    let Some(snapshot) = failure.and_then(kgraph::EvalError::limit) else {
        return Ok(None);
    };
    let crossing = match snapshot.resource {
        ResourceKind::Work => ledger.charge_resource(snapshot.stage, snapshot.resource, 1),
        ResourceKind::Depth => ledger.observe(snapshot.stage, snapshot.resource, snapshot.consumed),
        _ => {
            return Err(OperationPolicyError::UnknownLimit {
                stage: snapshot.stage,
                resource: snapshot.resource,
            });
        }
    };
    match crossing {
        Err(OperationPolicyError::LimitReached(actual)) => Ok(Some(actual)),
        Err(error) => Err(error),
        Ok(()) => Err(OperationPolicyError::UnknownLimit {
            stage: snapshot.stage,
            resource: snapshot.resource,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::SurfaceGeom;
    use kcore::operation::{
        ExecutionPolicy, NumericalPolicy, OperationContext, PolicyVersion, SessionPolicy,
        SessionPrecision,
    };
    use kgeom::frame::Frame;
    use kgeom::surface::Plane;
    use kgraph::{OffsetSurfaceDescriptor, SurfaceClass};

    fn context_for(plan: BudgetPlan) -> (SessionPolicy, Tolerances) {
        (
            SessionPolicy::new(
                SessionPrecision::parasolid(),
                NumericalPolicy::v1(),
                ExecutionPolicy::Serial,
                plan,
                PolicyVersion::V1,
            ),
            Tolerances::default(),
        )
    }

    #[test]
    fn sequential_queries_restart_local_caps_and_accumulate_parent_usage() {
        let mut store = Store::new();
        let plane = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        let (session, tolerances) = context_for(EvalBudgetProfile::v1_defaults());
        let context = OperationContext::new(&session, tolerances).unwrap();
        let mut scope = OperationScope::new(&context);

        for _ in 0..2 {
            assert_eq!(
                query_sequential(&mut scope, &store, |eval| eval.surface_leaf_class(plane)),
                Ok(Ok(SurfaceClass::Plane))
            );
        }
        let visits = snapshot(
            &scope.ledger().snapshots(),
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
        )
        .unwrap();
        assert_eq!(visits.consumed, 2);
        assert!(scope.ledger().limit_events().is_empty());
    }

    #[test]
    fn tighter_parent_stage_and_root_fail_at_aggregate_n_plus_one_atomically() {
        let mut store = Store::new();
        let plane = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        for root in [false, true] {
            let override_plan = if root {
                BudgetPlan::new([LimitSpec::new(
                    kgraph::eval_stage::NODE_VISITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    10,
                )])
                .unwrap()
                .with_total_work_limit(1)
            } else {
                BudgetPlan::new([LimitSpec::new(
                    kgraph::eval_stage::NODE_VISITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    1,
                )])
                .unwrap()
            };
            let plan = EvalBudgetProfile::v1_defaults().overlaid(&override_plan);
            let (session, tolerances) = context_for(plan);
            let context = OperationContext::new(&session, tolerances).unwrap();
            let mut scope = OperationScope::new(&context);
            assert!(
                query_sequential(&mut scope, &store, |eval| {
                    eval.surface_leaf_class(plane)
                })
                .unwrap()
                .is_ok()
            );
            let expected_stage = if root {
                TOTAL_WORK_STAGE
            } else {
                kgraph::eval_stage::NODE_VISITS
            };
            assert_eq!(
                query_sequential(&mut scope, &store, |eval| eval.surface_leaf_class(plane)),
                Err(OperationPolicyError::LimitReached(LimitSnapshot {
                    stage: expected_stage,
                    resource: ResourceKind::Work,
                    consumed: 2,
                    allowed: 1,
                }))
            );
            let visits = snapshot(
                &scope.ledger().snapshots(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            )
            .unwrap();
            assert_eq!(visits.consumed, 1, "failed unit must not mutate usage");
            assert_eq!(scope.ledger().limit_events().len(), 1);
        }
    }

    #[test]
    fn local_depth_cap_remains_query_local_under_a_looser_parent() {
        let mut store = Store::new();
        let mut surface = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        for _ in 0..64 {
            surface = store
                .insert_surface(OffsetSurfaceDescriptor::new(surface, 1.0).into())
                .unwrap();
        }
        let parent = EvalBudgetProfile::v1_defaults().overlaid(
            &BudgetPlan::new([LimitSpec::new(
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                100,
            )])
            .unwrap(),
        );
        let (session, tolerances) = context_for(parent);
        let context = OperationContext::new(&session, tolerances).unwrap();
        let mut scope = OperationScope::new(&context);
        assert_eq!(
            query_sequential(&mut scope, &store, |eval| eval.surface_leaf_class(surface)),
            Err(OperationPolicyError::LimitReached(LimitSnapshot {
                stage: kgraph::eval_stage::DEPENDENCY_DEPTH,
                resource: ResourceKind::Depth,
                consumed: 65,
                allowed: 64,
            }))
        );
        let depth = snapshot(
            &scope.ledger().snapshots(),
            kgraph::eval_stage::DEPENDENCY_DEPTH,
            ResourceKind::Depth,
        )
        .unwrap();
        assert_eq!(depth.consumed, 64);
        assert_eq!(
            scope.ledger().limit_events(),
            &[LimitSnapshot {
                stage: kgraph::eval_stage::DEPENDENCY_DEPTH,
                resource: ResourceKind::Depth,
                consumed: 65,
                allowed: 64,
            }]
        );
    }

    #[test]
    fn sequential_query_preflight_respects_active_child_reservations() {
        let mut store = Store::new();
        let plane = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        let parent = EvalBudgetProfile::for_limits(64, 5).with_total_work_limit(5);
        let (session, tolerances) = context_for(parent);
        let context = OperationContext::new(&session, tolerances).unwrap();
        let mut scope = OperationScope::new(&context);
        let child_plan = EvalBudgetProfile::for_limits(64, 3);
        let mut child = scope.ledger_mut().reserve_child(1, child_plan).unwrap();
        child
            .ledger_mut()
            .charge(kgraph::eval_stage::NODE_VISITS, 3)
            .unwrap();

        for _ in 0..2 {
            assert_eq!(
                query_sequential(&mut scope, &store, |eval| eval.surface_leaf_class(plane)),
                Ok(Ok(SurfaceClass::Plane))
            );
        }
        assert_eq!(
            query_sequential(&mut scope, &store, |eval| eval.surface_leaf_class(plane)),
            Err(OperationPolicyError::LimitReached(LimitSnapshot {
                stage: kgraph::eval_stage::NODE_VISITS,
                resource: ResourceKind::Work,
                consumed: 3,
                allowed: 2,
            }))
        );
        assert_eq!(
            snapshot(
                &scope.ledger().snapshots(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            )
            .unwrap()
            .consumed,
            2
        );

        scope.ledger_mut().merge_children(vec![child]).unwrap();
        assert_eq!(scope.ledger().total_work_consumed(), 5);
    }
}
