//! Shared operation-scope accounting for topology-owned graph queries.
//!
//! This is public only for reviewed lower-layer adapters such as `kxt`.
//! Application code should enter through the `kernel` facade.

use crate::store::Store;
use kcore::operation::{
    AccountingMode, BudgetPlan, ChildWorkLedger, LimitSnapshot, LimitSpec, OperationPolicyError,
    OperationScope, ResourceKind, TOTAL_WORK_STAGE,
};
use kcore::tolerance::Tolerances;
use kgraph::{EvalContext, EvalLimits, EvalResult, EvalUsage};

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
