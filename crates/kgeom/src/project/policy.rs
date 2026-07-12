//! Stable operation-policy vocabulary for deterministic closest-point projection.

use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, LimitSpec, ResourceKind, StageId,
};

use super::{
    CURVE_CANDIDATES, CURVE_SAMPLES, MAX_HALVINGS, MAX_ITER_CURVE, MAX_ITER_SURFACE,
    SURFACE_CANDIDATES, SURFACE_SAMPLES,
};

const fn known_stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid built-in projection stage identifier"),
    }
}

const fn known_diagnostic(value: &'static str) -> DiagnosticCode {
    match DiagnosticCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in projection diagnostic identifier"),
    }
}

/// Cumulative number of curve-projection queries in one scope.
pub const CURVE_PROJECTION_QUERIES: StageId = known_stage("kgeom.project.curve-queries");
/// High-water number of coarse samples evaluated by one curve query.
pub const CURVE_PROJECTION_SAMPLES: StageId = known_stage("kgeom.project.curve-samples");
/// High-water number of candidates polished by one curve query.
pub const CURVE_PROJECTION_CANDIDATES: StageId = known_stage("kgeom.project.curve-candidates");
/// High-water Newton iterations used by one curve candidate.
pub const CURVE_PROJECTION_NEWTON_ITERATIONS: StageId =
    known_stage("kgeom.project.curve-newton-iterations");
/// High-water backtracking halvings used by one curve Newton step.
pub const CURVE_PROJECTION_HALVINGS: StageId =
    known_stage("kgeom.project.curve-backtracking-halvings");

/// Cumulative number of surface-projection queries in one scope.
pub const SURFACE_PROJECTION_QUERIES: StageId = known_stage("kgeom.project.surface-queries");
/// High-water number of coarse samples evaluated by one surface query.
pub const SURFACE_PROJECTION_SAMPLES: StageId = known_stage("kgeom.project.surface-samples");
/// High-water number of candidates polished by one surface query.
pub const SURFACE_PROJECTION_CANDIDATES: StageId = known_stage("kgeom.project.surface-candidates");
/// High-water Newton iterations used by one surface candidate.
pub const SURFACE_PROJECTION_NEWTON_ITERATIONS: StageId =
    known_stage("kgeom.project.surface-newton-iterations");
/// High-water backtracking halvings used by one surface Newton step.
pub const SURFACE_PROJECTION_HALVINGS: StageId =
    known_stage("kgeom.project.surface-backtracking-halvings");

/// Diagnostic identity for any projection resource limit.
pub const PROJECTION_LIMIT_REACHED: DiagnosticCode =
    known_diagnostic("kgeom.project.limit-reached");

/// Version-1 deterministic budget profile for closest-point projection.
#[derive(Debug, Clone, Copy, Default)]
pub struct ProjectionBudgetProfile;

impl ProjectionBudgetProfile {
    /// Returns the exact ceilings of one legacy curve projection.
    pub fn curve_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                CURVE_PROJECTION_QUERIES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                1,
            ),
            LimitSpec::new(
                CURVE_PROJECTION_SAMPLES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                (CURVE_SAMPLES + 1) as u64,
            ),
            LimitSpec::new(
                CURVE_PROJECTION_CANDIDATES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                CURVE_CANDIDATES as u64,
            ),
            LimitSpec::new(
                CURVE_PROJECTION_NEWTON_ITERATIONS,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                MAX_ITER_CURVE as u64,
            ),
            LimitSpec::new(
                CURVE_PROJECTION_HALVINGS,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                MAX_HALVINGS as u64,
            ),
        ])
        .expect("built-in curve-projection budget is valid")
    }

    /// Returns compatibility accounting for an owner that may issue multiple
    /// sequential curve projections.
    ///
    /// Per-query high-water limits retain the exact legacy ceilings. The
    /// aggregate query count is intentionally non-binding until an owning
    /// corpus justifies a finite model-level cap; callers may replace it with
    /// an explicit request override.
    pub fn curve_aggregate_compatibility() -> BudgetPlan {
        BudgetPlan::new(Self::curve_defaults().limits().iter().map(|limit| {
            if limit.stage == CURVE_PROJECTION_QUERIES {
                LimitSpec::new(limit.stage, limit.resource, limit.mode, u64::MAX)
            } else {
                *limit
            }
        }))
        .expect("built-in aggregate curve-projection budget is valid")
    }

    /// Returns the exact ceilings of one legacy surface projection.
    pub fn surface_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                SURFACE_PROJECTION_QUERIES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                1,
            ),
            LimitSpec::new(
                SURFACE_PROJECTION_SAMPLES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                ((SURFACE_SAMPLES + 1) * (SURFACE_SAMPLES + 1)) as u64,
            ),
            LimitSpec::new(
                SURFACE_PROJECTION_CANDIDATES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                SURFACE_CANDIDATES as u64,
            ),
            LimitSpec::new(
                SURFACE_PROJECTION_NEWTON_ITERATIONS,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                MAX_ITER_SURFACE as u64,
            ),
            LimitSpec::new(
                SURFACE_PROJECTION_HALVINGS,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                MAX_HALVINGS as u64,
            ),
        ])
        .expect("built-in surface-projection budget is valid")
    }

    /// Returns both projection-family profiles for a shared caller scope.
    pub fn v1_defaults() -> BudgetPlan {
        BudgetPlan::new(
            Self::curve_defaults()
                .limits()
                .iter()
                .chain(Self::surface_defaults().limits())
                .copied(),
        )
        .expect("combined built-in projection budget is valid")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kcore::operation::{OperationPolicyError, WorkLedger};

    #[test]
    fn v1_profile_is_an_exact_ordered_contract() {
        let profile = ProjectionBudgetProfile::v1_defaults();
        let allowed = |stage| {
            profile
                .limits()
                .iter()
                .find(|limit| limit.stage == stage)
                .unwrap()
                .allowed
        };

        assert_eq!(allowed(CURVE_PROJECTION_QUERIES), 1);
        assert_eq!(allowed(CURVE_PROJECTION_SAMPLES), 65);
        assert_eq!(allowed(CURVE_PROJECTION_CANDIDATES), 8);
        assert_eq!(allowed(CURVE_PROJECTION_NEWTON_ITERATIONS), 50);
        assert_eq!(allowed(CURVE_PROJECTION_HALVINGS), 30);
        assert_eq!(allowed(SURFACE_PROJECTION_QUERIES), 1);
        assert_eq!(allowed(SURFACE_PROJECTION_SAMPLES), 625);
        assert_eq!(allowed(SURFACE_PROJECTION_CANDIDATES), 6);
        assert_eq!(allowed(SURFACE_PROJECTION_NEWTON_ITERATIONS), 60);
        assert_eq!(allowed(SURFACE_PROJECTION_HALVINGS), 30);
        assert_eq!(profile.total_work_limit(), None);
        assert_eq!(ProjectionBudgetProfile::curve_defaults().limits().len(), 5);
        assert_eq!(
            ProjectionBudgetProfile::curve_aggregate_compatibility()
                .limits()
                .len(),
            5
        );
        assert_eq!(
            ProjectionBudgetProfile::curve_aggregate_compatibility()
                .limits()
                .iter()
                .find(|limit| limit.stage == CURVE_PROJECTION_QUERIES)
                .unwrap()
                .allowed,
            u64::MAX
        );
        assert_eq!(
            ProjectionBudgetProfile::surface_defaults().limits().len(),
            5
        );
    }

    #[test]
    fn every_v1_allowance_is_inclusive_and_rejects_n_plus_one() {
        for limit in ProjectionBudgetProfile::v1_defaults().limits() {
            let mut ledger = WorkLedger::new(ProjectionBudgetProfile::v1_defaults());
            let accepted = match limit.mode {
                AccountingMode::Cumulative => ledger.charge(limit.stage, limit.allowed),
                AccountingMode::HighWater => {
                    ledger.observe(limit.stage, limit.resource, limit.allowed)
                }
            };
            assert_eq!(accepted, Ok(()), "{}", limit.stage.as_str());

            let rejected = match limit.mode {
                AccountingMode::Cumulative => ledger.charge(limit.stage, 1),
                AccountingMode::HighWater => {
                    ledger.observe(limit.stage, limit.resource, limit.allowed + 1)
                }
            };
            assert!(
                matches!(rejected, Err(OperationPolicyError::LimitReached(snapshot))
                    if snapshot.consumed == limit.allowed + 1
                        && snapshot.allowed == limit.allowed),
                "{}: {rejected:?}",
                limit.stage.as_str()
            );
        }
    }
}
