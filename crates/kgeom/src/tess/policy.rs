//! Stable operation-policy vocabulary for deterministic face tessellation.
//!
//! This module records the version-1 ceilings already enforced by
//! [`super::tessellate`] without changing its control flow. Surface evaluations
//! and triangle output intentionally have no v1 budget entries: the existing
//! API does not bound trim-loop input size, and the triangle backstop applies
//! only after interior refinement is required. There is therefore no truthful
//! finite ceiling for either resource that preserves every currently accepted
//! call. Contextual accounting can add those stages once input and
//! nested-operation limits are defined.

use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, LimitSpec, ResourceKind, StageId,
};

use super::{MAX_BOUNDARY_DEPTH, MAX_REFINE_PASSES};

const fn known_stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid built-in face-tessellation stage identifier"),
    }
}

const fn known_diagnostic(value: &'static str) -> DiagnosticCode {
    match DiagnosticCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in face-tessellation diagnostic identifier"),
    }
}

/// High-water stage for boundary edge-refinement recursion depth.
pub const FACE_TESSELLATION_BOUNDARY_DEPTH: StageId = known_stage("kgeom.tess.boundary-depth");

/// Diagnostic identity for reaching the boundary-refinement depth ceiling.
pub const FACE_TESSELLATION_BOUNDARY_DEPTH_LIMIT: DiagnosticCode =
    known_diagnostic("kgeom.tess.boundary-depth-limit");

/// Cumulative-work stage for completed interior-refinement passes.
pub const FACE_TESSELLATION_REFINEMENT_PASSES: StageId =
    known_stage("kgeom.tess.interior-refinement-passes");

/// Diagnostic identity for requiring another interior-refinement pass.
pub const FACE_TESSELLATION_REFINEMENT_PASS_LIMIT: DiagnosticCode =
    known_diagnostic("kgeom.tess.interior-refinement-pass-limit");

/// Version-1 deterministic budget profile for one face tessellation.
///
/// Requested mesh quality remains in [`super::TessOptions`]; this profile
/// contains only resource ceilings already enforced by the legacy path.
#[derive(Debug, Clone, Copy, Default)]
pub struct FaceTessellationBudgetProfile;

impl FaceTessellationBudgetProfile {
    /// Returns the exact resource ceilings used by the current tessellator.
    pub fn v1_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                FACE_TESSELLATION_BOUNDARY_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                MAX_BOUNDARY_DEPTH as u64,
            ),
            LimitSpec::new(
                FACE_TESSELLATION_REFINEMENT_PASSES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                MAX_REFINE_PASSES as u64,
            ),
        ])
        .expect("built-in face-tessellation budget is valid")
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSpec, OperationPolicyError, ResourceKind, StageId,
        WorkLedger,
    };

    use super::*;

    #[test]
    fn v1_profile_is_an_exact_ordered_golden_contract() {
        let profile = FaceTessellationBudgetProfile::v1_defaults();

        assert_eq!(
            profile.limits(),
            [
                LimitSpec::new(
                    FACE_TESSELLATION_BOUNDARY_DEPTH,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    16,
                ),
                LimitSpec::new(
                    FACE_TESSELLATION_REFINEMENT_PASSES,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    24,
                ),
            ]
        );
        assert_eq!(profile.total_work_limit(), None);
    }

    #[test]
    fn identifiers_are_namespaced_unique_and_stable() {
        let stages = [
            FACE_TESSELLATION_BOUNDARY_DEPTH.as_str(),
            FACE_TESSELLATION_REFINEMENT_PASSES.as_str(),
        ];
        let diagnostics = [
            FACE_TESSELLATION_BOUNDARY_DEPTH_LIMIT.as_str(),
            FACE_TESSELLATION_REFINEMENT_PASS_LIMIT.as_str(),
        ];

        assert_eq!(
            stages,
            [
                "kgeom.tess.boundary-depth",
                "kgeom.tess.interior-refinement-passes",
            ]
        );
        assert_eq!(
            diagnostics,
            [
                "kgeom.tess.boundary-depth-limit",
                "kgeom.tess.interior-refinement-pass-limit",
            ]
        );

        let all = stages.into_iter().chain(diagnostics).collect::<Vec<_>>();
        assert!(all.iter().all(|id| id.starts_with("kgeom.tess.")));
        assert_eq!(
            all.iter().copied().collect::<BTreeSet<_>>().len(),
            all.len()
        );
    }

    #[test]
    fn v1_allowances_are_inclusive() {
        let mut ledger = WorkLedger::new(FaceTessellationBudgetProfile::v1_defaults());

        ledger
            .observe(FACE_TESSELLATION_BOUNDARY_DEPTH, ResourceKind::Depth, 16)
            .unwrap();
        ledger
            .charge(FACE_TESSELLATION_REFINEMENT_PASSES, 24)
            .unwrap();
        assert!(matches!(
            ledger.observe(
                FACE_TESSELLATION_BOUNDARY_DEPTH,
                ResourceKind::Depth,
                17,
            ),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.consumed == 17 && snapshot.allowed == 16
        ));
        assert!(matches!(
            ledger.charge(FACE_TESSELLATION_REFINEMENT_PASSES, 1),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.consumed == 25 && snapshot.allowed == 24
        ));
    }

    #[test]
    fn profile_composes_without_duplicate_stage_resources() {
        const CALLER_STAGE: StageId = match StageId::new("kgeom.tess.caller-work") {
            Ok(stage) => stage,
            Err(_) => panic!("valid test stage"),
        };

        let profile = FaceTessellationBudgetProfile::v1_defaults();
        let composed = BudgetPlan::new(profile.limits().iter().copied().chain([LimitSpec::new(
            CALLER_STAGE,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            1,
        )]));

        assert!(composed.is_ok());
        assert_eq!(composed.unwrap().limits().len(), 3);
    }
}
