//! Stable operation-policy vocabulary for deterministic face tessellation.
//!
//! Compatibility v1 preserves the existing refinement ceilings while making
//! the output-sized allocation boundaries explicit. Derived index and task
//! scratch remains linearly bounded by admitted vertices and triangles. The
//! u32-addressable mesh-vertex ceiling is intentionally nonbinding for
//! existing inputs; the triangle ceiling is the existing 200,000-item
//! backstop.

use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, LimitSpec, ResourceKind, StageId,
};

use super::{MAX_BOUNDARY_DEPTH, MAX_REFINE_PASSES, MAX_TRIANGLES};

/// Inclusive number of mesh vertices addressable by u32 indices.
pub const FACE_TESSELLATION_U32_ITEM_LIMIT: u64 = u32::MAX as u64 + 1;

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

/// Cumulative work for accepted boundary midpoint splits.
pub const FACE_TESSELLATION_BOUNDARY_SPLITS: StageId = known_stage("kgeom.tess.boundary-splits");

/// Diagnostic identity for exhausting the boundary-split allowance.
pub const FACE_TESSELLATION_BOUNDARY_SPLIT_LIMIT: DiagnosticCode =
    known_diagnostic("kgeom.tess.boundary-splits-limit");

/// Cumulative-work stage for completed interior-refinement passes.
pub const FACE_TESSELLATION_REFINEMENT_PASSES: StageId =
    known_stage("kgeom.tess.interior-refinement-passes");

/// Diagnostic identity for requiring another interior-refinement pass.
pub const FACE_TESSELLATION_REFINEMENT_PASS_LIMIT: DiagnosticCode =
    known_diagnostic("kgeom.tess.interior-refinement-pass-limit");

/// High-water count of triangle allocations admitted during the invocation.
pub const FACE_TESSELLATION_MESH_TRIANGLES: StageId = known_stage("kgeom.tess.mesh-triangles");

/// Diagnostic identity for reaching the admitted-triangle ceiling.
pub const FACE_TESSELLATION_MESH_TRIANGLE_LIMIT: DiagnosticCode =
    known_diagnostic("kgeom.tess.mesh-triangles-limit");

/// Cumulative face-mesh vertex allocations admitted during the invocation.
pub const FACE_TESSELLATION_MESH_VERTICES: StageId = known_stage("kgeom.tess.mesh-vertices");

/// Diagnostic identity for exhausting the mesh-vertex allowance.
pub const FACE_TESSELLATION_MESH_VERTEX_LIMIT: DiagnosticCode =
    known_diagnostic("kgeom.tess.mesh-vertices-limit");

/// Version-1 deterministic budget profile for one face tessellation.
///
/// Requested mesh quality remains in [`super::TessOptions`]; this profile
/// uses compatibility-safe representability ceilings and the legacy path's
/// intended 200,000-triangle backstop. Contextual accounting now applies that
/// triangle ceiling to the initial earclip generation as well as refinement.
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
                FACE_TESSELLATION_BOUNDARY_SPLITS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                FACE_TESSELLATION_U32_ITEM_LIMIT,
            ),
            LimitSpec::new(
                FACE_TESSELLATION_REFINEMENT_PASSES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                MAX_REFINE_PASSES as u64,
            ),
            LimitSpec::new(
                FACE_TESSELLATION_MESH_TRIANGLES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                MAX_TRIANGLES as u64,
            ),
            LimitSpec::new(
                FACE_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                AccountingMode::Cumulative,
                FACE_TESSELLATION_U32_ITEM_LIMIT,
            ),
        ])
        .expect("built-in face-tessellation budget is valid")
    }

    /// Returns the finite version-1 profile derived from the certified face matrix.
    ///
    /// Each allowance is the next power of two at or above twice the
    /// corresponding measured maximum, clamped by the tessellator's existing
    /// local ceilings where necessary. The root allowance applies the same rule
    /// to the largest per-row sum of cumulative work stages.
    pub fn bounded_v1() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                FACE_TESSELLATION_BOUNDARY_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                16,
            ),
            LimitSpec::new(
                FACE_TESSELLATION_BOUNDARY_SPLITS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                512,
            ),
            LimitSpec::new(
                FACE_TESSELLATION_REFINEMENT_PASSES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                16,
            ),
            LimitSpec::new(
                FACE_TESSELLATION_MESH_TRIANGLES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                131_072,
            ),
            LimitSpec::new(
                FACE_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                AccountingMode::Cumulative,
                65_536,
            ),
        ])
        .expect("built-in bounded face-tessellation budget is valid")
        .with_total_work_limit(512)
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
                    FACE_TESSELLATION_BOUNDARY_SPLITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    1_u64 << 32,
                ),
                LimitSpec::new(
                    FACE_TESSELLATION_REFINEMENT_PASSES,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    24,
                ),
                LimitSpec::new(
                    FACE_TESSELLATION_MESH_TRIANGLES,
                    ResourceKind::Items,
                    AccountingMode::HighWater,
                    200_000,
                ),
                LimitSpec::new(
                    FACE_TESSELLATION_MESH_VERTICES,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    1_u64 << 32,
                ),
            ]
        );
        assert_eq!(profile.total_work_limit(), None);
    }

    #[test]
    fn bounded_v1_profile_is_an_exact_ordered_golden_contract() {
        let profile = FaceTessellationBudgetProfile::bounded_v1();

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
                    FACE_TESSELLATION_BOUNDARY_SPLITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    512,
                ),
                LimitSpec::new(
                    FACE_TESSELLATION_REFINEMENT_PASSES,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    16,
                ),
                LimitSpec::new(
                    FACE_TESSELLATION_MESH_TRIANGLES,
                    ResourceKind::Items,
                    AccountingMode::HighWater,
                    131_072,
                ),
                LimitSpec::new(
                    FACE_TESSELLATION_MESH_VERTICES,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    65_536,
                ),
            ]
        );
        assert_eq!(profile.total_work_limit(), Some(512));
        assert!(
            profile
                .limits()
                .iter()
                .all(|limit| limit.allowed < u64::MAX)
        );
    }

    #[test]
    fn identifiers_are_namespaced_unique_and_stable() {
        let stages = [
            FACE_TESSELLATION_BOUNDARY_DEPTH.as_str(),
            FACE_TESSELLATION_BOUNDARY_SPLITS.as_str(),
            FACE_TESSELLATION_REFINEMENT_PASSES.as_str(),
            FACE_TESSELLATION_MESH_TRIANGLES.as_str(),
            FACE_TESSELLATION_MESH_VERTICES.as_str(),
        ];
        let diagnostics = [
            FACE_TESSELLATION_BOUNDARY_DEPTH_LIMIT.as_str(),
            FACE_TESSELLATION_BOUNDARY_SPLIT_LIMIT.as_str(),
            FACE_TESSELLATION_REFINEMENT_PASS_LIMIT.as_str(),
            FACE_TESSELLATION_MESH_TRIANGLE_LIMIT.as_str(),
            FACE_TESSELLATION_MESH_VERTEX_LIMIT.as_str(),
        ];

        assert_eq!(
            stages,
            [
                "kgeom.tess.boundary-depth",
                "kgeom.tess.boundary-splits",
                "kgeom.tess.interior-refinement-passes",
                "kgeom.tess.mesh-triangles",
                "kgeom.tess.mesh-vertices",
            ]
        );
        assert_eq!(
            diagnostics,
            [
                "kgeom.tess.boundary-depth-limit",
                "kgeom.tess.boundary-splits-limit",
                "kgeom.tess.interior-refinement-pass-limit",
                "kgeom.tess.mesh-triangles-limit",
                "kgeom.tess.mesh-vertices-limit",
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
            .charge(FACE_TESSELLATION_BOUNDARY_SPLITS, 1_u64 << 32)
            .unwrap();
        ledger
            .charge(FACE_TESSELLATION_REFINEMENT_PASSES, 24)
            .unwrap();
        ledger
            .observe(
                FACE_TESSELLATION_MESH_TRIANGLES,
                ResourceKind::Items,
                200_000,
            )
            .unwrap();
        ledger
            .charge_resource(
                FACE_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                1_u64 << 32,
            )
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
            ledger.charge(FACE_TESSELLATION_BOUNDARY_SPLITS, 1),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.consumed == (1_u64 << 32) + 1
                    && snapshot.allowed == 1_u64 << 32
        ));
        assert!(matches!(
            ledger.charge(FACE_TESSELLATION_REFINEMENT_PASSES, 1),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.consumed == 25 && snapshot.allowed == 24
        ));
        assert!(matches!(
            ledger.observe(
                FACE_TESSELLATION_MESH_TRIANGLES,
                ResourceKind::Items,
                200_001,
            ),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.consumed == 200_001 && snapshot.allowed == 200_000
        ));
        assert!(matches!(
            ledger.charge_resource(
                FACE_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                1,
            ),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.consumed == (1_u64 << 32) + 1
                    && snapshot.allowed == 1_u64 << 32
        ));
    }

    #[test]
    fn bounded_v1_stage_allowances_are_inclusive() {
        let mut depth = WorkLedger::new(FaceTessellationBudgetProfile::bounded_v1());
        depth
            .observe(FACE_TESSELLATION_BOUNDARY_DEPTH, ResourceKind::Depth, 16)
            .unwrap();
        assert!(matches!(
            depth.observe(FACE_TESSELLATION_BOUNDARY_DEPTH, ResourceKind::Depth, 17),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.stage == FACE_TESSELLATION_BOUNDARY_DEPTH
                    && snapshot.consumed == 17
                    && snapshot.allowed == 16
        ));

        let mut splits = WorkLedger::new(FaceTessellationBudgetProfile::bounded_v1());
        splits
            .charge(FACE_TESSELLATION_BOUNDARY_SPLITS, 512)
            .unwrap();
        assert!(matches!(
            splits.charge(FACE_TESSELLATION_BOUNDARY_SPLITS, 1),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.stage == FACE_TESSELLATION_BOUNDARY_SPLITS
                    && snapshot.consumed == 513
                    && snapshot.allowed == 512
        ));

        let mut passes = WorkLedger::new(FaceTessellationBudgetProfile::bounded_v1());
        passes
            .charge(FACE_TESSELLATION_REFINEMENT_PASSES, 16)
            .unwrap();
        assert!(matches!(
            passes.charge(FACE_TESSELLATION_REFINEMENT_PASSES, 1),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.stage == FACE_TESSELLATION_REFINEMENT_PASSES
                    && snapshot.consumed == 17
                    && snapshot.allowed == 16
        ));

        let mut triangles = WorkLedger::new(FaceTessellationBudgetProfile::bounded_v1());
        triangles
            .observe(
                FACE_TESSELLATION_MESH_TRIANGLES,
                ResourceKind::Items,
                131_072,
            )
            .unwrap();
        assert!(matches!(
            triangles.observe(
                FACE_TESSELLATION_MESH_TRIANGLES,
                ResourceKind::Items,
                131_073,
            ),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.stage == FACE_TESSELLATION_MESH_TRIANGLES
                    && snapshot.consumed == 131_073
                    && snapshot.allowed == 131_072
        ));

        let mut vertices = WorkLedger::new(FaceTessellationBudgetProfile::bounded_v1());
        vertices
            .charge_resource(FACE_TESSELLATION_MESH_VERTICES, ResourceKind::Items, 65_536)
            .unwrap();
        assert!(matches!(
            vertices.charge_resource(
                FACE_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                1,
            ),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.stage == FACE_TESSELLATION_MESH_VERTICES
                    && snapshot.consumed == 65_537
                    && snapshot.allowed == 65_536
        ));
    }

    #[test]
    fn bounded_v1_root_work_allowance_is_inclusive() {
        let mut ledger = WorkLedger::new(FaceTessellationBudgetProfile::bounded_v1());
        ledger
            .charge(FACE_TESSELLATION_BOUNDARY_SPLITS, 496)
            .unwrap();
        ledger
            .charge(FACE_TESSELLATION_REFINEMENT_PASSES, 16)
            .unwrap();

        assert!(matches!(
            ledger.charge(FACE_TESSELLATION_BOUNDARY_SPLITS, 1),
            Err(OperationPolicyError::LimitReached(snapshot))
                if snapshot.stage == kcore::operation::TOTAL_WORK_STAGE
                    && snapshot.consumed == 513
                    && snapshot.allowed == 512
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
        assert_eq!(composed.unwrap().limits().len(), 6);
    }
}
