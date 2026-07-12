//! Stable operation-policy vocabulary for deterministic whole-body tessellation.
//!
//! Whole-body tessellation owns one shared operation scope across edge
//! discretization, patch preparation, face tessellation, graph evaluation,
//! surface projection, and output retention. This module defines and validates
//! that aggregate profile while compatibility defaults preserve legacy output.

use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, LimitSpec, OperationPolicyError, ResourceKind,
    StageId,
};
use kgeom::project::{ProjectionBudgetProfile, SURFACE_PROJECTION_QUERIES};
use kgeom::tess::{
    FACE_TESSELLATION_BOUNDARY_SPLITS, FACE_TESSELLATION_MESH_VERTICES,
    FACE_TESSELLATION_REFINEMENT_PASSES, FaceTessellationBudgetProfile,
};
use kgraph::EvalBudgetProfile;

use super::MAX_DEPTH;

const fn known_stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid built-in body-tessellation stage identifier"),
    }
}

const fn known_diagnostic(value: &'static str) -> DiagnosticCode {
    match DiagnosticCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in body-tessellation diagnostic identifier"),
    }
}

/// High-water stage for exact-edge curve-refinement depth.
pub const BODY_TESSELLATION_EDGE_DEPTH: StageId = known_stage("ktopo.body-tessellation.edge-depth");
/// Diagnostic identity for reaching the exact-edge refinement depth ceiling.
pub const BODY_TESSELLATION_EDGE_DEPTH_LIMIT_REACHED: DiagnosticCode =
    known_diagnostic("ktopo.body-tessellation.edge-depth-limit");
/// Cumulative accepted exact-edge refinement splits.
pub const BODY_TESSELLATION_EDGE_SPLITS: StageId =
    known_stage("ktopo.body-tessellation.edge-splits");
/// Diagnostic identity for exhausting exact-edge refinement split work.
pub const BODY_TESSELLATION_EDGE_SPLIT_LIMIT_REACHED: DiagnosticCode =
    known_diagnostic("ktopo.body-tessellation.edge-splits-limit");

/// High-water stage for surface iso/seam arc-refinement depth.
pub const BODY_TESSELLATION_ISO_ARC_DEPTH: StageId =
    known_stage("ktopo.body-tessellation.iso-arc-depth");
/// Diagnostic identity for reaching the iso/seam refinement depth ceiling.
pub const BODY_TESSELLATION_ISO_ARC_DEPTH_LIMIT_REACHED: DiagnosticCode =
    known_diagnostic("ktopo.body-tessellation.iso-arc-depth-limit");
/// Cumulative accepted surface iso/seam arc-refinement splits.
pub const BODY_TESSELLATION_ISO_ARC_SPLITS: StageId =
    known_stage("ktopo.body-tessellation.iso-arc-splits");
/// Diagnostic identity for exhausting surface iso/seam arc-refinement work.
pub const BODY_TESSELLATION_ISO_ARC_SPLIT_LIMIT_REACHED: DiagnosticCode =
    known_diagnostic("ktopo.body-tessellation.iso-arc-splits-limit");

/// Cumulative retained vertices in a whole-body mesh.
pub const BODY_TESSELLATION_MESH_VERTICES: StageId =
    known_stage("ktopo.body-tessellation.mesh-vertices");
/// Diagnostic identity for exhausting the `u32` mesh-vertex address space.
pub const BODY_TESSELLATION_MESH_VERTEX_LIMIT_REACHED: DiagnosticCode =
    known_diagnostic("ktopo.body-tessellation.mesh-vertices-limit");

/// Cumulative body-owned logical items materialized while preparing UV patches.
///
/// One item is one sequence slot in a raw/unwrapped `(uv, global-id)` chain,
/// arc, row, shifted loop, patch polygon, cleaned trim copy, or local/global
/// map. An intentional copy is a new item; ownership moves are not.
pub const BODY_TESSELLATION_PREPARED_PATCH_ITEMS: StageId =
    known_stage("ktopo.body-tessellation.prepared-patch-items");
/// Diagnostic identity for exhausting body-owned UV/patch preparation items.
pub const BODY_TESSELLATION_PREPARED_PATCH_ITEM_LIMIT_REACHED: DiagnosticCode =
    known_diagnostic("ktopo.body-tessellation.prepared-patch-items-limit");

/// Cumulative nondegenerate triangles retained for the whole-body result.
///
/// A triangle is charged once before its first body-owned output allocation.
/// Moving it through patch, face, and body aggregation does not recharge it.
pub const BODY_TESSELLATION_RETAINED_TRIANGLES: StageId =
    known_stage("ktopo.body-tessellation.retained-triangles");
/// Diagnostic identity for exhausting retained whole-body triangles.
pub const BODY_TESSELLATION_RETAINED_TRIANGLE_LIMIT_REACHED: DiagnosticCode =
    known_diagnostic("ktopo.body-tessellation.retained-triangles-limit");

/// Diagnostic identity for an ambiguous nested face root-work crossing.
pub const BODY_TESSELLATION_TOTAL_WORK_LIMIT_REACHED: DiagnosticCode =
    known_diagnostic("ktopo.body-tessellation.total-work-limit");

/// Inclusive legacy exact-edge refinement depth allowance.
pub const BODY_TESSELLATION_EDGE_DEPTH_LIMIT: u64 = MAX_DEPTH as u64;
/// Inclusive legacy iso/seam arc refinement depth allowance.
pub const BODY_TESSELLATION_ISO_ARC_DEPTH_LIMIT: u64 = MAX_DEPTH as u64;
/// Compatibility-v1 aggregate accepted split allowance per refinement family.
pub const BODY_TESSELLATION_SPLIT_LIMIT: u64 = u32::MAX as u64 + 1;
/// Inclusive number of vertices addressable by `u32` mesh indices.
pub const BODY_TESSELLATION_MESH_VERTEX_LIMIT: u64 = u32::MAX as u64 + 1;

/// Version-1 aggregate budget profile for one whole-body tessellation.
///
/// Cumulative child-family defaults are deliberately nonbinding here. A body
/// can tessellate multiple faces and make many sequential graph/projection
/// queries, so the face-refinement, graph-visit, and surface-query totals have
/// no truthful finite operation-wide ceiling yet. Their local algorithms keep
/// the existing finite per-patch/per-query caps (24 refinement passes, 4,096
/// graph visits, and one surface projection invocation). The graph aggregate
/// uses `usize::MAX` because graph visit accounting converts back to the
/// platform-sized evaluator limit. High-water limits compose truthfully and
/// therefore retain the exact child-family defaults. Body-owned edge and iso
/// splits use the u32 representability ceiling: every accepted split denotes
/// one prospective interior point in its refinement scratch.
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyTessellationBudgetProfile;

impl BodyTessellationBudgetProfile {
    /// Returns canonical whole-body family defaults without a root work cap.
    pub fn v1_defaults() -> BudgetPlan {
        let face = FaceTessellationBudgetProfile::v1_defaults().overlaid(
            &BudgetPlan::new([
                LimitSpec::new(
                    FACE_TESSELLATION_BOUNDARY_SPLITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    u64::MAX,
                ),
                LimitSpec::new(
                    FACE_TESSELLATION_REFINEMENT_PASSES,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    u64::MAX,
                ),
                LimitSpec::new(
                    FACE_TESSELLATION_MESH_VERTICES,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    u64::MAX,
                ),
            ])
            .expect("body face-tessellation aggregate override is valid"),
        );
        let graph = EvalBudgetProfile::v1_defaults().overlaid(
            &BudgetPlan::new([LimitSpec::new(
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                usize::MAX as u64,
            )])
            .expect("body graph-evaluation aggregate override is valid"),
        );
        let surface_projection = ProjectionBudgetProfile::surface_defaults().overlaid(
            &BudgetPlan::new([LimitSpec::new(
                SURFACE_PROJECTION_QUERIES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                u64::MAX,
            )])
            .expect("body surface-projection aggregate override is valid"),
        );

        BudgetPlan::new(
            [
                LimitSpec::new(
                    BODY_TESSELLATION_EDGE_DEPTH,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    BODY_TESSELLATION_EDGE_DEPTH_LIMIT,
                ),
                LimitSpec::new(
                    BODY_TESSELLATION_EDGE_SPLITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    BODY_TESSELLATION_SPLIT_LIMIT,
                ),
                LimitSpec::new(
                    BODY_TESSELLATION_ISO_ARC_DEPTH,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    BODY_TESSELLATION_ISO_ARC_DEPTH_LIMIT,
                ),
                LimitSpec::new(
                    BODY_TESSELLATION_ISO_ARC_SPLITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    BODY_TESSELLATION_SPLIT_LIMIT,
                ),
                LimitSpec::new(
                    BODY_TESSELLATION_MESH_VERTICES,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    BODY_TESSELLATION_MESH_VERTEX_LIMIT,
                ),
                LimitSpec::new(
                    BODY_TESSELLATION_PREPARED_PATCH_ITEMS,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    u64::MAX,
                ),
                LimitSpec::new(
                    BODY_TESSELLATION_RETAINED_TRIANGLES,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    u64::MAX,
                ),
            ]
            .into_iter()
            .chain(face.limits().iter().copied())
            .chain(graph.limits().iter().copied())
            .chain(surface_projection.limits().iter().copied()),
        )
        .expect("built-in body-tessellation budget is valid and collision-free")
    }
}

/// Validate the complete aggregate profile through a caller-selected budget view.
#[allow(
    dead_code,
    reason = "the policy slice lands before the contextual body-tessellation entry point"
)]
pub(crate) fn validate_body_tessellation_budget(
    mut require: impl FnMut(
        StageId,
        ResourceKind,
        AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError>,
) -> core::result::Result<(), OperationPolicyError> {
    for required in BodyTessellationBudgetProfile::v1_defaults().limits() {
        require(required.stage, required.resource, required.mode)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use kcore::operation::{
        ExecutionPolicy, NumericalPolicy, OperationContext, PolicyVersion, SessionPolicy,
        SessionPrecision,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::project::{
        CURVE_PROJECTION_CANDIDATES, CURVE_PROJECTION_HALVINGS, CURVE_PROJECTION_NEWTON_ITERATIONS,
        CURVE_PROJECTION_QUERIES, CURVE_PROJECTION_SAMPLES, SURFACE_PROJECTION_CANDIDATES,
        SURFACE_PROJECTION_HALVINGS, SURFACE_PROJECTION_NEWTON_ITERATIONS,
        SURFACE_PROJECTION_SAMPLES,
    };
    use kgeom::tess::{FACE_TESSELLATION_BOUNDARY_DEPTH, FACE_TESSELLATION_MESH_TRIANGLES};

    use super::*;

    fn limit(
        stage: StageId,
        resource: ResourceKind,
        mode: AccountingMode,
        allowed: u64,
    ) -> LimitSpec {
        LimitSpec::new(stage, resource, mode, allowed)
    }

    #[test]
    fn v1_profile_is_an_exact_ordered_golden_contract() {
        let profile = BodyTessellationBudgetProfile::v1_defaults();

        assert_eq!(
            profile.limits(),
            [
                limit(
                    SURFACE_PROJECTION_HALVINGS,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    30
                ),
                limit(
                    SURFACE_PROJECTION_CANDIDATES,
                    ResourceKind::Items,
                    AccountingMode::HighWater,
                    6
                ),
                limit(
                    SURFACE_PROJECTION_NEWTON_ITERATIONS,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    60
                ),
                limit(
                    SURFACE_PROJECTION_QUERIES,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    u64::MAX
                ),
                limit(
                    SURFACE_PROJECTION_SAMPLES,
                    ResourceKind::Items,
                    AccountingMode::HighWater,
                    625
                ),
                limit(
                    FACE_TESSELLATION_BOUNDARY_DEPTH,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    16
                ),
                limit(
                    FACE_TESSELLATION_BOUNDARY_SPLITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    u64::MAX
                ),
                limit(
                    FACE_TESSELLATION_REFINEMENT_PASSES,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    u64::MAX
                ),
                limit(
                    FACE_TESSELLATION_MESH_TRIANGLES,
                    ResourceKind::Items,
                    AccountingMode::HighWater,
                    200_000
                ),
                limit(
                    FACE_TESSELLATION_MESH_VERTICES,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    u64::MAX
                ),
                limit(
                    kgraph::eval_stage::DEPENDENCY_DEPTH,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    64
                ),
                limit(
                    kgraph::eval_stage::NODE_VISITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    usize::MAX as u64
                ),
                limit(
                    BODY_TESSELLATION_EDGE_DEPTH,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    16
                ),
                limit(
                    BODY_TESSELLATION_EDGE_SPLITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    1_u64 << 32
                ),
                limit(
                    BODY_TESSELLATION_ISO_ARC_DEPTH,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    16
                ),
                limit(
                    BODY_TESSELLATION_ISO_ARC_SPLITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    1_u64 << 32
                ),
                limit(
                    BODY_TESSELLATION_MESH_VERTICES,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    1_u64 << 32
                ),
                limit(
                    BODY_TESSELLATION_PREPARED_PATCH_ITEMS,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    u64::MAX
                ),
                limit(
                    BODY_TESSELLATION_RETAINED_TRIANGLES,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    u64::MAX
                ),
            ]
        );
        assert_eq!(profile.total_work_limit(), None);
    }

    #[test]
    fn profile_stages_and_diagnostics_are_sorted_unique_and_stable() {
        let profile = BodyTessellationBudgetProfile::v1_defaults();
        let stages = profile
            .limits()
            .iter()
            .map(|entry| entry.stage.as_str())
            .collect::<Vec<_>>();
        assert!(stages.windows(2).all(|pair| pair[0] < pair[1]));
        assert_eq!(stages.iter().copied().collect::<BTreeSet<_>>().len(), 19);

        let diagnostics = [
            BODY_TESSELLATION_EDGE_DEPTH_LIMIT_REACHED.as_str(),
            BODY_TESSELLATION_EDGE_SPLIT_LIMIT_REACHED.as_str(),
            BODY_TESSELLATION_ISO_ARC_DEPTH_LIMIT_REACHED.as_str(),
            BODY_TESSELLATION_ISO_ARC_SPLIT_LIMIT_REACHED.as_str(),
            BODY_TESSELLATION_MESH_VERTEX_LIMIT_REACHED.as_str(),
            BODY_TESSELLATION_PREPARED_PATCH_ITEM_LIMIT_REACHED.as_str(),
            BODY_TESSELLATION_RETAINED_TRIANGLE_LIMIT_REACHED.as_str(),
            BODY_TESSELLATION_TOTAL_WORK_LIMIT_REACHED.as_str(),
        ];
        assert_eq!(
            diagnostics,
            [
                "ktopo.body-tessellation.edge-depth-limit",
                "ktopo.body-tessellation.edge-splits-limit",
                "ktopo.body-tessellation.iso-arc-depth-limit",
                "ktopo.body-tessellation.iso-arc-splits-limit",
                "ktopo.body-tessellation.mesh-vertices-limit",
                "ktopo.body-tessellation.prepared-patch-items-limit",
                "ktopo.body-tessellation.retained-triangles-limit",
                "ktopo.body-tessellation.total-work-limit",
            ]
        );
        assert_eq!(diagnostics.into_iter().collect::<BTreeSet<_>>().len(), 8);
    }

    #[test]
    fn modes_and_nonbinding_aggregate_defaults_match_composition_contract() {
        let profile = BodyTessellationBudgetProfile::v1_defaults();
        assert!(profile.limits().iter().all(|entry| match entry.resource {
            ResourceKind::Work => entry.mode == AccountingMode::Cumulative,
            ResourceKind::Depth => entry.mode == AccountingMode::HighWater,
            ResourceKind::Bytes => false,
            ResourceKind::Items => true,
            _ => false,
        }));
        let allowed = |stage| {
            let entry = profile
                .limits()
                .iter()
                .find(|entry| entry.stage == stage)
                .expect("aggregate stage exists");
            assert_eq!(entry.mode, AccountingMode::Cumulative);
            entry.allowed
        };
        assert_eq!(allowed(FACE_TESSELLATION_REFINEMENT_PASSES), u64::MAX);
        assert_eq!(allowed(FACE_TESSELLATION_BOUNDARY_SPLITS), u64::MAX);
        assert_eq!(allowed(BODY_TESSELLATION_EDGE_SPLITS), 1_u64 << 32);
        assert_eq!(allowed(BODY_TESSELLATION_ISO_ARC_SPLITS), 1_u64 << 32);
        assert_eq!(allowed(FACE_TESSELLATION_MESH_VERTICES), u64::MAX);
        assert_eq!(allowed(BODY_TESSELLATION_PREPARED_PATCH_ITEMS), u64::MAX);
        assert_eq!(allowed(BODY_TESSELLATION_RETAINED_TRIANGLES), u64::MAX);
        assert_eq!(allowed(kgraph::eval_stage::NODE_VISITS), usize::MAX as u64);
        assert_eq!(allowed(SURFACE_PROJECTION_QUERIES), u64::MAX);
        for curve_stage in [
            CURVE_PROJECTION_QUERIES,
            CURVE_PROJECTION_SAMPLES,
            CURVE_PROJECTION_CANDIDATES,
            CURVE_PROJECTION_NEWTON_ITERATIONS,
            CURVE_PROJECTION_HALVINGS,
        ] {
            assert!(
                profile
                    .limits()
                    .iter()
                    .all(|entry| entry.stage != curve_stage)
            );
        }
    }

    #[test]
    fn validation_rejects_a_missing_limit_before_work_starts() {
        let profile = BodyTessellationBudgetProfile::v1_defaults();
        let incomplete = BudgetPlan::new(
            profile
                .limits()
                .iter()
                .copied()
                .filter(|entry| entry.stage != BODY_TESSELLATION_EDGE_DEPTH),
        )
        .unwrap();

        assert_eq!(
            validate_body_tessellation_budget(|stage, resource, mode| {
                incomplete.require_limit(stage, resource, mode)
            }),
            Err(OperationPolicyError::UnknownLimit {
                stage: BODY_TESSELLATION_EDGE_DEPTH,
                resource: ResourceKind::Depth,
            })
        );
    }

    #[test]
    fn validation_rejects_a_wrong_accounting_mode() {
        let wrong_mode = BodyTessellationBudgetProfile::v1_defaults().overlaid(
            &BudgetPlan::new([limit(
                BODY_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                BODY_TESSELLATION_MESH_VERTEX_LIMIT,
            )])
            .unwrap(),
        );

        assert_eq!(
            validate_body_tessellation_budget(|stage, resource, mode| {
                wrong_mode.require_limit(stage, resource, mode)
            }),
            Err(OperationPolicyError::AccountingModeMismatch {
                stage: BODY_TESSELLATION_MESH_VERTICES,
                resource: ResourceKind::Items,
            })
        );
    }

    #[test]
    fn family_session_request_overlays_preserve_complete_valid_profile() {
        let family = BodyTessellationBudgetProfile::v1_defaults();
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            BudgetPlan::new([limit(
                BODY_TESSELLATION_EDGE_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                12,
            )])
            .unwrap(),
            PolicyVersion::V1,
        );
        let request = BudgetPlan::new([limit(
            BODY_TESSELLATION_MESH_VERTICES,
            ResourceKind::Items,
            AccountingMode::Cumulative,
            1_000,
        )])
        .unwrap();
        let effective = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(family)
            .with_budget_overrides(request)
            .effective_budget();

        validate_body_tessellation_budget(|stage, resource, mode| {
            effective.require_limit(stage, resource, mode)
        })
        .unwrap();
        assert_eq!(
            effective
                .limits()
                .iter()
                .find(|entry| entry.stage == BODY_TESSELLATION_EDGE_DEPTH)
                .unwrap()
                .allowed,
            12
        );
        assert_eq!(
            effective
                .limits()
                .iter()
                .find(|entry| entry.stage == BODY_TESSELLATION_MESH_VERTICES)
                .unwrap()
                .allowed,
            1_000
        );
        assert_eq!(effective.limits().len(), 19);
    }
}
