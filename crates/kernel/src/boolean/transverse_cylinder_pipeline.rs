//! Full-checked consumer of complete transverse finite-cylinder Section evidence.

use kcore::operation::OperationScope;

use super::curved_pipeline::{
    CurvedBooleanPipelineOutcome, CurvedBooleanPipelineRefusal, PipelineFailure, StageResult,
    adapt_operation, extract_cylinder_operand, mixed_boundary_failure, mixed_plan_failure,
    realize_certified_cylinder_pair_shell,
};
use super::cylinder_pair_boundary::prepare_cylinder_pair_boundary;
use super::mixed_shell_plan::cylinder_pair::{CylinderPairPlanError, plan_cylinder_pair_boundary};
use super::select::PlanarBooleanOperation;
use crate::BodyId;
use crate::section::section_bodies_in_scope;
use crate::session::PartEdit;

/// Arrange, truth-select, and realize one exact-nonparallel Cylinder pair.
///
/// The complete Section graph remains the geometry case distinction. The
/// cylinder-pair adapter proves every admitted fragment survives into exactly
/// one physical edge before the generic analytic-shell realization opens a
/// topology transaction.
pub(super) fn execute_transverse_cylinder_boolean(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: [BodyId; 2],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let first = extract_cylinder_operand(edit, bodies[0].clone(), 0, scope)?;
    let second = extract_cylinder_operand(edit, bodies[1].clone(), 1, scope)?;
    let graph = section_bodies_in_scope(&edit.as_part(), &bodies[0], &bodies[1], linear, scope)?;
    let prepared = prepare_cylinder_pair_boundary(
        &edit.as_part(),
        &graph,
        &bodies,
        [&first, &second],
        linear,
        scope,
    )
    .map_err(mixed_boundary_failure)?;
    let certified = plan_cylinder_pair_boundary(
        &edit.state.store,
        &graph,
        &prepared,
        adapt_operation(operation),
        scope,
    )
    .map_err(cylinder_pair_plan_failure)?;
    realize_certified_cylinder_pair_shell(edit, &certified, linear, scope)
}

fn cylinder_pair_plan_failure(error: CylinderPairPlanError) -> PipelineFailure {
    match error {
        CylinderPairPlanError::Selection(error) => {
            PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error))
        }
        CylinderPairPlanError::Plan(error) => mixed_plan_failure(error),
        CylinderPairPlanError::Execution(error) => PipelineFailure::Execution(error),
        CylinderPairPlanError::WorkCountOverflow => {
            PipelineFailure::Refused(CurvedBooleanPipelineRefusal::WorkCountOverflow)
        }
        CylinderPairPlanError::PhysicalIncidence(_)
        | CylinderPairPlanError::UnknownPlannedSectionFragment { .. }
        | CylinderPairPlanError::SectionEdgePayloadMismatch { .. }
        | CylinderPairPlanError::SectionFragmentCoverage { .. }
        | CylinderPairPlanError::SectionCarrierFacesMismatch { .. }
        | CylinderPairPlanError::SectionUseLineageMismatch { .. } => {
            PipelineFailure::Refused(CurvedBooleanPipelineRefusal::AssemblyContract(
                "transverse cylinder-pair complete-plan validation failed",
            ))
        }
    }
}
