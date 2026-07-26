//! Proof-bearing convex-planar/finite-cylinder Boolean pipeline.
//!
//! Complete Plane/Cylinder sections flow through the shared arrangement,
//! boundary-selection, shell-plan, and analytic materialization spine. Whole
//! endpoint-free rings are ordinary arrangement cuts; zero-cut results reuse
//! the same truth selection and copy a source only when its entire boundary is
//! retained. Certified support contact is admitted as a degenerate section
//! relation and grafted into the same shell plan. Incomplete evidence remains
//! an explicit typed refusal.

use kcore::operation::OperationScope;
use ktopo::transaction::{FullBodyCheck, Journal};

use super::boundary_select::{
    BoundarySelectionError, RegularizedBooleanOperation, SelectedOrientation,
    select_boundary_fragments,
};
use super::curved_realize::{
    realize_analytic_shell_inputs, realize_analytic_shell_region, realize_source_body_copies,
};
use super::curved_source::{
    CertifiedCylinderSource, CylinderSourceGap, CylinderSourceOutcome, extract_cylinder_source,
};
use super::curved_support_separation::{
    CertifiedAxialCapContact, ConvexHostCylinderSupportRelation,
    certify_convex_host_cylinder_support_relation, certify_strict_axial_cap_contact,
};
use super::cylinder_dispatch::CylinderOperandScan;
use super::extract::{
    ExtractedPlanarSourceBody, PlanarSourceExtractionError, PlanarSourceGap,
    PlanarSourceProofFailure, extract_planar_source_body,
};
use super::mixed_boundary::{MixedBoundaryError, prepare_mixed_bounded_arc_boundary};
use super::mixed_shell_plan::components::{
    MixedShellComponentError, mixed_shell_component_work, partition_prepared_mixed_shell_components,
};
use super::mixed_shell_plan::cylinder_pair::CertifiedCylinderPairPlan;
use super::mixed_shell_plan::materialize::{
    MixedShellMaterializationBlueprint, MixedShellMaterializationError, MixedShellScalarInputs,
    materialize_mixed_shell_component_inputs, prepare_mixed_shell_materialization,
};
use super::mixed_shell_plan::{
    MixedShellPlanError, arrange_projected_ring_hole_mixed_shell, complete_mixed_shell_plan,
    plan_mixed_shell,
};
use super::pipeline::PLANAR_BOOLEAN_REALIZATION_WORK;
use super::select::PlanarBooleanOperation;
use crate::BodyId;
use crate::error::{Error, Result};
use crate::operation::{BodyCheckReport, adapt_live_body_check};
use crate::section::{BodySectionGraph, SectionCompletion, section_bodies_in_scope};
use crate::session::PartEdit;

/// One curved result that survived Full checking and committed atomically.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CommittedCurvedBoolean {
    bodies: Vec<BodyId>,
    journal: Journal,
    full_checks: Vec<FullBodyCheck>,
}

impl CommittedCurvedBoolean {
    pub(super) fn new(
        bodies: Vec<BodyId>,
        journal: Journal,
        full_checks: Vec<FullBodyCheck>,
    ) -> Self {
        Self {
            bodies,
            journal,
            full_checks,
        }
    }

    pub(crate) fn into_parts(self) -> (Vec<BodyId>, Journal, Vec<FullBodyCheck>) {
        (self.bodies, self.journal, self.full_checks)
    }
}

/// Typed, non-persistent refusal from the curved pipeline.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum CurvedBooleanPipelineRefusal {
    PlanarSourceNotFastValid {
        operand: u8,
        report: BodyCheckReport,
    },
    PlanarSourceUnsupported {
        operand: u8,
        gap: PlanarSourceGap,
    },
    PlanarSourceUncertified {
        operand: u8,
        failure: PlanarSourceProofFailure,
    },
    CylinderSourceNotFullValid {
        operand: u8,
        report: BodyCheckReport,
    },
    CylinderSourceUnsupported {
        operand: u8,
        gap: CylinderSourceGap,
    },
    SectionIncomplete,
    ClassificationBoundaryContact,
    ClassificationIndeterminate {
        reason: &'static str,
    },
    Selection(BoundarySelectionError),
    ResultTopologyUnsupported,
    AssemblyContract(&'static str),
    FullTopologyFault {
        fault_count: usize,
    },
    FullProofRejected(Vec<FullBodyCheck>),
    WorkCountOverflow,
}

/// Complete internal outcome from the curved path.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum CurvedBooleanPipelineOutcome {
    ProvenEmpty,
    Committed(CommittedCurvedBoolean),
    Refused(CurvedBooleanPipelineRefusal),
}

#[derive(Debug)]
pub(super) enum PipelineFailure {
    Execution(Error),
    Refused(CurvedBooleanPipelineRefusal),
}

impl From<Error> for PipelineFailure {
    fn from(error: Error) -> Self {
        Self::Execution(error)
    }
}

impl From<kcore::error::Error> for PipelineFailure {
    fn from(error: kcore::error::Error) -> Self {
        Self::Execution(error.into())
    }
}

pub(super) type StageResult<T> = core::result::Result<T, PipelineFailure>;

/// Execute the curved stages inside the dispatcher-owned operation scope.
pub(crate) fn execute_curved_in_scope(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    left: BodyId,
    right: BodyId,
    cylinder_scan: CylinderOperandScan,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<CurvedBooleanPipelineOutcome> {
    match execute_stages(edit, operation, [left, right], cylinder_scan, linear, scope) {
        Ok(outcome) => Ok(outcome),
        Err(PipelineFailure::Execution(error)) => Err(error),
        Err(PipelineFailure::Refused(refusal)) => {
            Ok(CurvedBooleanPipelineOutcome::Refused(refusal))
        }
    }
}

fn execute_stages(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: [BodyId; 2],
    cylinder_scan: CylinderOperandScan,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    super::pipeline::validate_pipeline_budget(scope)?;
    let cylinder_mask = cylinder_scan.mask();
    if cylinder_mask == [true, true] {
        return match cylinder_scan.pair_axes_exactly_parallel() {
            Some(true) => super::parallel_cylinder_pipeline::execute_parallel_cylinder_boolean(
                edit, operation, bodies, linear, scope,
            ),
            Some(false) => {
                super::transverse_cylinder_pipeline::execute_transverse_cylinder_boolean(
                    edit, operation, bodies, linear, scope,
                )
            }
            None => refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported),
        };
    }
    let (planar_operand, cylinder_operand) = match cylinder_mask {
        [true, false] => (1_usize, 0_usize),
        [false, true] => (0_usize, 1_usize),
        _ => return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported),
    };

    let (planar_source, cylinder_source) = if cylinder_operand == 0 {
        let cylinder = extract_cylinder_operand(
            edit,
            bodies[cylinder_operand].clone(),
            cylinder_operand as u8,
            scope,
        )?;
        let planar = extract_planar_operand(
            edit,
            bodies[planar_operand].clone(),
            planar_operand as u8,
            scope,
        )?;
        (planar, cylinder)
    } else {
        let planar = extract_planar_operand(
            edit,
            bodies[planar_operand].clone(),
            planar_operand as u8,
            scope,
        )?;
        let cylinder = extract_cylinder_operand(
            edit,
            bodies[cylinder_operand].clone(),
            cylinder_operand as u8,
            scope,
        )?;
        (planar, cylinder)
    };

    let planar_certificate = planar_source.convex_certificate().ok();

    if operation == PlanarBooleanOperation::Intersect
        && let Some(planar_certificate) = planar_certificate
    {
        let relation = certify_convex_host_cylinder_support_relation(
            &edit.state.store,
            planar_certificate,
            &cylinder_source,
            scope,
        )?;
        if relation.is_some() {
            return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
        }
    }

    let graph = section_bodies_in_scope(&edit.as_part(), &bodies[0], &bodies[1], linear, scope)?;
    if graph.completion() == SectionCompletion::Complete && graph.gaps().is_empty() {
        return execute_mixed_bounded_arc(
            edit,
            operation,
            &graph,
            &bodies,
            &planar_source,
            &cylinder_source,
            planar_operand,
            cylinder_operand,
            linear,
            scope,
        );
    }
    if operation == PlanarBooleanOperation::Unite
        && let Some(planar_certificate) = planar_certificate
    {
        let relation = certify_convex_host_cylinder_support_relation(
            &edit.state.store,
            planar_certificate,
            &cylinder_source,
            scope,
        )?;
        if let Some(relation @ ConvexHostCylinderSupportRelation::CertifiedAxialSingleCap { .. }) =
            relation
            && let Some(contact) = certify_strict_axial_cap_contact(
                &edit.state.store,
                planar_certificate,
                &cylinder_source,
                relation,
                scope,
            )?
        {
            return execute_mixed_support_contact(
                edit,
                operation,
                &graph,
                &bodies,
                &planar_source,
                &cylinder_source,
                planar_operand,
                cylinder_operand,
                &contact,
                linear,
                scope,
            );
        }
    }

    refused(CurvedBooleanPipelineRefusal::SectionIncomplete)
}

/// Consume one complete bounded Plane/Cylinder arrangement through generic
/// Boolean truth. Every choice before allocation is backed by Section
/// identity or exact arrangement incidence. Endpoint-free source rings are
/// retained through the same exact incidence path when generic truth selects
/// either finite-cylinder cap.
#[allow(clippy::too_many_arguments)]
fn execute_mixed_bounded_arc(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    graph: &BodySectionGraph,
    bodies: &[BodyId; 2],
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    planar_operand: usize,
    cylinder_operand: usize,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let operation = adapt_operation(operation);
    let prepared = prepare_mixed_bounded_arc_boundary(
        &edit.as_part(),
        graph,
        bodies,
        planar,
        cylinder,
        planar_operand,
        cylinder_operand,
        None,
        linear,
        scope,
    )
    .map_err(mixed_boundary_failure)?;
    let selected = select_boundary_fragments(operation, prepared.classified())
        .map_err(|error| refused_error(CurvedBooleanPipelineRefusal::Selection(error)))?;
    if selected.is_empty() {
        return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
    }
    if let Some(operands) = prepared.whole_source_operands(&selected) {
        let sources = operands
            .into_iter()
            .map(|operand| bodies[operand].clone())
            .collect::<Vec<_>>();
        return realize_source_body_copies(edit, &sources, scope);
    }
    let plan = plan_mixed_shell(&edit.state.store, graph, prepared.bindings(), selected)
        .map_err(mixed_plan_failure)?;

    // Complete exact-scalar evidence is materialized and preflighted before
    // the failure-atomic realization transaction opens.
    realize_mixed_shell(edit, &plan, linear, scope)
}

#[allow(clippy::too_many_arguments)]
fn execute_mixed_support_contact(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    graph: &BodySectionGraph,
    bodies: &[BodyId; 2],
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    planar_operand: usize,
    cylinder_operand: usize,
    contact: &CertifiedAxialCapContact,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let operation = adapt_operation(operation);
    let prepared = prepare_mixed_bounded_arc_boundary(
        &edit.as_part(),
        graph,
        bodies,
        planar,
        cylinder,
        planar_operand,
        cylinder_operand,
        Some(contact),
        linear,
        scope,
    )
    .map_err(mixed_boundary_failure)?;
    let selected = select_boundary_fragments(operation, prepared.classified())
        .map_err(|error| refused_error(CurvedBooleanPipelineRefusal::Selection(error)))?;
    let contact_ring = prepared
        .cap_ring(contact.boundary())
        .ok_or_else(|| refused_error(CurvedBooleanPipelineRefusal::SectionIncomplete))?;
    let arrangement = arrange_projected_ring_hole_mixed_shell(
        &edit.state.store,
        graph,
        prepared.bindings(),
        selected,
        contact.host_face(),
        contact_ring,
        linear,
    )
    .map_err(mixed_plan_failure)?;
    let plan = complete_mixed_shell_plan(&edit.state.store, graph, arrangement)
        .map_err(mixed_plan_failure)?;
    realize_mixed_shell(edit, &plan, linear, scope)
}

pub(super) fn realize_mixed_shell(
    edit: &mut PartEdit<'_>,
    plan: &super::mixed_shell_plan::MixedShellProofPlan,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    if !plan.materialization_gaps().is_empty() {
        return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
    }
    let blueprint = prepare_mixed_shell_materialization(plan, &edit.state.store)
        .map_err(mixed_materialization_failure)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_REALIZATION_WORK, blueprint.work())
        .map_err(Error::from)?;
    realize_prepared_mixed_shell(edit, plan, &blueprint, linear, scope)
}

/// Realize a mixed shell from an allocation-free blueprint already charged by
/// its caller.
///
/// The cylinder-pair complete-plan adapter prepares, validates, and charges
/// this exact blueprint together with its stronger fragment-coverage proof.
/// Reusing it here keeps persistent composite certification and physical-edge
/// coalescing on their single admitted work frontier.
pub(super) fn realize_certified_cylinder_pair_shell(
    edit: &mut PartEdit<'_>,
    certified: &CertifiedCylinderPairPlan,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    realize_prepared_mixed_shell(edit, certified.plan(), certified.blueprint(), linear, scope)
}

fn realize_prepared_mixed_shell(
    edit: &mut PartEdit<'_>,
    plan: &super::mixed_shell_plan::MixedShellProofPlan,
    blueprint: &MixedShellMaterializationBlueprint,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    if !plan.materialization_gaps().is_empty() {
        return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
    }
    let component_work = mixed_shell_component_work(plan, blueprint)
        .ok_or_else(|| refused_error(CurvedBooleanPipelineRefusal::WorkCountOverflow))?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_REALIZATION_WORK, component_work)
        .map_err(Error::from)?;
    let mut components = partition_prepared_mixed_shell_components(plan, blueprint)
        .map_err(mixed_component_failure)?;
    let region_layout = prepare_region_component_order(plan, &mut components);
    let inputs = materialize_mixed_shell_component_inputs(
        plan,
        blueprint,
        &components,
        &edit.state.store,
        &MixedShellScalarInputs::empty(),
        linear,
    )
    .map_err(mixed_materialization_failure)?;
    if region_layout {
        realize_analytic_shell_region(edit, &inputs, linear, scope)
    } else {
        realize_analytic_shell_inputs(edit, &inputs, linear, scope)
    }
}

fn prepare_region_component_order(
    plan: &super::mixed_shell_plan::MixedShellProofPlan,
    components: &mut [super::mixed_shell_plan::components::MixedShellComponent],
) -> bool {
    if !plan.section_edges().is_empty() || components.len() < 2 {
        return false;
    }
    let orientations = components
        .iter()
        .map(|component| {
            let mut values = component
                .faces()
                .iter()
                .map(|face| plan.faces()[face.plan_index()].selected_orientation());
            let first = values.next()?;
            values
                .all(|orientation| orientation == first)
                .then_some(first)
        })
        .collect::<Option<Vec<_>>>();
    let Some(orientations) = orientations else {
        return false;
    };
    let mut exteriors = orientations
        .iter()
        .enumerate()
        .filter_map(|(index, orientation)| {
            (*orientation == SelectedOrientation::Preserved).then_some(index)
        });
    let Some(exterior) = exteriors.next().filter(|_| exteriors.next().is_none()) else {
        return false;
    };
    if orientations.iter().enumerate().any(|(index, orientation)| {
        index != exterior && *orientation != SelectedOrientation::Reversed
    }) {
        return false;
    }
    components.swap(0, exterior);
    true
}

fn extract_planar_operand(
    edit: &PartEdit<'_>,
    body: BodyId,
    operand: u8,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<ExtractedPlanarSourceBody> {
    match extract_planar_source_body(&edit.as_part(), body.clone(), operand, scope) {
        Ok(source) => Ok(source),
        Err(PlanarSourceExtractionError::NotFastValid(report)) => {
            let report = adapt_live_body_check(&edit.id, &edit.state.store, body.raw(), report)?;
            refused(CurvedBooleanPipelineRefusal::PlanarSourceNotFastValid { operand, report })
        }
        Err(PlanarSourceExtractionError::Unsupported(gap)) => {
            refused(CurvedBooleanPipelineRefusal::PlanarSourceUnsupported { operand, gap })
        }
        Err(PlanarSourceExtractionError::Uncertified(failure)) => {
            refused(CurvedBooleanPipelineRefusal::PlanarSourceUncertified { operand, failure })
        }
        Err(PlanarSourceExtractionError::Topology(source)) => Err(source.into()),
        Err(PlanarSourceExtractionError::WrongPart) => Err(kcore::error::Error::InvalidGeometry {
            reason: "prevalidated curved Boolean operand changed part",
        }
        .into()),
        Err(PlanarSourceExtractionError::InvalidOperand) => {
            Err(kcore::error::Error::InvalidGeometry {
                reason: "internal curved Boolean operand index is invalid",
            }
            .into())
        }
    }
}

pub(super) fn extract_cylinder_operand(
    edit: &PartEdit<'_>,
    body: BodyId,
    operand: u8,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CertifiedCylinderSource> {
    match extract_cylinder_source(&edit.state.store, body.raw(), scope)? {
        CylinderSourceOutcome::Ready(source) => Ok(source),
        CylinderSourceOutcome::NotFullValid(report) => {
            let report = adapt_live_body_check(&edit.id, &edit.state.store, body.raw(), report)?;
            refused(CurvedBooleanPipelineRefusal::CylinderSourceNotFullValid { operand, report })
        }
        CylinderSourceOutcome::Unsupported(gap) => {
            refused(CurvedBooleanPipelineRefusal::CylinderSourceUnsupported { operand, gap })
        }
    }
}

pub(super) fn adapt_operation(operation: PlanarBooleanOperation) -> RegularizedBooleanOperation {
    match operation {
        PlanarBooleanOperation::Unite => RegularizedBooleanOperation::Unite,
        PlanarBooleanOperation::Intersect => RegularizedBooleanOperation::Intersect,
        PlanarBooleanOperation::Subtract => RegularizedBooleanOperation::Subtract,
    }
}

pub(super) fn mixed_boundary_failure(error: MixedBoundaryError) -> PipelineFailure {
    match error {
        MixedBoundaryError::Execution(error) => PipelineFailure::Execution(error),
        MixedBoundaryError::IncompleteSection => {
            refused_error(CurvedBooleanPipelineRefusal::SectionIncomplete)
        }
        MixedBoundaryError::AnchorBoundaryContact => {
            refused_error(CurvedBooleanPipelineRefusal::ClassificationBoundaryContact)
        }
        MixedBoundaryError::AnchorIndeterminate(reason) => {
            refused_error(CurvedBooleanPipelineRefusal::ClassificationIndeterminate { reason })
        }
        MixedBoundaryError::PlanarArrangement(_)
        | MixedBoundaryError::PeriodicArrangement(_)
        | MixedBoundaryError::MissingPeriodicFaceEvidence
        | MixedBoundaryError::SourceTopology
        | MixedBoundaryError::AnchorUnavailable
        | MixedBoundaryError::ContradictoryDual
        | MixedBoundaryError::DisconnectedDual
        | MixedBoundaryError::CylinderCapNotExterior => {
            refused_error(CurvedBooleanPipelineRefusal::AssemblyContract(
                "mixed boundary arrangement contract failed",
            ))
        }
    }
}

pub(super) fn mixed_plan_failure(error: MixedShellPlanError) -> PipelineFailure {
    match error {
        MixedShellPlanError::SectionIncomplete => {
            refused_error(CurvedBooleanPipelineRefusal::SectionIncomplete)
        }
        _ => refused_error(CurvedBooleanPipelineRefusal::AssemblyContract(
            "mixed shell proof-plan contract failed",
        )),
    }
}

fn mixed_materialization_failure(error: MixedShellMaterializationError) -> PipelineFailure {
    match error {
        MixedShellMaterializationError::WorkCountOverflow => {
            refused_error(CurvedBooleanPipelineRefusal::WorkCountOverflow)
        }
        MixedShellMaterializationError::MissingSourceRootScalar(_)
        | MixedShellMaterializationError::MissingSectionTrimScalar(_) => {
            refused_error(CurvedBooleanPipelineRefusal::SectionIncomplete)
        }
        MixedShellMaterializationError::AnalyticPreflight(
            ktopo::analytic_shell::AnalyticShellPlanError::DisconnectedShell,
        ) => refused_error(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported),
        _ => refused_error(CurvedBooleanPipelineRefusal::AssemblyContract(
            "mixed analytic-shell materialization failed",
        )),
    }
}

fn mixed_component_failure(error: MixedShellComponentError) -> PipelineFailure {
    match error {
        MixedShellComponentError::PhysicalIncidence(
            MixedShellMaterializationError::WorkCountOverflow,
        ) => refused_error(CurvedBooleanPipelineRefusal::WorkCountOverflow),
        _ => refused_error(CurvedBooleanPipelineRefusal::AssemblyContract(
            "mixed shell component partition failed",
        )),
    }
}

fn refused_error(refusal: CurvedBooleanPipelineRefusal) -> PipelineFailure {
    PipelineFailure::Refused(refusal)
}

pub(super) fn refused<T>(refusal: CurvedBooleanPipelineRefusal) -> StageResult<T> {
    Err(refused_error(refusal))
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};
    use ktopo::check::CheckOutcome;
    use ktopo::entity::RegionKind;

    use super::*;
    use crate::{BlockRequest, CylinderRequest, Kernel};

    fn reverse_body_face_storage(edit: &mut crate::session::PartEdit<'_>, body: &BodyId) {
        let store = edit.store_mut_for_test();
        let material = store
            .get(body.raw())
            .unwrap()
            .regions()
            .iter()
            .copied()
            .find(|region| store.get(*region).unwrap().kind() == RegionKind::Solid)
            .unwrap();
        let shell = store.get(material).unwrap().shells()[0];
        let mut transaction = store.transaction().unwrap();
        transaction
            .assembly()
            .get_mut(shell)
            .unwrap()
            .faces
            .reverse();
        transaction.commit_checked_body(body.raw()).unwrap();
    }

    #[test]
    fn capped_features_ignore_operand_and_face_storage_order() {
        for (operation, cylinder_first) in [
            (PlanarBooleanOperation::Unite, false),
            (PlanarBooleanOperation::Unite, true),
            (PlanarBooleanOperation::Subtract, false),
        ] {
            let mut session = Kernel::new().create_session();
            let part = session.create_part();
            let (block, cylinder) = {
                let mut edit = session.edit_part(part.clone()).unwrap();
                let block = edit
                    .create_block(BlockRequest::new(Frame::world(), [4.0, 4.0, 2.0]))
                    .unwrap()
                    .into_result()
                    .unwrap()
                    .body();
                let cylinder = edit
                    .create_cylinder(CylinderRequest::new(Frame::world(), 0.75, 2.0))
                    .unwrap()
                    .into_result()
                    .unwrap()
                    .body();
                reverse_body_face_storage(&mut edit, &block);
                reverse_body_face_storage(&mut edit, &cylinder);
                (block, cylinder)
            };
            let (left, right) = if cylinder_first {
                (cylinder, block)
            } else {
                (block, cylinder)
            };
            let outcome = super::super::dispatch::execute_boolean(
                &mut session.edit_part(part).unwrap(),
                operation,
                left,
                right,
                crate::OperationSettings::new(),
            )
            .unwrap()
            .into_result()
            .unwrap();
            let super::super::dispatch::BooleanPipelineOutcome::Curved(
                CurvedBooleanPipelineOutcome::Committed(committed),
            ) = outcome
            else {
                panic!("expected committed capped feature, got {outcome:?}")
            };
            assert_eq!(committed.bodies.len(), 1);
            assert!(
                committed
                    .full_checks
                    .iter()
                    .all(|check| check.report().outcome() == CheckOutcome::Valid)
            );
        }
    }

    #[test]
    fn axial_band_results_ignore_both_operand_face_storage_orders() {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let base = Point3::new(3.0, -2.0, 1.25);
        let cylinder_frame =
            Frame::new(base, Vec3::new(0.0, 0.6, 0.8), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let block_frame = cylinder_frame.with_origin(base + cylinder_frame.z());
        let (block, cylinder) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(block_frame, [4.0, 4.0, 1.0]))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(cylinder_frame, 0.75, 2.0))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            reverse_body_face_storage(&mut edit, &block);
            reverse_body_face_storage(&mut edit, &cylinder);
            (block, cylinder)
        };

        for (operation, left, right, expected_bodies) in [
            (
                PlanarBooleanOperation::Intersect,
                block.clone(),
                cylinder.clone(),
                1,
            ),
            (PlanarBooleanOperation::Subtract, cylinder, block, 2),
        ] {
            let outcome = super::super::dispatch::execute_boolean(
                &mut session.edit_part(part.clone()).unwrap(),
                operation,
                left,
                right,
                crate::OperationSettings::new(),
            )
            .unwrap()
            .into_result()
            .unwrap();
            let super::super::dispatch::BooleanPipelineOutcome::Curved(
                CurvedBooleanPipelineOutcome::Committed(committed),
            ) = outcome
            else {
                panic!("expected committed curved result, got {outcome:?}")
            };
            assert_eq!(committed.bodies.len(), expected_bodies);
        }
    }

    #[test]
    fn zero_cut_contained_cylinder_is_one_complete_source_copy() {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(Frame::world(), [6.0, 6.0, 6.0]))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(
                    Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
                    0.75,
                    2.0,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (block, cylinder)
        };
        let outcome = super::super::dispatch::execute_boolean(
            &mut session.edit_part(part).unwrap(),
            PlanarBooleanOperation::Intersect,
            block,
            cylinder,
            crate::OperationSettings::new(),
        )
        .unwrap()
        .into_result()
        .unwrap();
        assert!(
            matches!(
                outcome,
                super::super::dispatch::BooleanPipelineOutcome::Curved(
                    CurvedBooleanPipelineOutcome::Committed(_)
                )
            ),
            "outcome: {outcome:?}"
        );
    }

    #[test]
    fn zero_cut_disjoint_cylinder_is_proven_empty() {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(
                    Frame::world().with_origin(Point3::new(8.0, 0.0, 0.0)),
                    [2.0, 2.0, 2.0],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(
                    Frame::world().with_origin(Point3::new(-8.0, 0.0, -1.0)),
                    0.75,
                    2.0,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (block, cylinder)
        };
        let outcome = super::super::dispatch::execute_boolean(
            &mut session.edit_part(part).unwrap(),
            PlanarBooleanOperation::Intersect,
            block,
            cylinder,
            crate::OperationSettings::new(),
        )
        .unwrap()
        .into_result()
        .unwrap();
        assert!(
            matches!(
                outcome,
                super::super::dispatch::BooleanPipelineOutcome::Curved(
                    CurvedBooleanPipelineOutcome::ProvenEmpty
                )
            ),
            "outcome: {outcome:?}"
        );
    }

    #[test]
    fn bounded_arc_planar_subtract_commits_two_full_valid_components_atomically() {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(
                    Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
                    [2.0, 6.0, 1.0],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(Frame::world(), 1.5, 2.0))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (block, cylinder)
        };
        let signature = |session: &crate::Session| {
            let part = session.part(part.clone()).unwrap();
            (
                part.bodies().len(),
                part.body(block.clone())
                    .unwrap()
                    .faces()
                    .unwrap()
                    .collect::<Vec<_>>(),
                part.body(cylinder.clone())
                    .unwrap()
                    .faces()
                    .unwrap()
                    .collect::<Vec<_>>(),
            )
        };
        let before = signature(&session);

        let outcome = super::super::dispatch::execute_boolean(
            &mut session.edit_part(part.clone()).unwrap(),
            PlanarBooleanOperation::Subtract,
            block.clone(),
            cylinder.clone(),
            crate::OperationSettings::new(),
        )
        .unwrap()
        .into_result()
        .unwrap();
        let super::super::dispatch::BooleanPipelineOutcome::Curved(
            CurvedBooleanPipelineOutcome::Committed(committed),
        ) = outcome
        else {
            panic!("unexpected ordered subtract outcome: {outcome:?}")
        };
        assert_eq!(committed.bodies.len(), 2);
        assert_eq!(committed.full_checks.len(), 2);
        assert!(
            committed
                .full_checks
                .iter()
                .all(|check| check.report().outcome() == CheckOutcome::Valid)
        );
        let after = signature(&session);
        assert_eq!(after.0, before.0 + 2);
        assert_eq!(after.1, before.1);
        assert_eq!(after.2, before.2);
    }
}

#[cfg(test)]
#[path = "curved_pipeline_bounded_skew_tests.rs"]
mod bounded_skew_tests;
