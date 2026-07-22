//! Full-checked Boolean consumer of parallel-cylinder Section evidence.

use kcore::operation::OperationScope;
use ktopo::entity::Body as TopologyBody;

use super::boundary_select::select_boundary_fragments;
use super::curved_pipeline::{
    CurvedBooleanPipelineOutcome, CurvedBooleanPipelineRefusal, PipelineFailure, StageResult,
    adapt_operation, extract_cylinder_operand, mixed_boundary_failure, mixed_plan_failure,
    realize_mixed_shell, refused,
};
use super::mixed_shell_plan::{MixedShellProofPlan, plan_mixed_shell};
use super::parallel_cylinder_boundary::prepare_parallel_cylinder_boundary;
use super::parallel_cylinder_relation::{
    ParallelCylinderRelationOutcome, certify_parallel_cylinder_relation,
};
use super::select::PlanarBooleanOperation;
use crate::BodyId;
use crate::section::section_bodies_in_scope;
use crate::session::PartEdit;

/// Consume the strict nested-height parallel-cylinder theorem through the
/// shared arrangement, truth-selection, planning, and Full-check path.
///
/// Intersect and Unite are commutative and receive a canonical source order.
/// Subtract preserves caller order; the certified relation identifies which
/// operand owns the shorter axial band for either ordered material meaning.
pub(super) fn execute_parallel_cylinder_boolean(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: [BodyId; 2],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let bodies = match operation {
        PlanarBooleanOperation::Intersect | PlanarBooleanOperation::Unite => {
            canonical_commutative_order(edit, bodies)?
        }
        PlanarBooleanOperation::Subtract => bodies,
    };
    let first = extract_cylinder_operand(edit, bodies[0].clone(), 0, scope)?;
    let second = extract_cylinder_operand(edit, bodies[1].clone(), 1, scope)?;
    let graph = section_bodies_in_scope(&edit.as_part(), &bodies[0], &bodies[1], linear, scope)?;
    let relation = certify_parallel_cylinder_relation(&graph, [&first, &second], scope)?;
    let ParallelCylinderRelationOutcome::Certified(relation) = relation else {
        return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
    };
    let prepared = prepare_parallel_cylinder_boundary(
        &edit.as_part(),
        &graph,
        &bodies,
        [&first, &second],
        &relation,
        linear,
        scope,
    )
    .map_err(mixed_boundary_failure)?;
    let selected = select_boundary_fragments(adapt_operation(operation), prepared.classified())
        .map_err(|error| {
            PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error))
        })?;
    if selected.is_empty() {
        return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
            "certified positive-volume parallel-cylinder Boolean selected no boundary",
        ));
    }
    let plan = plan_mixed_shell(&edit.state.store, &graph, prepared.bindings(), selected)
        .map_err(mixed_plan_failure)?;
    if !plan_matches_relation(&plan, &relation) {
        return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
            "parallel-cylinder shell omitted certified section evidence",
        ));
    }
    realize_mixed_shell(edit, &plan, linear, scope)
}

/// Give a commutative operation one caller-order-independent source order.
/// Store iteration is deterministic slot order and carries no geometric case
/// decision; operand meaning is recovered later by the certified relation.
fn canonical_commutative_order(
    edit: &PartEdit<'_>,
    bodies: [BodyId; 2],
) -> StageResult<[BodyId; 2]> {
    for (candidate, _) in edit.state.store.iter::<TopologyBody>() {
        if candidate == bodies[0].raw() {
            return Ok(bodies);
        }
        if candidate == bodies[1].raw() {
            return Ok([bodies[1].clone(), bodies[0].clone()]);
        }
    }
    Err(kcore::error::Error::InvalidGeometry {
        reason: "prevalidated parallel-cylinder operands left the part store",
    }
    .into())
}

fn plan_matches_relation(
    plan: &MixedShellProofPlan,
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderLensRelation,
) -> bool {
    if plan.section_edges().len() != 4 {
        return false;
    }
    let rulings_match = relation.rulings().iter().all(|witness| {
        let mut matches = plan
            .section_edges()
            .iter()
            .filter(|edge| edge.fragment_index() == witness.fragment());
        let Some(edge) = matches.next() else {
            return false;
        };
        if matches.next().is_some() {
            return false;
        }
        let actual_endpoints = edge.endpoints();
        let expected_endpoints = witness.endpoints();
        edge.fragment().branch() == witness.branch()
            && (actual_endpoints == expected_endpoints
                || actual_endpoints == [expected_endpoints[1], expected_endpoints[0]])
    });
    let caps_match = relation.cap_boundaries().iter().all(|witness| {
        let mut matches = plan
            .section_edges()
            .iter()
            .filter(|edge| edge.fragment_index() == witness.fragment());
        let Some(edge) = matches.next() else {
            return false;
        };
        matches.next().is_none() && edge.fragment().branch() == witness.branch()
    });
    rulings_match && caps_match
}
