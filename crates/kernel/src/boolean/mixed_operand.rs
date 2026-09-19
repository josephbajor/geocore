//! Reuse Full-valid mixed operands through the shared arrangement spine.

use super::boundary_select::select_boundary_fragments;
use super::curved_pipeline::*;
use super::mixed_boundary::prepare_mixed_bounded_arc_boundary;
use super::mixed_shell_plan::{arrange_mixed_shell, plan_mixed_shell};
use super::select::PlanarBooleanOperation;
use crate::error::Error;
use crate::section::section_bodies_in_scope;
use crate::session::PartEdit;
use crate::{BodyId, FaceId, SectionCompletion};
use kcore::operation::OperationScope;
use ktopo::check::{CheckLevel, CheckOutcome, check_body_report_in_scope};
use ktopo::geom::SurfaceGeom;

/// Admit a composite minuend and a primitive cylindrical cutter. Every
/// existing cylindrical face must remain uncut; untouched source rings are
/// retained by the ordinary mixed-shell planner, never reconstructed by case.
pub(super) fn try_execute(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: &[BodyId; 2],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<Option<CurvedBooleanPipelineOutcome>> {
    if operation != PlanarBooleanOperation::Subtract {
        return Ok(None);
    }
    let raw_faces = edit.state.store.faces_of_body(bodies[0].raw())?;
    if raw_faces.len() <= 3 {
        return Ok(None);
    }
    let tool_faces = edit.state.store.faces_of_body(bodies[1].raw())?;
    if tool_faces.len() != 3 {
        return Ok(None);
    }
    let mut planes = Vec::new();
    let mut annuli = Vec::new();
    let mut topology_work = 0_u64;
    for raw in raw_faces {
        let face = edit.state.store.get(raw)?;
        let fail = || PipelineFailure::Refused(CurvedBooleanPipelineRefusal::WorkCountOverflow);
        topology_work = topology_work.checked_add(1).ok_or_else(fail)?;
        for &ring in face.loops() {
            let fins = edit.state.store.get(ring)?.fins().len();
            let count = u64::try_from(fins).map_err(|_| fail())?;
            topology_work = topology_work
                .checked_add(1)
                .and_then(|n| n.checked_add(count))
                .ok_or_else(fail)?;
        }
        let id = FaceId::new(edit.id.clone(), raw);
        match edit.state.store.surface(face.surface())? {
            SurfaceGeom::Plane(_) => planes.push(id),
            SurfaceGeom::Cylinder(_) => annuli.push(id),
            _ => return Ok(None),
        }
    }
    if planes.is_empty() || annuli.is_empty() {
        return Ok(None);
    }
    // One extraction unit per source face, loop, and directed edge use.
    scope
        .ledger_mut()
        .charge(super::extract::PLANAR_SOURCE_EXTRACTION_WORK, topology_work)
        .map_err(Error::from)?;
    let report =
        check_body_report_in_scope(&edit.state.store, bodies[0].raw(), CheckLevel::Full, scope)
            .map_err(Error::from)?;
    if report.outcome() != CheckOutcome::Valid {
        return Ok(Some(CurvedBooleanPipelineOutcome::Refused(
            CurvedBooleanPipelineRefusal::ResultTopologyUnsupported,
        )));
    }
    let cylinder = extract_cylinder_operand(edit, bodies[1].clone(), 1, scope)?;
    let graph = section_bodies_in_scope(&edit.as_part(), &bodies[0], &bodies[1], linear, scope)?;
    if graph.completion() != SectionCompletion::Complete || !graph.gaps().is_empty() {
        return Ok(Some(CurvedBooleanPipelineOutcome::Refused(
            CurvedBooleanPipelineRefusal::SectionIncomplete,
        )));
    }
    // Cover retained-loop comparisons, periodic-source discovery, and source
    // bindings before arranging any new cells. The ordinary boundary adapter
    // separately charges the cutter and all Section fragments.
    let extra_work = u64::try_from(graph.curve_fragments().len())
        .ok()
        .and_then(|n| n.checked_add(topology_work))
        .and_then(|n| n.checked_mul(topology_work))
        .ok_or(PipelineFailure::Refused(
            CurvedBooleanPipelineRefusal::WorkCountOverflow,
        ))?;
    scope
        .ledger_mut()
        .charge(super::pipeline::PLANAR_BOOLEAN_BSP_WORK, extra_work)
        .map_err(Error::from)?;
    let mut prepared = prepare_mixed_bounded_arc_boundary(
        &edit.as_part(),
        &graph,
        bodies,
        &planes,
        &cylinder,
        0,
        1,
        None,
        linear,
        scope,
    )
    .map_err(mixed_boundary_failure)?;
    prepared
        .append_uncut_annuli(&edit.as_part(), &graph, bodies, 0, &annuli, linear, scope)
        .map_err(mixed_boundary_failure)?;
    let selected = select_boundary_fragments(adapt_operation(operation), prepared.classified())
        .map_err(|error| {
            PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error))
        })?;
    let arrangement = arrange_mixed_shell(&edit.state.store, &graph, prepared.bindings(), selected)
        .map_err(mixed_plan_failure)?;
    let plan =
        plan_mixed_shell(&edit.state.store, &graph, arrangement).map_err(mixed_plan_failure)?;
    realize_mixed_shell(edit, &plan, linear, scope).map(Some)
}
