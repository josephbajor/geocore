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
/// existing cylindrical annulus is arranged from its complete Section subset;
/// split and untouched boundaries retain topology-owned source lineage.
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
        .append_source_annuli(&edit.as_part(), &graph, bodies, 0, &annuli, linear, scope)
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

#[cfg(test)]
mod tests {
    use crate::*;
    use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};

    #[test]
    fn overlapping_cut_full_checker_rejects_changed_arc_and_annulus_geometry() {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let mut body = session
            .edit_part(part.clone())
            .unwrap()
            .extrude_profile(ExtrudeProfileRequest::new(
                Frame::world(),
                vec![
                    Point2::new(-5.0, -4.0),
                    Point2::new(5.0, -4.0),
                    Point2::new(5.0, 4.0),
                    Point2::new(-5.0, 4.0),
                ],
                vec![],
                2.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        for (x, y, r) in [(0.0, 0.0, 0.75), (3.0, 2.0, 0.25), (0.75, 0.0, 0.75)] {
            let tool = session
                .edit_part(part.clone())
                .unwrap()
                .create_cylinder(CylinderRequest::new(
                    Frame::world().with_origin(Point3::new(x, y, -1.0)),
                    r,
                    4.0,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let result = session
                .edit_part(part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Subtract,
                    body,
                    tool,
                ))
                .unwrap()
                .into_result()
                .unwrap();
            let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
                panic!("supported overlap must commit");
            };
            body = created.bodies()[0].clone();
        }
        let baseline = session.part(part).unwrap().state.store.clone();
        let faces = baseline.faces_of_body(body.raw()).unwrap();
        let mut checked = 0;
        // Mutate each planar circular pcurve, covering both new bounded arcs
        // and the untouched complete circles, without changing topology.
        for &face in &faces {
            let data = baseline.get(face).unwrap();
            if !matches!(
                baseline.surface(data.surface()).unwrap(),
                SurfaceGeom::Plane(_)
            ) {
                continue;
            }
            for &ring in data.loops() {
                for &fin in baseline.get(ring).unwrap().fins() {
                    let use_ = baseline.get(fin).unwrap().pcurve().unwrap();
                    let Curve2dGeom::Circle(circle) = *baseline.pcurve(use_.curve()).unwrap()
                    else {
                        continue;
                    };
                    let mut store = baseline.clone();
                    let mut txn = store.transaction().unwrap();
                    txn.assembly()
                        .replace_pcurve(
                            use_.curve(),
                            Curve2dGeom::Circle(
                                kgeom::curve2d::Circle2d::new(
                                    circle.center(),
                                    circle.radius() + 0.125,
                                    circle.x_dir(),
                                )
                                .unwrap(),
                            ),
                        )
                        .unwrap();
                    let report =
                        ktopo::check::check_body_report(txn.store(), body.raw(), CheckLevel::Full)
                            .unwrap();
                    assert_ne!(report.outcome(), CheckOutcome::Valid);
                    checked += 1;
                }
            }
        }
        assert_eq!(checked, 6);
        for edge in baseline.edges_of_body(body.raw()).unwrap() {
            let data = baseline.get(edge).unwrap();
            let Some(curve) = data.curve() else {
                continue;
            };
            let CurveGeom::Circle(circle) = *baseline.curve(curve).unwrap() else {
                continue;
            };
            let mut store = baseline.clone();
            let mut txn = store.transaction().unwrap();
            txn.assembly()
                .replace_curve(
                    curve,
                    CurveGeom::Circle(
                        kgeom::curve::Circle::new(*circle.frame(), circle.radius() + 0.125)
                            .unwrap(),
                    ),
                )
                .unwrap();
            let report =
                ktopo::check::check_body_report(txn.store(), body.raw(), CheckLevel::Full).unwrap();
            assert_ne!(report.outcome(), CheckOutcome::Valid);
        }
    }
}
