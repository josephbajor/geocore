//! Facade-only lifecycle tests: no lower-layer crate is imported.

use kernel::{
    BlockRequest, BoundedCurve, CheckBodyRequest, CheckLevel, CheckOutcome, Error, ExportXtRequest,
    Frame, ImportXtRequest, IntersectCurvesRequest, Kernel, ParamRange, SessionPolicy,
    SurfaceDerivativeOrder, SurfaceEvaluationRequest,
};

#[test]
fn sessions_own_independent_parts_and_policy() {
    let configured = SessionPolicy::v1();
    let kernel = Kernel::with_default_policy(configured.clone());
    let mut first = kernel.create_session();
    let mut second = kernel.create_session();
    assert_eq!(first.policy(), &configured);
    assert_eq!(second.policy(), &configured);

    let first_part = first.create_part();
    let second_part = second.create_part();
    assert_eq!(format!("{first_part:?}"), "PartId(<opaque>)");
    assert_eq!(first.parts().len(), 1);
    assert_eq!(second.parts().len(), 1);
    assert!(matches!(first.part(second_part), Err(Error::UnknownPart)));

    let part = first.part(first_part.clone()).unwrap();
    assert_eq!(part.id(), first_part);
    assert_eq!(part.bodies().len(), 0);
    assert_eq!(part.regions().len(), 0);
    assert_eq!(part.shells().len(), 0);
    assert_eq!(part.faces().len(), 0);
    assert_eq!(part.loops().len(), 0);
    assert_eq!(part.fins().len(), 0);
    assert_eq!(part.edges().len(), 0);
    assert_eq!(part.vertices().len(), 0);
}

#[test]
fn removed_part_ids_are_stale_and_generation_safe() {
    let mut session = Kernel::new().create_session();
    let first = session.create_part();
    let stale = session.create_part();
    let third = session.create_part();
    session.remove_part(stale.clone()).unwrap();
    assert!(matches!(
        session.part(stale.clone()),
        Err(Error::UnknownPart)
    ));

    let replacement = session.create_part();
    assert_ne!(stale, replacement);
    assert!(matches!(session.edit_part(stale), Err(Error::UnknownPart)));
    assert_eq!(
        session.parts().collect::<Vec<_>>(),
        vec![first, replacement, third]
    );
}

#[test]
fn exclusive_part_capability_still_allows_read_views() {
    let mut session = Kernel::new().create_session();
    let id = session.create_part();
    let expected_policy = session.policy().clone();
    let edit = session.edit_part(id.clone()).unwrap();
    assert_eq!(edit.id(), id);
    assert_eq!(edit.policy(), &expected_policy);
    assert_eq!(edit.as_part().bodies().len(), 0);
}

#[test]
fn facade_only_client_can_construct_and_check_a_block_with_reports() {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let creation = session
        .edit_part(part_id.clone())
        .unwrap()
        .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))
        .unwrap();
    assert!(creation.report().usage().is_empty());
    let created = creation.into_result().unwrap();
    assert_eq!(created.journal().part(), part_id);
    assert!(created.journal().mutation_count() > 0);
    assert_eq!(created.journal().lineage_count(), 0);

    let check = session
        .part(part_id)
        .unwrap()
        .check_body(CheckBodyRequest::new(created.body(), CheckLevel::Fast))
        .unwrap();
    assert_eq!(check.result().unwrap().outcome(), CheckOutcome::Valid);
    assert!(check.result().unwrap().faults().is_empty());
    assert!(!check.report().usage().is_empty());
    assert!(check.report().limit_events().is_empty());
}

#[test]
fn facade_only_client_can_run_a_full_check_with_family_defaults() {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let body = session
        .edit_part(part_id.clone())
        .unwrap()
        .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))
        .unwrap()
        .into_result()
        .unwrap()
        .body();
    let check = session
        .part(part_id)
        .unwrap()
        .check_body(CheckBodyRequest::new(body, CheckLevel::Full))
        .unwrap();

    assert_eq!(check.result().unwrap().outcome(), CheckOutcome::Valid);
    assert!(!check.report().usage().is_empty());
}

#[test]
fn facade_only_client_can_evaluate_an_opaque_surface_with_one_report() {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let body = session
        .edit_part(part_id.clone())
        .unwrap()
        .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))
        .unwrap()
        .into_result()
        .unwrap()
        .body();
    let part = session.part(part_id).unwrap();
    let face = part.body(body).unwrap().faces().unwrap().next().unwrap();
    let surface = part.face(face).unwrap().surface();

    let evaluation = part
        .evaluate_surface(SurfaceEvaluationRequest::new(
            surface.clone(),
            [0.0, 0.0],
            SurfaceDerivativeOrder::First,
        ))
        .unwrap();

    assert_eq!(evaluation.result().unwrap().surface(), surface);
    assert!(
        evaluation
            .result()
            .unwrap()
            .derivatives()
            .p
            .to_array()
            .iter()
            .all(|v| v.is_finite())
    );
    assert_eq!(evaluation.report().usage().len(), 2);
    assert!(
        evaluation
            .report()
            .usage()
            .iter()
            .all(|usage| usage.consumed == 1)
    );
}

#[test]
fn facade_only_client_can_intersect_graph_owned_curves_with_identity_and_one_report() {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let body = session
        .edit_part(part_id.clone())
        .unwrap()
        .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))
        .unwrap()
        .into_result()
        .unwrap()
        .body();
    let part = session.part(part_id).unwrap();
    let edges = part
        .body(body)
        .unwrap()
        .edges()
        .unwrap()
        .map(|edge_id| {
            let edge = part.edge(edge_id).unwrap();
            let (lo, hi) = edge.bounds().expect("block edges are bounded");
            (
                BoundedCurve::new(
                    edge.curve().expect("block edges have supporting curves"),
                    ParamRange::new(lo, hi),
                ),
                edge.vertices(),
            )
        })
        .collect::<Vec<_>>();
    let (first, second) = (0..edges.len())
        .find_map(|left| {
            ((left + 1)..edges.len()).find_map(|right| {
                let share_vertex = edges[left].1.iter().flatten().any(|left_vertex| {
                    edges[right]
                        .1
                        .iter()
                        .flatten()
                        .any(|right_vertex| right_vertex == left_vertex)
                });
                share_vertex.then(|| (edges[left].0.clone(), edges[right].0.clone()))
            })
        })
        .expect("a block has adjacent bounded edges");
    let first_id = first.curve();
    let second_id = second.curve();

    let outcome = part
        .intersect_curves(IntersectCurvesRequest::new(first, second))
        .unwrap();
    let result = outcome.result();
    let intersections = result.as_ref().unwrap();
    assert_eq!(intersections.first(), first_id);
    assert_eq!(intersections.second(), second_id);
    assert!(intersections.is_complete());
    assert!(!intersections.is_empty());
    assert_eq!(intersections.points().len(), 1);
    assert!(intersections.overlaps().is_empty());
    assert_eq!(outcome.report().usage().len(), 9);
    assert!(
        outcome
            .report()
            .usage()
            .iter()
            .all(|usage| usage.consumed == 0)
    );
    assert!(outcome.report().limit_events().is_empty());
}

#[test]
fn curve_intersection_rejects_foreign_identity_before_starting_an_operation() {
    let mut session = Kernel::new().create_session();
    let first_part_id = session.create_part();
    let first_body = session
        .edit_part(first_part_id.clone())
        .unwrap()
        .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
        .unwrap()
        .into_result()
        .unwrap()
        .body();
    let second_part_id = session.create_part();
    let second_body = session
        .edit_part(second_part_id.clone())
        .unwrap()
        .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
        .unwrap()
        .into_result()
        .unwrap()
        .body();

    let foreign = {
        let part = session.part(first_part_id).unwrap();
        let edge = part
            .edge(
                part.body(first_body)
                    .unwrap()
                    .edges()
                    .unwrap()
                    .next()
                    .unwrap(),
            )
            .unwrap();
        let (lo, hi) = edge.bounds().unwrap();
        BoundedCurve::new(edge.curve().unwrap(), ParamRange::new(lo, hi))
    };
    let local = {
        let part = session.part(second_part_id.clone()).unwrap();
        let edge = part
            .edge(
                part.body(second_body)
                    .unwrap()
                    .edges()
                    .unwrap()
                    .next()
                    .unwrap(),
            )
            .unwrap();
        let (lo, hi) = edge.bounds().unwrap();
        BoundedCurve::new(edge.curve().unwrap(), ParamRange::new(lo, hi))
    };

    assert!(matches!(
        session
            .part(second_part_id)
            .unwrap()
            .intersect_curves(IntersectCurvesRequest::new(foreign, local)),
        Err(Error::WrongPart { .. })
    ));
}

#[test]
fn facade_only_client_can_import_inspect_and_deterministically_export_xt() {
    let mut session = Kernel::new().create_session();
    let source_part = session.create_part();
    let source_body = session
        .edit_part(source_part.clone())
        .unwrap()
        .create_block(BlockRequest::new(Frame::world(), [0.1, 0.15, 0.2]))
        .unwrap()
        .into_result()
        .unwrap()
        .body();
    let authored = session
        .part(source_part)
        .unwrap()
        .export_xt(ExportXtRequest::new(source_body))
        .unwrap()
        .into_result()
        .unwrap();

    let part_id = session.create_part();
    let imported = session
        .edit_part(part_id.clone())
        .unwrap()
        .import_xt(ImportXtRequest::new(authored.bytes()))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(imported.bodies().len(), 1);
    assert!(imported.journal().mutation_count() > 0);
    let body = imported.bodies()[0].clone();
    assert!(
        session
            .part(part_id.clone())
            .unwrap()
            .body(body.clone())
            .is_ok()
    );

    let first = session
        .part(part_id.clone())
        .unwrap()
        .export_xt(ExportXtRequest::new(body.clone()))
        .unwrap()
        .into_result()
        .unwrap();
    let second = session
        .part(part_id)
        .unwrap()
        .export_xt(ExportXtRequest::new(body))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(first.bytes(), second.bytes());
    assert_eq!(authored.bytes(), first.bytes());
    assert!(first.text().starts_with("**ABCDEFGHIJKLMNOPQRSTUVWXYZ"));
}
