//! Facade-only lifecycle tests: no lower-layer crate is imported.

use kernel::{
    BlockRequest, BodyTessellationBudgetProfile, BoundedCurve, BoundedPcurve, CheckBodyRequest,
    CheckLevel, CheckOutcome, CreateSeedBodyRequest, CreateStrutRequest, EntityKind, Error,
    ExportXtRequest, Frame, ImportXtRequest, IntersectCurvesRequest, JoinRingRequest, Kernel,
    MergeFaceAsHoleRequest, MutationKind, OperationSettings, ParamRange, PcurveChart,
    PcurveEndpointKind, PcurveMetadata, PcurveSeam, PcurveSeamSide, Point3, RemoveBridgeRequest,
    RemoveSeedBodyRequest, RemoveStrutRequest, SessionPolicy, SplitHoleAsFaceRequest,
    SurfaceDerivativeOrder, SurfaceEvaluationRequest, SurfaceParameter, TessOptions,
    TessellateBodyRequest,
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
    let mutations = created.journal().mutations().collect::<Vec<_>>();
    assert_eq!(mutations.len(), created.journal().mutation_count());
    assert!(
        mutations
            .iter()
            .all(|mutation| mutation.entity().part() == part_id)
    );
    assert!(
        mutations
            .iter()
            .all(|mutation| mutation.kind() == MutationKind::Created)
    );
    let kinds = mutations
        .iter()
        .map(|mutation| mutation.entity().kind())
        .collect::<Vec<_>>();
    for expected in [
        EntityKind::Body,
        EntityKind::Face,
        EntityKind::Edge,
        EntityKind::Vertex,
        EntityKind::Curve,
        EntityKind::Surface,
        EntityKind::Point,
        EntityKind::Pcurve,
    ] {
        assert!(kinds.contains(&expected), "missing {expected:?}");
    }
    assert_eq!(created.journal().lineage().len(), 0);
    assert_eq!(created.journal().tolerance_budgets().len(), 0);
    assert_eq!(created.journal().tolerance_events().len(), 0);

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
fn facade_only_client_can_inspect_and_author_pcurve_metadata_values() {
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
    let fins = part
        .body(body)
        .unwrap()
        .edges()
        .unwrap()
        .flat_map(|edge| part.edge(edge).unwrap().fins().collect::<Vec<_>>())
        .collect::<Vec<_>>();
    assert!(!fins.is_empty());
    for fin in fins {
        let view = part.fin(fin).unwrap();
        assert_eq!(view.pcurve_metadata(), Some(PcurveMetadata::regular()));
        assert_eq!(view.pcurve_chart(), Some(PcurveChart::identity()));
        assert_eq!(
            view.pcurve_endpoint_kinds(),
            Some([PcurveEndpointKind::Regular; 2])
        );
        assert_eq!(view.pcurve_closure_winding(), None);
        assert_eq!(view.pcurve_seam(), None);
    }

    let seam = PcurveSeam::new(SurfaceParameter::U, PcurveSeamSide::Upper);
    let periodic = PcurveMetadata::regular()
        .with_chart(PcurveChart::shifted([1.0, 0.0]).unwrap())
        .with_endpoint_kinds([
            PcurveEndpointKind::Regular,
            PcurveEndpointKind::SurfaceSingularity,
        ])
        .with_closure_winding([1, 0])
        .with_seam(seam);
    assert_eq!(periodic.chart().period_shifts(), [1, 0]);
    assert_eq!(periodic.seam(), Some(seam));
    assert!(PcurveChart::shifted([f64::NAN, 0.0]).is_err());
}

#[test]
fn facade_only_ring_edit_requests_remain_checked_and_failure_atomic() {
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
    let (loop_id, fin_id, edge_id, curve, pcurve) = {
        let part = session.part(part_id.clone()).unwrap();
        let face = part
            .body(body.clone())
            .unwrap()
            .faces()
            .unwrap()
            .next()
            .unwrap();
        let loop_id = part.face(face).unwrap().loops().next().unwrap();
        let fin_id = part.loop_(loop_id.clone()).unwrap().fins().next().unwrap();
        let fin = part.fin(fin_id.clone()).unwrap();
        let edge_id = fin.edge();
        let edge = part.edge(edge_id.clone()).unwrap();
        let (lo, hi) = edge.bounds().unwrap();
        let curve = BoundedCurve::new(edge.curve().unwrap(), ParamRange::new(lo, hi));
        let pcurve = BoundedPcurve::new(fin.pcurve().unwrap(), fin.pcurve_range().unwrap())
            .with_parameter_map(fin.pcurve_parameter_map().unwrap())
            .with_metadata(fin.pcurve_metadata().unwrap());
        (loop_id, fin_id, edge_id, curve, pcurve)
    };
    let request = JoinRingRequest::new(
        loop_id.clone(),
        0,
        loop_id.clone(),
        0,
        curve,
        [pcurve.clone(), pcurve],
    );
    assert_eq!(request.outer(), loop_id);
    assert_eq!(request.ring(), loop_id);
    assert_eq!(request.outer_fin_index(), 0);
    assert_eq!(request.ring_fin_index(), 0);
    assert_eq!(request.pcurves().len(), 2);

    let mut edit = session.edit_part(part_id.clone()).unwrap();
    let original_loop_count = edit.as_part().loops().len();
    let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
    assert!(transaction.join_ring(request).is_err());
    transaction.rollback().unwrap();
    let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
    assert!(
        transaction
            .remove_bridge(RemoveBridgeRequest::new(edge_id))
            .is_err()
    );
    transaction.rollback().unwrap();
    assert_eq!(edit.as_part().loops().len(), original_loop_count);
    assert!(edit.as_part().fin(fin_id).is_ok());
    drop(edit);

    let check = session
        .part(part_id)
        .unwrap()
        .check_body(CheckBodyRequest::new(body, CheckLevel::Fast))
        .unwrap();
    assert_eq!(check.result().unwrap().outcome(), CheckOutcome::Valid);
}

#[test]
fn facade_only_strut_requests_hide_points_and_remain_failure_atomic() {
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
    let (loop_id, edge_id, curve, pcurve) = {
        let part = session.part(part_id.clone()).unwrap();
        let face = part
            .body(body.clone())
            .unwrap()
            .faces()
            .unwrap()
            .next()
            .unwrap();
        let loop_id = part.face(face).unwrap().loops().next().unwrap();
        let fin_id = part.loop_(loop_id.clone()).unwrap().fins().next().unwrap();
        let fin = part.fin(fin_id).unwrap();
        let edge_id = fin.edge();
        let edge = part.edge(edge_id.clone()).unwrap();
        let (lo, hi) = edge.bounds().unwrap();
        let curve = BoundedCurve::new(edge.curve().unwrap(), ParamRange::new(lo, hi));
        let pcurve = BoundedPcurve::new(fin.pcurve().unwrap(), fin.pcurve_range().unwrap())
            .with_parameter_map(fin.pcurve_parameter_map().unwrap())
            .with_metadata(fin.pcurve_metadata().unwrap());
        (loop_id, edge_id, curve, pcurve)
    };
    let request = CreateStrutRequest::new(
        loop_id.clone(),
        0,
        curve,
        Point3::new(f64::NAN, 0.0, 0.0),
        [pcurve.clone(), pcurve],
    );
    assert_eq!(request.loop_id(), loop_id);
    assert_eq!(request.fin_index(), 0);
    assert!(request.position().x.is_nan());
    assert_eq!(request.pcurves().len(), 2);

    let mut edit = session.edit_part(part_id.clone()).unwrap();
    let original = (
        edit.as_part().edges().len(),
        edit.as_part().vertices().len(),
        edit.as_part().fins().len(),
    );
    let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
    assert!(transaction.create_strut(request).is_err());
    transaction.rollback().unwrap();
    let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
    assert!(
        transaction
            .remove_strut(RemoveStrutRequest::new(edge_id.clone()))
            .is_err()
    );
    transaction.rollback().unwrap();
    assert_eq!(
        (
            edit.as_part().edges().len(),
            edit.as_part().vertices().len(),
            edit.as_part().fins().len(),
        ),
        original
    );
    assert!(edit.as_part().edge(edge_id).is_ok());
    drop(edit);

    let check = session
        .part(part_id)
        .unwrap()
        .check_body(CheckBodyRequest::new(body, CheckLevel::Fast))
        .unwrap();
    assert_eq!(check.result().unwrap().outcome(), CheckOutcome::Valid);
}

#[test]
fn facade_only_seed_body_round_trip_is_explicitly_transient_and_checked() {
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
    let (surface, sense, position) = {
        let part = session.part(part_id.clone()).unwrap();
        let face_id = part
            .body(body.clone())
            .unwrap()
            .faces()
            .unwrap()
            .next()
            .unwrap();
        let face = part.face(face_id).unwrap();
        let loop_id = face.loops().next().unwrap();
        let fin_id = part.loop_(loop_id).unwrap().fins().next().unwrap();
        let vertex = part.fin(fin_id).unwrap().tail().unwrap().unwrap();
        (
            face.surface(),
            face.sense(),
            part.vertex(vertex).unwrap().position().unwrap(),
        )
    };
    let request = CreateSeedBodyRequest::new(surface.clone(), sense, position);
    assert_eq!(request.surface(), surface);
    assert_eq!(request.sense(), sense);
    assert_eq!(request.position(), position);

    let mut edit = session.edit_part(part_id.clone()).unwrap();
    let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
    let seed = transaction.create_seed_body(request).unwrap();
    assert_eq!(format!("{:?}", seed.body()), "BodyId(<opaque>)");
    assert_eq!(format!("{:?}", seed.void_region()), "RegionId(<opaque>)");
    assert_eq!(format!("{:?}", seed.solid_region()), "RegionId(<opaque>)");
    assert_eq!(format!("{:?}", seed.shell()), "ShellId(<opaque>)");
    assert_eq!(format!("{:?}", seed.face()), "FaceId(<opaque>)");
    assert_eq!(format!("{:?}", seed.loop_id()), "LoopId(<opaque>)");
    assert_eq!(format!("{:?}", seed.vertex()), "VertexId(<opaque>)");
    let removed = transaction
        .remove_seed_body(RemoveSeedBodyRequest::new(seed.body()))
        .unwrap();
    assert_eq!(removed.body(), seed.body());
    let journal = transaction
        .commit(core::slice::from_ref(&body))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(journal.lineage_count(), 3);
    assert!(matches!(
        edit.as_part().body(seed.body()),
        Err(Error::StaleEntity {
            kind: EntityKind::Body
        })
    ));
    drop(edit);

    let check = session
        .part(part_id)
        .unwrap()
        .check_body(CheckBodyRequest::new(body, CheckLevel::Fast))
        .unwrap();
    assert_eq!(check.result().unwrap().outcome(), CheckOutcome::Valid);
}

#[test]
fn facade_only_face_hole_requests_are_failure_atomic_persistence_candidates() {
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
    let (face_id, loop_id, surface, sense) = {
        let part = session.part(part_id.clone()).unwrap();
        let face_id = part
            .body(body.clone())
            .unwrap()
            .faces()
            .unwrap()
            .next()
            .unwrap();
        let face = part.face(face_id.clone()).unwrap();
        (
            face_id,
            face.loops().next().unwrap(),
            face.surface(),
            face.sense(),
        )
    };
    let merge = MergeFaceAsHoleRequest::new(face_id.clone(), face_id.clone());
    assert_eq!(merge.keep(), face_id);
    assert_eq!(merge.remove(), face_id);
    let split = SplitHoleAsFaceRequest::new(loop_id.clone(), surface.clone(), sense);
    assert_eq!(split.ring(), loop_id);
    assert_eq!(split.surface(), surface);
    assert_eq!(split.sense(), sense);

    let mut edit = session.edit_part(part_id.clone()).unwrap();
    let original_face_count = edit.as_part().faces().len();
    let original_loop_count = edit.as_part().loops().len();
    let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
    assert!(transaction.merge_face_as_hole(merge).is_err());
    assert!(transaction.split_hole_as_face(split).is_err());
    transaction.rollback().unwrap();
    assert_eq!(edit.as_part().faces().len(), original_face_count);
    assert_eq!(edit.as_part().loops().len(), original_loop_count);
    drop(edit);

    let check = session
        .part(part_id)
        .unwrap()
        .check_body(CheckBodyRequest::new(body, CheckLevel::Fast))
        .unwrap();
    assert_eq!(check.result().unwrap().outcome(), CheckOutcome::Valid);
}

#[test]
fn facade_only_client_can_tessellate_with_opaque_topology_identity() {
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
    let options = TessOptions {
        chord_tol: 1.0e-3,
        max_edge_len: None,
    };
    let part = session.part(part_id).unwrap();
    let bounded = BodyTessellationBudgetProfile::bounded_v1();
    let first = part
        .tessellate_body(
            TessellateBodyRequest::new(body.clone(), options)
                .with_settings(OperationSettings::new().with_budget_overrides(bounded.clone())),
        )
        .unwrap();
    assert!(first.report().limit_events().is_empty());
    let mesh = first.result().unwrap();
    assert_eq!(mesh.body(), body.clone());
    assert_eq!(mesh.positions().len(), 8);
    assert_eq!(mesh.triangles().len(), 12);
    assert_eq!(mesh.face_triangle_ranges().len(), 6);
    assert_eq!(mesh.edge_polylines().len(), 12);
    assert!(
        mesh.face_triangle_ranges()
            .iter()
            .all(|range| { !range.range().is_empty() && part.face(range.face()).is_ok() })
    );
    assert!(
        mesh.edge_polylines()
            .iter()
            .all(|line| { line.vertex_indices().len() == 2 && part.edge(line.edge()).is_ok() })
    );
    assert!(mesh.positions().iter().all(|point| {
        point
            .to_array()
            .iter()
            .all(|coordinate| coordinate.is_finite())
    }));

    let second = part
        .tessellate_body(
            TessellateBodyRequest::new(body, options)
                .with_settings(OperationSettings::new().with_budget_overrides(bounded)),
        )
        .unwrap();
    assert_eq!(first, second);
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
    assert_eq!(outcome.report().usage().len(), 11);
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
