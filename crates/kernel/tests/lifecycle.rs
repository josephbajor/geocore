//! Facade-only lifecycle tests: no lower-layer crate is imported.

use kernel::{
    AccountingMode, BOOLEAN_BSP_WORK, BOOLEAN_POST_SELECTION_WORK, BOOLEAN_REALIZED_VERTICES,
    BlockRequest, BodyId, BodyKind, BodyTessellationBudgetProfile, BooleanBodiesRequest,
    BooleanOperation, BooleanOutcome, BooleanRefusal, BooleanResult, BoundedCurve, BoundedPcurve,
    BudgetPlan, CheckBodyRequest, CheckLevel, CheckOutcome, ClassifyPointInBodyRequest,
    CreateSeedBodyRequest, CreateStrutRequest, CylinderRequest, EntityKind, Error, ExecutionPolicy,
    ExportXtRequest, ExtrudeProfileAlongRequest, ExtrudeProfileRequest, Frame,
    FullCommitRequirement, GrowTolerancesRequest, ImportXtRequest, IntersectCurvesRequest,
    JoinRingRequest, JournalEntity, Kernel, LimitSpec, LineageView, MergeFaceAsHoleRequest,
    MutationKind, NumericalPolicy, OperationSettings, ParamRange, PartId, PcurveChart,
    PcurveEndpointKind, PcurveMetadata, PcurveSeam, PcurveSeamSide, Point2, Point3, PolicyVersion,
    RegionKind, RemoveBridgeRequest, RemoveSeedBodyRequest, RemoveStrutRequest, ResourceKind,
    SECTION_WORK, SectionBodiesRequest, SectionCompletion, SectionCurveFragmentSpan, SectionRing,
    Session, SessionPolicy, SessionPrecision, SplitHoleAsFaceRequest, SurfaceDerivativeOrder,
    SurfaceEvaluationRequest, SurfaceParameter, TessOptions, TessellateBodyRequest,
    ToleranceGrowth, ToleranceGrowthTarget, Tolerances, Vec3,
};

#[path = "lifecycle/body_distance.rs"]
mod body_distance;
#[path = "lifecycle/bounded_skew_body_properties.rs"]
mod bounded_skew_body_properties;
#[path = "lifecycle/bounded_skew_contact_roots.rs"]
mod bounded_skew_contact_roots;
#[path = "lifecycle/bounded_skew_xt.rs"]
mod bounded_skew_xt;
#[path = "lifecycle/cap_crossing_secant.rs"]
mod cap_crossing_secant;
#[path = "lifecycle/curved_cavity.rs"]
mod curved_cavity;
#[path = "lifecycle/curved_constructive_contact.rs"]
mod curved_constructive_contact;
#[path = "lifecycle/curved_cylinder_cylinder_rulings.rs"]
mod curved_cylinder_cylinder_rulings;
#[path = "lifecycle/curved_inverse_cavity.rs"]
mod curved_inverse_cavity;
#[path = "lifecycle/curved_one_port_budget.rs"]
mod curved_one_port_budget;
#[path = "lifecycle/curved_plane_cylinder_rulings.rs"]
mod curved_plane_cylinder_rulings;
#[path = "lifecycle/curved_support_contact.rs"]
mod curved_support_contact;
#[path = "lifecycle/curved_two_port.rs"]
mod curved_two_port;
#[path = "lifecycle/curved_two_ring_union.rs"]
mod curved_two_ring_union;
#[path = "lifecycle/mixed_plane_cylinder_cycles.rs"]
mod mixed_plane_cylinder_cycles;
#[path = "lifecycle/parallel_cylinder_boolean.rs"]
mod parallel_cylinder_boolean;

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
    assert_eq!(created.journal().face_tolerance_propagation_count(), 0);
    assert_eq!(created.journal().face_tolerance_propagations().len(), 0);

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
fn facade_only_client_can_construct_and_check_a_cylinder_with_reports() {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let rejected = session
        .edit_part(part_id.clone())
        .unwrap()
        .create_cylinder(CylinderRequest::new(Frame::world(), -1.0, 3.0))
        .unwrap();
    assert!(rejected.into_result().is_err());
    assert_eq!(session.part(part_id.clone()).unwrap().bodies().len(), 0);

    let creation = session
        .edit_part(part_id.clone())
        .unwrap()
        .create_cylinder(CylinderRequest::new(Frame::world(), 1.25, 3.0))
        .unwrap();
    assert!(creation.report().usage().is_empty());
    let created = creation.into_result().unwrap();
    assert_eq!(created.journal().part(), part_id);
    assert!(created.journal().mutation_count() > 0);
    assert_eq!(created.journal().lineage_count(), 0);

    let part = session.part(part_id.clone()).unwrap();
    let body = part.body(created.body()).unwrap();
    assert_eq!(body.kind(), BodyKind::Solid);
    assert_eq!(body.faces().unwrap().len(), 3);
    assert_eq!(body.edges().unwrap().len(), 2);
    assert_eq!(body.vertices().unwrap().len(), 0);

    let check = part
        .check_body(CheckBodyRequest::new(created.body(), CheckLevel::Full))
        .unwrap();
    assert_eq!(check.result().unwrap().outcome(), CheckOutcome::Valid);
    assert!(check.result().unwrap().gaps().is_empty());
    assert!(check.report().limit_events().is_empty());
}

#[test]
fn facade_only_client_can_observe_exact_plane_cylinder_section_rings() {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let block = edit
            .create_block(BlockRequest::new(
                Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
                [4.0, 4.0, 1.0],
            ))
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
        (block, cylinder)
    };

    let outcome = session
        .part(part_id)
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(block, cylinder))
        .unwrap();
    let graph = outcome.into_result().unwrap();
    assert_eq!(graph.completion(), SectionCompletion::Complete);
    let rings: &[SectionRing] = graph.rings();
    assert_eq!(rings.len(), 2);
    assert!(
        rings
            .iter()
            .all(|ring| ring.branch() < graph.branches().len())
    );
    assert!(
        graph
            .curve_fragments()
            .iter()
            .all(|fragment| matches!(fragment.span(), SectionCurveFragmentSpan::Whole))
    );
    assert!(graph.gaps().is_empty());
}

#[test]
fn facade_only_client_can_extrude_a_holed_polygonal_profile() {
    let outer = vec![
        Point2::new(-2.0, -2.0),
        Point2::new(2.0, -2.0),
        Point2::new(2.0, 2.0),
        Point2::new(-2.0, 2.0),
    ];
    let hole = vec![
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ];
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let outcome = session
        .edit_part(part_id.clone())
        .unwrap()
        .extrude_profile(ExtrudeProfileRequest::new(
            Frame::world(),
            outer,
            vec![hole],
            2.0,
        ))
        .unwrap();
    assert!(outcome.report().usage().is_empty());
    let created = outcome.into_result().unwrap();
    assert_eq!(created.journal().part(), part_id);
    assert!(
        created
            .journal()
            .mutations()
            .all(|mutation| mutation.kind() == MutationKind::Created)
    );
    assert_eq!(created.journal().lineage_count(), 0);

    let part = session.part(part_id).unwrap();
    assert_eq!(part.faces().len(), 10);
    assert_eq!(part.edges().len(), 24);
    let check = part
        .check_body(CheckBodyRequest::new(created.body(), CheckLevel::Full))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(check.outcome(), CheckOutcome::Valid);
}

#[test]
fn facade_only_client_can_extrude_a_holed_profile_obliquely() {
    let outer = vec![
        Point2::new(-2.0, -1.0),
        Point2::new(2.0, -1.0),
        Point2::new(2.0, 3.0),
        Point2::new(-2.0, 3.0),
    ];
    let hole = vec![
        Point2::new(-1.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(1.0, 2.0),
        Point2::new(-1.0, 2.0),
    ];
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let created = session
        .edit_part(part_id.clone())
        .unwrap()
        .extrude_profile_along(ExtrudeProfileAlongRequest::new(
            Frame::world(),
            outer,
            vec![hole],
            Vec3::new(0.75, -0.5, -2.0),
        ))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(created.journal().part(), part_id);
    assert!(
        created
            .journal()
            .mutations()
            .all(|mutation| mutation.kind() == MutationKind::Created)
    );

    let part = session.part(part_id).unwrap();
    assert_eq!(part.faces().len(), 10);
    assert_eq!(part.edges().len(), 24);
    let check = part
        .check_body(CheckBodyRequest::new(created.body(), CheckLevel::Full))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(check.outcome(), CheckOutcome::Valid);
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
fn facade_only_tolerance_batch_is_journal_scoped_and_request_ordered() {
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
    let (face_id, edge_id, vertex_id) = {
        let part = session.part(part_id.clone()).unwrap();
        let face_id = part
            .body(body.clone())
            .unwrap()
            .faces()
            .unwrap()
            .next()
            .unwrap();
        let loop_id = part.face(face_id.clone()).unwrap().loops().next().unwrap();
        let fin_id = part.loop_(loop_id).unwrap().fins().next().unwrap();
        let edge_id = part.fin(fin_id).unwrap().edge();
        let vertex_id = part.edge(edge_id.clone()).unwrap().vertices()[0]
            .clone()
            .unwrap();
        (face_id, edge_id, vertex_id)
    };
    let resolution = OperationSettings::default().tolerances().linear();
    let growth = vec![
        ToleranceGrowth::new(
            ToleranceGrowthTarget::Vertex(vertex_id.clone()),
            2.0 * resolution,
        ),
        ToleranceGrowth::new(
            ToleranceGrowthTarget::Face(face_id.clone()),
            3.0 * resolution,
        ),
        ToleranceGrowth::new(
            ToleranceGrowthTarget::Edge(edge_id.clone()),
            4.0 * resolution,
        ),
    ];
    let request = GrowTolerancesRequest::new("facade-lifecycle-heal", 6.0 * resolution, growth);
    assert_eq!(request.operation(), "facade-lifecycle-heal");
    assert_eq!(request.max_total_growth(), 6.0 * resolution);
    assert_eq!(request.growth().len(), 3);
    assert!(matches!(
        request.growth()[0].target(),
        ToleranceGrowthTarget::Vertex(id) if id == &vertex_id
    ));
    assert_eq!(request.growth()[0].requested(), 2.0 * resolution);

    let mut edit = session.edit_part(part_id).unwrap();
    let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
    let applied = transaction.grow_tolerances(request).unwrap();
    let journal = transaction
        .commit(core::slice::from_ref(&body))
        .unwrap()
        .into_result()
        .unwrap();
    let budget = journal.tolerance_budget(applied.budget()).unwrap();
    assert_eq!(budget.id(), applied.budget());
    assert_eq!(budget.operation(), "facade-lifecycle-heal");
    assert_eq!(budget.consumed(), 6.0 * resolution);
    let events = journal.tolerance_events().collect::<Vec<_>>();
    assert_eq!(events.len(), 3);
    assert_eq!(events[0].entity().kind(), EntityKind::Vertex);
    assert_eq!(events[1].entity().kind(), EntityKind::Face);
    assert_eq!(events[2].entity().kind(), EntityKind::Edge);
    assert!(
        events
            .iter()
            .all(|event| event.budget() == applied.budget())
    );
    assert_eq!(
        edit.as_part()
            .vertex(vertex_id)
            .unwrap()
            .tolerance()
            .unwrap()
            .value(),
        2.0 * resolution
    );
    assert_eq!(
        edit.as_part()
            .face(face_id)
            .unwrap()
            .tolerance()
            .unwrap()
            .value(),
        3.0 * resolution
    );
    assert_eq!(
        edit.as_part()
            .edge(edge_id)
            .unwrap()
            .tolerance()
            .unwrap()
            .value(),
        4.0 * resolution
    );
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
fn facade_only_client_can_full_commit_with_owned_evidence() {
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
    let face = session
        .part(part_id.clone())
        .unwrap()
        .body(body.clone())
        .unwrap()
        .faces()
        .unwrap()
        .next()
        .unwrap();
    let resolution = OperationSettings::default().tolerances().linear();
    let mut edit = session.edit_part(part_id).unwrap();
    let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
    let applied = transaction
        .grow_tolerances(GrowTolerancesRequest::new(
            "facade-full-commit",
            resolution,
            vec![ToleranceGrowth::new(
                ToleranceGrowthTarget::Face(face.clone()),
                2.0 * resolution,
            )],
        ))
        .unwrap();
    let outcome = transaction
        .commit_full(
            core::slice::from_ref(&body),
            FullCommitRequirement::RequireValid,
        )
        .unwrap();
    assert!(!outcome.report().usage().is_empty());
    let committed = outcome.into_result().unwrap();
    assert!(committed.is_committed());
    assert_eq!(committed.reports().len(), 1);
    assert_eq!(committed.reports()[0].body(), body);
    assert_eq!(
        committed.reports()[0].report().outcome(),
        CheckOutcome::Valid
    );
    let journal = committed.journal().unwrap();
    assert_eq!(
        journal
            .tolerance_budget(applied.budget())
            .unwrap()
            .consumed(),
        resolution
    );
    assert_eq!(journal.tolerance_events().len(), 1);
    assert_eq!(
        edit.as_part()
            .face(face)
            .unwrap()
            .tolerance()
            .unwrap()
            .value(),
        2.0 * resolution
    );
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

struct BooleanFixture {
    session: Session,
    part: PartId,
    left: BodyId,
    right: BodyId,
}

fn boolean_frame(origin: [f64; 3], z: [f64; 3], x: [f64; 3]) -> Frame {
    Frame::new(
        Point3::from_array(origin),
        Vec3::from_array(z),
        Vec3::from_array(x),
    )
    .unwrap()
}

fn boolean_fixture(
    left_frame: Frame,
    left_extents: [f64; 3],
    right_frame: Frame,
    right_extents: [f64; 3],
) -> BooleanFixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (left, right) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let left = edit
            .create_block(BlockRequest::new(left_frame, left_extents))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let right = edit
            .create_block(BlockRequest::new(right_frame, right_extents))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (left, right)
    };
    BooleanFixture {
        session,
        part,
        left,
        right,
    }
}

fn block_cylinder_boolean_fixture(
    block_frame: Frame,
    block_extents: [f64; 3],
    cylinder_frame: Frame,
    radius: f64,
    height: f64,
) -> BooleanFixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (left, right) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let block = edit
            .create_block(BlockRequest::new(block_frame, block_extents))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(cylinder_frame, radius, height))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    BooleanFixture {
        session,
        part,
        left,
        right,
    }
}

fn one_ring_cylindrical_boss_fixture() -> BooleanFixture {
    block_cylinder_boolean_fixture(Frame::world(), [4.0, 4.0, 2.0], Frame::world(), 0.75, 2.0)
}

fn one_ring_blind_pocket_fixture() -> BooleanFixture {
    block_cylinder_boolean_fixture(Frame::world(), [4.0, 4.0, 2.0], Frame::world(), 0.75, 2.0)
}

fn overlapping_boolean_fixture() -> BooleanFixture {
    boolean_fixture(
        boolean_frame([1.25, -0.75, 0.5], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),
        [3.5, 2.75, 2.5],
        boolean_frame([1.75, -0.25, 0.75], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        [3.0, 2.0, 2.75],
    )
}

fn disjoint_boolean_fixture() -> BooleanFixture {
    boolean_fixture(
        Frame::world().with_origin(Point3::new(-8.0, 1.0, 0.5)),
        [2.0, 1.5, 2.5],
        boolean_frame([8.0, -1.0, -0.5], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),
        [1.75, 2.25, 1.25],
    )
}

fn contained_boolean_fixture() -> BooleanFixture {
    boolean_fixture(
        Frame::world(),
        [6.0, 5.0, 4.0],
        Frame::world().with_origin(Point3::new(0.25, -0.2, 0.1)),
        [1.5, 1.0, 0.8],
    )
}

fn run_boolean(
    fixture: &mut BooleanFixture,
    operation: BooleanOperation,
    settings: OperationSettings,
) -> kernel::OperationOutcome<BooleanOutcome> {
    let request = BooleanBodiesRequest::new(operation, fixture.left.clone(), fixture.right.clone())
        .with_settings(settings);
    fixture
        .session
        .edit_part(fixture.part.clone())
        .unwrap()
        .boolean_bodies(request)
        .unwrap()
}

fn boolean_success(outcome: kernel::OperationOutcome<BooleanOutcome>) -> BooleanResult {
    match outcome.into_result().unwrap() {
        BooleanOutcome::Success(result) => result,
        BooleanOutcome::Refused(refusal) => panic!("unexpected Boolean refusal: {refusal:?}"),
        other => panic!("unexpected Boolean outcome: {other:?}"),
    }
}

fn assert_boolean_sources_retained(fixture: &BooleanFixture, expected_body_count: usize) {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    assert_eq!(part.bodies().len(), expected_body_count);
    assert_eq!(
        part.body(fixture.left.clone()).unwrap().kind(),
        BodyKind::Solid
    );
    assert_eq!(
        part.body(fixture.right.clone()).unwrap().kind(),
        BodyKind::Solid
    );
}

fn boolean_topology_counts(fixture: &BooleanFixture) -> [usize; 8] {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    [
        part.bodies().len(),
        part.regions().len(),
        part.shells().len(),
        part.faces().len(),
        part.loops().len(),
        part.fins().len(),
        part.edges().len(),
        part.vertices().len(),
    ]
}

fn boolean_body_x_center(fixture: &BooleanFixture, body: BodyId) -> f64 {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let body = part.body(body).unwrap();
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for vertex in body.vertices().unwrap() {
        let x = part.vertex(vertex).unwrap().position().unwrap().x;
        min = min.min(x);
        max = max.max(x);
    }
    (min + max) * 0.5
}

fn boolean_body_topology_signature(fixture: &BooleanFixture, body: BodyId) -> [usize; 3] {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let body = part.body(body).unwrap();
    [
        body.faces().unwrap().len(),
        body.edges().unwrap().len(),
        body.vertices().unwrap().len(),
    ]
}

fn assert_boolean_created_full_valid(created: &kernel::BooleanCreatedResult) {
    assert_eq!(created.reports().len(), created.bodies().len());
    assert!(
        created
            .reports()
            .iter()
            .zip(created.bodies())
            .all(|(report, body)| report.body() == *body
                && report.report().level() == CheckLevel::Full
                && report.report().outcome() == CheckOutcome::Valid
                && report.report().faults().is_empty()
                && report.report().gaps().is_empty())
    );
}

fn assert_whole_source_copy_lineage(
    fixture: &BooleanFixture,
    created: &kernel::BooleanCreatedResult,
    expected_sources: &[BodyId],
) {
    assert_eq!(created.journal().part(), fixture.part);
    let mutations = created.journal().mutations().collect::<Vec<_>>();
    assert!(!mutations.is_empty());
    assert!(
        mutations
            .iter()
            .all(|mutation| mutation.kind() == MutationKind::Created)
    );
    assert_eq!(created.journal().lineage_count(), mutations.len());

    let mut derived = Vec::new();
    let mut body_pairs = Vec::new();
    let mut face_pairs = Vec::new();
    for event in created.journal().lineage() {
        let LineageView::DerivedFrom {
            derived: derived_entity,
            source,
        } = event
        else {
            panic!("whole-source copy lineage must contain only DerivedFrom events")
        };
        assert!(!derived.contains(&derived_entity));
        derived.push(derived_entity.clone());
        match (derived_entity, source) {
            (JournalEntity::Body(result), JournalEntity::Body(source)) => {
                body_pairs.push((result, source));
            }
            (JournalEntity::Face(result), JournalEntity::Face(source)) => {
                face_pairs.push((result, source));
            }
            (derived, source) => assert_eq!(derived.kind(), source.kind()),
        }
    }
    assert_eq!(derived.len(), mutations.len());
    assert!(
        mutations
            .iter()
            .all(|mutation| derived.contains(mutation.entity()))
    );
    assert_eq!(
        body_pairs,
        created
            .bodies()
            .iter()
            .cloned()
            .zip(expected_sources.iter().cloned())
            .collect::<Vec<_>>()
    );

    let part = fixture.session.part(fixture.part.clone()).unwrap();
    for (result, source) in created.bodies().iter().zip(expected_sources) {
        let result_faces = part
            .body(result.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>();
        let source_faces = part
            .body(source.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>();
        assert_eq!(
            face_pairs
                .iter()
                .filter(|(derived, original)| {
                    result_faces.contains(derived) && source_faces.contains(original)
                })
                .count(),
            result_faces.len()
        );
        assert!(result_faces.iter().all(|face| {
            face_pairs
                .iter()
                .filter(|(derived, source)| derived == face && source_faces.contains(source))
                .count()
                == 1
        }));
    }
}

fn assert_deterministic_xt_and_fast_self_import(
    fixture: &mut BooleanFixture,
    bodies: &[BodyId],
) -> Vec<Vec<u8>> {
    let exports = {
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        bodies
            .iter()
            .map(|body| {
                let first = part
                    .export_xt(ExportXtRequest::new(body.clone()))
                    .unwrap()
                    .into_result()
                    .unwrap();
                let second = part
                    .export_xt(ExportXtRequest::new(body.clone()))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(first.bytes(), second.bytes());
                first.bytes().to_vec()
            })
            .collect::<Vec<_>>()
    };

    let imported_part = fixture.session.create_part();
    for bytes in &exports {
        let imported = fixture
            .session
            .edit_part(imported_part.clone())
            .unwrap()
            .import_xt(ImportXtRequest::new(bytes))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(imported.bodies().len(), 1);
        let report = fixture
            .session
            .part(imported_part.clone())
            .unwrap()
            .check_body(CheckBodyRequest::new(
                imported.bodies()[0].clone(),
                CheckLevel::Fast,
            ))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(report.outcome(), CheckOutcome::Valid);
    }
    assert_eq!(
        fixture.session.part(imported_part).unwrap().bodies().len(),
        bodies.len()
    );
    exports
}

#[test]
fn public_boolean_connected_operations_commit_one_full_valid_body_and_retain_sources() {
    for operation in [
        BooleanOperation::Unite,
        BooleanOperation::Intersect,
        BooleanOperation::Subtract,
    ] {
        let mut fixture = overlapping_boolean_fixture();
        let request =
            BooleanBodiesRequest::new(operation, fixture.left.clone(), fixture.right.clone());
        assert_eq!(request.operation(), operation);
        assert_eq!(request.left(), fixture.left);
        assert_eq!(request.right(), fixture.right);
        assert_eq!(request.settings(), &OperationSettings::default());

        let result = boolean_success(run_boolean(
            &mut fixture,
            operation,
            OperationSettings::new(),
        ));
        let BooleanResult::Created(created) = result else {
            panic!("connected Boolean must create topology")
        };
        assert_eq!(created.bodies().len(), 1);
        assert_eq!(created.reports().len(), created.bodies().len());
        assert!(
            created
                .reports()
                .iter()
                .zip(created.bodies())
                .all(|(report, body)| report.body() == *body
                    && report.report().level() == CheckLevel::Full
                    && report.report().outcome() == CheckOutcome::Valid)
        );
        assert_eq!(created.journal().part(), fixture.part);
        assert!(created.journal().mutation_count() > 0);
        assert_boolean_sources_retained(&fixture, 3);
    }
}

#[test]
fn public_boolean_axial_block_cylinder_intersection_commits_a_full_valid_band() {
    for swapped in [false, true] {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let base = Point3::new(3.0, -2.0, 1.25);
        let cylinder_frame = boolean_frame(base.to_array(), [0.0, 0.6, 0.8], [1.0, 0.0, 0.0]);
        let block_frame = cylinder_frame.with_origin(base + cylinder_frame.z());
        let (block, cylinder) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
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
            (block, cylinder)
        };
        let (left, right) = if swapped {
            (cylinder.clone(), block.clone())
        } else {
            (block.clone(), cylinder.clone())
        };
        let outcome = session
            .edit_part(part_id.clone())
            .unwrap()
            .boolean_bodies(BooleanBodiesRequest::new(
                BooleanOperation::Intersect,
                left,
                right,
            ))
            .unwrap();
        let result = outcome.into_result().unwrap();
        let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
            panic!("axial block/cylinder intersection must create one band: {result:?}")
        };
        assert_eq!(created.bodies().len(), 1);
        assert_eq!(created.reports().len(), 1);
        assert_eq!(created.reports()[0].report().level(), CheckLevel::Full);
        assert_eq!(created.reports()[0].report().outcome(), CheckOutcome::Valid);
        assert!(created.reports()[0].report().gaps().is_empty());
        assert_eq!(created.journal().lineage_count(), 3);

        let result = created.bodies()[0].clone();
        let part = session.part(part_id.clone()).unwrap();
        assert_eq!(part.bodies().len(), 3);
        assert!(part.body(block).is_ok());
        assert!(part.body(cylinder).is_ok());
        let result_view = part.body(result.clone()).unwrap();
        assert_eq!(result_view.faces().unwrap().len(), 3);
        assert_eq!(result_view.edges().unwrap().len(), 2);
        assert_eq!(result_view.vertices().unwrap().len(), 0);
        let surface_classes = result_view
            .faces()
            .unwrap()
            .map(|face| {
                let face = part.face(face).unwrap();
                part.surface(face.surface()).unwrap().class_key().as_str()
            })
            .collect::<Vec<_>>();
        assert_eq!(
            surface_classes
                .iter()
                .filter(|class| **class == "kernel.surface.cylinder.v1")
                .count(),
            1
        );
        assert_eq!(
            surface_classes
                .iter()
                .filter(|class| **class == "kernel.surface.plane.v1")
                .count(),
            2
        );
        let interior = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(
                result.clone(),
                base + cylinder_frame.z(),
            ))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(interior.verdict(), &kernel::PointBodyVerdict::Interior);
        let exterior = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(
                result.clone(),
                base + cylinder_frame.z() * 0.25,
            ))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(exterior.verdict(), &kernel::PointBodyVerdict::Exterior);
        let first_xt = part
            .export_xt(ExportXtRequest::new(result.clone()))
            .unwrap()
            .into_result()
            .unwrap();
        let second_xt = part
            .export_xt(ExportXtRequest::new(result))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(first_xt.bytes(), second_xt.bytes());
        drop(part);

        let imported_part = session.create_part();
        let imported = session
            .edit_part(imported_part.clone())
            .unwrap()
            .import_xt(ImportXtRequest::new(first_xt.bytes()))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(imported.bodies().len(), 1);
        let imported_check = session
            .part(imported_part)
            .unwrap()
            .check_body(CheckBodyRequest::new(
                imported.bodies()[0].clone(),
                CheckLevel::Fast,
            ))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            imported_check.outcome(),
            CheckOutcome::Valid,
            "imported axial Boolean Fast report: {imported_check:#?}"
        );
    }
}

#[test]
fn public_curved_boolean_zero_cut_selection_handles_containment_and_empty() {
    let cylinder_frame = Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0));
    let mut contained =
        block_cylinder_boolean_fixture(Frame::world(), [6.0, 6.0, 6.0], cylinder_frame, 0.75, 2.0);
    let section = contained
        .session
        .part(contained.part.clone())
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(
            contained.left.clone(),
            contained.right.clone(),
        ))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(
        section.completion(),
        SectionCompletion::Complete,
        "contained zero-cut graph gaps: {:?}",
        section.gaps()
    );
    assert!(
        section.edges().is_empty()
            && section.loops().is_empty()
            && section.branches().is_empty()
            && section.rings().is_empty()
            && section.curve_endpoints().is_empty()
            && section.curve_fragments().is_empty()
            && section.curve_components().is_empty(),
        "contained zero-cut graph: {section:#?}"
    );
    let part = contained.session.part(contained.part.clone()).unwrap();
    for vertex in part
        .body(contained.left.clone())
        .unwrap()
        .vertices()
        .unwrap()
    {
        let point = part.vertex(vertex).unwrap().position().unwrap();
        let classification = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(
                contained.right.clone(),
                point,
            ))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            classification.verdict(),
            &kernel::PointBodyVerdict::Exterior,
            "block vertex {point:?}"
        );
    }
    for point in [
        Point3::new(0.75, 0.0, 0.0),
        Point3::new(0.0, 0.0, -1.0),
        Point3::new(0.0, 0.0, 1.0),
    ] {
        let classification = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(
                contained.left.clone(),
                point,
            ))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            classification.verdict(),
            &kernel::PointBodyVerdict::Interior,
            "cylinder boundary anchor {point:?}"
        );
    }
    drop(part);
    let result = boolean_success(run_boolean(
        &mut contained,
        BooleanOperation::Intersect,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("a contained finite cylinder must retain one complete source boundary")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_boolean_created_full_valid(&created);
    assert_whole_source_copy_lineage(&contained, &created, &[contained.right.clone()]);
    assert_boolean_sources_retained(&contained, 3);

    let mut disjoint = block_cylinder_boolean_fixture(
        Frame::world().with_origin(Point3::new(8.0, 0.0, 0.0)),
        [2.0, 2.0, 2.0],
        Frame::world().with_origin(Point3::new(-8.0, 0.0, -1.0)),
        0.75,
        2.0,
    );
    let result = boolean_success(run_boolean(
        &mut disjoint,
        BooleanOperation::Intersect,
        OperationSettings::new(),
    ));
    assert!(matches!(result, BooleanResult::ProvenEmpty));
    assert_boolean_sources_retained(&disjoint, 2);
}

#[test]
fn public_curved_boolean_zero_cut_whole_source_union_and_subtraction_commit_copies() {
    let cylinder_frame = Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0));
    for swapped in [false, true] {
        let mut fixture = block_cylinder_boolean_fixture(
            Frame::world(),
            [6.0, 6.0, 6.0],
            cylinder_frame,
            0.75,
            2.0,
        );
        let block = fixture.left.clone();
        if swapped {
            core::mem::swap(&mut fixture.left, &mut fixture.right);
        }
        let result = boolean_success(run_boolean(
            &mut fixture,
            BooleanOperation::Unite,
            OperationSettings::new(),
        ));
        let BooleanResult::Created(created) = result else {
            panic!("contained curved union must copy the complete outer boundary")
        };
        assert_eq!(created.bodies().len(), 1);
        assert_boolean_created_full_valid(&created);
        assert_eq!(
            boolean_body_topology_signature(&fixture, created.bodies()[0].clone()),
            [6, 12, 8]
        );
        assert_boolean_sources_retained(&fixture, 3);
        assert_whole_source_copy_lineage(&fixture, &created, &[block]);

        let result_body = created.bodies()[0].clone();
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        for (point, expected) in [
            (
                Point3::new(2.0, 0.0, 0.0),
                kernel::PointBodyVerdict::Interior,
            ),
            (
                Point3::new(4.0, 0.0, 0.0),
                kernel::PointBodyVerdict::Exterior,
            ),
        ] {
            let classification = part
                .classify_point_in_body(ClassifyPointInBodyRequest::new(result_body.clone(), point))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(classification.verdict(), &expected);
        }
        drop(part);
        assert_deterministic_xt_and_fast_self_import(&mut fixture, &[result_body]);
    }

    let mut union = block_cylinder_boolean_fixture(
        Frame::world().with_origin(Point3::new(8.0, 0.0, 0.0)),
        [2.0, 2.0, 2.0],
        Frame::world().with_origin(Point3::new(-8.0, 0.0, -1.0)),
        0.75,
        2.0,
    );
    let union_sources = [union.left.clone(), union.right.clone()];
    let result = boolean_success(run_boolean(
        &mut union,
        BooleanOperation::Unite,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("disjoint curved union must copy both complete source boundaries")
    };
    assert_eq!(created.bodies().len(), 2);
    assert_boolean_created_full_valid(&created);
    assert_eq!(
        created
            .bodies()
            .iter()
            .map(|body| boolean_body_topology_signature(&union, body.clone()))
            .collect::<Vec<_>>(),
        vec![[6, 12, 8], [3, 2, 0]]
    );
    assert_boolean_sources_retained(&union, 4);
    assert_whole_source_copy_lineage(&union, &created, &union_sources);
    let result_bodies = created.bodies().to_vec();
    let part = union.session.part(union.part.clone()).unwrap();
    for (body_index, point) in [Point3::new(8.0, 0.0, 0.0), Point3::new(-8.0, 0.0, 0.0)]
        .into_iter()
        .enumerate()
    {
        for (candidate, body) in result_bodies.iter().enumerate() {
            let classification = part
                .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
                .unwrap()
                .into_result()
                .unwrap();
            let expected = if candidate == body_index {
                kernel::PointBodyVerdict::Interior
            } else {
                kernel::PointBodyVerdict::Exterior
            };
            assert_eq!(classification.verdict(), &expected);
        }
    }
    drop(part);
    let exports = assert_deterministic_xt_and_fast_self_import(&mut union, &result_bodies);
    assert_ne!(exports[0], exports[1]);

    for swapped in [false, true] {
        let mut fixture = block_cylinder_boolean_fixture(
            Frame::world().with_origin(Point3::new(8.0, 0.0, 0.0)),
            [2.0, 2.0, 2.0],
            Frame::world().with_origin(Point3::new(-8.0, 0.0, -1.0)),
            0.75,
            2.0,
        );
        if swapped {
            core::mem::swap(&mut fixture.left, &mut fixture.right);
        }
        let expected_source = fixture.left.clone();
        let expected_signature = if swapped { [3, 2, 0] } else { [6, 12, 8] };
        let left_anchor = if swapped {
            Point3::new(-8.0, 0.0, 0.0)
        } else {
            Point3::new(8.0, 0.0, 0.0)
        };
        let right_anchor = if swapped {
            Point3::new(8.0, 0.0, 0.0)
        } else {
            Point3::new(-8.0, 0.0, 0.0)
        };
        let result = boolean_success(run_boolean(
            &mut fixture,
            BooleanOperation::Subtract,
            OperationSettings::new(),
        ));
        let BooleanResult::Created(created) = result else {
            panic!("disjoint curved subtraction must copy its complete left boundary")
        };
        assert_eq!(created.bodies().len(), 1);
        assert_boolean_created_full_valid(&created);
        assert_eq!(
            boolean_body_topology_signature(&fixture, created.bodies()[0].clone()),
            expected_signature
        );
        assert_boolean_sources_retained(&fixture, 3);
        assert_whole_source_copy_lineage(&fixture, &created, &[expected_source]);
        let result_body = created.bodies()[0].clone();
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        for (point, expected) in [
            (left_anchor, kernel::PointBodyVerdict::Interior),
            (right_anchor, kernel::PointBodyVerdict::Exterior),
        ] {
            let classification = part
                .classify_point_in_body(ClassifyPointInBodyRequest::new(result_body.clone(), point))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(classification.verdict(), &expected);
        }
        drop(part);
        assert_deterministic_xt_and_fast_self_import(&mut fixture, &[result_body]);
    }
}

fn assert_curved_subtraction_band_topology_and_lineage(
    fixture: &BooleanFixture,
    created: &kernel::BooleanCreatedResult,
) {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let mut result_faces = Vec::new();
    for body in created.bodies() {
        let body = part.body(body.clone()).unwrap();
        assert_eq!(body.kind(), BodyKind::Solid);
        assert_eq!(body.faces().unwrap().len(), 3);
        assert_eq!(body.edges().unwrap().len(), 2);
        assert_eq!(body.vertices().unwrap().len(), 0);
        let faces = body.faces().unwrap().collect::<Vec<_>>();
        let surface_classes = faces
            .iter()
            .map(|face| {
                let face = part.face(face.clone()).unwrap();
                part.surface(face.surface()).unwrap().class_key().as_str()
            })
            .collect::<Vec<_>>();
        assert_eq!(
            surface_classes
                .iter()
                .filter(|class| **class == "kernel.surface.cylinder.v1")
                .count(),
            1
        );
        assert_eq!(
            surface_classes
                .iter()
                .filter(|class| **class == "kernel.surface.plane.v1")
                .count(),
            2
        );

        let cylinder_face = faces
            .iter()
            .find(|face| {
                let face = part.face((*face).clone()).unwrap();
                part.surface(face.surface()).unwrap().class_key().as_str()
                    == "kernel.surface.cylinder.v1"
            })
            .unwrap();
        let cylinder_face = part.face(cylinder_face.clone()).unwrap();
        let cylinder_eval = part
            .evaluate_surface(SurfaceEvaluationRequest::new(
                cylinder_face.surface(),
                [0.0, 0.0],
                SurfaceDerivativeOrder::First,
            ))
            .unwrap()
            .into_result()
            .unwrap();
        let axis = cylinder_eval.derivatives().dv;
        let mut caps = faces
            .iter()
            .filter_map(|face| {
                let face = part.face(face.clone()).unwrap();
                (part.surface(face.surface()).unwrap().class_key().as_str()
                    == "kernel.surface.plane.v1")
                    .then_some(face)
            })
            .map(|face| {
                let evaluation = part
                    .evaluate_surface(SurfaceEvaluationRequest::new(
                        face.surface(),
                        [0.0, 0.0],
                        SurfaceDerivativeOrder::First,
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                let derivatives = evaluation.derivatives();
                let position = derivatives.p;
                let normal_sign = derivatives.du.cross(derivatives.dv).dot(axis)
                    * if face.sense().is_forward() { 1.0 } else { -1.0 };
                (
                    position.x * axis.x + position.y * axis.y + position.z * axis.z,
                    normal_sign,
                )
            })
            .collect::<Vec<_>>();
        caps.sort_by(|left, right| left.0.total_cmp(&right.0));
        assert_eq!(caps.len(), 2);
        assert!(caps[0].1 < 0.0, "low cap must point against the band axis");
        assert!(caps[1].1 > 0.0, "high cap must point with the band axis");
        result_faces.extend(faces);
    }

    let source_faces = [fixture.left.clone(), fixture.right.clone()]
        .into_iter()
        .flat_map(|body| {
            part.body(body)
                .unwrap()
                .faces()
                .unwrap()
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let mut derived_faces = Vec::new();
    let mut cylindrical_sources = 0;
    let mut planar_sources = 0;
    for event in created.journal().lineage() {
        let LineageView::DerivedFrom {
            derived: JournalEntity::Face(derived),
            source: JournalEntity::Face(source),
        } = event
        else {
            panic!("curved subtraction lineage must derive result faces from source faces")
        };
        assert!(result_faces.contains(&derived));
        assert!(source_faces.contains(&source));
        assert!(!derived_faces.contains(&derived));
        derived_faces.push(derived);
        let source_face = part.face(source).unwrap();
        match part
            .surface(source_face.surface())
            .unwrap()
            .class_key()
            .as_str()
        {
            "kernel.surface.cylinder.v1" => cylindrical_sources += 1,
            "kernel.surface.plane.v1" => planar_sources += 1,
            class => panic!("unexpected curved subtraction lineage surface: {class}"),
        }
    }
    assert_eq!(derived_faces.len(), result_faces.len());
    assert_eq!(cylindrical_sources, 2);
    assert_eq!(planar_sources, 4);
}

#[test]
fn public_curved_boolean_axial_cylinder_subtraction_commits_two_ordered_full_valid_bands() {
    let base = Point3::new(3.0, -2.0, 1.25);
    let cylinder_frame = boolean_frame(base.to_array(), [0.0, 0.6, 0.8], [1.0, 0.0, 0.0]);
    let mut fixture = block_cylinder_boolean_fixture(
        cylinder_frame.with_origin(base + cylinder_frame.z()),
        [4.0, 4.0, 1.0],
        cylinder_frame,
        0.75,
        2.0,
    );
    core::mem::swap(&mut fixture.left, &mut fixture.right);

    let result = boolean_success(run_boolean(
        &mut fixture,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("an axial cylinder-minus-block subtraction must create two bands")
    };
    assert_eq!(created.bodies().len(), 2);
    assert_eq!(created.reports().len(), 2);
    assert!(
        created
            .reports()
            .iter()
            .zip(created.bodies())
            .all(|(report, body)| report.body() == *body
                && report.report().level() == CheckLevel::Full
                && report.report().outcome() == CheckOutcome::Valid
                && report.report().faults().is_empty()
                && report.report().gaps().is_empty())
    );
    assert_eq!(created.journal().part(), fixture.part);
    assert!(created.journal().mutation_count() > 0);
    assert_eq!(created.journal().lineage_count(), 6);
    assert_boolean_sources_retained(&fixture, 4);
    assert_curved_subtraction_band_topology_and_lineage(&fixture, &created);

    let bodies = created.bodies().to_vec();
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let low_anchor = base + cylinder_frame.z() * 0.25;
    let removed_anchor = base + cylinder_frame.z();
    let high_anchor = base + cylinder_frame.z() * 1.75;
    for (body, inside, outside) in [
        (bodies[0].clone(), low_anchor, high_anchor),
        (bodies[1].clone(), high_anchor, low_anchor),
    ] {
        for (point, expected) in [
            (inside, kernel::PointBodyVerdict::Interior),
            (outside, kernel::PointBodyVerdict::Exterior),
            (removed_anchor, kernel::PointBodyVerdict::Exterior),
        ] {
            let classification = part
                .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(
                classification.verdict(),
                &expected,
                "body {body:?} at {point:?}"
            );
        }
    }

    let mut exports = Vec::new();
    for body in &bodies {
        let first = part
            .export_xt(ExportXtRequest::new(body.clone()))
            .unwrap()
            .into_result()
            .unwrap();
        let second = part
            .export_xt(ExportXtRequest::new(body.clone()))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(first.bytes(), second.bytes());
        exports.push(first.bytes().to_vec());
    }
    assert_ne!(exports[0], exports[1]);
    drop(part);

    let imported_part = fixture.session.create_part();
    for bytes in &exports {
        let imported = fixture
            .session
            .edit_part(imported_part.clone())
            .unwrap()
            .import_xt(ImportXtRequest::new(bytes))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(imported.bodies().len(), 1);
        let check = fixture
            .session
            .part(imported_part.clone())
            .unwrap()
            .check_body(CheckBodyRequest::new(
                imported.bodies()[0].clone(),
                CheckLevel::Fast,
            ))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            check.outcome(),
            CheckOutcome::Valid,
            "imported band: {check:#?}"
        );
    }
    assert_eq!(
        fixture.session.part(imported_part).unwrap().bodies().len(),
        2
    );
}

#[test]
fn public_curved_boolean_one_ring_connected_union_commits_a_full_valid_boss() {
    for swapped in [false, true] {
        let mut fixture = one_ring_cylindrical_boss_fixture();
        let block = fixture.left.clone();
        let cylinder = fixture.right.clone();
        if swapped {
            core::mem::swap(&mut fixture.left, &mut fixture.right);
        }
        let result = boolean_success(run_boolean(
            &mut fixture,
            BooleanOperation::Unite,
            OperationSettings::new(),
        ));
        let BooleanResult::Created(created) = result else {
            panic!("one-ring connected union must create a cylindrical boss")
        };
        assert_eq!(created.bodies().len(), 1);
        assert_boolean_created_full_valid(&created);
        assert_eq!(created.journal().part(), fixture.part);
        assert!(created.journal().mutation_count() > 0);
        assert_eq!(created.journal().lineage_count(), 8);
        assert_boolean_sources_retained(&fixture, 3);
        let body = created.bodies()[0].clone();
        assert_eq!(
            boolean_body_topology_signature(&fixture, body.clone()),
            [8, 14, 8]
        );
        assert_eq!(
            boolean_topology_counts(&fixture),
            [3, 6, 3, 17, 20, 56, 28, 16]
        );

        let part = fixture.session.part(fixture.part.clone()).unwrap();
        let result_faces = part
            .body(body.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>();
        let block_faces = part
            .body(block)
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>();
        let cylinder_faces = part
            .body(cylinder)
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>();
        let mut derived_faces = Vec::new();
        let mut block_sources = 0;
        let mut cylinder_sources = 0;
        for event in created.journal().lineage() {
            let LineageView::DerivedFrom {
                derived: JournalEntity::Face(derived),
                source: JournalEntity::Face(source),
            } = event
            else {
                panic!("cylindrical boss lineage must be face-only DerivedFrom")
            };
            assert!(result_faces.contains(&derived));
            assert!(!derived_faces.contains(&derived));
            derived_faces.push(derived);
            if block_faces.contains(&source) {
                block_sources += 1;
            } else if cylinder_faces.contains(&source) {
                cylinder_sources += 1;
            } else {
                panic!("cylindrical boss lineage escaped both source bodies")
            }
        }
        assert_eq!(derived_faces.len(), result_faces.len());
        assert_eq!(block_sources, 6);
        assert_eq!(cylinder_sources, 2);

        let mut surface_classes = Vec::new();
        let mut loop_counts = Vec::new();
        for face in &result_faces {
            let face = part.face(face.clone()).unwrap();
            surface_classes.push(part.surface(face.surface()).unwrap().class_key().as_str());
            loop_counts.push(face.loops().len());
        }
        loop_counts.sort_unstable();
        assert_eq!(loop_counts, vec![1, 1, 1, 1, 1, 1, 2, 2]);
        assert_eq!(
            surface_classes
                .iter()
                .filter(|class| **class == "kernel.surface.plane.v1")
                .count(),
            7
        );
        assert_eq!(
            surface_classes
                .iter()
                .filter(|class| **class == "kernel.surface.cylinder.v1")
                .count(),
            1
        );

        let mut bounded_edges = 0;
        let mut circle_edges = 0;
        for edge in part.body(body.clone()).unwrap().edges().unwrap() {
            let edge = part.edge(edge).unwrap();
            assert_eq!(edge.fins().len(), 2);
            let class = part
                .curve(edge.curve().expect("boss edges retain analytic curves"))
                .unwrap()
                .class_key()
                .as_str();
            match class {
                "kernel.curve.intersection.v1" => {
                    bounded_edges += 1;
                    assert!(edge.vertices().iter().all(Option::is_some));
                    assert!(edge.bounds().is_some());
                }
                "kernel.curve.circle.v1" => {
                    circle_edges += 1;
                    assert_eq!(edge.vertices(), [None, None]);
                    assert!(edge.bounds().is_none());
                }
                class => panic!("unexpected cylindrical boss edge class: {class}"),
            }
        }
        assert_eq!((bounded_edges, circle_edges), (12, 2));

        for (point, expected) in [
            (
                Point3::new(1.5, 0.0, 0.0),
                kernel::PointBodyVerdict::Interior,
            ),
            (
                Point3::new(0.0, 0.0, 0.5),
                kernel::PointBodyVerdict::Interior,
            ),
            (
                Point3::new(0.0, 0.0, 1.5),
                kernel::PointBodyVerdict::Interior,
            ),
            (
                Point3::new(0.0, 0.0, -1.25),
                kernel::PointBodyVerdict::Exterior,
            ),
            (
                Point3::new(2.25, 0.0, 0.0),
                kernel::PointBodyVerdict::Exterior,
            ),
            (
                Point3::new(0.0, 0.0, 2.25),
                kernel::PointBodyVerdict::Exterior,
            ),
        ] {
            let classification = part
                .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(classification.verdict(), &expected, "point {point:?}");
            assert!(classification.witness().is_some());
        }
        drop(part);
        let first = assert_deterministic_xt_and_fast_self_import(&mut fixture, &[body]);

        let mut replay = one_ring_cylindrical_boss_fixture();
        if swapped {
            core::mem::swap(&mut replay.left, &mut replay.right);
        }
        let replay_result = boolean_success(run_boolean(
            &mut replay,
            BooleanOperation::Unite,
            OperationSettings::new(),
        ));
        let BooleanResult::Created(replayed) = replay_result else {
            panic!("fresh replay must create a cylindrical boss")
        };
        let second = assert_deterministic_xt_and_fast_self_import(
            &mut replay,
            &[replayed.bodies()[0].clone()],
        );
        assert_eq!(first, second);
    }
}

#[test]
fn public_curved_boolean_one_ring_subtraction_commits_a_full_valid_blind_pocket() {
    let mut fixture = one_ring_blind_pocket_fixture();
    let block = fixture.left.clone();
    let cylinder = fixture.right.clone();
    let result = boolean_success(run_boolean(
        &mut fixture,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("one-ring block-minus-cylinder subtraction must create a blind pocket")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_boolean_created_full_valid(&created);
    assert_eq!(created.journal().part(), fixture.part);
    assert!(created.journal().mutation_count() > 0);
    assert_eq!(created.journal().lineage_count(), 8);
    assert_boolean_sources_retained(&fixture, 3);
    let body = created.bodies()[0].clone();
    assert_eq!(
        boolean_body_topology_signature(&fixture, body.clone()),
        [8, 14, 8]
    );
    assert_eq!(
        boolean_topology_counts(&fixture),
        [3, 6, 3, 17, 20, 56, 28, 16]
    );

    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let result_faces = part
        .body(body.clone())
        .unwrap()
        .faces()
        .unwrap()
        .collect::<Vec<_>>();
    let block_faces = part
        .body(block)
        .unwrap()
        .faces()
        .unwrap()
        .collect::<Vec<_>>();
    let cylinder_faces = part
        .body(cylinder)
        .unwrap()
        .faces()
        .unwrap()
        .collect::<Vec<_>>();
    let mut derived_faces = Vec::new();
    let mut block_sources = 0;
    let mut cylinder_sources = 0;
    for event in created.journal().lineage() {
        let LineageView::DerivedFrom {
            derived: JournalEntity::Face(derived),
            source: JournalEntity::Face(source),
        } = event
        else {
            panic!("blind-pocket lineage must be face-only DerivedFrom")
        };
        assert!(result_faces.contains(&derived));
        assert!(!derived_faces.contains(&derived));
        derived_faces.push(derived);
        if block_faces.contains(&source) {
            block_sources += 1;
        } else if cylinder_faces.contains(&source) {
            cylinder_sources += 1;
        } else {
            panic!("blind-pocket lineage escaped both source bodies")
        }
    }
    assert_eq!(derived_faces.len(), result_faces.len());
    assert_eq!(block_sources, 6);
    assert_eq!(cylinder_sources, 2);

    let mut surface_classes = Vec::new();
    let mut loop_counts = Vec::new();
    for face in &result_faces {
        let face = part.face(face.clone()).unwrap();
        surface_classes.push(part.surface(face.surface()).unwrap().class_key().as_str());
        loop_counts.push(face.loops().len());
    }
    loop_counts.sort_unstable();
    assert_eq!(loop_counts, vec![1, 1, 1, 1, 1, 1, 2, 2]);
    assert_eq!(
        surface_classes
            .iter()
            .filter(|class| **class == "kernel.surface.plane.v1")
            .count(),
        7
    );
    assert_eq!(
        surface_classes
            .iter()
            .filter(|class| **class == "kernel.surface.cylinder.v1")
            .count(),
        1
    );

    let mut bounded_edges = 0;
    let mut circle_edges = 0;
    for edge in part.body(body.clone()).unwrap().edges().unwrap() {
        let edge = part.edge(edge).unwrap();
        assert_eq!(edge.fins().len(), 2);
        let class = part
            .curve(
                edge.curve()
                    .expect("blind-pocket edges retain analytic curves"),
            )
            .unwrap()
            .class_key()
            .as_str();
        match class {
            "kernel.curve.intersection.v1" => {
                bounded_edges += 1;
                assert!(edge.vertices().iter().all(Option::is_some));
                assert!(edge.bounds().is_some());
            }
            "kernel.curve.circle.v1" => {
                circle_edges += 1;
                assert_eq!(edge.vertices(), [None, None]);
                assert!(edge.bounds().is_none());
            }
            class => panic!("unexpected blind-pocket edge class: {class}"),
        }
    }
    assert_eq!((bounded_edges, circle_edges), (12, 2));

    for (point, expected) in [
        (
            Point3::new(1.5, 0.0, 0.0),
            kernel::PointBodyVerdict::Interior,
        ),
        (
            Point3::new(0.0, 0.0, -0.5),
            kernel::PointBodyVerdict::Interior,
        ),
        (
            Point3::new(0.0, 0.0, 0.5),
            kernel::PointBodyVerdict::Exterior,
        ),
        (
            Point3::new(0.0, 0.0, 1.5),
            kernel::PointBodyVerdict::Exterior,
        ),
        (
            Point3::new(2.25, 0.0, 0.0),
            kernel::PointBodyVerdict::Exterior,
        ),
    ] {
        let classification = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(classification.verdict(), &expected, "point {point:?}");
        assert!(classification.witness().is_some());
    }
    for point in [Point3::new(0.0, 0.0, 0.0), Point3::new(0.75, 0.0, 0.5)] {
        let classification = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert!(
            matches!(
                classification.verdict(),
                kernel::PointBodyVerdict::Boundary { .. }
            ),
            "point {point:?}: {classification:?}"
        );
        assert!(classification.witness().is_none());
    }
    drop(part);
    let first = assert_deterministic_xt_and_fast_self_import(&mut fixture, &[body]);

    let mut replay = one_ring_blind_pocket_fixture();
    let replay_result = boolean_success(run_boolean(
        &mut replay,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(replayed) = replay_result else {
        panic!("fresh replay must create a blind pocket")
    };
    let second =
        assert_deterministic_xt_and_fast_self_import(&mut replay, &[replayed.bodies()[0].clone()]);
    assert_eq!(first, second);

    let mut reverse_order = one_ring_blind_pocket_fixture();
    core::mem::swap(&mut reverse_order.left, &mut reverse_order.right);
    let reverse_result = boolean_success(run_boolean(
        &mut reverse_order,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(reverse_created) = reverse_result else {
        panic!("cylinder-minus-block order must retain its positive upper band")
    };
    assert_eq!(reverse_created.bodies().len(), 1);
    assert_boolean_created_full_valid(&reverse_created);
    assert_eq!(
        boolean_body_topology_signature(&reverse_order, reverse_created.bodies()[0].clone()),
        [3, 2, 0]
    );
    assert_boolean_sources_retained(&reverse_order, 3);
}

#[test]
fn public_boolean_represents_multiple_and_empty_results_without_special_cases() {
    let mut union_fixture = disjoint_boolean_fixture();
    let union = boolean_success(run_boolean(
        &mut union_fixture,
        BooleanOperation::Unite,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = union else {
        panic!("disjoint union must create two bodies")
    };
    assert_eq!(created.bodies().len(), 2);
    assert_eq!(created.reports().len(), 2);
    assert!(boolean_body_x_center(&union_fixture, created.bodies()[0].clone()) < 0.0);
    assert!(boolean_body_x_center(&union_fixture, created.bodies()[1].clone()) > 0.0);
    assert_boolean_sources_retained(&union_fixture, 4);

    let mut intersection_fixture = disjoint_boolean_fixture();
    let intersection = boolean_success(run_boolean(
        &mut intersection_fixture,
        BooleanOperation::Intersect,
        OperationSettings::new(),
    ));
    assert!(matches!(intersection, BooleanResult::ProvenEmpty));
    assert!(intersection.is_empty());
    assert!(intersection.bodies().is_empty());
    assert!(intersection.created().is_none());
    assert_boolean_sources_retained(&intersection_fixture, 2);
}

#[test]
fn public_boolean_contained_subtraction_commits_one_two_shell_solid() {
    let mut fixture = contained_boolean_fixture();
    let result = boolean_success(run_boolean(
        &mut fixture,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("contained subtraction must create a cavity body")
    };
    assert_eq!(created.bodies().len(), 1);
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let body = part.body(created.bodies()[0].clone()).unwrap();
    assert_eq!(body.regions().len(), 3);
    let solid = body
        .regions()
        .find(|region| part.region(region.clone()).unwrap().kind() == RegionKind::Solid)
        .unwrap();
    assert_eq!(part.region(solid).unwrap().shells().len(), 2);
    drop(part);
    assert_boolean_sources_retained(&fixture, 3);
}

#[test]
fn public_boolean_exact_contact_refuses_without_persisting_candidate_topology() {
    let mut fixture = boolean_fixture(
        Frame::world(),
        [2.0, 2.0, 2.0],
        Frame::world().with_origin(Point3::new(2.0, 0.0, 0.0)),
        [2.0, 2.0, 2.0],
    );
    let before = boolean_topology_counts(&fixture);
    let outcome = run_boolean(
        &mut fixture,
        BooleanOperation::Intersect,
        OperationSettings::new(),
    );
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Refused(BooleanRefusal::BoundaryContact)
    ));
    assert_eq!(boolean_topology_counts(&fixture), before);
    assert_boolean_sources_retained(&fixture, 2);
}

#[test]
fn public_boolean_rejects_wrong_part_and_stale_operands_before_invalid_settings() {
    let precision = SessionPrecision::try_new(1.0e-6, 1.0e-11, 500.0).unwrap();
    let policy = SessionPolicy::new(
        precision,
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        BudgetPlan::empty(),
        PolicyVersion::V1,
    );
    let valid_settings =
        OperationSettings::new().with_tolerances(Tolerances::with_linear(1.0e-6).unwrap());
    let mut session = Kernel::with_default_policy(policy).create_session();
    let local_part = session.create_part();
    let (left, right) = {
        let mut edit = session.edit_part(local_part.clone()).unwrap();
        let left = edit
            .create_block(
                BlockRequest::new(Frame::world(), [2.0, 2.0, 2.0])
                    .with_settings(valid_settings.clone()),
            )
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let right = edit
            .create_block(
                BlockRequest::new(
                    Frame::world().with_origin(Point3::new(0.5, 0.25, 0.125)),
                    [2.0, 2.0, 2.0],
                )
                .with_settings(valid_settings.clone()),
            )
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (left, right)
    };
    let foreign_part = session.create_part();
    let foreign = session
        .edit_part(foreign_part)
        .unwrap()
        .create_block(
            BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0])
                .with_settings(valid_settings.clone()),
        )
        .unwrap()
        .into_result()
        .unwrap()
        .body();

    let wrong_part = session
        .edit_part(local_part.clone())
        .unwrap()
        .boolean_bodies(BooleanBodiesRequest::new(
            BooleanOperation::Intersect,
            foreign,
            right.clone(),
        ));
    assert!(matches!(wrong_part, Err(Error::WrongPart { .. })));

    let (surface, sense, position) = {
        let part = session.part(local_part.clone()).unwrap();
        let face = part
            .face(
                part.body(left.clone())
                    .unwrap()
                    .faces()
                    .unwrap()
                    .next()
                    .unwrap(),
            )
            .unwrap();
        let loop_id = face.loops().next().unwrap();
        let fin = part
            .fin(part.loop_(loop_id).unwrap().fins().next().unwrap())
            .unwrap();
        let vertex = fin.tail().unwrap().unwrap();
        (
            face.surface(),
            face.sense(),
            part.vertex(vertex).unwrap().position().unwrap(),
        )
    };
    let stale = {
        let mut edit = session.edit_part(local_part.clone()).unwrap();
        let mut transaction = edit.begin_edit(valid_settings.clone()).unwrap();
        let seed = transaction
            .create_seed_body(CreateSeedBodyRequest::new(surface, sense, position))
            .unwrap();
        let stale = seed.body();
        transaction
            .remove_seed_body(RemoveSeedBodyRequest::new(stale.clone()))
            .unwrap();
        transaction
            .commit(core::slice::from_ref(&left))
            .unwrap()
            .into_result()
            .unwrap();
        stale
    };
    let stale_operand = session
        .edit_part(local_part.clone())
        .unwrap()
        .boolean_bodies(BooleanBodiesRequest::new(
            BooleanOperation::Intersect,
            stale,
            right.clone(),
        ));
    assert!(matches!(
        stale_operand,
        Err(Error::StaleEntity {
            kind: EntityKind::Body
        })
    ));

    let invalid_settings =
        session
            .edit_part(local_part)
            .unwrap()
            .boolean_bodies(BooleanBodiesRequest::new(
                BooleanOperation::Intersect,
                left,
                right,
            ));
    assert!(matches!(invalid_settings, Err(Error::Core { .. })));
}

#[test]
fn public_boolean_bsp_budget_accepts_n_and_reports_exact_n_minus_one_crossing() {
    let baseline = run_boolean(
        &mut overlapping_boolean_fixture(),
        BooleanOperation::Intersect,
        OperationSettings::new(),
    );
    let usage = *baseline
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == BOOLEAN_BSP_WORK && usage.resource == ResourceKind::Work)
        .unwrap();
    assert!(usage.consumed > 0);

    let settings_at = |allowed| {
        OperationSettings::new().with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                BOOLEAN_BSP_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap(),
        )
    };
    let admitted = run_boolean(
        &mut overlapping_boolean_fixture(),
        BooleanOperation::Intersect,
        settings_at(usage.consumed),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let denied = run_boolean(
        &mut overlapping_boolean_fixture(),
        BooleanOperation::Intersect,
        settings_at(usage.consumed - 1),
    );
    let expected = kernel::LimitSnapshot {
        allowed: usage.consumed - 1,
        ..usage
    };
    assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
    assert_eq!(denied.report().limit_events(), &[expected]);
}

#[test]
fn public_curved_boolean_bsp_budget_is_exact_and_denial_is_failure_atomic() {
    let fixture = || {
        let base = Point3::new(3.0, -2.0, 1.25);
        let cylinder_frame = boolean_frame(base.to_array(), [0.0, 0.6, 0.8], [1.0, 0.0, 0.0]);
        block_cylinder_boolean_fixture(
            cylinder_frame.with_origin(base + cylinder_frame.z()),
            [4.0, 4.0, 1.0],
            cylinder_frame,
            0.75,
            2.0,
        )
    };
    let baseline = run_boolean(
        &mut fixture(),
        BooleanOperation::Intersect,
        OperationSettings::new(),
    );
    let usage = *baseline
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == BOOLEAN_BSP_WORK && usage.resource == ResourceKind::Work)
        .unwrap();
    assert!(usage.consumed > 0);

    let settings_at = |allowed| {
        OperationSettings::new().with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                BOOLEAN_BSP_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap(),
        )
    };
    let admitted = run_boolean(
        &mut fixture(),
        BooleanOperation::Intersect,
        settings_at(usage.consumed),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied_fixture = fixture();
    let before = boolean_topology_counts(&denied_fixture);
    let denied = run_boolean(
        &mut denied_fixture,
        BooleanOperation::Intersect,
        settings_at(usage.consumed - 1),
    );
    let expected = kernel::LimitSnapshot {
        allowed: usage.consumed - 1,
        ..usage
    };
    assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(boolean_topology_counts(&denied_fixture), before);
    assert_boolean_sources_retained(&denied_fixture, 2);
}

#[test]
fn public_curved_whole_source_copy_budget_is_exact_and_denial_is_failure_atomic() {
    let fixture = || {
        block_cylinder_boolean_fixture(
            Frame::world().with_origin(Point3::new(8.0, 0.0, 0.0)),
            [2.0, 2.0, 2.0],
            Frame::world().with_origin(Point3::new(-8.0, 0.0, -1.0)),
            0.75,
            2.0,
        )
    };
    let baseline = run_boolean(
        &mut fixture(),
        BooleanOperation::Unite,
        OperationSettings::new(),
    );
    assert!(matches!(
        baseline.result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));
    let usage = *baseline
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == BOOLEAN_POST_SELECTION_WORK && usage.resource == ResourceKind::Work
        })
        .unwrap();
    assert!(usage.consumed > 0);

    let settings_at = |allowed| {
        OperationSettings::new().with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                BOOLEAN_POST_SELECTION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap(),
        )
    };
    let admitted = run_boolean(
        &mut fixture(),
        BooleanOperation::Unite,
        settings_at(usage.consumed),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied_fixture = fixture();
    let before = boolean_topology_counts(&denied_fixture);
    let denied = run_boolean(
        &mut denied_fixture,
        BooleanOperation::Unite,
        settings_at(usage.consumed - 1),
    );
    let expected = kernel::LimitSnapshot {
        allowed: usage.consumed - 1,
        ..usage
    };
    assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(boolean_topology_counts(&denied_fixture), before);
    assert_boolean_sources_retained(&denied_fixture, 2);
}
