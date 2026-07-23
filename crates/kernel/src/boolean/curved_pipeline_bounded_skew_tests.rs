use kcore::operation::{AccountingMode, BudgetPlan, LimitSnapshot, LimitSpec, ResourceKind};
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};
use ktopo::check::{CheckLevel, CheckOutcome};
use ktopo::entity::{
    Body as RawBody, Edge as RawEdge, Face as RawFace, Fin as RawFin, Loop as RawLoop,
    Region as RawRegion, Shell as RawShell, Vertex as RawVertex,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};

use crate::{
    BOOLEAN_POST_SELECTION_WORK, BOOLEAN_REALIZED_VERTICES, BlockRequest, BodyId,
    BooleanBodiesRequest, BooleanOperation, BooleanOutcome, BooleanRefusal, BooleanResult,
    CheckBodyRequest, CopyBodyRequest, CylinderRequest, JournalEntity, Kernel, LineageView,
    MutationKind, OperationOutcome, OperationSettings, PartId, Session,
};

const BOUNDED_LOWER: f64 = 1.8;
const BOUNDED_UPPER: f64 = 1.9;
const TRANSVERSE_HALF_HEIGHT: f64 = 1.25;
const TRANSVERSE_RADIUS: f64 = 2.0;
// Complete-plan/blueprint 3_088 + component N=53 work 3_233 +
// two analytic inputs at N=31 work 1_457 each.
const BOUNDED_SKEW_POST_SELECTION_WORK: u64 = 9_235;
// Two disconnected four-vertex lobe inputs coexist in the batch transaction.
const BOUNDED_SKEW_REALIZED_VERTICES: u64 = 8;
// Each 31-entity lobe charges 31^2 + 16*31 = 1_457 theorem work.
const BOUNDED_SKEW_SHELL_PROOF_WORK: u64 = 2_914;
const ONE_BOUNDED_SKEW_LOBE_SHELL_PROOF_WORK: u64 = BOUNDED_SKEW_SHELL_PROOF_WORK / 2;
// Fast checked-copy graph validation over one exact 31-entity lobe closure.
const RIGID_COPY_GRAPH_NODE_VISITS: u64 = 156;

struct Fixture {
    session: Session,
    part: PartId,
    bounded: BodyId,
    transverse: BodyId,
}

fn placement_frames() -> [Frame; 2] {
    [
        Frame::world(),
        Frame::new(
            Point3::new(2.5, -1.75, 0.625),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap(),
    ]
}

fn rigid_copy_frames() -> [Frame; 2] {
    [
        Frame::world(),
        Frame::new(
            Point3::new(-3.25, 2.0, 1.125),
            Vec3::new(1.0, 2.0, 3.0).normalized().unwrap(),
            Vec3::new(2.0, -1.0, 0.0).normalized().unwrap(),
        )
        .unwrap(),
    ]
}

fn fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (bounded, transverse) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let bounded = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, BOUNDED_LOWER)),
                1.0,
                BOUNDED_UPPER - BOUNDED_LOWER,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let transverse_frame = Frame::new(
            frame.point_at(-TRANSVERSE_HALF_HEIGHT, 0.0, 0.0),
            frame.x(),
            frame.y(),
        )
        .unwrap();
        let transverse = edit
            .create_cylinder(CylinderRequest::new(
                transverse_frame,
                TRANSVERSE_RADIUS,
                2.0 * TRANSVERSE_HALF_HEIGHT,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (bounded, transverse)
    };
    Fixture {
        session,
        part,
        bounded,
        transverse,
    }
}

fn subtract(
    fixture: &mut Fixture,
    settings: OperationSettings,
) -> OperationOutcome<BooleanOutcome> {
    fixture
        .session
        .edit_part(fixture.part.clone())
        .unwrap()
        .boolean_bodies(
            BooleanBodiesRequest::new(
                BooleanOperation::Subtract,
                fixture.bounded.clone(),
                fixture.transverse.clone(),
            )
            .with_settings(settings),
        )
        .unwrap()
}

fn usage<T>(
    outcome: &OperationOutcome<T>,
    stage: kcore::operation::StageId,
    resource: ResourceKind,
) -> LimitSnapshot {
    outcome
        .report()
        .usage()
        .iter()
        .copied()
        .find(|snapshot| snapshot.stage == stage && snapshot.resource == resource)
        .expect("bounded-skew operation must report the requested accounting frontier")
}

fn produced_lobes(fixture: &mut Fixture) -> Vec<BodyId> {
    let outcome = subtract(fixture, OperationSettings::new());
    let BooleanOutcome::Success(BooleanResult::Created(created)) = outcome.into_result().unwrap()
    else {
        panic!("expected two committed bounded-skew lobes");
    };
    assert_eq!(created.bodies().len(), 2);
    assert!(
        created
            .reports()
            .iter()
            .all(|report| report.report().outcome() == CheckOutcome::Valid)
    );
    created.bodies().to_vec()
}

fn assert_full_valid_lobe(fixture: &Fixture, body: BodyId) {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let body_view = part.body(body.clone()).unwrap();
    assert_eq!(body_view.faces().unwrap().len(), 4);
    assert_eq!(body_view.edges().unwrap().len(), 6);
    assert_eq!(body_view.vertices().unwrap().len(), 4);
    let outcome = part
        .check_body(CheckBodyRequest::new(body, CheckLevel::Full))
        .unwrap();
    let shell_proof_stage =
        kcore::operation::StageId::new("ktopo.check.bounded-skew-lobe-shell-work").unwrap();
    assert_eq!(
        usage(&outcome, shell_proof_stage, ResourceKind::Work).consumed,
        ONE_BOUNDED_SKEW_LOBE_SHELL_PROOF_WORK
    );
    let checked = outcome.into_result().unwrap();
    assert_eq!(checked.outcome(), CheckOutcome::Valid, "{checked:#?}");
}

fn copy_lobe(
    fixture: &mut Fixture,
    source: BodyId,
    placement: Frame,
    allowed: u64,
) -> crate::BodyCreated {
    let graph_work_stage = kcore::operation::StageId::new("kgraph.eval.node-visits").unwrap();
    let outcome = fixture
        .session
        .edit_part(fixture.part.clone())
        .unwrap()
        .copy_body_rigid(
            CopyBodyRequest::new(source.clone(), placement).with_settings(settings_at(
                graph_work_stage,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )),
        )
        .unwrap();
    let snapshot = usage(&outcome, graph_work_stage, ResourceKind::Work);
    assert_eq!(snapshot.consumed, RIGID_COPY_GRAPH_NODE_VISITS);
    assert_eq!(snapshot.allowed, allowed);
    assert!(outcome.report().limit_events().is_empty());
    let created = outcome.into_result().unwrap();
    assert_ne!(created.body(), source);
    assert!(
        created
            .journal()
            .mutations()
            .all(|mutation| mutation.kind() == MutationKind::Created)
    );
    assert_eq!(
        created.journal().lineage_count(),
        created.journal().mutation_count()
    );
    assert!(created.journal().lineage().any(|lineage| matches!(
        lineage,
        LineageView::DerivedFrom {
            derived: JournalEntity::Body(derived),
            source: JournalEntity::Body(original),
        } if derived == created.body() && original == source
    )));
    created
}

fn settings_at(
    stage: kcore::operation::StageId,
    resource: ResourceKind,
    mode: AccountingMode,
    allowed: u64,
) -> OperationSettings {
    OperationSettings::new().with_budget_overrides(
        BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap(),
    )
}

fn store_signature(fixture: &Fixture) -> [usize; 12] {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let store = &part.state.store;
    [
        store.count::<RawBody>(),
        store.count::<RawRegion>(),
        store.count::<RawShell>(),
        store.count::<RawFace>(),
        store.count::<RawLoop>(),
        store.count::<RawFin>(),
        store.count::<RawEdge>(),
        store.count::<RawVertex>(),
        store.count::<Point3>(),
        store.count::<CurveGeom>(),
        store.count::<SurfaceGeom>(),
        store.count::<Curve2dGeom>(),
    ]
}

fn add_probe(fixture: &mut Fixture) -> ktopo::entity::BodyId {
    fixture
        .session
        .edit_part(fixture.part.clone())
        .unwrap()
        .create_block(BlockRequest::new(
            Frame::world().with_origin(Point3::new(20.0, -7.0, 3.0)),
            [1.5, 1.25, 0.75],
        ))
        .unwrap()
        .into_result()
        .unwrap()
        .body()
        .raw()
}

fn assert_two_valid_four_face_lobes(fixture: &Fixture, outcome: OperationOutcome<BooleanOutcome>) {
    let BooleanOutcome::Success(BooleanResult::Created(created)) = outcome.into_result().unwrap()
    else {
        panic!("expected two committed bounded-skew lobes");
    };

    assert_eq!(created.bodies().len(), 2);
    assert_eq!(created.reports().len(), 2);
    assert!(
        created
            .reports()
            .iter()
            .all(|report| report.report().outcome() == CheckOutcome::Valid)
    );
    let view = fixture.session.part(fixture.part.clone()).unwrap();
    for body in created.bodies() {
        let body = view.body(body.clone()).unwrap();
        assert_eq!(body.faces().unwrap().len(), 4);
        assert_eq!(body.edges().unwrap().len(), 6);
        assert_eq!(body.vertices().unwrap().len(), 4);
    }
    assert!(view.body(fixture.bounded.clone()).is_ok());
    assert!(view.body(fixture.transverse.clone()).is_ok());
}

fn map_vector(placement: Frame, vector: Vec3) -> Vec3 {
    placement.x() * vector.x + placement.y() * vector.y + placement.z() * vector.z
}

fn map_frame(placement: Frame, source: Frame) -> Frame {
    Frame::new(
        placement.point_at(source.origin().x, source.origin().y, source.origin().z),
        map_vector(placement, source.z()),
        map_vector(placement, source.x()),
    )
    .unwrap()
}

fn assert_transformed_lobe_geometry(
    fixture: &Fixture,
    source: &BodyId,
    copied: &BodyId,
    placement: Frame,
) {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let store = &part.state.store;
    let source_faces = store.faces_of_body(source.raw()).unwrap();
    let copied_faces = store.faces_of_body(copied.raw()).unwrap();
    assert_eq!(source_faces.len(), copied_faces.len());
    for (source_face, copied_face) in source_faces.into_iter().zip(copied_faces) {
        let source_surface = store.get(source_face).unwrap().surface();
        let copied_surface = store.get(copied_face).unwrap().surface();
        assert_ne!(source_surface, copied_surface);
        match (
            store.get(source_surface).unwrap(),
            store.get(copied_surface).unwrap(),
        ) {
            (SurfaceGeom::Plane(source), SurfaceGeom::Plane(copied)) => {
                assert_eq!(*copied.frame(), map_frame(placement, *source.frame()));
            }
            (SurfaceGeom::Cylinder(source), SurfaceGeom::Cylinder(copied)) => {
                assert_eq!(*copied.frame(), map_frame(placement, *source.frame()));
                assert_eq!(copied.radius(), source.radius());
            }
            pair => panic!("rigid lobe copy changed a support family: {pair:?}"),
        }
    }
    let source_vertices = store.vertices_of_body(source.raw()).unwrap();
    let copied_vertices = store.vertices_of_body(copied.raw()).unwrap();
    assert_eq!(source_vertices.len(), copied_vertices.len());
    for (source_vertex, copied_vertex) in source_vertices.into_iter().zip(copied_vertices) {
        let source_point = store.get(source_vertex).unwrap().point();
        let copied_point = store.get(copied_vertex).unwrap().point();
        assert_ne!(source_point, copied_point);
        let source_point = *store.get(source_point).unwrap();
        assert_eq!(
            *store.get(copied_point).unwrap(),
            placement.point_at(source_point.x, source_point.y, source_point.z)
        );
    }
    let source_edges = store.edges_of_body(source.raw()).unwrap();
    let copied_edges = store.edges_of_body(copied.raw()).unwrap();
    assert_eq!(source_edges.len(), copied_edges.len());
    for (source_edge, copied_edge) in source_edges.into_iter().zip(copied_edges) {
        let source_edge = store.get(source_edge).unwrap();
        let copied_edge = store.get(copied_edge).unwrap();
        assert_eq!(copied_edge.bounds(), source_edge.bounds());
        let source_curve = source_edge.curve().unwrap();
        let copied_curve = copied_edge.curve().unwrap();
        assert_ne!(source_curve, copied_curve);
        match (
            store.get(source_curve).unwrap(),
            store.get(copied_curve).unwrap(),
        ) {
            (
                CurveGeom::PersistentSkewCylinderOpenSpan(source),
                CurveGeom::PersistentSkewCylinderOpenSpan(copied),
            ) => {
                let source_certificate = source.certificate();
                let copied_certificate = copied.certificate();
                assert_eq!(
                    copied_certificate.endpoint_points(),
                    source_certificate
                        .endpoint_points()
                        .map(|point| { placement.point_at(point.x, point.y, point.z) })
                );
                for parameter in [0.0, 0.37, 1.0] {
                    let point = source_certificate.carrier().eval(parameter);
                    let expected = placement.point_at(point.x, point.y, point.z);
                    assert!(copied_certificate.carrier().eval(parameter).dist(expected) <= 1.0e-12);
                }
                for index in 0..2 {
                    assert_ne!(
                        source.source_surfaces()[index],
                        copied.source_surfaces()[index]
                    );
                    assert_ne!(source.pcurves()[index], copied.pcurves()[index]);
                    assert_eq!(
                        store
                            .get(copied.pcurves()[index])
                            .unwrap()
                            .as_persistent_skew_cylinder_open_span()
                            .copied(),
                        Some(copied_certificate.pcurves()[index])
                    );
                }
            }
            (CurveGeom::Circle(source), CurveGeom::Circle(copied)) => {
                assert_eq!(*copied.frame(), map_frame(placement, *source.frame()));
                assert_eq!(copied.radius(), source.radius());
            }
            (CurveGeom::Line(source), CurveGeom::Line(copied)) => {
                assert_eq!(
                    copied.origin(),
                    placement.point_at(source.origin().x, source.origin().y, source.origin().z)
                );
                assert_eq!(copied.dir(), map_vector(placement, source.dir()));
            }
            pair => panic!("rigid lobe copy changed an edge carrier family: {pair:?}"),
        }
    }
}

fn assert_deterministic_lobe_replay(fixture: &Fixture, first: &BodyId, second: &BodyId) {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let store = &part.state.store;
    let first_faces = store.faces_of_body(first.raw()).unwrap();
    let second_faces = store.faces_of_body(second.raw()).unwrap();
    assert_eq!(first_faces.len(), second_faces.len());
    for (first, second) in first_faces.into_iter().zip(second_faces) {
        let first = store.get(first).unwrap();
        let second = store.get(second).unwrap();
        assert_eq!(first.sense(), second.sense());
        assert_eq!(first.domain(), second.domain());
        assert_eq!(
            store.get(first.surface()).unwrap(),
            store.get(second.surface()).unwrap()
        );
    }
    let first_vertices = store.vertices_of_body(first.raw()).unwrap();
    let second_vertices = store.vertices_of_body(second.raw()).unwrap();
    assert_eq!(first_vertices.len(), second_vertices.len());
    for (first, second) in first_vertices.into_iter().zip(second_vertices) {
        assert_eq!(
            store.get(store.get(first).unwrap().point()).unwrap(),
            store.get(store.get(second).unwrap().point()).unwrap()
        );
    }
    let first_edges = store.edges_of_body(first.raw()).unwrap();
    let second_edges = store.edges_of_body(second.raw()).unwrap();
    assert_eq!(first_edges.len(), second_edges.len());
    for (first, second) in first_edges.into_iter().zip(second_edges) {
        let first = store.get(first).unwrap();
        let second = store.get(second).unwrap();
        assert_eq!(first.bounds(), second.bounds());
        let first = store.get(first.curve().unwrap()).unwrap();
        let second = store.get(second.curve().unwrap()).unwrap();
        match (first, second) {
            (
                CurveGeom::PersistentSkewCylinderOpenSpan(first),
                CurveGeom::PersistentSkewCylinderOpenSpan(second),
            ) => assert_eq!(first.certificate(), second.certificate()),
            _ => assert_eq!(first, second),
        }
    }
    assert_eq!(
        body_pcurve_values(store, first.raw()),
        body_pcurve_values(store, second.raw())
    );
}

fn body_pcurve_values(
    store: &ktopo::store::Store,
    body: ktopo::entity::BodyId,
) -> Vec<Curve2dGeom> {
    let mut values = Vec::new();
    for face in store.faces_of_body(body).unwrap() {
        for &loop_ in store.get(face).unwrap().loops() {
            for &fin in store.get(loop_).unwrap().fins() {
                if let Some(pcurve) = store.get(fin).unwrap().pcurve() {
                    values.push(store.get(pcurve.curve()).unwrap().clone());
                }
            }
        }
    }
    values
}

#[test]
fn public_bounded_skew_subtract_require_valid_commits_two_four_face_lobes() {
    for frame in placement_frames() {
        let mut fixture = fixture(frame);
        let outcome = subtract(&mut fixture, OperationSettings::new());
        assert_eq!(
            usage(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work).consumed,
            BOUNDED_SKEW_POST_SELECTION_WORK
        );
        assert_eq!(
            usage(&outcome, BOOLEAN_REALIZED_VERTICES, ResourceKind::Items).consumed,
            BOUNDED_SKEW_REALIZED_VERTICES
        );
        assert_two_valid_four_face_lobes(&fixture, outcome);
    }
}

#[test]
fn public_bounded_skew_lobes_rigid_copy_full_valid_and_replay_stable() {
    for (source_frame, copy_frame) in placement_frames().into_iter().zip(rigid_copy_frames()) {
        let mut fixture = fixture(source_frame);
        let lobes = produced_lobes(&mut fixture);
        for source in lobes {
            assert_full_valid_lobe(&fixture, source.clone());
            let first = copy_lobe(
                &mut fixture,
                source.clone(),
                copy_frame,
                RIGID_COPY_GRAPH_NODE_VISITS,
            );
            let second = copy_lobe(
                &mut fixture,
                source.clone(),
                copy_frame,
                RIGID_COPY_GRAPH_NODE_VISITS,
            );
            assert_full_valid_lobe(&fixture, source.clone());
            assert_full_valid_lobe(&fixture, first.body());
            assert_full_valid_lobe(&fixture, second.body());
            assert_transformed_lobe_geometry(&fixture, &source, &first.body(), copy_frame);
            assert_deterministic_lobe_replay(&fixture, &first.body(), &second.body());
        }
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        assert!(part.body(fixture.bounded.clone()).is_ok());
        assert!(part.body(fixture.transverse.clone()).is_ok());
    }
}

#[test]
fn bounded_skew_lobe_rigid_copy_n_minus_one_is_failure_atomic() {
    let mut attempted = fixture(Frame::world());
    let mut control = fixture(Frame::world());
    let attempted_source = produced_lobes(&mut attempted).remove(0);
    let control_source = produced_lobes(&mut control).remove(0);
    assert_eq!(attempted_source.raw(), control_source.raw());
    let before = store_signature(&attempted);
    let graph_work_stage = kcore::operation::StageId::new("kgraph.eval.node-visits").unwrap();
    let denied = attempted
        .session
        .edit_part(attempted.part.clone())
        .unwrap()
        .copy_body_rigid(
            CopyBodyRequest::new(attempted_source.clone(), Frame::world()).with_settings(
                settings_at(
                    graph_work_stage,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    RIGID_COPY_GRAPH_NODE_VISITS - 1,
                ),
            ),
        )
        .unwrap();
    let crossing = LimitSnapshot {
        stage: graph_work_stage,
        resource: ResourceKind::Work,
        consumed: RIGID_COPY_GRAPH_NODE_VISITS,
        allowed: RIGID_COPY_GRAPH_NODE_VISITS - 1,
    };
    assert_eq!(denied.report().limit_events(), &[crossing]);
    assert_eq!(denied.into_result().unwrap_err().limit(), Some(crossing));
    assert_eq!(store_signature(&attempted), before);
    assert_full_valid_lobe(&attempted, attempted_source);
    assert_eq!(add_probe(&mut attempted), add_probe(&mut control));
    assert_eq!(store_signature(&attempted), store_signature(&control));
}

#[test]
fn bounded_skew_public_budgets_accept_n_and_refuse_n_minus_one_atomically() {
    let shell_proof_stage =
        kcore::operation::StageId::new("ktopo.check.bounded-skew-lobe-shell-work").unwrap();
    for (stage, resource, mode, exact) in [
        (
            BOOLEAN_POST_SELECTION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            BOUNDED_SKEW_POST_SELECTION_WORK,
        ),
        (
            BOOLEAN_REALIZED_VERTICES,
            ResourceKind::Items,
            AccountingMode::HighWater,
            BOUNDED_SKEW_REALIZED_VERTICES,
        ),
        (
            shell_proof_stage,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            BOUNDED_SKEW_SHELL_PROOF_WORK,
        ),
    ] {
        let mut admitted = fixture(Frame::world());
        let exact_outcome = subtract(&mut admitted, settings_at(stage, resource, mode, exact));
        let exact_snapshot = usage(&exact_outcome, stage, resource);
        assert_eq!(exact_snapshot.consumed, exact);
        assert_eq!(exact_snapshot.allowed, exact);
        assert!(exact_outcome.report().limit_events().is_empty());
        assert_two_valid_four_face_lobes(&admitted, exact_outcome);

        let mut attempted = fixture(Frame::world());
        let mut control = fixture(Frame::world());
        let before = store_signature(&attempted);
        let denied = subtract(
            &mut attempted,
            settings_at(stage, resource, mode, exact - 1),
        );
        let crossing = LimitSnapshot {
            stage,
            resource,
            consumed: exact,
            allowed: exact - 1,
        };
        assert_eq!(denied.report().limit_events(), &[crossing]);
        assert_eq!(denied.into_result().unwrap_err().limit(), Some(crossing));
        assert_eq!(store_signature(&attempted), before);
        let view = attempted.session.part(attempted.part.clone()).unwrap();
        assert!(view.body(attempted.bounded.clone()).is_ok());
        assert!(view.body(attempted.transverse.clone()).is_ok());
        assert_eq!(add_probe(&mut attempted), add_probe(&mut control));
        assert_eq!(store_signature(&attempted), store_signature(&control));
    }
}

#[test]
fn bounded_skew_public_dispatch_is_proof_gated_across_operation_order_matrix() {
    for frame in placement_frames() {
        for swapped in [false, true] {
            for operation in [
                BooleanOperation::Unite,
                BooleanOperation::Intersect,
                BooleanOperation::Subtract,
            ] {
                let mut fixture = fixture(frame);
                let before = store_signature(&fixture);
                let (left, right) = if swapped {
                    (fixture.transverse.clone(), fixture.bounded.clone())
                } else {
                    (fixture.bounded.clone(), fixture.transverse.clone())
                };
                let outcome = fixture
                    .session
                    .edit_part(fixture.part.clone())
                    .unwrap()
                    .boolean_bodies(BooleanBodiesRequest::new(operation, left, right))
                    .unwrap();
                if !swapped && operation == BooleanOperation::Subtract {
                    assert_two_valid_four_face_lobes(&fixture, outcome);
                    continue;
                }
                let BooleanOutcome::Refused(BooleanRefusal::FullValidationRejected { reports }) =
                    outcome.into_result().unwrap()
                else {
                    panic!(
                        "unsupported bounded-skew meaning must fail closed: swapped={swapped} \
                         operation={operation:?}"
                    );
                };
                assert_eq!(reports.len(), 1);
                assert!(reports[0].report().faults().is_empty());
                assert!(!reports[0].report().gaps().is_empty());
                assert_eq!(store_signature(&fixture), before);
                let view = fixture.session.part(fixture.part.clone()).unwrap();
                assert!(view.body(fixture.bounded.clone()).is_ok());
                assert!(view.body(fixture.transverse.clone()).is_ok());
            }
        }
    }
}
