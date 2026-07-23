use kcore::operation::{AccountingMode, BudgetPlan, LimitSnapshot, LimitSpec, ResourceKind};
use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};
use ktopo::check::CheckOutcome;
use ktopo::entity::{
    Body as RawBody, Edge as RawEdge, Face as RawFace, Fin as RawFin, Loop as RawLoop,
    Region as RawRegion, Shell as RawShell, Vertex as RawVertex,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};

use crate::{
    BOOLEAN_POST_SELECTION_WORK, BOOLEAN_REALIZED_VERTICES, BlockRequest, BodyId,
    BooleanBodiesRequest, BooleanOperation, BooleanOutcome, BooleanRefusal, BooleanResult,
    CylinderRequest, Kernel, OperationOutcome, OperationSettings, PartId, Session,
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

fn usage(
    outcome: &OperationOutcome<BooleanOutcome>,
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
