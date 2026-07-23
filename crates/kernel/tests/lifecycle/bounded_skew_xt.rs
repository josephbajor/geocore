//! Facade-only X_T lifecycle evidence for bounded-skew four-face lobes.
//! Wall-time budget: less than 60 seconds for two rigid placements and replay.

use super::*;
use kernel::{ImportXtResult, OperationOutcome, StageId};

const BOUNDED_LOWER: f64 = 1.8;
const BOUNDED_UPPER: f64 = 1.9;
const TRANSVERSE_HALF_HEIGHT: f64 = 1.25;
const TRANSVERSE_RADIUS: f64 = 2.0;

struct Fixture {
    session: Session,
    part: PartId,
    bounded: BodyId,
    transverse: BodyId,
}

fn placements() -> [Frame; 2] {
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

fn fixture(frame: Frame, preseed: bool) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (bounded, transverse) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        if preseed {
            edit.create_block(BlockRequest::new(
                Frame::world().with_origin(Point3::new(40.0, -30.0, 20.0)),
                [1.0, 2.0, 3.0],
            ))
            .unwrap()
            .into_result()
            .unwrap();
        }
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

fn subtract_lobes(fixture: &mut Fixture) -> Vec<BodyId> {
    let result = fixture
        .session
        .edit_part(fixture.part.clone())
        .unwrap()
        .boolean_bodies(BooleanBodiesRequest::new(
            BooleanOperation::Subtract,
            fixture.bounded.clone(),
            fixture.transverse.clone(),
        ))
        .unwrap()
        .into_result()
        .unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("bounded-skew subtraction did not create its two lobes: {result:#?}")
    };
    assert_eq!(created.bodies().len(), 2);
    assert!(created.reports().iter().all(|report| {
        report.report().outcome() == CheckOutcome::Valid && report.report().gaps().is_empty()
    }));
    created.bodies().to_vec()
}

fn export_lobes(fixture: &Fixture, bodies: &[BodyId]) -> Vec<Vec<u8>> {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    bodies
        .iter()
        .cloned()
        .map(|body| {
            let first = part
                .export_xt(ExportXtRequest::new(body.clone()))
                .unwrap()
                .into_result()
                .unwrap()
                .bytes()
                .to_vec();
            let second = part
                .export_xt(ExportXtRequest::new(body))
                .unwrap()
                .into_result()
                .unwrap()
                .bytes()
                .to_vec();
            assert_eq!(first, second, "repeated bounded-skew X_T export changed");
            first
        })
        .collect()
}

fn assert_imported_lobe(session: &Session, part_id: PartId, body_id: BodyId) {
    let part = session.part(part_id).unwrap();
    let body = part.body(body_id.clone()).unwrap();
    let faces = body.faces().unwrap().collect::<Vec<_>>();
    let edges = body.edges().unwrap().collect::<Vec<_>>();
    let vertices = body.vertices().unwrap().collect::<Vec<_>>();
    assert_eq!((faces.len(), edges.len(), vertices.len()), (4, 6, 4));

    let mut cylinders = 0;
    let mut planes = 0;
    for face in faces {
        match part
            .surface(part.face(face).unwrap().surface())
            .unwrap()
            .class_key()
            .as_str()
        {
            "kernel.surface.cylinder.v1" => cylinders += 1,
            "kernel.surface.plane.v1" => planes += 1,
            class => panic!("unexpected imported bounded-skew surface class: {class}"),
        }
    }
    assert_eq!((cylinders, planes), (2, 2));

    let mut tolerant = 0;
    let mut exact_lines = 0;
    let mut exact_circles = 0;
    for edge_id in edges {
        let edge = part.edge(edge_id).unwrap();
        assert!(edge.vertices().iter().all(Option::is_some));
        assert_eq!(edge.fins().len(), 2);
        if let Some(curve) = edge.curve() {
            match part.curve(curve).unwrap().class_key().as_str() {
                "kernel.curve.line.v1" => exact_lines += 1,
                "kernel.curve.circle.v1" => exact_circles += 1,
                class => panic!("unexpected imported bounded-skew edge curve class: {class}"),
            }
            continue;
        }
        tolerant += 1;
        assert!(edge.tolerance().is_some());
        for fin in edge.fins() {
            let pcurve = part
                .fin(fin)
                .unwrap()
                .pcurve()
                .expect("curve-less imported edge retains a pcurve on each fin");
            assert_eq!(
                part.pcurve(pcurve).unwrap().class_key().as_str(),
                "kernel.curve2d.nurbs.v1"
            );
        }
    }
    assert_eq!((tolerant, exact_lines, exact_circles), (2, 2, 2));
}

fn import_and_reexport(
    session: &mut Session,
    bytes: &[u8],
) -> (OperationOutcome<ImportXtResult>, Vec<u8>) {
    let part = session.create_part();
    let imported = session
        .edit_part(part.clone())
        .unwrap()
        .import_xt(ImportXtRequest::new(bytes))
        .unwrap();
    let body = imported.result().unwrap().bodies()[0].clone();
    assert_eq!(imported.result().unwrap().bodies().len(), 1);
    assert_imported_lobe(session, part.clone(), body.clone());
    let fast = session
        .part(part.clone())
        .unwrap()
        .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Fast))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(fast.outcome(), CheckOutcome::Valid);
    assert!(fast.faults().is_empty());
    let full = session
        .part(part.clone())
        .unwrap()
        .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Full))
        .unwrap()
        .into_result()
        .unwrap();
    assert_ne!(full.outcome(), CheckOutcome::Invalid);
    assert!(full.faults().is_empty());
    let replay = session
        .part(part.clone())
        .unwrap()
        .export_xt(ExportXtRequest::new(body.clone()))
        .unwrap()
        .into_result()
        .unwrap()
        .bytes()
        .to_vec();
    let repeated = session
        .part(part)
        .unwrap()
        .export_xt(ExportXtRequest::new(body))
        .unwrap()
        .into_result()
        .unwrap()
        .bytes()
        .to_vec();
    assert_eq!(
        replay, repeated,
        "repeated export of one imported lobe changed canonical bytes"
    );
    (imported, replay)
}

fn part_shape(session: &Session, part: PartId) -> [usize; 8] {
    let part = session.part(part).unwrap();
    [
        part.bodies().len(),
        part.faces().len(),
        part.loops().len(),
        part.fins().len(),
        part.edges().len(),
        part.vertices().len(),
        part.curves().len(),
        part.pcurves().len(),
    ]
}

#[test]
fn bounded_skew_lobes_have_stable_xt_fast_self_import_twice_and_rigid_replay() {
    let mut world_payload = None;
    for frame in placements() {
        let mut baseline = fixture(frame, false);
        let bodies = subtract_lobes(&mut baseline);
        let payloads = export_lobes(&baseline, &bodies);
        for payload in &payloads {
            let (_, first_replay) = import_and_reexport(&mut baseline.session, payload);
            let (_, second_replay) = import_and_reexport(&mut baseline.session, payload);
            assert_eq!(
                first_replay, second_replay,
                "independent imports did not canonicalize to identical X_T bytes"
            );
        }

        let mut shifted_ids = fixture(frame, true);
        let replay_bodies = subtract_lobes(&mut shifted_ids);
        assert_eq!(
            export_lobes(&shifted_ids, &replay_bodies),
            payloads,
            "unrelated prior allocations changed bounded-skew X_T bytes"
        );
        world_payload.get_or_insert_with(|| payloads[0].clone());
    }

    let bytes = world_payload.expect("world placement emits one payload");
    let node_visits = StageId::new("kgraph.eval.node-visits").unwrap();
    let mut baseline = Kernel::new().create_session();
    let baseline_part = baseline.create_part();
    let baseline_import = baseline
        .edit_part(baseline_part)
        .unwrap()
        .import_xt(ImportXtRequest::new(&bytes))
        .unwrap();
    let exact = baseline_import
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == node_visits && usage.resource == ResourceKind::Work)
        .expect("X_T reconstruction reports graph visits")
        .consumed;
    assert!(exact > 1);
    let settings_at = |allowed| {
        OperationSettings::new().with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                node_visits,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap(),
        )
    };

    let mut admitted = Kernel::new().create_session();
    let admitted_part = admitted.create_part();
    let accepted = admitted
        .edit_part(admitted_part.clone())
        .unwrap()
        .import_xt(ImportXtRequest::new(&bytes).with_settings(settings_at(exact)))
        .unwrap();
    let accepted_body = accepted.into_result().unwrap().bodies()[0].clone();
    let accepted_replay = admitted
        .part(admitted_part)
        .unwrap()
        .export_xt(ExportXtRequest::new(accepted_body))
        .unwrap()
        .into_result()
        .unwrap()
        .bytes()
        .to_vec();

    let mut denied = Kernel::new().create_session();
    let denied_part = denied.create_part();
    let before = part_shape(&denied, denied_part.clone());
    let refused = denied
        .edit_part(denied_part.clone())
        .unwrap()
        .import_xt(ImportXtRequest::new(&bytes).with_settings(settings_at(exact - 1)))
        .unwrap();
    assert!(refused.result().is_err());
    let [event] = refused.report().limit_events() else {
        panic!("N-1 import must publish exactly one structured refusal")
    };
    assert_eq!(event.stage, node_visits);
    assert_eq!(event.resource, ResourceKind::Work);
    assert_eq!(event.consumed, exact);
    assert_eq!(event.allowed, exact - 1);
    assert_eq!(part_shape(&denied, denied_part.clone()), before);

    let retried = denied
        .edit_part(denied_part.clone())
        .unwrap()
        .import_xt(ImportXtRequest::new(&bytes).with_settings(settings_at(exact)))
        .unwrap()
        .into_result()
        .unwrap();
    let retried_body = retried.bodies()[0].clone();
    assert_imported_lobe(&denied, denied_part.clone(), retried_body.clone());
    let retried_replay = denied
        .part(denied_part)
        .unwrap()
        .export_xt(ExportXtRequest::new(retried_body))
        .unwrap()
        .into_result()
        .unwrap()
        .bytes()
        .to_vec();
    assert_eq!(
        retried_replay, accepted_replay,
        "N-1 refusal changed the canonical bytes of a later admitted import"
    );
}
