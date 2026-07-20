//! Public lifecycle evidence for a convex-planar void inside a finite cylinder.
//!
//! Wall-time budget: less than 60 seconds as part of the `lifecycle` target.

use super::*;
use kernel::PointBodyVerdict;

const INVERSE_CAVITY_REALIZATION_WORK: u64 = 280;

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Rigid,
}

fn placement_frame(placement: Placement) -> Frame {
    match placement {
        Placement::World => Frame::world(),
        Placement::Rigid => boolean_frame([3.0, -2.0, 1.25], [0.0, 0.6, 0.8], [1.0, 0.0, 0.0]),
    }
}

fn inverse_cavity_fixture(placement: Placement) -> BooleanFixture {
    let block_frame = placement_frame(placement);
    let cylinder_frame = block_frame.with_origin(block_frame.point_at(0.0, 0.0, -2.0));
    let mut fixture =
        block_cylinder_boolean_fixture(block_frame, [1.0, 1.0, 1.0], cylinder_frame, 2.0, 4.0);
    core::mem::swap(&mut fixture.left, &mut fixture.right);
    fixture
}

fn classify_body(fixture: &BooleanFixture, body: BodyId, point: Point3) -> PointBodyVerdict {
    fixture
        .session
        .part(fixture.part.clone())
        .unwrap()
        .classify_point_in_body(ClassifyPointInBodyRequest::new(body, point))
        .unwrap()
        .into_result()
        .unwrap()
        .verdict()
        .clone()
}

fn assert_inverse_source_geometry(fixture: &BooleanFixture, placement: Placement) {
    let frame = placement_frame(placement);
    let cylinder = fixture.left.clone();
    let block = fixture.right.clone();
    for (body, local, expected) in [
        (
            cylinder.clone(),
            [0.0, 0.0, 0.0],
            PointBodyVerdict::Interior,
        ),
        (block.clone(), [0.0, 0.0, 0.0], PointBodyVerdict::Interior),
        (
            cylinder.clone(),
            [1.5, 0.0, 0.0],
            PointBodyVerdict::Interior,
        ),
        (block.clone(), [1.5, 0.0, 0.0], PointBodyVerdict::Exterior),
        (
            cylinder.clone(),
            [2.5, 0.0, 0.0],
            PointBodyVerdict::Exterior,
        ),
        (block.clone(), [2.5, 0.0, 0.0], PointBodyVerdict::Exterior),
        (
            cylinder.clone(),
            [0.5, 0.0, 0.0],
            PointBodyVerdict::Interior,
        ),
    ] {
        assert_eq!(
            classify_body(fixture, body, frame.point_at(local[0], local[1], local[2])),
            expected
        );
    }
    for (body, local) in [
        (cylinder.clone(), [2.0, 0.0, 0.0]),
        (cylinder.clone(), [0.0, 0.0, -2.0]),
        (cylinder, [0.0, 0.0, 2.0]),
        (block, [0.5, 0.0, 0.0]),
    ] {
        assert!(matches!(
            classify_body(fixture, body, frame.point_at(local[0], local[1], local[2])),
            PointBodyVerdict::Boundary { .. }
        ));
    }
}

fn run_inverse_cavity(fixture: &mut BooleanFixture) -> kernel::BooleanCreatedResult {
    let result = boolean_success(run_boolean(
        fixture,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("finite cylinder minus its contained convex body must create a cavity")
    };
    created
}

fn assert_inverse_topology_and_lineage(
    fixture: &BooleanFixture,
    created: &kernel::BooleanCreatedResult,
    cylinder: BodyId,
    block: BodyId,
    body: BodyId,
) {
    assert_eq!(
        boolean_body_topology_signature(fixture, body.clone()),
        [9, 14, 8]
    );
    assert_eq!(
        boolean_topology_counts(fixture),
        [3, 7, 4, 18, 20, 56, 28, 16]
    );
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let body_view = part.body(body).unwrap();
    let regions = body_view.regions().collect::<Vec<_>>();
    assert_eq!(regions.len(), 3);
    let solid = regions
        .iter()
        .find(|region| part.region((*region).clone()).unwrap().kind() == RegionKind::Solid)
        .unwrap();
    let mut shell_face_counts = part
        .region(solid.clone())
        .unwrap()
        .shells()
        .map(|shell| part.shell(shell).unwrap().faces().len())
        .collect::<Vec<_>>();
    shell_face_counts.sort_unstable();
    assert_eq!(shell_face_counts, vec![3, 6]);
    assert_eq!(
        regions
            .iter()
            .filter(|region| part.region((*region).clone()).unwrap().kind() == RegionKind::Void)
            .count(),
        2
    );

    let result_faces = body_view.faces().unwrap().collect::<Vec<_>>();
    let source_faces = [
        part.body(cylinder)
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>(),
        part.body(block)
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>(),
    ];
    let mut derived_faces = Vec::new();
    let mut source_counts = [0_usize; 2];
    for event in created.journal().lineage() {
        let LineageView::DerivedFrom {
            derived: JournalEntity::Face(derived),
            source: JournalEntity::Face(source),
        } = event
        else {
            panic!("inverse-cavity lineage must be face-only DerivedFrom")
        };
        assert!(result_faces.contains(&derived));
        assert!(!derived_faces.contains(&derived));
        derived_faces.push(derived.clone());
        let source_index = source_faces
            .iter()
            .position(|faces| faces.contains(&source))
            .expect("inverse-cavity lineage escaped both source bodies");
        source_counts[source_index] += 1;
        let derived_sense = part.face(derived).unwrap().sense();
        let source_sense = part.face(source).unwrap().sense();
        assert_eq!(derived_sense == source_sense, source_index == 0);
    }
    assert_eq!(derived_faces.len(), result_faces.len());
    assert_eq!(source_counts, [3, 6]);

    let mut surface_classes = Vec::new();
    let mut loop_counts = Vec::new();
    for face in &result_faces {
        let face = part.face(face.clone()).unwrap();
        surface_classes.push(part.surface(face.surface()).unwrap().class_key().as_str());
        loop_counts.push(face.loops().len());
    }
    loop_counts.sort_unstable();
    assert_eq!(loop_counts, vec![1, 1, 1, 1, 1, 1, 1, 1, 2]);
    assert_eq!(
        surface_classes
            .iter()
            .filter(|class| **class == "kernel.surface.plane.v1")
            .count(),
        8
    );
    assert_eq!(
        surface_classes
            .iter()
            .filter(|class| **class == "kernel.surface.cylinder.v1")
            .count(),
        1
    );
}

fn assert_inverse_edges_and_points(fixture: &BooleanFixture, placement: Placement, body: BodyId) {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let mut edge_classes = [0_usize; 2];
    for edge in part.body(body.clone()).unwrap().edges().unwrap() {
        let edge = part.edge(edge).unwrap();
        assert_eq!(edge.fins().len(), 2);
        match part
            .curve(edge.curve().unwrap())
            .unwrap()
            .class_key()
            .as_str()
        {
            "kernel.curve.intersection.v1" => {
                edge_classes[0] += 1;
                assert!(edge.vertices().iter().all(Option::is_some));
                assert!(edge.bounds().is_some());
            }
            "kernel.curve.circle.v1" => {
                edge_classes[1] += 1;
                assert_eq!(edge.vertices(), [None, None]);
                assert!(edge.bounds().is_none());
            }
            class => panic!("unexpected inverse-cavity edge class: {class}"),
        }
    }
    assert_eq!(edge_classes, [12, 2]);

    let frame = placement_frame(placement);
    for (local, expected) in [
        ([1.5, 0.0, 0.0], PointBodyVerdict::Interior),
        ([0.0, 0.0, 0.0], PointBodyVerdict::Exterior),
        ([2.5, 0.0, 0.0], PointBodyVerdict::Exterior),
    ] {
        assert_eq!(
            classify_body(
                fixture,
                body.clone(),
                frame.point_at(local[0], local[1], local[2])
            ),
            expected,
            "{placement:?} result point {local:?}"
        );
    }
    for local in [
        [2.0, 0.0, 0.0],
        [0.0, 0.0, -2.0],
        [0.0, 0.0, 2.0],
        [0.5, 0.0, 0.0],
    ] {
        assert!(matches!(
            classify_body(
                fixture,
                body.clone(),
                frame.point_at(local[0], local[1], local[2])
            ),
            PointBodyVerdict::Boundary { .. }
        ));
    }
}

#[test]
fn public_cylinder_minus_contained_block_commits_an_exact_inverse_cavity() {
    for placement in [Placement::World, Placement::Rigid] {
        let mut fixture = inverse_cavity_fixture(placement);
        assert_inverse_source_geometry(&fixture, placement);
        assert_eq!(
            boolean_topology_counts(&fixture),
            [2, 4, 2, 9, 10, 28, 14, 8]
        );
        let (cylinder, block) = (fixture.left.clone(), fixture.right.clone());
        let created = run_inverse_cavity(&mut fixture);
        assert_eq!(created.bodies().len(), 1);
        assert_boolean_created_full_valid(&created);
        assert_eq!(created.journal().part(), fixture.part);
        assert_eq!(created.journal().lineage_count(), 9);
        assert_boolean_sources_retained(&fixture, 3);
        let body = created.bodies()[0].clone();
        assert_inverse_topology_and_lineage(&fixture, &created, cylinder, block, body.clone());
        assert_inverse_edges_and_points(&fixture, placement, body.clone());

        let first = assert_deterministic_xt_and_fast_self_import(&mut fixture, &[body]);
        let mut replay = inverse_cavity_fixture(placement);
        let replayed = run_inverse_cavity(&mut replay);
        let second = assert_deterministic_xt_and_fast_self_import(
            &mut replay,
            &[replayed.bodies()[0].clone()],
        );
        assert_eq!(first, second);
    }
}

#[test]
fn public_inverse_cavity_work_is_exact_and_denial_is_failure_atomic() {
    let baseline = run_boolean(
        &mut inverse_cavity_fixture(Placement::World),
        BooleanOperation::Subtract,
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
    assert_eq!(usage.consumed, INVERSE_CAVITY_REALIZATION_WORK);

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
        &mut inverse_cavity_fixture(Placement::World),
        BooleanOperation::Subtract,
        settings_at(INVERSE_CAVITY_REALIZATION_WORK),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied_fixture = inverse_cavity_fixture(Placement::World);
    let before = boolean_topology_counts(&denied_fixture);
    let denied = run_boolean(
        &mut denied_fixture,
        BooleanOperation::Subtract,
        settings_at(INVERSE_CAVITY_REALIZATION_WORK - 1),
    );
    let expected = kernel::LimitSnapshot {
        allowed: INVERSE_CAVITY_REALIZATION_WORK - 1,
        ..usage
    };
    assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(boolean_topology_counts(&denied_fixture), before);
    assert_boolean_sources_retained(&denied_fixture, 2);
}
