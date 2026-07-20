//! Public lifecycle evidence for exact two-port curved Boolean results.

use super::*;

fn two_port_through_hole_fixture() -> BooleanFixture {
    block_cylinder_boolean_fixture(
        Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
        [4.0, 4.0, 1.0],
        Frame::world(),
        0.75,
        2.0,
    )
}

#[test]
fn public_block_minus_spanning_cylinder_commits_a_full_valid_through_hole() {
    let mut fixture = two_port_through_hole_fixture();
    let block = fixture.left.clone();
    let cylinder = fixture.right.clone();
    let result = boolean_success(run_boolean(
        &mut fixture,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("block minus a spanning coaxial cylinder must create a through-hole")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_boolean_created_full_valid(&created);
    assert_eq!(created.journal().part(), fixture.part);
    assert!(created.journal().mutation_count() > 0);
    assert_eq!(created.journal().lineage_count(), 7);
    assert_boolean_sources_retained(&fixture, 3);

    let body = created.bodies()[0].clone();
    assert_eq!(
        boolean_body_topology_signature(&fixture, body.clone()),
        [7, 14, 8]
    );
    assert_eq!(
        boolean_topology_counts(&fixture),
        [3, 6, 3, 16, 20, 56, 28, 16]
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
            panic!("through-hole lineage must be face-only DerivedFrom")
        };
        assert!(result_faces.contains(&derived));
        assert!(!derived_faces.contains(&derived));
        derived_faces.push(derived);
        if block_faces.contains(&source) {
            block_sources += 1;
        } else if cylinder_faces.contains(&source) {
            cylinder_sources += 1;
        } else {
            panic!("through-hole lineage escaped both source bodies")
        }
    }
    assert_eq!(derived_faces.len(), result_faces.len());
    assert_eq!(block_sources, 6);
    assert_eq!(cylinder_sources, 1);

    let mut surface_classes = Vec::new();
    let mut loop_counts = Vec::new();
    for face in &result_faces {
        let face = part.face(face.clone()).unwrap();
        surface_classes.push(part.surface(face.surface()).unwrap().class_key().as_str());
        loop_counts.push(face.loops().len());
    }
    loop_counts.sort_unstable();
    assert_eq!(loop_counts, vec![1, 1, 1, 1, 2, 2, 2]);
    assert_eq!(
        surface_classes
            .iter()
            .filter(|class| **class == "kernel.surface.plane.v1")
            .count(),
        6
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
                    .expect("through-hole edges retain analytic curves"),
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
            class => panic!("unexpected through-hole edge class: {class}"),
        }
    }
    assert_eq!((bounded_edges, circle_edges), (12, 2));

    for (point, expected) in [
        (
            Point3::new(1.5, 0.0, 1.0),
            kernel::PointBodyVerdict::Interior,
        ),
        (
            Point3::new(0.0, 0.0, 1.0),
            kernel::PointBodyVerdict::Exterior,
        ),
        (
            Point3::new(0.0, 0.0, 2.25),
            kernel::PointBodyVerdict::Exterior,
        ),
        (
            Point3::new(2.25, 0.0, 1.0),
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
    for point in [Point3::new(0.75, 0.0, 1.0), Point3::new(1.5, 0.0, 0.5)] {
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

    let mut replay = two_port_through_hole_fixture();
    let replay_result = boolean_success(run_boolean(
        &mut replay,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(replayed) = replay_result else {
        panic!("fresh replay must create a through-hole")
    };
    let second =
        assert_deterministic_xt_and_fast_self_import(&mut replay, &[replayed.bodies()[0].clone()]);
    assert_eq!(first, second);

    let mut reverse_order = two_port_through_hole_fixture();
    core::mem::swap(&mut reverse_order.left, &mut reverse_order.right);
    let reverse_result = boolean_success(run_boolean(
        &mut reverse_order,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(reverse_created) = reverse_result else {
        panic!("cylinder minus a spanning block must create two remainder bands")
    };
    assert_eq!(reverse_created.bodies().len(), 2);
    assert_boolean_created_full_valid(&reverse_created);
    assert_eq!(
        reverse_created
            .bodies()
            .iter()
            .map(|body| boolean_body_topology_signature(&reverse_order, body.clone()))
            .collect::<Vec<_>>(),
        vec![[3, 2, 0], [3, 2, 0]]
    );
    assert_boolean_sources_retained(&reverse_order, 4);
}

#[test]
fn public_two_port_realization_budget_is_exact_and_denial_is_failure_atomic() {
    let baseline = run_boolean(
        &mut two_port_through_hole_fixture(),
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
        &mut two_port_through_hole_fixture(),
        BooleanOperation::Subtract,
        settings_at(usage.consumed),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied_fixture = two_port_through_hole_fixture();
    let before = boolean_topology_counts(&denied_fixture);
    let denied = run_boolean(
        &mut denied_fixture,
        BooleanOperation::Subtract,
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
