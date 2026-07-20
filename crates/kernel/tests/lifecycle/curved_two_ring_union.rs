//! Public lifecycle evidence for a two-ring, two-sided connected cylinder union.

use super::*;
use kernel::PointBodyVerdict;

fn two_ring_union_fixture() -> BooleanFixture {
    block_cylinder_boolean_fixture(
        Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
        [4.0, 4.0, 1.0],
        Frame::world(),
        0.75,
        2.0,
    )
}

fn run_two_ring_union(fixture: &mut BooleanFixture) -> kernel::BooleanCreatedResult {
    let result = boolean_success(run_boolean(
        fixture,
        BooleanOperation::Unite,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("two-ring connected union must create one two-ended cylinder host")
    };
    created
}

#[test]
fn public_two_ring_union_commits_one_exact_connected_shell() {
    let mut fixture = two_ring_union_fixture();
    let block = fixture.left.clone();
    let cylinder = fixture.right.clone();
    let created = run_two_ring_union(&mut fixture);
    assert_eq!(created.bodies().len(), 1);
    assert_boolean_created_full_valid(&created);
    assert_eq!(created.journal().part(), fixture.part);
    assert!(created.journal().mutation_count() > 0);
    assert_eq!(created.journal().lineage_count(), 10);
    assert_boolean_sources_retained(&fixture, 3);

    let body = created.bodies()[0].clone();
    assert_eq!(
        boolean_body_topology_signature(&fixture, body.clone()),
        [10, 16, 8]
    );
    assert_eq!(
        boolean_topology_counts(&fixture),
        [3, 6, 3, 19, 24, 60, 30, 16]
    );

    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let body_view = part.body(body.clone()).unwrap();
    let regions = body_view.regions().collect::<Vec<_>>();
    assert_eq!(regions.len(), 2);
    let solid = regions
        .iter()
        .find(|region| part.region((*region).clone()).unwrap().kind() == RegionKind::Solid)
        .unwrap();
    let exterior = regions
        .iter()
        .find(|region| part.region((*region).clone()).unwrap().kind() == RegionKind::Void)
        .unwrap();
    let shells = part
        .region(solid.clone())
        .unwrap()
        .shells()
        .collect::<Vec<_>>();
    assert_eq!(shells.len(), 1);
    assert_eq!(part.shell(shells[0].clone()).unwrap().faces().len(), 10);
    assert_eq!(part.region(exterior.clone()).unwrap().shells().len(), 0);

    let result_faces = body_view.faces().unwrap().collect::<Vec<_>>();
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
    let mut cylinder_side_sources = 0;
    let mut cylinder_cap_sources = 0;
    for event in created.journal().lineage() {
        let LineageView::DerivedFrom {
            derived: JournalEntity::Face(derived),
            source: JournalEntity::Face(source),
        } = event
        else {
            panic!("two-ring union lineage must be face-only DerivedFrom")
        };
        assert!(result_faces.contains(&derived));
        assert!(!derived_faces.contains(&derived));
        derived_faces.push(derived.clone());
        assert_eq!(
            part.face(derived).unwrap().sense(),
            part.face(source.clone()).unwrap().sense()
        );
        if block_faces.contains(&source) {
            block_sources += 1;
        } else if cylinder_faces.contains(&source) {
            match part
                .surface(part.face(source).unwrap().surface())
                .unwrap()
                .class_key()
                .as_str()
            {
                "kernel.surface.cylinder.v1" => cylinder_side_sources += 1,
                "kernel.surface.plane.v1" => cylinder_cap_sources += 1,
                class => panic!("unexpected two-ring source surface: {class}"),
            }
        } else {
            panic!("two-ring union lineage escaped both source bodies")
        }
    }
    assert_eq!(derived_faces.len(), result_faces.len());
    assert_eq!(
        (block_sources, cylinder_side_sources, cylinder_cap_sources),
        (6, 2, 2)
    );

    let mut surface_classes = Vec::new();
    let mut loop_counts = Vec::new();
    for face in &result_faces {
        let face = part.face(face.clone()).unwrap();
        surface_classes.push(part.surface(face.surface()).unwrap().class_key().as_str());
        loop_counts.push(face.loops().len());
    }
    loop_counts.sort_unstable();
    assert_eq!(loop_counts, vec![1, 1, 1, 1, 1, 1, 2, 2, 2, 2]);
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
        2
    );

    let mut bounded_edges = 0;
    let mut circle_edges = 0;
    for edge in body_view.edges().unwrap() {
        let edge = part.edge(edge).unwrap();
        assert_eq!(edge.fins().len(), 2);
        let class = part
            .curve(edge.curve().expect("two-ring edges retain analytic curves"))
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
            class => panic!("unexpected two-ring edge class: {class}"),
        }
    }
    assert_eq!((bounded_edges, circle_edges), (12, 4));

    for (point, expected) in [
        (Point3::new(1.5, 0.1, 1.0), PointBodyVerdict::Interior),
        (Point3::new(0.2, 0.1, 0.25), PointBodyVerdict::Interior),
        (Point3::new(0.2, 0.1, 1.75), PointBodyVerdict::Interior),
        (Point3::new(0.2, 0.1, 1.0), PointBodyVerdict::Interior),
        (Point3::new(0.2, 0.1, 0.5), PointBodyVerdict::Interior),
        (Point3::new(2.25, 0.1, 1.0), PointBodyVerdict::Exterior),
        (Point3::new(0.2, 0.1, -0.25), PointBodyVerdict::Exterior),
        (Point3::new(0.2, 0.1, 2.25), PointBodyVerdict::Exterior),
    ] {
        let classification = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(classification.verdict(), &expected, "point {point:?}");
    }
    for point in [
        Point3::new(0.75, 0.0, 0.25),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.75, 0.0, 1.75),
        Point3::new(0.0, 0.0, 2.0),
        Point3::new(1.0, 0.0, 0.5),
        Point3::new(0.75, 0.0, 0.5),
        Point3::new(2.0, 0.0, 1.0),
    ] {
        let classification = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert!(matches!(
            classification.verdict(),
            PointBodyVerdict::Boundary { .. }
        ));
        assert!(classification.witness().is_none());
    }
    drop(part);
    let first = assert_deterministic_xt_and_fast_self_import(&mut fixture, &[body]);

    let mut replay = two_ring_union_fixture();
    let replayed = run_two_ring_union(&mut replay);
    let second =
        assert_deterministic_xt_and_fast_self_import(&mut replay, &[replayed.bodies()[0].clone()]);
    assert_eq!(first, second);

    let mut swapped = two_ring_union_fixture();
    core::mem::swap(&mut swapped.left, &mut swapped.right);
    let swapped_created = run_two_ring_union(&mut swapped);
    assert_eq!(
        boolean_body_topology_signature(&swapped, swapped_created.bodies()[0].clone()),
        [10, 16, 8]
    );
    let swapped_xt = assert_deterministic_xt_and_fast_self_import(
        &mut swapped,
        &[swapped_created.bodies()[0].clone()],
    );
    assert_eq!(first, swapped_xt);
}

#[test]
fn public_two_ring_union_budget_is_exact_and_failure_atomic() {
    let baseline = run_boolean(
        &mut two_ring_union_fixture(),
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
    assert_eq!(usage.consumed, 313);

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
        &mut two_ring_union_fixture(),
        BooleanOperation::Unite,
        settings_at(usage.consumed),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied_fixture = two_ring_union_fixture();
    let before = boolean_topology_counts(&denied_fixture);
    assert_eq!(before, [2, 4, 2, 9, 10, 28, 14, 8]);
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
