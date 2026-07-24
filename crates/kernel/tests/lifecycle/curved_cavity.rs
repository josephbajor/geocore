//! Public lifecycle evidence for one exact contained cylindrical cavity.

use super::*;
use kernel::PointBodyVerdict;

fn contained_cylinder_fixture() -> BooleanFixture {
    block_cylinder_boolean_fixture(
        Frame::world(),
        [6.0, 6.0, 6.0],
        Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
        0.75,
        2.0,
    )
}

fn offset_cylinder_fixture() -> BooleanFixture {
    block_cylinder_boolean_fixture(
        Frame::world(),
        [6.0, 6.0, 6.0],
        Frame::world().with_origin(Point3::new(1.0, 0.5, -1.0)),
        0.75,
        2.0,
    )
}

#[test]
fn public_contained_cylinder_subtraction_commits_an_exact_two_shell_cavity() {
    let mut fixture = contained_cylinder_fixture();
    let block = fixture.left.clone();
    let cylinder = fixture.right.clone();
    let result = boolean_success(run_boolean(
        &mut fixture,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("block minus its contained cylinder must create a cavity")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_boolean_created_full_valid(&created);
    assert_eq!(created.journal().part(), fixture.part);
    assert!(created.journal().mutation_count() > 0);
    assert_eq!(created.journal().lineage_count(), 9);
    assert_boolean_sources_retained(&fixture, 3);

    let body = created.bodies()[0].clone();
    assert_eq!(
        boolean_body_topology_signature(&fixture, body.clone()),
        [9, 14, 8]
    );
    assert_eq!(
        boolean_topology_counts(&fixture),
        [3, 7, 4, 18, 20, 56, 28, 16]
    );

    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let properties_query = || {
        part.body_properties(kernel::BodyPropertiesRequest::new(body.clone()))
            .unwrap()
    };
    let properties_outcome = properties_query();
    assert_eq!(properties_query(), properties_outcome);
    let kernel::BodyPropertiesOutcome::Certified {
        properties,
        full_check,
    } = properties_outcome.into_result().unwrap()
    else {
        panic!("Full-valid exact cavity properties were refused")
    };
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);
    assert!(
        properties
            .volume()
            .contains(216.0 - core::f64::consts::PI * 0.75 * 0.75 * 2.0)
    );
    assert!(
        properties
            .surface_area()
            .contains(216.0 + 2.0 * core::f64::consts::PI * 0.75 * (0.75 + 2.0))
    );
    let cavity_volume = core::f64::consts::PI * 0.75 * 0.75 * 2.0;
    let cavity_transverse_inertia = cavity_volume * (3.0 * 0.75 * 0.75 + 2.0 * 2.0) / 12.0;
    let cavity_axial_inertia = 0.5 * cavity_volume * 0.75 * 0.75;
    assert!(properties.centroidal_inertia().contains([
        [1296.0 - cavity_transverse_inertia, 0.0, 0.0],
        [0.0, 1296.0 - cavity_transverse_inertia, 0.0],
        [0.0, 0.0, 1296.0 - cavity_axial_inertia],
    ]));
    let body_view = part.body(body.clone()).unwrap();
    let regions = body_view.regions().collect::<Vec<_>>();
    assert_eq!(regions.len(), 3);
    let solid_regions = regions
        .iter()
        .filter(|region| part.region((*region).clone()).unwrap().kind() == RegionKind::Solid)
        .cloned()
        .collect::<Vec<_>>();
    let void_regions = regions
        .iter()
        .filter(|region| part.region((*region).clone()).unwrap().kind() == RegionKind::Void)
        .cloned()
        .collect::<Vec<_>>();
    assert_eq!(solid_regions.len(), 1);
    assert_eq!(void_regions.len(), 2);
    assert!(
        void_regions
            .iter()
            .all(|region| part.region(region.clone()).unwrap().shells().len() == 0)
    );
    let solid = part.region(solid_regions[0].clone()).unwrap();
    let shells = solid.shells().collect::<Vec<_>>();
    assert_eq!(shells.len(), 2);
    let mut shell_face_counts = shells
        .iter()
        .map(|shell| {
            let shell = part.shell(shell.clone()).unwrap();
            assert_eq!(shell.region(), solid_regions[0]);
            shell.faces().len()
        })
        .collect::<Vec<_>>();
    shell_face_counts.sort_unstable();
    assert_eq!(shell_face_counts, vec![3, 6]);

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
    let mut cylinder_sources = 0;
    for event in created.journal().lineage() {
        let LineageView::DerivedFrom {
            derived: JournalEntity::Face(derived),
            source: JournalEntity::Face(source),
        } = event
        else {
            panic!("cavity lineage must be face-only DerivedFrom")
        };
        assert!(result_faces.contains(&derived));
        assert!(!derived_faces.contains(&derived));
        derived_faces.push(derived.clone());
        if block_faces.contains(&source) {
            block_sources += 1;
            assert_eq!(
                part.face(derived).unwrap().sense(),
                part.face(source).unwrap().sense()
            );
        } else if cylinder_faces.contains(&source) {
            cylinder_sources += 1;
            assert_ne!(
                part.face(derived).unwrap().sense(),
                part.face(source).unwrap().sense()
            );
        } else {
            panic!("cavity lineage escaped both source bodies")
        }
    }
    assert_eq!(derived_faces.len(), result_faces.len());
    assert_eq!((block_sources, cylinder_sources), (6, 3));

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

    let mut bounded_edges = 0;
    let mut circle_edges = 0;
    for edge in body_view.edges().unwrap() {
        let edge = part.edge(edge).unwrap();
        assert_eq!(edge.fins().len(), 2);
        let class = part
            .curve(edge.curve().expect("cavity edges retain analytic curves"))
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
            class => panic!("unexpected cavity edge class: {class}"),
        }
    }
    assert_eq!((bounded_edges, circle_edges), (12, 2));

    for (point, expected, crossings) in [
        (Point3::new(1.5, 0.0, 0.0), PointBodyVerdict::Interior, 1),
        (Point3::new(0.2, 0.1, 0.0), PointBodyVerdict::Exterior, 2),
        (Point3::new(0.2, 0.1, 2.0), PointBodyVerdict::Interior, 1),
        (Point3::new(0.2, 0.1, -2.0), PointBodyVerdict::Interior, 3),
        (Point3::new(4.0, 0.0, 0.0), PointBodyVerdict::Exterior, 0),
    ] {
        let classification = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(classification.verdict(), &expected, "point {point:?}");
        assert_eq!(classification.witness().unwrap().crossings(), crossings);
    }
    for point in [
        Point3::new(0.75, 0.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(3.0, 0.0, 0.0),
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

    let mut replay = contained_cylinder_fixture();
    let replay_result = boolean_success(run_boolean(
        &mut replay,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(replayed) = replay_result else {
        panic!("fresh replay must create the same cavity")
    };
    let second =
        assert_deterministic_xt_and_fast_self_import(&mut replay, &[replayed.bodies()[0].clone()]);
    assert_eq!(first, second);

    let mut reversed = contained_cylinder_fixture();
    core::mem::swap(&mut reversed.left, &mut reversed.right);
    let before = boolean_topology_counts(&reversed);
    assert_eq!(before, [2, 4, 2, 9, 10, 28, 14, 8]);
    assert!(matches!(
        boolean_success(run_boolean(
            &mut reversed,
            BooleanOperation::Subtract,
            OperationSettings::new(),
        )),
        BooleanResult::ProvenEmpty
    ));
    assert_eq!(boolean_topology_counts(&reversed), before);
    assert_boolean_sources_retained(&reversed, 2);
}

#[test]
fn offset_cavity_properties_obey_parallel_axis_subtraction() {
    let mut fixture = offset_cylinder_fixture();
    let result = boolean_success(run_boolean(
        &mut fixture,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("block minus its offset contained cylinder must create a cavity")
    };
    assert_boolean_created_full_valid(&created);
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let kernel::BodyPropertiesOutcome::Certified {
        properties,
        full_check,
    } = part
        .body_properties(kernel::BodyPropertiesRequest::new(
            created.bodies()[0].clone(),
        ))
        .unwrap()
        .into_result()
        .unwrap()
    else {
        panic!("offset cavity properties were refused")
    };
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);

    let cavity_mass = core::f64::consts::PI * 0.75 * 0.75 * 2.0;
    let remaining_volume = 216.0 - cavity_mass;
    let cavity_center = [1.0, 0.5, 0.0];
    let remaining_center = cavity_center.map(|value| -cavity_mass * value / remaining_volume);
    assert!(properties.volume().contains(remaining_volume));
    assert!(properties.centroid().contains(Point3::new(
        remaining_center[0],
        remaining_center[1],
        remaining_center[2],
    )));
    assert!(
        properties
            .surface_area()
            .contains(216.0 + 2.0 * core::f64::consts::PI * 0.75 * (0.75 + 2.0))
    );

    let parallel_axis = |mass: f64, displacement: [f64; 3]| -> [[f64; 3]; 3] {
        let squared = displacement
            .into_iter()
            .map(|value| value * value)
            .sum::<f64>();
        core::array::from_fn(|row| {
            core::array::from_fn(|column| {
                mass * (if row == column { squared } else { 0.0 }
                    - displacement[row] * displacement[column])
            })
        })
    };
    let cavity_transverse = cavity_mass * (3.0 * 0.75 * 0.75 + 4.0) / 12.0;
    let cavity_axial = 0.5 * cavity_mass * 0.75 * 0.75;
    let cavity_centroidal = [
        [cavity_transverse, 0.0, 0.0],
        [0.0, cavity_transverse, 0.0],
        [0.0, 0.0, cavity_axial],
    ];
    let cavity_shift = parallel_axis(cavity_mass, cavity_center);
    let result_shift = parallel_axis(remaining_volume, remaining_center);
    let expected_inertia: [[f64; 3]; 3] = core::array::from_fn(|row| {
        core::array::from_fn(|column| {
            (if row == column { 1296.0 } else { 0.0 })
                - cavity_centroidal[row][column]
                - cavity_shift[row][column]
                - result_shift[row][column]
        })
    });
    assert!(expected_inertia[0][1].abs() > 0.0);
    assert!(properties.centroidal_inertia().contains(expected_inertia));
    assert!(properties.centroidal_inertia().error_bound() <= 1.0e-7);
}

#[test]
fn public_contained_cylinder_cavity_budget_is_exact_and_failure_atomic() {
    let baseline = run_boolean(
        &mut contained_cylinder_fixture(),
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
    assert!(usage.consumed > 0, "stage must meter work");

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
        &mut contained_cylinder_fixture(),
        BooleanOperation::Subtract,
        settings_at(usage.consumed),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied_fixture = contained_cylinder_fixture();
    let before = boolean_topology_counts(&denied_fixture);
    assert_eq!(before, [2, 4, 2, 9, 10, 28, 14, 8]);
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
