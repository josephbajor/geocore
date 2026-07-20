//! Public lifecycle evidence for a constructive full-ring axial contact.
//!
//! Wall-time budget: less than 60 seconds as part of the `lifecycle` target.

use super::*;
use kernel::{ClassifyPointOnFaceRequest, PointBodyVerdict, PointFaceVerdict};

const CONSTRUCTIVE_CONTACT_REALIZATION_WORK: u64 = 280;
const PORT_X: f64 = 0.5;
const PORT_Y: f64 = -0.25;

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Oblique,
}

#[derive(Debug, Clone, Copy)]
enum ContactEndpoint {
    Lower,
    Upper,
}

#[derive(Debug, Clone, Copy)]
struct ContactCase {
    placement: Placement,
    endpoint: ContactEndpoint,
    swapped: bool,
}

const CONTACT_CASES: [ContactCase; 8] = [
    ContactCase {
        placement: Placement::World,
        endpoint: ContactEndpoint::Lower,
        swapped: false,
    },
    ContactCase {
        placement: Placement::World,
        endpoint: ContactEndpoint::Lower,
        swapped: true,
    },
    ContactCase {
        placement: Placement::Oblique,
        endpoint: ContactEndpoint::Lower,
        swapped: false,
    },
    ContactCase {
        placement: Placement::Oblique,
        endpoint: ContactEndpoint::Lower,
        swapped: true,
    },
    ContactCase {
        placement: Placement::World,
        endpoint: ContactEndpoint::Upper,
        swapped: false,
    },
    ContactCase {
        placement: Placement::World,
        endpoint: ContactEndpoint::Upper,
        swapped: true,
    },
    ContactCase {
        placement: Placement::Oblique,
        endpoint: ContactEndpoint::Upper,
        swapped: false,
    },
    ContactCase {
        placement: Placement::Oblique,
        endpoint: ContactEndpoint::Upper,
        swapped: true,
    },
];

fn contact_frame(placement: Placement) -> Frame {
    match placement {
        Placement::World => Frame::world(),
        Placement::Oblique => boolean_frame([3.0, -2.0, 1.25], [0.0, 0.6, 0.8], [1.0, 0.0, 0.0]),
    }
}

fn contact_axial_geometry(endpoint: ContactEndpoint) -> (f64, f64, f64, f64) {
    match endpoint {
        // `(source origin, contacted cap, far cap, outward direction)`.
        ContactEndpoint::Lower => (1.0, 1.0, 3.0, 1.0),
        ContactEndpoint::Upper => (-3.0, -1.0, -3.0, -1.0),
    }
}

fn flush_contact_fixture(case: ContactCase) -> (BooleanFixture, BodyId, BodyId) {
    let block_frame = contact_frame(case.placement);
    let (source_origin_z, _, _, _) = contact_axial_geometry(case.endpoint);
    let cylinder_frame = Frame::new(
        block_frame.point_at(PORT_X, PORT_Y, source_origin_z),
        block_frame.z(),
        block_frame.x(),
    )
    .unwrap();
    let mut fixture =
        block_cylinder_boolean_fixture(block_frame, [4.0, 4.0, 2.0], cylinder_frame, 0.75, 2.0);
    let block = fixture.left.clone();
    let cylinder = fixture.right.clone();
    if case.swapped {
        core::mem::swap(&mut fixture.left, &mut fixture.right);
    }
    (fixture, block, cylinder)
}

fn source_signatures(fixture: &BooleanFixture) -> [[usize; 3]; 2] {
    [
        boolean_body_topology_signature(fixture, fixture.left.clone()),
        boolean_body_topology_signature(fixture, fixture.right.clone()),
    ]
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

fn assert_flush_source_contact(
    fixture: &BooleanFixture,
    case: ContactCase,
    block: BodyId,
    cylinder: BodyId,
) {
    assert_eq!(
        boolean_topology_counts(fixture),
        [2, 4, 2, 9, 10, 28, 14, 8]
    );
    let frame = contact_frame(case.placement);
    let (_, contact_z, _, outward) = contact_axial_geometry(case.endpoint);
    for local in [
        [PORT_X, PORT_Y, contact_z],
        [PORT_X + 0.75, PORT_Y, contact_z],
    ] {
        let point = frame.point_at(local[0], local[1], local[2]);
        for body in [block.clone(), cylinder.clone()] {
            assert!(
                matches!(
                    classify_body(fixture, body, point),
                    PointBodyVerdict::Boundary { .. }
                ),
                "{case:?} source contact point {local:?}"
            );
        }
    }
    for (body, local, expected) in [
        (
            block.clone(),
            [PORT_X, PORT_Y, contact_z - outward * 0.25],
            PointBodyVerdict::Interior,
        ),
        (
            cylinder.clone(),
            [PORT_X, PORT_Y, contact_z - outward * 0.25],
            PointBodyVerdict::Exterior,
        ),
        (
            block,
            [PORT_X, PORT_Y, contact_z + outward * 0.25],
            PointBodyVerdict::Exterior,
        ),
        (
            cylinder,
            [PORT_X, PORT_Y, contact_z + outward * 0.25],
            PointBodyVerdict::Interior,
        ),
    ] {
        assert_eq!(
            classify_body(fixture, body, frame.point_at(local[0], local[1], local[2])),
            expected,
            "{case:?} source point {local:?}"
        );
    }
}

fn run_flush_contact_union(fixture: &mut BooleanFixture) -> kernel::BooleanCreatedResult {
    let result = boolean_success(run_boolean(
        fixture,
        BooleanOperation::Unite,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("a flush full-ring contact union must create one connected boss")
    };
    created
}

fn assert_contact_lineage_and_topology(
    fixture: &BooleanFixture,
    created: &kernel::BooleanCreatedResult,
    case: ContactCase,
    block: BodyId,
    cylinder: BodyId,
    body: BodyId,
) {
    assert_eq!(
        boolean_body_topology_signature(fixture, body.clone()),
        [8, 14, 8]
    );
    assert_eq!(
        boolean_topology_counts(fixture),
        [3, 6, 3, 17, 20, 56, 28, 16]
    );

    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let result_faces = part
        .body(body)
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
    assert_eq!(
        (result_faces.len(), block_faces.len(), cylinder_faces.len()),
        (8, 6, 3)
    );

    let frame = contact_frame(case.placement);
    let (_, contact_z, far_z, _) = contact_axial_geometry(case.endpoint);
    let on_face = |face, point| {
        matches!(
            part.classify_point_on_face(ClassifyPointOnFaceRequest::new(face, point))
                .unwrap()
                .into_result()
                .unwrap()
                .verdict(),
            PointFaceVerdict::On(_)
        )
    };
    let contact_cap = cylinder_faces
        .iter()
        .find(|face| on_face((*face).clone(), frame.point_at(PORT_X, PORT_Y, contact_z)))
        .unwrap()
        .clone();
    let far_cap = cylinder_faces
        .iter()
        .find(|face| on_face((*face).clone(), frame.point_at(PORT_X, PORT_Y, far_z)))
        .unwrap()
        .clone();
    let side = cylinder_faces
        .iter()
        .find(|face| {
            part.surface(part.face((*face).clone()).unwrap().surface())
                .unwrap()
                .class_key()
                .as_str()
                == "kernel.surface.cylinder.v1"
        })
        .unwrap()
        .clone();
    assert!(contact_cap != far_cap && contact_cap != side && far_cap != side);

    let mut derived_faces = Vec::new();
    let mut block_sources = Vec::new();
    let mut cylinder_sources = Vec::new();
    for event in created.journal().lineage() {
        let LineageView::DerivedFrom {
            derived: JournalEntity::Face(derived),
            source: JournalEntity::Face(source),
        } = event
        else {
            panic!("constructive contact lineage must be face-only DerivedFrom")
        };
        assert!(result_faces.contains(&derived));
        assert!(!derived_faces.contains(&derived));
        assert_eq!(
            part.face(derived.clone()).unwrap().sense(),
            part.face(source.clone()).unwrap().sense()
        );
        derived_faces.push(derived);
        if block_faces.contains(&source) {
            block_sources.push(source);
        } else if cylinder_faces.contains(&source) {
            cylinder_sources.push(source);
        } else {
            panic!("constructive contact lineage escaped both source bodies")
        }
    }
    assert_eq!(derived_faces.len(), result_faces.len());
    assert_eq!(block_sources.len(), 6);
    assert!(block_faces.iter().all(|face| block_sources.contains(face)));
    assert_eq!(cylinder_sources.len(), 2);
    assert!(cylinder_sources.contains(&side));
    assert!(cylinder_sources.contains(&far_cap));
    assert!(!cylinder_sources.contains(&contact_cap));

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
}

fn assert_contact_edges_and_classification(
    fixture: &BooleanFixture,
    case: ContactCase,
    body: BodyId,
) {
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
            class => panic!("unexpected constructive-contact edge class: {class}"),
        }
    }
    assert_eq!(edge_classes, [12, 2]);
    drop(part);

    let frame = contact_frame(case.placement);
    let (_, contact_z, far_z, outward) = contact_axial_geometry(case.endpoint);
    let boss_midpoint = (contact_z + far_z) * 0.5;
    for (local, expected) in [
        ([1.5, 0.0, 0.0], PointBodyVerdict::Interior),
        ([0.0, 0.0, 0.0], PointBodyVerdict::Interior),
        ([PORT_X, PORT_Y, contact_z], PointBodyVerdict::Interior),
        ([PORT_X, PORT_Y, boss_midpoint], PointBodyVerdict::Interior),
        ([2.25, 0.0, 0.0], PointBodyVerdict::Exterior),
        (
            [PORT_X + 1.0, PORT_Y, boss_midpoint],
            PointBodyVerdict::Exterior,
        ),
        (
            [PORT_X, PORT_Y, far_z + outward * 0.25],
            PointBodyVerdict::Exterior,
        ),
    ] {
        assert_eq!(
            classify_body(
                fixture,
                body.clone(),
                frame.point_at(local[0], local[1], local[2])
            ),
            expected,
            "{case:?} result point {local:?}"
        );
    }
    for local in [
        [PORT_X + 0.75, PORT_Y, contact_z],
        [PORT_X + 1.0, PORT_Y, contact_z],
        [PORT_X + 0.75, PORT_Y, boss_midpoint],
        [PORT_X, PORT_Y, far_z],
    ] {
        assert!(
            matches!(
                classify_body(
                    fixture,
                    body.clone(),
                    frame.point_at(local[0], local[1], local[2])
                ),
                PointBodyVerdict::Boundary { .. }
            ),
            "{case:?} result boundary point {local:?}"
        );
    }
}

#[test]
fn public_flush_cap_contact_unite_commits_one_exact_connected_boss() {
    for case in CONTACT_CASES {
        let (mut fixture, block, cylinder) = flush_contact_fixture(case);
        assert_flush_source_contact(&fixture, case, block.clone(), cylinder.clone());
        let created = run_flush_contact_union(&mut fixture);
        assert_eq!(created.bodies().len(), 1);
        assert_boolean_created_full_valid(&created);
        assert_eq!(created.journal().part(), fixture.part);
        assert_eq!(created.journal().lineage_count(), 8);
        assert_boolean_sources_retained(&fixture, 3);
        let body = created.bodies()[0].clone();
        assert_contact_lineage_and_topology(
            &fixture,
            &created,
            case,
            block,
            cylinder,
            body.clone(),
        );
        assert_contact_edges_and_classification(&fixture, case, body.clone());

        let first = assert_deterministic_xt_and_fast_self_import(&mut fixture, &[body]);
        let (mut replay, _, _) = flush_contact_fixture(case);
        let replayed = run_flush_contact_union(&mut replay);
        let second = assert_deterministic_xt_and_fast_self_import(
            &mut replay,
            &[replayed.bodies()[0].clone()],
        );
        assert_eq!(first, second, "{case:?} deterministic X_T replay");
    }
}

#[test]
fn public_flush_cap_contact_realization_work_is_exact_and_denial_is_failure_atomic() {
    let case = ContactCase {
        placement: Placement::World,
        endpoint: ContactEndpoint::Lower,
        swapped: false,
    };
    let baseline = run_boolean(
        &mut flush_contact_fixture(case).0,
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
    assert_eq!(usage.consumed, CONSTRUCTIVE_CONTACT_REALIZATION_WORK);

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
        &mut flush_contact_fixture(case).0,
        BooleanOperation::Unite,
        settings_at(CONSTRUCTIVE_CONTACT_REALIZATION_WORK),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let (mut denied_fixture, _, _) = flush_contact_fixture(case);
    let before = boolean_topology_counts(&denied_fixture);
    let before_sources = source_signatures(&denied_fixture);
    let denied = run_boolean(
        &mut denied_fixture,
        BooleanOperation::Unite,
        settings_at(CONSTRUCTIVE_CONTACT_REALIZATION_WORK - 1),
    );
    let expected = kernel::LimitSnapshot {
        allowed: CONSTRUCTIVE_CONTACT_REALIZATION_WORK - 1,
        ..usage
    };
    assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(boolean_topology_counts(&denied_fixture), before);
    assert_eq!(source_signatures(&denied_fixture), before_sources);
    assert_boolean_sources_retained(&denied_fixture, 2);
}
