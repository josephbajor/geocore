//! Public lifecycle evidence for exact axial cap/planar-support contact.
//!
//! Wall-time budget: less than 60 seconds as part of the `lifecycle` target.

use super::*;

#[derive(Clone, Copy)]
enum ContactEndpoint {
    Lower,
    Upper,
}

fn transformed_block_frame() -> Frame {
    boolean_frame([1.25, -0.75, 0.5], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
}

fn transformed_contact_fixture(endpoint: ContactEndpoint) -> BooleanFixture {
    let block_frame = transformed_block_frame();
    let cylinder_origin = match endpoint {
        ContactEndpoint::Lower => block_frame.point_at(0.0, 0.0, 1.0),
        ContactEndpoint::Upper => block_frame.point_at(0.0, 0.0, -3.0),
    };
    let cylinder_frame = Frame::new(cylinder_origin, block_frame.z(), block_frame.x()).unwrap();
    block_cylinder_boolean_fixture(block_frame, [4.0, 4.0, 2.0], cylinder_frame, 0.75, 2.0)
}

fn transformed_strict_overlap_fixture() -> BooleanFixture {
    let block_frame = transformed_block_frame();
    let cylinder_frame = Frame::new(
        block_frame.point_at(0.0, 0.0, 0.5),
        block_frame.z(),
        block_frame.x(),
    )
    .unwrap();
    block_cylinder_boolean_fixture(block_frame, [4.0, 4.0, 2.0], cylinder_frame, 0.75, 2.0)
}

fn source_signatures(fixture: &BooleanFixture) -> [[usize; 3]; 2] {
    [
        boolean_body_topology_signature(fixture, fixture.left.clone()),
        boolean_body_topology_signature(fixture, fixture.right.clone()),
    ]
}

fn assert_unchanged_sources(
    fixture: &BooleanFixture,
    before_counts: [usize; 8],
    before_sources: [[usize; 3]; 2],
) {
    assert_eq!(boolean_topology_counts(fixture), before_counts);
    assert_eq!(source_signatures(fixture), before_sources);
    assert_boolean_sources_retained(fixture, 2);
}

fn assert_contact_classification(
    fixture: &BooleanFixture,
    block: BodyId,
    cylinder: BodyId,
    endpoint: ContactEndpoint,
) {
    let frame = transformed_block_frame();
    let (contact_z, cylinder_step, block_step) = match endpoint {
        ContactEndpoint::Lower => (1.0, 0.25, -0.25),
        ContactEndpoint::Upper => (-1.0, -0.25, 0.25),
    };
    let contact = frame.point_at(0.0, 0.0, contact_z);
    let into_cylinder = frame.point_at(0.0, 0.0, contact_z + cylinder_step);
    let into_block = frame.point_at(0.0, 0.0, contact_z + block_step);
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let classify = |body, point| {
        part.classify_point_in_body(ClassifyPointInBodyRequest::new(body, point))
            .unwrap()
            .into_result()
            .unwrap()
    };

    for body in [block.clone(), cylinder.clone()] {
        assert!(matches!(
            classify(body, contact).verdict(),
            kernel::PointBodyVerdict::Boundary { .. }
        ));
    }
    assert_eq!(
        classify(cylinder.clone(), into_cylinder).verdict(),
        &kernel::PointBodyVerdict::Interior
    );
    assert_eq!(
        classify(block.clone(), into_cylinder).verdict(),
        &kernel::PointBodyVerdict::Exterior
    );
    assert_eq!(
        classify(block, into_block).verdict(),
        &kernel::PointBodyVerdict::Interior
    );
    assert_eq!(
        classify(cylinder, into_block).verdict(),
        &kernel::PointBodyVerdict::Exterior
    );
}

#[test]
fn public_axial_cap_contact_intersection_is_proven_empty_without_source_mutation() {
    for endpoint in [ContactEndpoint::Lower, ContactEndpoint::Upper] {
        for swapped in [false, true] {
            let mut fixture = transformed_contact_fixture(endpoint);
            let block = fixture.left.clone();
            let cylinder = fixture.right.clone();
            assert_contact_classification(&fixture, block, cylinder, endpoint);
            if swapped {
                core::mem::swap(&mut fixture.left, &mut fixture.right);
            }
            let before = boolean_topology_counts(&fixture);
            assert_eq!(before, [2, 4, 2, 9, 10, 28, 14, 8]);
            let before_sources = source_signatures(&fixture);
            assert!(
                before_sources == [[6, 12, 8], [3, 2, 0]]
                    || before_sources == [[3, 2, 0], [6, 12, 8]]
            );

            let result = boolean_success(run_boolean(
                &mut fixture,
                BooleanOperation::Intersect,
                OperationSettings::new(),
            ));
            assert!(matches!(result, BooleanResult::ProvenEmpty));
            assert!(result.is_empty());
            assert!(result.bodies().is_empty());
            assert!(result.created().is_none());
            assert_unchanged_sources(&fixture, before, before_sources);
        }
    }

    let mut overlap = transformed_strict_overlap_fixture();
    let result = boolean_success(run_boolean(
        &mut overlap,
        BooleanOperation::Intersect,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("strict axial overlap must not take the contact-empty shortcut")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_boolean_created_full_valid(&created);
    assert_eq!(
        boolean_body_topology_signature(&overlap, created.bodies()[0].clone()),
        [3, 2, 0]
    );
    assert_boolean_sources_retained(&overlap, 3);
}

#[test]
fn public_axial_cap_contact_work_is_exact_and_denial_is_failure_atomic() {
    let baseline = run_boolean(
        &mut transformed_contact_fixture(ContactEndpoint::Lower),
        BooleanOperation::Intersect,
        OperationSettings::new(),
    );
    assert!(matches!(
        baseline.result().unwrap(),
        BooleanOutcome::Success(BooleanResult::ProvenEmpty)
    ));
    let usage = *baseline
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == BOOLEAN_BSP_WORK && usage.resource == ResourceKind::Work)
        .unwrap();
    assert!(usage.consumed > 0, "stage must meter work");

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
        &mut transformed_contact_fixture(ContactEndpoint::Lower),
        BooleanOperation::Intersect,
        settings_at(usage.consumed),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::ProvenEmpty)
    ));

    let mut denied_fixture = transformed_contact_fixture(ContactEndpoint::Lower);
    let before = boolean_topology_counts(&denied_fixture);
    let before_sources = source_signatures(&denied_fixture);
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
    assert_unchanged_sources(&denied_fixture, before, before_sources);
}
