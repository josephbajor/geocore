//! Public lifecycle evidence for certified material-distance enclosures.
//!
//! Wall-time budget: less than 60 seconds as part of the `lifecycle` target.

use super::*;
use kernel::{BodyDistanceOutcome, BodyDistanceRequest, CertifiedBodyDistance};

fn certified_distance(
    part: &kernel::Part<'_>,
    body_a: BodyId,
    body_b: BodyId,
) -> CertifiedBodyDistance {
    let outcome = part
        .body_distance(BodyDistanceRequest::new(body_a.clone(), body_b.clone()))
        .unwrap()
        .into_result()
        .unwrap();
    let BodyDistanceOutcome::Certified {
        distance,
        full_checks,
        ..
    } = outcome
    else {
        panic!("Full-valid exact Plane/Cylinder bodies must admit distance bounds")
    };
    assert_eq!(distance.bodies(), [body_a, body_b]);
    assert!(
        full_checks
            .iter()
            .all(|check| check.report().outcome() == CheckOutcome::Valid)
    );
    assert_eq!(
        distance.upper_witness().distance().upper(),
        distance.distance().upper()
    );
    distance
}

fn rigid_frame() -> Frame {
    Frame::new(
        Point3::new(10.0, -7.0, 3.0),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(2.0, -1.0, 0.0),
    )
    .unwrap()
}

#[test]
fn public_distance_certifies_block_and_cylinder_clearance_under_rigid_motion_and_swap() {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let frame = rigid_frame();
    let (block_a, block_b, radial_block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let block_a = edit
            .create_block(BlockRequest::new(frame, [2.0, 2.0, 2.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let block_b = edit
            .create_block(BlockRequest::new(
                frame.with_origin(frame.point_at(5.0, 0.0, 0.0)),
                [2.0, 2.0, 2.0],
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let radial_block = edit
            .create_block(BlockRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, 1.0)),
                [2.0, 2.0, 2.0],
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder_frame =
            Frame::new(frame.point_at(4.0, 0.0, 0.0), frame.z(), frame.x()).unwrap();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(cylinder_frame, 1.0, 2.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block_a, block_b, radial_block, cylinder)
    };
    let part = session.part(part_id).unwrap();

    let block_gap = certified_distance(&part, block_a.clone(), block_b.clone());
    let block_gap_repeat = certified_distance(&part, block_a.clone(), block_b.clone());
    let block_gap_swapped = certified_distance(&part, block_b, block_a);
    assert!(block_gap.distance().contains(3.0));
    assert_eq!(block_gap.distance(), block_gap_repeat.distance());
    assert_eq!(block_gap.distance(), block_gap_swapped.distance());

    let radial_gap = certified_distance(&part, radial_block.clone(), cylinder.clone());
    let radial_gap_swapped = certified_distance(&part, cylinder, radial_block);
    assert!(radial_gap.distance().contains(2.0));
    assert_eq!(radial_gap.distance(), radial_gap_swapped.distance());
    assert!(radial_gap.distance().lower() > 0.0);
}

#[test]
fn public_distance_preserves_material_semantics_for_containment_and_a_boolean_cavity() {
    let mut fixture = block_cylinder_boolean_fixture(
        Frame::world(),
        [6.0, 6.0, 6.0],
        Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
        0.75,
        2.0,
    );

    let contained = {
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        certified_distance(&part, fixture.left.clone(), fixture.right.clone())
    };
    assert_eq!(contained.distance().lower(), 0.0);
    assert!(contained.distance().contains(0.0));

    let result = boolean_success(run_boolean(
        &mut fixture,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    ));
    let BooleanResult::Created(created) = result else {
        panic!("contained cylinder subtraction must create one cavity body")
    };
    assert_boolean_created_full_valid(&created);
    let cavity = created.bodies()[0].clone();
    let inner = fixture
        .session
        .edit_part(fixture.part.clone())
        .unwrap()
        .create_cylinder(CylinderRequest::new(
            Frame::world().with_origin(Point3::new(0.0, 0.0, -0.5)),
            0.25,
            1.0,
        ))
        .unwrap()
        .into_result()
        .unwrap()
        .body();

    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let cavity_gap = certified_distance(&part, cavity, inner);
    assert_eq!(cavity_gap.distance().lower(), 0.0);
    assert!(cavity_gap.distance().contains(0.5));
    assert!(cavity_gap.distance().upper().is_finite());
}
