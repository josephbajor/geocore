//! Public lifecycle evidence for certified material-distance enclosures.
//!
//! Wall-time budget: less than 60 seconds as part of the `lifecycle` target.

use super::*;
use kernel::{
    BodyClashAssessment, BodyClashOutcome, BodyClashRequest, BodyClashVerdict, BodyDistanceOutcome,
    BodyDistanceRequest, CertifiedBodyDistance,
};

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

fn assessed_clash(
    part: &kernel::Part<'_>,
    body_a: BodyId,
    body_b: BodyId,
    clearance: f64,
) -> BodyClashAssessment {
    let outcome = part
        .body_clash(BodyClashRequest::new(
            body_a.clone(),
            body_b.clone(),
            clearance,
        ))
        .unwrap()
        .into_result()
        .unwrap();
    assert!(outcome.assessment().is_some());
    assert!(outcome.refusal().is_none());
    let BodyClashOutcome::Assessed {
        assessment,
        full_checks,
    } = outcome
    else {
        panic!("Full-valid exact Plane/Cylinder bodies must admit clash assessment")
    };
    assert_eq!(assessment.bodies(), [body_a.clone(), body_b.clone()]);
    assert_eq!(assessment.body_a(), body_a);
    assert_eq!(assessment.body_b(), body_b);
    assert_eq!(assessment.clearance().to_bits(), clearance.to_bits());
    assert_eq!(assessment.distance().bodies(), [body_a, body_b]);
    assert_eq!(
        assessment.distance().upper_witness().distance().upper(),
        assessment.distance().distance().upper()
    );
    assert_eq!(full_checks[0].body(), assessment.body_a());
    assert_eq!(full_checks[1].body(), assessment.body_b());
    assert!(
        full_checks
            .iter()
            .all(|check| check.report().outcome() == CheckOutcome::Valid)
    );
    assessment
}

fn distance_bits(distance: &CertifiedBodyDistance) -> [u64; 2] {
    [
        distance.distance().lower().to_bits(),
        distance.distance().upper().to_bits(),
    ]
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
fn public_distance_and_clash_certify_block_and_cylinder_thresholds_under_rigid_motion_and_swap() {
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
    let block_gap_swapped = certified_distance(&part, block_b.clone(), block_a.clone());
    assert!(block_gap.distance().contains(3.0));
    assert_eq!(block_gap.distance(), block_gap_repeat.distance());
    assert_eq!(block_gap.distance(), block_gap_swapped.distance());

    let lower = block_gap.distance().lower();
    let upper = block_gap.distance().upper();
    assert!(0.0 < lower);
    assert!(lower < upper);
    let interior_clearance = lower + (upper - lower) * 0.5;
    assert!(lower < interior_clearance && interior_clearance < upper);

    let clear = assessed_clash(&part, block_a.clone(), block_b.clone(), 0.0);
    assert!(clear.clearance() < clear.distance().distance().lower());
    assert_eq!(clear.verdict(), BodyClashVerdict::Clear);

    let clashing = assessed_clash(&part, block_a.clone(), block_b.clone(), upper);
    assert!(clashing.distance().distance().upper() <= clashing.clearance());
    assert_eq!(clashing.verdict(), BodyClashVerdict::Clashing);

    let indeterminate = assessed_clash(&part, block_a.clone(), block_b.clone(), interior_clearance);
    let indeterminate_repeat =
        assessed_clash(&part, block_a.clone(), block_b.clone(), interior_clearance);
    let indeterminate_swapped =
        assessed_clash(&part, block_b.clone(), block_a.clone(), interior_clearance);
    assert_eq!(indeterminate.verdict(), BodyClashVerdict::Indeterminate);
    assert!(indeterminate.distance().distance().lower() < indeterminate.clearance());
    assert!(indeterminate.clearance() < indeterminate.distance().distance().upper());
    assert_eq!(indeterminate_repeat.verdict(), indeterminate.verdict());
    assert_eq!(indeterminate_swapped.verdict(), indeterminate.verdict());
    assert_eq!(
        distance_bits(indeterminate.distance()),
        distance_bits(indeterminate_repeat.distance())
    );
    assert_eq!(
        distance_bits(indeterminate.distance()),
        distance_bits(indeterminate_swapped.distance())
    );
    assert_eq!(indeterminate.bodies(), [block_a.clone(), block_b.clone()]);
    assert_eq!(indeterminate_swapped.bodies(), [block_b, block_a]);

    let radial_gap = certified_distance(&part, radial_block.clone(), cylinder.clone());
    let radial_gap_swapped = certified_distance(&part, cylinder, radial_block);
    assert!(radial_gap.distance().contains(2.0));
    assert_eq!(radial_gap.distance(), radial_gap_swapped.distance());
    assert!(radial_gap.distance().lower() > 0.0);
}

#[test]
fn public_distance_and_clash_preserve_material_semantics_for_containment_and_a_boolean_cavity() {
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
    let contained_zero = {
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        assessed_clash(&part, fixture.left.clone(), fixture.right.clone(), 0.0)
    };
    assert_eq!(contained_zero.distance().distance().lower(), 0.0);
    assert!(contained_zero.distance().distance().upper() > 0.0);
    assert_eq!(contained_zero.verdict(), BodyClashVerdict::Indeterminate);

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
    let cavity_gap = certified_distance(&part, cavity.clone(), inner.clone());
    assert_eq!(cavity_gap.distance().lower(), 0.0);
    assert!(cavity_gap.distance().contains(0.5));
    assert!(cavity_gap.distance().upper().is_finite());
    let cavity_proximity = assessed_clash(&part, cavity, inner, cavity_gap.distance().upper());
    assert_eq!(cavity_proximity.verdict(), BodyClashVerdict::Clashing);
}
