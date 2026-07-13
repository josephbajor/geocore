//! Coverage for magnitude-two primitive-integer spatial-root lifts.

use kcore::predicates::{Orientation, orient3d};
use kgeom::nurbs::{CurvePairProjectionPlane, NurbsCurve, certify_curve_pair_unique_root};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

const KNOTS: [f64; 6] = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];

fn curve(points: [Point3; 3], weights: Option<[f64; 3]>) -> NurbsCurve {
    NurbsCurve::new(2, KNOTS.to_vec(), points.to_vec(), weights.map(Vec::from)).unwrap()
}

fn pair(weights: Option<([f64; 3], [f64; 3])>) -> (NurbsCurve, NurbsCurve) {
    let (first_weights, second_weights) =
        weights.map_or((None, None), |(first, second)| (Some(first), Some(second)));
    (
        curve(
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(4.0, 0.0, 1.0),
                Point3::new(8.0, 0.0, 0.0),
            ],
            first_weights,
        ),
        curve(
            [
                Point3::new(2.0, -4.0, 1.0),
                Point3::new(3.0, 2.0, 0.5),
                Point3::new(4.0, 8.0, -2.0),
            ],
            second_weights,
        ),
    )
}

fn full_range() -> ParamRange {
    ParamRange::new(0.0, 1.0)
}

fn partial_range() -> ParamRange {
    ParamRange::new(0.25, 0.5)
}

fn component(point: Point3, axis: usize) -> f64 {
    match axis {
        0 => point.x,
        1 => point.y,
        2 => point.z,
        _ => unreachable!(),
    }
}

fn form(point: Point3, first_axis: usize, second_axis: usize, sign: f64) -> f64 {
    component(point, first_axis) + sign * component(point, second_axis)
}

fn assert_no_coordinate_or_unit_carrier_corresponds(first: &NurbsCurve, second: &NurbsCurve) {
    for first_axis in 0..3 {
        assert!(
            !first
                .points()
                .iter()
                .zip(second.points())
                .all(|(&first, &second)| {
                    component(first, first_axis) == component(second, first_axis)
                })
        );
        for second_axis in first_axis + 1..3 {
            for sign in [1.0, -1.0] {
                assert!(
                    !first
                        .points()
                        .iter()
                        .zip(second.points())
                        .all(|(&first, &second)| {
                            form(first, first_axis, second_axis, sign)
                                == form(second, first_axis, second_axis, sign)
                        })
                );
                assert!(
                    !first.points().iter().zip(second.points().iter().rev()).all(
                        |(&first, &second)| {
                            form(first, first_axis, second_axis, sign)
                                == form(second, first_axis, second_axis, sign)
                        }
                    )
                );
            }
        }
    }
}

fn assert_genuinely_noncoplanar(first: &NurbsCurve, second: &NurbsCurve) {
    let [a, b, c] = first.points() else {
        panic!("fixture is a quadratic Bezier");
    };
    let d = second.points()[0];
    assert_ne!(
        orient3d(
            [a.x, a.y, a.z],
            [b.x, b.y, b.z],
            [c.x, c.y, c.z],
            [d.x, d.y, d.z],
        ),
        Orientation::Zero
    );
}

#[test]
fn magnitude_two_carrier_and_residual_certify_a_new_partial_range_family() {
    let (first, second) = pair(None);
    assert_no_coordinate_or_unit_carrier_corresponds(&first, &second);
    assert_genuinely_noncoplanar(&first, &second);
    let root = 1.0 / 3.0;
    assert!(partial_range().lo < root && root < partial_range().hi);
    assert_ne!(root, partial_range().lo);
    assert_ne!(root, partial_range().lerp(0.5));
    assert_ne!(root, partial_range().hi);
    assert!(!first.knots().as_slice().contains(&root));

    // Differences are `(2,-4,1)*(1-3t)`. Thus `2x+y` and `2z-x`
    // correspond exactly, while every coordinate and unit signed carrier
    // differs. The certificate discovers the projected root algebraically.
    let certificate =
        certify_curve_pair_unique_root(&first, full_range(), &second, partial_range())
            .unwrap()
            .expect("primitive magnitude-two forms lift the normalized one-third root");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert_eq!(certificate.first_range(), full_range());
    assert_eq!(certificate.second_range(), partial_range());
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn primitive_magnitude_two_forms_support_swap_and_affine_reversal() {
    let (first, second) = pair(None);
    let swapped = certify_curve_pair_unique_root(&second, partial_range(), &first, full_range())
        .unwrap()
        .expect("primitive form enumeration is stable under operand swap");
    assert_eq!(swapped.projection_plane(), CurvePairProjectionPlane::Xy);

    let reversed_second = NurbsCurve::new(
        2,
        vec![10.0, 10.0, 10.0, 16.0, 16.0, 16.0],
        second.points().iter().copied().rev().collect(),
        None,
    )
    .unwrap();
    let reversed = certify_curve_pair_unique_root(
        &first,
        full_range(),
        &reversed_second,
        ParamRange::new(13.0, 14.5),
    )
    .unwrap()
    .expect("normalized reversal lifts the t=1/3, s=14 root");
    assert_eq!(reversed.projection_plane(), CurvePairProjectionPlane::Xy);
}

#[test]
fn proportional_positive_rational_forms_preserve_the_new_family() {
    let (first, second) = pair(Some(([1.0, 1.0, 1.0], [2.0, 2.0, 2.0])));
    assert_no_coordinate_or_unit_carrier_corresponds(&first, &second);

    let certificate =
        certify_curve_pair_unique_root(&first, partial_range(), &second, partial_range())
            .unwrap()
            .expect("globally proportional rational weights preserve primitive forms");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn broken_primitive_forms_and_nonproportional_weights_fail_closed() {
    let (first, second) = pair(None);

    let mut carrier_points = second.points().to_vec();
    carrier_points[1].y += 0.25;
    let broken_carrier = NurbsCurve::new(2, KNOTS.to_vec(), carrier_points, None).unwrap();
    assert!(
        certify_curve_pair_unique_root(&first, full_range(), &broken_carrier, partial_range())
            .unwrap()
            .is_none()
    );

    let mut residual_points = second.points().to_vec();
    residual_points[1].z += 0.25;
    let broken_residual = NurbsCurve::new(2, KNOTS.to_vec(), residual_points, None).unwrap();
    assert!(
        certify_curve_pair_unique_root(&first, full_range(), &broken_residual, partial_range())
            .unwrap()
            .is_none()
    );

    let (rational_first, nonproportional) = pair(Some(([1.0, 1.0, 1.0], [2.0, 2.5, 2.0])));
    assert!(
        certify_curve_pair_unique_root(
            &rational_first,
            partial_range(),
            &nonproportional,
            partial_range(),
        )
        .unwrap()
        .is_none()
    );
}

#[test]
fn nonfinite_source_inputs_are_rejected_before_certification() {
    assert!(
        NurbsCurve::new(
            2,
            KNOTS.to_vec(),
            vec![
                Point3::new(f64::INFINITY, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            None,
        )
        .is_err()
    );
}
