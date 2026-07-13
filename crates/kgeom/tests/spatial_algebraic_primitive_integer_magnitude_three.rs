//! Coverage for magnitude-three primitive-integer spatial-root lifts.

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
                Point3::new(1.0, 3.0, 7.0),
                Point3::new(3.5, -1.5, -2.5),
                Point3::new(6.0, -6.0, -14.0),
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

fn linear_form(point: Point3, coefficients: [i8; 3]) -> f64 {
    f64::from(coefficients[0]) * point.x
        + f64::from(coefficients[1]) * point.y
        + f64::from(coefficients[2]) * point.z
}

fn controls_correspond(
    first: &NurbsCurve,
    second: &NurbsCurve,
    coefficients: [i8; 3],
    reversed: bool,
) -> bool {
    let second: Box<dyn Iterator<Item = &Point3>> = if reversed {
        Box::new(second.points().iter().rev())
    } else {
        Box::new(second.points().iter())
    };
    first.points().iter().zip(second).all(|(&first, &second)| {
        linear_form(first, coefficients) == linear_form(second, coefficients)
    })
}

fn assert_no_magnitude_two_projected_carrier_corresponds(first: &NurbsCurve, second: &NurbsCurve) {
    for first_axis in 0..3 {
        let mut coordinate = [0; 3];
        coordinate[first_axis] = 1;
        assert!(!controls_correspond(first, second, coordinate, false));
        assert!(!controls_correspond(first, second, coordinate, true));

        for second_axis in first_axis + 1..3 {
            for first_coefficient in -2_i8..=2 {
                for second_coefficient in -2_i8..=2 {
                    if first_coefficient == 0 || second_coefficient == 0 {
                        continue;
                    }
                    let mut coefficients = [0; 3];
                    coefficients[first_axis] = first_coefficient;
                    coefficients[second_axis] = second_coefficient;
                    assert!(
                        !controls_correspond(first, second, coefficients, false),
                        "same-orientation magnitude-two carrier {coefficients:?}"
                    );
                    assert!(
                        !controls_correspond(first, second, coefficients, true),
                        "reversed magnitude-two carrier {coefficients:?}"
                    );
                }
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
fn magnitude_three_carrier_certifies_a_new_partial_range_family() {
    let (first, second) = pair(None);
    assert_no_magnitude_two_projected_carrier_corresponds(&first, &second);
    assert_genuinely_noncoplanar(&first, &second);
    assert!(controls_correspond(&first, &second, [3, -1, 0], false));
    assert!(controls_correspond(&first, &second, [-1, -2, 1], false));

    let root = 1.0 / 3.0;
    assert!(partial_range().lo < root && root < partial_range().hi);
    assert_ne!(root, partial_range().lo);
    assert_ne!(root, partial_range().lerp(0.5));
    assert_ne!(root, partial_range().hi);
    assert!(!first.knots().as_slice().contains(&root));

    // Corresponding control differences are `(1,3,7)*(1-3t)`.
    // The smallest projected carrier is `3x-y`; no coordinate or form
    // bounded by magnitude two corresponds. The residual `-x-2y+z`
    // completes the exact spatial lift.
    let certificate =
        certify_curve_pair_unique_root(&first, full_range(), &second, partial_range())
            .unwrap()
            .expect("the primitive magnitude-three carrier lifts the normalized one-third root");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert_eq!(certificate.first_range(), full_range());
    assert_eq!(certificate.second_range(), partial_range());
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn primitive_magnitude_three_forms_support_swap_and_affine_reversal() {
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
fn proportional_positive_rational_weights_preserve_magnitude_three_forms() {
    let (first, second) = pair(Some(([1.0, 1.0, 1.0], [2.0, 2.0, 2.0])));
    assert_no_magnitude_two_projected_carrier_corresponds(&first, &second);

    let certificate =
        certify_curve_pair_unique_root(&first, partial_range(), &second, partial_range())
            .unwrap()
            .expect("globally proportional rational weights preserve magnitude-three forms");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn broken_magnitude_three_forms_and_nonproportional_weights_fail_closed() {
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
