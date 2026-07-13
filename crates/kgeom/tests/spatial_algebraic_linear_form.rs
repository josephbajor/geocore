//! Coverage for signed-linear-form algebraic spatial-root lifts.

use kcore::predicates::{Orientation, orient3d};
use kgeom::nurbs::{CurvePairProjectionPlane, NurbsCurve, certify_curve_pair_unique_root};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

const KNOTS: [f64; 6] = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];

fn curve(points: [Point3; 3], weights: Option<[f64; 3]>) -> NurbsCurve {
    NurbsCurve::new(2, KNOTS.to_vec(), points.to_vec(), weights.map(Vec::from)).unwrap()
}

fn polynomial_pair() -> (NurbsCurve, NurbsCurve) {
    (
        curve(
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 1.0),
                Point3::new(4.0, 0.0, 0.0),
            ],
            None,
        ),
        curve(
            [
                Point3::new(1.0, -1.0, 1.0),
                Point3::new(1.5, 0.5, 0.5),
                Point3::new(2.0, 2.0, -2.0),
            ],
            None,
        ),
    )
}

fn rational_pair(second_weights: [f64; 3]) -> (NurbsCurve, NurbsCurve) {
    (
        curve(
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 1.0),
                Point3::new(4.0, 0.0, 0.0),
            ],
            Some([1.0, 1.0, 1.0]),
        ),
        curve(
            [
                Point3::new(1.0, -1.0, 1.0),
                Point3::new(1.5, 0.5, 0.5),
                Point3::new(2.0, 2.0, -2.0),
            ],
            Some(second_weights),
        ),
    )
}

fn full_range() -> ParamRange {
    ParamRange::new(0.0, 1.0)
}

fn partial_range() -> ParamRange {
    ParamRange::new(0.25, 0.5)
}

fn rational_first_range() -> ParamRange {
    partial_range()
}

fn assert_no_coordinate_scalar_corresponds(first: &NurbsCurve, second: &NurbsCurve) {
    for axis in 0..3 {
        let component = |point: Point3| match axis {
            0 => point.x,
            1 => point.y,
            2 => point.z,
            _ => unreachable!(),
        };
        assert!(
            !first
                .points()
                .iter()
                .zip(second.points())
                .all(|(&first, &second)| component(first) == component(second))
        );
        assert!(
            !first
                .points()
                .iter()
                .zip(second.points().iter().rev())
                .all(|(&first, &second)| component(first) == component(second))
        );
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
fn signed_carrier_and_residual_lift_a_root_without_any_shared_coordinate_scalar() {
    let (first, second) = polynomial_pair();
    assert_no_coordinate_scalar_corresponds(&first, &second);
    assert_genuinely_noncoplanar(&first, &second);

    // The sources share x+y=4t and z-x, but no individual coordinate.  Their
    // projected differences vanish only where 3t-1=0.  The requested second
    // range excludes endpoints, midpoint 3/8, and every full-multiplicity knot.
    let certificate =
        certify_curve_pair_unique_root(&first, full_range(), &second, partial_range())
            .unwrap()
            .expect("a monotone signed carrier and signed omitted residual lift the root");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert_eq!(certificate.first_range(), full_range());
    assert_eq!(certificate.second_range(), partial_range());
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn signed_linear_form_lift_supports_swap_and_affine_reversal() {
    let (first, second) = polynomial_pair();
    let swapped = certify_curve_pair_unique_root(&second, partial_range(), &first, full_range())
        .unwrap()
        .expect("the broader algebraic proof is operand-order independent");
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
    .expect("exact normalized reversed forms lift the t=1/3, s=14 root");
    assert_eq!(reversed.projection_plane(), CurvePairProjectionPlane::Xy);
}

#[test]
fn proportional_rational_forms_lift_the_same_algebraic_family() {
    let (first, second) = rational_pair([2.0, 2.0, 2.0]);
    assert_no_coordinate_scalar_corresponds(&first, &second);
    assert_genuinely_noncoplanar(&first, &second);

    let certificate =
        certify_curve_pair_unique_root(&first, rational_first_range(), &second, partial_range())
            .unwrap()
            .expect("proportional rational denominators preserve exact signed forms");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn broken_carrier_residual_and_weight_gates_remain_inconclusive() {
    let (first, second) = polynomial_pair();

    let mut carrier_points = second.points().to_vec();
    carrier_points[1].y += 0.25;
    let broken_carrier = NurbsCurve::new(2, KNOTS.to_vec(), carrier_points, None).unwrap();
    assert!(
        certify_curve_pair_unique_root(&first, full_range(), &broken_carrier, partial_range(),)
            .unwrap()
            .is_none()
    );

    let mut residual_points = second.points().to_vec();
    residual_points[1].z += 0.25;
    let broken_residual = NurbsCurve::new(2, KNOTS.to_vec(), residual_points, None).unwrap();
    assert!(
        certify_curve_pair_unique_root(&first, full_range(), &broken_residual, partial_range(),)
            .unwrap()
            .is_none()
    );

    let (rational_first, nonproportional) = rational_pair([2.0, 2.5, 2.0]);
    assert!(
        certify_curve_pair_unique_root(
            &rational_first,
            rational_first_range(),
            &nonproportional,
            partial_range(),
        )
        .unwrap()
        .is_none()
    );
}
