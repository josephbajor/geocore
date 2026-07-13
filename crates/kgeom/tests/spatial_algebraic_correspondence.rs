//! Adversarial coverage for non-sampled algebraic spatial-root lifts.

use kcore::predicates::{Orientation, orient3d};
use kgeom::nurbs::{CurvePairProjectionPlane, NurbsCurve, certify_curve_pair_unique_root};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

const BEZIER_KNOTS: [f64; 6] = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
const ASYMMETRIC_KNOTS: [f64; 7] = [0.0, 0.0, 0.0, 0.75, 1.0, 1.0, 1.0];

fn quadratic(points: [Point3; 3], weights: Option<[f64; 3]>) -> NurbsCurve {
    NurbsCurve::new(
        2,
        BEZIER_KNOTS.to_vec(),
        points.to_vec(),
        weights.map(Vec::from),
    )
    .unwrap()
}

fn polynomial_pair() -> (NurbsCurve, NurbsCurve) {
    (
        NurbsCurve::new(
            2,
            ASYMMETRIC_KNOTS.to_vec(),
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.375, 0.0, 1.0),
                Point3::new(0.875, 0.0, -0.5),
                Point3::new(1.0, 0.0, 0.0),
            ],
            None,
        )
        .unwrap(),
        NurbsCurve::new(
            2,
            ASYMMETRIC_KNOTS.to_vec(),
            vec![
                Point3::new(0.0, -1.0, 0.0),
                Point3::new(0.375, 0.125, 1.0),
                Point3::new(0.875, 1.625, -0.5),
                Point3::new(1.0, 2.0, 0.0),
            ],
            None,
        )
        .unwrap(),
    )
}

fn rational_pair(second_weights: [f64; 3]) -> (NurbsCurve, NurbsCurve) {
    (
        quadratic(
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.5, 0.0, 1.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            Some([1.0, 2.0, 1.0]),
        ),
        quadratic(
            [
                Point3::new(0.0, -1.0, 0.0),
                Point3::new(0.5, 0.25, 1.0),
                Point3::new(1.0, 2.0, 0.0),
            ],
            Some(second_weights),
        ),
    )
}

fn partial_range() -> ParamRange {
    ParamRange::new(0.25, 0.5)
}

fn assert_genuinely_noncoplanar(first: &NurbsCurve, second: &NurbsCurve) {
    let [a, b, c, ..] = first.points() else {
        panic!("fixture needs three source controls");
    };
    assert_ne!(
        orient3d(
            [a.x, a.y, a.z],
            [b.x, b.y, b.z],
            [c.x, c.y, c.z],
            [
                second.points()[0].x,
                second.points()[0].y,
                second.points()[0].z,
            ],
        ),
        Orientation::Zero,
        "the algebraic lift fixture must not fall back to a common plane"
    );
}

#[test]
fn polynomial_root_at_normalized_one_third_certifies_on_a_partial_range() {
    let (first, second) = polynomial_pair();
    let range = partial_range();
    let normalized_root = 1.0 / 3.0;
    assert!(range.lo < normalized_root && normalized_root < range.hi);
    assert_ne!(normalized_root, range.lerp(0.5));
    assert!(!first.knots().as_slice().contains(&normalized_root));
    assert_ne!(first.points().first(), second.points().first());
    assert_ne!(first.points().last(), second.points().last());
    assert_genuinely_noncoplanar(&first, &second);

    // These Greville-abscissa controls reproduce the scalar -1 + 3t exactly,
    // so its algebraic zero is normalized t=1/3.  The certificate does not
    // evaluate either curve there (or at any other guessed parameter).
    let certificate = certify_curve_pair_unique_root(&first, range, &second, range)
        .unwrap()
        .expect("the carrier/omitted scalar lift proves the spatial root");
    assert_eq!(certificate.first_range(), range);
    assert_eq!(certificate.second_range(), range);
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn algebraic_lift_is_stable_under_operand_swap_and_reversed_correspondence() {
    let (first, second) = polynomial_pair();
    let range = partial_range();
    let swapped = certify_curve_pair_unique_root(&second, range, &first, range)
        .unwrap()
        .expect("the complementary projected-axis order handles operand swap");
    assert_eq!(swapped.projection_plane(), CurvePairProjectionPlane::Xy);

    let reversed_second = NurbsCurve::new(
        2,
        vec![10.0, 10.0, 10.0, 11.5, 16.0, 16.0, 16.0],
        vec![
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(0.875, 1.625, -0.5),
            Point3::new(0.375, 0.125, 1.0),
            Point3::new(0.0, -1.0, 0.0),
        ],
        None,
    )
    .unwrap();
    let reversed_range = ParamRange::new(13.0, 14.5);
    let reversed = certify_curve_pair_unique_root(&first, range, &reversed_second, reversed_range)
        .unwrap()
        .expect("affine normalized reverse knots lift the t=1/3, s=14 root");
    assert_eq!(reversed.first_range(), range);
    assert_eq!(reversed.second_range(), reversed_range);
    assert_eq!(reversed.projection_plane(), CurvePairProjectionPlane::Xy);
}

#[test]
fn globally_proportional_rational_weights_preserve_the_algebraic_lift() {
    let (first, second) = rational_pair([2.0, 4.0, 2.0]);
    let range = partial_range();
    assert_genuinely_noncoplanar(&first, &second);

    let certificate = certify_curve_pair_unique_root(&first, range, &second, range)
        .unwrap()
        .expect("exactly proportional weights preserve both shared scalar functions");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn altered_omitted_coordinate_and_nonproportional_weights_fail_closed() {
    let (first, mut altered) = polynomial_pair();
    let mut altered_points = altered.points().to_vec();
    altered_points[1].z = 1.25;
    altered = NurbsCurve::new(
        altered.degree(),
        altered.knots().as_slice().to_vec(),
        altered_points,
        None,
    )
    .unwrap();
    assert_genuinely_noncoplanar(&first, &altered);
    assert!(
        certify_curve_pair_unique_root(&first, partial_range(), &altered, partial_range())
            .unwrap()
            .is_none(),
        "a projected root cannot lift through a changed omitted scalar"
    );

    let (_, knot_mismatch_source) = polynomial_pair();
    let knot_mismatch = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.625, 1.0, 1.0, 1.0],
        knot_mismatch_source.points().to_vec(),
        None,
    )
    .unwrap();
    assert!(
        certify_curve_pair_unique_root(&first, partial_range(), &knot_mismatch, partial_range(),)
            .unwrap()
            .is_none(),
        "a changed normalized source knot invalidates parameter correspondence"
    );

    let (rational_first, nonproportional) = rational_pair([2.0, 5.0, 2.0]);
    assert!(
        certify_curve_pair_unique_root(
            &rational_first,
            partial_range(),
            &nonproportional,
            partial_range(),
        )
        .unwrap()
        .is_none(),
        "nonproportional denominators invalidate scalar control correspondence"
    );
}

#[test]
fn nonmonotone_shared_scalars_do_not_create_a_parameter_correspondence() {
    let first = quadratic(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, -1.0),
            Point3::new(0.0, 0.0, 1.0),
        ],
        None,
    );
    let second = quadratic(
        [
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, 0.5, -1.0),
            Point3::new(0.0, 2.0, 1.0),
        ],
        None,
    );
    assert_genuinely_noncoplanar(&first, &second);

    assert!(
        certify_curve_pair_unique_root(&first, partial_range(), &second, partial_range())
            .unwrap()
            .is_none(),
        "carrier derivative intervals containing zero must remain inconclusive"
    );
}
