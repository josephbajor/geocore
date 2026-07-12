//! Standalone verification of exact interior-knot spatial root certificates.

use kcore::predicates::{Orientation, orient3d};
use kgeom::curve::Curve;
use kgeom::nurbs::{CurvePairProjectionPlane, NurbsCurve, certify_curve_pair_unique_root};
use kgeom::vec::Point3;

fn full_break_quadratic(points: [Point3; 5], weights: Option<[f64; 5]>) -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        points.to_vec(),
        weights.map(Vec::from),
    )
    .unwrap()
}

#[test]
fn exact_full_multiplicity_knots_certify_a_unique_spatial_interior_root() {
    let first = full_break_quadratic(
        [
            Point3::new(-2.0, -2.0, -1.0),
            Point3::new(-1.0, -1.0, 2.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, -2.0),
            Point3::new(2.0, 2.0, 1.0),
        ],
        Some([1.0, 1.001, 1.002, 1.001, 1.0]),
    );
    let second = full_break_quadratic(
        [
            Point3::new(-2.0, 2.0, 3.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, -1.0, 4.0),
            Point3::new(2.0, -2.0, -3.0),
        ],
        Some([1.002, 1.001, 1.0, 1.001, 1.002]),
    );

    assert_ne!(first.points().first(), second.points().first());
    assert_ne!(first.points().last(), second.points().last());
    assert_ne!(
        orient3d(
            [-2.0, -2.0, -1.0],
            [-1.0, -1.0, 2.0],
            [0.0, 0.0, 0.0],
            [-2.0, 2.0, 3.0],
        ),
        Orientation::Zero,
        "the combined control points must be genuinely noncoplanar"
    );
    assert_eq!(first.eval(0.5), Point3::new(0.0, 0.0, 0.0));
    assert_eq!(second.eval(0.5), Point3::new(0.0, 0.0, 0.0));

    let certificate =
        certify_curve_pair_unique_root(&first, first.param_range(), &second, second.param_range())
            .unwrap()
            .expect("exact interior witness plus injective xy difference proves one root");
    assert_eq!(certificate.first_range(), first.param_range());
    assert_eq!(certificate.second_range(), second.param_range());
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn equal_projected_full_multiplicity_points_do_not_prove_a_3d_root() {
    let first = full_break_quadratic(
        [
            Point3::new(-2.0, -2.0, 0.0),
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
        ],
        None,
    );
    let second = full_break_quadratic(
        [
            Point3::new(-2.0, 2.0, 1.0),
            Point3::new(-1.0, 1.0, 1.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, -1.0, 1.0),
            Point3::new(2.0, -2.0, 1.0),
        ],
        None,
    );

    assert_eq!(first.eval(0.5).x, second.eval(0.5).x);
    assert_eq!(first.eval(0.5).y, second.eval(0.5).y);
    assert_ne!(first.eval(0.5).z, second.eval(0.5).z);
    assert!(
        certify_curve_pair_unique_root(&first, first.param_range(), &second, second.param_range(),)
            .unwrap()
            .is_none()
    );
}

#[test]
fn a_shared_control_point_at_a_simple_knot_is_not_an_exact_witness() {
    let knots = vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0];
    let shared_control = Point3::new(0.5, 0.5, 0.0);
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(-1.5, -1.5, -1.0),
            Point3::new(-0.5, -0.5, 1.0),
            shared_control,
            Point3::new(1.5, 1.5, 2.0),
        ],
        None,
    )
    .unwrap();
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(-1.5, 1.5, 3.0),
            Point3::new(-0.5, 0.5, -2.0),
            shared_control,
            Point3::new(1.5, -1.5, -3.0),
        ],
        None,
    )
    .unwrap();

    assert!(first.points().contains(&shared_control));
    assert!(second.points().contains(&shared_control));
    assert!(
        certify_curve_pair_unique_root(&first, first.param_range(), &second, second.param_range(),)
            .unwrap()
            .is_none()
    );
}
