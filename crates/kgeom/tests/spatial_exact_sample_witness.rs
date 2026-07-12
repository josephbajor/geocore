//! Low-level verification of exact rational NURBS sample witnesses.

use kcore::predicates::{Orientation, orient3d};
use kgeom::curve::Curve;
use kgeom::nurbs::NurbsCurve;
use kgeom::vec::Point3;

#[path = "../src/nurbs/spatial_exact_sample.rs"]
mod spatial_exact_sample;

use spatial_exact_sample::certify_exact_spatial_sample;

fn rational_quadratic(points: [Point3; 3], weights: [f64; 3]) -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points.to_vec(),
        Some(weights.to_vec()),
    )
    .unwrap()
}

#[test]
fn exact_rational_samples_prove_a_spatial_interior_point_without_knots_or_shared_corners() {
    let first = rational_quadratic(
        [
            Point3::new(-2.0, -2.0, 1.0),
            Point3::new(0.0, 0.0, -0.5),
            Point3::new(2.0, 2.0, 1.0),
        ],
        [1.0, 2.0, 1.0],
    );
    let second = rational_quadratic(
        [
            Point3::new(-2.0, 2.0, -1.0),
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(2.0, -2.0, -1.0),
        ],
        [2.0, 1.0, 2.0],
    );

    assert_eq!(first.knots().as_slice(), &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    assert_eq!(second.knots().as_slice(), &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    assert_ne!(first.points().first(), second.points().first());
    assert_ne!(first.points().last(), second.points().last());
    assert_ne!(
        orient3d(
            [-2.0, -2.0, 1.0],
            [0.0, 0.0, -0.5],
            [2.0, 2.0, 1.0],
            [-2.0, 2.0, -1.0],
        ),
        Orientation::Zero,
        "the combined control points must be genuinely noncoplanar"
    );
    assert_eq!(first.eval(0.5), Point3::new(0.0, 0.0, 0.0));
    assert_eq!(second.eval(0.5), Point3::new(0.0, 0.0, 0.0));

    let witness = certify_exact_spatial_sample(&first, 0.5, &second, 0.5)
        .expect("exact rational de Boor evaluation proves the common 3D sample");
    assert_eq!(witness.first_parameter(), 0.5);
    assert_eq!(witness.second_parameter(), 0.5);
}

#[test]
fn coincident_rounded_evaluations_do_not_grant_an_exact_witness() {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
        ],
        None,
    )
    .unwrap();
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(0.0, 0.0, 1.0_f64.next_up()),
            Point3::new(1.0, 1.0, 1.0),
        ],
        None,
    )
    .unwrap();

    assert_eq!(
        first.eval(0.5),
        second.eval(0.5),
        "the ordinary evaluator rounds distinct exact rational values together"
    );
    assert!(certify_exact_spatial_sample(&first, 0.5, &second, 0.5).is_none());
}

#[test]
fn arithmetic_outside_the_bounded_exact_corridor_is_inconclusive() {
    let huge = f64::from_bits(((1023 + 501) as u64) << 52);
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(huge, 0.0, 0.0), Point3::new(huge, 1.0, 0.0)],
        None,
    )
    .unwrap();

    assert!(certify_exact_spatial_sample(&curve, 0.5, &curve, 0.5).is_none());
}
