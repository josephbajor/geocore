//! Adversarial coverage for exact shared-corner spatial root certificates.

use kcore::expansion;
use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
use kcore::predicates::{Orientation, orient3d};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::{
    CurvePairProjectionPlane, NurbsCurve, NurbsCurvePairBudgetProfile,
    certify_curve_pair_unique_root, isolate_curve_pair_candidates_in_scope,
};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

fn quadratic(points: [Point3; 3], weights: Option<[f64; 3]>) -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points.to_vec(),
        weights.map(Vec::from),
    )
    .unwrap()
}

fn full_range() -> ParamRange {
    ParamRange::new(0.0, 1.0)
}

fn rounded_restriction_adversary() -> (NurbsCurve, NurbsCurve) {
    (
        quadratic(
            [
                Point3::new(-0.5, -0.5, 1.0),
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(0.5, 0.5, 1.0),
            ],
            None,
        ),
        quadratic(
            [
                Point3::new(-0.5, 0.5, 1.0),
                Point3::new(0.0, 0.0, 1.0_f64.next_up()),
                Point3::new(0.5, -0.5, 1.0),
            ],
            None,
        ),
    )
}

#[test]
fn certifies_non_coplanar_polynomial_curves_at_an_exact_shared_corner() {
    let first = quadratic(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(2.0, 2.0, 0.25),
        ],
        None,
    );
    let second = quadratic(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(-2.0, 2.0, 0.75),
        ],
        None,
    );

    let certificate = certify_curve_pair_unique_root(&first, full_range(), &second, full_range())
        .unwrap()
        .expect("the exact spatial corner and injective xy map prove one root");

    assert_eq!(certificate.first_range(), full_range());
    assert_eq!(certificate.second_range(), full_range());
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn certifies_non_coplanar_rational_curves_with_different_parameter_directions() {
    let first = quadratic(
        [
            Point3::new(-2.0, 2.0, 0.25),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(0.0, 0.0, 0.0),
        ],
        Some([1.0, 1.1, 1.2]),
    );
    let second = quadratic(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(2.0, 2.0, 0.75),
        ],
        Some([1.2, 1.1, 1.0]),
    );

    let certificate = certify_curve_pair_unique_root(&first, full_range(), &second, full_range())
        .unwrap()
        .expect("opposite parameter corners remain exact rational endpoints");
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn projected_crossing_without_a_shared_3d_point_is_not_an_existence_witness() {
    let first = quadratic(
        [
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(0.0, 0.0, 0.5),
            Point3::new(1.0, 1.0, 1.0),
        ],
        None,
    );
    let second = quadratic(
        [
            Point3::new(-1.0, 1.0, 2.0),
            Point3::new(0.0, 0.0, 2.5),
            Point3::new(1.0, -1.0, 3.0),
        ],
        None,
    );

    assert!(
        certify_curve_pair_unique_root(&first, full_range(), &second, full_range())
            .unwrap()
            .is_none()
    );
}

#[test]
fn exact_spatial_corner_with_projection_singularity_stays_inconclusive() {
    let first = quadratic(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(2.0, 0.0, 0.0),
        ],
        None,
    );
    let second = quadratic(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, -1.0),
            Point3::new(2.0, 0.0, 0.5),
        ],
        None,
    );

    assert!(
        certify_curve_pair_unique_root(&first, full_range(), &second, full_range())
            .unwrap()
            .is_none()
    );
}

#[test]
fn exact_rational_midpoints_certify_a_non_coplanar_root_without_knots_or_shared_corners() {
    let first = quadratic(
        [
            Point3::new(-2.0, -2.0, 1.125),
            Point3::new(0.0, 0.0, -1.0),
            Point3::new(2.0, 2.0, 1.125),
        ],
        Some([1.0, 1.125, 1.0]),
    );
    let second = quadratic(
        [
            Point3::new(-2.0, 2.0, -1.0),
            Point3::new(0.0, 0.0, 1.125),
            Point3::new(2.0, -2.0, -1.0),
        ],
        Some([1.125, 1.0, 1.125]),
    );

    assert_ne!(first.points().first(), second.points().first());
    assert_ne!(first.points().last(), second.points().last());
    assert_ne!(
        orient3d(
            [-2.0, -2.0, 1.125],
            [0.0, 0.0, -1.0],
            [2.0, 2.0, 1.125],
            [-2.0, 2.0, -1.0],
        ),
        Orientation::Zero,
        "the midpoint proof fixture must not fall back to coplanarity"
    );
    assert_eq!(first.eval(0.5), Point3::new(0.0, 0.0, 0.0));
    assert_eq!(second.eval(0.5), Point3::new(0.0, 0.0, 0.0));

    let certificate = certify_curve_pair_unique_root(&first, full_range(), &second, full_range())
        .unwrap()
        .expect("exact rational midpoint equality plus injective xy map proves one root");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn coincident_rounded_midpoints_do_not_grant_a_spatial_root_certificate() {
    let first = quadratic(
        [
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
        ],
        None,
    );
    let second = quadratic(
        [
            Point3::new(-1.0, 1.0, 1.0),
            Point3::new(0.0, 0.0, 1.0_f64.next_up()),
            Point3::new(1.0, -1.0, 1.0),
        ],
        None,
    );

    assert_eq!(
        first.eval(0.5),
        second.eval(0.5),
        "rounded evaluation deliberately hides the exact z mismatch"
    );
    assert!(
        certify_curve_pair_unique_root(&first, full_range(), &second, full_range())
            .unwrap()
            .is_none()
    );
}

#[test]
fn rounded_restricted_corner_does_not_grant_a_source_range_certificate() {
    let (first, second) = rounded_restriction_adversary();
    let range = ParamRange::new(0.0, 0.5);
    let restricted_first = first.restricted_to(range).unwrap();
    let restricted_second = second.restricted_to(range).unwrap();
    assert_eq!(
        restricted_first.points().last(),
        restricted_second.points().last(),
        "the adversary must keep reproducing the rounded false corner"
    );

    let delta = 1.0_f64.next_up() - 1.0;
    let exact_source_difference = expansion::scale(&[delta], 0.5);
    assert_eq!(expansion::sign(&exact_source_difference), 1);
    assert!(
        certify_curve_pair_unique_root(&first, range, &second, range)
            .unwrap()
            .is_none()
    );
}

#[test]
fn rounded_isolation_children_validate_against_shared_sources() {
    let (first, second) = rounded_restriction_adversary();
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_family_budget_defaults(NurbsCurvePairBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    let isolation = isolate_curve_pair_candidates_in_scope(
        &first,
        full_range(),
        &second,
        full_range(),
        0.0,
        1,
        &mut scope,
    )
    .unwrap();

    assert_eq!(isolation.candidates().len(), 4);
    assert!(
        isolation
            .candidates()
            .iter()
            .all(|cell| cell.certify_unique_root().is_none())
    );
}

#[test]
fn exact_source_midpoint_endpoint_cross_pair_preserves_partial_range_certification() {
    let first = quadratic(
        [
            Point3::new(-2.0, -2.0, 1.125),
            Point3::new(0.0, 0.0, -1.0),
            Point3::new(2.0, 2.0, 1.125),
        ],
        Some([1.0, 1.125, 1.0]),
    );
    let second = quadratic(
        [
            Point3::new(-2.0, 2.0, -1.0),
            Point3::new(0.0, 0.0, 1.125),
            Point3::new(2.0, -2.0, -1.0),
        ],
        Some([1.125, 1.0, 1.125]),
    );
    let first_range = ParamRange::new(0.25, 0.75);
    let second_range = ParamRange::new(0.25, 0.5);

    let certificate = certify_curve_pair_unique_root(&first, first_range, &second, second_range)
        .unwrap()
        .expect("the exact source midpoint/endpoint pair and global P-matrix remain valid");
    assert_eq!(certificate.first_range(), first_range);
    assert_eq!(certificate.second_range(), second_range);
}
