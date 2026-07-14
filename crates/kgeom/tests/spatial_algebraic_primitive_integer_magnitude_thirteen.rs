//! Configured coverage for magnitude-thirteen primitive-integer spatial lifts.

use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
use kcore::predicates::{Orientation, orient3d};
use kcore::tolerance::Tolerances;
use kgeom::nurbs::{
    CurvePairAlgebraicSearchConfig, CurvePairProjectionPlane, NurbsCurve,
    NurbsCurvePairBudgetProfile, certify_curve_pair_unique_root,
    certify_curve_pair_unique_root_with_config, isolate_curve_pair_candidates_in_scope,
};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

const KNOTS: [f64; 6] = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];

fn curve(points: [Point3; 3], weights: Option<[f64; 3]>) -> NurbsCurve {
    NurbsCurve::new(2, KNOTS.to_vec(), points.to_vec(), weights.map(Vec::from)).unwrap()
}

fn first_curve(weights: Option<[f64; 3]>) -> NurbsCurve {
    curve(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        weights,
    )
}

fn second_curve(coefficient: f64, weights: Option<[f64; 3]>) -> NurbsCurve {
    curve(
        [
            Point3::new(1.0, coefficient, 2.0 * coefficient + 1.0),
            Point3::new(3.5, -coefficient / 2.0, -coefficient + 0.5),
            Point3::new(6.0, -2.0 * coefficient, -4.0 * coefficient - 2.0),
        ],
        weights,
    )
}

fn magnitude_thirteen_pair(weights: Option<([f64; 3], [f64; 3])>) -> (NurbsCurve, NurbsCurve) {
    let (first_weights, second_weights) =
        weights.map_or((None, None), |(first, second)| (Some(first), Some(second)));
    (
        first_curve(first_weights),
        second_curve(13.0, second_weights),
    )
}

fn full_range() -> ParamRange {
    ParamRange::new(0.0, 1.0)
}

fn partial_range() -> ParamRange {
    ParamRange::new(0.25, 0.5)
}

fn magnitude_thirteen_config() -> CurvePairAlgebraicSearchConfig {
    CurvePairAlgebraicSearchConfig::new(13).unwrap()
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

fn assert_no_projected_carrier_through(first: &NurbsCurve, second: &NurbsCurve, bound: i8) {
    for first_axis in 0..3 {
        let mut coordinate = [0; 3];
        coordinate[first_axis] = 1;
        assert!(!controls_correspond(first, second, coordinate, false));
        assert!(!controls_correspond(first, second, coordinate, true));

        for second_axis in first_axis + 1..3 {
            for first_coefficient in -bound..=bound {
                for second_coefficient in -bound..=bound {
                    if first_coefficient == 0 || second_coefficient == 0 {
                        continue;
                    }
                    let mut coefficients = [0; 3];
                    coefficients[first_axis] = first_coefficient;
                    coefficients[second_axis] = second_coefficient;
                    assert!(
                        !controls_correspond(first, second, coefficients, false),
                        "same-orientation magnitude-{bound} carrier {coefficients:?}"
                    );
                    assert!(
                        !controls_correspond(first, second, coefficients, true),
                        "reversed magnitude-{bound} carrier {coefficients:?}"
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
fn configured_magnitude_thirteen_certifies_the_new_normalized_third_family() {
    let (first, second) = magnitude_thirteen_pair(None);
    assert_no_projected_carrier_through(&first, &second, 12);
    assert_genuinely_noncoplanar(&first, &second);
    assert!(controls_correspond(&first, &second, [13, -1, 0], false));
    assert!(controls_correspond(&first, &second, [-1, -2, 1], false));

    let root = 1.0 / 3.0;
    assert!(partial_range().lo < root && root < partial_range().hi);
    assert_ne!(root, partial_range().lo);
    assert_ne!(root, partial_range().lerp(0.5));
    assert_ne!(root, partial_range().hi);
    assert!(!first.knots().as_slice().contains(&root));

    assert!(
        certify_curve_pair_unique_root(&first, full_range(), &second, partial_range())
            .unwrap()
            .is_none(),
        "the compatibility entry must retain its magnitude-twelve ceiling"
    );
    assert!(
        certify_curve_pair_unique_root_with_config(
            &first,
            full_range(),
            &second,
            partial_range(),
            CurvePairAlgebraicSearchConfig::new(12).unwrap(),
        )
        .unwrap()
        .is_none()
    );

    // Corresponding control differences are `(1,13,27)*(1-3t)`.
    // The smallest projected carrier is `13x-y`; the residual `-x-2y+z`
    // completes the exact spatial lift.
    let certificate = certify_curve_pair_unique_root_with_config(
        &first,
        full_range(),
        &second,
        partial_range(),
        magnitude_thirteen_config(),
    )
    .unwrap()
    .expect("the explicit magnitude-thirteen shell lifts the normalized one-third root");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert_eq!(certificate.first_range(), full_range());
    assert_eq!(certificate.second_range(), partial_range());
    assert!(certificate.determinant_lower_bound() > 0.0);

    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(NurbsCurvePairBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    let isolation = isolate_curve_pair_candidates_in_scope(
        &first,
        full_range(),
        &second,
        partial_range(),
        0.0,
        0,
        &mut scope,
    )
    .unwrap();
    let [candidate] = isolation.candidates() else {
        panic!("the one-span pair must retain exactly one source-provenanced cell");
    };
    assert!(candidate.certify_unique_root().is_none());
    assert_eq!(
        candidate
            .certify_unique_root_with_config(magnitude_thirteen_config())
            .expect("candidate-cell opt-in uses the same exact source proof"),
        certificate,
    );
}

#[test]
fn coefficient_ceiling_is_validated_and_enforced_as_a_finite_search_limit() {
    let compatibility = CurvePairAlgebraicSearchConfig::default();
    assert_eq!(
        compatibility.maximum_primitive_form_coefficient(),
        CurvePairAlgebraicSearchConfig::compatibility_maximum_primitive_form_coefficient()
    );
    assert_eq!(compatibility.maximum_primitive_form_coefficient(), 12);
    assert_eq!(
        CurvePairAlgebraicSearchConfig::supported_maximum_primitive_form_coefficient(),
        13
    );
    assert_eq!(
        CurvePairAlgebraicSearchConfig::new(6)
            .unwrap()
            .maximum_primitive_form_coefficient(),
        6
    );

    for requested in [0, 5, 14, u8::MAX] {
        let error = CurvePairAlgebraicSearchConfig::new(requested).unwrap_err();
        assert_eq!(error.requested(), requested);
        assert_eq!(error.supported_range(), 6..=13);
        assert!(error.to_string().contains(&requested.to_string()));
    }

    let (first, second) = magnitude_thirteen_pair(None);
    assert!(
        certify_curve_pair_unique_root_with_config(
            &first,
            full_range(),
            &second,
            partial_range(),
            CurvePairAlgebraicSearchConfig::new(6).unwrap(),
        )
        .unwrap()
        .is_none(),
        "a lower configured ceiling must stop before the magnitude-thirteen shell"
    );
}

#[test]
fn magnitude_thirteen_search_is_repeatable_and_symmetric() {
    let (first, second) = magnitude_thirteen_pair(None);
    let certify = || {
        certify_curve_pair_unique_root_with_config(
            &first,
            full_range(),
            &second,
            partial_range(),
            magnitude_thirteen_config(),
        )
        .unwrap()
        .unwrap()
    };
    let expected = certify();
    for _ in 0..4 {
        assert_eq!(certify(), expected);
    }

    let swapped = certify_curve_pair_unique_root_with_config(
        &second,
        partial_range(),
        &first,
        full_range(),
        magnitude_thirteen_config(),
    )
    .unwrap()
    .expect("configured enumeration is stable under operand swap");
    assert_eq!(swapped, expected.swapped());

    let reversed_second = NurbsCurve::new(
        2,
        vec![10.0, 10.0, 10.0, 16.0, 16.0, 16.0],
        second.points().iter().copied().rev().collect(),
        None,
    )
    .unwrap();
    let reversed = certify_curve_pair_unique_root_with_config(
        &first,
        full_range(),
        &reversed_second,
        ParamRange::new(13.0, 14.5),
        magnitude_thirteen_config(),
    )
    .unwrap()
    .expect("configured normalized reversal lifts the t=1/3, s=14 root");
    assert_eq!(reversed.projection_plane(), CurvePairProjectionPlane::Xy);
}

#[test]
fn compatibility_results_are_unchanged_when_the_later_shell_is_enabled() {
    let first = first_curve(None);
    let second = second_curve(12.0, None);
    let compatibility =
        certify_curve_pair_unique_root(&first, full_range(), &second, partial_range())
            .unwrap()
            .unwrap();
    let explicit_twelve = certify_curve_pair_unique_root_with_config(
        &first,
        full_range(),
        &second,
        partial_range(),
        CurvePairAlgebraicSearchConfig::new(12).unwrap(),
    )
    .unwrap()
    .unwrap();
    let extended = certify_curve_pair_unique_root_with_config(
        &first,
        full_range(),
        &second,
        partial_range(),
        magnitude_thirteen_config(),
    )
    .unwrap()
    .unwrap();

    assert_eq!(compatibility, explicit_twelve);
    assert_eq!(compatibility, extended);
}

#[test]
fn configured_thirteen_preserves_broken_weight_and_arithmetic_fail_closed_gates() {
    let (first, second) = magnitude_thirteen_pair(None);

    let mut carrier_points = second.points().to_vec();
    carrier_points[1].y += 0.25;
    carrier_points[1].z += 0.5;
    let broken_carrier = NurbsCurve::new(2, KNOTS.to_vec(), carrier_points, None).unwrap();
    assert!(
        certify_curve_pair_unique_root_with_config(
            &first,
            full_range(),
            &broken_carrier,
            partial_range(),
            magnitude_thirteen_config(),
        )
        .unwrap()
        .is_none()
    );

    let mut residual_points = second.points().to_vec();
    residual_points[1].z += 0.25;
    let broken_residual = NurbsCurve::new(2, KNOTS.to_vec(), residual_points, None).unwrap();
    assert!(
        certify_curve_pair_unique_root_with_config(
            &first,
            full_range(),
            &broken_residual,
            partial_range(),
            magnitude_thirteen_config(),
        )
        .unwrap()
        .is_none()
    );

    let (rational_first, nonproportional) =
        magnitude_thirteen_pair(Some(([1.0, 1.0, 1.0], [2.0, 2.5, 2.0])));
    assert!(
        certify_curve_pair_unique_root_with_config(
            &rational_first,
            partial_range(),
            &nonproportional,
            partial_range(),
            magnitude_thirteen_config(),
        )
        .unwrap()
        .is_none()
    );

    let overflow_first = curve(
        [
            Point3::new(f64::MAX, 0.0, 0.0),
            Point3::new(f64::MAX, 0.0, 1.0),
            Point3::new(f64::MAX, 0.0, 0.0),
        ],
        None,
    );
    let overflow_second = curve(
        [
            Point3::new(f64::MAX, 13.0, 27.0),
            Point3::new(f64::MAX, -6.5, -12.5),
            Point3::new(f64::MAX, -26.0, -54.0),
        ],
        None,
    );
    assert!(
        certify_curve_pair_unique_root_with_config(
            &overflow_first,
            full_range(),
            &overflow_second,
            partial_range(),
            magnitude_thirteen_config(),
        )
        .unwrap()
        .is_none()
    );

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

#[test]
fn proportional_positive_rational_weights_preserve_configured_thirteen_forms() {
    let (first, second) = magnitude_thirteen_pair(Some(([1.0, 1.0, 1.0], [2.0, 2.0, 2.0])));
    assert_no_projected_carrier_through(&first, &second, 12);

    let certificate = certify_curve_pair_unique_root_with_config(
        &first,
        partial_range(),
        &second,
        partial_range(),
        magnitude_thirteen_config(),
    )
    .unwrap()
    .expect("globally proportional rational weights preserve configured forms");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);
}
