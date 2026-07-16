//! Opt-in coverage for the magnitude-fourteen primitive-integer spatial shell.

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

fn magnitude_fourteen_pair(weights: Option<([f64; 3], [f64; 3])>) -> (NurbsCurve, NurbsCurve) {
    let (first_weights, second_weights) =
        weights.map_or((None, None), |(first, second)| (Some(first), Some(second)));
    (
        first_curve(first_weights),
        second_curve(14.0, second_weights),
    )
}

fn full_range() -> ParamRange {
    ParamRange::new(0.0, 1.0)
}

fn partial_range() -> ParamRange {
    ParamRange::new(0.25, 0.5)
}

fn magnitude_fourteen_config() -> CurvePairAlgebraicSearchConfig {
    CurvePairAlgebraicSearchConfig::new(14).unwrap()
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
fn magnitude_fourteen_is_the_first_shell_to_certify_the_normalized_third_root() {
    assert_eq!(
        CurvePairAlgebraicSearchConfig::default().maximum_primitive_form_coefficient(),
        12
    );
    assert_eq!(
        CurvePairAlgebraicSearchConfig::supported_maximum_primitive_form_coefficient(),
        14
    );
    let unsupported = CurvePairAlgebraicSearchConfig::new(15).unwrap_err();
    assert_eq!(unsupported.supported_range(), 6..=14);

    let (first, second) = magnitude_fourteen_pair(None);
    assert_no_projected_carrier_through(&first, &second, 13);
    assert_genuinely_noncoplanar(&first, &second);
    assert!(controls_correspond(&first, &second, [14, -1, 0], false));
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
        "the compatibility entry retains its magnitude-twelve ceiling"
    );
    for ceiling in 6..=13 {
        assert!(
            certify_curve_pair_unique_root_with_config(
                &first,
                full_range(),
                &second,
                partial_range(),
                CurvePairAlgebraicSearchConfig::new(ceiling).unwrap(),
            )
            .unwrap()
            .is_none(),
            "ceiling {ceiling} must stop before the magnitude-fourteen shell"
        );
    }

    // Second-minus-first control differences are `(1,14,29)*(1-3t)`.
    // The smallest projected carrier is `14x-y`; the residual `-x-2y+z`
    // completes the exact spatial lift.
    let certificate = certify_curve_pair_unique_root_with_config(
        &first,
        full_range(),
        &second,
        partial_range(),
        magnitude_fourteen_config(),
    )
    .unwrap()
    .expect("the explicit magnitude-fourteen shell lifts the normalized one-third root");
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
            .certify_unique_root_with_config(magnitude_fourteen_config())
            .expect("candidate-cell opt-in uses the same exact source proof"),
        certificate,
    );
}

#[test]
fn magnitude_fourteen_search_is_repeatable_symmetric_and_reversible() {
    let (first, second) = magnitude_fourteen_pair(None);
    let certify = || {
        certify_curve_pair_unique_root_with_config(
            &first,
            full_range(),
            &second,
            partial_range(),
            magnitude_fourteen_config(),
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
        magnitude_fourteen_config(),
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
        magnitude_fourteen_config(),
    )
    .unwrap()
    .expect("configured normalized reversal lifts the t=1/3, s=14 root");
    assert_eq!(reversed.projection_plane(), CurvePairProjectionPlane::Xy);
}

#[test]
fn magnitude_fourteen_extension_preserves_earlier_certificate_goldens() {
    let first = first_curve(None);

    let twelve = second_curve(12.0, None);
    let compatibility =
        certify_curve_pair_unique_root(&first, full_range(), &twelve, partial_range())
            .unwrap()
            .unwrap();
    let explicit_twelve = certify_curve_pair_unique_root_with_config(
        &first,
        full_range(),
        &twelve,
        partial_range(),
        CurvePairAlgebraicSearchConfig::new(12).unwrap(),
    )
    .unwrap()
    .unwrap();
    let fourteen_over_twelve = certify_curve_pair_unique_root_with_config(
        &first,
        full_range(),
        &twelve,
        partial_range(),
        magnitude_fourteen_config(),
    )
    .unwrap()
    .unwrap();
    assert_eq!(compatibility, explicit_twelve);
    assert_eq!(compatibility, fourteen_over_twelve);

    let thirteen = second_curve(13.0, None);
    assert!(
        certify_curve_pair_unique_root(&first, full_range(), &thirteen, partial_range())
            .unwrap()
            .is_none()
    );
    let explicit_thirteen = certify_curve_pair_unique_root_with_config(
        &first,
        full_range(),
        &thirteen,
        partial_range(),
        CurvePairAlgebraicSearchConfig::new(13).unwrap(),
    )
    .unwrap()
    .unwrap();
    let fourteen_over_thirteen = certify_curve_pair_unique_root_with_config(
        &first,
        full_range(),
        &thirteen,
        partial_range(),
        magnitude_fourteen_config(),
    )
    .unwrap()
    .unwrap();
    assert_eq!(explicit_thirteen, fourteen_over_thirteen);
}

#[test]
fn proportional_positive_rational_weights_preserve_magnitude_fourteen_forms() {
    let (first, second) = magnitude_fourteen_pair(Some(([1.0, 1.0, 1.0], [2.0, 2.0, 2.0])));
    assert_no_projected_carrier_through(&first, &second, 13);

    let certificate = certify_curve_pair_unique_root_with_config(
        &first,
        partial_range(),
        &second,
        partial_range(),
        magnitude_fourteen_config(),
    )
    .unwrap()
    .expect("globally proportional rational weights preserve configured forms");
    assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
    assert!(certificate.determinant_lower_bound() > 0.0);
}

#[test]
fn magnitude_fourteen_forms_fail_closed_for_broken_or_unsafe_inputs() {
    let (first, second) = magnitude_fourteen_pair(None);

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
            magnitude_fourteen_config(),
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
            magnitude_fourteen_config(),
        )
        .unwrap()
        .is_none()
    );

    let (rational_first, nonproportional) =
        magnitude_fourteen_pair(Some(([1.0, 1.0, 1.0], [2.0, 2.5, 2.0])));
    assert!(
        certify_curve_pair_unique_root_with_config(
            &rational_first,
            partial_range(),
            &nonproportional,
            partial_range(),
            magnitude_fourteen_config(),
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
            Point3::new(f64::MAX, 14.0, 29.0),
            Point3::new(f64::MAX, -7.0, -13.5),
            Point3::new(f64::MAX, -28.0, -58.0),
        ],
        None,
    );
    assert!(
        certify_curve_pair_unique_root_with_config(
            &overflow_first,
            full_range(),
            &overflow_second,
            partial_range(),
            magnitude_fourteen_config(),
        )
        .unwrap()
        .is_none()
    );

    assert!(
        certify_curve_pair_unique_root_with_config(
            &first,
            ParamRange::new(-0.25, 0.5),
            &second,
            partial_range(),
            magnitude_fourteen_config(),
        )
        .is_err(),
        "ranges outside the original source domain remain invalid"
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
