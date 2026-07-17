//! Table-driven coverage for primitive-integer spatial-root lifts across the
//! canonical coefficient magnitudes three..=fourteen.
//!
//! This target replaces the former template-stamped
//! `spatial_algebraic_primitive_integer_magnitude_{three..fourteen}` family
//! (12 files). One row per magnitude drives the shared certify and fail-closed
//! logic. Magnitudes at or below the default coefficient ceiling (twelve)
//! certify through the compatibility entry point; magnitudes above it opt in
//! through an explicit `CurvePairAlgebraicSearchConfig`.
//!
//! Wall-time budget (ORCHESTRATION R7): < 10 s in the `standard` lane. This is
//! pure exact algebra with no fixture reconstruction, well under the 60 s
//! corpus-ratchet threshold, so no corpus justification is required.

use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
use kcore::predicates::{Orientation, orient3d};
use kcore::tolerance::Tolerances;
use kgeom::nurbs::{
    CurvePairAlgebraicSearchConfig, CurvePairProjectionPlane, CurvePairRootCertificate, NurbsCurve,
    NurbsCurvePairBudgetProfile, certify_curve_pair_unique_root,
    certify_curve_pair_unique_root_with_config, isolate_curve_pair_candidates_in_scope,
};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

/// The canonical coefficient magnitudes exercised by the former per-magnitude
/// files, in ascending order. `three`..=`twelve` certify under the default
/// search ceiling; `thirteen` and `fourteen` require an explicit opt-in.
const MAGNITUDES: [u8; 12] = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14];

/// Inclusive coefficient magnitude reached by the default (compatibility)
/// algebraic search. Magnitudes at or below this certify without an opt-in.
const DEFAULT_CEILING: u8 = 12;

/// First magnitude whose broken-carrier fixture also perturbs `z` (magnitudes
/// three and four perturbed only `y` in the originals).
const FIRST_Z_PERTURBED_MAGNITUDE: u8 = 5;

/// First magnitude whose fail-closed coverage includes an overflowing source
/// pair (added at magnitude eight in the originals, present through fourteen).
const FIRST_OVERFLOW_MAGNITUDE: u8 = 8;

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

/// The magnitude-`coefficient` carrier. The second-minus-first control
/// differences are the primitive integer form `(1, c, 2c+1) * (1 - 3t)`, so the
/// smallest projected carrier is `c*x - y` and the residual is `-x - 2y + z`.
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

fn pair(magnitude: u8, weights: Option<([f64; 3], [f64; 3])>) -> (NurbsCurve, NurbsCurve) {
    let (first_weights, second_weights) =
        weights.map_or((None, None), |(first, second)| (Some(first), Some(second)));
    (
        first_curve(first_weights),
        second_curve(f64::from(magnitude), second_weights),
    )
}

fn full_range() -> ParamRange {
    ParamRange::new(0.0, 1.0)
}

fn partial_range() -> ParamRange {
    ParamRange::new(0.25, 0.5)
}

fn requires_config(magnitude: u8) -> bool {
    magnitude > DEFAULT_CEILING
}

fn config_for(magnitude: u8) -> CurvePairAlgebraicSearchConfig {
    CurvePairAlgebraicSearchConfig::new(magnitude).unwrap()
}

/// Certify a curve pair using the search path a given magnitude needs: the
/// compatibility entry point for magnitudes within the default ceiling, or the
/// explicit configured shell for magnitudes above it.
fn certify_for(
    magnitude: u8,
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
) -> Option<CurvePairRootCertificate> {
    if requires_config(magnitude) {
        certify_curve_pair_unique_root_with_config(
            first,
            first_range,
            second,
            second_range,
            config_for(magnitude),
        )
        .unwrap()
    } else {
        certify_curve_pair_unique_root(first, first_range, second, second_range).unwrap()
    }
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

/// Assert that no coordinate or two-axis primitive form bounded in magnitude by
/// `bound` corresponds between the two control polygons, in either orientation.
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

/// Every magnitude's primitive carrier lifts the normalized one-third root of
/// the partial-range family, projecting onto Xy with a positive determinant
/// lower bound. Magnitudes above the default ceiling are rejected by the
/// compatibility entry point and by every lower configured ceiling, then lifted
/// by their explicit shell — including through the isolation candidate cell.
#[test]
fn each_magnitude_carrier_certifies_the_normalized_third_family() {
    for &magnitude in &MAGNITUDES {
        let (first, second) = pair(magnitude, None);

        assert_no_projected_carrier_through(&first, &second, (magnitude - 1) as i8);
        assert_genuinely_noncoplanar(&first, &second);
        assert!(
            controls_correspond(&first, &second, [magnitude as i8, -1, 0], false),
            "magnitude {magnitude}: the smallest projected carrier {magnitude}x-y must correspond"
        );
        assert!(controls_correspond(&first, &second, [-1, -2, 1], false));

        let root = 1.0 / 3.0;
        assert!(partial_range().lo < root && root < partial_range().hi);
        assert_ne!(root, partial_range().lo);
        assert_ne!(root, partial_range().lerp(0.5));
        assert_ne!(root, partial_range().hi);
        assert!(!first.knots().as_slice().contains(&root));

        if requires_config(magnitude) {
            assert!(
                certify_curve_pair_unique_root(&first, full_range(), &second, partial_range())
                    .unwrap()
                    .is_none(),
                "magnitude {magnitude}: the compatibility entry retains its magnitude-twelve ceiling"
            );
            for ceiling in 6..magnitude {
                assert!(
                    certify_curve_pair_unique_root_with_config(
                        &first,
                        full_range(),
                        &second,
                        partial_range(),
                        config_for(ceiling),
                    )
                    .unwrap()
                    .is_none(),
                    "magnitude {magnitude}: ceiling {ceiling} must stop before the shell"
                );
            }
        }

        let certificate = certify_for(magnitude, &first, full_range(), &second, partial_range())
            .unwrap_or_else(|| {
                panic!("magnitude {magnitude}: the carrier must lift the normalized one-third root")
            });
        assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
        assert_eq!(certificate.first_range(), full_range());
        assert_eq!(certificate.second_range(), partial_range());
        assert!(certificate.determinant_lower_bound() > 0.0);

        if requires_config(magnitude) {
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
                panic!(
                    "magnitude {magnitude}: the one-span pair must retain exactly one \
                     source-provenanced cell"
                );
            };
            assert!(candidate.certify_unique_root().is_none());
            assert_eq!(
                candidate
                    .certify_unique_root_with_config(config_for(magnitude))
                    .expect("candidate-cell opt-in uses the same exact source proof"),
                certificate,
            );
        }
    }
}

/// Every magnitude's enumeration is stable under operand swap and lifts the
/// normalized affine reversal. Configured magnitudes additionally certify
/// deterministically and symmetrically (repeat runs and the swapped certificate
/// agree exactly).
#[test]
fn each_magnitude_supports_operand_swap_and_affine_reversal() {
    for &magnitude in &MAGNITUDES {
        let (first, second) = pair(magnitude, None);

        if requires_config(magnitude) {
            let expected = certify_for(magnitude, &first, full_range(), &second, partial_range())
                .expect("configured magnitude certifies the source pair");
            for _ in 0..4 {
                let repeated =
                    certify_for(magnitude, &first, full_range(), &second, partial_range())
                        .expect("configured search is repeatable");
                assert_eq!(repeated, expected);
            }
            let swapped = certify_for(magnitude, &second, partial_range(), &first, full_range())
                .expect("configured enumeration is stable under operand swap");
            assert_eq!(swapped, expected.swapped());
        } else {
            let swapped = certify_for(magnitude, &second, partial_range(), &first, full_range())
                .expect("primitive form enumeration is stable under operand swap");
            assert_eq!(swapped.projection_plane(), CurvePairProjectionPlane::Xy);
        }

        let reversed_second = NurbsCurve::new(
            2,
            vec![10.0, 10.0, 10.0, 16.0, 16.0, 16.0],
            second.points().iter().copied().rev().collect(),
            None,
        )
        .unwrap();
        let reversed = certify_for(
            magnitude,
            &first,
            full_range(),
            &reversed_second,
            ParamRange::new(13.0, 14.5),
        )
        .expect("normalized reversal lifts the t=1/3, s=14 root");
        assert_eq!(reversed.projection_plane(), CurvePairProjectionPlane::Xy);
    }
}

/// Globally proportional positive rational weights preserve every magnitude's
/// primitive forms and its positive determinant lower bound.
#[test]
fn each_magnitude_preserves_proportional_positive_rational_weights() {
    for &magnitude in &MAGNITUDES {
        let (first, second) = pair(magnitude, Some(([1.0, 1.0, 1.0], [2.0, 2.0, 2.0])));
        assert_no_projected_carrier_through(&first, &second, (magnitude - 1) as i8);

        let certificate = certify_for(magnitude, &first, partial_range(), &second, partial_range())
            .expect("globally proportional rational weights preserve the primitive forms");
        assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
        assert!(certificate.determinant_lower_bound() > 0.0);
    }
}

/// Every magnitude fails closed for a broken carrier, a broken residual,
/// nonproportional weights, and (from magnitude eight) overflowing sources.
/// Magnitude fourteen additionally rejects a parameter range outside the
/// original source domain.
#[test]
fn each_magnitude_fails_closed_for_broken_forms_and_unsafe_inputs() {
    for &magnitude in &MAGNITUDES {
        let coefficient = f64::from(magnitude);
        let (first, second) = pair(magnitude, None);

        let mut carrier_points = second.points().to_vec();
        carrier_points[1].y += 0.25;
        if magnitude >= FIRST_Z_PERTURBED_MAGNITUDE {
            carrier_points[1].z += 0.5;
        }
        let broken_carrier = NurbsCurve::new(2, KNOTS.to_vec(), carrier_points, None).unwrap();
        assert!(
            certify_for(
                magnitude,
                &first,
                full_range(),
                &broken_carrier,
                partial_range()
            )
            .is_none(),
            "magnitude {magnitude}: a broken carrier must fail closed"
        );

        let mut residual_points = second.points().to_vec();
        residual_points[1].z += 0.25;
        let broken_residual = NurbsCurve::new(2, KNOTS.to_vec(), residual_points, None).unwrap();
        assert!(
            certify_for(
                magnitude,
                &first,
                full_range(),
                &broken_residual,
                partial_range()
            )
            .is_none(),
            "magnitude {magnitude}: a broken residual must fail closed"
        );

        let (rational_first, nonproportional) =
            pair(magnitude, Some(([1.0, 1.0, 1.0], [2.0, 2.5, 2.0])));
        assert!(
            certify_for(
                magnitude,
                &rational_first,
                partial_range(),
                &nonproportional,
                partial_range(),
            )
            .is_none(),
            "magnitude {magnitude}: nonproportional weights must fail closed"
        );

        if magnitude >= FIRST_OVERFLOW_MAGNITUDE {
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
                    Point3::new(f64::MAX, coefficient, 2.0 * coefficient + 1.0),
                    Point3::new(f64::MAX, -coefficient / 2.0, -coefficient + 0.5),
                    Point3::new(f64::MAX, -2.0 * coefficient, -4.0 * coefficient - 2.0),
                ],
                None,
            );
            assert!(
                certify_for(
                    magnitude,
                    &overflow_first,
                    full_range(),
                    &overflow_second,
                    partial_range(),
                )
                .is_none(),
                "magnitude {magnitude}: overflowing sources must fail closed"
            );
        }

        if magnitude == 14 {
            assert!(
                certify_curve_pair_unique_root_with_config(
                    &first,
                    ParamRange::new(-0.25, 0.5),
                    &second,
                    partial_range(),
                    config_for(magnitude),
                )
                .is_err(),
                "ranges outside the original source domain remain invalid"
            );
        }
    }
}

/// Nonfinite source control points are rejected at construction, independent of
/// any magnitude configuration.
#[test]
fn nonfinite_source_control_points_are_rejected() {
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

/// The configured coefficient ceiling is validated and enforced as a finite
/// search limit: the compatibility default is twelve, the supported maximum is
/// fourteen, out-of-range requests fail with the reviewed range, and a ceiling
/// below a pair's magnitude stops before its shell.
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
        14
    );
    assert_eq!(
        CurvePairAlgebraicSearchConfig::new(6)
            .unwrap()
            .maximum_primitive_form_coefficient(),
        6
    );

    for requested in [0, 5, 15, u8::MAX] {
        let error = CurvePairAlgebraicSearchConfig::new(requested).unwrap_err();
        assert_eq!(error.requested(), requested);
        assert_eq!(error.supported_range(), 6..=14);
        assert!(error.to_string().contains(&requested.to_string()));
    }

    let (first, second) = pair(13, None);
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

/// Raising the search ceiling never changes an already-certifiable certificate.
/// A magnitude-twelve pair certifies identically under the compatibility entry
/// and under any higher explicit ceiling; a magnitude-thirteen pair is rejected
/// by the compatibility entry but agrees across the thirteen and fourteen
/// shells.
#[test]
fn raising_the_search_ceiling_preserves_earlier_certificate_goldens() {
    let first = first_curve(None);

    let twelve = second_curve(12.0, None);
    let compatibility =
        certify_curve_pair_unique_root(&first, full_range(), &twelve, partial_range())
            .unwrap()
            .unwrap();
    for ceiling in [12, 13, 14] {
        let raised = certify_curve_pair_unique_root_with_config(
            &first,
            full_range(),
            &twelve,
            partial_range(),
            CurvePairAlgebraicSearchConfig::new(ceiling).unwrap(),
        )
        .unwrap()
        .unwrap();
        assert_eq!(
            compatibility, raised,
            "ceiling {ceiling} must reproduce the magnitude-twelve golden"
        );
    }

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
        CurvePairAlgebraicSearchConfig::new(14).unwrap(),
    )
    .unwrap()
    .unwrap();
    assert_eq!(explicit_thirteen, fourteen_over_thirteen);
}
