//! Shared numerical-policy adapters for intersection solvers.

use kcore::operation::{NumericGuardKind, NumericalPolicy, ParameterScale};
use kgeom::vec::Vec3;

/// Returns whether both Newton directional gradients are numerically zero.
///
/// Each residual/derivative dot product is normalized by the two vector
/// magnitudes before comparison with the policy's dimensionless coefficient-
/// cancellation guard. This decision may stop polishing, but cannot accept a
/// contact or prove a miss. A zero vector contributes an exact zero gradient;
/// non-finite vectors stop conservatively.
pub(super) fn directional_gradients_are_numerically_zero(
    policy: NumericalPolicy,
    residual: Vec3,
    derivative_a: Vec3,
    derivative_b: Vec3,
) -> bool {
    let Some(a) = normalized_abs_dot(residual, derivative_a) else {
        return true;
    };
    let Some(b) = normalized_abs_dot(residual, derivative_b) else {
        return true;
    };
    let threshold = policy.rounding_guard(NumericGuardKind::CoefficientCancellation, 1.0);
    a.max(b) <= threshold
}

fn normalized_abs_dot(a: Vec3, b: Vec3) -> Option<f64> {
    let scale_a = a.x.abs().max(a.y.abs()).max(a.z.abs());
    let scale_b = b.x.abs().max(b.y.abs()).max(b.z.abs());
    if !scale_a.is_finite() || !scale_b.is_finite() {
        return None;
    }
    if scale_a == 0.0 || scale_b == 0.0 {
        return Some(0.0);
    }
    let a = a / scale_a;
    let b = b / scale_b;
    let denominator = (a.dot(a) * b.dot(b)).sqrt();
    let value = a.dot(b).abs() / denominator;
    value.is_finite().then_some(value.min(1.0))
}

/// Returns the sine of the angle between two non-zero finite directions.
///
/// Component-wise scaling keeps the cross product and magnitude calculation
/// away from overflow and underflow. `None` identifies a singular or invalid
/// direction pair; callers must not use this arithmetic guard to accept a
/// geometric candidate.
pub(super) fn normalized_cross_magnitude(a: Vec3, b: Vec3) -> Option<f64> {
    let scale_a = a.x.abs().max(a.y.abs()).max(a.z.abs());
    let scale_b = b.x.abs().max(b.y.abs()).max(b.z.abs());
    if !scale_a.is_finite() || !scale_b.is_finite() || scale_a == 0.0 || scale_b == 0.0 {
        return None;
    }
    let a = a / scale_a;
    let b = b / scale_b;
    let denominator = (a.dot(a) * b.dot(b)).sqrt();
    let value = a.cross(b).norm() / denominator;
    value.is_finite().then_some(value.min(1.0))
}

/// Compares finite non-negative objective values on a dimensionless scale.
///
/// This only controls an iterative search branch. It cannot accept a contact
/// or prove a miss. Invalid or negative objectives return `None` so callers can
/// stop conservatively.
pub(super) fn nonnegative_values_are_numerically_equal(
    policy: NumericalPolicy,
    a: f64,
    b: f64,
) -> Option<bool> {
    if !a.is_finite() || !b.is_finite() || a < 0.0 || b < 0.0 {
        return None;
    }
    let scale = a.max(b);
    if scale == 0.0 {
        return Some(true);
    }
    let difference = (a / scale - b / scale).abs();
    Some(difference <= policy.rounding_guard(NumericGuardKind::CoefficientCancellation, 1.0))
}

/// Returns whether a ternary-search interval cannot make useful progress.
///
/// The active width is measured in coordinates normalized by the original
/// search span. Exact comparisons with the proposed interior points catch
/// affine-offset cases where floating-point arithmetic cannot represent a
/// smaller interval. This is a termination guard only.
pub(super) fn ternary_interval_has_no_progress(
    policy: NumericalPolicy,
    original_span: f64,
    lo: f64,
    left: f64,
    right: f64,
    hi: f64,
) -> bool {
    if !original_span.is_finite()
        || original_span <= 0.0
        || !lo.is_finite()
        || !left.is_finite()
        || !right.is_finite()
        || !hi.is_finite()
        || !(lo < left && left < right && right < hi)
    {
        return true;
    }
    let normalized_width = (hi - lo) / original_span;
    if !normalized_width.is_finite() || normalized_width <= 0.0 {
        return true;
    }
    let Some(threshold) = parameter_progress_step(policy, 1.0, 1.0, 1.0) else {
        return true;
    };
    normalized_width <= threshold
}

/// Derives a scale-aware parameter-progress step with no acceptance authority.
///
/// Callers must still check their model-space residual before accepting a
/// candidate. An invalid scale returns `None` so iterative callers can stop
/// conservatively without panicking.
pub(super) fn parameter_progress_step(
    policy: NumericalPolicy,
    coordinate_magnitude: f64,
    span: f64,
    output_tolerance: f64,
) -> Option<f64> {
    policy
        .parameter_tolerance(
            ParameterScale {
                coordinate_magnitude,
                span,
                output_rate_upper: None,
            },
            output_tolerance,
        )
        .ok()
        .map(|tolerance| tolerance.termination_step)
}

/// Solves a symmetric 2×2 system after a scale-invariant conditioning check.
///
/// The policy decision uses an infinity-norm reciprocal-condition estimate.
/// When the direct arithmetic is finite, it is deliberately retained so the
/// v1 migration does not perturb established result bits. Normalized
/// arithmetic is only a fallback for coefficient scales that overflow or
/// underflow the direct determinant calculation.
pub(super) fn solve_symmetric_2x2(
    policy: NumericalPolicy,
    a00: f64,
    a01: f64,
    a11: f64,
    rhs0: f64,
    rhs1: f64,
) -> Option<(f64, f64)> {
    let coefficient_scale = a00.abs().max(a01.abs()).max(a11.abs());
    if !coefficient_scale.is_finite() || coefficient_scale == 0.0 {
        return None;
    }

    let n00 = a00 / coefficient_scale;
    let n01 = a01 / coefficient_scale;
    let n11 = a11 / coefficient_scale;
    let normalized_determinant = n00 * n11 - n01 * n01;
    let norm = (n00.abs() + n01.abs()).max(n01.abs() + n11.abs());
    let reciprocal_condition = normalized_determinant.abs() / (norm * norm);
    if !policy.reciprocal_condition_is_usable(reciprocal_condition) {
        return None;
    }

    let normalized_rhs0 = rhs0 / coefficient_scale;
    let normalized_rhs1 = rhs1 / coefficient_scale;
    let normalized = (
        (normalized_rhs0 * n11 - n01 * normalized_rhs1) / normalized_determinant,
        (n00 * normalized_rhs1 - n01 * normalized_rhs0) / normalized_determinant,
    );
    if !normalized.0.is_finite() || !normalized.1.is_finite() {
        return None;
    }

    let determinant = a00 * a11 - a01 * a01;
    let numerator0 = rhs0 * a11 - a01 * rhs1;
    let numerator1 = a00 * rhs1 - a01 * rhs0;
    let direct = (numerator0 / determinant, numerator1 / determinant);
    let direct_preserves_nonzero =
        (numerator0 != 0.0 || normalized.0 == 0.0) && (numerator1 != 0.0 || normalized.1 == 0.0);
    if determinant.is_finite()
        && determinant != 0.0
        && numerator0.is_finite()
        && numerator1.is_finite()
        && direct_preserves_nonzero
        && direct.0.is_finite()
        && direct.1.is_finite()
    {
        Some(direct)
    } else {
        Some(normalized)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalized_gradient_stop_is_invariant_across_vector_scales() {
        let policy = NumericalPolicy::v1();
        for scale in [1.0e-200, 1.0, 1.0e200] {
            let residual = Vec3::new(scale, 0.0, 0.0);
            let orthogonal = Vec3::new(0.0, scale, 0.0);
            let aligned = Vec3::new(scale, scale, 0.0);
            assert!(directional_gradients_are_numerically_zero(
                policy, residual, orthogonal, orthogonal
            ));
            assert!(!directional_gradients_are_numerically_zero(
                policy, residual, aligned, orthogonal
            ));
        }
    }

    #[test]
    fn normalized_gradient_stop_honors_policy_and_rejects_invalid_vectors() {
        let residual = Vec3::new(1.0, 0.0, 0.0);
        let derivative = Vec3::new(1.0, 2.0, 0.0);
        assert!(!directional_gradients_are_numerically_zero(
            NumericalPolicy::v1(),
            residual,
            derivative,
            derivative,
        ));
        assert!(!directional_gradients_are_numerically_zero(
            NumericalPolicy::v1(),
            residual,
            Vec3::default(),
            derivative,
        ));

        let coarse = NumericalPolicy::try_new(1.0e16, 64.0, 128.0 * f64::EPSILON).unwrap();
        assert!(directional_gradients_are_numerically_zero(
            coarse, residual, derivative, derivative,
        ));
        assert!(directional_gradients_are_numerically_zero(
            NumericalPolicy::v1(),
            Vec3::new(f64::NAN, 0.0, 0.0),
            derivative,
            derivative,
        ));
    }

    #[test]
    fn normalized_cross_magnitude_is_invariant_across_extreme_vector_scales() {
        for scale in [1.0e-200, 1.0, 1.0e200] {
            assert_eq!(
                normalized_cross_magnitude(Vec3::new(scale, 0.0, 0.0), Vec3::new(0.0, scale, 0.0),),
                Some(1.0)
            );
            assert_eq!(
                normalized_cross_magnitude(Vec3::new(scale, 0.0, 0.0), Vec3::new(-scale, 0.0, 0.0),),
                Some(0.0)
            );
        }
        assert_eq!(
            normalized_cross_magnitude(Vec3::default(), Vec3::new(1.0, 0.0, 0.0)),
            None
        );
        assert_eq!(
            normalized_cross_magnitude(
                Vec3::new(f64::INFINITY, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            ),
            None
        );
    }

    #[test]
    fn objective_comparison_is_relative_and_policy_controlled() {
        let policy = NumericalPolicy::v1();
        for scale in [1.0e-200, 1.0, 1.0e200] {
            assert_eq!(
                nonnegative_values_are_numerically_equal(
                    policy,
                    scale,
                    scale * (1.0 + 8.0 * f64::EPSILON),
                ),
                Some(true)
            );
            assert_eq!(
                nonnegative_values_are_numerically_equal(
                    policy,
                    scale,
                    scale * (1.0 + 256.0 * f64::EPSILON),
                ),
                Some(false)
            );
        }
        assert_eq!(
            nonnegative_values_are_numerically_equal(policy, 0.0, 0.0),
            Some(true)
        );
        assert_eq!(
            nonnegative_values_are_numerically_equal(policy, -1.0, 1.0),
            None
        );
    }

    #[test]
    fn ternary_progress_is_relative_and_detects_unrepresentable_bounds() {
        let policy = NumericalPolicy::v1();
        for scale in [1.0e-13, 1.0, 1.0e13] {
            assert!(!ternary_interval_has_no_progress(
                policy,
                scale,
                0.0,
                scale / 3.0,
                scale * 2.0 / 3.0,
                scale,
            ));
        }
        let coarse = NumericalPolicy::try_new(32.0, 1.0e16, 128.0 * f64::EPSILON).unwrap();
        assert!(ternary_interval_has_no_progress(
            coarse,
            1.0,
            0.0,
            1.0 / 3.0,
            2.0 / 3.0,
            1.0,
        ));

        let lo = 1.0e16;
        let hi = lo + 2.0;
        assert!(ternary_interval_has_no_progress(
            policy, 2.0, lo, lo, hi, hi,
        ));
    }

    #[test]
    fn parameter_progress_step_tracks_small_and_large_parameter_scales() {
        let policy = NumericalPolicy::v1();
        let base = f64::EPSILON * 64.0;
        for (coordinate, span, expected) in [
            (5.0e-14, 1.0e-13, base),
            (0.5, 1.0, base),
            (5.0e12, 1.0e13, base * 1.0e13),
        ] {
            let step = parameter_progress_step(policy, coordinate, span, 1.0e-8).unwrap();
            assert!((step / expected - 1.0).abs() <= f64::EPSILON);
        }
    }

    #[test]
    fn parameter_progress_step_honors_custom_policy_and_rejects_invalid_scales() {
        let v1 = parameter_progress_step(NumericalPolicy::v1(), 0.5, 1.0, 1.0e-8).unwrap();
        let custom = NumericalPolicy::try_new(32.0, 640.0, 128.0 * f64::EPSILON).unwrap();
        let custom = parameter_progress_step(custom, 0.5, 1.0, 1.0e-8).unwrap();
        assert!((custom / v1 - 10.0).abs() <= 4.0 * f64::EPSILON);
        assert_eq!(
            parameter_progress_step(NumericalPolicy::v1(), 0.0, 0.0, 1.0e-8),
            None
        );
    }

    #[test]
    fn solve_decision_and_result_are_invariant_across_coefficient_scale() {
        for scale in [1.0e-200, 1.0, 1.0e200] {
            let solved = solve_symmetric_2x2(
                NumericalPolicy::v1(),
                2.0 * scale,
                0.0,
                scale,
                4.0 * scale,
                3.0 * scale,
            )
            .unwrap();
            assert!((solved.0 - 2.0).abs() <= 4.0 * f64::EPSILON);
            assert!((solved.1 - 3.0).abs() <= 4.0 * f64::EPSILON);
        }
    }

    #[test]
    fn normalized_ill_conditioning_is_rejected_at_every_scale() {
        for scale in [1.0e-100, 1.0, 1.0e100] {
            assert_eq!(
                solve_symmetric_2x2(
                    NumericalPolicy::v1(),
                    scale,
                    scale,
                    scale * (1.0 + 8.0 * f64::EPSILON),
                    scale,
                    scale,
                ),
                None
            );
        }
    }

    #[test]
    fn normalized_fallback_avoids_false_finite_zero_from_extreme_products() {
        let overflow =
            solve_symmetric_2x2(NumericalPolicy::v1(), 1.0e200, 0.0, 2.0e200, 1.0, 2.0).unwrap();
        assert!((overflow.0 / 1.0e-200 - 1.0).abs() <= 4.0 * f64::EPSILON);
        assert!((overflow.1 / 1.0e-200 - 1.0).abs() <= 4.0 * f64::EPSILON);

        let underflow = solve_symmetric_2x2(
            NumericalPolicy::v1(),
            1.0e-160,
            0.0,
            2.0e-160,
            1.0e-320,
            2.0e-320,
        )
        .unwrap();
        assert!(underflow.0 > 0.0 && underflow.1 > 0.0);
        assert!((underflow.0 / 1.0e-160 - 1.0).abs() < 1.0e-3);
        assert!((underflow.1 / 1.0e-160 - 1.0).abs() < 1.0e-3);
    }

    #[test]
    fn symmetric_rhs_signs_match_the_exact_solution() {
        let solved = solve_symmetric_2x2(NumericalPolicy::v1(), 4.0, 1.0, 3.0, 1.0, 2.0).unwrap();
        assert!((solved.0 - 1.0 / 11.0).abs() <= 4.0 * f64::EPSILON);
        assert!((solved.1 - 7.0 / 11.0).abs() <= 4.0 * f64::EPSILON);
    }

    #[test]
    fn validated_policy_floor_controls_the_conditioning_decision() {
        let coefficients = (1.0, 0.0, 0.25, 1.0, 0.25);
        assert!(
            solve_symmetric_2x2(
                NumericalPolicy::v1(),
                coefficients.0,
                coefficients.1,
                coefficients.2,
                coefficients.3,
                coefficients.4,
            )
            .is_some()
        );
        let strict = NumericalPolicy::try_new(32.0, 64.0, 0.5).unwrap();
        assert_eq!(
            solve_symmetric_2x2(
                strict,
                coefficients.0,
                coefficients.1,
                coefficients.2,
                coefficients.3,
                coefficients.4,
            ),
            None
        );
    }
}
