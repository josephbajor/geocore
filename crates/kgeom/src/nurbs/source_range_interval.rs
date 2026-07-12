//! Source-representation interval bounds over finite NURBS parameter ranges.

use super::NurbsCurve;
use crate::param::ParamRange;
use kcore::interval::Interval;

/// Enclose one Euclidean position component over a closed source range.
pub(super) fn position_component_interval(
    curve: &NurbsCurve,
    range: ParamRange,
    axis: usize,
) -> Option<Interval> {
    if axis >= 3 || !range.is_finite() || range.width() < 0.0 {
        return None;
    }
    let domain = curve.knots().domain();
    if range.lo < domain.lo || range.hi > domain.hi {
        return None;
    }
    let homogeneous = homogeneous_controls(curve)?;
    let knots = curve.knots().as_slice();
    let degree = curve.degree();
    let last_span = curve.points().len() - 1;
    let mut result = None;
    for span in degree..=last_span {
        if knots[span] >= knots[span + 1] {
            continue;
        }
        let local_lo = range.lo.max(knots[span]);
        let local_hi = range.hi.min(knots[span + 1]);
        if local_lo > local_hi {
            continue;
        }
        let position = interval_de_boor(
            knots,
            degree,
            span,
            Interval::new(local_lo, local_hi),
            &homogeneous,
        )?;
        let component = if curve.weights().is_none() {
            position[axis]
        } else {
            if position[3].lo() <= 0.0 {
                return None;
            }
            position[axis].checked_div(position[3])?
        };
        if !finite(component) {
            return None;
        }
        result = Some(hull(result, component));
    }
    result
}

/// Enclose one Euclidean first-derivative component over a source range.
///
/// The source knot spans are processed directly; no knot insertion or rounded
/// restricted representation participates in the bound.
pub(super) fn derivative_component_interval(
    curve: &NurbsCurve,
    range: ParamRange,
    axis: usize,
) -> Option<Interval> {
    if axis >= 3 || !range.is_finite() || range.width() <= 0.0 {
        return None;
    }
    let domain = curve.knots().domain();
    if range.lo < domain.lo || range.hi > domain.hi {
        return None;
    }

    let degree = curve.degree();
    if degree == 0 {
        return None;
    }
    let homogeneous = homogeneous_controls(curve)?;
    let derivative = homogeneous_derivative_controls(curve, &homogeneous)?;
    let knots = curve.knots().as_slice();
    let derivative_knots = &knots[1..knots.len() - 1];
    let last_span = curve.points().len() - 1;
    let mut result = None;

    for span in degree..=last_span {
        if knots[span] >= knots[span + 1] {
            continue;
        }
        let local_lo = range.lo.max(knots[span]);
        let local_hi = range.hi.min(knots[span + 1]);
        if local_lo > local_hi {
            continue;
        }
        let parameter = Interval::new(local_lo, local_hi);
        let position = interval_de_boor(knots, degree, span, parameter, &homogeneous)?;
        let homogeneous_derivative = interval_de_boor(
            derivative_knots,
            degree - 1,
            span - 1,
            parameter,
            &derivative,
        )?;
        let component = if curve.weights().is_none() {
            homogeneous_derivative[axis]
        } else {
            let weight = position[3];
            if weight.lo() <= 0.0 {
                return None;
            }
            let numerator =
                homogeneous_derivative[axis] * weight - position[axis] * homogeneous_derivative[3];
            numerator.checked_div(weight.square())?
        };
        if !finite(component) {
            return None;
        }
        result = Some(hull(result, component));
    }
    result
}

fn homogeneous_controls(curve: &NurbsCurve) -> Option<Vec<[Interval; 4]>> {
    curve
        .points()
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let weight = curve.weights().map_or(1.0, |weights| weights[index]);
            let weight = Interval::point(weight);
            let control = [
                Interval::point(point.x) * weight,
                Interval::point(point.y) * weight,
                Interval::point(point.z) * weight,
                weight,
            ];
            control.iter().copied().all(finite).then_some(control)
        })
        .collect()
}

fn homogeneous_derivative_controls(
    curve: &NurbsCurve,
    controls: &[[Interval; 4]],
) -> Option<Vec<[Interval; 4]>> {
    let degree = curve.degree();
    let knots = curve.knots().as_slice();
    (0..controls.len().checked_sub(1)?)
        .map(|index| {
            let denominator =
                Interval::point(knots[index + degree + 1]) - Interval::point(knots[index + 1]);
            let scale = Interval::point(degree as f64).checked_div(denominator)?;
            let derivative = core::array::from_fn(|axis| {
                scale * (controls[index + 1][axis] - controls[index][axis])
            });
            derivative.iter().copied().all(finite).then_some(derivative)
        })
        .collect()
}

fn interval_de_boor(
    knots: &[f64],
    degree: usize,
    span: usize,
    parameter: Interval,
    controls: &[[Interval; 4]],
) -> Option<[Interval; 4]> {
    let base = span.checked_sub(degree)?;
    let mut work = controls.get(base..=span)?.to_vec();
    for level in 1..=degree {
        for local in (level..=degree).rev() {
            let control_index = base + local;
            let denominator = Interval::point(knots[control_index + degree - level + 1])
                - Interval::point(knots[control_index]);
            let alpha =
                (parameter - Interval::point(knots[control_index])).checked_div(denominator)?;
            // On this fixed nonempty knot span, the exact de Boor alpha lies
            // in [0, 1]. Intersect away only outward-rounding spill so the
            // blend keeps nonnegative convex coefficients.
            let alpha_lo = alpha.lo().max(0.0);
            let alpha_hi = alpha.hi().min(1.0);
            if alpha_lo > alpha_hi {
                return None;
            }
            let alpha = Interval::new(alpha_lo, alpha_hi);
            let blended = core::array::from_fn(|axis| {
                interval_blend(work[local - 1][axis], work[local][axis], alpha)
            });
            if !blended.iter().copied().all(finite) {
                return None;
            }
            work[local] = blended;
        }
    }
    work.get(degree).copied()
}

fn interval_blend(first: Interval, second: Interval, alpha: Interval) -> Interval {
    // For independent first/second interval boxes, each extremum uses the
    // matching control endpoints. The resulting scalar is affine in alpha,
    // so its extrema occur at alpha's two endpoints. Evaluating those two
    // correlated blends avoids the dependency loss from `(1-A)X + AY` while
    // retaining every combination in the interval box.
    let blend_at = |parameter| {
        let parameter = Interval::point(parameter);
        (Interval::point(1.0) - parameter) * first + parameter * second
    };
    let low = blend_at(alpha.lo());
    let high = blend_at(alpha.hi());
    Interval::new(low.lo().min(high.lo()), low.hi().max(high.hi()))
}

fn hull(current: Option<Interval>, next: Interval) -> Interval {
    current.map_or(next, |current| {
        Interval::new(current.lo().min(next.lo()), current.hi().max(next.hi()))
    })
}

fn finite(interval: Interval) -> bool {
    interval.lo().is_finite() && interval.hi().is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::Curve;
    use crate::vec::Point3;

    fn assert_derivatives_enclosed(curve: &NurbsCurve, range: ParamRange) {
        let bounds = [
            derivative_component_interval(curve, range, 0).unwrap(),
            derivative_component_interval(curve, range, 1).unwrap(),
            derivative_component_interval(curve, range, 2).unwrap(),
        ];
        for sample in 0..=512 {
            let parameter = range.lerp(f64::from(sample) / 512.0);
            let point = curve.eval(parameter);
            let derivative = curve.eval_derivs(parameter, 1).d[1];
            for (axis, value) in [point.x, point.y, point.z].into_iter().enumerate() {
                let bound = position_component_interval(curve, range, axis).unwrap();
                assert!(bound.contains(value));
            }
            for (bound, value) in bounds
                .iter()
                .zip([derivative.x, derivative.y, derivative.z])
            {
                assert!(
                    bound.contains(value),
                    "parameter={parameter}, value={value}, bound={bound:?}"
                );
            }
        }
    }

    #[test]
    fn polynomial_bounds_enclose_partial_bezier_ranges() {
        let curve = NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-3.0, 1.0, 0.0),
                Point3::new(-1.0, 4.0, 2.0),
                Point3::new(2.0, -2.0, -1.0),
                Point3::new(5.0, 3.0, 1.0),
            ],
            None,
        )
        .unwrap();
        assert_derivatives_enclosed(&curve, ParamRange::new(0.1875, 0.6875));
    }

    #[test]
    fn rational_bounds_enclose_nonuniform_weight_ranges() {
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-2.0, -1.0, 0.5),
                Point3::new(0.5, 3.0, -2.0),
                Point3::new(4.0, 0.25, 1.5),
            ],
            Some(vec![0.75, 2.0, 1.25]),
        )
        .unwrap();
        assert_derivatives_enclosed(&curve, ParamRange::new(0.125, 0.875));
    }

    #[test]
    fn closed_ranges_include_both_sides_of_a_repeated_knot() {
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-2.0, 0.0, 1.0),
                Point3::new(-1.0, 2.0, -1.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, -3.0, 2.0),
                Point3::new(2.0, 0.0, -2.0),
            ],
            Some(vec![1.0, 1.25, 0.75, 1.5, 1.0]),
        )
        .unwrap();
        assert_derivatives_enclosed(&curve, ParamRange::new(0.0, 0.5));
        assert_derivatives_enclosed(&curve, ParamRange::new(0.5, 1.0));
        assert_derivatives_enclosed(&curve, ParamRange::new(0.375, 0.625));
    }

    #[test]
    fn zero_containing_denominators_and_nonfinite_homogeneous_controls_fail_closed() {
        let tiny = f64::from_bits(1);
        let tiny_domain = NurbsCurve::new(
            1,
            vec![0.0, 0.0, tiny, tiny],
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)],
            None,
        )
        .unwrap();
        assert!(
            derivative_component_interval(&tiny_domain, tiny_domain.param_range(), 0).is_none()
        );

        let overflow = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                Point3::new(f64::MAX, 0.0, 0.0),
                Point3::new(f64::MAX, 1.0, 0.0),
            ],
            Some(vec![2.0, 2.0]),
        )
        .unwrap();
        assert!(position_component_interval(&overflow, overflow.param_range(), 0).is_none());
        assert!(derivative_component_interval(&overflow, overflow.param_range(), 0).is_none());
    }
}
