//! Source-representation interval bounds over finite NURBS parameter ranges.

use super::NurbsCurve;
use crate::aabb::Aabb3;
use crate::param::ParamRange;
use crate::vec::{Point3, Vec3};
use kcore::interval::Interval;
use kcore::predicates::{Orientation, affine_dot3};

const MAX_LINEAR_FORM_COEFFICIENT: i8 = 14;

/// Certified relation of an original NURBS curve range to a plane tolerance
/// slab.
///
/// The represented affine slab is
/// `|normal · (point - origin)| <= half_width`. For a unit normal,
/// `half_width` therefore has model-space distance units. For any other finite
/// nonzero normal it is measured in the correspondingly scaled dot-product
/// units. Scaling `normal` by `s` preserves the represented slab only when
/// `half_width` is also scaled by `|s|`; a negative `s` swaps the
/// [`Self::Negative`] and [`Self::Positive`] labels. [`Self::Candidate`] is
/// deliberately conservative: the curve may meet the slab, or finite
/// arithmetic may have been inconclusive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneCurveRangeRelation {
    /// The complete source range lies strictly below the negative slab boundary.
    Negative,
    /// The source range may meet the slab, or arithmetic was inconclusive.
    Candidate,
    /// The complete source range lies in the closed tolerance slab.
    WithinSlab,
    /// The complete source range lies strictly above the positive slab boundary.
    Positive,
}

/// Classify an original NURBS curve range against a plane tolerance slab.
///
/// The exact contract is the affine slab
/// `|normal · (point - origin)| <= half_width`; callers that require physical
/// distance semantics supply a unit normal. Arbitrary finite nonzero normals
/// are accepted without rounded unit-length validation and scale the slab
/// coordinate directly. The pair `(normal, half_width)` defines the slab:
/// replacing it by `(s * normal, |s| * half_width)` preserves the point set,
/// while a negative `s` reverses the named side relations.
///
/// Outward interval de Boor evaluation acts directly on the original
/// homogeneous source controls. Rounded controls produced by restriction,
/// Bezier extraction, or subdivision never participate in the proof. When the
/// interval field meets a slab boundary because of cancellation, exact affine
/// signs of the active original Euclidean control support provide a second
/// sound convex-hull filter; positive rational weights make those controls
/// valid projective convex-hull witnesses.
///
/// Invalid ranges or plane data, non-finite intermediate arithmetic, and an
/// unavailable exact fallback all return [`PlaneCurveRangeRelation::Candidate`]
/// rather than certifying a side or contained range.
pub fn classify_curve_range_against_plane_slab(
    curve: &NurbsCurve,
    range: ParamRange,
    origin: Point3,
    normal: Vec3,
    half_width: f64,
) -> PlaneCurveRangeRelation {
    if !valid_plane_range_query(curve, range, origin, normal, half_width) {
        return PlaneCurveRangeRelation::Candidate;
    }

    if let Some(distance) = signed_plane_form_interval(curve, range, origin, normal) {
        let relation = classify_plane_distance_interval(distance, half_width);
        if relation != PlaneCurveRangeRelation::Candidate {
            return relation;
        }
    }

    exact_active_control_relation(curve, range, origin, normal, half_width)
        .unwrap_or(PlaneCurveRangeRelation::Candidate)
}

/// Exact number of source knot-span slots inspected by one position-range
/// enclosure.
///
/// This count intentionally includes repeated/empty span slots: the interval
/// loop inspects each slot before deciding whether it contributes. Callers can
/// therefore admit the complete scan without first performing an unaccounted
/// knot-vector traversal.
pub(super) fn position_range_work_units(curve: &NurbsCurve) -> usize {
    curve.points().len() - curve.degree()
}

/// Conservatively enclose source-curve positions over a closed parameter range.
///
/// The range box is derived directly from the original representation; rounded
/// knot-insertion or restriction controls do not participate. Intersecting it
/// with the whole-source positive-weight control hull can only tighten two
/// independently conservative bounds. If interval evaluation is inconclusive,
/// the whole-source hull is retained so callers fail open rather than exclude
/// source geometry.
pub(super) fn position_range_aabb(curve: &NurbsCurve, range: ParamRange) -> Aabb3 {
    let source_hull = Aabb3::from_points(curve.points());
    let Some([x, y, z]) = position_component_intervals(curve, range, [0, 1, 2]) else {
        return source_hull;
    };
    let range_box = Aabb3 {
        min: Vec3::new(x.lo(), y.lo(), z.lo()),
        max: Vec3::new(x.hi(), y.hi(), z.hi()),
    };
    let bounded = Aabb3 {
        min: source_hull.min.max(range_box.min),
        max: source_hull.max.min(range_box.max),
    };
    if bounded.is_empty() {
        source_hull
    } else {
        bounded
    }
}

/// Enclose one Euclidean position component over a closed source range.
pub(super) fn position_component_interval(
    curve: &NurbsCurve,
    range: ParamRange,
    axis: usize,
) -> Option<Interval> {
    position_component_intervals(curve, range, [axis]).map(|bounds| bounds[0])
}

fn position_component_intervals<const N: usize>(
    curve: &NurbsCurve,
    range: ParamRange,
    axes: [usize; N],
) -> Option<[Interval; N]> {
    if axes.iter().any(|&axis| axis >= 3) || !range.is_finite() || range.width() < 0.0 {
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
    let mut result: Option<[Interval; N]> = None;
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
        let components = if curve.weights().is_none() {
            core::array::from_fn(|index| position[axes[index]])
        } else {
            if position[3].lo() <= 0.0 {
                return None;
            }
            let mut components = core::array::from_fn(|index| position[axes[index]]);
            for component in &mut components {
                *component = component.checked_div(position[3])?;
            }
            components
        };
        if !components.iter().copied().all(finite) {
            return None;
        }
        result = Some(match result {
            Some(current) => {
                core::array::from_fn(|index| hull(Some(current[index]), components[index]))
            }
            None => components,
        });
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
    if axis >= 3 {
        return None;
    }
    let homogeneous = homogeneous_controls(curve)?;
    derivative_homogeneous_coordinate_interval(curve, range, axis, &homogeneous)
}

/// Enclose the derivative of a bounded integer Euclidean coordinate form.
///
/// The scalar numerator is formed in homogeneous source-control space before
/// interval de Boor evaluation. This preserves correlations such as `x+y`
/// that independent component bounds lose. Coefficients outside the reviewed
/// `[-14,14]` corridor and inconclusive arithmetic fail closed.
pub(super) fn derivative_signed_linear_form_interval(
    curve: &NurbsCurve,
    range: ParamRange,
    coefficients: [i8; 3],
) -> Option<Interval> {
    if coefficients.iter().any(|&coefficient| {
        !(-MAX_LINEAR_FORM_COEFFICIENT..=MAX_LINEAR_FORM_COEFFICIENT).contains(&coefficient)
    }) || coefficients.iter().all(|&coefficient| coefficient == 0)
    {
        return None;
    }
    let homogeneous = homogeneous_controls(curve)?;
    let scalar_homogeneous = homogeneous
        .iter()
        .map(|control| {
            let scalar = coefficients.iter().enumerate().fold(
                Interval::point(0.0),
                |value, (axis, &coefficient)| {
                    value + Interval::point(f64::from(coefficient)) * control[axis]
                },
            );
            let packed = [
                scalar,
                Interval::point(0.0),
                Interval::point(0.0),
                control[3],
            ];
            packed.iter().copied().all(finite).then_some(packed)
        })
        .collect::<Option<Vec<_>>>()?;
    derivative_homogeneous_coordinate_interval(curve, range, 0, &scalar_homogeneous)
}

fn valid_plane_range_query(
    curve: &NurbsCurve,
    range: ParamRange,
    origin: Point3,
    normal: Vec3,
    half_width: f64,
) -> bool {
    let domain = curve.knots().domain();
    range.is_finite()
        && range.width() >= 0.0
        && range.lo >= domain.lo
        && range.hi <= domain.hi
        && finite_point(origin)
        && finite_point(normal)
        && (normal.x != 0.0 || normal.y != 0.0 || normal.z != 0.0)
        && half_width.is_finite()
        && half_width >= 0.0
}

fn signed_plane_form_interval(
    curve: &NurbsCurve,
    range: ParamRange,
    origin: Point3,
    normal: Vec3,
) -> Option<Interval> {
    let homogeneous = homogeneous_controls(curve)?;
    let plane_controls = homogeneous
        .iter()
        .map(|control| {
            let weight = control[3];
            let shifted = [
                control[0] - Interval::point(origin.x) * weight,
                control[1] - Interval::point(origin.y) * weight,
                control[2] - Interval::point(origin.z) * weight,
            ];
            let distance_numerator = Interval::point(normal.x) * shifted[0]
                + Interval::point(normal.y) * shifted[1]
                + Interval::point(normal.z) * shifted[2];
            let packed = [
                distance_numerator,
                Interval::point(0.0),
                Interval::point(0.0),
                weight,
            ];
            packed.iter().copied().all(finite).then_some(packed)
        })
        .collect::<Option<Vec<_>>>()?;

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
        let homogeneous_distance = interval_de_boor(
            knots,
            degree,
            span,
            Interval::new(local_lo, local_hi),
            &plane_controls,
        )?;
        let weight = homogeneous_distance[3];
        if weight.lo() <= 0.0 {
            return None;
        }
        let distance = homogeneous_distance[0].checked_div(weight)?;
        if !finite(distance) {
            return None;
        }
        result = Some(hull(result, distance));
    }
    result
}

fn classify_plane_distance_interval(
    distance: Interval,
    half_width: f64,
) -> PlaneCurveRangeRelation {
    if distance.hi() < -half_width {
        PlaneCurveRangeRelation::Negative
    } else if distance.lo() > half_width {
        PlaneCurveRangeRelation::Positive
    } else if distance.lo() >= -half_width && distance.hi() <= half_width {
        PlaneCurveRangeRelation::WithinSlab
    } else {
        PlaneCurveRangeRelation::Candidate
    }
}

fn exact_active_control_relation(
    curve: &NurbsCurve,
    range: ParamRange,
    origin: Point3,
    normal: Vec3,
    half_width: f64,
) -> Option<PlaneCurveRangeRelation> {
    let active = active_source_controls(curve, range)?;
    let mut common = None;
    for index in active {
        let relation = exact_control_relation(curve.points()[index], origin, normal, half_width)?;
        common = match common {
            None => Some(relation),
            Some(current) if current == relation => Some(current),
            Some(_) => return Some(PlaneCurveRangeRelation::Candidate),
        };
    }
    common
}

fn active_source_controls(curve: &NurbsCurve, range: ParamRange) -> Option<Vec<usize>> {
    let degree = curve.degree();
    let knots = curve.knots().as_slice();
    let last_span = curve.points().len().checked_sub(1)?;
    let mut active = vec![false; curve.points().len()];
    for span in degree..=last_span {
        if knots[span] >= knots[span + 1] {
            continue;
        }
        let local_lo = range.lo.max(knots[span]);
        let local_hi = range.hi.min(knots[span + 1]);
        if local_lo > local_hi {
            continue;
        }
        for is_active in active
            .iter_mut()
            .take(span + 1)
            .skip(span.checked_sub(degree)?)
        {
            *is_active = true;
        }
    }
    let active = active
        .into_iter()
        .enumerate()
        .filter_map(|(index, active)| active.then_some(index))
        .collect::<Vec<_>>();
    (!active.is_empty()).then_some(active)
}

fn exact_control_relation(
    point: Point3,
    origin: Point3,
    normal: Vec3,
    half_width: f64,
) -> Option<PlaneCurveRangeRelation> {
    let normal = normal.to_array();
    let point = point.to_array();
    let origin = origin.to_array();
    let above_positive_boundary = affine_dot3(normal, point, origin, -half_width)?.sign();
    if above_positive_boundary == Orientation::Positive {
        return Some(PlaneCurveRangeRelation::Positive);
    }
    let below_negative_boundary = affine_dot3(normal, point, origin, half_width)?.sign();
    if below_negative_boundary == Orientation::Negative {
        return Some(PlaneCurveRangeRelation::Negative);
    }
    Some(PlaneCurveRangeRelation::WithinSlab)
}

fn derivative_homogeneous_coordinate_interval(
    curve: &NurbsCurve,
    range: ParamRange,
    axis: usize,
    homogeneous: &[[Interval; 4]],
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
    let derivative = homogeneous_derivative_controls(curve, homogeneous)?;
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
        let position = interval_de_boor(knots, degree, span, parameter, homogeneous)?;
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

fn finite_point(point: Vec3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::Curve;
    use crate::vec::Point3;

    fn assert_derivatives_enclosed(curve: &NurbsCurve, range: ParamRange) {
        let position_bounds = position_range_aabb(curve, range);
        let bounds = [
            derivative_component_interval(curve, range, 0).unwrap(),
            derivative_component_interval(curve, range, 1).unwrap(),
            derivative_component_interval(curve, range, 2).unwrap(),
        ];
        for sample in 0..=512 {
            let parameter = range.lerp(f64::from(sample) / 512.0);
            let point = curve.eval(parameter);
            let derivative = curve.eval_derivs(parameter, 1).d[1];
            assert!(
                position_bounds.contains(point),
                "parameter={parameter}, point={point:?}, bounds={position_bounds:?}"
            );
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

    fn line(first: Point3, second: Point3, weights: Option<Vec<f64>>) -> NurbsCurve {
        NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![first, second], weights).unwrap()
    }

    fn legacy_control_relation(
        curve: &NurbsCurve,
        origin: Point3,
        normal: Vec3,
        half_width: f64,
    ) -> PlaneCurveRangeRelation {
        let distances = curve
            .points()
            .iter()
            .map(|point| normal.dot(*point - origin))
            .collect::<Vec<_>>();
        let minimum = distances.iter().copied().fold(f64::INFINITY, f64::min);
        let maximum = distances.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        if maximum < -half_width {
            PlaneCurveRangeRelation::Negative
        } else if minimum > half_width {
            PlaneCurveRangeRelation::Positive
        } else if distances
            .iter()
            .all(|distance| distance.abs() <= half_width)
        {
            PlaneCurveRangeRelation::WithinSlab
        } else {
            PlaneCurveRangeRelation::Candidate
        }
    }

    #[test]
    fn plane_slab_classification_preserves_ordinary_control_relations() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let half_width = 0.1;
        for curve in [
            line(
                Point3::new(-1.0, 0.0, 2.0),
                Point3::new(1.0, 0.0, 3.0),
                None,
            ),
            line(
                Point3::new(-1.0, 0.0, -3.0),
                Point3::new(1.0, 0.0, -2.0),
                Some(vec![0.75, 2.0]),
            ),
            line(
                Point3::new(-1.0, 0.0, 0.05),
                Point3::new(1.0, 0.0, -0.025),
                Some(vec![2.0, 0.5]),
            ),
            line(
                Point3::new(-1.0, 0.0, -1.0),
                Point3::new(1.0, 0.0, 1.0),
                None,
            ),
        ] {
            assert_eq!(
                classify_curve_range_against_plane_slab(
                    &curve,
                    curve.param_range(),
                    origin,
                    normal,
                    half_width,
                ),
                legacy_control_relation(&curve, origin, normal, half_width),
            );
        }
    }

    #[test]
    fn plane_slab_boundaries_are_closed_and_side_relations_are_strict() {
        let origin = Point3::default();
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let half_width = 0.25;
        for height in [half_width, -half_width] {
            let curve = line(
                Point3::new(-1.0, 0.0, height),
                Point3::new(1.0, 0.0, height),
                Some(vec![0.5, 2.0]),
            );
            assert_eq!(
                classify_curve_range_against_plane_slab(
                    &curve,
                    curve.param_range(),
                    origin,
                    normal,
                    half_width,
                ),
                PlaneCurveRangeRelation::WithinSlab,
            );
        }

        let positive = line(
            Point3::new(0.0, 0.0, half_width.next_up()),
            Point3::new(1.0, 0.0, half_width.next_up()),
            None,
        );
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &positive,
                positive.param_range(),
                origin,
                normal,
                half_width,
            ),
            PlaneCurveRangeRelation::Positive,
        );

        let negative = line(
            Point3::new(0.0, 0.0, (-half_width).next_down()),
            Point3::new(1.0, 0.0, (-half_width).next_down()),
            None,
        );
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &negative,
                negative.param_range(),
                origin,
                normal,
                half_width,
            ),
            PlaneCurveRangeRelation::Negative,
        );
    }

    #[test]
    fn nonunit_normals_use_scaled_affine_slab_units() {
        let origin = Point3::default();
        let scaled_normal = Vec3::new(0.0, 0.0, 2.0);
        let within = line(
            Point3::new(0.0, 0.0, 0.4),
            Point3::new(1.0, 0.0, 0.4),
            Some(vec![0.5, 2.0]),
        );
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &within,
                within.param_range(),
                origin,
                scaled_normal,
                1.0,
            ),
            PlaneCurveRangeRelation::WithinSlab,
        );

        let positive = line(Point3::new(0.0, 0.0, 0.6), Point3::new(1.0, 0.0, 0.6), None);
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &positive,
                positive.param_range(),
                origin,
                scaled_normal,
                1.0,
            ),
            PlaneCurveRangeRelation::Positive,
        );
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &positive,
                positive.param_range(),
                origin,
                -scaled_normal,
                1.0,
            ),
            PlaneCurveRangeRelation::Negative,
        );

        let tiny_normal = Vec3::new(0.0, 0.0, 1.0e-100);
        let tiny_within = line(Point3::new(0.0, 0.0, 1.0), Point3::new(1.0, 0.0, 1.0), None);
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &tiny_within,
                tiny_within.param_range(),
                origin,
                tiny_normal,
                2.0e-100,
            ),
            PlaneCurveRangeRelation::WithinSlab,
        );
        let tiny_positive = line(
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(1.0, 0.0, 3.0),
            Some(vec![0.75, 1.25]),
        );
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &tiny_positive,
                tiny_positive.param_range(),
                origin,
                tiny_normal,
                2.0e-100,
            ),
            PlaneCurveRangeRelation::Positive,
        );
    }

    #[test]
    fn active_source_support_is_range_local_and_conservative_at_repeated_knots() {
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-2.0, 0.0, 2.0),
                Point3::new(-1.0, 0.0, 2.0),
                Point3::new(0.0, 0.0, 2.0),
                Point3::new(1.0, 0.0, -10.0),
                Point3::new(2.0, 0.0, -10.0),
            ],
            None,
        )
        .unwrap();
        assert_eq!(
            active_source_controls(&curve, ParamRange::new(0.1, 0.2)).unwrap(),
            vec![0, 1, 2],
        );
        assert_eq!(
            active_source_controls(&curve, ParamRange::new(0.8, 0.9)).unwrap(),
            vec![2, 3, 4],
        );
        assert_eq!(
            active_source_controls(&curve, ParamRange::new(0.5, 0.5)).unwrap(),
            vec![0, 1, 2, 3, 4],
        );

        assert_eq!(
            classify_curve_range_against_plane_slab(
                &curve,
                ParamRange::new(0.1, 0.2),
                Point3::default(),
                Vec3::new(0.0, 0.0, 1.0),
                0.25,
            ),
            PlaneCurveRangeRelation::Positive,
            "inactive opposite-side controls must not weaken a strict first-span proof",
        );
    }

    #[test]
    fn plane_slab_classification_covers_polynomial_rational_and_multispan_sources() {
        let polynomial = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 0.35, 0.7, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-3.0, 0.0, 2.0),
                Point3::new(-1.0, 2.0, 2.5),
                Point3::new(1.0, -1.0, 3.0),
                Point3::new(2.0, 3.0, 2.25),
                Point3::new(4.0, 0.0, 2.75),
            ],
            None,
        )
        .unwrap();
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &polynomial,
                ParamRange::new(0.2, 0.85),
                Point3::default(),
                Vec3::new(0.0, 0.0, 1.0),
                0.25,
            ),
            PlaneCurveRangeRelation::Positive,
        );

        let normal = Vec3::new(0.6, 0.8, 0.0);
        let tangent = Vec3::new(0.8, -0.6, 0.0);
        let rational_points = [-3.0, -1.0, 0.5, 2.0, 4.0]
            .into_iter()
            .map(|along| normal * 2.0 + tangent * along)
            .collect();
        let rational = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 0.4, 0.8, 1.0, 1.0, 1.0],
            rational_points,
            Some(vec![0.5, 2.0, 0.75, 1.5, 1.0]),
        )
        .unwrap();
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &rational,
                ParamRange::new(0.15, 0.9),
                Point3::default(),
                normal,
                0.5,
            ),
            PlaneCurveRangeRelation::Positive,
        );
        for sample in 0..=256 {
            let parameter = 0.15 + 0.75 * f64::from(sample) / 256.0;
            assert!(normal.dot(rational.eval(parameter)) > 0.5);
        }
    }

    #[test]
    fn exact_active_controls_resolve_oblique_distance_cancellation() {
        let normal = Vec3::new(0.6, 0.8, 0.0);
        let positive = Point3::new(-2_863_298_200.0, 2_147_473_650.0, 0.0);
        assert_eq!(normal.dot(positive), 0.0);
        let positive_curve = line(positive, positive, None);
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &positive_curve,
                positive_curve.param_range(),
                Point3::default(),
                normal,
                1.0e-8,
            ),
            PlaneCurveRangeRelation::Positive,
        );

        let negative_curve = line(-positive, -positive, Some(vec![0.75, 2.0]));
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &negative_curve,
                negative_curve.param_range(),
                Point3::default(),
                normal,
                1.0e-8,
            ),
            PlaneCurveRangeRelation::Negative,
        );

        let crossing = line(positive, -positive, None);
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &crossing,
                crossing.param_range(),
                Point3::default(),
                normal,
                1.0e-8,
            ),
            PlaneCurveRangeRelation::Candidate,
        );

        let contained = line(Point3::default(), Point3::default(), Some(vec![0.5, 2.0]));
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &contained,
                contained.param_range(),
                Point3::default(),
                normal,
                1.0e-8,
            ),
            PlaneCurveRangeRelation::WithinSlab,
        );
    }

    #[test]
    fn source_ranges_retain_contact_lost_by_rounded_split_controls() {
        let contact_z = 9_007_199_254_740_991.0;
        let source = NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-1.0, 0.0, 9_007_199_254_740_360.0),
                Point3::new(-1.0 / 3.0, 0.0, 9_007_199_254_740_978.0),
                Point3::new(1.0 / 3.0, 0.0, 9_007_199_254_741_648.0),
                Point3::new(1.0, 0.0, 9_007_199_254_739_690.0),
            ],
            None,
        )
        .unwrap();
        let origin = Point3::new(0.0, 0.0, contact_z);
        let normal = Vec3::new(0.0, 0.0, 1.0);
        for range in [
            source.param_range(),
            ParamRange::new(0.0, 0.5),
            ParamRange::new(0.5, 1.0),
        ] {
            assert_eq!(
                classify_curve_range_against_plane_slab(&source, range, origin, normal, 0.0,),
                PlaneCurveRangeRelation::Candidate,
            );
        }

        let (left, right) = source.split_at(0.5).unwrap();
        assert!(left.points().iter().all(|point| point.z < contact_z));
        assert!(right.points().iter().all(|point| point.z < contact_z));
        assert_eq!(source.eval(0.5).z, contact_z);
    }

    #[test]
    fn invalid_or_inconclusive_plane_slab_queries_fail_open() {
        let curve = line(Point3::new(0.0, 0.0, 1.0), Point3::new(1.0, 0.0, 2.0), None);
        let candidate = PlaneCurveRangeRelation::Candidate;
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &curve,
                ParamRange::new(-0.1, 0.5),
                Point3::default(),
                Vec3::new(0.0, 0.0, 1.0),
                0.0,
            ),
            candidate,
        );
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &curve,
                ParamRange::unbounded(),
                Point3::default(),
                Vec3::new(0.0, 0.0, 1.0),
                0.0,
            ),
            candidate,
        );
        for (origin, normal, half_width) in [
            (Point3::default(), Vec3::default(), 0.0),
            (
                Point3::new(f64::NAN, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                0.0,
            ),
            (Point3::default(), Vec3::new(f64::INFINITY, 0.0, 0.0), 0.0),
            (Point3::default(), Vec3::new(0.0, 0.0, 1.0), -1.0),
            (Point3::default(), Vec3::new(0.0, 0.0, 1.0), f64::INFINITY),
        ] {
            assert_eq!(
                classify_curve_range_against_plane_slab(
                    &curve,
                    curve.param_range(),
                    origin,
                    normal,
                    half_width,
                ),
                candidate,
            );
        }

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
        assert_eq!(
            classify_curve_range_against_plane_slab(
                &overflow,
                overflow.param_range(),
                Point3::default(),
                Vec3::new(f64::MAX, 1.0, 0.0),
                0.0,
            ),
            candidate,
        );
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
        let range = ParamRange::new(0.125, 0.875);
        assert_derivatives_enclosed(&curve, range);
        let signed = derivative_signed_linear_form_interval(&curve, range, [1, -1, 1]).unwrap();
        let magnitude_twelve =
            derivative_signed_linear_form_interval(&curve, range, [12, -11, 0]).unwrap();
        let magnitude_thirteen =
            derivative_signed_linear_form_interval(&curve, range, [13, -12, 0]).unwrap();
        let magnitude_fourteen =
            derivative_signed_linear_form_interval(&curve, range, [14, -13, 0]).unwrap();
        assert!(derivative_signed_linear_form_interval(&curve, range, [15, 0, 0]).is_none());
        assert!(derivative_signed_linear_form_interval(&curve, range, [-15, 0, 0]).is_none());
        for sample in 0..=512 {
            let derivative = curve
                .eval_derivs(range.lerp(f64::from(sample) / 512.0), 1)
                .d[1];
            assert!(signed.contains(derivative.x - derivative.y + derivative.z));
            assert!(magnitude_twelve.contains(12.0 * derivative.x - 11.0 * derivative.y));
            assert!(magnitude_thirteen.contains(13.0 * derivative.x - 12.0 * derivative.y));
            assert!(magnitude_fourteen.contains(14.0 * derivative.x - 13.0 * derivative.y));
        }
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
        assert!(
            derivative_signed_linear_form_interval(
                &tiny_domain,
                tiny_domain.param_range(),
                [1, 1, 0],
            )
            .is_none()
        );
        assert!(
            derivative_signed_linear_form_interval(
                &tiny_domain,
                tiny_domain.param_range(),
                [11, 0, 0],
            )
            .is_none()
        );
        assert!(
            derivative_signed_linear_form_interval(
                &tiny_domain,
                tiny_domain.param_range(),
                [-11, 0, 0],
            )
            .is_none()
        );
        assert!(
            derivative_signed_linear_form_interval(
                &tiny_domain,
                tiny_domain.param_range(),
                [12, 0, 0],
            )
            .is_none()
        );
        assert!(
            derivative_signed_linear_form_interval(
                &tiny_domain,
                tiny_domain.param_range(),
                [-12, 0, 0],
            )
            .is_none()
        );
        assert!(
            derivative_signed_linear_form_interval(
                &tiny_domain,
                tiny_domain.param_range(),
                [13, 0, 0],
            )
            .is_none()
        );
        assert!(
            derivative_signed_linear_form_interval(
                &tiny_domain,
                tiny_domain.param_range(),
                [-13, 0, 0],
            )
            .is_none()
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
        assert!(
            derivative_signed_linear_form_interval(&overflow, overflow.param_range(), [12, 1, 0],)
                .is_none()
        );
        assert_eq!(
            position_range_aabb(&overflow, overflow.param_range()),
            Aabb3::from_points(overflow.points()),
            "an inconclusive source-range interval must retain the whole source hull"
        );
    }
}
