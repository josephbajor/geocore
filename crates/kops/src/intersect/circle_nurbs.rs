use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint, ParamOrientation,
    accept_curve_curve_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};

const MIN_STEPS: usize = 96;
const MAX_STEPS: usize = 512;
const MAX_BISECTION_STEPS: usize = 80;
const COMPLETION_REASON: &str =
    "fixed-grid circle/NURBS candidate discovery does not prove complete coverage";

#[derive(Debug, Clone, Copy)]
struct Sample {
    t_curve: f64,
    distance: f64,
    circle_unwrapped: f64,
}

fn provisional_result(
    points: Vec<CurveCurvePoint>,
    overlaps: Vec<CurveCurveOverlap>,
) -> Result<CurveCurveIntersections> {
    CurveCurveIntersections::canonicalized_indeterminate(points, overlaps, COMPLETION_REASON)
}

/// Intersect a finite circle arc with a clamped NURBS curve restricted to a
/// finite range.
///
/// This fixed-grid bridge samples the point-to-circle distance along the
/// NURBS curve, polishes local minima, and clips all-on-circle spans to the
/// finite periodic circle interval.
pub fn intersect_bounded_circle_nurbs(
    circle: &Circle,
    circle_range: ParamRange,
    curve: &NurbsCurve,
    curve_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    validate_ranges(circle, circle_range, curve, curve_range, tolerances)?;

    let curve_range = clamp_to_domain(curve_range, curve.param_range());
    let curve_parameter_tol = curve_parameter_tolerance(curve_range, tolerances);
    if curve_range.width() <= curve_parameter_tol {
        return single_parameter_intersection(
            circle,
            circle_range,
            curve,
            curve_range.lo,
            tolerances,
        );
    }

    let samples = sample_curve(circle, curve, curve_range);
    if samples
        .iter()
        .all(|sample| sample.distance <= tolerances.linear())
    {
        return contained_curve_intersections(circle, circle_range, curve, &samples, tolerances);
    }

    let mut points = Vec::new();
    if let Some(first) = samples.first()
        && first.distance <= tolerances.linear()
    {
        push_root_candidate(
            circle,
            circle_range,
            curve,
            first.t_curve,
            None,
            &mut points,
            tolerances,
        );
    }
    if let Some(last) = samples.last()
        && last.distance <= tolerances.linear()
    {
        push_root_candidate(
            circle,
            circle_range,
            curve,
            last.t_curve,
            None,
            &mut points,
            tolerances,
        );
    }
    for triple in samples.windows(3) {
        let [a, b, c] = triple else {
            continue;
        };
        if b.distance > a.distance || b.distance > c.distance {
            continue;
        }
        let root = minimize_distance(circle, curve, a.t_curve, c.t_curve, curve_parameter_tol);
        push_root_candidate(
            circle,
            circle_range,
            curve,
            root,
            Some(local_minimum_kind(
                circle, curve, a.t_curve, c.t_curve, tolerances,
            )),
            &mut points,
            tolerances,
        );
    }

    provisional_result(points, Vec::new())
}

fn single_parameter_intersection(
    circle: &Circle,
    circle_range: ParamRange,
    curve: &NurbsCurve,
    t_curve: f64,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    if distance_to_circle(curve.eval(t_curve), circle) > tolerances.linear() {
        return Ok(CurveCurveIntersections::indeterminate_empty(
            COMPLETION_REASON,
        ));
    }
    let mut points = Vec::new();
    push_root_candidate(
        circle,
        circle_range,
        curve,
        t_curve,
        None,
        &mut points,
        tolerances,
    );
    provisional_result(points, Vec::new())
}

fn contained_curve_intersections(
    circle: &Circle,
    circle_range: ParamRange,
    curve: &NurbsCurve,
    samples: &[Sample],
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let global_range = ParamRange::new(samples[0].t_curve, samples[samples.len() - 1].t_curve);
    let curve_parameter_tol = curve_parameter_tolerance(global_range, tolerances);
    let mut overlaps = Vec::new();
    for pair in samples.windows(2) {
        let [a, b] = pair else {
            continue;
        };
        collect_circle_range_overlaps(
            circle,
            circle_range,
            curve,
            *a,
            *b,
            curve_parameter_tol,
            tolerances,
            &mut overlaps,
        );
    }
    merge_overlaps(&mut overlaps, global_range, tolerances);
    provisional_result(Vec::new(), overlaps)
}

#[allow(clippy::too_many_arguments)]
fn collect_circle_range_overlaps(
    circle: &Circle,
    circle_range: ParamRange,
    curve: &NurbsCurve,
    a: Sample,
    b: Sample,
    curve_parameter_tol: f64,
    tolerances: Tolerances,
    overlaps: &mut Vec<CurveCurveOverlap>,
) {
    let circle_tol = parameter_tolerance(circle.radius(), tolerances);
    let mut cuts = vec![a.t_curve, b.t_curve];
    for target in circle_boundary_images(a.circle_unwrapped, b.circle_unwrapped, circle_range) {
        if let Some(root) =
            circle_parameter_root(circle, curve, a, b, target, curve_parameter_tol, circle_tol)
        {
            cuts.push(root);
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() <= curve_parameter_tol);

    for pair in cuts.windows(2) {
        let [lo, hi] = pair else {
            continue;
        };
        if hi - lo <= curve_parameter_tol {
            continue;
        }
        let mid = (lo + hi) / 2.0;
        if circle_parameter(curve.eval(mid), circle, circle_range, circle_tol).is_none() {
            continue;
        }
        let start_unwrapped = circle_parameter_unwrapped_near(curve.eval(*lo), circle, a);
        let end_unwrapped = circle_parameter_unwrapped_near(curve.eval(*hi), circle, b);
        let Some(start_circle) =
            circle_parameter(curve.eval(*lo), circle, circle_range, circle_tol)
        else {
            continue;
        };
        let Some(end_circle) = circle_parameter(curve.eval(*hi), circle, circle_range, circle_tol)
        else {
            continue;
        };
        overlaps.push(CurveCurveOverlap {
            a: ParamRange::new(start_circle.min(end_circle), start_circle.max(end_circle)),
            b: ParamRange::new(*lo, *hi),
            orientation: if end_unwrapped >= start_unwrapped {
                ParamOrientation::Same
            } else {
                ParamOrientation::Reversed
            },
        });
    }
}

fn circle_boundary_images(a: f64, b: f64, range: ParamRange) -> Vec<f64> {
    let lo = a.min(b);
    let hi = a.max(b);
    let mut out = Vec::new();
    for base in [range.lo, range.hi] {
        let period = core::f64::consts::TAU;
        let k_min = ((lo - base) / period).floor() as i64 - 1;
        let k_max = ((hi - base) / period).ceil() as i64 + 1;
        for k in k_min..=k_max {
            let target = base + k as f64 * period;
            if target >= lo && target <= hi {
                out.push(target);
            }
        }
    }
    out.sort_by(f64::total_cmp);
    out.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
    out
}

fn circle_parameter_root(
    circle: &Circle,
    curve: &NurbsCurve,
    a: Sample,
    b: Sample,
    target: f64,
    curve_parameter_tol: f64,
    circle_tol: f64,
) -> Option<f64> {
    let mut lo = a.t_curve;
    let mut hi = b.t_curve;
    let mut f_lo = a.circle_unwrapped - target;
    let f_hi = b.circle_unwrapped - target;
    if f_lo.abs() <= circle_tol {
        return Some(lo);
    }
    if f_hi.abs() <= circle_tol {
        return Some(hi);
    }
    if same_sign(f_lo, f_hi) {
        return None;
    }
    let mut root = (lo + hi) / 2.0;
    for _ in 0..MAX_BISECTION_STEPS {
        root = (lo + hi) / 2.0;
        let raw = raw_circle_parameter(curve.eval(root), circle);
        let f_mid = unwrap_angle_near(raw, target) - target;
        if f_mid.abs() <= circle_tol || hi - lo <= curve_parameter_tol {
            break;
        }
        if same_sign(f_lo, f_mid) {
            lo = root;
            f_lo = f_mid;
        } else {
            hi = root;
        }
    }
    Some(root)
}

fn sample_curve(circle: &Circle, curve: &NurbsCurve, curve_range: ParamRange) -> Vec<Sample> {
    let span_hint = curve
        .knots()
        .control_count()
        .saturating_sub(curve.degree())
        .max(1);
    let steps = (span_hint * curve.degree().max(1) * 32).clamp(MIN_STEPS, MAX_STEPS);
    let mut previous = None;
    (0..=steps)
        .map(|i| {
            let t_curve = curve_range.lerp(i as f64 / steps as f64);
            let point = curve.eval(t_curve);
            let raw = raw_circle_parameter(point, circle);
            let circle_unwrapped = previous
                .map(|angle| unwrap_angle_near(raw, angle))
                .unwrap_or(raw);
            previous = Some(circle_unwrapped);
            Sample {
                t_curve,
                distance: distance_to_circle(point, circle),
                circle_unwrapped,
            }
        })
        .collect()
}

fn minimize_distance(
    circle: &Circle,
    curve: &NurbsCurve,
    mut lo: f64,
    mut hi: f64,
    curve_parameter_tol: f64,
) -> f64 {
    for _ in 0..MAX_BISECTION_STEPS {
        if hi - lo <= curve_parameter_tol {
            break;
        }
        let third = (hi - lo) / 3.0;
        let left = lo + third;
        let right = hi - third;
        let f_left = distance_to_circle(curve.eval(left), circle);
        let f_right = distance_to_circle(curve.eval(right), circle);
        if (f_left - f_right).abs() <= 1e-18 {
            lo = left;
            hi = right;
        } else if f_left < f_right {
            hi = right;
        } else {
            lo = left;
        }
    }
    (lo + hi) / 2.0
}

fn push_root_candidate(
    circle: &Circle,
    circle_range: ParamRange,
    curve: &NurbsCurve,
    t_curve: f64,
    forced_kind: Option<ContactKind>,
    points: &mut Vec<CurveCurvePoint>,
    tolerances: Tolerances,
) {
    let point = curve.eval(t_curve);
    if distance_to_circle(point, circle) > tolerances.linear() {
        return;
    }
    let Some(t_circle) = circle_parameter(
        point,
        circle,
        circle_range,
        parameter_tolerance(circle.radius(), tolerances),
    ) else {
        return;
    };
    let Some(point) = accept_curve_curve_candidate(
        circle,
        t_circle,
        curve,
        t_curve,
        forced_kind
            .map(|kind| forced_contact_kind(curve, t_curve, kind, tolerances))
            .unwrap_or_else(|| contact_kind(circle, curve, t_curve, t_circle, tolerances)),
        tolerances,
    ) else {
        return;
    };
    push_distinct_point(points, point, circle, tolerances);
}

fn local_minimum_kind(
    circle: &Circle,
    curve: &NurbsCurve,
    lo: f64,
    hi: f64,
    tolerances: Tolerances,
) -> ContactKind {
    let a = offset_from_circle(curve.eval(lo), circle);
    let b = offset_from_circle(curve.eval(hi), circle);
    if a.norm() <= tolerances.linear() || b.norm() <= tolerances.linear() {
        ContactKind::Tangent
    } else if a.dot(b) < 0.0 {
        ContactKind::Transverse
    } else {
        ContactKind::Tangent
    }
}

fn forced_contact_kind(
    curve: &NurbsCurve,
    t_curve: f64,
    kind: ContactKind,
    tolerances: Tolerances,
) -> ContactKind {
    let tangent = curve.eval_derivs(t_curve, 1).d[1];
    if tangent.norm() <= tolerances.linear() {
        ContactKind::Singular
    } else {
        kind
    }
}

fn contact_kind(
    circle: &Circle,
    curve: &NurbsCurve,
    t_curve: f64,
    t_circle: f64,
    tolerances: Tolerances,
) -> ContactKind {
    let curve_tangent = curve.eval_derivs(t_curve, 1).d[1];
    let circle_tangent = circle.eval_derivs(t_circle, 1).d[1];
    let scale = curve_tangent.norm() * circle_tangent.norm();
    if scale <= tolerances.linear() {
        ContactKind::Singular
    } else if curve_tangent.cross(circle_tangent).norm() > scale * tolerances.angular() {
        ContactKind::Transverse
    } else {
        ContactKind::Tangent
    }
}

fn push_distinct_point(
    points: &mut Vec<CurveCurvePoint>,
    candidate: CurveCurvePoint,
    circle: &Circle,
    tolerances: Tolerances,
) {
    let circle_tol = parameter_tolerance(circle.radius(), tolerances);
    if !points.iter().any(|point| {
        point.point.dist(candidate.point) <= tolerances.linear()
            || (point.t_a - candidate.t_a).abs() <= circle_tol
                && (point.t_b - candidate.t_b).abs() <= tolerances.angular()
    }) {
        points.push(candidate);
    }
}

fn merge_overlaps(
    overlaps: &mut Vec<CurveCurveOverlap>,
    global_range: ParamRange,
    tolerances: Tolerances,
) {
    overlaps.sort_by(|a, b| a.b.lo.total_cmp(&b.b.lo));
    let curve_parameter_tol = curve_parameter_tolerance(global_range, tolerances);
    let mut merged: Vec<CurveCurveOverlap> = Vec::new();
    for overlap in overlaps.drain(..) {
        if let Some(last) = merged.last_mut()
            && last.orientation == overlap.orientation
            && overlap.b.lo <= last.b.hi + curve_parameter_tol
        {
            last.a = ParamRange::new(last.a.lo.min(overlap.a.lo), last.a.hi.max(overlap.a.hi));
            last.b = ParamRange::new(last.b.lo, last.b.hi.max(overlap.b.hi));
            continue;
        }
        merged.push(overlap);
    }
    *overlaps = merged;
}

fn distance_to_circle(point: Point3, circle: &Circle) -> f64 {
    offset_from_circle(point, circle).norm()
}

fn offset_from_circle(point: Point3, circle: &Circle) -> Vec3 {
    let local = circle.frame().to_local(point);
    let radial = (local.x * local.x + local.y * local.y).sqrt();
    let closest = if radial <= 1e-14 {
        circle.frame().point_at(circle.radius(), 0.0, 0.0)
    } else {
        let scale = circle.radius() / radial;
        circle
            .frame()
            .point_at(local.x * scale, local.y * scale, 0.0)
    };
    point - closest
}

fn circle_parameter(
    point: Point3,
    circle: &Circle,
    circle_range: ParamRange,
    tolerance: f64,
) -> Option<f64> {
    fit_periodic_parameter(raw_circle_parameter(point, circle), circle_range, tolerance)
}

fn circle_parameter_unwrapped_near(point: Point3, circle: &Circle, sample: Sample) -> f64 {
    unwrap_angle_near(raw_circle_parameter(point, circle), sample.circle_unwrapped)
}

fn raw_circle_parameter(point: Point3, circle: &Circle) -> f64 {
    let local = circle.frame().to_local(point);
    math::atan2(local.y, local.x)
}

fn unwrap_angle_near(raw: f64, reference: f64) -> f64 {
    let period = core::f64::consts::TAU;
    raw + ((reference - raw) / period).round() * period
}

fn same_sign(a: f64, b: f64) -> bool {
    (a < 0.0 && b < 0.0) || (a > 0.0 && b > 0.0)
}

fn clamp_to_domain(range: ParamRange, domain: ParamRange) -> ParamRange {
    ParamRange::new(
        range.lo.clamp(domain.lo, domain.hi),
        range.hi.clamp(domain.lo, domain.hi),
    )
}

fn curve_parameter_tolerance(range: ParamRange, tolerances: Tolerances) -> f64 {
    (range.width().abs() * 1e-10)
        .max(tolerances.angular())
        .max(1e-12)
}

fn validate_ranges(
    circle: &Circle,
    circle_range: ParamRange,
    curve: &NurbsCurve,
    curve_range: ParamRange,
    tolerances: Tolerances,
) -> Result<()> {
    if !circle_range.is_finite()
        || !curve_range.is_finite()
        || circle_range.width() < 0.0
        || curve_range.width() < 0.0
    {
        return Err(Error::InvalidGeometry {
            reason: "circle/nurbs intersection requires finite non-reversed ranges",
        });
    }
    if circle_range.width()
        > core::f64::consts::TAU + parameter_tolerance(circle.radius(), tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded circle range cannot span more than one period",
        });
    }
    if !curve.knots().is_clamped() {
        return Err(Error::InvalidGeometry {
            reason: "circle/nurbs intersection requires a clamped NURBS curve",
        });
    }
    let domain = curve.param_range();
    let curve_parameter_tol = curve_parameter_tolerance(domain, tolerances);
    if curve_range.lo < domain.lo - curve_parameter_tol
        || curve_range.hi > domain.hi + curve_parameter_tol
    {
        return Err(Error::InvalidGeometry {
            reason: "circle/nurbs intersection curve range must lie within the NURBS domain",
        });
    }
    Ok(())
}
