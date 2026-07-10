use super::conic::{
    ellipse_parameter as raw_ellipse_parameter, fit_periodic_parameter, parameter_tolerance,
};
use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint, ParamOrientation,
    accept_curve_curve_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Ellipse};
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};

const MIN_STEPS: usize = 96;
const MAX_STEPS: usize = 512;
const MAX_BISECTION_STEPS: usize = 80;
const MAX_PROJECTION_STEPS: usize = 32;
const COMPLETION_REASON: &str =
    "fixed-grid ellipse/NURBS candidate discovery does not prove complete coverage";

#[derive(Debug, Clone, Copy)]
struct Sample {
    t_curve: f64,
    distance: f64,
    ellipse_unwrapped: f64,
}

fn provisional_result(
    points: Vec<CurveCurvePoint>,
    overlaps: Vec<CurveCurveOverlap>,
) -> Result<CurveCurveIntersections> {
    CurveCurveIntersections::canonicalized_indeterminate(points, overlaps, COMPLETION_REASON)
}

/// Intersect a finite ellipse arc with a clamped NURBS curve restricted to a
/// finite range.
///
/// This fixed-grid bridge samples the point-to-ellipse distance along the
/// NURBS curve, polishes local minima, and clips all-on-ellipse spans to the
/// finite periodic ellipse interval.
pub fn intersect_bounded_ellipse_nurbs(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    curve: &NurbsCurve,
    curve_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    validate_ranges(ellipse, ellipse_range, curve, curve_range, tolerances)?;

    let curve_range = clamp_to_domain(curve_range, curve.param_range());
    let curve_parameter_tol = curve_parameter_tolerance(curve_range, tolerances);
    if curve_range.width() <= curve_parameter_tol {
        return single_parameter_intersection(
            ellipse,
            ellipse_range,
            curve,
            curve_range.lo,
            tolerances,
        );
    }

    let samples = sample_curve(ellipse, curve, curve_range);
    if samples
        .iter()
        .all(|sample| sample.distance <= tolerances.linear())
    {
        return contained_curve_intersections(ellipse, ellipse_range, curve, &samples, tolerances);
    }

    let mut points = Vec::new();
    if let Some(first) = samples.first()
        && first.distance <= tolerances.linear()
    {
        push_root_candidate(
            ellipse,
            ellipse_range,
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
            ellipse,
            ellipse_range,
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
        let root = minimize_distance(ellipse, curve, a.t_curve, c.t_curve, curve_parameter_tol);
        push_root_candidate(
            ellipse,
            ellipse_range,
            curve,
            root,
            Some(local_minimum_kind(
                ellipse, curve, a.t_curve, c.t_curve, tolerances,
            )),
            &mut points,
            tolerances,
        );
    }

    provisional_result(points, Vec::new())
}

fn single_parameter_intersection(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    curve: &NurbsCurve,
    t_curve: f64,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    if distance_to_ellipse(curve.eval(t_curve), ellipse) > tolerances.linear() {
        return Ok(CurveCurveIntersections::indeterminate_empty(
            COMPLETION_REASON,
        ));
    }
    let mut points = Vec::new();
    push_root_candidate(
        ellipse,
        ellipse_range,
        curve,
        t_curve,
        None,
        &mut points,
        tolerances,
    );
    provisional_result(points, Vec::new())
}

fn contained_curve_intersections(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
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
        collect_ellipse_range_overlaps(
            ellipse,
            ellipse_range,
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
fn collect_ellipse_range_overlaps(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    curve: &NurbsCurve,
    a: Sample,
    b: Sample,
    curve_parameter_tol: f64,
    tolerances: Tolerances,
    overlaps: &mut Vec<CurveCurveOverlap>,
) {
    let ellipse_tol = parameter_tolerance(ellipse.minor_radius(), tolerances);
    let mut cuts = vec![a.t_curve, b.t_curve];
    for target in ellipse_boundary_images(a.ellipse_unwrapped, b.ellipse_unwrapped, ellipse_range) {
        if let Some(root) = ellipse_parameter_root(
            ellipse,
            curve,
            a,
            b,
            target,
            curve_parameter_tol,
            ellipse_tol,
        ) {
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
        if ellipse_parameter(curve.eval(mid), ellipse, ellipse_range, ellipse_tol).is_none() {
            continue;
        }
        let start_unwrapped = ellipse_parameter_unwrapped_near(curve.eval(*lo), ellipse, a);
        let end_unwrapped = ellipse_parameter_unwrapped_near(curve.eval(*hi), ellipse, b);
        let Some(start_ellipse) =
            ellipse_parameter(curve.eval(*lo), ellipse, ellipse_range, ellipse_tol)
        else {
            continue;
        };
        let Some(end_ellipse) =
            ellipse_parameter(curve.eval(*hi), ellipse, ellipse_range, ellipse_tol)
        else {
            continue;
        };
        overlaps.push(CurveCurveOverlap {
            a: ParamRange::new(
                start_ellipse.min(end_ellipse),
                start_ellipse.max(end_ellipse),
            ),
            b: ParamRange::new(*lo, *hi),
            orientation: if end_unwrapped >= start_unwrapped {
                ParamOrientation::Same
            } else {
                ParamOrientation::Reversed
            },
        });
    }
}

fn ellipse_boundary_images(a: f64, b: f64, range: ParamRange) -> Vec<f64> {
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

fn ellipse_parameter_root(
    ellipse: &Ellipse,
    curve: &NurbsCurve,
    a: Sample,
    b: Sample,
    target: f64,
    curve_parameter_tol: f64,
    ellipse_tol: f64,
) -> Option<f64> {
    let mut lo = a.t_curve;
    let mut hi = b.t_curve;
    let mut f_lo = a.ellipse_unwrapped - target;
    let f_hi = b.ellipse_unwrapped - target;
    if f_lo.abs() <= ellipse_tol {
        return Some(lo);
    }
    if f_hi.abs() <= ellipse_tol {
        return Some(hi);
    }
    if same_sign(f_lo, f_hi) {
        return None;
    }
    let mut root = (lo + hi) / 2.0;
    for _ in 0..MAX_BISECTION_STEPS {
        root = (lo + hi) / 2.0;
        let raw = closest_ellipse_parameter(curve.eval(root), ellipse);
        let f_mid = unwrap_angle_near(raw, target) - target;
        if f_mid.abs() <= ellipse_tol || hi - lo <= curve_parameter_tol {
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

fn sample_curve(ellipse: &Ellipse, curve: &NurbsCurve, curve_range: ParamRange) -> Vec<Sample> {
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
            let raw = closest_ellipse_parameter(point, ellipse);
            let ellipse_unwrapped = previous
                .map(|angle| unwrap_angle_near(raw, angle))
                .unwrap_or(raw);
            previous = Some(ellipse_unwrapped);
            Sample {
                t_curve,
                distance: distance_to_ellipse(point, ellipse),
                ellipse_unwrapped,
            }
        })
        .collect()
}

fn minimize_distance(
    ellipse: &Ellipse,
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
        let f_left = distance_to_ellipse(curve.eval(left), ellipse);
        let f_right = distance_to_ellipse(curve.eval(right), ellipse);
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
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    curve: &NurbsCurve,
    t_curve: f64,
    forced_kind: Option<ContactKind>,
    points: &mut Vec<CurveCurvePoint>,
    tolerances: Tolerances,
) {
    let point = curve.eval(t_curve);
    if distance_to_ellipse(point, ellipse) > tolerances.linear() {
        return;
    }
    let Some(t_ellipse) = ellipse_parameter(
        point,
        ellipse,
        ellipse_range,
        parameter_tolerance(ellipse.minor_radius(), tolerances),
    ) else {
        return;
    };
    let Some(point) = accept_curve_curve_candidate(
        ellipse,
        t_ellipse,
        curve,
        t_curve,
        forced_kind
            .map(|kind| forced_contact_kind(curve, t_curve, kind, tolerances))
            .unwrap_or_else(|| contact_kind(ellipse, curve, t_curve, t_ellipse, tolerances)),
        tolerances,
    ) else {
        return;
    };
    push_distinct_point(points, point, ellipse, tolerances);
}

fn local_minimum_kind(
    ellipse: &Ellipse,
    curve: &NurbsCurve,
    lo: f64,
    hi: f64,
    tolerances: Tolerances,
) -> ContactKind {
    let a = offset_from_ellipse(curve.eval(lo), ellipse);
    let b = offset_from_ellipse(curve.eval(hi), ellipse);
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
    ellipse: &Ellipse,
    curve: &NurbsCurve,
    t_curve: f64,
    t_ellipse: f64,
    tolerances: Tolerances,
) -> ContactKind {
    let curve_tangent = curve.eval_derivs(t_curve, 1).d[1];
    let ellipse_tangent = ellipse.eval_derivs(t_ellipse, 1).d[1];
    let scale = curve_tangent.norm() * ellipse_tangent.norm();
    if scale <= tolerances.linear() {
        ContactKind::Singular
    } else if curve_tangent.cross(ellipse_tangent).norm() > scale * tolerances.angular() {
        ContactKind::Transverse
    } else {
        ContactKind::Tangent
    }
}

fn push_distinct_point(
    points: &mut Vec<CurveCurvePoint>,
    candidate: CurveCurvePoint,
    ellipse: &Ellipse,
    tolerances: Tolerances,
) {
    let ellipse_tol = parameter_tolerance(ellipse.minor_radius(), tolerances);
    if !points.iter().any(|point| {
        point.point.dist(candidate.point) <= tolerances.linear()
            || (point.t_a - candidate.t_a).abs() <= ellipse_tol
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

fn distance_to_ellipse(point: Point3, ellipse: &Ellipse) -> f64 {
    offset_from_ellipse(point, ellipse).norm()
}

fn offset_from_ellipse(point: Point3, ellipse: &Ellipse) -> Vec3 {
    point - ellipse.eval(closest_ellipse_parameter(point, ellipse))
}

fn ellipse_parameter(
    point: Point3,
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    tolerance: f64,
) -> Option<f64> {
    fit_periodic_parameter(
        closest_ellipse_parameter(point, ellipse),
        ellipse_range,
        tolerance,
    )
}

fn ellipse_parameter_unwrapped_near(point: Point3, ellipse: &Ellipse, sample: Sample) -> f64 {
    unwrap_angle_near(
        closest_ellipse_parameter(point, ellipse),
        sample.ellipse_unwrapped,
    )
}

fn closest_ellipse_parameter(point: Point3, ellipse: &Ellipse) -> f64 {
    let local = ellipse.frame().to_local(point);
    let initial = raw_ellipse_parameter(local, ellipse);
    let mut candidates = [
        refine_projection_parameter(initial, local, ellipse),
        refine_projection_parameter(initial + core::f64::consts::PI, local, ellipse),
        0.0,
        core::f64::consts::FRAC_PI_2,
        core::f64::consts::PI,
        3.0 * core::f64::consts::FRAC_PI_2,
    ];
    candidates.sort_by(|a, b| {
        ellipse_distance_sq(local, ellipse, *a).total_cmp(&ellipse_distance_sq(local, ellipse, *b))
    });
    candidates[0]
}

fn refine_projection_parameter(mut t: f64, local: Vec3, ellipse: &Ellipse) -> f64 {
    for _ in 0..MAX_PROJECTION_STEPS {
        let (point, d1, d2) = ellipse_local_derivs(ellipse, t);
        let residual = point - local;
        let f = residual.dot(d1);
        let df = d1.dot(d1) + residual.dot(d2);
        if df.abs() <= 1e-18 {
            break;
        }
        let step = f / df;
        t -= step;
        if step.abs() <= 1e-14 {
            break;
        }
    }
    t
}

fn ellipse_distance_sq(local: Vec3, ellipse: &Ellipse, t: f64) -> f64 {
    let (point, _, _) = ellipse_local_derivs(ellipse, t);
    (local - point).norm_sq()
}

fn ellipse_local_derivs(ellipse: &Ellipse, t: f64) -> (Vec3, Vec3, Vec3) {
    let (sin, cos) = math::sincos(t);
    let major = ellipse.major_radius();
    let minor = ellipse.minor_radius();
    (
        Vec3::new(major * cos, minor * sin, 0.0),
        Vec3::new(-major * sin, minor * cos, 0.0),
        Vec3::new(-major * cos, -minor * sin, 0.0),
    )
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
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    curve: &NurbsCurve,
    curve_range: ParamRange,
    tolerances: Tolerances,
) -> Result<()> {
    if !ellipse_range.is_finite()
        || !curve_range.is_finite()
        || ellipse_range.width() < 0.0
        || curve_range.width() < 0.0
    {
        return Err(Error::InvalidGeometry {
            reason: "ellipse/nurbs intersection requires finite non-reversed ranges",
        });
    }
    if ellipse_range.width()
        > core::f64::consts::TAU + parameter_tolerance(ellipse.minor_radius(), tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded ellipse range cannot span more than one period",
        });
    }
    if !curve.knots().is_clamped() {
        return Err(Error::InvalidGeometry {
            reason: "ellipse/nurbs intersection requires a clamped NURBS curve",
        });
    }
    let domain = curve.param_range();
    let curve_parameter_tol = curve_parameter_tolerance(domain, tolerances);
    if curve_range.lo < domain.lo - curve_parameter_tol
        || curve_range.hi > domain.hi + curve_parameter_tol
    {
        return Err(Error::InvalidGeometry {
            reason: "ellipse/nurbs intersection curve range must lie within the NURBS domain",
        });
    }
    Ok(())
}
