use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint, ParamOrientation,
    accept_curve_curve_candidate,
};
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Line};
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};

const MIN_STEPS: usize = 96;
const MAX_STEPS: usize = 512;
const MAX_BISECTION_STEPS: usize = 80;
const COMPLETION_REASON: &str =
    "fixed-grid line/NURBS candidate discovery does not prove complete coverage";

#[derive(Debug, Clone, Copy)]
struct Sample {
    t_curve: f64,
    distance: f64,
    t_line: f64,
}

fn provisional_result(
    points: Vec<CurveCurvePoint>,
    overlaps: Vec<CurveCurveOverlap>,
) -> Result<CurveCurveIntersections> {
    CurveCurveIntersections::canonicalized_indeterminate(points, overlaps, COMPLETION_REASON)
}

/// Intersect a finite line range with a clamped NURBS curve restricted to a
/// finite range.
///
/// This is the first fixed-grid bridge for NURBS curve/curve work: it samples
/// the point-to-line distance along the NURBS curve, polishes local minima,
/// and clips all-on-line spans to the finite line interval.
pub fn intersect_bounded_line_nurbs(
    line: &Line,
    line_range: ParamRange,
    curve: &NurbsCurve,
    curve_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    validate_ranges(curve, line_range, curve_range, tolerances)?;

    let curve_range = clamp_to_domain(curve_range, curve.param_range());
    let parameter_tol = parameter_tolerance(curve_range, tolerances);
    if curve_range.width() <= parameter_tol {
        return single_parameter_intersection(line, line_range, curve, curve_range.lo, tolerances);
    }

    let samples = sample_curve(line, curve, curve_range);
    if samples
        .iter()
        .all(|sample| sample.distance <= tolerances.linear())
    {
        return contained_curve_intersections(line, line_range, curve, &samples, tolerances);
    }

    let mut points = Vec::new();
    if let Some(first) = samples.first()
        && first.distance <= tolerances.linear()
    {
        push_root_candidate(
            line,
            line_range,
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
            line,
            line_range,
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
        let root = minimize_distance(line, curve, a.t_curve, c.t_curve, parameter_tol);
        push_root_candidate(
            line,
            line_range,
            curve,
            root,
            Some(local_minimum_kind(
                line, curve, a.t_curve, c.t_curve, tolerances,
            )),
            &mut points,
            tolerances,
        );
    }

    provisional_result(points, Vec::new())
}

fn single_parameter_intersection(
    line: &Line,
    line_range: ParamRange,
    curve: &NurbsCurve,
    t_curve: f64,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    if distance_to_line(curve.eval(t_curve), line) > tolerances.linear() {
        return Ok(CurveCurveIntersections::indeterminate_empty(
            COMPLETION_REASON,
        ));
    }
    let mut points = Vec::new();
    push_root_candidate(
        line,
        line_range,
        curve,
        t_curve,
        None,
        &mut points,
        tolerances,
    );
    provisional_result(points, Vec::new())
}

fn contained_curve_intersections(
    line: &Line,
    line_range: ParamRange,
    curve: &NurbsCurve,
    samples: &[Sample],
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let global_range = ParamRange::new(samples[0].t_curve, samples[samples.len() - 1].t_curve);
    let parameter_tol = parameter_tolerance(global_range, tolerances);
    let mut overlaps = Vec::new();
    for pair in samples.windows(2) {
        let [a, b] = pair else {
            continue;
        };
        collect_line_range_overlaps(
            line,
            line_range,
            curve,
            *a,
            *b,
            parameter_tol,
            tolerances,
            &mut overlaps,
        );
    }
    merge_overlaps(&mut overlaps, global_range, tolerances);
    provisional_result(Vec::new(), overlaps)
}

#[allow(clippy::too_many_arguments)]
fn collect_line_range_overlaps(
    line: &Line,
    line_range: ParamRange,
    curve: &NurbsCurve,
    a: Sample,
    b: Sample,
    parameter_tol: f64,
    tolerances: Tolerances,
    overlaps: &mut Vec<CurveCurveOverlap>,
) {
    let mut cuts = vec![a.t_curve, b.t_curve];
    for bound in [line_range.lo, line_range.hi] {
        if let Some(root) =
            line_parameter_root(line, curve, a, b, bound, parameter_tol, tolerances.linear())
        {
            cuts.push(root);
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() <= parameter_tol);

    for pair in cuts.windows(2) {
        let [lo, hi] = pair else {
            continue;
        };
        if hi - lo <= parameter_tol {
            continue;
        }
        let mid = (lo + hi) / 2.0;
        let mid_line = line_parameter(curve.eval(mid), line);
        if !line_range_contains(mid_line, line_range, tolerances) {
            continue;
        }
        let start_line = line_parameter(curve.eval(*lo), line).clamp(line_range.lo, line_range.hi);
        let end_line = line_parameter(curve.eval(*hi), line).clamp(line_range.lo, line_range.hi);
        overlaps.push(CurveCurveOverlap {
            a: ParamRange::new(start_line.min(end_line), start_line.max(end_line)),
            b: ParamRange::new(*lo, *hi),
            orientation: if end_line >= start_line {
                ParamOrientation::Same
            } else {
                ParamOrientation::Reversed
            },
        });
    }
}

fn line_parameter_root(
    line: &Line,
    curve: &NurbsCurve,
    a: Sample,
    b: Sample,
    target: f64,
    parameter_tol: f64,
    linear_tol: f64,
) -> Option<f64> {
    let mut lo = a.t_curve;
    let mut hi = b.t_curve;
    let mut f_lo = a.t_line - target;
    let f_hi = b.t_line - target;
    if f_lo.abs() <= linear_tol {
        return Some(lo);
    }
    if f_hi.abs() <= linear_tol {
        return Some(hi);
    }
    if (f_lo < 0.0 && f_hi < 0.0) || (f_lo > 0.0 && f_hi > 0.0) {
        return None;
    }
    let mut root = (lo + hi) / 2.0;
    for _ in 0..MAX_BISECTION_STEPS {
        root = (lo + hi) / 2.0;
        let f_mid = line_parameter(curve.eval(root), line) - target;
        if f_mid.abs() <= linear_tol || hi - lo <= parameter_tol {
            break;
        }
        if (f_lo < 0.0 && f_mid < 0.0) || (f_lo > 0.0 && f_mid > 0.0) {
            lo = root;
            f_lo = f_mid;
        } else {
            hi = root;
        }
    }
    Some(root)
}

fn sample_curve(line: &Line, curve: &NurbsCurve, curve_range: ParamRange) -> Vec<Sample> {
    let span_hint = curve
        .knots()
        .control_count()
        .saturating_sub(curve.degree())
        .max(1);
    let steps = (span_hint * curve.degree().max(1) * 32).clamp(MIN_STEPS, MAX_STEPS);
    (0..=steps)
        .map(|i| {
            let t_curve = curve_range.lerp(i as f64 / steps as f64);
            let point = curve.eval(t_curve);
            Sample {
                t_curve,
                distance: distance_to_line(point, line),
                t_line: line_parameter(point, line),
            }
        })
        .collect()
}

fn minimize_distance(
    line: &Line,
    curve: &NurbsCurve,
    mut lo: f64,
    mut hi: f64,
    parameter_tol: f64,
) -> f64 {
    for _ in 0..MAX_BISECTION_STEPS {
        if hi - lo <= parameter_tol {
            break;
        }
        let third = (hi - lo) / 3.0;
        let left = lo + third;
        let right = hi - third;
        let f_left = distance_to_line(curve.eval(left), line);
        let f_right = distance_to_line(curve.eval(right), line);
        if f_left <= f_right {
            hi = right;
        } else {
            lo = left;
        }
    }
    (lo + hi) / 2.0
}

fn push_root_candidate(
    line: &Line,
    line_range: ParamRange,
    curve: &NurbsCurve,
    t_curve: f64,
    forced_kind: Option<ContactKind>,
    points: &mut Vec<CurveCurvePoint>,
    tolerances: Tolerances,
) {
    let point = curve.eval(t_curve);
    if distance_to_line(point, line) > tolerances.linear() {
        return;
    }
    let t_line = line_parameter(point, line);
    if !line_range_contains(t_line, line_range, tolerances) {
        return;
    }
    let Some(point) = accept_curve_curve_candidate(
        line,
        t_line.clamp(line_range.lo, line_range.hi),
        curve,
        t_curve,
        forced_kind
            .map(|kind| forced_contact_kind(curve, t_curve, kind, tolerances))
            .unwrap_or_else(|| contact_kind(line, curve, t_curve, tolerances)),
        tolerances,
    ) else {
        return;
    };
    push_distinct_point(points, point, tolerances);
}

fn local_minimum_kind(
    line: &Line,
    curve: &NurbsCurve,
    lo: f64,
    hi: f64,
    tolerances: Tolerances,
) -> ContactKind {
    let a = perpendicular_to_line(curve.eval(lo), line);
    let b = perpendicular_to_line(curve.eval(hi), line);
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
    line: &Line,
    curve: &NurbsCurve,
    t_curve: f64,
    tolerances: Tolerances,
) -> ContactKind {
    let tangent = curve.eval_derivs(t_curve, 1).d[1];
    let tangent_norm = tangent.norm();
    if tangent_norm <= tolerances.linear() {
        ContactKind::Singular
    } else if tangent.cross(line.dir()).norm() > tangent_norm * tolerances.angular() {
        ContactKind::Transverse
    } else {
        ContactKind::Tangent
    }
}

fn push_distinct_point(
    points: &mut Vec<CurveCurvePoint>,
    candidate: CurveCurvePoint,
    tolerances: Tolerances,
) {
    if !points.iter().any(|point| {
        point.point.dist(candidate.point) <= tolerances.linear()
            || (point.t_a - candidate.t_a).abs() <= tolerances.linear()
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
    let parameter_tol = parameter_tolerance(global_range, tolerances);
    let mut merged: Vec<CurveCurveOverlap> = Vec::new();
    for overlap in overlaps.drain(..) {
        if let Some(last) = merged.last_mut()
            && last.orientation == overlap.orientation
            && overlap.b.lo <= last.b.hi + parameter_tol
        {
            last.a = ParamRange::new(last.a.lo.min(overlap.a.lo), last.a.hi.max(overlap.a.hi));
            last.b = ParamRange::new(last.b.lo, last.b.hi.max(overlap.b.hi));
            continue;
        }
        merged.push(overlap);
    }
    *overlaps = merged;
}

fn distance_to_line(point: Point3, line: &Line) -> f64 {
    perpendicular_to_line(point, line).norm()
}

fn perpendicular_to_line(point: Point3, line: &Line) -> Vec3 {
    let offset = point - line.origin();
    offset - line.dir() * offset.dot(line.dir())
}

fn line_parameter(point: Point3, line: &Line) -> f64 {
    (point - line.origin()).dot(line.dir())
}

fn line_range_contains(t_line: f64, line_range: ParamRange, tolerances: Tolerances) -> bool {
    t_line >= line_range.lo - tolerances.linear() && t_line <= line_range.hi + tolerances.linear()
}

fn clamp_to_domain(range: ParamRange, domain: ParamRange) -> ParamRange {
    ParamRange::new(
        range.lo.clamp(domain.lo, domain.hi),
        range.hi.clamp(domain.lo, domain.hi),
    )
}

fn parameter_tolerance(range: ParamRange, tolerances: Tolerances) -> f64 {
    (range.width().abs() * 1e-10)
        .max(tolerances.angular())
        .max(1e-12)
}

fn validate_ranges(
    curve: &NurbsCurve,
    line_range: ParamRange,
    curve_range: ParamRange,
    tolerances: Tolerances,
) -> Result<()> {
    if !line_range.is_finite()
        || !curve_range.is_finite()
        || line_range.width() < 0.0
        || curve_range.width() < 0.0
    {
        return Err(Error::InvalidGeometry {
            reason: "line/nurbs intersection requires finite non-reversed ranges",
        });
    }
    if !curve.knots().is_clamped() {
        return Err(Error::InvalidGeometry {
            reason: "line/nurbs intersection requires a clamped NURBS curve",
        });
    }
    let domain = curve.param_range();
    let parameter_tol = parameter_tolerance(domain, tolerances);
    if curve_range.lo < domain.lo - parameter_tol || curve_range.hi > domain.hi + parameter_tol {
        return Err(Error::InvalidGeometry {
            reason: "line/nurbs intersection curve range must lie within the NURBS domain",
        });
    }
    Ok(())
}
