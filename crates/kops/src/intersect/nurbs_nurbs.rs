use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint, ParamOrientation,
    accept_curve_curve_candidate,
};
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

const MIN_STEPS: usize = 96;
const MAX_STEPS: usize = 384;
const MAX_POLISH_STEPS: usize = 32;
const MAX_MINIMIZE_STEPS: usize = 80;
const OVERLAP_SAMPLES: usize = 32;

#[derive(Debug, Clone, Copy)]
struct Sample {
    t: f64,
    point: Point3,
}

/// Intersect two clamped NURBS curves restricted to finite ranges.
///
/// This is the first general NURBS/NURBS curve bridge: it discovers candidate
/// contacts from closest sampled segment pairs, polishes them by Newton
/// iteration on the two curve parameters, and reports simple contained spans
/// when the first curve range is proven to lie on the second.
pub fn intersect_bounded_nurbs_nurbs(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    validate_ranges(a, range_a, b, range_b, tolerances)?;

    let range_a = clamp_to_domain(range_a, a.param_range());
    let range_b = clamp_to_domain(range_b, b.param_range());
    if range_a.width() <= parameter_tolerance(range_a, tolerances)
        || range_b.width() <= parameter_tolerance(range_b, tolerances)
    {
        return degenerate_range_intersections(a, range_a, b, range_b, tolerances);
    }

    if let Some(overlap) = contained_overlap(a, range_a, b, range_b, tolerances) {
        return CurveCurveIntersections::canonicalized(Vec::new(), vec![overlap]);
    }

    let samples_a = sample_curve(a, range_a);
    let samples_b = sample_curve(b, range_b);
    let seed_tol = seed_tolerance(&samples_a, &samples_b, tolerances);
    let mut points = Vec::new();
    for pair_a in samples_a.windows(2) {
        let [a0, a1] = pair_a else {
            continue;
        };
        for pair_b in samples_b.windows(2) {
            let [b0, b1] = pair_b else {
                continue;
            };
            let (s, t, distance) =
                closest_segment_parameters(a0.point, a1.point, b0.point, b1.point);
            if distance > seed_tol {
                continue;
            }
            let t_a = a0.t + (a1.t - a0.t) * s;
            let t_b = b0.t + (b1.t - b0.t) * t;
            if let Some((t_a, t_b)) = polish_candidate(a, b, t_a, t_b, range_a, range_b, tolerances)
            {
                push_root_candidate(a, t_a, b, t_b, &mut points, tolerances);
            }
        }
    }

    CurveCurveIntersections::canonicalized(points, Vec::new())
}

fn degenerate_range_intersections(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let t_a = range_a.lo;
    let t_b = closest_parameter_to_point(b, range_b, a.eval(t_a));
    let mut points = Vec::new();
    push_root_candidate(a, t_a, b, t_b, &mut points, tolerances);
    CurveCurveIntersections::canonicalized(points, Vec::new())
}

fn contained_overlap(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Option<CurveCurveOverlap> {
    let mut mapped = Vec::with_capacity(OVERLAP_SAMPLES + 1);
    for i in 0..=OVERLAP_SAMPLES {
        let t_a = range_a.lerp(i as f64 / OVERLAP_SAMPLES as f64);
        let point = a.eval(t_a);
        let t_b = closest_parameter_to_point(b, range_b, point);
        if point.dist(b.eval(t_b)) > tolerances.linear() {
            return None;
        }
        mapped.push(t_b);
    }

    let parameter_tol = parameter_tolerance(range_b, tolerances);
    let increasing = mapped
        .windows(2)
        .all(|pair| pair[1] + parameter_tol >= pair[0]);
    let decreasing = mapped
        .windows(2)
        .all(|pair| pair[0] + parameter_tol >= pair[1]);
    if !increasing && !decreasing {
        return None;
    }

    let first = snap_to_range_bounds(mapped[0], range_b, parameter_tol);
    let last = snap_to_range_bounds(mapped[mapped.len() - 1], range_b, parameter_tol);
    if (last - first).abs() <= parameter_tol {
        return None;
    }
    Some(CurveCurveOverlap {
        a: range_a,
        b: ParamRange::new(first.min(last), first.max(last)),
        orientation: if last >= first {
            ParamOrientation::Same
        } else {
            ParamOrientation::Reversed
        },
    })
}

fn sample_curve(curve: &NurbsCurve, range: ParamRange) -> Vec<Sample> {
    let span_hint = curve
        .knots()
        .control_count()
        .saturating_sub(curve.degree())
        .max(1);
    let steps = (span_hint * curve.degree().max(1) * 32).clamp(MIN_STEPS, MAX_STEPS);
    (0..=steps)
        .map(|i| {
            let t = range.lerp(i as f64 / steps as f64);
            Sample {
                t,
                point: curve.eval(t),
            }
        })
        .collect()
}

fn seed_tolerance(a: &[Sample], b: &[Sample], tolerances: Tolerances) -> f64 {
    let chord_a = max_chord(a);
    let chord_b = max_chord(b);
    tolerances
        .linear()
        .max((chord_a.max(chord_b)).sqrt() * tolerances.linear().sqrt())
        .max((chord_a + chord_b) * 0.25)
}

fn max_chord(samples: &[Sample]) -> f64 {
    samples
        .windows(2)
        .map(|pair| pair[0].point.dist(pair[1].point))
        .fold(0.0, f64::max)
}

fn closest_segment_parameters(p0: Point3, p1: Point3, q0: Point3, q1: Point3) -> (f64, f64, f64) {
    let d1 = p1 - p0;
    let d2 = q1 - q0;
    let r = p0 - q0;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let (s, t) = if a <= 1e-30 && e <= 1e-30 {
        (0.0, 0.0)
    } else if a <= 1e-30 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= 1e-30 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s = if denom.abs() > 1e-30 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    let p = p0 + d1 * s;
    let q = q0 + d2 * t;
    (s, t, p.dist(q))
}

fn polish_candidate(
    a: &NurbsCurve,
    b: &NurbsCurve,
    t_a: f64,
    t_b: f64,
    range_a: ParamRange,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Option<(f64, f64)> {
    let (mut t_a, mut t_b) = newton_polish_pair(a, b, t_a, t_b, range_a, range_b, tolerances);
    let distance = a.eval(t_a).dist(b.eval(t_b));
    if distance <= tolerances.linear() * 16.0 {
        let (refined_a, refined_b) = refine_local_pair(a, b, t_a, t_b, range_a, range_b);
        if a.eval(refined_a).dist(b.eval(refined_b)) < distance {
            (t_a, t_b) =
                newton_polish_pair(a, b, refined_a, refined_b, range_a, range_b, tolerances);
        }
    }
    Some((t_a, t_b))
}

fn newton_polish_pair(
    a: &NurbsCurve,
    b: &NurbsCurve,
    mut t_a: f64,
    mut t_b: f64,
    range_a: ParamRange,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> (f64, f64) {
    let gradient_tol = (tolerances.linear() * tolerances.linear() * tolerances.linear()).max(1e-30);
    for _ in 0..MAX_POLISH_STEPS {
        let da = a.eval_derivs(t_a, 2);
        let db = b.eval_derivs(t_b, 2);
        let r = da.d[0] - db.d[0];
        let g0 = r.dot(da.d[1]);
        let g1 = -r.dot(db.d[1]);
        if g0.abs().max(g1.abs()) <= gradient_tol {
            break;
        }

        let h00 = da.d[1].dot(da.d[1]) + r.dot(da.d[2]);
        let h01 = -da.d[1].dot(db.d[1]);
        let h11 = db.d[1].dot(db.d[1]) - r.dot(db.d[2]);
        let det = h00 * h11 - h01 * h01;
        let (step_a, step_b) = if det.abs() > 1e-24 {
            ((-g0 * h11 + h01 * g1) / det, (h01 * g0 - h00 * g1) / det)
        } else {
            break;
        };

        let old = r.norm_sq();
        let mut scale = 1.0;
        let mut accepted = false;
        for _ in 0..16 {
            let next_a = (t_a + step_a * scale).clamp(range_a.lo, range_a.hi);
            let next_b = (t_b + step_b * scale).clamp(range_b.lo, range_b.hi);
            let next = a.eval(next_a).dist(b.eval(next_b));
            if next * next <= old {
                accepted = true;
                t_a = next_a;
                t_b = next_b;
                break;
            }
            scale *= 0.5;
        }
        if !accepted || step_a.abs().max(step_b.abs()) * scale <= 1e-12 {
            break;
        }
    }
    (t_a, t_b)
}

fn refine_local_pair(
    a: &NurbsCurve,
    b: &NurbsCurve,
    t_a: f64,
    t_b: f64,
    range_a: ParamRange,
    range_b: ParamRange,
) -> (f64, f64) {
    let width_a = (range_a.width() / MIN_STEPS as f64 * 2.0)
        .max(parameter_tolerance(range_a, Tolerances::default()));
    let width_b = (range_b.width() / MIN_STEPS as f64 * 2.0)
        .max(parameter_tolerance(range_b, Tolerances::default()));

    let a0 = minimize_curve_to_curve_distance(
        a,
        b,
        ParamRange::new(
            (t_a - width_a).max(range_a.lo),
            (t_a + width_a).min(range_a.hi),
        ),
        range_b,
    );
    let b0 = closest_parameter_to_point(b, range_b, a.eval(a0));

    let b1 = minimize_curve_to_curve_distance(
        b,
        a,
        ParamRange::new(
            (t_b - width_b).max(range_b.lo),
            (t_b + width_b).min(range_b.hi),
        ),
        range_a,
    );
    let a1 = closest_parameter_to_point(a, range_a, b.eval(b1));

    if a.eval(a0).dist(b.eval(b0)) <= a.eval(a1).dist(b.eval(b1)) {
        (a0, b0)
    } else {
        (a1, b1)
    }
}

fn minimize_curve_to_curve_distance(
    curve: &NurbsCurve,
    other: &NurbsCurve,
    mut range: ParamRange,
    other_range: ParamRange,
) -> f64 {
    for _ in 0..MAX_MINIMIZE_STEPS {
        if range.width() <= 1e-12 {
            break;
        }
        let third = range.width() / 3.0;
        let left = range.lo + third;
        let right = range.hi - third;
        let f_left = distance_from_point_to_curve(curve.eval(left), other, other_range);
        let f_right = distance_from_point_to_curve(curve.eval(right), other, other_range);
        if (f_left - f_right).abs() <= 1e-18 {
            range = ParamRange::new(left, right);
        } else if f_left < f_right {
            range = ParamRange::new(range.lo, right);
        } else {
            range = ParamRange::new(left, range.hi);
        }
    }
    range.lerp(0.5)
}

fn distance_from_point_to_curve(point: Point3, curve: &NurbsCurve, range: ParamRange) -> f64 {
    let t = closest_parameter_to_point(curve, range, point);
    point.dist(curve.eval(t))
}

fn push_root_candidate(
    a: &NurbsCurve,
    t_a: f64,
    b: &NurbsCurve,
    t_b: f64,
    points: &mut Vec<CurveCurvePoint>,
    tolerances: Tolerances,
) {
    if a.eval(t_a).dist(b.eval(t_b)) > tolerances.linear() {
        return;
    }
    let Some(point) = accept_curve_curve_candidate(
        a,
        t_a,
        b,
        t_b,
        contact_kind(a, t_a, b, t_b, tolerances),
        tolerances,
    ) else {
        return;
    };
    push_distinct_point(points, point, tolerances);
}

fn contact_kind(
    a: &NurbsCurve,
    t_a: f64,
    b: &NurbsCurve,
    t_b: f64,
    tolerances: Tolerances,
) -> ContactKind {
    let da = a.eval_derivs(t_a, 1).d[1];
    let db = b.eval_derivs(t_b, 1).d[1];
    let scale = da.norm() * db.norm();
    if scale <= tolerances.linear() {
        ContactKind::Singular
    } else if da.cross(db).norm() > scale * working_angular_tolerance(tolerances) {
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
    if let Some(point) = points
        .iter_mut()
        .find(|point| duplicate_point(point, &candidate, tolerances))
    {
        if better_representative(&candidate, point, tolerances) {
            *point = candidate;
        }
    } else {
        points.push(candidate);
    }
}

fn duplicate_point(
    point: &CurveCurvePoint,
    candidate: &CurveCurvePoint,
    tolerances: Tolerances,
) -> bool {
    let spatial_tol =
        if point.kind == ContactKind::Tangent || candidate.kind == ContactKind::Tangent {
            tolerances.linear().sqrt()
        } else {
            tolerances.linear()
        };
    point.point.dist(candidate.point) <= spatial_tol
        || (point.t_a - candidate.t_a).abs() <= working_angular_tolerance(tolerances)
            && (point.t_b - candidate.t_b).abs() <= working_angular_tolerance(tolerances)
}

fn better_representative(
    candidate: &CurveCurvePoint,
    point: &CurveCurvePoint,
    tolerances: Tolerances,
) -> bool {
    candidate.residual + tolerances.linear() * 1e-6 < point.residual
        || candidate.kind > point.kind && candidate.residual <= point.residual + tolerances.linear()
}

fn working_angular_tolerance(tolerances: Tolerances) -> f64 {
    tolerances.angular().max(tolerances.linear().sqrt())
}

fn closest_parameter_to_point(curve: &NurbsCurve, range: ParamRange, point: Point3) -> f64 {
    let samples = sample_curve(curve, range);
    let (best_idx, _) = samples
        .iter()
        .enumerate()
        .map(|(i, sample)| (i, sample.point.dist(point)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .expect("sample_curve always returns at least one sample");
    let lo = samples[best_idx.saturating_sub(1)].t;
    let hi = samples[(best_idx + 1).min(samples.len() - 1)].t;
    minimize_point_distance(curve, lo, hi, point)
}

fn minimize_point_distance(curve: &NurbsCurve, mut lo: f64, mut hi: f64, point: Point3) -> f64 {
    for _ in 0..MAX_MINIMIZE_STEPS {
        if hi - lo <= 1e-12 {
            break;
        }
        let third = (hi - lo) / 3.0;
        let left = lo + third;
        let right = hi - third;
        let f_left = curve.eval(left).dist(point);
        let f_right = curve.eval(right).dist(point);
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

fn clamp_to_domain(range: ParamRange, domain: ParamRange) -> ParamRange {
    ParamRange::new(
        range.lo.clamp(domain.lo, domain.hi),
        range.hi.clamp(domain.lo, domain.hi),
    )
}

fn snap_to_range_bounds(t: f64, range: ParamRange, tolerance: f64) -> f64 {
    if (t - range.lo).abs() <= tolerance {
        range.lo
    } else if (t - range.hi).abs() <= tolerance {
        range.hi
    } else {
        t.clamp(range.lo, range.hi)
    }
}

fn parameter_tolerance(range: ParamRange, tolerances: Tolerances) -> f64 {
    (range.width().abs() * 1e-10)
        .max(tolerances.angular())
        .max(1e-12)
}

fn validate_ranges(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<()> {
    if !range_a.is_finite()
        || !range_b.is_finite()
        || range_a.width() < 0.0
        || range_b.width() < 0.0
    {
        return Err(Error::InvalidGeometry {
            reason: "nurbs/nurbs intersection requires finite non-reversed ranges",
        });
    }
    if !a.knots().is_clamped() || !b.knots().is_clamped() {
        return Err(Error::InvalidGeometry {
            reason: "nurbs/nurbs intersection requires clamped NURBS curves",
        });
    }
    let domain_a = a.param_range();
    let domain_b = b.param_range();
    let parameter_tol_a = parameter_tolerance(domain_a, tolerances);
    let parameter_tol_b = parameter_tolerance(domain_b, tolerances);
    if range_a.lo < domain_a.lo - parameter_tol_a
        || range_a.hi > domain_a.hi + parameter_tol_a
        || range_b.lo < domain_b.lo - parameter_tol_b
        || range_b.hi > domain_b.hi + parameter_tol_b
    {
        return Err(Error::InvalidGeometry {
            reason: "nurbs/nurbs intersection ranges must lie within the NURBS domains",
        });
    }
    Ok(())
}
