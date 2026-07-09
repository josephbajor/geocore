use super::line_circle::intersect_bounded_line_circle;
use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint, ParamOrientation,
    accept_curve_curve_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Line};
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};

struct CirclePair<'a> {
    a: &'a Circle,
    range_a: ParamRange,
    b: &'a Circle,
    range_b: ParamRange,
    parameter_tol: f64,
    tolerances: Tolerances,
}

/// Intersect two circles restricted to finite parameter ranges.
///
/// Handles coplanar secants/tangencies, skew-plane contacts, periodic arc
/// filtering, and positive-length coincident arc overlaps.
pub fn intersect_bounded_circles(
    a: &Circle,
    range_a: ParamRange,
    b: &Circle,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    validate_ranges(range_a, range_b, a.radius(), b.radius(), tolerances)?;

    let normal_cross = a.frame().z().cross(b.frame().z());
    if normal_cross.norm() <= tolerances.angular() {
        return intersect_parallel_plane_circles(a, range_a, b, range_b, tolerances);
    }

    intersect_plane_crossing_circles(a, range_a, b, range_b, tolerances)
}

fn intersect_parallel_plane_circles(
    a: &Circle,
    range_a: ParamRange,
    b: &Circle,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let center_delta = b.frame().origin() - a.frame().origin();
    if center_delta.dot(a.frame().z()).abs() > tolerances.linear() {
        return Ok(CurveCurveIntersections::default());
    }

    let local_b_center = a.frame().to_local(b.frame().origin());
    let center_distance =
        (local_b_center.x * local_b_center.x + local_b_center.y * local_b_center.y).sqrt();
    let radius_delta = (a.radius() - b.radius()).abs();
    if center_distance + radius_delta <= tolerances.linear() {
        return intersect_coincident_circles(a, range_a, b, range_b, tolerances);
    }
    if center_distance <= tolerances.linear() {
        return Ok(CurveCurveIntersections::default());
    }

    intersect_coplanar_distinct_circles(
        a,
        range_a,
        b,
        range_b,
        local_b_center,
        center_distance,
        tolerances,
    )
}

fn intersect_coplanar_distinct_circles(
    a: &Circle,
    range_a: ParamRange,
    b: &Circle,
    range_b: ParamRange,
    local_b_center: Vec3,
    center_distance: f64,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let ra = a.radius();
    let rb = b.radius();
    let tangent = (center_distance - (ra + rb)).abs() <= tolerances.linear()
        || (center_distance - (ra - rb).abs()).abs() <= tolerances.linear();
    if center_distance > ra + rb + tolerances.linear()
        || center_distance < (ra - rb).abs() - tolerances.linear()
    {
        return Ok(CurveCurveIntersections::default());
    }

    let axis = Vec3::new(
        local_b_center.x / center_distance,
        local_b_center.y / center_distance,
        0.0,
    );
    let perp = Vec3::new(-axis.y, axis.x, 0.0);
    let along = (center_distance * center_distance + ra * ra - rb * rb) / (2.0 * center_distance);
    let height_sq = ra * ra - along * along;
    let offsets = if tangent || height_sq <= 0.0 {
        vec![0.0]
    } else {
        let height = height_sq.sqrt();
        vec![-height, height]
    };

    let parameter_tol = circle_pair_parameter_tolerance(a, b, tolerances);
    let pair = CirclePair {
        a,
        range_a,
        b,
        range_b,
        parameter_tol,
        tolerances,
    };
    let mut points = Vec::with_capacity(offsets.len());
    for offset in offsets {
        let local = axis * along + perp * offset;
        let point = a.frame().point_at(local.x, local.y, 0.0);
        push_candidate_from_point(
            &pair,
            point,
            if tangent {
                ContactKind::Tangent
            } else {
                ContactKind::Transverse
            },
            &mut points,
        );
    }
    CurveCurveIntersections::canonicalized(points, Vec::new())
}

fn intersect_plane_crossing_circles(
    a: &Circle,
    range_a: ParamRange,
    b: &Circle,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let n1 = a.frame().z();
    let n2 = b.frame().z();
    let direction = n1.cross(n2);
    let denom = direction.norm_sq();
    let c1 = n1.dot(a.frame().origin());
    let c2 = n2.dot(b.frame().origin());
    let origin = ((n2 * c1 - n1 * c2).cross(direction)) / denom;
    let line = Line::new(origin, direction)?;
    let center_parameter = line.dir().dot(a.frame().origin() - line.origin());
    let line_range = ParamRange::new(
        center_parameter - a.radius() - tolerances.linear(),
        center_parameter + a.radius() + tolerances.linear(),
    );
    let line_circle_hits =
        intersect_bounded_line_circle(&line, line_range, a, range_a, tolerances)?;

    let parameter_tol = circle_pair_parameter_tolerance(a, b, tolerances);
    let mut points = Vec::with_capacity(line_circle_hits.points.len());
    for line_hit in line_circle_hits.points {
        let t_a = line_hit.t_b;
        let point = a.eval(t_a);
        let local_b = b.frame().to_local(point);
        let raw_b = math::atan2(local_b.y, local_b.x);
        let Some(t_b) = fit_circle_parameter(raw_b, range_b, parameter_tol) else {
            continue;
        };
        let kind = contact_kind(a, t_a, b, t_b, tolerances);
        if let Some(point) = accept_curve_curve_candidate(a, t_a, b, t_b, kind, tolerances) {
            push_distinct(&mut points, point, tolerances);
        }
    }
    CurveCurveIntersections::canonicalized(points, Vec::new())
}

fn intersect_coincident_circles(
    a: &Circle,
    range_a: ParamRange,
    b: &Circle,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let same_normal = a.frame().z().dot(b.frame().z()) >= 0.0;
    let alpha = math::atan2(
        a.frame().y().dot(b.frame().x()),
        a.frame().x().dot(b.frame().x()),
    );
    let (sign, offset, orientation) = if same_normal {
        (1.0, -alpha, ParamOrientation::Same)
    } else {
        (-1.0, alpha, ParamOrientation::Reversed)
    };
    let parameter_tol = circle_pair_parameter_tolerance(a, b, tolerances);
    let period = core::f64::consts::TAU;

    let mapped_lo = if sign > 0.0 {
        range_a.lo + offset
    } else {
        offset - range_a.hi
    };
    let mapped_hi = if sign > 0.0 {
        range_a.hi + offset
    } else {
        offset - range_a.lo
    };
    let k_min = ((range_b.lo - mapped_hi - parameter_tol) / period).ceil() as i64;
    let k_max = ((range_b.hi - mapped_lo + parameter_tol) / period).floor() as i64;

    let mut overlaps = Vec::new();
    let mut point_parameters = Vec::new();
    for k in k_min..=k_max {
        let shift = k as f64 * period;
        let inverse = if sign > 0.0 {
            ParamRange::new(range_b.lo - offset - shift, range_b.hi - offset - shift)
        } else {
            ParamRange::new(offset + shift - range_b.hi, offset + shift - range_b.lo)
        };
        let lo = range_a.lo.max(inverse.lo);
        let hi = range_a.hi.min(inverse.hi);
        if hi < lo - parameter_tol {
            continue;
        }
        let lo = lo.clamp(range_a.lo, range_a.hi);
        let hi = hi.clamp(range_a.lo, range_a.hi);
        if hi - lo > parameter_tol {
            let b0 = sign * lo + offset + shift;
            let b1 = sign * hi + offset + shift;
            overlaps.push(CurveCurveOverlap {
                a: ParamRange::new(lo, hi),
                b: ParamRange::new(b0.min(b1), b0.max(b1)),
                orientation,
            });
        } else {
            point_parameters.push(((lo + hi) / 2.0).clamp(range_a.lo, range_a.hi));
        }
    }

    let mut points = Vec::new();
    for t_a in point_parameters {
        if overlaps.iter().any(|overlap| {
            overlap.a.lo - parameter_tol <= t_a && t_a <= overlap.a.hi + parameter_tol
        }) {
            continue;
        }
        let raw_b = sign * t_a + offset;
        let Some(t_b) = fit_circle_parameter(raw_b, range_b, parameter_tol) else {
            continue;
        };
        if let Some(point) =
            accept_curve_curve_candidate(a, t_a, b, t_b, ContactKind::Tangent, tolerances)
        {
            push_distinct(&mut points, point, tolerances);
        }
    }

    CurveCurveIntersections::canonicalized(points, overlaps)
}

fn push_candidate_from_point(
    pair: &CirclePair<'_>,
    point: Point3,
    fallback_kind: ContactKind,
    points: &mut Vec<CurveCurvePoint>,
) {
    let local_a = pair.a.frame().to_local(point);
    let raw_a = math::atan2(local_a.y, local_a.x);
    let Some(t_a) = fit_circle_parameter(raw_a, pair.range_a, pair.parameter_tol) else {
        return;
    };
    let local_b = pair.b.frame().to_local(point);
    let raw_b = math::atan2(local_b.y, local_b.x);
    let Some(t_b) = fit_circle_parameter(raw_b, pair.range_b, pair.parameter_tol) else {
        return;
    };
    let kind = if fallback_kind == ContactKind::Tangent {
        ContactKind::Tangent
    } else {
        contact_kind(pair.a, t_a, pair.b, t_b, pair.tolerances)
    };
    if let Some(point) =
        accept_curve_curve_candidate(pair.a, t_a, pair.b, t_b, kind, pair.tolerances)
    {
        push_distinct(points, point, pair.tolerances);
    }
}

fn contact_kind(a: &Circle, t_a: f64, b: &Circle, t_b: f64, tolerances: Tolerances) -> ContactKind {
    let da = a.eval_derivs(t_a, 1).d[1];
    let db = b.eval_derivs(t_b, 1).d[1];
    let Some(ua) = da.normalized() else {
        return ContactKind::Singular;
    };
    let Some(ub) = db.normalized() else {
        return ContactKind::Singular;
    };
    if ua.cross(ub).norm() <= tolerances.angular() {
        ContactKind::Tangent
    } else {
        ContactKind::Transverse
    }
}

fn fit_circle_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    let period = core::f64::consts::TAU;
    let k_min = ((range.lo - tolerance - candidate) / period).ceil() as i64;
    let k_max = ((range.hi + tolerance - candidate) / period).floor() as i64;
    if k_min > k_max {
        return None;
    }
    Some((candidate + k_min as f64 * period).clamp(range.lo, range.hi))
}

fn circle_pair_parameter_tolerance(a: &Circle, b: &Circle, tolerances: Tolerances) -> f64 {
    circle_parameter_tolerance(a.radius(), tolerances)
        .max(circle_parameter_tolerance(b.radius(), tolerances))
}

fn circle_parameter_tolerance(radius: f64, tolerances: Tolerances) -> f64 {
    (tolerances.linear() / radius).max(tolerances.angular())
}

fn push_distinct(
    points: &mut Vec<CurveCurvePoint>,
    candidate: CurveCurvePoint,
    tolerances: Tolerances,
) {
    if !points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        points.push(candidate);
    }
}

fn validate_ranges(
    range_a: ParamRange,
    range_b: ParamRange,
    radius_a: f64,
    radius_b: f64,
    tolerances: Tolerances,
) -> Result<()> {
    if !range_a.is_finite()
        || !range_b.is_finite()
        || range_a.width() < 0.0
        || range_b.width() < 0.0
    {
        return Err(Error::InvalidGeometry {
            reason: "circle/circle intersection requires finite non-reversed ranges",
        });
    }
    if range_a.width() > core::f64::consts::TAU + circle_parameter_tolerance(radius_a, tolerances)
        || range_b.width()
            > core::f64::consts::TAU + circle_parameter_tolerance(radius_b, tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded circle ranges cannot span more than one period",
        });
    }
    Ok(())
}
