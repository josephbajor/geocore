use super::circle_circle::intersect_bounded_circles;
use super::conic::{
    canonical_angle, ellipse_parameter, fit_periodic_parameter, parameter_tolerance,
    polynomial_derivative, real_polynomial_roots,
};
use super::line_ellipse::intersect_bounded_line_ellipse;
use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint, ParamOrientation,
    accept_curve_curve_candidate,
};
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::param::ParamRange;
use kgeom::project::project_to_curve;
use kgeom::vec::Point3;

struct EllipsePair<'a> {
    a: &'a Ellipse,
    range_a: ParamRange,
    b: &'a Ellipse,
    range_b: ParamRange,
    parameter_tol: f64,
    tolerances: Tolerances,
}

/// Intersect two ellipses restricted to finite parameter ranges.
///
/// Handles skew-plane contacts, coplanar secants/tangencies, periodic arc
/// filtering, tolerance-aware near tangencies, and coincident arc overlaps.
pub fn intersect_bounded_ellipses(
    a: &Ellipse,
    range_a: ParamRange,
    b: &Ellipse,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    validate_ranges(
        range_a,
        range_b,
        a.minor_radius(),
        b.minor_radius(),
        tolerances,
    )?;

    let normal_cross = a.frame().z().cross(b.frame().z());
    if normal_cross.norm() <= tolerances.angular() {
        return intersect_parallel_plane(a, range_a, b, range_b, tolerances);
    }

    intersect_plane_crossing(a, range_a, b, range_b, tolerances)
}

fn intersect_parallel_plane(
    a: &Ellipse,
    range_a: ParamRange,
    b: &Ellipse,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let center_delta = b.frame().origin() - a.frame().origin();
    if center_delta.dot(a.frame().z()).abs() > tolerances.linear() {
        return Ok(CurveCurveIntersections::default());
    }

    if ellipse_is_circle(a, tolerances) && ellipse_is_circle(b, tolerances) {
        let ca = Circle::new(*a.frame(), a.major_radius())?;
        let cb = Circle::new(*b.frame(), b.major_radius())?;
        return intersect_bounded_circles(&ca, range_a, &cb, range_b, tolerances);
    }

    if ellipses_are_coincident(a, b, tolerances) {
        return intersect_coincident_ellipses(a, range_a, b, range_b, tolerances);
    }

    intersect_coplanar_distinct(a, range_a, b, range_b, tolerances)
}

fn intersect_plane_crossing(
    a: &Ellipse,
    range_a: ParamRange,
    b: &Ellipse,
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
        center_parameter - a.major_radius() - tolerances.linear(),
        center_parameter + a.major_radius() + tolerances.linear(),
    );
    let line_hits = intersect_bounded_line_ellipse(&line, line_range, a, range_a, tolerances)?;

    let pair = EllipsePair {
        a,
        range_a,
        b,
        range_b,
        parameter_tol: ellipse_pair_parameter_tolerance(a, b, tolerances),
        tolerances,
    };
    let mut points = Vec::with_capacity(line_hits.points.len());
    for line_hit in line_hits.points {
        let point = a.eval(line_hit.t_b);
        push_candidate_from_point(&pair, point, &mut points);
    }
    CurveCurveIntersections::canonicalized(points, Vec::new())
}

fn intersect_coplanar_distinct(
    a: &Ellipse,
    range_a: ParamRange,
    b: &Ellipse,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let pair = EllipsePair {
        a,
        range_a,
        b,
        range_b,
        parameter_tol: ellipse_pair_parameter_tolerance(a, b, tolerances),
        tolerances,
    };
    let mut points = Vec::new();
    for (t_b, tangent_hint) in coplanar_candidate_parameters(a, b, tolerances) {
        push_projected_from_b(&pair, t_b, tangent_hint, &mut points);
    }
    for (t_a, tangent_hint) in coplanar_candidate_parameters(b, a, tolerances) {
        push_projected_from_a(&pair, t_a, tangent_hint, &mut points);
    }
    CurveCurveIntersections::canonicalized(points, Vec::new())
}

fn coplanar_candidate_parameters(
    target: &Ellipse,
    source: &Ellipse,
    tolerances: Tolerances,
) -> Vec<(f64, bool)> {
    let poly = ellipse_into_ellipse_quartic(target, source);
    let mut roots = Vec::new();
    for z in real_polynomial_roots(&poly) {
        push_parameter_candidate(&mut roots, 2.0 * kcore::math::atan(z), false);
    }
    for z in real_polynomial_roots(&polynomial_derivative(&poly)) {
        let t = canonical_angle(2.0 * kcore::math::atan(z));
        let point = source.eval(t);
        if project_to_curve(target, point, target.param_range())
            .is_some_and(|projection| projection.dist <= tolerances.linear())
        {
            push_parameter_candidate(&mut roots, t, true);
        }
    }
    let point = source.eval(core::f64::consts::PI);
    if let Some(projection) = project_to_curve(target, point, target.param_range())
        && projection.dist <= tolerances.linear()
    {
        push_parameter_candidate(
            &mut roots,
            core::f64::consts::PI,
            projection.dist > kcore::tolerance::LINEAR_RESOLUTION,
        );
    }
    roots
}

fn push_parameter_candidate(candidates: &mut Vec<(f64, bool)>, t: f64, tangent_hint: bool) {
    let t = canonical_angle(t);
    if let Some(existing) = candidates
        .iter_mut()
        .find(|(existing, _)| angular_distance(*existing, t) <= 1e-10)
    {
        existing.1 |= tangent_hint;
    } else {
        candidates.push((t, tangent_hint));
    }
}

fn angular_distance(a: f64, b: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let d = (a - b).abs();
    d.min(period - d)
}

fn ellipse_into_ellipse_quartic(target: &Ellipse, source: &Ellipse) -> Vec<f64> {
    let center = target.frame().to_local(source.frame().origin());
    let ux = source.frame().x().dot(target.frame().x()) * source.major_radius();
    let vx = source.frame().y().dot(target.frame().x()) * source.minor_radius();
    let uy = source.frame().x().dot(target.frame().y()) * source.major_radius();
    let vy = source.frame().y().dot(target.frame().y()) * source.minor_radius();
    let qx = [center.x + ux, 2.0 * vx, center.x - ux];
    let qy = [center.y + uy, 2.0 * vy, center.y - uy];
    let mut coeffs = [0.0; 5];
    add_scaled_square(
        &mut coeffs,
        qx,
        1.0 / (target.major_radius() * target.major_radius()),
    );
    add_scaled_square(
        &mut coeffs,
        qy,
        1.0 / (target.minor_radius() * target.minor_radius()),
    );
    coeffs[0] -= 1.0;
    coeffs[2] -= 2.0;
    coeffs[4] -= 1.0;
    coeffs.to_vec()
}

fn add_scaled_square(coeffs: &mut [f64; 5], q: [f64; 3], scale: f64) {
    coeffs[0] += scale * q[0] * q[0];
    coeffs[1] += scale * 2.0 * q[0] * q[1];
    coeffs[2] += scale * (2.0 * q[0] * q[2] + q[1] * q[1]);
    coeffs[3] += scale * 2.0 * q[1] * q[2];
    coeffs[4] += scale * q[2] * q[2];
}

fn intersect_coincident_ellipses(
    a: &Ellipse,
    range_a: ParamRange,
    b: &Ellipse,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let (sign, offset) = coincident_parameter_map(a, b);
    let orientation = if sign > 0.0 {
        ParamOrientation::Same
    } else {
        ParamOrientation::Reversed
    };
    let parameter_tol = ellipse_pair_parameter_tolerance(a, b, tolerances);
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
        let Some(t_b) = fit_periodic_parameter(raw_b, range_b, parameter_tol) else {
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

fn coincident_parameter_map(a: &Ellipse, b: &Ellipse) -> (f64, f64) {
    let b0 = ellipse_parameter(b.frame().to_local(a.eval(0.0)), b);
    let b1 = ellipse_parameter(b.frame().to_local(a.eval(core::f64::consts::FRAC_PI_2)), b);
    let delta = signed_periodic_delta(b1 - b0);
    let sign = if delta >= 0.0 { 1.0 } else { -1.0 };
    (sign, b0)
}

fn signed_periodic_delta(delta: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let mut d = delta % period;
    if d <= -core::f64::consts::PI {
        d += period;
    }
    if d > core::f64::consts::PI {
        d -= period;
    }
    d
}

fn push_projected_from_b(
    pair: &EllipsePair<'_>,
    t_b: f64,
    tangent_hint: bool,
    points: &mut Vec<CurveCurvePoint>,
) {
    let Some(t_b) = fit_periodic_parameter(t_b, pair.range_b, pair.parameter_tol) else {
        return;
    };
    let point_b = pair.b.eval(t_b);
    let Some(projection) = project_to_curve(pair.a, point_b, pair.a.param_range()) else {
        return;
    };
    if projection.dist > pair.tolerances.linear() {
        return;
    }
    let Some(t_a) = fit_periodic_parameter(projection.t, pair.range_a, pair.parameter_tol) else {
        return;
    };
    push_candidate_from_params(pair, t_a, t_b, tangent_hint, points);
}

fn push_projected_from_a(
    pair: &EllipsePair<'_>,
    t_a: f64,
    tangent_hint: bool,
    points: &mut Vec<CurveCurvePoint>,
) {
    let Some(t_a) = fit_periodic_parameter(t_a, pair.range_a, pair.parameter_tol) else {
        return;
    };
    let point_a = pair.a.eval(t_a);
    let Some(projection) = project_to_curve(pair.b, point_a, pair.b.param_range()) else {
        return;
    };
    if projection.dist > pair.tolerances.linear() {
        return;
    }
    let Some(t_b) = fit_periodic_parameter(projection.t, pair.range_b, pair.parameter_tol) else {
        return;
    };
    push_candidate_from_params(pair, t_a, t_b, tangent_hint, points);
}

fn push_candidate_from_point(
    pair: &EllipsePair<'_>,
    point: Point3,
    points: &mut Vec<CurveCurvePoint>,
) {
    let local_a = pair.a.frame().to_local(point);
    let raw_a = ellipse_parameter(local_a, pair.a);
    let Some(t_a) = fit_periodic_parameter(raw_a, pair.range_a, pair.parameter_tol) else {
        return;
    };
    let local_b = pair.b.frame().to_local(point);
    let raw_b = ellipse_parameter(local_b, pair.b);
    let Some(t_b) = fit_periodic_parameter(raw_b, pair.range_b, pair.parameter_tol) else {
        return;
    };
    push_candidate_from_params(pair, t_a, t_b, false, points);
}

fn push_candidate_from_params(
    pair: &EllipsePair<'_>,
    t_a: f64,
    t_b: f64,
    tangent_hint: bool,
    points: &mut Vec<CurveCurvePoint>,
) {
    let kind = if tangent_hint {
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

fn contact_kind(
    a: &Ellipse,
    t_a: f64,
    b: &Ellipse,
    t_b: f64,
    tolerances: Tolerances,
) -> ContactKind {
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

fn ellipses_are_coincident(a: &Ellipse, b: &Ellipse, tolerances: Tolerances) -> bool {
    a.frame().origin().dist(b.frame().origin()) <= tolerances.linear()
        && (a.major_radius() - b.major_radius()).abs() <= tolerances.linear()
        && (a.minor_radius() - b.minor_radius()).abs() <= tolerances.linear()
        && a.frame().z().cross(b.frame().z()).norm() <= tolerances.angular()
        && (ellipse_is_circle(a, tolerances)
            || a.frame().x().cross(b.frame().x()).norm() <= tolerances.angular())
}

fn ellipse_is_circle(ellipse: &Ellipse, tolerances: Tolerances) -> bool {
    (ellipse.major_radius() - ellipse.minor_radius()).abs() <= tolerances.linear()
}

fn ellipse_pair_parameter_tolerance(a: &Ellipse, b: &Ellipse, tolerances: Tolerances) -> f64 {
    parameter_tolerance(a.minor_radius(), tolerances)
        .max(parameter_tolerance(b.minor_radius(), tolerances))
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
    minor_a: f64,
    minor_b: f64,
    tolerances: Tolerances,
) -> Result<()> {
    if !range_a.is_finite()
        || !range_b.is_finite()
        || range_a.width() < 0.0
        || range_b.width() < 0.0
    {
        return Err(Error::InvalidGeometry {
            reason: "ellipse/ellipse intersection requires finite non-reversed ranges",
        });
    }
    if range_a.width() > core::f64::consts::TAU + parameter_tolerance(minor_a, tolerances)
        || range_b.width() > core::f64::consts::TAU + parameter_tolerance(minor_b, tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded ellipse ranges cannot span more than one period",
        });
    }
    Ok(())
}
