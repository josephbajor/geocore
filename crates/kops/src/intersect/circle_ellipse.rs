use super::circle_circle::intersect_bounded_circles;
use super::conic::{
    canonical_angle, ellipse_parameter, fit_periodic_parameter, parameter_tolerance,
    push_angle_root, real_polynomial_roots,
};
use super::line_circle::intersect_bounded_line_circle;
use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurvePoint, accept_curve_curve_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};

struct CircleEllipsePair<'a> {
    circle: &'a Circle,
    circle_range: ParamRange,
    ellipse: &'a Ellipse,
    ellipse_range: ParamRange,
    parameter_tol: f64,
    tolerances: Tolerances,
}

/// Intersect a circle and ellipse restricted to finite parameter ranges.
///
/// Handles skew-plane contacts, coplanar secants/tangencies, periodic arc
/// filtering, tolerance-aware near tangencies, and the exact circle-as-ellipse
/// coincident case.
pub fn intersect_bounded_circle_ellipse(
    circle: &Circle,
    circle_range: ParamRange,
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    validate_ranges(
        circle_range,
        ellipse_range,
        circle.radius(),
        ellipse.minor_radius(),
        tolerances,
    )?;

    let normal_cross = circle.frame().z().cross(ellipse.frame().z());
    if normal_cross.norm() <= tolerances.angular() {
        return intersect_parallel_plane(circle, circle_range, ellipse, ellipse_range, tolerances);
    }

    intersect_plane_crossing(circle, circle_range, ellipse, ellipse_range, tolerances)
}

fn intersect_parallel_plane(
    circle: &Circle,
    circle_range: ParamRange,
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let center_delta = circle.frame().origin() - ellipse.frame().origin();
    if center_delta.dot(ellipse.frame().z()).abs() > tolerances.linear() {
        return Ok(CurveCurveIntersections::complete_empty());
    }

    if ellipse_is_circle(ellipse, tolerances)
        && circle.frame().origin().dist(ellipse.frame().origin()) <= tolerances.linear()
        && (circle.radius() - ellipse.major_radius()).abs() <= tolerances.linear()
    {
        let ellipse_circle = Circle::new(*ellipse.frame(), ellipse.major_radius())?;
        return intersect_bounded_circles(
            circle,
            circle_range,
            &ellipse_circle,
            ellipse_range,
            tolerances,
        );
    }

    intersect_coplanar_distinct(circle, circle_range, ellipse, ellipse_range, tolerances)
}

fn intersect_plane_crossing(
    circle: &Circle,
    circle_range: ParamRange,
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let n1 = circle.frame().z();
    let n2 = ellipse.frame().z();
    let direction = n1.cross(n2);
    let denom = direction.norm_sq();
    let c1 = n1.dot(circle.frame().origin());
    let c2 = n2.dot(ellipse.frame().origin());
    let origin = ((n2 * c1 - n1 * c2).cross(direction)) / denom;
    let line = Line::new(origin, direction)?;
    let center_parameter = line.dir().dot(circle.frame().origin() - line.origin());
    let line_range = ParamRange::new(
        center_parameter - circle.radius() - tolerances.linear(),
        center_parameter + circle.radius() + tolerances.linear(),
    );
    let line_circle_hits =
        intersect_bounded_line_circle(&line, line_range, circle, circle_range, tolerances)?;

    let pair = CircleEllipsePair {
        circle,
        circle_range,
        ellipse,
        ellipse_range,
        parameter_tol: circle_ellipse_parameter_tolerance(circle, ellipse, tolerances),
        tolerances,
    };
    let mut points = Vec::with_capacity(line_circle_hits.points.len());
    for line_hit in line_circle_hits.points {
        let point = circle.eval(line_hit.t_b);
        push_candidate_from_point(&pair, point, None, &mut points);
    }
    CurveCurveIntersections::canonicalized_complete(points, Vec::new())
}

fn intersect_coplanar_distinct(
    circle: &Circle,
    circle_range: ParamRange,
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let pair = CircleEllipsePair {
        circle,
        circle_range,
        ellipse,
        ellipse_range,
        parameter_tol: circle_ellipse_parameter_tolerance(circle, ellipse, tolerances),
        tolerances,
    };
    let center = ellipse.frame().to_local(circle.frame().origin());
    let roots = coplanar_roots(ellipse, center, circle.radius(), tolerances);
    let mut points = Vec::with_capacity(roots.len());
    for t_ellipse in roots {
        let point = ellipse.eval(t_ellipse);
        push_candidate_from_point(&pair, point, None, &mut points);
    }
    CurveCurveIntersections::canonicalized_complete(points, Vec::new())
}

fn coplanar_roots(
    ellipse: &Ellipse,
    circle_center_in_ellipse: Vec3,
    circle_radius: f64,
    tolerances: Tolerances,
) -> Vec<f64> {
    let major = ellipse.major_radius();
    let minor = ellipse.minor_radius();
    let cx = circle_center_in_ellipse.x;
    let cy = circle_center_in_ellipse.y;
    let roots_poly = circle_ellipse_quartic(major, minor, cx, cy, circle_radius);
    let extrema_poly = circle_ellipse_extrema_quartic(major, minor, cx, cy);
    let mut roots = Vec::new();

    for z in real_polynomial_roots(&roots_poly) {
        push_angle_root(&mut roots, 2.0 * math::atan(z));
    }

    for z in real_polynomial_roots(&extrema_poly) {
        let t = canonical_angle(2.0 * math::atan(z));
        if circle_ellipse_radial_residual(ellipse, circle_center_in_ellipse, circle_radius, t)
            <= tolerances.linear()
        {
            push_angle_root(&mut roots, t);
        }
    }

    for t in [0.0, core::f64::consts::PI] {
        if circle_ellipse_radial_residual(ellipse, circle_center_in_ellipse, circle_radius, t)
            <= tolerances.linear()
        {
            push_angle_root(&mut roots, t);
        }
    }
    roots
}

fn circle_ellipse_quartic(
    major: f64,
    minor: f64,
    cx: f64,
    cy: f64,
    circle_radius: f64,
) -> Vec<f64> {
    let x0 = major - cx;
    let x2 = -major - cx;
    let y0 = -cy;
    let y1 = 2.0 * minor;
    let y2 = -cy;
    vec![
        x0 * x0 + y0 * y0 - circle_radius * circle_radius,
        2.0 * y0 * y1,
        2.0 * x0 * x2 + y1 * y1 + 2.0 * y0 * y2 - 2.0 * circle_radius * circle_radius,
        2.0 * y1 * y2,
        x2 * x2 + y2 * y2 - circle_radius * circle_radius,
    ]
}

fn circle_ellipse_extrema_quartic(major: f64, minor: f64, cx: f64, cy: f64) -> Vec<f64> {
    let k = minor * minor - major * major;
    vec![
        -minor * cy,
        2.0 * (k + major * cx),
        0.0,
        2.0 * (major * cx - k),
        minor * cy,
    ]
}

fn circle_ellipse_radial_residual(
    ellipse: &Ellipse,
    circle_center_in_ellipse: Vec3,
    circle_radius: f64,
    t: f64,
) -> f64 {
    let p = ellipse.frame().to_local(ellipse.eval(t));
    let dx = p.x - circle_center_in_ellipse.x;
    let dy = p.y - circle_center_in_ellipse.y;
    ((dx * dx + dy * dy).sqrt() - circle_radius).abs()
}

fn push_candidate_from_point(
    pair: &CircleEllipsePair<'_>,
    point: Point3,
    fallback_kind: Option<ContactKind>,
    points: &mut Vec<CurveCurvePoint>,
) {
    let local_circle = pair.circle.frame().to_local(point);
    let raw_circle = math::atan2(local_circle.y, local_circle.x);
    let Some(t_circle) = fit_periodic_parameter(raw_circle, pair.circle_range, pair.parameter_tol)
    else {
        return;
    };
    let local_ellipse = pair.ellipse.frame().to_local(point);
    let raw_ellipse = ellipse_parameter(local_ellipse, pair.ellipse);
    let Some(t_ellipse) =
        fit_periodic_parameter(raw_ellipse, pair.ellipse_range, pair.parameter_tol)
    else {
        return;
    };
    let kind = fallback_kind.unwrap_or_else(|| {
        contact_kind(
            pair.circle,
            t_circle,
            pair.ellipse,
            t_ellipse,
            pair.tolerances,
        )
    });
    if let Some(point) = accept_curve_curve_candidate(
        pair.circle,
        t_circle,
        pair.ellipse,
        t_ellipse,
        kind,
        pair.tolerances,
    ) {
        push_distinct(points, point, pair.tolerances);
    }
}

fn contact_kind(
    circle: &Circle,
    t_circle: f64,
    ellipse: &Ellipse,
    t_ellipse: f64,
    tolerances: Tolerances,
) -> ContactKind {
    let dc = circle.eval_derivs(t_circle, 1).d[1];
    let de = ellipse.eval_derivs(t_ellipse, 1).d[1];
    let Some(uc) = dc.normalized() else {
        return ContactKind::Singular;
    };
    let Some(ue) = de.normalized() else {
        return ContactKind::Singular;
    };
    if uc.cross(ue).norm() <= tolerances.angular() {
        ContactKind::Tangent
    } else {
        ContactKind::Transverse
    }
}

fn circle_ellipse_parameter_tolerance(
    circle: &Circle,
    ellipse: &Ellipse,
    tolerances: Tolerances,
) -> f64 {
    parameter_tolerance(circle.radius(), tolerances)
        .max(parameter_tolerance(ellipse.minor_radius(), tolerances))
}

fn ellipse_is_circle(ellipse: &Ellipse, tolerances: Tolerances) -> bool {
    (ellipse.major_radius() - ellipse.minor_radius()).abs() <= tolerances.linear()
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
    circle_range: ParamRange,
    ellipse_range: ParamRange,
    circle_radius: f64,
    ellipse_minor_radius: f64,
    tolerances: Tolerances,
) -> Result<()> {
    if !circle_range.is_finite()
        || !ellipse_range.is_finite()
        || circle_range.width() < 0.0
        || ellipse_range.width() < 0.0
    {
        return Err(Error::InvalidGeometry {
            reason: "circle/ellipse intersection requires finite non-reversed ranges",
        });
    }
    if circle_range.width()
        > core::f64::consts::TAU + parameter_tolerance(circle_radius, tolerances)
        || ellipse_range.width()
            > core::f64::consts::TAU + parameter_tolerance(ellipse_minor_radius, tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded circle and ellipse ranges cannot span more than one period",
        });
    }
    Ok(())
}
