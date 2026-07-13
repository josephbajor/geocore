use super::circle_circle::intersect_bounded_circles;
use super::conic::{
    ConicPairConfig, ConicPlaneRelation, canonical_angle, push_angle_root, real_polynomial_roots,
};
use super::line_circle::intersect_bounded_line_circle;
use super::result::CurveCurveIntersections;
use kcore::error::Result;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::param::ParamRange;
use kgeom::vec::Vec3;

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
    let pair =
        ConicPairConfig::circle_ellipse(circle, circle_range, ellipse, ellipse_range, tolerances)?;
    match pair.plane_relation()? {
        ConicPlaneRelation::Parallel => intersect_parallel_plane(
            circle,
            circle_range,
            ellipse,
            ellipse_range,
            tolerances,
            pair,
        ),
        ConicPlaneRelation::Crossing(line) => {
            intersect_plane_crossing(circle, circle_range, tolerances, line, pair)
        }
    }
}

fn intersect_parallel_plane(
    circle: &Circle,
    circle_range: ParamRange,
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    tolerances: Tolerances,
    pair: ConicPairConfig<'_>,
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

    intersect_coplanar_distinct(circle, ellipse, tolerances, pair)
}

fn intersect_plane_crossing(
    circle: &Circle,
    circle_range: ParamRange,
    tolerances: Tolerances,
    line: Line,
    pair: ConicPairConfig<'_>,
) -> Result<CurveCurveIntersections> {
    let center_parameter = line.dir().dot(circle.frame().origin() - line.origin());
    let line_range = ParamRange::new(
        center_parameter - circle.radius() - tolerances.linear(),
        center_parameter + circle.radius() + tolerances.linear(),
    );
    let line_circle_hits =
        intersect_bounded_line_circle(&line, line_range, circle, circle_range, tolerances)?;

    let mut points = Vec::with_capacity(line_circle_hits.points.len());
    for line_hit in line_circle_hits.points {
        let point = circle.eval(line_hit.t_b);
        pair.push_point(point, None, &mut points);
    }
    CurveCurveIntersections::canonicalized_complete(points, Vec::new())
}

fn intersect_coplanar_distinct(
    circle: &Circle,
    ellipse: &Ellipse,
    tolerances: Tolerances,
    pair: ConicPairConfig<'_>,
) -> Result<CurveCurveIntersections> {
    let center = ellipse.frame().to_local(circle.frame().origin());
    let roots = coplanar_roots(ellipse, center, circle.radius(), tolerances);
    let mut points = Vec::with_capacity(roots.len());
    for t_ellipse in roots {
        let point = ellipse.eval(t_ellipse);
        pair.push_point(point, None, &mut points);
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

fn ellipse_is_circle(ellipse: &Ellipse, tolerances: Tolerances) -> bool {
    (ellipse.major_radius() - ellipse.minor_radius()).abs() <= tolerances.linear()
}
