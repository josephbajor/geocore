use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurvePoint, accept_curve_curve_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Line};
use kgeom::param::ParamRange;

/// Intersect a line and circle restricted to finite parameter ranges.
///
/// The line and circle may have any relative orientation in 3D. A line in the
/// circle plane can produce a secant or tangent contact; a line crossing the
/// plane can produce at most one transverse contact.
pub fn intersect_bounded_line_circle(
    line: &Line,
    line_range: ParamRange,
    circle: &Circle,
    circle_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    validate_ranges(line_range, circle_range, circle.radius(), tolerances)?;

    let local_origin = circle.frame().to_local(line.origin());
    let direction = line.dir();
    let local_direction = kgeom::vec::Vec3::new(
        direction.dot(circle.frame().x()),
        direction.dot(circle.frame().y()),
        direction.dot(circle.frame().z()),
    );

    if local_direction.z.abs() > tolerances.angular() {
        let t_line = -local_origin.z / local_direction.z;
        let Some(t_line) = fit_line_parameter(t_line, line_range, tolerances.linear()) else {
            return Ok(CurveCurveIntersections::default());
        };
        let local = local_origin + local_direction * t_line;
        let raw_circle = math::atan2(local.y, local.x);
        let Some(t_circle) = fit_circle_parameter(
            raw_circle,
            circle_range,
            circle_parameter_tolerance(circle.radius(), tolerances),
        ) else {
            return Ok(CurveCurveIntersections::default());
        };
        let points = accept_curve_curve_candidate(
            line,
            t_line,
            circle,
            t_circle,
            ContactKind::Transverse,
            tolerances,
        )
        .into_iter()
        .collect();
        return CurveCurveIntersections::canonicalized(points, Vec::new());
    }

    if local_origin.z.abs() > tolerances.linear() {
        return Ok(CurveCurveIntersections::default());
    }

    intersect_coplanar(
        line,
        line_range,
        circle,
        circle_range,
        local_origin,
        local_direction,
        tolerances,
    )
}

fn intersect_coplanar(
    line: &Line,
    line_range: ParamRange,
    circle: &Circle,
    circle_range: ParamRange,
    local_origin: kgeom::vec::Vec3,
    local_direction: kgeom::vec::Vec3,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let planar_speed_sq =
        local_direction.x * local_direction.x + local_direction.y * local_direction.y;
    let center_parameter = -(local_origin.x * local_direction.x
        + local_origin.y * local_direction.y)
        / planar_speed_sq;
    let closest_x = local_origin.x + center_parameter * local_direction.x;
    let closest_y = local_origin.y + center_parameter * local_direction.y;
    let closest_radius = (closest_x * closest_x + closest_y * closest_y).sqrt();
    let radius = circle.radius();
    if closest_radius > radius + tolerances.linear() {
        return Ok(CurveCurveIntersections::default());
    }

    let tangent = (closest_radius - radius).abs() <= tolerances.linear();
    let line_parameters: Vec<f64> = if tangent {
        vec![center_parameter]
    } else {
        let offset = ((radius * radius - closest_radius * closest_radius) / planar_speed_sq)
            .max(0.0)
            .sqrt();
        vec![center_parameter - offset, center_parameter + offset]
    };

    let circle_parameter_tol = circle_parameter_tolerance(radius, tolerances);
    let mut points = Vec::with_capacity(line_parameters.len());
    for t_line in line_parameters {
        let Some(t_line) = fit_line_parameter(t_line, line_range, tolerances.linear()) else {
            continue;
        };
        let local = local_origin + local_direction * t_line;
        let raw_circle = math::atan2(local.y, local.x);
        let Some(t_circle) = fit_circle_parameter(raw_circle, circle_range, circle_parameter_tol)
        else {
            continue;
        };
        if let Some(point) = accept_curve_curve_candidate(
            line,
            t_line,
            circle,
            t_circle,
            if tangent {
                ContactKind::Tangent
            } else {
                ContactKind::Transverse
            },
            tolerances,
        ) {
            push_distinct(&mut points, point, tolerances);
        }
    }
    CurveCurveIntersections::canonicalized(points, Vec::new())
}

fn fit_line_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
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
    line_range: ParamRange,
    circle_range: ParamRange,
    radius: f64,
    tolerances: Tolerances,
) -> Result<()> {
    if !line_range.is_finite()
        || !circle_range.is_finite()
        || line_range.width() < 0.0
        || circle_range.width() < 0.0
    {
        return Err(Error::InvalidGeometry {
            reason: "line/circle intersection requires finite non-reversed ranges",
        });
    }
    if circle_range.width()
        > core::f64::consts::TAU + circle_parameter_tolerance(radius, tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded circle range cannot span more than one period",
        });
    }
    Ok(())
}
