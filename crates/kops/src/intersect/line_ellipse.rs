use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurvePoint, accept_curve_curve_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Ellipse, Line};
use kgeom::param::ParamRange;
use kgeom::vec::Vec3;

/// Intersect a line and ellipse restricted to finite parameter ranges.
///
/// The line and ellipse may have any relative orientation in 3D. A coplanar
/// line can produce a secant or tangent contact; a line crossing the ellipse
/// plane can produce at most one transverse contact.
pub fn intersect_bounded_line_ellipse(
    line: &Line,
    line_range: ParamRange,
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    validate_ranges(
        line_range,
        ellipse_range,
        ellipse.minor_radius(),
        tolerances,
    )?;

    let local_origin = ellipse.frame().to_local(line.origin());
    let direction = line.dir();
    let local_direction = Vec3::new(
        direction.dot(ellipse.frame().x()),
        direction.dot(ellipse.frame().y()),
        direction.dot(ellipse.frame().z()),
    );

    if local_direction.z.abs() > tolerances.angular() {
        let t_line = -local_origin.z / local_direction.z;
        let Some(t_line) = fit_line_parameter(t_line, line_range, tolerances.linear()) else {
            return Ok(CurveCurveIntersections::complete_empty());
        };
        let local = local_origin + local_direction * t_line;
        let raw_ellipse = ellipse_parameter(local, ellipse);
        let Some(t_ellipse) = fit_ellipse_parameter(
            raw_ellipse,
            ellipse_range,
            ellipse_parameter_tolerance(ellipse.minor_radius(), tolerances),
        ) else {
            return Ok(CurveCurveIntersections::complete_empty());
        };
        let points = accept_curve_curve_candidate(
            line,
            t_line,
            ellipse,
            t_ellipse,
            ContactKind::Transverse,
            tolerances,
        )
        .into_iter()
        .collect();
        return CurveCurveIntersections::canonicalized_complete(points, Vec::new());
    }

    if local_origin.z.abs() > tolerances.linear() {
        return Ok(CurveCurveIntersections::complete_empty());
    }

    intersect_coplanar(
        line,
        line_range,
        ellipse,
        ellipse_range,
        local_origin,
        local_direction,
        tolerances,
    )
}

fn intersect_coplanar(
    line: &Line,
    line_range: ParamRange,
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    local_origin: Vec3,
    local_direction: Vec3,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let major = ellipse.major_radius();
    let minor = ellipse.minor_radius();
    let scaled_origin = Vec3::new(local_origin.x / major, local_origin.y / minor, 0.0);
    let scaled_direction = Vec3::new(local_direction.x / major, local_direction.y / minor, 0.0);
    let speed_sq =
        scaled_direction.x * scaled_direction.x + scaled_direction.y * scaled_direction.y;
    let center_parameter = -scaled_origin.dot(scaled_direction) / speed_sq;
    let closest = scaled_origin + scaled_direction * center_parameter;
    let closest_radius = (closest.x * closest.x + closest.y * closest.y).sqrt();
    let scaled_tol = ellipse_parameter_tolerance(minor, tolerances);
    if closest_radius > 1.0 + scaled_tol {
        return Ok(CurveCurveIntersections::complete_empty());
    }

    let tangent = (closest_radius - 1.0).abs() <= scaled_tol;
    let line_parameters: Vec<f64> = if tangent {
        vec![center_parameter]
    } else {
        let offset = ((1.0 - closest_radius * closest_radius) / speed_sq)
            .max(0.0)
            .sqrt();
        vec![center_parameter - offset, center_parameter + offset]
    };

    let parameter_tol = ellipse_parameter_tolerance(minor, tolerances);
    let mut points = Vec::with_capacity(line_parameters.len());
    for t_line in line_parameters {
        let Some(t_line) = fit_line_parameter(t_line, line_range, tolerances.linear()) else {
            continue;
        };
        let local = local_origin + local_direction * t_line;
        let raw_ellipse = ellipse_parameter(local, ellipse);
        let Some(t_ellipse) = fit_ellipse_parameter(raw_ellipse, ellipse_range, parameter_tol)
        else {
            continue;
        };
        if let Some(point) = accept_curve_curve_candidate(
            line,
            t_line,
            ellipse,
            t_ellipse,
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
    CurveCurveIntersections::canonicalized_complete(points, Vec::new())
}

fn ellipse_parameter(local: Vec3, ellipse: &Ellipse) -> f64 {
    math::atan2(
        local.y / ellipse.minor_radius(),
        local.x / ellipse.major_radius(),
    )
}

fn fit_line_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn fit_ellipse_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    let period = core::f64::consts::TAU;
    let k_min = ((range.lo - tolerance - candidate) / period).ceil() as i64;
    let k_max = ((range.hi + tolerance - candidate) / period).floor() as i64;
    if k_min > k_max {
        return None;
    }
    Some((candidate + k_min as f64 * period).clamp(range.lo, range.hi))
}

fn ellipse_parameter_tolerance(minor_radius: f64, tolerances: Tolerances) -> f64 {
    (tolerances.linear() / minor_radius).max(tolerances.angular())
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
    ellipse_range: ParamRange,
    minor_radius: f64,
    tolerances: Tolerances,
) -> Result<()> {
    if !line_range.is_finite()
        || !ellipse_range.is_finite()
        || line_range.width() < 0.0
        || ellipse_range.width() < 0.0
    {
        return Err(Error::InvalidGeometry {
            reason: "line/ellipse intersection requires finite non-reversed ranges",
        });
    }
    if ellipse_range.width()
        > core::f64::consts::TAU + ellipse_parameter_tolerance(minor_radius, tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded ellipse range cannot span more than one period",
        });
    }
    Ok(())
}
