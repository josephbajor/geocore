use super::conic::fit_periodic_parameter;
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfacePoint, accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::param::ParamRange;
use kgeom::surface::{Sphere, Surface};
use kgeom::vec::Vec3;

/// Intersect a line restricted to a finite range with a finite sphere
/// parameter window.
pub fn intersect_bounded_line_sphere(
    line: &Line,
    line_range: ParamRange,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(line_range, sphere_range)?;

    let local_origin = sphere.frame().to_local(line.origin());
    let direction = line.dir();
    let local_direction = Vec3::new(
        direction.dot(sphere.frame().x()),
        direction.dot(sphere.frame().y()),
        direction.dot(sphere.frame().z()),
    );

    let center_parameter = -local_origin.dot(local_direction);
    let closest = local_origin + local_direction * center_parameter;
    let closest_radius = closest.norm();
    let radius = sphere.radius();
    if closest_radius > radius + tolerances.linear() {
        return Ok(CurveSurfaceIntersections::complete_empty());
    }

    let tangent = (closest_radius - radius).abs() <= tolerances.linear();
    let line_parameters: Vec<f64> = if tangent {
        vec![center_parameter]
    } else {
        let offset = (radius * radius - closest_radius * closest_radius)
            .max(0.0)
            .sqrt();
        vec![center_parameter - offset, center_parameter + offset]
    };

    let mut points = Vec::with_capacity(line_parameters.len());
    for t_line in line_parameters {
        let Some(t_line) = fit_line_parameter(t_line, line_range, tolerances.linear()) else {
            continue;
        };
        let local = local_origin + local_direction * t_line;
        let Some(uv) = sphere_uv(local, sphere_range, radius, tolerances) else {
            continue;
        };
        let kind = if tangent {
            ContactKind::Tangent
        } else if sphere.normal(uv).is_none() {
            ContactKind::Singular
        } else {
            ContactKind::Transverse
        };
        if let Some(point) =
            accept_curve_surface_candidate(line, t_line, sphere, uv, kind, tolerances)
        {
            push_distinct(&mut points, point, tolerances);
        }
    }

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn sphere_uv(
    local: Vec3,
    sphere_range: [ParamRange; 2],
    radius: f64,
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let raw_v = math::atan2(local.z, xy);
    let v_tol = (tolerances.linear() / radius).max(tolerances.angular());
    let v = fit_scalar_parameter(raw_v, sphere_range[1], v_tol)?;
    let u = if xy <= tolerances.linear() {
        sphere_range[0].lo
    } else {
        let raw_u = math::atan2(local.y, local.x);
        fit_periodic_parameter(raw_u, sphere_range[0], v_tol)?
    };
    Some([u, v])
}

fn fit_line_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn push_distinct(
    points: &mut Vec<CurveSurfacePoint>,
    candidate: CurveSurfacePoint,
    tolerances: Tolerances,
) {
    if !points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        points.push(candidate);
    }
}

fn validate_ranges(line_range: ParamRange, sphere_range: [ParamRange; 2]) -> Result<()> {
    if !line_range.is_finite() || line_range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "line/sphere intersection requires a finite non-reversed line range",
        });
    }
    if sphere_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "line/sphere intersection requires finite non-reversed surface ranges",
        });
    }
    Ok(())
}
