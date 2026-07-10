use super::conic::fit_periodic_parameter;
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgeom::vec::Vec3;

/// Intersect a line restricted to a finite range with a finite cylinder
/// parameter window.
///
/// A transverse or tangent line can produce isolated points. A line lying on a
/// cylinder ruling clips against the finite `(u, v)` window and can produce a
/// positive-length contained interval.
pub fn intersect_bounded_line_cylinder(
    line: &Line,
    line_range: ParamRange,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(line_range, cylinder_range)?;

    let local_origin = cylinder.frame().to_local(line.origin());
    let direction = line.dir();
    let local_direction = Vec3::new(
        direction.dot(cylinder.frame().x()),
        direction.dot(cylinder.frame().y()),
        direction.dot(cylinder.frame().z()),
    );
    let radial_speed_sq =
        local_direction.x * local_direction.x + local_direction.y * local_direction.y;

    if radial_speed_sq <= tolerances.angular() * tolerances.angular() {
        return contained_ruling_interval(
            line,
            line_range,
            cylinder,
            cylinder_range,
            local_origin,
            local_direction,
            tolerances,
        );
    }

    let radial_dot = local_origin.x * local_direction.x + local_origin.y * local_direction.y;
    let center_parameter = -radial_dot / radial_speed_sq;
    let closest = local_origin + local_direction * center_parameter;
    let closest_radius = (closest.x * closest.x + closest.y * closest.y).sqrt();
    let radius = cylinder.radius();
    if closest_radius > radius + tolerances.linear() {
        return Ok(CurveSurfaceIntersections::complete_empty());
    }

    let tangent = (closest_radius - radius).abs() <= tolerances.linear();
    let line_parameters: Vec<f64> = if tangent {
        vec![center_parameter]
    } else {
        let offset = ((radius * radius - closest_radius * closest_radius) / radial_speed_sq)
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
        let Some(uv) = cylinder_uv(local, cylinder_range, radius, tolerances) else {
            continue;
        };
        if let Some(point) = accept_curve_surface_candidate(
            line,
            t_line,
            cylinder,
            uv,
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

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn contained_ruling_interval(
    line: &Line,
    line_range: ParamRange,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    local_origin: Vec3,
    local_direction: Vec3,
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    let radius = cylinder.radius();
    let radial_radius = (local_origin.x * local_origin.x + local_origin.y * local_origin.y).sqrt();
    if (radial_radius - radius).abs() > tolerances.linear() {
        return Ok(CurveSurfaceIntersections::complete_empty());
    }

    let raw_u = math::atan2(local_origin.y, local_origin.x);
    let Some(u) = fit_periodic_parameter(
        raw_u,
        cylinder_range[0],
        parameter_tolerance(radius, tolerances),
    ) else {
        return Ok(CurveSurfaceIntersections::complete_empty());
    };

    let Some(interval) = clip_linear_interval(
        line_range,
        local_origin.z,
        local_direction.z,
        cylinder_range[1],
        tolerances,
    ) else {
        return Ok(CurveSurfaceIntersections::complete_empty());
    };

    if interval.width() > tolerances.linear() {
        let Some(v_start) = fit_scalar_parameter(
            v_at(local_origin, local_direction, interval.lo),
            cylinder_range[1],
            tolerances.linear(),
        ) else {
            return Ok(CurveSurfaceIntersections::complete_empty());
        };
        let Some(v_end) = fit_scalar_parameter(
            v_at(local_origin, local_direction, interval.hi),
            cylinder_range[1],
            tolerances.linear(),
        ) else {
            return Ok(CurveSurfaceIntersections::complete_empty());
        };
        let uv_start = [u, v_start];
        let uv_end = [u, v_end];
        let overlap = CurveSurfaceOverlap {
            curve: interval,
            uv_start,
            uv_end,
        };
        return CurveSurfaceIntersections::canonicalized_complete(Vec::new(), vec![overlap]);
    }

    let t_line = ((interval.lo + interval.hi) / 2.0).clamp(line_range.lo, line_range.hi);
    let Some(v) = fit_scalar_parameter(
        v_at(local_origin, local_direction, t_line),
        cylinder_range[1],
        tolerances.linear(),
    ) else {
        return Ok(CurveSurfaceIntersections::complete_empty());
    };
    let points = accept_curve_surface_candidate(
        line,
        t_line,
        cylinder,
        [u, v],
        ContactKind::Tangent,
        tolerances,
    )
    .into_iter()
    .collect::<Vec<CurveSurfacePoint>>();
    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn cylinder_uv(
    local: Vec3,
    cylinder_range: [ParamRange; 2],
    radius: f64,
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let raw_u = math::atan2(local.y, local.x);
    let u = fit_periodic_parameter(
        raw_u,
        cylinder_range[0],
        parameter_tolerance(radius, tolerances),
    )?;
    let v = fit_scalar_parameter(local.z, cylinder_range[1], tolerances.linear())?;
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

fn clip_linear_interval(
    interval: ParamRange,
    origin: f64,
    direction: f64,
    range: ParamRange,
    tolerances: Tolerances,
) -> Option<ParamRange> {
    if direction.abs() <= tolerances.angular() {
        if origin < range.lo - tolerances.linear() || origin > range.hi + tolerances.linear() {
            None
        } else {
            Some(interval)
        }
    } else {
        let t0 = (range.lo - origin) / direction;
        let t1 = (range.hi - origin) / direction;
        let lo = interval.lo.max(t0.min(t1));
        let hi = interval.hi.min(t0.max(t1));
        if hi < lo - tolerances.linear() {
            None
        } else {
            Some(ParamRange::new(
                lo.clamp(interval.lo, interval.hi),
                hi.clamp(interval.lo, interval.hi),
            ))
        }
    }
}

fn v_at(local_origin: Vec3, local_direction: Vec3, t: f64) -> f64 {
    local_origin.z + local_direction.z * t
}

fn parameter_tolerance(radius: f64, tolerances: Tolerances) -> f64 {
    (tolerances.linear() / radius).max(tolerances.angular())
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

fn validate_ranges(line_range: ParamRange, cylinder_range: [ParamRange; 2]) -> Result<()> {
    if !line_range.is_finite() || line_range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "line/cylinder intersection requires a finite non-reversed line range",
        });
    }
    if cylinder_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "line/cylinder intersection requires finite non-reversed surface ranges",
        });
    }
    Ok(())
}
