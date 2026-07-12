use super::parameter::{fit_parameter_pair, fit_scalar_parameter, validate_curve_surface_ranges};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    accept_curve_surface_candidate,
};
use kcore::error::Result;
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::Vec3;

/// Intersect a line restricted to a finite range with a finite plane
/// parameter window.
///
/// A transverse line can produce one point. A line lying in the plane clips
/// against the plane's `(u, v)` window and can produce a positive-length
/// contained interval.
pub fn intersect_bounded_line_plane(
    line: &Line,
    line_range: ParamRange,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(line_range, plane_range)?;

    let local_origin = plane.frame().to_local(line.origin());
    let direction = line.dir();
    let local_direction = Vec3::new(
        direction.dot(plane.frame().x()),
        direction.dot(plane.frame().y()),
        direction.dot(plane.frame().z()),
    );

    if local_direction.z.abs() > tolerances.angular() {
        let t_line = -local_origin.z / local_direction.z;
        let Some(t_line) = fit_scalar_parameter(t_line, line_range, tolerances.linear()) else {
            return Ok(CurveSurfaceIntersections::complete_empty());
        };
        let local = local_origin + local_direction * t_line;
        let Some(uv) = fit_parameter_pair([local.x, local.y], plane_range, tolerances.linear())
        else {
            return Ok(CurveSurfaceIntersections::complete_empty());
        };
        let points = accept_curve_surface_candidate(
            line,
            t_line,
            plane,
            uv,
            ContactKind::Transverse,
            tolerances,
        )
        .into_iter()
        .collect();
        return CurveSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    if local_origin.z.abs() > tolerances.linear() {
        return Ok(CurveSurfaceIntersections::complete_empty());
    }

    contained_line_interval(
        line,
        line_range,
        plane,
        plane_range,
        local_origin,
        local_direction,
        tolerances,
    )
}

fn contained_line_interval(
    line: &Line,
    line_range: ParamRange,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    local_origin: Vec3,
    local_direction: Vec3,
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    let mut interval = line_range;
    for (axis, range) in [(0, plane_range[0]), (1, plane_range[1])] {
        let origin = if axis == 0 {
            local_origin.x
        } else {
            local_origin.y
        };
        let direction = if axis == 0 {
            local_direction.x
        } else {
            local_direction.y
        };
        let Some(next) = clip_linear_interval(interval, origin, direction, range, tolerances)
        else {
            return Ok(CurveSurfaceIntersections::complete_empty());
        };
        interval = next;
    }

    if interval.width() > tolerances.linear() {
        let uv_start = uv_at(local_origin, local_direction, interval.lo);
        let uv_end = uv_at(local_origin, local_direction, interval.hi);
        let overlap = CurveSurfaceOverlap {
            curve: interval,
            uv_start,
            uv_end,
        };
        return CurveSurfaceIntersections::canonicalized_complete(Vec::new(), vec![overlap]);
    }

    let t_line = ((interval.lo + interval.hi) / 2.0).clamp(line_range.lo, line_range.hi);
    let Some(uv) = fit_parameter_pair(
        uv_at(local_origin, local_direction, t_line),
        plane_range,
        tolerances.linear(),
    ) else {
        return Ok(CurveSurfaceIntersections::complete_empty());
    };
    let points =
        accept_curve_surface_candidate(line, t_line, plane, uv, ContactKind::Tangent, tolerances)
            .into_iter()
            .collect::<Vec<CurveSurfacePoint>>();
    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
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

fn uv_at(local_origin: Vec3, local_direction: Vec3, t: f64) -> [f64; 2] {
    let local = local_origin + local_direction * t;
    [local.x, local.y]
}

fn validate_ranges(line_range: ParamRange, plane_range: [ParamRange; 2]) -> Result<()> {
    validate_curve_surface_ranges(
        line_range,
        plane_range,
        "line/plane intersection requires a finite non-reversed line range",
        "line/plane intersection requires finite non-reversed surface ranges",
    )
}
