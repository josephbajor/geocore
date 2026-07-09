use super::line_plane::intersect_bounded_line_plane;
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, accept_surface_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Line};
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::Point3;

/// Intersect two finite plane windows.
pub fn intersect_bounded_planes(
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(a_range, b_range)?;

    let n_a = a.frame().z();
    let n_b = b.frame().z();
    let direction = n_a.cross(n_b);
    let direction_norm = direction.norm();
    if direction_norm <= tolerances.angular() {
        let separation = (b.frame().origin() - a.frame().origin()).dot(n_a).abs();
        if separation <= tolerances.linear() {
            return Err(Error::InvalidGeometry {
                reason: "coincident plane/plane intersection is a surface overlap",
            });
        }
        return Ok(SurfaceSurfaceIntersections::default());
    }

    let c_a = n_a.dot(a.frame().origin());
    let c_b = n_b.dot(b.frame().origin());
    let origin = ((n_b * c_a - n_a * c_b).cross(direction)) / direction.norm_sq();
    let line = Line::new(origin, direction)?;
    let Some(line_range) = candidate_line_range(&line, a, a_range, b, b_range, tolerances) else {
        return Ok(SurfaceSurfaceIntersections::default());
    };

    let a_hit = intersect_bounded_line_plane(&line, line_range, a, a_range, tolerances)?;
    let b_hit = intersect_bounded_line_plane(&line, line_range, b, b_range, tolerances)?;

    let mut points = Vec::new();
    let mut curves = Vec::new();
    add_clipped_line_branch(
        &mut points,
        &mut curves,
        line,
        line_range,
        &a_hit,
        &b_hit,
        a,
        a_range,
        b,
        b_range,
        tolerances,
    );
    SurfaceSurfaceIntersections::canonicalized(points, curves)
}

fn candidate_line_range(
    line: &Line,
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<ParamRange> {
    let a_projection = plane_window_projection(line, a, a_range);
    let b_projection = plane_window_projection(line, b, b_range);
    let lo = a_projection.lo.max(b_projection.lo);
    let hi = a_projection.hi.min(b_projection.hi);
    if hi < lo - tolerances.linear() {
        None
    } else if hi < lo {
        let mid = (lo + hi) / 2.0;
        Some(ParamRange::new(mid, mid))
    } else {
        Some(ParamRange::new(lo, hi))
    }
}

fn plane_window_projection(line: &Line, plane: &Plane, range: [ParamRange; 2]) -> ParamRange {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for u in [range[0].lo, range[0].hi] {
        for v in [range[1].lo, range[1].hi] {
            let t = (plane.frame().point_at(u, v, 0.0) - line.origin()).dot(line.dir());
            lo = lo.min(t);
            hi = hi.max(t);
        }
    }
    ParamRange::new(lo, hi)
}

#[allow(clippy::too_many_arguments)]
fn add_clipped_line_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    line: Line,
    line_range: ParamRange,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    for a_overlap in &a_hit.overlaps {
        for b_overlap in &b_hit.overlaps {
            let lo = a_overlap.curve.lo.max(b_overlap.curve.lo);
            let hi = a_overlap.curve.hi.min(b_overlap.curve.hi);
            if hi - lo > tolerances.linear() {
                let Some(uv_a_start) = plane_uv_at(line.eval(lo), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_a_end) = plane_uv_at(line.eval(hi), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_start) = plane_uv_at(line.eval(lo), b, b_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_end) = plane_uv_at(line.eval(hi), b, b_range, tolerances) else {
                    continue;
                };
                curves.push(SurfaceSurfaceCurve {
                    curve: SurfaceIntersectionCurve::Line(line),
                    curve_range: ParamRange::new(lo, hi),
                    uv_a_start,
                    uv_a_end,
                    uv_b_start,
                    uv_b_end,
                    kind: ContactKind::Transverse,
                });
            } else if (hi - lo).abs() <= tolerances.linear() {
                add_point_from_parameter(
                    points,
                    &line,
                    line_range,
                    (lo + hi) / 2.0,
                    a,
                    a_range,
                    b,
                    b_range,
                    tolerances,
                );
            }
        }
    }

    add_isolated_points(
        points, &line, line_range, a_hit, b_hit, a, a_range, b, b_range, tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    line: &Line,
    line_range: ParamRange,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    for point in &a_hit.points {
        if hit_contains_t(b_hit, point.t_curve, tolerances) {
            add_point_from_parameter(
                points,
                line,
                line_range,
                point.t_curve,
                a,
                a_range,
                b,
                b_range,
                tolerances,
            );
        }
    }
    for point in &b_hit.points {
        if hit_contains_t(a_hit, point.t_curve, tolerances) {
            add_point_from_parameter(
                points,
                line,
                line_range,
                point.t_curve,
                a,
                a_range,
                b,
                b_range,
                tolerances,
            );
        }
    }
    for a_point in &a_hit.points {
        for b_point in &b_hit.points {
            if curve_parameters_match(a_point, b_point, tolerances) {
                add_point_from_parameter(
                    points,
                    line,
                    line_range,
                    a_point.t_curve,
                    a,
                    a_range,
                    b,
                    b_range,
                    tolerances,
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn add_point_from_parameter(
    points: &mut Vec<SurfaceSurfacePoint>,
    line: &Line,
    line_range: ParamRange,
    t: f64,
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, line_range, tolerances.linear()) else {
        return;
    };
    let point = line.eval(t);
    let Some(uv_a) = plane_uv_at(point, a, a_range, tolerances) else {
        return;
    };
    let Some(uv_b) = plane_uv_at(point, b, b_range, tolerances) else {
        return;
    };
    if let Some(point) =
        accept_surface_surface_candidate(a, uv_a, b, uv_b, ContactKind::Transverse, tolerances)
    {
        push_point(points, point, tolerances);
    }
}

fn plane_uv_at(
    point: Point3,
    plane: &Plane,
    range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = plane.frame().to_local(point);
    Some([
        fit_scalar_parameter(local.x, range[0], tolerances.linear())?,
        fit_scalar_parameter(local.y, range[1], tolerances.linear())?,
    ])
}

fn hit_contains_t(hit: &CurveSurfaceIntersections, t: f64, tolerances: Tolerances) -> bool {
    hit.overlaps
        .iter()
        .any(|overlap| overlap_contains_t(overlap, t, tolerances))
        || hit
            .points
            .iter()
            .any(|point| (point.t_curve - t).abs() <= tolerances.linear())
}

fn overlap_contains_t(overlap: &CurveSurfaceOverlap, t: f64, tolerances: Tolerances) -> bool {
    t >= overlap.curve.lo - tolerances.linear() && t <= overlap.curve.hi + tolerances.linear()
}

fn curve_parameters_match(
    a: &CurveSurfacePoint,
    b: &CurveSurfacePoint,
    tolerances: Tolerances,
) -> bool {
    (a.t_curve - b.t_curve).abs() <= tolerances.linear()
        || a.point.dist(b.point) <= tolerances.linear()
}

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn push_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    candidate: SurfaceSurfacePoint,
    tolerances: Tolerances,
) {
    if !points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        points.push(candidate);
    }
}

fn validate_ranges(a_range: [ParamRange; 2], b_range: [ParamRange; 2]) -> Result<()> {
    if a_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/plane intersection requires finite non-reversed first-plane ranges",
        });
    }
    if b_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/plane intersection requires finite non-reversed second-plane ranges",
        });
    }
    Ok(())
}
