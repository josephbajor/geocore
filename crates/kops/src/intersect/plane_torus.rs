use super::circle_torus::intersect_bounded_circle_torus;
use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::planar_curve_plane::intersect_bounded_circle_plane;
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, accept_surface_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Torus};
use kgeom::vec::Point3;

/// Intersect a finite plane window with a finite torus parameter window.
///
/// Supports the circular closed-form torus sections: planes normal to the
/// torus axis (latitude circles) and meridian planes containing the torus axis
/// (tube circles). General plane/torus sections are quartic curves and remain
/// explicit until SSI result geometry can carry that branch family.
pub fn intersect_bounded_plane_torus(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(plane_range, torus_range)?;

    let plane_normal = plane.frame().z();
    let torus_axis = torus.frame().z();
    if plane_normal.cross(torus_axis).norm() <= tolerances.angular() {
        return intersect_axis_normal_plane_torus(
            plane,
            plane_range,
            torus,
            torus_range,
            tolerances,
        );
    }

    let axial_alignment = plane_normal.dot(torus_axis).abs();
    let axis_offset = (torus.frame().origin() - plane.frame().origin()).dot(plane_normal);
    if axial_alignment <= tolerances.angular() && axis_offset.abs() <= tolerances.linear() {
        return intersect_meridian_plane_torus(plane, plane_range, torus, torus_range, tolerances);
    }

    Err(Error::InvalidGeometry {
        reason: "plane/torus intersection currently supports only axis-normal latitude circles or axis-containing meridian circles",
    })
}

fn intersect_axis_normal_plane_torus(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let axis = torus.frame().z();
    let h = (plane.frame().origin() - torus.frame().origin()).dot(axis);
    let minor = torus.minor_radius();
    let h_sq = minor * minor - h * h;
    let sq_tol = squared_tolerance(minor, tolerances);
    if h_sq < -sq_tol {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let center = torus.frame().origin() + axis * h;
    let mut points = Vec::new();
    let mut curves = Vec::new();
    if h_sq <= sq_tol {
        let circle = Circle::new(
            Frame::new(center, axis, torus.frame().x())?,
            torus.major_radius(),
        )?;
        add_circle_branch(
            &mut points,
            &mut curves,
            circle,
            ContactKind::Tangent,
            plane,
            plane_range,
            torus,
            torus_range,
            tolerances,
        )?;
    } else {
        let radial_delta = h_sq.sqrt();
        for radius in [
            torus.major_radius() - radial_delta,
            torus.major_radius() + radial_delta,
        ] {
            let circle = Circle::new(Frame::new(center, axis, torus.frame().x())?, radius)?;
            add_circle_branch(
                &mut points,
                &mut curves,
                circle,
                ContactKind::Transverse,
                plane,
                plane_range,
                torus,
                torus_range,
                tolerances,
            )?;
        }
    }

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

fn intersect_meridian_plane_torus(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let axis = torus.frame().z();
    let normal = (plane.frame().z() - axis * plane.frame().z().dot(axis))
        .normalized()
        .ok_or(Error::InvalidGeometry {
            reason: "axis-containing plane/torus section has degenerate plane normal",
        })?;
    let radial = axis
        .cross(normal)
        .normalized()
        .ok_or(Error::InvalidGeometry {
            reason: "axis-containing plane/torus section has degenerate radial direction",
        })?;

    let mut points = Vec::new();
    let mut curves = Vec::new();
    for local_radial in [-radial, radial] {
        let center = torus.frame().origin() + local_radial * torus.major_radius();
        let circle = Circle::new(
            Frame::new(center, local_radial.cross(axis), local_radial)?,
            torus.minor_radius(),
        )?;
        add_circle_branch(
            &mut points,
            &mut curves,
            circle,
            ContactKind::Transverse,
            plane,
            plane_range,
            torus,
            torus_range,
            tolerances,
        )?;
    }

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

#[allow(clippy::too_many_arguments)]
fn add_circle_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    circle: Circle,
    branch_kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let plane_hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        plane,
        plane_range,
        tolerances,
    )?;
    let torus_hit = intersect_bounded_circle_torus(
        &circle,
        circle.param_range(),
        torus,
        torus_range,
        tolerances,
    )?;
    add_clipped_branch(
        points,
        curves,
        &circle,
        &plane_hit,
        &torus_hit,
        branch_kind,
        plane,
        plane_range,
        torus,
        torus_range,
        tolerances,
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_clipped_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    circle: &Circle,
    plane_hit: &CurveSurfaceIntersections,
    torus_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let t_tol = parameter_tolerance(circle.radius(), tolerances);
    for plane_overlap in &plane_hit.overlaps {
        for torus_overlap in &torus_hit.overlaps {
            let lo = plane_overlap.curve.lo.max(torus_overlap.curve.lo);
            let hi = plane_overlap.curve.hi.min(torus_overlap.curve.hi);
            if hi - lo > t_tol {
                let Some(uv_plane_start) =
                    plane_uv_at(circle.eval(lo), plane, plane_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_plane_end) =
                    plane_uv_at(circle.eval(hi), plane, plane_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_torus_start) =
                    torus_uv_at(circle.eval(lo), torus, torus_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_torus_end) =
                    torus_uv_at(circle.eval(hi), torus, torus_range, tolerances)
                else {
                    continue;
                };
                push_curve(
                    curves,
                    SurfaceSurfaceCurve {
                        curve: SurfaceIntersectionCurve::Circle(*circle),
                        curve_range: ParamRange::new(lo, hi),
                        uv_a_start: uv_plane_start,
                        uv_a_end: uv_plane_end,
                        uv_b_start: uv_torus_start,
                        uv_b_end: uv_torus_end,
                        kind: branch_kind,
                    },
                    t_tol.max(tolerances.linear()),
                );
            } else if (hi - lo).abs() <= t_tol {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    ((lo + hi) / 2.0).clamp(circle.param_range().lo, circle.param_range().hi),
                    branch_kind,
                    plane,
                    plane_range,
                    torus,
                    torus_range,
                    t_tol,
                    tolerances,
                );
            }
        }
    }

    add_isolated_points(
        points,
        circle,
        plane_hit,
        torus_hit,
        branch_kind,
        plane,
        plane_range,
        torus,
        torus_range,
        t_tol,
        tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    circle: &Circle,
    plane_hit: &CurveSurfaceIntersections,
    torus_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    for point in &plane_hit.points {
        if hit_contains_t(torus_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                plane,
                plane_range,
                torus,
                torus_range,
                t_tol,
                tolerances,
            );
        }
    }
    for point in &torus_hit.points {
        if hit_contains_t(plane_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                plane,
                plane_range,
                torus,
                torus_range,
                t_tol,
                tolerances,
            );
        }
    }
    for plane_point in &plane_hit.points {
        for torus_point in &torus_hit.points {
            if curve_parameters_match(plane_point, torus_point, t_tol, tolerances) {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    plane_point.t_curve,
                    branch_kind,
                    plane,
                    plane_range,
                    torus,
                    torus_range,
                    t_tol,
                    tolerances,
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn add_point_from_curve_parameter(
    points: &mut Vec<SurfaceSurfacePoint>,
    circle: &Circle,
    t: f64,
    kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, circle.param_range(), t_tol) else {
        return;
    };
    let point = circle.eval(t);
    let Some(uv_plane) = plane_uv_at(point, plane, plane_range, tolerances) else {
        return;
    };
    let Some(uv_torus) = torus_uv_at(point, torus, torus_range, tolerances) else {
        return;
    };
    if let Some(point) =
        accept_surface_surface_candidate(plane, uv_plane, torus, uv_torus, kind, tolerances)
    {
        push_point(points, point, tolerances);
    }
}

fn plane_uv_at(
    point: Point3,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = plane.frame().to_local(point);
    Some([
        fit_scalar_parameter(local.x, plane_range[0], tolerances.linear())?,
        fit_scalar_parameter(local.y, plane_range[1], tolerances.linear())?,
    ])
}

fn torus_uv_at(
    point: Point3,
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = torus.frame().to_local(point);
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let raw_u = if xy <= tolerances.linear() {
        torus_range[0].lo
    } else {
        math::atan2(local.y, local.x)
    };
    let u_tol = parameter_tolerance(
        xy.max(torus.major_radius() - torus.minor_radius()),
        tolerances,
    );
    let u = fit_periodic_parameter(raw_u, torus_range[0], u_tol)?;
    let raw_v = math::atan2(local.z, xy - torus.major_radius());
    let v = fit_periodic_parameter(
        raw_v,
        torus_range[1],
        parameter_tolerance(torus.minor_radius(), tolerances),
    )?;
    Some([u, v])
}

fn hit_contains_t(
    hit: &CurveSurfaceIntersections,
    t: f64,
    t_tol: f64,
    tolerances: Tolerances,
) -> bool {
    hit.overlaps
        .iter()
        .any(|overlap| overlap_contains_t(overlap, t, t_tol))
        || hit.points.iter().any(|point| {
            curve_parameter_distance(point.t_curve, t) <= t_tol.max(tolerances.angular())
        })
}

fn overlap_contains_t(overlap: &CurveSurfaceOverlap, t: f64, t_tol: f64) -> bool {
    [t, t - core::f64::consts::TAU, t + core::f64::consts::TAU]
        .into_iter()
        .any(|candidate| {
            candidate >= overlap.curve.lo - t_tol && candidate <= overlap.curve.hi + t_tol
        })
}

fn curve_parameters_match(
    a: &CurveSurfacePoint,
    b: &CurveSurfacePoint,
    t_tol: f64,
    tolerances: Tolerances,
) -> bool {
    curve_parameter_distance(a.t_curve, b.t_curve) <= t_tol.max(tolerances.angular())
        || a.point.dist(b.point) <= tolerances.linear()
}

fn curve_parameter_distance(a: f64, b: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let diff = (a - b).abs();
    diff.min((period - diff).abs())
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

fn push_curve(
    curves: &mut Vec<SurfaceSurfaceCurve>,
    candidate: SurfaceSurfaceCurve,
    tolerance: f64,
) {
    if !curves.iter().any(|curve| {
        (curve.curve_range.lo - candidate.curve_range.lo).abs() <= tolerance
            && (curve.curve_range.hi - candidate.curve_range.hi).abs() <= tolerance
            && curve
                .curve
                .eval(curve.curve_range.lo)
                .dist(candidate.curve.eval(candidate.curve_range.lo))
                <= tolerance
            && curve
                .curve
                .eval(curve.curve_range.hi)
                .dist(candidate.curve.eval(candidate.curve_range.hi))
                <= tolerance
    }) {
        curves.push(candidate);
    }
}

fn squared_tolerance(radius: f64, tolerances: Tolerances) -> f64 {
    tolerances.linear() * radius.max(1.0)
}

fn validate_ranges(plane_range: [ParamRange; 2], torus_range: [ParamRange; 2]) -> Result<()> {
    if plane_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/torus intersection requires finite non-reversed plane ranges",
        });
    }
    if torus_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/torus intersection requires finite non-reversed torus ranges",
        });
    }
    Ok(())
}
