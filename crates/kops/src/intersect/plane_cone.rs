use super::circle_cone::intersect_bounded_circle_cone;
use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::ellipse_cone::intersect_bounded_ellipse_cone;
use super::planar_curve_plane::{intersect_bounded_circle_plane, intersect_bounded_ellipse_plane};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, accept_surface_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Plane, Surface};
use kgeom::vec::Point3;

/// Intersect a finite plane window with a finite cone parameter window.
///
/// Supports axis-perpendicular circular cuts, oblique elliptic cuts, and
/// singular apex contacts. Parabolic and hyperbolic plane/cone sections remain
/// explicit until those branch geometries are represented in SSI results.
pub fn intersect_bounded_plane_cone(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(plane_range, cone_range)?;

    let normal = plane.frame().z();
    let axis = cone.frame().z();
    let nx = normal.dot(cone.frame().x());
    let ny = normal.dot(cone.frame().y());
    let nz = normal.dot(axis);
    let radial_len = (nx * nx + ny * ny).sqrt();
    if nz.abs() <= tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "plane/cone intersection currently supports only circular and elliptic cuts",
        });
    }

    let offset = (cone.frame().origin() - plane.frame().origin()).dot(normal);
    let z = -offset / nz;
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let v = z / cos_a;
    let signed_radius = cone.radius() + v * sin_a;

    if radial_len > tolerances.angular() {
        return intersect_elliptic_plane_cone(
            plane,
            plane_range,
            cone,
            cone_range,
            nx,
            ny,
            nz,
            radial_len,
            z,
            signed_radius,
            tolerances,
        );
    }

    if signed_radius.abs() <= tolerances.linear() {
        let mut points = Vec::new();
        add_point(
            &mut points,
            cone.apex(),
            ContactKind::Singular,
            plane,
            plane_range,
            cone,
            cone_range,
            tolerances,
        );
        return SurfaceSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    let center = cone.frame().origin() + axis * z;
    let circle = Circle::new(
        Frame::new(center, normal, plane.frame().x())?,
        signed_radius.abs(),
    )?;
    clip_circle_branch(circle, plane, plane_range, cone, cone_range, tolerances)
}

#[allow(clippy::too_many_arguments)]
fn intersect_elliptic_plane_cone(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cone: &Cone,
    cone_range: [ParamRange; 2],
    nx: f64,
    ny: f64,
    nz: f64,
    radial_len: f64,
    z0: f64,
    radius_at_axis: f64,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let tan_a = sin_a / cos_a;
    let beta = radial_len / nz;
    let conic_slope = tan_a * beta;
    let axial = 1.0 - conic_slope * conic_slope;
    if axial <= tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "plane/cone intersection currently supports only circular and elliptic cuts",
        });
    }

    if radius_at_axis.abs() <= tolerances.linear() {
        let mut points = Vec::new();
        add_point(
            &mut points,
            cone.apex(),
            ContactKind::Singular,
            plane,
            plane_range,
            cone,
            cone_range,
            tolerances,
        );
        return SurfaceSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    let radial = (cone.frame().x() * nx + cone.frame().y() * ny) / radial_len;
    let center_p = -radius_at_axis * conic_slope / axial;
    let center_z = z0 - beta * center_p;
    let center = cone.frame().origin() + radial * center_p + cone.frame().z() * center_z;
    let x_axis = (radial - cone.frame().z() * beta)
        .normalized()
        .ok_or(Error::InvalidGeometry {
            reason: "plane/cone ellipse axis has zero length",
        })?;
    let frame = Frame::new(center, plane.frame().z(), x_axis)?;
    let p_radius = radius_at_axis.abs() / axial;
    let ellipse = Ellipse::new(
        frame,
        p_radius * (1.0 + beta * beta).sqrt(),
        radius_at_axis.abs() / axial.sqrt(),
    )?;
    clip_ellipse_branch(ellipse, plane, plane_range, cone, cone_range, tolerances)
}

fn clip_circle_branch(
    circle: Circle,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let plane_hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        plane,
        plane_range,
        tolerances,
    )?;
    let cone_hit =
        intersect_bounded_circle_cone(&circle, circle.param_range(), cone, cone_range, tolerances)?;
    let mut points = Vec::new();
    let mut curves = Vec::new();
    add_clipped_branch(
        &mut points,
        &mut curves,
        SurfaceIntersectionCurve::Circle(circle),
        circle.param_range(),
        parameter_tolerance(circle.radius(), tolerances),
        &plane_hit,
        &cone_hit,
        plane,
        plane_range,
        cone,
        cone_range,
        tolerances,
    );
    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

fn clip_ellipse_branch(
    ellipse: Ellipse,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let plane_hit = intersect_bounded_ellipse_plane(
        &ellipse,
        ellipse.param_range(),
        plane,
        plane_range,
        tolerances,
    )?;
    let cone_hit = intersect_bounded_ellipse_cone(
        &ellipse,
        ellipse.param_range(),
        cone,
        cone_range,
        tolerances,
    )?;
    let mut points = Vec::new();
    let mut curves = Vec::new();
    add_clipped_branch(
        &mut points,
        &mut curves,
        SurfaceIntersectionCurve::Ellipse(ellipse),
        ellipse.param_range(),
        parameter_tolerance(ellipse.minor_radius(), tolerances),
        &plane_hit,
        &cone_hit,
        plane,
        plane_range,
        cone,
        cone_range,
        tolerances,
    );
    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

#[allow(clippy::too_many_arguments)]
fn add_clipped_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    curve: SurfaceIntersectionCurve,
    curve_range: ParamRange,
    t_tol: f64,
    plane_hit: &CurveSurfaceIntersections,
    cone_hit: &CurveSurfaceIntersections,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    for plane_overlap in &plane_hit.overlaps {
        for cone_overlap in &cone_hit.overlaps {
            let lo = plane_overlap.curve.lo.max(cone_overlap.curve.lo);
            let hi = plane_overlap.curve.hi.min(cone_overlap.curve.hi);
            if hi - lo > t_tol {
                let start = curve.eval(lo);
                let end = curve.eval(hi);
                let Some(uv_plane_start) = plane_uv_at(start, plane, plane_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_plane_end) = plane_uv_at(end, plane, plane_range, tolerances) else {
                    continue;
                };
                let Some(uv_cone_start) = cone_uv_at(start, cone, cone_range, tolerances) else {
                    continue;
                };
                let Some(uv_cone_end) = cone_uv_at(end, cone, cone_range, tolerances) else {
                    continue;
                };
                push_curve(
                    curves,
                    SurfaceSurfaceCurve {
                        curve: curve.clone(),
                        curve_range: ParamRange::new(lo, hi),
                        uv_a_start: uv_plane_start,
                        uv_a_end: uv_plane_end,
                        uv_b_start: uv_cone_start,
                        uv_b_end: uv_cone_end,
                        kind: ContactKind::Transverse,
                    },
                    t_tol.max(tolerances.linear()),
                );
            } else if (hi - lo).abs() <= t_tol {
                add_point_from_curve_parameter(
                    points,
                    &curve,
                    curve_range,
                    ((lo + hi) / 2.0).clamp(curve_range.lo, curve_range.hi),
                    ContactKind::Transverse,
                    plane,
                    plane_range,
                    cone,
                    cone_range,
                    t_tol,
                    tolerances,
                );
            }
        }
    }

    add_isolated_points(
        points,
        &curve,
        curve_range,
        plane_hit,
        cone_hit,
        plane,
        plane_range,
        cone,
        cone_range,
        t_tol,
        tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    curve: &SurfaceIntersectionCurve,
    curve_range: ParamRange,
    plane_hit: &CurveSurfaceIntersections,
    cone_hit: &CurveSurfaceIntersections,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cone: &Cone,
    cone_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    for point in &plane_hit.points {
        if hit_contains_t(cone_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                curve,
                curve_range,
                point.t_curve,
                ContactKind::Transverse,
                plane,
                plane_range,
                cone,
                cone_range,
                t_tol,
                tolerances,
            );
        }
    }
    for point in &cone_hit.points {
        if hit_contains_t(plane_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                curve,
                curve_range,
                point.t_curve,
                ContactKind::Transverse,
                plane,
                plane_range,
                cone,
                cone_range,
                t_tol,
                tolerances,
            );
        }
    }
    for plane_point in &plane_hit.points {
        for cone_point in &cone_hit.points {
            if curve_parameters_match(plane_point, cone_point, t_tol, tolerances) {
                add_point_from_curve_parameter(
                    points,
                    curve,
                    curve_range,
                    plane_point.t_curve,
                    ContactKind::Transverse,
                    plane,
                    plane_range,
                    cone,
                    cone_range,
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
    curve: &SurfaceIntersectionCurve,
    curve_range: ParamRange,
    t: f64,
    kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cone: &Cone,
    cone_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, curve_range, t_tol) else {
        return;
    };
    add_point(
        points,
        curve.eval(t),
        kind,
        plane,
        plane_range,
        cone,
        cone_range,
        tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    point: Point3,
    kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let Some(uv_plane) = plane_uv_at(point, plane, plane_range, tolerances) else {
        return;
    };
    let Some(uv_cone) = cone_uv_at(point, cone, cone_range, tolerances) else {
        return;
    };
    let kind = if plane.normal(uv_plane).is_none() || cone.normal(uv_cone).is_none() {
        ContactKind::Singular
    } else {
        kind
    };
    if let Some(point) =
        accept_surface_surface_candidate(plane, uv_plane, cone, uv_cone, kind, tolerances)
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

fn cone_uv_at(
    point: Point3,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = cone.frame().to_local(point);
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let v = fit_scalar_parameter(local.z / cos_a, cone_range[1], tolerances.linear())?;
    let signed_radius = cone.radius() + v * sin_a;
    let u = if signed_radius.abs() <= tolerances.linear() {
        cone_range[0].lo
    } else {
        let raw_u = math::atan2(local.y / signed_radius, local.x / signed_radius);
        fit_periodic_parameter(
            raw_u,
            cone_range[0],
            parameter_tolerance(signed_radius.abs(), tolerances),
        )?
    };
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
    let diff = (a - b).abs();
    diff.min((core::f64::consts::TAU - diff).abs())
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

fn validate_ranges(plane_range: [ParamRange; 2], cone_range: [ParamRange; 2]) -> Result<()> {
    if plane_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/cone intersection requires finite non-reversed plane ranges",
        });
    }
    if cone_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/cone intersection requires finite non-reversed cone ranges",
        });
    }
    Ok(())
}
