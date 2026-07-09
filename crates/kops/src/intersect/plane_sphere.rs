use super::circle_sphere::intersect_bounded_circle_sphere;
use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::planar_curve_plane::intersect_bounded_circle_plane;
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, accept_surface_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere, Surface};
use kgeom::vec::{Point3, Vec3};

/// Intersect a finite plane window with a finite sphere parameter window.
pub fn intersect_bounded_plane_sphere(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(plane_range, sphere_range)?;

    let normal = plane.frame().z();
    let signed_distance = (sphere.frame().origin() - plane.frame().origin()).dot(normal);
    let distance = signed_distance.abs();
    if distance > sphere.radius() + tolerances.linear() {
        return Ok(SurfaceSurfaceIntersections::default());
    }

    if (distance - sphere.radius()).abs() <= tolerances.linear() {
        let point = sphere.frame().origin() - normal * signed_distance;
        let mut points = Vec::new();
        add_tangent_point(
            &mut points,
            point,
            plane,
            plane_range,
            sphere,
            sphere_range,
            tolerances,
        );
        return SurfaceSurfaceIntersections::canonicalized(points, Vec::new());
    }

    let center = sphere.frame().origin() - normal * signed_distance;
    let radius = (sphere.radius() * sphere.radius() - signed_distance * signed_distance)
        .max(0.0)
        .sqrt();
    let frame = Frame::new(center, normal, plane.frame().x())?;
    let circle = Circle::new(frame, radius)?;
    let plane_hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        plane,
        plane_range,
        tolerances,
    )?;
    let sphere_hit = intersect_bounded_circle_sphere(
        &circle,
        circle.param_range(),
        sphere,
        sphere_range,
        tolerances,
    )?;

    let t_tol = parameter_tolerance(circle.radius(), tolerances);
    let mut points = Vec::new();
    let mut curves = Vec::new();
    for plane_overlap in &plane_hit.overlaps {
        for sphere_overlap in &sphere_hit.overlaps {
            let lo = plane_overlap.curve.lo.max(sphere_overlap.curve.lo);
            let hi = plane_overlap.curve.hi.min(sphere_overlap.curve.hi);
            if hi - lo > t_tol {
                let Some(uv_a_start) = fit_uv(plane_uv(circle.eval(lo), plane), plane_range) else {
                    continue;
                };
                let Some(uv_a_end) = fit_uv(plane_uv(circle.eval(hi), plane), plane_range) else {
                    continue;
                };
                let Some(uv_b_start) = sphere_uv(
                    sphere.frame().to_local(circle.eval(lo)),
                    sphere,
                    sphere_range,
                    tolerances,
                ) else {
                    continue;
                };
                let Some(uv_b_end) = sphere_uv(
                    sphere.frame().to_local(circle.eval(hi)),
                    sphere,
                    sphere_range,
                    tolerances,
                ) else {
                    continue;
                };
                push_curve(
                    &mut curves,
                    SurfaceSurfaceCurve {
                        curve: SurfaceIntersectionCurve::Circle(circle),
                        curve_range: ParamRange::new(lo, hi),
                        uv_a_start,
                        uv_a_end,
                        uv_b_start,
                        uv_b_end,
                        kind: ContactKind::Transverse,
                    },
                    t_tol,
                );
            } else if (hi - lo).abs() <= t_tol {
                let t = ((lo + hi) / 2.0).clamp(circle.param_range().lo, circle.param_range().hi);
                add_boundary_point(
                    &mut points,
                    circle.eval(t),
                    plane,
                    plane_range,
                    sphere,
                    sphere_range,
                    tolerances,
                );
            }
        }
    }

    SurfaceSurfaceIntersections::canonicalized(points, curves)
}

fn add_tangent_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    point: Point3,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let Some(uv_plane) = fit_uv(plane_uv(point, plane), plane_range) else {
        return;
    };
    let Some(uv_sphere) = sphere_uv(
        sphere.frame().to_local(point),
        sphere,
        sphere_range,
        tolerances,
    ) else {
        return;
    };
    let kind = if plane.normal(uv_plane).is_none() || sphere.normal(uv_sphere).is_none() {
        ContactKind::Singular
    } else {
        ContactKind::Tangent
    };
    if let Some(point) =
        accept_surface_surface_candidate(plane, uv_plane, sphere, uv_sphere, kind, tolerances)
    {
        push_point(points, point, tolerances);
    }
}

fn add_boundary_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    point: Point3,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let Some(uv_plane) = fit_uv(plane_uv(point, plane), plane_range) else {
        return;
    };
    let Some(uv_sphere) = sphere_uv(
        sphere.frame().to_local(point),
        sphere,
        sphere_range,
        tolerances,
    ) else {
        return;
    };
    if let Some(point) = accept_surface_surface_candidate(
        plane,
        uv_plane,
        sphere,
        uv_sphere,
        ContactKind::Tangent,
        tolerances,
    ) {
        push_point(points, point, tolerances);
    }
}

fn plane_uv(point: Point3, plane: &Plane) -> [f64; 2] {
    let local = plane.frame().to_local(point);
    [local.x, local.y]
}

fn sphere_uv(
    local: Vec3,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let raw_v = math::atan2(local.z, xy);
    let v_tol = parameter_tolerance(sphere.radius(), tolerances);
    let v = fit_scalar_parameter(raw_v, sphere_range[1], v_tol)?;
    let u = if xy <= tolerances.linear() {
        sphere_range[0].lo
    } else {
        let raw_u = math::atan2(local.y, local.x);
        fit_periodic_parameter(raw_u, sphere_range[0], v_tol)?
    };
    Some([u, v])
}

fn fit_uv(candidate: [f64; 2], ranges: [ParamRange; 2]) -> Option<[f64; 2]> {
    Some([
        fit_scalar_parameter(candidate[0], ranges[0], 0.0)?,
        fit_scalar_parameter(candidate[1], ranges[1], 0.0)?,
    ])
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
    }) {
        curves.push(candidate);
    }
}

fn validate_ranges(plane_range: [ParamRange; 2], sphere_range: [ParamRange; 2]) -> Result<()> {
    if plane_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/sphere intersection requires finite non-reversed plane ranges",
        });
    }
    if sphere_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/sphere intersection requires finite non-reversed sphere ranges",
        });
    }
    Ok(())
}
