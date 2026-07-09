use super::conic::fit_periodic_parameter;
use super::nurbs_curve_march::{CurveMarchConfig, march_nurbs_curve_surface_intersection};
use super::result::CurveSurfaceIntersections;
use kcore::error::Result;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::{Sphere, Surface};
use kgeom::vec::Point3;

/// Intersect a clamped NURBS curve restricted to a finite range with a finite
/// sphere parameter window.
///
/// This is a fixed-grid marching bridge for broader curve/surface
/// intersection: it samples the sphere signed-distance field along the NURBS
/// curve, polishes sign-change roots, and reports all-on-surface spans as
/// contained overlaps.
pub fn intersect_bounded_nurbs_sphere(
    curve: &NurbsCurve,
    curve_range: ParamRange,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    let signed_distance = |point: Point3| {
        let local = sphere.frame().to_local(point);
        let radius = sphere.radius();
        (local.norm_sq() - radius * radius) / (local.norm() + radius)
    };
    let surface_uv = |point: Point3| sphere_uv(point, sphere, sphere_range, tolerances);
    let surface_normal = |uv| sphere.normal(uv);

    march_nurbs_curve_surface_intersection(CurveMarchConfig {
        curve,
        curve_range,
        surface: sphere,
        surface_range: sphere_range,
        tolerances,
        signed_distance: &signed_distance,
        surface_uv: &surface_uv,
        surface_normal: &surface_normal,
        finite_curve_range_reason: "nurbs/sphere intersection requires a finite non-reversed curve range",
        finite_surface_range_reason: "nurbs/sphere intersection requires finite non-reversed surface ranges",
        clamped_curve_reason: "nurbs/sphere intersection requires a clamped NURBS curve",
        domain_range_reason: "nurbs/sphere intersection curve range must lie within the NURBS domain",
    })
}

fn sphere_uv(
    point: Point3,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = sphere.frame().to_local(point);
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let raw_v = math::atan2(local.z, xy);
    let v_tol = (tolerances.linear() / sphere.radius()).max(tolerances.angular());
    let v = fit_scalar_parameter(raw_v, sphere_range[1], v_tol)?;
    let u = if xy <= tolerances.linear() {
        sphere_range[0].lo
    } else {
        let raw_u = math::atan2(local.y, local.x);
        fit_periodic_parameter(raw_u, sphere_range[0], v_tol)?
    };
    Some([u, v])
}

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}
