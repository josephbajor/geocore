use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::nurbs_surface_march::{MarchConfig, MarchPoint, march_nurbs_surface_intersection};
use super::result::{ContactKind, SurfaceSurfaceIntersections};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::{Sphere, Surface};
use kgeom::vec::{Point3, Vec3};

/// Intersect a finite sphere parameter window with a clamped NURBS surface
/// over a finite parameter window.
///
/// The branch geometry is a degree-1 NURBS polyline traced by marching the
/// sphere implicit equation over the NURBS surface parameter rectangle.
pub fn intersect_bounded_sphere_nurbs_surface(
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_sphere_range(sphere_range)?;

    let signed_distance = |point| sphere.frame().to_local(point).norm() - sphere.radius();
    let other_uv = |point| sphere_uv_at(point, sphere, sphere_range, tolerances);
    let branch_kind =
        |points: &[MarchPoint]| sphere_branch_kind(surface, sphere, points, tolerances);
    march_nurbs_surface_intersection(MarchConfig {
        surface,
        surface_range,
        tolerances,
        signed_distance: &signed_distance,
        other_uv: &other_uv,
        branch_kind: &branch_kind,
        overlap_reason: "coincident sphere/nurbs-surface intersection is a surface overlap",
        non_finite_reason: "sphere/nurbs-surface intersection sampled non-finite geometry",
        finite_range_reason: "sphere/nurbs-surface intersection requires finite non-reversed NURBS surface ranges",
        clamped_surface_reason: "sphere/nurbs-surface intersection requires a clamped NURBS surface",
        domain_range_reason: "sphere/nurbs-surface intersection surface range must lie within the NURBS domain",
    })
}

fn sphere_branch_kind(
    surface: &NurbsSurface,
    sphere: &Sphere,
    points: &[MarchPoint],
    tolerances: Tolerances,
) -> ContactKind {
    let mid = points[points.len() / 2];
    let Some(surface_normal) = surface.normal(mid.surface_uv) else {
        return ContactKind::Singular;
    };
    let Some(sphere_normal) = (mid.point - sphere.frame().origin()).normalized() else {
        return ContactKind::Singular;
    };
    if surface_normal.cross(sphere_normal).norm() <= tolerances.angular() {
        ContactKind::Tangent
    } else {
        ContactKind::Transverse
    }
}

fn sphere_uv_at(
    point: Point3,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    sphere_uv(
        sphere.frame().to_local(point),
        sphere,
        sphere_range,
        tolerances,
    )
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

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn validate_sphere_range(sphere_range: [ParamRange; 2]) -> Result<()> {
    if sphere_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "sphere/nurbs-surface intersection requires finite non-reversed sphere ranges",
        });
    }
    Ok(())
}
