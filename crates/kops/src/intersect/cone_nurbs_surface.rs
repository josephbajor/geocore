use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::nurbs_surface_march::{MarchConfig, MarchPoint, march_nurbs_surface_intersection};
use super::result::{ContactKind, SurfaceSurfaceIntersections};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Surface};
use kgeom::vec::Point3;

/// Intersect a finite cone parameter window with a clamped NURBS surface over
/// a finite parameter window.
///
/// Branches are degree-1 NURBS polylines traced by marching the cone's
/// implicit radial-distance equation over the NURBS surface parameter
/// rectangle.
pub fn intersect_bounded_cone_nurbs_surface(
    cone: &Cone,
    cone_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_cone_range(cone_range)?;

    let signed_distance = |point| {
        let local = cone.frame().to_local(point);
        let (sin_a, cos_a) = math::sincos(cone.half_angle());
        let v = local.z / cos_a;
        let cone_radius = cone.radius() + v * sin_a;
        (local.x * local.x + local.y * local.y).sqrt() - cone_radius.abs()
    };
    let other_uv = |point| cone_uv_at(point, cone, cone_range, tolerances);
    let branch_kind = |points: &[MarchPoint]| cone_branch_kind(surface, cone, points, tolerances);
    march_nurbs_surface_intersection(MarchConfig {
        surface,
        surface_range,
        tolerances,
        implicit_surface: cone,
        implicit_empty_is_authoritative: true,
        signed_distance: &signed_distance,
        other_uv: &other_uv,
        branch_kind: &branch_kind,
        overlap_reason: "coincident cone/nurbs-surface intersection is a surface overlap",
        non_finite_reason: "cone/nurbs-surface intersection sampled non-finite geometry",
        finite_range_reason: "cone/nurbs-surface intersection requires finite non-reversed NURBS surface ranges",
        clamped_surface_reason: "cone/nurbs-surface intersection requires a clamped NURBS surface",
        domain_range_reason: "cone/nurbs-surface intersection surface range must lie within the NURBS domain",
    })
}

fn cone_branch_kind(
    surface: &NurbsSurface,
    cone: &Cone,
    points: &[MarchPoint],
    tolerances: Tolerances,
) -> ContactKind {
    let mid = points[points.len() / 2];
    let Some(surface_normal) = surface.normal(mid.surface_uv) else {
        return ContactKind::Singular;
    };
    let Some(cone_normal) = cone.normal(mid.other_uv) else {
        return ContactKind::Singular;
    };
    if surface_normal.cross(cone_normal).norm() <= tolerances.angular() {
        ContactKind::Tangent
    } else {
        ContactKind::Transverse
    }
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

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn validate_cone_range(cone_range: [ParamRange; 2]) -> Result<()> {
    if cone_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cone/nurbs-surface intersection requires finite non-reversed cone ranges",
        });
    }
    Ok(())
}
