use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::nurbs_surface_march::{MarchConfig, MarchPoint, march_nurbs_surface_intersection};
use super::result::{ContactKind, SurfaceSurfaceIntersections};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::Point3;

/// Intersect a finite cylinder parameter window with a clamped NURBS surface
/// over a finite parameter window.
///
/// Branches are degree-1 NURBS polylines traced by marching the cylinder's
/// implicit radial-distance equation over the NURBS surface parameter
/// rectangle.
pub fn intersect_bounded_cylinder_nurbs_surface(
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_cylinder_range(cylinder_range)?;

    let signed_distance = |point| {
        let local = cylinder.frame().to_local(point);
        (local.x * local.x + local.y * local.y).sqrt() - cylinder.radius()
    };
    let other_uv = |point| cylinder_uv_at(point, cylinder, cylinder_range, tolerances);
    let branch_kind =
        |points: &[MarchPoint]| cylinder_branch_kind(surface, cylinder, points, tolerances);
    march_nurbs_surface_intersection(MarchConfig {
        surface,
        surface_range,
        tolerances,
        signed_distance: &signed_distance,
        other_uv: &other_uv,
        branch_kind: &branch_kind,
        overlap_reason: "coincident cylinder/nurbs-surface intersection is a surface overlap",
        non_finite_reason: "cylinder/nurbs-surface intersection sampled non-finite geometry",
        finite_range_reason: "cylinder/nurbs-surface intersection requires finite non-reversed NURBS surface ranges",
        clamped_surface_reason: "cylinder/nurbs-surface intersection requires a clamped NURBS surface",
        domain_range_reason: "cylinder/nurbs-surface intersection surface range must lie within the NURBS domain",
    })
}

fn cylinder_branch_kind(
    surface: &NurbsSurface,
    cylinder: &Cylinder,
    points: &[MarchPoint],
    tolerances: Tolerances,
) -> ContactKind {
    let mid = points[points.len() / 2];
    let Some(surface_normal) = surface.normal(mid.surface_uv) else {
        return ContactKind::Singular;
    };
    let Some(cylinder_normal) = cylinder.normal(mid.other_uv) else {
        return ContactKind::Singular;
    };
    if surface_normal.cross(cylinder_normal).norm() <= tolerances.angular() {
        ContactKind::Tangent
    } else {
        ContactKind::Transverse
    }
}

fn cylinder_uv_at(
    point: Point3,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = cylinder.frame().to_local(point);
    let raw_u = math::atan2(local.y, local.x);
    let u = fit_periodic_parameter(
        raw_u,
        cylinder_range[0],
        parameter_tolerance(cylinder.radius(), tolerances),
    )?;
    let v = fit_scalar_parameter(local.z, cylinder_range[1], tolerances.linear())?;
    Some([u, v])
}

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn validate_cylinder_range(cylinder_range: [ParamRange; 2]) -> Result<()> {
    if cylinder_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/nurbs-surface intersection requires finite non-reversed cylinder ranges",
        });
    }
    Ok(())
}
