use super::nurbs_surface_march::{MarchConfig, MarchPoint, march_nurbs_surface_intersection};
use super::result::{ContactKind, SurfaceSurfaceIntersections};
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Surface};
use kgeom::vec::Point3;

/// Intersect a finite plane window with a clamped NURBS surface over a finite
/// parameter window.
///
/// This is a fixed-grid marching bridge for the broader SSI path: it samples
/// the plane's signed-distance field over the NURBS parameter rectangle and
/// returns joined zero-contour branches as degree-1 NURBS polylines.
pub fn intersect_bounded_plane_nurbs_surface(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_plane_range(plane_range)?;

    let signed_distance = |point| plane.frame().to_local(point).z;
    let other_uv = |point| plane_uv_at(point, plane, plane_range, tolerances);
    let branch_kind = |points: &[MarchPoint]| plane_branch_kind(surface, plane, points, tolerances);
    march_nurbs_surface_intersection(MarchConfig {
        surface,
        surface_range,
        tolerances,
        signed_distance: &signed_distance,
        other_uv: &other_uv,
        branch_kind: &branch_kind,
        overlap_reason: "coincident plane/nurbs-surface intersection is a surface overlap",
        non_finite_reason: "plane/nurbs-surface intersection sampled non-finite geometry",
        finite_range_reason: "plane/nurbs-surface intersection requires finite non-reversed NURBS surface ranges",
        clamped_surface_reason: "plane/nurbs-surface intersection requires a clamped NURBS surface",
        domain_range_reason: "plane/nurbs-surface intersection surface range must lie within the NURBS domain",
    })
}

fn plane_branch_kind(
    surface: &NurbsSurface,
    plane: &Plane,
    points: &[MarchPoint],
    tolerances: Tolerances,
) -> ContactKind {
    let mid = points[points.len() / 2].surface_uv;
    let Some(surface_normal) = surface.normal(mid) else {
        return ContactKind::Singular;
    };
    if surface_normal.cross(plane.frame().z()).norm() <= tolerances.angular() {
        ContactKind::Tangent
    } else {
        ContactKind::Transverse
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

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn validate_plane_range(plane_range: [ParamRange; 2]) -> Result<()> {
    if plane_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/nurbs-surface intersection requires finite non-reversed plane ranges",
        });
    }
    Ok(())
}
