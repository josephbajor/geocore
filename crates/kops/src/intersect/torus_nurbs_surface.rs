use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::nurbs_surface_march::{MarchConfig, MarchPoint, march_nurbs_surface_intersection};
use super::result::{ContactKind, SurfaceSurfaceIntersections};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::{Surface, Torus};
use kgeom::vec::Point3;

/// Intersect a finite torus parameter window with a clamped NURBS surface over
/// a finite parameter window.
///
/// Branches are degree-1 NURBS polylines traced by marching the torus tube
/// implicit equation over the NURBS surface parameter rectangle.
pub fn intersect_bounded_torus_nurbs_surface(
    torus: &Torus,
    torus_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_torus_range(torus_range)?;

    let signed_distance = |point| {
        let local = torus.frame().to_local(point);
        let xy = (local.x * local.x + local.y * local.y).sqrt();
        let dr = xy - torus.major_radius();
        (dr * dr + local.z * local.z).sqrt() - torus.minor_radius()
    };
    let other_uv = |point| torus_uv_at(point, torus, torus_range, tolerances);
    let branch_kind = |points: &[MarchPoint]| torus_branch_kind(surface, torus, points, tolerances);
    march_nurbs_surface_intersection(MarchConfig {
        surface,
        surface_range,
        tolerances,
        signed_distance: &signed_distance,
        other_uv: &other_uv,
        branch_kind: &branch_kind,
        overlap_reason: "coincident torus/nurbs-surface intersection is a surface overlap",
        non_finite_reason: "torus/nurbs-surface intersection sampled non-finite geometry",
        finite_range_reason: "torus/nurbs-surface intersection requires finite non-reversed NURBS surface ranges",
        clamped_surface_reason: "torus/nurbs-surface intersection requires a clamped NURBS surface",
        domain_range_reason: "torus/nurbs-surface intersection surface range must lie within the NURBS domain",
    })
}

fn torus_branch_kind(
    surface: &NurbsSurface,
    torus: &Torus,
    points: &[MarchPoint],
    tolerances: Tolerances,
) -> ContactKind {
    let mid = points[points.len() / 2];
    let Some(surface_normal) = surface.normal(mid.surface_uv) else {
        return ContactKind::Singular;
    };
    let Some(torus_normal) = torus.normal(mid.other_uv) else {
        return ContactKind::Singular;
    };
    if surface_normal.cross(torus_normal).norm() <= tolerances.angular() {
        ContactKind::Tangent
    } else {
        ContactKind::Transverse
    }
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

fn validate_torus_range(torus_range: [ParamRange; 2]) -> Result<()> {
    if torus_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "torus/nurbs-surface intersection requires finite non-reversed torus ranges",
        });
    }
    Ok(())
}
