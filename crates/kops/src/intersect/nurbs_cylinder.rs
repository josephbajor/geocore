use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::nurbs_curve_march::{CurveMarchConfig, march_nurbs_curve_surface_intersection};
use super::result::CurveSurfaceIntersections;
use kcore::error::Result;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::Point3;

/// Intersect a clamped NURBS curve restricted to a finite range with a finite
/// cylinder parameter window.
///
/// This fixed-grid bridge samples the cylinder radial signed-distance field
/// along the NURBS curve, polishes sign-change roots and local tangent minima,
/// and reports all-on-cylinder spans as contained overlaps.
pub fn intersect_bounded_nurbs_cylinder(
    curve: &NurbsCurve,
    curve_range: ParamRange,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    let signed_distance = |point: Point3| {
        let local = cylinder.frame().to_local(point);
        let radial = (local.x * local.x + local.y * local.y).sqrt();
        radial - cylinder.radius()
    };
    let surface_uv = |point: Point3| cylinder_uv(point, cylinder, cylinder_range, tolerances);
    let surface_normal = |uv| cylinder.normal(uv);

    march_nurbs_curve_surface_intersection(CurveMarchConfig {
        curve,
        curve_range,
        surface: cylinder,
        surface_range: cylinder_range,
        tolerances,
        signed_distance: &signed_distance,
        surface_uv: &surface_uv,
        surface_normal: &surface_normal,
        finite_curve_range_reason: "nurbs/cylinder intersection requires a finite non-reversed curve range",
        finite_surface_range_reason: "nurbs/cylinder intersection requires finite non-reversed surface ranges",
        clamped_curve_reason: "nurbs/cylinder intersection requires a clamped NURBS curve",
        domain_range_reason: "nurbs/cylinder intersection curve range must lie within the NURBS domain",
    })
}

fn cylinder_uv(
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
