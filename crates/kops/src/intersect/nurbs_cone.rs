use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::nurbs_curve_march::{CurveMarchConfig, march_nurbs_curve_surface_intersection};
use super::result::CurveSurfaceIntersections;
use kcore::error::Result;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Surface};
use kgeom::vec::Point3;

/// Intersect a clamped NURBS curve restricted to a finite range with a finite
/// cone parameter window.
///
/// This fixed-grid bridge samples the cone radial signed-distance field along
/// the NURBS curve, polishes sign-change roots and local tangent minima, and
/// reports all-on-cone spans as contained overlaps.
pub fn intersect_bounded_nurbs_cone(
    curve: &NurbsCurve,
    curve_range: ParamRange,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    let signed_distance = |point: Point3| {
        let local = cone.frame().to_local(point);
        let (sin_a, cos_a) = math::sincos(cone.half_angle());
        let signed_radius = cone.radius() + local.z * (sin_a / cos_a);
        let radial = (local.x * local.x + local.y * local.y).sqrt();
        radial - signed_radius.abs()
    };
    let surface_uv = |point: Point3| cone_uv(point, cone, cone_range, tolerances);
    let surface_normal = |uv| cone.normal(uv);

    march_nurbs_curve_surface_intersection(CurveMarchConfig {
        curve,
        curve_range,
        surface: cone,
        surface_range: cone_range,
        tolerances,
        signed_distance: &signed_distance,
        surface_uv: &surface_uv,
        surface_normal: &surface_normal,
        finite_curve_range_reason: "nurbs/cone intersection requires a finite non-reversed curve range",
        finite_surface_range_reason: "nurbs/cone intersection requires finite non-reversed surface ranges",
        clamped_curve_reason: "nurbs/cone intersection requires a clamped NURBS curve",
        domain_range_reason: "nurbs/cone intersection curve range must lie within the NURBS domain",
    })
}

fn cone_uv(
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
