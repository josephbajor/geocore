use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::nurbs_curve_march::{CurveMarchConfig, march_nurbs_curve_surface_intersection};
use super::result::CurveSurfaceIntersections;
use kcore::error::Result;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::{Surface, Torus};
use kgeom::vec::Point3;

/// Intersect a clamped NURBS curve restricted to a finite range with a finite
/// torus parameter window.
///
/// This fixed-grid bridge samples the torus tube signed-distance field along
/// the NURBS curve, polishes sign-change roots and local tangent minima, and
/// reports all-on-torus spans as contained overlaps.
pub fn intersect_bounded_nurbs_torus(
    curve: &NurbsCurve,
    curve_range: ParamRange,
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    let signed_distance = |point: Point3| {
        let local = torus.frame().to_local(point);
        let xy = (local.x * local.x + local.y * local.y).sqrt();
        let dr = xy - torus.major_radius();
        let tube_sq = dr * dr + local.z * local.z;
        let tube = tube_sq.sqrt();
        (tube_sq - torus.minor_radius() * torus.minor_radius()) / (tube + torus.minor_radius())
    };
    let surface_uv = |point: Point3| torus_uv(point, torus, torus_range, tolerances);
    let surface_normal = |uv| torus.normal(uv);

    march_nurbs_curve_surface_intersection(CurveMarchConfig {
        curve,
        curve_range,
        surface: torus,
        surface_range: torus_range,
        tolerances,
        signed_distance: &signed_distance,
        surface_uv: &surface_uv,
        surface_normal: &surface_normal,
        finite_curve_range_reason: "nurbs/torus intersection requires a finite non-reversed curve range",
        finite_surface_range_reason: "nurbs/torus intersection requires finite non-reversed surface ranges",
        clamped_curve_reason: "nurbs/torus intersection requires a clamped NURBS curve",
        domain_range_reason: "nurbs/torus intersection curve range must lie within the NURBS domain",
    })
}

fn torus_uv(
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
