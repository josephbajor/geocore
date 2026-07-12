use super::circle_cone::intersect_bounded_circle_cone;
use super::circle_cylinder::intersect_bounded_circle_cylinder;
use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::parameter::fit_scalar_parameter;
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint,
};
use super::support_curve_pair::{SupportCurvePairConfig, emit_support_curve_pair};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Cylinder};
use kgeom::vec::Point3;

/// Intersect a finite cone parameter window with a finite cylinder parameter
/// window.
///
/// Supports coaxial cone/cylinder intersections. The cylinder radius can hit
/// either cone nappe, producing up to two exact circle branches. General offset
/// or skew cone/cylinder intersections are quartic space curves and remain
/// explicit until SSI result geometry can carry that branch family.
pub fn intersect_bounded_cone_cylinder(
    cone: &Cone,
    cone_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(cone_range, cylinder_range)?;

    let cone_axis = cone.frame().z();
    if cone_axis.cross(cylinder.frame().z()).norm() > tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "cone/cylinder intersection currently supports only coaxial circular cuts",
        });
    }

    let offset = cylinder.frame().origin() - cone.frame().origin();
    let radial_offset = offset - cone_axis * offset.dot(cone_axis);
    if radial_offset.norm() > tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "cone/cylinder intersection currently supports only coaxial circular cuts",
        });
    }

    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let mut points = Vec::new();
    let mut curves = Vec::new();
    for signed_radius in [-cylinder.radius(), cylinder.radius()] {
        let v = (signed_radius - cone.radius()) / sin_a;
        let z = v * cos_a;
        add_circle_branch(
            &mut points,
            &mut curves,
            z,
            cylinder.radius(),
            ContactKind::Transverse,
            cone,
            cone_range,
            cylinder,
            cylinder_range,
            tolerances,
        )?;
    }

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

#[allow(clippy::too_many_arguments)]
fn add_circle_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    z: f64,
    radius: f64,
    branch_kind: ContactKind,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let center = cone.frame().origin() + cone.frame().z() * z;
    let circle = Circle::new(
        Frame::new(center, cone.frame().z(), cone.frame().x())?,
        radius,
    )?;
    let cone_hit =
        intersect_bounded_circle_cone(&circle, circle.param_range(), cone, cone_range, tolerances)?;
    let cylinder_hit = intersect_bounded_circle_cylinder(
        &circle,
        circle.param_range(),
        cylinder,
        cylinder_range,
        tolerances,
    )?;
    let parameter_tolerance = parameter_tolerance(circle.radius(), tolerances);
    let curve = SurfaceIntersectionCurve::Circle(circle);
    let first_uv = |point| cone_uv_at(point, cone, cone_range, tolerances);
    let second_uv = |point| cylinder_uv_at(point, cylinder, cylinder_range, tolerances);
    emit_support_curve_pair(
        SupportCurvePairConfig {
            curve: &curve,
            curve_range: curve.param_range(),
            first_hit: &cone_hit,
            second_hit: &cylinder_hit,
            kind: branch_kind,
            parameter_tolerance,
            parameter_period: Some(core::f64::consts::TAU),
            branch_tolerance: parameter_tolerance.max(tolerances.linear()),
            first_surface: cone,
            second_surface: cylinder,
            first_uv: &first_uv,
            second_uv: &second_uv,
            tolerances,
        },
        points,
        curves,
    );
    Ok(())
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

fn validate_ranges(cone_range: [ParamRange; 2], cylinder_range: [ParamRange; 2]) -> Result<()> {
    if cone_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cone/cylinder intersection requires finite non-reversed cone ranges",
        });
    }
    if cylinder_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cone/cylinder intersection requires finite non-reversed cylinder ranges",
        });
    }
    Ok(())
}
