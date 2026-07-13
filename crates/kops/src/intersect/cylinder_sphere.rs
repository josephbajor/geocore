use super::circle_cylinder::intersect_bounded_circle_cylinder;
use super::circle_sphere::intersect_bounded_circle_sphere;
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
use kgeom::surface::{Cylinder, Sphere};
use kgeom::vec::{Point3, Vec3};

/// Intersect a finite cylinder parameter window with a finite sphere parameter
/// window.
///
/// Supports coaxial cylinder/sphere intersections, which produce one tangent
/// circle or two transverse circles. General offset cylinder/sphere
/// intersections are quartic space curves and remain explicit until SSI result
/// geometry can carry that branch family.
pub fn intersect_bounded_cylinder_sphere(
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(cylinder_range, sphere_range)?;

    let center_local = cylinder.frame().to_local(sphere.frame().origin());
    let center_radial = Vec3::new(center_local.x, center_local.y, 0.0);
    if center_radial.norm() > tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/sphere intersection currently supports only coaxial circular cuts",
        });
    }

    let cylinder_radius = cylinder.radius();
    let sphere_radius = sphere.radius();
    let h_sq = sphere_radius * sphere_radius - cylinder_radius * cylinder_radius;
    let sq_tol = squared_tolerance(cylinder_radius, sphere_radius, tolerances);
    if h_sq < -sq_tol {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let mut points = Vec::new();
    let mut curves = Vec::new();
    if h_sq <= sq_tol {
        add_circle_branch(
            &mut points,
            &mut curves,
            center_local.z,
            ContactKind::Tangent,
            cylinder,
            cylinder_range,
            sphere,
            sphere_range,
            tolerances,
        )?;
    } else {
        let h = h_sq.sqrt();
        for z in [center_local.z - h, center_local.z + h] {
            add_circle_branch(
                &mut points,
                &mut curves,
                z,
                ContactKind::Transverse,
                cylinder,
                cylinder_range,
                sphere,
                sphere_range,
                tolerances,
            )?;
        }
    }

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

#[allow(clippy::too_many_arguments)]
fn add_circle_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    z: f64,
    branch_kind: ContactKind,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let center = cylinder.frame().origin() + cylinder.frame().z() * z;
    let circle = Circle::new(
        Frame::new(center, cylinder.frame().z(), cylinder.frame().x())?,
        cylinder.radius(),
    )?;
    let cylinder_hit = intersect_bounded_circle_cylinder(
        &circle,
        circle.param_range(),
        cylinder,
        cylinder_range,
        tolerances,
    )?;
    let sphere_hit = intersect_bounded_circle_sphere(
        &circle,
        circle.param_range(),
        sphere,
        sphere_range,
        tolerances,
    )?;
    let parameter_tolerance = parameter_tolerance(circle.radius(), tolerances);
    let curve = SurfaceIntersectionCurve::Circle(circle);
    let first_uv = |point| cylinder_uv_at(point, cylinder, cylinder_range, tolerances);
    let second_uv = |point| sphere_uv_at(point, sphere, sphere_range, tolerances);
    emit_support_curve_pair(
        SupportCurvePairConfig {
            curve: &curve,
            curve_range: curve.param_range(),
            first_hit: &cylinder_hit,
            second_hit: &sphere_hit,
            kind: branch_kind,
            parameter_tolerance,
            parameter_period: Some(core::f64::consts::TAU),
            branch_tolerance: parameter_tolerance.max(tolerances.linear()),
            first_surface: cylinder,
            second_surface: sphere,
            first_uv: &first_uv,
            second_uv: &second_uv,
            tolerances,
        },
        points,
        curves,
    );
    Ok(())
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

fn sphere_uv_at(
    point: Point3,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = sphere.frame().to_local(point);
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let v_tol = parameter_tolerance(sphere.radius(), tolerances);
    let raw_v = math::atan2(local.z, xy);
    let v = fit_scalar_parameter(raw_v, sphere_range[1], v_tol)?;
    let u = if xy <= tolerances.linear() {
        sphere_range[0].lo
    } else {
        let raw_u = math::atan2(local.y, local.x);
        fit_periodic_parameter(raw_u, sphere_range[0], v_tol)?
    };
    Some([u, v])
}

fn squared_tolerance(cylinder_radius: f64, sphere_radius: f64, tolerances: Tolerances) -> f64 {
    tolerances.linear() * (cylinder_radius + sphere_radius).max(1.0)
}

fn validate_ranges(cylinder_range: [ParamRange; 2], sphere_range: [ParamRange; 2]) -> Result<()> {
    if cylinder_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/sphere intersection requires finite non-reversed cylinder ranges",
        });
    }
    if sphere_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/sphere intersection requires finite non-reversed sphere ranges",
        });
    }
    Ok(())
}
