use super::circle_cone::intersect_bounded_circle_cone;
use super::circle_sphere::clip_bounded_circle_on_sphere;
use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::parameter::fit_scalar_parameter;
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, accept_surface_surface_candidate,
};
use super::support_curve_pair::{SupportCurvePairConfig, emit_support_curve_pair};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Sphere, Surface};
use kgeom::vec::{Point3, Vec3};

/// Intersect a finite cone parameter window with a finite sphere parameter
/// window.
///
/// Supports coaxial cone/sphere intersections, which produce circle branches
/// and possible singular apex contacts. General offset cone/sphere
/// intersections remain explicit until SSI result geometry can carry those
/// higher-order branch families.
pub fn intersect_bounded_cone_sphere(
    cone: &Cone,
    cone_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(cone_range, sphere_range)?;

    let center_local = cone.frame().to_local(sphere.frame().origin());
    let center_radial = Vec3::new(center_local.x, center_local.y, 0.0);
    if center_radial.norm() > tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "cone/sphere intersection currently supports only coaxial circular cuts",
        });
    }

    let roots = axial_roots(cone, center_local.z, sphere.radius(), tolerances);
    let mut points = Vec::new();
    let mut curves = Vec::new();
    for root in roots {
        let signed_radius = cone.radius() + root.z * root.tan_a;
        if signed_radius.abs() <= tolerances.linear() {
            add_point(
                &mut points,
                cone.apex(),
                cone,
                cone_range,
                sphere,
                sphere_range,
                ContactKind::Singular,
                tolerances,
            );
        } else {
            add_circle_branch(
                &mut points,
                &mut curves,
                root.z,
                signed_radius,
                root.kind,
                cone,
                cone_range,
                sphere,
                sphere_range,
                tolerances,
            )?;
        }
    }

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

#[derive(Clone, Copy)]
struct AxialRoot {
    z: f64,
    tan_a: f64,
    kind: ContactKind,
}

fn axial_roots(
    cone: &Cone,
    sphere_center_z: f64,
    sphere_radius: f64,
    tolerances: Tolerances,
) -> Vec<AxialRoot> {
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let tan_a = sin_a / cos_a;
    let a = 1.0 + tan_a * tan_a;
    let b = 2.0 * (cone.radius() * tan_a - sphere_center_z);
    let c = cone.radius() * cone.radius() + sphere_center_z * sphere_center_z
        - sphere_radius * sphere_radius;
    let discriminant = b * b - 4.0 * a * c;
    let discriminant_tolerance = tolerances.linear()
        * (a.abs() + b.abs() + c.abs() + sphere_radius + cone.radius()).max(1.0);
    if discriminant < -discriminant_tolerance {
        return Vec::new();
    }
    if discriminant.abs() <= discriminant_tolerance {
        return vec![AxialRoot {
            z: -b / (2.0 * a),
            tan_a,
            kind: ContactKind::Tangent,
        }];
    }

    let root = discriminant.sqrt();
    [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)]
        .into_iter()
        .map(|z| AxialRoot {
            z,
            tan_a,
            kind: ContactKind::Transverse,
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn add_circle_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    z: f64,
    signed_radius: f64,
    branch_kind: ContactKind,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let center = cone.frame().origin() + cone.frame().z() * z;
    let circle = Circle::new(
        Frame::new(center, cone.frame().z(), cone.frame().x())?,
        signed_radius.abs(),
    )?;
    let cone_hit =
        intersect_bounded_circle_cone(&circle, circle.param_range(), cone, cone_range, tolerances)?;
    let sphere_hit = clip_bounded_circle_on_sphere(
        &circle,
        circle.param_range(),
        sphere,
        sphere_range,
        tolerances,
    )?;
    let parameter_tolerance = parameter_tolerance(circle.radius(), tolerances);
    let curve = SurfaceIntersectionCurve::Circle(circle);
    let first_uv = |point| cone_uv_at(point, cone, cone_range, tolerances);
    let second_uv = |point| sphere_uv_at(point, sphere, sphere_range, tolerances);
    emit_support_curve_pair(
        SupportCurvePairConfig {
            curve: &curve,
            curve_range: curve.param_range(),
            first_hit: &cone_hit,
            second_hit: &sphere_hit,
            kind: branch_kind,
            parameter_tolerance,
            parameter_period: Some(core::f64::consts::TAU),
            branch_tolerance: parameter_tolerance.max(tolerances.linear()),
            first_surface: cone,
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

#[allow(clippy::too_many_arguments)]
fn add_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    point: Point3,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    kind: ContactKind,
    tolerances: Tolerances,
) {
    let Some(uv_cone) = cone_uv_at(point, cone, cone_range, tolerances) else {
        return;
    };
    let Some(uv_sphere) = sphere_uv_at(point, sphere, sphere_range, tolerances) else {
        return;
    };
    let kind = if cone.normal(uv_cone).is_none() || sphere.normal(uv_sphere).is_none() {
        ContactKind::Singular
    } else {
        kind
    };
    if let Some(point) =
        accept_surface_surface_candidate(cone, uv_cone, sphere, uv_sphere, kind, tolerances)
    {
        push_point(points, point, tolerances);
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

fn push_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    candidate: SurfaceSurfacePoint,
    tolerances: Tolerances,
) {
    if !points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        points.push(candidate);
    }
}

fn validate_ranges(cone_range: [ParamRange; 2], sphere_range: [ParamRange; 2]) -> Result<()> {
    if cone_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cone/sphere intersection requires finite non-reversed cone ranges",
        });
    }
    if sphere_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cone/sphere intersection requires finite non-reversed sphere ranges",
        });
    }
    Ok(())
}
