use super::circle_cone::intersect_bounded_circle_cone;
use super::circle_sphere::intersect_bounded_circle_sphere;
use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, accept_surface_surface_candidate,
};
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

    SurfaceSurfaceIntersections::canonicalized(points, curves)
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
    let sphere_hit = intersect_bounded_circle_sphere(
        &circle,
        circle.param_range(),
        sphere,
        sphere_range,
        tolerances,
    )?;
    add_clipped_branch(
        points,
        curves,
        &circle,
        &cone_hit,
        &sphere_hit,
        branch_kind,
        cone,
        cone_range,
        sphere,
        sphere_range,
        tolerances,
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_clipped_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    circle: &Circle,
    cone_hit: &CurveSurfaceIntersections,
    sphere_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let t_tol = parameter_tolerance(circle.radius(), tolerances);
    for cone_overlap in &cone_hit.overlaps {
        for sphere_overlap in &sphere_hit.overlaps {
            let lo = cone_overlap.curve.lo.max(sphere_overlap.curve.lo);
            let hi = cone_overlap.curve.hi.min(sphere_overlap.curve.hi);
            if hi - lo > t_tol {
                let Some(uv_cone_start) = cone_uv_at(circle.eval(lo), cone, cone_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_cone_end) = cone_uv_at(circle.eval(hi), cone, cone_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_sphere_start) =
                    sphere_uv_at(circle.eval(lo), sphere, sphere_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_sphere_end) =
                    sphere_uv_at(circle.eval(hi), sphere, sphere_range, tolerances)
                else {
                    continue;
                };
                push_curve(
                    curves,
                    SurfaceSurfaceCurve {
                        curve: SurfaceIntersectionCurve::Circle(*circle),
                        curve_range: ParamRange::new(lo, hi),
                        uv_a_start: uv_cone_start,
                        uv_a_end: uv_cone_end,
                        uv_b_start: uv_sphere_start,
                        uv_b_end: uv_sphere_end,
                        kind: branch_kind,
                    },
                    t_tol.max(tolerances.linear()),
                );
            } else if (hi - lo).abs() <= t_tol {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    ((lo + hi) / 2.0).clamp(circle.param_range().lo, circle.param_range().hi),
                    branch_kind,
                    cone,
                    cone_range,
                    sphere,
                    sphere_range,
                    t_tol,
                    tolerances,
                );
            }
        }
    }

    add_isolated_points(
        points,
        circle,
        cone_hit,
        sphere_hit,
        branch_kind,
        cone,
        cone_range,
        sphere,
        sphere_range,
        t_tol,
        tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    circle: &Circle,
    cone_hit: &CurveSurfaceIntersections,
    sphere_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    for point in &cone_hit.points {
        if hit_contains_t(sphere_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                cone,
                cone_range,
                sphere,
                sphere_range,
                t_tol,
                tolerances,
            );
        }
    }
    for point in &sphere_hit.points {
        if hit_contains_t(cone_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                cone,
                cone_range,
                sphere,
                sphere_range,
                t_tol,
                tolerances,
            );
        }
    }
    for cone_point in &cone_hit.points {
        for sphere_point in &sphere_hit.points {
            if curve_parameters_match(cone_point, sphere_point, t_tol, tolerances) {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    cone_point.t_curve,
                    branch_kind,
                    cone,
                    cone_range,
                    sphere,
                    sphere_range,
                    t_tol,
                    tolerances,
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn add_point_from_curve_parameter(
    points: &mut Vec<SurfaceSurfacePoint>,
    circle: &Circle,
    t: f64,
    kind: ContactKind,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, circle.param_range(), t_tol) else {
        return;
    };
    add_point(
        points,
        circle.eval(t),
        cone,
        cone_range,
        sphere,
        sphere_range,
        kind,
        tolerances,
    );
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

fn hit_contains_t(
    hit: &CurveSurfaceIntersections,
    t: f64,
    t_tol: f64,
    tolerances: Tolerances,
) -> bool {
    hit.overlaps
        .iter()
        .any(|overlap| overlap_contains_t(overlap, t, t_tol))
        || hit.points.iter().any(|point| {
            curve_parameter_distance(point.t_curve, t) <= t_tol.max(tolerances.angular())
        })
}

fn overlap_contains_t(overlap: &CurveSurfaceOverlap, t: f64, t_tol: f64) -> bool {
    [t, t - core::f64::consts::TAU, t + core::f64::consts::TAU]
        .into_iter()
        .any(|candidate| {
            candidate >= overlap.curve.lo - t_tol && candidate <= overlap.curve.hi + t_tol
        })
}

fn curve_parameters_match(
    a: &CurveSurfacePoint,
    b: &CurveSurfacePoint,
    t_tol: f64,
    tolerances: Tolerances,
) -> bool {
    curve_parameter_distance(a.t_curve, b.t_curve) <= t_tol.max(tolerances.angular())
        || a.point.dist(b.point) <= tolerances.linear()
}

fn curve_parameter_distance(a: f64, b: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let diff = (a - b).abs();
    diff.min((period - diff).abs())
}

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
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

fn push_curve(
    curves: &mut Vec<SurfaceSurfaceCurve>,
    candidate: SurfaceSurfaceCurve,
    tolerance: f64,
) {
    if !curves.iter().any(|curve| {
        (curve.curve_range.lo - candidate.curve_range.lo).abs() <= tolerance
            && (curve.curve_range.hi - candidate.curve_range.hi).abs() <= tolerance
            && curve
                .curve
                .eval(curve.curve_range.lo)
                .dist(candidate.curve.eval(candidate.curve_range.lo))
                <= tolerance
            && curve
                .curve
                .eval(curve.curve_range.hi)
                .dist(candidate.curve.eval(candidate.curve_range.hi))
                <= tolerance
    }) {
        curves.push(candidate);
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
