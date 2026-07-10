use super::circle_sphere::intersect_bounded_circle_sphere;
use super::circle_torus::intersect_bounded_circle_torus;
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
use kgeom::surface::{Sphere, Torus};
use kgeom::vec::{Point3, Vec3};

/// Intersect a finite sphere parameter window with a finite torus parameter
/// window.
///
/// Supports sphere centers on the torus axis, where the meridian section
/// reduces to a circle/circle solve and every meridian intersection point
/// revolves into an exact circle branch. General offset sphere/torus
/// intersections are quartic space curves and remain explicit until SSI result
/// geometry can carry that branch family.
pub fn intersect_bounded_sphere_torus(
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(sphere_range, torus_range)?;

    let center_local = torus.frame().to_local(sphere.frame().origin());
    let center_radial = Vec3::new(center_local.x, center_local.y, 0.0);
    if center_radial.norm() > tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "sphere/torus intersection currently supports only coaxial circular cuts",
        });
    }

    let major = torus.major_radius();
    let minor = torus.minor_radius();
    let sphere_radius = sphere.radius();
    let center_z = center_local.z;
    let distance = (major * major + center_z * center_z).sqrt();
    let along =
        (sphere_radius * sphere_radius - minor * minor + distance * distance) / (2.0 * distance);
    let h_sq = sphere_radius * sphere_radius - along * along;
    let sq_tol = squared_tolerance(sphere_radius, minor, distance, tolerances);
    if h_sq < -sq_tol {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let e_rho = major / distance;
    let e_z = -center_z / distance;
    let base_rho = along * e_rho;
    let base_z = center_z + along * e_z;
    let normal_rho = -e_z;
    let normal_z = e_rho;

    let mut points = Vec::new();
    let mut curves = Vec::new();
    if h_sq <= sq_tol {
        add_circle_branch(
            &mut points,
            &mut curves,
            base_rho,
            base_z,
            ContactKind::Tangent,
            sphere,
            sphere_range,
            torus,
            torus_range,
            tolerances,
        )?;
    } else {
        let h = h_sq.sqrt();
        for sign in [-1.0, 1.0] {
            add_circle_branch(
                &mut points,
                &mut curves,
                base_rho + sign * h * normal_rho,
                base_z + sign * h * normal_z,
                ContactKind::Transverse,
                sphere,
                sphere_range,
                torus,
                torus_range,
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
    radius: f64,
    z: f64,
    branch_kind: ContactKind,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    if radius <= tolerances.linear() {
        return Ok(());
    }
    let center = torus.frame().origin() + torus.frame().z() * z;
    let circle = Circle::new(
        Frame::new(center, torus.frame().z(), torus.frame().x())?,
        radius,
    )?;
    let sphere_hit = intersect_bounded_circle_sphere(
        &circle,
        circle.param_range(),
        sphere,
        sphere_range,
        tolerances,
    )?;
    let torus_hit = intersect_bounded_circle_torus(
        &circle,
        circle.param_range(),
        torus,
        torus_range,
        tolerances,
    )?;
    add_clipped_branch(
        points,
        curves,
        &circle,
        &sphere_hit,
        &torus_hit,
        branch_kind,
        sphere,
        sphere_range,
        torus,
        torus_range,
        tolerances,
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_clipped_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    circle: &Circle,
    sphere_hit: &CurveSurfaceIntersections,
    torus_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let t_tol = parameter_tolerance(circle.radius(), tolerances);
    for sphere_overlap in &sphere_hit.overlaps {
        for torus_overlap in &torus_hit.overlaps {
            let lo = sphere_overlap.curve.lo.max(torus_overlap.curve.lo);
            let hi = sphere_overlap.curve.hi.min(torus_overlap.curve.hi);
            if hi - lo > t_tol {
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
                let Some(uv_torus_start) =
                    torus_uv_at(circle.eval(lo), torus, torus_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_torus_end) =
                    torus_uv_at(circle.eval(hi), torus, torus_range, tolerances)
                else {
                    continue;
                };
                push_curve(
                    curves,
                    SurfaceSurfaceCurve {
                        curve: SurfaceIntersectionCurve::Circle(*circle),
                        curve_range: ParamRange::new(lo, hi),
                        uv_a_start: uv_sphere_start,
                        uv_a_end: uv_sphere_end,
                        uv_b_start: uv_torus_start,
                        uv_b_end: uv_torus_end,
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
                    sphere,
                    sphere_range,
                    torus,
                    torus_range,
                    t_tol,
                    tolerances,
                );
            }
        }
    }

    add_isolated_points(
        points,
        circle,
        sphere_hit,
        torus_hit,
        branch_kind,
        sphere,
        sphere_range,
        torus,
        torus_range,
        t_tol,
        tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    circle: &Circle,
    sphere_hit: &CurveSurfaceIntersections,
    torus_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    for point in &sphere_hit.points {
        if hit_contains_t(torus_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                sphere,
                sphere_range,
                torus,
                torus_range,
                t_tol,
                tolerances,
            );
        }
    }
    for point in &torus_hit.points {
        if hit_contains_t(sphere_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                sphere,
                sphere_range,
                torus,
                torus_range,
                t_tol,
                tolerances,
            );
        }
    }
    for sphere_point in &sphere_hit.points {
        for torus_point in &torus_hit.points {
            if curve_parameters_match(sphere_point, torus_point, t_tol, tolerances) {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    sphere_point.t_curve,
                    branch_kind,
                    sphere,
                    sphere_range,
                    torus,
                    torus_range,
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
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, circle.param_range(), t_tol) else {
        return;
    };
    let point = circle.eval(t);
    let Some(uv_sphere) = sphere_uv_at(point, sphere, sphere_range, tolerances) else {
        return;
    };
    let Some(uv_torus) = torus_uv_at(point, torus, torus_range, tolerances) else {
        return;
    };
    if let Some(point) =
        accept_surface_surface_candidate(sphere, uv_sphere, torus, uv_torus, kind, tolerances)
    {
        push_point(points, point, tolerances);
    }
}

fn sphere_uv_at(
    point: Point3,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = sphere.frame().to_local(point);
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let raw_v = math::atan2(local.z, xy);
    let v_tol = parameter_tolerance(sphere.radius(), tolerances);
    let v = fit_scalar_parameter(raw_v, sphere_range[1], v_tol)?;
    let u = if xy <= tolerances.linear() {
        sphere_range[0].lo
    } else {
        let raw_u = math::atan2(local.y, local.x);
        fit_periodic_parameter(raw_u, sphere_range[0], v_tol)?
    };
    Some([u, v])
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

fn squared_tolerance(
    sphere_radius: f64,
    torus_minor_radius: f64,
    center_distance: f64,
    tolerances: Tolerances,
) -> f64 {
    tolerances.linear() * (sphere_radius + torus_minor_radius + center_distance).max(1.0)
}

fn validate_ranges(sphere_range: [ParamRange; 2], torus_range: [ParamRange; 2]) -> Result<()> {
    if sphere_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "sphere/torus intersection requires finite non-reversed sphere ranges",
        });
    }
    if torus_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "sphere/torus intersection requires finite non-reversed torus ranges",
        });
    }
    Ok(())
}
