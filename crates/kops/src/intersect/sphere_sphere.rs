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
use kgeom::surface::{Sphere, Surface};
use kgeom::vec::{Point3, Vec3};

/// Intersect two finite sphere parameter windows.
pub fn intersect_bounded_spheres(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(a_range, b_range)?;

    let delta = b.frame().origin() - a.frame().origin();
    let distance = delta.norm();
    let radius_a = a.radius();
    let radius_b = b.radius();
    if distance <= tolerances.linear() {
        if (radius_a - radius_b).abs() <= tolerances.linear() {
            return Err(Error::InvalidGeometry {
                reason: "coincident sphere/sphere intersection is a surface overlap",
            });
        }
        return Ok(SurfaceSurfaceIntersections::default());
    }

    if distance > radius_a + radius_b + tolerances.linear()
        || distance < (radius_a - radius_b).abs() - tolerances.linear()
    {
        return Ok(SurfaceSurfaceIntersections::default());
    }

    let axis = delta / distance;
    let center_offset =
        (radius_a * radius_a - radius_b * radius_b + distance * distance) / (2.0 * distance);
    let circle_radius_sq = radius_a * radius_a - center_offset * center_offset;
    let sq_tol = squared_tolerance(distance, radius_a, radius_b, tolerances);
    if circle_radius_sq < -sq_tol {
        return Ok(SurfaceSurfaceIntersections::default());
    }
    if circle_radius_sq <= sq_tol {
        let point = tangent_point(a.frame().origin(), axis, center_offset, radius_a);
        let mut points = Vec::new();
        add_point(
            &mut points,
            point,
            a,
            a_range,
            b,
            b_range,
            ContactKind::Tangent,
            tolerances,
        );
        return SurfaceSurfaceIntersections::canonicalized(points, Vec::new());
    }

    let circle_center = a.frame().origin() + axis * center_offset;
    let circle = Circle::new(
        Frame::from_z(circle_center, axis)?,
        circle_radius_sq.max(0.0).sqrt(),
    )?;
    let a_hit =
        intersect_bounded_circle_sphere(&circle, circle.param_range(), a, a_range, tolerances)?;
    let b_hit =
        intersect_bounded_circle_sphere(&circle, circle.param_range(), b, b_range, tolerances)?;

    let t_tol = parameter_tolerance(circle.radius(), tolerances);
    let mut points = Vec::new();
    let mut curves = Vec::new();
    add_overlap_intersections(
        &mut points,
        &mut curves,
        &circle,
        &a_hit,
        &b_hit,
        a,
        a_range,
        b,
        b_range,
        t_tol,
        tolerances,
    );
    add_isolated_point_intersections(
        &mut points,
        &circle,
        &a_hit,
        &b_hit,
        a,
        a_range,
        b,
        b_range,
        t_tol,
        tolerances,
    );

    SurfaceSurfaceIntersections::canonicalized(points, curves)
}

#[allow(clippy::too_many_arguments)]
fn add_overlap_intersections(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    circle: &Circle,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    for a_overlap in &a_hit.overlaps {
        for b_overlap in &b_hit.overlaps {
            let lo = a_overlap.curve.lo.max(b_overlap.curve.lo);
            let hi = a_overlap.curve.hi.min(b_overlap.curve.hi);
            if hi - lo > t_tol {
                let Some(uv_a_start) = sphere_uv_at(circle.eval(lo), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_a_end) = sphere_uv_at(circle.eval(hi), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_start) = sphere_uv_at(circle.eval(lo), b, b_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_end) = sphere_uv_at(circle.eval(hi), b, b_range, tolerances) else {
                    continue;
                };
                push_curve(
                    curves,
                    SurfaceSurfaceCurve {
                        curve: SurfaceIntersectionCurve::Circle(*circle),
                        curve_range: ParamRange::new(lo, hi),
                        uv_a_start,
                        uv_a_end,
                        uv_b_start,
                        uv_b_end,
                        kind: ContactKind::Transverse,
                    },
                    t_tol,
                );
            } else if (hi - lo).abs() <= t_tol {
                let t = ((lo + hi) / 2.0).clamp(circle.param_range().lo, circle.param_range().hi);
                add_point(
                    points,
                    circle.eval(t),
                    a,
                    a_range,
                    b,
                    b_range,
                    ContactKind::Transverse,
                    tolerances,
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_point_intersections(
    points: &mut Vec<SurfaceSurfacePoint>,
    circle: &Circle,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    for point in &a_hit.points {
        if hit_contains_t(b_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                a,
                a_range,
                b,
                b_range,
                ContactKind::Transverse,
                tolerances,
            );
        }
    }
    for point in &b_hit.points {
        if hit_contains_t(a_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                a,
                a_range,
                b,
                b_range,
                ContactKind::Transverse,
                tolerances,
            );
        }
    }
    for a_point in &a_hit.points {
        for b_point in &b_hit.points {
            if curve_parameters_match(a_point, b_point, t_tol, tolerances) {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    a_point.t_curve,
                    a,
                    a_range,
                    b,
                    b_range,
                    ContactKind::Transverse,
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
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    kind: ContactKind,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, circle.param_range(), t_tol(circle, tolerances)) else {
        return;
    };
    add_point(
        points,
        circle.eval(t),
        a,
        a_range,
        b,
        b_range,
        kind,
        tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    point: Point3,
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    kind: ContactKind,
    tolerances: Tolerances,
) {
    let Some(uv_a) = sphere_uv_at(point, a, a_range, tolerances) else {
        return;
    };
    let Some(uv_b) = sphere_uv_at(point, b, b_range, tolerances) else {
        return;
    };
    let kind = if a.normal(uv_a).is_none() || b.normal(uv_b).is_none() {
        ContactKind::Singular
    } else {
        kind
    };
    if let Some(point) = accept_surface_surface_candidate(a, uv_a, b, uv_b, kind, tolerances) {
        push_point(points, point, tolerances);
    }
}

fn tangent_point(origin: Point3, axis: Vec3, center_offset: f64, radius: f64) -> Point3 {
    let sign = if center_offset < 0.0 { -1.0 } else { 1.0 };
    origin + axis * (sign * radius)
}

fn sphere_uv_at(
    point: Point3,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    sphere_uv(
        sphere.frame().to_local(point),
        sphere,
        sphere_range,
        tolerances,
    )
}

fn sphere_uv(
    local: Vec3,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
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

fn t_tol(circle: &Circle, tolerances: Tolerances) -> f64 {
    parameter_tolerance(circle.radius(), tolerances)
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
    }) {
        curves.push(candidate);
    }
}

fn squared_tolerance(
    center_distance: f64,
    radius_a: f64,
    radius_b: f64,
    tolerances: Tolerances,
) -> f64 {
    tolerances.linear() * (center_distance + radius_a + radius_b).max(1.0)
}

fn validate_ranges(a_range: [ParamRange; 2], b_range: [ParamRange; 2]) -> Result<()> {
    if a_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "sphere/sphere intersection requires finite non-reversed first-sphere ranges",
        });
    }
    if b_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "sphere/sphere intersection requires finite non-reversed second-sphere ranges",
        });
    }
    Ok(())
}
