use super::circle_cone::intersect_bounded_circle_cone;
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
use kgeom::surface::{Cone, Surface};
use kgeom::vec::Point3;

/// Intersect two finite cone parameter windows.
///
/// Supports coaxial cone/cone intersections. Each cone contributes two signed
/// meridian lines; non-coincident line pairs revolve into exact circle branches,
/// while shared apex roots become singular point contacts. Coincident signed
/// meridian lines are surface overlaps and remain explicit.
pub fn intersect_bounded_cones(
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(a_range, b_range)?;

    let axis = a.frame().z();
    if axis.cross(b.frame().z()).norm() > tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "cone/cone intersection currently supports only coaxial circular cuts",
        });
    }

    let offset = b.frame().origin() - a.frame().origin();
    let radial_offset = offset - axis * offset.dot(axis);
    if radial_offset.norm() > tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "cone/cone intersection currently supports only coaxial circular cuts",
        });
    }

    let roots = meridian_roots(a, b, offset.dot(axis), tolerances)?;
    let mut points = Vec::new();
    let mut curves = Vec::new();
    for root in roots {
        match root {
            MeridianRoot::Circle { radius, z } => add_circle_branch(
                &mut points,
                &mut curves,
                radius,
                z,
                a,
                a_range,
                b,
                b_range,
                tolerances,
            )?,
            MeridianRoot::Apex { z } => add_point(
                &mut points,
                a.frame().origin() + axis * z,
                a,
                a_range,
                b,
                b_range,
                ContactKind::Singular,
                tolerances,
            ),
        }
    }

    SurfaceSurfaceIntersections::canonicalized(points, curves)
}

#[derive(Clone, Copy)]
enum MeridianRoot {
    Circle { radius: f64, z: f64 },
    Apex { z: f64 },
}

#[derive(Clone, Copy)]
struct MeridianLine {
    slope: f64,
    intercept: f64,
}

fn meridian_roots(
    a: &Cone,
    b: &Cone,
    b_origin_z: f64,
    tolerances: Tolerances,
) -> Result<Vec<MeridianRoot>> {
    let (_, cos_a) = math::sincos(a.half_angle());
    let (_, cos_b) = math::sincos(b.half_angle());
    let tan_a = math::sin(a.half_angle()) / cos_a;
    let tan_b = math::sin(b.half_angle()) / cos_b;
    let axis_sign = if a.frame().z().dot(b.frame().z()) < 0.0 {
        -1.0
    } else {
        1.0
    };

    let mut roots = Vec::new();
    let scale = (a.radius() + b.radius() + b_origin_z.abs()).max(1.0);
    for sign_a in [-1.0, 1.0] {
        let line_a = MeridianLine {
            slope: sign_a * tan_a,
            intercept: sign_a * a.radius(),
        };
        for sign_b in [-1.0, 1.0] {
            let line_b = MeridianLine {
                slope: sign_b * axis_sign * tan_b,
                intercept: sign_b * (b.radius() - axis_sign * b_origin_z * tan_b),
            };
            push_line_intersection(&mut roots, line_a, line_b, scale, tolerances)?;
        }
    }
    roots.sort_by(|a, b| {
        root_z(*a)
            .total_cmp(&root_z(*b))
            .then(root_radius(*a).total_cmp(&root_radius(*b)))
    });
    Ok(roots)
}

fn push_line_intersection(
    roots: &mut Vec<MeridianRoot>,
    a: MeridianLine,
    b: MeridianLine,
    scale: f64,
    tolerances: Tolerances,
) -> Result<()> {
    let denom = a.slope - b.slope;
    let rhs = b.intercept - a.intercept;
    if denom.abs() <= tolerances.angular() {
        if rhs.abs() <= tolerances.linear() * scale {
            return Err(Error::InvalidGeometry {
                reason: "coincident cone/cone intersection is a surface overlap",
            });
        }
        return Ok(());
    }

    let z = rhs / denom;
    let radius = a.slope * z + a.intercept;
    if radius < -tolerances.linear() {
        return Ok(());
    }
    if radius <= tolerances.linear() {
        push_apex_root(roots, z, tolerances);
    } else {
        push_circle_root(roots, radius, z, tolerances);
    }
    Ok(())
}

fn push_circle_root(roots: &mut Vec<MeridianRoot>, radius: f64, z: f64, tolerances: Tolerances) {
    if !roots.iter().any(|root| match *root {
        MeridianRoot::Circle {
            radius: other_radius,
            z: other_z,
        } => {
            (radius - other_radius).abs() <= tolerances.linear()
                && (z - other_z).abs() <= tolerances.linear()
        }
        MeridianRoot::Apex { .. } => false,
    }) {
        roots.push(MeridianRoot::Circle { radius, z });
    }
}

fn push_apex_root(roots: &mut Vec<MeridianRoot>, z: f64, tolerances: Tolerances) {
    if !roots.iter().any(|root| match *root {
        MeridianRoot::Apex { z: other_z } => (z - other_z).abs() <= tolerances.linear(),
        MeridianRoot::Circle { radius, z: other_z } => {
            radius <= tolerances.linear() && (z - other_z).abs() <= tolerances.linear()
        }
    }) {
        roots.push(MeridianRoot::Apex { z });
    }
}

fn root_z(root: MeridianRoot) -> f64 {
    match root {
        MeridianRoot::Circle { z, .. } | MeridianRoot::Apex { z } => z,
    }
}

fn root_radius(root: MeridianRoot) -> f64 {
    match root {
        MeridianRoot::Circle { radius, .. } => radius,
        MeridianRoot::Apex { .. } => 0.0,
    }
}

#[allow(clippy::too_many_arguments)]
fn add_circle_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    radius: f64,
    z: f64,
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let center = a.frame().origin() + a.frame().z() * z;
    let circle = Circle::new(Frame::new(center, a.frame().z(), a.frame().x())?, radius)?;
    let a_hit =
        intersect_bounded_circle_cone(&circle, circle.param_range(), a, a_range, tolerances)?;
    let b_hit =
        intersect_bounded_circle_cone(&circle, circle.param_range(), b, b_range, tolerances)?;
    add_clipped_branch(
        points, curves, &circle, &a_hit, &b_hit, a, a_range, b, b_range, tolerances,
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_clipped_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    circle: &Circle,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let t_tol = parameter_tolerance(circle.radius(), tolerances);
    for a_overlap in &a_hit.overlaps {
        for b_overlap in &b_hit.overlaps {
            let lo = a_overlap.curve.lo.max(b_overlap.curve.lo);
            let hi = a_overlap.curve.hi.min(b_overlap.curve.hi);
            if hi - lo > t_tol {
                let Some(uv_a_start) = cone_uv_at(circle.eval(lo), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_a_end) = cone_uv_at(circle.eval(hi), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_start) = cone_uv_at(circle.eval(lo), b, b_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_end) = cone_uv_at(circle.eval(hi), b, b_range, tolerances) else {
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
                    t_tol.max(tolerances.linear()),
                );
            } else if (hi - lo).abs() <= t_tol {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    ((lo + hi) / 2.0).clamp(circle.param_range().lo, circle.param_range().hi),
                    a,
                    a_range,
                    b,
                    b_range,
                    t_tol,
                    ContactKind::Transverse,
                    tolerances,
                );
            }
        }
    }

    add_isolated_points(
        points, circle, a_hit, b_hit, a, a_range, b, b_range, t_tol, tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    circle: &Circle,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
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
                t_tol,
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
                t_tol,
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
                    t_tol,
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
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    t_tol: f64,
    kind: ContactKind,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, circle.param_range(), t_tol) else {
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
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    kind: ContactKind,
    tolerances: Tolerances,
) {
    let Some(uv_a) = cone_uv_at(point, a, a_range, tolerances) else {
        return;
    };
    let Some(uv_b) = cone_uv_at(point, b, b_range, tolerances) else {
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

fn validate_ranges(a_range: [ParamRange; 2], b_range: [ParamRange; 2]) -> Result<()> {
    if a_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cone/cone intersection requires finite non-reversed first-cone ranges",
        });
    }
    if b_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cone/cone intersection requires finite non-reversed second-cone ranges",
        });
    }
    Ok(())
}
