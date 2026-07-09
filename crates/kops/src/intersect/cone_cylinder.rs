use super::circle_cone::intersect_bounded_circle_cone;
use super::circle_cylinder::intersect_bounded_circle_cylinder;
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

    SurfaceSurfaceIntersections::canonicalized(points, curves)
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
    add_clipped_branch(
        points,
        curves,
        &circle,
        &cone_hit,
        &cylinder_hit,
        branch_kind,
        cone,
        cone_range,
        cylinder,
        cylinder_range,
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
    cylinder_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let t_tol = parameter_tolerance(circle.radius(), tolerances);
    for cone_overlap in &cone_hit.overlaps {
        for cylinder_overlap in &cylinder_hit.overlaps {
            let lo = cone_overlap.curve.lo.max(cylinder_overlap.curve.lo);
            let hi = cone_overlap.curve.hi.min(cylinder_overlap.curve.hi);
            if hi - lo > t_tol {
                let Some(uv_cone_start) = cone_uv_at(circle.eval(lo), cone, cone_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_cone_end) = cone_uv_at(circle.eval(hi), cone, cone_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_cylinder_start) =
                    cylinder_uv_at(circle.eval(lo), cylinder, cylinder_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_cylinder_end) =
                    cylinder_uv_at(circle.eval(hi), cylinder, cylinder_range, tolerances)
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
                        uv_b_start: uv_cylinder_start,
                        uv_b_end: uv_cylinder_end,
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
                    cylinder,
                    cylinder_range,
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
        cylinder_hit,
        branch_kind,
        cone,
        cone_range,
        cylinder,
        cylinder_range,
        t_tol,
        tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    circle: &Circle,
    cone_hit: &CurveSurfaceIntersections,
    cylinder_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    for point in &cone_hit.points {
        if hit_contains_t(cylinder_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                cone,
                cone_range,
                cylinder,
                cylinder_range,
                t_tol,
                tolerances,
            );
        }
    }
    for point in &cylinder_hit.points {
        if hit_contains_t(cone_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                cone,
                cone_range,
                cylinder,
                cylinder_range,
                t_tol,
                tolerances,
            );
        }
    }
    for cone_point in &cone_hit.points {
        for cylinder_point in &cylinder_hit.points {
            if curve_parameters_match(cone_point, cylinder_point, t_tol, tolerances) {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    cone_point.t_curve,
                    branch_kind,
                    cone,
                    cone_range,
                    cylinder,
                    cylinder_range,
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
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, circle.param_range(), t_tol) else {
        return;
    };
    let point = circle.eval(t);
    let Some(uv_cone) = cone_uv_at(point, cone, cone_range, tolerances) else {
        return;
    };
    let Some(uv_cylinder) = cylinder_uv_at(point, cylinder, cylinder_range, tolerances) else {
        return;
    };
    if let Some(point) =
        accept_surface_surface_candidate(cone, uv_cone, cylinder, uv_cylinder, kind, tolerances)
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
