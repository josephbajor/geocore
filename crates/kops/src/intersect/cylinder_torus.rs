use super::circle_cylinder::intersect_bounded_circle_cylinder;
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
use kgeom::surface::{Cylinder, Torus};
use kgeom::vec::Point3;

/// Intersect a finite cylinder parameter window with a finite torus parameter
/// window.
///
/// Supports coaxial cylinder/torus intersections. The cylinder radius cuts the
/// torus tube circle in the meridian section, producing one tangent latitude
/// circle or two transverse latitude circles. General offset or skew
/// cylinder/torus intersections remain explicit until SSI result geometry can
/// carry that quartic branch family.
pub fn intersect_bounded_cylinder_torus(
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(cylinder_range, torus_range)?;

    let axis = torus.frame().z();
    if axis.cross(cylinder.frame().z()).norm() > tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/torus intersection currently supports only coaxial circular cuts",
        });
    }

    let offset = cylinder.frame().origin() - torus.frame().origin();
    let radial_offset = offset - axis * offset.dot(axis);
    if radial_offset.norm() > tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/torus intersection currently supports only coaxial circular cuts",
        });
    }

    let delta = cylinder.radius() - torus.major_radius();
    let h_sq = torus.minor_radius() * torus.minor_radius() - delta * delta;
    let sq_tol = squared_tolerance(cylinder.radius(), torus, tolerances);
    if h_sq < -sq_tol {
        return Ok(SurfaceSurfaceIntersections::default());
    }

    let mut points = Vec::new();
    let mut curves = Vec::new();
    if h_sq <= sq_tol {
        add_circle_branch(
            &mut points,
            &mut curves,
            0.0,
            ContactKind::Tangent,
            cylinder,
            cylinder_range,
            torus,
            torus_range,
            tolerances,
        )?;
    } else {
        let h = h_sq.sqrt();
        for z in [-h, h] {
            add_circle_branch(
                &mut points,
                &mut curves,
                z,
                ContactKind::Transverse,
                cylinder,
                cylinder_range,
                torus,
                torus_range,
                tolerances,
            )?;
        }
    }

    SurfaceSurfaceIntersections::canonicalized(points, curves)
}

#[allow(clippy::too_many_arguments)]
fn add_circle_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    z: f64,
    branch_kind: ContactKind,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let center = torus.frame().origin() + torus.frame().z() * z;
    let circle = Circle::new(
        Frame::new(center, torus.frame().z(), torus.frame().x())?,
        cylinder.radius(),
    )?;
    let cylinder_hit = intersect_bounded_circle_cylinder(
        &circle,
        circle.param_range(),
        cylinder,
        cylinder_range,
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
        &cylinder_hit,
        &torus_hit,
        branch_kind,
        cylinder,
        cylinder_range,
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
    cylinder_hit: &CurveSurfaceIntersections,
    torus_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let t_tol = parameter_tolerance(circle.radius(), tolerances);
    for cylinder_overlap in &cylinder_hit.overlaps {
        for torus_overlap in &torus_hit.overlaps {
            let lo = cylinder_overlap.curve.lo.max(torus_overlap.curve.lo);
            let hi = cylinder_overlap.curve.hi.min(torus_overlap.curve.hi);
            if hi - lo > t_tol {
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
                        uv_a_start: uv_cylinder_start,
                        uv_a_end: uv_cylinder_end,
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
                    cylinder,
                    cylinder_range,
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
        cylinder_hit,
        torus_hit,
        branch_kind,
        cylinder,
        cylinder_range,
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
    cylinder_hit: &CurveSurfaceIntersections,
    torus_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    for point in &cylinder_hit.points {
        if hit_contains_t(torus_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                cylinder,
                cylinder_range,
                torus,
                torus_range,
                t_tol,
                tolerances,
            );
        }
    }
    for point in &torus_hit.points {
        if hit_contains_t(cylinder_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                cylinder,
                cylinder_range,
                torus,
                torus_range,
                t_tol,
                tolerances,
            );
        }
    }
    for cylinder_point in &cylinder_hit.points {
        for torus_point in &torus_hit.points {
            if curve_parameters_match(cylinder_point, torus_point, t_tol, tolerances) {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    cylinder_point.t_curve,
                    branch_kind,
                    cylinder,
                    cylinder_range,
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
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    torus: &Torus,
    torus_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, circle.param_range(), t_tol) else {
        return;
    };
    let point = circle.eval(t);
    let Some(uv_cylinder) = cylinder_uv_at(point, cylinder, cylinder_range, tolerances) else {
        return;
    };
    let Some(uv_torus) = torus_uv_at(point, torus, torus_range, tolerances) else {
        return;
    };
    if let Some(point) =
        accept_surface_surface_candidate(cylinder, uv_cylinder, torus, uv_torus, kind, tolerances)
    {
        push_point(points, point, tolerances);
    }
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

fn squared_tolerance(cylinder_radius: f64, torus: &Torus, tolerances: Tolerances) -> f64 {
    tolerances.linear() * (cylinder_radius + torus.major_radius() + torus.minor_radius()).max(1.0)
}

fn validate_ranges(cylinder_range: [ParamRange; 2], torus_range: [ParamRange; 2]) -> Result<()> {
    if cylinder_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/torus intersection requires finite non-reversed cylinder ranges",
        });
    }
    if torus_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/torus intersection requires finite non-reversed torus ranges",
        });
    }
    Ok(())
}
