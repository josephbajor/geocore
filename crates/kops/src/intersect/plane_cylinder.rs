use super::circle_cylinder::intersect_bounded_circle_cylinder;
use super::ellipse_cylinder::intersect_bounded_ellipse_cylinder;
use super::line_cylinder::intersect_bounded_line_cylinder;
use super::line_plane::intersect_bounded_line_plane;
use super::planar_curve_plane::{intersect_bounded_circle_plane, intersect_bounded_ellipse_plane};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, accept_surface_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point3, Vec3};

/// Intersect a finite plane window with a finite cylinder parameter window.
pub fn intersect_bounded_plane_cylinder(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(plane_range, cylinder_range)?;

    let normal = plane.frame().z();
    let axis = cylinder.frame().z();
    let nx = normal.dot(cylinder.frame().x());
    let ny = normal.dot(cylinder.frame().y());
    let nz = normal.dot(axis);
    let radial_len = (nx * nx + ny * ny).sqrt();
    let offset = (cylinder.frame().origin() - plane.frame().origin()).dot(normal);

    if nz.abs() <= tolerances.angular() {
        return intersect_parallel_plane_cylinder(
            plane,
            plane_range,
            cylinder,
            cylinder_range,
            normal,
            nx,
            ny,
            radial_len,
            offset,
            tolerances,
        );
    }

    if radial_len <= tolerances.angular() {
        let v = -offset / nz;
        let center = cylinder.frame().origin() + axis * v;
        let circle = Circle::new(
            Frame::new(center, normal, cylinder.frame().x())?,
            cylinder.radius(),
        )?;
        return clip_circle_branch(
            circle,
            ContactKind::Transverse,
            plane,
            plane_range,
            cylinder,
            cylinder_range,
            tolerances,
        );
    }

    let radial_dir = (cylinder.frame().x() * nx + cylinder.frame().y() * ny) / radial_len;
    let center = cylinder.frame().origin() - axis * (offset / nz);
    let x_hint = radial_dir - axis * (radial_len / nz);
    let ellipse = Ellipse::new(
        Frame::new(center, normal, x_hint)?,
        cylinder.radius() / nz.abs(),
        cylinder.radius(),
    )?;
    clip_ellipse_branch(
        ellipse,
        ContactKind::Transverse,
        plane,
        plane_range,
        cylinder,
        cylinder_range,
        tolerances,
    )
}

#[allow(clippy::too_many_arguments)]
fn intersect_parallel_plane_cylinder(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    normal: Vec3,
    nx: f64,
    ny: f64,
    radial_len: f64,
    offset: f64,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    if radial_len <= tolerances.angular() {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let radius = cylinder.radius();
    let q = -offset / (radius * radial_len);
    let q_tol = tolerances.linear() / radius;
    if q.abs() > 1.0 + q_tol {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let radial_x = (cylinder.frame().x() * nx + cylinder.frame().y() * ny) / radial_len;
    let radial_y = cylinder.frame().z().cross(radial_x);
    let tangent = q.abs() >= 1.0 - q_tol;
    let sin_values = if tangent {
        vec![0.0]
    } else {
        let sin = (1.0 - q * q).max(0.0).sqrt();
        vec![-sin, sin]
    };

    let mut points = Vec::new();
    let mut curves = Vec::new();
    for sin in sin_values {
        let radial = radial_x * q.clamp(-1.0, 1.0) + radial_y * sin;
        let line = Line::new(
            cylinder.frame().origin() + radial * radius,
            cylinder.frame().z(),
        )?;
        let branch_kind = if tangent || normal.cross(radial).norm() <= tolerances.angular() {
            ContactKind::Tangent
        } else {
            ContactKind::Transverse
        };
        add_line_branch(
            &mut points,
            &mut curves,
            line,
            cylinder_range[1],
            branch_kind,
            plane,
            plane_range,
            cylinder,
            cylinder_range,
            tolerances,
        )?;
    }

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

#[allow(clippy::too_many_arguments)]
fn add_line_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    line: Line,
    line_range: ParamRange,
    branch_kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let plane_hit =
        intersect_bounded_line_plane(&line, line_range, plane, plane_range, tolerances)?;
    let cylinder_hit =
        intersect_bounded_line_cylinder(&line, line_range, cylinder, cylinder_range, tolerances)?;
    add_clipped_branch(
        points,
        curves,
        SurfaceIntersectionCurve::Line(line),
        line_range,
        &plane_hit,
        &cylinder_hit,
        branch_kind,
        plane,
        plane_range,
        cylinder,
        cylinder_range,
        tolerances.linear(),
        false,
        tolerances,
    );
    Ok(())
}

fn clip_circle_branch(
    circle: Circle,
    branch_kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let plane_hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        plane,
        plane_range,
        tolerances,
    )?;
    let cylinder_hit = intersect_bounded_circle_cylinder(
        &circle,
        circle.param_range(),
        cylinder,
        cylinder_range,
        tolerances,
    )?;
    let mut points = Vec::new();
    let mut curves = Vec::new();
    add_clipped_branch(
        &mut points,
        &mut curves,
        SurfaceIntersectionCurve::Circle(circle),
        circle.param_range(),
        &plane_hit,
        &cylinder_hit,
        branch_kind,
        plane,
        plane_range,
        cylinder,
        cylinder_range,
        parameter_tolerance(circle.radius(), tolerances),
        true,
        tolerances,
    );
    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

fn clip_ellipse_branch(
    ellipse: Ellipse,
    branch_kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let plane_hit = intersect_bounded_ellipse_plane(
        &ellipse,
        ellipse.param_range(),
        plane,
        plane_range,
        tolerances,
    )?;
    let cylinder_hit = intersect_bounded_ellipse_cylinder(
        &ellipse,
        ellipse.param_range(),
        cylinder,
        cylinder_range,
        tolerances,
    )?;
    let mut points = Vec::new();
    let mut curves = Vec::new();
    add_clipped_branch(
        &mut points,
        &mut curves,
        SurfaceIntersectionCurve::Ellipse(ellipse),
        ellipse.param_range(),
        &plane_hit,
        &cylinder_hit,
        branch_kind,
        plane,
        plane_range,
        cylinder,
        cylinder_range,
        parameter_tolerance(ellipse.minor_radius(), tolerances),
        true,
        tolerances,
    );
    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

#[allow(clippy::too_many_arguments)]
fn add_clipped_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    curve: SurfaceIntersectionCurve,
    curve_range: ParamRange,
    plane_hit: &CurveSurfaceIntersections,
    cylinder_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    t_tol: f64,
    periodic: bool,
    tolerances: Tolerances,
) {
    for plane_overlap in &plane_hit.overlaps {
        for cylinder_overlap in &cylinder_hit.overlaps {
            let lo = plane_overlap.curve.lo.max(cylinder_overlap.curve.lo);
            let hi = plane_overlap.curve.hi.min(cylinder_overlap.curve.hi);
            if hi - lo > t_tol {
                let Some(uv_plane_start) =
                    plane_uv_at(curve.eval(lo), plane, plane_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_plane_end) =
                    plane_uv_at(curve.eval(hi), plane, plane_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_cylinder_start) =
                    cylinder_uv_at(curve.eval(lo), cylinder, cylinder_range, tolerances)
                else {
                    continue;
                };
                let Some(uv_cylinder_end) =
                    cylinder_uv_at(curve.eval(hi), cylinder, cylinder_range, tolerances)
                else {
                    continue;
                };
                push_curve(
                    curves,
                    SurfaceSurfaceCurve {
                        curve: curve.clone(),
                        curve_range: ParamRange::new(lo, hi),
                        uv_a_start: uv_plane_start,
                        uv_a_end: uv_plane_end,
                        uv_b_start: uv_cylinder_start,
                        uv_b_end: uv_cylinder_end,
                        kind: branch_kind,
                    },
                    t_tol.max(tolerances.linear()),
                );
            } else if (hi - lo).abs() <= t_tol {
                add_point_from_curve_parameter(
                    points,
                    &curve,
                    curve_range,
                    ((lo + hi) / 2.0).clamp(curve_range.lo, curve_range.hi),
                    branch_kind,
                    plane,
                    plane_range,
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
        &curve,
        curve_range,
        plane_hit,
        cylinder_hit,
        branch_kind,
        plane,
        plane_range,
        cylinder,
        cylinder_range,
        t_tol,
        periodic,
        tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    curve: &SurfaceIntersectionCurve,
    curve_range: ParamRange,
    plane_hit: &CurveSurfaceIntersections,
    cylinder_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    t_tol: f64,
    periodic: bool,
    tolerances: Tolerances,
) {
    for point in &plane_hit.points {
        if hit_contains_t(cylinder_hit, point.t_curve, t_tol, periodic, tolerances) {
            add_point_from_curve_parameter(
                points,
                curve,
                curve_range,
                point.t_curve,
                branch_kind,
                plane,
                plane_range,
                cylinder,
                cylinder_range,
                t_tol,
                tolerances,
            );
        }
    }
    for point in &cylinder_hit.points {
        if hit_contains_t(plane_hit, point.t_curve, t_tol, periodic, tolerances) {
            add_point_from_curve_parameter(
                points,
                curve,
                curve_range,
                point.t_curve,
                branch_kind,
                plane,
                plane_range,
                cylinder,
                cylinder_range,
                t_tol,
                tolerances,
            );
        }
    }
    for plane_point in &plane_hit.points {
        for cylinder_point in &cylinder_hit.points {
            if curve_parameters_match(plane_point, cylinder_point, t_tol, periodic, tolerances) {
                add_point_from_curve_parameter(
                    points,
                    curve,
                    curve_range,
                    plane_point.t_curve,
                    branch_kind,
                    plane,
                    plane_range,
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
    curve: &SurfaceIntersectionCurve,
    curve_range: ParamRange,
    t: f64,
    branch_kind: ContactKind,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, curve_range, t_tol) else {
        return;
    };
    let point = curve.eval(t);
    let Some(uv_plane) = plane_uv_at(point, plane, plane_range, tolerances) else {
        return;
    };
    let Some(uv_cylinder) = cylinder_uv_at(point, cylinder, cylinder_range, tolerances) else {
        return;
    };
    if let Some(point) = accept_surface_surface_candidate(
        plane,
        uv_plane,
        cylinder,
        uv_cylinder,
        branch_kind,
        tolerances,
    ) {
        push_point(points, point, tolerances);
    }
}

fn plane_uv_at(
    point: Point3,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = plane.frame().to_local(point);
    Some([
        fit_scalar_parameter(local.x, plane_range[0], tolerances.linear())?,
        fit_scalar_parameter(local.y, plane_range[1], tolerances.linear())?,
    ])
}

fn cylinder_uv_at(
    point: Point3,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = cylinder.frame().to_local(point);
    let raw_u = math::atan2(local.y, local.x);
    let u = super::conic::fit_periodic_parameter(
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
    periodic: bool,
    tolerances: Tolerances,
) -> bool {
    hit.overlaps
        .iter()
        .any(|overlap| overlap_contains_t(overlap, t, t_tol, periodic))
        || hit.points.iter().any(|point| {
            curve_parameter_distance(point.t_curve, t, periodic) <= t_tol.max(tolerances.angular())
        })
}

fn overlap_contains_t(overlap: &CurveSurfaceOverlap, t: f64, t_tol: f64, periodic: bool) -> bool {
    if periodic {
        [t, t - core::f64::consts::TAU, t + core::f64::consts::TAU]
            .into_iter()
            .any(|candidate| {
                candidate >= overlap.curve.lo - t_tol && candidate <= overlap.curve.hi + t_tol
            })
    } else {
        t >= overlap.curve.lo - t_tol && t <= overlap.curve.hi + t_tol
    }
}

fn curve_parameters_match(
    a: &CurveSurfacePoint,
    b: &CurveSurfacePoint,
    t_tol: f64,
    periodic: bool,
    tolerances: Tolerances,
) -> bool {
    curve_parameter_distance(a.t_curve, b.t_curve, periodic) <= t_tol.max(tolerances.angular())
        || a.point.dist(b.point) <= tolerances.linear()
}

fn curve_parameter_distance(a: f64, b: f64, periodic: bool) -> f64 {
    let diff = (a - b).abs();
    if periodic {
        diff.min((core::f64::consts::TAU - diff).abs())
    } else {
        diff
    }
}

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn parameter_tolerance(radius: f64, tolerances: Tolerances) -> f64 {
    (tolerances.linear() / radius).max(tolerances.angular())
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

fn validate_ranges(plane_range: [ParamRange; 2], cylinder_range: [ParamRange; 2]) -> Result<()> {
    if plane_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/cylinder intersection requires finite non-reversed plane ranges",
        });
    }
    if cylinder_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/cylinder intersection requires finite non-reversed cylinder ranges",
        });
    }
    Ok(())
}
