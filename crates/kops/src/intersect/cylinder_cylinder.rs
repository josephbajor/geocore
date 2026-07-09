use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::line_cylinder::intersect_bounded_line_cylinder;
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, accept_surface_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Line};
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgeom::vec::Point3;

/// Intersect two finite cylinder parameter windows.
///
/// Supports parallel-axis cylinder/cylinder intersections. Those reduce to a
/// circle/circle solve in the plane normal to the axes and produce one tangent
/// ruling or two transverse rulings. Skew and oblique cylinder/cylinder
/// intersections are quartic space curves and remain explicit until SSI result
/// geometry can carry that branch family.
pub fn intersect_bounded_cylinders(
    a: &Cylinder,
    a_range: [ParamRange; 2],
    b: &Cylinder,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(a_range, b_range)?;

    let axis = a.frame().z();
    if axis.cross(b.frame().z()).norm() > tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/cylinder intersection currently supports only parallel-axis ruling cuts",
        });
    }

    let offset = b.frame().origin() - a.frame().origin();
    let radial_offset = offset - axis * offset.dot(axis);
    let distance = radial_offset.norm();
    let radius_a = a.radius();
    let radius_b = b.radius();

    if distance <= tolerances.linear() {
        if (radius_a - radius_b).abs() <= tolerances.linear() {
            return Err(Error::InvalidGeometry {
                reason: "coincident cylinder/cylinder intersection is a surface overlap",
            });
        }
        return Ok(SurfaceSurfaceIntersections::default());
    }

    let x = (radius_a * radius_a - radius_b * radius_b + distance * distance) / (2.0 * distance);
    let h_sq = radius_a * radius_a - x * x;
    let sq_tol = squared_tolerance(radius_a, radius_b, distance, tolerances);
    if h_sq < -sq_tol {
        return Ok(SurfaceSurfaceIntersections::default());
    }

    let radial_x = radial_offset / distance;
    let radial_y = axis.cross(radial_x);
    let mut points = Vec::new();
    let mut curves = Vec::new();
    if h_sq <= sq_tol {
        let point = a.frame().origin() + radial_x * x.clamp(-radius_a, radius_a);
        add_line_branch(
            &mut points,
            &mut curves,
            point,
            ContactKind::Tangent,
            a,
            a_range,
            b,
            b_range,
            tolerances,
        )?;
    } else {
        let h = h_sq.sqrt();
        for point in [
            a.frame().origin() + radial_x * x - radial_y * h,
            a.frame().origin() + radial_x * x + radial_y * h,
        ] {
            add_line_branch(
                &mut points,
                &mut curves,
                point,
                ContactKind::Transverse,
                a,
                a_range,
                b,
                b_range,
                tolerances,
            )?;
        }
    }

    SurfaceSurfaceIntersections::canonicalized(points, curves)
}

#[allow(clippy::too_many_arguments)]
fn add_line_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    origin: Point3,
    branch_kind: ContactKind,
    a: &Cylinder,
    a_range: [ParamRange; 2],
    b: &Cylinder,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let line = Line::new(origin, a.frame().z())?;
    let line_range = a_range[1];
    let a_hit = intersect_bounded_line_cylinder(&line, line_range, a, a_range, tolerances)?;
    let b_hit = intersect_bounded_line_cylinder(&line, line_range, b, b_range, tolerances)?;
    add_clipped_branch(
        points,
        curves,
        &line,
        &a_hit,
        &b_hit,
        branch_kind,
        a,
        a_range,
        b,
        b_range,
        tolerances,
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_clipped_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    line: &Line,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    a: &Cylinder,
    a_range: [ParamRange; 2],
    b: &Cylinder,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let t_tol = tolerances.linear();
    for a_overlap in &a_hit.overlaps {
        for b_overlap in &b_hit.overlaps {
            let lo = a_overlap.curve.lo.max(b_overlap.curve.lo);
            let hi = a_overlap.curve.hi.min(b_overlap.curve.hi);
            if hi - lo > t_tol {
                let Some(uv_a_start) = cylinder_uv_at(line.eval(lo), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_a_end) = cylinder_uv_at(line.eval(hi), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_start) = cylinder_uv_at(line.eval(lo), b, b_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_end) = cylinder_uv_at(line.eval(hi), b, b_range, tolerances) else {
                    continue;
                };
                push_curve(
                    curves,
                    SurfaceSurfaceCurve {
                        curve: SurfaceIntersectionCurve::Line(*line),
                        curve_range: ParamRange::new(lo, hi),
                        uv_a_start,
                        uv_a_end,
                        uv_b_start,
                        uv_b_end,
                        kind: branch_kind,
                    },
                    t_tol,
                );
            } else if (hi - lo).abs() <= t_tol {
                add_point_from_curve_parameter(
                    points,
                    line,
                    ((lo + hi) / 2.0).clamp(line.param_range().lo, line.param_range().hi),
                    branch_kind,
                    a,
                    a_range,
                    b,
                    b_range,
                    t_tol,
                    tolerances,
                );
            }
        }
    }

    add_isolated_points(
        points,
        line,
        a_hit,
        b_hit,
        branch_kind,
        a,
        a_range,
        b,
        b_range,
        t_tol,
        tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    line: &Line,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    a: &Cylinder,
    a_range: [ParamRange; 2],
    b: &Cylinder,
    b_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    for point in &a_hit.points {
        if hit_contains_t(b_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                line,
                point.t_curve,
                branch_kind,
                a,
                a_range,
                b,
                b_range,
                t_tol,
                tolerances,
            );
        }
    }
    for point in &b_hit.points {
        if hit_contains_t(a_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                line,
                point.t_curve,
                branch_kind,
                a,
                a_range,
                b,
                b_range,
                t_tol,
                tolerances,
            );
        }
    }
    for a_point in &a_hit.points {
        for b_point in &b_hit.points {
            if curve_parameters_match(a_point, b_point, t_tol, tolerances) {
                add_point_from_curve_parameter(
                    points,
                    line,
                    a_point.t_curve,
                    branch_kind,
                    a,
                    a_range,
                    b,
                    b_range,
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
    line: &Line,
    t: f64,
    kind: ContactKind,
    a: &Cylinder,
    a_range: [ParamRange; 2],
    b: &Cylinder,
    b_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, a_range[1], t_tol) else {
        return;
    };
    let point = line.eval(t);
    let Some(uv_a) = cylinder_uv_at(point, a, a_range, tolerances) else {
        return;
    };
    let Some(uv_b) = cylinder_uv_at(point, b, b_range, tolerances) else {
        return;
    };
    if let Some(point) = accept_surface_surface_candidate(a, uv_a, b, uv_b, kind, tolerances) {
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
    t >= overlap.curve.lo - t_tol && t <= overlap.curve.hi + t_tol
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
    (a - b).abs()
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

fn squared_tolerance(radius_a: f64, radius_b: f64, distance: f64, tolerances: Tolerances) -> f64 {
    tolerances.linear() * (radius_a + radius_b + distance).max(1.0)
}

fn validate_ranges(a_range: [ParamRange; 2], b_range: [ParamRange; 2]) -> Result<()> {
    if a_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/cylinder intersection requires finite non-reversed first cylinder ranges",
        });
    }
    if b_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/cylinder intersection requires finite non-reversed second cylinder ranges",
        });
    }
    Ok(())
}
