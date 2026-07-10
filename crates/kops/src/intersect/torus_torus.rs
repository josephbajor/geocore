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
use kgeom::surface::Torus;
use kgeom::vec::Point3;

/// Intersect two finite torus parameter windows.
///
/// Supports coaxial torus/torus intersections. The meridian reduction is a
/// circle/circle solve between the tube cross-sections; every accepted
/// meridian point revolves into an exact latitude circle. General offset or
/// skew torus/torus intersections remain explicit until SSI result geometry can
/// carry those higher-order branch families.
pub fn intersect_bounded_tori(
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(a_range, b_range)?;

    let axis = a.frame().z();
    if axis.cross(b.frame().z()).norm() > tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "torus/torus intersection currently supports only coaxial circular cuts",
        });
    }

    let offset = b.frame().origin() - a.frame().origin();
    let radial_offset = offset - axis * offset.dot(axis);
    if radial_offset.norm() > tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "torus/torus intersection currently supports only coaxial circular cuts",
        });
    }

    let roots = meridian_roots(a, b, offset.dot(axis), tolerances)?;
    let mut points = Vec::new();
    let mut curves = Vec::new();
    for root in roots {
        add_circle_branch(
            &mut points,
            &mut curves,
            root.radius,
            root.z,
            root.kind,
            a,
            a_range,
            b,
            b_range,
            tolerances,
        )?;
    }

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

#[derive(Clone, Copy)]
struct MeridianRoot {
    radius: f64,
    z: f64,
    kind: ContactKind,
}

fn meridian_roots(
    a: &Torus,
    b: &Torus,
    b_origin_z: f64,
    tolerances: Tolerances,
) -> Result<Vec<MeridianRoot>> {
    let delta_radius = b.major_radius() - a.major_radius();
    let distance = (delta_radius * delta_radius + b_origin_z * b_origin_z).sqrt();
    let minor_a = a.minor_radius();
    let minor_b = b.minor_radius();

    if distance <= tolerances.linear() {
        if (minor_a - minor_b).abs() <= tolerances.linear() {
            return Err(Error::InvalidGeometry {
                reason: "coincident torus/torus intersection is a surface overlap",
            });
        }
        return Ok(Vec::new());
    }

    if distance > minor_a + minor_b + tolerances.linear()
        || distance < (minor_a - minor_b).abs() - tolerances.linear()
    {
        return Ok(Vec::new());
    }

    let along = (minor_a * minor_a - minor_b * minor_b + distance * distance) / (2.0 * distance);
    let h_sq = minor_a * minor_a - along * along;
    let sq_tol = squared_tolerance(distance, minor_a, minor_b, tolerances);
    if h_sq < -sq_tol {
        return Ok(Vec::new());
    }

    let e_radius = delta_radius / distance;
    let e_z = b_origin_z / distance;
    let base_radius = a.major_radius() + along * e_radius;
    let base_z = along * e_z;
    let normal_radius = -e_z;
    let normal_z = e_radius;

    let mut roots = Vec::new();
    if h_sq <= sq_tol {
        push_meridian_root(
            &mut roots,
            base_radius,
            base_z,
            ContactKind::Tangent,
            tolerances,
        );
    } else {
        let h = h_sq.sqrt();
        for sign in [-1.0, 1.0] {
            push_meridian_root(
                &mut roots,
                base_radius + sign * h * normal_radius,
                base_z + sign * h * normal_z,
                ContactKind::Transverse,
                tolerances,
            );
        }
    }
    roots.sort_by(|a, b| {
        a.z.total_cmp(&b.z)
            .then(a.radius.total_cmp(&b.radius))
            .then(a.kind.cmp(&b.kind))
    });
    Ok(roots)
}

fn push_meridian_root(
    roots: &mut Vec<MeridianRoot>,
    radius: f64,
    z: f64,
    kind: ContactKind,
    tolerances: Tolerances,
) {
    if radius <= tolerances.linear() {
        return;
    }
    let candidate = MeridianRoot { radius, z, kind };
    if !roots.iter().any(|root| {
        (root.radius - candidate.radius).abs() <= tolerances.linear()
            && (root.z - candidate.z).abs() <= tolerances.linear()
    }) {
        roots.push(candidate);
    }
}

#[allow(clippy::too_many_arguments)]
fn add_circle_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    radius: f64,
    z: f64,
    branch_kind: ContactKind,
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let center = a.frame().origin() + a.frame().z() * z;
    let circle = Circle::new(Frame::new(center, a.frame().z(), a.frame().x())?, radius)?;
    let a_hit =
        intersect_bounded_circle_torus(&circle, circle.param_range(), a, a_range, tolerances)?;
    let b_hit =
        intersect_bounded_circle_torus(&circle, circle.param_range(), b, b_range, tolerances)?;
    add_clipped_branch(
        points,
        curves,
        &circle,
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
    circle: &Circle,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let t_tol = parameter_tolerance(circle.radius(), tolerances);
    for a_overlap in &a_hit.overlaps {
        for b_overlap in &b_hit.overlaps {
            let lo = a_overlap.curve.lo.max(b_overlap.curve.lo);
            let hi = a_overlap.curve.hi.min(b_overlap.curve.hi);
            if hi - lo > t_tol {
                let Some(uv_a_start) = torus_uv_at(circle.eval(lo), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_a_end) = torus_uv_at(circle.eval(hi), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_start) = torus_uv_at(circle.eval(lo), b, b_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_end) = torus_uv_at(circle.eval(hi), b, b_range, tolerances) else {
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
        circle,
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
    circle: &Circle,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
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
                circle,
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
                    circle,
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
    circle: &Circle,
    t: f64,
    kind: ContactKind,
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, circle.param_range(), t_tol) else {
        return;
    };
    let point = circle.eval(t);
    let Some(uv_a) = torus_uv_at(point, a, a_range, tolerances) else {
        return;
    };
    let Some(uv_b) = torus_uv_at(point, b, b_range, tolerances) else {
        return;
    };
    if let Some(point) = accept_surface_surface_candidate(a, uv_a, b, uv_b, kind, tolerances) {
        push_point(points, point, tolerances);
    }
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
    center_distance: f64,
    minor_a: f64,
    minor_b: f64,
    tolerances: Tolerances,
) -> f64 {
    tolerances.linear() * (center_distance + minor_a + minor_b).max(1.0)
}

fn validate_ranges(a_range: [ParamRange; 2], b_range: [ParamRange; 2]) -> Result<()> {
    if a_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "torus/torus intersection requires finite non-reversed first-torus ranges",
        });
    }
    if b_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "torus/torus intersection requires finite non-reversed second-torus ranges",
        });
    }
    Ok(())
}
