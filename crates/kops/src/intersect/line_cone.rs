use super::conic::fit_periodic_parameter;
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Surface};
use kgeom::vec::Vec3;

/// Intersect a line restricted to a finite range with a finite cone parameter
/// window.
///
/// Isolated roots come from the implicit cone equation. A line lying on a cone
/// ruling clips against the finite `(u, v)` window and can produce a
/// positive-length contained interval.
pub fn intersect_bounded_line_cone(
    line: &Line,
    line_range: ParamRange,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(line_range, cone_range)?;

    let local_origin = cone.frame().to_local(line.origin());
    let direction = line.dir();
    let local_direction = Vec3::new(
        direction.dot(cone.frame().x()),
        direction.dot(cone.frame().y()),
        direction.dot(cone.frame().z()),
    );
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let tan_a = sin_a / cos_a;
    let q0 = cone.radius() + local_origin.z * tan_a;
    let qd = local_direction.z * tan_a;
    let a = local_direction.x * local_direction.x + local_direction.y * local_direction.y - qd * qd;
    let b =
        2.0 * (local_origin.x * local_direction.x + local_origin.y * local_direction.y - q0 * qd);
    let c = local_origin.x * local_origin.x + local_origin.y * local_origin.y - q0 * q0;

    if coefficients_are_zero(a, b, c, local_origin, line_range, tolerances)
        || (line_range.width() > tolerances.linear()
            && implicit_zero_on_range(
                local_origin,
                local_direction,
                cone.radius(),
                tan_a,
                line_range,
                tolerances,
            ))
    {
        return contained_ruling_interval(
            line,
            line_range,
            cone,
            cone_range,
            local_origin,
            local_direction,
            tolerances,
        );
    }

    let line_parameters = solve_quadratic(a, b, c, tolerances);
    let mut points = Vec::with_capacity(line_parameters.len());
    for (t_line, tangent) in line_parameters {
        let Some(t_line) = fit_line_parameter(t_line, line_range, tolerances.linear()) else {
            continue;
        };
        let local = local_origin + local_direction * t_line;
        let Some(uv) = cone_uv(local, cone, cone_range, tolerances) else {
            continue;
        };
        let kind = if cone.normal(uv).is_none() {
            ContactKind::Singular
        } else if tangent {
            ContactKind::Tangent
        } else {
            ContactKind::Transverse
        };
        if let Some(point) =
            accept_curve_surface_candidate(line, t_line, cone, uv, kind, tolerances)
        {
            push_distinct(&mut points, point, tolerances);
        }
    }

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn contained_ruling_interval(
    line: &Line,
    line_range: ParamRange,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    local_origin: Vec3,
    local_direction: Vec3,
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    let Some(u) = generator_u(
        local_origin,
        local_direction,
        line_range,
        cone,
        cone_range,
        tolerances,
    ) else {
        return Ok(CurveSurfaceIntersections::complete_empty());
    };
    let cos_a = math::cos(cone.half_angle());
    let Some(interval) = clip_linear_interval(
        line_range,
        local_origin.z / cos_a,
        local_direction.z / cos_a,
        cone_range[1],
        tolerances,
    ) else {
        return Ok(CurveSurfaceIntersections::complete_empty());
    };

    if interval.width() > tolerances.linear() {
        let Some(v_start) = fit_scalar_parameter(
            v_at(local_origin, local_direction, interval.lo, cos_a),
            cone_range[1],
            tolerances.linear(),
        ) else {
            return Ok(CurveSurfaceIntersections::complete_empty());
        };
        let Some(v_end) = fit_scalar_parameter(
            v_at(local_origin, local_direction, interval.hi, cos_a),
            cone_range[1],
            tolerances.linear(),
        ) else {
            return Ok(CurveSurfaceIntersections::complete_empty());
        };
        let overlap = CurveSurfaceOverlap {
            curve: interval,
            uv_start: [u, v_start],
            uv_end: [u, v_end],
        };
        return CurveSurfaceIntersections::canonicalized_complete(Vec::new(), vec![overlap]);
    }

    let t_line = ((interval.lo + interval.hi) / 2.0).clamp(line_range.lo, line_range.hi);
    let Some(v) = fit_scalar_parameter(
        v_at(local_origin, local_direction, t_line, cos_a),
        cone_range[1],
        tolerances.linear(),
    ) else {
        return Ok(CurveSurfaceIntersections::complete_empty());
    };
    let uv = [u, v];
    let kind = if cone.normal(uv).is_none() {
        ContactKind::Singular
    } else {
        ContactKind::Tangent
    };
    let points = accept_curve_surface_candidate(line, t_line, cone, uv, kind, tolerances)
        .into_iter()
        .collect::<Vec<CurveSurfacePoint>>();
    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn cone_uv(
    local: Vec3,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
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

fn generator_u(
    local_origin: Vec3,
    local_direction: Vec3,
    line_range: ParamRange,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<f64> {
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    for t in [
        line_range.lo,
        line_range.hi,
        (line_range.lo + line_range.hi) / 2.0,
        0.0_f64.clamp(line_range.lo, line_range.hi),
    ] {
        let local = local_origin + local_direction * t;
        let v = local.z / cos_a;
        let signed_radius = cone.radius() + v * sin_a;
        if signed_radius.abs() > tolerances.linear() {
            let raw_u = math::atan2(local.y / signed_radius, local.x / signed_radius);
            return fit_periodic_parameter(
                raw_u,
                cone_range[0],
                parameter_tolerance(signed_radius.abs(), tolerances),
            );
        }
    }

    let v_direction = local_direction.z / cos_a;
    if v_direction.abs() > tolerances.angular() {
        let ruling = local_direction / v_direction;
        let radial = Vec3::new(ruling.x / sin_a, ruling.y / sin_a, 0.0);
        let raw_u = math::atan2(radial.y, radial.x);
        return fit_periodic_parameter(raw_u, cone_range[0], tolerances.angular());
    }

    Some(cone_range[0].lo)
}

fn solve_quadratic(a: f64, b: f64, c: f64, tolerances: Tolerances) -> Vec<(f64, bool)> {
    let coeff_tol = 1e-14;
    if a.abs() <= coeff_tol {
        if b.abs() <= coeff_tol {
            return Vec::new();
        }
        return vec![(-c / b, false)];
    }

    let discriminant = b * b - 4.0 * a * c;
    let discriminant_tolerance =
        (tolerances.linear() * (a.abs() + b.abs() + c.abs() + 1.0)).max(1e-12);
    if discriminant < -discriminant_tolerance {
        return Vec::new();
    }
    if discriminant.abs() <= discriminant_tolerance {
        return vec![(-b / (2.0 * a), true)];
    }

    let root = discriminant.max(0.0).sqrt();
    vec![
        ((-b - root) / (2.0 * a), false),
        ((-b + root) / (2.0 * a), false),
    ]
}

fn coefficients_are_zero(
    a: f64,
    b: f64,
    c: f64,
    local_origin: Vec3,
    line_range: ParamRange,
    tolerances: Tolerances,
) -> bool {
    let scale = (local_origin.norm() + line_range.lo.abs().max(line_range.hi.abs()) + 1.0).max(1.0);
    a.abs() <= tolerances.angular().max(1e-12)
        && b.abs() <= tolerances.linear() * scale
        && c.abs() <= tolerances.linear() * scale * scale
}

fn implicit_zero_on_range(
    local_origin: Vec3,
    local_direction: Vec3,
    radius: f64,
    tan_a: f64,
    range: ParamRange,
    tolerances: Tolerances,
) -> bool {
    [range.lo, (range.lo + range.hi) / 2.0, range.hi]
        .into_iter()
        .all(|t| {
            let local = local_origin + local_direction * t;
            let q = radius + local.z * tan_a;
            let scale = (local.x * local.x + local.y * local.y)
                .sqrt()
                .max(q.abs())
                .max(1.0);
            implicit_value(local, radius, tan_a).abs() <= 4.0 * tolerances.linear() * scale
        })
}

fn implicit_value(local: Vec3, radius: f64, tan_a: f64) -> f64 {
    let q = radius + local.z * tan_a;
    local.x * local.x + local.y * local.y - q * q
}

fn fit_line_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn clip_linear_interval(
    interval: ParamRange,
    origin: f64,
    direction: f64,
    range: ParamRange,
    tolerances: Tolerances,
) -> Option<ParamRange> {
    if direction.abs() <= tolerances.angular() {
        if origin < range.lo - tolerances.linear() || origin > range.hi + tolerances.linear() {
            None
        } else {
            Some(interval)
        }
    } else {
        let t0 = (range.lo - origin) / direction;
        let t1 = (range.hi - origin) / direction;
        let lo = interval.lo.max(t0.min(t1));
        let hi = interval.hi.min(t0.max(t1));
        if hi < lo - tolerances.linear() {
            None
        } else {
            Some(ParamRange::new(
                lo.clamp(interval.lo, interval.hi),
                hi.clamp(interval.lo, interval.hi),
            ))
        }
    }
}

fn v_at(local_origin: Vec3, local_direction: Vec3, t: f64, cos_a: f64) -> f64 {
    (local_origin.z + local_direction.z * t) / cos_a
}

fn parameter_tolerance(radius: f64, tolerances: Tolerances) -> f64 {
    (tolerances.linear() / radius).max(tolerances.angular())
}

fn push_distinct(
    points: &mut Vec<CurveSurfacePoint>,
    candidate: CurveSurfacePoint,
    tolerances: Tolerances,
) {
    if !points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        points.push(candidate);
    }
}

fn validate_ranges(line_range: ParamRange, cone_range: [ParamRange; 2]) -> Result<()> {
    if !line_range.is_finite() || line_range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "line/cone intersection requires a finite non-reversed line range",
        });
    }
    if cone_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "line/cone intersection requires finite non-reversed surface ranges",
        });
    }
    Ok(())
}
