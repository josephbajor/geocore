use super::conic::{fit_periodic_parameter, polynomial_derivative, real_polynomial_roots};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfacePoint, accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::param::ParamRange;
use kgeom::surface::{Surface, Torus};
use kgeom::vec::Vec3;

/// Intersect a line restricted to a finite range with a finite torus
/// parameter window.
///
/// The torus implicit equation reduces to a quartic in the line parameter.
/// Critical points of that quartic are also tested as tolerance candidates so
/// near-tangent contacts just outside the exact surface are reported.
pub fn intersect_bounded_line_torus(
    line: &Line,
    line_range: ParamRange,
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(line_range, torus_range)?;

    let local_origin = torus.frame().to_local(line.origin());
    let direction = line.dir();
    let local_direction = Vec3::new(
        direction.dot(torus.frame().x()),
        direction.dot(torus.frame().y()),
        direction.dot(torus.frame().z()),
    );
    let coefficients = implicit_line_coefficients(local_origin, local_direction, torus);
    let context = TorusLineContext {
        line,
        line_range,
        torus,
        torus_range,
        local_origin,
        local_direction,
        tolerances,
    };
    let mut points = Vec::new();

    for t_line in real_polynomial_roots(&coefficients) {
        context.add_candidate(&mut points, t_line, false);
    }

    for t_line in real_polynomial_roots(&polynomial_derivative(&coefficients)) {
        context.add_candidate(&mut points, t_line, true);
    }

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

struct TorusLineContext<'a> {
    line: &'a Line,
    line_range: ParamRange,
    torus: &'a Torus,
    torus_range: [ParamRange; 2],
    local_origin: Vec3,
    local_direction: Vec3,
    tolerances: Tolerances,
}

impl TorusLineContext<'_> {
    fn add_candidate(&self, points: &mut Vec<CurveSurfacePoint>, t_line: f64, force_tangent: bool) {
        let Some(t_line) = fit_line_parameter(t_line, self.line_range, self.tolerances.linear())
        else {
            return;
        };
        let local = self.local_origin + self.local_direction * t_line;
        let Some(uv) = torus_uv(local, self.torus, self.torus_range, self.tolerances) else {
            return;
        };
        let Some(normal) = self.torus.normal(uv) else {
            return;
        };
        let kind =
            if force_tangent || normal.dot(self.line.dir()).abs() <= self.tolerances.angular() {
                ContactKind::Tangent
            } else {
                ContactKind::Transverse
            };
        if let Some(point) =
            accept_curve_surface_candidate(self.line, t_line, self.torus, uv, kind, self.tolerances)
        {
            push_distinct(points, point, self.tolerances);
        }
    }
}

fn implicit_line_coefficients(
    local_origin: Vec3,
    local_direction: Vec3,
    torus: &Torus,
) -> [f64; 5] {
    let s2 = local_direction.dot(local_direction);
    let s1 = 2.0 * local_origin.dot(local_direction);
    let s0 = local_origin.dot(local_origin);
    let q2 = local_direction.x * local_direction.x + local_direction.y * local_direction.y;
    let q1 = 2.0 * (local_origin.x * local_direction.x + local_origin.y * local_direction.y);
    let q0 = local_origin.x * local_origin.x + local_origin.y * local_origin.y;
    let major_sq = torus.major_radius() * torus.major_radius();
    let h0 = s0 + major_sq - torus.minor_radius() * torus.minor_radius();
    let h1 = s1;
    let h2 = s2;

    [
        h0 * h0 - 4.0 * major_sq * q0,
        2.0 * h0 * h1 - 4.0 * major_sq * q1,
        h1 * h1 + 2.0 * h0 * h2 - 4.0 * major_sq * q2,
        2.0 * h1 * h2,
        h2 * h2,
    ]
}

fn torus_uv(
    local: Vec3,
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
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

fn fit_line_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
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

fn validate_ranges(line_range: ParamRange, torus_range: [ParamRange; 2]) -> Result<()> {
    if !line_range.is_finite() || line_range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "line/torus intersection requires a finite non-reversed line range",
        });
    }
    if torus_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "line/torus intersection requires finite non-reversed surface ranges",
        });
    }
    Ok(())
}
