use super::conic::{
    fit_periodic_parameter, parameter_tolerance, polynomial_derivative, real_polynomial_roots,
    trig_linear_roots,
};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Ellipse};
use kgeom::param::ParamRange;
use kgeom::surface::{Surface, Torus};
use kgeom::vec::Vec3;

/// Intersect an ellipse restricted to a finite range with a finite torus
/// parameter window.
pub fn intersect_bounded_ellipse_torus(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(
        ellipse_range,
        ellipse.minor_radius(),
        torus_range,
        tolerances,
    )?;

    let context = EllipseTorusContext::new(ellipse, ellipse_range, torus, torus_range, tolerances);
    let coeffs = implicit_coefficients(&context);
    let tolerance = implicit_tolerance(&context);
    if coeffs.iter().all(|coeff| coeff.abs() <= tolerance) {
        return contained_ellipse_torus(&context);
    }

    let mut points = Vec::new();
    for t_curve in implicit_roots(&coeffs, ellipse_range, tolerance) {
        context.add_contact(&mut points, t_curve, false);
    }
    for t_curve in implicit_roots(&polynomial_derivative(&coeffs), ellipse_range, tolerance) {
        context.add_contact(&mut points, t_curve, true);
    }
    if implicit_value(&context, core::f64::consts::PI).abs() <= tolerance {
        context.add_contact(&mut points, core::f64::consts::PI, true);
    }

    CurveSurfaceIntersections::canonicalized(points, Vec::new())
}

struct EllipseTorusContext<'a> {
    ellipse: &'a Ellipse,
    ellipse_range: ParamRange,
    torus: &'a Torus,
    torus_range: [ParamRange; 2],
    local_center: Vec3,
    local_x: Vec3,
    local_y: Vec3,
    tolerances: Tolerances,
}

impl<'a> EllipseTorusContext<'a> {
    fn new(
        ellipse: &'a Ellipse,
        ellipse_range: ParamRange,
        torus: &'a Torus,
        torus_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let ellipse_x = ellipse.frame().x();
        let ellipse_y = ellipse.frame().y();
        Self {
            ellipse,
            ellipse_range,
            torus,
            torus_range,
            local_center: torus.frame().to_local(ellipse.frame().origin()),
            local_x: Vec3::new(
                ellipse_x.dot(torus.frame().x()),
                ellipse_x.dot(torus.frame().y()),
                ellipse_x.dot(torus.frame().z()),
            ),
            local_y: Vec3::new(
                ellipse_y.dot(torus.frame().x()),
                ellipse_y.dot(torus.frame().y()),
                ellipse_y.dot(torus.frame().z()),
            ),
            tolerances,
        }
    }

    fn local_point(&self, t_curve: f64) -> Vec3 {
        let (sin, cos) = math::sincos(t_curve);
        self.local_center
            + self.local_x * (self.ellipse.major_radius() * cos)
            + self.local_y * (self.ellipse.minor_radius() * sin)
    }

    fn add_contact(&self, points: &mut Vec<CurveSurfacePoint>, t_curve: f64, force_tangent: bool) {
        let Some(t_curve) = fit_curve_parameter(
            t_curve,
            self.ellipse_range,
            self.curve_parameter_tolerance(),
        ) else {
            return;
        };
        let local = self.local_point(t_curve);
        let Some(uv) = torus_uv(local, self.torus, self.torus_range, self.tolerances) else {
            return;
        };
        let kind = self.contact_kind(t_curve, uv, force_tangent);
        if let Some(point) = accept_curve_surface_candidate(
            self.ellipse,
            t_curve,
            self.torus,
            uv,
            kind,
            self.tolerances,
        ) {
            push_distinct(points, point, self.tolerances);
        }
    }

    fn contact_kind(&self, t_curve: f64, uv: [f64; 2], force_tangent: bool) -> ContactKind {
        if force_tangent {
            return ContactKind::Tangent;
        }
        let Some(normal) = self.torus.normal(uv) else {
            return ContactKind::Singular;
        };
        let tangent = self.ellipse.eval_derivs(t_curve, 1).d[1];
        let Some(tangent) = tangent.normalized() else {
            return ContactKind::Singular;
        };
        if normal.dot(tangent).abs() <= self.tolerances.angular() {
            ContactKind::Tangent
        } else {
            ContactKind::Transverse
        }
    }

    fn curve_parameter_tolerance(&self) -> f64 {
        parameter_tolerance(self.ellipse.minor_radius(), self.tolerances)
    }

    fn local_extent(&self) -> f64 {
        self.local_center.norm() + self.ellipse.major_radius()
    }
}

fn contained_ellipse_torus(context: &EllipseTorusContext<'_>) -> Result<CurveSurfaceIntersections> {
    let t_tol = context.curve_parameter_tolerance();
    if context.ellipse_range.width() <= t_tol {
        let mut points = Vec::new();
        context.add_contact(&mut points, context.ellipse_range.lo, true);
        return CurveSurfaceIntersections::canonicalized(points, Vec::new());
    }

    let mut cuts = vec![context.ellipse_range.lo, context.ellipse_range.hi];
    push_torus_window_cuts(context, &mut cuts);
    cuts.sort_by(f64::total_cmp);
    dedup_sorted(&mut cuts, t_tol);

    let mut points = Vec::new();
    let mut overlaps = Vec::new();
    for interval in cuts.windows(2) {
        let lo = interval[0];
        let hi = interval[1];
        if hi - lo <= t_tol {
            continue;
        }
        let mid = (lo + hi) / 2.0;
        if torus_uv(
            context.local_point(mid),
            context.torus,
            context.torus_range,
            context.tolerances,
        )
        .is_none()
        {
            continue;
        }
        let Some(uv_start) = torus_uv(
            context.local_point(lo),
            context.torus,
            context.torus_range,
            context.tolerances,
        ) else {
            continue;
        };
        let Some(uv_end) = torus_uv(
            context.local_point(hi),
            context.torus,
            context.torus_range,
            context.tolerances,
        ) else {
            continue;
        };
        overlaps.push(CurveSurfaceOverlap {
            curve: ParamRange::new(lo, hi),
            uv_start,
            uv_end,
        });
    }

    for &cut in &cuts {
        let cut_point = context.ellipse.eval(cut);
        if overlaps.iter().any(|overlap| {
            (cut >= overlap.curve.lo - t_tol && cut <= overlap.curve.hi + t_tol)
                || cut_point.dist(context.ellipse.eval(overlap.curve.lo))
                    <= context.tolerances.linear()
                || cut_point.dist(context.ellipse.eval(overlap.curve.hi))
                    <= context.tolerances.linear()
        }) {
            continue;
        }
        context.add_contact(&mut points, cut, true);
    }

    CurveSurfaceIntersections::canonicalized(points, overlaps)
}

fn push_torus_window_cuts(context: &EllipseTorusContext<'_>, cuts: &mut Vec<f64>) {
    let z_c = context.local_center.z;
    let z_a = context.local_x.z * context.ellipse.major_radius();
    let z_b = context.local_y.z * context.ellipse.minor_radius();
    for v_bound in [context.torus_range[1].lo, context.torus_range[1].hi] {
        let z_bound = context.torus.minor_radius() * math::sin(v_bound);
        for (root, _) in trig_linear_roots(
            z_a,
            z_b,
            z_c - z_bound,
            context.ellipse_range,
            context.tolerances.linear(),
        ) {
            if !tube_angle_matches_bound(context, context.local_point(root), v_bound) {
                continue;
            }
            push_scalar(cuts, root, context.curve_parameter_tolerance());
        }
    }

    for u_bound in [context.torus_range[0].lo, context.torus_range[0].hi] {
        let (sin_u, cos_u) = math::sincos(u_bound);
        let c = -sin_u * context.local_center.x + cos_u * context.local_center.y;
        let a = context.ellipse.major_radius()
            * (-sin_u * context.local_x.x + cos_u * context.local_x.y);
        let b = context.ellipse.minor_radius()
            * (-sin_u * context.local_y.x + cos_u * context.local_y.y);
        for (root, _) in
            trig_linear_roots(a, b, c, context.ellipse_range, context.tolerances.linear())
        {
            if !longitude_matches_bound(context, context.local_point(root), u_bound) {
                continue;
            }
            push_scalar(cuts, root, context.curve_parameter_tolerance());
        }
    }
}

fn implicit_coefficients(context: &EllipseTorusContext<'_>) -> Vec<f64> {
    let a_vec = context.local_x * context.ellipse.major_radius();
    let b_vec = context.local_y * context.ellipse.minor_radius();
    let c = context.local_center;
    let major_sq = context.torus.major_radius() * context.torus.major_radius();
    let h0 = c.dot(c) + major_sq - context.torus.minor_radius() * context.torus.minor_radius();
    let h_cos = 2.0 * c.dot(a_vec);
    let h_sin = 2.0 * c.dot(b_vec);
    let h_cos2 = a_vec.dot(a_vec);
    let h_sin2 = b_vec.dot(b_vec);
    let h_sin_cos = 2.0 * a_vec.dot(b_vec);
    let h_coeffs =
        trig_quadratic_half_angle_coefficients(h0, h_cos, h_sin, h_cos2, h_sin2, h_sin_cos);

    let q0 = c.x * c.x + c.y * c.y;
    let q_cos = 2.0 * (c.x * a_vec.x + c.y * a_vec.y);
    let q_sin = 2.0 * (c.x * b_vec.x + c.y * b_vec.y);
    let q_cos2 = a_vec.x * a_vec.x + a_vec.y * a_vec.y;
    let q_sin2 = b_vec.x * b_vec.x + b_vec.y * b_vec.y;
    let q_sin_cos = 2.0 * (a_vec.x * b_vec.x + a_vec.y * b_vec.y);
    let q_coeffs =
        trig_quadratic_half_angle_coefficients(q0, q_cos, q_sin, q_cos2, q_sin2, q_sin_cos);

    let h_sq = poly_square(&h_coeffs);
    let q_with_denominator = poly_mul(&q_coeffs, &[1.0, 0.0, 2.0, 0.0, 1.0]);
    h_sq.iter()
        .zip(q_with_denominator.iter())
        .map(|(h, q)| h - 4.0 * major_sq * q)
        .collect()
}

fn trig_quadratic_half_angle_coefficients(
    c0: f64,
    cos: f64,
    sin: f64,
    cos2: f64,
    sin2: f64,
    sin_cos: f64,
) -> [f64; 5] {
    [
        c0 + cos + cos2,
        2.0 * sin + 2.0 * sin_cos,
        2.0 * c0 - 2.0 * cos2 + 4.0 * sin2,
        2.0 * sin - 2.0 * sin_cos,
        c0 - cos + cos2,
    ]
}

fn poly_square(coeffs: &[f64]) -> Vec<f64> {
    poly_mul(coeffs, coeffs)
}

fn poly_mul(a: &[f64], b: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; a.len() + b.len() - 1];
    for (i, a_coeff) in a.iter().enumerate() {
        for (j, b_coeff) in b.iter().enumerate() {
            out[i + j] += a_coeff * b_coeff;
        }
    }
    out
}

fn implicit_value(context: &EllipseTorusContext<'_>, t_curve: f64) -> f64 {
    let local = context.local_point(t_curve);
    let s = local.dot(local);
    let q = local.x * local.x + local.y * local.y;
    let h = s + context.torus.major_radius() * context.torus.major_radius()
        - context.torus.minor_radius() * context.torus.minor_radius();
    h * h - 4.0 * context.torus.major_radius() * context.torus.major_radius() * q
}

fn implicit_roots(coeffs: &[f64], range: ParamRange, tolerance: f64) -> Vec<f64> {
    let mut roots = Vec::new();
    for y in real_polynomial_roots(coeffs) {
        let t = 2.0 * math::atan2(y, 1.0);
        let Some(t) = fit_periodic_parameter(t, range, tolerance) else {
            continue;
        };
        push_scalar(&mut roots, t, tolerance.max(1e-10));
    }
    roots
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

fn longitude_matches_bound(context: &EllipseTorusContext<'_>, local: Vec3, bound: f64) -> bool {
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    if xy <= context.tolerances.linear() {
        return true;
    }
    let raw_u = math::atan2(local.y, local.x);
    fit_periodic_parameter(
        raw_u,
        ParamRange::new(bound, bound),
        parameter_tolerance(xy, context.tolerances),
    )
    .is_some()
}

fn tube_angle_matches_bound(context: &EllipseTorusContext<'_>, local: Vec3, bound: f64) -> bool {
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let raw_v = math::atan2(local.z, xy - context.torus.major_radius());
    fit_periodic_parameter(
        raw_v,
        ParamRange::new(bound, bound),
        parameter_tolerance(context.torus.minor_radius(), context.tolerances),
    )
    .is_some()
}

fn fit_curve_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
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

fn push_scalar(values: &mut Vec<f64>, candidate: f64, tolerance: f64) {
    if !values
        .iter()
        .any(|existing| (*existing - candidate).abs() <= tolerance.max(1e-12))
    {
        values.push(candidate);
    }
}

fn dedup_sorted(values: &mut Vec<f64>, tolerance: f64) {
    let mut deduped = Vec::with_capacity(values.len());
    for value in values.drain(..) {
        if !deduped
            .iter()
            .any(|existing: &f64| (*existing - value).abs() <= tolerance.max(1e-12))
        {
            deduped.push(value);
        }
    }
    *values = deduped;
}

fn implicit_tolerance(context: &EllipseTorusContext<'_>) -> f64 {
    let scale =
        (context.local_extent() + context.torus.major_radius() + context.torus.minor_radius())
            .max(1.0);
    context.tolerances.linear() * scale * scale * scale
}

fn validate_ranges(
    ellipse_range: ParamRange,
    ellipse_radius: f64,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    if !ellipse_range.is_finite() || ellipse_range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "ellipse/torus intersection requires a finite non-reversed curve range",
        });
    }
    if ellipse_range.width()
        > core::f64::consts::TAU + parameter_tolerance(ellipse_radius, tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded ellipse range cannot span more than one period",
        });
    }
    if torus_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "ellipse/torus intersection requires finite non-reversed surface ranges",
        });
    }
    Ok(())
}
