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
use kgeom::surface::{Cone, Surface};
use kgeom::vec::Vec3;

/// Intersect an ellipse restricted to a finite range with a finite cone
/// parameter window.
pub fn intersect_bounded_ellipse_cone(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(
        ellipse_range,
        ellipse.minor_radius(),
        cone_range,
        tolerances,
    )?;

    let context = EllipseConeContext::new(ellipse, ellipse_range, cone, cone_range, tolerances);
    let coeffs = implicit_coefficients(&context);
    let tolerance = implicit_tolerance(&context);
    if coeffs.iter().all(|coeff| coeff.abs() <= tolerance) {
        return contained_ellipse_cone(&context);
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

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

struct EllipseConeContext<'a> {
    ellipse: &'a Ellipse,
    ellipse_range: ParamRange,
    cone: &'a Cone,
    cone_range: [ParamRange; 2],
    local_center: Vec3,
    local_x: Vec3,
    local_y: Vec3,
    sin_a: f64,
    cos_a: f64,
    tan_a: f64,
    tolerances: Tolerances,
}

impl<'a> EllipseConeContext<'a> {
    fn new(
        ellipse: &'a Ellipse,
        ellipse_range: ParamRange,
        cone: &'a Cone,
        cone_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let ellipse_x = ellipse.frame().x();
        let ellipse_y = ellipse.frame().y();
        let (sin_a, cos_a) = math::sincos(cone.half_angle());
        Self {
            ellipse,
            ellipse_range,
            cone,
            cone_range,
            local_center: cone.frame().to_local(ellipse.frame().origin()),
            local_x: Vec3::new(
                ellipse_x.dot(cone.frame().x()),
                ellipse_x.dot(cone.frame().y()),
                ellipse_x.dot(cone.frame().z()),
            ),
            local_y: Vec3::new(
                ellipse_y.dot(cone.frame().x()),
                ellipse_y.dot(cone.frame().y()),
                ellipse_y.dot(cone.frame().z()),
            ),
            sin_a,
            cos_a,
            tan_a: sin_a / cos_a,
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
        let Some(uv) = cone_uv(local, self.cone, self.cone_range, self.tolerances) else {
            return;
        };
        let kind = self.contact_kind(t_curve, uv, force_tangent);
        if let Some(point) = accept_curve_surface_candidate(
            self.ellipse,
            t_curve,
            self.cone,
            uv,
            kind,
            self.tolerances,
        ) {
            push_distinct(points, point, self.tolerances);
        }
    }

    fn contact_kind(&self, t_curve: f64, uv: [f64; 2], force_tangent: bool) -> ContactKind {
        if self.cone.normal(uv).is_none() {
            return ContactKind::Singular;
        }
        if force_tangent {
            return ContactKind::Tangent;
        }
        let Some(normal) = self.cone.normal(uv) else {
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

fn contained_ellipse_cone(context: &EllipseConeContext<'_>) -> Result<CurveSurfaceIntersections> {
    let t_tol = context.curve_parameter_tolerance();
    if context.ellipse_range.width() <= t_tol {
        let mut points = Vec::new();
        context.add_contact(&mut points, context.ellipse_range.lo, true);
        return CurveSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    let mut cuts = vec![context.ellipse_range.lo, context.ellipse_range.hi];
    push_cone_window_cuts(context, &mut cuts);
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
        if cone_uv(
            context.local_point(mid),
            context.cone,
            context.cone_range,
            context.tolerances,
        )
        .is_none()
        {
            continue;
        }
        let Some(uv_start) = cone_uv(
            context.local_point(lo),
            context.cone,
            context.cone_range,
            context.tolerances,
        ) else {
            continue;
        };
        let Some(uv_end) = cone_uv(
            context.local_point(hi),
            context.cone,
            context.cone_range,
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

    CurveSurfaceIntersections::canonicalized_complete(points, overlaps)
}

fn push_cone_window_cuts(context: &EllipseConeContext<'_>, cuts: &mut Vec<f64>) {
    let z_c = context.local_center.z;
    let z_a = context.local_x.z * context.ellipse.major_radius();
    let z_b = context.local_y.z * context.ellipse.minor_radius();
    for v_bound in [context.cone_range[1].lo, context.cone_range[1].hi] {
        let z_bound = v_bound * context.cos_a;
        for (root, _) in trig_linear_roots(
            z_a,
            z_b,
            z_c - z_bound,
            context.ellipse_range,
            context.tolerances.linear(),
        ) {
            push_scalar(cuts, root, context.curve_parameter_tolerance());
        }
    }

    for u_bound in [context.cone_range[0].lo, context.cone_range[0].hi] {
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

fn implicit_coefficients(context: &EllipseConeContext<'_>) -> [f64; 5] {
    let a_vec = context.local_x * context.ellipse.major_radius();
    let b_vec = context.local_y * context.ellipse.minor_radius();
    let c = [context.local_center.x, context.local_center.y];
    let a = [a_vec.x, a_vec.y];
    let b = [b_vec.x, b_vec.y];
    let q_c = context.cone.radius() + context.local_center.z * context.tan_a;
    let q_a = a_vec.z * context.tan_a;
    let q_b = b_vec.z * context.tan_a;
    let c0 = c[0] * c[0] + c[1] * c[1] - q_c * q_c;
    let cos = 2.0 * (c[0] * a[0] + c[1] * a[1]) - 2.0 * q_c * q_a;
    let sin = 2.0 * (c[0] * b[0] + c[1] * b[1]) - 2.0 * q_c * q_b;
    let cos2 = a[0] * a[0] + a[1] * a[1] - q_a * q_a;
    let sin2 = b[0] * b[0] + b[1] * b[1] - q_b * q_b;
    let sin_cos = 2.0 * (a[0] * b[0] + a[1] * b[1]) - 2.0 * q_a * q_b;

    [
        c0 + cos + cos2,
        2.0 * sin + 2.0 * sin_cos,
        2.0 * c0 - 2.0 * cos2 + 4.0 * sin2,
        2.0 * sin - 2.0 * sin_cos,
        c0 - cos + cos2,
    ]
}

fn implicit_value(context: &EllipseConeContext<'_>, t_curve: f64) -> f64 {
    let local = context.local_point(t_curve);
    let q = context.cone.radius() + local.z * context.tan_a;
    local.x * local.x + local.y * local.y - q * q
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

fn longitude_matches_bound(context: &EllipseConeContext<'_>, local: Vec3, bound: f64) -> bool {
    let v = local.z / context.cos_a;
    let signed_radius = context.cone.radius() + v * context.sin_a;
    if signed_radius.abs() <= context.tolerances.linear() {
        return true;
    }
    let raw_u = math::atan2(local.y / signed_radius, local.x / signed_radius);
    fit_periodic_parameter(
        raw_u,
        ParamRange::new(bound, bound),
        parameter_tolerance(signed_radius.abs(), context.tolerances),
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

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
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

fn implicit_tolerance(context: &EllipseConeContext<'_>) -> f64 {
    let scale = (context.local_extent() + context.cone.radius()).max(1.0);
    context.tolerances.linear() * scale
}

fn validate_ranges(
    ellipse_range: ParamRange,
    ellipse_radius: f64,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    if !ellipse_range.is_finite() || ellipse_range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "ellipse/cone intersection requires a finite non-reversed curve range",
        });
    }
    if ellipse_range.width()
        > core::f64::consts::TAU + parameter_tolerance(ellipse_radius, tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded ellipse range cannot span more than one period",
        });
    }
    if cone_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "ellipse/cone intersection requires finite non-reversed surface ranges",
        });
    }
    Ok(())
}
