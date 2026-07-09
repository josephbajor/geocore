use super::conic::{fit_periodic_parameter, parameter_tolerance, trig_linear_roots};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::param::ParamRange;
use kgeom::surface::{Sphere, Surface};
use kgeom::vec::Vec3;

/// Intersect a circle restricted to a finite range with a finite sphere
/// parameter window.
pub fn intersect_bounded_circle_sphere(
    circle: &Circle,
    circle_range: ParamRange,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(circle_range, circle.radius(), sphere_range, tolerances)?;

    let context = CircleSphereContext::new(circle, circle_range, sphere, sphere_range, tolerances);
    let delta = circle.frame().origin() - sphere.frame().origin();
    let dx = delta.dot(circle.frame().x());
    let dy = delta.dot(circle.frame().y());
    let radius = circle.radius();
    let a = 2.0 * radius * dx;
    let b = 2.0 * radius * dy;
    let c = delta.norm_sq() + radius * radius - sphere.radius() * sphere.radius();
    let tolerance = implicit_tolerance(delta.norm(), radius, sphere.radius(), tolerances);
    let amplitude = (a * a + b * b).sqrt();

    if amplitude <= tolerance {
        if c.abs() > tolerance {
            return Ok(CurveSurfaceIntersections::default());
        }
        return contained_circle_sphere(&context);
    }

    let mut points = Vec::new();
    for (t_curve, tangent) in trig_linear_roots(a, b, c, circle_range, tolerance) {
        context.add_contact(&mut points, t_curve, tangent);
    }

    CurveSurfaceIntersections::canonicalized(points, Vec::new())
}

struct CircleSphereContext<'a> {
    circle: &'a Circle,
    circle_range: ParamRange,
    sphere: &'a Sphere,
    sphere_range: [ParamRange; 2],
    local_center: Vec3,
    local_x: Vec3,
    local_y: Vec3,
    tolerances: Tolerances,
}

impl<'a> CircleSphereContext<'a> {
    fn new(
        circle: &'a Circle,
        circle_range: ParamRange,
        sphere: &'a Sphere,
        sphere_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let circle_x = circle.frame().x();
        let circle_y = circle.frame().y();
        Self {
            circle,
            circle_range,
            sphere,
            sphere_range,
            local_center: sphere.frame().to_local(circle.frame().origin()),
            local_x: Vec3::new(
                circle_x.dot(sphere.frame().x()),
                circle_x.dot(sphere.frame().y()),
                circle_x.dot(sphere.frame().z()),
            ),
            local_y: Vec3::new(
                circle_y.dot(sphere.frame().x()),
                circle_y.dot(sphere.frame().y()),
                circle_y.dot(sphere.frame().z()),
            ),
            tolerances,
        }
    }

    fn local_point(&self, t_curve: f64) -> Vec3 {
        let (sin, cos) = math::sincos(t_curve);
        self.local_center + (self.local_x * cos + self.local_y * sin) * self.circle.radius()
    }

    fn add_contact(&self, points: &mut Vec<CurveSurfacePoint>, t_curve: f64, force_tangent: bool) {
        let Some(t_curve) =
            fit_curve_parameter(t_curve, self.circle_range, self.curve_parameter_tolerance())
        else {
            return;
        };
        let local = self.local_point(t_curve);
        let Some(uv) = sphere_uv(
            local,
            self.sphere_range,
            self.sphere.radius(),
            self.tolerances,
        ) else {
            return;
        };
        let kind = self.contact_kind(uv, force_tangent);
        if let Some(point) = accept_curve_surface_candidate(
            self.circle,
            t_curve,
            self.sphere,
            uv,
            kind,
            self.tolerances,
        ) {
            push_distinct(points, point, self.tolerances);
        }
    }

    fn contact_kind(&self, uv: [f64; 2], force_tangent: bool) -> ContactKind {
        if force_tangent {
            ContactKind::Tangent
        } else if self.sphere.normal(uv).is_none() {
            ContactKind::Singular
        } else {
            ContactKind::Transverse
        }
    }

    fn curve_parameter_tolerance(&self) -> f64 {
        parameter_tolerance(self.circle.radius(), self.tolerances)
    }
}

fn contained_circle_sphere(context: &CircleSphereContext<'_>) -> Result<CurveSurfaceIntersections> {
    let t_tol = context.curve_parameter_tolerance();
    if context.circle_range.width() <= t_tol {
        let mut points = Vec::new();
        context.add_contact(&mut points, context.circle_range.lo, true);
        return CurveSurfaceIntersections::canonicalized(points, Vec::new());
    }

    let mut cuts = vec![context.circle_range.lo, context.circle_range.hi];
    push_sphere_window_cuts(context, &mut cuts);
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
        if sphere_uv(
            context.local_point(mid),
            context.sphere_range,
            context.sphere.radius(),
            context.tolerances,
        )
        .is_none()
        {
            continue;
        }
        let Some(uv_start) = sphere_uv(
            context.local_point(lo),
            context.sphere_range,
            context.sphere.radius(),
            context.tolerances,
        ) else {
            continue;
        };
        let Some(uv_end) = sphere_uv(
            context.local_point(hi),
            context.sphere_range,
            context.sphere.radius(),
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
        let cut_point = context.circle.eval(cut);
        if overlaps.iter().any(|overlap| {
            (cut >= overlap.curve.lo - t_tol && cut <= overlap.curve.hi + t_tol)
                || cut_point.dist(context.circle.eval(overlap.curve.lo))
                    <= context.tolerances.linear()
                || cut_point.dist(context.circle.eval(overlap.curve.hi))
                    <= context.tolerances.linear()
        }) {
            continue;
        }
        context.add_contact(&mut points, cut, true);
    }

    CurveSurfaceIntersections::canonicalized(points, overlaps)
}

fn push_sphere_window_cuts(context: &CircleSphereContext<'_>, cuts: &mut Vec<f64>) {
    let radius = context.circle.radius();
    let sphere_radius = context.sphere.radius();
    let z_c = context.local_center.z;
    let z_a = context.local_x.z * radius;
    let z_b = context.local_y.z * radius;
    for v_bound in [context.sphere_range[1].lo, context.sphere_range[1].hi] {
        let z_bound = sphere_radius * math::sin(v_bound);
        for (root, _) in trig_linear_roots(
            z_a,
            z_b,
            z_c - z_bound,
            context.circle_range,
            context.tolerances.linear(),
        ) {
            push_scalar(cuts, root, context.curve_parameter_tolerance());
        }
    }

    for u_bound in [context.sphere_range[0].lo, context.sphere_range[0].hi] {
        let (sin_u, cos_u) = math::sincos(u_bound);
        let c = -sin_u * context.local_center.x + cos_u * context.local_center.y;
        let a = radius * (-sin_u * context.local_x.x + cos_u * context.local_x.y);
        let b = radius * (-sin_u * context.local_y.x + cos_u * context.local_y.y);
        for (root, _) in
            trig_linear_roots(a, b, c, context.circle_range, context.tolerances.linear())
        {
            if !longitude_matches_bound(context.local_point(root), u_bound, context.tolerances) {
                continue;
            }
            push_scalar(cuts, root, context.curve_parameter_tolerance());
        }
    }
}

fn sphere_uv(
    local: Vec3,
    sphere_range: [ParamRange; 2],
    radius: f64,
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let raw_v = math::atan2(local.z, xy);
    let v_tol = parameter_tolerance(radius, tolerances);
    let v = fit_scalar_parameter(raw_v, sphere_range[1], v_tol)?;
    let u = if xy <= tolerances.linear() {
        sphere_range[0].lo
    } else {
        let raw_u = math::atan2(local.y, local.x);
        fit_periodic_parameter(raw_u, sphere_range[0], v_tol)?
    };
    Some([u, v])
}

fn longitude_matches_bound(local: Vec3, bound: f64, tolerances: Tolerances) -> bool {
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    if xy <= tolerances.linear() {
        return true;
    }
    let raw_u = math::atan2(local.y, local.x);
    fit_periodic_parameter(
        raw_u,
        ParamRange::new(bound, bound),
        parameter_tolerance(xy, tolerances),
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

fn implicit_tolerance(
    center_distance: f64,
    circle_radius: f64,
    sphere_radius: f64,
    tolerances: Tolerances,
) -> f64 {
    let scale = (center_distance + circle_radius + sphere_radius).max(1.0);
    tolerances.linear() * scale
}

fn validate_ranges(
    circle_range: ParamRange,
    circle_radius: f64,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    if !circle_range.is_finite() || circle_range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "circle/sphere intersection requires a finite non-reversed curve range",
        });
    }
    if circle_range.width()
        > core::f64::consts::TAU + parameter_tolerance(circle_radius, tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded circle range cannot span more than one period",
        });
    }
    if sphere_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "circle/sphere intersection requires finite non-reversed surface ranges",
        });
    }
    Ok(())
}
