use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint,
    CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint, ParamOrientation,
    accept_curve_curve_candidate, accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::predicates::{Orientation, harmonic_half_angle_roots};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Cylinder, Sphere, Surface, Torus};
use kgeom::vec::{Point3, Vec3};

pub(super) const HARMONIC_ROOT_CLASSIFICATION_REASON: &str =
    "exact harmonic root classification could not be represented";

pub(super) use super::parameter::angular_parameter_tolerance as parameter_tolerance;

/// Compatibility seam for existing trigonometric conic solvers, whose
/// angular parameterizations all have period `TAU`.
pub(super) fn fit_periodic_parameter(
    candidate: f64,
    range: ParamRange,
    tolerance: f64,
) -> Option<f64> {
    super::parameter::fit_periodic_parameter(candidate, range, core::f64::consts::TAU, tolerance)
}

pub(super) fn ellipse_parameter(local: Vec3, ellipse: &Ellipse) -> f64 {
    math::atan2(
        local.y / ellipse.minor_radius(),
        local.x / ellipse.major_radius(),
    )
}

pub(super) fn push_angle_root(roots: &mut Vec<f64>, t: f64) {
    let t = canonical_angle(t);
    if !roots
        .iter()
        .any(|existing| angular_distance(*existing, t) <= 1e-10)
    {
        roots.push(t);
    }
}

pub(super) fn canonical_angle(t: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let mut s = t % period;
    if s < 0.0 {
        s += period;
    }
    if period - s <= 1e-14 { 0.0 } else { s }
}

pub(super) fn trig_linear_roots(
    a: f64,
    b: f64,
    c: f64,
    range: ParamRange,
    tolerance: f64,
) -> Option<Vec<(f64, bool)>> {
    let solution = harmonic_half_angle_roots(a, b, c)?;
    let mut roots = Vec::new();
    let discriminant = solution.discriminant();
    let tangent = discriminant == Orientation::Zero;
    for &y in solution.finite_roots() {
        push_periodic_trig_root(
            &mut roots,
            2.0 * math::atan2(y, 1.0),
            range,
            tolerance,
            tangent,
        );
    }
    if solution.has_infinity_root() {
        push_periodic_trig_root(&mut roots, core::f64::consts::PI, range, tolerance, tangent);
    }
    Some(roots)
}

fn angular_distance(a: f64, b: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let d = (a - b).abs();
    d.min(period - d)
}

#[cfg(test)]
mod quadratic_root_tests {
    use super::*;

    #[test]
    fn public_trig_roots_preserve_ordinary_bits_and_scale() {
        let range = ParamRange::new(-core::f64::consts::PI, core::f64::consts::PI);
        let ordinary = trig_linear_roots(0.5, -1.5, 1.5, range, 1e-14).unwrap();
        assert_eq!(
            ordinary
                .iter()
                .map(|(root, tangent)| (root.to_bits(), *tangent))
                .collect::<Vec<_>>(),
            vec![
                ((2.0 * math::atan2(1.0, 1.0)).to_bits(), false),
                ((2.0 * math::atan2(2.0, 1.0)).to_bits(), false),
            ]
        );
        for scale in [2.0_f64.powi(700), 2.0_f64.powi(-700)] {
            assert_eq!(
                trig_linear_roots(0.5 * scale, -1.5 * scale, 1.5 * scale, range, 1e-14).unwrap(),
                ordinary
            );
        }
    }

    #[test]
    fn trig_multiplicity_ignores_root_count_tolerance() {
        let range = ParamRange::new(0.0, core::f64::consts::TAU);
        let tolerance = 1e-2;

        let secant = trig_linear_roots(1.0, 0.0, 0.991, range, tolerance).unwrap();
        assert_eq!(secant.len(), 2);
        assert!(!secant[0].1, "exact-positive discriminant is transverse");
        assert!(!secant[1].1, "exact-positive discriminant is transverse");
        assert!((secant[0].0 - secant[1].0).abs() > 0.25);
        for &(root, _) in &secant {
            assert!((math::cos(root) + 0.991).abs() < 2.0e-15);
        }

        let tangent = trig_linear_roots(1.0, 0.0, 1.0, range, tolerance).unwrap();
        assert_eq!(tangent.len(), 1);
        assert!(tangent[0].1);

        assert!(
            trig_linear_roots(1.0, 0.0, 1.0 + 1e-6, range, tolerance)
                .unwrap()
                .is_empty()
        );

        let partial = ParamRange::new(core::f64::consts::PI, core::f64::consts::TAU);
        assert_eq!(
            trig_linear_roots(1.0, 0.0, 0.991, partial, tolerance)
                .unwrap()
                .len(),
            1
        );
        assert!(
            trig_linear_roots(f64::MAX, f64::from_bits(1), 0.0, range, tolerance,).is_none(),
            "unrepresentable exact normalization must not become a miss"
        );
    }
}

fn push_periodic_trig_root(
    roots: &mut Vec<(f64, bool)>,
    candidate: f64,
    range: ParamRange,
    tolerance: f64,
    tangent: bool,
) {
    let Some(candidate) = fit_periodic_parameter(candidate, range, tolerance) else {
        return;
    };
    if !roots
        .iter()
        .any(|(existing, _)| (*existing - candidate).abs() <= tolerance.max(1e-12))
    {
        roots.push((candidate, tangent));
    }
}

pub(super) fn real_polynomial_roots(coeffs: &[f64]) -> Vec<f64> {
    let poly = trim_polynomial(coeffs);
    let degree = poly.len().saturating_sub(1);
    if degree == 0 {
        return Vec::new();
    }
    if degree == 1 {
        return vec![-poly[0] / poly[1]];
    }

    let bound = polynomial_root_bound(&poly);
    let mut critical = real_polynomial_roots(&polynomial_derivative(&poly));
    critical.retain(|x| x.is_finite() && *x > -bound && *x < bound);
    critical.sort_by(f64::total_cmp);
    dedup_sorted_scalars(&mut critical, 1e-10);

    let value_tol = polynomial_value_tolerance(&poly);
    let mut roots = Vec::new();
    let mut cuts = Vec::with_capacity(critical.len() + 2);
    cuts.push(-bound);
    cuts.extend(critical.iter().copied());
    cuts.push(bound);

    for &x in &critical {
        if eval_polynomial(&poly, x).abs() <= value_tol {
            roots.push(x);
        }
    }

    for interval in cuts.windows(2) {
        let lo = interval[0];
        let hi = interval[1];
        let f_lo = eval_polynomial(&poly, lo);
        let f_hi = eval_polynomial(&poly, hi);
        if f_lo.abs() <= value_tol {
            roots.push(lo);
            continue;
        }
        if f_hi.abs() <= value_tol {
            roots.push(hi);
            continue;
        }
        if f_lo.signum() == f_hi.signum() {
            continue;
        }
        roots.push(bisect_polynomial_root(&poly, lo, hi));
    }

    roots.retain(|x| x.is_finite());
    roots.sort_by(f64::total_cmp);
    dedup_sorted_scalars(&mut roots, 1e-9);
    roots
}

fn trim_polynomial(coeffs: &[f64]) -> Vec<f64> {
    let mut hi = coeffs.len();
    while hi > 1 && coeffs[hi - 1].abs() <= 1e-14 {
        hi -= 1;
    }
    coeffs[..hi].to_vec()
}

pub(super) fn polynomial_derivative(poly: &[f64]) -> Vec<f64> {
    poly.iter()
        .enumerate()
        .skip(1)
        .map(|(i, c)| *c * i as f64)
        .collect()
}

fn polynomial_root_bound(poly: &[f64]) -> f64 {
    let leading = poly[poly.len() - 1].abs();
    let mut max_ratio: f64 = 0.0;
    for coeff in &poly[..poly.len() - 1] {
        max_ratio = max_ratio.max(coeff.abs() / leading);
    }
    1.0 + max_ratio
}

fn polynomial_value_tolerance(poly: &[f64]) -> f64 {
    let scale = poly.iter().fold(0.0_f64, |acc, coeff| acc.max(coeff.abs()));
    (scale * 1e-12).max(1e-12)
}

fn eval_polynomial(poly: &[f64], x: f64) -> f64 {
    let mut y = 0.0;
    for &coeff in poly.iter().rev() {
        y = y * x + coeff;
    }
    y
}

fn bisect_polynomial_root(poly: &[f64], mut lo: f64, mut hi: f64) -> f64 {
    let mut f_lo = eval_polynomial(poly, lo);
    for _ in 0..100 {
        let mid = (lo + hi) / 2.0;
        let f_mid = eval_polynomial(poly, mid);
        if f_mid == 0.0 || (hi - lo).abs() <= 1e-13 * (1.0 + mid.abs()) {
            return mid;
        }
        if f_lo.signum() == f_mid.signum() {
            lo = mid;
            f_lo = f_mid;
        } else {
            hi = mid;
        }
    }
    (lo + hi) / 2.0
}

fn dedup_sorted_scalars(values: &mut Vec<f64>, tolerance: f64) {
    let mut out = Vec::with_capacity(values.len());
    for value in values.drain(..) {
        if !out
            .iter()
            .any(|existing: &f64| (*existing - value).abs() <= tolerance * (1.0 + value.abs()))
        {
            out.push(value);
        }
    }
    *values = out;
}

#[derive(Clone, Copy)]
enum ConicCurve<'a> {
    Circle(&'a Circle),
    Ellipse(&'a Ellipse),
}

impl<'a> ConicCurve<'a> {
    fn curve(self) -> &'a dyn Curve {
        match self {
            Self::Circle(curve) => curve,
            Self::Ellipse(curve) => curve,
        }
    }

    fn frame(self) -> &'a kgeom::frame::Frame {
        match self {
            Self::Circle(curve) => curve.frame(),
            Self::Ellipse(curve) => curve.frame(),
        }
    }

    fn parameter_scale(self) -> f64 {
        match self {
            Self::Circle(curve) => curve.radius(),
            Self::Ellipse(curve) => curve.minor_radius(),
        }
    }

    fn raw_parameter(self, point: Point3) -> f64 {
        let local = self.frame().to_local(point);
        match self {
            Self::Circle(_) => math::atan2(local.y, local.x),
            Self::Ellipse(curve) => ellipse_parameter(local, curve),
        }
    }
}

/// Shared bounded analytic-conic pair orchestration.
///
/// This owns the common finite-period validation, plane routing, inverse
/// parameter fitting, candidate classification/deduplication, and coincident
/// periodic overlap construction. Pair-specific quadratic/quartic root
/// arithmetic remains in the circle/circle, circle/ellipse, and
/// ellipse/ellipse strategies.
#[derive(Clone, Copy)]
pub(super) struct ConicPairConfig<'a> {
    a: ConicCurve<'a>,
    range_a: ParamRange,
    b: ConicCurve<'a>,
    range_b: ParamRange,
    parameter_tol: f64,
    tolerances: Tolerances,
}

/// Relative plane routing shared by every analytic conic pair.
pub(super) enum ConicPlaneRelation {
    Parallel,
    Crossing(Line),
}

impl<'a> ConicPairConfig<'a> {
    /// Validated circle/circle strategy inputs with legacy diagnostics.
    pub(super) fn circles(
        a: &'a Circle,
        range_a: ParamRange,
        b: &'a Circle,
        range_b: ParamRange,
        tolerances: Tolerances,
    ) -> Result<Self> {
        Self::new(
            ConicCurve::Circle(a),
            range_a,
            ConicCurve::Circle(b),
            range_b,
            tolerances,
            "circle/circle intersection requires finite non-reversed ranges",
            "bounded circle ranges cannot span more than one period",
        )
    }

    /// Validated circle/ellipse strategy inputs with legacy diagnostics.
    pub(super) fn circle_ellipse(
        circle: &'a Circle,
        circle_range: ParamRange,
        ellipse: &'a Ellipse,
        ellipse_range: ParamRange,
        tolerances: Tolerances,
    ) -> Result<Self> {
        Self::new(
            ConicCurve::Circle(circle),
            circle_range,
            ConicCurve::Ellipse(ellipse),
            ellipse_range,
            tolerances,
            "circle/ellipse intersection requires finite non-reversed ranges",
            "bounded circle and ellipse ranges cannot span more than one period",
        )
    }

    /// Validated ellipse/ellipse strategy inputs with legacy diagnostics.
    pub(super) fn ellipses(
        a: &'a Ellipse,
        range_a: ParamRange,
        b: &'a Ellipse,
        range_b: ParamRange,
        tolerances: Tolerances,
    ) -> Result<Self> {
        Self::new(
            ConicCurve::Ellipse(a),
            range_a,
            ConicCurve::Ellipse(b),
            range_b,
            tolerances,
            "ellipse/ellipse intersection requires finite non-reversed ranges",
            "bounded ellipse ranges cannot span more than one period",
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn new(
        a: ConicCurve<'a>,
        range_a: ParamRange,
        b: ConicCurve<'a>,
        range_b: ParamRange,
        tolerances: Tolerances,
        invalid_range_reason: &'static str,
        overwide_range_reason: &'static str,
    ) -> Result<Self> {
        if !range_a.is_finite()
            || !range_b.is_finite()
            || range_a.width() < 0.0
            || range_b.width() < 0.0
        {
            return Err(Error::InvalidGeometry {
                reason: invalid_range_reason,
            });
        }
        let tolerance_a = parameter_tolerance(a.parameter_scale(), tolerances);
        let tolerance_b = parameter_tolerance(b.parameter_scale(), tolerances);
        if range_a.width() > core::f64::consts::TAU + tolerance_a
            || range_b.width() > core::f64::consts::TAU + tolerance_b
        {
            return Err(Error::InvalidGeometry {
                reason: overwide_range_reason,
            });
        }
        Ok(Self {
            a,
            range_a,
            b,
            range_b,
            parameter_tol: tolerance_a.max(tolerance_b),
            tolerances,
        })
    }

    /// Classify parallel planes or construct their exact crossing line using
    /// the same operand-ordered arithmetic as the specialized solvers.
    pub(super) fn plane_relation(self) -> Result<ConicPlaneRelation> {
        let n1 = self.a.frame().z();
        let n2 = self.b.frame().z();
        let direction = n1.cross(n2);
        if direction.norm() <= self.tolerances.angular() {
            return Ok(ConicPlaneRelation::Parallel);
        }
        let denom = direction.norm_sq();
        let c1 = n1.dot(self.a.frame().origin());
        let c2 = n2.dot(self.b.frame().origin());
        let origin = ((n2 * c1 - n1 * c2).cross(direction)) / denom;
        Ok(ConicPlaneRelation::Crossing(Line::new(origin, direction)?))
    }

    pub(super) const fn tolerances(self) -> Tolerances {
        self.tolerances
    }

    /// Fit a model-space point to the first conic's requested periodic range.
    pub(super) fn fit_a(self, point: Point3) -> Option<f64> {
        self.fit_parameter_a(self.a.raw_parameter(point))
    }

    /// Fit a model-space point to the second conic's requested periodic range.
    pub(super) fn fit_b(self, point: Point3) -> Option<f64> {
        self.fit_parameter_b(self.b.raw_parameter(point))
    }

    /// Fit a raw parameter to the first conic's requested periodic range.
    pub(super) fn fit_parameter_a(self, candidate: f64) -> Option<f64> {
        fit_periodic_parameter(candidate, self.range_a, self.parameter_tol)
    }

    /// Fit a raw parameter to the second conic's requested periodic range.
    pub(super) fn fit_parameter_b(self, candidate: f64) -> Option<f64> {
        fit_periodic_parameter(candidate, self.range_b, self.parameter_tol)
    }

    /// Accept, classify, and first-wins deduplicate one model-space candidate.
    pub(super) fn push_point(
        self,
        point: Point3,
        fallback_kind: Option<ContactKind>,
        points: &mut Vec<CurveCurvePoint>,
    ) {
        let Some(t_a) = self.fit_a(point) else {
            return;
        };
        let Some(t_b) = self.fit_b(point) else {
            return;
        };
        self.push_parameters(t_a, t_b, fallback_kind, points);
    }

    /// Accept, classify, and first-wins deduplicate one paired parameter
    /// candidate. A supplied kind retains pair-specific tangent evidence.
    pub(super) fn push_parameters(
        self,
        t_a: f64,
        t_b: f64,
        fallback_kind: Option<ContactKind>,
        points: &mut Vec<CurveCurvePoint>,
    ) {
        let kind = fallback_kind.unwrap_or_else(|| self.contact_kind(t_a, t_b));
        if let Some(candidate) = accept_curve_curve_candidate(
            self.a.curve(),
            t_a,
            self.b.curve(),
            t_b,
            kind,
            self.tolerances,
        ) {
            push_distinct_conic_point(points, candidate, self.tolerances);
        }
    }

    fn contact_kind(self, t_a: f64, t_b: f64) -> ContactKind {
        let da = self.a.curve().eval_derivs(t_a, 1).d[1];
        let db = self.b.curve().eval_derivs(t_b, 1).d[1];
        let Some(ua) = da.normalized() else {
            return ContactKind::Singular;
        };
        let Some(ub) = db.normalized() else {
            return ContactKind::Singular;
        };
        if ua.cross(ub).norm() <= self.tolerances.angular() {
            ContactKind::Tangent
        } else {
            ContactKind::Transverse
        }
    }

    /// Construct the clipped periodic overlap or endpoint contact stream for
    /// a pair-specific exact affine parameter map `t_b = sign*t_a + offset`.
    pub(super) fn coincident(self, sign: f64, offset: f64) -> Result<CurveCurveIntersections> {
        let orientation = if sign > 0.0 {
            ParamOrientation::Same
        } else {
            ParamOrientation::Reversed
        };
        let period = core::f64::consts::TAU;
        let mapped_lo = if sign > 0.0 {
            self.range_a.lo + offset
        } else {
            offset - self.range_a.hi
        };
        let mapped_hi = if sign > 0.0 {
            self.range_a.hi + offset
        } else {
            offset - self.range_a.lo
        };
        let k_min = ((self.range_b.lo - mapped_hi - self.parameter_tol) / period).ceil() as i64;
        let k_max = ((self.range_b.hi - mapped_lo + self.parameter_tol) / period).floor() as i64;

        let mut overlaps = Vec::new();
        let mut point_parameters = Vec::new();
        for k in k_min..=k_max {
            let shift = k as f64 * period;
            let inverse = if sign > 0.0 {
                ParamRange::new(
                    self.range_b.lo - offset - shift,
                    self.range_b.hi - offset - shift,
                )
            } else {
                ParamRange::new(
                    offset + shift - self.range_b.hi,
                    offset + shift - self.range_b.lo,
                )
            };
            let lo = self.range_a.lo.max(inverse.lo);
            let hi = self.range_a.hi.min(inverse.hi);
            if hi < lo - self.parameter_tol {
                continue;
            }
            let lo = lo.clamp(self.range_a.lo, self.range_a.hi);
            let hi = hi.clamp(self.range_a.lo, self.range_a.hi);
            if hi - lo > self.parameter_tol {
                let b0 = sign * lo + offset + shift;
                let b1 = sign * hi + offset + shift;
                overlaps.push(CurveCurveOverlap {
                    a: ParamRange::new(lo, hi),
                    b: ParamRange::new(b0.min(b1), b0.max(b1)),
                    orientation,
                });
            } else {
                point_parameters.push(((lo + hi) / 2.0).clamp(self.range_a.lo, self.range_a.hi));
            }
        }

        let mut points = Vec::new();
        for t_a in point_parameters {
            if overlaps.iter().any(|overlap| {
                overlap.a.lo - self.parameter_tol <= t_a && t_a <= overlap.a.hi + self.parameter_tol
            }) {
                continue;
            }
            let raw_b = sign * t_a + offset;
            let Some(t_b) = fit_periodic_parameter(raw_b, self.range_b, self.parameter_tol) else {
                continue;
            };
            self.push_parameters(t_a, t_b, Some(ContactKind::Tangent), &mut points);
        }
        CurveCurveIntersections::canonicalized_complete(points, overlaps)
    }
}

fn push_distinct_conic_point(
    points: &mut Vec<CurveCurvePoint>,
    candidate: CurveCurvePoint,
    tolerances: Tolerances,
) {
    if !points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        points.push(candidate);
    }
}

/// Geometry-specific inputs for the shared bounded conic/cylinder pipeline.
///
/// Circle and ellipse adapters keep their original arithmetic expression order
/// in the few places where the parameterizations differ. The root, overlap,
/// window, candidate, ordering, and completion stages are shared below.
#[derive(Clone, Copy)]
pub(super) struct ConicCylinderConfig<'a> {
    conic: ConicCurve<'a>,
    curve_range: ParamRange,
    cylinder: &'a Cylinder,
    cylinder_range: [ParamRange; 2],
    local_center: Vec3,
    local_x: Vec3,
    local_y: Vec3,
    tolerances: Tolerances,
}

impl<'a> ConicCylinderConfig<'a> {
    pub(super) fn circle(
        circle: &'a Circle,
        curve_range: ParamRange,
        cylinder: &'a Cylinder,
        cylinder_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let curve_x = circle.frame().x();
        let curve_y = circle.frame().y();
        Self::new(
            ConicCurve::Circle(circle),
            curve_range,
            circle.frame().origin(),
            curve_x,
            curve_y,
            cylinder,
            cylinder_range,
            tolerances,
        )
    }

    pub(super) fn ellipse(
        ellipse: &'a Ellipse,
        curve_range: ParamRange,
        cylinder: &'a Cylinder,
        cylinder_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let curve_x = ellipse.frame().x();
        let curve_y = ellipse.frame().y();
        Self::new(
            ConicCurve::Ellipse(ellipse),
            curve_range,
            ellipse.frame().origin(),
            curve_x,
            curve_y,
            cylinder,
            cylinder_range,
            tolerances,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn new(
        conic: ConicCurve<'a>,
        curve_range: ParamRange,
        curve_origin: kgeom::vec::Point3,
        curve_x: Vec3,
        curve_y: Vec3,
        cylinder: &'a Cylinder,
        cylinder_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        Self {
            conic,
            curve_range,
            cylinder,
            cylinder_range,
            local_center: cylinder.frame().to_local(curve_origin),
            local_x: Vec3::new(
                curve_x.dot(cylinder.frame().x()),
                curve_x.dot(cylinder.frame().y()),
                curve_x.dot(cylinder.frame().z()),
            ),
            local_y: Vec3::new(
                curve_y.dot(cylinder.frame().x()),
                curve_y.dot(cylinder.frame().y()),
                curve_y.dot(cylinder.frame().z()),
            ),
            tolerances,
        }
    }

    fn curve(&self) -> &dyn Curve {
        match self.conic {
            ConicCurve::Circle(circle) => circle,
            ConicCurve::Ellipse(ellipse) => ellipse,
        }
    }

    fn local_point(&self, t_curve: f64) -> Vec3 {
        let (sin, cos) = math::sincos(t_curve);
        match self.conic {
            ConicCurve::Circle(circle) => {
                self.local_center + (self.local_x * cos + self.local_y * sin) * circle.radius()
            }
            ConicCurve::Ellipse(ellipse) => {
                self.local_center
                    + self.local_x * (ellipse.major_radius() * cos)
                    + self.local_y * (ellipse.minor_radius() * sin)
            }
        }
    }

    fn parameter_scale(&self) -> f64 {
        match self.conic {
            ConicCurve::Circle(circle) => circle.radius(),
            ConicCurve::Ellipse(ellipse) => ellipse.minor_radius(),
        }
    }

    fn radial_extent(&self) -> f64 {
        let center = (self.local_center.x * self.local_center.x
            + self.local_center.y * self.local_center.y)
            .sqrt();
        center
            + match self.conic {
                ConicCurve::Circle(circle) => circle.radius(),
                ConicCurve::Ellipse(ellipse) => ellipse.major_radius(),
            }
    }

    fn implicit_coefficients(&self) -> [f64; 5] {
        match self.conic {
            ConicCurve::Circle(circle) => {
                let radius = circle.radius();
                let c = [self.local_center.x, self.local_center.y];
                let x = [self.local_x.x, self.local_x.y];
                let y = [self.local_y.x, self.local_y.y];
                let c0 =
                    c[0] * c[0] + c[1] * c[1] - self.cylinder.radius() * self.cylinder.radius();
                let cos = 2.0 * radius * (c[0] * x[0] + c[1] * x[1]);
                let sin = 2.0 * radius * (c[0] * y[0] + c[1] * y[1]);
                let cos2 = radius * radius * (x[0] * x[0] + x[1] * x[1]);
                let sin2 = radius * radius * (y[0] * y[0] + y[1] * y[1]);
                let sin_cos = 2.0 * radius * radius * (x[0] * y[0] + x[1] * y[1]);

                [
                    c0 + cos + cos2,
                    2.0 * sin + 2.0 * sin_cos,
                    2.0 * c0 - 2.0 * cos2 + 4.0 * sin2,
                    2.0 * sin - 2.0 * sin_cos,
                    c0 - cos + cos2,
                ]
            }
            ConicCurve::Ellipse(ellipse) => {
                let a_vec = self.local_x * ellipse.major_radius();
                let b_vec = self.local_y * ellipse.minor_radius();
                let c = [self.local_center.x, self.local_center.y];
                let a = [a_vec.x, a_vec.y];
                let b = [b_vec.x, b_vec.y];
                let c0 =
                    c[0] * c[0] + c[1] * c[1] - self.cylinder.radius() * self.cylinder.radius();
                let cos = 2.0 * (c[0] * a[0] + c[1] * a[1]);
                let sin = 2.0 * (c[0] * b[0] + c[1] * b[1]);
                let cos2 = a[0] * a[0] + a[1] * a[1];
                let sin2 = b[0] * b[0] + b[1] * b[1];
                let sin_cos = 2.0 * (a[0] * b[0] + a[1] * b[1]);

                [
                    c0 + cos + cos2,
                    2.0 * sin + 2.0 * sin_cos,
                    2.0 * c0 - 2.0 * cos2 + 4.0 * sin2,
                    2.0 * sin - 2.0 * sin_cos,
                    c0 - cos + cos2,
                ]
            }
        }
    }

    fn window_z_coefficients(&self) -> (f64, f64) {
        match self.conic {
            ConicCurve::Circle(circle) => (
                self.local_x.z * circle.radius(),
                self.local_y.z * circle.radius(),
            ),
            ConicCurve::Ellipse(ellipse) => (
                self.local_x.z * ellipse.major_radius(),
                self.local_y.z * ellipse.minor_radius(),
            ),
        }
    }

    fn window_longitude_coefficients(&self, sin_u: f64, cos_u: f64) -> (f64, f64) {
        match self.conic {
            ConicCurve::Circle(circle) => {
                let radius = circle.radius();
                (
                    radius * (-sin_u * self.local_x.x + cos_u * self.local_x.y),
                    radius * (-sin_u * self.local_y.x + cos_u * self.local_y.y),
                )
            }
            ConicCurve::Ellipse(ellipse) => (
                ellipse.major_radius() * (-sin_u * self.local_x.x + cos_u * self.local_x.y),
                ellipse.minor_radius() * (-sin_u * self.local_y.x + cos_u * self.local_y.y),
            ),
        }
    }

    fn validation_reasons(&self) -> (&'static str, &'static str, &'static str) {
        match self.conic {
            ConicCurve::Circle(_) => (
                "circle/cylinder intersection requires a finite non-reversed curve range",
                "bounded circle range cannot span more than one period",
                "circle/cylinder intersection requires finite non-reversed surface ranges",
            ),
            ConicCurve::Ellipse(_) => (
                "ellipse/cylinder intersection requires a finite non-reversed curve range",
                "bounded ellipse range cannot span more than one period",
                "ellipse/cylinder intersection requires finite non-reversed surface ranges",
            ),
        }
    }

    fn validate(&self) -> Result<()> {
        let (curve_reason, period_reason, surface_reason) = self.validation_reasons();
        if !self.curve_range.is_finite() || self.curve_range.width() < 0.0 {
            return Err(Error::InvalidGeometry {
                reason: curve_reason,
            });
        }
        if self.curve_range.width()
            > core::f64::consts::TAU + parameter_tolerance(self.parameter_scale(), self.tolerances)
        {
            return Err(Error::InvalidGeometry {
                reason: period_reason,
            });
        }
        if self
            .cylinder_range
            .iter()
            .any(|range| !range.is_finite() || range.width() < 0.0)
        {
            return Err(Error::InvalidGeometry {
                reason: surface_reason,
            });
        }
        Ok(())
    }

    fn curve_parameter_tolerance(&self) -> f64 {
        parameter_tolerance(self.parameter_scale(), self.tolerances)
    }

    fn add_contact(&self, points: &mut Vec<CurveSurfacePoint>, t_curve: f64, force_tangent: bool) {
        let Some(t_curve) =
            fit_curve_parameter(t_curve, self.curve_range, self.curve_parameter_tolerance())
        else {
            return;
        };
        let local = self.local_point(t_curve);
        let Some(uv) = cylinder_uv(
            local,
            self.cylinder_range,
            self.cylinder.radius(),
            self.tolerances,
        ) else {
            return;
        };
        let kind = self.contact_kind(t_curve, uv, force_tangent);
        if let Some(point) = accept_curve_surface_candidate(
            self.curve(),
            t_curve,
            self.cylinder,
            uv,
            kind,
            self.tolerances,
        ) {
            push_distinct(&mut *points, point, self.tolerances);
        }
    }

    fn contact_kind(&self, t_curve: f64, uv: [f64; 2], force_tangent: bool) -> ContactKind {
        if force_tangent {
            return ContactKind::Tangent;
        }
        let Some(normal) = self.cylinder.normal(uv) else {
            return ContactKind::Singular;
        };
        let tangent = self.curve().eval_derivs(t_curve, 1).d[1];
        let Some(tangent) = tangent.normalized() else {
            return ContactKind::Singular;
        };
        if normal.dot(tangent).abs() <= self.tolerances.angular() {
            ContactKind::Tangent
        } else {
            ContactKind::Transverse
        }
    }
}

/// Run the common bounded circle/ellipse-by-cylinder intersection pipeline.
pub(super) fn intersect_bounded_conic_cylinder(
    config: ConicCylinderConfig<'_>,
) -> Result<CurveSurfaceIntersections> {
    config.validate()?;

    let coeffs = config.implicit_coefficients();
    let tolerance = implicit_tolerance(&config);
    if coeffs.iter().all(|coeff| coeff.abs() <= tolerance) {
        return contained_conic_cylinder(&config);
    }

    let mut points = Vec::new();
    for t_curve in implicit_roots(&coeffs, config.curve_range, tolerance) {
        config.add_contact(&mut points, t_curve, false);
    }
    for t_curve in implicit_roots(
        &polynomial_derivative(&coeffs),
        config.curve_range,
        tolerance,
    ) {
        config.add_contact(&mut points, t_curve, true);
    }
    if implicit_value(&config, core::f64::consts::PI).abs() <= tolerance {
        config.add_contact(&mut points, core::f64::consts::PI, true);
    }

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn contained_conic_cylinder(config: &ConicCylinderConfig<'_>) -> Result<CurveSurfaceIntersections> {
    let t_tol = config.curve_parameter_tolerance();
    if config.curve_range.width() <= t_tol {
        let mut points = Vec::new();
        config.add_contact(&mut points, config.curve_range.lo, true);
        return CurveSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    let mut cuts = vec![config.curve_range.lo, config.curve_range.hi];
    if !push_cylinder_window_cuts(config, &mut cuts) {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            HARMONIC_ROOT_CLASSIFICATION_REASON,
        ));
    }
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
        if cylinder_uv(
            config.local_point(mid),
            config.cylinder_range,
            config.cylinder.radius(),
            config.tolerances,
        )
        .is_none()
        {
            continue;
        }
        let Some(uv_start) = cylinder_uv(
            config.local_point(lo),
            config.cylinder_range,
            config.cylinder.radius(),
            config.tolerances,
        ) else {
            continue;
        };
        let Some(uv_end) = cylinder_uv(
            config.local_point(hi),
            config.cylinder_range,
            config.cylinder.radius(),
            config.tolerances,
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
        let cut_point = config.curve().eval(cut);
        if overlaps.iter().any(|overlap| {
            (cut >= overlap.curve.lo - t_tol && cut <= overlap.curve.hi + t_tol)
                || cut_point.dist(config.curve().eval(overlap.curve.lo))
                    <= config.tolerances.linear()
                || cut_point.dist(config.curve().eval(overlap.curve.hi))
                    <= config.tolerances.linear()
        }) {
            continue;
        }
        config.add_contact(&mut points, cut, true);
    }

    CurveSurfaceIntersections::canonicalized_complete(points, overlaps)
}

fn push_cylinder_window_cuts(config: &ConicCylinderConfig<'_>, cuts: &mut Vec<f64>) -> bool {
    let z_c = config.local_center.z;
    let (z_a, z_b) = config.window_z_coefficients();
    for v_bound in [config.cylinder_range[1].lo, config.cylinder_range[1].hi] {
        let Some(roots) = trig_linear_roots(
            z_a,
            z_b,
            z_c - v_bound,
            config.curve_range,
            config.tolerances.linear(),
        ) else {
            return false;
        };
        for (root, _) in roots {
            push_scalar(cuts, root, config.curve_parameter_tolerance());
        }
    }

    for u_bound in [config.cylinder_range[0].lo, config.cylinder_range[0].hi] {
        let (sin_u, cos_u) = math::sincos(u_bound);
        let c = -sin_u * config.local_center.x + cos_u * config.local_center.y;
        let (a, b) = config.window_longitude_coefficients(sin_u, cos_u);
        let Some(roots) =
            trig_linear_roots(a, b, c, config.curve_range, config.tolerances.linear())
        else {
            return false;
        };
        for (root, _) in roots {
            if !longitude_matches_bound(config.local_point(root), u_bound, config.tolerances) {
                continue;
            }
            push_scalar(cuts, root, config.curve_parameter_tolerance());
        }
    }
    true
}

fn implicit_value(config: &ConicCylinderConfig<'_>, t_curve: f64) -> f64 {
    let local = config.local_point(t_curve);
    local.x * local.x + local.y * local.y - config.cylinder.radius() * config.cylinder.radius()
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

fn cylinder_uv(
    local: Vec3,
    cylinder_range: [ParamRange; 2],
    radius: f64,
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let raw_u = math::atan2(local.y, local.x);
    let u = fit_periodic_parameter(
        raw_u,
        cylinder_range[0],
        parameter_tolerance(radius, tolerances),
    )?;
    let v = fit_scalar_parameter(local.z, cylinder_range[1], tolerances.linear())?;
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

fn implicit_tolerance(config: &ConicCylinderConfig<'_>) -> f64 {
    let scale = (config.radial_extent() + config.cylinder.radius()).max(1.0);
    config.tolerances.linear() * scale
}

/// Geometry-specific inputs for the shared bounded conic/cone pipeline.
///
/// The variant adapter retains the original circle and ellipse expression
/// order, while the proof and result pipeline below is curve-class agnostic.
#[derive(Clone, Copy)]
pub(super) struct ConicConeConfig<'a> {
    conic: ConicCurve<'a>,
    curve_range: ParamRange,
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

impl<'a> ConicConeConfig<'a> {
    pub(super) fn circle(
        circle: &'a Circle,
        curve_range: ParamRange,
        cone: &'a Cone,
        cone_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let curve_x = circle.frame().x();
        let curve_y = circle.frame().y();
        Self::new(
            ConicCurve::Circle(circle),
            curve_range,
            circle.frame().origin(),
            curve_x,
            curve_y,
            cone,
            cone_range,
            tolerances,
        )
    }

    pub(super) fn ellipse(
        ellipse: &'a Ellipse,
        curve_range: ParamRange,
        cone: &'a Cone,
        cone_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let curve_x = ellipse.frame().x();
        let curve_y = ellipse.frame().y();
        Self::new(
            ConicCurve::Ellipse(ellipse),
            curve_range,
            ellipse.frame().origin(),
            curve_x,
            curve_y,
            cone,
            cone_range,
            tolerances,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn new(
        conic: ConicCurve<'a>,
        curve_range: ParamRange,
        curve_origin: kgeom::vec::Point3,
        curve_x: Vec3,
        curve_y: Vec3,
        cone: &'a Cone,
        cone_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let (sin_a, cos_a) = math::sincos(cone.half_angle());
        Self {
            conic,
            curve_range,
            cone,
            cone_range,
            local_center: cone.frame().to_local(curve_origin),
            local_x: Vec3::new(
                curve_x.dot(cone.frame().x()),
                curve_x.dot(cone.frame().y()),
                curve_x.dot(cone.frame().z()),
            ),
            local_y: Vec3::new(
                curve_y.dot(cone.frame().x()),
                curve_y.dot(cone.frame().y()),
                curve_y.dot(cone.frame().z()),
            ),
            sin_a,
            cos_a,
            tan_a: sin_a / cos_a,
            tolerances,
        }
    }

    fn curve(&self) -> &dyn Curve {
        match self.conic {
            ConicCurve::Circle(circle) => circle,
            ConicCurve::Ellipse(ellipse) => ellipse,
        }
    }

    fn local_point(&self, t_curve: f64) -> Vec3 {
        let (sin, cos) = math::sincos(t_curve);
        match self.conic {
            ConicCurve::Circle(circle) => {
                self.local_center + (self.local_x * cos + self.local_y * sin) * circle.radius()
            }
            ConicCurve::Ellipse(ellipse) => {
                self.local_center
                    + self.local_x * (ellipse.major_radius() * cos)
                    + self.local_y * (ellipse.minor_radius() * sin)
            }
        }
    }

    fn parameter_scale(&self) -> f64 {
        match self.conic {
            ConicCurve::Circle(circle) => circle.radius(),
            ConicCurve::Ellipse(ellipse) => ellipse.minor_radius(),
        }
    }

    fn local_extent(&self) -> f64 {
        self.local_center.norm()
            + match self.conic {
                ConicCurve::Circle(circle) => circle.radius(),
                ConicCurve::Ellipse(ellipse) => ellipse.major_radius(),
            }
    }

    fn window_z_coefficients(&self) -> (f64, f64) {
        match self.conic {
            ConicCurve::Circle(circle) => (
                self.local_x.z * circle.radius(),
                self.local_y.z * circle.radius(),
            ),
            ConicCurve::Ellipse(ellipse) => (
                self.local_x.z * ellipse.major_radius(),
                self.local_y.z * ellipse.minor_radius(),
            ),
        }
    }

    fn window_longitude_coefficients(&self, sin_u: f64, cos_u: f64) -> (f64, f64) {
        match self.conic {
            ConicCurve::Circle(circle) => {
                let radius = circle.radius();
                (
                    radius * (-sin_u * self.local_x.x + cos_u * self.local_x.y),
                    radius * (-sin_u * self.local_y.x + cos_u * self.local_y.y),
                )
            }
            ConicCurve::Ellipse(ellipse) => (
                ellipse.major_radius() * (-sin_u * self.local_x.x + cos_u * self.local_x.y),
                ellipse.minor_radius() * (-sin_u * self.local_y.x + cos_u * self.local_y.y),
            ),
        }
    }

    fn implicit_coefficients(&self) -> [f64; 5] {
        match self.conic {
            ConicCurve::Circle(circle) => {
                let radius = circle.radius();
                let c = [self.local_center.x, self.local_center.y];
                let x = [self.local_x.x, self.local_x.y];
                let y = [self.local_y.x, self.local_y.y];
                let q_c = self.cone.radius() + self.local_center.z * self.tan_a;
                let q_x = radius * self.local_x.z * self.tan_a;
                let q_y = radius * self.local_y.z * self.tan_a;
                let c0 = c[0] * c[0] + c[1] * c[1] - q_c * q_c;
                let cos = 2.0 * radius * (c[0] * x[0] + c[1] * x[1]) - 2.0 * q_c * q_x;
                let sin = 2.0 * radius * (c[0] * y[0] + c[1] * y[1]) - 2.0 * q_c * q_y;
                let cos2 = radius * radius * (x[0] * x[0] + x[1] * x[1]) - q_x * q_x;
                let sin2 = radius * radius * (y[0] * y[0] + y[1] * y[1]) - q_y * q_y;
                let sin_cos = 2.0 * radius * radius * (x[0] * y[0] + x[1] * y[1]) - 2.0 * q_x * q_y;

                [
                    c0 + cos + cos2,
                    2.0 * sin + 2.0 * sin_cos,
                    2.0 * c0 - 2.0 * cos2 + 4.0 * sin2,
                    2.0 * sin - 2.0 * sin_cos,
                    c0 - cos + cos2,
                ]
            }
            ConicCurve::Ellipse(ellipse) => {
                let a_vec = self.local_x * ellipse.major_radius();
                let b_vec = self.local_y * ellipse.minor_radius();
                let c = [self.local_center.x, self.local_center.y];
                let a = [a_vec.x, a_vec.y];
                let b = [b_vec.x, b_vec.y];
                let q_c = self.cone.radius() + self.local_center.z * self.tan_a;
                let q_a = a_vec.z * self.tan_a;
                let q_b = b_vec.z * self.tan_a;
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
        }
    }

    fn validation_reasons(&self) -> (&'static str, &'static str, &'static str) {
        match self.conic {
            ConicCurve::Circle(_) => (
                "circle/cone intersection requires a finite non-reversed curve range",
                "bounded circle range cannot span more than one period",
                "circle/cone intersection requires finite non-reversed surface ranges",
            ),
            ConicCurve::Ellipse(_) => (
                "ellipse/cone intersection requires a finite non-reversed curve range",
                "bounded ellipse range cannot span more than one period",
                "ellipse/cone intersection requires finite non-reversed surface ranges",
            ),
        }
    }

    fn validate(&self) -> Result<()> {
        let (curve_reason, period_reason, surface_reason) = self.validation_reasons();
        if !self.curve_range.is_finite() || self.curve_range.width() < 0.0 {
            return Err(Error::InvalidGeometry {
                reason: curve_reason,
            });
        }
        if self.curve_range.width()
            > core::f64::consts::TAU + parameter_tolerance(self.parameter_scale(), self.tolerances)
        {
            return Err(Error::InvalidGeometry {
                reason: period_reason,
            });
        }
        if self
            .cone_range
            .iter()
            .any(|range| !range.is_finite() || range.width() < 0.0)
        {
            return Err(Error::InvalidGeometry {
                reason: surface_reason,
            });
        }
        Ok(())
    }

    fn curve_parameter_tolerance(&self) -> f64 {
        parameter_tolerance(self.parameter_scale(), self.tolerances)
    }

    fn add_contact(&self, points: &mut Vec<CurveSurfacePoint>, t_curve: f64, force_tangent: bool) {
        let Some(t_curve) =
            fit_curve_parameter(t_curve, self.curve_range, self.curve_parameter_tolerance())
        else {
            return;
        };
        let local = self.local_point(t_curve);
        let Some(uv) = cone_uv(local, self.cone, self.cone_range, self.tolerances) else {
            return;
        };
        let kind = self.contact_kind(t_curve, uv, force_tangent);
        if let Some(point) = accept_curve_surface_candidate(
            self.curve(),
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
        let tangent = self.curve().eval_derivs(t_curve, 1).d[1];
        let Some(tangent) = tangent.normalized() else {
            return ContactKind::Singular;
        };
        if normal.dot(tangent).abs() <= self.tolerances.angular() {
            ContactKind::Tangent
        } else {
            ContactKind::Transverse
        }
    }
}

/// Run the common bounded circle/ellipse-by-cone intersection pipeline.
pub(super) fn intersect_bounded_conic_cone(
    config: ConicConeConfig<'_>,
) -> Result<CurveSurfaceIntersections> {
    config.validate()?;

    let coeffs = config.implicit_coefficients();
    let tolerance = cone_implicit_tolerance(&config);
    if coeffs.iter().all(|coeff| coeff.abs() <= tolerance) {
        return contained_conic_cone(&config);
    }

    let mut points = Vec::new();
    for t_curve in implicit_roots(&coeffs, config.curve_range, tolerance) {
        config.add_contact(&mut points, t_curve, false);
    }
    for t_curve in implicit_roots(
        &polynomial_derivative(&coeffs),
        config.curve_range,
        tolerance,
    ) {
        config.add_contact(&mut points, t_curve, true);
    }
    if cone_implicit_value(&config, core::f64::consts::PI).abs() <= tolerance {
        config.add_contact(&mut points, core::f64::consts::PI, true);
    }

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn contained_conic_cone(config: &ConicConeConfig<'_>) -> Result<CurveSurfaceIntersections> {
    let t_tol = config.curve_parameter_tolerance();
    if config.curve_range.width() <= t_tol {
        let mut points = Vec::new();
        config.add_contact(&mut points, config.curve_range.lo, true);
        return CurveSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    let mut cuts = vec![config.curve_range.lo, config.curve_range.hi];
    if !push_cone_window_cuts(config, &mut cuts) {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            HARMONIC_ROOT_CLASSIFICATION_REASON,
        ));
    }
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
            config.local_point(mid),
            config.cone,
            config.cone_range,
            config.tolerances,
        )
        .is_none()
        {
            continue;
        }
        let Some(uv_start) = cone_uv(
            config.local_point(lo),
            config.cone,
            config.cone_range,
            config.tolerances,
        ) else {
            continue;
        };
        let Some(uv_end) = cone_uv(
            config.local_point(hi),
            config.cone,
            config.cone_range,
            config.tolerances,
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
        let cut_point = config.curve().eval(cut);
        if overlaps.iter().any(|overlap| {
            (cut >= overlap.curve.lo - t_tol && cut <= overlap.curve.hi + t_tol)
                || cut_point.dist(config.curve().eval(overlap.curve.lo))
                    <= config.tolerances.linear()
                || cut_point.dist(config.curve().eval(overlap.curve.hi))
                    <= config.tolerances.linear()
        }) {
            continue;
        }
        config.add_contact(&mut points, cut, true);
    }

    CurveSurfaceIntersections::canonicalized_complete(points, overlaps)
}

fn push_cone_window_cuts(config: &ConicConeConfig<'_>, cuts: &mut Vec<f64>) -> bool {
    let z_c = config.local_center.z;
    let (z_a, z_b) = config.window_z_coefficients();
    for v_bound in [config.cone_range[1].lo, config.cone_range[1].hi] {
        let z_bound = v_bound * config.cos_a;
        let Some(roots) = trig_linear_roots(
            z_a,
            z_b,
            z_c - z_bound,
            config.curve_range,
            config.tolerances.linear(),
        ) else {
            return false;
        };
        for (root, _) in roots {
            push_scalar(cuts, root, config.curve_parameter_tolerance());
        }
    }

    for u_bound in [config.cone_range[0].lo, config.cone_range[0].hi] {
        let (sin_u, cos_u) = math::sincos(u_bound);
        let c = -sin_u * config.local_center.x + cos_u * config.local_center.y;
        let (a, b) = config.window_longitude_coefficients(sin_u, cos_u);
        let Some(roots) =
            trig_linear_roots(a, b, c, config.curve_range, config.tolerances.linear())
        else {
            return false;
        };
        for (root, _) in roots {
            if !cone_longitude_matches_bound(config, config.local_point(root), u_bound) {
                continue;
            }
            push_scalar(cuts, root, config.curve_parameter_tolerance());
        }
    }
    true
}

fn cone_implicit_value(config: &ConicConeConfig<'_>, t_curve: f64) -> f64 {
    let local = config.local_point(t_curve);
    let q = config.cone.radius() + local.z * config.tan_a;
    local.x * local.x + local.y * local.y - q * q
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

fn cone_longitude_matches_bound(config: &ConicConeConfig<'_>, local: Vec3, bound: f64) -> bool {
    let v = local.z / config.cos_a;
    let signed_radius = config.cone.radius() + v * config.sin_a;
    if signed_radius.abs() <= config.tolerances.linear() {
        return true;
    }
    let raw_u = math::atan2(local.y / signed_radius, local.x / signed_radius);
    fit_periodic_parameter(
        raw_u,
        ParamRange::new(bound, bound),
        parameter_tolerance(signed_radius.abs(), config.tolerances),
    )
    .is_some()
}

fn cone_implicit_tolerance(config: &ConicConeConfig<'_>) -> f64 {
    let scale = (config.local_extent() + config.cone.radius()).max(1.0);
    config.tolerances.linear() * scale
}

/// Geometry-specific inputs for the shared bounded conic/torus pipeline.
///
/// Circle quartics and general ellipse octics retain their original arithmetic
/// construction behind the variant adapter. Root processing and bounded result
/// construction are shared.
#[derive(Clone, Copy)]
pub(super) struct ConicTorusConfig<'a> {
    conic: ConicCurve<'a>,
    curve_range: ParamRange,
    torus: &'a Torus,
    torus_range: [ParamRange; 2],
    local_center: Vec3,
    local_x: Vec3,
    local_y: Vec3,
    tolerances: Tolerances,
}

impl<'a> ConicTorusConfig<'a> {
    pub(super) fn circle(
        circle: &'a Circle,
        curve_range: ParamRange,
        torus: &'a Torus,
        torus_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let curve_x = circle.frame().x();
        let curve_y = circle.frame().y();
        Self::new(
            ConicCurve::Circle(circle),
            curve_range,
            circle.frame().origin(),
            curve_x,
            curve_y,
            torus,
            torus_range,
            tolerances,
        )
    }

    pub(super) fn ellipse(
        ellipse: &'a Ellipse,
        curve_range: ParamRange,
        torus: &'a Torus,
        torus_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let curve_x = ellipse.frame().x();
        let curve_y = ellipse.frame().y();
        Self::new(
            ConicCurve::Ellipse(ellipse),
            curve_range,
            ellipse.frame().origin(),
            curve_x,
            curve_y,
            torus,
            torus_range,
            tolerances,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn new(
        conic: ConicCurve<'a>,
        curve_range: ParamRange,
        curve_origin: kgeom::vec::Point3,
        curve_x: Vec3,
        curve_y: Vec3,
        torus: &'a Torus,
        torus_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        Self {
            conic,
            curve_range,
            torus,
            torus_range,
            local_center: torus.frame().to_local(curve_origin),
            local_x: Vec3::new(
                curve_x.dot(torus.frame().x()),
                curve_x.dot(torus.frame().y()),
                curve_x.dot(torus.frame().z()),
            ),
            local_y: Vec3::new(
                curve_y.dot(torus.frame().x()),
                curve_y.dot(torus.frame().y()),
                curve_y.dot(torus.frame().z()),
            ),
            tolerances,
        }
    }

    fn curve(&self) -> &dyn Curve {
        match self.conic {
            ConicCurve::Circle(circle) => circle,
            ConicCurve::Ellipse(ellipse) => ellipse,
        }
    }

    fn local_point(&self, t_curve: f64) -> Vec3 {
        let (sin, cos) = math::sincos(t_curve);
        match self.conic {
            ConicCurve::Circle(circle) => {
                self.local_center + (self.local_x * cos + self.local_y * sin) * circle.radius()
            }
            ConicCurve::Ellipse(ellipse) => {
                self.local_center
                    + self.local_x * (ellipse.major_radius() * cos)
                    + self.local_y * (ellipse.minor_radius() * sin)
            }
        }
    }

    fn parameter_scale(&self) -> f64 {
        match self.conic {
            ConicCurve::Circle(circle) => circle.radius(),
            ConicCurve::Ellipse(ellipse) => ellipse.minor_radius(),
        }
    }

    fn local_extent(&self) -> f64 {
        self.local_center.norm()
            + match self.conic {
                ConicCurve::Circle(circle) => circle.radius(),
                ConicCurve::Ellipse(ellipse) => ellipse.major_radius(),
            }
    }

    fn window_z_coefficients(&self) -> (f64, f64) {
        match self.conic {
            ConicCurve::Circle(circle) => (
                self.local_x.z * circle.radius(),
                self.local_y.z * circle.radius(),
            ),
            ConicCurve::Ellipse(ellipse) => (
                self.local_x.z * ellipse.major_radius(),
                self.local_y.z * ellipse.minor_radius(),
            ),
        }
    }

    fn window_longitude_coefficients(&self, sin_u: f64, cos_u: f64) -> (f64, f64) {
        match self.conic {
            ConicCurve::Circle(circle) => {
                let radius = circle.radius();
                (
                    radius * (-sin_u * self.local_x.x + cos_u * self.local_x.y),
                    radius * (-sin_u * self.local_y.x + cos_u * self.local_y.y),
                )
            }
            ConicCurve::Ellipse(ellipse) => (
                ellipse.major_radius() * (-sin_u * self.local_x.x + cos_u * self.local_x.y),
                ellipse.minor_radius() * (-sin_u * self.local_y.x + cos_u * self.local_y.y),
            ),
        }
    }

    fn implicit_coefficients(&self) -> Vec<f64> {
        match self.conic {
            ConicCurve::Circle(circle) => {
                let radius = circle.radius();
                let c = self.local_center;
                let x = self.local_x;
                let y = self.local_y;
                let major_sq = self.torus.major_radius() * self.torus.major_radius();
                let h0 = c.dot(c) + radius * radius + major_sq
                    - self.torus.minor_radius() * self.torus.minor_radius();
                let h_cos = 2.0 * radius * c.dot(x);
                let h_sin = 2.0 * radius * c.dot(y);

                let q0 = c.x * c.x + c.y * c.y;
                let q_cos = 2.0 * radius * (c.x * x.x + c.y * x.y);
                let q_sin = 2.0 * radius * (c.x * y.x + c.y * y.y);
                let q_cos2 = radius * radius * (x.x * x.x + x.y * x.y);
                let q_sin2 = radius * radius * (y.x * y.x + y.y * y.y);
                let q_sin_cos = 2.0 * radius * radius * (x.x * y.x + x.y * y.y);
                let q_coeffs = torus_trig_quadratic_half_angle_coefficients(
                    q0, q_cos, q_sin, q_cos2, q_sin2, q_sin_cos,
                );
                let h_coeffs = [h0 + h_cos, 2.0 * h_sin, h0 - h_cos];
                let h_sq = [
                    h_coeffs[0] * h_coeffs[0],
                    2.0 * h_coeffs[0] * h_coeffs[1],
                    h_coeffs[1] * h_coeffs[1] + 2.0 * h_coeffs[0] * h_coeffs[2],
                    2.0 * h_coeffs[1] * h_coeffs[2],
                    h_coeffs[2] * h_coeffs[2],
                ];

                vec![
                    h_sq[0] - 4.0 * major_sq * q_coeffs[0],
                    h_sq[1] - 4.0 * major_sq * q_coeffs[1],
                    h_sq[2] - 4.0 * major_sq * q_coeffs[2],
                    h_sq[3] - 4.0 * major_sq * q_coeffs[3],
                    h_sq[4] - 4.0 * major_sq * q_coeffs[4],
                ]
            }
            ConicCurve::Ellipse(ellipse) => {
                let a_vec = self.local_x * ellipse.major_radius();
                let b_vec = self.local_y * ellipse.minor_radius();
                let c = self.local_center;
                let major_sq = self.torus.major_radius() * self.torus.major_radius();
                let h0 =
                    c.dot(c) + major_sq - self.torus.minor_radius() * self.torus.minor_radius();
                let h_cos = 2.0 * c.dot(a_vec);
                let h_sin = 2.0 * c.dot(b_vec);
                let h_cos2 = a_vec.dot(a_vec);
                let h_sin2 = b_vec.dot(b_vec);
                let h_sin_cos = 2.0 * a_vec.dot(b_vec);
                let h_coeffs = torus_trig_quadratic_half_angle_coefficients(
                    h0, h_cos, h_sin, h_cos2, h_sin2, h_sin_cos,
                );

                let q0 = c.x * c.x + c.y * c.y;
                let q_cos = 2.0 * (c.x * a_vec.x + c.y * a_vec.y);
                let q_sin = 2.0 * (c.x * b_vec.x + c.y * b_vec.y);
                let q_cos2 = a_vec.x * a_vec.x + a_vec.y * a_vec.y;
                let q_sin2 = b_vec.x * b_vec.x + b_vec.y * b_vec.y;
                let q_sin_cos = 2.0 * (a_vec.x * b_vec.x + a_vec.y * b_vec.y);
                let q_coeffs = torus_trig_quadratic_half_angle_coefficients(
                    q0, q_cos, q_sin, q_cos2, q_sin2, q_sin_cos,
                );

                let h_sq = torus_poly_square(&h_coeffs);
                let q_with_denominator = torus_poly_mul(&q_coeffs, &[1.0, 0.0, 2.0, 0.0, 1.0]);
                h_sq.iter()
                    .zip(q_with_denominator.iter())
                    .map(|(h, q)| h - 4.0 * major_sq * q)
                    .collect()
            }
        }
    }

    fn validation_reasons(&self) -> (&'static str, &'static str, &'static str) {
        match self.conic {
            ConicCurve::Circle(_) => (
                "circle/torus intersection requires a finite non-reversed curve range",
                "bounded circle range cannot span more than one period",
                "circle/torus intersection requires finite non-reversed surface ranges",
            ),
            ConicCurve::Ellipse(_) => (
                "ellipse/torus intersection requires a finite non-reversed curve range",
                "bounded ellipse range cannot span more than one period",
                "ellipse/torus intersection requires finite non-reversed surface ranges",
            ),
        }
    }

    fn validate(&self) -> Result<()> {
        let (curve_reason, period_reason, surface_reason) = self.validation_reasons();
        if !self.curve_range.is_finite() || self.curve_range.width() < 0.0 {
            return Err(Error::InvalidGeometry {
                reason: curve_reason,
            });
        }
        if self.curve_range.width()
            > core::f64::consts::TAU + parameter_tolerance(self.parameter_scale(), self.tolerances)
        {
            return Err(Error::InvalidGeometry {
                reason: period_reason,
            });
        }
        if self
            .torus_range
            .iter()
            .any(|range| !range.is_finite() || range.width() < 0.0)
        {
            return Err(Error::InvalidGeometry {
                reason: surface_reason,
            });
        }
        Ok(())
    }

    fn curve_parameter_tolerance(&self) -> f64 {
        parameter_tolerance(self.parameter_scale(), self.tolerances)
    }

    fn add_contact(&self, points: &mut Vec<CurveSurfacePoint>, t_curve: f64, force_tangent: bool) {
        let Some(t_curve) =
            fit_curve_parameter(t_curve, self.curve_range, self.curve_parameter_tolerance())
        else {
            return;
        };
        let local = self.local_point(t_curve);
        let Some(uv) = torus_uv(local, self.torus, self.torus_range, self.tolerances) else {
            return;
        };
        let kind = self.contact_kind(t_curve, uv, force_tangent);
        if let Some(point) = accept_curve_surface_candidate(
            self.curve(),
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
        let tangent = self.curve().eval_derivs(t_curve, 1).d[1];
        let Some(tangent) = tangent.normalized() else {
            return ContactKind::Singular;
        };
        if normal.dot(tangent).abs() <= self.tolerances.angular() {
            ContactKind::Tangent
        } else {
            ContactKind::Transverse
        }
    }
}

/// Run the common bounded circle/ellipse-by-torus intersection pipeline.
pub(super) fn intersect_bounded_conic_torus(
    config: ConicTorusConfig<'_>,
) -> Result<CurveSurfaceIntersections> {
    config.validate()?;

    let coeffs = config.implicit_coefficients();
    let tolerance = torus_implicit_tolerance(&config);
    if coeffs.iter().all(|coeff| coeff.abs() <= tolerance) {
        return contained_conic_torus(&config);
    }

    let mut points = Vec::new();
    for t_curve in implicit_roots(&coeffs, config.curve_range, tolerance) {
        config.add_contact(&mut points, t_curve, false);
    }
    for t_curve in implicit_roots(
        &polynomial_derivative(&coeffs),
        config.curve_range,
        tolerance,
    ) {
        config.add_contact(&mut points, t_curve, true);
    }
    if torus_implicit_value(&config, core::f64::consts::PI).abs() <= tolerance {
        config.add_contact(&mut points, core::f64::consts::PI, true);
    }

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn contained_conic_torus(config: &ConicTorusConfig<'_>) -> Result<CurveSurfaceIntersections> {
    let t_tol = config.curve_parameter_tolerance();
    if config.curve_range.width() <= t_tol {
        let mut points = Vec::new();
        config.add_contact(&mut points, config.curve_range.lo, true);
        return CurveSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    let mut cuts = vec![config.curve_range.lo, config.curve_range.hi];
    if !push_torus_window_cuts(config, &mut cuts) {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            HARMONIC_ROOT_CLASSIFICATION_REASON,
        ));
    }
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
            config.local_point(mid),
            config.torus,
            config.torus_range,
            config.tolerances,
        )
        .is_none()
        {
            continue;
        }
        let Some(uv_start) = torus_uv(
            config.local_point(lo),
            config.torus,
            config.torus_range,
            config.tolerances,
        ) else {
            continue;
        };
        let Some(uv_end) = torus_uv(
            config.local_point(hi),
            config.torus,
            config.torus_range,
            config.tolerances,
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
        let cut_point = config.curve().eval(cut);
        if overlaps.iter().any(|overlap| {
            (cut >= overlap.curve.lo - t_tol && cut <= overlap.curve.hi + t_tol)
                || cut_point.dist(config.curve().eval(overlap.curve.lo))
                    <= config.tolerances.linear()
                || cut_point.dist(config.curve().eval(overlap.curve.hi))
                    <= config.tolerances.linear()
        }) {
            continue;
        }
        config.add_contact(&mut points, cut, true);
    }

    CurveSurfaceIntersections::canonicalized_complete(points, overlaps)
}

fn push_torus_window_cuts(config: &ConicTorusConfig<'_>, cuts: &mut Vec<f64>) -> bool {
    let z_c = config.local_center.z;
    let (z_a, z_b) = config.window_z_coefficients();
    for v_bound in [config.torus_range[1].lo, config.torus_range[1].hi] {
        let z_bound = config.torus.minor_radius() * math::sin(v_bound);
        let Some(roots) = trig_linear_roots(
            z_a,
            z_b,
            z_c - z_bound,
            config.curve_range,
            config.tolerances.linear(),
        ) else {
            return false;
        };
        for (root, _) in roots {
            if !torus_tube_angle_matches_bound(config, config.local_point(root), v_bound) {
                continue;
            }
            push_scalar(cuts, root, config.curve_parameter_tolerance());
        }
    }

    for u_bound in [config.torus_range[0].lo, config.torus_range[0].hi] {
        let (sin_u, cos_u) = math::sincos(u_bound);
        let c = -sin_u * config.local_center.x + cos_u * config.local_center.y;
        let (a, b) = config.window_longitude_coefficients(sin_u, cos_u);
        let Some(roots) =
            trig_linear_roots(a, b, c, config.curve_range, config.tolerances.linear())
        else {
            return false;
        };
        for (root, _) in roots {
            if !torus_longitude_matches_bound(config, config.local_point(root), u_bound) {
                continue;
            }
            push_scalar(cuts, root, config.curve_parameter_tolerance());
        }
    }
    true
}

fn torus_trig_quadratic_half_angle_coefficients(
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

fn torus_poly_square(coeffs: &[f64]) -> Vec<f64> {
    torus_poly_mul(coeffs, coeffs)
}

fn torus_poly_mul(a: &[f64], b: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; a.len() + b.len() - 1];
    for (i, a_coeff) in a.iter().enumerate() {
        for (j, b_coeff) in b.iter().enumerate() {
            out[i + j] += a_coeff * b_coeff;
        }
    }
    out
}

fn torus_implicit_value(config: &ConicTorusConfig<'_>, t_curve: f64) -> f64 {
    let local = config.local_point(t_curve);
    let s = local.dot(local);
    let q = local.x * local.x + local.y * local.y;
    let h = s + config.torus.major_radius() * config.torus.major_radius()
        - config.torus.minor_radius() * config.torus.minor_radius();
    h * h - 4.0 * config.torus.major_radius() * config.torus.major_radius() * q
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

fn torus_longitude_matches_bound(config: &ConicTorusConfig<'_>, local: Vec3, bound: f64) -> bool {
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    if xy <= config.tolerances.linear() {
        return true;
    }
    let raw_u = math::atan2(local.y, local.x);
    fit_periodic_parameter(
        raw_u,
        ParamRange::new(bound, bound),
        parameter_tolerance(xy, config.tolerances),
    )
    .is_some()
}

fn torus_tube_angle_matches_bound(config: &ConicTorusConfig<'_>, local: Vec3, bound: f64) -> bool {
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let raw_v = math::atan2(local.z, xy - config.torus.major_radius());
    fit_periodic_parameter(
        raw_v,
        ParamRange::new(bound, bound),
        parameter_tolerance(config.torus.minor_radius(), config.tolerances),
    )
    .is_some()
}

fn torus_implicit_tolerance(config: &ConicTorusConfig<'_>) -> f64 {
    let scale = (config.local_extent() + config.torus.major_radius() + config.torus.minor_radius())
        .max(1.0);
    config.tolerances.linear() * scale * scale * scale
}

/// Geometry-specific inputs for the shared bounded conic/sphere pipeline.
///
/// The circle keeps its reduced linear-trigonometric solve, while the ellipse
/// keeps its polynomial plus derivative-root solve. Both strategies feed the
/// same bounded candidate, window, overlap, ordering, and completion stages.
#[derive(Clone, Copy)]
pub(super) struct ConicSphereConfig<'a> {
    conic: ConicCurve<'a>,
    curve_range: ParamRange,
    sphere: &'a Sphere,
    sphere_range: [ParamRange; 2],
    local_center: Vec3,
    local_x: Vec3,
    local_y: Vec3,
    tolerances: Tolerances,
}

impl<'a> ConicSphereConfig<'a> {
    pub(super) fn circle(
        circle: &'a Circle,
        curve_range: ParamRange,
        sphere: &'a Sphere,
        sphere_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let curve_x = circle.frame().x();
        let curve_y = circle.frame().y();
        Self::new(
            ConicCurve::Circle(circle),
            curve_range,
            circle.frame().origin(),
            curve_x,
            curve_y,
            sphere,
            sphere_range,
            tolerances,
        )
    }

    pub(super) fn ellipse(
        ellipse: &'a Ellipse,
        curve_range: ParamRange,
        sphere: &'a Sphere,
        sphere_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        let curve_x = ellipse.frame().x();
        let curve_y = ellipse.frame().y();
        Self::new(
            ConicCurve::Ellipse(ellipse),
            curve_range,
            ellipse.frame().origin(),
            curve_x,
            curve_y,
            sphere,
            sphere_range,
            tolerances,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn new(
        conic: ConicCurve<'a>,
        curve_range: ParamRange,
        curve_origin: kgeom::vec::Point3,
        curve_x: Vec3,
        curve_y: Vec3,
        sphere: &'a Sphere,
        sphere_range: [ParamRange; 2],
        tolerances: Tolerances,
    ) -> Self {
        Self {
            conic,
            curve_range,
            sphere,
            sphere_range,
            local_center: sphere.frame().to_local(curve_origin),
            local_x: Vec3::new(
                curve_x.dot(sphere.frame().x()),
                curve_x.dot(sphere.frame().y()),
                curve_x.dot(sphere.frame().z()),
            ),
            local_y: Vec3::new(
                curve_y.dot(sphere.frame().x()),
                curve_y.dot(sphere.frame().y()),
                curve_y.dot(sphere.frame().z()),
            ),
            tolerances,
        }
    }

    fn curve(&self) -> &dyn Curve {
        match self.conic {
            ConicCurve::Circle(circle) => circle,
            ConicCurve::Ellipse(ellipse) => ellipse,
        }
    }

    fn local_point(&self, t_curve: f64) -> Vec3 {
        let (sin, cos) = math::sincos(t_curve);
        match self.conic {
            ConicCurve::Circle(circle) => {
                self.local_center + (self.local_x * cos + self.local_y * sin) * circle.radius()
            }
            ConicCurve::Ellipse(ellipse) => {
                self.local_center
                    + self.local_x * (ellipse.major_radius() * cos)
                    + self.local_y * (ellipse.minor_radius() * sin)
            }
        }
    }

    fn parameter_scale(&self) -> f64 {
        match self.conic {
            ConicCurve::Circle(circle) => circle.radius(),
            ConicCurve::Ellipse(ellipse) => ellipse.minor_radius(),
        }
    }

    fn local_extent(&self) -> f64 {
        self.local_center.norm()
            + match self.conic {
                ConicCurve::Circle(circle) => circle.radius(),
                ConicCurve::Ellipse(ellipse) => ellipse.major_radius(),
            }
    }

    fn window_z_coefficients(&self) -> (f64, f64) {
        match self.conic {
            ConicCurve::Circle(circle) => (
                self.local_x.z * circle.radius(),
                self.local_y.z * circle.radius(),
            ),
            ConicCurve::Ellipse(ellipse) => (
                self.local_x.z * ellipse.major_radius(),
                self.local_y.z * ellipse.minor_radius(),
            ),
        }
    }

    fn window_longitude_coefficients(&self, sin_u: f64, cos_u: f64) -> (f64, f64) {
        match self.conic {
            ConicCurve::Circle(circle) => {
                let radius = circle.radius();
                (
                    radius * (-sin_u * self.local_x.x + cos_u * self.local_x.y),
                    radius * (-sin_u * self.local_y.x + cos_u * self.local_y.y),
                )
            }
            ConicCurve::Ellipse(ellipse) => (
                ellipse.major_radius() * (-sin_u * self.local_x.x + cos_u * self.local_x.y),
                ellipse.minor_radius() * (-sin_u * self.local_y.x + cos_u * self.local_y.y),
            ),
        }
    }

    fn ellipse_implicit_coefficients(&self, ellipse: &Ellipse) -> [f64; 5] {
        let a_vec = self.local_x * ellipse.major_radius();
        let b_vec = self.local_y * ellipse.minor_radius();
        let c0 =
            self.local_center.dot(self.local_center) - self.sphere.radius() * self.sphere.radius();
        let cos = 2.0 * self.local_center.dot(a_vec);
        let sin = 2.0 * self.local_center.dot(b_vec);
        let cos2 = a_vec.dot(a_vec);
        let sin2 = b_vec.dot(b_vec);
        let sin_cos = 2.0 * a_vec.dot(b_vec);

        [
            c0 + cos + cos2,
            2.0 * sin + 2.0 * sin_cos,
            2.0 * c0 - 2.0 * cos2 + 4.0 * sin2,
            2.0 * sin - 2.0 * sin_cos,
            c0 - cos + cos2,
        ]
    }

    fn validation_reasons(&self) -> (&'static str, &'static str, &'static str) {
        match self.conic {
            ConicCurve::Circle(_) => (
                "circle/sphere intersection requires a finite non-reversed curve range",
                "bounded circle range cannot span more than one period",
                "circle/sphere intersection requires finite non-reversed surface ranges",
            ),
            ConicCurve::Ellipse(_) => (
                "ellipse/sphere intersection requires a finite non-reversed curve range",
                "bounded ellipse range cannot span more than one period",
                "ellipse/sphere intersection requires finite non-reversed surface ranges",
            ),
        }
    }

    fn validate(&self) -> Result<()> {
        let (curve_reason, period_reason, surface_reason) = self.validation_reasons();
        if !self.curve_range.is_finite() || self.curve_range.width() < 0.0 {
            return Err(Error::InvalidGeometry {
                reason: curve_reason,
            });
        }
        if self.curve_range.width()
            > core::f64::consts::TAU + parameter_tolerance(self.parameter_scale(), self.tolerances)
        {
            return Err(Error::InvalidGeometry {
                reason: period_reason,
            });
        }
        if self
            .sphere_range
            .iter()
            .any(|range| !range.is_finite() || range.width() < 0.0)
        {
            return Err(Error::InvalidGeometry {
                reason: surface_reason,
            });
        }
        Ok(())
    }

    fn curve_parameter_tolerance(&self) -> f64 {
        parameter_tolerance(self.parameter_scale(), self.tolerances)
    }

    fn add_contact(&self, points: &mut Vec<CurveSurfacePoint>, t_curve: f64, force_tangent: bool) {
        let Some(t_curve) =
            fit_curve_parameter(t_curve, self.curve_range, self.curve_parameter_tolerance())
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
        let kind = self.contact_kind(t_curve, uv, force_tangent);
        if let Some(point) = accept_curve_surface_candidate(
            self.curve(),
            t_curve,
            self.sphere,
            uv,
            kind,
            self.tolerances,
        ) {
            push_distinct(points, point, self.tolerances);
        }
    }

    fn contact_kind(&self, t_curve: f64, uv: [f64; 2], force_tangent: bool) -> ContactKind {
        match self.conic {
            ConicCurve::Circle(_) => {
                if force_tangent {
                    ContactKind::Tangent
                } else if self.sphere.normal(uv).is_none() {
                    ContactKind::Singular
                } else {
                    ContactKind::Transverse
                }
            }
            ConicCurve::Ellipse(_) => {
                if self.sphere.normal(uv).is_none() {
                    return ContactKind::Singular;
                }
                if force_tangent {
                    return ContactKind::Tangent;
                }
                let Some(normal) = self.sphere.normal(uv) else {
                    return ContactKind::Singular;
                };
                let tangent = self.curve().eval_derivs(t_curve, 1).d[1];
                let Some(tangent) = tangent.normalized() else {
                    return ContactKind::Singular;
                };
                if normal.dot(tangent).abs() <= self.tolerances.angular() {
                    ContactKind::Tangent
                } else {
                    ContactKind::Transverse
                }
            }
        }
    }
}

/// Run the shared bounded circle/ellipse-by-sphere intersection pipeline.
pub(super) fn intersect_bounded_conic_sphere(
    config: ConicSphereConfig<'_>,
) -> Result<CurveSurfaceIntersections> {
    config.validate()?;
    match config.conic {
        ConicCurve::Circle(circle) => intersect_circle_sphere_strategy(&config, circle),
        ConicCurve::Ellipse(ellipse) => intersect_ellipse_sphere_strategy(&config, ellipse),
    }
}

fn intersect_circle_sphere_strategy(
    config: &ConicSphereConfig<'_>,
    circle: &Circle,
) -> Result<CurveSurfaceIntersections> {
    let delta = circle.frame().origin() - config.sphere.frame().origin();
    let dx = delta.dot(circle.frame().x());
    let dy = delta.dot(circle.frame().y());
    let radius = circle.radius();
    let a = 2.0 * radius * dx;
    let b = 2.0 * radius * dy;
    let c = delta.norm_sq() + radius * radius - config.sphere.radius() * config.sphere.radius();
    let tolerance = circle_sphere_implicit_tolerance(
        delta.norm(),
        radius,
        config.sphere.radius(),
        config.tolerances,
    );
    let amplitude_scale = a.abs().max(b.abs());
    let amplitude = if amplitude_scale == 0.0 {
        0.0
    } else {
        amplitude_scale
            * ((a / amplitude_scale) * (a / amplitude_scale)
                + (b / amplitude_scale) * (b / amplitude_scale))
                .sqrt()
    };

    if amplitude <= tolerance {
        if c.abs() > tolerance {
            return Ok(CurveSurfaceIntersections::complete_empty());
        }
        return contained_conic_sphere(config);
    }

    let Some(roots) = trig_linear_roots(a, b, c, config.curve_range, tolerance) else {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            HARMONIC_ROOT_CLASSIFICATION_REASON,
        ));
    };
    let mut points = Vec::new();
    for (t_curve, tangent) in roots {
        config.add_contact(&mut points, t_curve, tangent);
    }

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn intersect_ellipse_sphere_strategy(
    config: &ConicSphereConfig<'_>,
    ellipse: &Ellipse,
) -> Result<CurveSurfaceIntersections> {
    let coeffs = config.ellipse_implicit_coefficients(ellipse);
    let tolerance = ellipse_sphere_implicit_tolerance(config);
    if coeffs.iter().all(|coeff| coeff.abs() <= tolerance) {
        return contained_conic_sphere(config);
    }

    let mut points = Vec::new();
    for t_curve in implicit_roots(&coeffs, config.curve_range, tolerance) {
        config.add_contact(&mut points, t_curve, false);
    }
    for t_curve in implicit_roots(
        &polynomial_derivative(&coeffs),
        config.curve_range,
        tolerance,
    ) {
        config.add_contact(&mut points, t_curve, true);
    }
    if sphere_implicit_value(config, core::f64::consts::PI).abs() <= tolerance {
        config.add_contact(&mut points, core::f64::consts::PI, true);
    }

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn contained_conic_sphere(config: &ConicSphereConfig<'_>) -> Result<CurveSurfaceIntersections> {
    let t_tol = config.curve_parameter_tolerance();
    if config.curve_range.width() <= t_tol {
        let mut points = Vec::new();
        config.add_contact(&mut points, config.curve_range.lo, true);
        return CurveSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    let mut cuts = vec![config.curve_range.lo, config.curve_range.hi];
    if !push_sphere_window_cuts(config, &mut cuts) {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            HARMONIC_ROOT_CLASSIFICATION_REASON,
        ));
    }
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
            config.local_point(mid),
            config.sphere_range,
            config.sphere.radius(),
            config.tolerances,
        )
        .is_none()
        {
            continue;
        }
        let Some(uv_start) = sphere_uv(
            config.local_point(lo),
            config.sphere_range,
            config.sphere.radius(),
            config.tolerances,
        ) else {
            continue;
        };
        let Some(uv_end) = sphere_uv(
            config.local_point(hi),
            config.sphere_range,
            config.sphere.radius(),
            config.tolerances,
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
        let cut_point = config.curve().eval(cut);
        if overlaps.iter().any(|overlap| {
            (cut >= overlap.curve.lo - t_tol && cut <= overlap.curve.hi + t_tol)
                || cut_point.dist(config.curve().eval(overlap.curve.lo))
                    <= config.tolerances.linear()
                || cut_point.dist(config.curve().eval(overlap.curve.hi))
                    <= config.tolerances.linear()
        }) {
            continue;
        }
        config.add_contact(&mut points, cut, true);
    }

    CurveSurfaceIntersections::canonicalized_complete(points, overlaps)
}

fn push_sphere_window_cuts(config: &ConicSphereConfig<'_>, cuts: &mut Vec<f64>) -> bool {
    let z_c = config.local_center.z;
    let (z_a, z_b) = config.window_z_coefficients();
    for v_bound in [config.sphere_range[1].lo, config.sphere_range[1].hi] {
        let z_bound = config.sphere.radius() * math::sin(v_bound);
        let Some(roots) = trig_linear_roots(
            z_a,
            z_b,
            z_c - z_bound,
            config.curve_range,
            config.tolerances.linear(),
        ) else {
            return false;
        };
        for (root, _) in roots {
            push_scalar(cuts, root, config.curve_parameter_tolerance());
        }
    }

    for u_bound in [config.sphere_range[0].lo, config.sphere_range[0].hi] {
        let (sin_u, cos_u) = math::sincos(u_bound);
        let c = -sin_u * config.local_center.x + cos_u * config.local_center.y;
        let (a, b) = config.window_longitude_coefficients(sin_u, cos_u);
        let Some(roots) =
            trig_linear_roots(a, b, c, config.curve_range, config.tolerances.linear())
        else {
            return false;
        };
        for (root, _) in roots {
            if !sphere_longitude_matches_bound(config.local_point(root), u_bound, config.tolerances)
            {
                continue;
            }
            push_scalar(cuts, root, config.curve_parameter_tolerance());
        }
    }
    true
}

fn sphere_implicit_value(config: &ConicSphereConfig<'_>, t_curve: f64) -> f64 {
    let local = config.local_point(t_curve);
    local.dot(local) - config.sphere.radius() * config.sphere.radius()
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

fn sphere_longitude_matches_bound(local: Vec3, bound: f64, tolerances: Tolerances) -> bool {
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

fn circle_sphere_implicit_tolerance(
    center_distance: f64,
    circle_radius: f64,
    sphere_radius: f64,
    tolerances: Tolerances,
) -> f64 {
    let scale = (center_distance + circle_radius + sphere_radius).max(1.0);
    tolerances.linear() * scale
}

fn ellipse_sphere_implicit_tolerance(config: &ConicSphereConfig<'_>) -> f64 {
    let scale = (config.local_extent() + config.sphere.radius()).max(1.0);
    config.tolerances.linear() * scale
}

const CONIC_NURBS_MIN_STEPS: usize = 96;
const CONIC_NURBS_MAX_STEPS: usize = 512;
const CONIC_NURBS_MAX_BISECTION_STEPS: usize = 80;
const ELLIPSE_NURBS_MAX_PROJECTION_STEPS: usize = 32;

#[derive(Debug, Clone, Copy)]
struct ConicNurbsSample {
    t_curve: f64,
    distance: f64,
    conic_unwrapped: f64,
}

/// Geometry-specific inputs for the shared bounded conic/NURBS marcher.
///
/// Circle distance and parameter recovery remain radial, while ellipse
/// distance and parameter recovery use closest-point projection. The grid,
/// polishing, overlap clipping, candidate, ordering, and completion stages
/// are common to both strategies.
#[derive(Clone, Copy)]
pub(super) struct ConicNurbsConfig<'a> {
    conic: ConicCurve<'a>,
    conic_range: ParamRange,
    curve: &'a NurbsCurve,
    curve_range: ParamRange,
    tolerances: Tolerances,
}

impl<'a> ConicNurbsConfig<'a> {
    pub(super) fn circle(
        circle: &'a Circle,
        conic_range: ParamRange,
        curve: &'a NurbsCurve,
        curve_range: ParamRange,
        tolerances: Tolerances,
    ) -> Self {
        Self {
            conic: ConicCurve::Circle(circle),
            conic_range,
            curve,
            curve_range,
            tolerances,
        }
    }

    pub(super) fn ellipse(
        ellipse: &'a Ellipse,
        conic_range: ParamRange,
        curve: &'a NurbsCurve,
        curve_range: ParamRange,
        tolerances: Tolerances,
    ) -> Self {
        Self {
            conic: ConicCurve::Ellipse(ellipse),
            conic_range,
            curve,
            curve_range,
            tolerances,
        }
    }

    fn conic(&self) -> &dyn Curve {
        match self.conic {
            ConicCurve::Circle(circle) => circle,
            ConicCurve::Ellipse(ellipse) => ellipse,
        }
    }

    fn parameter_scale(&self) -> f64 {
        match self.conic {
            ConicCurve::Circle(circle) => circle.radius(),
            ConicCurve::Ellipse(ellipse) => ellipse.minor_radius(),
        }
    }

    fn completion_reason(&self) -> &'static str {
        match self.conic {
            ConicCurve::Circle(_) => {
                "fixed-grid circle/NURBS candidate discovery does not prove complete coverage"
            }
            ConicCurve::Ellipse(_) => {
                "fixed-grid ellipse/NURBS candidate discovery does not prove complete coverage"
            }
        }
    }

    fn validation_reasons(&self) -> (&'static str, &'static str, &'static str, &'static str) {
        match self.conic {
            ConicCurve::Circle(_) => (
                "circle/nurbs intersection requires finite non-reversed ranges",
                "bounded circle range cannot span more than one period",
                "circle/nurbs intersection requires a clamped NURBS curve",
                "circle/nurbs intersection curve range must lie within the NURBS domain",
            ),
            ConicCurve::Ellipse(_) => (
                "ellipse/nurbs intersection requires finite non-reversed ranges",
                "bounded ellipse range cannot span more than one period",
                "ellipse/nurbs intersection requires a clamped NURBS curve",
                "ellipse/nurbs intersection curve range must lie within the NURBS domain",
            ),
        }
    }

    fn validate(&self) -> Result<()> {
        let (range_reason, period_reason, clamp_reason, domain_reason) = self.validation_reasons();
        if !self.conic_range.is_finite()
            || !self.curve_range.is_finite()
            || self.conic_range.width() < 0.0
            || self.curve_range.width() < 0.0
        {
            return Err(Error::InvalidGeometry {
                reason: range_reason,
            });
        }
        if self.conic_range.width()
            > core::f64::consts::TAU + parameter_tolerance(self.parameter_scale(), self.tolerances)
        {
            return Err(Error::InvalidGeometry {
                reason: period_reason,
            });
        }
        if !self.curve.knots().is_clamped() {
            return Err(Error::InvalidGeometry {
                reason: clamp_reason,
            });
        }
        let domain = self.curve.param_range();
        let curve_parameter_tol = conic_nurbs_curve_parameter_tolerance(domain, self.tolerances);
        if self.curve_range.lo < domain.lo - curve_parameter_tol
            || self.curve_range.hi > domain.hi + curve_parameter_tol
        {
            return Err(Error::InvalidGeometry {
                reason: domain_reason,
            });
        }
        Ok(())
    }

    fn raw_parameter(&self, point: Point3) -> f64 {
        match self.conic {
            ConicCurve::Circle(circle) => {
                let local = circle.frame().to_local(point);
                math::atan2(local.y, local.x)
            }
            ConicCurve::Ellipse(ellipse) => closest_ellipse_nurbs_parameter(point, ellipse),
        }
    }

    fn offset(&self, point: Point3) -> Vec3 {
        match self.conic {
            ConicCurve::Circle(circle) => {
                let local = circle.frame().to_local(point);
                let radial = (local.x * local.x + local.y * local.y).sqrt();
                let closest = if radial <= 1e-14 {
                    circle.frame().point_at(circle.radius(), 0.0, 0.0)
                } else {
                    let scale = circle.radius() / radial;
                    circle
                        .frame()
                        .point_at(local.x * scale, local.y * scale, 0.0)
                };
                point - closest
            }
            ConicCurve::Ellipse(ellipse) => {
                point - ellipse.eval(closest_ellipse_nurbs_parameter(point, ellipse))
            }
        }
    }

    fn distance(&self, point: Point3) -> f64 {
        self.offset(point).norm()
    }

    fn parameter(&self, point: Point3) -> Option<f64> {
        fit_periodic_parameter(
            self.raw_parameter(point),
            self.conic_range,
            parameter_tolerance(self.parameter_scale(), self.tolerances),
        )
    }

    fn parameter_unwrapped_near(&self, point: Point3, sample: ConicNurbsSample) -> f64 {
        conic_nurbs_unwrap_angle_near(self.raw_parameter(point), sample.conic_unwrapped)
    }

    fn provisional_result(
        &self,
        points: Vec<CurveCurvePoint>,
        overlaps: Vec<CurveCurveOverlap>,
    ) -> Result<CurveCurveIntersections> {
        CurveCurveIntersections::canonicalized_indeterminate(
            points,
            overlaps,
            self.completion_reason(),
        )
    }

    fn add_root_candidate(
        &self,
        t_curve: f64,
        forced_kind: Option<ContactKind>,
        points: &mut Vec<CurveCurvePoint>,
    ) {
        let point = self.curve.eval(t_curve);
        if self.distance(point) > self.tolerances.linear() {
            return;
        }
        let Some(t_conic) = self.parameter(point) else {
            return;
        };
        let Some(point) = accept_curve_curve_candidate(
            self.conic(),
            t_conic,
            self.curve,
            t_curve,
            forced_kind
                .map(|kind| self.forced_contact_kind(t_curve, kind))
                .unwrap_or_else(|| self.contact_kind(t_curve, t_conic)),
            self.tolerances,
        ) else {
            return;
        };
        self.push_distinct_point(points, point);
    }

    fn local_minimum_kind(&self, lo: f64, hi: f64) -> ContactKind {
        let a = self.offset(self.curve.eval(lo));
        let b = self.offset(self.curve.eval(hi));
        if a.norm() <= self.tolerances.linear() || b.norm() <= self.tolerances.linear() {
            ContactKind::Tangent
        } else if a.dot(b) < 0.0 {
            ContactKind::Transverse
        } else {
            ContactKind::Tangent
        }
    }

    fn forced_contact_kind(&self, t_curve: f64, kind: ContactKind) -> ContactKind {
        let tangent = self.curve.eval_derivs(t_curve, 1).d[1];
        if tangent.norm() <= self.tolerances.linear() {
            ContactKind::Singular
        } else {
            kind
        }
    }

    fn contact_kind(&self, t_curve: f64, t_conic: f64) -> ContactKind {
        let curve_tangent = self.curve.eval_derivs(t_curve, 1).d[1];
        let conic_tangent = self.conic().eval_derivs(t_conic, 1).d[1];
        let scale = curve_tangent.norm() * conic_tangent.norm();
        if scale <= self.tolerances.linear() {
            ContactKind::Singular
        } else if curve_tangent.cross(conic_tangent).norm() > scale * self.tolerances.angular() {
            ContactKind::Transverse
        } else {
            ContactKind::Tangent
        }
    }

    fn push_distinct_point(&self, points: &mut Vec<CurveCurvePoint>, candidate: CurveCurvePoint) {
        let conic_tol = parameter_tolerance(self.parameter_scale(), self.tolerances);
        if !points.iter().any(|point| {
            point.point.dist(candidate.point) <= self.tolerances.linear()
                || (point.t_a - candidate.t_a).abs() <= conic_tol
                    && (point.t_b - candidate.t_b).abs() <= self.tolerances.angular()
        }) {
            points.push(candidate);
        }
    }
}

/// Run the common fixed-grid bounded circle/ellipse-by-NURBS pipeline.
pub(super) fn intersect_bounded_conic_nurbs(
    mut config: ConicNurbsConfig<'_>,
) -> Result<CurveCurveIntersections> {
    config.validate()?;

    config.curve_range =
        conic_nurbs_clamp_to_domain(config.curve_range, config.curve.param_range());
    let curve_parameter_tol =
        conic_nurbs_curve_parameter_tolerance(config.curve_range, config.tolerances);
    if config.curve_range.width() <= curve_parameter_tol {
        return conic_nurbs_single_parameter_intersection(&config, config.curve_range.lo);
    }

    let samples = conic_nurbs_sample_curve(&config);
    if samples
        .iter()
        .all(|sample| sample.distance <= config.tolerances.linear())
    {
        return conic_nurbs_contained_curve_intersections(&config, &samples);
    }

    let mut points = Vec::new();
    if let Some(first) = samples.first()
        && first.distance <= config.tolerances.linear()
    {
        config.add_root_candidate(first.t_curve, None, &mut points);
    }
    if let Some(last) = samples.last()
        && last.distance <= config.tolerances.linear()
    {
        config.add_root_candidate(last.t_curve, None, &mut points);
    }
    for triple in samples.windows(3) {
        let [a, b, c] = triple else {
            continue;
        };
        if b.distance > a.distance || b.distance > c.distance {
            continue;
        }
        let root =
            conic_nurbs_minimize_distance(&config, a.t_curve, c.t_curve, curve_parameter_tol);
        config.add_root_candidate(
            root,
            Some(config.local_minimum_kind(a.t_curve, c.t_curve)),
            &mut points,
        );
    }

    config.provisional_result(points, Vec::new())
}

fn conic_nurbs_single_parameter_intersection(
    config: &ConicNurbsConfig<'_>,
    t_curve: f64,
) -> Result<CurveCurveIntersections> {
    if config.distance(config.curve.eval(t_curve)) > config.tolerances.linear() {
        return Ok(CurveCurveIntersections::indeterminate_empty(
            config.completion_reason(),
        ));
    }
    let mut points = Vec::new();
    config.add_root_candidate(t_curve, None, &mut points);
    config.provisional_result(points, Vec::new())
}

fn conic_nurbs_contained_curve_intersections(
    config: &ConicNurbsConfig<'_>,
    samples: &[ConicNurbsSample],
) -> Result<CurveCurveIntersections> {
    let global_range = ParamRange::new(samples[0].t_curve, samples[samples.len() - 1].t_curve);
    let curve_parameter_tol =
        conic_nurbs_curve_parameter_tolerance(global_range, config.tolerances);
    let mut overlaps = Vec::new();
    for pair in samples.windows(2) {
        let [a, b] = pair else {
            continue;
        };
        conic_nurbs_collect_range_overlaps(config, *a, *b, curve_parameter_tol, &mut overlaps);
    }
    conic_nurbs_merge_overlaps(&mut overlaps, global_range, config.tolerances);
    config.provisional_result(Vec::new(), overlaps)
}

fn conic_nurbs_collect_range_overlaps(
    config: &ConicNurbsConfig<'_>,
    a: ConicNurbsSample,
    b: ConicNurbsSample,
    curve_parameter_tol: f64,
    overlaps: &mut Vec<CurveCurveOverlap>,
) {
    let conic_tol = parameter_tolerance(config.parameter_scale(), config.tolerances);
    let mut cuts = vec![a.t_curve, b.t_curve];
    for target in
        conic_nurbs_boundary_images(a.conic_unwrapped, b.conic_unwrapped, config.conic_range)
    {
        if let Some(root) =
            conic_nurbs_parameter_root(config, a, b, target, curve_parameter_tol, conic_tol)
        {
            cuts.push(root);
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() <= curve_parameter_tol);

    for pair in cuts.windows(2) {
        let [lo, hi] = pair else {
            continue;
        };
        if hi - lo <= curve_parameter_tol {
            continue;
        }
        let mid = (lo + hi) / 2.0;
        if config.parameter(config.curve.eval(mid)).is_none() {
            continue;
        }
        let start_unwrapped = config.parameter_unwrapped_near(config.curve.eval(*lo), a);
        let end_unwrapped = config.parameter_unwrapped_near(config.curve.eval(*hi), b);
        let Some(start_conic) = config.parameter(config.curve.eval(*lo)) else {
            continue;
        };
        let Some(end_conic) = config.parameter(config.curve.eval(*hi)) else {
            continue;
        };
        overlaps.push(CurveCurveOverlap {
            a: ParamRange::new(start_conic.min(end_conic), start_conic.max(end_conic)),
            b: ParamRange::new(*lo, *hi),
            orientation: if end_unwrapped >= start_unwrapped {
                ParamOrientation::Same
            } else {
                ParamOrientation::Reversed
            },
        });
    }
}

fn conic_nurbs_boundary_images(a: f64, b: f64, range: ParamRange) -> Vec<f64> {
    let lo = a.min(b);
    let hi = a.max(b);
    let mut out = Vec::new();
    for base in [range.lo, range.hi] {
        let period = core::f64::consts::TAU;
        let k_min = ((lo - base) / period).floor() as i64 - 1;
        let k_max = ((hi - base) / period).ceil() as i64 + 1;
        for k in k_min..=k_max {
            let target = base + k as f64 * period;
            if target >= lo && target <= hi {
                out.push(target);
            }
        }
    }
    out.sort_by(f64::total_cmp);
    out.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
    out
}

fn conic_nurbs_parameter_root(
    config: &ConicNurbsConfig<'_>,
    a: ConicNurbsSample,
    b: ConicNurbsSample,
    target: f64,
    curve_parameter_tol: f64,
    conic_tol: f64,
) -> Option<f64> {
    let mut lo = a.t_curve;
    let mut hi = b.t_curve;
    let mut f_lo = a.conic_unwrapped - target;
    let f_hi = b.conic_unwrapped - target;
    if f_lo.abs() <= conic_tol {
        return Some(lo);
    }
    if f_hi.abs() <= conic_tol {
        return Some(hi);
    }
    if conic_nurbs_same_sign(f_lo, f_hi) {
        return None;
    }
    let mut root = (lo + hi) / 2.0;
    for _ in 0..CONIC_NURBS_MAX_BISECTION_STEPS {
        root = (lo + hi) / 2.0;
        let raw = config.raw_parameter(config.curve.eval(root));
        let f_mid = conic_nurbs_unwrap_angle_near(raw, target) - target;
        if f_mid.abs() <= conic_tol || hi - lo <= curve_parameter_tol {
            break;
        }
        if conic_nurbs_same_sign(f_lo, f_mid) {
            lo = root;
            f_lo = f_mid;
        } else {
            hi = root;
        }
    }
    Some(root)
}

fn conic_nurbs_sample_curve(config: &ConicNurbsConfig<'_>) -> Vec<ConicNurbsSample> {
    let span_hint = config
        .curve
        .knots()
        .control_count()
        .saturating_sub(config.curve.degree())
        .max(1);
    let steps = (span_hint * config.curve.degree().max(1) * 32)
        .clamp(CONIC_NURBS_MIN_STEPS, CONIC_NURBS_MAX_STEPS);
    let mut previous = None;
    (0..=steps)
        .map(|i| {
            let t_curve = config.curve_range.lerp(i as f64 / steps as f64);
            let point = config.curve.eval(t_curve);
            let raw = config.raw_parameter(point);
            let conic_unwrapped = previous
                .map(|angle| conic_nurbs_unwrap_angle_near(raw, angle))
                .unwrap_or(raw);
            previous = Some(conic_unwrapped);
            ConicNurbsSample {
                t_curve,
                distance: config.distance(point),
                conic_unwrapped,
            }
        })
        .collect()
}

fn conic_nurbs_minimize_distance(
    config: &ConicNurbsConfig<'_>,
    mut lo: f64,
    mut hi: f64,
    curve_parameter_tol: f64,
) -> f64 {
    for _ in 0..CONIC_NURBS_MAX_BISECTION_STEPS {
        if hi - lo <= curve_parameter_tol {
            break;
        }
        let third = (hi - lo) / 3.0;
        let left = lo + third;
        let right = hi - third;
        let f_left = config.distance(config.curve.eval(left));
        let f_right = config.distance(config.curve.eval(right));
        if (f_left - f_right).abs() <= 1e-18 {
            lo = left;
            hi = right;
        } else if f_left < f_right {
            hi = right;
        } else {
            lo = left;
        }
    }
    (lo + hi) / 2.0
}

fn conic_nurbs_merge_overlaps(
    overlaps: &mut Vec<CurveCurveOverlap>,
    global_range: ParamRange,
    tolerances: Tolerances,
) {
    overlaps.sort_by(|a, b| a.b.lo.total_cmp(&b.b.lo));
    let curve_parameter_tol = conic_nurbs_curve_parameter_tolerance(global_range, tolerances);
    let mut merged: Vec<CurveCurveOverlap> = Vec::new();
    for overlap in overlaps.drain(..) {
        if let Some(last) = merged.last_mut()
            && last.orientation == overlap.orientation
            && overlap.b.lo <= last.b.hi + curve_parameter_tol
        {
            last.a = ParamRange::new(last.a.lo.min(overlap.a.lo), last.a.hi.max(overlap.a.hi));
            last.b = ParamRange::new(last.b.lo, last.b.hi.max(overlap.b.hi));
            continue;
        }
        merged.push(overlap);
    }
    *overlaps = merged;
}

fn closest_ellipse_nurbs_parameter(point: Point3, ellipse: &Ellipse) -> f64 {
    let local = ellipse.frame().to_local(point);
    let initial = ellipse_parameter(local, ellipse);
    let mut candidates = [
        refine_ellipse_nurbs_projection_parameter(initial, local, ellipse),
        refine_ellipse_nurbs_projection_parameter(initial + core::f64::consts::PI, local, ellipse),
        0.0,
        core::f64::consts::FRAC_PI_2,
        core::f64::consts::PI,
        3.0 * core::f64::consts::FRAC_PI_2,
    ];
    candidates.sort_by(|a, b| {
        ellipse_nurbs_distance_sq(local, ellipse, *a)
            .total_cmp(&ellipse_nurbs_distance_sq(local, ellipse, *b))
    });
    candidates[0]
}

fn refine_ellipse_nurbs_projection_parameter(mut t: f64, local: Vec3, ellipse: &Ellipse) -> f64 {
    for _ in 0..ELLIPSE_NURBS_MAX_PROJECTION_STEPS {
        let (point, d1, d2) = ellipse_nurbs_local_derivs(ellipse, t);
        let residual = point - local;
        let f = residual.dot(d1);
        let df = d1.dot(d1) + residual.dot(d2);
        if df.abs() <= 1e-18 {
            break;
        }
        let step = f / df;
        t -= step;
        if step.abs() <= 1e-14 {
            break;
        }
    }
    t
}

fn ellipse_nurbs_distance_sq(local: Vec3, ellipse: &Ellipse, t: f64) -> f64 {
    let (point, _, _) = ellipse_nurbs_local_derivs(ellipse, t);
    (local - point).norm_sq()
}

fn ellipse_nurbs_local_derivs(ellipse: &Ellipse, t: f64) -> (Vec3, Vec3, Vec3) {
    let (sin, cos) = math::sincos(t);
    let major = ellipse.major_radius();
    let minor = ellipse.minor_radius();
    (
        Vec3::new(major * cos, minor * sin, 0.0),
        Vec3::new(-major * sin, minor * cos, 0.0),
        Vec3::new(-major * cos, -minor * sin, 0.0),
    )
}

fn conic_nurbs_unwrap_angle_near(raw: f64, reference: f64) -> f64 {
    let period = core::f64::consts::TAU;
    raw + ((reference - raw) / period).round() * period
}

fn conic_nurbs_same_sign(a: f64, b: f64) -> bool {
    (a < 0.0 && b < 0.0) || (a > 0.0 && b > 0.0)
}

fn conic_nurbs_clamp_to_domain(range: ParamRange, domain: ParamRange) -> ParamRange {
    ParamRange::new(
        range.lo.clamp(domain.lo, domain.hi),
        range.hi.clamp(domain.lo, domain.hi),
    )
}

fn conic_nurbs_curve_parameter_tolerance(range: ParamRange, tolerances: Tolerances) -> f64 {
    (range.width().abs() * 1e-10)
        .max(tolerances.angular())
        .max(1e-12)
}
