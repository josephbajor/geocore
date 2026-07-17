//! Curves in a surface's two-dimensional parameter space.
//!
//! Pcurves are geometry, not topology: a topology fin owns a *use* of one
//! of these curves together with its parameter range and the map from the
//! supporting 3D edge parameter.  Keeping this evaluator in `kgeom` lets
//! intersection and fitting code produce paired 3D/2D curves without a
//! dependency on the B-rep layer.

use core::any::Any;

use crate::aabb::Aabb2;
use crate::nurbs::KnotVector;
use crate::nurbs::basis::ders_basis_funs;
use crate::nurbs::ops::{Comb, insert_knot, refine_knots};
use crate::param::ParamRange;
use crate::vec::{Point2, Vec2};
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::math;

/// Position and derivatives of a parameter-space curve.
///
/// `d[k]` is `d^k P / dt^k` for `k = 0..=3`; entries above the requested
/// order are zero.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Curve2dDerivs {
    /// Derivatives by order; `d[0]` is the position.
    pub d: [Vec2; 4],
}

/// Uniform evaluator protocol for pcurves and other 2D curves.
pub trait Curve2d: Any {
    /// Runtime type view for exact analytic dispatch.
    fn as_any(&self) -> &dyn Any;

    /// Position at `t`.
    fn eval(&self, t: f64) -> Point2 {
        self.eval_derivs(t, 0).d[0]
    }

    /// Position and derivatives through `order` (capped at 3).
    fn eval_derivs(&self, t: f64, order: usize) -> Curve2dDerivs;

    /// Natural parameter range.
    fn param_range(&self) -> ParamRange;

    /// Parameter period, if periodic.
    fn periodicity(&self) -> Option<f64>;

    /// Bounding box over a finite range.
    fn bounding_box(&self, range: ParamRange) -> Aabb2;

    /// Conservative range of an affine coordinate form over the original
    /// source representation.
    ///
    /// The returned interval encloses
    /// `bias + linear.x * point.x + linear.y * point.y` for every point whose
    /// parameter lies in `range`. Implementations must derive the enclosure
    /// directly from their authored analytic data or original control net;
    /// sampled or rounded restricted representations are not proof sources.
    ///
    /// Unsupported representations, invalid ranges, and non-finite or
    /// inconclusive arithmetic return `None` so callers fail open.
    fn source_affine_range(
        &self,
        _range: ParamRange,
        _linear: Vec2,
        _bias: f64,
    ) -> Option<Interval> {
        None
    }
}

fn finite_point(p: Point2) -> bool {
    p.x.is_finite() && p.y.is_finite()
}

fn normalized_nonzero_direction(direction: Vec2) -> Option<Vec2> {
    if !finite_point(direction) {
        return None;
    }
    let norm = direction.norm();
    if norm == 0.0 {
        None
    } else if norm.is_finite() {
        // Preserve the established ordinary-input bits and computed-zero
        // rejection, including norms that underflow to zero.
        Some(direction / norm)
    } else {
        // Finite components can produce an infinite norm only through
        // squared-length overflow. Scaling first keeps the retry bounded.
        let scale = direction.x.abs().max(direction.y.abs());
        let scaled = direction / scale;
        let scaled_norm = scaled.norm();
        (scaled_norm.is_finite() && scaled_norm > 0.0).then_some(scaled / scaled_norm)
    }
}

fn finite_interval(interval: Interval) -> Option<Interval> {
    (interval.lo().is_finite() && interval.hi().is_finite()).then_some(interval)
}

fn affine_point_interval(point: Point2, linear: Vec2, bias: f64) -> Option<Interval> {
    if !finite_point(point) || !finite_point(linear) || !bias.is_finite() {
        return None;
    }
    finite_interval(
        Interval::point(bias)
            + Interval::point(linear.x) * Interval::point(point.x)
            + Interval::point(linear.y) * Interval::point(point.y),
    )
}

#[derive(Debug, Clone, Copy, Default)]
struct Hpt2 {
    x: f64,
    y: f64,
    w: f64,
}

impl Hpt2 {
    fn lift(point: Point2, weight: f64) -> Self {
        Self {
            x: point.x * weight,
            y: point.y * weight,
            w: weight,
        }
    }

    fn project(self) -> (Point2, f64) {
        (Point2::new(self.x / self.w, self.y / self.w), self.w)
    }
}

impl Comb for Hpt2 {
    fn comb(a: Self, b: Self, alpha: f64) -> Self {
        let one_minus = 1.0 - alpha;
        Self {
            x: a.x * one_minus + b.x * alpha,
            y: a.y * one_minus + b.y * alpha,
            w: a.w * one_minus + b.w * alpha,
        }
    }
}

/// An unbounded parameter-space line, parameterized by 2D arc length.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line2d {
    origin: Point2,
    dir: Vec2,
}

impl Line2d {
    /// Construct a line through `origin` in `dir`, normalizing `dir`.
    pub fn new(origin: Point2, dir: Vec2) -> Result<Self> {
        let Some(dir) = normalized_nonzero_direction(dir) else {
            return Err(Error::InvalidGeometry {
                reason: "2D line requires a finite origin and nonzero finite direction",
            });
        };
        if !finite_point(origin) {
            return Err(Error::InvalidGeometry {
                reason: "2D line requires a finite origin and nonzero finite direction",
            });
        }
        Ok(Self { origin, dir })
    }

    /// Point at parameter zero.
    pub fn origin(&self) -> Point2 {
        self.origin
    }

    /// Unit parameter-space direction.
    pub fn dir(&self) -> Vec2 {
        self.dir
    }
}

impl Curve2d for Line2d {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn eval_derivs(&self, t: f64, order: usize) -> Curve2dDerivs {
        let mut out = Curve2dDerivs::default();
        out.d[0] = self.origin + self.dir * t;
        if order >= 1 {
            out.d[1] = self.dir;
        }
        out
    }

    fn param_range(&self) -> ParamRange {
        ParamRange::unbounded()
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb2 {
        debug_assert!(range.is_finite());
        Aabb2::from_points(&[self.eval(range.lo), self.eval(range.hi)])
    }

    fn source_affine_range(&self, range: ParamRange, linear: Vec2, bias: f64) -> Option<Interval> {
        if !range.is_finite() || range.width() < 0.0 {
            return None;
        }
        let origin = affine_point_interval(self.origin, linear, bias)?;
        let slope = affine_point_interval(self.dir, linear, 0.0)?;
        finite_interval(origin + slope * Interval::new(range.lo, range.hi))
    }
}

/// A parameter-space circle with counterclockwise increasing parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Circle2d {
    center: Point2,
    x: Vec2,
    radius: f64,
}

impl Circle2d {
    /// Construct a circle with parameter zero along `x`; `x` is normalized.
    pub fn new(center: Point2, radius: f64, x: Vec2) -> Result<Self> {
        if !finite_point(center) || !radius.is_finite() || radius <= 0.0 {
            return Err(Error::InvalidGeometry {
                reason: "2D circle requires finite center/axis and positive finite radius",
            });
        }
        let x = normalized_nonzero_direction(x).ok_or(Error::InvalidGeometry {
            reason: "2D circle requires finite center/axis and positive finite radius",
        })?;
        Ok(Self { center, x, radius })
    }

    /// Circle center.
    pub fn center(&self) -> Point2 {
        self.center
    }

    /// Radius.
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// Unit direction from the center to parameter zero.
    pub fn x_dir(&self) -> Vec2 {
        self.x
    }
}

impl Curve2d for Circle2d {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn eval_derivs(&self, t: f64, order: usize) -> Curve2dDerivs {
        let (sin, cos) = math::sincos(t);
        let y = self.x.perp();
        let radial = self.x * cos + y * sin;
        let tangent = y * cos - self.x * sin;
        let mut out = Curve2dDerivs::default();
        out.d[0] = self.center + radial * self.radius;
        if order >= 1 {
            out.d[1] = tangent * self.radius;
        }
        if order >= 2 {
            out.d[2] = -radial * self.radius;
        }
        if order >= 3 {
            out.d[3] = -tangent * self.radius;
        }
        out
    }

    fn param_range(&self) -> ParamRange {
        ParamRange::new(0.0, core::f64::consts::TAU)
    }

    fn periodicity(&self) -> Option<f64> {
        Some(core::f64::consts::TAU)
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb2 {
        debug_assert!(range.is_finite());
        let mut out = Aabb2::from_points(&[self.eval(range.lo), self.eval(range.hi)]);
        let y = self.x.perp();
        for (xc, yc) in [(self.x.x, y.x), (self.x.y, y.y)] {
            let base = math::atan2(yc, xc);
            for k in -2..=2 {
                let t = base + core::f64::consts::PI * f64::from(k);
                if range.contains(t) {
                    out = out.including(self.eval(t));
                }
            }
        }
        out
    }

    fn source_affine_range(&self, range: ParamRange, linear: Vec2, bias: f64) -> Option<Interval> {
        if !range.is_finite() || range.width() < 0.0 {
            return None;
        }
        let center = affine_point_interval(self.center, linear, bias)?;
        let cosine = affine_point_interval(self.x, linear, 0.0)?;
        let sine = affine_point_interval(self.x.perp(), linear, 0.0)?;
        let norm = (cosine.square() + sine.square()).sqrt()?;
        let amplitude = finite_interval(Interval::point(self.radius) * norm)?.hi();
        finite_interval(center + Interval::new(-amplitude, amplitude))
    }
}

/// A polynomial or rational B-spline curve in parameter space.
///
/// This shares the kernel's validated [`KnotVector`] and basis evaluator
/// with 3D NURBS curves.  Control points remain genuinely two-dimensional;
/// they are not embedded into a synthetic model-space plane.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsCurve2d {
    knots: KnotVector,
    points: Vec<Point2>,
    weights: Option<Vec<f64>>,
}

impl NurbsCurve2d {
    /// Validated construction of a 2D NURBS curve.
    pub fn new(
        degree: usize,
        knots: Vec<f64>,
        points: Vec<Point2>,
        weights: Option<Vec<f64>>,
    ) -> Result<Self> {
        let knots = KnotVector::new(degree, knots)?;
        if points.len() != knots.control_count() {
            return Err(Error::InvalidGeometry {
                reason: "2D control point count does not match knot vector",
            });
        }
        if points.iter().any(|&p| !finite_point(p)) {
            return Err(Error::InvalidGeometry {
                reason: "2D NURBS control points must be finite",
            });
        }
        if let Some(w) = &weights
            && (w.len() != points.len()
                || w.iter().any(|&value| !value.is_finite() || value <= 0.0))
        {
            return Err(Error::InvalidGeometry {
                reason: "2D NURBS weights must match control points and be positive and finite",
            });
        }
        Ok(Self {
            knots,
            points,
            weights,
        })
    }

    /// Degree.
    pub fn degree(&self) -> usize {
        self.knots.degree()
    }

    /// Knot vector.
    pub fn knots(&self) -> &KnotVector {
        &self.knots
    }

    /// Euclidean control points.
    pub fn points(&self) -> &[Point2] {
        &self.points
    }

    /// Rational weights, if present.
    pub fn weights(&self) -> Option<&[f64]> {
        self.weights.as_deref()
    }

    fn from_hpts(degree: usize, knots: Vec<f64>, points: Vec<Hpt2>) -> Result<Self> {
        let (points, weights): (Vec<Point2>, Vec<f64>) =
            points.into_iter().map(Hpt2::project).unzip();
        Self::new(degree, knots, points, Some(weights))
    }

    /// Curve with `u` inserted `times` times. The represented point set and
    /// rational weights are preserved exactly in homogeneous space.
    pub fn with_knot_inserted(&self, u: f64, times: usize) -> Result<Self> {
        match &self.weights {
            Some(weights) => {
                let points: Vec<_> = self
                    .points
                    .iter()
                    .zip(weights)
                    .map(|(&point, &weight)| Hpt2::lift(point, weight))
                    .collect();
                let (knots, points) = insert_knot(&self.knots, &points, u, times)?;
                Self::from_hpts(self.degree(), knots, points)
            }
            None => {
                let (knots, points) = insert_knot(&self.knots, &self.points, u, times)?;
                Self::new(self.degree(), knots, points, None)
            }
        }
    }

    /// Curve with each value in `knots` inserted once per occurrence.
    pub fn with_knots_refined(&self, knots: &[f64]) -> Result<Self> {
        let degree = self.degree();
        match &self.weights {
            Some(weights) => {
                let points: Vec<_> = self
                    .points
                    .iter()
                    .zip(weights)
                    .map(|(&point, &weight)| Hpt2::lift(point, weight))
                    .collect();
                let (knots, points) = refine_knots(degree, &self.knots, &points, knots)?;
                Self::from_hpts(degree, knots, points)
            }
            None => {
                let (knots, points) = refine_knots(degree, &self.knots, &self.points, knots)?;
                Self::new(degree, knots, points, None)
            }
        }
    }

    /// Split a clamped curve at a parameter strictly inside its domain.
    pub fn split_at(&self, parameter: f64) -> Result<(Self, Self)> {
        if !self.knots.is_clamped() {
            return Err(Error::InvalidGeometry {
                reason: "splitting a 2D NURBS requires a clamped knot vector",
            });
        }
        let domain = self.knots.domain();
        if !(domain.lo < parameter && parameter < domain.hi) {
            return Err(Error::InvalidGeometry {
                reason: "2D NURBS split parameter must lie strictly inside the domain",
            });
        }
        let degree = self.degree();
        if degree == 0 {
            return Err(Error::InvalidGeometry {
                reason: "degree-zero 2D NURBS splitting is unsupported",
            });
        }
        let needed = degree - self.knots.multiplicity(parameter);
        let full = if needed > 0 {
            self.with_knot_inserted(parameter, needed)?
        } else {
            self.clone()
        };
        let knots = full.knots.as_slice();
        let split = knots
            .iter()
            .position(|&knot| knot == parameter)
            .expect("split knot has full multiplicity after insertion");
        let mut left_knots = knots[..split + degree].to_vec();
        left_knots.push(parameter);
        let mut right_knots = vec![parameter];
        right_knots.extend_from_slice(&knots[split..]);
        let left_points = full.points[..split].to_vec();
        let right_points = full.points[split - 1..].to_vec();
        let (left_weights, right_weights) = match &full.weights {
            Some(weights) => (
                Some(weights[..split].to_vec()),
                Some(weights[split - 1..].to_vec()),
            ),
            None => (None, None),
        };
        Ok((
            Self::new(degree, left_knots, left_points, left_weights)?,
            Self::new(degree, right_knots, right_points, right_weights)?,
        ))
    }

    /// Clamped subcurve over `range`, preserving exact parameter values.
    pub fn restricted_to(&self, range: ParamRange) -> Result<Self> {
        if !self.knots.is_clamped() {
            return Err(Error::InvalidGeometry {
                reason: "restricting a 2D NURBS requires a clamped knot vector",
            });
        }
        let domain = self.knots.domain();
        if range.lo < domain.lo || range.hi > domain.hi {
            return Err(Error::InvalidGeometry {
                reason: "2D NURBS restriction lies outside its domain",
            });
        }
        let mut restricted = self.clone();
        if range.lo > domain.lo {
            restricted = restricted.split_at(range.lo)?.1;
        }
        if range.hi < domain.hi {
            restricted = restricted.split_at(range.hi)?.0;
        }
        Ok(restricted)
    }

    fn subrange_control_box(&self, range: ParamRange) -> Aabb2 {
        self.restricted_to(range).map_or_else(
            |_| Aabb2::from_points(&self.points),
            |curve| Aabb2::from_points(&curve.points),
        )
    }

    fn active_source_controls(
        &self,
        range: ParamRange,
    ) -> Option<core::ops::RangeInclusive<usize>> {
        if !range.is_finite() || range.width() < 0.0 {
            return None;
        }
        let domain = self.knots.domain();
        if range.lo < domain.lo || range.hi > domain.hi {
            return None;
        }

        let degree = self.degree();
        let knots = self.knots.as_slice();
        let last_span = self.points.len().checked_sub(1)?;
        let mut first = None;
        let mut last = None;
        for span in degree..=last_span {
            if knots[span] >= knots[span + 1] {
                continue;
            }
            let local_lo = range.lo.max(knots[span]);
            let local_hi = range.hi.min(knots[span + 1]);
            if local_lo > local_hi {
                continue;
            }
            let span_first = span.checked_sub(degree)?;
            first = Some(first.map_or(span_first, |current: usize| current.min(span_first)));
            last = Some(last.map_or(span, |current: usize| current.max(span)));
        }
        Some(first?..=last?)
    }
}

impl Curve2d for NurbsCurve2d {
    fn as_any(&self) -> &dyn Any {
        self
    }

    #[allow(clippy::needless_range_loop)]
    fn eval_derivs(&self, t: f64, order: usize) -> Curve2dDerivs {
        const BINOMIAL: [[f64; 4]; 4] = [
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0, 0.0],
            [1.0, 2.0, 1.0, 0.0],
            [1.0, 3.0, 3.0, 1.0],
        ];

        let order = order.min(3);
        let t = self.knots.domain().clamp_param(t);
        let span = self.knots.find_span(t);
        let p = self.degree();
        let base = span - p;
        let ders = ders_basis_funs(&self.knots, span, t, order);
        let mut out = Curve2dDerivs::default();
        match &self.weights {
            None => {
                for k in 0..=order {
                    for (j, &basis) in ders[k].iter().enumerate() {
                        out.d[k] = out.d[k] + self.points[base + j] * basis;
                    }
                }
            }
            Some(weights) => {
                let mut weighted = [Vec2::default(); 4];
                let mut weight_ders = [0.0; 4];
                for k in 0..=order {
                    for (j, &basis) in ders[k].iter().enumerate() {
                        let index = base + j;
                        let wb = weights[index] * basis;
                        weighted[k] = weighted[k] + self.points[index] * wb;
                        weight_ders[k] += wb;
                    }
                }
                for k in 0..=order {
                    let mut value = weighted[k];
                    for i in 1..=k {
                        value = value - out.d[k - i] * (BINOMIAL[k][i] * weight_ders[i]);
                    }
                    out.d[k] = value / weight_ders[0];
                }
            }
        }
        out
    }

    fn param_range(&self) -> ParamRange {
        self.knots.domain()
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb2 {
        debug_assert!(range.is_finite());
        self.subrange_control_box(range)
    }

    fn source_affine_range(&self, range: ParamRange, linear: Vec2, bias: f64) -> Option<Interval> {
        let mut result: Option<Interval> = None;
        for index in self.active_source_controls(range)? {
            let value = affine_point_interval(self.points[index], linear, bias)?;
            result = Some(match result {
                Some(current) => {
                    Interval::new(current.lo().min(value.lo()), current.hi().max(value.hi()))
                }
                None => value,
            });
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_and_circle_derivatives_are_consistent() {
        let line = Line2d::new(Point2::new(2.0, -1.0), Vec2::new(3.0, 4.0)).unwrap();
        assert!(line.eval(5.0).dist(Point2::new(5.0, 3.0)) < 1e-14);

        let circle = Circle2d::new(Point2::new(1.0, 2.0), 3.0, Vec2::new(1.0, 0.0)).unwrap();
        let d = circle.eval_derivs(core::f64::consts::FRAC_PI_2, 2);
        assert!(d.d[0].dist(Point2::new(1.0, 5.0)) < 1e-14);
        assert!(d.d[1].dist(Vec2::new(-3.0, 0.0)) < 1e-14);
        assert!(d.d[2].dist(Vec2::new(0.0, -3.0)) < 1e-14);
    }

    #[test]
    fn analytic_directions_accept_finite_overflow_scales_without_changing_ordinary_bits() {
        let origin = Point2::new(1.0, -2.0);
        let ordinary_direction = Vec2::new(1.0, -0.5);
        let ordinary_line = Line2d::new(origin, ordinary_direction).unwrap();
        assert_eq!(
            ordinary_line.dir(),
            ordinary_direction / ordinary_direction.norm()
        );
        let scaled_line =
            Line2d::new(origin, Vec2::new(2.0_f64.powi(700), -2.0_f64.powi(699))).unwrap();
        assert_eq!(scaled_line, ordinary_line);

        let diagonal_line = Line2d::new(origin, Vec2::new(f64::MAX, f64::MAX)).unwrap();
        assert_eq!(
            diagonal_line,
            Line2d::new(origin, Vec2::new(1.0, 1.0)).unwrap()
        );

        let center = Point2::new(-3.0, 4.0);
        let ordinary_axis = Vec2::new(1.0, 1.0);
        let ordinary_circle = Circle2d::new(center, 2.5, ordinary_axis).unwrap();
        assert_eq!(
            ordinary_circle.x_dir(),
            ordinary_axis / ordinary_axis.norm()
        );
        let extreme_circle = Circle2d::new(center, 2.5, Vec2::new(f64::MAX, f64::MAX)).unwrap();
        assert_eq!(extreme_circle, ordinary_circle);
        assert_eq!(
            extreme_circle.eval_derivs(0.25, 3),
            ordinary_circle.eval_derivs(0.25, 3)
        );
    }

    #[test]
    fn analytic_directions_preserve_nonfinite_zero_and_underflow_rejection() {
        let origin = Point2::default();
        for direction in [
            Vec2::new(0.0, 0.0),
            Vec2::new(f64::MIN_POSITIVE, 0.0),
            Vec2::new(2.0_f64.powi(-700), 0.0),
            Vec2::new(f64::INFINITY, 0.0),
            Vec2::new(f64::NAN, 1.0),
        ] {
            assert!(Line2d::new(origin, direction).is_err());
            assert!(Circle2d::new(origin, 1.0, direction).is_err());
        }

        let below_model_floor = Vec2::new(1.0e-9, 0.0);
        assert_eq!(
            Line2d::new(origin, below_model_floor).unwrap().dir(),
            Vec2::new(1.0, 0.0)
        );
        assert_eq!(
            Circle2d::new(origin, 1.0, below_model_floor)
                .unwrap()
                .x_dir(),
            Vec2::new(1.0, 0.0)
        );
    }

    #[test]
    fn rational_quadratic_is_an_exact_quarter_circle() {
        let curve = NurbsCurve2d::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point2::new(1.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(0.0, 1.0),
            ],
            Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
        )
        .unwrap();
        for i in 0..=100 {
            let p = curve.eval(i as f64 / 100.0);
            assert!((p.norm() - 1.0).abs() < 1e-12);
        }
    }

    #[test]
    fn knot_insertion_split_and_restriction_preserve_2d_nurbs() {
        let curve = NurbsCurve2d::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point2::new(1.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(0.0, 1.0),
            ],
            Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
        )
        .unwrap();
        let refined = curve.with_knot_inserted(0.4, 1).unwrap();
        let (left, right) = curve.split_at(0.4).unwrap();
        let restricted = curve.restricted_to(ParamRange::new(0.2, 0.6)).unwrap();
        assert_eq!(left.param_range(), ParamRange::new(0.0, 0.4));
        assert_eq!(right.param_range(), ParamRange::new(0.4, 1.0));
        assert_eq!(restricted.param_range(), ParamRange::new(0.2, 0.6));
        for index in 0..=100 {
            let parameter = f64::from(index) / 100.0;
            assert!(refined.eval(parameter).dist(curve.eval(parameter)) < 1.0e-12);
            if left.param_range().contains(parameter) {
                assert!(left.eval(parameter).dist(curve.eval(parameter)) < 1.0e-12);
            }
            if right.param_range().contains(parameter) {
                assert!(right.eval(parameter).dist(curve.eval(parameter)) < 1.0e-12);
            }
            if restricted.param_range().contains(parameter) {
                assert!(restricted.eval(parameter).dist(curve.eval(parameter)) < 1.0e-12);
            }
        }
    }

    #[test]
    fn nurbs_subrange_box_tightens_and_contains_the_curve() {
        let curve = NurbsCurve2d::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point2::new(0.0, 0.0),
                Point2::new(0.0, 10.0),
                Point2::new(10.0, 10.0),
                Point2::new(10.0, 0.0),
            ],
            None,
        )
        .unwrap();
        let range = ParamRange::new(0.0, 0.1);
        let full = Aabb2::from_points(curve.points());
        let subrange = curve
            .bounding_box(range)
            .inflated(kcore::tolerance::LINEAR_RESOLUTION);
        assert!(subrange.max.x < full.max.x);
        assert!(subrange.max.y < full.max.y);
        for index in 0..=100 {
            let parameter = range.lo + range.width() * f64::from(index) / 100.0;
            let point = curve.eval(parameter);
            assert!(
                point.x >= subrange.min.x
                    && point.x <= subrange.max.x
                    && point.y >= subrange.min.y
                    && point.y <= subrange.max.y
            );
        }
    }

    #[test]
    fn analytic_source_affine_ranges_enclose_line_and_circle_coordinates() {
        let line = Line2d::new(Point2::new(2.0, -1.0), Vec2::new(3.0, 4.0)).unwrap();
        let line_range = ParamRange::new(-2.0, 5.0);
        let line_bound = line
            .source_affine_range(line_range, Vec2::new(0.0, 1.0), 0.0)
            .unwrap();
        for index in 0..=100 {
            let point = line.eval(line_range.lerp(f64::from(index) / 100.0));
            assert!(line_bound.contains(point.y));
        }

        let circle = Circle2d::new(Point2::new(3.0, -2.0), 4.0, Vec2::new(0.6, 0.8)).unwrap();
        let circle_range = ParamRange::new(0.2, 0.6);
        let circle_bound = circle
            .source_affine_range(circle_range, Vec2::new(-3.0, 2.0), 7.0)
            .unwrap();
        for index in 0..=100 {
            let point = circle.eval(circle_range.lerp(f64::from(index) / 100.0));
            assert!(circle_bound.contains(7.0 - 3.0 * point.x + 2.0 * point.y));
        }
    }

    #[test]
    fn source_affine_range_rejects_invalid_or_unrepresentable_queries() {
        let line = Line2d::new(Point2::default(), Vec2::new(1.0, 0.0)).unwrap();
        assert!(
            line.source_affine_range(
                ParamRange::new(0.0, f64::INFINITY),
                Vec2::new(0.0, 1.0),
                0.0,
            )
            .is_none()
        );

        let curve = NurbsCurve2d::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)],
            None,
        )
        .unwrap();
        assert!(
            curve
                .source_affine_range(ParamRange::new(-1.0, 0.5), Vec2::new(0.0, 1.0), 0.0,)
                .is_none()
        );
    }

    #[test]
    fn hostile_degree_five_source_range_exposes_hidden_vertical_excursion() {
        let tau = core::f64::consts::TAU;
        let v = [1.0, -43.0 / 5.0, 109.0 / 5.0, -99.0 / 5.0, 53.0 / 5.0, 1.0];
        let curve = NurbsCurve2d::new(
            5,
            vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            (0..=5)
                .map(|index| Point2::new(tau * f64::from(index) / 5.0, v[index as usize]))
                .collect(),
            None,
        )
        .unwrap();
        for parameter in [0.0, 0.25, 0.5, 0.75, 1.0] {
            assert!((curve.eval(parameter).y - 1.0).abs() < 1.0e-13);
        }
        assert!((curve.eval(0.125).y + 41.0 / 64.0).abs() < 1.0e-13);

        let source = curve
            .source_affine_range(curve.param_range(), Vec2::new(0.0, 1.0), 0.0)
            .unwrap();
        assert!(source.lo() <= -99.0 / 5.0);
        assert!(source.hi() >= 109.0 / 5.0);
        assert!(source.contains(curve.eval(0.125).y));
    }

    #[test]
    fn rational_source_affine_range_uses_positive_weight_control_hull() {
        let curve = NurbsCurve2d::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point2::new(1.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(0.0, 1.0),
            ],
            Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
        )
        .unwrap();
        let linear = Vec2::new(-2.0, 3.0);
        let bias = 5.0;
        let source = curve
            .source_affine_range(curve.param_range(), linear, bias)
            .unwrap();
        for index in 0..=256 {
            let point = curve.eval(f64::from(index) / 256.0);
            assert!(source.contains(bias + linear.dot(point)));
        }
    }

    #[test]
    fn constructors_reject_non_finite_data() {
        assert!(Line2d::new(Point2::new(f64::NAN, 0.0), Vec2::new(1.0, 0.0)).is_err());
        assert!(Circle2d::new(Point2::new(0.0, 0.0), -1.0, Vec2::new(1.0, 0.0)).is_err());
        assert!(
            NurbsCurve2d::new(
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![Point2::new(0.0, 0.0), Point2::new(f64::INFINITY, 0.0)],
                None,
            )
            .is_err()
        );
    }
}
