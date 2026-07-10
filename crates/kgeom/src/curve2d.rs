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
use crate::param::ParamRange;
use crate::vec::{Point2, Vec2};
use kcore::error::{Error, Result};
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
}

fn finite_point(p: Point2) -> bool {
    p.x.is_finite() && p.y.is_finite()
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
        let n = dir.norm();
        if !finite_point(origin) || !finite_point(dir) || !n.is_finite() || n == 0.0 {
            return Err(Error::InvalidGeometry {
                reason: "2D line requires a finite origin and nonzero finite direction",
            });
        }
        Ok(Self {
            origin,
            dir: dir / n,
        })
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
        let n = x.norm();
        if !finite_point(center)
            || !finite_point(x)
            || !radius.is_finite()
            || radius <= 0.0
            || !n.is_finite()
            || n == 0.0
        {
            return Err(Error::InvalidGeometry {
                reason: "2D circle requires finite center/axis and positive finite radius",
            });
        }
        Ok(Self {
            center,
            x: x / n,
            radius,
        })
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
        let domain = self.knots.domain();
        let t = t.clamp(domain.lo, domain.hi);
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
        Aabb2::from_points(&self.points)
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
