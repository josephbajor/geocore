//! The curve evaluator protocol and analytic curve classes.
//!
//! Every curve class in the kernel — analytic, NURBS, and later procedural
//! (intersection curves, SP-curves) — implements [`Curve`]. The protocol is
//! object-safe so higher layers can hold `&dyn Curve` behind topology.
//!
//! Parameterization conventions (XT-aligned, re-verified at M3):
//! - [`Line`]: `P(t) = origin + t·dir`, `dir` unit, `t` is arc length,
//!   unbounded.
//! - [`Circle`]: `P(t) = c + r(cos t · X + sin t · Y)`, `t ∈ [0, 2π)`,
//!   periodic.
//! - [`Ellipse`]: `P(t) = c + r₁ cos t · X + r₂ sin t · Y`, `r₁ ≥ r₂ > 0`,
//!   `t ∈ [0, 2π)`, periodic. (Note: `t` is *not* arc angle.)

use core::any::Any;

use crate::aabb::Aabb3;
use crate::frame::Frame;
use crate::param::ParamRange;
use crate::vec::{Point3, Vec3};
use kcore::error::{Error, Result};
use kcore::math;

/// Position and derivatives of a curve at a parameter: `d[k]` is
/// `d^k P / dt^k` for `k = 0..=3` (entries above the requested order are
/// zero).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CurveDerivs {
    /// Derivatives by order; `d[0]` is the position.
    pub d: [Vec3; 4],
}

/// The uniform curve evaluator protocol (spec §L1).
///
/// Implementations must be exact for their class (no internal approximation)
/// and deterministic. Parameters outside the periodic base range are wrapped;
/// parameters outside a bounded range are a caller bug (debug-asserted, then
/// clamped).
pub trait Curve: Any {
    /// Runtime type view for dispatch layers that need exact analytic cases.
    fn as_any(&self) -> &dyn Any;

    /// Position at `t`.
    fn eval(&self, t: f64) -> Point3 {
        self.eval_derivs(t, 0).d[0]
    }

    /// Position and derivatives up to `order` (≤ 3).
    fn eval_derivs(&self, t: f64, order: usize) -> CurveDerivs;

    /// Natural parameter range (may be unbounded).
    fn param_range(&self) -> ParamRange;

    /// Period of the parameterization, if periodic.
    fn periodicity(&self) -> Option<f64>;

    /// Bounding box of the curve restricted to `range` (finite required),
    /// exact for analytic classes.
    fn bounding_box(&self, range: ParamRange) -> Aabb3;
}

/// An unbounded straight line, parameterized by arc length.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line {
    origin: Point3,
    dir: Vec3,
}

impl Line {
    /// Line through `origin` in direction `dir` (normalized internally).
    pub fn new(origin: Point3, dir: Vec3) -> Result<Line> {
        let dir = dir.normalized().ok_or(Error::InvalidGeometry {
            reason: "line direction has zero length",
        })?;
        Ok(Line { origin, dir })
    }

    /// Point on the line at parameter 0.
    pub fn origin(&self) -> Point3 {
        self.origin
    }

    /// Unit direction.
    pub fn dir(&self) -> Vec3 {
        self.dir
    }
}

impl Curve for Line {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn eval_derivs(&self, t: f64, order: usize) -> CurveDerivs {
        let mut d = CurveDerivs::default();
        d.d[0] = self.origin + self.dir * t;
        if order >= 1 {
            d.d[1] = self.dir;
        }
        d
    }

    fn param_range(&self) -> ParamRange {
        ParamRange::unbounded()
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb3 {
        debug_assert!(range.is_finite());
        Aabb3::from_points(&[self.eval(range.lo), self.eval(range.hi)])
    }
}

/// A full circle in the `frame.x`/`frame.y` plane centered at the frame
/// origin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Circle {
    frame: Frame,
    radius: f64,
}

impl Circle {
    /// Circle of `radius` in the given frame's xy plane.
    pub fn new(frame: Frame, radius: f64) -> Result<Circle> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(Error::InvalidGeometry {
                reason: "circle radius must be positive and finite",
            });
        }
        Ok(Circle { frame, radius })
    }

    /// Positioning frame (circle lies in its xy plane).
    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Radius.
    pub fn radius(&self) -> f64 {
        self.radius
    }
}

impl Curve for Circle {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn eval_derivs(&self, t: f64, order: usize) -> CurveDerivs {
        let (sin, cos) = math::sincos(t);
        let radial = self.frame.x() * cos + self.frame.y() * sin;
        let tangential = self.frame.y() * cos - self.frame.x() * sin;
        let mut d = CurveDerivs::default();
        d.d[0] = self.frame.origin() + radial * self.radius;
        if order >= 1 {
            d.d[1] = tangential * self.radius;
        }
        if order >= 2 {
            d.d[2] = -radial * self.radius;
        }
        if order >= 3 {
            d.d[3] = -tangential * self.radius;
        }
        d
    }

    fn param_range(&self) -> ParamRange {
        ParamRange::new(0.0, core::f64::consts::TAU)
    }

    fn periodicity(&self) -> Option<f64> {
        Some(core::f64::consts::TAU)
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb3 {
        debug_assert!(range.is_finite());
        // Endpoints plus every axis-extreme parameter inside the range.
        // Extremes of coordinate i occur where d/dt [P(t)·e_i] = 0:
        // tan t = (y_i r) / (x_i r) per world axis — equivalently at
        // atan2(y_i, x_i) and its antipode, with x_i, y_i the frame axis
        // components.
        let mut bb = Aabb3::from_points(&[self.eval(range.lo), self.eval(range.hi)]);
        let x = self.frame.x().to_array();
        let y = self.frame.y().to_array();
        for i in 0..3 {
            let base = math::atan2(y[i], x[i]);
            for k in -2..=2 {
                let t = base + core::f64::consts::PI * f64::from(k);
                if range.contains(t) {
                    bb = bb.including(self.eval(t));
                }
            }
        }
        bb
    }
}

/// An ellipse in the `frame.x`/`frame.y` plane centered at the frame origin:
/// `P(t) = c + r₁ cos t · X + r₂ sin t · Y` with the major radius `r₁` along
/// `X` (Parasolid convention, `r₁ ≥ r₂ > 0`), `t ∈ [0, 2π)`, periodic.
///
/// Note that `t` is the parametric angle, *not* the polar angle of the point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ellipse {
    frame: Frame,
    r1: f64,
    r2: f64,
}

impl Ellipse {
    /// Ellipse with major radius `r1` along the frame's x axis and minor
    /// radius `r2` along y. Requires `r1 ≥ r2 > 0`, both finite.
    pub fn new(frame: Frame, r1: f64, r2: f64) -> Result<Ellipse> {
        if !r1.is_finite() || !r2.is_finite() || r2 <= 0.0 || r1 < r2 {
            return Err(Error::InvalidGeometry {
                reason: "ellipse radii must satisfy r1 >= r2 > 0 and be finite",
            });
        }
        Ok(Ellipse { frame, r1, r2 })
    }

    /// Positioning frame (ellipse lies in its xy plane).
    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Major radius (along the frame's x axis).
    pub fn major_radius(&self) -> f64 {
        self.r1
    }

    /// Minor radius (along the frame's y axis).
    pub fn minor_radius(&self) -> f64 {
        self.r2
    }
}

impl Curve for Ellipse {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn eval_derivs(&self, t: f64, order: usize) -> CurveDerivs {
        let (sin, cos) = math::sincos(t);
        let radial = self.frame.x() * (self.r1 * cos) + self.frame.y() * (self.r2 * sin);
        let tangential = self.frame.y() * (self.r2 * cos) - self.frame.x() * (self.r1 * sin);
        let mut d = CurveDerivs::default();
        d.d[0] = self.frame.origin() + radial;
        if order >= 1 {
            d.d[1] = tangential;
        }
        if order >= 2 {
            d.d[2] = -radial;
        }
        if order >= 3 {
            d.d[3] = -tangential;
        }
        d
    }

    fn param_range(&self) -> ParamRange {
        ParamRange::new(0.0, core::f64::consts::TAU)
    }

    fn periodicity(&self) -> Option<f64> {
        Some(core::f64::consts::TAU)
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb3 {
        debug_assert!(range.is_finite());
        // Endpoints plus every axis-extreme parameter inside the range.
        // Coordinate i extremizes where d/dt [P(t)·e_i] = 0:
        // -r₁ sin t · X_i + r₂ cos t · Y_i = 0, i.e.
        // t = atan2(r₂ Y_i, r₁ X_i) (+ kπ).
        let mut bb = Aabb3::from_points(&[self.eval(range.lo), self.eval(range.hi)]);
        let x = self.frame.x().to_array();
        let y = self.frame.y().to_array();
        for i in 0..3 {
            let base = math::atan2(self.r2 * y[i], self.r1 * x[i]);
            let k_lo = ((range.lo - base) / core::f64::consts::PI).floor() as i64;
            let k_hi = ((range.hi - base) / core::f64::consts::PI).ceil() as i64;
            for k in k_lo..=k_hi {
                let t = base + core::f64::consts::PI * k as f64;
                if range.contains(t) {
                    bb = bb.including(self.eval(t));
                }
            }
        }
        bb
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conformance::check_curve;

    fn tilted_frame() -> Frame {
        Frame::new(
            Point3::new(1.0, -2.0, 0.5),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap()
    }

    #[test]
    fn circle_points_are_equidistant_from_center() {
        let c = Circle::new(tilted_frame(), 2.5).unwrap();
        for i in 0..64 {
            let t = i as f64 * core::f64::consts::TAU / 64.0;
            let p = c.eval(t);
            assert!((p.dist(c.frame().origin()) - 2.5).abs() < 1e-12);
        }
    }

    #[test]
    fn circle_conformance() {
        let c = Circle::new(tilted_frame(), 2.5).unwrap();
        check_curve(&c);
    }

    #[test]
    fn line_conformance() {
        let l = Line::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(-1.0, 0.5, 2.0)).unwrap();
        check_curve(&l);
    }

    #[test]
    fn circle_bounding_box_is_exact_for_full_turn() {
        // Unit circle in the world xy plane: box must be exactly ±r in x, y
        // and flat in z.
        let c = Circle::new(Frame::world(), 1.5).unwrap();
        let bb = c.bounding_box(c.param_range());
        assert!((bb.min.x + 1.5).abs() < 1e-12);
        assert!((bb.max.x - 1.5).abs() < 1e-12);
        assert!((bb.min.y + 1.5).abs() < 1e-12);
        assert!((bb.max.y - 1.5).abs() < 1e-12);
        assert!(bb.min.z.abs() < 1e-12 && bb.max.z.abs() < 1e-12);
    }

    #[test]
    fn degenerate_radius_rejected() {
        assert!(Circle::new(Frame::world(), 0.0).is_err());
        assert!(Circle::new(Frame::world(), -1.0).is_err());
        assert!(Circle::new(Frame::world(), f64::NAN).is_err());
    }

    #[test]
    fn ellipse_conformance() {
        let e = Ellipse::new(tilted_frame(), 3.0, 1.25).unwrap();
        check_curve(&e);
    }

    #[test]
    fn ellipse_points_satisfy_implicit_equation() {
        let e = Ellipse::new(tilted_frame(), 3.0, 1.25).unwrap();
        for i in 0..64 {
            let t = i as f64 * core::f64::consts::TAU / 64.0;
            let l = e.frame().to_local(e.eval(t));
            let residual = (l.x / 3.0).powi(2) + (l.y / 1.25).powi(2) - 1.0;
            assert!(residual.abs() < 1e-12, "t = {t}: residual {residual:e}");
            assert!(l.z.abs() < 1e-12);
        }
    }

    #[test]
    fn ellipse_bounding_box_is_exact_for_full_turn() {
        let e = Ellipse::new(Frame::world(), 3.0, 1.25).unwrap();
        let bb = e.bounding_box(e.param_range());
        assert!((bb.min.x + 3.0).abs() < 1e-12 && (bb.max.x - 3.0).abs() < 1e-12);
        assert!((bb.min.y + 1.25).abs() < 1e-12 && (bb.max.y - 1.25).abs() < 1e-12);
        assert!(bb.min.z.abs() < 1e-12 && bb.max.z.abs() < 1e-12);
    }

    #[test]
    fn ellipse_bounding_box_is_exact_on_subrange() {
        // Tilted frame, partial arc: the box from the extreme-parameter
        // enumeration must match a dense sampling to sampling accuracy.
        let e = Ellipse::new(tilted_frame(), 3.0, 1.25).unwrap();
        let range = ParamRange::new(0.4, 2.9);
        let bb = e.bounding_box(range);
        let mut sampled = Aabb3::empty();
        const N: usize = 4096;
        for i in 0..=N {
            let t = range.lo + range.width() * i as f64 / N as f64;
            let p = e.eval(t);
            assert!(
                bb.inflated(1e-12).contains(p),
                "sample at t = {t} escapes box"
            );
            sampled = sampled.including(p);
        }
        // Exactness: dense sampling approaches the true box from inside.
        for (b, s) in [
            (bb.min.x, sampled.min.x),
            (bb.min.y, sampled.min.y),
            (bb.min.z, sampled.min.z),
            (bb.max.x, sampled.max.x),
            (bb.max.y, sampled.max.y),
            (bb.max.z, sampled.max.z),
        ] {
            assert!((b - s).abs() < 1e-5, "box bound {b} vs sampled {s}");
        }
    }

    #[test]
    fn ellipse_degenerate_inputs_rejected() {
        let f = Frame::world();
        assert!(Ellipse::new(f, 1.0, 2.0).is_err()); // r1 < r2
        assert!(Ellipse::new(f, 1.0, 0.0).is_err());
        assert!(Ellipse::new(f, f64::INFINITY, 1.0).is_err());
        assert!(Ellipse::new(f, 2.0, f64::NAN).is_err());
        assert!(Ellipse::new(f, 2.0, 2.0).is_ok()); // circle-as-ellipse allowed
    }
}
