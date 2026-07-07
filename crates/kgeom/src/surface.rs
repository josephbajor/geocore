//! The surface evaluator protocol and analytic surface classes.
//!
//! Every surface class implements [`Surface`]; the protocol is object-safe.
//! Analytic classes are exact — they are never approximated by NURBS
//! (spec §2, exactness commitment).
//!
//! Parameterization conventions (XT-aligned, re-verified at M3):
//! - [`Plane`]: `P(u,v) = origin + u·X + v·Y`, both directions unbounded.
//! - [`Cylinder`]: `P(u,v) = origin + r(cos u·X + sin u·Y) + v·Z`,
//!   `u ∈ [0, 2π)` periodic, `v` unbounded (axis distance).
//! - [`Cone`]: `P(u,v) = origin + (r + v·sin α)(cos u·X + sin u·Y) +
//!   v·cos α·Z` with half-angle `α ∈ (0, π/2)`; apex (degenerate iso-line)
//!   at `v = -r / sin α`.
//! - [`Sphere`]: `P(u,v) = origin + r(cos v(cos u·X + sin u·Y) + sin v·Z)`,
//!   `u ∈ [0, 2π)` periodic, `v ∈ [-π/2, π/2]`, poles degenerate.
//! - [`Torus`]: `P(u,v) = origin + (R + r cos v)(cos u·X + sin u·Y) +
//!   r sin v·Z`, both periodic, requires `R > r > 0` (open torus first;
//!   degenerate self-intersecting tori deferred).

use crate::aabb::Aabb3;
use crate::frame::Frame;
use crate::param::ParamRange;
use crate::vec::{Point3, Vec3};
use kcore::error::{Error, Result};
use kcore::math;

/// Position and partial derivatives of a surface at `(u, v)`, up to second
/// order. Entries above the requested order are zero.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceDerivs {
    /// Position `P(u,v)`.
    pub p: Point3,
    /// `∂P/∂u`.
    pub du: Vec3,
    /// `∂P/∂v`.
    pub dv: Vec3,
    /// `∂²P/∂u²`.
    pub duu: Vec3,
    /// `∂²P/∂u∂v`.
    pub duv: Vec3,
    /// `∂²P/∂v²`.
    pub dvv: Vec3,
}

/// One of the two parameter directions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    /// The `u` direction.
    U,
    /// The `v` direction.
    V,
}

/// A degenerate iso-parameter line: the entire iso-curve at `at` in
/// direction `dir` collapses to a single point (sphere pole, cone apex).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Degeneracy {
    /// Which parameter is held fixed at the degeneracy.
    pub dir: Dir,
    /// The degenerate parameter value.
    pub at: f64,
}

/// The uniform surface evaluator protocol (spec §L1).
pub trait Surface {
    /// Position at `(u, v)`.
    fn eval(&self, uv: [f64; 2]) -> Point3 {
        self.eval_derivs(uv, 0).p
    }

    /// Position and partial derivatives up to `order` (≤ 2).
    fn eval_derivs(&self, uv: [f64; 2], order: usize) -> SurfaceDerivs;

    /// Unit normal `du × dv / |du × dv|`, or `None` at a degeneracy.
    fn normal(&self, uv: [f64; 2]) -> Option<Vec3> {
        let d = self.eval_derivs(uv, 1);
        d.du.cross(d.dv).normalized()
    }

    /// Natural parameter ranges `[u, v]` (either may be unbounded).
    fn param_range(&self) -> [ParamRange; 2];

    /// Period per direction, if periodic.
    fn periodicity(&self) -> [Option<f64>; 2];

    /// Degenerate iso-lines (empty for most classes).
    fn degeneracies(&self) -> Vec<Degeneracy> {
        Vec::new()
    }

    /// Bounding box of the surface restricted to the given finite ranges;
    /// exact or tightly conservative per class.
    fn bounding_box(&self, range: [ParamRange; 2]) -> Aabb3;
}

/// An unbounded plane through `frame.origin` spanned by `frame.x`/`frame.y`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    frame: Frame,
}

impl Plane {
    /// Plane with the given positioning frame (normal is `frame.z`).
    pub fn new(frame: Frame) -> Plane {
        Plane { frame }
    }

    /// Positioning frame.
    pub fn frame(&self) -> &Frame {
        &self.frame
    }
}

impl Surface for Plane {
    fn eval_derivs(&self, uv: [f64; 2], order: usize) -> SurfaceDerivs {
        let mut d = SurfaceDerivs {
            p: self.frame.point_at(uv[0], uv[1], 0.0),
            ..Default::default()
        };
        if order >= 1 {
            d.du = self.frame.x();
            d.dv = self.frame.y();
        }
        d
    }

    fn param_range(&self) -> [ParamRange; 2] {
        [ParamRange::unbounded(), ParamRange::unbounded()]
    }

    fn periodicity(&self) -> [Option<f64>; 2] {
        [None, None]
    }

    fn bounding_box(&self, range: [ParamRange; 2]) -> Aabb3 {
        debug_assert!(range[0].is_finite() && range[1].is_finite());
        Aabb3::from_points(&[
            self.eval([range[0].lo, range[1].lo]),
            self.eval([range[0].lo, range[1].hi]),
            self.eval([range[0].hi, range[1].lo]),
            self.eval([range[0].hi, range[1].hi]),
        ])
    }
}

/// An infinite right circular cylinder about `frame.z`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cylinder {
    frame: Frame,
    radius: f64,
}

impl Cylinder {
    /// Cylinder of `radius` about the frame's z axis.
    pub fn new(frame: Frame, radius: f64) -> Result<Cylinder> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(Error::InvalidGeometry {
                reason: "cylinder radius must be positive and finite",
            });
        }
        Ok(Cylinder { frame, radius })
    }

    /// Positioning frame (axis is `frame.z`).
    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Radius.
    pub fn radius(&self) -> f64 {
        self.radius
    }
}

impl Surface for Cylinder {
    fn eval_derivs(&self, uv: [f64; 2], order: usize) -> SurfaceDerivs {
        let (sin, cos) = math::sincos(uv[0]);
        let radial = self.frame.x() * cos + self.frame.y() * sin;
        let tangential = self.frame.y() * cos - self.frame.x() * sin;
        let mut d = SurfaceDerivs {
            p: self.frame.origin() + radial * self.radius + self.frame.z() * uv[1],
            ..Default::default()
        };
        if order >= 1 {
            d.du = tangential * self.radius;
            d.dv = self.frame.z();
        }
        if order >= 2 {
            d.duu = -radial * self.radius;
            // duv and dvv are identically zero.
        }
        d
    }

    fn param_range(&self) -> [ParamRange; 2] {
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::unbounded(),
        ]
    }

    fn periodicity(&self) -> [Option<f64>; 2] {
        [Some(core::f64::consts::TAU), None]
    }

    fn bounding_box(&self, range: [ParamRange; 2]) -> Aabb3 {
        debug_assert!(range[0].is_finite() && range[1].is_finite());
        // The cylinder box over [u-range] × [v-range] is the box of the two
        // bounding circles (cross-sections at v.lo and v.hi). Reuse the
        // circle's exact arc box via the curve class.
        use crate::curve::{Circle, Curve};
        let mut bb = Aabb3::empty();
        for v in [range[1].lo, range[1].hi] {
            let center = self.frame.origin() + self.frame.z() * v;
            let section_frame = Frame::new(center, self.frame.z(), self.frame.x())
                .expect("cylinder frame is orthonormal");
            let circle = Circle::new(section_frame, self.radius).expect("radius already validated");
            bb = bb.union(circle.bounding_box(range[0]));
        }
        bb
    }
}

/// Exact bounding box for surfaces of revolution about `frame.z` whose
/// world-coordinate functions have the form
/// `P_i(u,v) = O_i + f(v)·(cos u·X_i + sin u·Y_i) + g(v)·Z_i`.
///
/// For each world coordinate `i`, extremes in `u` lie at
/// `u = atan2(Y_i, X_i) + kπ` (where `∂c_u/∂u = 0`), and extremes in `v` at
/// fixed `u` are supplied by `v_critical(i, u)` as the base of a `+kπ`
/// family — or `None` when `P_i` is monotone/linear in `v` (cone), in which
/// case the `v` endpoints suffice. Candidate products cover all corner,
/// edge, and interior extremes, so the resulting box is exact.
fn revolution_box(
    surface: &dyn Surface,
    frame: &Frame,
    range: [ParamRange; 2],
    v_critical: &dyn Fn(usize, f64) -> Option<f64>,
) -> Aabb3 {
    let pi = core::f64::consts::PI;
    let [ur, vr] = range;
    debug_assert!(ur.is_finite() && vr.is_finite());
    let x = frame.x().to_array();
    let y = frame.y().to_array();

    let family = |base: f64, r: ParamRange, out: &mut Vec<f64>| {
        let k_lo = ((r.lo - base) / pi).floor() as i64;
        let k_hi = ((r.hi - base) / pi).ceil() as i64;
        for k in k_lo..=k_hi {
            let t = base + pi * k as f64;
            if r.contains(t) {
                out.push(t);
            }
        }
    };

    let mut us = vec![ur.lo, ur.hi];
    for i in 0..3 {
        family(math::atan2(y[i], x[i]), ur, &mut us);
    }

    let mut bb = Aabb3::empty();
    for &u in &us {
        let mut vs = vec![vr.lo, vr.hi];
        for i in 0..3 {
            if let Some(base) = v_critical(i, u) {
                family(base, vr, &mut vs);
            }
        }
        for &v in &vs {
            bb = bb.including(surface.eval([u, v]));
        }
    }
    bb
}

/// An infinite right circular cone about `frame.z`:
/// `P(u,v) = origin + (r + v·sin α)(cos u·X + sin u·Y) + v·cos α·Z`
/// with radius `r > 0` at `v = 0` and half-angle `α ∈ (0, π/2)` (the cone
/// widens with increasing `v`). `u ∈ [0, 2π)` periodic; `v` unbounded and
/// arc-length along rulings. The apex is the degenerate iso-line at
/// `v = -r / sin α`, where the normal is undefined.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cone {
    frame: Frame,
    radius: f64,
    half_angle: f64,
}

impl Cone {
    /// Cone about the frame's z axis with radius `r > 0` at the frame origin
    /// plane and half-angle `α ∈ (0, π/2)` exclusive, both finite.
    pub fn new(frame: Frame, radius: f64, half_angle: f64) -> Result<Cone> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(Error::InvalidGeometry {
                reason: "cone radius must be positive and finite",
            });
        }
        if !half_angle.is_finite()
            || half_angle <= 0.0
            || half_angle >= core::f64::consts::FRAC_PI_2
        {
            return Err(Error::InvalidGeometry {
                reason: "cone half-angle must lie strictly between 0 and pi/2",
            });
        }
        Ok(Cone {
            frame,
            radius,
            half_angle,
        })
    }

    /// Positioning frame (axis is `frame.z`).
    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Radius of the cross-section at `v = 0`.
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// Half-angle between the axis and the rulings, in radians.
    pub fn half_angle(&self) -> f64 {
        self.half_angle
    }

    /// The `v` parameter of the apex (`-r / sin α`).
    pub fn apex_v(&self) -> f64 {
        -self.radius / math::sin(self.half_angle)
    }

    /// The apex point.
    pub fn apex(&self) -> Point3 {
        self.frame.origin() + self.frame.z() * (self.apex_v() * math::cos(self.half_angle))
    }
}

impl Surface for Cone {
    fn eval_derivs(&self, uv: [f64; 2], order: usize) -> SurfaceDerivs {
        let (sin_u, cos_u) = math::sincos(uv[0]);
        let (sin_a, cos_a) = math::sincos(self.half_angle);
        let rho = self.radius + uv[1] * sin_a;
        let radial = self.frame.x() * cos_u + self.frame.y() * sin_u;
        let tangential = self.frame.y() * cos_u - self.frame.x() * sin_u;
        let mut d = SurfaceDerivs {
            p: self.frame.origin() + radial * rho + self.frame.z() * (uv[1] * cos_a),
            ..Default::default()
        };
        if order >= 1 {
            d.du = tangential * rho;
            d.dv = radial * sin_a + self.frame.z() * cos_a;
        }
        if order >= 2 {
            d.duu = -radial * rho;
            d.duv = tangential * sin_a;
            // dvv is identically zero (rulings are straight).
        }
        d
    }

    fn param_range(&self) -> [ParamRange; 2] {
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::unbounded(),
        ]
    }

    fn periodicity(&self) -> [Option<f64>; 2] {
        [Some(core::f64::consts::TAU), None]
    }

    fn degeneracies(&self) -> Vec<Degeneracy> {
        vec![Degeneracy {
            dir: Dir::V,
            at: self.apex_v(),
        }]
    }

    fn bounding_box(&self, range: [ParamRange; 2]) -> Aabb3 {
        // P_i is linear in v for fixed u, so v endpoints suffice.
        revolution_box(self, &self.frame, range, &|_, _| None)
    }
}

/// A sphere about `frame.origin`:
/// `P(u,v) = origin + r(cos v(cos u·X + sin u·Y) + sin v·Z)` —
/// `u` is longitude, `[0, 2π)` periodic; `v` is latitude in `[-π/2, π/2]`
/// with degenerate poles at `v = ±π/2`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sphere {
    frame: Frame,
    radius: f64,
}

impl Sphere {
    /// Sphere of `radius` centered at the frame origin.
    pub fn new(frame: Frame, radius: f64) -> Result<Sphere> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(Error::InvalidGeometry {
                reason: "sphere radius must be positive and finite",
            });
        }
        Ok(Sphere { frame, radius })
    }

    /// Positioning frame (poles along `frame.z`).
    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Radius.
    pub fn radius(&self) -> f64 {
        self.radius
    }
}

impl Surface for Sphere {
    fn eval_derivs(&self, uv: [f64; 2], order: usize) -> SurfaceDerivs {
        let (sin_u, cos_u) = math::sincos(uv[0]);
        let (sin_v, cos_v) = math::sincos(uv[1]);
        let radial = self.frame.x() * cos_u + self.frame.y() * sin_u;
        let tangential = self.frame.y() * cos_u - self.frame.x() * sin_u;
        let r = self.radius;
        let mut d = SurfaceDerivs {
            p: self.frame.origin() + (radial * cos_v + self.frame.z() * sin_v) * r,
            ..Default::default()
        };
        if order >= 1 {
            d.du = tangential * (r * cos_v);
            d.dv = (self.frame.z() * cos_v - radial * sin_v) * r;
        }
        if order >= 2 {
            d.duu = -radial * (r * cos_v);
            d.duv = -tangential * (r * sin_v);
            d.dvv = -(radial * cos_v + self.frame.z() * sin_v) * r;
        }
        d
    }

    fn param_range(&self) -> [ParamRange; 2] {
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
        ]
    }

    fn periodicity(&self) -> [Option<f64>; 2] {
        [Some(core::f64::consts::TAU), None]
    }

    fn degeneracies(&self) -> Vec<Degeneracy> {
        vec![
            Degeneracy {
                dir: Dir::V,
                at: -core::f64::consts::FRAC_PI_2,
            },
            Degeneracy {
                dir: Dir::V,
                at: core::f64::consts::FRAC_PI_2,
            },
        ]
    }

    fn bounding_box(&self, range: [ParamRange; 2]) -> Aabb3 {
        let x = self.frame.x().to_array();
        let y = self.frame.y().to_array();
        let z = self.frame.z().to_array();
        revolution_box(self, &self.frame, range, &|i, u| {
            // ∂P_i/∂v = 0  ⇔  -sin v·c_u + cos v·Z_i = 0.
            let c_u = math::cos(u) * x[i] + math::sin(u) * y[i];
            Some(math::atan2(z[i], c_u))
        })
    }
}

/// An open torus about `frame.z`:
/// `P(u,v) = origin + (R + r cos v)(cos u·X + sin u·Y) + r sin v·Z` with
/// major radius `R` (spine circle) and minor radius `r` (tube), requiring
/// `R > r > 0`. Both directions are `[0, 2π)` periodic; no degeneracies.
/// Degenerate/self-intersecting tori (`R ≤ r`) are deferred.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Torus {
    frame: Frame,
    major_radius: f64,
    minor_radius: f64,
}

impl Torus {
    /// Torus about the frame's z axis; requires `R > r > 0`, both finite.
    pub fn new(frame: Frame, major_radius: f64, minor_radius: f64) -> Result<Torus> {
        if !major_radius.is_finite()
            || !minor_radius.is_finite()
            || minor_radius <= 0.0
            || major_radius <= minor_radius
        {
            return Err(Error::InvalidGeometry {
                reason: "torus radii must satisfy R > r > 0 and be finite",
            });
        }
        Ok(Torus {
            frame,
            major_radius,
            minor_radius,
        })
    }

    /// Positioning frame (spine circle in its xy plane).
    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Major (spine) radius.
    pub fn major_radius(&self) -> f64 {
        self.major_radius
    }

    /// Minor (tube) radius.
    pub fn minor_radius(&self) -> f64 {
        self.minor_radius
    }
}

impl Surface for Torus {
    fn eval_derivs(&self, uv: [f64; 2], order: usize) -> SurfaceDerivs {
        let (sin_u, cos_u) = math::sincos(uv[0]);
        let (sin_v, cos_v) = math::sincos(uv[1]);
        let radial = self.frame.x() * cos_u + self.frame.y() * sin_u;
        let tangential = self.frame.y() * cos_u - self.frame.x() * sin_u;
        let r = self.minor_radius;
        let rho = self.major_radius + r * cos_v;
        let mut d = SurfaceDerivs {
            p: self.frame.origin() + radial * rho + self.frame.z() * (r * sin_v),
            ..Default::default()
        };
        if order >= 1 {
            d.du = tangential * rho;
            d.dv = self.frame.z() * (r * cos_v) - radial * (r * sin_v);
        }
        if order >= 2 {
            d.duu = -radial * rho;
            d.duv = -tangential * (r * sin_v);
            d.dvv = -radial * (r * cos_v) - self.frame.z() * (r * sin_v);
        }
        d
    }

    fn param_range(&self) -> [ParamRange; 2] {
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ]
    }

    fn periodicity(&self) -> [Option<f64>; 2] {
        [Some(core::f64::consts::TAU), Some(core::f64::consts::TAU)]
    }

    fn bounding_box(&self, range: [ParamRange; 2]) -> Aabb3 {
        let x = self.frame.x().to_array();
        let y = self.frame.y().to_array();
        let z = self.frame.z().to_array();
        revolution_box(self, &self.frame, range, &|i, u| {
            // ∂P_i/∂v = 0  ⇔  -r sin v·c_u + r cos v·Z_i = 0.
            let c_u = math::cos(u) * x[i] + math::sin(u) * y[i];
            Some(math::atan2(z[i], c_u))
        })
    }
}

#[cfg(test)]
#[allow(clippy::disallowed_methods)] // tests may cross-check against platform libm
mod tests {
    use super::*;
    use crate::conformance::check_surface;

    fn tilted_frame() -> Frame {
        Frame::new(
            Point3::new(0.5, 1.0, -2.0),
            Vec3::new(1.0, 2.0, 2.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap()
    }

    #[test]
    fn plane_conformance() {
        check_surface(&Plane::new(tilted_frame()));
    }

    #[test]
    fn cylinder_conformance() {
        check_surface(&Cylinder::new(tilted_frame(), 1.75).unwrap());
    }

    #[test]
    fn cylinder_points_are_axis_equidistant() {
        let cyl = Cylinder::new(tilted_frame(), 1.75).unwrap();
        for i in 0..32 {
            let u = i as f64 * core::f64::consts::TAU / 32.0;
            let p = cyl.eval([u, 3.0]);
            let local = cyl.frame().to_local(p);
            assert!((Vec3::new(local.x, local.y, 0.0).norm() - 1.75).abs() < 1e-12);
            assert!((local.z - 3.0).abs() < 1e-12);
        }
    }

    #[test]
    fn cylinder_normal_points_radially() {
        let cyl = Cylinder::new(Frame::world(), 2.0).unwrap();
        let n = cyl.normal([0.0, 5.0]).unwrap();
        assert!((n - Vec3::new(1.0, 0.0, 0.0)).norm() < 1e-14);
    }

    #[test]
    fn cylinder_bounding_box_full_turn() {
        let cyl = Cylinder::new(Frame::world(), 2.0).unwrap();
        let bb = cyl.bounding_box([
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-1.0, 4.0),
        ]);
        assert!((bb.min.x + 2.0).abs() < 1e-12 && (bb.max.x - 2.0).abs() < 1e-12);
        assert!((bb.min.y + 2.0).abs() < 1e-12 && (bb.max.y - 2.0).abs() < 1e-12);
        assert!((bb.min.z + 1.0).abs() < 1e-12 && (bb.max.z - 4.0).abs() < 1e-12);
    }

    #[test]
    fn cone_conformance() {
        check_surface(&Cone::new(tilted_frame(), 1.2, 0.5).unwrap());
    }

    #[test]
    fn sphere_conformance() {
        check_surface(&Sphere::new(tilted_frame(), 2.0).unwrap());
    }

    #[test]
    fn torus_conformance() {
        check_surface(&Torus::new(tilted_frame(), 2.0, 0.5).unwrap());
    }

    #[test]
    fn cone_apex_iso_line_collapses_and_normal_vanishes() {
        let cone = Cone::new(tilted_frame(), 1.2, 0.5).unwrap();
        let v0 = cone.apex_v();
        for i in 0..32 {
            let u = i as f64 * core::f64::consts::TAU / 32.0;
            assert!(cone.eval([u, v0]).dist(cone.apex()) < 1e-12);
            assert!(cone.normal([u, v0]).is_none());
        }
        // Away from the apex the normal exists.
        assert!(cone.normal([1.0, v0 + 0.5]).is_some());
    }

    #[test]
    fn cone_rulings_are_straight_and_unit_speed() {
        // v is arc length along rulings: |dv| == 1 and dvv == 0.
        let cone = Cone::new(tilted_frame(), 1.2, 0.5).unwrap();
        let d = cone.eval_derivs([2.0, 3.0], 2);
        assert!((d.dv.norm() - 1.0).abs() < 1e-12);
        assert_eq!(d.dvv, Vec3::default());
    }

    #[test]
    fn sphere_points_lie_at_radius() {
        let s = Sphere::new(tilted_frame(), 2.0).unwrap();
        for i in 0..16 {
            for j in 0..16 {
                let u = i as f64 * core::f64::consts::TAU / 16.0;
                let v = -core::f64::consts::FRAC_PI_2 + core::f64::consts::PI * j as f64 / 15.0;
                let p = s.eval([u, v]);
                assert!((p.dist(s.frame().origin()) - 2.0).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn sphere_normal_is_radial_and_absent_at_poles() {
        let s = Sphere::new(tilted_frame(), 2.0).unwrap();
        let uv = [1.3, 0.4];
        let n = s.normal(uv).unwrap();
        let radial = (s.eval(uv) - s.frame().origin()).normalized().unwrap();
        assert!((n - radial).norm() < 1e-12 || (n + radial).norm() < 1e-12);
        assert!(s.normal([0.7, core::f64::consts::FRAC_PI_2]).is_none());
        assert!(s.normal([0.7, -core::f64::consts::FRAC_PI_2]).is_none());
    }

    #[test]
    fn torus_points_lie_on_tube_around_spine() {
        let t = Torus::new(tilted_frame(), 2.0, 0.5).unwrap();
        for i in 0..16 {
            for j in 0..16 {
                let u = i as f64 * core::f64::consts::TAU / 16.0;
                let v = j as f64 * core::f64::consts::TAU / 16.0;
                let l = t.frame().to_local(t.eval([u, v]));
                let spine_dist = (l.x.hypot(l.y) - 2.0).hypot(l.z);
                assert!((spine_dist - 0.5).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn sphere_bounding_box_full_ranges_is_exact() {
        let s = Sphere::new(tilted_frame(), 2.0).unwrap();
        let bb = s.bounding_box(s.param_range());
        let o = s.frame().origin();
        for (lo, hi, c) in [
            (bb.min.x, bb.max.x, o.x),
            (bb.min.y, bb.max.y, o.y),
            (bb.min.z, bb.max.z, o.z),
        ] {
            assert!((lo - (c - 2.0)).abs() < 1e-12);
            assert!((hi - (c + 2.0)).abs() < 1e-12);
        }
    }

    #[test]
    fn torus_bounding_box_full_ranges_is_exact() {
        let t = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
        let bb = t.bounding_box(t.param_range());
        assert!((bb.min.x + 2.5).abs() < 1e-12 && (bb.max.x - 2.5).abs() < 1e-12);
        assert!((bb.min.y + 2.5).abs() < 1e-12 && (bb.max.y - 2.5).abs() < 1e-12);
        assert!((bb.min.z + 0.5).abs() < 1e-12 && (bb.max.z - 0.5).abs() < 1e-12);
    }

    #[test]
    fn cone_bounding_box_full_turn() {
        let alpha = 0.5_f64;
        let cone = Cone::new(Frame::world(), 1.0, alpha).unwrap();
        let bb = cone.bounding_box([
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.0, 3.0),
        ]);
        let r_hi = 1.0 + 3.0 * alpha.sin();
        assert!((bb.min.x + r_hi).abs() < 1e-12 && (bb.max.x - r_hi).abs() < 1e-12);
        assert!((bb.min.y + r_hi).abs() < 1e-12 && (bb.max.y - r_hi).abs() < 1e-12);
        assert!(bb.min.z.abs() < 1e-12 && (bb.max.z - 3.0 * alpha.cos()).abs() < 1e-12);
    }

    /// Sub-range boxes from `revolution_box` must be exact: every dense
    /// sample is inside, and each bound is approached by the samples.
    fn assert_patch_box_exact(surface: &dyn Surface, range: [ParamRange; 2]) {
        let bb = surface.bounding_box(range);
        let mut sampled = Aabb3::empty();
        const N: usize = 512;
        for i in 0..=N {
            for j in 0..=N {
                let uv = [
                    range[0].lerp(i as f64 / N as f64),
                    range[1].lerp(j as f64 / N as f64),
                ];
                let p = surface.eval(uv);
                assert!(
                    bb.inflated(1e-12).contains(p),
                    "sample escapes box at {uv:?}"
                );
                sampled = sampled.including(p);
            }
        }
        for (b, s) in [
            (bb.min.x, sampled.min.x),
            (bb.min.y, sampled.min.y),
            (bb.min.z, sampled.min.z),
            (bb.max.x, sampled.max.x),
            (bb.max.y, sampled.max.y),
            (bb.max.z, sampled.max.z),
        ] {
            assert!((b - s).abs() < 1e-4, "box bound {b} vs dense sample {s}");
        }
    }

    #[test]
    fn subrange_bounding_boxes_are_exact() {
        let range = [ParamRange::new(0.4, 2.9), ParamRange::new(-0.3, 0.9)];
        assert_patch_box_exact(&Sphere::new(tilted_frame(), 2.0).unwrap(), range);
        assert_patch_box_exact(&Torus::new(tilted_frame(), 2.0, 0.5).unwrap(), range);
        assert_patch_box_exact(&Cone::new(tilted_frame(), 1.2, 0.5).unwrap(), range);
    }

    #[test]
    fn surface_degenerate_inputs_rejected() {
        let f = Frame::world();
        assert!(Cone::new(f, 0.0, 0.5).is_err());
        assert!(Cone::new(f, 1.0, 0.0).is_err());
        assert!(Cone::new(f, 1.0, core::f64::consts::FRAC_PI_2).is_err());
        assert!(Cone::new(f, 1.0, f64::NAN).is_err());
        assert!(Sphere::new(f, 0.0).is_err());
        assert!(Sphere::new(f, f64::INFINITY).is_err());
        assert!(Torus::new(f, 0.5, 0.5).is_err()); // R == r: degenerate
        assert!(Torus::new(f, 0.4, 0.5).is_err()); // R < r: self-intersecting
        assert!(Torus::new(f, 2.0, 0.0).is_err());
        assert!(Torus::new(f, f64::NAN, 0.5).is_err());
    }
}
