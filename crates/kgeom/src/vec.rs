//! 2D and 3D vector/point types.
//!
//! One type serves for both points and vectors (`Vec3` aliased as `Point3`),
//! matching kernel-internal usage where the distinction adds ceremony without
//! catching real bugs; the topology layer's typed handles are where identity
//! discipline lives.

use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

/// A 3D vector or point of `f64`s.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec3 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
}

/// A point in 3D model space (alias of [`Vec3`]).
pub type Point3 = Vec3;

/// The zero vector.
pub const ZERO3: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};

impl Vec3 {
    /// Construct from components.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }

    /// Dot product.
    pub fn dot(self, rhs: Vec3) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// Cross product.
    pub fn cross(self, rhs: Vec3) -> Vec3 {
        Vec3 {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }

    /// Euclidean length.
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Squared length (no sqrt; prefer for comparisons).
    pub fn norm_sq(self) -> f64 {
        self.dot(self)
    }

    /// Distance to another point.
    pub fn dist(self, rhs: Vec3) -> f64 {
        (self - rhs).norm()
    }

    /// Unit vector in this direction, or `None` if the length is
    /// indistinguishable from zero at session resolution.
    pub fn normalized(self) -> Option<Vec3> {
        if !self.x.is_finite() || !self.y.is_finite() || !self.z.is_finite() {
            return None;
        }
        let n = self.norm();
        if n <= kcore::tolerance::LINEAR_RESOLUTION {
            None
        } else if n.is_finite() {
            // Preserve the established ordinary-input bits.
            Some(self / n)
        } else {
            // The components are finite, so an infinite norm can only come
            // from squared-length overflow. Scale first; the resulting norm
            // is in [1, sqrt(3)] and cannot overflow or underflow.
            let scale = self.x.abs().max(self.y.abs()).max(self.z.abs());
            let scaled = self / scale;
            let scaled_norm = scaled.norm();
            (scaled_norm.is_finite() && scaled_norm > 0.0).then_some(scaled / scaled_norm)
        }
    }

    /// Component array `[x, y, z]`.
    pub fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// Construct from a component array.
    pub fn from_array(a: [f64; 3]) -> Self {
        Vec3::new(a[0], a[1], a[2])
    }

    /// Component-wise minimum.
    pub fn min(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x.min(rhs.x), self.y.min(rhs.y), self.z.min(rhs.z))
    }

    /// Component-wise maximum.
    pub fn max(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x.max(rhs.x), self.y.max(rhs.y), self.z.max(rhs.z))
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}
impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Vec3) {
        *self = *self + rhs;
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}
impl SubAssign for Vec3 {
    fn sub_assign(&mut self, rhs: Vec3) {
        *self = *self - rhs;
    }
}
impl Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Mul<Vec3> for f64 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}
impl Div<f64> for Vec3 {
    type Output = Vec3;
    fn div(self, s: f64) -> Vec3 {
        Vec3::new(self.x / s, self.y / s, self.z / s)
    }
}
impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

/// A 2D vector or point of `f64`s (parameter space, sketch plane).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
}

/// A point in 2D parameter space (alias of [`Vec2`]).
pub type Point2 = Vec2;

impl Vec2 {
    /// Construct from components.
    pub const fn new(x: f64, y: f64) -> Self {
        Vec2 { x, y }
    }

    /// Dot product.
    pub fn dot(self, rhs: Vec2) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }

    /// 2D cross product (z-component of the 3D cross).
    pub fn cross(self, rhs: Vec2) -> f64 {
        self.x * rhs.y - self.y * rhs.x
    }

    /// Euclidean length.
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Squared length.
    pub fn norm_sq(self) -> f64 {
        self.dot(self)
    }

    /// Distance to another point.
    pub fn dist(self, rhs: Vec2) -> f64 {
        (self - rhs).norm()
    }

    /// Counterclockwise perpendicular `(-y, x)`.
    pub fn perp(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.x + rhs.x, self.y + rhs.y)
    }
}
impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.x - rhs.x, self.y - rhs.y)
    }
}
impl Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, s: f64) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}
impl Mul<Vec2> for f64 {
    type Output = Vec2;
    fn mul(self, v: Vec2) -> Vec2 {
        v * self
    }
}
impl Div<f64> for Vec2 {
    type Output = Vec2;
    fn div(self, s: f64) -> Vec2 {
        Vec2::new(self.x / s, self.y / s)
    }
}
impl Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cross_products_are_right_handed() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(x.cross(y), Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(Vec2::new(1.0, 0.0).cross(Vec2::new(0.0, 1.0)), 1.0);
    }

    #[test]
    fn normalization_rejects_resolution_scale_vectors() {
        assert!(Vec3::new(1e-9, 0.0, 0.0).normalized().is_none());
        let n = Vec3::new(3.0, 4.0, 0.0).normalized().unwrap();
        assert!((n.norm() - 1.0).abs() < 1e-15);
    }

    #[test]
    fn normalization_preserves_ordinary_bits_and_accepts_finite_overflow_scales() {
        let ordinary = Vec3::new(1.0, -0.5, 0.25);
        assert_eq!(ordinary.normalized(), Some(ordinary / ordinary.norm()));

        let scaled = Vec3::new(2.0_f64.powi(700), -2.0_f64.powi(699), 2.0_f64.powi(698));
        assert_eq!(scaled.normalized(), ordinary.normalized());

        let maximum = Vec3::new(f64::MAX, f64::MAX, 0.0).normalized().unwrap();
        assert_eq!(maximum.x, maximum.y);
        assert_eq!(maximum.z, 0.0);
        assert!((maximum.norm() - 1.0).abs() < 2.0 * f64::EPSILON);
    }

    #[test]
    fn normalization_rejects_nonfinite_and_respects_the_euclidean_floor() {
        assert!(Vec3::new(f64::NAN, 1.0, 0.0).normalized().is_none());
        assert!(Vec3::new(f64::INFINITY, 0.0, 0.0).normalized().is_none());
        assert!(
            Vec3::new(f64::MIN_POSITIVE, 0.0, 0.0)
                .normalized()
                .is_none()
        );

        let floor = kcore::tolerance::LINEAR_RESOLUTION;
        assert!(
            Vec3::new(floor.next_down(), 0.0, 0.0)
                .normalized()
                .is_none()
        );
        assert!(Vec3::new(floor.next_up(), 0.0, 0.0).normalized().is_some());
        assert!(
            Vec3::new(0.8 * floor, 0.8 * floor, 0.0)
                .normalized()
                .is_some()
        );
    }
}
