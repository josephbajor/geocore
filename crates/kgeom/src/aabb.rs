//! Axis-aligned bounding boxes (2D parameter space and 3D model space).

use crate::vec::{Vec2, Vec3};

/// A 3D axis-aligned bounding box. The empty box has `min > max`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb3 {
    /// Componentwise minimum corner.
    pub min: Vec3,
    /// Componentwise maximum corner.
    pub max: Vec3,
}

impl Aabb3 {
    /// The empty box (identity for [`Aabb3::union`]).
    pub fn empty() -> Self {
        Aabb3 {
            min: Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            max: Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    /// Box containing a single point.
    pub fn from_point(p: Vec3) -> Self {
        Aabb3 { min: p, max: p }
    }

    /// Box containing all given points.
    pub fn from_points(points: &[Vec3]) -> Self {
        points.iter().fold(Self::empty(), |bb, &p| bb.including(p))
    }

    /// True if no point is contained.
    pub fn is_empty(self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    /// Smallest box containing `self` and `p`.
    pub fn including(self, p: Vec3) -> Self {
        Aabb3 {
            min: self.min.min(p),
            max: self.max.max(p),
        }
    }

    /// Smallest box containing both boxes.
    pub fn union(self, other: Aabb3) -> Self {
        Aabb3 {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// True if `p` lies inside (closed) bounds.
    pub fn contains(self, p: Vec3) -> bool {
        self.min.x <= p.x
            && p.x <= self.max.x
            && self.min.y <= p.y
            && p.y <= self.max.y
            && self.min.z <= p.z
            && p.z <= self.max.z
    }

    /// True if the boxes share at least one point.
    pub fn intersects(self, other: Aabb3) -> bool {
        self.min.x <= other.max.x
            && other.min.x <= self.max.x
            && self.min.y <= other.max.y
            && other.min.y <= self.max.y
            && self.min.z <= other.max.z
            && other.min.z <= self.max.z
    }

    /// Box grown outward by `margin` on every side.
    pub fn inflated(self, margin: f64) -> Self {
        debug_assert!(margin >= 0.0);
        if self.is_empty() {
            return self;
        }
        let m = Vec3::new(margin, margin, margin);
        Aabb3 {
            min: self.min - m,
            max: self.max + m,
        }
    }
}

/// A 2D axis-aligned bounding box (parameter space).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb2 {
    /// Componentwise minimum corner.
    pub min: Vec2,
    /// Componentwise maximum corner.
    pub max: Vec2,
}

impl Aabb2 {
    /// The empty box (identity for [`Aabb2::union`]).
    pub fn empty() -> Self {
        Aabb2 {
            min: Vec2::new(f64::INFINITY, f64::INFINITY),
            max: Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    /// Smallest box containing `self` and `p`.
    pub fn including(self, p: Vec2) -> Self {
        Aabb2 {
            min: Vec2::new(self.min.x.min(p.x), self.min.y.min(p.y)),
            max: Vec2::new(self.max.x.max(p.x), self.max.y.max(p.y)),
        }
    }

    /// Box containing all given points.
    pub fn from_points(points: &[Vec2]) -> Self {
        points.iter().fold(Self::empty(), |bb, &p| bb.including(p))
    }

    /// True if `p` lies inside (closed) bounds.
    pub fn contains(self, p: Vec2) -> bool {
        self.min.x <= p.x && p.x <= self.max.x && self.min.y <= p.y && p.y <= self.max.y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_box_behaves_as_union_identity() {
        let e = Aabb3::empty();
        assert!(e.is_empty());
        let b = Aabb3::from_point(Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(e.union(b), b);
        assert!(!e.contains(Vec3::new(0.0, 0.0, 0.0)));
    }

    #[test]
    fn inclusion_and_intersection() {
        let b = Aabb3::from_points(&[Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 2.0)]);
        assert!(b.contains(Vec3::new(1.0, 1.0, 1.0)));
        let c = Aabb3::from_points(&[Vec3::new(1.5, 1.5, 1.5), Vec3::new(3.0, 3.0, 3.0)]);
        assert!(b.intersects(c));
        let d = Aabb3::from_point(Vec3::new(5.0, 5.0, 5.0));
        assert!(!b.intersects(d));
    }

    #[test]
    fn inflation_pads_all_sides() {
        let b = Aabb3::from_point(Vec3::new(1.0, 1.0, 1.0)).inflated(0.5);
        assert!(b.contains(Vec3::new(0.6, 1.4, 1.0)));
        assert!(!b.contains(Vec3::new(0.4, 1.0, 1.0)));
    }
}
