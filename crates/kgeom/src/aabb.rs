//! Axis-aligned bounding boxes (2D parameter space and 3D model space).

use crate::vec::{Vec2, Vec3};
use kcore::interval::Interval;

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

    /// True if all six bounds are finite.
    pub fn is_finite(self) -> bool {
        self.min.x.is_finite()
            && self.min.y.is_finite()
            && self.min.z.is_finite()
            && self.max.x.is_finite()
            && self.max.y.is_finite()
            && self.max.z.is_finite()
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

    /// Conservative lower bound for squared Euclidean distance between boxes.
    ///
    /// Interval arithmetic rounds every coordinate gap and square outward, so
    /// the returned value never exceeds the true squared distance. Empty boxes
    /// have infinite separation from every represented point set.
    pub fn squared_distance_lower_bound(self, other: Aabb3) -> f64 {
        if self.is_empty() || other.is_empty() {
            return f64::INFINITY;
        }
        let mut squared = Interval::point(0.0);
        for (first_lo, first_hi, second_lo, second_hi) in [
            (self.min.x, self.max.x, other.min.x, other.max.x),
            (self.min.y, self.max.y, other.min.y, other.max.y),
            (self.min.z, self.max.z, other.min.z, other.max.z),
        ] {
            let gap = if first_hi < second_lo {
                Interval::point(second_lo) - Interval::point(first_hi)
            } else if second_hi < first_lo {
                Interval::point(first_lo) - Interval::point(second_hi)
            } else {
                Interval::point(0.0)
            };
            squared = squared + gap * gap;
        }
        squared.lo().max(0.0)
    }

    /// Box grown by `margin` on every side with outward-rounded bounds.
    pub fn inflated(self, margin: f64) -> Self {
        debug_assert!(margin >= 0.0);
        if self.is_empty() || margin == 0.0 {
            return self;
        }
        let m = Vec3::new(margin, margin, margin);
        let min = self.min - m;
        let max = self.max + m;
        Aabb3 {
            min: Vec3::new(min.x.next_down(), min.y.next_down(), min.z.next_down()),
            max: Vec3::new(max.x.next_up(), max.y.next_up(), max.z.next_up()),
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

    /// True if no point is contained.
    pub fn is_empty(self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y
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

    /// Smallest box containing both boxes.
    pub fn union(self, other: Aabb2) -> Self {
        Aabb2 {
            min: Vec2::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
            max: Vec2::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
        }
    }

    /// True if `p` lies inside (closed) bounds.
    pub fn contains(self, p: Vec2) -> bool {
        self.min.x <= p.x && p.x <= self.max.x && self.min.y <= p.y && p.y <= self.max.y
    }

    /// Box grown by `margin` on every side with outward-rounded bounds.
    pub fn inflated(self, margin: f64) -> Self {
        debug_assert!(margin >= 0.0);
        if self.is_empty() || margin == 0.0 {
            return self;
        }
        let amount = Vec2::new(margin, margin);
        let min = self.min - amount;
        let max = self.max + amount;
        Self {
            min: Vec2::new(min.x.next_down(), min.y.next_down()),
            max: Vec2::new(max.x.next_up(), max.y.next_up()),
        }
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

        let e = Aabb2::empty();
        assert!(e.is_empty());
        let b = Aabb2::from_points(&[Vec2::new(-1.0, 2.0), Vec2::new(3.0, 4.0)]);
        assert_eq!(e.union(b), b);
        assert!(!b.is_empty());
    }

    #[test]
    fn inclusion_and_intersection() {
        let b = Aabb3::from_points(&[Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 2.0)]);
        assert!(b.contains(Vec3::new(1.0, 1.0, 1.0)));
        let c = Aabb3::from_points(&[Vec3::new(1.5, 1.5, 1.5), Vec3::new(3.0, 3.0, 3.0)]);
        assert!(b.intersects(c));
        let d = Aabb3::from_point(Vec3::new(5.0, 5.0, 5.0));
        assert!(!b.intersects(d));
        assert!(b.is_finite());
        assert!(!Aabb3::empty().is_finite());
    }

    #[test]
    fn inflation_pads_all_sides() {
        let b = Aabb3::from_point(Vec3::new(1.0, 1.0, 1.0)).inflated(0.5);
        assert!(b.contains(Vec3::new(0.6, 1.4, 1.0)));
        assert!(!b.contains(Vec3::new(0.4, 1.0, 1.0)));
        assert_eq!(b.inflated(0.0), b);

        let b = Aabb2::from_points(&[Vec2::new(1.0, 1.0)]).inflated(0.5);
        assert!(b.contains(Vec2::new(0.6, 1.4)));
        assert!(!b.contains(Vec2::new(0.4, 1.0)));
    }

    #[test]
    fn squared_distance_lower_bound_is_euclidean_outward_and_symmetric() {
        let origin = Aabb3::from_point(Vec3::new(0.0, 0.0, 0.0));
        let diagonal = Aabb3::from_point(Vec3::new(0.75, 0.75, 0.0));
        let lower = origin.squared_distance_lower_bound(diagonal);
        assert!(lower <= 0.75 * 0.75 + 0.75 * 0.75);
        assert!(lower > 1.0);
        assert_eq!(lower, diagonal.squared_distance_lower_bound(origin));
        assert_eq!(origin.squared_distance_lower_bound(origin), 0.0);
        assert_eq!(
            Aabb3::empty().squared_distance_lower_bound(origin),
            f64::INFINITY
        );
    }
}
