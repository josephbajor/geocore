//! Midpoint-free interval primitives for semantic planar shell proofs.
//!
//! Every vector accepted here is a complete enclosure of the ideal value.
//! Strict comparisons are the only source of authority; a zero-containing
//! interval is deliberately inconclusive.

use kcore::interval::Interval;
use kcore::predicates::OrientedPlanePoints;

pub(super) type IntervalVec3 = [Interval; 3];

#[derive(Debug, Clone, Copy)]
pub(super) struct IntervalPlane {
    pub(super) origin: IntervalVec3,
    pub(super) normal: IntervalVec3,
}

pub(super) fn plane_from_witness(witness: OrientedPlanePoints) -> Option<IntervalPlane> {
    let origin = point(witness[0]);
    let u = sub(point(witness[1]), origin);
    let v = sub(point(witness[2]), origin);
    let normal = cross(u, v);
    (finite_vec(origin) && finite_vec(normal) && certified_nonzero(normal))
        .then_some(IntervalPlane { origin, normal })
}

pub(super) fn point(value: [f64; 3]) -> IntervalVec3 {
    value.map(Interval::point)
}

pub(super) fn sub(left: IntervalVec3, right: IntervalVec3) -> IntervalVec3 {
    core::array::from_fn(|axis| left[axis] - right[axis])
}

pub(super) fn cross(left: IntervalVec3, right: IntervalVec3) -> IntervalVec3 {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

pub(super) fn dot(left: IntervalVec3, right: IntervalVec3) -> Interval {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

pub(super) fn plane_value(plane: IntervalPlane, point: IntervalVec3) -> Interval {
    dot(plane.normal, sub(point, plane.origin))
}

pub(super) fn determinant(
    first: IntervalVec3,
    second: IntervalVec3,
    third: IntervalVec3,
) -> Interval {
    dot(first, cross(second, third))
}

pub(super) fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

pub(super) fn finite_vec(value: IntervalVec3) -> bool {
    value.into_iter().all(finite_interval)
}

pub(super) fn certified_nonzero(value: IntervalVec3) -> bool {
    value
        .into_iter()
        .any(|coordinate| matches!(coordinate.sign(), Some(-1) | Some(1)))
}

pub(super) fn exact_zero(value: IntervalVec3) -> bool {
    value
        .into_iter()
        .all(|coordinate| coordinate.lo() == 0.0 && coordinate.hi() == 0.0)
}

#[derive(Debug, Clone, Copy)]
pub(super) struct ProjectionBounds {
    min: f64,
    max: f64,
}

impl ProjectionBounds {
    fn from_points(points: &[IntervalVec3], axis: IntervalVec3) -> Option<Self> {
        let first = dot(axis, *points.first()?);
        if !finite_interval(first) {
            return None;
        }
        let mut bounds = Self {
            min: first.lo(),
            max: first.hi(),
        };
        for &point in &points[1..] {
            let projection = dot(axis, point);
            if !finite_interval(projection) {
                return None;
            }
            bounds.min = bounds.min.min(projection.lo());
            bounds.max = bounds.max.max(projection.hi());
        }
        Some(bounds)
    }
}

/// Certify a positive projection gap on one complete interval axis.
pub(super) fn strictly_separated(
    left: &[IntervalVec3],
    right: &[IntervalVec3],
    axis: IntervalVec3,
) -> Option<bool> {
    if !finite_vec(axis) {
        return None;
    }
    if exact_zero(axis) {
        return Some(false);
    }
    let left = ProjectionBounds::from_points(left, axis)?;
    let right = ProjectionBounds::from_points(right, axis)?;
    Some(left.max < right.min || right.max < left.min)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strict_projection_gap_uses_complete_boxes() {
        let axis = point([1.0, 0.0, 0.0]);
        let left = [[
            Interval::new(0.0, 1.0),
            Interval::point(0.0),
            Interval::point(0.0),
        ]];
        let separated = [[
            Interval::new(2.0, 3.0),
            Interval::point(0.0),
            Interval::point(0.0),
        ]];
        let uncertain = [[
            Interval::new(0.5, 2.0),
            Interval::point(0.0),
            Interval::point(0.0),
        ]];

        assert_eq!(strictly_separated(&left, &separated, axis), Some(true));
        assert_eq!(strictly_separated(&left, &uncertain, axis), Some(false));
    }

    #[test]
    fn interval_determinant_contains_independent_endpoint_oracle() {
        let first = [
            Interval::new(1.0, 2.0),
            Interval::new(-2.0, -1.0),
            Interval::new(3.0, 4.0),
        ];
        let second = [
            Interval::new(-1.0, 1.0),
            Interval::new(2.0, 3.0),
            Interval::new(0.0, 2.0),
        ];
        let third = [
            Interval::new(4.0, 5.0),
            Interval::new(-3.0, -2.0),
            Interval::new(1.0, 2.0),
        ];
        let enclosure = determinant(first, second, third);

        let boxes = [first, second, third];
        for mask in 0_u16..512 {
            let vectors: [[f64; 3]; 3] = core::array::from_fn(|vector| {
                core::array::from_fn(|axis| {
                    if mask & (1_u16 << (axis + 3 * vector)) == 0 {
                        boxes[vector][axis].lo()
                    } else {
                        boxes[vector][axis].hi()
                    }
                })
            });
            let [a, b, c] = vectors;
            let exact = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]);
            assert!(enclosure.contains(exact));
        }
    }
}
