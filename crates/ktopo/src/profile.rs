//! Validated planar profile inputs for sheet and feature construction.
//!
//! A profile is geometry input, not B-rep topology. Validating it once keeps
//! sheet, extrude, revolve, and future region builders on the same robust
//! winding and degeneracy contract. The initial slice supports one simple
//! polygonal outer loop; holes and curve loops remain explicit future work.

use kcore::error::{Error, Result};
use kcore::predicates::{Orientation, orient2d};
use kcore::tolerance::{LINEAR_RESOLUTION, check_in_size_box};
use kgeom::frame::Frame;
use kgeom::vec::Point2;

/// One validated simple polygon in a positioned plane.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanarProfile {
    frame: Frame,
    outer: Vec<Point2>,
}

impl PlanarProfile {
    /// Validate one polygonal outer boundary and normalize it counterclockwise.
    ///
    /// Repeated, sub-resolution, collinear-consecutive, non-finite,
    /// outside-size-box, and self-intersecting boundaries are rejected.
    pub fn from_polygon(frame: Frame, polygon: &[Point2]) -> Result<Self> {
        let outer = simple_ccw_polygon(&frame, polygon)?;
        Ok(Self { frame, outer })
    }

    /// Positioned plane in which the profile coordinates are expressed.
    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Counterclockwise simple outer polygon, without a repeated endpoint.
    pub fn outer(&self) -> &[Point2] {
        &self.outer
    }
}

fn uv(point: Point2) -> [f64; 2] {
    [point.x, point.y]
}

fn point_on_segment(point: Point2, start: Point2, end: Point2) -> bool {
    orient2d(uv(start), uv(end), uv(point)) == Orientation::Zero
        && point.x >= start.x.min(end.x)
        && point.x <= start.x.max(end.x)
        && point.y >= start.y.min(end.y)
        && point.y <= start.y.max(end.y)
}

fn segments_intersect(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    let ab_c = orient2d(uv(a), uv(b), uv(c));
    let ab_d = orient2d(uv(a), uv(b), uv(d));
    let cd_a = orient2d(uv(c), uv(d), uv(a));
    let cd_b = orient2d(uv(c), uv(d), uv(b));
    (ab_c != Orientation::Zero
        && ab_d != Orientation::Zero
        && ab_c != ab_d
        && cd_a != Orientation::Zero
        && cd_b != Orientation::Zero
        && cd_a != cd_b)
        || (ab_c == Orientation::Zero && point_on_segment(c, a, b))
        || (ab_d == Orientation::Zero && point_on_segment(d, a, b))
        || (cd_a == Orientation::Zero && point_on_segment(a, c, d))
        || (cd_b == Orientation::Zero && point_on_segment(b, c, d))
}

fn simple_ccw_polygon(frame: &Frame, polygon: &[Point2]) -> Result<Vec<Point2>> {
    if polygon.len() < 3 {
        return Err(Error::InvalidGeometry {
            reason: "planar profile polygon requires at least three vertices",
        });
    }
    let mut points = polygon.to_vec();
    for &point in &points {
        if !point.x.is_finite() || !point.y.is_finite() {
            return Err(Error::InvalidGeometry {
                reason: "planar profile polygon coordinates must be finite",
            });
        }
        check_in_size_box(frame.point_at(point.x, point.y, 0.0).to_array())?;
    }
    for index in 0..points.len() {
        let previous = points[(index + points.len() - 1) % points.len()];
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        if (next - current).norm() <= LINEAR_RESOLUTION {
            return Err(Error::InvalidGeometry {
                reason: "planar profile polygon has a zero-length boundary edge",
            });
        }
        if orient2d(uv(previous), uv(current), uv(next)) == Orientation::Zero {
            return Err(Error::InvalidGeometry {
                reason: "planar profile polygon has collinear consecutive vertices",
            });
        }
    }
    for first in 0..points.len() {
        let first_next = (first + 1) % points.len();
        for second in (first + 1)..points.len() {
            let second_next = (second + 1) % points.len();
            let adjacent = first == second || first_next == second || second_next == first;
            if !adjacent
                && segments_intersect(
                    points[first],
                    points[first_next],
                    points[second],
                    points[second_next],
                )
            {
                return Err(Error::InvalidGeometry {
                    reason: "planar profile polygon boundary self-intersects",
                });
            }
        }
    }
    let leftmost = (0..points.len())
        .min_by(|&a, &b| {
            points[a]
                .x
                .total_cmp(&points[b].x)
                .then(points[a].y.total_cmp(&points[b].y))
        })
        .expect("polygon is nonempty");
    let winding = orient2d(
        uv(points[(leftmost + points.len() - 1) % points.len()]),
        uv(points[leftmost]),
        uv(points[(leftmost + 1) % points.len()]),
    );
    if winding == Orientation::Negative {
        points.reverse();
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clockwise_polygon_is_normalized() {
        let profile = PlanarProfile::from_polygon(
            Frame::world(),
            &[
                Point2::new(0.0, 0.0),
                Point2::new(0.0, 1.0),
                Point2::new(1.0, 1.0),
                Point2::new(1.0, 0.0),
            ],
        )
        .unwrap();
        let points = profile.outer();
        let leftmost = points
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)))
            .unwrap()
            .0;
        assert_eq!(
            orient2d(
                uv(points[(leftmost + points.len() - 1) % points.len()]),
                uv(points[leftmost]),
                uv(points[(leftmost + 1) % points.len()]),
            ),
            Orientation::Positive
        );
    }
}
