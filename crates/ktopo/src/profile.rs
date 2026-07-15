//! Validated planar profile inputs for sheet and feature construction.
//!
//! A profile is geometry input, not B-rep topology. Validating it once keeps
//! sheet, extrude, revolve, and future region builders on the same robust
//! winding and degeneracy contract. The initial slice supports one simple
//! polygonal outer loop plus strictly contained, pairwise-disjoint polygonal
//! holes. Curve loops and nested material islands remain future work.

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
    holes: Vec<Vec<Point2>>,
}

impl PlanarProfile {
    /// Validate one polygonal outer boundary and normalize it counterclockwise.
    ///
    /// Repeated, sub-resolution, collinear-consecutive, non-finite,
    /// outside-size-box, and self-intersecting boundaries are rejected.
    pub fn from_polygon(frame: Frame, polygon: &[Point2]) -> Result<Self> {
        Self::from_polygon_with_holes(frame, polygon, &[])
    }

    /// Validate a polygonal outer boundary and zero or more polygonal holes.
    ///
    /// The outer boundary is normalized counterclockwise and holes clockwise.
    /// Every hole must lie strictly inside the outer boundary; holes may not
    /// touch, cross, overlap, or nest one another.
    pub fn from_polygon_with_holes(
        frame: Frame,
        polygon: &[Point2],
        holes: &[&[Point2]],
    ) -> Result<Self> {
        let outer = simple_polygon(&frame, polygon, true)?;
        let mut normalized_holes: Vec<Vec<Point2>> = Vec::with_capacity(holes.len());
        for hole in holes {
            let hole = simple_polygon(&frame, hole, false)?;
            if polygons_intersect(&outer, &hole)
                || hole
                    .iter()
                    .any(|&point| point_location(&outer, point) != PointLocation::Inside)
            {
                return Err(Error::InvalidGeometry {
                    reason: "planar profile hole must lie strictly inside the outer boundary",
                });
            }
            for other in &normalized_holes {
                if polygons_intersect(other, &hole)
                    || point_location(other, hole[0]) != PointLocation::Outside
                    || point_location(&hole, other[0]) != PointLocation::Outside
                {
                    return Err(Error::InvalidGeometry {
                        reason: "planar profile holes must be pairwise disjoint and unnested",
                    });
                }
            }
            normalized_holes.push(hole);
        }
        Ok(Self {
            frame,
            outer,
            holes: normalized_holes,
        })
    }

    /// Positioned plane in which the profile coordinates are expressed.
    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Counterclockwise simple outer polygon, without a repeated endpoint.
    pub fn outer(&self) -> &[Point2] {
        &self.outer
    }

    /// Clockwise simple polygonal holes in deterministic input order.
    pub fn holes(&self) -> &[Vec<Point2>] {
        &self.holes
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PointLocation {
    Outside,
    Boundary,
    Inside,
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

fn polygons_intersect(a: &[Point2], b: &[Point2]) -> bool {
    a.iter().enumerate().any(|(a_index, &a_start)| {
        let a_end = a[(a_index + 1) % a.len()];
        b.iter().enumerate().any(|(b_index, &b_start)| {
            let b_end = b[(b_index + 1) % b.len()];
            segments_intersect(a_start, a_end, b_start, b_end)
        })
    })
}

fn point_location(polygon: &[Point2], point: Point2) -> PointLocation {
    let mut winding = 0_i32;
    for (index, &start) in polygon.iter().enumerate() {
        let end = polygon[(index + 1) % polygon.len()];
        if point_on_segment(point, start, end) {
            return PointLocation::Boundary;
        }
        let side = orient2d(uv(start), uv(end), uv(point));
        if start.y <= point.y {
            if end.y > point.y && side == Orientation::Positive {
                winding += 1;
            }
        } else if end.y <= point.y && side == Orientation::Negative {
            winding -= 1;
        }
    }
    if winding == 0 {
        PointLocation::Outside
    } else {
        PointLocation::Inside
    }
}

fn simple_polygon(
    frame: &Frame,
    polygon: &[Point2],
    counterclockwise: bool,
) -> Result<Vec<Point2>> {
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
    if (winding == Orientation::Positive) != counterclockwise {
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

    #[test]
    fn holes_are_normalized_and_must_be_strictly_disjoint() {
        let outer = [
            Point2::new(-3.0, -3.0),
            Point2::new(3.0, -3.0),
            Point2::new(3.0, 3.0),
            Point2::new(-3.0, 3.0),
        ];
        let first = [
            Point2::new(-2.0, -1.0),
            Point2::new(-1.0, -1.0),
            Point2::new(-1.0, 1.0),
            Point2::new(-2.0, 1.0),
        ];
        let second = [
            Point2::new(1.0, -1.0),
            Point2::new(2.0, -1.0),
            Point2::new(2.0, 1.0),
            Point2::new(1.0, 1.0),
        ];
        let profile =
            PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&first, &second])
                .unwrap();
        assert_eq!(profile.holes().len(), 2);
        assert_eq!(
            point_location(profile.outer(), Point2::new(0.0, 0.0)),
            PointLocation::Inside
        );
        for hole in profile.holes() {
            assert_eq!(point_location(hole, hole[0]), PointLocation::Boundary);
            let leftmost = (0..hole.len())
                .min_by(|&a, &b| {
                    hole[a]
                        .x
                        .total_cmp(&hole[b].x)
                        .then(hole[a].y.total_cmp(&hole[b].y))
                })
                .unwrap();
            assert_eq!(
                orient2d(
                    uv(hole[(leftmost + hole.len() - 1) % hole.len()]),
                    uv(hole[leftmost]),
                    uv(hole[(leftmost + 1) % hole.len()]),
                ),
                Orientation::Negative
            );
        }

        let touching = [
            Point2::new(-3.0, -0.5),
            Point2::new(-2.0, -0.5),
            Point2::new(-2.0, 0.5),
            Point2::new(-3.0, 0.5),
        ];
        assert!(
            PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&touching]).is_err()
        );
        let nested = [
            Point2::new(-1.8, -0.5),
            Point2::new(-1.2, -0.5),
            Point2::new(-1.2, 0.5),
            Point2::new(-1.8, 0.5),
        ];
        assert!(
            PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&first, &nested])
                .is_err()
        );
    }
}
