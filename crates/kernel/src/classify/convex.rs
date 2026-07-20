//! Exact and interval-certified support tests for convex planar trim loops.

use kcore::interval::Interval;
use kcore::predicates::{Orientation, orient2d};
use kgeom::curve::Circle;

use super::{
    PreparedFace, PreparedLoop, WindingOutcome, polygon_orientation2d_iter, project,
    project_intervals,
};

/// Source-size-exact conservative bound for orientation plus support tests.
pub(super) fn support_work(vertex_count: usize) -> Option<u64> {
    let vertices = u64::try_from(vertex_count).ok()?;
    vertices.checked_mul(vertices)
}

/// Prove one loop is the strict boundary of its projected convex hull.
///
/// Every directed edge must support every nonincident vertex on the same
/// exact side. This rejects concavity, collinearity, and self-intersecting
/// star order rather than inferring convexity from turn signs alone.
pub(super) fn certify_strict_polygon(
    polygon: &PreparedLoop,
    drop_axis: usize,
) -> Option<Orientation> {
    certify_projected_polygon(polygon.vertices.len(), |index| {
        project(polygon.vertices[index].point, drop_axis)
    })
}

fn certify_projected_polygon(
    vertex_count: usize,
    point: impl Fn(usize) -> [f64; 2],
) -> Option<Orientation> {
    if vertex_count < 3 {
        return None;
    }
    let orientation = polygon_orientation2d_iter((0..vertex_count).map(&point));
    if orientation == Orientation::Zero {
        return None;
    }
    for edge in 0..vertex_count {
        let next = (edge + 1) % vertex_count;
        let start = point(edge);
        let end = point(next);
        for candidate in 0..vertex_count {
            if candidate != edge
                && candidate != next
                && orient2d(start, end, point(candidate)) != orientation
            {
                return None;
            }
        }
    }
    Some(orientation)
}

/// Classify a certified line/plane hit against one proven convex polygon.
pub(super) fn polygon_parity_at_line(
    face: &PreparedFace,
    point: [f64; 3],
    direction: [f64; 3],
    t: Interval,
) -> WindingOutcome {
    let ([ring], Some(orientation)) = (face.loops.as_slice(), face.convex_orientation) else {
        return WindingOutcome::Gap;
    };
    let hit = project_intervals(
        core::array::from_fn(|axis| {
            Interval::point(point[axis]) + Interval::point(direction[axis]) * t
        }),
        face.drop_axis,
    );
    for index in 0..ring.vertices.len() {
        let start = project(ring.vertices[index].point, face.drop_axis);
        let end = project(
            ring.vertices[(index + 1) % ring.vertices.len()].point,
            face.drop_axis,
        );
        let determinant = (Interval::point(end[0]) - Interval::point(start[0]))
            * (hit[1] - Interval::point(start[1]))
            - (Interval::point(end[1]) - Interval::point(start[1]))
                * (hit[0] - Interval::point(start[0]));
        match orientation {
            Orientation::Positive if determinant.lo() > 0.0 => {}
            Orientation::Positive if determinant.hi() < 0.0 => return WindingOutcome::Outside,
            Orientation::Negative if determinant.hi() < 0.0 => {}
            Orientation::Negative if determinant.lo() > 0.0 => return WindingOutcome::Outside,
            _ => return WindingOutcome::Gap,
        }
    }
    WindingOutcome::Inside
}

/// Prove that one full circle is a strict hole of one convex polygon.
pub(super) fn certify_circle_hole(
    polygon: &PreparedLoop,
    circle: Circle,
    drop_axis: usize,
    orientation: Orientation,
) -> bool {
    let vertices = &polygon.vertices;
    let center = project(circle.frame().origin().to_array(), drop_axis);
    let x = project(circle.frame().x().to_array(), drop_axis);
    let y = project(circle.frame().y().to_array(), drop_axis);
    for index in 0..vertices.len() {
        let start = project(vertices[index].point, drop_axis);
        let end = project(vertices[(index + 1) % vertices.len()].point, drop_axis);
        if orient2d(start, end, center) != orientation {
            return false;
        }
        let edge = [
            Interval::point(end[0]) - Interval::point(start[0]),
            Interval::point(end[1]) - Interval::point(start[1]),
        ];
        let center_offset = [
            Interval::point(center[0]) - Interval::point(start[0]),
            Interval::point(center[1]) - Interval::point(start[1]),
        ];
        let mut center_det = edge[0] * center_offset[1] - edge[1] * center_offset[0];
        let x_det = edge[0] * Interval::point(x[1]) - edge[1] * Interval::point(x[0]);
        let y_det = edge[0] * Interval::point(y[1]) - edge[1] * Interval::point(y[0]);
        let Some(amplitude) = (x_det.square() + y_det.square()).sqrt() else {
            return false;
        };
        let amplitude = amplitude * Interval::point(circle.radius());
        if orientation == Orientation::Negative {
            center_det = -center_det;
        }
        if center_det.lo() <= amplitude.hi() {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_support_order_accepts_convex_loops_and_rejects_fallback_shapes() {
        let square = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        assert_eq!(
            certify_projected_polygon(square.len(), |index| square[index]),
            Some(Orientation::Positive)
        );

        let concave = [[0.0, 0.0], [2.0, 0.0], [1.0, 0.5], [2.0, 2.0], [0.0, 2.0]];
        assert_eq!(
            certify_projected_polygon(concave.len(), |index| concave[index]),
            None
        );

        let star = [
            [0.0, 2.0],
            [1.2, -1.6],
            [-1.9, 0.6],
            [1.9, 0.6],
            [-1.2, -1.6],
        ];
        assert_eq!(
            certify_projected_polygon(star.len(), |index| star[index]),
            None
        );
    }
}
