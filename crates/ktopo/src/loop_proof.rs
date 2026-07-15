//! Conservative whole-loop simplicity proofs.
//!
//! The first proof slice handles exact straight-segment loops on planes and
//! one-fin circle/ellipse loops. Segment intersection signs use the robust
//! kernel predicates; unsupported curved or nonlinear-chart compositions
//! remain indeterminate.

use crate::entity::{Edge, FinPcurve, LoopId, Sense};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::incidence::{
    IncidenceCertification, certify_edge_surface_incidence, certify_pcurve_incidence,
};
use crate::store::Store;
use kcore::error::Result;
use kcore::predicates::{Orientation, orient2d};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve::Curve;
use kgeom::param::ParamRange;
use kgeom::vec::{Point2, Point3};

/// Result of attempting to prove one loop simple.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LoopSimplicity {
    /// The supported exact representations are pairwise disjoint except at
    /// adjacent topological endpoints.
    Certified,
    /// A proper crossing, non-adjacent touch, or positive-length adjacent
    /// overlap was proven.
    SelfIntersecting,
    /// The loop contains a representation not covered by this proof slice.
    Indeterminate,
}

/// Result of attempting to prove one outer polygonal loop and its holes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LoopContainment {
    /// Exactly one loop contains every other loop, and hole loops are pairwise
    /// disjoint and unnested.
    Certified,
    /// At least one loop representation is outside this proof slice or the
    /// supported strict-containment relation was not established.
    Indeterminate,
}

#[derive(Debug, Clone, Copy)]
struct Segment2 {
    start: Point2,
    end: Point2,
}

/// Certify that `loop_id` has no self-intersection.
pub(crate) fn certify_loop_simplicity(store: &Store, loop_id: LoopId) -> Result<LoopSimplicity> {
    let loop_ = store.get(loop_id)?;
    if loop_.fins.len() == 1 {
        return certify_single_fin_loop(store, loop_.fins[0]);
    }
    if loop_.fins.len() < 2 {
        return Ok(LoopSimplicity::Indeterminate);
    }
    let mut tails = Vec::with_capacity(loop_.fins.len());
    for (index, &fin_id) in loop_.fins.iter().enumerate() {
        let Some(tail) = store.fin_tail(fin_id)? else {
            return Ok(LoopSimplicity::Indeterminate);
        };
        if let Some(previous) = tails.iter().position(|&seen| seen == tail) {
            let cyclically_adjacent =
                index == previous + 1 || previous == 0 && index + 1 == loop_.fins.len();
            if !cyclically_adjacent {
                return Ok(LoopSimplicity::SelfIntersecting);
            }
        }
        tails.push(tail);
    }
    let Some(segments) = planar_segment_ring(store, loop_id)? else {
        return Ok(LoopSimplicity::Indeterminate);
    };
    Ok(certify_segment_ring(&segments))
}

/// Certify strict outer/hole containment for exact polygonal loops on a plane.
pub(crate) fn certify_loop_containment(
    store: &Store,
    loop_ids: &[LoopId],
) -> Result<LoopContainment> {
    if loop_ids.len() < 2 {
        return Ok(LoopContainment::Certified);
    }
    let mut rings = Vec::with_capacity(loop_ids.len());
    for &loop_id in loop_ids {
        let Some(segments) = planar_segment_ring(store, loop_id)? else {
            return Ok(LoopContainment::Indeterminate);
        };
        if certify_segment_ring(&segments) != LoopSimplicity::Certified {
            return Ok(LoopContainment::Indeterminate);
        }
        rings.push(segments);
    }
    for first in 0..rings.len() {
        for second in first + 1..rings.len() {
            if rings[first].iter().any(|&left| {
                rings[second]
                    .iter()
                    .any(|&right| segments_intersect(left, right))
            }) {
                return Ok(LoopContainment::Indeterminate);
            }
        }
    }

    let mut containers = vec![Vec::new(); rings.len()];
    for inner in 0..rings.len() {
        let witness = rings[inner][0].start;
        for (outer, ring) in rings.iter().enumerate() {
            if inner != outer && point_location(witness, ring) == PointLocation::Inside {
                containers[inner].push(outer);
            }
        }
    }
    let outers: Vec<_> = containers
        .iter()
        .enumerate()
        .filter_map(|(index, containers)| containers.is_empty().then_some(index))
        .collect();
    let [outer] = outers.as_slice() else {
        return Ok(LoopContainment::Indeterminate);
    };
    if containers
        .iter()
        .enumerate()
        .all(|(index, containers)| index == *outer || containers.as_slice() == [*outer])
    {
        Ok(LoopContainment::Certified)
    } else {
        Ok(LoopContainment::Indeterminate)
    }
}

fn planar_segment_ring(store: &Store, loop_id: LoopId) -> Result<Option<Vec<Segment2>>> {
    let loop_ = store.get(loop_id)?;
    if loop_.fins.len() < 2 {
        return Ok(None);
    }
    let face = store.get(loop_.face)?;
    let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
        return Ok(None);
    };

    let mut segments = Vec::with_capacity(loop_.fins.len());
    for &fin_id in &loop_.fins {
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        if edge.tolerance.is_some() {
            return Ok(None);
        }
        let Some(range) = active_edge_range(edge, store) else {
            return Ok(None);
        };
        let tolerance = LINEAR_RESOLUTION;
        let segment = if let Some(pcurve) = fin.pcurve {
            if certify_pcurve_incidence(store, fin.edge, face.surface, pcurve, tolerance)?
                != IncidenceCertification::Certified
            {
                return Ok(None);
            }
            pcurve_line_segment(store, pcurve, edge, fin.sense, range)?
        } else {
            if certify_edge_surface_incidence(store, fin.edge, face.surface, tolerance)?
                != IncidenceCertification::Certified
            {
                return Ok(None);
            }
            model_line_segment(store, edge, fin.sense, range, plane.frame())?
        };
        let Some(segment) = segment else {
            return Ok(None);
        };
        segments.push(segment);
    }
    Ok(Some(segments))
}

fn active_edge_range(edge: &Edge, store: &Store) -> Option<ParamRange> {
    match edge.bounds {
        Some((lo, hi)) if lo.is_finite() && hi.is_finite() && lo < hi => {
            Some(ParamRange::new(lo, hi))
        }
        Some(_) => None,
        None => {
            let curve = store.get(edge.curve?).ok()?.as_curve();
            let range = curve.param_range();
            (range.is_finite() && range.lo < range.hi).then_some(range)
        }
    }
}

fn traversal_bounds(sense: Sense, range: ParamRange) -> (f64, f64) {
    if sense.is_forward() {
        (range.lo, range.hi)
    } else {
        (range.hi, range.lo)
    }
}

fn pcurve_line_segment(
    store: &Store,
    use_: FinPcurve,
    edge: &Edge,
    sense: Sense,
    range: ParamRange,
) -> Result<Option<Segment2>> {
    let Curve2dGeom::Line(curve) = store.get(use_.curve())? else {
        return Ok(None);
    };
    let (start, end) = traversal_bounds(sense, range);
    let periods = [None, None];
    let start = use_.evaluate_uv(curve, start, periods)?;
    let end = use_.evaluate_uv(curve, end, periods)?;
    // The caller has already required an exact edge; retaining `edge` in
    // the signature makes that precondition explicit at this boundary.
    if edge.curve.is_none() {
        return Ok(None);
    }
    Ok(Some(Segment2 { start, end }))
}

fn model_line_segment(
    store: &Store,
    edge: &Edge,
    sense: Sense,
    range: ParamRange,
    frame: &kgeom::frame::Frame,
) -> Result<Option<Segment2>> {
    let Some(curve_id) = edge.curve else {
        return Ok(None);
    };
    let CurveGeom::Line(line) = store.get(curve_id)? else {
        return Ok(None);
    };
    let (start, end) = traversal_bounds(sense, range);
    Ok(Some(Segment2 {
        start: plane_uv(frame, line.eval(start)),
        end: plane_uv(frame, line.eval(end)),
    }))
}

fn plane_uv(frame: &kgeom::frame::Frame, point: Point3) -> Point2 {
    let local = frame.to_local(point);
    Point2::new(local.x, local.y)
}

fn certify_single_fin_loop(store: &Store, fin_id: crate::entity::FinId) -> Result<LoopSimplicity> {
    let fin = store.get(fin_id)?;
    let edge = store.get(fin.edge)?;
    if edge.tolerance.is_some() || edge.curve.is_none() {
        return Ok(LoopSimplicity::Indeterminate);
    }
    let curve = store.get(edge.curve.expect("checked above"))?;
    if !matches!(curve, CurveGeom::Circle(_) | CurveGeom::Ellipse(_)) {
        return Ok(LoopSimplicity::Indeterminate);
    }
    let natural = curve.as_curve().param_range();
    let period = curve.as_curve().periodicity();
    let Some(period) = period else {
        return Ok(LoopSimplicity::Indeterminate);
    };
    let range = match edge.bounds {
        Some((lo, hi)) if lo.is_finite() && hi.is_finite() && lo < hi => ParamRange::new(lo, hi),
        Some(_) => return Ok(LoopSimplicity::Indeterminate),
        None if natural.is_finite() => natural,
        None => return Ok(LoopSimplicity::Indeterminate),
    };
    let slack = 256.0 * f64::EPSILON * (1.0 + range.lo.abs().max(range.hi.abs()).max(period.abs()));
    if range.width() > period + slack {
        return Ok(LoopSimplicity::Indeterminate);
    }
    Ok(LoopSimplicity::Certified)
}

fn certify_segment_ring(segments: &[Segment2]) -> LoopSimplicity {
    let scale = segments
        .iter()
        .flat_map(|segment| [segment.start, segment.end])
        .flat_map(|point| [point.x.abs(), point.y.abs()])
        .fold(0.0, f64::max);
    let join_tolerance = LINEAR_RESOLUTION.max(4096.0 * f64::EPSILON * (1.0 + scale));
    for (index, segment) in segments.iter().enumerate() {
        if !finite_point(segment.start)
            || !finite_point(segment.end)
            || segment.start.dist(segment.end) <= join_tolerance
        {
            return LoopSimplicity::Indeterminate;
        }
        let next = segments[(index + 1) % segments.len()];
        if segment.end.dist(next.start) > join_tolerance {
            return LoopSimplicity::Indeterminate;
        }
    }

    for left in 0..segments.len() {
        for right in left + 1..segments.len() {
            let adjacent = right == left + 1 || left == 0 && right + 1 == segments.len();
            if adjacent {
                if adjacent_overlap(segments[left], segments[right], join_tolerance) {
                    return LoopSimplicity::SelfIntersecting;
                }
            } else if segments_intersect(segments[left], segments[right]) {
                return LoopSimplicity::SelfIntersecting;
            }
        }
    }
    LoopSimplicity::Certified
}

fn finite_point(point: Point2) -> bool {
    point.x.is_finite() && point.y.is_finite()
}

fn adjacent_overlap(left: Segment2, right: Segment2, tolerance: f64) -> bool {
    if orient(left.start, left.end, right.start) != Orientation::Zero
        || orient(left.start, left.end, right.end) != Orientation::Zero
    {
        return false;
    }
    let (left_lo, left_hi, right_lo, right_hi) =
        if (left.end.x - left.start.x).abs() >= (left.end.y - left.start.y).abs() {
            ordered_intervals(left.start.x, left.end.x, right.start.x, right.end.x)
        } else {
            ordered_intervals(left.start.y, left.end.y, right.start.y, right.end.y)
        };
    left_hi.min(right_hi) - left_lo.max(right_lo) > tolerance
}

fn ordered_intervals(a: f64, b: f64, c: f64, d: f64) -> (f64, f64, f64, f64) {
    (a.min(b), a.max(b), c.min(d), c.max(d))
}

fn segments_intersect(left: Segment2, right: Segment2) -> bool {
    let o1 = orient(left.start, left.end, right.start);
    let o2 = orient(left.start, left.end, right.end);
    let o3 = orient(right.start, right.end, left.start);
    let o4 = orient(right.start, right.end, left.end);
    if o1.as_i8() * o2.as_i8() < 0 && o3.as_i8() * o4.as_i8() < 0 {
        return true;
    }
    (o1 == Orientation::Zero && point_on_segment(right.start, left))
        || (o2 == Orientation::Zero && point_on_segment(right.end, left))
        || (o3 == Orientation::Zero && point_on_segment(left.start, right))
        || (o4 == Orientation::Zero && point_on_segment(left.end, right))
}

fn orient(a: Point2, b: Point2, c: Point2) -> Orientation {
    orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y])
}

fn point_on_segment(point: Point2, segment: Segment2) -> bool {
    point.x >= segment.start.x.min(segment.end.x)
        && point.x <= segment.start.x.max(segment.end.x)
        && point.y >= segment.start.y.min(segment.end.y)
        && point.y <= segment.start.y.max(segment.end.y)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PointLocation {
    Outside,
    Boundary,
    Inside,
}

fn point_location(point: Point2, segments: &[Segment2]) -> PointLocation {
    let mut winding = 0_i32;
    for &segment in segments {
        if orient(segment.start, segment.end, point) == Orientation::Zero
            && point_on_segment(point, segment)
        {
            return PointLocation::Boundary;
        }
        let side = orient(segment.start, segment.end, point);
        if segment.start.y <= point.y {
            if segment.end.y > point.y && side == Orientation::Positive {
                winding += 1;
            }
        } else if segment.end.y <= point.y && side == Orientation::Negative {
            winding -= 1;
        }
    }
    if winding == 0 {
        PointLocation::Outside
    } else {
        PointLocation::Inside
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn segment(start: [f64; 2], end: [f64; 2]) -> Segment2 {
        Segment2 {
            start: Point2::new(start[0], start[1]),
            end: Point2::new(end[0], end[1]),
        }
    }

    #[test]
    fn robust_segment_ring_distinguishes_simple_crossing_and_overlap() {
        let square = [
            segment([0.0, 0.0], [1.0, 0.0]),
            segment([1.0, 0.0], [1.0, 1.0]),
            segment([1.0, 1.0], [0.0, 1.0]),
            segment([0.0, 1.0], [0.0, 0.0]),
        ];
        assert_eq!(certify_segment_ring(&square), LoopSimplicity::Certified);

        let bow_tie = [
            segment([0.0, 0.0], [1.0, 1.0]),
            segment([1.0, 1.0], [0.0, 1.0]),
            segment([0.0, 1.0], [1.0, 0.0]),
            segment([1.0, 0.0], [0.0, 0.0]),
        ];
        assert_eq!(
            certify_segment_ring(&bow_tie),
            LoopSimplicity::SelfIntersecting
        );

        let backtrack = [
            segment([0.0, 0.0], [1.0, 0.0]),
            segment([1.0, 0.0], [0.5, 0.0]),
            segment([0.5, 0.0], [0.5, 1.0]),
            segment([0.5, 1.0], [0.0, 0.0]),
        ];
        assert_eq!(
            certify_segment_ring(&backtrack),
            LoopSimplicity::SelfIntersecting
        );
    }
}
