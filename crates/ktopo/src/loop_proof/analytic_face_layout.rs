//! Certified interval layouts for multi-loop analytic faces.
//!
//! This proof is deliberately representation-driven rather than
//! constructor-driven. Plane faces may use a convex exact Line2d outer or a
//! full Circle2d outer and any number of bounded Line2d/Circle2d holes whose
//! complete interval envelopes are strictly contained and pairwise separated.
//! Cylinder faces use the intrinsic `S1 x R` topology: two whole-period
//! horizontal rings bound an open axial band, and any number of contractible
//! analytic holes must lie strictly in that band and be pairwise separated
//! modulo the authored u period.
//!
//! R1/R6 are preserved throughout. Every accepted relation encloses complete
//! active carriers with outward intervals; no sampled chord stands in for an
//! arc, tolerant joins are consumed only through the independently certified
//! bounded-loop proof, and contact, overlap, nesting, unsupported winding, or
//! unresolved arithmetic fails closed.

use super::{
    BoundedLoopSpan, LoopSimplicity, Segment2, certify_bounded_analytic_loop_orientation,
    certify_convex_polygon_circle_containment, certify_loop_orientation, certify_loop_simplicity,
    prepare_bounded_analytic_loop, strict_planar_ring,
};
use crate::entity::{EdgeId, LoopId};
use crate::geom::{Curve2dGeom, SurfaceGeom};
use crate::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use crate::store::Store;
use kcore::error::{Error, Result};
use kcore::expansion;
use kcore::interval::Interval;
use kcore::math;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::predicates::{Orientation, orient2d};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve2d::Circle2d;
use kgeom::vec::Point2;

/// Cumulative structural work for one Full face-loop containment proof.
pub(crate) const FACE_LOOP_CONTAINMENT_WORK: StageId =
    match StageId::new("ktopo.check.face-loop-containment-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid face-loop containment stage"),
    };

const DEFAULT_FACE_LOOP_CONTAINMENT_WORK: u64 = 1_048_576;
const MAX_CIRCLE_CHUNKS: usize = 64;
const CIRCLE_CHUNK_WIDTH: f64 = 0.125;

/// Default Full-check allowance for analytic face-loop layouts.
pub(crate) fn face_loop_containment_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        FACE_LOOP_CONTAINMENT_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_FACE_LOOP_CONTAINMENT_WORK,
    )])
    .expect("valid face-loop containment budget")
}

/// Exact structural work charged before allocating proof scratch.
pub(crate) fn face_loop_containment_work(loop_count: usize, fin_count: usize) -> Option<u64> {
    let loops = u64::try_from(loop_count).ok()?;
    let fins = u64::try_from(fin_count).ok()?;
    let loop_pairs = loops.checked_mul(loops.saturating_sub(1))?.checked_div(2)?;
    // One loop record, one fin record plus the maximum complete interval
    // cover for each analytic span, every unordered peer comparison, and one
    // candidate-outer comparison slot per ordered loop pair.
    loops
        .checked_add(fins.checked_mul(1 + MAX_CIRCLE_CHUNKS as u64)?)?
        .checked_add(loop_pairs)?
        .checked_add(loops.checked_mul(loops)?)
}

pub(super) fn charge_face_loop_containment_work(
    scope: &mut OperationScope<'_, '_>,
    loop_count: usize,
    fin_count: usize,
) -> Result<()> {
    let amount = face_loop_containment_work(loop_count, fin_count).ok_or({
        Error::InvalidGeometry {
            reason: "face-loop containment work overflow",
        }
    })?;
    scope
        .ledger_mut()
        .charge(FACE_LOOP_CONTAINMENT_WORK, amount)
        .map_err(Error::from)
}

#[derive(Debug, Clone, Copy)]
struct Box2 {
    u: Interval,
    v: Interval,
}

#[derive(Debug, Clone, Copy)]
struct FullCircle {
    circle: Circle2d,
    center: Point2,
}

#[derive(Debug, Clone, Copy)]
struct LoopEnvelope {
    bounds: Box2,
    full_circle: Option<FullCircle>,
}

#[derive(Debug, Clone, Copy)]
struct PeriodicRing {
    edge: EdgeId,
    height: Interval,
}

/// Attempt the representation-general analytic layout proof.
pub(super) fn certify_analytic_face_layout(
    store: &Store,
    face_id: crate::entity::FaceId,
) -> Result<bool> {
    let face = store.get(face_id)?;
    match store.get(face.surface)? {
        SurfaceGeom::Plane(_) => certify_plane_layout(store, face_id),
        SurfaceGeom::Cylinder(_) => certify_cylinder_layout(store, face_id),
        _ => Ok(false),
    }
}

fn certify_plane_layout(store: &Store, face_id: crate::entity::FaceId) -> Result<bool> {
    let face = store.get(face_id)?;
    if face.loops.len() < 2 {
        return Ok(true);
    }
    let mut envelopes = Vec::with_capacity(face.loops.len());
    for &loop_id in &face.loops {
        let Some(envelope) = prepare_plane_envelope(store, face_id, loop_id)? else {
            return Ok(false);
        };
        envelopes.push(envelope);
    }

    let mut outer = None;
    for candidate in 0..face.loops.len() {
        let contains_all = if let Some(circle) = envelopes[candidate].full_circle {
            envelopes.iter().enumerate().all(|(index, inner)| {
                index == candidate || circle_contains_envelope(circle, *inner)
            })
        } else if let Some((segments, orientation)) =
            strict_planar_ring(store, face.loops[candidate])?
        {
            convex_polygon(&segments, orientation).is_some()
                && envelopes.iter().enumerate().all(|(index, inner)| {
                    index == candidate || polygon_contains_envelope(&segments, orientation, *inner)
                })
        } else {
            false
        };
        if contains_all && outer.replace(candidate).is_some() {
            return Ok(false);
        }
    }
    let Some(outer) = outer else {
        return Ok(false);
    };
    for left in 0..envelopes.len() {
        if left == outer {
            continue;
        }
        for right in left + 1..envelopes.len() {
            if right != outer && !plane_envelopes_separated(envelopes[left], envelopes[right]) {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

fn certify_cylinder_layout(store: &Store, face_id: crate::entity::FaceId) -> Result<bool> {
    let face = store.get(face_id)?;
    let SurfaceGeom::Cylinder(_) = store.get(face.surface)? else {
        return Ok(false);
    };
    let period = core::f64::consts::TAU;
    let mut rings = Vec::with_capacity(2);
    let mut holes = Vec::with_capacity(face.loops.len().saturating_sub(2));
    for &loop_id in &face.loops {
        if let Some(ring) = prepare_periodic_ring(store, face_id, loop_id, period)? {
            rings.push(ring);
            continue;
        }
        let Some(envelope) = prepare_bounded_envelope(store, face_id, loop_id)? else {
            return Ok(false);
        };
        holes.push(envelope);
    }
    let [first, second] = rings.as_slice() else {
        return Ok(false);
    };
    if first.edge == second.edge {
        return Ok(false);
    }
    let hole_bounds = holes.iter().map(|hole| hole.bounds).collect::<Vec<_>>();
    Ok(certify_periodic_band_layout(
        [first.height, second.height],
        &hole_bounds,
        period,
    ))
}

fn certify_periodic_band_layout(ring_heights: [Interval; 2], holes: &[Box2], period: f64) -> bool {
    let [first, second] = ring_heights;
    let (low, high) = if first.hi() < second.lo() {
        (first, second)
    } else if second.hi() < first.lo() {
        (second, first)
    } else {
        return false;
    };
    for hole in holes {
        let width = Interval::point(hole.u.hi()) - Interval::point(hole.u.lo());
        if !finite_interval(width)
            || width.hi() >= period
            || hole.v.lo() <= low.hi()
            || hole.v.hi() >= high.lo()
        {
            return false;
        }
    }
    for left in 0..holes.len() {
        for right in left + 1..holes.len() {
            if !periodic_envelopes_separated(holes[left], holes[right], period) {
                return false;
            }
        }
    }
    true
}

fn prepare_plane_envelope(
    store: &Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
) -> Result<Option<LoopEnvelope>> {
    if let Some(circle) = prepare_full_plane_circle(store, face_id, loop_id)? {
        return Ok(Some(circle));
    }
    prepare_bounded_envelope(store, face_id, loop_id)
}

fn prepare_bounded_envelope(
    store: &Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
) -> Result<Option<LoopEnvelope>> {
    if certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified
        || certify_bounded_analytic_loop_orientation(store, face_id, loop_id)?.is_none()
    {
        return Ok(None);
    }
    let Some(prepared) = prepare_bounded_analytic_loop(store, face_id, loop_id)? else {
        return Ok(None);
    };
    let Some(bounds) = bounds_for_spans(&prepared) else {
        return Ok(None);
    };
    Ok(Some(LoopEnvelope {
        bounds,
        full_circle: None,
    }))
}

fn prepare_full_plane_circle(
    store: &Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
) -> Result<Option<LoopEnvelope>> {
    let face = store.get(face_id)?;
    if !matches!(store.get(face.surface)?, SurfaceGeom::Plane(_)) {
        return Ok(None);
    }
    let loop_ = store.get(loop_id)?;
    let [fin_id] = loop_.fins.as_slice() else {
        return Ok(None);
    };
    let fin = store.get(*fin_id)?;
    let edge = store.get(fin.edge)?;
    let Some(use_) = fin.pcurve else {
        return Ok(None);
    };
    let Curve2dGeom::Circle(circle) = store.get(use_.curve())? else {
        return Ok(None);
    };
    if loop_.face != face_id
        || fin.parent != loop_id
        || edge.tolerance.is_some()
        || edge.bounds.is_some()
        || edge.vertices != [None, None]
        || use_.closure_winding() != Some([0, 0])
        || use_.seam().is_some()
        || !exact_period(use_.range().lo, use_.range().hi, core::f64::consts::TAU)
        || certify_whole_fin_incidence(store, face_id, loop_id, *fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        || certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified
        || certify_loop_orientation(store, face_id, loop_id)?.is_none()
    {
        return Ok(None);
    }
    let offset = use_.chart().apply(Point2::default(), [None, None])?;
    let center = circle.center() + offset;
    let radius = Interval::point(circle.radius());
    let u = Interval::point(center.x) + Interval::new(-radius.hi(), radius.hi());
    let v = Interval::point(center.y) + Interval::new(-radius.hi(), radius.hi());
    if !finite_interval(u) || !finite_interval(v) {
        return Ok(None);
    }
    Ok(Some(LoopEnvelope {
        bounds: Box2 { u, v },
        full_circle: Some(FullCircle {
            circle: *circle,
            center,
        }),
    }))
}

fn prepare_periodic_ring(
    store: &Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
    period: f64,
) -> Result<Option<PeriodicRing>> {
    let loop_ = store.get(loop_id)?;
    let [fin_id] = loop_.fins.as_slice() else {
        return Ok(None);
    };
    let fin = store.get(*fin_id)?;
    let edge = store.get(fin.edge)?;
    let Some(use_) = fin.pcurve else {
        return Ok(None);
    };
    let Curve2dGeom::Line(line) = store.get(use_.curve())? else {
        return Ok(None);
    };
    let winding = use_.closure_winding();
    if loop_.face != face_id
        || fin.parent != loop_id
        || edge.tolerance.is_some()
        || edge.bounds.is_some()
        || edge.vertices != [None, None]
        || !matches!(winding, Some([1 | -1, 0]))
        || use_.seam().is_some()
        || line.dir().y != 0.0
        || line.dir().x == 0.0
        || certify_whole_fin_incidence(store, face_id, loop_id, *fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        || certify_loop_orientation(store, face_id, loop_id)?.is_none()
    {
        return Ok(None);
    }
    let offset = use_
        .chart()
        .apply(Point2::default(), [Some(period), None])?;
    let height = Interval::point(line.origin().y) + Interval::point(offset.y);
    if !finite_interval(height) {
        return Ok(None);
    }
    Ok(Some(PeriodicRing {
        edge: fin.edge,
        height,
    }))
}

fn exact_period(lo: f64, hi: f64, period: f64) -> bool {
    if !lo.is_finite() || !hi.is_finite() || !period.is_finite() || lo >= hi || period <= 0.0 {
        return false;
    }
    let width = expansion::sum(&[hi], &expansion::negate(&[lo]));
    expansion::sign(&expansion::sum(&width, &expansion::negate(&[period]))) == 0
}

fn bounds_for_spans(spans: &[BoundedLoopSpan<'_, crate::entity::VertexId>]) -> Option<Box2> {
    let mut bounds = None;
    for span in spans {
        let next = span_bounds(span.geometry())?;
        bounds = Some(match bounds {
            None => next,
            Some(current) => union_box(current, next),
        });
    }
    bounds
}

fn span_bounds(span: super::BoundedPcurveSpan<'_>) -> Option<Box2> {
    let parameters = Interval::new(span.start().min(span.end()), span.start().max(span.end()));
    if !finite_interval(parameters) || parameters.width() == 0.0 {
        return None;
    }
    let offset = span.chart_offset();
    match span.curve() {
        Curve2dGeom::Line(line) => {
            let u = Interval::point(line.origin().x)
                + Interval::point(offset.x)
                + Interval::point(line.dir().x) * parameters;
            let v = Interval::point(line.origin().y)
                + Interval::point(offset.y)
                + Interval::point(line.dir().y) * parameters;
            (finite_interval(u) && finite_interval(v)).then_some(Box2 { u, v })
        }
        Curve2dGeom::Circle(circle) => circle_span_bounds(*circle, offset, parameters),
        _ => None,
    }
}

fn circle_span_bounds(circle: Circle2d, offset: Point2, parameters: Interval) -> Option<Box2> {
    if parameters.width() >= core::f64::consts::TAU {
        return None;
    }
    let chunks = (parameters.width() / CIRCLE_CHUNK_WIDTH).ceil() as usize;
    if chunks == 0 || chunks > MAX_CIRCLE_CHUNKS {
        return None;
    }
    let step = parameters.width() / chunks as f64;
    if !step.is_finite() || step <= 0.0 {
        return None;
    }
    let x = circle.x_dir();
    let y = x.perp();
    let radius = Interval::point(circle.radius());
    let center_u = Interval::point(circle.center().x) + Interval::point(offset.x);
    let center_v = Interval::point(circle.center().y) + Interval::point(offset.y);
    let radial_x_u = Interval::point(x.x) * radius;
    let radial_x_v = Interval::point(x.y) * radius;
    let radial_y_u = Interval::point(y.x) * radius;
    let radial_y_v = Interval::point(y.y) * radius;
    let mut bounds = None;
    for index in 0..chunks {
        let lo = parameters.lo() + step * index as f64;
        let hi = if index + 1 == chunks {
            parameters.hi()
        } else {
            parameters.lo() + step * (index + 1) as f64
        };
        let (sin, cos) = trig_cover(Interval::new(lo.min(hi), lo.max(hi)))?;
        let u = center_u + radial_x_u * cos + radial_y_u * sin;
        let v = center_v + radial_x_v * cos + radial_y_v * sin;
        let next = Box2 { u, v };
        if !finite_interval(u) || !finite_interval(v) {
            return None;
        }
        bounds = Some(match bounds {
            None => next,
            Some(current) => union_box(current, next),
        });
    }
    bounds
}

fn trig_cover(parameter: Interval) -> Option<(Interval, Interval)> {
    if !finite_interval(parameter) || parameter.width() > CIRCLE_CHUNK_WIDTH.next_up() {
        return None;
    }
    let midpoint = 0.5 * parameter.lo() + 0.5 * parameter.hi();
    let radius = (0.5 * parameter.width()).next_up();
    let (sin, cos) = math::sincos(midpoint);
    if !midpoint.is_finite() || !radius.is_finite() || !sin.is_finite() || !cos.is_finite() {
        return None;
    }
    Some((
        Interval::new(
            (-1.0_f64).max((sin.next_down() - radius).next_down()),
            1.0_f64.min((sin.next_up() + radius).next_up()),
        ),
        Interval::new(
            (-1.0_f64).max((cos.next_down() - radius).next_down()),
            1.0_f64.min((cos.next_up() + radius).next_up()),
        ),
    ))
}

fn union_box(first: Box2, second: Box2) -> Box2 {
    Box2 {
        u: Interval::new(
            first.u.lo().min(second.u.lo()),
            first.u.hi().max(second.u.hi()),
        ),
        v: Interval::new(
            first.v.lo().min(second.v.lo()),
            first.v.hi().max(second.v.hi()),
        ),
    }
}

fn convex_polygon(segments: &[Segment2], orientation: Orientation) -> Option<()> {
    if segments.len() < 3 {
        return None;
    }
    for index in 0..segments.len() {
        let first = segments[index].start;
        let second = segments[(index + 1) % segments.len()].start;
        let third = segments[(index + 2) % segments.len()].start;
        if orient2d([first.x, first.y], [second.x, second.y], [third.x, third.y]) != orientation {
            return None;
        }
    }
    Some(())
}

fn polygon_contains_envelope(
    segments: &[Segment2],
    orientation: Orientation,
    inner: LoopEnvelope,
) -> bool {
    if let Some(circle) = inner.full_circle {
        let points = segments
            .iter()
            .map(|segment| segment.start)
            .collect::<Vec<_>>();
        let Ok(translated) =
            Circle2d::new(circle.center, circle.circle.radius(), circle.circle.x_dir())
        else {
            return false;
        };
        return certify_convex_polygon_circle_containment(&points, translated);
    }
    for segment in segments {
        let edge_u = Interval::point(segment.end.x) - Interval::point(segment.start.x);
        let edge_v = Interval::point(segment.end.y) - Interval::point(segment.start.y);
        let offset_u = inner.bounds.u - Interval::point(segment.start.x);
        let offset_v = inner.bounds.v - Interval::point(segment.start.y);
        let mut side = edge_u * offset_v - edge_v * offset_u;
        if orientation == Orientation::Negative {
            side = -side;
        }
        if !finite_interval(side) || side.lo() <= 0.0 {
            return false;
        }
    }
    true
}

fn circle_contains_envelope(outer: FullCircle, inner: LoopEnvelope) -> bool {
    let outer_radius = Interval::point(outer.circle.radius());
    if let Some(inner) = inner.full_circle {
        let du = Interval::point(inner.center.x) - Interval::point(outer.center.x);
        let dv = Interval::point(inner.center.y) - Interval::point(outer.center.y);
        let Some(distance) = (du.square() + dv.square()).sqrt() else {
            return false;
        };
        let reach = distance + Interval::point(inner.circle.radius());
        return finite_interval(reach) && reach.hi() < outer_radius.lo();
    }
    let du = inner.bounds.u - Interval::point(outer.center.x);
    let dv = inner.bounds.v - Interval::point(outer.center.y);
    let distance_squared = du.square() + dv.square();
    let radius_squared = outer_radius.square();
    finite_interval(distance_squared)
        && finite_interval(radius_squared)
        && distance_squared.hi() < radius_squared.lo()
}

fn plane_envelopes_separated(first: LoopEnvelope, second: LoopEnvelope) -> bool {
    if let (Some(first), Some(second)) = (first.full_circle, second.full_circle) {
        let du = Interval::point(second.center.x) - Interval::point(first.center.x);
        let dv = Interval::point(second.center.y) - Interval::point(first.center.y);
        let distance_squared = du.square() + dv.square();
        let reach =
            Interval::point(first.circle.radius()) + Interval::point(second.circle.radius());
        return finite_interval(distance_squared)
            && finite_interval(reach)
            && distance_squared.lo() > reach.square().hi();
    }
    boxes_strictly_separated(first.bounds, second.bounds)
}

fn periodic_envelopes_separated(first: Box2, second: Box2, period: f64) -> bool {
    if intervals_strictly_separated(first.v, second.v) {
        return true;
    }
    let Some(first_pieces) = canonical_periodic_pieces(first.u, period) else {
        return false;
    };
    let Some(second_pieces) = canonical_periodic_pieces(second.u, period) else {
        return false;
    };
    first_pieces.iter().all(|&left| {
        second_pieces
            .iter()
            .all(|&right| intervals_strictly_separated(left, right))
    })
}

fn canonical_periodic_pieces(value: Interval, period: f64) -> Option<Vec<Interval>> {
    if !finite_interval(value) || !period.is_finite() || period <= 0.0 {
        return None;
    }
    let width = Interval::point(value.hi()) - Interval::point(value.lo());
    if !finite_interval(width) || width.hi() >= period {
        return None;
    }
    let quotient_interval = Interval::point(value.lo()).checked_div(Interval::point(period))?;
    if !finite_interval(quotient_interval)
        || quotient_interval.width() >= 1.0
        || quotient_interval.lo().abs() > (1_u64 << 52) as f64
    {
        return None;
    }
    let quotient = quotient_interval.lo().floor();
    let shift = Interval::point(-quotient) * Interval::point(period);
    let mut shifted = value + shift;
    for _ in 0..2 {
        if shifted.hi() < 0.0 {
            shifted = shifted + Interval::point(period);
        } else if shifted.lo() > period {
            shifted = shifted - Interval::point(period);
        } else {
            break;
        }
    }
    if shifted.lo() >= 0.0 && shifted.hi() <= period {
        return Some(vec![shifted]);
    }
    if shifted.lo() < 0.0 && shifted.hi() < period {
        let wrapped = shifted + Interval::point(period);
        return Some(vec![
            Interval::new(0.0, shifted.hi().max(0.0)),
            Interval::new(wrapped.lo().min(period), period),
        ]);
    }
    if shifted.lo() >= 0.0 && shifted.hi() > period {
        let wrapped = shifted - Interval::point(period);
        return Some(vec![
            Interval::new(shifted.lo().min(period), period),
            Interval::new(0.0, wrapped.hi().max(0.0)),
        ]);
    }
    None
}

fn boxes_strictly_separated(first: Box2, second: Box2) -> bool {
    intervals_strictly_separated(first.u, second.u)
        || intervals_strictly_separated(first.v, second.v)
}

fn intervals_strictly_separated(first: Interval, second: Interval) -> bool {
    first.hi() < second.lo() || second.hi() < first.lo()
}

fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite() && value.lo() <= value.hi()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn session_with_limit(allowed: u64) -> kcore::operation::SessionPolicy {
        let budget = BudgetPlan::new([LimitSpec::new(
            FACE_LOOP_CONTAINMENT_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap();
        kcore::operation::SessionPolicy::new(
            kcore::operation::SessionPrecision::parasolid(),
            kcore::operation::NumericalPolicy::v1(),
            kcore::operation::ExecutionPolicy::Serial,
            budget,
            kcore::operation::PolicyVersion::V1,
        )
    }

    fn bounds(u: [f64; 2], v: [f64; 2]) -> Box2 {
        Box2 {
            u: Interval::new(u[0], u[1]),
            v: Interval::new(v[0], v[1]),
        }
    }

    #[test]
    fn periodic_band_accepts_any_number_of_separated_holes_and_chart_phases() {
        let period = core::f64::consts::TAU;
        let base = [
            bounds([0.4, 1.0], [0.5, 1.5]),
            bounds([2.0, 2.6], [0.5, 1.5]),
            bounds([4.0, 4.6], [0.5, 1.5]),
        ];
        for count in 0..=base.len() {
            for phase in [-3.0 * period, 0.0, 4.0 * period] {
                let shifted = base[..count]
                    .iter()
                    .map(|value| Box2 {
                        u: value.u + Interval::point(phase),
                        v: value.v,
                    })
                    .collect::<Vec<_>>();
                assert!(certify_periodic_band_layout(
                    [Interval::point(0.0), Interval::point(2.0)],
                    &shifted,
                    period
                ));
                for left in 0..shifted.len() {
                    for right in left + 1..shifted.len() {
                        assert!(periodic_envelopes_separated(
                            shifted[left],
                            shifted[right],
                            period
                        ));
                    }
                }
            }
        }
    }

    #[test]
    fn periodic_separation_tamper_table_fails_closed() {
        let period = core::f64::consts::TAU;
        let first = bounds([0.25, 1.25], [0.5, 1.5]);
        let cases = [
            ("overlap", bounds([1.0, 2.0], [0.5, 1.5])),
            ("touch", bounds([1.25, 2.0], [0.5, 1.5])),
            ("nested", bounds([0.5, 1.0], [0.75, 1.25])),
            (
                "periodic duplicate",
                bounds([0.25 + period, 1.25 + period], [0.5, 1.5]),
            ),
        ];
        for (name, second) in cases {
            assert!(
                !periodic_envelopes_separated(first, second, period),
                "{name}"
            );
        }
        let v_separated = bounds([0.5, 1.0], [1.6, 1.9]);
        assert!(periodic_envelopes_separated(first, v_separated, period));

        let band_cases = [
            (
                "duplicate ring height",
                [Interval::point(0.0), Interval::point(0.0)],
                vec![first],
            ),
            (
                "touch lower ring",
                [Interval::point(0.0), Interval::point(2.0)],
                vec![bounds([0.25, 1.25], [0.0, 1.0])],
            ),
            (
                "outside slab",
                [Interval::point(0.0), Interval::point(2.0)],
                vec![bounds([0.25, 1.25], [-0.5, 0.5])],
            ),
            (
                "whole-period hole",
                [Interval::point(0.0), Interval::point(2.0)],
                vec![bounds([0.0, period], [0.5, 1.5])],
            ),
            (
                "overlapping peers",
                [Interval::point(0.0), Interval::point(2.0)],
                vec![first, bounds([1.0, 2.0], [0.5, 1.5])],
            ),
        ];
        for (name, heights, holes) in band_cases {
            assert!(
                !certify_periodic_band_layout(heights, &holes, period),
                "{name}"
            );
        }
    }

    #[test]
    fn work_formula_accepts_exact_n_and_rejects_n_minus_one() {
        let required = face_loop_containment_work(4, 10).unwrap();
        assert_eq!(required, 4 + 650 + 6 + 16);

        let exact_session = session_with_limit(required);
        let exact_context = kcore::operation::OperationContext::new(
            &exact_session,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap();
        let mut exact_scope = OperationScope::new(&exact_context);
        charge_face_loop_containment_work(&mut exact_scope, 4, 10).unwrap();

        let short_session = session_with_limit(required - 1);
        let short_context = kcore::operation::OperationContext::new(
            &short_session,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap();
        let mut short_scope = OperationScope::new(&short_context);
        let error = charge_face_loop_containment_work(&mut short_scope, 4, 10).unwrap_err();
        assert_eq!(
            error.limit().map(|limit| limit.stage),
            Some(FACE_LOOP_CONTAINMENT_WORK)
        );

        assert!(face_loop_containment_work(usize::MAX, usize::MAX).is_none());
    }

    #[test]
    fn topological_period_and_periodic_reduction_fail_closed_at_rounding_boundaries() {
        let period = core::f64::consts::TAU;
        assert!(exact_period(0.0, period, period));
        assert!(!exact_period(0.0, period.next_up(), period));
        assert!(!exact_period(0.0, period.next_down(), period));

        let huge = period * ((1_u64 << 53) as f64);
        assert!(canonical_periodic_pieces(Interval::new(huge, huge + 1.0), period).is_none());

        let seam_left = bounds([-0.25, 0.25], [0.5, 1.5]);
        let seam_right = bounds([period - 0.1, period + 0.1], [0.5, 1.5]);
        assert!(!periodic_envelopes_separated(seam_left, seam_right, period));
    }

    #[test]
    fn polygon_containment_uses_the_live_chart_shifted_circle_center() {
        let segments = [
            Segment2 {
                start: Point2::new(-2.0, -2.0),
                end: Point2::new(2.0, -2.0),
            },
            Segment2 {
                start: Point2::new(2.0, -2.0),
                end: Point2::new(2.0, 2.0),
            },
            Segment2 {
                start: Point2::new(2.0, 2.0),
                end: Point2::new(-2.0, 2.0),
            },
            Segment2 {
                start: Point2::new(-2.0, 2.0),
                end: Point2::new(-2.0, -2.0),
            },
        ];
        let authored =
            Circle2d::new(Point2::default(), 0.25, kgeom::vec::Vec2::new(1.0, 0.0)).unwrap();
        let shifted = LoopEnvelope {
            bounds: bounds([9.75, 10.25], [-0.25, 0.25]),
            full_circle: Some(FullCircle {
                circle: authored,
                center: Point2::new(10.0, 0.0),
            }),
        };
        assert!(!polygon_contains_envelope(
            &segments,
            Orientation::Positive,
            shifted
        ));
    }

    #[test]
    fn plane_convex_outer_accepts_multiple_line_and_circle_hole_envelopes() {
        let segments = [
            Segment2 {
                start: Point2::new(-5.0, -5.0),
                end: Point2::new(5.0, -5.0),
            },
            Segment2 {
                start: Point2::new(5.0, -5.0),
                end: Point2::new(5.0, 5.0),
            },
            Segment2 {
                start: Point2::new(5.0, 5.0),
                end: Point2::new(-5.0, 5.0),
            },
            Segment2 {
                start: Point2::new(-5.0, 5.0),
                end: Point2::new(-5.0, -5.0),
            },
        ];
        let circle =
            Circle2d::new(Point2::new(0.0, 3.0), 0.5, kgeom::vec::Vec2::new(1.0, 0.0)).unwrap();
        let holes = [
            LoopEnvelope {
                bounds: bounds([-4.0, -3.0], [-1.0, 1.0]),
                full_circle: None,
            },
            LoopEnvelope {
                bounds: bounds([3.0, 4.0], [-1.0, 1.0]),
                full_circle: None,
            },
            LoopEnvelope {
                bounds: bounds([-0.5, 0.5], [2.5, 3.5]),
                full_circle: Some(FullCircle {
                    circle,
                    center: circle.center(),
                }),
            },
        ];
        assert!(holes.iter().all(|hole| polygon_contains_envelope(
            &segments,
            Orientation::Positive,
            *hole
        )));
        for left in 0..holes.len() {
            for right in left + 1..holes.len() {
                assert!(plane_envelopes_separated(holes[left], holes[right]));
            }
        }

        let nested = LoopEnvelope {
            bounds: bounds([-0.25, 0.25], [2.75, 3.25]),
            full_circle: Some(FullCircle {
                circle: Circle2d::new(Point2::new(0.0, 3.0), 0.25, kgeom::vec::Vec2::new(1.0, 0.0))
                    .unwrap(),
                center: Point2::new(0.0, 3.0),
            }),
        };
        assert!(!plane_envelopes_separated(holes[2], nested));
    }
}
