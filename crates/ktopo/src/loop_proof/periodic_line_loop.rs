//! Alternating axis-aligned periodic graph loops on cylinder charts.
//!
//! A bounded cylinder loop may represent a noncontractible ring without an
//! endpoint-free edge. This authority places a topology-ordered, strictly
//! alternating sequence of horizontal and vertical bounded Line2d uses in one
//! proof-local universal-cover chart. It admits only total winding `(+/-1, 0)`
//! and requires every horizontal span to advance with that winding. Within
//! this representation the lifted walk is a single-valued periodic height
//! graph (vertical jumps are allowed) and cannot meet itself or a nontrivial
//! period translate. Other curve types, same-axis subdivisions, backtracking,
//! ambiguous lifts, or repeated vertical fibers fail closed.

use super::bounded_pcurve_integral::BoundedPcurveSpan;
use super::bounded_pcurve_simplicity::{
    BoundedLoopSimplicity, BoundedLoopSpan, CertifiedBoundedLoopJoin,
    certify_bounded_span_family_simplicity,
};
use super::{bounded_join_chart_neighborhood, certify_model_distance};
use crate::entity::{EdgeId, LoopId, Sense, VertexId};
use crate::geom::{Curve2dGeom, SurfaceGeom};
use crate::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use crate::store::Store;
use kcore::error::Result;
use kcore::interval::Interval;
use kcore::predicates::Orientation;
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve2d::Curve2d;
use kgeom::vec::Point2;

const MAX_PERIOD_LIFT: i64 = 1_i64 << 40;

/// Checked quadratic fin/pair supplement for the periodic line-loop authority.
///
/// Callers separately cover their linear traversal/integration work. This
/// supplement covers one fin-index pass, three conservative base-fin pair
/// passes, and every pair in the three-layer universal-cover family.
pub(crate) fn periodic_line_loop_proof_work(fin_count: usize) -> Option<u64> {
    let fins = u64::try_from(fin_count).ok()?;
    let fin_pairs = fins.checked_mul(fins.saturating_sub(1))?.checked_div(2)?;
    let layered_fins = fins.checked_mul(3)?;
    let layered_pairs = layered_fins
        .checked_mul(layered_fins.saturating_sub(1))?
        .checked_div(2)?;
    fins.checked_add(fin_pairs.checked_mul(3)?)?
        .checked_add(layered_pairs)
}

/// Certified intrinsic data consumed by loop orientation and face layout.
#[derive(Debug, Clone)]
pub(super) struct CertifiedPeriodicLineLoop {
    orientation: Orientation,
    height: Interval,
    edges: Vec<EdgeId>,
}

impl CertifiedPeriodicLineLoop {
    pub(super) const fn orientation(&self) -> Orientation {
        self.orientation
    }

    pub(super) const fn height(&self) -> Interval {
        self.height
    }

    pub(super) fn edges(&self) -> &[EdgeId] {
        &self.edges
    }
}

#[derive(Debug, Clone, Copy)]
struct RawSpan<'a> {
    edge: EdgeId,
    tail: VertexId,
    head: VertexId,
    geometry: BoundedPcurveSpan<'a>,
    start: Point2,
    end: Point2,
    horizontal: bool,
}

#[derive(Debug)]
struct LiftedCycle<'a> {
    spans: Vec<RawSpan<'a>>,
    joins: Vec<CertifiedBoundedLoopJoin>,
    winding: i64,
}

/// Certify an alternating finite cycle of horizontal and vertical bounded
/// Line2d uses as one simple noncontractible cylinder boundary.
pub(super) fn certify_piecewise_periodic_line_loop(
    store: &Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
) -> Result<Option<CertifiedPeriodicLineLoop>> {
    let Some(cycle) = certify_piecewise_periodic_line_cycle(store, face_id, loop_id)? else {
        return Ok(None);
    };
    let Some(height) = height_envelope(&cycle.spans, 2.0 * LINEAR_RESOLUTION) else {
        return Ok(None);
    };
    Ok(Some(CertifiedPeriodicLineLoop {
        orientation: if cycle.winding > 0 {
            Orientation::Positive
        } else {
            Orientation::Negative
        },
        height,
        edges: cycle.spans.iter().map(|span| span.edge).collect(),
    }))
}

/// Return the theorem's already-certified base traversal in one universal-
/// cover chart. Consumers must not reinterpret unlifted authored pcurves when
/// this authority succeeds.
pub(crate) fn certify_piecewise_periodic_line_spans<'a>(
    store: &'a Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
) -> Result<Option<Vec<BoundedPcurveSpan<'a>>>> {
    Ok(
        certify_piecewise_periodic_line_cycle(store, face_id, loop_id)?
            .map(|cycle| cycle.spans.into_iter().map(|span| span.geometry).collect()),
    )
}

fn certify_piecewise_periodic_line_cycle<'a>(
    store: &'a Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
) -> Result<Option<LiftedCycle<'a>>> {
    let face = store.get(face_id)?;
    let surface = store.get(face.surface)?;
    let SurfaceGeom::Cylinder(cylinder) = surface else {
        return Ok(None);
    };
    let loop_ = store.get(loop_id)?;
    let loop_owned_once = face
        .loops
        .iter()
        .position(|&candidate| candidate == loop_id)
        .is_some_and(|index| !face.loops[index + 1..].contains(&loop_id));
    if loop_.face != face_id || !loop_owned_once || loop_.fins.len() < 2 {
        return Ok(None);
    }
    let period = core::f64::consts::TAU;
    let u_tolerance = 2.0 * LINEAR_RESOLUTION / cylinder.radius().max(1.0);
    let v_tolerance = 2.0 * LINEAR_RESOLUTION;
    let Some(raw) = prepare_spans(store, face_id, loop_id)? else {
        return Ok(None);
    };
    let Some(cycle) = lift_cycle(surface, &raw, period) else {
        return Ok(None);
    };
    if !periodic_graph_is_simple(
        &cycle.spans,
        cycle.winding,
        period,
        [u_tolerance, v_tolerance],
    ) || !periodic_span_family_is_simple(&cycle, period)
    {
        return Ok(None);
    }
    Ok(Some(cycle))
}

fn prepare_spans<'a>(
    store: &'a Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
) -> Result<Option<Vec<RawSpan<'a>>>> {
    let face = store.get(face_id)?;
    let periods = store
        .get(face.surface)?
        .as_leaf_surface()
        .map(|surface| surface.periodicity())
        .unwrap_or([None, None]);
    let loop_ = store.get(loop_id)?;
    let mut spans = Vec::with_capacity(loop_.fins.len());
    let mut edges = Vec::with_capacity(loop_.fins.len());
    let mut tails = Vec::with_capacity(loop_.fins.len());
    for &fin_id in &loop_.fins {
        if certify_whole_fin_incidence(store, face_id, loop_id, fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        {
            return Ok(None);
        }
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let (Some((lo, hi)), [Some(_), Some(_)], Some(use_), Some(tail), Some(head)) = (
            edge.bounds,
            edge.vertices,
            fin.pcurve,
            store.fin_tail(fin_id)?,
            store.fin_head(fin_id)?,
        ) else {
            return Ok(None);
        };
        let curve = store.get(use_.curve())?;
        let Curve2dGeom::Line(line) = curve else {
            return Ok(None);
        };
        if fin.parent != loop_id
            || edge.tolerance.is_some()
            || !lo.is_finite()
            || !hi.is_finite()
            || lo >= hi
            || use_.closure_winding().is_some()
            || use_.seam().is_some()
            || edges.contains(&fin.edge)
            || tails.contains(&tail)
        {
            return Ok(None);
        }
        let (edge_start, edge_end) = match fin.sense {
            Sense::Forward => (lo, hi),
            Sense::Reversed => (hi, lo),
        };
        let chart = use_.chart().apply(Point2::default(), periods)?;
        let start = line.eval(use_.edge_to_pcurve().map(edge_start)) + chart;
        let end = line.eval(use_.edge_to_pcurve().map(edge_end)) + chart;
        let horizontal = line.dir().y == 0.0 && line.dir().x != 0.0;
        let vertical = line.dir().x == 0.0 && line.dir().y != 0.0;
        if (!horizontal && !vertical) || !finite_point(start) || !finite_point(end) {
            return Ok(None);
        }
        edges.push(fin.edge);
        tails.push(tail);
        spans.push(RawSpan {
            edge: fin.edge,
            tail,
            head,
            geometry: BoundedPcurveSpan::new(
                curve,
                use_.edge_to_pcurve().map(edge_start),
                use_.edge_to_pcurve().map(edge_end),
                chart,
            ),
            start,
            end,
            horizontal,
        });
    }
    Ok(Some(spans))
}

/// Place one topology cycle in a universal-cover chart. Integer lifts are
/// unique nearest-period cells, while any residual join is authorized only
/// by topology identity, whole-fin incidence, and certified surface distance.
fn lift_cycle<'a>(
    surface: &SurfaceGeom,
    raw: &[RawSpan<'a>],
    period: f64,
) -> Option<LiftedCycle<'a>> {
    let first = *raw.first()?;
    let mut lifts = vec![0_i64; raw.len()];
    for index in 0..raw.len().saturating_sub(1) {
        if raw[index].head != raw[index + 1].tail {
            return None;
        }
        let relative = nearest_period_lift(raw[index].end.x, raw[index + 1].start.x, period)?;
        lifts[index + 1] = lifts[index].checked_add(relative)?;
        if lifts[index + 1].abs() > MAX_PERIOD_LIFT {
            return None;
        }
    }
    let last = *raw.last()?;
    if last.head != first.tail {
        return None;
    }
    let closure = nearest_period_lift(last.end.x, first.start.x, period)?;
    let winding = lifts.last()?.checked_add(closure)?;
    if winding.unsigned_abs() != 1 || winding.abs() > MAX_PERIOD_LIFT {
        return None;
    }

    let mut spans = Vec::with_capacity(raw.len());
    for (span, lift) in raw.iter().copied().zip(lifts) {
        let geometry = shift_geometry(span.geometry, lift, period)?;
        let (start, end) = span_endpoints(geometry)?;
        spans.push(RawSpan {
            geometry,
            start,
            end,
            ..span
        });
    }

    let chart_neighborhood = bounded_join_chart_neighborhood(surface, 2.0 * LINEAR_RESOLUTION)?;
    let evidence = CertifiedBoundedLoopJoin::new(chart_neighborhood)?;
    let leaf = surface.as_leaf_surface()?;
    let mut joins = Vec::with_capacity(spans.len());
    for index in 0..spans.len() {
        let next = (index + 1) % spans.len();
        let next_start = if next == 0 {
            span_endpoints(shift_geometry(spans[0].geometry, winding, period)?)?.0
        } else {
            spans[next].start
        };
        if spans[index].head != spans[next].tail
            || !certify_model_distance(
                leaf.eval([spans[index].end.x, spans[index].end.y]),
                leaf.eval([next_start.x, next_start.y]),
                2.0 * LINEAR_RESOLUTION,
            )
        {
            return None;
        }
        joins.push(evidence);
    }
    Some(LiftedCycle {
        spans,
        joins,
        winding,
    })
}

fn periodic_graph_is_simple(
    spans: &[RawSpan<'_>],
    winding: i64,
    period: f64,
    tolerance: [f64; 2],
) -> bool {
    let mut horizontal_count = 0_usize;
    let mut verticals = Vec::new();
    for (index, span) in spans.iter().enumerate() {
        let next = spans[(index + 1) % spans.len()];
        if span.head != next.tail || span.horizontal == next.horizontal {
            return false;
        }
        let du = Interval::point(span.end.x) - Interval::point(span.start.x);
        let dv = Interval::point(span.end.y) - Interval::point(span.start.y);
        if span.horizontal {
            horizontal_count += 1;
            if !near_zero(dv, tolerance[1])
                || winding > 0 && du.lo() <= 0.0
                || winding < 0 && du.hi() >= 0.0
            {
                return false;
            }
        } else {
            if !near_zero(du, tolerance[0]) || dv.lo() <= 0.0 && dv.hi() >= 0.0 {
                return false;
            }
            let Curve2dGeom::Line(line) = span.geometry.curve() else {
                return false;
            };
            verticals.push(line.origin().x + span.geometry.chart_offset().x);
        }
    }
    if horizontal_count == 0 || verticals.is_empty() {
        return false;
    }
    verticals
        .iter()
        .copied()
        .chain(core::iter::once(verticals[0] + winding as f64 * period))
        .collect::<Vec<_>>()
        .windows(2)
        .all(|pair| {
            let advance = Interval::point(pair[1]) - Interval::point(pair[0]);
            advance.lo().is_finite()
                && if winding > 0 {
                    advance.lo() > 0.0
                } else {
                    advance.hi() < 0.0
                }
        })
}

/// Check one base traversal together with both adjacent period translates.
/// Strict ordered vertical carriers prove that farther translates cannot
/// meet; the existing bounded pair engine then proves every possible base or
/// adjacent-translate contact disjoint except a topology-owned confined join.
fn periodic_span_family_is_simple(cycle: &LiftedCycle<'_>, period: f64) -> bool {
    let Some(capacity) = cycle.spans.len().checked_mul(3) else {
        return false;
    };
    let mut family = Vec::with_capacity(capacity);
    for layer in [-1_i64, 0, 1] {
        let Some(layer_lift) = layer.checked_mul(cycle.winding) else {
            return false;
        };
        for (span, evidence) in cycle.spans.iter().zip(&cycle.joins) {
            let Some(geometry) = shift_geometry(span.geometry, layer_lift, period) else {
                return false;
            };
            let tail = family.len();
            family.push(BoundedLoopSpan::new(geometry, tail, tail + 1).with_head_join(*evidence));
        }
    }
    certify_bounded_span_family_simplicity(&family) == BoundedLoopSimplicity::Certified
}

fn nearest_period_lift(current: f64, next: f64, period: f64) -> Option<i64> {
    if !current.is_finite() || !next.is_finite() || !period.is_finite() || period <= 0.0 {
        return None;
    }
    let quotient =
        (Interval::point(current) - Interval::point(next)).checked_div(Interval::point(period))?;
    let midpoint = 0.5 * quotient.lo() + 0.5 * quotient.hi();
    let candidate = midpoint.round();
    if !candidate.is_finite()
        || candidate.abs() > MAX_PERIOD_LIFT as f64
        || quotient.lo() <= candidate - 0.5
        || quotient.hi() >= candidate + 0.5
    {
        return None;
    }
    Some(candidate as i64)
}

fn shift_geometry<'a>(
    geometry: BoundedPcurveSpan<'a>,
    lift: i64,
    period: f64,
) -> Option<BoundedPcurveSpan<'a>> {
    if lift.abs() > MAX_PERIOD_LIFT {
        return None;
    }
    let shift = lift as f64 * period;
    let chart = geometry.chart_offset();
    let x = chart.x + shift;
    (shift.is_finite() && x.is_finite())
        .then_some(geometry.with_chart_offset(Point2::new(x, chart.y)))
}

fn span_endpoints(span: BoundedPcurveSpan<'_>) -> Option<(Point2, Point2)> {
    let curve = span.curve().as_curve();
    let start = curve.eval(span.start()) + span.chart_offset();
    let end = curve.eval(span.end()) + span.chart_offset();
    (finite_point(start) && finite_point(end)).then_some((start, end))
}

fn near_zero(value: Interval, tolerance: f64) -> bool {
    tolerance.is_finite()
        && tolerance >= 0.0
        && value.lo().is_finite()
        && value.hi().is_finite()
        && value.lo() >= -tolerance
        && value.hi() <= tolerance
}

fn height_envelope(spans: &[RawSpan<'_>], tolerance: f64) -> Option<Interval> {
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for point in spans.iter().flat_map(|span| [span.start, span.end]) {
        low = low.min(point.y);
        high = high.max(point.y);
    }
    let low = (low - tolerance).next_down();
    let high = (high + tolerance).next_up();
    (low.is_finite() && high.is_finite() && low <= high).then_some(Interval::new(low, high))
}

fn finite_point(point: Point2) -> bool {
    point.x.is_finite() && point.y.is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::{Edge, Vertex};
    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::surface::Cylinder;
    use kgeom::vec::Point3;

    fn cylinder_surface(radius: f64) -> SurfaceGeom {
        SurfaceGeom::Cylinder(Cylinder::new(Frame::world(), radius).unwrap())
    }

    fn segment_chain(points: &[Point2]) -> Vec<(Point2, Point2)> {
        points.windows(2).map(|pair| (pair[0], pair[1])).collect()
    }

    fn line_curves(segments: &[(Point2, Point2)]) -> Vec<Curve2dGeom> {
        segments
            .iter()
            .map(|&(start, end)| Curve2dGeom::Line(Line2d::new(start, end - start).unwrap()))
            .collect()
    }

    fn topology_identities(count: usize) -> (Vec<EdgeId>, Vec<VertexId>) {
        let mut store = Store::new();
        let point = store.insert_point(Point3::default()).unwrap();
        let vertices = (0..count)
            .map(|_| {
                store.add(Vertex {
                    point,
                    tolerance: None,
                })
            })
            .collect::<Vec<_>>();
        let edges = (0..count)
            .map(|index| {
                store.add(Edge {
                    curve: None,
                    vertices: [Some(vertices[index]), Some(vertices[(index + 1) % count])],
                    bounds: Some((0.0, 1.0)),
                    fins: Vec::new(),
                    tolerance: None,
                })
            })
            .collect();
        (edges, vertices)
    }

    fn raw_spans<'a>(curves: &'a [Curve2dGeom], segments: &[(Point2, Point2)]) -> Vec<RawSpan<'a>> {
        let (edges, vertices) = topology_identities(segments.len());
        curves
            .iter()
            .zip(segments)
            .enumerate()
            .map(|(index, (curve, &(authored_start, authored_end)))| {
                let length = (authored_end - authored_start).norm();
                let geometry = BoundedPcurveSpan::new(curve, 0.0, length, Point2::default());
                let (start, end) = span_endpoints(geometry).unwrap();
                RawSpan {
                    edge: edges[index],
                    tail: vertices[index],
                    head: vertices[(index + 1) % vertices.len()],
                    geometry,
                    start,
                    end,
                    horizontal: authored_start.y == authored_end.y
                        && authored_start.x != authored_end.x,
                }
            })
            .collect()
    }

    fn certified_cycle<'a>(surface: &SurfaceGeom, raw: &[RawSpan<'a>]) -> Option<LiftedCycle<'a>> {
        let SurfaceGeom::Cylinder(cylinder) = surface else {
            return None;
        };
        let period = core::f64::consts::TAU;
        let cycle = lift_cycle(surface, raw, period)?;
        let tolerance = [
            2.0 * LINEAR_RESOLUTION / cylinder.radius().max(1.0),
            2.0 * LINEAR_RESOLUTION,
        ];
        (periodic_graph_is_simple(&cycle.spans, cycle.winding, period, tolerance)
            && periodic_span_family_is_simple(&cycle, period))
        .then_some(cycle)
    }

    fn positive_graph_points() -> Vec<Point2> {
        let period = core::f64::consts::TAU;
        vec![
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(2.0, 1.0),
            Point2::new(4.0, 1.0),
            Point2::new(4.0, -1.0),
            Point2::new(period, -1.0),
            Point2::new(period, 0.0),
        ]
    }

    #[test]
    fn periodic_line_authority_accepts_every_cycle_anchor_and_reverse_winding() {
        let surface = cylinder_surface(1.0);
        let segments = segment_chain(&positive_graph_points());
        let curves = line_curves(&segments);
        let raw = raw_spans(&curves, &segments);
        for anchor in 0..raw.len() {
            let mut rotated = raw.clone();
            rotated.rotate_left(anchor);
            let cycle = certified_cycle(&surface, &rotated)
                .unwrap_or_else(|| panic!("positive graph refused at cyclic anchor {anchor}"));
            assert_eq!(cycle.winding, 1);
        }

        let period = core::f64::consts::TAU;
        let reverse_points = [
            Point2::new(0.0, 0.0),
            Point2::new(0.0, -1.0),
            Point2::new(4.0 - period, -1.0),
            Point2::new(4.0 - period, 1.0),
            Point2::new(2.0 - period, 1.0),
            Point2::new(2.0 - period, 0.0),
            Point2::new(-period, 0.0),
        ];
        let reverse_segments = segment_chain(&reverse_points);
        let reverse_curves = line_curves(&reverse_segments);
        let reverse_raw = raw_spans(&reverse_curves, &reverse_segments);
        for anchor in 0..reverse_raw.len() {
            let mut rotated = reverse_raw.clone();
            rotated.rotate_left(anchor);
            let cycle = certified_cycle(&surface, &rotated)
                .unwrap_or_else(|| panic!("negative graph refused at cyclic anchor {anchor}"));
            assert_eq!(cycle.winding, -1);
        }
    }

    #[test]
    fn periodic_line_authority_tamper_table_fails_closed() {
        let period = core::f64::consts::TAU;
        let unit = 1.0e-6;
        let crossing = vec![
            (Point2::new(0.0, 0.0), Point2::new(4.0 * unit, 0.0)),
            (Point2::new(4.0 * unit, 0.0), Point2::new(4.0 * unit, 2.0)),
            (Point2::new(3.1 * unit, 2.0), Point2::new(3.2 * unit, 2.0)),
            (Point2::new(2.3 * unit, 2.0), Point2::new(2.3 * unit, -1.0)),
            (Point2::new(1.4 * unit, -1.0), Point2::new(period, -1.0)),
            (Point2::new(period, -1.0), Point2::new(period, 0.0)),
        ];
        let mut repeated_carrier = segment_chain(&positive_graph_points());
        repeated_carrier[3] = (Point2::new(2.0, 1.0), Point2::new(2.0, -1.0));
        let mut unauthorized_join = segment_chain(&positive_graph_points());
        unauthorized_join[1] = (
            Point2::new(2.0 - unit, -1.0e-9),
            Point2::new(2.0 - unit, 1.0),
        );

        for (name, radius, segments, graph_expected) in [
            ("self crossing", 1.0e-12, crossing, false),
            (
                "repeated vertical carrier",
                1.0e-12,
                repeated_carrier,
                false,
            ),
            ("unauthorized near join", 1.0e-4, unauthorized_join, true),
        ] {
            let surface = cylinder_surface(radius);
            let curves = line_curves(&segments);
            let raw = raw_spans(&curves, &segments);
            let cycle = lift_cycle(&surface, &raw, period)
                .unwrap_or_else(|| panic!("{name}: setup did not reach the lifted proof"));
            let graph = periodic_graph_is_simple(
                &cycle.spans,
                cycle.winding,
                period,
                [
                    2.0 * LINEAR_RESOLUTION / radius.max(1.0),
                    2.0 * LINEAR_RESOLUTION,
                ],
            );
            assert_eq!(graph, graph_expected, "{name}: wrong graph gate");
            assert!(
                !periodic_span_family_is_simple(&cycle, period),
                "{name}: bounded family admitted the tamper"
            );
            assert!(
                certified_cycle(&surface, &raw).is_none(),
                "{name}: complete authority admitted the tamper"
            );
        }
    }

    #[test]
    fn periodic_line_authority_rejects_ambiguous_lifts_and_two_windings() {
        let period = core::f64::consts::TAU;
        assert!(nearest_period_lift(0.5 * period, 0.0, period).is_none());
        assert!(nearest_period_lift(-0.5 * period, 0.0, period).is_none());

        for winding in [-2.0_f64, 2.0_f64] {
            let points = [
                Point2::new(0.0, 0.0),
                Point2::new(4.0 * winding.signum(), 0.0),
                Point2::new(4.0 * winding.signum(), 1.0),
                Point2::new(8.0 * winding.signum(), 1.0),
                Point2::new(8.0 * winding.signum(), -1.0),
                Point2::new(winding * period, -1.0),
                Point2::new(winding * period, 0.0),
            ];
            let segments = segment_chain(&points);
            let curves = line_curves(&segments);
            let raw = raw_spans(&curves, &segments);
            assert!(
                lift_cycle(&cylinder_surface(1.0), &raw, period).is_none(),
                "winding {winding} was admitted"
            );
        }
    }

    #[test]
    fn periodic_line_proof_work_matches_the_three_layer_pair_formula() {
        let formula = |fins: u64| {
            let fin_pairs = fins * fins.saturating_sub(1) / 2;
            let layered = 3 * fins;
            fins + 3 * fin_pairs + layered * layered.saturating_sub(1) / 2
        };
        for fins in [0_usize, 1, 2, 4, 17] {
            assert_eq!(
                periodic_line_loop_proof_work(fins),
                Some(formula(fins as u64))
            );
        }
        assert_eq!(periodic_line_loop_proof_work(usize::MAX), None);
    }
}
