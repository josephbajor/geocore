//! Conservative whole-loop simplicity proofs.
//!
//! Exact plane-segment loops and one-fin circle/ellipse loops use their
//! specialized predicates. Finite mixed `Line2d`/`Circle2d` loops on planes
//! and cylinders use pairwise exact/interval intersection proofs plus a
//! bounded Green integral. Near chart joins are admitted only after exact
//! topology identity, whole-fin incidence, and outward surface-lifted
//! distance certification. Unsupported curves and nonlinear charts remain
//! indeterminate.

mod analytic_face_layout;
pub(crate) mod bounded_pcurve_integral;
pub(crate) mod bounded_pcurve_simplicity;
#[cfg(test)]
mod periodic_lift_tests;

#[cfg(test)]
pub(crate) use analytic_face_layout::FACE_LOOP_CONTAINMENT_WORK;
pub(crate) use analytic_face_layout::face_loop_containment_budget;

use self::analytic_face_layout::{certify_analytic_face_layout, charge_face_loop_containment_work};
use self::bounded_pcurve_integral::{
    BoundedPcurveSpan, SignedLineIntegralProof, certify_signed_line_integral,
};
use self::bounded_pcurve_simplicity::{
    BoundedLoopSimplicity, BoundedLoopSpan, CertifiedBoundedLoopJoin,
    certify_bounded_loop_simplicity,
};
use crate::entity::{Edge, FinPcurve, LoopId, Sense, VertexId};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::incidence::{
    IncidenceCertification, certify_edge_surface_incidence, certify_pcurve_incidence,
};
use crate::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use crate::store::Store;
use kcore::error::Result;
use kcore::interval::Interval;
use kcore::predicates::{Orientation, orient2d, polygon_orientation2d_iter};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve::Curve;
use kgeom::curve2d::Circle2d;
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

/// Exact planar straight-loop orientation and outer-loop evidence.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct PlanarLoopLayout {
    /// Unique outer loop when every loop has an exact nonzero orientation and
    /// the supported strict-containment relation is certified.
    pub(crate) outer: Option<LoopId>,
    /// Exact orientation for each input loop, or `None` when that loop is
    /// outside this proof slice.
    pub(crate) orientations: Vec<(LoopId, Option<Orientation>)>,
}

/// Certify one loop's intrinsic traversal orientation in its owning surface
/// chart.
///
/// Polygonal plane loops use exact projected predicates. A single-fin circle
/// on a plane and a full-period constant-height line on a cylinder use the
/// exact sign of their stored pcurve parameter correspondence and fin sense,
/// after topology-owned whole-fin incidence succeeds. Periodic chart closure
/// is therefore proven without sampling or inventing a duplicate seam
/// vertex.
pub(crate) fn certify_loop_orientation(
    store: &Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
) -> Result<Option<Orientation>> {
    let loop_ = store.get(loop_id)?;
    if loop_.face != face_id
        || store
            .get(face_id)?
            .loops
            .iter()
            .filter(|&&candidate| candidate == loop_id)
            .count()
            != 1
    {
        return Ok(None);
    }
    if let Some((_, orientation)) = strict_planar_ring(store, loop_id)? {
        return Ok(Some(orientation));
    }
    if let Some(orientation) = certify_bounded_analytic_loop_orientation(store, face_id, loop_id)? {
        return Ok(Some(orientation));
    }
    let [fin_id] = loop_.fins.as_slice() else {
        return Ok(None);
    };
    let fin = store.get(*fin_id)?;
    let edge = store.get(fin.edge)?;
    if edge.tolerance.is_some()
        || certify_whole_fin_incidence(store, face_id, loop_id, *fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
    {
        return Ok(None);
    }
    let Some(use_) = fin.pcurve else {
        return Ok(None);
    };
    let face = store.get(face_id)?;
    let parameter_orientation = match (store.get(face.surface)?, store.get(use_.curve())?) {
        (SurfaceGeom::Plane(_), Curve2dGeom::Circle(_))
            if use_.closure_winding() == Some([0, 0]) =>
        {
            traversal_orientation([use_.edge_to_pcurve().scale(), 1.0], fin.sense)
        }
        (SurfaceGeom::Cylinder(_), Curve2dGeom::Line(line))
            if line.dir().y == 0.0
                && line.dir().x != 0.0
                && matches!(use_.closure_winding(), Some([1 | -1, 0])) =>
        {
            traversal_orientation([line.dir().x, use_.edge_to_pcurve().scale()], fin.sense)
        }
        _ => None,
    };
    Ok(parameter_orientation)
}

/// Consume bounded analytic integral evidence only after independent
/// topology, whole-incidence, certified chart-closure, and simplicity proofs.
fn certify_bounded_analytic_loop_orientation(
    store: &Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
) -> Result<Option<Orientation>> {
    if certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified {
        return Ok(None);
    }
    let Some(prepared) = prepare_bounded_analytic_loop(store, face_id, loop_id)? else {
        return Ok(None);
    };
    let spans = prepared
        .iter()
        .copied()
        .map(BoundedLoopSpan::geometry)
        .collect::<Vec<_>>();

    Ok(match certify_signed_line_integral(&spans) {
        SignedLineIntegralProof::Certified(proof) => Some(proof.orientation()),
        SignedLineIntegralProof::Indeterminate(_) => None,
    })
}

pub(crate) fn prepare_bounded_analytic_loop<'a>(
    store: &'a Store,
    face_id: crate::entity::FaceId,
    loop_id: LoopId,
) -> Result<Option<Vec<BoundedLoopSpan<'a, VertexId>>>> {
    let loop_ = store.get(loop_id)?;
    if loop_.face != face_id || loop_.fins.len() < 2 {
        return Ok(None);
    }
    let face = store.get(face_id)?;
    let surface = store.get(face.surface)?;
    if !matches!(surface, SurfaceGeom::Plane(_) | SurfaceGeom::Cylinder(_)) {
        return Ok(None);
    }
    let Some(leaf_surface) = surface.as_leaf_surface() else {
        return Ok(None);
    };
    let periods = leaf_surface.periodicity();
    let mut spans = Vec::with_capacity(loop_.fins.len());
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
        if edge.tolerance.is_some() || use_.closure_winding().is_some() || use_.seam().is_some() {
            return Ok(None);
        }
        let map = use_.edge_to_pcurve();
        let q_lo = map.map(lo);
        let q_hi = map.map(hi);
        // Whole-fin incidence already certifies the authored active pcurve
        // range against this affine map with the kernel's conservative
        // parameter comparison. Requiring the recomputed endpoints to have
        // identical floating-point bits would reject an equivalent nonzero
        // offset/phase map after one rounded multiply-add.
        if !q_lo.is_finite() || !q_hi.is_finite() {
            return Ok(None);
        }
        let (edge_start, edge_end) = traversal_bounds(fin.sense, ParamRange::new(lo, hi));
        let start = map.map(edge_start);
        let end = map.map(edge_end);
        let curve = store.get(use_.curve())?;
        if !matches!(curve, Curve2dGeom::Line(_) | Curve2dGeom::Circle(_)) {
            return Ok(None);
        }
        let chart_offset = use_.chart().apply(Point2::default(), periods)?;
        spans.push(BoundedLoopSpan::new(
            BoundedPcurveSpan::new(curve, start, end, chart_offset),
            tail,
            head,
        ));
    }
    if !certify_bounded_chart_joins(surface, periods, &mut spans) {
        return Ok(None);
    }
    Ok(Some(spans))
}

fn certify_bounded_chart_joins<K: Copy + Eq>(
    surface: &SurfaceGeom,
    periods: [Option<f64>; 2],
    spans: &mut [BoundedLoopSpan<'_, K>],
) -> bool {
    certify_bounded_chart_lifts(surface, periods, spans).is_ok()
}

const MAX_BOUNDED_CHART_LIFT: i64 = 1_i64 << 40;

/// Typed reason that an ordered bounded loop could not be placed in one
/// coherent proof-local periodic chart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BoundedChartLiftGap {
    TooFewSpans,
    UnsupportedSurface,
    InvalidPeriod { direction: usize },
    TopologyDiscontinuity { span_index: usize },
    NonFiniteEndpoint { span_index: usize },
    AmbiguousPeriodLift { span_index: usize, direction: usize },
    PeriodLiftOverflow { span_index: usize, direction: usize },
    AccumulatedLiftOverflow { span_index: usize, direction: usize },
    NonzeroWinding { direction: usize },
    ShiftOverflow { span_index: usize, direction: usize },
    ModelJoinMismatch { span_index: usize },
}

/// Solve the integer chart gauge for a complete ordered loop before any
/// simplicity or signed-integral proof consumes its pcurves.
///
/// R1/R6: a relative lift is admitted only when outward interval arithmetic
/// places the exact endpoint delta strictly inside one nearest-integer period
/// cell. There is no sampling, epsilon-based rounding, or layout recognition.
/// Prefix propagation fixes span zero as the proof-local gauge anchor, and a
/// zero final sum proves that the bounded loop is contractible in every
/// periodic direction.
fn certify_bounded_chart_lifts<K: Copy + Eq>(
    surface: &SurfaceGeom,
    periods: [Option<f64>; 2],
    spans: &mut [BoundedLoopSpan<'_, K>],
) -> core::result::Result<(), BoundedChartLiftGap> {
    if spans.len() < 2 {
        return Err(BoundedChartLiftGap::TooFewSpans);
    }
    let model_tolerance = 2.0 * LINEAR_RESOLUTION;
    let Some(chart_neighborhood) = bounded_join_chart_neighborhood(surface, model_tolerance) else {
        return Err(BoundedChartLiftGap::UnsupportedSurface);
    };
    let Some(leaf_surface) = surface.as_leaf_surface() else {
        return Err(BoundedChartLiftGap::UnsupportedSurface);
    };
    for (direction, period) in periods.into_iter().enumerate() {
        if period.is_some_and(|period| !period.is_finite() || period <= 0.0) {
            return Err(BoundedChartLiftGap::InvalidPeriod { direction });
        }
    }

    let mut endpoints = Vec::with_capacity(spans.len());
    for (span_index, span) in spans.iter().copied().enumerate() {
        let Some(start) = bounded_span_endpoint(span.geometry(), true) else {
            return Err(BoundedChartLiftGap::NonFiniteEndpoint { span_index });
        };
        let Some(end) = bounded_span_endpoint(span.geometry(), false) else {
            return Err(BoundedChartLiftGap::NonFiniteEndpoint { span_index });
        };
        endpoints.push((start, end));
    }

    // `relative_lifts[i]` is the integer shift that span i+1 needs relative
    // to span i. It is inferred from the authored charts before any mutation,
    // so alternating representatives cannot make the result order-dependent.
    let mut relative_lifts = vec![[0_i64; 2]; spans.len()];
    for index in 0..spans.len() {
        let next = (index + 1) % spans.len();
        if spans[index].head() != spans[next].tail() {
            return Err(BoundedChartLiftGap::TopologyDiscontinuity { span_index: index });
        }
        let current_end = endpoints[index].1;
        let next_start = endpoints[next].0;
        for (direction, period) in periods.into_iter().enumerate() {
            let Some(period) = period else { continue };
            let values = if direction == 0 {
                [current_end.x, next_start.x]
            } else {
                [current_end.y, next_start.y]
            };
            relative_lifts[index][direction] =
                certified_integer_period_lift(values[0], values[1], period, index, direction)?;
        }
    }

    let mut lifts = vec![[0_i64; 2]; spans.len()];
    for direction in 0..2 {
        let mut accumulated = 0_i64;
        for index in 0..spans.len() - 1 {
            accumulated = checked_accumulated_lift(
                accumulated,
                relative_lifts[index][direction],
                index,
                direction,
            )?;
            lifts[index + 1][direction] = accumulated;
        }
        let closure = checked_accumulated_lift(
            accumulated,
            relative_lifts[spans.len() - 1][direction],
            spans.len() - 1,
            direction,
        )?;
        if closure != 0 {
            return Err(BoundedChartLiftGap::NonzeroWinding { direction });
        }
    }

    let mut lifted_spans = spans.to_vec();
    for (span_index, span) in lifted_spans.iter_mut().enumerate() {
        let geometry = span.geometry();
        let chart_offset = shifted_chart_offset(
            geometry.chart_offset(),
            periods,
            lifts[span_index],
            span_index,
        )?;
        *span = span.with_geometry(geometry.with_chart_offset(chart_offset));
    }

    // Whole-period propagation establishes a coherent chart. Rounded f64
    // additions need not produce bit-identical endpoint coordinates, so the
    // existing topology-owned, surface-lifted near-join certificate remains
    // the final authorization for any residual.
    for index in 0..lifted_spans.len() {
        let next = (index + 1) % lifted_spans.len();
        let Some(current_end) = bounded_span_endpoint(lifted_spans[index].geometry(), false) else {
            return Err(BoundedChartLiftGap::NonFiniteEndpoint { span_index: index });
        };
        let Some(next_start) = bounded_span_endpoint(lifted_spans[next].geometry(), true) else {
            return Err(BoundedChartLiftGap::NonFiniteEndpoint { span_index: next });
        };
        if point2_bits_equal(current_end, next_start) {
            continue;
        }
        let current_point = leaf_surface.eval([current_end.x, current_end.y]);
        let next_point = leaf_surface.eval([next_start.x, next_start.y]);
        if !certify_model_distance(current_point, next_point, model_tolerance) {
            return Err(BoundedChartLiftGap::ModelJoinMismatch { span_index: index });
        }
        let Some(evidence) = CertifiedBoundedLoopJoin::new(chart_neighborhood) else {
            return Err(BoundedChartLiftGap::UnsupportedSurface);
        };
        lifted_spans[index] = lifted_spans[index].with_head_join(evidence);
    }
    spans.copy_from_slice(&lifted_spans);
    Ok(())
}

fn certified_integer_period_lift(
    current: f64,
    next: f64,
    period: f64,
    span_index: usize,
    direction: usize,
) -> core::result::Result<i64, BoundedChartLiftGap> {
    if current.to_bits() == next.to_bits() {
        return Ok(0);
    }
    let Some(quotient) =
        (Interval::point(current) - Interval::point(next)).checked_div(Interval::point(period))
    else {
        return Err(BoundedChartLiftGap::PeriodLiftOverflow {
            span_index,
            direction,
        });
    };
    if !quotient.lo().is_finite() || !quotient.hi().is_finite() {
        return Err(BoundedChartLiftGap::PeriodLiftOverflow {
            span_index,
            direction,
        });
    }
    let midpoint = 0.5 * quotient.lo() + 0.5 * quotient.hi();
    let candidate = midpoint.round();
    if !candidate.is_finite() || candidate.abs() > MAX_BOUNDED_CHART_LIFT as f64 {
        return Err(BoundedChartLiftGap::PeriodLiftOverflow {
            span_index,
            direction,
        });
    }

    // Strict containment is essential: touching either half-period boundary
    // means two nearest integer lifts remain possible, so the proof fails.
    if quotient.lo() <= candidate - 0.5 || quotient.hi() >= candidate + 0.5 {
        return Err(BoundedChartLiftGap::AmbiguousPeriodLift {
            span_index,
            direction,
        });
    }
    Ok(candidate as i64)
}

fn checked_accumulated_lift(
    accumulated: i64,
    relative: i64,
    span_index: usize,
    direction: usize,
) -> core::result::Result<i64, BoundedChartLiftGap> {
    let Some(lift) = accumulated.checked_add(relative) else {
        return Err(BoundedChartLiftGap::AccumulatedLiftOverflow {
            span_index,
            direction,
        });
    };
    if lift.abs() > MAX_BOUNDED_CHART_LIFT {
        return Err(BoundedChartLiftGap::AccumulatedLiftOverflow {
            span_index,
            direction,
        });
    }
    Ok(lift)
}

fn shifted_chart_offset(
    chart_offset: Point2,
    periods: [Option<f64>; 2],
    lifts: [i64; 2],
    span_index: usize,
) -> core::result::Result<Point2, BoundedChartLiftGap> {
    let mut coordinates = [chart_offset.x, chart_offset.y];
    for direction in 0..2 {
        let Some(period) = periods[direction] else {
            if lifts[direction] != 0 {
                return Err(BoundedChartLiftGap::ShiftOverflow {
                    span_index,
                    direction,
                });
            }
            continue;
        };
        let shift = lifts[direction] as f64 * period;
        let shifted = coordinates[direction] + shift;
        if !shift.is_finite()
            || !shifted.is_finite()
            || lifts[direction] != 0 && shifted.to_bits() == coordinates[direction].to_bits()
        {
            return Err(BoundedChartLiftGap::ShiftOverflow {
                span_index,
                direction,
            });
        }
        coordinates[direction] = shifted;
    }
    Ok(Point2::new(coordinates[0], coordinates[1]))
}

fn bounded_join_chart_neighborhood(surface: &SurfaceGeom, model_tolerance: f64) -> Option<f64> {
    if !model_tolerance.is_finite() || model_tolerance < 0.0 {
        return None;
    }
    let tolerance = Interval::point(model_tolerance);
    let neighborhood = match surface {
        SurfaceGeom::Plane(_) => tolerance.lo(),
        SurfaceGeom::Cylinder(cylinder) => {
            // Line2d directions are normalized in (u,v). The cylinder metric
            // has maximum speed max(radius, 1). Keep the admitted parameter
            // radius no larger than model_tolerance / max(radius, 1); using
            // the outward quotient's lower bound cannot enlarge that radius.
            let metric_scale = Interval::point(cylinder.radius().max(1.0));
            tolerance.checked_div(metric_scale)?.lo()
        }
        _ => return None,
    };
    (neighborhood.is_finite() && neighborhood >= 0.0).then_some(neighborhood)
}

fn bounded_span_endpoint(span: BoundedPcurveSpan<'_>, start: bool) -> Option<Point2> {
    let parameter = if start { span.start() } else { span.end() };
    let point = span.curve().as_curve().eval(parameter) + span.chart_offset();
    (point.x.is_finite() && point.y.is_finite()).then_some(point)
}

fn point2_bits_equal(left: Point2, right: Point2) -> bool {
    left.x.to_bits() == right.x.to_bits() && left.y.to_bits() == right.y.to_bits()
}

fn certify_model_distance(left: Point3, right: Point3, tolerance: f64) -> bool {
    let distance_squared = [left.x, left.y, left.z]
        .into_iter()
        .zip([right.x, right.y, right.z])
        .fold(Interval::point(0.0), |sum, (left, right)| {
            sum + (Interval::point(left) - Interval::point(right)).square()
        });
    let allowed_squared = Interval::point(tolerance).square();
    distance_squared.hi().is_finite()
        && allowed_squared.lo().is_finite()
        && distance_squared.hi() <= allowed_squared.lo()
}

fn traversal_orientation(factors: [f64; 2], sense: Sense) -> Option<Orientation> {
    if factors
        .iter()
        .any(|factor| !factor.is_finite() || *factor == 0.0)
    {
        return None;
    }
    let negative = factors
        .iter()
        .filter(|factor| factor.is_sign_negative())
        .count()
        + usize::from(sense == Sense::Reversed);
    Some(if negative % 2 == 0 {
        Orientation::Positive
    } else {
        Orientation::Negative
    })
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
        let face_id = loop_.face;
        let Some(spans) = prepare_bounded_analytic_loop(store, face_id, loop_id)? else {
            return Ok(LoopSimplicity::Indeterminate);
        };
        return Ok(match certify_bounded_loop_simplicity(&spans) {
            BoundedLoopSimplicity::Certified => LoopSimplicity::Certified,
            BoundedLoopSimplicity::SelfIntersecting => LoopSimplicity::SelfIntersecting,
            BoundedLoopSimplicity::Indeterminate(_) => LoopSimplicity::Indeterminate,
        });
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
    Ok(if containment_outer_index(&rings).is_some() {
        LoopContainment::Certified
    } else {
        LoopContainment::Indeterminate
    })
}

/// Certify that all loops on one face form a supported non-overlapping face
/// boundary layout.
///
/// Plane faces retain strict outer/hole containment. A full-period cylinder
/// band instead has two disjoint constant-height periodic boundaries; neither
/// contains the other in an unwrapped chart. That layout is certified from
/// two distinct topology-owned whole-fin line pcurves, without cutting the
/// periodic surface at an artificial seam.
pub(crate) fn certify_face_loop_layout(
    store: &Store,
    face_id: crate::entity::FaceId,
) -> Result<LoopContainment> {
    let face = store.get(face_id)?;
    if face.loops.len() < 2 {
        return Ok(LoopContainment::Certified);
    }
    if matches!(store.get(face.surface)?, SurfaceGeom::Plane(_)) {
        let polygonal = certify_loop_containment(store, &face.loops)?;
        if polygonal == LoopContainment::Certified
            || certify_convex_polygon_circle_face(store, face_id)?
            || certify_analytic_face_layout(store, face_id)?
        {
            return Ok(LoopContainment::Certified);
        }
        return Ok(LoopContainment::Indeterminate);
    }
    Ok(if certify_analytic_face_layout(store, face_id)? {
        LoopContainment::Certified
    } else {
        LoopContainment::Indeterminate
    })
}

/// Budgeted Full-check entry point for face-loop layout certification.
pub(crate) fn certify_face_loop_layout_in_scope(
    store: &Store,
    face_id: crate::entity::FaceId,
    scope: &mut kcore::operation::OperationScope<'_, '_>,
) -> Result<LoopContainment> {
    let face = store.get(face_id)?;
    if face.loops.len() >= 2 {
        let mut fin_count = 0_usize;
        for &loop_id in &face.loops {
            fin_count = fin_count
                .checked_add(store.get(loop_id)?.fins.len())
                .ok_or(kcore::error::Error::InvalidGeometry {
                    reason: "face-loop containment fin count overflow",
                })?;
        }
        charge_face_loop_containment_work(scope, face.loops.len(), fin_count)?;
    }
    certify_face_loop_layout(store, face_id)
}

/// Certify that one circle lies strictly inside one convex polygon.
///
/// Every edge is treated as an oriented affine form over the complete circle.
/// Outward interval arithmetic bounds the harmonic amplitude, so a successful
/// result proves strict containment without sampling or a caller tolerance.
pub(crate) fn certify_convex_polygon_circle_containment(
    polygon: &[Point2],
    circle: Circle2d,
) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let mut winding = None;
    for index in 0..polygon.len() {
        let first = polygon[index];
        let second = polygon[(index + 1) % polygon.len()];
        let third = polygon[(index + 2) % polygon.len()];
        let turn = orient2d([first.x, first.y], [second.x, second.y], [third.x, third.y]);
        if turn == Orientation::Zero || winding.is_some_and(|expected| expected != turn) {
            return false;
        }
        winding = Some(turn);
    }
    let winding = winding.expect("a polygon with three vertices has a turn");
    let center = circle.center();
    let x = circle.x_dir();
    let y = x.perp();
    let radius = Interval::point(circle.radius());
    for index in 0..polygon.len() {
        let start = polygon[index];
        let end = polygon[(index + 1) % polygon.len()];
        let edge = [
            Interval::point(end.x) - Interval::point(start.x),
            Interval::point(end.y) - Interval::point(start.y),
        ];
        let offset = [
            Interval::point(center.x) - Interval::point(start.x),
            Interval::point(center.y) - Interval::point(start.y),
        ];
        let radial_x = [Interval::point(x.x) * radius, Interval::point(x.y) * radius];
        let radial_y = [Interval::point(y.x) * radius, Interval::point(y.y) * radius];
        let mut constant = cross2_interval(edge, offset);
        let cosine = cross2_interval(edge, radial_x);
        let sine = cross2_interval(edge, radial_y);
        if winding == Orientation::Negative {
            constant = -constant;
        }
        let Some(amplitude) = (cosine.square() + sine.square()).sqrt() else {
            return false;
        };
        if !constant.lo().is_finite()
            || !amplitude.hi().is_finite()
            || constant.lo() <= amplitude.hi()
        {
            return false;
        }
    }
    true
}

fn cross2_interval(left: [Interval; 2], right: [Interval; 2]) -> Interval {
    left[0] * right[1] - left[1] * right[0]
}

fn certify_convex_polygon_circle_face(
    store: &Store,
    face_id: crate::entity::FaceId,
) -> Result<bool> {
    let face = store.get(face_id)?;
    if face.loops.len() != 2 || !matches!(store.get(face.surface)?, SurfaceGeom::Plane(_)) {
        return Ok(false);
    }
    let mut polygon = None;
    let mut circle = None;
    for &loop_id in &face.loops {
        if let Some(segments) = planar_segment_ring(store, loop_id)? {
            if polygon.replace(segments).is_some() {
                return Ok(false);
            }
            continue;
        }
        let loop_ = store.get(loop_id)?;
        let [fin_id] = loop_.fins.as_slice() else {
            return Ok(false);
        };
        let fin = store.get(*fin_id)?;
        let Some(use_) = fin.pcurve else {
            return Ok(false);
        };
        let Curve2dGeom::Circle(value) = store.get(use_.curve())? else {
            return Ok(false);
        };
        if use_.closure_winding() != Some([0, 0])
            || certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified
            || circle.replace(*value).is_some()
        {
            return Ok(false);
        }
    }
    let (Some(polygon), Some(circle)) = (polygon, circle) else {
        return Ok(false);
    };
    let points = polygon
        .iter()
        .map(|segment| segment.start)
        .collect::<Vec<_>>();
    Ok(certify_convex_polygon_circle_containment(&points, circle))
}

/// Certify exact orientation and the unique outer identity for planar
/// straight-loop representations.
///
/// Unlike the tolerance-aware simplicity proof, this authority requires every
/// segment endpoint to equal the next segment start exactly. Sampled curves and
/// tolerance-joined chords remain unsupported and therefore indeterminate.
pub(crate) fn certify_planar_loop_layout(
    store: &Store,
    loop_ids: &[LoopId],
) -> Result<PlanarLoopLayout> {
    let mut orientations = Vec::with_capacity(loop_ids.len());
    let mut rings = Vec::with_capacity(loop_ids.len());
    let mut complete = !loop_ids.is_empty();
    for &loop_id in loop_ids {
        let certified = strict_planar_ring(store, loop_id)?;
        match certified {
            Some((segments, orientation)) => {
                orientations.push((loop_id, Some(orientation)));
                rings.push(segments);
            }
            None => {
                orientations.push((loop_id, None));
                complete = false;
            }
        }
    }
    let outer = complete
        .then(|| containment_outer_index(&rings))
        .flatten()
        .map(|index| loop_ids[index]);
    Ok(PlanarLoopLayout {
        outer,
        orientations,
    })
}

fn strict_planar_ring(
    store: &Store,
    loop_id: LoopId,
) -> Result<Option<(Vec<Segment2>, Orientation)>> {
    let Some(segments) = planar_segment_ring(store, loop_id)? else {
        return Ok(None);
    };
    let Some(orientation) = strict_ring_orientation(&segments) else {
        return Ok(None);
    };
    Ok(Some((segments, orientation)))
}

fn strict_ring_orientation(segments: &[Segment2]) -> Option<Orientation> {
    if !strict_segment_ring(segments) {
        return None;
    }
    match polygon_orientation2d_iter(
        segments
            .iter()
            .map(|segment| [segment.start.x, segment.start.y]),
    ) {
        Orientation::Zero => None,
        orientation => Some(orientation),
    }
}

fn strict_segment_ring(segments: &[Segment2]) -> bool {
    if segments.len() < 3 {
        return false;
    }
    for (index, segment) in segments.iter().enumerate() {
        let next = segments[(index + 1) % segments.len()];
        if !finite_point(segment.start)
            || !finite_point(segment.end)
            || segment.start == segment.end
            || !points_bit_equal(segment.end, next.start)
        {
            return false;
        }
    }
    for left in 0..segments.len() {
        for right in left + 1..segments.len() {
            let adjacent = right == left + 1 || left == 0 && right + 1 == segments.len();
            if adjacent {
                if adjacent_overlap(segments[left], segments[right], 0.0) {
                    return false;
                }
            } else if segments_intersect(segments[left], segments[right]) {
                return false;
            }
        }
    }
    true
}

fn points_bit_equal(first: Point2, second: Point2) -> bool {
    first.x.to_bits() == second.x.to_bits() && first.y.to_bits() == second.y.to_bits()
}

fn containment_outer_index(rings: &[Vec<Segment2>]) -> Option<usize> {
    for first in 0..rings.len() {
        for second in first + 1..rings.len() {
            if rings[first].iter().any(|&left| {
                rings[second]
                    .iter()
                    .any(|&right| segments_intersect(left, right))
            }) {
                return None;
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
        return None;
    };
    if containers
        .iter()
        .enumerate()
        .all(|(index, containers)| index == *outer || containers.as_slice() == [*outer])
    {
        Some(*outer)
    } else {
        None
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
            if let Some(segment) = verified_plane_line_vertex_segment(
                store,
                fin_id,
                pcurve,
                edge,
                face.surface,
                plane.frame(),
            )? {
                Some(segment)
            } else {
                pcurve_line_segment(store, pcurve, edge, fin.sense, range)?
            }
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

fn verified_plane_line_vertex_segment(
    store: &Store,
    fin_id: crate::entity::FinId,
    use_: FinPcurve,
    edge: &Edge,
    face_surface: crate::entity::SurfaceId,
    frame: &kgeom::frame::Frame,
) -> Result<Option<Segment2>> {
    let Some(curve_id) = edge.curve else {
        return Ok(None);
    };
    let Some(intersection) = store.get(curve_id)?.as_intersection() else {
        return Ok(None);
    };
    if intersection.certificate().as_plane_line().is_none() {
        return Ok(None);
    }
    let Some(source_index) = intersection
        .source_surfaces()
        .iter()
        .position(|surface| *surface == face_surface)
    else {
        return Ok(None);
    };
    if intersection.pcurves()[source_index] != use_.curve() {
        return Ok(None);
    }
    let (Some(tail), Some(head)) = (store.fin_tail(fin_id)?, store.fin_head(fin_id)?) else {
        return Ok(None);
    };
    Ok(Some(Segment2 {
        start: plane_uv(frame, store.vertex_position(tail)?),
        end: plane_uv(frame, store.vertex_position(head)?),
    }))
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
    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::surface::Plane;
    use kgeom::vec::Vec2;

    fn segment(start: [f64; 2], end: [f64; 2]) -> Segment2 {
        Segment2 {
            start: Point2::new(start[0], start[1]),
            end: Point2::new(end[0], end[1]),
        }
    }

    fn ring(points: &[[f64; 2]]) -> Vec<Segment2> {
        points
            .iter()
            .copied()
            .zip(points.iter().copied().cycle().skip(1))
            .take(points.len())
            .map(|(start, end)| segment(start, end))
            .collect()
    }

    fn rounded_twice_area(points: &[[f64; 2]]) -> f64 {
        points
            .iter()
            .copied()
            .zip(points.iter().copied().cycle().skip(1))
            .take(points.len())
            .map(|([x0, y0], [x1, y1])| x0 * y1 - x1 * y0)
            .sum()
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

    #[test]
    fn strict_ring_orientation_resolves_cancellation_and_rejects_unsafe_input() {
        const M: i64 = (1_i64 << 52) - 1;
        let coordinates = [[M, M], [M + 16, M], [M + 16, M + 16], [M, M + 16]];
        let points = coordinates.map(|[u, v]| [u as f64, v as f64]);
        assert_eq!(rounded_twice_area(&points), 0.0);
        assert_eq!(
            strict_ring_orientation(&ring(&points)),
            Some(Orientation::Positive)
        );
        assert_eq!(
            strict_ring_orientation(&ring(&points)),
            Some(Orientation::Positive)
        );

        let mut rotated = points;
        rotated.rotate_left(2);
        assert_eq!(
            strict_ring_orientation(&ring(&rotated)),
            Some(Orientation::Positive)
        );

        let mut reversed = points;
        reversed.reverse();
        assert_eq!(
            strict_ring_orientation(&ring(&reversed)),
            Some(Orientation::Negative)
        );

        let mut hole_points = [
            [M + 4, M + 4],
            [M + 4, M + 8],
            [M + 8, M + 8],
            [M + 8, M + 4],
        ]
        .map(|[u, v]| [u as f64, v as f64]);
        assert_eq!(rounded_twice_area(&hole_points), 0.0);
        let outer_ring = ring(&points);
        let hole_ring = ring(&hole_points);
        assert_eq!(
            strict_ring_orientation(&hole_ring),
            Some(Orientation::Negative)
        );
        assert_eq!(
            containment_outer_index(&[outer_ring.clone(), hole_ring.clone()]),
            Some(0)
        );
        assert_eq!(
            containment_outer_index(&[hole_ring.clone(), outer_ring.clone()]),
            Some(1)
        );
        hole_points.rotate_left(1);
        assert_eq!(
            containment_outer_index(&[ring(&hole_points), outer_ring]),
            Some(1)
        );

        let exact_zero = ring(&[[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]);
        assert_eq!(strict_ring_orientation(&exact_zero), None);

        let non_finite = ring(&[[0.0, 0.0], [f64::NAN, 0.0], [0.0, 1.0]]);
        assert_eq!(strict_ring_orientation(&non_finite), None);
        let infinite = ring(&[[0.0, 0.0], [f64::INFINITY, 0.0], [0.0, 1.0]]);
        assert_eq!(strict_ring_orientation(&infinite), None);

        let tolerance_joined = [
            segment([0.0, 0.0], [1.0, 0.0]),
            segment([1.0 + 0.5 * LINEAR_RESOLUTION, 0.0], [1.0, 1.0]),
            segment([1.0, 1.0], [0.0, 1.0]),
            segment([0.0, 1.0], [0.0, 0.0]),
        ];
        assert_eq!(
            certify_segment_ring(&tolerance_joined),
            LoopSimplicity::Certified
        );
        assert_eq!(strict_ring_orientation(&tolerance_joined), None);
    }

    #[test]
    fn certified_chart_join_mints_only_for_surface_lifted_near_incidence() {
        let prove = |epsilon: f64| {
            let curves = [
                Curve2dGeom::Line(Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
                Curve2dGeom::Line(
                    Line2d::new(Point2::new(1.0 + epsilon, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
                ),
                Curve2dGeom::Line(
                    Line2d::new(Point2::new(1.0, 1.0), Vec2::new(-1.0, 0.0)).unwrap(),
                ),
                Curve2dGeom::Line(
                    Line2d::new(Point2::new(0.0, 1.0), Vec2::new(0.0, -1.0)).unwrap(),
                ),
            ];
            let mut spans = (0..4)
                .map(|index| {
                    BoundedLoopSpan::new(
                        BoundedPcurveSpan::new(&curves[index], 0.0, 1.0, Point2::default()),
                        index,
                        (index + 1) % 4,
                    )
                })
                .collect::<Vec<_>>();
            let surface = SurfaceGeom::Plane(Plane::new(Frame::world()));
            certify_bounded_chart_joins(&surface, [None, None], &mut spans)
        };
        assert!(prove(5.0e-13));
        assert!(!prove(0.25));
    }
}
