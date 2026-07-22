//! Certified topology-owned clipping of affine ruling carriers.
//!
//! The carrier is supplied through one face's already-certified affine
//! pcurve.  This module reads the face's loops, fins, edges, pcurves, and
//! whole-fin incidence evidence directly; `FaceDomain` and graph discovery
//! windows never become trim authority.
//!
//! Supported trims are deliberately exact-family and fail closed:
//!
//! - polygonal loops (including holes and non-convex loops) on a plane, and
//! - vertex-less whole-period horizontal ring loops on a cylinder.
//!
//! Crossings retain conservative carrier- and source-edge-parameter
//! enclosures.  Root ordinals are intentionally absent: the section
//! operation's shared root-identity authority assigns them after both
//! operand clips have been collected.

use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, orient2d};
use kgeom::curve2d::Line2d;
use kgeom::param::ParamRange;
use kgeom::vec::Point2;
use ktopo::entity::{
    EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId, Loop, LoopId as RawLoopId,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use ktopo::store::Store;

use super::ruling_publish::RulingEndpointCoincidenceProof;
use super::{SECTION_WORK, SectionUvLine};
use crate::error::{Error, Result};

/// Stable failure classes for affine ruling trim and merge proofs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RulingClipGap {
    /// The surface, carrier pcurve, or exact boundary family is unsupported.
    UnsupportedTrim,
    /// Stored face/loop/fin ownership or closure is malformed.
    MalformedTrim,
    /// Outward arithmetic could not certify a finite non-degenerate result.
    ArithmeticGuard,
    /// The carrier touches a boundary without a certified positive crossing.
    TangentialContact,
    /// The carrier passes through a polygon vertex.
    VertexContact,
    /// The affine carrier is coincident with a trim boundary.
    CoincidentBoundary,
    /// Crossing or endpoint enclosures cannot be strictly ordered.
    UnorderedCrossings,
}

impl RulingClipGap {
    /// Stable diagnostic suitable for a public section gap.
    pub(crate) const fn reason(self) -> &'static str {
        match self {
            Self::UnsupportedTrim => "ruling clipping does not support this exact face trim class",
            Self::MalformedTrim => "ruling clipping requires a closed source-provenanced face trim",
            Self::ArithmeticGuard => {
                "ruling clipping could not certify a source-derived arithmetic guard"
            }
            Self::TangentialContact => {
                "a ruling has an unresolved tangent or zero-length trim contact"
            }
            Self::VertexContact => {
                "a ruling passes through a trim vertex this slice does not resolve"
            }
            Self::CoincidentBoundary => "a ruling is coincident with a source trim boundary",
            Self::UnorderedCrossings => "ruling trim crossings could not be certifiably ordered",
        }
    }
}

/// One topology-owned transverse crossing of an affine ruling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RulingTrimSite {
    pub(crate) face: RawFaceId,
    pub(crate) loop_id: RawLoopId,
    pub(crate) fin: RawFinId,
    pub(crate) edge: RawEdgeId,
    /// Conservative enclosure in the ruling carrier's canonical parameter.
    pub(crate) carrier_parameter: Interval,
    /// Conservative enclosure in the source edge's intrinsic parameter.
    pub(crate) edge_parameter: Interval,
}

/// One maximal positive-length portion certified inside one face trim.
///
/// Its physical endpoints may lie outside the graph carrier's discovery
/// range. They remain present so the other operand can supply the active
/// endpoints during two-list intersection. A merged span is discarded only
/// when it is certifiably disjoint from that discovery range; publication
/// expands and reissues the carrier proof around retained root enclosures.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RulingClipSpan {
    pub(crate) start: RulingTrimSite,
    pub(crate) end: RulingTrimSite,
}

/// Fail-closed result of clipping one affine carrier to one face.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum RulingClipOutcome {
    /// Strictly ordered, pairwise-disjoint maximal spans.
    Spans(Vec<RulingClipSpan>),
    /// Exact trimming could not certify a result.
    Indeterminate(RulingClipGap),
}

/// One endpoint of the intersection of two operand-local span lists.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MergedRulingEndpoint {
    /// Topology-owned endpoint contributor in operand order.
    pub(crate) sites: [Option<RulingTrimSite>; 2],
    /// Conservative carrier-parameter enclosure after endpoint selection.
    pub(crate) carrier_parameter: Interval,
    /// Intrinsic source-edge evidence corresponding exactly to `sites`.
    pub(crate) edge_parameters: [Option<Interval>; 2],
}

/// One positive-length intersection of two operand-local ruling spans.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MergedRulingSpan {
    pub(crate) start: MergedRulingEndpoint,
    pub(crate) end: MergedRulingEndpoint,
}

/// Fail-closed result of intersecting two ordered ruling-span lists.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum RulingMergeOutcome {
    Spans(Vec<MergedRulingSpan>),
    Indeterminate(RulingClipGap),
}

#[derive(Debug, Clone, Copy)]
struct IntervalPoint2 {
    x: Interval,
    y: Interval,
}

impl IntervalPoint2 {
    fn point(point: Point2) -> Self {
        Self {
            x: Interval::point(point.x),
            y: Interval::point(point.y),
        }
    }

    fn finite(self) -> bool {
        finite(self.x) && finite(self.y)
    }
}

#[derive(Debug, Clone, Copy)]
struct PlaneTrimSegment {
    face: RawFaceId,
    loop_id: RawLoopId,
    fin: RawFinId,
    edge: RawEdgeId,
    start: Point2,
    end: Point2,
    start_interval: IntervalPoint2,
    end_interval: IntervalPoint2,
    edge_parameters: [f64; 2],
}

fn read<T>(result: kcore::error::Result<T>) -> Result<T> {
    result.map_err(|source| Error::InconsistentTopology { source })
}

fn charge(scope: &mut OperationScope<'_, '_>, amount: u64) -> Result<()> {
    scope
        .ledger_mut()
        .charge(SECTION_WORK, amount)
        .map_err(Error::from)
}

fn finite(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

fn excludes_zero(value: Interval) -> bool {
    value.hi() < 0.0 || value.lo() > 0.0
}

fn intersect(x: Interval, y: Interval) -> Option<Interval> {
    let lo = x.lo().max(y.lo());
    let hi = x.hi().min(y.hi());
    (lo <= hi).then(|| Interval::new(lo, hi))
}

fn sub(a: IntervalPoint2, b: IntervalPoint2) -> IntervalPoint2 {
    IntervalPoint2 {
        x: a.x - b.x,
        y: a.y - b.y,
    }
}

fn cross(a: IntervalPoint2, b: IntervalPoint2) -> Interval {
    a.x * b.y - a.y * b.x
}

fn certified_line_side(trace: SectionUvLine, point: IntervalPoint2) -> Option<Orientation> {
    let origin = IntervalPoint2::point(trace.origin());
    let direction = IntervalPoint2::point(Point2::new(trace.direction().x, trace.direction().y));
    let side = cross(direction, sub(point, origin));
    if side.hi() < 0.0 {
        Some(Orientation::Negative)
    } else if side.lo() > 0.0 {
        Some(Orientation::Positive)
    } else {
        None
    }
}

fn mapped_parameter(scale: f64, offset: f64, parameter: f64) -> Option<Interval> {
    let value = Interval::point(scale) * Interval::point(parameter) + Interval::point(offset);
    finite(value).then_some(value)
}

fn line_point(line: &Line2d, parameter: f64) -> Option<Point2> {
    let point = line.origin() + line.dir() * parameter;
    [point.x, point.y]
        .into_iter()
        .all(f64::is_finite)
        .then_some(point)
}

fn interval_line_point(line: &Line2d, parameter: Interval) -> Option<IntervalPoint2> {
    let origin = IntervalPoint2::point(line.origin());
    let direction = IntervalPoint2::point(Point2::new(line.dir().x, line.dir().y));
    let point = IntervalPoint2 {
        x: origin.x + direction.x * parameter,
        y: origin.y + direction.y * parameter,
    };
    point.finite().then_some(point)
}

fn valid_trace(trace: SectionUvLine, range: ParamRange) -> bool {
    let origin = trace.origin();
    let direction = trace.direction();
    let direction_norm = direction.norm();
    [
        origin.x,
        origin.y,
        direction.x,
        direction.y,
        range.lo,
        range.hi,
    ]
    .into_iter()
    .all(f64::is_finite)
        && direction_norm.is_finite()
        && direction_norm > 0.0
        && range.lo < range.hi
}

/// Clip one certified affine ruling pcurve to an exact plane or cylinder face
/// trim. The face surface selects the admitted topology family.
pub(crate) fn clip_ruling_to_face(
    store: &Store,
    face: RawFaceId,
    trace: SectionUvLine,
    carrier_range: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<RulingClipOutcome> {
    charge(scope, 1)?;
    if !valid_trace(trace, carrier_range) {
        return Ok(RulingClipOutcome::Indeterminate(
            RulingClipGap::UnsupportedTrim,
        ));
    }
    let face_data = read(store.get(face))?;
    let surface = read(store.surface(face_data.surface))?;
    match surface {
        SurfaceGeom::Plane(_) => clip_line_to_planar_trim(store, face, trace, carrier_range, scope),
        SurfaceGeom::Cylinder(_) => {
            clip_longitude_to_periodic_trim(store, face, trace, carrier_range, scope)
        }
        _ => Ok(RulingClipOutcome::Indeterminate(
            RulingClipGap::UnsupportedTrim,
        )),
    }
}

fn clip_line_to_planar_trim(
    store: &Store,
    face: RawFaceId,
    trace: SectionUvLine,
    carrier_range: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<RulingClipOutcome> {
    let segments = match prepare_plane_segments(store, face, scope)? {
        Ok(segments) => segments,
        Err(gap) => return Ok(RulingClipOutcome::Indeterminate(gap)),
    };
    let origin = trace.origin();
    let direction = trace.direction();
    let second = origin + direction;
    let mut signs = Vec::with_capacity(segments.len());
    let mut coincident_boundary = false;
    let mut vertex_contact = false;
    for segment in &segments {
        charge(scope, 1)?;
        let start = orient2d(
            [origin.x, origin.y],
            [second.x, second.y],
            [segment.start.x, segment.start.y],
        );
        let end = orient2d(
            [origin.x, origin.y],
            [second.x, second.y],
            [segment.end.x, segment.end.y],
        );
        if start == Orientation::Zero && end == Orientation::Zero {
            coincident_boundary = true;
        } else if start == Orientation::Zero || end == Orientation::Zero {
            vertex_contact = true;
        }
        signs.push([
            certified_line_side(trace, segment.start_interval),
            certified_line_side(trace, segment.end_interval),
        ]);
    }
    // Classification is independent of stored fin order: a coincident edge
    // is the stronger defect even though its endpoints also appear as zero
    // signs on the adjacent fins.
    if coincident_boundary {
        return Ok(RulingClipOutcome::Indeterminate(
            RulingClipGap::CoincidentBoundary,
        ));
    }
    if vertex_contact {
        return Ok(RulingClipOutcome::Indeterminate(
            RulingClipGap::VertexContact,
        ));
    }
    if signs.iter().flatten().any(Option::is_none) {
        return Ok(RulingClipOutcome::Indeterminate(
            RulingClipGap::ArithmeticGuard,
        ));
    }

    let mut crossings = Vec::new();
    for (segment, [start, end]) in segments.iter().zip(signs) {
        let (Some(start), Some(end)) = (start, end) else {
            return Ok(RulingClipOutcome::Indeterminate(
                RulingClipGap::ArithmeticGuard,
            ));
        };
        if start == end {
            continue;
        }
        let site = match plane_segment_crossing(trace, segment) {
            Some(site) => site,
            None => {
                return Ok(RulingClipOutcome::Indeterminate(
                    RulingClipGap::ArithmeticGuard,
                ));
            }
        };
        crossings.push(site);
    }
    finish_crossings(crossings, carrier_range, scope)
}

fn prepare_plane_segments(
    store: &Store,
    face: RawFaceId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<core::result::Result<Vec<PlaneTrimSegment>, RulingClipGap>> {
    let face_data = read(store.get(face))?;
    if face_data.loops().is_empty() {
        return Ok(Err(RulingClipGap::MalformedTrim));
    }
    let mut segments = Vec::new();
    for &loop_id in face_data.loops() {
        charge(scope, 1)?;
        let ring = read(store.get::<Loop>(loop_id))?;
        if ring.fins().len() < 3 {
            return Ok(Err(RulingClipGap::MalformedTrim));
        }
        let mut first_tail = None;
        let mut previous_head = None;
        for &fin_id in ring.fins() {
            charge(scope, 1)?;
            let fin = read(store.get(fin_id))?;
            if ring.face != face
                || fin.parent != loop_id
                || certify_whole_fin_incidence(
                    store,
                    face,
                    loop_id,
                    fin_id,
                    scope.context().tolerances().linear(),
                ) != WholeFinIncidence::Certified
            {
                return Ok(Err(RulingClipGap::MalformedTrim));
            }
            let edge = read(store.get(fin.edge))?;
            let (Some(v0), Some(v1), Some((lo, hi)), Some(curve_id), Some(use_)) = (
                edge.vertices[0],
                edge.vertices[1],
                edge.bounds,
                edge.curve,
                fin.pcurve,
            ) else {
                return Ok(Err(RulingClipGap::UnsupportedTrim));
            };
            if !lo.is_finite()
                || !hi.is_finite()
                || lo >= hi
                || !matches!(read(store.curve(curve_id))?, CurveGeom::Line(_))
                || !use_.chart().is_identity()
                || use_.closure_winding().is_some()
                || use_.seam().is_some()
            {
                return Ok(Err(RulingClipGap::UnsupportedTrim));
            }
            let Curve2dGeom::Line(line) = read(store.pcurve(use_.curve()))? else {
                return Ok(Err(RulingClipGap::UnsupportedTrim));
            };
            let (tail, head, edge_parameters) = if fin.sense.is_forward() {
                (v0, v1, [lo, hi])
            } else {
                (v1, v0, [hi, lo])
            };
            if previous_head.is_some_and(|previous| previous != tail) {
                return Ok(Err(RulingClipGap::MalformedTrim));
            }
            first_tail.get_or_insert(tail);
            previous_head = Some(head);

            let map = use_.edge_to_pcurve();
            let mapped = [map.map(edge_parameters[0]), map.map(edge_parameters[1])];
            let active = use_.range();
            if mapped.iter().any(|parameter| {
                !parameter.is_finite() || *parameter < active.lo || *parameter > active.hi
            }) {
                return Ok(Err(RulingClipGap::MalformedTrim));
            }
            let (Some(start_parameter), Some(end_parameter)) = (
                mapped_parameter(map.scale(), map.offset(), edge_parameters[0]),
                mapped_parameter(map.scale(), map.offset(), edge_parameters[1]),
            ) else {
                return Ok(Err(RulingClipGap::ArithmeticGuard));
            };
            let (Some(start), Some(end), Some(start_interval), Some(end_interval)) = (
                line_point(line, mapped[0]),
                line_point(line, mapped[1]),
                interval_line_point(line, start_parameter),
                interval_line_point(line, end_parameter),
            ) else {
                return Ok(Err(RulingClipGap::ArithmeticGuard));
            };
            if start == end {
                return Ok(Err(RulingClipGap::MalformedTrim));
            }
            segments.push(PlaneTrimSegment {
                face,
                loop_id,
                fin: fin_id,
                edge: fin.edge,
                start,
                end,
                start_interval,
                end_interval,
                edge_parameters,
            });
        }
        if first_tail != previous_head {
            return Ok(Err(RulingClipGap::MalformedTrim));
        }
    }
    Ok(Ok(segments))
}

fn plane_segment_crossing(
    trace: SectionUvLine,
    segment: &PlaneTrimSegment,
) -> Option<RulingTrimSite> {
    let origin = IntervalPoint2::point(trace.origin());
    let direction = IntervalPoint2::point(Point2::new(trace.direction().x, trace.direction().y));
    let edge_direction = sub(segment.end_interval, segment.start_interval);
    let relative = sub(segment.start_interval, origin);
    let denominator = cross(direction, edge_direction);
    if !excludes_zero(denominator) {
        return None;
    }
    let carrier_parameter = cross(relative, edge_direction).checked_div(denominator)?;
    let fraction = cross(relative, direction).checked_div(denominator)?;
    if !finite(carrier_parameter) || !finite(fraction) {
        return None;
    }
    let fraction = intersect(fraction, Interval::new(0.0, 1.0))?;
    let [edge_start, edge_end] = segment.edge_parameters;
    let edge_parameter = Interval::point(edge_start)
        + fraction * (Interval::point(edge_end) - Interval::point(edge_start));
    let active = Interval::new(edge_start.min(edge_end), edge_start.max(edge_end));
    let edge_parameter = intersect(edge_parameter, active)?;
    finite(edge_parameter).then_some(RulingTrimSite {
        face: segment.face,
        loop_id: segment.loop_id,
        fin: segment.fin,
        edge: segment.edge,
        carrier_parameter,
        edge_parameter,
    })
}

fn clip_longitude_to_periodic_trim(
    store: &Store,
    face: RawFaceId,
    trace: SectionUvLine,
    carrier_range: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<RulingClipOutcome> {
    let origin = trace.origin();
    let direction = trace.direction();
    if direction.x != 0.0 || direction.y == 0.0 {
        return Ok(RulingClipOutcome::Indeterminate(
            RulingClipGap::UnsupportedTrim,
        ));
    }
    let face_data = read(store.get(face))?;
    if face_data.loops().is_empty() {
        return Ok(RulingClipOutcome::Indeterminate(
            RulingClipGap::MalformedTrim,
        ));
    }
    let mut crossings = Vec::with_capacity(face_data.loops().len());
    for &loop_id in face_data.loops() {
        charge(scope, 1)?;
        let ring = read(store.get::<Loop>(loop_id))?;
        let [fin_id] = ring.fins() else {
            return Ok(RulingClipOutcome::Indeterminate(
                RulingClipGap::UnsupportedTrim,
            ));
        };
        let fin = read(store.get(*fin_id))?;
        let edge = read(store.get(fin.edge))?;
        let (Some(curve_id), Some(use_)) = (edge.curve, fin.pcurve) else {
            return Ok(RulingClipOutcome::Indeterminate(
                RulingClipGap::UnsupportedTrim,
            ));
        };
        if ring.face != face
            || fin.parent != loop_id
            || certify_whole_fin_incidence(
                store,
                face,
                loop_id,
                *fin_id,
                scope.context().tolerances().linear(),
            ) != WholeFinIncidence::Certified
            || edge.vertices != [None, None]
            || edge.bounds.is_some()
            || !matches!(read(store.curve(curve_id))?, CurveGeom::Circle(_))
            || !matches!(use_.closure_winding(), Some([1 | -1, 0]))
            || use_.seam().is_some()
            || use_.chart().period_shifts()[1] != 0
        {
            return Ok(RulingClipOutcome::Indeterminate(
                RulingClipGap::UnsupportedTrim,
            ));
        }
        let Curve2dGeom::Line(boundary) = read(store.pcurve(use_.curve()))? else {
            return Ok(RulingClipOutcome::Indeterminate(
                RulingClipGap::UnsupportedTrim,
            ));
        };
        let winding = use_
            .closure_winding()
            .expect("validated whole-period winding");
        let rate = boundary.dir().x * use_.edge_to_pcurve().scale();
        let winding_matches_rate = winding[0] == 1 && rate > 0.0 || winding[0] == -1 && rate < 0.0;
        if boundary.dir().x == 0.0 || boundary.dir().y != 0.0 || !winding_matches_rate {
            return Ok(RulingClipOutcome::Indeterminate(
                RulingClipGap::UnsupportedTrim,
            ));
        }
        let boundary_height = Interval::point(boundary.origin().y);
        let carrier_parameter = match (boundary_height - Interval::point(origin.y))
            .checked_div(Interval::point(direction.y))
        {
            Some(parameter) if finite(parameter) => parameter,
            _ => {
                return Ok(RulingClipOutcome::Indeterminate(
                    RulingClipGap::ArithmeticGuard,
                ));
            }
        };
        let edge_parameter = match periodic_edge_parameter(origin.x, boundary, use_) {
            Some(parameter) => parameter,
            None => {
                return Ok(RulingClipOutcome::Indeterminate(
                    RulingClipGap::ArithmeticGuard,
                ));
            }
        };
        crossings.push(RulingTrimSite {
            face,
            loop_id,
            fin: *fin_id,
            edge: fin.edge,
            carrier_parameter,
            edge_parameter,
        });
    }
    finish_crossings(crossings, carrier_range, scope)
}

fn periodic_edge_parameter(
    trace_longitude: f64,
    boundary: &Line2d,
    use_: ktopo::entity::FinPcurve,
) -> Option<Interval> {
    let active = use_.range();
    let map = use_.edge_to_pcurve();
    let direction = boundary.dir();
    let [chart_winding, chart_height] = use_.chart().period_shifts();
    let [closure_winding, closure_height] = use_.closure_winding()?;
    let period = core::f64::consts::TAU;

    // This slice admits exactly one horizontal longitude period, expressed in
    // either source-edge direction and in any integer U chart.  Exact authored
    // equality deliberately narrows the metadata class beyond the tolerant
    // whole-fin checker so the integer search below has a proved finite bound.
    let longitude_span = direction.x.abs() * active.width();
    let edge_parameters = [map.inverse(active.lo), map.inverse(active.hi)];
    let rate = direction.x * map.scale();
    let winding_matches_rate =
        closure_winding == 1 && rate > 0.0 || closure_winding == -1 && rate < 0.0;
    if !trace_longitude.is_finite()
        || !active.is_finite()
        || direction.x == 0.0
        || direction.y != 0.0
        || chart_height != 0
        || closure_height != 0
        || !matches!(closure_winding, 1 | -1)
        || !winding_matches_rate
        || longitude_span != period
        || edge_parameters.into_iter().any(|value| !value.is_finite())
        || edge_parameters[0].min(edge_parameters[1]) != 0.0
        || edge_parameters[0].max(edge_parameters[1]) != period
        || use_.seam().is_some()
    {
        return None;
    }

    let chart_shift = Interval::point(f64::from(chart_winding)) * Interval::point(period);
    let longitude_at = |parameter: f64| {
        Interval::point(boundary.origin().x)
            + Interval::point(direction.x) * Interval::point(parameter)
            + chart_shift
    };
    let endpoints = [longitude_at(active.lo), longitude_at(active.hi)];
    if endpoints.into_iter().any(|value| !finite(value)) {
        return None;
    }
    let active_longitudes = Interval::new(
        endpoints[0].lo().min(endpoints[1].lo()),
        endpoints[0].hi().max(endpoints[1].hi()),
    );
    let possible_windings = (active_longitudes - Interval::point(trace_longitude))
        .checked_div(Interval::point(period))?;
    if !finite(possible_windings) {
        return None;
    }
    let first = possible_windings.lo().ceil();
    let last = possible_windings.hi().floor();
    if !first.is_finite()
        || !last.is_finite()
        || first > last
        || first < i32::MIN as f64
        || last > i32::MAX as f64
        || last - first > 2.0
    {
        return None;
    }

    let mut candidate = None;
    for winding in (first as i32)..=(last as i32) {
        let winding = f64::from(winding);
        let numerator = Interval::point(trace_longitude)
            + Interval::point(winding) * Interval::point(period)
            - chart_shift
            - Interval::point(boundary.origin().x);
        let q = numerator.checked_div(Interval::point(direction.x))?;
        if !finite(q) {
            return None;
        }
        if q.hi() < active.lo || q.lo() >= active.hi {
            continue;
        }
        // Membership is an interval proof.  Never select from a rounded
        // representative and then truncate the enclosure to the active range:
        // that can discard the exact root while manufacturing in-range data.
        if q.lo() < active.lo || q.hi() >= active.hi {
            return None;
        }
        let edge_parameter =
            (q - Interval::point(map.offset())).checked_div(Interval::point(map.scale()))?;
        if !finite(edge_parameter) || candidate.replace(edge_parameter).is_some() {
            return None;
        }
    }
    candidate
}

fn finish_crossings(
    mut crossings: Vec<RulingTrimSite>,
    _carrier_range: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<RulingClipOutcome> {
    charge(scope, crossings.len() as u64)?;
    crossings.sort_by(|a, b| {
        a.carrier_parameter
            .lo()
            .total_cmp(&b.carrier_parameter.lo())
            .then(
                a.carrier_parameter
                    .hi()
                    .total_cmp(&b.carrier_parameter.hi()),
            )
    });
    if crossings
        .iter()
        .any(|site| !finite(site.carrier_parameter) || !finite(site.edge_parameter))
        || crossings
            .windows(2)
            .any(|pair| pair[0].carrier_parameter.hi() >= pair[1].carrier_parameter.lo())
        || !crossings.len().is_multiple_of(2)
    {
        return Ok(RulingClipOutcome::Indeterminate(
            RulingClipGap::UnorderedCrossings,
        ));
    }
    // A zero-crossing containing case is impossible for the admitted
    // families: plane loops are bounded polygons, and every admitted
    // horizontal cylinder ring crosses a constant-longitude ruling once.
    // Therefore zero crossings certify an empty per-face intersection.
    let spans = crossings
        .chunks_exact(2)
        .map(|pair| RulingClipSpan {
            start: pair[0],
            end: pair[1],
        })
        .collect();
    Ok(RulingClipOutcome::Spans(spans))
}

fn filter_merged_span_by_discovery_range(
    span: MergedRulingSpan,
    range: ParamRange,
) -> core::result::Result<Option<MergedRulingSpan>, RulingClipGap> {
    if span.end.carrier_parameter.hi() <= range.lo || span.start.carrier_parameter.lo() >= range.hi
    {
        return Ok(None);
    }
    // Retain topology-owned parameters through identity certification. The
    // facade adapter later intersects the certified source-root projection
    // with this topology enclosure, expands the carrier range around the
    // result, and reissues the paired trace proof. A pcurve-derived enclosure
    // may straddle the original range under a rigid frame even when both name
    // the same exact source root.
    Ok(Some(span))
}

fn validate_spans(spans: &[RulingClipSpan]) -> bool {
    spans.iter().all(|span| {
        finite(span.start.carrier_parameter)
            && finite(span.end.carrier_parameter)
            && finite(span.start.edge_parameter)
            && finite(span.end.edge_parameter)
            && span.start.carrier_parameter.hi() < span.end.carrier_parameter.lo()
    }) && spans
        .windows(2)
        .all(|pair| pair[0].end.carrier_parameter.hi() < pair[1].start.carrier_parameter.lo())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StrictOrder {
    Before,
    After,
    Overlap,
}

fn strict_order(a: Interval, b: Interval) -> StrictOrder {
    if a.hi() < b.lo() {
        StrictOrder::Before
    } else if b.hi() < a.lo() {
        StrictOrder::After
    } else {
        StrictOrder::Overlap
    }
}

fn merged_endpoint(site: RulingTrimSite, operand: usize) -> MergedRulingEndpoint {
    let mut sites = [None, None];
    sites[operand] = Some(site);
    let mut edge_parameters = [None, None];
    edge_parameters[operand] = Some(site.edge_parameter);
    MergedRulingEndpoint {
        sites,
        carrier_parameter: site.carrier_parameter,
        edge_parameters,
    }
}

fn coincident_endpoint(
    left: RulingTrimSite,
    right: RulingTrimSite,
    proof: Option<&RulingEndpointCoincidenceProof>,
) -> Option<MergedRulingEndpoint> {
    proof.copied().filter(|proof| proof.proves(left, right))?;
    Some(MergedRulingEndpoint {
        sites: [Some(left), Some(right)],
        carrier_parameter: Interval::new(
            left.carrier_parameter
                .lo()
                .min(right.carrier_parameter.lo()),
            left.carrier_parameter
                .hi()
                .max(right.carrier_parameter.hi()),
        ),
        edge_parameters: [Some(left.edge_parameter), Some(right.edge_parameter)],
    })
}

/// Intersect two strictly ordered operand-local span lists, retaining common
/// spans that could overlap the graph discovery range.
///
/// Endpoint order needs strict interval separation or an exact source-edge
/// coincidence proof; the generic path refuses overlapping enclosures.
/// Single-point contacts do not become zero-length spans. This stage never
/// truncates topology endpoints to the discovery range; publication reissues
/// the carrier proof over retained root enclosures.
pub(crate) fn merge_ruling_spans(
    a: &[RulingClipSpan],
    b: &[RulingClipSpan],
    carrier_range: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<RulingMergeOutcome> {
    merge_ruling_spans_impl(a, b, carrier_range, None, scope)
}

/// Merge with exact semantic-root authority for selected endpoint edge pairs.
pub(crate) fn merge_ruling_spans_with_endpoint_proof(
    a: &[RulingClipSpan],
    b: &[RulingClipSpan],
    carrier_range: ParamRange,
    proof: &RulingEndpointCoincidenceProof,
    scope: &mut OperationScope<'_, '_>,
) -> Result<RulingMergeOutcome> {
    merge_ruling_spans_impl(a, b, carrier_range, Some(proof), scope)
}

fn merge_ruling_spans_impl(
    a: &[RulingClipSpan],
    b: &[RulingClipSpan],
    carrier_range: ParamRange,
    endpoint_proof: Option<&RulingEndpointCoincidenceProof>,
    scope: &mut OperationScope<'_, '_>,
) -> Result<RulingMergeOutcome> {
    if !carrier_range.is_finite()
        || carrier_range.lo >= carrier_range.hi
        || !validate_spans(a)
        || !validate_spans(b)
    {
        return Ok(RulingMergeOutcome::Indeterminate(
            RulingClipGap::UnorderedCrossings,
        ));
    }
    let mut spans = Vec::new();
    let (mut i, mut j) = (0, 0);
    while i < a.len() && j < b.len() {
        charge(scope, 1)?;
        let (left, right) = (a[i], b[j]);
        if left.end.carrier_parameter.hi() < right.start.carrier_parameter.lo() {
            i += 1;
            continue;
        }
        if right.end.carrier_parameter.hi() < left.start.carrier_parameter.lo() {
            j += 1;
            continue;
        }
        if left.start.carrier_parameter.hi() >= right.end.carrier_parameter.lo()
            || right.start.carrier_parameter.hi() >= left.end.carrier_parameter.lo()
        {
            return Ok(RulingMergeOutcome::Indeterminate(
                RulingClipGap::TangentialContact,
            ));
        }
        let start = match strict_order(left.start.carrier_parameter, right.start.carrier_parameter)
        {
            StrictOrder::Before => merged_endpoint(right.start, 1),
            StrictOrder::After => merged_endpoint(left.start, 0),
            StrictOrder::Overlap => {
                match coincident_endpoint(left.start, right.start, endpoint_proof) {
                    Some(endpoint) => endpoint,
                    None => {
                        return Ok(RulingMergeOutcome::Indeterminate(
                            RulingClipGap::UnorderedCrossings,
                        ));
                    }
                }
            }
        };
        let (end, exhausted_a, exhausted_b) =
            match strict_order(left.end.carrier_parameter, right.end.carrier_parameter) {
                StrictOrder::Before => (merged_endpoint(left.end, 0), true, false),
                StrictOrder::After => (merged_endpoint(right.end, 1), false, true),
                StrictOrder::Overlap => {
                    match coincident_endpoint(left.end, right.end, endpoint_proof) {
                        Some(endpoint) => (endpoint, true, true),
                        None => {
                            return Ok(RulingMergeOutcome::Indeterminate(
                                RulingClipGap::UnorderedCrossings,
                            ));
                        }
                    }
                }
            };
        if start.carrier_parameter.hi() >= end.carrier_parameter.lo() {
            return Ok(RulingMergeOutcome::Indeterminate(
                RulingClipGap::TangentialContact,
            ));
        }
        match filter_merged_span_by_discovery_range(MergedRulingSpan { start, end }, carrier_range)
        {
            Ok(Some(span)) => spans.push(span),
            Ok(None) => {}
            Err(gap) => return Ok(RulingMergeOutcome::Indeterminate(gap)),
        }
        if exhausted_a {
            i += 1;
        }
        if exhausted_b {
            j += 1;
        }
    }
    Ok(RulingMergeOutcome::Spans(spans))
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::vec::Vec2;
    use ktopo::entity::{BodyId as RawBodyId, FinPcurve, ParamMap1d, PcurveChart};
    use ktopo::profile::PlanarProfile;

    use super::*;
    use crate::section::BodySectionBudgetProfile;

    fn with_scope<T>(run: impl FnOnce(&mut OperationScope<'_, '_>) -> T) -> T {
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        run(&mut scope)
    }

    fn face_where(
        store: &Store,
        body: RawBodyId,
        predicate: impl Fn(&SurfaceGeom) -> bool,
    ) -> RawFaceId {
        store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .find(|face| predicate(store.surface(store.get(*face).unwrap().surface).unwrap()))
            .unwrap()
    }

    fn trace(origin: [f64; 2], direction: [f64; 2]) -> SectionUvLine {
        SectionUvLine {
            origin: Point2::new(origin[0], origin[1]),
            direction: Vec2::new(direction[0], direction[1]),
        }
    }

    fn periodic_use(
        store: &mut Store,
        boundary: Line2d,
        range: ParamRange,
        map: ParamMap1d,
        winding: [i32; 2],
        chart: [i32; 2],
    ) -> FinPcurve {
        let curve = store.insert_pcurve(Curve2dGeom::Line(boundary)).unwrap();
        FinPcurve::new(curve, range, map)
            .unwrap()
            .with_closure_winding(winding)
            .with_chart(PcurveChart::shifted(chart))
    }

    fn synthetic_span(template: RulingClipSpan, range: [f64; 2]) -> RulingClipSpan {
        RulingClipSpan {
            start: RulingTrimSite {
                carrier_parameter: Interval::point(range[0]),
                edge_parameter: Interval::point(range[0]),
                ..template.start
            },
            end: RulingTrimSite {
                carrier_parameter: Interval::point(range[1]),
                edge_parameter: Interval::point(range[1]),
                ..template.end
            },
        }
    }

    #[test]
    fn gap_reasons_are_stable_and_distinct() {
        let gaps = [
            RulingClipGap::UnsupportedTrim,
            RulingClipGap::MalformedTrim,
            RulingClipGap::ArithmeticGuard,
            RulingClipGap::TangentialContact,
            RulingClipGap::VertexContact,
            RulingClipGap::CoincidentBoundary,
            RulingClipGap::UnorderedCrossings,
        ];
        let reasons = gaps.map(RulingClipGap::reason);
        assert!(reasons.iter().all(|reason| !reason.is_empty()));
        for (index, reason) in reasons.iter().enumerate() {
            assert!(!reasons[..index].contains(reason));
        }
    }

    #[test]
    fn polygon_line_clip_retains_strict_topology_owned_endpoints() {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [4.0, 4.0, 1.0]).unwrap();
        let face = face_where(
            &store,
            body,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let outcome = with_scope(|scope| {
            clip_ruling_to_face(
                &store,
                face,
                trace([0.0, 0.0], [1.0, 0.0]),
                ParamRange::new(-3.0, 3.0),
                scope,
            )
            .unwrap()
        });
        let RulingClipOutcome::Spans(spans) = outcome else {
            panic!("expected one polygon span, got {outcome:?}");
        };
        let [span] = spans.as_slice() else {
            panic!("expected one polygon span, got {spans:?}");
        };
        assert_eq!(span.start.face, face);
        assert_eq!(span.end.face, face);
        assert_ne!(span.start.edge, span.end.edge);
        assert!(span.start.carrier_parameter.hi() < span.end.carrier_parameter.lo());
        assert!(finite(span.start.edge_parameter));
        assert!(finite(span.end.edge_parameter));
    }

    #[test]
    fn polygon_hole_and_nonconvex_outer_produce_maximal_affine_spans() {
        let mut store = Store::new();
        let outer = [
            Point2::new(-3.0, -2.0),
            Point2::new(3.0, -2.0),
            Point2::new(3.0, 2.0),
            Point2::new(1.0, 2.0),
            Point2::new(1.0, 1.0),
            Point2::new(-3.0, 1.0),
        ];
        let hole = [
            Point2::new(-1.5, -1.0),
            Point2::new(0.5, -1.0),
            Point2::new(0.5, 0.5),
            Point2::new(-1.5, 0.5),
        ];
        let profile =
            PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&hole]).unwrap();
        let body = ktopo::make::extrude_profile(&mut store, &profile, 1.0).unwrap();
        let face = face_where(
            &store,
            body,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let outcome = with_scope(|scope| {
            clip_ruling_to_face(
                &store,
                face,
                trace([0.0, 0.0], [1.0, 0.0]),
                ParamRange::new(-4.0, 4.0),
                scope,
            )
            .unwrap()
        });
        let RulingClipOutcome::Spans(spans) = outcome else {
            panic!("expected holed polygon spans, got {outcome:?}");
        };
        let [first, second] = spans.as_slice() else {
            panic!("expected two exact holed-polygon spans, got {spans:?}")
        };
        assert!(first.start.carrier_parameter.contains(-3.0));
        assert!(first.end.carrier_parameter.contains(-1.5));
        assert!(second.start.carrier_parameter.contains(0.5));
        assert!(second.end.carrier_parameter.contains(3.0));
        assert!(spans[0].end.carrier_parameter.hi() < spans[1].start.carrier_parameter.lo());
    }

    #[test]
    fn interval_side_must_strictly_exclude_zero() {
        let line = trace([0.0, 0.0], [1.0, 0.0]);
        assert_eq!(
            certified_line_side(
                line,
                IntervalPoint2 {
                    x: Interval::point(2.0),
                    y: Interval::new(1.0, 2.0),
                },
            ),
            Some(Orientation::Positive)
        );
        assert_eq!(
            certified_line_side(
                line,
                IntervalPoint2 {
                    x: Interval::point(2.0),
                    y: Interval::new(-f64::EPSILON, f64::EPSILON),
                },
            ),
            None
        );
    }

    #[test]
    fn polygon_vertex_and_coincident_boundary_fail_closed() {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [4.0, 4.0, 1.0]).unwrap();
        let face = face_where(
            &store,
            body,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let run = |line| {
            with_scope(|scope| {
                clip_ruling_to_face(&store, face, line, ParamRange::new(-3.0, 3.0), scope).unwrap()
            })
        };
        assert_eq!(
            run(trace([0.0, 0.0], [1.0, 1.0])),
            RulingClipOutcome::Indeterminate(RulingClipGap::VertexContact)
        );
        assert_eq!(
            run(trace([0.0, 2.0], [1.0, 0.0])),
            RulingClipOutcome::Indeterminate(RulingClipGap::CoincidentBoundary)
        );
    }

    #[test]
    fn cylinder_longitude_clip_retains_ring_edge_parameters() {
        let mut store = Store::new();
        let body = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let face = face_where(&store, body, |surface| {
            matches!(surface, SurfaceGeom::Cylinder(_))
        });
        let outcome = with_scope(|scope| {
            clip_ruling_to_face(
                &store,
                face,
                trace([1.0, 0.0], [0.0, 1.0]),
                ParamRange::new(-1.0, 3.0),
                scope,
            )
            .unwrap()
        });
        let RulingClipOutcome::Spans(spans) = outcome else {
            panic!("expected one cylinder band, got {outcome:?}");
        };
        let [span] = spans.as_slice() else {
            panic!("expected one cylinder band, got {spans:?}");
        };
        assert_ne!(span.start.edge, span.end.edge);
        assert!(span.start.carrier_parameter.contains(0.0));
        assert!(span.end.carrier_parameter.contains(2.0));
        assert!(span.start.edge_parameter.contains(1.0));
        assert!(span.end.edge_parameter.contains(1.0));
    }

    #[test]
    fn periodic_parameter_supports_reversed_source_direction_and_shifted_chart() {
        let tau = core::f64::consts::TAU;
        let range = ParamRange::new(0.0, tau);
        let mut store = Store::new();

        let reversed_boundary = Line2d::new(Point2::new(tau, 0.0), Vec2::new(-1.0, 0.0)).unwrap();
        let reversed = periodic_use(
            &mut store,
            reversed_boundary,
            range,
            ParamMap1d::identity(),
            [-1, 0],
            [0, 0],
        );
        assert!(
            periodic_edge_parameter(1.0, &reversed_boundary, reversed)
                .expect("reversed whole-period use must retain one root")
                .contains(tau - 1.0)
        );

        let forward_boundary = Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap();
        let reversed_map = periodic_use(
            &mut store,
            forward_boundary,
            range,
            ParamMap1d::affine(-1.0, tau).unwrap(),
            [-1, 0],
            [0, 0],
        );
        assert!(
            periodic_edge_parameter(1.0, &forward_boundary, reversed_map)
                .expect("reversed edge map must retain one intrinsic root")
                .contains(tau - 1.0)
        );

        let shifted = periodic_use(
            &mut store,
            forward_boundary,
            range,
            ParamMap1d::identity(),
            [1, 0],
            [3, 0],
        );
        assert!(
            periodic_edge_parameter(1.0, &forward_boundary, shifted)
                .expect("integer chart shifts must preserve the intrinsic root")
                .contains(1.0)
        );
    }

    #[test]
    fn periodic_parameter_refuses_unproved_membership_and_noncanonical_periods() {
        let tau = core::f64::consts::TAU;
        let boundary = Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap();
        let mut store = Store::new();

        let canonical = periodic_use(
            &mut store,
            boundary,
            ParamRange::new(0.0, tau),
            ParamMap1d::identity(),
            [1, 0],
            [0, 0],
        );
        assert_eq!(
            periodic_edge_parameter(0.0, &boundary, canonical),
            None,
            "a root whose enclosure touches the half-open seam is not admitted"
        );

        let doubled = periodic_use(
            &mut store,
            boundary,
            ParamRange::new(0.0, 2.0 * tau),
            ParamMap1d::affine(2.0, 0.0).unwrap(),
            [1, 0],
            [0, 0],
        );
        assert_eq!(periodic_edge_parameter(1.0, &boundary, doubled), None);

        let shifted_height = periodic_use(
            &mut store,
            boundary,
            ParamRange::new(0.0, tau),
            ParamMap1d::identity(),
            [1, 0],
            [0, 1],
        );
        assert_eq!(
            periodic_edge_parameter(1.0, &boundary, shifted_height),
            None
        );
    }

    #[test]
    fn per_face_topology_span_survives_when_it_contains_the_carrier_window() {
        let mut store = Store::new();
        let body = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let face = face_where(&store, body, |surface| {
            matches!(surface, SurfaceGeom::Cylinder(_))
        });
        let outcome = with_scope(|scope| {
            clip_ruling_to_face(
                &store,
                face,
                trace([1.0, 0.0], [0.0, 1.0]),
                ParamRange::new(0.5, 1.5),
                scope,
            )
            .unwrap()
        });
        let RulingClipOutcome::Spans(spans) = outcome else {
            panic!("topology span must survive its contained source window: {outcome:?}");
        };
        let [span] = spans.as_slice() else {
            panic!("expected one containing topology span, got {spans:?}");
        };
        assert!(span.start.carrier_parameter.contains(0.0));
        assert!(span.end.carrier_parameter.contains(2.0));
    }

    #[test]
    fn containing_operand_defers_to_other_operands_physical_endpoints() {
        let mut store = Store::new();
        let block = ktopo::make::block(&mut store, &Frame::world(), [1.0, 4.0, 1.0]).unwrap();
        let cylinder = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let plane = face_where(
            &store,
            block,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let side = face_where(&store, cylinder, |surface| {
            matches!(surface, SurfaceGeom::Cylinder(_))
        });
        let range = ParamRange::new(0.5, 1.5);
        let (plane_spans, side_spans) = with_scope(|scope| {
            let RulingClipOutcome::Spans(plane_spans) =
                clip_ruling_to_face(&store, plane, trace([-4.0, 0.0], [4.0, 0.0]), range, scope)
                    .unwrap()
            else {
                panic!("plane clip must succeed")
            };
            let RulingClipOutcome::Spans(side_spans) =
                clip_ruling_to_face(&store, side, trace([1.0, 0.0], [0.0, 1.0]), range, scope)
                    .unwrap()
            else {
                panic!("cylinder clip must succeed")
            };
            (plane_spans, side_spans)
        });
        let merged = with_scope(|scope| {
            merge_ruling_spans(&plane_spans, &side_spans, range, scope).unwrap()
        });
        let RulingMergeOutcome::Spans(spans) = merged else {
            panic!("containing operand must defer to the other topology: {merged:?}");
        };
        let [span] = spans.as_slice() else {
            panic!("expected one merged span, got {spans:?}");
        };
        assert_eq!(span.start.sites.map(|site| site.is_some()), [true, false]);
        assert_eq!(span.end.sites.map(|site| site.is_some()), [true, false]);
        assert!(span.start.carrier_parameter.contains(range.lo));
        assert!(span.end.carrier_parameter.contains(range.hi));
    }

    #[test]
    fn two_list_merge_selects_strict_operand_owned_endpoints() {
        let mut store = Store::new();
        let block = ktopo::make::block(&mut store, &Frame::world(), [4.0, 4.0, 1.0]).unwrap();
        let cylinder = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 3.0).unwrap();
        let plane = face_where(
            &store,
            block,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let side = face_where(&store, cylinder, |surface| {
            matches!(surface, SurfaceGeom::Cylinder(_))
        });
        let (plane_spans, side_spans) = with_scope(|scope| {
            let RulingClipOutcome::Spans(plane_spans) = clip_ruling_to_face(
                &store,
                plane,
                trace([0.0, 0.0], [1.0, 0.0]),
                ParamRange::new(-3.0, 4.0),
                scope,
            )
            .unwrap() else {
                panic!("plane clip must succeed")
            };
            let RulingClipOutcome::Spans(side_spans) = clip_ruling_to_face(
                &store,
                side,
                trace([1.0, 0.0], [0.0, 1.0]),
                ParamRange::new(-3.0, 4.0),
                scope,
            )
            .unwrap() else {
                panic!("cylinder clip must succeed")
            };
            (plane_spans, side_spans)
        });
        let merged = with_scope(|scope| {
            merge_ruling_spans(&plane_spans, &side_spans, ParamRange::new(-3.0, 4.0), scope)
                .unwrap()
        });
        let RulingMergeOutcome::Spans(spans) = merged else {
            panic!("strict span overlap must merge, got {merged:?}");
        };
        let [span] = spans.as_slice() else {
            panic!("expected one merged span, got {spans:?}");
        };
        assert_eq!(span.start.sites.map(|site| site.is_some()), [false, true]);
        assert_eq!(span.end.sites.map(|site| site.is_some()), [true, false]);
        assert_eq!(
            span.start
                .edge_parameters
                .map(|parameter| parameter.is_some()),
            [false, true]
        );
        assert_eq!(
            span.end
                .edge_parameters
                .map(|parameter| parameter.is_some()),
            [true, false]
        );
    }

    #[test]
    fn two_list_merge_advances_both_multi_span_inputs() {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [4.0, 4.0, 1.0]).unwrap();
        let face = face_where(
            &store,
            body,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let template = with_scope(|scope| {
            let RulingClipOutcome::Spans(spans) = clip_ruling_to_face(
                &store,
                face,
                trace([0.0, 0.0], [1.0, 0.0]),
                ParamRange::new(-3.0, 3.0),
                scope,
            )
            .unwrap() else {
                panic!("template plane clip must succeed")
            };
            spans[0]
        });
        let a = [
            synthetic_span(template, [0.0, 1.0]),
            synthetic_span(template, [4.0, 7.0]),
            synthetic_span(template, [10.0, 11.0]),
        ];
        let b = [
            synthetic_span(template, [2.0, 3.0]),
            synthetic_span(template, [5.0, 6.0]),
            synthetic_span(template, [8.0, 9.0]),
        ];
        let outcome = with_scope(|scope| {
            merge_ruling_spans(&a, &b, ParamRange::new(-1.0, 12.0), scope).unwrap()
        });
        let RulingMergeOutcome::Spans(spans) = outcome else {
            panic!("strict multi-span lists must merge, got {outcome:?}")
        };
        let [span] = spans.as_slice() else {
            panic!("only the middle spans overlap, got {spans:?}")
        };
        assert!(span.start.carrier_parameter.contains(5.0));
        assert!(span.end.carrier_parameter.contains(6.0));
        assert_eq!(span.start.sites.map(|site| site.is_some()), [false, true]);
        assert_eq!(span.end.sites.map(|site| site.is_some()), [false, true]);
    }

    #[test]
    fn two_list_merge_refuses_same_point_endpoint_ambiguity() {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [4.0, 4.0, 1.0]).unwrap();
        let face = face_where(
            &store,
            body,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let spans = with_scope(|scope| {
            let RulingClipOutcome::Spans(spans) = clip_ruling_to_face(
                &store,
                face,
                trace([0.0, 0.0], [1.0, 0.0]),
                ParamRange::new(-3.0, 3.0),
                scope,
            )
            .unwrap() else {
                panic!("plane clip must succeed")
            };
            spans
        });
        let outcome = with_scope(|scope| {
            merge_ruling_spans(&spans, &spans, ParamRange::new(-3.0, 3.0), scope).unwrap()
        });
        assert_eq!(
            outcome,
            RulingMergeOutcome::Indeterminate(RulingClipGap::UnorderedCrossings)
        );
    }
}
