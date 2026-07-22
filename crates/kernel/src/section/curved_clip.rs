//! Certified clipping of one closed conic carrier against one face trim.
//!
//! This module owns the first curved-trim admission classes needed by a
//! Plane/Cylinder circle branch:
//!
//! - a circular plane pcurve against any number of bounded polygonal trim
//!   loops, and
//! - a constant-height, whole-period cylinder pcurve against any number of
//!   vertexless whole-period ring loops.
//!
//! Polygon crossings are roots of the source-derived harmonic line form.
//! Every coefficient and root enclosure is built with outward interval
//! arithmetic from the authored pcurve and fin-pcurve values. Rounded angles
//! are retained only as representatives; cyclic ordering is exclusively by
//! disjoint projective half-angle intervals. Tangency, endpoint contact,
//! coincident boundaries, overlapping root enclosures, or unsupported trim
//! geometry return an explicit indeterminate outcome.

use kcore::interval::Interval;
use kcore::math;
use kcore::operation::OperationScope;
use kgeom::curve2d::Line2d;
use kgeom::param::ParamRange;
use kgeom::vec::Point2;
use ktopo::entity::{
    EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId, FinPcurve, Loop,
    LoopId as RawLoopId,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use ktopo::store::Store;

use crate::error::{Error, Result};

use super::{SECTION_WORK, SectionUvCircle, SectionUvCurve, SectionUvLine};

/// Stable failure classes for a closed-conic trim proof.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ClosedConicClipGap {
    /// The face/pcurve pair is outside the supported exact trim classes.
    UnsupportedTrim,
    /// The stored trim graph is not a closed, source-provenanced boundary.
    MalformedTrim,
    /// A source value or certified enclosure is unusable.
    ArithmeticGuard,
    /// The carrier touches a trim boundary without a transverse crossing.
    TangentialContact,
    /// A whole-period carrier lies on a whole-period trim boundary.
    CoincidentBoundary,
    /// The two circles do not have exactly two transverse boundary crossings.
    NonSecantBoundary,
    /// A crossing cannot be separated from a periodic parameter seam.
    ParameterSeamContact,
    /// Distinct crossing enclosures cannot be put in a strict cyclic order.
    UnorderedCrossings,
}

impl ClosedConicClipGap {
    /// Stable diagnostic suitable for a section gap.
    pub(crate) const fn reason(self) -> &'static str {
        match self {
            Self::UnsupportedTrim => {
                "closed-conic clipping does not support this exact face trim class"
            }
            Self::MalformedTrim => {
                "closed-conic clipping requires a closed source-provenanced face trim"
            }
            Self::ArithmeticGuard => {
                "closed-conic clipping could not certify a source-derived arithmetic guard"
            }
            Self::TangentialContact => {
                "a closed conic has an unresolved tangent or trim-vertex contact"
            }
            Self::CoincidentBoundary => {
                "a closed conic is coincident with a whole-period trim boundary"
            }
            Self::NonSecantBoundary => {
                "a closed circle does not have two transverse disk-boundary crossings"
            }
            Self::ParameterSeamContact => {
                "a circle trim crossing cannot be separated from a periodic parameter seam"
            }
            Self::UnorderedCrossings => {
                "closed-conic trim crossings could not be certifiably ordered"
            }
        }
    }
}

/// One exact topology site crossed by a closed conic.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ClosedConicTrimSite {
    pub(crate) face: RawFaceId,
    pub(crate) loop_id: RawLoopId,
    pub(crate) fin: RawFinId,
    pub(crate) edge: RawEdgeId,
    /// Ordinal in this source edge's certified strict intrinsic-parameter
    /// order. Together with `edge`, this is the proof-owned crossing
    /// identity; it is independent of fin sense and the local face chart.
    pub(crate) root_ordinal: usize,
    /// Root enclosure in `y = tan(q/2)`, where `q` is the source circle
    /// pcurve parameter. This interval, not the rounded angle, owns order.
    pub(crate) pcurve_half_angle: Interval,
    /// Numeric carrier-parameter representative. It is geometric evidence,
    /// never authority for crossing identity or order.
    pub(crate) carrier_parameter: f64,
    /// Outward carrier-parameter enclosure derived from the projective root.
    /// This interval, never `carrier_parameter`, is used when the same root
    /// must be transported through another topology-owned pcurve.
    pub(crate) carrier_parameter_enclosure: Interval,
    /// Intrinsic source-edge parameter enclosure.
    pub(crate) edge_parameter: Interval,
}

/// Exact topology and parameter-map evidence for a closed carrier that is one
/// whole-period boundary loop of its source face.
///
/// This is deliberately not a successful trim result by itself: coincidence
/// remains a graph gap. The merge layer may retain it only beside an
/// independently certified strict disk clip, where its periodic map can bind
/// both source-ring identities to every published endpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CertifiedCoincidentSourceBoundary {
    pub(crate) face: RawFaceId,
    pub(crate) loop_id: RawLoopId,
    pub(crate) fin: RawFinId,
    pub(crate) edge: RawEdgeId,
    trace: SectionUvLine,
    boundary: Line2d,
    use_: FinPcurve,
}

/// One maximal connected portion certified inside a face trim.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ClosedConicFragment {
    /// Boundary crossed when entering this fragment. `None` denotes a whole
    /// closed carrier with no trim crossing.
    pub(crate) start: Option<ClosedConicTrimSite>,
    /// Boundary crossed when leaving this fragment. `None` denotes a whole
    /// closed carrier with no trim crossing.
    pub(crate) end: Option<ClosedConicTrimSite>,
    /// Whether the fragment crosses the source circle pcurve's projective
    /// chart seam at `q = +/- pi`.
    pub(crate) wraps_pcurve_seam: bool,
}

/// Fail-closed result of clipping one closed carrier to one face.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum ClosedConicClipOutcome {
    /// Maximal fragments in increasing carrier-parameter orientation.
    /// An empty vector is a certified miss; one site-less wrapping fragment
    /// is the complete closed carrier.
    Fragments(Vec<ClosedConicFragment>),
    /// The carrier is coefficient-identical to exactly one topology-owned
    /// whole-period source-ring trim. It is not globally complete evidence;
    /// a peer strict clip and two-sided source-root proof are still required.
    CoincidentSourceBoundary(CertifiedCoincidentSourceBoundary),
    /// The trim topology could not be certified.
    Indeterminate(ClosedConicClipGap),
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
    start: IntervalPoint2,
    end: IntervalPoint2,
    edge_parameters: [f64; 2],
}

#[derive(Debug, Clone, Copy)]
struct CircleSource {
    center: IntervalPoint2,
    x: IntervalPoint2,
    y: IntervalPoint2,
    radius: Interval,
    map_scale: f64,
    map_offset: f64,
    carrier_range: ParamRange,
}

#[derive(Debug, Clone, Copy)]
struct Crossing {
    site: ClosedConicTrimSite,
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

/// Lift one finite projective root into the branch's authored carrier range.
///
/// The rounded period index is only a search seed. Acceptance requires one
/// and only one outward enclosure to lie strictly inside the half-open source
/// period, so a seam-straddling root can only fail closed.
pub(super) fn carrier_parameter_enclosure(
    half_angle: Interval,
    scale: f64,
    offset: f64,
    carrier_range: ParamRange,
) -> core::result::Result<Interval, ClosedConicClipGap> {
    if !finite(half_angle)
        || !scale.is_finite()
        || !offset.is_finite()
        || scale.abs() != 1.0
        || !carrier_range.is_finite()
        || carrier_range.width() != core::f64::consts::TAU
    {
        return Err(ClosedConicClipGap::ArithmeticGuard);
    }
    let natural = twice_atan_interval(half_angle)?;
    let unlifted = (natural - Interval::point(offset))
        .checked_div(Interval::point(scale))
        .filter(|value| finite(*value))
        .ok_or(ClosedConicClipGap::ArithmeticGuard)?;
    let midpoint = 0.5 * unlifted.lo() + 0.5 * unlifted.hi();
    let base = ((carrier_range.lo - midpoint) / core::f64::consts::TAU).round();
    if !midpoint.is_finite() || !base.is_finite() {
        return Err(ClosedConicClipGap::ArithmeticGuard);
    }
    let mut accepted = None;
    for offset in [-1.0, 0.0, 1.0] {
        let shift = Interval::point(base + offset) * Interval::point(core::f64::consts::TAU);
        let candidate = unlifted + shift;
        if !finite(candidate) {
            return Err(ClosedConicClipGap::ArithmeticGuard);
        }
        if candidate.lo() > carrier_range.lo
            && candidate.hi() < carrier_range.hi
            && accepted.replace(candidate).is_some()
        {
            return Err(ClosedConicClipGap::ParameterSeamContact);
        }
    }
    accepted.ok_or(ClosedConicClipGap::ParameterSeamContact)
}

fn twice_atan_interval(value: Interval) -> core::result::Result<Interval, ClosedConicClipGap> {
    if !finite(value) {
        return Err(ClosedConicClipGap::ArithmeticGuard);
    }
    let mut lo = 2.0 * math::atan(value.lo());
    let mut hi = 2.0 * math::atan(value.hi());
    if !lo.is_finite() || !hi.is_finite() || lo > hi {
        return Err(ClosedConicClipGap::ArithmeticGuard);
    }
    for _ in 0..4 {
        lo = lo.next_down();
        hi = hi.next_up();
    }
    Ok(Interval::new(lo, hi))
}

/// Map one certified carrier-root enclosure onto the coincident source ring's
/// intrinsic edge parameter. Integer-period search is finite and accepted
/// only when exactly one full-period pcurve copy contains the whole interval.
pub(crate) fn coincident_source_edge_parameter(
    evidence: CertifiedCoincidentSourceBoundary,
    carrier_parameter: Interval,
) -> core::result::Result<Interval, ClosedConicClipGap> {
    if !finite(carrier_parameter) {
        return Err(ClosedConicClipGap::ArithmeticGuard);
    }
    let trace = evidence.trace;
    let longitude = Interval::point(trace.origin().x)
        + Interval::point(trace.direction().x) * carrier_parameter;
    periodic_edge_parameter_interval(longitude, evidence.boundary, evidence.use_)
}

fn periodic_edge_parameter_interval(
    longitude: Interval,
    boundary: Line2d,
    use_: FinPcurve,
) -> core::result::Result<Interval, ClosedConicClipGap> {
    let period = core::f64::consts::TAU;
    let active = use_.range();
    let map = use_.edge_to_pcurve();
    let direction = boundary.dir();
    let [chart_winding, chart_height] = use_.chart().period_shifts();
    let [closure_winding, closure_height] = use_
        .closure_winding()
        .ok_or(ClosedConicClipGap::UnsupportedTrim)?;
    let edge_parameters = [map.inverse(active.lo), map.inverse(active.hi)];
    let rate = direction.x * map.scale();
    let winding_matches_rate =
        closure_winding == 1 && rate > 0.0 || closure_winding == -1 && rate < 0.0;
    if !finite(longitude)
        || !active.is_finite()
        || direction.x == 0.0
        || direction.y != 0.0
        || chart_height != 0
        || closure_height != 0
        || !matches!(closure_winding, 1 | -1)
        || !winding_matches_rate
        || direction.x.abs() * active.width() != period
        || edge_parameters.into_iter().any(|value| !value.is_finite())
        || edge_parameters[0].min(edge_parameters[1]) != 0.0
        || edge_parameters[0].max(edge_parameters[1]) != period
        || use_.seam().is_some()
    {
        return Err(ClosedConicClipGap::UnsupportedTrim);
    }

    let chart_shift = Interval::point(f64::from(chart_winding)) * Interval::point(period);
    let longitude_at = |parameter: f64| {
        Interval::point(boundary.origin().x)
            + Interval::point(direction.x) * Interval::point(parameter)
            + chart_shift
    };
    let endpoints = [longitude_at(active.lo), longitude_at(active.hi)];
    if endpoints.into_iter().any(|value| !finite(value)) {
        return Err(ClosedConicClipGap::ArithmeticGuard);
    }
    let active_longitudes = Interval::new(
        endpoints[0].lo().min(endpoints[1].lo()),
        endpoints[0].hi().max(endpoints[1].hi()),
    );
    let possible_windings = (active_longitudes - longitude)
        .checked_div(Interval::point(period))
        .filter(|value| finite(*value))
        .ok_or(ClosedConicClipGap::ArithmeticGuard)?;
    let first = possible_windings.lo().ceil();
    let last = possible_windings.hi().floor();
    if !first.is_finite()
        || !last.is_finite()
        || first > last
        || first < i32::MIN as f64
        || last > i32::MAX as f64
        || last - first > 2.0
    {
        return Err(ClosedConicClipGap::ParameterSeamContact);
    }

    let mut accepted = None;
    for winding in (first as i32)..=(last as i32) {
        let numerator = longitude + Interval::point(f64::from(winding)) * Interval::point(period)
            - chart_shift
            - Interval::point(boundary.origin().x);
        let q = numerator
            .checked_div(Interval::point(direction.x))
            .filter(|value| finite(*value))
            .ok_or(ClosedConicClipGap::ArithmeticGuard)?;
        if q.hi() < active.lo || q.lo() >= active.hi {
            continue;
        }
        if q.lo() < active.lo || q.hi() >= active.hi {
            return Err(ClosedConicClipGap::ParameterSeamContact);
        }
        let edge_parameter = (q - Interval::point(map.offset()))
            .checked_div(Interval::point(map.scale()))
            .filter(|value| finite(*value))
            .ok_or(ClosedConicClipGap::ArithmeticGuard)?;
        if accepted.replace(edge_parameter).is_some() {
            return Err(ClosedConicClipGap::ParameterSeamContact);
        }
    }
    accepted.ok_or(ClosedConicClipGap::ParameterSeamContact)
}

fn add(a: IntervalPoint2, b: IntervalPoint2) -> IntervalPoint2 {
    IntervalPoint2 {
        x: a.x + b.x,
        y: a.y + b.y,
    }
}

fn sub(a: IntervalPoint2, b: IntervalPoint2) -> IntervalPoint2 {
    IntervalPoint2 {
        x: a.x - b.x,
        y: a.y - b.y,
    }
}

fn scale(a: IntervalPoint2, scalar: Interval) -> IntervalPoint2 {
    IntervalPoint2 {
        x: a.x * scalar,
        y: a.y * scalar,
    }
}

fn dot(a: IntervalPoint2, b: IntervalPoint2) -> Interval {
    a.x * b.x + a.y * b.y
}

fn cross(a: IntervalPoint2, b: IntervalPoint2) -> Interval {
    a.x * b.y - a.y * b.x
}

fn mapped_parameter(scale: f64, offset: f64, parameter: f64) -> Option<Interval> {
    let value = Interval::point(scale) * Interval::point(parameter) + Interval::point(offset);
    finite(value).then_some(value)
}

fn line_point(line: &Line2d, parameter: Interval) -> Option<IntervalPoint2> {
    let origin = IntervalPoint2::point(line.origin());
    let direction = IntervalPoint2::point(Point2::new(line.dir().x, line.dir().y));
    let point = add(origin, scale(direction, parameter));
    point.finite().then_some(point)
}

fn circle_source(
    circle: SectionUvCircle,
    carrier_range: ParamRange,
) -> core::result::Result<CircleSource, ClosedConicClipGap> {
    let center = circle.center();
    let x = circle.x_direction();
    let y = x.perp();
    let values = [
        center.x,
        center.y,
        x.x,
        x.y,
        circle.radius(),
        circle.parameter_scale(),
        circle.parameter_offset(),
        carrier_range.lo,
        carrier_range.hi,
    ];
    if values.iter().any(|value| !value.is_finite())
        || circle.radius() <= 0.0
        || circle.parameter_scale().abs() != 1.0
        || carrier_range.width() != core::f64::consts::TAU
    {
        return Err(ClosedConicClipGap::UnsupportedTrim);
    }
    Ok(CircleSource {
        center: IntervalPoint2::point(center),
        x: IntervalPoint2::point(Point2::new(x.x, x.y)),
        y: IntervalPoint2::point(Point2::new(y.x, y.y)),
        radius: Interval::point(circle.radius()),
        map_scale: circle.parameter_scale(),
        map_offset: circle.parameter_offset(),
        carrier_range,
    })
}

/// Clip a proof-bearing closed Plane/Cylinder carrier pcurve to the exact
/// stored trim of one source face.
///
/// The caller supplies the pcurve already paired with `face` by the graph
/// branch certificate. The function does not use `FaceDomain`: it reads the
/// face's loops, fins, pcurves, edges, and closure metadata as proof sources.
pub(crate) fn clip_closed_conic_to_face(
    store: &Store,
    face: RawFaceId,
    pcurve: SectionUvCurve,
    carrier_range: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ClosedConicClipOutcome> {
    charge(scope, 1)?;
    let face_data = read(store.get(face))?;
    let surface = read(store.surface(face_data.surface))?;
    match (surface, pcurve) {
        (SurfaceGeom::Plane(_), SectionUvCurve::Circle(circle)) => {
            clip_circle_to_plane_trim(store, face, circle, carrier_range, scope)
        }
        (SurfaceGeom::Cylinder(_), SectionUvCurve::Line(line)) => {
            clip_longitude_to_periodic_trim(store, face, line, carrier_range, scope)
        }
        _ => Ok(ClosedConicClipOutcome::Indeterminate(
            ClosedConicClipGap::UnsupportedTrim,
        )),
    }
}

fn clip_circle_to_plane_trim(
    store: &Store,
    face: RawFaceId,
    circle: SectionUvCircle,
    carrier_range: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ClosedConicClipOutcome> {
    if let Some(outcome) = super::circle_disk_clip::try_clip_circle_to_disk_trim(
        store,
        face,
        circle,
        carrier_range,
        scope,
    )? {
        return Ok(outcome);
    }
    let source = match circle_source(circle, carrier_range) {
        Ok(source) => source,
        Err(gap) => return Ok(ClosedConicClipOutcome::Indeterminate(gap)),
    };
    let segments = match prepare_plane_segments(store, face, scope)? {
        Ok(segments) => segments,
        Err(gap) => return Ok(ClosedConicClipOutcome::Indeterminate(gap)),
    };
    let seam_inside = match seam_inside_polygon(&source, &segments, scope)? {
        Ok(inside) => inside,
        Err(gap) => return Ok(ClosedConicClipOutcome::Indeterminate(gap)),
    };

    let mut crossings = Vec::new();
    for segment in &segments {
        match segment_crossings(&source, segment, scope)? {
            Ok(mut found) => crossings.append(&mut found),
            Err(gap) => return Ok(ClosedConicClipOutcome::Indeterminate(gap)),
        }
    }
    charge(scope, crossings.len() as u64)?;
    crossings.sort_by(|a, b| {
        a.site
            .pcurve_half_angle
            .lo()
            .total_cmp(&b.site.pcurve_half_angle.lo())
            .then(
                a.site
                    .pcurve_half_angle
                    .hi()
                    .total_cmp(&b.site.pcurve_half_angle.hi()),
            )
    });
    if crossings
        .windows(2)
        .any(|pair| pair[0].site.pcurve_half_angle.hi() >= pair[1].site.pcurve_half_angle.lo())
        || !crossings.len().is_multiple_of(2)
    {
        return Ok(ClosedConicClipOutcome::Indeterminate(
            ClosedConicClipGap::UnorderedCrossings,
        ));
    }

    let mut fragments = assemble_fragments(&crossings, seam_inside);
    if source.map_scale < 0.0 {
        fragments.reverse();
        for fragment in &mut fragments {
            core::mem::swap(&mut fragment.start, &mut fragment.end);
        }
    }
    Ok(ClosedConicClipOutcome::Fragments(fragments))
}

fn prepare_plane_segments(
    store: &Store,
    face: RawFaceId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<core::result::Result<Vec<PlaneTrimSegment>, ClosedConicClipGap>> {
    let face_data = read(store.get(face))?;
    if face_data.loops().is_empty() {
        return Ok(Err(ClosedConicClipGap::MalformedTrim));
    }
    let mut segments = Vec::new();
    for &loop_id in face_data.loops() {
        charge(scope, 1)?;
        let ring = read(store.get::<Loop>(loop_id))?;
        if ring.fins().len() < 3 {
            return Ok(Err(ClosedConicClipGap::MalformedTrim));
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
                return Ok(Err(ClosedConicClipGap::MalformedTrim));
            }
            let edge = read(store.get(fin.edge))?;
            let (Some(v0), Some(v1), Some((lo, hi)), Some(curve_id), Some(use_)) = (
                edge.vertices[0],
                edge.vertices[1],
                edge.bounds,
                edge.curve,
                fin.pcurve,
            ) else {
                return Ok(Err(ClosedConicClipGap::UnsupportedTrim));
            };
            if !lo.is_finite()
                || !hi.is_finite()
                || lo >= hi
                || !matches!(read(store.curve(curve_id))?, CurveGeom::Line(_))
                || !use_.chart().is_identity()
                || use_.closure_winding().is_some()
                || use_.seam().is_some()
            {
                return Ok(Err(ClosedConicClipGap::UnsupportedTrim));
            }
            let Curve2dGeom::Line(line) = read(store.pcurve(use_.curve()))? else {
                return Ok(Err(ClosedConicClipGap::UnsupportedTrim));
            };
            let (tail, head, edge_parameters) = if fin.sense.is_forward() {
                (v0, v1, [lo, hi])
            } else {
                (v1, v0, [hi, lo])
            };
            if previous_head.is_some_and(|previous| previous != tail) {
                return Ok(Err(ClosedConicClipGap::MalformedTrim));
            }
            first_tail.get_or_insert(tail);
            previous_head = Some(head);

            let map = use_.edge_to_pcurve();
            let Some(start_parameter) =
                mapped_parameter(map.scale(), map.offset(), edge_parameters[0])
            else {
                return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
            };
            let Some(end_parameter) =
                mapped_parameter(map.scale(), map.offset(), edge_parameters[1])
            else {
                return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
            };
            let active = use_.range();
            // `FinPcurve` evaluation intentionally defines the affine map in
            // f64. Validate the authored active-use contract on those exact
            // semantic values; retain the outward intervals above only for
            // subsequent geometric arithmetic.
            let mapped = [map.map(edge_parameters[0]), map.map(edge_parameters[1])];
            if mapped.iter().any(|parameter| {
                !parameter.is_finite() || *parameter < active.lo || *parameter > active.hi
            }) {
                return Ok(Err(ClosedConicClipGap::MalformedTrim));
            }
            let (Some(start), Some(end)) = (
                line_point(line, start_parameter),
                line_point(line, end_parameter),
            ) else {
                return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
            };
            let direction = sub(end, start);
            let length_squared = dot(direction, direction);
            if length_squared.lo() <= 0.0 || !finite(length_squared) {
                return Ok(Err(ClosedConicClipGap::MalformedTrim));
            }
            segments.push(PlaneTrimSegment {
                face,
                loop_id,
                fin: fin_id,
                edge: fin.edge,
                start,
                end,
                edge_parameters,
            });
        }
        if first_tail != previous_head {
            return Ok(Err(ClosedConicClipGap::MalformedTrim));
        }
    }
    Ok(Ok(segments))
}

fn seam_point(source: &CircleSource) -> IntervalPoint2 {
    sub(source.center, scale(source.x, source.radius))
}

fn seam_inside_polygon(
    source: &CircleSource,
    segments: &[PlaneTrimSegment],
    scope: &mut OperationScope<'_, '_>,
) -> Result<core::result::Result<bool, ClosedConicClipGap>> {
    let query = seam_point(source);
    if !query.finite() {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    }
    let mut inside = false;
    for segment in segments {
        charge(scope, 1)?;
        let a_above = segment.start.y.lo() > query.y.hi();
        let a_below = segment.start.y.hi() < query.y.lo();
        let b_above = segment.end.y.lo() > query.y.hi();
        let b_below = segment.end.y.hi() < query.y.lo();
        if !(a_above || a_below) || !(b_above || b_below) {
            return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
        }
        if a_above == b_above {
            continue;
        }
        let denominator = segment.end.y - segment.start.y;
        if !excludes_zero(denominator) {
            return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
        }
        let fraction = match (query.y - segment.start.y).checked_div(denominator) {
            Some(fraction) if finite(fraction) => fraction,
            _ => return Ok(Err(ClosedConicClipGap::ArithmeticGuard)),
        };
        let x = segment.start.x + fraction * (segment.end.x - segment.start.x);
        if x.lo() > query.x.hi() {
            inside = !inside;
        } else if x.hi() >= query.x.lo() {
            return Ok(Err(ClosedConicClipGap::TangentialContact));
        }
    }
    Ok(Ok(inside))
}

fn segment_crossings(
    source: &CircleSource,
    segment: &PlaneTrimSegment,
    scope: &mut OperationScope<'_, '_>,
) -> Result<core::result::Result<Vec<Crossing>, ClosedConicClipGap>> {
    charge(scope, 1)?;
    let direction = sub(segment.end, segment.start);
    let relative_center = sub(source.center, segment.start);
    let cosine = source.radius * cross(direction, source.x);
    let sine = source.radius * cross(direction, source.y);
    let constant = cross(direction, relative_center);
    if !finite(cosine) || !finite(sine) || !finite(constant) {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    }
    let discriminant = cosine.square() + sine.square() - constant.square();
    if !finite(discriminant) {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    }
    if discriminant.hi() < 0.0 {
        return Ok(Ok(Vec::new()));
    }
    if discriminant.lo() <= 0.0 {
        return Ok(Err(ClosedConicClipGap::TangentialContact));
    }

    let quadratic = [
        constant - cosine,
        Interval::point(2.0) * sine,
        constant + cosine,
    ];
    if !excludes_zero(quadratic[0]) {
        // The projective chart seam is on, or cannot be separated from, a
        // crossing. Refuse instead of fabricating a cyclic order.
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    }
    let root_discriminant = Interval::point(4.0) * discriminant;
    let Some(root) = root_discriminant.sqrt() else {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    };
    let denominator = Interval::point(2.0) * quadratic[0];
    let Some(first) = (-quadratic[1] - root).checked_div(denominator) else {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    };
    let Some(second) = (-quadratic[1] + root).checked_div(denominator) else {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    };
    if !finite(first) || !finite(second) {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    }

    let mut roots = [first, second];
    roots.sort_by(|a, b| a.lo().total_cmp(&b.lo()).then(a.hi().total_cmp(&b.hi())));
    if roots[0].hi() >= roots[1].lo() {
        return Ok(Err(ClosedConicClipGap::UnorderedCrossings));
    }
    let mut crossings = Vec::new();
    for half_angle in roots {
        charge(scope, 1)?;
        let carrier_parameter_enclosure = match carrier_parameter_enclosure(
            half_angle,
            source.map_scale,
            source.map_offset,
            source.carrier_range,
        ) {
            Ok(parameter) => parameter,
            Err(gap) => return Ok(Err(gap)),
        };
        let point = match circle_point_from_half_angle(source, half_angle) {
            Some(point) => point,
            None => return Ok(Err(ClosedConicClipGap::ArithmeticGuard)),
        };
        let length_squared = dot(direction, direction);
        let Some(fraction) = dot(sub(point, segment.start), direction).checked_div(length_squared)
        else {
            return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
        };
        if !finite(fraction) {
            return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
        }
        if fraction.hi() < 0.0 || fraction.lo() > 1.0 {
            continue;
        }
        if fraction.lo() <= 0.0 || fraction.hi() >= 1.0 {
            return Ok(Err(ClosedConicClipGap::TangentialContact));
        }
        let [edge_start, edge_end] = segment.edge_parameters;
        let edge_parameter = Interval::point(edge_start)
            + fraction * (Interval::point(edge_end) - Interval::point(edge_start));
        let active = Interval::new(edge_start.min(edge_end), edge_start.max(edge_end));
        let Some(edge_parameter) = intersect(edge_parameter, active) else {
            return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
        };
        crossings.push(Crossing {
            site: ClosedConicTrimSite {
                face: segment.face,
                loop_id: segment.loop_id,
                fin: segment.fin,
                edge: segment.edge,
                // Assigned below only after the surviving finite-segment
                // roots are strictly ordered in intrinsic edge parameter.
                root_ordinal: 0,
                pcurve_half_angle: half_angle,
                carrier_parameter: carrier_parameter_representative(source, half_angle),
                carrier_parameter_enclosure,
                edge_parameter,
            },
        });
    }
    crossings.sort_by(|a, b| {
        a.site
            .edge_parameter
            .lo()
            .total_cmp(&b.site.edge_parameter.lo())
            .then(
                a.site
                    .edge_parameter
                    .hi()
                    .total_cmp(&b.site.edge_parameter.hi()),
            )
    });
    if crossings
        .windows(2)
        .any(|pair| pair[0].site.edge_parameter.hi() >= pair[1].site.edge_parameter.lo())
    {
        return Ok(Err(ClosedConicClipGap::UnorderedCrossings));
    }
    for (root_ordinal, crossing) in crossings.iter_mut().enumerate() {
        crossing.site.root_ordinal = root_ordinal;
    }
    Ok(Ok(crossings))
}

fn circle_point_from_half_angle(
    source: &CircleSource,
    half_angle: Interval,
) -> Option<IntervalPoint2> {
    let square = half_angle.square();
    let denominator = Interval::point(1.0) + square;
    let cosine = (Interval::point(1.0) - square).checked_div(denominator)?;
    let sine = (Interval::point(2.0) * half_angle).checked_div(denominator)?;
    let radial = add(scale(source.x, cosine), scale(source.y, sine));
    let point = add(source.center, scale(radial, source.radius));
    point.finite().then_some(point)
}

fn carrier_parameter_representative(source: &CircleSource, half_angle: Interval) -> f64 {
    let midpoint = 0.5 * half_angle.lo() + 0.5 * half_angle.hi();
    let natural = 2.0 * math::atan2(midpoint, 1.0);
    let carrier = (natural - source.map_offset) / source.map_scale;
    let period = core::f64::consts::TAU;
    if !carrier.is_finite() {
        return source.carrier_range.lo;
    }
    (carrier - source.carrier_range.lo).rem_euclid(period) + source.carrier_range.lo
}

fn assemble_fragments(crossings: &[Crossing], seam_inside: bool) -> Vec<ClosedConicFragment> {
    if crossings.is_empty() {
        return seam_inside
            .then_some(ClosedConicFragment {
                start: None,
                end: None,
                wraps_pcurve_seam: true,
            })
            .into_iter()
            .collect();
    }
    let mut fragments = Vec::new();
    if seam_inside {
        fragments.push(ClosedConicFragment {
            start: Some(crossings[crossings.len() - 1].site),
            end: Some(crossings[0].site),
            wraps_pcurve_seam: true,
        });
    }
    let mut inside = seam_inside;
    for pair in crossings.windows(2) {
        inside = !inside;
        if inside {
            fragments.push(ClosedConicFragment {
                start: Some(pair[0].site),
                end: Some(pair[1].site),
                wraps_pcurve_seam: false,
            });
        }
    }
    fragments
}

fn clip_longitude_to_periodic_trim(
    store: &Store,
    face: RawFaceId,
    trace: SectionUvLine,
    carrier_range: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ClosedConicClipOutcome> {
    let origin = trace.origin();
    let direction = trace.direction();
    if [
        origin.x,
        origin.y,
        direction.x,
        direction.y,
        carrier_range.lo,
        carrier_range.hi,
    ]
    .iter()
    .any(|value| !value.is_finite())
        || direction.x == 0.0
        || direction.y != 0.0
        || direction.x.abs() * carrier_range.width() != core::f64::consts::TAU
        || carrier_range.width() != core::f64::consts::TAU
    {
        return Ok(ClosedConicClipOutcome::Indeterminate(
            ClosedConicClipGap::UnsupportedTrim,
        ));
    }
    let face_data = read(store.get(face))?;
    if face_data.loops().is_empty() {
        return Ok(ClosedConicClipOutcome::Indeterminate(
            ClosedConicClipGap::MalformedTrim,
        ));
    }
    let trace_height = Interval::point(origin.y);
    let mut inside = false;
    let mut coincident = None;
    for &loop_id in face_data.loops() {
        charge(scope, 1)?;
        let ring = read(store.get::<Loop>(loop_id))?;
        let [fin_id] = ring.fins() else {
            return Ok(ClosedConicClipOutcome::Indeterminate(
                ClosedConicClipGap::UnsupportedTrim,
            ));
        };
        let fin = read(store.get(*fin_id))?;
        let edge = read(store.get(fin.edge))?;
        let (Some(curve_id), Some(use_)) = (edge.curve, fin.pcurve) else {
            return Ok(ClosedConicClipOutcome::Indeterminate(
                ClosedConicClipGap::UnsupportedTrim,
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
            || edge.tolerance().is_some()
            || !matches!(read(store.curve(curve_id))?, CurveGeom::Circle(_))
            || use_
                .closure_winding()
                .is_none_or(|winding| winding[0] == 0 || winding[1] != 0)
            || use_.seam().is_some()
            || use_.chart().period_shifts()[1] != 0
        {
            return Ok(ClosedConicClipOutcome::Indeterminate(
                ClosedConicClipGap::UnsupportedTrim,
            ));
        }
        let Curve2dGeom::Line(boundary) = read(store.pcurve(use_.curve()))? else {
            return Ok(ClosedConicClipOutcome::Indeterminate(
                ClosedConicClipGap::UnsupportedTrim,
            ));
        };
        if boundary.dir().x == 0.0 || boundary.dir().y != 0.0 {
            return Ok(ClosedConicClipOutcome::Indeterminate(
                ClosedConicClipGap::UnsupportedTrim,
            ));
        }
        let active = use_.range();
        let map = use_.edge_to_pcurve();
        let edge_parameters = [map.inverse(active.lo), map.inverse(active.hi)];
        let [winding, height_winding] = use_
            .closure_winding()
            .expect("validated whole-period winding");
        let rate = boundary.dir().x * map.scale();
        if !active.is_finite()
            || !matches!(winding, 1 | -1)
            || height_winding != 0
            || boundary.dir().x.abs() * active.width() != core::f64::consts::TAU
            || edge_parameters.into_iter().any(|value| !value.is_finite())
            || edge_parameters[0].min(edge_parameters[1]) != 0.0
            || edge_parameters[0].max(edge_parameters[1]) != core::f64::consts::TAU
            || !(winding == 1 && rate > 0.0 || winding == -1 && rate < 0.0)
        {
            return Ok(ClosedConicClipOutcome::Indeterminate(
                ClosedConicClipGap::UnsupportedTrim,
            ));
        }
        let boundary_height = Interval::point(boundary.origin().y);
        if trace_height.hi() < boundary_height.lo() {
            inside = !inside;
        } else if trace_height.lo() <= boundary_height.hi() {
            if origin.y != boundary.origin().y || coincident.is_some() {
                return Ok(ClosedConicClipOutcome::Indeterminate(
                    ClosedConicClipGap::CoincidentBoundary,
                ));
            }
            coincident = Some(CertifiedCoincidentSourceBoundary {
                face,
                loop_id,
                fin: *fin_id,
                edge: fin.edge,
                trace,
                boundary: *boundary,
                use_,
            });
        }
    }
    if let Some(evidence) = coincident {
        return Ok(ClosedConicClipOutcome::CoincidentSourceBoundary(evidence));
    }
    Ok(ClosedConicClipOutcome::Fragments(
        inside
            .then_some(ClosedConicFragment {
                start: None,
                end: None,
                wraps_pcurve_seam: true,
            })
            .into_iter()
            .collect(),
    ))
}

#[cfg(test)]
mod tests {
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationScope, ResourceKind,
        SessionPolicy,
    };
    use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances};
    use kgeom::frame::Frame;
    use kgeom::vec::Vec2;
    use ktopo::entity::BodyId as RawBodyId;
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
        surface: impl Fn(&SurfaceGeom) -> bool,
    ) -> RawFaceId {
        store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .find(|face| surface(store.surface(store.get(*face).unwrap().surface).unwrap()))
            .unwrap()
    }

    fn circle(radius: f64) -> SectionUvCurve {
        SectionUvCurve::Circle(SectionUvCircle {
            center: Point2::new(0.0, 0.0),
            radius,
            x_direction: Vec2::new(1.0, 0.0),
            parameter_scale: 1.0,
            parameter_offset: 0.0,
        })
    }

    #[test]
    fn polygon_clip_retains_edge_provenance_and_cyclic_fragments() {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [2.0, 4.0, 1.0]).unwrap();
        let face = face_where(
            &store,
            body,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let outcome = with_scope(|scope| {
            clip_closed_conic_to_face(
                &store,
                face,
                circle(1.5),
                ParamRange::new(0.0, core::f64::consts::TAU),
                scope,
            )
            .unwrap()
        });
        let ClosedConicClipOutcome::Fragments(fragments) = outcome else {
            panic!("expected certified fragments, got {outcome:?}");
        };
        assert_eq!(fragments.len(), 2);
        let mut sites = Vec::new();
        for fragment in fragments {
            let (start, end) = (fragment.start.unwrap(), fragment.end.unwrap());
            assert_eq!(start.face, face);
            assert_eq!(end.face, face);
            assert_ne!(start.edge, end.edge);
            assert!(start.root_ordinal <= 1);
            assert!(end.root_ordinal <= 1);
            assert!(start.edge_parameter.lo() < start.edge_parameter.hi());
            assert!(end.edge_parameter.lo() < end.edge_parameter.hi());
            sites.extend([start, end]);
        }
        let ordered_same_edge = sites.iter().find_map(|first| {
            sites
                .iter()
                .find(|second| {
                    first.edge == second.edge && first.root_ordinal == 0 && second.root_ordinal == 1
                })
                .map(|second| (*first, *second))
        });
        let (first, second) = ordered_same_edge.expect("one source edge contributes two roots");
        assert!(first.edge_parameter.hi() < second.edge_parameter.lo());
    }

    #[test]
    fn polygon_whole_miss_and_tangent_are_distinct() {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [2.0, 2.0, 1.0]).unwrap();
        let face = face_where(
            &store,
            body,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let run = |pcurve| {
            with_scope(|scope| {
                clip_closed_conic_to_face(
                    &store,
                    face,
                    pcurve,
                    ParamRange::new(0.0, core::f64::consts::TAU),
                    scope,
                )
                .unwrap()
            })
        };
        assert!(matches!(
            run(circle(0.5)),
            ClosedConicClipOutcome::Fragments(ref fragments)
                if fragments == &[ClosedConicFragment {
                    start: None,
                    end: None,
                    wraps_pcurve_seam: true,
                }]
        ));
        assert!(matches!(
            run(SectionUvCurve::Circle(SectionUvCircle {
                center: Point2::new(4.0, 0.0),
                radius: 0.5,
                x_direction: Vec2::new(1.0, 0.0),
                parameter_scale: 1.0,
                parameter_offset: 0.0,
            })),
            ClosedConicClipOutcome::Fragments(ref fragments) if fragments.is_empty()
        ));
        assert_eq!(
            run(circle(1.0)),
            ClosedConicClipOutcome::Indeterminate(ClosedConicClipGap::TangentialContact)
        );
    }

    #[test]
    fn polygon_loop_parity_excludes_holes_without_layout_cases() {
        let outer = [
            Point2::new(-3.0, -3.0),
            Point2::new(3.0, -3.0),
            Point2::new(3.0, 3.0),
            Point2::new(-3.0, 3.0),
        ];
        let hole = [
            Point2::new(-1.0, -1.0),
            Point2::new(-1.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, -1.0),
        ];
        let profile =
            PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&hole]).unwrap();
        let mut store = Store::new();
        let body = ktopo::make::extrude_profile(&mut store, &profile, 1.0).unwrap();
        let face = face_where(
            &store,
            body,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let run = |radius| {
            with_scope(|scope| {
                clip_closed_conic_to_face(
                    &store,
                    face,
                    circle(radius),
                    ParamRange::new(0.0, core::f64::consts::TAU),
                    scope,
                )
                .unwrap()
            })
        };
        assert!(matches!(
            run(0.5),
            ClosedConicClipOutcome::Fragments(ref fragments) if fragments.is_empty()
        ));
        assert!(matches!(
            run(2.0),
            ClosedConicClipOutcome::Fragments(ref fragments)
                if fragments.len() == 1 && fragments[0].start.is_none()
        ));
    }

    #[test]
    fn periodic_ring_trim_certifies_inside_outside_and_coincident() {
        let mut store = Store::new();
        let body = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let side = face_where(&store, body, |surface| {
            matches!(surface, SurfaceGeom::Cylinder(_))
        });
        let boundary = {
            let run = |height| {
                with_scope(|scope| {
                    clip_closed_conic_to_face(
                        &store,
                        side,
                        SectionUvCurve::Line(SectionUvLine {
                            origin: Point2::new(0.0, height),
                            direction: Point2::new(1.0, 0.0),
                        }),
                        ParamRange::new(0.0, core::f64::consts::TAU),
                        scope,
                    )
                    .unwrap()
                })
            };
            assert!(matches!(
                run(1.0),
                ClosedConicClipOutcome::Fragments(ref fragments)
                    if fragments.len() == 1 && fragments[0].start.is_none()
            ));
            assert!(matches!(
                run(3.0),
                ClosedConicClipOutcome::Fragments(ref fragments) if fragments.is_empty()
            ));
            let ClosedConicClipOutcome::CoincidentSourceBoundary(boundary) = run(0.0) else {
                panic!("exact ring coincidence must retain topology-owned evidence")
            };
            boundary
        };
        assert_eq!(boundary.face, side);
        let mapped = coincident_source_edge_parameter(boundary, Interval::point(1.0)).unwrap();
        assert!(mapped.contains(1.0));

        let loop_id = store.get(side).unwrap().loops()[0];
        let fin = store.get(loop_id).unwrap().fins()[0];
        let edge = store.get(fin).unwrap().edge();
        let mut transaction = store.transaction().unwrap();
        transaction.assembly().get_mut(edge).unwrap().tolerance =
            Some(ktopo::tolerance::EntityTolerance::imported_xt(2.0 * LINEAR_RESOLUTION).unwrap());
        transaction.commit_checked_body(body).unwrap();
        let tolerance_backed = with_scope(|scope| {
            clip_closed_conic_to_face(
                &store,
                side,
                SectionUvCurve::Line(SectionUvLine {
                    origin: Point2::new(0.0, 0.0),
                    direction: Point2::new(1.0, 0.0),
                }),
                ParamRange::new(0.0, core::f64::consts::TAU),
                scope,
            )
            .unwrap()
        });
        assert_eq!(
            tolerance_backed,
            ClosedConicClipOutcome::Indeterminate(ClosedConicClipGap::UnsupportedTrim),
            "a tolerance-backed ring cannot become exact coincidence evidence"
        );
    }

    #[test]
    fn polygon_clip_work_has_an_exact_n_and_n_minus_one_boundary() {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [2.0, 4.0, 1.0]).unwrap();
        let face = face_where(
            &store,
            body,
            |surface| matches!(surface, SurfaceGeom::Plane(plane) if plane.frame().z().z.abs() == 1.0),
        );
        let policy = SessionPolicy::v1();
        let tolerances = Tolerances::default();
        let baseline_context = OperationContext::new(&policy, tolerances)
            .unwrap()
            .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults());
        let mut baseline_scope = OperationScope::new(&baseline_context);
        let baseline = clip_closed_conic_to_face(
            &store,
            face,
            circle(1.5),
            ParamRange::new(0.0, core::f64::consts::TAU),
            &mut baseline_scope,
        )
        .unwrap();
        assert!(matches!(baseline, ClosedConicClipOutcome::Fragments(_)));
        let consumed = baseline_scope
            .ledger()
            .snapshots()
            .into_iter()
            .find(|snapshot| {
                snapshot.stage == SECTION_WORK && snapshot.resource == ResourceKind::Work
            })
            .unwrap()
            .consumed;
        assert!(consumed > 0);

        let run = |allowed| {
            let overrides = BudgetPlan::new([LimitSpec::new(
                SECTION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap();
            let context = OperationContext::new(&policy, tolerances)
                .unwrap()
                .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults())
                .with_budget_overrides(overrides);
            let mut scope = OperationScope::new(&context);
            clip_closed_conic_to_face(
                &store,
                face,
                circle(1.5),
                ParamRange::new(0.0, core::f64::consts::TAU),
                &mut scope,
            )
        };
        assert_eq!(
            run(consumed - 1).unwrap_err().limit().unwrap().stage,
            SECTION_WORK
        );
        assert!(matches!(
            run(consumed).unwrap(),
            ClosedConicClipOutcome::Fragments(_)
        ));
    }
}
