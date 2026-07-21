//! Exact clipping of a section carrier line against one planar face.
//!
//! All combinatorial decisions — which side of the cutting plane a loop
//! vertex sits on, whether a loop edge crosses the carrier, whether a
//! crossing lands on a vertex — are exact `orient3d`/`orient2d` signs
//! evaluated on stored vertex coordinates. The cutting plane is represented
//! by three stored vertices of the opposing face, never by a derived
//! normal, so every sign is a statement about input data. Metric crossing
//! parameters along the carrier are conservative intervals used only to
//! order crossings; an ordering the intervals cannot certify and no exact
//! coincidence key resolves is a structured gap, never a tie-break.

use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, affine_dot3, orient2d, orient3d};
use kgeom::surface::Plane;
use kgeom::vec::Point3;
use ktopo::entity::{EdgeId as RawEdgeId, FaceId as RawFaceId, Loop, VertexId as RawVertexId};
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::store::Store;

use crate::error::{Error, Result};

use super::{
    GAP_BOUNDED_EDGES_ONLY, GAP_CARRIER_ORIENTATION, GAP_LINE_EDGES_ONLY, GAP_NO_LOOPS,
    GAP_PLANAR_ONLY, GAP_SHORT_LOOP, GAP_TANGENT_CONTACT, GAP_UNORDERED_CROSSINGS, SECTION_WORK,
};

/// Exact plane representation: three affinely independent stored vertex
/// coordinates of the owning face, plus the orientation of the resulting
/// `orient3d` sign relative to the face's outward normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct PlaneWitness {
    /// Stored, unmodified vertex coordinates spanning the face plane.
    pub points: [[f64; 3]; 3],
    /// `true` when `orient3d(points[0], points[1], points[2], x) > 0` means
    /// `x` lies on the face's outward-normal side.
    pub positive_is_outward: bool,
}

/// One vertex of a prepared boundary ring with its exact coordinates and
/// the edge leading to the ring successor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct PreparedRingVertex {
    pub vertex: RawVertexId,
    pub point: [f64; 3],
    /// Edge from this vertex to the ring successor (the owning fin's edge).
    pub edge: RawEdgeId,
    /// Intrinsic source-edge parameters at this vertex and the ring
    /// successor, in ring traversal order. A reversed fin therefore stores
    /// `[hi, lo]`, preserving the source curve's parameterization rather
    /// than silently replacing it with a traversal-local fraction.
    pub edge_parameters: [f64; 2],
}

/// One boundary loop of a prepared face as an ordered exact vertex ring.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct PreparedRing {
    pub vertices: Vec<PreparedRingVertex>,
}

/// A face admitted to the certified planar slice.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct PreparedSectionFace {
    pub raw: RawFaceId,
    pub witness: PlaneWitness,
    pub rings: Vec<PreparedRing>,
    /// Conservative axis-aligned bounds over every ring vertex, outward
    /// rounded — the broad-phase box.
    pub bounds: [Interval; 3],
}

/// Numeric carrier line of one candidate face pair, as returned by the
/// certified plane/plane intersection branch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SectionCarrierLine {
    pub origin: [f64; 3],
    pub direction: [f64; 3],
}

/// Where along its owning face boundary a carrier crossing occurred.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CrossingSite {
    /// The carrier crosses the interior of this boundary edge.
    EdgeInterior(RawEdgeId),
    /// The carrier passes exactly through this boundary vertex.
    AtVertex(RawVertexId),
}

/// One certified boundary crossing along the carrier.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LineCrossing {
    pub site: CrossingSite,
    /// Conservative enclosure of the crossing's carrier parameter.
    pub parameter: Interval,
    /// Conservative enclosure of the intrinsic source-edge parameter.
    /// Present exactly for [`CrossingSite::EdgeInterior`].
    pub edge_parameter: Option<Interval>,
}

/// One maximal carrier span certified inside the face's trim region.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ClipSpan {
    pub start: LineCrossing,
    pub end: LineCrossing,
}

/// Outcome of clipping one carrier against one face.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum ClipOutcome {
    /// Certified inside spans in certified carrier order (possibly empty).
    Spans(Vec<ClipSpan>),
    /// The clip could not be certified; stable reason.
    Gap(&'static str),
}

/// Endpoint attribution of one merged span: which operand boundary produced
/// it. `None` means the carrier stays inside that operand's face there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MergedEndpoint {
    pub a: Option<CrossingSite>,
    pub b: Option<CrossingSite>,
    pub parameter: Interval,
    /// Intrinsic source-edge parameter evidence in operand order. Each slot
    /// is present exactly when the corresponding site is `EdgeInterior`.
    pub edge_parameters: [Option<Interval>; 2],
}

/// One maximal carrier span certified inside both faces' trim regions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MergedSpan {
    pub start: MergedEndpoint,
    pub end: MergedEndpoint,
}

/// Outcome of intersecting the two operands' clip spans along one carrier.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum MergeOutcome {
    /// Certified common spans in certified carrier order (possibly empty).
    Spans(Vec<MergedSpan>),
    /// The merge could not be certified; stable reason.
    Gap(&'static str),
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

fn as_coords(point: Point3) -> [f64; 3] {
    [point.x, point.y, point.z]
}

fn project(point: [f64; 3], drop_axis: usize) -> [f64; 2] {
    match drop_axis {
        0 => [point[1], point[2]],
        1 => [point[0], point[2]],
        _ => [point[0], point[1]],
    }
}

/// Per-axis exact-point difference `a - b` as conservative intervals.
fn sub_iv(a: [f64; 3], b: [f64; 3]) -> [Interval; 3] {
    [0, 1, 2].map(|axis| Interval::point(a[axis]) - Interval::point(b[axis]))
}

/// Conservative cross product of interval vectors.
fn cross_iv(a: [Interval; 3], b: [Interval; 3]) -> [Interval; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Conservative dot product of interval vectors.
fn dot_iv(a: [Interval; 3], b: [Interval; 3]) -> Interval {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// An interval usable for certified ordering: both bounds finite.
fn finite(t: Interval) -> Option<Interval> {
    (t.lo().is_finite() && t.hi().is_finite()).then_some(t)
}

/// Exact affine independence of three points: some coordinate-plane
/// projection has a nonzero `orient2d`, i.e. the cross product of the two
/// spanned edge vectors has a nonzero component.
fn affinely_independent(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> bool {
    (0..3).any(|axis| {
        orient2d(project(a, axis), project(b, axis), project(c, axis)) != Orientation::Zero
    })
}

/// Deterministically select three affinely independent ring vertices and
/// certify the sign relation between their `orient3d` and the face's
/// outward normal: `orient3d(a, b, c, x) > 0` iff
/// `((b-a) × (c-a)) · (x-a) < 0`, so a certified sign of the witness
/// normal against `outward` fixes [`PlaneWitness::positive_is_outward`].
/// `Ok(None)` means no triple certifies — the face is metrically
/// degenerate for this purpose.
fn certify_plane_witness(
    rings: &[PreparedRing],
    outward: [f64; 3],
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<PlaneWitness>> {
    let points: Vec<[f64; 3]> = rings
        .iter()
        .flat_map(|ring| ring.vertices.iter().map(|v| v.point))
        .collect();
    let first = points[0];
    let outward_iv = [0, 1, 2].map(|axis| Interval::point(outward[axis]));
    for i1 in 1..points.len() {
        for i2 in (i1 + 1)..points.len() {
            charge(scope, 1)?;
            let (b, c) = (points[i1], points[i2]);
            if !affinely_independent(first, b, c) {
                continue;
            }
            let normal = cross_iv(sub_iv(b, first), sub_iv(c, first));
            let aligned = dot_iv(normal, outward_iv);
            let positive_is_outward = if aligned.hi() < 0.0 {
                true
            } else if aligned.lo() > 0.0 {
                false
            } else {
                // The interval straddles zero: this triple cannot certify
                // the orientation; try the next one.
                continue;
            };
            return Ok(Some(PlaneWitness {
                points: [first, b, c],
                positive_is_outward,
            }));
        }
    }
    Ok(None)
}

/// Conservative outward-rounded axis bounds over every ring vertex.
fn ring_bounds(rings: &[PreparedRing]) -> [Interval; 3] {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for ring in rings {
        for v in &ring.vertices {
            for axis in 0..3 {
                lo[axis] = lo[axis].min(v.point[axis]);
                hi[axis] = hi[axis].max(v.point[axis]);
            }
        }
    }
    [0, 1, 2].map(|axis| Interval::new(lo[axis].next_down(), hi[axis].next_up()))
}

/// Prepare one face for exact sectioning, or return the stable admission
/// gap that excludes it from the certified planar slice.
///
/// Admission requires: a planar surface, at least one loop, every loop with
/// at least three vertices, every boundary edge a bounded straight line
/// with vertices at both ends, and three affinely independent loop vertices
/// for the plane witness. The witness orientation is certified against the
/// face's stored surface frame and sense.
pub(crate) fn prepare_section_face(
    store: &Store,
    face: RawFaceId,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<core::result::Result<PreparedSectionFace, &'static str>> {
    // Admission is combinatorial and exact; the session linear tolerance
    // participates only in the broad phase over the prepared bounds.
    let _ = linear;
    let face_data = read(store.get(face))?;
    let SurfaceGeom::Plane(plane) = read(store.surface(face_data.surface))? else {
        return Ok(Err(GAP_PLANAR_ONLY));
    };
    if face_data.loops().is_empty() {
        return Ok(Err(GAP_NO_LOOPS));
    }

    let mut rings = Vec::with_capacity(face_data.loops().len());
    for &loop_id in face_data.loops() {
        let ring = read(store.get::<Loop>(loop_id))?;
        charge(scope, 1 + ring.fins().len() as u64)?;
        let mut vertices = Vec::with_capacity(ring.fins().len());
        for &fin_id in ring.fins() {
            let fin = read(store.get(fin_id))?;
            let edge = read(store.get(fin.edge))?;
            let Some(curve_id) = edge.curve else {
                return Ok(Err(GAP_LINE_EDGES_ONLY));
            };
            if !matches!(read(store.curve(curve_id))?, CurveGeom::Line(_)) {
                return Ok(Err(GAP_LINE_EDGES_ONLY));
            }
            let (Some(v0), Some(v1)) = (edge.vertices[0], edge.vertices[1]) else {
                return Ok(Err(GAP_BOUNDED_EDGES_ONLY));
            };
            let Some((lo, hi)) = edge.bounds else {
                return Ok(Err(GAP_BOUNDED_EDGES_ONLY));
            };
            if !lo.is_finite() || !hi.is_finite() || lo >= hi {
                return Ok(Err(GAP_BOUNDED_EDGES_ONLY));
            }
            let tail = if fin.sense.is_forward() { v0 } else { v1 };
            let edge_parameters = if fin.sense.is_forward() {
                [lo, hi]
            } else {
                [hi, lo]
            };
            let point = as_coords(read(store.vertex_position(tail))?);
            if point.iter().any(|c| !c.is_finite()) {
                // A non-finite stored coordinate can certify nothing; the
                // witness orientation below could never be certified.
                return Ok(Err(GAP_CARRIER_ORIENTATION));
            }
            vertices.push(PreparedRingVertex {
                vertex: tail,
                point,
                edge: fin.edge,
                edge_parameters,
            });
        }
        if vertices.len() < 3 {
            return Ok(Err(GAP_SHORT_LOOP));
        }
        rings.push(PreparedRing { vertices });
    }

    let z = plane.frame().z();
    let outward = if face_data.sense.is_forward() {
        [z.x, z.y, z.z]
    } else {
        [-z.x, -z.y, -z.z]
    };
    if outward.iter().any(|c| !c.is_finite()) {
        return Ok(Err(GAP_CARRIER_ORIENTATION));
    }
    let Some(witness) = certify_plane_witness(&rings, outward, scope)? else {
        return Ok(Err(GAP_CARRIER_ORIENTATION));
    };
    let bounds = ring_bounds(&rings);
    Ok(Ok(PreparedSectionFace {
        raw: face,
        witness,
        rings,
        bounds,
    }))
}

/// Certify whether the two faces' broad-phase boxes provably miss each
/// other under the session linear tolerance. `true` means a proven miss.
pub(crate) fn boxes_certifiably_disjoint(
    a: &PreparedSectionFace,
    b: &PreparedSectionFace,
    linear: f64,
) -> bool {
    if !linear.is_finite() || linear < 0.0 {
        // An unusable inflation cannot certify a miss; fail closed.
        return false;
    }
    let pad = Interval::new(-linear, linear);
    for axis in 0..3 {
        let ia = a.bounds[axis] + pad;
        let ib = b.bounds[axis] + pad;
        if ia.hi() < ib.lo() || ib.hi() < ia.lo() {
            return true;
        }
    }
    false
}

/// Conservative enclosure of `((c1-c0) × (c2-c0)) · (p - c0)`: the signed
/// cutter offset whose exact sign `orient3d(c0, c1, c2, p)` decides.
fn cutter_offset(cutter: &PlaneWitness, p: [f64; 3]) -> Interval {
    let normal = cross_iv(
        sub_iv(cutter.points[1], cutter.points[0]),
        sub_iv(cutter.points[2], cutter.points[0]),
    );
    dot_iv(normal, sub_iv(p, cutter.points[0]))
}

/// Exact and interval views of the plane used to cut a prepared polygon.
///
/// Stored-vertex witnesses retain the original planar/planar path.  Analytic
/// planes cover vertexless topology such as finite-cylinder caps: their
/// authored origin and normal feed the exact affine predicate directly, so
/// no derived sample point or rounded plane reconstruction owns a side sign.
#[derive(Debug, Clone, Copy)]
enum PlaneCutter<'a> {
    Witness(&'a PlaneWitness),
    Analytic(&'a Plane),
}

impl PlaneCutter<'_> {
    fn sign(self, point: [f64; 3]) -> Option<Orientation> {
        match self {
            Self::Witness(witness) => Some(orient3d(
                witness.points[0],
                witness.points[1],
                witness.points[2],
                point,
            )),
            Self::Analytic(plane) => affine_dot3(
                plane.frame().z().to_array(),
                point,
                plane.frame().origin().to_array(),
                0.0,
            )
            .map(|value| value.sign()),
        }
    }

    fn offset(self, point: [f64; 3]) -> Interval {
        match self {
            Self::Witness(witness) => cutter_offset(witness, point),
            Self::Analytic(plane) => dot_iv(
                plane.frame().z().to_array().map(Interval::point),
                sub_iv(point, plane.frame().origin().to_array()),
            ),
        }
    }
}

/// Conservative enclosure of the carrier parameter of one exact point:
/// `((p - origin) · direction) / |direction|²`.
fn vertex_parameter(
    point: [f64; 3],
    carrier: &SectionCarrierLine,
    direction_sq: Interval,
) -> Option<Interval> {
    let direction = [0, 1, 2].map(|axis| Interval::point(carrier.direction[axis]));
    let along = dot_iv(sub_iv(point, carrier.origin), direction);
    finite(along.checked_div(direction_sq)?)
}

/// Conservative enclosure of the carrier parameter where the boundary
/// segment `p0 → p1` meets the cutter plane. The endpoints carry opposite
/// exact side signs, so the true crossing exists; the cutter offset is
/// affine along the segment, so the interval interpolation ratio encloses
/// its exact root.
fn edge_crossing_parameters(
    p0: [f64; 3],
    p1: [f64; 3],
    edge_parameters: [f64; 2],
    carrier: &SectionCarrierLine,
    cutter: PlaneCutter<'_>,
    direction_sq: Interval,
) -> Option<(Interval, Interval)> {
    let f0 = cutter.offset(p0);
    let f1 = cutter.offset(p1);
    let s = f0.checked_div(f0 - f1)?;
    let mut along = Interval::point(0.0);
    for axis in 0..3 {
        let x =
            Interval::point(p0[axis]) + s * (Interval::point(p1[axis]) - Interval::point(p0[axis]));
        along = along
            + (x - Interval::point(carrier.origin[axis]))
                * Interval::point(carrier.direction[axis]);
    }
    let carrier_parameter = finite(along.checked_div(direction_sq)?)?;

    // Use the same certified affine interpolation ratio to retain the
    // crossing in the source edge's intrinsic parameterization. Intersect
    // with the active edge bounds: the exact sign flip proves the root lies
    // in the open segment, while interval widening may harmlessly extend
    // beyond its endpoints.
    let [t0, t1] = edge_parameters;
    let source_parameter =
        finite(Interval::point(t0) + s * (Interval::point(t1) - Interval::point(t0)))?;
    let active = Interval::new(t0.min(t1), t0.max(t1));
    let source_parameter = intersect_intervals(source_parameter, active)?;
    Some((carrier_parameter, source_parameter))
}

/// Exact crossing discovery over one boundary ring: an `orient3d` sign
/// against the cutter at every vertex, a transverse crossing for every
/// nonzero sign flip along an edge, and one `AtVertex` crossing for every
/// zero-sign vertex whose neighbors take opposite signs. A zero vertex
/// whose neighbors agree (a touch) and any run of two or more zero
/// vertices (a collinear boundary edge) are tangential contacts this slice
/// refuses. `Ok(Some(reason))` reports the ring's stable gap.
fn ring_crossings(
    ring: &PreparedRing,
    carrier: &SectionCarrierLine,
    cutter: PlaneCutter<'_>,
    direction_sq: Interval,
    crossings: &mut Vec<LineCrossing>,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<&'static str>> {
    let n = ring.vertices.len();
    charge(scope, n as u64)?;
    let Some(signs): Option<Vec<Orientation>> = ring
        .vertices
        .iter()
        .map(|vertex| cutter.sign(vertex.point))
        .collect()
    else {
        return Ok(Some(GAP_CARRIER_ORIENTATION));
    };
    for i in 0..n {
        if signs[i] == Orientation::Zero && signs[(i + 1) % n] == Orientation::Zero {
            return Ok(Some(GAP_TANGENT_CONTACT));
        }
    }
    for i in 0..n {
        if signs[i] == Orientation::Zero {
            // Both cyclic neighbors are nonzero here (checked above).
            let prev = signs[(i + n - 1) % n];
            let next = signs[(i + 1) % n];
            if prev == next {
                return Ok(Some(GAP_TANGENT_CONTACT));
            }
            charge(scope, 1)?;
            let Some(parameter) = vertex_parameter(ring.vertices[i].point, carrier, direction_sq)
            else {
                return Ok(Some(GAP_UNORDERED_CROSSINGS));
            };
            crossings.push(LineCrossing {
                site: CrossingSite::AtVertex(ring.vertices[i].vertex),
                parameter,
                edge_parameter: None,
            });
        } else {
            let j = (i + 1) % n;
            if signs[j] != Orientation::Zero && signs[j] != signs[i] {
                charge(scope, 1)?;
                let Some((parameter, edge_parameter)) = edge_crossing_parameters(
                    ring.vertices[i].point,
                    ring.vertices[j].point,
                    ring.vertices[i].edge_parameters,
                    carrier,
                    cutter,
                    direction_sq,
                ) else {
                    return Ok(Some(GAP_UNORDERED_CROSSINGS));
                };
                crossings.push(LineCrossing {
                    site: CrossingSite::EdgeInterior(ring.vertices[i].edge),
                    parameter,
                    edge_parameter: Some(edge_parameter),
                });
            }
        }
    }
    Ok(None)
}

/// Clip the carrier against one prepared face using the opposing face's
/// plane witness for every side sign.
///
/// Every ring vertex takes an exact `orient3d` sign against `cutter`;
/// sign changes along a ring identify crossings, zero signs identify
/// exact vertex contacts. Crossing parameters are conservative intervals
/// along `carrier`; crossings are ordered by certified interval
/// separation with exact combinatorial coincidence as the only tie-break.
/// Even-odd parity over the ordered crossings yields the inside spans.
pub(crate) fn clip_face_with_plane(
    face: &PreparedSectionFace,
    carrier: &SectionCarrierLine,
    cutter: &PlaneWitness,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ClipOutcome> {
    clip_face_with_cutter(face, carrier, PlaneCutter::Witness(cutter), linear, scope)
}

/// Clip a polygonal planar face by an authored analytic plane.
///
/// This is the vertexless-cap counterpart of [`clip_face_with_plane`]. Exact
/// affine signs use the plane frame directly; interval interpolation retains
/// the same conservative crossing and source-edge parameter evidence.
pub(crate) fn clip_face_with_analytic_plane(
    face: &PreparedSectionFace,
    carrier: &SectionCarrierLine,
    cutter: &Plane,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ClipOutcome> {
    clip_face_with_cutter(face, carrier, PlaneCutter::Analytic(cutter), linear, scope)
}

fn clip_face_with_cutter(
    face: &PreparedSectionFace,
    carrier: &SectionCarrierLine,
    cutter: PlaneCutter<'_>,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ClipOutcome> {
    // The clip itself is exact; the session linear tolerance participates
    // only in the broad phase.
    let _ = linear;
    let inputs_finite = carrier
        .origin
        .iter()
        .chain(carrier.direction.iter())
        .all(|c| c.is_finite());
    let cutter_finite = match cutter {
        PlaneCutter::Witness(witness) => witness.points.iter().flatten().all(|c| c.is_finite()),
        PlaneCutter::Analytic(plane) => plane
            .frame()
            .origin()
            .to_array()
            .into_iter()
            .chain(plane.frame().z().to_array())
            .all(f64::is_finite),
    };
    if !inputs_finite || !cutter_finite {
        return Ok(ClipOutcome::Gap(GAP_CARRIER_ORIENTATION));
    }
    let direction_sq = [0, 1, 2]
        .map(|axis| Interval::point(carrier.direction[axis]).square())
        .into_iter()
        .fold(Interval::point(0.0), |sum, term| sum + term);
    if direction_sq.lo() <= 0.0 || direction_sq.lo().is_nan() {
        return Ok(ClipOutcome::Gap(GAP_CARRIER_ORIENTATION));
    }

    let mut crossings: Vec<LineCrossing> = Vec::new();
    for ring in &face.rings {
        if let Some(reason) =
            ring_crossings(ring, carrier, cutter, direction_sq, &mut crossings, scope)?
        {
            return Ok(ClipOutcome::Gap(reason));
        }
    }

    charge(scope, crossings.len() as u64)?;
    crossings.sort_by(|x, y| {
        x.parameter
            .lo()
            .total_cmp(&y.parameter.lo())
            .then(x.parameter.hi().total_cmp(&y.parameter.hi()))
    });
    for pair in crossings.windows(2) {
        // Crossing enclosures are finite by construction, so `>=` is the
        // exact complement of certified strict separation.
        if pair[0].parameter.hi() >= pair[1].parameter.lo() {
            // The same stored site meeting the carrier twice at one point
            // is a pinch — a tangential contact; distinct sites whose
            // enclosures overlap are an uncertifiable ordering.
            return Ok(ClipOutcome::Gap(if pair[0].site == pair[1].site {
                GAP_TANGENT_CONTACT
            } else {
                GAP_UNORDERED_CROSSINGS
            }));
        }
    }
    if !crossings.len().is_multiple_of(2) {
        // Impossible for closed rings with only parity-flipping crossings;
        // refuse rather than guess a span structure.
        return Ok(ClipOutcome::Gap(GAP_UNORDERED_CROSSINGS));
    }
    let spans = crossings
        .chunks_exact(2)
        .map(|pair| ClipSpan {
            start: pair[0],
            end: pair[1],
        })
        .collect();
    Ok(ClipOutcome::Spans(spans))
}

/// Certified relation of two crossings along one shared carrier.
enum CrossingOrder {
    /// The first crossing certifiably precedes the second.
    Before,
    /// The second crossing certifiably precedes the first.
    After,
    /// Exactly the same point, proven by a shared stored vertex.
    Same,
    /// Overlapping enclosures with no exact coincidence key.
    Unordered,
}

/// Order two crossings by strict interval separation, with a shared stored
/// vertex as the only exact coincidence key (an edge interior does not pin
/// a point, so equal edge sites stay unordered).
fn order_crossings(x: &LineCrossing, y: &LineCrossing) -> CrossingOrder {
    let bounds = [
        x.parameter.lo(),
        x.parameter.hi(),
        y.parameter.lo(),
        y.parameter.hi(),
    ];
    if bounds.iter().any(|value| !value.is_finite()) {
        return CrossingOrder::Unordered;
    }
    if x.parameter.hi() < y.parameter.lo() {
        CrossingOrder::Before
    } else if y.parameter.hi() < x.parameter.lo() {
        CrossingOrder::After
    } else if x.site == y.site && matches!(x.site, CrossingSite::AtVertex(_)) {
        CrossingOrder::Same
    } else {
        CrossingOrder::Unordered
    }
}

/// Intersection of two conservative enclosures of the same true value;
/// `None` when the bounds no longer overlap (or are not comparable).
fn intersect_intervals(x: Interval, y: Interval) -> Option<Interval> {
    let lo = x.lo().max(y.lo());
    let hi = x.hi().min(y.hi());
    (lo <= hi).then(|| Interval::new(lo, hi))
}

/// The later of the two operands' span starts as a merged endpoint, or
/// `None` when the order cannot be certified.
fn merged_start(a: &LineCrossing, b: &LineCrossing) -> Option<MergedEndpoint> {
    match order_crossings(a, b) {
        CrossingOrder::Before => Some(MergedEndpoint {
            a: None,
            b: Some(b.site),
            parameter: b.parameter,
            edge_parameters: [None, b.edge_parameter],
        }),
        CrossingOrder::After => Some(MergedEndpoint {
            a: Some(a.site),
            b: None,
            parameter: a.parameter,
            edge_parameters: [a.edge_parameter, None],
        }),
        CrossingOrder::Same => {
            intersect_intervals(a.parameter, b.parameter).map(|parameter| MergedEndpoint {
                a: Some(a.site),
                b: Some(b.site),
                parameter,
                edge_parameters: [a.edge_parameter, b.edge_parameter],
            })
        }
        CrossingOrder::Unordered => None,
    }
}

/// The earlier of the two operands' span ends as a merged endpoint plus
/// which operand span(s) it exhausts, or `None` when the order cannot be
/// certified.
fn merged_end(a: &LineCrossing, b: &LineCrossing) -> Option<(MergedEndpoint, bool, bool)> {
    match order_crossings(a, b) {
        CrossingOrder::Before => Some((
            MergedEndpoint {
                a: Some(a.site),
                b: None,
                parameter: a.parameter,
                edge_parameters: [a.edge_parameter, None],
            },
            true,
            false,
        )),
        CrossingOrder::After => Some((
            MergedEndpoint {
                a: None,
                b: Some(b.site),
                parameter: b.parameter,
                edge_parameters: [None, b.edge_parameter],
            },
            false,
            true,
        )),
        CrossingOrder::Same => intersect_intervals(a.parameter, b.parameter).map(|parameter| {
            (
                MergedEndpoint {
                    a: Some(a.site),
                    b: Some(b.site),
                    parameter,
                    edge_parameters: [a.edge_parameter, b.edge_parameter],
                },
                true,
                true,
            )
        }),
        CrossingOrder::Unordered => None,
    }
}

/// Intersect the two operands' inside spans along one shared carrier.
///
/// Span endpoints merge by certified interval ordering; endpoints whose
/// intervals overlap merge only when an exact coincidence key (a shared
/// stored vertex lying exactly on both cutting planes) proves they are the
/// same point, otherwise the merge refuses with a stable gap.
pub(crate) fn merge_clip_spans(
    a: &[ClipSpan],
    b: &[ClipSpan],
    scope: &mut OperationScope<'_, '_>,
) -> Result<MergeOutcome> {
    let mut spans = Vec::new();
    let (mut i, mut j) = (0, 0);
    while i < a.len() && j < b.len() {
        charge(scope, 1)?;
        let (span_a, span_b) = (&a[i], &b[j]);
        // Disjointness: one span certifiably ends before the other starts.
        // A certified single-point touch (shared stored vertex) is a
        // tangential contact this slice does not stitch.
        match order_crossings(&span_a.end, &span_b.start) {
            CrossingOrder::Before => {
                i += 1;
                continue;
            }
            CrossingOrder::Same => return Ok(MergeOutcome::Gap(GAP_TANGENT_CONTACT)),
            CrossingOrder::Unordered => return Ok(MergeOutcome::Gap(GAP_UNORDERED_CROSSINGS)),
            CrossingOrder::After => {}
        }
        match order_crossings(&span_b.end, &span_a.start) {
            CrossingOrder::Before => {
                j += 1;
                continue;
            }
            CrossingOrder::Same => return Ok(MergeOutcome::Gap(GAP_TANGENT_CONTACT)),
            CrossingOrder::Unordered => return Ok(MergeOutcome::Gap(GAP_UNORDERED_CROSSINGS)),
            CrossingOrder::After => {}
        }
        // Both interior orderings are certified, so the spans overlap with
        // certified positive length.
        let Some(start) = merged_start(&span_a.start, &span_b.start) else {
            return Ok(MergeOutcome::Gap(GAP_UNORDERED_CROSSINGS));
        };
        let Some((end, exhausted_a, exhausted_b)) = merged_end(&span_a.end, &span_b.end) else {
            return Ok(MergeOutcome::Gap(GAP_UNORDERED_CROSSINGS));
        };
        spans.push(MergedSpan { start, end });
        if exhausted_a {
            i += 1;
        }
        if exhausted_b {
            j += 1;
        }
    }
    Ok(MergeOutcome::Spans(spans))
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point2, Point3, Vec3};
    use ktopo::entity::BodyId as RawBodyId;
    use ktopo::profile::PlanarProfile;

    use super::*;
    use crate::section::BodySectionBudgetProfile;

    fn linear() -> f64 {
        Tolerances::default().linear()
    }

    fn with_scope<T>(run: impl FnOnce(&mut OperationScope<'_, '_>) -> T) -> T {
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        run(&mut scope)
    }

    fn block_store() -> (Store, RawBodyId) {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
        (store, body)
    }

    /// Test oracle: `(vertex, position, successor edge)` rings read directly
    /// from store primitives, independent of `prepare_section_face`.
    fn oracle_rings(
        store: &Store,
        face: RawFaceId,
    ) -> Vec<Vec<(RawVertexId, [f64; 3], RawEdgeId)>> {
        let face_data = store.get(face).unwrap();
        face_data
            .loops()
            .iter()
            .map(|&loop_id| {
                let ring = store.get::<Loop>(loop_id).unwrap();
                ring.fins()
                    .iter()
                    .map(|&fin_id| {
                        let fin = store.get(fin_id).unwrap();
                        let tail = store.fin_tail(fin_id).unwrap().unwrap();
                        let p = store.vertex_position(tail).unwrap();
                        (tail, [p.x, p.y, p.z], fin.edge)
                    })
                    .collect()
            })
            .collect()
    }

    /// The unique face all of whose boundary vertices satisfy `pick`.
    fn face_where(store: &Store, body: RawBodyId, pick: impl Fn([f64; 3]) -> bool) -> RawFaceId {
        let mut found = None;
        for face in store.faces_of_body(body).unwrap() {
            let rings = oracle_rings(store, face);
            if !rings.is_empty() && rings.iter().flatten().all(|&(_, p, _)| pick(p)) {
                assert!(found.replace(face).is_none(), "face selection is ambiguous");
            }
        }
        found.expect("no face matched the selector")
    }

    fn prepared(store: &Store, face: RawFaceId) -> PreparedSectionFace {
        with_scope(|scope| {
            prepare_section_face(store, face, linear(), scope)
                .unwrap()
                .unwrap()
        })
    }

    fn clip(
        face: &PreparedSectionFace,
        carrier: &SectionCarrierLine,
        cutter: &PlaneWitness,
    ) -> ClipOutcome {
        with_scope(|scope| clip_face_with_plane(face, carrier, cutter, linear(), scope).unwrap())
    }

    /// `positive_is_outward` is irrelevant to clipping (only sign flips
    /// matter); a fixed arbitrary value keeps that contract honest here.
    fn cutter(points: [[f64; 3]; 3]) -> PlaneWitness {
        PlaneWitness {
            points,
            positive_is_outward: true,
        }
    }

    fn assert_tight(parameter: Interval, expected: f64) {
        assert!(
            parameter.contains(expected),
            "expected {expected} inside {parameter:?}"
        );
        assert!(
            parameter.width() < 1e-9,
            "enclosure unexpectedly wide: {parameter:?}"
        );
    }

    #[test]
    fn edge_interpolation_retains_intrinsic_parameter_through_reversal() {
        // Source segment x=0..10 is cut at independently known x=2. Its
        // intrinsic edge parameters run 3..13, so the crossing is t=5.
        let carrier = SectionCarrierLine {
            origin: [2.0, 0.0, 0.0],
            direction: [0.0, 1.0, 0.0],
        };
        let cutter = cutter([[2.0, -1.0, -1.0], [2.0, 1.0, -1.0], [2.0, 0.0, 1.0]]);
        let direction_sq = Interval::point(1.0);
        let (carrier_forward, edge_forward) = edge_crossing_parameters(
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [3.0, 13.0],
            &carrier,
            PlaneCutter::Witness(&cutter),
            direction_sq,
        )
        .expect("forward crossing certifies");
        assert_tight(carrier_forward, 0.0);
        assert_tight(edge_forward, 5.0);

        // Reversing both boundary traversal and its endpoint parameters
        // must preserve the intrinsic source-edge parameter, rather than
        // returning the traversal-local complement (11).
        let (carrier_reversed, edge_reversed) = edge_crossing_parameters(
            [10.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [13.0, 3.0],
            &carrier,
            PlaneCutter::Witness(&cutter),
            direction_sq,
        )
        .expect("reversed crossing certifies");
        assert_tight(carrier_reversed, 0.0);
        assert_tight(edge_reversed, 5.0);
    }

    fn edge_crossing(edge: RawEdgeId, t: f64) -> LineCrossing {
        LineCrossing {
            site: CrossingSite::EdgeInterior(edge),
            parameter: Interval::point(t),
            edge_parameter: Some(Interval::point(t)),
        }
    }

    fn vertex_crossing(vertex: RawVertexId, t: f64) -> LineCrossing {
        LineCrossing {
            site: CrossingSite::AtVertex(vertex),
            parameter: Interval::point(t),
            edge_parameter: None,
        }
    }

    #[test]
    fn block_admission_witness_and_bounds_are_certified() {
        let (store, body) = block_store();
        for face in store.faces_of_body(body).unwrap() {
            let rings = oracle_rings(&store, face);
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for &(_, p, _) in rings.iter().flatten() {
                for axis in 0..3 {
                    lo[axis] = lo[axis].min(p[axis]);
                    hi[axis] = hi[axis].max(p[axis]);
                }
            }

            let face_prep = prepared(&store, face);
            for axis in 0..3 {
                assert!(face_prep.bounds[axis].lo() <= lo[axis]);
                assert!(face_prep.bounds[axis].hi() >= hi[axis]);
                assert!(lo[axis] - face_prep.bounds[axis].lo() < 1e-12);
                assert!(face_prep.bounds[axis].hi() - hi[axis] < 1e-12);
            }

            // Witness points are stored vertex coordinates of this face.
            let stored: Vec<[f64; 3]> = rings.iter().flatten().map(|&(_, p, _)| p).collect();
            for w in face_prep.witness.points {
                assert!(stored.contains(&w), "witness point {w:?} is not stored");
            }

            // Orientation oracle: a point displaced along the face's known
            // outward normal (the degenerate bounds axis of a centered
            // block, signed by the face center) must land on the side the
            // flag declares outward.
            let axis = (0..3).find(|&axis| lo[axis] == hi[axis]).unwrap();
            let mut outside = [0, 1, 2].map(|axis| (lo[axis] + hi[axis]) / 2.0);
            outside[axis] += if outside[axis] > 0.0 { 1.0 } else { -1.0 };
            let [w0, w1, w2] = face_prep.witness.points;
            let side = orient3d(w0, w1, w2, outside);
            assert_ne!(side, Orientation::Zero);
            assert_eq!(
                side == Orientation::Positive,
                face_prep.witness.positive_is_outward
            );
        }
    }

    #[test]
    fn prepared_ring_parameters_follow_each_fins_edge_sense() {
        let (store, body) = block_store();
        let mut saw_forward = false;
        let mut saw_reversed = false;
        for face in store.faces_of_body(body).unwrap() {
            let prep = prepared(&store, face);
            for ring in &prep.rings {
                for vertex in &ring.vertices {
                    let edge = store.get(vertex.edge).unwrap();
                    let [Some(v0), Some(v1)] = edge.vertices else {
                        panic!("prepared block edge must be bounded");
                    };
                    let (lo, hi) = edge.bounds.unwrap();
                    if vertex.vertex == v0 {
                        saw_forward = true;
                        assert_eq!(vertex.edge_parameters, [lo, hi]);
                    } else {
                        saw_reversed = true;
                        assert_eq!(vertex.vertex, v1);
                        assert_eq!(vertex.edge_parameters, [hi, lo]);
                    }
                }
            }
        }
        assert!(saw_forward && saw_reversed);
    }

    #[test]
    fn curved_faces_are_an_honest_admission_gap() {
        let mut store = Store::new();
        let sphere = ktopo::make::sphere(&mut store, &Frame::world(), 1.0).unwrap();
        let face = store.faces_of_body(sphere).unwrap()[0];
        let outcome =
            with_scope(|scope| prepare_section_face(&store, face, linear(), scope).unwrap());
        assert_eq!(outcome, Err(crate::section::GAP_PLANAR_ONLY));

        // Cylinder: the wall is non-planar, the caps are planar but bounded
        // by circular ring edges.
        let cylinder = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        for face in store.faces_of_body(cylinder).unwrap() {
            let outcome =
                with_scope(|scope| prepare_section_face(&store, face, linear(), scope).unwrap());
            let reason = outcome.unwrap_err();
            assert!(
                reason == crate::section::GAP_PLANAR_ONLY
                    || reason == crate::section::GAP_LINE_EDGES_ONLY,
                "unexpected admission gap: {reason}"
            );
        }
    }

    enum Expected {
        Spans(&'static [(f64, f64)]),
        Gap(&'static str),
    }

    struct ClipCase {
        name: &'static str,
        cutter_points: [[f64; 3]; 3],
        origin: [f64; 3],
        direction: [f64; 3],
        expected: Expected,
    }

    #[test]
    fn rectangle_clip_cases_match_constructed_ground_truth() {
        let (store, body) = block_store();
        let top = prepared(&store, face_where(&store, body, |p| p[2] == 1.0));

        // The top face is the square (±1, ±1) at z = 1. Every expected
        // span parameter below is derived by hand from the cutter plane and
        // carrier stated in the case.
        let cases = [
            ClipCase {
                name: "transverse plane x=0: one span between the y=∓1 edges",
                cutter_points: [[0.0, -5.0, -5.0], [0.0, 5.0, -5.0], [0.0, 0.0, 5.0]],
                origin: [0.0, 0.0, 1.0],
                direction: [0.0, 1.0, 0.0],
                expected: Expected::Spans(&[(-1.0, 1.0)]),
            },
            ClipCase {
                name: "plane x=3 misses the face: no spans",
                cutter_points: [[3.0, -5.0, -5.0], [3.0, 5.0, -5.0], [3.0, 0.0, 5.0]],
                origin: [3.0, 0.0, 1.0],
                direction: [0.0, 1.0, 0.0],
                expected: Expected::Spans(&[]),
            },
            ClipCase {
                name: "diagonal plane x+y=0 through two opposite vertices",
                cutter_points: [[0.0, 0.0, 0.0], [1.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
                origin: [0.0, 0.0, 1.0],
                direction: [1.0, -1.0, 0.0],
                expected: Expected::Spans(&[(-1.0, 1.0)]),
            },
            ClipCase {
                name: "plane x+y=2 touches one vertex without entering",
                cutter_points: [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [2.0, 0.0, 1.0]],
                origin: [1.0, 1.0, 1.0],
                direction: [1.0, -1.0, 0.0],
                expected: Expected::Gap(crate::section::GAP_TANGENT_CONTACT),
            },
            ClipCase {
                name: "plane x=1 contains a whole boundary edge",
                cutter_points: [[1.0, -5.0, -5.0], [1.0, 5.0, -5.0], [1.0, 0.0, 5.0]],
                origin: [1.0, 0.0, 1.0],
                direction: [0.0, 1.0, 0.0],
                expected: Expected::Gap(crate::section::GAP_TANGENT_CONTACT),
            },
        ];

        for ClipCase {
            name,
            cutter_points,
            origin,
            direction,
            expected,
        } in cases
        {
            let outcome = clip(
                &top,
                &SectionCarrierLine { origin, direction },
                &cutter(cutter_points),
            );
            match expected {
                Expected::Spans(expected_spans) => {
                    let ClipOutcome::Spans(spans) = outcome else {
                        panic!("{name}: expected spans, got {outcome:?}");
                    };
                    assert_eq!(spans.len(), expected_spans.len(), "{name}");
                    for (span, &(start, end)) in spans.iter().zip(expected_spans) {
                        assert_tight(span.start.parameter, start);
                        assert_tight(span.end.parameter, end);
                    }
                }
                Expected::Gap(reason) => {
                    assert_eq!(outcome, ClipOutcome::Gap(reason), "{name}");
                }
            }
        }
    }

    #[test]
    fn authored_analytic_plane_clips_polygon_without_derived_witness_points() {
        let (store, body) = block_store();
        let top = prepared(&store, face_where(&store, body, |p| p[2] == 1.0));
        let carrier = SectionCarrierLine {
            origin: [0.0, 0.0, 1.0],
            direction: [0.0, 1.0, 0.0],
        };
        let plane = Plane::new(
            Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
        );
        let outcome = with_scope(|scope| {
            clip_face_with_analytic_plane(&top, &carrier, &plane, linear(), scope).unwrap()
        });
        let ClipOutcome::Spans(spans) = outcome else {
            panic!("authored analytic plane must certify the square chord")
        };
        let [span] = spans.as_slice() else {
            panic!("square chord must be one connected span: {spans:?}")
        };
        assert_tight(span.start.parameter, -1.0);
        assert_tight(span.end.parameter, 1.0);
        assert!(matches!(span.start.site, CrossingSite::EdgeInterior(_)));
        assert!(matches!(span.end.site, CrossingSite::EdgeInterior(_)));
    }

    #[test]
    fn transverse_crossings_carry_exact_edge_sites() {
        let (store, body) = block_store();
        let top_raw = face_where(&store, body, |p| p[2] == 1.0);
        let top = prepared(&store, top_raw);

        // Independent oracle: the two top-face edges whose endpoint x
        // coordinates change sign are the ones the plane x=0 must cross;
        // the y=-1 edge is crossed at carrier parameter -1, the y=+1 edge
        // at +1 (carrier origin (0,0,1), direction +y).
        let ring = &oracle_rings(&store, top_raw)[0];
        let n = ring.len();
        let (mut start_edge, mut end_edge) = (None, None);
        for i in 0..n {
            let (_, p0, edge) = ring[i];
            let (_, p1, _) = ring[(i + 1) % n];
            if p0[0] * p1[0] < 0.0 {
                if p0[1] == -1.0 && p1[1] == -1.0 {
                    start_edge = Some(edge);
                } else if p0[1] == 1.0 && p1[1] == 1.0 {
                    end_edge = Some(edge);
                }
            }
        }
        let (start_edge, end_edge) = (start_edge.unwrap(), end_edge.unwrap());

        let outcome = clip(
            &top,
            &SectionCarrierLine {
                origin: [0.0, 0.0, 1.0],
                direction: [0.0, 1.0, 0.0],
            },
            &cutter([[0.0, -5.0, -5.0], [0.0, 5.0, -5.0], [0.0, 0.0, 5.0]]),
        );
        let ClipOutcome::Spans(spans) = outcome else {
            panic!("expected spans, got {outcome:?}");
        };
        assert_eq!(spans.len(), 1);
        assert_eq!(spans[0].start.site, CrossingSite::EdgeInterior(start_edge));
        assert_eq!(spans[0].end.site, CrossingSite::EdgeInterior(end_edge));
        assert_tight(spans[0].start.edge_parameter.unwrap(), 1.0);
        assert_tight(spans[0].end.edge_parameter.unwrap(), 1.0);
    }

    #[test]
    fn vertex_crossings_carry_exact_vertex_sites_and_flip_parity() {
        let (store, body) = block_store();
        let top_raw = face_where(&store, body, |p| p[2] == 1.0);
        let top = prepared(&store, top_raw);

        // Independent oracle: the plane x+y=0 passes exactly through the
        // stored vertices at (-1,1,1) and (1,-1,1); with carrier direction
        // (1,-1,0) from (0,0,1) their parameters are -1 and +1.
        let ring = &oracle_rings(&store, top_raw)[0];
        let vertex_at = |target: [f64; 3]| {
            ring.iter()
                .find(|&&(_, p, _)| p == target)
                .expect("expected stored vertex")
                .0
        };
        let low = vertex_at([-1.0, 1.0, 1.0]);
        let high = vertex_at([1.0, -1.0, 1.0]);

        let outcome = clip(
            &top,
            &SectionCarrierLine {
                origin: [0.0, 0.0, 1.0],
                direction: [1.0, -1.0, 0.0],
            },
            &cutter([[0.0, 0.0, 0.0], [1.0, -1.0, 0.0], [0.0, 0.0, 1.0]]),
        );
        let ClipOutcome::Spans(spans) = outcome else {
            panic!("expected spans, got {outcome:?}");
        };
        assert_eq!(spans.len(), 1);
        assert_eq!(spans[0].start.site, CrossingSite::AtVertex(low));
        assert_eq!(spans[0].end.site, CrossingSite::AtVertex(high));
        assert_eq!(spans[0].start.edge_parameter, None);
        assert_eq!(spans[0].end.edge_parameter, None);
        assert_tight(spans[0].start.parameter, -1.0);
        assert_tight(spans[0].end.parameter, 1.0);
    }

    #[test]
    fn nonconvex_face_yields_two_spans() {
        let mut store = Store::new();
        let outer = [
            Point2::new(0.0, 0.0),
            Point2::new(3.0, 0.0),
            Point2::new(3.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, 3.0),
            Point2::new(0.0, 3.0),
        ];
        let profile = PlanarProfile::from_polygon(Frame::world(), &outer).unwrap();
        let body = ktopo::make::extrude_profile(&mut store, &profile, 1.0).unwrap();
        let top = prepared(&store, face_where(&store, body, |p| p[2] == 1.0));

        // Plane x+y=3.5 crosses the L twice: through the bottom arm at
        // (3, 0.5) → (2.5, 1) and through the left arm at (1, 2.5) →
        // (0.5, 3). With carrier origin (3.5, 0, 1) and direction (-1, 1, 0)
        // the parameters are t = (3.5 - x + y) / 2.
        let outcome = clip(
            &top,
            &SectionCarrierLine {
                origin: [3.5, 0.0, 1.0],
                direction: [-1.0, 1.0, 0.0],
            },
            &cutter([[3.5, 0.0, 0.0], [0.0, 3.5, 0.0], [3.5, 0.0, 1.0]]),
        );
        let ClipOutcome::Spans(spans) = outcome else {
            panic!("expected spans, got {outcome:?}");
        };
        assert_eq!(spans.len(), 2);
        assert_tight(spans[0].start.parameter, 0.5);
        assert_tight(spans[0].end.parameter, 1.0);
        assert_tight(spans[1].start.parameter, 2.5);
        assert_tight(spans[1].end.parameter, 3.0);

        let mut edges = Vec::new();
        for span in &spans {
            for crossing in [span.start, span.end] {
                let CrossingSite::EdgeInterior(edge) = crossing.site else {
                    panic!("expected edge-interior crossings, got {:?}", crossing.site);
                };
                assert!(!edges.contains(&edge), "crossed edges must be distinct");
                edges.push(edge);
            }
        }
    }

    #[test]
    fn holed_face_yields_spans_on_both_sides_of_the_hole() {
        let mut store = Store::new();
        let outer = [
            Point2::new(-2.0, -2.0),
            Point2::new(2.0, -2.0),
            Point2::new(2.0, 2.0),
            Point2::new(-2.0, 2.0),
        ];
        let hole = [
            Point2::new(-0.5, -0.5),
            Point2::new(0.5, -0.5),
            Point2::new(0.5, 0.5),
            Point2::new(-0.5, 0.5),
        ];
        let profile =
            PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&hole]).unwrap();
        let body = ktopo::make::extrude_profile(&mut store, &profile, 1.0).unwrap();
        let top_raw = face_where(&store, body, |p| p[2] == 1.0);
        let top = prepared(&store, top_raw);

        // Plane y=0 enters the outer boundary at x=-2, exits into the hole
        // at x=-0.5, re-enters at x=0.5, and leaves at x=2; the carrier
        // parameter is x.
        let outcome = clip(
            &top,
            &SectionCarrierLine {
                origin: [0.0, 0.0, 1.0],
                direction: [1.0, 0.0, 0.0],
            },
            &cutter([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
        );
        let ClipOutcome::Spans(spans) = outcome else {
            panic!("expected spans, got {outcome:?}");
        };
        assert_eq!(spans.len(), 2);
        assert_tight(spans[0].start.parameter, -2.0);
        assert_tight(spans[0].end.parameter, -0.5);
        assert_tight(spans[1].start.parameter, 0.5);
        assert_tight(spans[1].end.parameter, 2.0);

        // Independent oracle: the middle crossings sit on hole-ring edges
        // (every hole vertex is within |x|,|y| ≤ 0.5), the outer crossings
        // on outer-ring edges.
        let rings = oracle_rings(&store, top_raw);
        let ring_edges = |pick: &dyn Fn([f64; 3]) -> bool| -> Vec<RawEdgeId> {
            rings
                .iter()
                .filter(|ring| ring.iter().all(|&(_, p, _)| pick(p)))
                .flat_map(|ring| ring.iter().map(|&(_, _, e)| e))
                .collect()
        };
        let hole_edges = ring_edges(&|p| p[0].abs() <= 0.5 && p[1].abs() <= 0.5);
        let outer_edges = ring_edges(&|p| p[0].abs() == 2.0 || p[1].abs() == 2.0);
        assert_eq!(hole_edges.len(), 4);
        assert_eq!(outer_edges.len(), 4);
        let edge_of = |crossing: LineCrossing| {
            let CrossingSite::EdgeInterior(edge) = crossing.site else {
                panic!("expected edge-interior crossing, got {:?}", crossing.site);
            };
            edge
        };
        assert!(outer_edges.contains(&edge_of(spans[0].start)));
        assert!(hole_edges.contains(&edge_of(spans[0].end)));
        assert!(hole_edges.contains(&edge_of(spans[1].start)));
        assert!(outer_edges.contains(&edge_of(spans[1].end)));
    }

    #[test]
    fn merge_intersects_interleaved_and_nested_spans() {
        let mut store = Store::new();
        let body_a = ktopo::make::block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
        let body_b = ktopo::make::block(&mut store, &Frame::world(), [4.0, 4.0, 4.0]).unwrap();
        let ea = store.edges_of_body(body_a).unwrap();
        let eb = store.edges_of_body(body_b).unwrap();

        // Interleaved: A = [1,4] ∪ [6,9], B = [2,7] ⇒ [2,4] ∪ [6,7].
        let a = [
            ClipSpan {
                start: edge_crossing(ea[0], 1.0),
                end: edge_crossing(ea[1], 4.0),
            },
            ClipSpan {
                start: edge_crossing(ea[2], 6.0),
                end: edge_crossing(ea[3], 9.0),
            },
        ];
        let b = [ClipSpan {
            start: edge_crossing(eb[0], 2.0),
            end: edge_crossing(eb[1], 7.0),
        }];
        let outcome = with_scope(|scope| merge_clip_spans(&a, &b, scope).unwrap());
        let MergeOutcome::Spans(merged) = outcome else {
            panic!("expected merged spans, got {outcome:?}");
        };
        assert_eq!(merged.len(), 2);

        assert_eq!(merged[0].start.a, None);
        assert_eq!(merged[0].start.b, Some(CrossingSite::EdgeInterior(eb[0])));
        assert_eq!(merged[0].start.parameter, b[0].start.parameter);
        assert_eq!(
            merged[0].start.edge_parameters,
            [None, Some(Interval::point(2.0))]
        );
        assert_eq!(merged[0].end.a, Some(CrossingSite::EdgeInterior(ea[1])));
        assert_eq!(merged[0].end.b, None);
        assert_eq!(merged[0].end.parameter, a[0].end.parameter);
        assert_eq!(
            merged[0].end.edge_parameters,
            [Some(Interval::point(4.0)), None]
        );

        assert_eq!(merged[1].start.a, Some(CrossingSite::EdgeInterior(ea[2])));
        assert_eq!(merged[1].start.b, None);
        assert_eq!(merged[1].end.a, None);
        assert_eq!(merged[1].end.b, Some(CrossingSite::EdgeInterior(eb[1])));

        // Nested: B covers A ⇒ the merged span is A with no B attribution.
        let wide = [ClipSpan {
            start: edge_crossing(eb[0], 0.0),
            end: edge_crossing(eb[1], 9.5),
        }];
        let outcome = with_scope(|scope| merge_clip_spans(&a[..1], &wide, scope).unwrap());
        let MergeOutcome::Spans(merged) = outcome else {
            panic!("expected merged spans, got {outcome:?}");
        };
        assert_eq!(merged.len(), 1);
        assert_eq!(merged[0].start.a, Some(CrossingSite::EdgeInterior(ea[0])));
        assert_eq!(merged[0].start.b, None);
        assert_eq!(merged[0].end.a, Some(CrossingSite::EdgeInterior(ea[1])));
        assert_eq!(merged[0].end.b, None);
    }

    #[test]
    fn merge_refuses_uncertifiable_and_touching_overlaps() {
        let mut store = Store::new();
        let body_a = ktopo::make::block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
        let body_b = ktopo::make::block(&mut store, &Frame::world(), [4.0, 4.0, 4.0]).unwrap();
        let ea = store.edges_of_body(body_a).unwrap();
        let eb = store.edges_of_body(body_b).unwrap();
        let va = store.vertices_of_body(body_a).unwrap();

        // Cross-operand endpoints at the same parameter share no stored
        // entity: refused as unorderable.
        let a = [ClipSpan {
            start: edge_crossing(ea[0], 1.0),
            end: edge_crossing(ea[1], 4.0),
        }];
        let b = [ClipSpan {
            start: edge_crossing(eb[0], 4.0),
            end: edge_crossing(eb[1], 9.0),
        }];
        let outcome = with_scope(|scope| merge_clip_spans(&a, &b, scope).unwrap());
        assert_eq!(
            outcome,
            MergeOutcome::Gap(crate::section::GAP_UNORDERED_CROSSINGS)
        );

        // A shared stored vertex is an exact coincidence key: the spans
        // certifiably touch in a single point, a tangential contact.
        let a = [ClipSpan {
            start: edge_crossing(ea[0], 1.0),
            end: vertex_crossing(va[0], 4.0),
        }];
        let b = [ClipSpan {
            start: vertex_crossing(va[0], 4.0),
            end: edge_crossing(eb[1], 9.0),
        }];
        let outcome = with_scope(|scope| merge_clip_spans(&a, &b, scope).unwrap());
        assert_eq!(
            outcome,
            MergeOutcome::Gap(crate::section::GAP_TANGENT_CONTACT)
        );
    }

    #[test]
    fn disjoint_boxes_are_certified_only_when_separated() {
        let (mut store, body) = block_store();
        let far_outer = [
            Point2::new(10.0, 10.0),
            Point2::new(12.0, 10.0),
            Point2::new(12.0, 12.0),
            Point2::new(10.0, 12.0),
        ];
        let profile = PlanarProfile::from_polygon(Frame::world(), &far_outer).unwrap();
        let far_body = ktopo::make::extrude_profile(&mut store, &profile, 1.0).unwrap();

        let top = prepared(&store, face_where(&store, body, |p| p[2] == 1.0));
        let bottom = prepared(&store, face_where(&store, body, |p| p[2] == -1.0));
        let side = prepared(&store, face_where(&store, body, |p| p[0] == 1.0));
        let far = prepared(&store, face_where(&store, far_body, |p| p[2] == 1.0));

        // Separated by 9 on x: a certified miss under the session linear
        // tolerance, but not once the inflation swallows the separation.
        assert!(boxes_certifiably_disjoint(&top, &far, linear()));
        assert!(boxes_certifiably_disjoint(&far, &top, linear()));
        assert!(!boxes_certifiably_disjoint(&top, &far, 5.0));

        // Opposite faces of one block are separated on z; adjacent faces
        // share boundary and can never be certified disjoint.
        assert!(boxes_certifiably_disjoint(&top, &bottom, linear()));
        assert!(!boxes_certifiably_disjoint(&top, &side, linear()));

        // Unusable inflation values fail closed.
        assert!(!boxes_certifiably_disjoint(&top, &far, f64::NAN));
        assert!(!boxes_certifiably_disjoint(&top, &far, -1.0));
    }
}
