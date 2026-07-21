//! Exact arrangement and dual classification for a circular cap disk.
//!
//! Section supplies a complete intrinsic order of transverse roots on the
//! cap's vertexless source ring.  This adapter splits that ring into ordinary
//! two-vertex source arcs and inserts proof-keyed chord fragments.  Root and
//! fragment identities, never metric representatives, own the graph.
//!
//! A circular disk is convex, so two open chords cross exactly when their
//! four endpoints alternate in the certified circular order.  That theorem
//! lets this module prove an arbitrary number of chords disjoint without
//! geometric sampling.  The shared bounded-face core then proves source-span
//! conservation, opposed chord uses, cell closure, and connected dual
//! adjacency.  Partial root coverage, tangency, branching, and crossings are
//! typed refusals.

use std::collections::{BTreeMap, BTreeSet};

use ktopo::entity::{EdgeId as RawEdgeId, FinId as RawFinId, LoopId as RawLoopId, Sense};
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use ktopo::store::Store;

use super::face_arrangement::{
    ArrangementCutAdjacency, ArrangementDartKey, ArrangementDirection, ArrangementEdgeKey,
    CertifiedEndpointRotation, DirectedCutFragment, DirectedSourceSpan, FaceArrangement,
    FaceArrangementError, FaceArrangementInput, arrange_bounded_face,
};
use crate::{
    BodySectionGraph, FaceId, SectionBranchTopology, SectionCarrier, SectionCompletion,
    SectionCurveEndpointTopology, SectionCurveFragmentSpan, SectionRulingFragmentEnd, SectionSite,
    SectionUvCurve,
};

/// Whether Section proved that every cap-ring/cutter root was published.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DiskBoundaryCoverage {
    Complete,
    Partial,
}

/// Local intersection type at one exact cap-ring root.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DiskRootContact {
    Transverse,
    Tangent,
    Indeterminate,
}

/// Exact identity of one root on the cap ring.
///
/// `endpoint` is the operation-wide Section endpoint identity.  The two
/// ordinals retain its proof lineage: `circular_ordinal` orders all roots on
/// this ring, while `source_root_ordinal` is the ordinal issued by the
/// complete root authority for the contributing ring/cutter query.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct DiskRootKey {
    endpoint: usize,
    circular_ordinal: usize,
    source_root_ordinal: usize,
}

impl DiskRootKey {
    pub(crate) const fn new(
        endpoint: usize,
        circular_ordinal: usize,
        source_root_ordinal: usize,
    ) -> Self {
        Self {
            endpoint,
            circular_ordinal,
            source_root_ordinal,
        }
    }

    pub(crate) const fn endpoint(self) -> usize {
        self.endpoint
    }

    pub(crate) const fn circular_ordinal(self) -> usize {
        self.circular_ordinal
    }

    pub(crate) const fn source_root_ordinal(self) -> usize {
        self.source_root_ordinal
    }
}

/// Exact root identity plus realization-only scalar evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct DiskBoundaryRootEvidence {
    key: DiskRootKey,
    root_parameter_bits: u64,
    root_enclosure_bits: [u64; 2],
    contact: DiskRootContact,
}

impl DiskBoundaryRootEvidence {
    pub(crate) fn transverse(
        key: DiskRootKey,
        root_parameter: f64,
        root_enclosure: [f64; 2],
    ) -> Self {
        Self {
            key,
            root_parameter_bits: root_parameter.to_bits(),
            root_enclosure_bits: root_enclosure.map(f64::to_bits),
            contact: DiskRootContact::Transverse,
        }
    }

    pub(crate) fn with_contact(
        key: DiskRootKey,
        root_parameter: f64,
        root_enclosure: [f64; 2],
        contact: DiskRootContact,
    ) -> Self {
        Self {
            key,
            root_parameter_bits: root_parameter.to_bits(),
            root_enclosure_bits: root_enclosure.map(f64::to_bits),
            contact,
        }
    }

    pub(crate) const fn key(self) -> DiskRootKey {
        self.key
    }

    pub(crate) const fn root_parameter(self) -> f64 {
        f64::from_bits(self.root_parameter_bits)
    }

    pub(crate) const fn root_enclosure(self) -> [f64; 2] {
        [
            f64::from_bits(self.root_enclosure_bits[0]),
            f64::from_bits(self.root_enclosure_bits[1]),
        ]
    }

    pub(crate) const fn contact(self) -> DiskRootContact {
        self.contact
    }
}

/// Complete source-ring evidence for one circular cap.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CertifiedDiskBoundary {
    edge: RawEdgeId,
    fin: RawFinId,
    sense: Sense,
    coverage: DiskBoundaryCoverage,
    roots: Vec<DiskBoundaryRootEvidence>,
}

impl CertifiedDiskBoundary {
    pub(crate) const fn new(
        edge: RawEdgeId,
        fin: RawFinId,
        sense: Sense,
        coverage: DiskBoundaryCoverage,
        roots: Vec<DiskBoundaryRootEvidence>,
    ) -> Self {
        Self {
            edge,
            fin,
            sense,
            coverage,
            roots,
        }
    }

    pub(crate) const fn edge(&self) -> RawEdgeId {
        self.edge
    }

    pub(crate) const fn fin(&self) -> RawFinId {
        self.fin
    }

    pub(crate) const fn sense(&self) -> Sense {
        self.sense
    }

    pub(crate) fn roots(&self) -> &[DiskBoundaryRootEvidence] {
        &self.roots
    }
}

/// Stable exact identity of one chord: its Section fragment index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct DiskChordKey {
    fragment: usize,
}

impl DiskChordKey {
    pub(crate) const fn new(fragment: usize) -> Self {
        Self { fragment }
    }

    pub(crate) const fn fragment(self) -> usize {
        self.fragment
    }
}

/// One oriented Section chord whose endpoints are exact Section identities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CertifiedDiskChord {
    key: DiskChordKey,
    endpoints: [usize; 2],
}

impl CertifiedDiskChord {
    pub(crate) const fn new(fragment: usize, endpoints: [usize; 2]) -> Self {
        Self {
            key: DiskChordKey::new(fragment),
            endpoints,
        }
    }

    pub(crate) const fn key(self) -> DiskChordKey {
        self.key
    }

    pub(crate) const fn endpoints(self) -> [usize; 2] {
        self.endpoints
    }
}

/// Exact identity of one ordinary source arc after splitting the cap ring.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct DiskSourceArcKey {
    sense_forward: bool,
    start_endpoint: usize,
    end_endpoint: usize,
}

impl DiskSourceArcKey {
    pub(crate) const fn sense(self) -> Sense {
        if self.sense_forward {
            Sense::Forward
        } else {
            Sense::Reversed
        }
    }

    pub(crate) const fn endpoints(self) -> [usize; 2] {
        [self.start_endpoint, self.end_endpoint]
    }
}

/// Realization lineage for one split source arc.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct DiskSourceArcLineage {
    key: DiskSourceArcKey,
    edge: RawEdgeId,
    fin: RawFinId,
    roots: [DiskBoundaryRootEvidence; 2],
    period_shifts: [i32; 2],
}

impl DiskSourceArcLineage {
    pub(crate) const fn key(self) -> DiskSourceArcKey {
        self.key
    }

    pub(crate) const fn edge(self) -> RawEdgeId {
        self.edge
    }

    pub(crate) const fn fin(self) -> RawFinId {
        self.fin
    }

    pub(crate) const fn roots(self) -> [DiskBoundaryRootEvidence; 2] {
        self.roots
    }

    /// Physical root identities in fin traversal order.
    ///
    /// Together with `edge()` and `key().sense()`, this is the
    /// canonical source-arc vocabulary shared by cap and periodic adapters.
    pub(crate) const fn source_roots(self) -> [DiskRootKey; 2] {
        [self.roots[0].key, self.roots[1].key]
    }

    /// Integer period lifts paired with `roots()` in fin traversal order.
    pub(crate) const fn period_shifts(self) -> [i32; 2] {
        self.period_shifts
    }
}

pub(crate) type DiskFaceArrangement = FaceArrangement<DiskSourceArcKey, DiskChordKey, usize>;

/// Auditable disk-specific conservation facts checked against the generic core.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct DiskArrangementProof {
    roots_conserved: usize,
    source_arcs_conserved: usize,
    opposed_chords: usize,
    cells: usize,
    dual_edges: usize,
    dual_connected: bool,
}

impl DiskArrangementProof {
    pub(crate) const fn roots_conserved(self) -> usize {
        self.roots_conserved
    }

    pub(crate) const fn source_arcs_conserved(self) -> usize {
        self.source_arcs_conserved
    }

    pub(crate) const fn opposed_chords(self) -> usize {
        self.opposed_chords
    }

    pub(crate) const fn cells(self) -> usize {
        self.cells
    }

    pub(crate) const fn dual_edges(self) -> usize {
        self.dual_edges
    }

    pub(crate) const fn dual_connected(self) -> bool {
        self.dual_connected
    }
}

/// Proof-bearing disk cells and materialization lineage.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ArrangedDiskFace {
    arrangement: DiskFaceArrangement,
    source_arcs: Vec<DiskSourceArcLineage>,
    proof: DiskArrangementProof,
}

impl ArrangedDiskFace {
    pub(crate) const fn arrangement(&self) -> &DiskFaceArrangement {
        &self.arrangement
    }

    pub(crate) fn source_arcs(&self) -> &[DiskSourceArcLineage] {
        &self.source_arcs
    }

    pub(crate) const fn proof(&self) -> DiskArrangementProof {
        self.proof
    }
}

/// Fail-closed refusals while adapting disk evidence to the arrangement core.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum DiskArrangementError {
    PartialBoundaryEvidence,
    BoundaryRootsRequired,
    InvalidRootScalar(DiskRootKey),
    TangentialRoot(DiskRootKey),
    IndeterminateRoot(DiskRootKey),
    DuplicateRootEndpoint(usize),
    DuplicateCircularOrdinal(usize),
    NonContiguousCircularOrdinals {
        expected: usize,
        actual: usize,
    },
    IncompatibleIntrinsicRootOrder {
        previous: DiskRootKey,
        next: DiskRootKey,
    },
    DuplicateChord(DiskChordKey),
    UnknownChordEndpoint {
        chord: DiskChordKey,
        endpoint: usize,
    },
    DegenerateChord(DiskChordKey),
    UnpairedRoot(usize),
    BranchedRoot(usize),
    CrossingChords {
        first: DiskChordKey,
        second: DiskChordKey,
    },
    ConservationMismatch,
    Arrangement(FaceArrangementError<DiskSourceArcKey, DiskChordKey, usize>),
}

/// Typed refusals while binding a public Section graph to one source cap.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum SectionDiskArrangementError {
    InvalidOperand(usize),
    IncompleteSectionGraph,
    CapPartMismatch,
    CapOutsideOperand,
    MissingCapTopology,
    UnsupportedCapSurface,
    UnsupportedCapBoundary,
    WholeFinIncidenceRequired,
    MissingCapChord,
    UnknownBranch { fragment: usize, branch: usize },
    UnsupportedCapFragment(usize),
    FragmentComponentMismatch(usize),
    InconsistentGraphTolerance,
    MissingEndpoint { fragment: usize, endpoint: usize },
    EndpointProvenanceMismatch { fragment: usize, endpoint: usize },
    IncompatibleRootEnclosures { previous: usize, next: usize },
    Arrangement(DiskArrangementError),
}

#[derive(Debug, Clone, Copy)]
struct DiskCapTopology {
    loop_id: RawLoopId,
    fin: RawFinId,
    edge: RawEdgeId,
    sense: Sense,
}

#[derive(Debug, Clone, Copy)]
struct UnorderedSectionRoot {
    endpoint: usize,
    source_root_ordinal: usize,
    parameter: f64,
    enclosure: [f64; 2],
}

/// Read-only adaptation of one complete Section graph into one cut cap disk.
///
/// Every branch carried by `cap` must be one bounded line fragment with two
/// exact roots on the cap's sole whole-circle fin. Intrinsic root enclosures
/// establish the global circular order; model points and chord carrier
/// parameters never participate in identity or ordering.
pub(crate) fn arrange_section_disk_face(
    store: &Store,
    graph: &BodySectionGraph,
    cap: &FaceId,
    operand: usize,
) -> Result<ArrangedDiskFace, SectionDiskArrangementError> {
    if operand >= graph.bodies().len() {
        return Err(SectionDiskArrangementError::InvalidOperand(operand));
    }
    if graph.completion() != SectionCompletion::Complete || !graph.gaps().is_empty() {
        return Err(SectionDiskArrangementError::IncompleteSectionGraph);
    }
    if graph.bodies()[operand].part() != cap.part() {
        return Err(SectionDiskArrangementError::CapPartMismatch);
    }
    if !store
        .faces_of_body(graph.bodies()[operand].raw())
        .map_err(|_| SectionDiskArrangementError::CapOutsideOperand)?
        .contains(&cap.raw())
    {
        return Err(SectionDiskArrangementError::CapOutsideOperand);
    }

    let topology = disk_cap_topology(store, cap.raw())?;
    let (mut roots, chords, tolerance) = collect_section_cap_chords(graph, cap, operand, topology)?;
    if certify_whole_fin_incidence(store, cap.raw(), topology.loop_id, topology.fin, tolerance)
        != WholeFinIncidence::Certified
    {
        return Err(SectionDiskArrangementError::WholeFinIncidenceRequired);
    }

    // Sorting only proposes an interval order. The strict separation check
    // below is the proof that the proposal is a complete intrinsic order;
    // no point or scalar representative is promoted to ordering authority.
    roots.sort_by(|left, right| {
        left.enclosure[0]
            .total_cmp(&right.enclosure[0])
            .then(left.enclosure[1].total_cmp(&right.enclosure[1]))
            .then(left.endpoint.cmp(&right.endpoint))
    });
    for pair in roots.windows(2) {
        if pair[0].enclosure[1] >= pair[1].enclosure[0] {
            return Err(SectionDiskArrangementError::IncompatibleRootEnclosures {
                previous: pair[0].endpoint,
                next: pair[1].endpoint,
            });
        }
    }
    let roots = roots
        .into_iter()
        .enumerate()
        .map(|(circular_ordinal, root)| {
            DiskBoundaryRootEvidence::transverse(
                DiskRootKey::new(root.endpoint, circular_ordinal, root.source_root_ordinal),
                root.parameter,
                root.enclosure,
            )
        })
        .collect();
    arrange_disk_face(
        CertifiedDiskBoundary::new(
            topology.edge,
            topology.fin,
            topology.sense,
            DiskBoundaryCoverage::Complete,
            roots,
        ),
        chords,
    )
    .map_err(SectionDiskArrangementError::Arrangement)
}

fn disk_cap_topology(
    store: &Store,
    cap: ktopo::entity::FaceId,
) -> Result<DiskCapTopology, SectionDiskArrangementError> {
    let face = store
        .get(cap)
        .map_err(|_| SectionDiskArrangementError::MissingCapTopology)?;
    if !matches!(
        store
            .surface(face.surface())
            .map_err(|_| SectionDiskArrangementError::MissingCapTopology)?,
        SurfaceGeom::Plane(_)
    ) {
        return Err(SectionDiskArrangementError::UnsupportedCapSurface);
    }
    let [loop_id] = face.loops() else {
        return Err(SectionDiskArrangementError::UnsupportedCapBoundary);
    };
    let loop_ = store
        .get(*loop_id)
        .map_err(|_| SectionDiskArrangementError::MissingCapTopology)?;
    let [fin_id] = loop_.fins() else {
        return Err(SectionDiskArrangementError::UnsupportedCapBoundary);
    };
    let fin = store
        .get(*fin_id)
        .map_err(|_| SectionDiskArrangementError::MissingCapTopology)?;
    let edge = store
        .get(fin.edge())
        .map_err(|_| SectionDiskArrangementError::MissingCapTopology)?;
    let Some(curve) = edge.curve() else {
        return Err(SectionDiskArrangementError::UnsupportedCapBoundary);
    };
    if loop_.face() != cap
        || fin.parent() != *loop_id
        || edge.vertices() != [None, None]
        || edge.bounds().is_some()
        || edge.tolerance().is_some()
        || !edge.fins().contains(fin_id)
        || !matches!(
            store
                .curve(curve)
                .map_err(|_| SectionDiskArrangementError::MissingCapTopology)?,
            CurveGeom::Circle(_)
        )
    {
        return Err(SectionDiskArrangementError::UnsupportedCapBoundary);
    }
    Ok(DiskCapTopology {
        loop_id: *loop_id,
        fin: *fin_id,
        edge: fin.edge(),
        sense: fin.sense(),
    })
}

fn collect_section_cap_chords(
    graph: &BodySectionGraph,
    cap: &FaceId,
    operand: usize,
    topology: DiskCapTopology,
) -> Result<(Vec<UnorderedSectionRoot>, Vec<CertifiedDiskChord>, f64), SectionDiskArrangementError>
{
    let mut roots = Vec::new();
    let mut chords = Vec::new();
    let mut tolerance_bits = None;
    for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
        let branch = graph.branches().get(fragment.branch()).ok_or(
            SectionDiskArrangementError::UnknownBranch {
                fragment: fragment_index,
                branch: fragment.branch(),
            },
        )?;
        if branch.faces()[operand] != *cap {
            continue;
        }
        let SectionCurveFragmentSpan::LineSegment { endpoints } = fragment.span() else {
            return Err(SectionDiskArrangementError::UnsupportedCapFragment(
                fragment_index,
            ));
        };
        if branch.topology() != SectionBranchTopology::Open
            || !matches!(branch.carrier(), SectionCarrier::Line { .. })
            || !matches!(branch.pcurves()[operand], SectionUvCurve::Line(_))
        {
            return Err(SectionDiskArrangementError::UnsupportedCapFragment(
                fragment_index,
            ));
        }
        let mut component_uses = 0usize;
        for component in graph.curve_components() {
            let uses = component
                .fragments()
                .iter()
                .filter(|&&candidate| candidate == fragment_index)
                .count();
            if uses != 0 && !component.closed() {
                return Err(SectionDiskArrangementError::FragmentComponentMismatch(
                    fragment_index,
                ));
            }
            component_uses += uses;
        }
        if component_uses != 1 {
            return Err(SectionDiskArrangementError::FragmentComponentMismatch(
                fragment_index,
            ));
        }
        let tolerance = branch.evidence().tolerance();
        if !tolerance.is_finite() || tolerance < 0.0 {
            return Err(SectionDiskArrangementError::InconsistentGraphTolerance);
        }
        match tolerance_bits {
            None => tolerance_bits = Some(tolerance.to_bits()),
            Some(bits) if bits == tolerance.to_bits() => {}
            Some(_) => return Err(SectionDiskArrangementError::InconsistentGraphTolerance),
        }
        let bound = endpoints
            .iter()
            .map(|end| {
                bind_section_cap_root(
                    graph,
                    end,
                    fragment_index,
                    cap,
                    &branch.faces()[1 - operand],
                    operand,
                    topology,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let [start, end]: [UnorderedSectionRoot; 2] = bound
            .try_into()
            .map_err(|_| SectionDiskArrangementError::UnsupportedCapFragment(fragment_index))?;
        roots.extend([start, end]);
        chords.push(CertifiedDiskChord::new(
            fragment_index,
            [start.endpoint, end.endpoint],
        ));
    }
    let tolerance = tolerance_bits
        .map(f64::from_bits)
        .ok_or(SectionDiskArrangementError::MissingCapChord)?;
    Ok((roots, chords, tolerance))
}

fn bind_section_cap_root(
    graph: &BodySectionGraph,
    end: &SectionRulingFragmentEnd,
    fragment: usize,
    cap: &FaceId,
    opposing_face: &FaceId,
    operand: usize,
    topology: DiskCapTopology,
) -> Result<UnorderedSectionRoot, SectionDiskArrangementError> {
    let endpoint_index = end.endpoint();
    let endpoint = graph.curve_endpoints().get(endpoint_index).ok_or(
        SectionDiskArrangementError::MissingEndpoint {
            fragment,
            endpoint: endpoint_index,
        },
    )?;
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = endpoint.topology()
    else {
        return Err(root_mismatch(fragment, endpoint_index));
    };
    let Some(source) = source_parameters[operand].as_ref() else {
        return Err(root_mismatch(fragment, endpoint_index));
    };
    let Some(trim) = end.trims()[operand].as_ref() else {
        return Err(root_mismatch(fragment, endpoint_index));
    };
    let Some(common) = endpoint.edge_parameters()[operand] else {
        return Err(root_mismatch(fragment, endpoint_index));
    };
    let other = 1 - operand;
    let enclosure = source.root_parameter_enclosure();
    let observed = trim.edge_parameter();
    let same_materialization = source == trim.source_parameter()
        && source.root_parameter().to_bits() == trim.source_parameter().root_parameter().to_bits()
        && enclosure.lo().to_bits()
            == trim
                .source_parameter()
                .root_parameter_enclosure()
                .lo()
                .to_bits()
        && enclosure.hi().to_bits()
            == trim
                .source_parameter()
                .root_parameter_enclosure()
                .hi()
                .to_bits();
    if !matches!(&sites[operand], SectionSite::EdgeInterior(edge) if edge.raw() == topology.edge)
        || !matches!(&sites[other], SectionSite::FaceInterior(face) if face == opposing_face)
        || source_parameters[other].is_some()
        || endpoint.edge_parameters()[other].is_some()
        || end.trims()[other].is_some()
        || source.edge().raw() != topology.edge
        || trim.operand() != operand
        || trim.face() != cap.clone()
        || trim.loop_id().raw() != topology.loop_id
        || trim.fin().raw() != topology.fin
        || trim.source_parameter().edge().raw() != topology.edge
        || !same_materialization
        || !common.contains(source.root_parameter())
        || observed.lo() > common.lo()
        || common.hi() > observed.hi()
    {
        return Err(root_mismatch(fragment, endpoint_index));
    }
    Ok(UnorderedSectionRoot {
        endpoint: endpoint_index,
        source_root_ordinal: source.root_ordinal(),
        parameter: source.root_parameter(),
        enclosure: [enclosure.lo(), enclosure.hi()],
    })
}

const fn root_mismatch(fragment: usize, endpoint: usize) -> SectionDiskArrangementError {
    SectionDiskArrangementError::EndpointProvenanceMismatch { fragment, endpoint }
}

/// Arrange any certified noncrossing chord set on one circular disk.
pub(crate) fn arrange_disk_face(
    boundary: CertifiedDiskBoundary,
    chords: impl IntoIterator<Item = CertifiedDiskChord>,
) -> Result<ArrangedDiskFace, DiskArrangementError> {
    if boundary.coverage != DiskBoundaryCoverage::Complete {
        return Err(DiskArrangementError::PartialBoundaryEvidence);
    }
    if boundary.roots.len() < 2 {
        return Err(DiskArrangementError::BoundaryRootsRequired);
    }

    let mut roots = boundary.roots;
    validate_roots(&roots)?;
    roots.sort_by_key(|root| root.key.circular_ordinal);
    for (expected, root) in roots.iter().enumerate() {
        let actual = root.key.circular_ordinal;
        if actual != expected {
            return Err(DiskArrangementError::NonContiguousCircularOrdinals { expected, actual });
        }
    }
    for pair in roots.windows(2) {
        if pair[0].root_enclosure()[1] >= pair[1].root_enclosure()[0] {
            return Err(DiskArrangementError::IncompatibleIntrinsicRootOrder {
                previous: pair[0].key,
                next: pair[1].key,
            });
        }
    }

    let mut chords = chords.into_iter().collect::<Vec<_>>();
    chords.sort_by_key(|chord| chord.key);
    validate_chords(&roots, &chords)?;

    let source_arcs = build_source_arcs(boundary.edge, boundary.fin, boundary.sense, &roots);
    let source_spans = source_arcs
        .iter()
        .map(|arc| {
            let endpoints = arc.key.endpoints();
            DirectedSourceSpan::new(arc.key, endpoints[0], endpoints[1])
        })
        .collect::<Vec<_>>();
    let cut_fragments = chords
        .iter()
        .map(|chord| DirectedCutFragment::new(chord.key, chord.endpoints[0], chord.endpoints[1]))
        .collect::<Vec<_>>();
    let rotations = build_rotations(&source_arcs, &chords);
    let arrangement = arrange_bounded_face(FaceArrangementInput::new(
        source_spans,
        cut_fragments,
        rotations,
    ))
    .map_err(DiskArrangementError::Arrangement)?;

    let expected_cells = chords
        .len()
        .checked_add(1)
        .ok_or(DiskArrangementError::ConservationMismatch)?;
    let core = arrangement.proof();
    if core.source_spans_conserved() != roots.len()
        || core.opposed_cut_pairs() != chords.len()
        || arrangement.cells().len() != expected_cells
        || arrangement.adjacency().len() != chords.len()
        || !core.dual_connected()
    {
        return Err(DiskArrangementError::ConservationMismatch);
    }
    let proof = DiskArrangementProof {
        roots_conserved: roots.len(),
        source_arcs_conserved: core.source_spans_conserved(),
        opposed_chords: core.opposed_cut_pairs(),
        cells: arrangement.cells().len(),
        dual_edges: arrangement.adjacency().len(),
        dual_connected: core.dual_connected(),
    };
    Ok(ArrangedDiskFace {
        arrangement,
        source_arcs,
        proof,
    })
}

fn validate_roots(roots: &[DiskBoundaryRootEvidence]) -> Result<(), DiskArrangementError> {
    let mut endpoints = BTreeSet::new();
    let mut circular_ordinals = BTreeSet::new();
    for root in roots {
        match root.contact {
            DiskRootContact::Transverse => {}
            DiskRootContact::Tangent => {
                return Err(DiskArrangementError::TangentialRoot(root.key));
            }
            DiskRootContact::Indeterminate => {
                return Err(DiskArrangementError::IndeterminateRoot(root.key));
            }
        }
        let parameter = root.root_parameter();
        let [lo, hi] = root.root_enclosure();
        if !parameter.is_finite()
            || !lo.is_finite()
            || !hi.is_finite()
            || lo > parameter
            || parameter > hi
        {
            return Err(DiskArrangementError::InvalidRootScalar(root.key));
        }
        if !endpoints.insert(root.key.endpoint) {
            return Err(DiskArrangementError::DuplicateRootEndpoint(
                root.key.endpoint,
            ));
        }
        if !circular_ordinals.insert(root.key.circular_ordinal) {
            return Err(DiskArrangementError::DuplicateCircularOrdinal(
                root.key.circular_ordinal,
            ));
        }
    }
    Ok(())
}

fn validate_chords(
    roots: &[DiskBoundaryRootEvidence],
    chords: &[CertifiedDiskChord],
) -> Result<(), DiskArrangementError> {
    let order = roots
        .iter()
        .map(|root| (root.key.endpoint, root.key.circular_ordinal))
        .collect::<BTreeMap<_, _>>();
    let mut chord_keys = BTreeSet::new();
    let mut incidence = BTreeMap::<usize, usize>::new();
    for chord in chords {
        if !chord_keys.insert(chord.key) {
            return Err(DiskArrangementError::DuplicateChord(chord.key));
        }
        if chord.endpoints[0] == chord.endpoints[1] {
            return Err(DiskArrangementError::DegenerateChord(chord.key));
        }
        for endpoint in chord.endpoints {
            if !order.contains_key(&endpoint) {
                return Err(DiskArrangementError::UnknownChordEndpoint {
                    chord: chord.key,
                    endpoint,
                });
            }
            let degree = incidence.entry(endpoint).or_default();
            *degree += 1;
            if *degree > 1 {
                return Err(DiskArrangementError::BranchedRoot(endpoint));
            }
        }
    }
    if let Some(root) = roots
        .iter()
        .find(|root| incidence.get(&root.key.endpoint).copied() != Some(1))
    {
        return Err(DiskArrangementError::UnpairedRoot(root.key.endpoint));
    }
    for (left_index, left) in chords.iter().enumerate() {
        for right in &chords[(left_index + 1)..] {
            if chords_cross(*left, *right, &order) {
                return Err(DiskArrangementError::CrossingChords {
                    first: left.key,
                    second: right.key,
                });
            }
        }
    }
    Ok(())
}

fn chords_cross(
    left: CertifiedDiskChord,
    right: CertifiedDiskChord,
    order: &BTreeMap<usize, usize>,
) -> bool {
    let mut left = left.endpoints.map(|endpoint| order[&endpoint]);
    let mut right = right.endpoints.map(|endpoint| order[&endpoint]);
    left.sort_unstable();
    right.sort_unstable();
    (left[0] < right[0] && right[0] < left[1] && left[1] < right[1])
        || (right[0] < left[0] && left[0] < right[1] && right[1] < left[1])
}

fn build_source_arcs(
    edge: RawEdgeId,
    fin: RawFinId,
    sense: Sense,
    roots: &[DiskBoundaryRootEvidence],
) -> Vec<DiskSourceArcLineage> {
    let traversal = match sense {
        Sense::Forward => roots.iter().copied().collect::<Vec<_>>(),
        Sense::Reversed => roots.iter().rev().copied().collect::<Vec<_>>(),
    };
    (0..traversal.len())
        .map(|index| {
            let start = traversal[index];
            let end = traversal[(index + 1) % traversal.len()];
            let wraps = index + 1 == traversal.len();
            let period_shifts = match (sense, wraps) {
                (Sense::Forward, true) => [0, 1],
                (Sense::Reversed, true) => [1, 0],
                _ => [0, 0],
            };
            DiskSourceArcLineage {
                key: DiskSourceArcKey {
                    sense_forward: sense == Sense::Forward,
                    start_endpoint: start.key.endpoint,
                    end_endpoint: end.key.endpoint,
                },
                edge,
                fin,
                roots: [start, end],
                period_shifts,
            }
        })
        .collect()
}

fn build_rotations(
    arcs: &[DiskSourceArcLineage],
    chords: &[CertifiedDiskChord],
) -> Vec<CertifiedEndpointRotation<DiskSourceArcKey, DiskChordKey, usize>> {
    let outgoing = chords
        .iter()
        .flat_map(|chord| {
            [
                (
                    chord.endpoints[0],
                    ArrangementDartKey::cut(chord.key, ArrangementDirection::Forward),
                ),
                (
                    chord.endpoints[1],
                    ArrangementDartKey::cut(chord.key, ArrangementDirection::Reverse),
                ),
            ]
        })
        .collect::<BTreeMap<_, _>>();
    arcs.iter()
        .enumerate()
        .map(|(index, arc)| {
            let endpoint = arc.key.start_endpoint;
            let previous = arcs[(index + arcs.len() - 1) % arcs.len()].key;
            CertifiedEndpointRotation::new(
                endpoint,
                vec![
                    ArrangementDartKey::source(arc.key, ArrangementDirection::Forward),
                    outgoing[&endpoint].clone(),
                    ArrangementDartKey::source(previous, ArrangementDirection::Reverse),
                ],
            )
        })
        .collect()
}

/// Constant open-set relation of one cap cell to the other body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DiskCellClassification {
    Interior,
    Exterior,
}

impl DiskCellClassification {
    const fn toggled(self) -> Self {
        match self {
            Self::Interior => Self::Exterior,
            Self::Exterior => Self::Interior,
        }
    }
}

/// Proof that every disk cell was classified through exact dual adjacency.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ClassifiedDiskFace {
    classes: BTreeMap<usize, DiskCellClassification>,
    anchor_arc: DiskSourceArcKey,
    anchor_cell: usize,
    dual_edges_checked: usize,
}

impl ClassifiedDiskFace {
    pub(crate) const fn classes(&self) -> &BTreeMap<usize, DiskCellClassification> {
        &self.classes
    }

    pub(crate) const fn anchor_arc(&self) -> DiskSourceArcKey {
        self.anchor_arc
    }

    pub(crate) const fn anchor_cell(&self) -> usize {
        self.anchor_cell
    }

    pub(crate) const fn dual_edges_checked(&self) -> usize {
        self.dual_edges_checked
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum DiskClassificationError {
    UnknownAnchorArc(DiskSourceArcKey),
    ContradictoryDual(DiskChordKey),
    DisconnectedDual,
}

/// Classify every disk cell from one exact source-arc anchor.
///
/// The caller may sample strictly inside the already-certified anchor arc to
/// obtain `anchor_classification`; that sample never chooses graph identity.
pub(crate) fn classify_disk_face_from_anchor(
    disk: &ArrangedDiskFace,
    anchor_arc: DiskSourceArcKey,
    anchor_classification: DiskCellClassification,
) -> Result<ClassifiedDiskFace, DiskClassificationError> {
    let anchor_cell = disk
        .arrangement
        .cells()
        .iter()
        .find(|cell| {
            cell.boundary().uses().iter().any(|use_| {
                matches!(
                    use_.edge(),
                    ArrangementEdgeKey::Source(key) if *key == anchor_arc
                ) && use_.direction() == ArrangementDirection::Forward
            })
        })
        .map(|cell| cell.key())
        .ok_or(DiskClassificationError::UnknownAnchorArc(anchor_arc))?;
    let mut classes = BTreeMap::from([(anchor_cell, anchor_classification)]);
    loop {
        let before = classes.len();
        for adjacency in disk.arrangement.adjacency() {
            propagate_classification(&mut classes, adjacency)?;
        }
        if classes.len() == before {
            break;
        }
    }
    if classes.len() != disk.arrangement.cells().len() {
        return Err(DiskClassificationError::DisconnectedDual);
    }
    Ok(ClassifiedDiskFace {
        classes,
        anchor_arc,
        anchor_cell,
        dual_edges_checked: disk.arrangement.adjacency().len(),
    })
}

fn propagate_classification(
    classes: &mut BTreeMap<usize, DiskCellClassification>,
    adjacency: &ArrangementCutAdjacency<DiskChordKey>,
) -> Result<(), DiskClassificationError> {
    let forward = adjacency.forward_cell();
    let reverse = adjacency.reverse_cell();
    match (
        classes.get(&forward).copied(),
        classes.get(&reverse).copied(),
    ) {
        (Some(left), Some(right)) if left == right => {
            Err(DiskClassificationError::ContradictoryDual(*adjacency.cut()))
        }
        (Some(value), None) => {
            classes.insert(reverse, value.toggled());
            Ok(())
        }
        (None, Some(value)) => {
            classes.insert(forward, value.toggled());
            Ok(())
        }
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::frame::Frame;
    use kgeom::vec::Point3;
    use ktopo::store::Store;

    use crate::{
        BlockRequest, BodyId, CylinderRequest, Kernel, PartId, SectionBodiesRequest, Session,
    };

    fn topology_ids() -> (RawEdgeId, RawFinId) {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        let loop_id = store.get(face).unwrap().loops()[0];
        let fin = store.get(loop_id).unwrap().fins()[0];
        (store.get(fin).unwrap().edge(), fin)
    }

    fn boundary(endpoint_count: usize, coverage: DiskBoundaryCoverage) -> CertifiedDiskBoundary {
        let (edge, fin) = topology_ids();
        CertifiedDiskBoundary::new(
            edge,
            fin,
            Sense::Forward,
            coverage,
            (0..endpoint_count)
                .map(|ordinal| {
                    DiskBoundaryRootEvidence::transverse(
                        DiskRootKey::new(100 + ordinal, ordinal, ordinal % 2),
                        ordinal as f64,
                        [ordinal as f64 - 0.125, ordinal as f64 + 0.125],
                    )
                })
                .collect(),
        )
    }

    fn chord(fragment: usize, start: usize, end: usize) -> CertifiedDiskChord {
        CertifiedDiskChord::new(fragment, [100 + start, 100 + end])
    }

    fn source_signature(
        session: &Session,
        part_id: &PartId,
        bodies: &[BodyId; 2],
    ) -> ([[usize; 3]; 2], usize) {
        let part = session.part(part_id.clone()).unwrap();
        let topology = bodies.each_ref().map(|body| {
            let body = part.body(body.clone()).unwrap();
            [
                body.faces().unwrap().len(),
                body.edges().unwrap().len(),
                body.vertices().unwrap().len(),
            ]
        });
        (topology, part.bodies().len())
    }

    #[test]
    fn chord_counts_prove_cell_conservation_and_dual_classification() {
        let cases = [
            (vec![chord(7, 0, 1)], 2, 1),
            (vec![chord(8, 0, 3), chord(7, 1, 2)], 3, 2),
            (vec![chord(9, 0, 5), chord(8, 1, 4), chord(7, 2, 3)], 4, 3),
        ];
        for (chords, expected_cells, expected_chords) in cases {
            let endpoint_count = expected_chords * 2;
            let disk = arrange_disk_face(
                boundary(endpoint_count, DiskBoundaryCoverage::Complete),
                chords,
            )
            .expect("nested noncrossing chords partition a disk");
            assert_eq!(disk.source_arcs().len(), endpoint_count);
            assert_eq!(disk.proof().roots_conserved(), endpoint_count);
            assert_eq!(disk.proof().source_arcs_conserved(), endpoint_count);
            assert_eq!(disk.proof().opposed_chords(), expected_chords);
            assert_eq!(disk.proof().cells(), expected_cells);
            assert_eq!(disk.proof().dual_edges(), expected_chords);
            assert!(disk.proof().dual_connected());

            let anchor = disk.source_arcs()[0].key();
            let classified =
                classify_disk_face_from_anchor(&disk, anchor, DiskCellClassification::Exterior)
                    .expect("the exact connected dual classifies every cell");
            assert_eq!(classified.classes().len(), expected_cells);
            assert_eq!(classified.anchor_arc(), anchor);
            assert_eq!(classified.dual_edges_checked(), expected_chords);
            assert_eq!(
                classified.classes()[&classified.anchor_cell()],
                DiskCellClassification::Exterior
            );
            for adjacency in disk.arrangement().adjacency() {
                assert_ne!(
                    classified.classes()[&adjacency.forward_cell()],
                    classified.classes()[&adjacency.reverse_cell()]
                );
            }
        }
    }

    #[test]
    fn input_permutations_preserve_exact_arrangement_and_lineage() {
        let first = arrange_disk_face(
            boundary(4, DiskBoundaryCoverage::Complete),
            [chord(12, 0, 3), chord(11, 1, 2)],
        )
        .unwrap();
        let mut permuted_boundary = boundary(4, DiskBoundaryCoverage::Complete);
        permuted_boundary.roots.reverse();
        let second =
            arrange_disk_face(permuted_boundary, [chord(11, 1, 2), chord(12, 0, 3)]).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn partial_tangent_branched_and_crossing_evidence_fail_closed() {
        assert_eq!(
            arrange_disk_face(boundary(2, DiskBoundaryCoverage::Partial), [chord(1, 0, 1)]),
            Err(DiskArrangementError::PartialBoundaryEvidence)
        );

        let mut tangent = boundary(2, DiskBoundaryCoverage::Complete);
        let key = tangent.roots[0].key();
        tangent.roots[0] = DiskBoundaryRootEvidence::with_contact(
            key,
            0.0,
            [-0.125, 0.125],
            DiskRootContact::Tangent,
        );
        assert_eq!(
            arrange_disk_face(tangent, [chord(1, 0, 1)]),
            Err(DiskArrangementError::TangentialRoot(key))
        );

        assert_eq!(
            arrange_disk_face(
                boundary(4, DiskBoundaryCoverage::Complete),
                [chord(1, 0, 1), chord(2, 0, 2)]
            ),
            Err(DiskArrangementError::BranchedRoot(100))
        );

        assert_eq!(
            arrange_disk_face(
                boundary(4, DiskBoundaryCoverage::Complete),
                [chord(1, 0, 2), chord(2, 1, 3)]
            ),
            Err(DiskArrangementError::CrossingChords {
                first: DiskChordKey::new(1),
                second: DiskChordKey::new(2),
            })
        );
    }

    #[test]
    fn missing_and_malformed_root_proofs_are_rejected() {
        assert_eq!(
            arrange_disk_face(boundary(2, DiskBoundaryCoverage::Complete), []),
            Err(DiskArrangementError::UnpairedRoot(100))
        );

        let mut gap = boundary(2, DiskBoundaryCoverage::Complete);
        gap.roots[1].key.circular_ordinal = 2;
        assert_eq!(
            arrange_disk_face(gap, [chord(1, 0, 1)]),
            Err(DiskArrangementError::NonContiguousCircularOrdinals {
                expected: 1,
                actual: 2,
            })
        );

        assert_eq!(
            arrange_disk_face(
                boundary(2, DiskBoundaryCoverage::Complete),
                [CertifiedDiskChord::new(1, [100, 999])]
            ),
            Err(DiskArrangementError::UnknownChordEndpoint {
                chord: DiskChordKey::new(1),
                endpoint: 999,
            })
        );

        let mut incompatible = boundary(2, DiskBoundaryCoverage::Complete);
        incompatible.roots[0].key.circular_ordinal = 1;
        incompatible.roots[1].key.circular_ordinal = 0;
        let previous = incompatible.roots[1].key;
        let next = incompatible.roots[0].key;
        assert_eq!(
            arrange_disk_face(incompatible, [chord(1, 0, 1)]),
            Err(DiskArrangementError::IncompatibleIntrinsicRootOrder { previous, next })
        );

        let mut touching = boundary(2, DiskBoundaryCoverage::Complete);
        let previous = touching.roots[0].key;
        let next = touching.roots[1].key;
        touching.roots[1] = DiskBoundaryRootEvidence::transverse(next, 0.25, [0.125, 0.5]);
        assert_eq!(
            arrange_disk_face(touching, [chord(1, 0, 1)]),
            Err(DiskArrangementError::IncompatibleIntrinsicRootOrder { previous, next })
        );
    }

    #[test]
    fn source_arc_lineage_retains_cap_and_both_root_materializations() {
        let disk = arrange_disk_face(
            boundary(2, DiskBoundaryCoverage::Complete),
            [chord(41, 0, 1)],
        )
        .unwrap();
        assert_eq!(disk.arrangement().cut_fragments()[0].key().fragment(), 41);
        for arc in disk.source_arcs() {
            assert_eq!(arc.edge(), disk.source_arcs()[0].edge());
            assert_eq!(arc.fin(), disk.source_arcs()[0].fin());
            assert_eq!(
                arc.key().endpoints(),
                arc.roots().map(|root| root.key().endpoint())
            );
            for root in arc.roots() {
                assert!(root.root_enclosure()[0] <= root.root_parameter());
                assert!(root.root_parameter() <= root.root_enclosure()[1]);
            }
        }
        assert_eq!(disk.source_arcs()[0].period_shifts(), [0, 0]);
        assert_eq!(disk.source_arcs()[1].period_shifts(), [0, 1]);
    }

    #[test]
    fn reversed_fin_uses_domain_on_left_traversal_and_intrinsic_seam_lift() {
        let mut source = boundary(4, DiskBoundaryCoverage::Complete);
        source.sense = Sense::Reversed;
        let disk = arrange_disk_face(source, [chord(1, 0, 3), chord(2, 1, 2)]).unwrap();
        assert_eq!(
            disk.source_arcs()
                .iter()
                .map(|arc| arc.key().endpoints())
                .collect::<Vec<_>>(),
            vec![[103, 102], [102, 101], [101, 100], [100, 103]]
        );
        assert!(
            disk.source_arcs()
                .iter()
                .all(|arc| arc.key().sense() == Sense::Reversed)
        );
        assert_eq!(disk.source_arcs()[0].period_shifts(), [0, 0]);
        assert_eq!(disk.source_arcs()[3].period_shifts(), [1, 0]);
        let intrinsic_wrap = disk.source_arcs()[3].period_shifts();
        assert_eq!([intrinsic_wrap[1], intrinsic_wrap[0]], [0, 1]);
    }

    #[test]
    fn section_adapter_arranges_offset_cap_chords_in_both_operand_orders_read_only() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(
                    Frame::world().with_origin(Point3::new(1.5, 0.0, 1.0)),
                    [2.0, 6.0, 4.0],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(Frame::world(), 1.5, 2.0))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (block, cylinder)
        };
        let sources = [block.clone(), cylinder.clone()];
        let before = source_signature(&session, &part_id, &sources);

        for (bodies, cylinder_operand) in [
            ([block.clone(), cylinder.clone()], 1usize),
            ([cylinder.clone(), block.clone()], 0usize),
        ] {
            let graph = session
                .part(part_id.clone())
                .unwrap()
                .section_bodies(SectionBodiesRequest::new(
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            let part = session.part(part_id.clone()).unwrap();
            let store = &part.state.store;
            let caps = store
                .faces_of_body(cylinder.raw())
                .unwrap()
                .into_iter()
                .filter(|face| {
                    store.get(*face).ok().is_some_and(|face| {
                        matches!(store.surface(face.surface()), Ok(SurfaceGeom::Plane(_)))
                    })
                })
                .collect::<Vec<_>>();
            assert_eq!(caps.len(), 2);
            for raw_cap in caps {
                let cap = FaceId::new(part_id.clone(), raw_cap);
                let topology = disk_cap_topology(store, raw_cap).unwrap();
                let disk = arrange_section_disk_face(store, &graph, &cap, cylinder_operand)
                    .expect("complete cap chord evidence must arrange");
                assert_eq!(disk.arrangement().cells().len(), 2);
                assert_eq!(disk.arrangement().cut_fragments().len(), 1);
                assert_eq!(disk.arrangement().adjacency().len(), 1);
                assert_eq!(disk.source_arcs().len(), 2);
                let fragment = disk.arrangement().cut_fragments()[0].key().fragment();
                assert_eq!(
                    graph.branches()[graph.curve_fragments()[fragment].branch()].faces()
                        [cylinder_operand],
                    cap
                );
                for arc in disk.source_arcs() {
                    assert_eq!(arc.edge(), topology.edge);
                    assert_eq!(arc.fin(), topology.fin);
                    assert_eq!(arc.key().sense(), topology.sense);
                    for root in arc.roots() {
                        let SectionCurveEndpointTopology::Trim {
                            source_parameters, ..
                        } = graph.curve_endpoints()[root.key().endpoint()].topology()
                        else {
                            panic!("cap root lost trim topology")
                        };
                        let source = source_parameters[cylinder_operand].as_ref().unwrap();
                        assert_eq!(root.key().source_root_ordinal(), source.root_ordinal());
                        assert_eq!(
                            root.root_parameter().to_bits(),
                            source.root_parameter().to_bits()
                        );
                    }
                }
            }
        }
        assert_eq!(source_signature(&session, &part_id, &sources), before);
    }
}
