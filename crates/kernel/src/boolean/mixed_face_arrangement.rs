//! Section-to-arrangement admission for one bounded planar source face.
//!
//! This adapter deliberately consumes exact section endpoint identity and
//! topology-owned source-loop order.  Metric endpoint representatives never
//! join or sort the graph.  Root intervals are used only to check that the
//! section publisher's exact root ordinals remain compatible with intrinsic
//! edge order.

use std::collections::{BTreeMap, BTreeSet};

use kcore::predicates::{Orientation, affine_dot3, orient2d, polygon_orientation2d};
use kgeom::curve2d::Curve2d;
use ktopo::entity::{
    EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId, LoopId as RawLoopId, Sense,
    VertexId as RawVertexId,
};
use ktopo::geom::{Curve2dGeom, SurfaceGeom};
use ktopo::store::Store;

use super::face_arrangement::{
    ArrangementCycle, ArrangementDartKey, ArrangementDirection, CertifiedCellTopology,
    CertifiedCycleAssignment, CertifiedCycleSide, CertifiedEndpointRotation,
    CertifiedSurfaceEmbedding, DirectedCutFragment, DirectedSourceSpan, FaceArrangement,
    FaceArrangementError, FaceArrangementInput, SurfaceArrangementError, SurfaceFaceArrangement,
    arrange_bounded_face, arrange_bounded_surface,
};
use crate::section::{
    BodySectionGraph, SectionCompletion, SectionCurveEndpointTopology, SectionCurveFragment,
    SectionCurveFragmentSpan, SectionEdgeParameterInterval, SectionSite, SectionUvCurve,
};
use crate::{FaceId, FinId, LoopId};

/// Exact identity of one source-boundary span after root splitting.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct MixedSourceSpanKey {
    pub(crate) fin_loop_ordinal: usize,
    pub(crate) traversal_ordinal: usize,
}

/// Stable section-owned identity of one bounded cut fragment.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct MixedCutFragmentKey {
    branch: usize,
    source_ordinal: usize,
}

/// Exact endpoint vocabulary shared with source topology and Section.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MixedArrangementVertex {
    SourceVertex(usize),
    SectionEndpoint(usize),
}

type ConnectedPlanarArrangement =
    FaceArrangement<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex>;
type EmbeddedPlanarArrangement =
    SurfaceFaceArrangement<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex, usize>;
type EmbeddedPlanarError =
    SurfaceArrangementError<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex, usize>;

/// One exact planar cell, including every disconnected boundary cycle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedPlanarArrangementCell {
    key: usize,
    boundaries:
        Vec<ArrangementCycle<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex>>,
}

impl MixedPlanarArrangementCell {
    pub(crate) const fn key(&self) -> usize {
        self.key
    }

    pub(crate) fn boundaries(
        &self,
    ) -> &[ArrangementCycle<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex>] {
        &self.boundaries
    }

    #[cfg(test)]
    fn boundary(
        &self,
    ) -> &ArrangementCycle<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex> {
        assert_eq!(self.boundaries.len(), 1);
        &self.boundaries[0]
    }
}

/// Exact cells on the forward and reverse sides of one planar cut fragment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedPlanarCutAdjacency {
    cut: MixedCutFragmentKey,
    forward_cell: usize,
    reverse_cell: usize,
}

impl MixedPlanarCutAdjacency {
    pub(crate) const fn cut(&self) -> &MixedCutFragmentKey {
        &self.cut
    }

    pub(crate) const fn forward_cell(&self) -> usize {
        self.forward_cell
    }

    pub(crate) const fn reverse_cell(&self) -> usize {
        self.reverse_cell
    }
}

/// Representation-independent proof summary used by focused adapter tests.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedPlanarArrangementProof {
    source_spans_conserved: usize,
    opposed_cut_pairs: usize,
    closed_cycles: usize,
    exterior_cycles: usize,
    dual_connected: bool,
}

impl MixedPlanarArrangementProof {
    pub(crate) const fn source_spans_conserved(&self) -> usize {
        self.source_spans_conserved
    }

    pub(crate) const fn opposed_cut_pairs(&self) -> usize {
        self.opposed_cut_pairs
    }

    pub(crate) const fn closed_cycles(&self) -> usize {
        self.closed_cycles
    }

    pub(crate) const fn exterior_cycles(&self) -> usize {
        self.exterior_cycles
    }

    pub(crate) const fn dual_connected(&self) -> bool {
        self.dual_connected
    }
}

/// Certified planar arrangement produced for one source face.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedPlanarFaceArrangement {
    source_spans: Vec<DirectedSourceSpan<MixedSourceSpanKey, MixedArrangementVertex>>,
    cut_fragments: Vec<DirectedCutFragment<MixedCutFragmentKey, MixedArrangementVertex>>,
    cells: Vec<MixedPlanarArrangementCell>,
    adjacency: Vec<MixedPlanarCutAdjacency>,
    proof: MixedPlanarArrangementProof,
}

impl MixedPlanarFaceArrangement {
    pub(crate) fn source_spans(
        &self,
    ) -> &[DirectedSourceSpan<MixedSourceSpanKey, MixedArrangementVertex>] {
        &self.source_spans
    }

    pub(crate) fn cut_fragments(
        &self,
    ) -> &[DirectedCutFragment<MixedCutFragmentKey, MixedArrangementVertex>] {
        &self.cut_fragments
    }

    pub(crate) fn cells(&self) -> &[MixedPlanarArrangementCell] {
        &self.cells
    }

    pub(crate) fn adjacency(&self) -> &[MixedPlanarCutAdjacency] {
        &self.adjacency
    }

    pub(crate) const fn proof(&self) -> &MixedPlanarArrangementProof {
        &self.proof
    }
}

/// Exact parameter authority at one end of a topology-owned source span.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum MixedSourceParameterEvidence {
    SourceVertex {
        topology_ordinal: usize,
        vertex: RawVertexId,
        edge_parameter_bits: u64,
    },
    SectionRoot {
        endpoint: usize,
        root_ordinal: usize,
        enclosure_bits: [u64; 2],
    },
}

impl MixedSourceParameterEvidence {
    /// Closed numeric enclosure retained for realization-only sampling.
    ///
    /// Source/root identity and ordering remain owned by the enum payload and
    /// the arrangement proof.  Callers may use this interval only to choose a
    /// point strictly inside an already-certified source span.
    pub(crate) fn parameter_interval(&self) -> [f64; 2] {
        match self {
            Self::SourceVertex {
                edge_parameter_bits,
                ..
            } => {
                let parameter = f64::from_bits(*edge_parameter_bits);
                [parameter, parameter]
            }
            Self::SectionRoot { enclosure_bits, .. } => enclosure_bits.map(f64::from_bits),
        }
    }
}

/// Non-key payload connecting one comparable span key to source topology.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedSourceSpanLineage {
    key: MixedSourceSpanKey,
    loop_id: RawLoopId,
    fin: RawFinId,
    edge: RawEdgeId,
    pub(crate) range: [MixedSourceParameterEvidence; 2],
}

impl MixedSourceSpanLineage {
    pub(crate) const fn key(&self) -> &MixedSourceSpanKey {
        &self.key
    }

    pub(crate) const fn loop_id(&self) -> RawLoopId {
        self.loop_id
    }

    pub(crate) const fn fin(&self) -> RawFinId {
        self.fin
    }

    pub(crate) const fn edge(&self) -> RawEdgeId {
        self.edge
    }

    pub(crate) const fn range(&self) -> &[MixedSourceParameterEvidence; 2] {
        &self.range
    }
}

/// Certified source topology behind every opaque arrangement ordinal.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedPlanarSourceLineage {
    pub(crate) spans: Vec<MixedSourceSpanLineage>,
    pub(crate) source_vertices: Vec<RawVertexId>,
}

impl MixedPlanarSourceLineage {
    pub(crate) fn spans(&self) -> &[MixedSourceSpanLineage] {
        &self.spans
    }

    pub(crate) fn source_vertices(&self) -> &[RawVertexId] {
        &self.source_vertices
    }
}

/// Arrangement and its separately certified, non-ordering lineage payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedPlanarFaceOutput {
    arrangement: MixedPlanarFaceArrangement,
    lineage: MixedPlanarSourceLineage,
}

impl MixedPlanarFaceOutput {
    pub(crate) const fn arrangement(&self) -> &MixedPlanarFaceArrangement {
        &self.arrangement
    }

    pub(crate) const fn lineage(&self) -> &MixedPlanarSourceLineage {
        &self.lineage
    }

    pub(crate) fn into_parts(self) -> (MixedPlanarFaceArrangement, MixedPlanarSourceLineage) {
        (self.arrangement, self.lineage)
    }
}

/// Fail-closed refusals while adapting Section evidence to one source face.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum MixedFaceArrangementError {
    InvalidOperand,
    SectionIncomplete,
    MissingSourceFace,
    UnsupportedSourceSurface,
    PeriodicSurfaceEmbeddingEvidenceRequired,
    MultipleSourceLoops,
    EmptySourceLoop,
    MissingSourceLoop,
    MissingSourceFin,
    MissingSourceEdge,
    FinParentMismatch,
    FinEdgeMismatch,
    OpenSourceLoop(MixedArrangementVertex),
    RepeatedBoundaryEdge(RawEdgeId),
    RingSourceEdge(RawEdgeId),
    InvalidSourceEdgeBounds(RawEdgeId),
    MissingSectionBranch(usize),
    WholeFragment(MixedCutFragmentKey),
    FragmentMissingFromClosedComponent(MixedCutFragmentKey),
    DuplicateFragmentComponentUse(MixedCutFragmentKey),
    MissingSectionEndpoint(usize),
    ParameterSeamEndpoint(usize),
    EndpointFaceMismatch(usize),
    EndpointSiteMismatch(usize),
    MissingRootProvenance(usize),
    UnexpectedRootProvenance(usize),
    RootFaceMismatch(usize),
    RootLoopMismatch(usize),
    RootFinMismatch(usize),
    RootEdgeMismatch(usize),
    RootParameterMismatch(usize),
    DuplicateRootProvenance {
        edge: RawEdgeId,
        root_ordinal: usize,
    },
    NonContiguousRootOrdinals(RawEdgeId),
    RootOutsideOpenEdgeRange {
        edge: RawEdgeId,
        root_ordinal: usize,
    },
    IncompatibleRootOrder(RawEdgeId),
    UnassignedRootProvenance(usize),
    InteriorCrossingProofRequired(Vec<MixedCutFragmentKey>),
    InteriorCycleEmbeddingRequired(Vec<MixedCutFragmentKey>),
    InteriorCycleOrientationRequired(Vec<MixedCutFragmentKey>),
    SourceBoundaryOrientationRequired,
    OpenCutEndpoint(MixedArrangementVertex),
    BranchedCutEndpoint(MixedArrangementVertex),
    InconsistentCutDirection(MixedArrangementVertex),
    AmbiguousBoundaryRotation(MixedArrangementVertex),
    Arrangement(
        FaceArrangementError<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex>,
    ),
    EmbeddedArrangement(EmbeddedPlanarError),
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct RootKey {
    edge: RawEdgeId,
    ordinal: usize,
}

#[derive(Debug, Clone)]
struct BoundaryRootEvidence {
    endpoint: usize,
    key: RootKey,
    interval: RootInterval,
    loop_id: RawLoopId,
    fin: RawFinId,
}

#[derive(Debug, Clone)]
struct CutEndpointEvidence {
    vertex: EvidenceEndpoint,
    boundary_root: Option<BoundaryRootEvidence>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum EvidenceEndpoint {
    SourceVertex(RawVertexId),
    SectionEndpoint(usize),
}

#[derive(Debug, Clone)]
struct FaceCutEvidence {
    key: MixedCutFragmentKey,
    endpoints: [CutEndpointEvidence; 2],
    embedding: CutEmbedding,
}

#[derive(Debug, Clone)]
enum CutEmbedding {
    Line {
        branch: usize,
        origin: [f64; 2],
        direction: [f64; 2],
        endpoints: [[f64; 2]; 2],
    },
    Circle {
        branch: usize,
    },
}

impl CutEmbedding {
    const fn branch(&self) -> usize {
        match self {
            Self::Line { branch, .. } | Self::Circle { branch } => *branch,
        }
    }
}

#[derive(Debug, Clone)]
struct RootOccurrence {
    face: FaceId,
    loop_id: LoopId,
    fin: FinId,
    edge: RawEdgeId,
    ordinal: usize,
    interval: SectionEdgeParameterInterval,
}

#[derive(Debug, Clone, Copy)]
struct RootInterval {
    lo: f64,
    hi: f64,
}

impl RootInterval {
    const fn from_section(interval: SectionEdgeParameterInterval) -> Self {
        Self {
            lo: interval.lo(),
            hi: interval.hi(),
        }
    }
}

/// Adapt a complete Section graph into one exact planar-face arrangement.
///
/// The function is read-only.  The caller must already have admitted the
/// source body and established that its loop traversal owns the face-domain
/// orientation (the Boolean pipeline's checked source extraction does so).
pub(crate) fn arrange_mixed_planar_face(
    store: &Store,
    graph: &BodySectionGraph,
    face: FaceId,
    operand: usize,
) -> Result<MixedPlanarFaceArrangement, MixedFaceArrangementError> {
    arrange_mixed_planar_face_with_lineage(store, graph, face, operand)
        .map(MixedPlanarFaceOutput::into_parts)
        .map(|(arrangement, _)| arrangement)
}

pub(crate) fn arrange_mixed_planar_face_with_lineage(
    store: &Store,
    graph: &BodySectionGraph,
    face: FaceId,
    operand: usize,
) -> Result<MixedPlanarFaceOutput, MixedFaceArrangementError> {
    if operand > 1 {
        return Err(MixedFaceArrangementError::InvalidOperand);
    }
    if graph.completion() != SectionCompletion::Complete || !graph.gaps().is_empty() {
        return Err(MixedFaceArrangementError::SectionIncomplete);
    }
    let raw_face = face.raw();
    let source_face = store
        .get(raw_face)
        .map_err(|_| MixedFaceArrangementError::MissingSourceFace)?;
    match store
        .get(source_face.surface())
        .map_err(|_| MixedFaceArrangementError::MissingSourceFace)?
    {
        SurfaceGeom::Plane(_) => {}
        SurfaceGeom::Cylinder(_) => {
            // Section does not yet publish periodic chart-unwrapping plus
            // cycle-to-cell embedding assignments for a cylinder side.
            return Err(MixedFaceArrangementError::PeriodicSurfaceEmbeddingEvidenceRequired);
        }
        _ => return Err(MixedFaceArrangementError::UnsupportedSourceSurface),
    }

    let cuts = collect_face_cuts(graph, &face, operand)?;
    arrange_planar_face_evidence_with_lineage(store, raw_face, cuts)
}

fn collect_face_cuts(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
) -> Result<Vec<FaceCutEvidence>, MixedFaceArrangementError> {
    let mut component_uses = vec![0_usize; graph.curve_fragments().len()];
    for component in graph.curve_components() {
        if !component.closed() {
            return Err(MixedFaceArrangementError::SectionIncomplete);
        }
        for &fragment in component.fragments() {
            let Some(uses) = component_uses.get_mut(fragment) else {
                return Err(MixedFaceArrangementError::MissingSectionBranch(fragment));
            };
            *uses += 1;
        }
    }

    let mut cuts = Vec::new();
    for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
        let Some(branch) = graph.branches().get(fragment.branch()) else {
            return Err(MixedFaceArrangementError::MissingSectionBranch(
                fragment.branch(),
            ));
        };
        if &branch.faces()[operand] != face {
            continue;
        }
        let key = MixedCutFragmentKey {
            branch: fragment.branch(),
            source_ordinal: fragment.source_ordinal(),
        };
        match component_uses[fragment_index] {
            0 => {
                return Err(MixedFaceArrangementError::FragmentMissingFromClosedComponent(key));
            }
            1 => {}
            _ => {
                return Err(MixedFaceArrangementError::DuplicateFragmentComponentUse(
                    key,
                ));
            }
        }
        cuts.push(adapt_fragment(graph, face, operand, fragment, key)?);
    }
    Ok(cuts)
}

fn adapt_fragment(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    fragment: &SectionCurveFragment,
    key: MixedCutFragmentKey,
) -> Result<FaceCutEvidence, MixedFaceArrangementError> {
    let branch = graph.branches().get(fragment.branch()).ok_or(
        MixedFaceArrangementError::MissingSectionBranch(fragment.branch()),
    )?;
    let carrier_parameters = match fragment.span() {
        SectionCurveFragmentSpan::Whole => None,
        SectionCurveFragmentSpan::Arc { endpoints, .. } => {
            Some(endpoints.each_ref().map(|end| end.carrier_parameter()))
        }
        SectionCurveFragmentSpan::LineSegment { endpoints } => {
            Some(endpoints.each_ref().map(|end| end.carrier_parameter()))
        }
    };
    let occurrences = match fragment.span() {
        SectionCurveFragmentSpan::Whole => {
            return Err(MixedFaceArrangementError::WholeFragment(key));
        }
        SectionCurveFragmentSpan::Arc { endpoints, .. } => endpoints.each_ref().map(|end| {
            let trim = end.trim();
            let root = (trim.operand() == operand).then(|| RootOccurrence {
                face: trim.face(),
                loop_id: trim.loop_id(),
                fin: trim.fin(),
                edge: trim.source_parameter().edge().raw(),
                ordinal: trim.source_parameter().root_ordinal(),
                interval: trim.edge_parameter(),
            });
            (end.endpoint(), root)
        }),
        SectionCurveFragmentSpan::LineSegment { endpoints } => endpoints.each_ref().map(|end| {
            let root = end.trims()[operand].as_ref().map(|trim| RootOccurrence {
                face: trim.face(),
                loop_id: trim.loop_id(),
                fin: trim.fin(),
                edge: trim.source_parameter().edge().raw(),
                ordinal: trim.source_parameter().root_ordinal(),
                interval: trim.edge_parameter(),
            });
            (end.endpoint(), root)
        }),
    };

    let endpoints = occurrences
        .map(|(endpoint, root)| adapt_endpoint(graph, face, operand, endpoint, root))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?;
    let [start, end]: [CutEndpointEvidence; 2] = endpoints
        .try_into()
        .map_err(|_| MixedFaceArrangementError::MissingSectionEndpoint(usize::MAX))?;
    let embedding = match (fragment.span(), branch.pcurves()[operand]) {
        (SectionCurveFragmentSpan::LineSegment { .. }, SectionUvCurve::Line(line)) => {
            let origin = line.origin();
            let direction = line.direction();
            let parameters = carrier_parameters.ok_or(
                MixedFaceArrangementError::EndpointSiteMismatch(start_endpoint(&start)),
            )?;
            CutEmbedding::Line {
                branch: fragment.branch(),
                origin: [origin.x, origin.y],
                direction: [direction.x, direction.y],
                endpoints: parameters.map(|parameter| {
                    let point = origin + direction * parameter;
                    [point.x, point.y]
                }),
            }
        }
        (SectionCurveFragmentSpan::Arc { .. }, SectionUvCurve::Circle(_)) => CutEmbedding::Circle {
            branch: fragment.branch(),
        },
        _ => {
            return Err(MixedFaceArrangementError::EndpointSiteMismatch(
                start_endpoint(&start),
            ));
        }
    };
    Ok(FaceCutEvidence {
        key,
        endpoints: [start, end],
        embedding,
    })
}

fn start_endpoint(endpoint: &CutEndpointEvidence) -> usize {
    match &endpoint.vertex {
        EvidenceEndpoint::SectionEndpoint(endpoint) => *endpoint,
        EvidenceEndpoint::SourceVertex(_) => usize::MAX,
    }
}

fn adapt_endpoint(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    endpoint_index: usize,
    occurrence: Option<RootOccurrence>,
) -> Result<CutEndpointEvidence, MixedFaceArrangementError> {
    let endpoint = graph.curve_endpoints().get(endpoint_index).ok_or(
        MixedFaceArrangementError::MissingSectionEndpoint(endpoint_index),
    )?;
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = endpoint.topology()
    else {
        return Err(MixedFaceArrangementError::ParameterSeamEndpoint(
            endpoint_index,
        ));
    };

    let source_parameter = source_parameters[operand].as_ref();
    let placement = match (&sites[operand], source_parameter) {
        (SectionSite::FaceInterior(actual), None) if actual == face => {
            if occurrence.is_some() {
                return Err(MixedFaceArrangementError::UnexpectedRootProvenance(
                    endpoint_index,
                ));
            }
            CutEndpointEvidence {
                vertex: EvidenceEndpoint::SectionEndpoint(endpoint_index),
                boundary_root: None,
            }
        }
        (SectionSite::AtVertex(vertex), None) => {
            if occurrence.is_some() {
                return Err(MixedFaceArrangementError::UnexpectedRootProvenance(
                    endpoint_index,
                ));
            }
            CutEndpointEvidence {
                vertex: EvidenceEndpoint::SourceVertex(vertex.raw()),
                boundary_root: None,
            }
        }
        (SectionSite::EdgeInterior(edge), Some(source)) if edge == &source.edge() => {
            let occurrence = occurrence.ok_or(MixedFaceArrangementError::MissingRootProvenance(
                endpoint_index,
            ))?;
            if &occurrence.face != face {
                return Err(MixedFaceArrangementError::RootFaceMismatch(endpoint_index));
            }
            if occurrence.edge != source.edge().raw() || occurrence.ordinal != source.root_ordinal()
            {
                return Err(MixedFaceArrangementError::RootEdgeMismatch(endpoint_index));
            }
            let common = endpoint.edge_parameters()[operand].ok_or(
                MixedFaceArrangementError::RootParameterMismatch(endpoint_index),
            )?;
            if common.lo() < occurrence.interval.lo() || common.hi() > occurrence.interval.hi() {
                return Err(MixedFaceArrangementError::RootParameterMismatch(
                    endpoint_index,
                ));
            }
            CutEndpointEvidence {
                vertex: EvidenceEndpoint::SectionEndpoint(endpoint_index),
                boundary_root: Some(BoundaryRootEvidence {
                    endpoint: endpoint_index,
                    key: RootKey {
                        edge: occurrence.edge,
                        ordinal: occurrence.ordinal,
                    },
                    interval: RootInterval::from_section(common),
                    loop_id: occurrence.loop_id.raw(),
                    fin: occurrence.fin.raw(),
                }),
            }
        }
        (SectionSite::FaceInterior(_), None) => {
            return Err(MixedFaceArrangementError::EndpointFaceMismatch(
                endpoint_index,
            ));
        }
        (SectionSite::EdgeInterior(_), None) => {
            return Err(MixedFaceArrangementError::MissingRootProvenance(
                endpoint_index,
            ));
        }
        _ => {
            return Err(MixedFaceArrangementError::EndpointSiteMismatch(
                endpoint_index,
            ));
        }
    };
    Ok(placement)
}

fn arrange_planar_face_evidence(
    store: &Store,
    face: RawFaceId,
    cuts: Vec<FaceCutEvidence>,
) -> Result<MixedPlanarFaceArrangement, MixedFaceArrangementError> {
    arrange_planar_face_evidence_with_lineage(store, face, cuts)
        .map(MixedPlanarFaceOutput::into_parts)
        .map(|(arrangement, _)| arrangement)
}

fn arrange_planar_face_evidence_with_lineage(
    store: &Store,
    face: RawFaceId,
    cuts: Vec<FaceCutEvidence>,
) -> Result<MixedPlanarFaceOutput, MixedFaceArrangementError> {
    let roots = collect_unique_roots(&cuts)?;
    let split = split_source_boundary(store, face, &roots)?;
    certify_cut_embedding(&cuts)?;
    let cut_fragments = cuts
        .iter()
        .map(|cut| {
            Ok(DirectedCutFragment::new(
                cut.key.clone(),
                normalize_cut_endpoint(&cut.endpoints[0].vertex, &split.source_vertices)?,
                normalize_cut_endpoint(&cut.endpoints[1].vertex, &split.source_vertices)?,
            ))
        })
        .collect::<Result<Vec<_>, MixedFaceArrangementError>>()?;
    let rotations = build_rotations(&split.spans, &cut_fragments)?;
    let source_anchor = split
        .spans
        .first()
        .map(|span| span.key().clone())
        .ok_or(MixedFaceArrangementError::EmptySourceLoop)?;
    let input = FaceArrangementInput::new(split.spans, cut_fragments, rotations);
    let arrangement = match arrange_bounded_face(input.clone()) {
        Ok(arrangement) => normalize_connected_arrangement(arrangement),
        Err(FaceArrangementError::DisconnectedPrimal) => {
            arrange_interior_planar_surface(store, face, &cuts, source_anchor, input)?
        }
        Err(error) => return Err(MixedFaceArrangementError::Arrangement(error)),
    };
    Ok(MixedPlanarFaceOutput {
        arrangement,
        lineage: MixedPlanarSourceLineage {
            spans: split.lineage,
            source_vertices: split.source_vertices,
        },
    })
}

fn normalize_connected_arrangement(
    arrangement: ConnectedPlanarArrangement,
) -> MixedPlanarFaceArrangement {
    let proof = arrangement.proof();
    MixedPlanarFaceArrangement {
        source_spans: arrangement.source_spans().to_vec(),
        cut_fragments: arrangement.cut_fragments().to_vec(),
        cells: arrangement
            .cells()
            .iter()
            .map(|cell| MixedPlanarArrangementCell {
                key: cell.key(),
                boundaries: vec![cell.boundary().clone()],
            })
            .collect(),
        adjacency: arrangement
            .adjacency()
            .iter()
            .map(|edge| MixedPlanarCutAdjacency {
                cut: edge.cut().clone(),
                forward_cell: edge.forward_cell(),
                reverse_cell: edge.reverse_cell(),
            })
            .collect(),
        proof: MixedPlanarArrangementProof {
            source_spans_conserved: proof.source_spans_conserved(),
            opposed_cut_pairs: proof.opposed_cut_pairs(),
            closed_cycles: proof.closed_cycles(),
            exterior_cycles: proof.exterior_cycles(),
            dual_connected: proof.dual_connected(),
        },
    }
}

fn normalize_embedded_arrangement(
    arrangement: EmbeddedPlanarArrangement,
) -> MixedPlanarFaceArrangement {
    let proof = arrangement.proof();
    MixedPlanarFaceArrangement {
        source_spans: arrangement.source_spans().to_vec(),
        cut_fragments: arrangement.cut_fragments().to_vec(),
        cells: arrangement
            .cells()
            .iter()
            .map(|cell| MixedPlanarArrangementCell {
                key: *cell.key(),
                boundaries: cell.boundaries().to_vec(),
            })
            .collect(),
        adjacency: arrangement
            .adjacency()
            .iter()
            .map(|edge| MixedPlanarCutAdjacency {
                cut: edge.cut().clone(),
                forward_cell: *edge.forward_cell(),
                reverse_cell: *edge.reverse_cell(),
            })
            .collect(),
        proof: MixedPlanarArrangementProof {
            source_spans_conserved: proof.source_spans_conserved(),
            opposed_cut_pairs: proof.opposed_cut_pairs(),
            closed_cycles: proof.closed_cycles(),
            exterior_cycles: proof.exterior_cycles(),
            dual_connected: proof.dual_connected(),
        },
    }
}

#[derive(Debug)]
struct InteriorLineCycle {
    anchor: MixedCutFragmentKey,
    orientation: Orientation,
    points: Vec<[f64; 2]>,
}

fn arrange_interior_planar_surface(
    store: &Store,
    face: RawFaceId,
    cuts: &[FaceCutEvidence],
    source_anchor: MixedSourceSpanKey,
    input: FaceArrangementInput<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex>,
) -> Result<MixedPlanarFaceArrangement, MixedFaceArrangementError> {
    let keys = || cuts.iter().map(|cut| cut.key.clone()).collect::<Vec<_>>();
    if cuts.iter().flat_map(|cut| &cut.endpoints).any(|endpoint| {
        endpoint.boundary_root.is_some()
            || !matches!(endpoint.vertex, EvidenceEndpoint::SectionEndpoint(_))
    }) {
        return Err(MixedFaceArrangementError::InteriorCycleEmbeddingRequired(
            keys(),
        ));
    }
    let cycles = collect_interior_line_cycles(cuts)?;
    let source_orientation = source_boundary_orientation(store, face)?;
    let containment = cycle_containment(&cycles)?;
    let parents = immediate_cycle_parents(&containment)
        .ok_or_else(|| MixedFaceArrangementError::InteriorCycleEmbeddingRequired(keys()))?;
    let mut child_counts = vec![0usize; cycles.len() + 1];
    for parent in &parents {
        child_counts[parent.map_or(0, |parent| parent + 1)] += 1;
    }

    let mut assignments = vec![
        CertifiedCycleAssignment::new(
            ArrangementDartKey::source(source_anchor.clone(), ArrangementDirection::Forward),
            CertifiedCycleSide::Cell(0usize),
        ),
        CertifiedCycleAssignment::new(
            ArrangementDartKey::source(source_anchor, ArrangementDirection::Reverse),
            CertifiedCycleSide::Exterior,
        ),
    ];
    for (index, cycle) in cycles.iter().enumerate() {
        let interior = index + 1;
        let exterior = parents[index].map_or(0, |parent| parent + 1);
        let (forward, reverse) = if cycle.orientation == source_orientation {
            (interior, exterior)
        } else {
            (exterior, interior)
        };
        assignments.push(CertifiedCycleAssignment::new(
            ArrangementDartKey::cut(cycle.anchor.clone(), ArrangementDirection::Forward),
            CertifiedCycleSide::Cell(forward),
        ));
        assignments.push(CertifiedCycleAssignment::new(
            ArrangementDartKey::cut(cycle.anchor.clone(), ArrangementDirection::Reverse),
            CertifiedCycleSide::Cell(reverse),
        ));
    }
    let cells = child_counts
        .into_iter()
        .enumerate()
        .map(|(key, children)| {
            i64::try_from(children)
                .ok()
                .and_then(|children| 1_i64.checked_sub(children))
                .map(|chi| CertifiedCellTopology::new(key, chi))
                .ok_or_else(|| MixedFaceArrangementError::InteriorCycleEmbeddingRequired(keys()))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let embedded =
        arrange_bounded_surface(input, CertifiedSurfaceEmbedding::new(assignments, cells, 1))
            .map_err(MixedFaceArrangementError::EmbeddedArrangement)?;
    Ok(normalize_embedded_arrangement(embedded))
}

fn collect_interior_line_cycles(
    cuts: &[FaceCutEvidence],
) -> Result<Vec<InteriorLineCycle>, MixedFaceArrangementError> {
    let keys = || cuts.iter().map(|cut| cut.key.clone()).collect::<Vec<_>>();
    let mut starts = BTreeMap::new();
    for (index, cut) in cuts.iter().enumerate() {
        let Some(start) = section_endpoint(&cut.endpoints[0].vertex) else {
            return Err(MixedFaceArrangementError::InteriorCycleOrientationRequired(
                keys(),
            ));
        };
        if !matches!(cut.embedding, CutEmbedding::Line { .. })
            || starts.insert(start, index).is_some()
        {
            return Err(MixedFaceArrangementError::InteriorCycleOrientationRequired(
                keys(),
            ));
        }
    }
    let mut unvisited = (0..cuts.len()).collect::<BTreeSet<_>>();
    let mut cycles = Vec::new();
    while let Some(first) = unvisited
        .iter()
        .min_by_key(|index| &cuts[**index].key)
        .copied()
    {
        let mut current = first;
        let mut points = Vec::new();
        loop {
            if !unvisited.remove(&current) {
                return Err(MixedFaceArrangementError::InteriorCycleOrientationRequired(
                    keys(),
                ));
            }
            let CutEmbedding::Line { endpoints, .. } = &cuts[current].embedding else {
                return Err(MixedFaceArrangementError::InteriorCycleOrientationRequired(
                    keys(),
                ));
            };
            points.push(endpoints[0]);
            let end = section_endpoint(&cuts[current].endpoints[1].vertex).ok_or_else(|| {
                MixedFaceArrangementError::InteriorCycleOrientationRequired(keys())
            })?;
            let next = *starts.get(&end).ok_or_else(|| {
                MixedFaceArrangementError::InteriorCycleOrientationRequired(keys())
            })?;
            if next == first {
                break;
            }
            current = next;
        }
        let orientation = polygon_orientation2d(&points);
        if orientation == Orientation::Zero {
            return Err(MixedFaceArrangementError::InteriorCycleOrientationRequired(
                keys(),
            ));
        }
        cycles.push(InteriorLineCycle {
            anchor: cuts[first].key.clone(),
            orientation,
            points,
        });
    }
    Ok(cycles)
}

const fn section_endpoint(endpoint: &EvidenceEndpoint) -> Option<usize> {
    match endpoint {
        EvidenceEndpoint::SectionEndpoint(endpoint) => Some(*endpoint),
        EvidenceEndpoint::SourceVertex(_) => None,
    }
}

fn source_boundary_orientation(
    store: &Store,
    face: RawFaceId,
) -> Result<Orientation, MixedFaceArrangementError> {
    let face = store
        .get(face)
        .map_err(|_| MixedFaceArrangementError::MissingSourceFace)?;
    let [loop_id] = face.loops() else {
        return Err(MixedFaceArrangementError::SourceBoundaryOrientationRequired);
    };
    let loop_ = store
        .get(*loop_id)
        .map_err(|_| MixedFaceArrangementError::SourceBoundaryOrientationRequired)?;
    let mut points = Vec::with_capacity(loop_.fins().len());
    for fin_id in loop_.fins() {
        let fin = store
            .get(*fin_id)
            .map_err(|_| MixedFaceArrangementError::SourceBoundaryOrientationRequired)?;
        let edge = store
            .get(fin.edge())
            .map_err(|_| MixedFaceArrangementError::SourceBoundaryOrientationRequired)?;
        let (lo, hi) = edge
            .bounds()
            .ok_or(MixedFaceArrangementError::SourceBoundaryOrientationRequired)?;
        let parameter = if fin.sense() == Sense::Forward {
            lo
        } else {
            hi
        };
        let pcurve = fin
            .pcurve()
            .ok_or(MixedFaceArrangementError::SourceBoundaryOrientationRequired)?;
        let mapped = pcurve.edge_to_pcurve().map(parameter);
        let point = match store
            .get(pcurve.curve())
            .map_err(|_| MixedFaceArrangementError::SourceBoundaryOrientationRequired)?
        {
            Curve2dGeom::Line(line) => line.eval(mapped),
            _ => return Err(MixedFaceArrangementError::SourceBoundaryOrientationRequired),
        };
        points.push([point.x, point.y]);
    }
    let orientation = polygon_orientation2d(&points);
    if orientation == Orientation::Zero {
        Err(MixedFaceArrangementError::SourceBoundaryOrientationRequired)
    } else {
        Ok(orientation)
    }
}

fn cycle_containment(
    cycles: &[InteriorLineCycle],
) -> Result<Vec<Vec<bool>>, MixedFaceArrangementError> {
    let keys = || {
        cycles
            .iter()
            .map(|cycle| cycle.anchor.clone())
            .collect::<Vec<_>>()
    };
    let mut result = vec![vec![false; cycles.len()]; cycles.len()];
    for outer in 0..cycles.len() {
        for inner in 0..cycles.len() {
            if outer == inner {
                continue;
            }
            result[outer][inner] =
                strict_point_in_polygon(&cycles[outer].points, cycles[inner].points[0])
                    .ok_or_else(|| {
                        MixedFaceArrangementError::InteriorCycleEmbeddingRequired(keys())
                    })?;
        }
    }
    if (0..cycles.len()).any(|left| {
        (0..cycles.len()).any(|right| left != right && result[left][right] && result[right][left])
    }) {
        return Err(MixedFaceArrangementError::InteriorCycleEmbeddingRequired(
            keys(),
        ));
    }
    Ok(result)
}

fn immediate_cycle_parents(containment: &[Vec<bool>]) -> Option<Vec<Option<usize>>> {
    (0..containment.len())
        .map(|child| {
            let containers = (0..containment.len())
                .filter(|&candidate| containment[candidate][child])
                .collect::<Vec<_>>();
            if containers.is_empty() {
                return Some(None);
            }
            let parents = containers
                .iter()
                .copied()
                .filter(|&candidate| {
                    containers
                        .iter()
                        .all(|&other| other == candidate || containment[other][candidate])
                })
                .collect::<Vec<_>>();
            (parents.len() == 1).then_some(Some(parents[0]))
        })
        .collect()
}

fn strict_point_in_polygon(polygon: &[[f64; 2]], point: [f64; 2]) -> Option<bool> {
    let mut winding = 0_i64;
    for index in 0..polygon.len() {
        let start = polygon[index];
        let end = polygon[(index + 1) % polygon.len()];
        let side = orient2d(start, end, point);
        if side == Orientation::Zero
            && point[0] >= start[0].min(end[0])
            && point[0] <= start[0].max(end[0])
            && point[1] >= start[1].min(end[1])
            && point[1] <= start[1].max(end[1])
        {
            return None;
        }
        if start[1] <= point[1] {
            if end[1] > point[1] && side == Orientation::Positive {
                winding += 1;
            }
        } else if end[1] <= point[1] && side == Orientation::Negative {
            winding -= 1;
        }
    }
    Some(winding != 0)
}

fn certify_cut_embedding(cuts: &[FaceCutEvidence]) -> Result<(), MixedFaceArrangementError> {
    let mut unproved = BTreeSet::new();
    for left in 0..cuts.len() {
        for right in (left + 1)..cuts.len() {
            if cut_pair_proven_disjoint(&cuts[left], &cuts[right]) {
                continue;
            }
            unproved.insert(cuts[left].key.clone());
            unproved.insert(cuts[right].key.clone());
        }
    }
    if unproved.is_empty() {
        Ok(())
    } else {
        Err(MixedFaceArrangementError::InteriorCrossingProofRequired(
            unproved.into_iter().collect(),
        ))
    }
}

fn cut_pair_proven_disjoint(left: &FaceCutEvidence, right: &FaceCutEvidence) -> bool {
    match (&left.embedding, &right.embedding) {
        (CutEmbedding::Circle { branch: left }, CutEmbedding::Circle { branch: right })
            if left == right =>
        {
            // Distinct source ordinals on one certified clipped carrier are
            // disjoint coverage by the Section publisher's conservation
            // contract.  The arrangement core independently rejects a
            // rotation that would make their endpoint topology non-planar.
            true
        }
        (
            CutEmbedding::Line {
                branch: left_branch,
                direction: left_direction,
                endpoints: left_endpoints,
                ..
            },
            CutEmbedding::Line {
                branch: right_branch,
                direction: right_direction,
                endpoints: right_endpoints,
                ..
            },
        ) => {
            if left_branch == right_branch {
                return true;
            }
            let shared = shared_endpoint_count(left, right);
            if shared == 1 {
                return line_direction_relation(*left_direction, *right_direction)
                    .is_some_and(|orientation| orientation != Orientation::Zero);
            }
            shared == 0 && line_segments_proven_disjoint(*left_endpoints, *right_endpoints)
        }
        _ => false,
    }
}

fn line_segments_proven_disjoint(left: [[f64; 2]; 2], right: [[f64; 2]; 2]) -> bool {
    let orientations = [
        orient2d(left[0], left[1], right[0]),
        orient2d(left[0], left[1], right[1]),
        orient2d(right[0], right[1], left[0]),
        orient2d(right[0], right[1], left[1]),
    ];
    if orientations.contains(&Orientation::Zero) {
        return false;
    }
    !(orientations[0] != orientations[1] && orientations[2] != orientations[3])
}

fn shared_endpoint_count(left: &FaceCutEvidence, right: &FaceCutEvidence) -> usize {
    left.endpoints
        .iter()
        .flat_map(|left| {
            right
                .endpoints
                .iter()
                .filter(move |right| left.vertex == right.vertex)
        })
        .count()
}

fn line_direction_relation(left: [f64; 2], right: [f64; 2]) -> Option<Orientation> {
    affine_dot3(
        [-left[1], left[0], 0.0],
        [right[0], right[1], 0.0],
        [0.0; 3],
        0.0,
    )
    .map(|side| side.sign())
}

fn collect_unique_roots(
    cuts: &[FaceCutEvidence],
) -> Result<Vec<BoundaryRootEvidence>, MixedFaceArrangementError> {
    let mut roots: Vec<BoundaryRootEvidence> = Vec::new();
    for endpoint in cuts.iter().flat_map(|cut| &cut.endpoints) {
        let Some(root) = &endpoint.boundary_root else {
            continue;
        };
        if roots.iter().any(|existing| existing.key == root.key) {
            return Err(MixedFaceArrangementError::DuplicateRootProvenance {
                edge: root.key.edge,
                root_ordinal: root.key.ordinal,
            });
        }
        roots.push(root.clone());
    }
    Ok(roots)
}

fn normalize_cut_endpoint(
    endpoint: &EvidenceEndpoint,
    source_vertices: &[RawVertexId],
) -> Result<MixedArrangementVertex, MixedFaceArrangementError> {
    match endpoint {
        EvidenceEndpoint::SectionEndpoint(endpoint) => {
            Ok(MixedArrangementVertex::SectionEndpoint(*endpoint))
        }
        EvidenceEndpoint::SourceVertex(vertex) => source_vertices
            .iter()
            .position(|candidate| candidate == vertex)
            .map(MixedArrangementVertex::SourceVertex)
            .ok_or(MixedFaceArrangementError::OpenSourceLoop(
                MixedArrangementVertex::SourceVertex(usize::MAX),
            )),
    }
}

struct SplitSourceBoundary {
    spans: Vec<DirectedSourceSpan<MixedSourceSpanKey, MixedArrangementVertex>>,
    source_vertices: Vec<RawVertexId>,
    lineage: Vec<MixedSourceSpanLineage>,
}

fn split_source_boundary(
    store: &Store,
    face_id: RawFaceId,
    roots: &[BoundaryRootEvidence],
) -> Result<SplitSourceBoundary, MixedFaceArrangementError> {
    let face = store
        .get(face_id)
        .map_err(|_| MixedFaceArrangementError::MissingSourceFace)?;
    let [loop_id] = face.loops() else {
        return if face.loops().is_empty() {
            Err(MixedFaceArrangementError::EmptySourceLoop)
        } else {
            Err(MixedFaceArrangementError::MultipleSourceLoops)
        };
    };
    let loop_ = store
        .get(*loop_id)
        .map_err(|_| MixedFaceArrangementError::MissingSourceLoop)?;
    if loop_.face() != face_id {
        return Err(MixedFaceArrangementError::MissingSourceLoop);
    }
    if loop_.fins().is_empty() {
        return Err(MixedFaceArrangementError::EmptySourceLoop);
    }

    let mut seen_edges = Vec::new();
    let mut source_vertices = Vec::new();
    let mut assigned_endpoints = BTreeSet::new();
    let mut spans = Vec::new();
    let mut lineage = Vec::new();
    for (fin_loop_ordinal, fin_id) in loop_.fins().iter().enumerate() {
        let fin = store
            .get(*fin_id)
            .map_err(|_| MixedFaceArrangementError::MissingSourceFin)?;
        if fin.parent() != *loop_id {
            return Err(MixedFaceArrangementError::FinParentMismatch);
        }
        let edge_id = fin.edge();
        if seen_edges.contains(&edge_id) {
            return Err(MixedFaceArrangementError::RepeatedBoundaryEdge(edge_id));
        }
        seen_edges.push(edge_id);
        let edge = store
            .get(edge_id)
            .map_err(|_| MixedFaceArrangementError::MissingSourceEdge)?;
        let [Some(edge_start), Some(edge_end)] = edge.vertices() else {
            return Err(MixedFaceArrangementError::RingSourceEdge(edge_id));
        };
        let Some((edge_lo, edge_hi)) = edge.bounds() else {
            return Err(MixedFaceArrangementError::RingSourceEdge(edge_id));
        };
        if !edge_lo.is_finite() || !edge_hi.is_finite() || edge_lo >= edge_hi {
            return Err(MixedFaceArrangementError::InvalidSourceEdgeBounds(edge_id));
        }

        let mut edge_roots = roots
            .iter()
            .filter(|root| root.key.edge == edge_id)
            .cloned()
            .collect::<Vec<_>>();
        edge_roots.sort_by_key(|root| root.key.ordinal);
        for (expected, root) in edge_roots.iter().enumerate() {
            if root.key.ordinal != expected {
                return Err(MixedFaceArrangementError::NonContiguousRootOrdinals(
                    edge_id,
                ));
            }
            if root.loop_id != *loop_id {
                return Err(MixedFaceArrangementError::RootLoopMismatch(root.endpoint));
            }
            if root.fin != *fin_id {
                return Err(MixedFaceArrangementError::RootFinMismatch(root.endpoint));
            }
            if root.interval.lo <= edge_lo || root.interval.hi >= edge_hi {
                return Err(MixedFaceArrangementError::RootOutsideOpenEdgeRange {
                    edge: edge_id,
                    root_ordinal: root.key.ordinal,
                });
            }
            if !assigned_endpoints.insert(root.endpoint) {
                return Err(MixedFaceArrangementError::DuplicateRootProvenance {
                    edge: edge_id,
                    root_ordinal: root.key.ordinal,
                });
            }
        }
        if edge_roots
            .windows(2)
            .any(|pair| pair[0].interval.hi >= pair[1].interval.lo)
        {
            return Err(MixedFaceArrangementError::IncompatibleRootOrder(edge_id));
        }
        if fin.sense() == Sense::Reversed {
            edge_roots.reverse();
        }

        let (start, end) = if fin.sense() == Sense::Forward {
            (edge_start, edge_end)
        } else {
            (edge_end, edge_start)
        };
        let start = source_vertex_ordinal(&mut source_vertices, start);
        let end = source_vertex_ordinal(&mut source_vertices, end);
        let (start_parameter, end_parameter) = if fin.sense() == Sense::Forward {
            (edge_lo, edge_hi)
        } else {
            (edge_hi, edge_lo)
        };
        let vertices = std::iter::once(MixedArrangementVertex::SourceVertex(start))
            .chain(
                edge_roots
                    .iter()
                    .map(|root| MixedArrangementVertex::SectionEndpoint(root.endpoint)),
            )
            .chain(std::iter::once(MixedArrangementVertex::SourceVertex(end)))
            .collect::<Vec<_>>();
        let parameters = std::iter::once(MixedSourceParameterEvidence::SourceVertex {
            topology_ordinal: start,
            vertex: source_vertices[start],
            edge_parameter_bits: start_parameter.to_bits(),
        })
        .chain(
            edge_roots
                .iter()
                .map(|root| MixedSourceParameterEvidence::SectionRoot {
                    endpoint: root.endpoint,
                    root_ordinal: root.key.ordinal,
                    enclosure_bits: [root.interval.lo.to_bits(), root.interval.hi.to_bits()],
                }),
        )
        .chain(std::iter::once(
            MixedSourceParameterEvidence::SourceVertex {
                topology_ordinal: end,
                vertex: source_vertices[end],
                edge_parameter_bits: end_parameter.to_bits(),
            },
        ))
        .collect::<Vec<_>>();
        for (traversal_ordinal, pair) in vertices.windows(2).enumerate() {
            let key = MixedSourceSpanKey {
                fin_loop_ordinal,
                traversal_ordinal,
            };
            spans.push(DirectedSourceSpan::new(
                key.clone(),
                pair[0].clone(),
                pair[1].clone(),
            ));
            lineage.push(MixedSourceSpanLineage {
                key,
                loop_id: *loop_id,
                fin: *fin_id,
                edge: edge_id,
                range: [
                    parameters[traversal_ordinal].clone(),
                    parameters[traversal_ordinal + 1].clone(),
                ],
            });
        }
    }

    if assigned_endpoints.len() != roots.len() {
        let endpoint = roots
            .iter()
            .find(|root| !assigned_endpoints.contains(&root.endpoint))
            .map_or(usize::MAX, |root| root.endpoint);
        return Err(MixedFaceArrangementError::UnassignedRootProvenance(
            endpoint,
        ));
    }
    for (index, span) in spans.iter().enumerate() {
        let next = &spans[(index + 1) % spans.len()];
        if span.endpoints()[1] != next.endpoints()[0] {
            return Err(MixedFaceArrangementError::OpenSourceLoop(
                span.endpoints()[1].clone(),
            ));
        }
    }
    Ok(SplitSourceBoundary {
        spans,
        source_vertices,
        lineage,
    })
}

fn source_vertex_ordinal(vertices: &mut Vec<RawVertexId>, vertex: RawVertexId) -> usize {
    if let Some(ordinal) = vertices.iter().position(|candidate| *candidate == vertex) {
        ordinal
    } else {
        let ordinal = vertices.len();
        vertices.push(vertex);
        ordinal
    }
}

fn build_rotations(
    source_spans: &[DirectedSourceSpan<MixedSourceSpanKey, MixedArrangementVertex>],
    cuts: &[DirectedCutFragment<MixedCutFragmentKey, MixedArrangementVertex>],
) -> Result<
    Vec<CertifiedEndpointRotation<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex>>,
    MixedFaceArrangementError,
> {
    type Dart = ArrangementDartKey<MixedSourceSpanKey, MixedCutFragmentKey>;
    let mut source_outgoing: BTreeMap<MixedArrangementVertex, (Option<Dart>, Option<Dart>)> =
        BTreeMap::new();
    for span in source_spans {
        let [start, end] = span.endpoints();
        let at_start = source_outgoing.entry(start.clone()).or_default();
        if at_start.0.is_some() {
            return Err(MixedFaceArrangementError::OpenSourceLoop(start.clone()));
        }
        at_start.0 = Some(ArrangementDartKey::source(
            span.key().clone(),
            ArrangementDirection::Forward,
        ));
        let at_end = source_outgoing.entry(end.clone()).or_default();
        if at_end.1.is_some() {
            return Err(MixedFaceArrangementError::OpenSourceLoop(end.clone()));
        }
        at_end.1 = Some(ArrangementDartKey::source(
            span.key().clone(),
            ArrangementDirection::Reverse,
        ));
    }

    let mut cut_outgoing: BTreeMap<MixedArrangementVertex, Vec<Dart>> = BTreeMap::new();
    for cut in cuts {
        let [start, end] = cut.endpoints();
        cut_outgoing
            .entry(start.clone())
            .or_default()
            .push(ArrangementDartKey::cut(
                cut.key().clone(),
                ArrangementDirection::Forward,
            ));
        cut_outgoing
            .entry(end.clone())
            .or_default()
            .push(ArrangementDartKey::cut(
                cut.key().clone(),
                ArrangementDirection::Reverse,
            ));
    }

    let endpoints = source_outgoing
        .keys()
        .chain(cut_outgoing.keys())
        .cloned()
        .collect::<BTreeSet<_>>();
    let mut rotations = Vec::with_capacity(endpoints.len());
    for endpoint in endpoints {
        let cuts = cut_outgoing.remove(&endpoint).unwrap_or_default();
        let outgoing = if let Some((next, previous)) = source_outgoing.remove(&endpoint) {
            let (Some(next), Some(previous)) = (next, previous) else {
                return Err(MixedFaceArrangementError::OpenSourceLoop(endpoint));
            };
            if cuts.len() > 1 {
                return Err(MixedFaceArrangementError::AmbiguousBoundaryRotation(
                    endpoint,
                ));
            }
            let mut order = vec![next];
            order.extend(cuts);
            order.push(previous);
            order
        } else {
            match cuts.len() {
                0 | 1 => return Err(MixedFaceArrangementError::OpenCutEndpoint(endpoint)),
                2 => {
                    if cuts[0].direction() == cuts[1].direction() {
                        return Err(MixedFaceArrangementError::InconsistentCutDirection(
                            endpoint,
                        ));
                    }
                    let mut cuts = cuts;
                    cuts.sort_unstable();
                    cuts
                }
                _ => return Err(MixedFaceArrangementError::BranchedCutEndpoint(endpoint)),
            }
        };
        rotations.push(CertifiedEndpointRotation::new(endpoint, outgoing));
    }
    Ok(rotations)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BlockRequest, CylinderRequest, Kernel, SectionBodiesRequest, SectionCurveFragmentSpan,
        SectionPeriodicFaceEmbeddingEvidence,
    };
    use kgeom::frame::Frame;
    use ktopo::entity::{Edge, Face, Fin, Loop, Vertex};

    struct PlanarFixture {
        store: Store,
        face: RawFaceId,
        loop_id: RawLoopId,
        fins: Vec<RawFinId>,
        edges: Vec<RawEdgeId>,
    }

    fn planar_fixture(prefix_with_unrelated_body: bool) -> PlanarFixture {
        let mut store = Store::new();
        if prefix_with_unrelated_body {
            ktopo::make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        }
        let body = ktopo::make::block(
            &mut store,
            &Frame::world().with_origin(kgeom::vec::Point3::new(4.0, 0.0, 0.0)),
            [4.0, 4.0, 4.0],
        )
        .unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        let loop_id = store.get(face).unwrap().loops()[0];
        let fins = store.get(loop_id).unwrap().fins().to_vec();
        let edges = fins
            .iter()
            .map(|fin| store.get(*fin).unwrap().edge())
            .collect();
        PlanarFixture {
            store,
            face,
            loop_id,
            fins,
            edges,
        }
    }

    fn root(
        fixture: &PlanarFixture,
        fin_index: usize,
        endpoint: usize,
        ordinal: usize,
        fraction: f64,
    ) -> CutEndpointEvidence {
        let edge = fixture.store.get(fixture.edges[fin_index]).unwrap();
        let (lo, hi) = edge.bounds().unwrap();
        let center = lo + (hi - lo) * fraction;
        let radius = (hi - lo) * 0.01;
        CutEndpointEvidence {
            vertex: EvidenceEndpoint::SectionEndpoint(endpoint),
            boundary_root: Some(BoundaryRootEvidence {
                endpoint,
                key: RootKey {
                    edge: fixture.edges[fin_index],
                    ordinal,
                },
                interval: RootInterval {
                    lo: center - radius,
                    hi: center + radius,
                },
                loop_id: fixture.loop_id,
                fin: fixture.fins[fin_index],
            }),
        }
    }

    fn chord(fixture: &PlanarFixture) -> FaceCutEvidence {
        FaceCutEvidence {
            key: MixedCutFragmentKey {
                branch: 7,
                source_ordinal: 3,
            },
            endpoints: [root(fixture, 0, 10, 0, 0.5), root(fixture, 2, 11, 0, 0.5)],
            embedding: CutEmbedding::Line {
                branch: 7,
                origin: [0.0, 0.0],
                direction: [1.0, 0.0],
                endpoints: [[0.0, 0.0], [1.0, 0.0]],
            },
        }
    }

    fn store_shape(store: &Store) -> [usize; 5] {
        [
            store.count::<Face>(),
            store.count::<Loop>(),
            store.count::<Fin>(),
            store.count::<Edge>(),
            store.count::<Vertex>(),
        ]
    }

    fn two_non_crossing_cuts(
        fixture: &PlanarFixture,
        embedding: impl Fn(usize) -> CutEmbedding,
    ) -> Vec<FaceCutEvidence> {
        let traversal_root =
            |fin_index: usize, traversal_index: usize, endpoint: usize| -> CutEndpointEvidence {
                let forward =
                    fixture.store.get(fixture.fins[fin_index]).unwrap().sense() == Sense::Forward;
                let intrinsic = if forward {
                    traversal_index
                } else {
                    1 - traversal_index
                };
                root(
                    fixture,
                    fin_index,
                    endpoint,
                    intrinsic,
                    if intrinsic == 0 { 0.25 } else { 0.75 },
                )
            };
        let first_embedding = embedding(0);
        let second_embedding = embedding(1);
        vec![
            FaceCutEvidence {
                key: MixedCutFragmentKey {
                    branch: first_embedding.branch(),
                    source_ordinal: 0,
                },
                endpoints: [traversal_root(0, 0, 40), traversal_root(2, 1, 41)],
                embedding: first_embedding,
            },
            FaceCutEvidence {
                key: MixedCutFragmentKey {
                    branch: second_embedding.branch(),
                    source_ordinal: 1,
                },
                endpoints: [traversal_root(0, 1, 42), traversal_root(2, 0, 43)],
                embedding: second_embedding,
            },
        ]
    }

    #[test]
    fn topology_ordered_chord_produces_two_cells_without_mutation() {
        for prefixed in [false, true] {
            let fixture = planar_fixture(prefixed);
            let before = store_shape(&fixture.store);
            let arrangement =
                arrange_planar_face_evidence(&fixture.store, fixture.face, vec![chord(&fixture)])
                    .unwrap();
            assert_eq!(arrangement.source_spans().len(), 6);
            assert_eq!(arrangement.cut_fragments().len(), 1);
            assert_eq!(arrangement.cells().len(), 2);
            assert_eq!(arrangement.adjacency().len(), 1);
            assert_eq!(arrangement.proof().source_spans_conserved(), 6);
            assert_eq!(arrangement.proof().opposed_cut_pairs(), 1);
            assert_eq!(arrangement.proof().closed_cycles(), 3);
            assert_eq!(arrangement.proof().exterior_cycles(), 1);
            assert!(arrangement.proof().dual_connected());
            assert_eq!(store_shape(&fixture.store), before);
        }
    }

    #[test]
    fn same_carrier_arcs_and_exactly_parallel_rulings_share_the_general_core() {
        let fixture = planar_fixture(false);
        let arc_arrangement = arrange_planar_face_evidence(
            &fixture.store,
            fixture.face,
            two_non_crossing_cuts(&fixture, |_| CutEmbedding::Circle { branch: 40 }),
        )
        .unwrap();
        assert_eq!(arc_arrangement.cells().len(), 3);
        assert_eq!(arc_arrangement.adjacency().len(), 2);

        let line_arrangement = arrange_planar_face_evidence(
            &fixture.store,
            fixture.face,
            two_non_crossing_cuts(&fixture, |index| CutEmbedding::Line {
                branch: 40 + index,
                origin: [0.0, index as f64],
                direction: [1.0, 0.0],
                endpoints: [[0.0, index as f64], [1.0, index as f64]],
            }),
        )
        .unwrap();
        assert_eq!(line_arrangement.proof(), arc_arrangement.proof());
        assert_eq!(line_arrangement.cells().len(), 3);
        assert_eq!(line_arrangement.adjacency().len(), 2);
    }

    #[test]
    fn public_section_graph_adapts_in_both_operand_orders_and_refuses_cylinder_embedding() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(
                    Frame::world().with_origin(kgeom::vec::Point3::new(0.0, 0.0, 1.0)),
                    [2.0, 5.0, 1.0],
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

        for swapped in [false, true] {
            let (left, right, block_slot) = if swapped {
                (cylinder.clone(), block.clone(), 1)
            } else {
                (block.clone(), cylinder.clone(), 0)
            };
            let graph = session
                .part(part_id.clone())
                .unwrap()
                .section_bodies(SectionBodiesRequest::new(left, right))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(graph.completion(), SectionCompletion::Complete);
            let [SectionPeriodicFaceEmbeddingEvidence::Certified(periodic)] =
                graph.periodic_face_embeddings()
            else {
                panic!(
                    "mixed cycles lost periodic embedding evidence: {:?}",
                    graph.periodic_face_embeddings()
                );
            };
            assert_eq!(periodic.components().len(), 2);
            assert!(
                periodic
                    .components()
                    .iter()
                    .all(|component| { component.winding() == 0 && component.parent().is_none() })
            );
            let part = session.part(part_id.clone()).unwrap();
            let store = &part.state.store;
            let mut planar_faces = Vec::new();
            for fragment in graph.curve_fragments() {
                assert!(!matches!(fragment.span(), SectionCurveFragmentSpan::Whole));
                let face = graph.branches()[fragment.branch()].faces()[block_slot].clone();
                if !planar_faces.contains(&face)
                    && store.get(face.raw()).unwrap().loops().len() == 1
                {
                    planar_faces.push(face);
                }
            }
            assert!(!planar_faces.is_empty());
            for face in planar_faces {
                let arrangement =
                    arrange_mixed_planar_face(store, &graph, face, block_slot).unwrap();
                assert!(!arrangement.cells().is_empty());
                assert!(arrangement.proof().dual_connected());
            }

            let cylinder_slot = 1 - block_slot;
            let cylinder_face = graph
                .branches()
                .iter()
                .map(|branch| branch.faces()[cylinder_slot].clone())
                .find(|face| {
                    matches!(
                        store.get(store.get(face.raw()).unwrap().surface()).unwrap(),
                        SurfaceGeom::Cylinder(_)
                    )
                })
                .unwrap();
            assert_eq!(
                arrange_mixed_planar_face(store, &graph, cylinder_face, cylinder_slot),
                Err(MixedFaceArrangementError::PeriodicSurfaceEmbeddingEvidenceRequired)
            );
        }
    }

    #[test]
    fn exact_shared_endpoint_proves_distinct_nonparallel_lines_meet_only_at_the_join() {
        let line = |branch: usize,
                    endpoint_ids: [usize; 2],
                    origin: [f64; 2],
                    direction: [f64; 2]| FaceCutEvidence {
            key: MixedCutFragmentKey {
                branch,
                source_ordinal: 0,
            },
            endpoints: endpoint_ids.map(|endpoint| CutEndpointEvidence {
                vertex: EvidenceEndpoint::SectionEndpoint(endpoint),
                boundary_root: None,
            }),
            embedding: CutEmbedding::Line {
                branch,
                origin,
                direction,
                endpoints: [origin, [origin[0] + direction[0], origin[1] + direction[1]]],
            },
        };
        let horizontal = line(0, [10, 11], [0.0, 0.0], [1.0, 0.0]);
        let joined_vertical = line(1, [11, 12], [1.0, 0.0], [0.0, 1.0]);
        assert!(certify_cut_embedding(&[horizontal.clone(), joined_vertical]).is_ok());

        let unjoined_vertical = line(1, [12, 13], [0.5, -1.0], [0.0, 1.0]);
        assert!(matches!(
            certify_cut_embedding(&[horizontal.clone(), unjoined_vertical]),
            Err(MixedFaceArrangementError::InteriorCrossingProofRequired(keys))
                if keys.len() == 2
        ));

        let ambiguous_collinear = line(2, [11, 14], [0.5, 0.0], [1.0, 0.0]);
        assert!(matches!(
            certify_cut_embedding(&[horizontal, ambiguous_collinear]),
            Err(MixedFaceArrangementError::InteriorCrossingProofRequired(keys))
                if keys.len() == 2
        ));
    }

    fn interior_polygon_cuts(
        branch_base: usize,
        endpoint_base: usize,
        points: &[[f64; 2]],
    ) -> Vec<FaceCutEvidence> {
        (0..points.len())
            .map(|index| {
                let next = (index + 1) % points.len();
                let start = points[index];
                let end = points[next];
                let branch = branch_base + index;
                FaceCutEvidence {
                    key: MixedCutFragmentKey {
                        branch,
                        source_ordinal: 0,
                    },
                    endpoints: [endpoint_base + index, endpoint_base + next].map(|endpoint| {
                        CutEndpointEvidence {
                            vertex: EvidenceEndpoint::SectionEndpoint(endpoint),
                            boundary_root: None,
                        }
                    }),
                    embedding: CutEmbedding::Line {
                        branch,
                        origin: start,
                        direction: [end[0] - start[0], end[1] - start[1]],
                        endpoints: [start, end],
                    },
                }
            })
            .collect()
    }

    #[test]
    fn any_nested_count_of_exact_line_cycles_builds_connected_planar_dual_cells() {
        let fixture = planar_fixture(false);
        for cycle_count in 1..=4 {
            let mut cuts = Vec::new();
            for cycle in 0..cycle_count {
                let radius = 3.5 - cycle as f64 * 0.6;
                cuts.extend(interior_polygon_cuts(
                    cycle * 10,
                    100 + cycle * 10,
                    &[
                        [-radius, -radius],
                        [radius, -radius],
                        [radius, radius],
                        [-radius, radius],
                    ],
                ));
            }
            let arrangement =
                arrange_planar_face_evidence(&fixture.store, fixture.face, cuts.clone()).unwrap();
            assert_eq!(arrangement.cells().len(), cycle_count + 1);
            assert_eq!(arrangement.adjacency().len(), cycle_count * 4);
            assert_eq!(arrangement.cells()[0].boundaries().len(), 2);
            assert_eq!(arrangement.cells().last().unwrap().boundaries().len(), 1);
            assert!(arrangement.proof().dual_connected());

            cuts.reverse();
            let permuted =
                arrange_planar_face_evidence(&fixture.store, fixture.face, cuts).unwrap();
            assert_eq!(permuted, arrangement);
        }
    }

    #[test]
    fn exact_root_defects_refuse_before_arrangement() {
        let fixture = planar_fixture(false);

        let mut wrong_ordinal = chord(&fixture);
        wrong_ordinal.endpoints[0]
            .boundary_root
            .as_mut()
            .unwrap()
            .key
            .ordinal = 1;
        assert!(matches!(
            arrange_planar_face_evidence(&fixture.store, fixture.face, vec![wrong_ordinal]),
            Err(MixedFaceArrangementError::NonContiguousRootOrdinals(edge))
                if edge == fixture.edges[0]
        ));

        let first = chord(&fixture);
        let mut duplicate = chord(&fixture);
        duplicate.key = MixedCutFragmentKey {
            branch: 8,
            source_ordinal: 0,
        };
        duplicate.endpoints[1] = root(&fixture, 1, 12, 0, 0.5);
        assert!(matches!(
            arrange_planar_face_evidence(
                &fixture.store,
                fixture.face,
                vec![first, duplicate]
            ),
            Err(MixedFaceArrangementError::DuplicateRootProvenance {
                edge,
                root_ordinal: 0
            }) if edge == fixture.edges[0]
        ));

        let cuts = vec![
            FaceCutEvidence {
                key: MixedCutFragmentKey {
                    branch: 0,
                    source_ordinal: 0,
                },
                endpoints: [
                    root(&fixture, 0, 20, 0, 0.75),
                    root(&fixture, 2, 21, 0, 0.25),
                ],
                embedding: CutEmbedding::Line {
                    branch: 0,
                    origin: [0.0, 0.0],
                    direction: [1.0, 0.0],
                    endpoints: [[0.0, 0.0], [1.0, 0.0]],
                },
            },
            FaceCutEvidence {
                key: MixedCutFragmentKey {
                    branch: 1,
                    source_ordinal: 0,
                },
                endpoints: [
                    root(&fixture, 0, 22, 1, 0.25),
                    root(&fixture, 2, 23, 1, 0.75),
                ],
                embedding: CutEmbedding::Line {
                    branch: 1,
                    origin: [0.0, 1.0],
                    direction: [1.0, 0.0],
                    endpoints: [[0.0, 1.0], [1.0, 1.0]],
                },
            },
        ];
        assert!(matches!(
            arrange_planar_face_evidence(&fixture.store, fixture.face, cuts),
            Err(MixedFaceArrangementError::IncompatibleRootOrder(edge))
                if edge == fixture.edges[0]
        ));
    }

    #[test]
    fn provenance_and_embedding_gaps_are_typed_and_read_only() {
        let fixture = planar_fixture(false);
        let before = store_shape(&fixture.store);
        let mut wrong_fin = chord(&fixture);
        wrong_fin.endpoints[0].boundary_root.as_mut().unwrap().fin = fixture.fins[1];
        assert!(matches!(
            arrange_planar_face_evidence(&fixture.store, fixture.face, vec![wrong_fin]),
            Err(MixedFaceArrangementError::RootFinMismatch(10))
        ));

        let mut second = chord(&fixture);
        second.key = MixedCutFragmentKey {
            branch: 9,
            source_ordinal: 0,
        };
        second.embedding = CutEmbedding::Line {
            branch: 9,
            origin: [0.0, 0.0],
            direction: [0.0, 1.0],
            endpoints: [[0.0, 0.0], [0.0, 1.0]],
        };
        second.endpoints = [root(&fixture, 1, 30, 0, 0.5), root(&fixture, 3, 31, 0, 0.5)];
        assert!(matches!(
            arrange_planar_face_evidence(
                &fixture.store,
                fixture.face,
                vec![chord(&fixture), second]
            ),
            Err(MixedFaceArrangementError::InteriorCrossingProofRequired(keys))
                if keys.len() == 2
        ));
        assert_eq!(store_shape(&fixture.store), before);
    }

    #[test]
    fn repeated_or_branched_endpoint_incidence_never_guesses_a_rotation() {
        let fixture = planar_fixture(false);
        let roots = collect_unique_roots(&[chord(&fixture)]).unwrap();
        let source = split_source_boundary(&fixture.store, fixture.face, &roots).unwrap();
        let boundary = MixedArrangementVertex::SectionEndpoint(10);
        let cuts = vec![
            DirectedCutFragment::new(
                MixedCutFragmentKey {
                    branch: 0,
                    source_ordinal: 0,
                },
                boundary.clone(),
                MixedArrangementVertex::SectionEndpoint(90),
            ),
            DirectedCutFragment::new(
                MixedCutFragmentKey {
                    branch: 1,
                    source_ordinal: 0,
                },
                MixedArrangementVertex::SectionEndpoint(91),
                boundary.clone(),
            ),
        ];
        assert_eq!(
            build_rotations(&source.spans, &cuts),
            Err(MixedFaceArrangementError::AmbiguousBoundaryRotation(
                boundary
            ))
        );

        let interior = MixedArrangementVertex::SectionEndpoint(99);
        let branch = (0..3)
            .map(|index| {
                DirectedCutFragment::new(
                    MixedCutFragmentKey {
                        branch: index,
                        source_ordinal: 0,
                    },
                    interior.clone(),
                    MixedArrangementVertex::SectionEndpoint(100 + index),
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(
            build_rotations(&[], &branch),
            Err(MixedFaceArrangementError::BranchedCutEndpoint(interior))
        );
    }
}
