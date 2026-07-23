//! Certified cylinder-side arrangements from periodic section evidence.
//!
//! This adapter admits one representation theorem, not a Boolean layout: the
//! source face is an annulus with two topology-owned whole-loop boundaries,
//! and the section evidence is either a set of certified simple,
//! contractible, pairwise-disjoint, nonnested cycles, a complete cyclically
//! matched transverse family, or a complete laminar returning-trace family.
//! Exact section endpoint indices
//! own incidence. Exact root order and integer chart shifts own bounded ring
//! spans. No rounded UV or model-space representative is used for an
//! arrangement decision.

use std::collections::{BTreeMap, BTreeSet};

use ktopo::entity::LoopId as RawLoopId;

use super::face_arrangement::{
    ArrangementCycle, ArrangementDartKey, ArrangementDirection, ArrangementEdgeKey,
    CertifiedCellTopology, CertifiedCycleAssignment, CertifiedCycleSide, CertifiedEndpointRotation,
    CertifiedSurfaceEmbedding, DirectedCutFragment, DirectedSourceSpan, FaceArrangementInput,
    SurfaceArrangementError, SurfaceFaceArrangement, arrange_bounded_surface,
    preview_bounded_surface_cycles,
};
use crate::{
    BodySectionGraph, FaceId, SectionCompletion, SectionPeriodicCycleOrientation,
    SectionPeriodicEmbeddingGap, SectionPeriodicFaceEmbeddingEvidence,
};

/// Exact identity of one directed section fragment on the periodic face.
#[derive(Debug, Clone, Copy)]
pub(crate) struct PeriodicCutFragmentKey {
    component: usize,
    source_component: Option<usize>,
    ordinal: usize,
    fragment: usize,
    cylinder_period_shift: i64,
}

impl PartialEq for PeriodicCutFragmentKey {
    fn eq(&self, other: &Self) -> bool {
        (
            self.component,
            self.source_component,
            self.ordinal,
            self.fragment,
        ) == (
            other.component,
            other.source_component,
            other.ordinal,
            other.fragment,
        )
    }
}

impl Eq for PeriodicCutFragmentKey {}

impl PartialOrd for PeriodicCutFragmentKey {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PeriodicCutFragmentKey {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        (
            self.component,
            self.source_component,
            self.ordinal,
            self.fragment,
        )
            .cmp(&(
                other.component,
                other.source_component,
                other.ordinal,
                other.fragment,
            ))
    }
}

impl PeriodicCutFragmentKey {
    pub(crate) const fn component(self) -> usize {
        self.component
    }

    /// Backing global component, or `None` for a face-local trace group.
    pub(crate) const fn source_component(self) -> Option<usize> {
        self.source_component
    }

    pub(crate) const fn ordinal(self) -> usize {
        self.ordinal
    }

    pub(crate) const fn fragment(self) -> usize {
        self.fragment
    }

    /// Whole cylinder periods applied to the canonical section pcurve.
    pub(crate) const fn cylinder_period_shift(self) -> i64 {
        self.cylinder_period_shift
    }
}

/// Exact source-root lineage retained on one bounded ring span.
#[derive(Debug, Clone, Copy)]
pub(crate) struct PeriodicSourceRootKey {
    endpoint: usize,
    cyclic_order: usize,
    source_root_ordinal: usize,
    root_parameter_bits: u64,
    root_enclosure_bits: [u64; 2],
    cylinder_chart_shift: i64,
}

impl PartialEq for PeriodicSourceRootKey {
    fn eq(&self, other: &Self) -> bool {
        (
            self.endpoint,
            self.cyclic_order,
            self.source_root_ordinal,
            self.cylinder_chart_shift,
        ) == (
            other.endpoint,
            other.cyclic_order,
            other.source_root_ordinal,
            other.cylinder_chart_shift,
        )
    }
}

impl Eq for PeriodicSourceRootKey {}

impl PartialOrd for PeriodicSourceRootKey {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PeriodicSourceRootKey {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        (
            self.endpoint,
            self.cyclic_order,
            self.source_root_ordinal,
            self.cylinder_chart_shift,
        )
            .cmp(&(
                other.endpoint,
                other.cyclic_order,
                other.source_root_ordinal,
                other.cylinder_chart_shift,
            ))
    }
}

impl PeriodicSourceRootKey {
    pub(crate) const fn endpoint(self) -> usize {
        self.endpoint
    }

    pub(crate) const fn cyclic_order(self) -> usize {
        self.cyclic_order
    }

    pub(crate) const fn source_root_ordinal(self) -> usize {
        self.source_root_ordinal
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

    pub(crate) const fn cylinder_chart_shift(self) -> i64 {
        self.cylinder_chart_shift
    }
}

/// Exact source-loop identity by position in the certified face loop vector.
///
/// The sealed evidence retains the corresponding raw loop handle. This
/// comparable key deliberately uses topology order because arena handles do
/// not expose or implement an ordering contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct PeriodicSourceLoopKey {
    topology_ordinal: usize,
    source_direction: ArrangementDirection,
    cyclic_span_ordinal: Option<usize>,
    terminal_roots: Option<[PeriodicSourceRootKey; 2]>,
}

impl PeriodicSourceLoopKey {
    pub(crate) const fn topology_ordinal(self) -> usize {
        self.topology_ordinal
    }

    /// Physical source-fin direction relative to the arrangement's canonical
    /// domain-on-left source dart.
    pub(crate) const fn source_direction(self) -> ArrangementDirection {
        self.source_direction
    }

    /// Increasing-canonical-longitude interval represented by this span.
    /// `None` denotes the legacy endpoint-free whole ring.
    pub(crate) const fn cyclic_span_ordinal(self) -> Option<usize> {
        self.cyclic_span_ordinal
    }

    /// Directed source-span endpoints, including exact root and chart lineage.
    pub(crate) const fn terminal_roots(self) -> Option<[PeriodicSourceRootKey; 2]> {
        self.terminal_roots
    }

    pub(crate) const fn is_whole_loop(self) -> bool {
        self.cyclic_span_ordinal.is_none() && self.terminal_roots.is_none()
    }
}

/// Physical section endpoint or a proof-only seam on a whole source ring.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum PeriodicArrangementVertexKey {
    SourceLoopSeam(PeriodicSourceLoopKey),
    SectionEndpoint(usize),
}

/// Proof-owned open cell on the annular cylinder side.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum PeriodicArrangementCellKey {
    AnnularRemainder,
    ComponentDisk(usize),
    TraceCell(PeriodicBoundaryTraceKey),
}

/// Stable identity of one trace-delimited annulus cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct PeriodicBoundaryTraceKey {
    component: usize,
    source_component: Option<usize>,
    first_component_ordinal: usize,
}

impl PeriodicBoundaryTraceKey {
    pub(crate) const fn component(self) -> usize {
        self.component
    }

    /// Backing global component, or `None` for a face-local trace group.
    pub(crate) const fn source_component(self) -> Option<usize> {
        self.source_component
    }

    pub(crate) const fn first_component_ordinal(self) -> usize {
        self.first_component_ordinal
    }
}

/// Exact cylinder-side arrangement consumed by a future analytic-shell plan.
pub(crate) type MixedPeriodicFaceArrangement = SurfaceFaceArrangement<
    PeriodicSourceLoopKey,
    PeriodicCutFragmentKey,
    PeriodicArrangementVertexKey,
    PeriodicArrangementCellKey,
>;

type PeriodicSurfaceError = SurfaceArrangementError<
    PeriodicSourceLoopKey,
    PeriodicCutFragmentKey,
    PeriodicArrangementVertexKey,
    PeriodicArrangementCellKey,
>;
type PeriodicArrangementInput = FaceArrangementInput<
    PeriodicSourceLoopKey,
    PeriodicCutFragmentKey,
    PeriodicArrangementVertexKey,
>;
type PeriodicSurfaceEmbedding = CertifiedSurfaceEmbedding<
    PeriodicSourceLoopKey,
    PeriodicCutFragmentKey,
    PeriodicArrangementCellKey,
>;
type PeriodicArrangementInputs = (PeriodicArrangementInput, PeriodicSurfaceEmbedding, usize);
type PeriodicArrangementCycle =
    ArrangementCycle<PeriodicSourceLoopKey, PeriodicCutFragmentKey, PeriodicArrangementVertexKey>;

mod error;
pub(crate) use error::{MixedPeriodicArrangementContractGap, MixedPeriodicArrangementError};
mod face_local;
use face_local::{
    collect_unstitched_fragment_paths, fragment_endpoints, validate_fragment_embedding_endpoints,
};
mod source_span;
pub(crate) use source_span::canonical_source_span_open_interval;

#[derive(Debug, Clone, PartialEq, Eq)]
struct PeriodicFragmentSpec {
    key: PeriodicCutFragmentKey,
    endpoints: [usize; 2],
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct PeriodicComponentSpec {
    component: usize,
    fragments: Vec<PeriodicFragmentSpec>,
    orientation: SectionPeriodicCycleOrientation,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct PeriodicBoundaryRootSpec {
    key: PeriodicSourceRootKey,
    source_loop_ordinal: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct PeriodicBoundaryTraceSpec {
    key: PeriodicBoundaryTraceKey,
    fragments: Vec<PeriodicFragmentSpec>,
    terminals: [PeriodicBoundaryRootSpec; 2],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct PeriodicBoundaryTraceOwner {
    trace_group: usize,
    source_component: Option<usize>,
}

impl PeriodicBoundaryTraceOwner {
    const fn global_component(component: usize) -> Self {
        Self {
            trace_group: component,
            source_component: Some(component),
        }
    }

    const fn face_local(trace_group: usize) -> Self {
        Self {
            trace_group,
            source_component: None,
        }
    }
}

/// Adapt the sealed periodic embedding for one source cylinder face.
pub(crate) fn arrange_mixed_periodic_face(
    graph: &BodySectionGraph,
    face: FaceId,
    operand: usize,
) -> Result<MixedPeriodicFaceArrangement, MixedPeriodicArrangementError> {
    if graph.completion() != SectionCompletion::Complete {
        return Err(MixedPeriodicArrangementError::IncompleteSectionGraph);
    }
    arrange_mixed_periodic_face_from_certified_embedding(graph, face, operand)
}

/// Adapt one sealed periodic embedding after an operation-local theorem has
/// accounted for unrelated global Section gaps.
pub(crate) fn arrange_mixed_periodic_face_from_certified_embedding(
    graph: &BodySectionGraph,
    face: FaceId,
    operand: usize,
) -> Result<MixedPeriodicFaceArrangement, MixedPeriodicArrangementError> {
    if operand >= graph.bodies().len() {
        return Err(MixedPeriodicArrangementError::InvalidOperand(operand));
    }
    let evidence = match select_evidence(graph, &face, operand)? {
        SectionPeriodicFaceEmbeddingEvidence::Certified(evidence) => evidence,
        SectionPeriodicFaceEmbeddingEvidence::Indeterminate { gap, .. } => {
            return Err(MixedPeriodicArrangementError::EmbeddingIndeterminate(
                gap.clone(),
            ));
        }
    };
    let expected = carried_occurrences(graph, &face, operand)?;
    arrange_mixed_periodic_face_with_occurrences(graph, evidence, expected)
}

/// Adapt an operation-local periodic certificate whose fragment identities
/// still address the original public Section graph.
pub(crate) fn arrange_mixed_periodic_face_from_embedding(
    graph: &BodySectionGraph,
    evidence: &crate::CertifiedSectionPeriodicFaceEmbedding,
) -> Result<MixedPeriodicFaceArrangement, MixedPeriodicArrangementError> {
    let face = evidence.face();
    let operand = evidence.operand();
    if operand >= graph.bodies().len() {
        return Err(MixedPeriodicArrangementError::InvalidOperand(operand));
    }
    let expected = carried_occurrences_for_embedding(graph, &face, operand, evidence)?;
    arrange_mixed_periodic_face_with_occurrences(graph, evidence, expected)
}

fn arrange_mixed_periodic_face_with_occurrences(
    graph: &BodySectionGraph,
    evidence: &crate::CertifiedSectionPeriodicFaceEmbedding,
    expected: CarriedOccurrences,
) -> Result<MixedPeriodicFaceArrangement, MixedPeriodicArrangementError> {
    let face = evidence.face();
    let raw_loops = evidence.source_loops().each_ref().map(|loop_id| {
        if loop_id.part() != face.part() {
            return Err(MixedPeriodicArrangementError::SourceLoopPartMismatch(
                loop_id.raw(),
            ));
        }
        Ok(loop_id.raw())
    });
    let [first, second] = raw_loops;
    let source_loops = [first?, second?];
    if source_loops[0] == source_loops[1] {
        return Err(MixedPeriodicArrangementError::DuplicateSourceLoop(
            source_loops[0],
        ));
    }

    let components = adapt_components(graph, evidence.components())?;
    let actual = components
        .iter()
        .map(|component| component.component)
        .collect::<BTreeSet<_>>();
    if let Some(component) = expected.fully_carried.difference(&actual).next() {
        return Err(MixedPeriodicArrangementError::MissingComponentEvidence(
            *component,
        ));
    }
    if let Some(component) = actual.difference(&expected.fully_carried).next() {
        return Err(MixedPeriodicArrangementError::UnexpectedComponentEvidence(
            *component,
        ));
    }

    let (source_roots, traces) = adapt_boundary_traces(
        graph,
        evidence.source_loop_roots(),
        evidence.boundary_traces(),
        &expected.boundary_carried,
    )?;
    if !components.is_empty() && !traces.is_empty() {
        return Err(MixedPeriodicArrangementError::MixedClosedAndBoundaryEvidence);
    }

    let source_directions = evidence.source_loop_windings().map(|winding| {
        if winding.is_positive() {
            ArrangementDirection::Forward
        } else {
            ArrangementDirection::Reverse
        }
    });
    if traces.is_empty() {
        arrange_periodic_spec_with_source_directions(components, source_directions)
    } else {
        arrange_boundary_trace_spec(traces, source_roots, source_directions)
    }
}

fn carried_occurrences_for_embedding(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    evidence: &crate::CertifiedSectionPeriodicFaceEmbedding,
) -> Result<CarriedOccurrences, MixedPeriodicArrangementError> {
    if evidence.components().is_empty()
        && evidence
            .boundary_traces()
            .iter()
            .all(|trace| trace.source_component().is_none())
    {
        let mut boundary_carried = BTreeMap::new();
        for trace in evidence.boundary_traces() {
            let owner = PeriodicBoundaryTraceOwner::face_local(trace.component());
            let mut fragments = BTreeMap::new();
            for (&ordinal, embedded) in trace.component_ordinals().iter().zip(trace.fragments()) {
                let fragment_index = embedded.fragment();
                let fragment = graph.curve_fragments().get(fragment_index).ok_or(
                    MixedPeriodicArrangementError::UnknownFragment {
                        component: trace.component(),
                        fragment: fragment_index,
                    },
                )?;
                let branch = graph.branches().get(fragment.branch()).ok_or(
                    MixedPeriodicArrangementError::UnknownBranch {
                        fragment: fragment_index,
                        branch: fragment.branch(),
                    },
                )?;
                if branch.faces()[operand] != *face
                    || fragments.insert(ordinal, fragment_index).is_some()
                {
                    return Err(MixedPeriodicArrangementError::FaceLocalPathUnavailable(
                        fragment_index,
                    ));
                }
            }
            if boundary_carried.insert(owner, fragments).is_some() {
                return Err(MixedPeriodicArrangementError::DuplicateComponentEvidence(
                    trace.component(),
                ));
            }
        }
        return Ok(CarriedOccurrences {
            fully_carried: BTreeSet::new(),
            boundary_carried,
        });
    }
    carried_occurrences(graph, face, operand)
}

fn select_evidence<'a>(
    graph: &'a BodySectionGraph,
    face: &FaceId,
    operand: usize,
) -> Result<&'a SectionPeriodicFaceEmbeddingEvidence, MixedPeriodicArrangementError> {
    let mut found = None;
    for evidence in graph
        .periodic_face_embeddings()
        .iter()
        .filter(|evidence| evidence.operand() == operand && evidence.face() == *face)
    {
        if found.replace(evidence).is_some() {
            return Err(MixedPeriodicArrangementError::DuplicateEmbeddingEvidence {
                operand,
                face: face.clone(),
            });
        }
    }
    found.ok_or_else(|| MixedPeriodicArrangementError::MissingEmbeddingEvidence {
        operand,
        face: face.clone(),
    })
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct CarriedOccurrences {
    fully_carried: BTreeSet<usize>,
    boundary_carried: BTreeMap<PeriodicBoundaryTraceOwner, BTreeMap<usize, usize>>,
}

fn carried_occurrences(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
) -> Result<CarriedOccurrences, MixedPeriodicArrangementError> {
    let mut fully_carried = BTreeSet::new();
    let mut boundary_carried = BTreeMap::new();
    for (component_index, component) in graph.curve_components().iter().enumerate() {
        let mut carried = BTreeMap::new();
        for (ordinal, &fragment_index) in component.fragments().iter().enumerate() {
            let fragment = graph.curve_fragments().get(fragment_index).ok_or(
                MixedPeriodicArrangementError::UnknownFragment {
                    component: component_index,
                    fragment: fragment_index,
                },
            )?;
            let branch = graph.branches().get(fragment.branch()).ok_or(
                MixedPeriodicArrangementError::UnknownBranch {
                    fragment: fragment_index,
                    branch: fragment.branch(),
                },
            )?;
            if branch.faces()[operand] == *face {
                carried.insert(ordinal, fragment_index);
            }
        }
        if carried.is_empty() {
            continue;
        }
        if carried.len() == component.fragments().len() {
            fully_carried.insert(component_index);
        } else {
            boundary_carried.insert(
                PeriodicBoundaryTraceOwner::global_component(component_index),
                carried,
            );
        }
    }

    let unstitched = collect_unstitched_fragment_paths(graph);
    for (path_index, path) in unstitched.paths.iter().enumerate() {
        let trace_group = graph
            .curve_components()
            .len()
            .checked_add(path_index)
            .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
        let mut carried = BTreeMap::new();
        for (ordinal, &fragment_index) in path.iter().enumerate() {
            let fragment = &graph.curve_fragments()[fragment_index];
            let branch = graph.branches().get(fragment.branch()).ok_or(
                MixedPeriodicArrangementError::UnknownBranch {
                    fragment: fragment_index,
                    branch: fragment.branch(),
                },
            )?;
            if branch.faces()[operand] == *face {
                carried.insert(ordinal, fragment_index);
            }
        }
        if !carried.is_empty() {
            boundary_carried.insert(PeriodicBoundaryTraceOwner::face_local(trace_group), carried);
        }
    }
    for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
        let branch = graph.branches().get(fragment.branch()).ok_or(
            MixedPeriodicArrangementError::UnknownBranch {
                fragment: fragment_index,
                branch: fragment.branch(),
            },
        )?;
        if branch.faces()[operand] == *face && !unstitched.assigned[fragment_index] {
            return Err(MixedPeriodicArrangementError::FaceLocalPathUnavailable(
                fragment_index,
            ));
        }
    }
    Ok(CarriedOccurrences {
        fully_carried,
        boundary_carried,
    })
}

fn adapt_components(
    graph: &BodySectionGraph,
    evidence: &[crate::SectionPeriodicComponentEmbedding],
) -> Result<Vec<PeriodicComponentSpec>, MixedPeriodicArrangementError> {
    let mut seen_components = BTreeSet::new();
    let mut seen_fragments = BTreeSet::new();
    let mut adapted = Vec::with_capacity(evidence.len());
    for component_evidence in evidence {
        let component_index = component_evidence.component();
        if !seen_components.insert(component_index) {
            return Err(MixedPeriodicArrangementError::DuplicateComponentEvidence(
                component_index,
            ));
        }
        let component = graph.curve_components().get(component_index).ok_or(
            MixedPeriodicArrangementError::UnexpectedComponentEvidence(component_index),
        )?;
        validate_component_evidence(component_index, component, component_evidence)?;
        let mut fragments = Vec::with_capacity(component.fragments().len());
        for (ordinal, (&expected, embedded)) in component
            .fragments()
            .iter()
            .zip(component_evidence.fragments())
            .enumerate()
        {
            let actual = embedded.fragment();
            if actual != expected {
                return Err(MixedPeriodicArrangementError::FragmentOrderMismatch {
                    component: component_index,
                    ordinal,
                    expected,
                    actual,
                });
            }
            if !seen_fragments.insert(actual) {
                return Err(MixedPeriodicArrangementError::DuplicateFragment(actual));
            }
            let fragment = graph.curve_fragments().get(actual).ok_or(
                MixedPeriodicArrangementError::UnknownFragment {
                    component: component_index,
                    fragment: actual,
                },
            )?;
            let endpoints = fragment_endpoints(actual, fragment)?;
            validate_fragment_embedding_endpoints(actual, endpoints, embedded)?;
            for endpoint in endpoints {
                if endpoint >= graph.curve_endpoints().len() {
                    return Err(MixedPeriodicArrangementError::UnknownEndpoint {
                        fragment: actual,
                        endpoint,
                    });
                }
            }
            fragments.push(PeriodicFragmentSpec {
                key: PeriodicCutFragmentKey {
                    component: component_index,
                    source_component: Some(component_index),
                    ordinal,
                    fragment: actual,
                    cylinder_period_shift: embedded.period_shift(),
                },
                endpoints,
            });
        }
        validate_component_chain(component_index, &fragments)?;
        adapted.push(PeriodicComponentSpec {
            component: component_index,
            fragments,
            orientation: component_evidence.orientation(),
        });
    }
    Ok(adapted)
}

fn adapt_boundary_traces(
    graph: &BodySectionGraph,
    evidence_roots: &[Vec<crate::SectionPeriodicBoundaryRootEmbedding>; 2],
    evidence_traces: &[crate::SectionPeriodicBoundaryTraceEmbedding],
    expected: &BTreeMap<PeriodicBoundaryTraceOwner, BTreeMap<usize, usize>>,
) -> Result<
    (
        [Vec<PeriodicBoundaryRootSpec>; 2],
        Vec<PeriodicBoundaryTraceSpec>,
    ),
    MixedPeriodicArrangementError,
> {
    let mut expected_root_counts = [0_usize; 2];
    for trace in evidence_traces {
        for terminal in trace.terminals() {
            let Some(count) = expected_root_counts.get_mut(terminal.source_loop_ordinal()) else {
                return Err(MixedPeriodicArrangementError::BoundaryRootCoverageMismatch(
                    terminal.endpoint(),
                ));
            };
            *count = count
                .checked_add(1)
                .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
        }
    }
    let roots = adapt_boundary_roots(graph, evidence_roots, expected_root_counts)?;
    let mut seen_traces = BTreeSet::new();
    let mut covered = BTreeMap::<PeriodicBoundaryTraceOwner, BTreeMap<usize, usize>>::new();
    let mut seen_fragments = BTreeSet::new();
    let mut traces = Vec::with_capacity(evidence_traces.len());
    for evidence in evidence_traces {
        let owner = PeriodicBoundaryTraceOwner {
            trace_group: evidence.component(),
            source_component: evidence.source_component(),
        };
        let Some(&first_component_ordinal) = evidence.component_ordinals().first() else {
            return Err(MixedPeriodicArrangementError::BoundaryTraceEmpty(
                PeriodicBoundaryTraceKey {
                    component: evidence.component(),
                    source_component: evidence.source_component(),
                    first_component_ordinal: 0,
                },
            ));
        };
        let key = PeriodicBoundaryTraceKey {
            component: evidence.component(),
            source_component: evidence.source_component(),
            first_component_ordinal,
        };
        if !seen_traces.insert(key) {
            return Err(MixedPeriodicArrangementError::DuplicateBoundaryTrace(key));
        }
        let Some(expected_ordinals) = expected.get(&owner) else {
            return Err(MixedPeriodicArrangementError::UnexpectedComponentEvidence(
                key.component,
            ));
        };
        if evidence.fragments().is_empty()
            || evidence.fragments().len() != evidence.component_ordinals().len()
        {
            return Err(MixedPeriodicArrangementError::BoundaryTraceEmpty(key));
        }
        let mut fragments = Vec::with_capacity(evidence.fragments().len());
        for (trace_ordinal, (&component_ordinal, embedded)) in evidence
            .component_ordinals()
            .iter()
            .zip(evidence.fragments())
            .enumerate()
        {
            let Some(&expected_fragment) = expected_ordinals.get(&component_ordinal) else {
                return Err(
                    MixedPeriodicArrangementError::BoundaryTraceOrdinalMismatch {
                        trace: key,
                        trace_ordinal,
                        component_ordinal,
                    },
                );
            };
            let actual_fragment = embedded.fragment();
            if expected_fragment != actual_fragment {
                return Err(
                    MixedPeriodicArrangementError::BoundaryTraceFragmentMismatch {
                        trace: key,
                        component_ordinal,
                        expected: expected_fragment,
                        actual: actual_fragment,
                    },
                );
            }
            if !covered
                .entry(owner)
                .or_default()
                .insert(component_ordinal, actual_fragment)
                .is_none()
                || !seen_fragments.insert(actual_fragment)
            {
                return Err(MixedPeriodicArrangementError::DuplicateFragment(
                    actual_fragment,
                ));
            }
            let fragment = graph.curve_fragments().get(actual_fragment).ok_or(
                MixedPeriodicArrangementError::UnknownFragment {
                    component: key.component,
                    fragment: actual_fragment,
                },
            )?;
            let endpoints = fragment_endpoints(actual_fragment, fragment)?;
            validate_fragment_embedding_endpoints(actual_fragment, endpoints, embedded)?;
            for endpoint in endpoints {
                if endpoint >= graph.curve_endpoints().len() {
                    return Err(MixedPeriodicArrangementError::UnknownEndpoint {
                        fragment: actual_fragment,
                        endpoint,
                    });
                }
            }
            fragments.push(PeriodicFragmentSpec {
                key: PeriodicCutFragmentKey {
                    component: key.component,
                    source_component: key.source_component,
                    ordinal: component_ordinal,
                    fragment: actual_fragment,
                    cylinder_period_shift: embedded.period_shift(),
                },
                endpoints,
            });
        }
        validate_trace_chain(key, &fragments)?;
        let terminals = evidence
            .terminals()
            .each_ref()
            .map(|terminal| PeriodicBoundaryRootSpec {
                key: boundary_root_key(terminal),
                source_loop_ordinal: terminal.source_loop_ordinal(),
            });
        let expected_endpoints = [
            fragments[0].endpoints[0],
            fragments[fragments.len() - 1].endpoints[1],
        ];
        for end in 0..2 {
            if terminals[end].key.endpoint != expected_endpoints[end] {
                return Err(
                    MixedPeriodicArrangementError::BoundaryTraceEndpointMismatch {
                        trace: key,
                        expected: expected_endpoints[end],
                        actual: terminals[end].key.endpoint,
                    },
                );
            }
            let loop_ordinal = terminals[end].source_loop_ordinal;
            let root_order = terminals[end].key.cyclic_order;
            let retained = roots
                .get(loop_ordinal)
                .and_then(|values| values.get(root_order));
            if retained != Some(&terminals[end])
                || retained.is_some_and(|root| {
                    root.key.root_parameter_bits != terminals[end].key.root_parameter_bits
                        || root.key.root_enclosure_bits != terminals[end].key.root_enclosure_bits
                })
            {
                return Err(MixedPeriodicArrangementError::BoundaryRootCoverageMismatch(
                    terminals[end].key.endpoint,
                ));
            }
        }
        traces.push(PeriodicBoundaryTraceSpec {
            key,
            fragments,
            terminals,
        });
    }
    for (&owner, ordinals) in expected {
        if covered.get(&owner) != Some(ordinals) {
            return Err(
                MixedPeriodicArrangementError::BoundaryTraceEvidenceRequired(owner.trace_group),
            );
        }
    }
    validate_boundary_trace_matching(&roots, &traces)?;
    Ok((roots, traces))
}

fn adapt_boundary_roots(
    graph: &BodySectionGraph,
    roots: &[Vec<crate::SectionPeriodicBoundaryRootEmbedding>; 2],
    expected_counts: [usize; 2],
) -> Result<[Vec<PeriodicBoundaryRootSpec>; 2], MixedPeriodicArrangementError> {
    let mut seen_endpoints = BTreeSet::new();
    let mut result: [Vec<PeriodicBoundaryRootSpec>; 2] = core::array::from_fn(|_| Vec::new());
    for source_loop in 0..2 {
        if roots[source_loop].len() != expected_counts[source_loop] {
            return Err(MixedPeriodicArrangementError::BoundaryRootCountMismatch {
                source_loop,
                expected: expected_counts[source_loop],
                actual: roots[source_loop].len(),
            });
        }
        for (expected_order, root) in roots[source_loop].iter().enumerate() {
            if root.source_loop_ordinal() != source_loop {
                return Err(MixedPeriodicArrangementError::BoundaryRootLoopMismatch {
                    endpoint: root.endpoint(),
                    expected: source_loop,
                    actual: root.source_loop_ordinal(),
                });
            }
            if root.cyclic_order() != expected_order {
                return Err(MixedPeriodicArrangementError::BoundaryRootOrderMismatch {
                    source_loop,
                    expected: expected_order,
                    actual: root.cyclic_order(),
                });
            }
            if root.endpoint() >= graph.curve_endpoints().len()
                || !seen_endpoints.insert(root.endpoint())
            {
                return Err(MixedPeriodicArrangementError::BoundaryRootCoverageMismatch(
                    root.endpoint(),
                ));
            }
            result[source_loop].push(PeriodicBoundaryRootSpec {
                key: boundary_root_key(root),
                source_loop_ordinal: source_loop,
            });
        }
    }
    Ok(result)
}

fn boundary_root_key(root: &crate::SectionPeriodicBoundaryRootEmbedding) -> PeriodicSourceRootKey {
    let source = root.source_parameter();
    let enclosure = source.root_parameter_enclosure();
    PeriodicSourceRootKey {
        endpoint: root.endpoint(),
        cyclic_order: root.cyclic_order(),
        source_root_ordinal: source.root_ordinal(),
        root_parameter_bits: source.root_parameter().to_bits(),
        root_enclosure_bits: [enclosure.lo().to_bits(), enclosure.hi().to_bits()],
        cylinder_chart_shift: root.cylinder_chart_shift(),
    }
}

fn validate_trace_chain(
    trace: PeriodicBoundaryTraceKey,
    fragments: &[PeriodicFragmentSpec],
) -> Result<(), MixedPeriodicArrangementError> {
    for ordinal in 0..fragments.len().saturating_sub(1) {
        if fragments[ordinal].endpoints[1] != fragments[ordinal + 1].endpoints[0] {
            return Err(
                MixedPeriodicArrangementError::BoundaryTraceEndpointMismatch {
                    trace,
                    expected: fragments[ordinal].endpoints[1],
                    actual: fragments[ordinal + 1].endpoints[0],
                },
            );
        }
    }
    Ok(())
}

fn validate_boundary_trace_matching(
    roots: &[Vec<PeriodicBoundaryRootSpec>; 2],
    traces: &[PeriodicBoundaryTraceSpec],
) -> Result<(), MixedPeriodicArrangementError> {
    let returning = traces
        .iter()
        .filter(|trace| {
            trace.terminals[0].source_loop_ordinal == trace.terminals[1].source_loop_ordinal
        })
        .map(|trace| trace.key)
        .min();
    let transverse = traces
        .iter()
        .filter(|trace| {
            trace.terminals[0].source_loop_ordinal != trace.terminals[1].source_loop_ordinal
        })
        .map(|trace| trace.key)
        .min();
    match (returning, transverse) {
        (None, None) => Ok(()),
        (Some(_), None) => validate_returning_trace_matching(roots, traces),
        (None, Some(_)) => validate_transverse_trace_matching(roots, traces),
        (Some(returning), Some(transverse)) => Err(
            MixedPeriodicArrangementError::MixedBoundaryTraceFamiliesUnsupported {
                returning,
                transverse,
            },
        ),
    }
}

fn validate_returning_trace_matching(
    roots: &[Vec<PeriodicBoundaryRootSpec>; 2],
    traces: &[PeriodicBoundaryTraceSpec],
) -> Result<(), MixedPeriodicArrangementError> {
    let mut used = BTreeSet::new();
    for trace in traces {
        returning_disk_spans(trace, roots)?;
        for terminal in &trace.terminals {
            if !used.insert((terminal.source_loop_ordinal, terminal.key.cyclic_order)) {
                return Err(
                    MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(trace.key),
                );
            }
        }
    }
    if used.len() != roots.iter().map(Vec::len).sum() {
        return Err(MixedPeriodicArrangementError::BoundaryRootCoverageMismatch(
            used.len(),
        ));
    }
    for source_loop in 0..2 {
        let on_loop = traces
            .iter()
            .filter(|trace| trace.terminals[0].source_loop_ordinal == source_loop)
            .collect::<Vec<_>>();
        for (index, first) in on_loop.iter().enumerate() {
            for second in &on_loop[index + 1..] {
                if terminal_pairs_alternate(first, second) {
                    return Err(
                        MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(second.key),
                    );
                }
            }
        }
    }
    Ok(())
}

fn terminal_pairs_alternate(
    first: &PeriodicBoundaryTraceSpec,
    second: &PeriodicBoundaryTraceSpec,
) -> bool {
    let mut first_orders = [
        first.terminals[0].key.cyclic_order,
        first.terminals[1].key.cyclic_order,
    ];
    first_orders.sort_unstable();
    let inside = |order| first_orders[0] < order && order < first_orders[1];
    inside(second.terminals[0].key.cyclic_order) != inside(second.terminals[1].key.cyclic_order)
}

fn validate_transverse_trace_matching(
    roots: &[Vec<PeriodicBoundaryRootSpec>; 2],
    traces: &[PeriodicBoundaryTraceSpec],
) -> Result<(), MixedPeriodicArrangementError> {
    let mut by_first_order = traces.iter().collect::<Vec<_>>();
    by_first_order.sort_unstable_by_key(|trace| {
        trace
            .terminals
            .iter()
            .find(|terminal| terminal.source_loop_ordinal == 0)
            .map(|terminal| terminal.key.cyclic_order)
    });
    for (expected, trace) in by_first_order.iter().enumerate() {
        let actual = trace
            .terminals
            .iter()
            .find(|terminal| terminal.source_loop_ordinal == 0)
            .map(|terminal| terminal.key.cyclic_order);
        if actual != Some(expected) {
            return Err(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(trace.key));
        }
    }
    let first_second_order = by_first_order[0]
        .terminals
        .iter()
        .find(|terminal| terminal.source_loop_ordinal == 1)
        .expect("transverse trace has one terminal on each source loop")
        .key
        .cyclic_order;
    for (offset, trace) in by_first_order.iter().enumerate() {
        let actual = trace
            .terminals
            .iter()
            .find(|terminal| terminal.source_loop_ordinal == 1)
            .expect("transverse trace has one terminal on each source loop")
            .key
            .cyclic_order;
        if actual != (first_second_order + offset) % traces.len() {
            return Err(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(trace.key));
        }
    }
    if roots.iter().any(|source| source.len() != traces.len()) {
        return Err(MixedPeriodicArrangementError::BoundaryRootCoverageMismatch(
            traces.len(),
        ));
    }
    Ok(())
}

/// Select all canonical source-ring spans homotopic to one directed returning
/// trace. The terminal chart-shift delta is the exact universal-cover winding
/// authority; no metric comparison chooses the disk side.
fn returning_disk_spans(
    trace: &PeriodicBoundaryTraceSpec,
    roots: &[Vec<PeriodicBoundaryRootSpec>; 2],
) -> Result<Vec<usize>, MixedPeriodicArrangementError> {
    let source_loop = trace.terminals[0].source_loop_ordinal;
    if source_loop >= roots.len()
        || trace.terminals[1].source_loop_ordinal != source_loop
        || roots[source_loop].len() < 2
    {
        return Err(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(trace.key));
    }
    let start = trace.terminals[0].key;
    let end = trace.terminals[1].key;
    if start.cyclic_order == end.cyclic_order
        || roots[source_loop].get(start.cyclic_order) != Some(&trace.terminals[0])
        || roots[source_loop].get(end.cyclic_order) != Some(&trace.terminals[1])
    {
        return Err(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(trace.key));
    }
    let shift = end
        .cylinder_chart_shift
        .checked_sub(start.cylinder_chart_shift)
        .ok_or(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(trace.key))?;
    let increasing_shift = i64::from(end.cyclic_order < start.cyclic_order);
    let decreasing_shift = -i64::from(start.cyclic_order < end.cyclic_order);
    let (mut span, stop) = if shift == increasing_shift {
        (start.cyclic_order, end.cyclic_order)
    } else if shift == decreasing_shift {
        (end.cyclic_order, start.cyclic_order)
    } else {
        return Err(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(trace.key));
    };
    let mut spans = Vec::new();
    while span != stop {
        spans.push(span);
        span = (span + 1) % roots[source_loop].len();
    }
    Ok(spans)
}

fn validate_component_evidence(
    component_index: usize,
    component: &crate::SectionCurveComponent,
    evidence: &crate::SectionPeriodicComponentEmbedding,
) -> Result<(), MixedPeriodicArrangementError> {
    if !component.closed() {
        return Err(MixedPeriodicArrangementError::OpenComponent(
            component_index,
        ));
    }
    if component.fragments().is_empty() {
        return Err(MixedPeriodicArrangementError::EmptyComponent(
            component_index,
        ));
    }
    if evidence.fragments().len() != component.fragments().len() {
        return Err(MixedPeriodicArrangementError::FragmentCountMismatch {
            component: component_index,
            expected: component.fragments().len(),
            actual: evidence.fragments().len(),
        });
    }
    if evidence.winding() != 0 {
        return Err(MixedPeriodicArrangementError::NonContractibleComponent {
            component: component_index,
            winding: evidence.winding(),
        });
    }
    if let Some(parent) = evidence.parent() {
        return Err(MixedPeriodicArrangementError::NestedComponent {
            component: component_index,
            parent,
        });
    }
    Ok(())
}

fn validate_component_chain(
    component: usize,
    fragments: &[PeriodicFragmentSpec],
) -> Result<(), MixedPeriodicArrangementError> {
    for ordinal in 0..fragments.len() {
        let current = &fragments[ordinal];
        let next = &fragments[(ordinal + 1) % fragments.len()];
        if current.endpoints[1] != next.endpoints[0] {
            return Err(MixedPeriodicArrangementError::ComponentEndpointMismatch {
                component,
                ordinal,
                expected: current.endpoints[1],
                actual: next.endpoints[0],
            });
        }
    }
    Ok(())
}

fn arrange_periodic_spec_with_source_directions(
    mut components: Vec<PeriodicComponentSpec>,
    source_directions: [ArrangementDirection; 2],
) -> Result<MixedPeriodicFaceArrangement, MixedPeriodicArrangementError> {
    if source_directions[0] == source_directions[1] {
        return Err(MixedPeriodicArrangementError::SourceLoopDirectionMismatch);
    }
    components.sort_unstable_by_key(|component| component.component);
    for pair in components.windows(2) {
        if pair[0].component == pair[1].component {
            return Err(MixedPeriodicArrangementError::DuplicateComponentEvidence(
                pair[0].component,
            ));
        }
    }
    for component in &components {
        if component.fragments.is_empty() {
            return Err(MixedPeriodicArrangementError::EmptyComponent(
                component.component,
            ));
        }
        validate_component_chain(component.component, &component.fragments)?;
    }

    let (input, embedding, cut_count) = arrangement_inputs(&components, source_directions)?;
    let arrangement = arrange_bounded_surface(input, embedding)
        .map_err(MixedPeriodicArrangementError::Arrangement)?;
    validate_arrangement_contract(&arrangement, &components, cut_count)?;
    Ok(arrangement)
}

fn arrange_boundary_trace_spec(
    mut traces: Vec<PeriodicBoundaryTraceSpec>,
    roots: [Vec<PeriodicBoundaryRootSpec>; 2],
    source_directions: [ArrangementDirection; 2],
) -> Result<MixedPeriodicFaceArrangement, MixedPeriodicArrangementError> {
    if source_directions[0] == source_directions[1] {
        return Err(MixedPeriodicArrangementError::SourceLoopDirectionMismatch);
    }
    validate_boundary_trace_matching(&roots, &traces)?;
    if traces.iter().all(|trace| {
        trace.terminals[0].source_loop_ordinal == trace.terminals[1].source_loop_ordinal
    }) {
        return arrange_returning_trace_spec(&traces, &roots, source_directions);
    }
    traces.sort_unstable_by_key(|trace| root_on_loop(trace, 0).key.cyclic_order);
    let (input, embedding, cut_count) =
        boundary_trace_arrangement_inputs(&traces, &roots, source_directions)?;
    let arrangement = arrange_bounded_surface(input, embedding)
        .map_err(MixedPeriodicArrangementError::Arrangement)?;
    validate_boundary_trace_contract(&arrangement, &traces, cut_count)?;
    Ok(arrangement)
}

fn arrange_returning_trace_spec(
    traces: &[PeriodicBoundaryTraceSpec],
    roots: &[Vec<PeriodicBoundaryRootSpec>; 2],
    source_directions: [ArrangementDirection; 2],
) -> Result<MixedPeriodicFaceArrangement, MixedPeriodicArrangementError> {
    let (input, embedding, counts) =
        returning_trace_arrangement_inputs(traces, roots, source_directions)?;
    let arrangement = arrange_bounded_surface(input, embedding)
        .map_err(MixedPeriodicArrangementError::Arrangement)?;
    validate_returning_trace_contract(&arrangement, &counts)?;
    Ok(arrangement)
}

struct ReturningTraceCounts {
    source_spans: usize,
    cut_fragments: usize,
    closed_cycles: usize,
    exterior_cycles: usize,
    remainder_boundaries: usize,
    trace_keys: Vec<PeriodicBoundaryTraceKey>,
}

fn returning_trace_arrangement_inputs(
    traces: &[PeriodicBoundaryTraceSpec],
    roots: &[Vec<PeriodicBoundaryRootSpec>; 2],
    source_directions: [ArrangementDirection; 2],
) -> Result<
    (
        PeriodicArrangementInput,
        PeriodicSurfaceEmbedding,
        ReturningTraceCounts,
    ),
    MixedPeriodicArrangementError,
> {
    let mut source_spans = Vec::new();
    let mut cuts = Vec::new();
    let mut rotations = Vec::new();
    for trace in traces {
        append_trace_cuts(trace, &mut cuts, &mut rotations);
    }
    for (source_loop, source_direction) in source_directions.into_iter().enumerate() {
        if roots[source_loop].is_empty() {
            append_whole_source_loop(
                source_loop,
                source_direction,
                &mut source_spans,
                &mut rotations,
            );
        } else {
            append_split_source_loop(
                source_loop,
                source_direction,
                &roots[source_loop],
                &mut source_spans,
            );
        }
    }
    append_trace_terminal_rotations(
        traces,
        roots,
        &source_spans,
        source_directions,
        &mut rotations,
    )?;
    let trace_anchors = traces
        .iter()
        .map(|trace| {
            let source_loop = trace.terminals[0].source_loop_ordinal;
            let spans = returning_disk_spans(trace, roots)?;
            let Some(&span) = spans.first() else {
                return Err(
                    MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(trace.key),
                );
            };
            Ok((
                trace.key,
                ArrangementDartKey::source(
                    source_span_key(&source_spans, source_loop, Some(span)),
                    ArrangementDirection::Forward,
                ),
            ))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let source_span_count = source_spans.len();
    let cut_count = cuts.len();
    let input = FaceArrangementInput::new(source_spans, cuts, rotations);
    let cycles = preview_bounded_surface_cycles(&input).map_err(|error| {
        MixedPeriodicArrangementError::Arrangement(SurfaceArrangementError::Graph(error))
    })?;
    let (embedding, counts) =
        returning_trace_embedding(&cycles, traces, trace_anchors, source_span_count, cut_count)?;
    Ok((input, embedding, counts))
}

fn returning_trace_embedding(
    cycles: &[PeriodicArrangementCycle],
    traces: &[PeriodicBoundaryTraceSpec],
    trace_anchors: Vec<(
        PeriodicBoundaryTraceKey,
        ArrangementDartKey<PeriodicSourceLoopKey, PeriodicCutFragmentKey>,
    )>,
    source_spans: usize,
    cut_fragments: usize,
) -> Result<(PeriodicSurfaceEmbedding, ReturningTraceCounts), MixedPeriodicArrangementError> {
    let gap_key = traces
        .first()
        .ok_or(MixedPeriodicArrangementError::BoundaryRootCoverageMismatch(
            0,
        ))?
        .key;
    let mut assignments = Vec::with_capacity(cycles.len());
    let mut trace_cycles = BTreeSet::new();
    for (key, anchor) in trace_anchors {
        let cycle = cycles
            .iter()
            .position(|cycle| cycle.uses().contains(&anchor))
            .ok_or(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(key))?;
        if !trace_cycles.insert(cycle)
            || cycle_has_reverse_source(&cycles[cycle])
            || returning_cycle_winding(&cycles[cycle], traces)? != 0
        {
            return Err(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(key));
        }
        assignments.push(CertifiedCycleAssignment::new(
            anchor,
            CertifiedCycleSide::Cell(PeriodicArrangementCellKey::TraceCell(key)),
        ));
    }

    let mut exterior_cycles = 0_usize;
    let mut remainder_boundaries = 0_usize;
    for (index, cycle) in cycles.iter().enumerate() {
        if trace_cycles.contains(&index) {
            continue;
        }
        let anchor = cycle
            .uses()
            .first()
            .cloned()
            .ok_or(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(gap_key))?;
        let side = if cycle_has_reverse_source(cycle) {
            exterior_cycles = exterior_cycles
                .checked_add(1)
                .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
            CertifiedCycleSide::Exterior
        } else {
            if returning_cycle_winding(cycle, traces)?.unsigned_abs() != 1 {
                return Err(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(gap_key));
            }
            remainder_boundaries = remainder_boundaries
                .checked_add(1)
                .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
            CertifiedCycleSide::Cell(PeriodicArrangementCellKey::AnnularRemainder)
        };
        assignments.push(CertifiedCycleAssignment::new(anchor, side));
    }
    let boundary_count = i64::try_from(remainder_boundaries)
        .map_err(|_| MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    let mut trace_keys = traces.iter().map(|trace| trace.key).collect::<Vec<_>>();
    trace_keys.sort_unstable();
    let mut cells = Vec::with_capacity(trace_keys.len() + 1);
    cells.push(CertifiedCellTopology::new(
        PeriodicArrangementCellKey::AnnularRemainder,
        2_i64
            .checked_sub(boundary_count)
            .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?,
    ));
    cells.extend(
        trace_keys
            .iter()
            .map(|key| CertifiedCellTopology::new(PeriodicArrangementCellKey::TraceCell(*key), 1)),
    );
    let counts = ReturningTraceCounts {
        source_spans,
        cut_fragments,
        closed_cycles: cycles.len(),
        exterior_cycles,
        remainder_boundaries,
        trace_keys,
    };
    Ok((
        CertifiedSurfaceEmbedding::new(assignments, cells, 0),
        counts,
    ))
}

fn cycle_has_reverse_source(cycle: &PeriodicArrangementCycle) -> bool {
    cycle.uses().iter().any(|use_| {
        matches!(use_.edge(), ArrangementEdgeKey::Source(_))
            && use_.direction() == ArrangementDirection::Reverse
    })
}

fn returning_cycle_winding(
    cycle: &PeriodicArrangementCycle,
    traces: &[PeriodicBoundaryTraceSpec],
) -> Result<i64, MixedPeriodicArrangementError> {
    let mut winding = 0_i64;
    for use_ in cycle.uses() {
        let ArrangementEdgeKey::Source(source) = use_.edge() else {
            continue;
        };
        let forward = match source.terminal_roots {
            Some([start, end]) => Some(match source.source_direction {
                ArrangementDirection::Forward => i64::from(end.cyclic_order < start.cyclic_order),
                ArrangementDirection::Reverse => -i64::from(end.cyclic_order > start.cyclic_order),
            }),
            None if source.is_whole_loop() => Some(match source.source_direction {
                ArrangementDirection::Forward => 1,
                ArrangementDirection::Reverse => -1,
            }),
            None => None,
        }
        .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
        let contribution = if use_.direction() == ArrangementDirection::Forward {
            forward
        } else {
            forward
                .checked_neg()
                .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?
        };
        winding = winding
            .checked_add(contribution)
            .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    }
    for trace in traces {
        let forward =
            ArrangementDartKey::cut(trace.fragments[0].key, ArrangementDirection::Forward);
        let reverse = ArrangementDartKey::cut(
            trace.fragments[trace.fragments.len() - 1].key,
            ArrangementDirection::Reverse,
        );
        let forward_cycle = cycle.uses().contains(&forward);
        let reverse_cycle = cycle.uses().contains(&reverse);
        let touches = cycle.uses().iter().any(|use_| match use_.edge() {
            ArrangementEdgeKey::Cut(key) => {
                trace.fragments.iter().any(|fragment| fragment.key == *key)
            }
            ArrangementEdgeKey::Source(_) => false,
        });
        if forward_cycle == reverse_cycle {
            if touches {
                return Err(
                    MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(trace.key),
                );
            }
            continue;
        }
        let shift = trace.terminals[1]
            .key
            .cylinder_chart_shift
            .checked_sub(trace.terminals[0].key.cylinder_chart_shift)
            .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
        let contribution = if forward_cycle {
            shift
        } else {
            shift
                .checked_neg()
                .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?
        };
        winding = winding
            .checked_add(contribution)
            .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    }
    Ok(winding)
}

fn append_trace_cuts(
    trace: &PeriodicBoundaryTraceSpec,
    cuts: &mut Vec<DirectedCutFragment<PeriodicCutFragmentKey, PeriodicArrangementVertexKey>>,
    rotations: &mut Vec<
        CertifiedEndpointRotation<
            PeriodicSourceLoopKey,
            PeriodicCutFragmentKey,
            PeriodicArrangementVertexKey,
        >,
    >,
) {
    cuts.extend(trace.fragments.iter().map(|fragment| {
        DirectedCutFragment::new(
            fragment.key,
            PeriodicArrangementVertexKey::SectionEndpoint(fragment.endpoints[0]),
            PeriodicArrangementVertexKey::SectionEndpoint(fragment.endpoints[1]),
        )
    }));
    rotations.extend(trace.fragments.windows(2).map(|pair| {
        CertifiedEndpointRotation::new(
            PeriodicArrangementVertexKey::SectionEndpoint(pair[0].endpoints[1]),
            vec![
                ArrangementDartKey::cut(pair[1].key, ArrangementDirection::Forward),
                ArrangementDartKey::cut(pair[0].key, ArrangementDirection::Reverse),
            ],
        )
    }));
}

fn append_split_source_loop(
    source_loop: usize,
    source_direction: ArrangementDirection,
    roots: &[PeriodicBoundaryRootSpec],
    source_spans: &mut Vec<DirectedSourceSpan<PeriodicSourceLoopKey, PeriodicArrangementVertexKey>>,
) {
    for cyclic_span in 0..roots.len() {
        let canonical = [
            roots[cyclic_span].key,
            roots[(cyclic_span + 1) % roots.len()].key,
        ];
        let terminal_roots = if source_direction == ArrangementDirection::Forward {
            canonical
        } else {
            [canonical[1], canonical[0]]
        };
        let key = PeriodicSourceLoopKey {
            topology_ordinal: source_loop,
            source_direction,
            cyclic_span_ordinal: Some(cyclic_span),
            terminal_roots: Some(terminal_roots),
        };
        source_spans.push(DirectedSourceSpan::new(
            key,
            PeriodicArrangementVertexKey::SectionEndpoint(terminal_roots[0].endpoint),
            PeriodicArrangementVertexKey::SectionEndpoint(terminal_roots[1].endpoint),
        ));
    }
}

fn append_whole_source_loop(
    source_loop: usize,
    source_direction: ArrangementDirection,
    source_spans: &mut Vec<DirectedSourceSpan<PeriodicSourceLoopKey, PeriodicArrangementVertexKey>>,
    rotations: &mut Vec<
        CertifiedEndpointRotation<
            PeriodicSourceLoopKey,
            PeriodicCutFragmentKey,
            PeriodicArrangementVertexKey,
        >,
    >,
) -> PeriodicSourceLoopKey {
    let key = PeriodicSourceLoopKey {
        topology_ordinal: source_loop,
        source_direction,
        cyclic_span_ordinal: None,
        terminal_roots: None,
    };
    let seam = PeriodicArrangementVertexKey::SourceLoopSeam(key);
    source_spans.push(DirectedSourceSpan::whole_loop(key, seam));
    rotations.push(CertifiedEndpointRotation::new(
        seam,
        vec![
            ArrangementDartKey::source(key, ArrangementDirection::Forward),
            ArrangementDartKey::source(key, ArrangementDirection::Reverse),
        ],
    ));
    key
}

fn source_span_key(
    source_spans: &[DirectedSourceSpan<PeriodicSourceLoopKey, PeriodicArrangementVertexKey>],
    source_loop: usize,
    cyclic_span: Option<usize>,
) -> PeriodicSourceLoopKey {
    *source_spans
        .iter()
        .find(|span| {
            span.key().topology_ordinal == source_loop
                && span.key().cyclic_span_ordinal == cyclic_span
        })
        .expect("complete source-loop construction retains the requested span")
        .key()
}

fn append_trace_terminal_rotations(
    traces: &[PeriodicBoundaryTraceSpec],
    roots: &[Vec<PeriodicBoundaryRootSpec>; 2],
    source_spans: &[DirectedSourceSpan<PeriodicSourceLoopKey, PeriodicArrangementVertexKey>],
    source_directions: [ArrangementDirection; 2],
    rotations: &mut Vec<
        CertifiedEndpointRotation<
            PeriodicSourceLoopKey,
            PeriodicCutFragmentKey,
            PeriodicArrangementVertexKey,
        >,
    >,
) -> Result<(), MixedPeriodicArrangementError> {
    for source_loop in 0..2 {
        for (root_order, root) in roots[source_loop].iter().enumerate() {
            let (previous_span, current_span) = source_incident_spans(
                source_spans,
                source_loop,
                root_order,
                source_directions[source_loop],
                roots[source_loop].len(),
            );
            let (trace, terminal) = traces
                .iter()
                .find_map(|trace| {
                    trace
                        .terminals
                        .iter()
                        .position(|terminal| terminal.key.endpoint == root.key.endpoint)
                        .map(|terminal| (trace, terminal))
                })
                .ok_or(MixedPeriodicArrangementError::BoundaryRootCoverageMismatch(
                    root.key.endpoint,
                ))?;
            let cut = if terminal == 0 {
                ArrangementDartKey::cut(trace.fragments[0].key, ArrangementDirection::Forward)
            } else {
                ArrangementDartKey::cut(
                    trace.fragments[trace.fragments.len() - 1].key,
                    ArrangementDirection::Reverse,
                )
            };
            rotations.push(CertifiedEndpointRotation::new(
                PeriodicArrangementVertexKey::SectionEndpoint(root.key.endpoint),
                vec![
                    ArrangementDartKey::source(previous_span, ArrangementDirection::Reverse),
                    ArrangementDartKey::source(current_span, ArrangementDirection::Forward),
                    cut,
                ],
            ));
        }
    }
    Ok(())
}

fn boundary_trace_arrangement_inputs(
    traces: &[PeriodicBoundaryTraceSpec],
    roots: &[Vec<PeriodicBoundaryRootSpec>; 2],
    source_directions: [ArrangementDirection; 2],
) -> Result<PeriodicArrangementInputs, MixedPeriodicArrangementError> {
    let trace_count = traces.len();
    let mut source_spans = Vec::with_capacity(2 * trace_count);
    let mut rotations = Vec::with_capacity(2 * trace_count);
    let mut assignments = Vec::with_capacity(trace_count + 2);
    let mut cuts = Vec::new();
    for trace in traces {
        append_trace_cuts(trace, &mut cuts, &mut rotations);
    }

    for (source_loop, source_direction) in source_directions.into_iter().enumerate() {
        for cyclic_span in 0..trace_count {
            let canonical = [
                roots[source_loop][cyclic_span].key,
                roots[source_loop][(cyclic_span + 1) % trace_count].key,
            ];
            let terminal_roots = if source_direction == ArrangementDirection::Forward {
                canonical
            } else {
                [canonical[1], canonical[0]]
            };
            let key = PeriodicSourceLoopKey {
                topology_ordinal: source_loop,
                source_direction,
                cyclic_span_ordinal: Some(cyclic_span),
                terminal_roots: Some(terminal_roots),
            };
            source_spans.push(DirectedSourceSpan::new(
                key,
                PeriodicArrangementVertexKey::SectionEndpoint(terminal_roots[0].endpoint),
                PeriodicArrangementVertexKey::SectionEndpoint(terminal_roots[1].endpoint),
            ));
            if source_loop == 0 {
                assignments.push(CertifiedCycleAssignment::new(
                    ArrangementDartKey::source(key, ArrangementDirection::Forward),
                    CertifiedCycleSide::Cell(PeriodicArrangementCellKey::TraceCell(
                        traces[cyclic_span].key,
                    )),
                ));
            }
        }
        let exterior_key = source_spans
            .iter()
            .find(|span| {
                span.key().topology_ordinal == source_loop
                    && span.key().cyclic_span_ordinal == Some(0)
            })
            .expect("a nonempty trace set splits each source ring")
            .key();
        assignments.push(CertifiedCycleAssignment::new(
            ArrangementDartKey::source(*exterior_key, ArrangementDirection::Reverse),
            CertifiedCycleSide::Exterior,
        ));
    }

    append_trace_terminal_rotations(
        traces,
        roots,
        &source_spans,
        source_directions,
        &mut rotations,
    )?;
    let cells = traces
        .iter()
        .map(|trace| {
            CertifiedCellTopology::new(PeriodicArrangementCellKey::TraceCell(trace.key), 1)
        })
        .collect();
    let cut_count = cuts.len();
    Ok((
        FaceArrangementInput::new(source_spans, cuts, rotations),
        CertifiedSurfaceEmbedding::new(assignments, cells, 0),
        cut_count,
    ))
}

fn source_incident_spans(
    source_spans: &[DirectedSourceSpan<PeriodicSourceLoopKey, PeriodicArrangementVertexKey>],
    source_loop: usize,
    root_order: usize,
    source_direction: ArrangementDirection,
    trace_count: usize,
) -> (PeriodicSourceLoopKey, PeriodicSourceLoopKey) {
    let (previous_ordinal, current_ordinal) = match source_direction {
        ArrangementDirection::Forward => ((root_order + trace_count - 1) % trace_count, root_order),
        ArrangementDirection::Reverse => (root_order, (root_order + trace_count - 1) % trace_count),
    };
    let find = |ordinal| {
        *source_spans
            .iter()
            .find(|span| {
                span.key().topology_ordinal == source_loop
                    && span.key().cyclic_span_ordinal == Some(ordinal)
            })
            .expect("complete cyclic source spans retain every incident interval")
            .key()
    };
    (find(previous_ordinal), find(current_ordinal))
}

fn root_on_loop(
    trace: &PeriodicBoundaryTraceSpec,
    source_loop: usize,
) -> &PeriodicBoundaryRootSpec {
    trace
        .terminals
        .iter()
        .find(|terminal| terminal.source_loop_ordinal == source_loop)
        .expect("a certified transverse trace has one terminal on each loop")
}

fn validate_boundary_trace_contract(
    arrangement: &MixedPeriodicFaceArrangement,
    traces: &[PeriodicBoundaryTraceSpec],
    cut_count: usize,
) -> Result<(), MixedPeriodicArrangementError> {
    if arrangement.cells().len() != traces.len() {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::CellCount,
        ));
    }
    for trace in traces {
        let key = PeriodicArrangementCellKey::TraceCell(trace.key);
        if !arrangement.cells().iter().any(|cell| {
            cell.key() == &key
                && cell.boundaries().len() == 1
                && cell.euler_characteristic() == 1
                && cell.genus() == 0
        }) {
            return Err(MixedPeriodicArrangementError::Contract(
                MixedPeriodicArrangementContractGap::TraceCellTopology(trace.key),
            ));
        }
    }
    let cells_by_root = traces.iter().map(|trace| trace.key).collect::<Vec<_>>();
    if arrangement.adjacency().len() != cut_count {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::Conservation,
        ));
    }
    for adjacency in arrangement.adjacency() {
        let cut = *adjacency.cut();
        let Some(root_order) = traces
            .iter()
            .position(|trace| trace.fragments.iter().any(|fragment| fragment.key == cut))
        else {
            return Err(MixedPeriodicArrangementError::Contract(
                MixedPeriodicArrangementContractGap::TraceCutAdjacency(cut),
            ));
        };
        let expected = BTreeSet::from([
            PeriodicArrangementCellKey::TraceCell(cells_by_root[root_order]),
            PeriodicArrangementCellKey::TraceCell(
                cells_by_root[(root_order + traces.len() - 1) % traces.len()],
            ),
        ]);
        let actual = BTreeSet::from([*adjacency.forward_cell(), *adjacency.reverse_cell()]);
        if actual != expected {
            return Err(MixedPeriodicArrangementError::Contract(
                MixedPeriodicArrangementContractGap::TraceCutAdjacency(cut),
            ));
        }
    }
    validate_boundary_trace_conservation(arrangement, traces.len(), cut_count)
}

fn validate_boundary_trace_conservation(
    arrangement: &MixedPeriodicFaceArrangement,
    trace_count: usize,
    cut_count: usize,
) -> Result<(), MixedPeriodicArrangementError> {
    let source_count = trace_count
        .checked_mul(2)
        .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    let expected_darts = source_count
        .checked_add(cut_count)
        .and_then(|count| count.checked_mul(2))
        .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    let proof = arrangement.proof();
    let valid = proof.directed_darts_conserved() == expected_darts
        && proof.source_spans_conserved() == source_count
        && proof.opposed_cut_pairs() == cut_count
        && proof.closed_cycles() == trace_count + 2
        && proof.exterior_cycles() == 2
        && proof.primal_components() == 1
        && proof.source_boundary_components() == 2
        && proof.cell_genera().len() == trace_count
        && proof.cell_genera().iter().all(|(_, genus)| *genus == 0)
        && proof.dual_connected()
        && proof.surface_euler_characteristic() == 0
        && proof.surface_genus() == 0;
    if !valid {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::Conservation,
        ));
    }
    Ok(())
}

fn validate_returning_trace_contract(
    arrangement: &MixedPeriodicFaceArrangement,
    counts: &ReturningTraceCounts,
) -> Result<(), MixedPeriodicArrangementError> {
    if arrangement.cells().len() != counts.trace_keys.len() + 1 {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::CellCount,
        ));
    }
    for &trace in &counts.trace_keys {
        let key = PeriodicArrangementCellKey::TraceCell(trace);
        if !arrangement.cells().iter().any(|cell| {
            cell.key() == &key
                && cell.boundaries().len() == 1
                && cell.euler_characteristic() == 1
                && cell.genus() == 0
        }) {
            return Err(MixedPeriodicArrangementError::Contract(
                MixedPeriodicArrangementContractGap::TraceCellTopology(trace),
            ));
        }
    }
    let remainder = arrangement
        .cells()
        .iter()
        .find(|cell| cell.key() == &PeriodicArrangementCellKey::AnnularRemainder);
    let remainder_chi = 2_i64
        .checked_sub(
            i64::try_from(counts.remainder_boundaries)
                .map_err(|_| MixedPeriodicArrangementError::TopologyArithmeticOverflow)?,
        )
        .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    if !remainder.is_some_and(|cell| {
        cell.boundaries().len() == counts.remainder_boundaries
            && cell.euler_characteristic() == remainder_chi
            && cell.genus() == 0
    }) {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::RemainderTopology,
        ));
    }
    let expected_darts = counts
        .source_spans
        .checked_add(counts.cut_fragments)
        .and_then(|count| count.checked_mul(2))
        .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    let proof = arrangement.proof();
    let valid = arrangement.adjacency().len() == counts.cut_fragments
        && proof.directed_darts_conserved() == expected_darts
        && proof.source_spans_conserved() == counts.source_spans
        && proof.opposed_cut_pairs() == counts.cut_fragments
        && proof.closed_cycles() == counts.closed_cycles
        && proof.exterior_cycles() == counts.exterior_cycles
        && proof.source_boundary_components() == 2
        && proof.cell_genera().len() == counts.trace_keys.len() + 1
        && proof.cell_genera().iter().all(|(_, genus)| *genus == 0)
        && proof.dual_connected()
        && proof.surface_euler_characteristic() == 0
        && proof.surface_genus() == 0;
    if !valid {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::Conservation,
        ));
    }
    Ok(())
}

#[cfg(test)]
fn arrange_periodic_spec(
    components: Vec<PeriodicComponentSpec>,
) -> Result<MixedPeriodicFaceArrangement, MixedPeriodicArrangementError> {
    arrange_periodic_spec_with_source_directions(
        components,
        [ArrangementDirection::Forward, ArrangementDirection::Reverse],
    )
}

fn arrangement_inputs(
    components: &[PeriodicComponentSpec],
    source_directions: [ArrangementDirection; 2],
) -> Result<PeriodicArrangementInputs, MixedPeriodicArrangementError> {
    let mut source_spans = Vec::with_capacity(2);
    let mut rotations = Vec::new();
    let mut assignments = Vec::new();
    for (topology_ordinal, source_direction) in source_directions.into_iter().enumerate() {
        let source_loop = PeriodicSourceLoopKey {
            topology_ordinal,
            source_direction,
            cyclic_span_ordinal: None,
            terminal_roots: None,
        };
        let seam = PeriodicArrangementVertexKey::SourceLoopSeam(source_loop);
        source_spans.push(DirectedSourceSpan::whole_loop(source_loop, seam));
        rotations.push(CertifiedEndpointRotation::new(
            seam,
            vec![
                ArrangementDartKey::source(source_loop, ArrangementDirection::Forward),
                ArrangementDartKey::source(source_loop, ArrangementDirection::Reverse),
            ],
        ));
        assignments.push(CertifiedCycleAssignment::new(
            ArrangementDartKey::source(source_loop, ArrangementDirection::Forward),
            CertifiedCycleSide::Cell(PeriodicArrangementCellKey::AnnularRemainder),
        ));
        assignments.push(CertifiedCycleAssignment::new(
            ArrangementDartKey::source(source_loop, ArrangementDirection::Reverse),
            CertifiedCycleSide::Exterior,
        ));
    }

    let mut cuts = Vec::new();
    for component in components {
        append_component(component, &mut cuts, &mut rotations, &mut assignments)?;
    }
    let component_count = i64::try_from(components.len())
        .map_err(|_| MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    let mut cells = Vec::with_capacity(components.len() + 1);
    cells.push(CertifiedCellTopology::new(
        PeriodicArrangementCellKey::AnnularRemainder,
        -component_count,
    ));
    cells.extend(components.iter().map(|component| {
        CertifiedCellTopology::new(
            PeriodicArrangementCellKey::ComponentDisk(component.component),
            1,
        )
    }));
    let cut_count = cuts.len();
    Ok((
        FaceArrangementInput::new(source_spans, cuts, rotations),
        CertifiedSurfaceEmbedding::new(assignments, cells, 0),
        cut_count,
    ))
}

fn append_component(
    component: &PeriodicComponentSpec,
    cuts: &mut Vec<DirectedCutFragment<PeriodicCutFragmentKey, PeriodicArrangementVertexKey>>,
    rotations: &mut Vec<
        CertifiedEndpointRotation<
            PeriodicSourceLoopKey,
            PeriodicCutFragmentKey,
            PeriodicArrangementVertexKey,
        >,
    >,
    assignments: &mut Vec<
        CertifiedCycleAssignment<
            PeriodicSourceLoopKey,
            PeriodicCutFragmentKey,
            PeriodicArrangementCellKey,
        >,
    >,
) -> Result<(), MixedPeriodicArrangementError> {
    let first =
        component
            .fragments
            .first()
            .ok_or(MixedPeriodicArrangementError::EmptyComponent(
                component.component,
            ))?;
    for ordinal in 0..component.fragments.len() {
        let fragment = &component.fragments[ordinal];
        let previous = &component.fragments
            [(ordinal + component.fragments.len() - 1) % component.fragments.len()];
        let start = PeriodicArrangementVertexKey::SectionEndpoint(fragment.endpoints[0]);
        let end = PeriodicArrangementVertexKey::SectionEndpoint(fragment.endpoints[1]);
        cuts.push(DirectedCutFragment::new(fragment.key, start, end));
        rotations.push(CertifiedEndpointRotation::new(
            start,
            vec![
                ArrangementDartKey::cut(fragment.key, ArrangementDirection::Forward),
                ArrangementDartKey::cut(previous.key, ArrangementDirection::Reverse),
            ],
        ));
    }

    let disk = PeriodicArrangementCellKey::ComponentDisk(component.component);
    let (forward, reverse) = match component.orientation {
        SectionPeriodicCycleOrientation::Counterclockwise => (
            CertifiedCycleSide::Cell(disk),
            CertifiedCycleSide::Cell(PeriodicArrangementCellKey::AnnularRemainder),
        ),
        SectionPeriodicCycleOrientation::Clockwise => (
            CertifiedCycleSide::Cell(PeriodicArrangementCellKey::AnnularRemainder),
            CertifiedCycleSide::Cell(disk),
        ),
    };
    assignments.push(CertifiedCycleAssignment::new(
        ArrangementDartKey::cut(first.key, ArrangementDirection::Forward),
        forward,
    ));
    assignments.push(CertifiedCycleAssignment::new(
        ArrangementDartKey::cut(first.key, ArrangementDirection::Reverse),
        reverse,
    ));
    Ok(())
}

fn validate_arrangement_contract(
    arrangement: &MixedPeriodicFaceArrangement,
    components: &[PeriodicComponentSpec],
    cut_count: usize,
) -> Result<(), MixedPeriodicArrangementError> {
    if arrangement.cells().len() != components.len() + 1 {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::CellCount,
        ));
    }
    let expected_remainder_chi = i64::try_from(components.len())
        .ok()
        .and_then(i64::checked_neg)
        .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    let Some(remainder) = arrangement
        .cells()
        .iter()
        .find(|cell| cell.key() == &PeriodicArrangementCellKey::AnnularRemainder)
    else {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::RemainderTopology,
        ));
    };
    if remainder.boundaries().len() != components.len() + 2
        || remainder.euler_characteristic() != expected_remainder_chi
        || remainder.genus() != 0
    {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::RemainderTopology,
        ));
    }
    for component in components {
        let disk_key = PeriodicArrangementCellKey::ComponentDisk(component.component);
        let valid = arrangement.cells().iter().any(|cell| {
            cell.key() == &disk_key
                && cell.boundaries().len() == 1
                && cell.euler_characteristic() == 1
                && cell.genus() == 0
        });
        if !valid {
            return Err(MixedPeriodicArrangementError::Contract(
                MixedPeriodicArrangementContractGap::DiskTopology(component.component),
            ));
        }
    }
    validate_cut_adjacency(arrangement, components, cut_count)?;
    validate_conservation(arrangement, components.len(), cut_count)
}

fn validate_cut_adjacency(
    arrangement: &MixedPeriodicFaceArrangement,
    components: &[PeriodicComponentSpec],
    cut_count: usize,
) -> Result<(), MixedPeriodicArrangementError> {
    if arrangement.adjacency().len() != cut_count {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::Conservation,
        ));
    }
    let orientations = components
        .iter()
        .map(|component| (component.component, component.orientation))
        .collect::<BTreeMap<_, _>>();
    for adjacency in arrangement.adjacency() {
        let key = *adjacency.cut();
        let disk = PeriodicArrangementCellKey::ComponentDisk(key.component);
        let root = PeriodicArrangementCellKey::AnnularRemainder;
        let valid = match orientations.get(&key.component) {
            Some(SectionPeriodicCycleOrientation::Counterclockwise) => {
                adjacency.forward_cell() == &disk && adjacency.reverse_cell() == &root
            }
            Some(SectionPeriodicCycleOrientation::Clockwise) => {
                adjacency.forward_cell() == &root && adjacency.reverse_cell() == &disk
            }
            None => false,
        };
        if !valid {
            return Err(MixedPeriodicArrangementError::Contract(
                MixedPeriodicArrangementContractGap::CutAdjacency(key),
            ));
        }
    }
    Ok(())
}

fn validate_conservation(
    arrangement: &MixedPeriodicFaceArrangement,
    component_count: usize,
    cut_count: usize,
) -> Result<(), MixedPeriodicArrangementError> {
    let proof = arrangement.proof();
    let expected_darts = 2_usize
        .checked_mul(
            2_usize
                .checked_add(cut_count)
                .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?,
        )
        .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    let expected_cycles = 2_usize
        .checked_mul(
            2_usize
                .checked_add(component_count)
                .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?,
        )
        .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    let expected_primal = component_count
        .checked_add(2)
        .ok_or(MixedPeriodicArrangementError::TopologyArithmeticOverflow)?;
    let valid = proof.directed_darts_conserved() == expected_darts
        && proof.source_spans_conserved() == 2
        && proof.opposed_cut_pairs() == cut_count
        && proof.closed_cycles() == expected_cycles
        && proof.exterior_cycles() == 2
        && proof.primal_components() == expected_primal
        && proof.source_boundary_components() == 2
        && proof.cell_genera().len() == component_count + 1
        && proof.cell_genera().iter().all(|(_, genus)| *genus == 0)
        && proof.dual_connected()
        && proof.surface_euler_characteristic() == 0
        && proof.surface_genus() == 0;
    if !valid {
        return Err(MixedPeriodicArrangementError::Contract(
            MixedPeriodicArrangementContractGap::Conservation,
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::super::face_arrangement::ArrangementEdgeKey;
    use super::*;
    use crate::{BlockRequest, CylinderRequest, Kernel, SectionBodiesRequest};
    use kgeom::frame::Frame;

    fn triangle(
        component: usize,
        fragment_base: usize,
        endpoint_base: usize,
        orientation: SectionPeriodicCycleOrientation,
    ) -> PeriodicComponentSpec {
        let endpoints = [endpoint_base, endpoint_base + 1, endpoint_base + 2];
        PeriodicComponentSpec {
            component,
            fragments: (0..3)
                .map(|ordinal| PeriodicFragmentSpec {
                    key: PeriodicCutFragmentKey {
                        component,
                        source_component: Some(component),
                        ordinal,
                        fragment: fragment_base + ordinal,
                        cylinder_period_shift: 0,
                    },
                    endpoints: [endpoints[ordinal], endpoints[(ordinal + 1) % 3]],
                })
                .collect(),
            orientation,
        }
    }

    fn transverse_specs(
        trace_count: usize,
        destination_offset: usize,
    ) -> (
        [Vec<PeriodicBoundaryRootSpec>; 2],
        Vec<PeriodicBoundaryTraceSpec>,
    ) {
        let roots = core::array::from_fn(|source_loop| {
            (0..trace_count)
                .map(|cyclic_order| {
                    let endpoint = source_loop * 100 + cyclic_order;
                    PeriodicBoundaryRootSpec {
                        key: PeriodicSourceRootKey {
                            endpoint,
                            cyclic_order,
                            source_root_ordinal: cyclic_order,
                            root_parameter_bits: (cyclic_order as f64).to_bits(),
                            root_enclosure_bits: [
                                (cyclic_order as f64).to_bits(),
                                (cyclic_order as f64).to_bits(),
                            ],
                            cylinder_chart_shift: i64::try_from(source_loop).unwrap(),
                        },
                        source_loop_ordinal: source_loop,
                    }
                })
                .collect::<Vec<_>>()
        });
        let mut traces = (0..trace_count)
            .map(|source_order| {
                let destination_order = (source_order + destination_offset) % trace_count;
                let key = PeriodicBoundaryTraceKey {
                    component: 1000 + source_order,
                    source_component: Some(1000 + source_order),
                    first_component_ordinal: source_order * 3,
                };
                let terminals = if source_order.is_multiple_of(2) {
                    [
                        roots[0][source_order].clone(),
                        roots[1][destination_order].clone(),
                    ]
                } else {
                    [
                        roots[1][destination_order].clone(),
                        roots[0][source_order].clone(),
                    ]
                };
                PeriodicBoundaryTraceSpec {
                    key,
                    fragments: vec![PeriodicFragmentSpec {
                        key: PeriodicCutFragmentKey {
                            component: key.component,
                            source_component: key.source_component,
                            ordinal: key.first_component_ordinal,
                            fragment: 2000 + source_order,
                            cylinder_period_shift: i64::try_from(source_order).unwrap() - 2,
                        },
                        endpoints: [terminals[0].key.endpoint, terminals[1].key.endpoint],
                    }],
                    terminals,
                }
            })
            .collect::<Vec<_>>();
        traces.reverse();
        (roots, traces)
    }

    fn returning_spec(
        source_loop: usize,
        reversed: bool,
        disk_span: usize,
    ) -> (
        [Vec<PeriodicBoundaryRootSpec>; 2],
        PeriodicBoundaryTraceSpec,
    ) {
        let orders = if reversed { [1, 0] } else { [0, 1] };
        let increasing_shift = i64::from(orders[1] < orders[0]);
        let decreasing_shift = -i64::from(orders[0] < orders[1]);
        let trace_shift = if disk_span == orders[0] {
            increasing_shift
        } else {
            decreasing_shift
        };
        let mut chart_shifts = [0_i64; 2];
        chart_shifts[orders[1]] = trace_shift;
        let mut roots: [Vec<PeriodicBoundaryRootSpec>; 2] = core::array::from_fn(|_| Vec::new());
        roots[source_loop] = (0..2)
            .map(|cyclic_order| {
                let parameter = 0.5 + 2.0 * cyclic_order as f64;
                PeriodicBoundaryRootSpec {
                    key: PeriodicSourceRootKey {
                        endpoint: 100 + source_loop * 10 + cyclic_order,
                        cyclic_order,
                        source_root_ordinal: 20 + cyclic_order,
                        root_parameter_bits: parameter.to_bits(),
                        root_enclosure_bits: [parameter.to_bits(), parameter.to_bits()],
                        cylinder_chart_shift: chart_shifts[cyclic_order],
                    },
                    source_loop_ordinal: source_loop,
                }
            })
            .collect();
        let terminals = [
            roots[source_loop][orders[0]].clone(),
            roots[source_loop][orders[1]].clone(),
        ];
        let path = [
            terminals[0].key.endpoint,
            900,
            901,
            terminals[1].key.endpoint,
        ];
        let key = PeriodicBoundaryTraceKey {
            component: 77,
            source_component: Some(77),
            first_component_ordinal: 4,
        };
        let fragments = (0..3)
            .map(|ordinal| PeriodicFragmentSpec {
                key: PeriodicCutFragmentKey {
                    component: key.component,
                    source_component: key.source_component,
                    ordinal: key.first_component_ordinal + ordinal,
                    fragment: 300 + ordinal,
                    cylinder_period_shift: ordinal as i64 - 1,
                },
                endpoints: [path[ordinal], path[ordinal + 1]],
            })
            .collect();
        (
            roots,
            PeriodicBoundaryTraceSpec {
                key,
                fragments,
                terminals,
            },
        )
    }

    #[test]
    fn any_supported_number_of_noncrossing_transverse_traces_obeys_annulus_theorem() {
        for trace_count in 2..=6 {
            for directions in [
                [ArrangementDirection::Forward, ArrangementDirection::Reverse],
                [ArrangementDirection::Reverse, ArrangementDirection::Forward],
            ] {
                let (roots, traces) = transverse_specs(trace_count, trace_count / 2);
                let arrangement = arrange_boundary_trace_spec(traces, roots, directions).unwrap();
                assert_eq!(arrangement.source_spans().len(), trace_count * 2);
                assert_eq!(arrangement.cut_fragments().len(), trace_count);
                assert_eq!(arrangement.cells().len(), trace_count);
                assert_eq!(arrangement.adjacency().len(), trace_count);
                assert_eq!(arrangement.proof().closed_cycles(), trace_count + 2);
                assert_eq!(arrangement.proof().exterior_cycles(), 2);
                assert_eq!(arrangement.proof().primal_components(), 1);
                assert!(arrangement.proof().dual_connected());
                assert!(arrangement.cells().iter().all(|cell| {
                    matches!(cell.key(), PeriodicArrangementCellKey::TraceCell(_))
                        && cell.euler_characteristic() == 1
                        && cell.genus() == 0
                }));
            }
        }
    }

    #[test]
    fn returning_trace_uses_exact_winding_to_split_disk_from_annular_remainder() {
        for source_loop in 0..2 {
            for reversed in [false, true] {
                for disk_span in 0..2 {
                    for directions in [
                        [ArrangementDirection::Forward, ArrangementDirection::Reverse],
                        [ArrangementDirection::Reverse, ArrangementDirection::Forward],
                    ] {
                        let (roots, trace) = returning_spec(source_loop, reversed, disk_span);
                        let trace_key = trace.key;
                        let arrangement =
                            arrange_boundary_trace_spec(vec![trace], roots, directions).unwrap();
                        assert_eq!(arrangement.source_spans().len(), 3);
                        assert_eq!(
                            arrangement
                                .source_spans()
                                .iter()
                                .filter(|span| span.is_whole_loop())
                                .count(),
                            1
                        );
                        assert_eq!(arrangement.cut_fragments().len(), 3);
                        assert_eq!(arrangement.cells().len(), 2);
                        assert_eq!(arrangement.adjacency().len(), 3);
                        assert_eq!(arrangement.proof().closed_cycles(), 5);
                        assert_eq!(arrangement.proof().primal_components(), 2);
                        let disk = arrangement
                            .cells()
                            .iter()
                            .find(|cell| {
                                cell.key() == &PeriodicArrangementCellKey::TraceCell(trace_key)
                            })
                            .unwrap();
                        let actual_disk_span = disk.boundaries()[0]
                            .uses()
                            .iter()
                            .find_map(|use_| match (use_.edge(), use_.direction()) {
                                (
                                    ArrangementEdgeKey::Source(key),
                                    ArrangementDirection::Forward,
                                ) => key.cyclic_span_ordinal(),
                                _ => None,
                            })
                            .unwrap();
                        assert_eq!(actual_disk_span, disk_span);
                        assert!(arrangement.adjacency().iter().all(|adjacency| {
                            BTreeSet::from([*adjacency.forward_cell(), *adjacency.reverse_cell()])
                                == BTreeSet::from([
                                    PeriodicArrangementCellKey::TraceCell(trace_key),
                                    PeriodicArrangementCellKey::AnnularRemainder,
                                ])
                        }));
                    }
                }
            }
        }
    }

    #[test]
    fn returning_trace_without_a_unique_zero_winding_cycle_fails_closed() {
        let (mut roots, mut trace) = returning_spec(0, false, 0);
        let end_order = trace.terminals[1].key.cyclic_order;
        roots[0][end_order].key.cylinder_chart_shift = 2;
        trace.terminals[1] = roots[0][end_order].clone();
        assert_eq!(
            arrange_boundary_trace_spec(
                vec![trace.clone()],
                roots,
                [ArrangementDirection::Forward, ArrangementDirection::Reverse],
            ),
            Err(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(trace.key,))
        );
    }

    #[test]
    fn nonseparating_single_trace_and_crossed_matching_fail_closed() {
        let (single_roots, single_traces) = transverse_specs(1, 0);
        assert!(matches!(
            arrange_boundary_trace_spec(
                single_traces,
                single_roots,
                [ArrangementDirection::Forward, ArrangementDirection::Reverse],
            ),
            Err(MixedPeriodicArrangementError::Arrangement(_))
        ));

        let (roots, mut crossed) = transverse_specs(3, 0);
        crossed[0].terminals[1] = roots[1][1].clone();
        crossed[1].terminals[0] = roots[1][2].clone();
        assert!(matches!(
            arrange_boundary_trace_spec(
                crossed,
                roots,
                [ArrangementDirection::Forward, ArrangementDirection::Reverse],
            ),
            Err(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(_))
        ));
    }

    #[test]
    fn annulus_with_any_number_of_nonnested_contractible_cycles_obeys_euler_theorem() {
        for component_count in 0..=4 {
            let components = (0..component_count)
                .map(|component| {
                    triangle(
                        component * 7 + 2,
                        component * 11,
                        component * 5,
                        if component % 2 == 0 {
                            SectionPeriodicCycleOrientation::Counterclockwise
                        } else {
                            SectionPeriodicCycleOrientation::Clockwise
                        },
                    )
                })
                .collect::<Vec<_>>();
            let cut_count = components.len() * 3;
            let orientations = components
                .iter()
                .map(|component| (component.component, component.orientation))
                .collect::<BTreeMap<_, _>>();
            let arrangement = arrange_periodic_spec(components).unwrap();
            let remainder = arrangement
                .cells()
                .iter()
                .find(|cell| cell.key() == &PeriodicArrangementCellKey::AnnularRemainder)
                .unwrap();
            assert_eq!(remainder.boundaries().len(), component_count + 2);
            assert_eq!(
                remainder.euler_characteristic(),
                -i64::try_from(component_count).unwrap()
            );
            assert_eq!(arrangement.cells().len(), component_count + 1);
            assert_eq!(arrangement.adjacency().len(), cut_count);
            for adjacency in arrangement.adjacency() {
                let key = *adjacency.cut();
                assert!(key.ordinal() < 3);
                assert_eq!(
                    key.fragment(),
                    ((key.component() - 2) / 7) * 11 + key.ordinal()
                );
                let disk = PeriodicArrangementCellKey::ComponentDisk(key.component());
                let remainder = PeriodicArrangementCellKey::AnnularRemainder;
                match orientations[&key.component()] {
                    SectionPeriodicCycleOrientation::Counterclockwise => {
                        assert_eq!(adjacency.forward_cell(), &disk);
                        assert_eq!(adjacency.reverse_cell(), &remainder);
                    }
                    SectionPeriodicCycleOrientation::Clockwise => {
                        assert_eq!(adjacency.forward_cell(), &remainder);
                        assert_eq!(adjacency.reverse_cell(), &disk);
                    }
                }
            }
            assert_eq!(
                arrangement
                    .source_spans()
                    .iter()
                    .map(|source| source.key().topology_ordinal())
                    .collect::<Vec<_>>(),
                vec![0, 1]
            );
            assert!(
                arrangement
                    .source_spans()
                    .iter()
                    .all(DirectedSourceSpan::is_whole_loop)
            );
            assert_eq!(
                arrangement
                    .proof()
                    .endpoint_degrees()
                    .iter()
                    .filter(|(_, degree)| degree.source() == 2 && degree.cut() == 0)
                    .count(),
                2
            );
            assert_eq!(
                arrangement
                    .proof()
                    .endpoint_degrees()
                    .iter()
                    .filter(|(_, degree)| degree.source() == 0 && degree.cut() == 2)
                    .count(),
                cut_count
            );
            assert!(
                arrangement
                    .proof()
                    .endpoint_degrees()
                    .iter()
                    .all(|(_, degree)| degree.total() == 2)
            );
            assert_eq!(arrangement.proof().surface_euler_characteristic(), 0);
            assert_eq!(arrangement.proof().source_boundary_components(), 2);
            assert!(arrangement.proof().dual_connected());
        }
    }

    #[test]
    fn exact_source_winding_controls_each_annular_boundary_direction() {
        for directions in [
            [ArrangementDirection::Forward, ArrangementDirection::Reverse],
            [ArrangementDirection::Reverse, ArrangementDirection::Forward],
        ] {
            let arrangement =
                arrange_periodic_spec_with_source_directions(Vec::new(), directions).unwrap();
            let remainder = arrangement
                .cells()
                .iter()
                .find(|cell| cell.key() == &PeriodicArrangementCellKey::AnnularRemainder)
                .unwrap();
            let mut observed = remainder
                .boundaries()
                .iter()
                .map(|cycle| {
                    let [use_] = cycle.uses() else {
                        panic!("an endpoint-free source ring must have one dart")
                    };
                    let ArrangementEdgeKey::Source(source) = use_.edge() else {
                        panic!("annular source boundary became a cut")
                    };
                    (
                        source.topology_ordinal(),
                        if use_.direction() == source.source_direction() {
                            ArrangementDirection::Forward
                        } else {
                            ArrangementDirection::Reverse
                        },
                    )
                })
                .collect::<Vec<_>>();
            observed.sort_unstable_by_key(|(ordinal, _)| *ordinal);
            assert_eq!(observed, vec![(0, directions[0]), (1, directions[1])]);
        }

        assert_eq!(
            arrange_periodic_spec_with_source_directions(
                Vec::new(),
                [ArrangementDirection::Forward, ArrangementDirection::Forward,],
            ),
            Err(MixedPeriodicArrangementError::SourceLoopDirectionMismatch)
        );
    }

    #[test]
    fn component_permutations_and_cycle_shifts_are_deterministic() {
        let components = vec![
            triangle(
                17,
                20,
                30,
                SectionPeriodicCycleOrientation::Counterclockwise,
            ),
            triangle(4, 50, 60, SectionPeriodicCycleOrientation::Clockwise),
            triangle(
                91,
                80,
                90,
                SectionPeriodicCycleOrientation::Counterclockwise,
            ),
        ];
        let expected = arrange_periodic_spec(components.clone()).unwrap();

        let mut permuted = components;
        permuted.reverse();
        for component in &mut permuted {
            component.fragments.rotate_left(1);
        }
        let actual = arrange_periodic_spec(permuted).unwrap();
        assert_eq!(actual, expected);
    }

    fn public_graph(swapped: bool) -> BodySectionGraph {
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
        let (left, right) = if swapped {
            (cylinder, block)
        } else {
            (block, cylinder)
        };
        session
            .part(part_id)
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(left, right))
            .unwrap()
            .into_result()
            .unwrap()
    }

    fn public_trace_graph(swapped: bool) -> BodySectionGraph {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(
                    Frame::world().with_origin(kgeom::vec::Point3::new(1.5, 0.0, 1.0)),
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
        let (left, right) = if swapped {
            (cylinder, block)
        } else {
            (block, cylinder)
        };
        session
            .part(part_id)
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(left, right))
            .unwrap()
            .into_result()
            .unwrap()
    }

    fn certified_face(graph: &BodySectionGraph) -> (usize, FaceId) {
        let [SectionPeriodicFaceEmbeddingEvidence::Certified(evidence)] =
            graph.periodic_face_embeddings()
        else {
            panic!(
                "fixture must retain one certified periodic face: {:?}",
                graph.periodic_face_embeddings()
            );
        };
        (evidence.operand(), evidence.face())
    }

    type FragmentLineage = (usize, usize);

    #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
    enum CellRole {
        Remainder,
        Disk(Vec<FragmentLineage>),
        Trace(usize, usize),
    }

    fn fragment_lineage(graph: &BodySectionGraph, fragment: usize) -> FragmentLineage {
        let fragment = &graph.curve_fragments()[fragment];
        (fragment.branch(), fragment.source_ordinal())
    }

    fn component_lineage(graph: &BodySectionGraph, component: usize) -> Vec<FragmentLineage> {
        let mut lineage = graph.curve_components()[component]
            .fragments()
            .iter()
            .map(|&fragment| fragment_lineage(graph, fragment))
            .collect::<Vec<_>>();
        lineage.sort_unstable();
        lineage
    }

    fn cell_role(graph: &BodySectionGraph, key: PeriodicArrangementCellKey) -> CellRole {
        match key {
            PeriodicArrangementCellKey::AnnularRemainder => CellRole::Remainder,
            PeriodicArrangementCellKey::ComponentDisk(component) => {
                CellRole::Disk(component_lineage(graph, component))
            }
            PeriodicArrangementCellKey::TraceCell(trace) => {
                CellRole::Trace(trace.component(), trace.first_component_ordinal())
            }
        }
    }

    fn component_orientations(
        graph: &BodySectionGraph,
    ) -> BTreeMap<Vec<FragmentLineage>, SectionPeriodicCycleOrientation> {
        let [SectionPeriodicFaceEmbeddingEvidence::Certified(evidence)] =
            graph.periodic_face_embeddings()
        else {
            panic!("fixture periodic evidence changed after certification")
        };
        evidence
            .components()
            .iter()
            .map(|component| {
                (
                    component_lineage(graph, component.component()),
                    component.orientation(),
                )
            })
            .collect()
    }

    include!("mixed_periodic_arrangement_public_section_tests.rs");

    #[test]
    fn public_boundary_traces_split_both_source_rings_into_exact_disk_cells() {
        for swapped in [false, true] {
            let graph = public_trace_graph(swapped);
            let (operand, face) = certified_face(&graph);
            let arrangement = arrange_mixed_periodic_face(&graph, face, operand).unwrap();
            assert_eq!(arrangement.cells().len(), 2);
            assert_eq!(arrangement.source_spans().len(), 4);
            assert_eq!(arrangement.cut_fragments().len(), 2);
            assert_eq!(arrangement.adjacency().len(), 2);
            assert!(arrangement.source_spans().iter().all(|span| {
                !span.is_whole_loop()
                    && !span.key().is_whole_loop()
                    && span.key().terminal_roots().is_some_and(|roots| {
                        roots[0].endpoint() != roots[1].endpoint()
                            && roots.iter().all(|root| {
                                root.root_parameter().is_finite()
                                    && root.root_enclosure()[0] <= root.root_parameter()
                                    && root.root_parameter() <= root.root_enclosure()[1]
                            })
                    })
            }));
            assert!(arrangement.cut_fragments().iter().all(|fragment| {
                fragment.key().source_component() == Some(fragment.key().component())
                    && fragment.key().cylinder_period_shift().unsigned_abs() <= 1
            }));
            assert!(arrangement.cells().iter().all(|cell| {
                matches!(cell.key(), PeriodicArrangementCellKey::TraceCell(_))
                    && cell.boundaries().len() == 1
                    && cell.euler_characteristic() == 1
            }));
            assert!(arrangement.adjacency().iter().all(|adjacency| {
                matches!(
                    adjacency.forward_cell(),
                    PeriodicArrangementCellKey::TraceCell(_)
                ) && matches!(
                    adjacency.reverse_cell(),
                    PeriodicArrangementCellKey::TraceCell(_)
                ) && adjacency.forward_cell() != adjacency.reverse_cell()
            }));
            assert_eq!(arrangement.proof().source_boundary_components(), 2);
            assert_eq!(arrangement.proof().surface_euler_characteristic(), 0);
            assert_eq!(arrangement.proof().surface_genus(), 0);
            assert!(arrangement.proof().dual_connected());
        }
    }

    #[test]
    fn incomplete_missing_duplicate_and_truncated_evidence_fail_closed() {
        let graph = public_graph(false);
        let (operand, face) = certified_face(&graph);

        let mut incomplete = graph.clone();
        incomplete.completion = SectionCompletion::Indeterminate;
        assert_eq!(
            arrange_mixed_periodic_face(&incomplete, face.clone(), operand),
            Err(MixedPeriodicArrangementError::IncompleteSectionGraph)
        );

        let mut missing = graph.clone();
        missing.periodic_face_embeddings.clear();
        assert_eq!(
            arrange_mixed_periodic_face(&missing, face.clone(), operand),
            Err(MixedPeriodicArrangementError::MissingEmbeddingEvidence {
                operand,
                face: face.clone(),
            })
        );

        let mut duplicate = graph.clone();
        duplicate
            .periodic_face_embeddings
            .push(duplicate.periodic_face_embeddings[0].clone());
        assert_eq!(
            arrange_mixed_periodic_face(&duplicate, face.clone(), operand),
            Err(MixedPeriodicArrangementError::DuplicateEmbeddingEvidence {
                operand,
                face: face.clone(),
            })
        );

        let mut truncated = graph;
        let removed = truncated.curve_fragments.len() - 1;
        truncated.curve_fragments.pop();
        assert!(matches!(
            arrange_mixed_periodic_face(&truncated, face, operand),
            Err(MixedPeriodicArrangementError::UnknownFragment { fragment, .. })
                if fragment == removed
        ));
    }

    #[test]
    fn malformed_distilled_cycles_are_typed_refusals() {
        let valid = triangle(3, 10, 20, SectionPeriodicCycleOrientation::Counterclockwise);
        assert_eq!(
            arrange_periodic_spec(vec![valid.clone(), valid.clone()]),
            Err(MixedPeriodicArrangementError::DuplicateComponentEvidence(3))
        );

        let mut open = valid;
        open.fragments[1].endpoints[0] = 999;
        assert_eq!(
            arrange_periodic_spec(vec![open]),
            Err(MixedPeriodicArrangementError::ComponentEndpointMismatch {
                component: 3,
                ordinal: 0,
                expected: 21,
                actual: 999,
            })
        );
    }
}

#[cfg(test)]
#[path = "mixed_periodic_arrangement_returning_tests.rs"]
mod returning_tests;

#[cfg(test)]
#[path = "mixed_periodic_arrangement_face_local_tests.rs"]
mod face_local_tests;

#[cfg(test)]
#[path = "mixed_periodic_arrangement_bounded_procedural_tests.rs"]
mod bounded_procedural_tests;
