//! Certified cylinder-side arrangements from periodic section evidence.
//!
//! This adapter admits one representation theorem, not a Boolean layout: the
//! source face is an annulus with two topology-owned whole-loop boundaries,
//! and the section evidence is either a set of certified simple,
//! contractible, pairwise-disjoint, nonnested cycles or a complete set of
//! noncrossing boundary-to-boundary traces. Exact section endpoint indices
//! own incidence. Exact root order and integer chart shifts own bounded ring
//! spans. No rounded UV or model-space representative is used for an
//! arrangement decision.

use std::collections::{BTreeMap, BTreeSet};

use ktopo::entity::LoopId as RawLoopId;

use super::face_arrangement::{
    ArrangementDartKey, ArrangementDirection, CertifiedCellTopology, CertifiedCycleAssignment,
    CertifiedCycleSide, CertifiedEndpointRotation, CertifiedSurfaceEmbedding, DirectedCutFragment,
    DirectedSourceSpan, FaceArrangementInput, SurfaceArrangementError, SurfaceFaceArrangement,
    arrange_bounded_surface,
};
use crate::{
    BodySectionGraph, FaceId, SectionCompletion, SectionCurveFragment, SectionCurveFragmentSpan,
    SectionPeriodicCycleOrientation, SectionPeriodicEmbeddingGap,
    SectionPeriodicFaceEmbeddingEvidence,
};

/// Exact identity of one directed section fragment on the periodic face.
#[derive(Debug, Clone, Copy)]
pub(crate) struct PeriodicCutFragmentKey {
    component: usize,
    ordinal: usize,
    fragment: usize,
    cylinder_period_shift: i64,
}

impl PartialEq for PeriodicCutFragmentKey {
    fn eq(&self, other: &Self) -> bool {
        (self.component, self.ordinal, self.fragment)
            == (other.component, other.ordinal, other.fragment)
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
        (self.component, self.ordinal, self.fragment).cmp(&(
            other.component,
            other.ordinal,
            other.fragment,
        ))
    }
}

impl PeriodicCutFragmentKey {
    pub(crate) const fn component(self) -> usize {
        self.component
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
    first_component_ordinal: usize,
}

impl PeriodicBoundaryTraceKey {
    pub(crate) const fn component(self) -> usize {
        self.component
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

/// Postcondition whose violation would contradict the admitted theorem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MixedPeriodicArrangementContractGap {
    CellCount,
    RemainderTopology,
    DiskTopology(usize),
    CutAdjacency(PeriodicCutFragmentKey),
    TraceCellTopology(PeriodicBoundaryTraceKey),
    TraceCutAdjacency(PeriodicCutFragmentKey),
    Conservation,
}

/// Typed refusal at the periodic-evidence/arrangement boundary.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum MixedPeriodicArrangementError {
    InvalidOperand(usize),
    IncompleteSectionGraph,
    MissingEmbeddingEvidence {
        operand: usize,
        face: FaceId,
    },
    DuplicateEmbeddingEvidence {
        operand: usize,
        face: FaceId,
    },
    EmbeddingIndeterminate(SectionPeriodicEmbeddingGap),
    SourceLoopPartMismatch(RawLoopId),
    DuplicateSourceLoop(RawLoopId),
    SourceLoopDirectionMismatch,
    MixedClosedAndBoundaryEvidence,
    /// One transverse cut is nonseparating on an annulus, so its two darts
    /// border the same disk cell. The current surface-arrangement adjacency
    /// contract requires distinct cells and cannot express that topology.
    SingleBoundaryTraceUnsupported,
    BoundaryTraceEvidenceRequired(usize),
    BoundaryRootCountMismatch {
        source_loop: usize,
        expected: usize,
        actual: usize,
    },
    BoundaryRootOrderMismatch {
        source_loop: usize,
        expected: usize,
        actual: usize,
    },
    BoundaryRootLoopMismatch {
        endpoint: usize,
        expected: usize,
        actual: usize,
    },
    BoundaryRootCoverageMismatch(usize),
    DuplicateBoundaryTrace(PeriodicBoundaryTraceKey),
    BoundaryTraceEmpty(PeriodicBoundaryTraceKey),
    BoundaryTraceOrdinalMismatch {
        trace: PeriodicBoundaryTraceKey,
        trace_ordinal: usize,
        component_ordinal: usize,
    },
    BoundaryTraceFragmentMismatch {
        trace: PeriodicBoundaryTraceKey,
        component_ordinal: usize,
        expected: usize,
        actual: usize,
    },
    BoundaryTraceEndpointMismatch {
        trace: PeriodicBoundaryTraceKey,
        expected: usize,
        actual: usize,
    },
    BoundaryTraceNotTransverse(PeriodicBoundaryTraceKey),
    BoundaryTraceMatchingMismatch(PeriodicBoundaryTraceKey),
    UnknownBranch {
        fragment: usize,
        branch: usize,
    },
    UnknownFragment {
        component: usize,
        fragment: usize,
    },
    ComponentLeavesFace(usize),
    MissingComponentEvidence(usize),
    UnexpectedComponentEvidence(usize),
    DuplicateComponentEvidence(usize),
    OpenComponent(usize),
    EmptyComponent(usize),
    NonContractibleComponent {
        component: usize,
        winding: i64,
    },
    NestedComponent {
        component: usize,
        parent: usize,
    },
    FragmentCountMismatch {
        component: usize,
        expected: usize,
        actual: usize,
    },
    FragmentOrderMismatch {
        component: usize,
        ordinal: usize,
        expected: usize,
        actual: usize,
    },
    DuplicateFragment(usize),
    WholeFragment(usize),
    UnknownEndpoint {
        fragment: usize,
        endpoint: usize,
    },
    ComponentEndpointMismatch {
        component: usize,
        ordinal: usize,
        expected: usize,
        actual: usize,
    },
    TopologyArithmeticOverflow,
    Arrangement(PeriodicSurfaceError),
    Contract(MixedPeriodicArrangementContractGap),
}

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

/// Adapt the sealed periodic embedding for one source cylinder face.
pub(crate) fn arrange_mixed_periodic_face(
    graph: &BodySectionGraph,
    face: FaceId,
    operand: usize,
) -> Result<MixedPeriodicFaceArrangement, MixedPeriodicArrangementError> {
    if operand >= graph.bodies().len() {
        return Err(MixedPeriodicArrangementError::InvalidOperand(operand));
    }
    if graph.completion() != SectionCompletion::Complete {
        return Err(MixedPeriodicArrangementError::IncompleteSectionGraph);
    }

    let evidence = match select_evidence(graph, &face, operand)? {
        SectionPeriodicFaceEmbeddingEvidence::Certified(evidence) => evidence,
        SectionPeriodicFaceEmbeddingEvidence::Indeterminate { gap, .. } => {
            return Err(MixedPeriodicArrangementError::EmbeddingIndeterminate(
                gap.clone(),
            ));
        }
    };
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

    let expected = carried_occurrences(graph, &face, operand)?;
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
        &expected.partially_carried,
    )?;
    if !components.is_empty() && !traces.is_empty() {
        return Err(MixedPeriodicArrangementError::MixedClosedAndBoundaryEvidence);
    }
    if let Some((&component, _)) = expected.partially_carried.iter().find(|(component, _)| {
        !traces
            .iter()
            .any(|trace| trace.key.component == **component)
    }) {
        return Err(MixedPeriodicArrangementError::BoundaryTraceEvidenceRequired(component));
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
    partially_carried: BTreeMap<usize, BTreeSet<usize>>,
}

fn carried_occurrences(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
) -> Result<CarriedOccurrences, MixedPeriodicArrangementError> {
    let mut fully_carried = BTreeSet::new();
    let mut partially_carried = BTreeMap::new();
    for (component_index, component) in graph.curve_components().iter().enumerate() {
        let mut carried = BTreeSet::new();
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
                carried.insert(ordinal);
            }
        }
        if carried.is_empty() {
            continue;
        }
        if carried.len() == component.fragments().len() {
            fully_carried.insert(component_index);
        } else {
            partially_carried.insert(component_index, carried);
        }
    }
    Ok(CarriedOccurrences {
        fully_carried,
        partially_carried,
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
    expected: &BTreeMap<usize, BTreeSet<usize>>,
) -> Result<
    (
        [Vec<PeriodicBoundaryRootSpec>; 2],
        Vec<PeriodicBoundaryTraceSpec>,
    ),
    MixedPeriodicArrangementError,
> {
    let roots = adapt_boundary_roots(graph, evidence_roots, evidence_traces.len())?;
    let mut seen_traces = BTreeSet::new();
    let mut covered = BTreeMap::<usize, BTreeSet<usize>>::new();
    let mut seen_fragments = BTreeSet::new();
    let mut traces = Vec::with_capacity(evidence_traces.len());
    for evidence in evidence_traces {
        let Some(&first_component_ordinal) = evidence.component_ordinals().first() else {
            return Err(MixedPeriodicArrangementError::BoundaryTraceEmpty(
                PeriodicBoundaryTraceKey {
                    component: evidence.component(),
                    first_component_ordinal: 0,
                },
            ));
        };
        let key = PeriodicBoundaryTraceKey {
            component: evidence.component(),
            first_component_ordinal,
        };
        if !seen_traces.insert(key) {
            return Err(MixedPeriodicArrangementError::DuplicateBoundaryTrace(key));
        }
        let component = graph.curve_components().get(key.component).ok_or(
            MixedPeriodicArrangementError::UnexpectedComponentEvidence(key.component),
        )?;
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
            let Some(&expected_fragment) = component.fragments().get(component_ordinal) else {
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
            if !expected
                .get(&key.component)
                .is_some_and(|ordinals| ordinals.contains(&component_ordinal))
            {
                return Err(MixedPeriodicArrangementError::UnexpectedComponentEvidence(
                    key.component,
                ));
            }
            if !covered
                .entry(key.component)
                .or_default()
                .insert(component_ordinal)
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
        if terminals[0].source_loop_ordinal == terminals[1].source_loop_ordinal {
            return Err(MixedPeriodicArrangementError::BoundaryTraceNotTransverse(
                key,
            ));
        }
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
    for (&component, ordinals) in expected {
        if covered.get(&component) != Some(ordinals) {
            return Err(MixedPeriodicArrangementError::BoundaryTraceEvidenceRequired(component));
        }
    }
    validate_boundary_trace_matching(&roots, &traces)?;
    Ok((roots, traces))
}

fn adapt_boundary_roots(
    graph: &BodySectionGraph,
    roots: &[Vec<crate::SectionPeriodicBoundaryRootEmbedding>; 2],
    trace_count: usize,
) -> Result<[Vec<PeriodicBoundaryRootSpec>; 2], MixedPeriodicArrangementError> {
    let mut seen_endpoints = BTreeSet::new();
    let mut result: [Vec<PeriodicBoundaryRootSpec>; 2] = core::array::from_fn(|_| Vec::new());
    for source_loop in 0..2 {
        if roots[source_loop].len() != trace_count {
            return Err(MixedPeriodicArrangementError::BoundaryRootCountMismatch {
                source_loop,
                expected: trace_count,
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
    if traces.is_empty() {
        return Ok(());
    }
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

fn fragment_endpoints(
    fragment_index: usize,
    fragment: &SectionCurveFragment,
) -> Result<[usize; 2], MixedPeriodicArrangementError> {
    match fragment.span() {
        SectionCurveFragmentSpan::Whole => {
            Err(MixedPeriodicArrangementError::WholeFragment(fragment_index))
        }
        SectionCurveFragmentSpan::Arc { endpoints, .. } => {
            Ok(endpoints.each_ref().map(|endpoint| endpoint.endpoint()))
        }
        SectionCurveFragmentSpan::LineSegment { endpoints } => {
            Ok(endpoints.each_ref().map(|endpoint| endpoint.endpoint()))
        }
    }
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
    if traces.len() == 1 {
        return Err(MixedPeriodicArrangementError::SingleBoundaryTraceUnsupported);
    }
    validate_boundary_trace_matching(&roots, &traces)?;
    traces.sort_unstable_by_key(|trace| root_on_loop(trace, 0).key.cyclic_order);
    let (input, embedding, cut_count) =
        boundary_trace_arrangement_inputs(&traces, &roots, source_directions)?;
    let arrangement = arrange_bounded_surface(input, embedding)
        .map_err(MixedPeriodicArrangementError::Arrangement)?;
    validate_boundary_trace_contract(&arrangement, &traces, cut_count)?;
    Ok(arrangement)
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
        for fragment in &trace.fragments {
            cuts.push(DirectedCutFragment::new(
                fragment.key,
                PeriodicArrangementVertexKey::SectionEndpoint(fragment.endpoints[0]),
                PeriodicArrangementVertexKey::SectionEndpoint(fragment.endpoints[1]),
            ));
        }
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

    for source_loop in 0..2 {
        for root_order in 0..trace_count {
            let root = roots[source_loop][root_order].key;
            let (previous_span, current_span) = source_incident_spans(
                &source_spans,
                source_loop,
                root_order,
                source_directions[source_loop],
                trace_count,
            );
            let trace = traces
                .iter()
                .find(|trace| root_on_loop(trace, source_loop).key.endpoint == root.endpoint)
                .ok_or(MixedPeriodicArrangementError::BoundaryRootCoverageMismatch(
                    root.endpoint,
                ))?;
            let terminal = trace
                .terminals
                .iter()
                .position(|terminal| terminal.key.endpoint == root.endpoint)
                .expect("the root-selected trace retains that terminal");
            let cut = if terminal == 0 {
                ArrangementDartKey::cut(trace.fragments[0].key, ArrangementDirection::Forward)
            } else {
                ArrangementDartKey::cut(
                    trace.fragments[trace.fragments.len() - 1].key,
                    ArrangementDirection::Reverse,
                )
            };
            rotations.push(CertifiedEndpointRotation::new(
                PeriodicArrangementVertexKey::SectionEndpoint(root.endpoint),
                vec![
                    ArrangementDartKey::source(previous_span, ArrangementDirection::Reverse),
                    ArrangementDartKey::source(current_span, ArrangementDirection::Forward),
                    cut,
                ],
            ));
        }
    }
    for trace in traces {
        for pair in trace.fragments.windows(2) {
            rotations.push(CertifiedEndpointRotation::new(
                PeriodicArrangementVertexKey::SectionEndpoint(pair[0].endpoints[1]),
                vec![
                    ArrangementDartKey::cut(pair[1].key, ArrangementDirection::Forward),
                    ArrangementDartKey::cut(pair[0].key, ArrangementDirection::Reverse),
                ],
            ));
        }
    }
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
    fn nonseparating_single_trace_and_crossed_matching_fail_closed() {
        let (single_roots, single_traces) = transverse_specs(1, 0);
        assert_eq!(
            arrange_boundary_trace_spec(
                single_traces,
                single_roots,
                [ArrangementDirection::Forward, ArrangementDirection::Reverse],
            ),
            Err(MixedPeriodicArrangementError::SingleBoundaryTraceUnsupported)
        );

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

    #[test]
    fn public_section_evidence_adapts_deterministically_in_both_operand_orders() {
        let first_graph = public_graph(false);
        let (first_operand, first_face) = certified_face(&first_graph);
        let first =
            arrange_mixed_periodic_face(&first_graph, first_face.clone(), first_operand).unwrap();
        assert_eq!(
            arrange_mixed_periodic_face(&first_graph, first_face, first_operand).unwrap(),
            first
        );

        let second_graph = public_graph(true);
        let (second_operand, second_face) = certified_face(&second_graph);
        let second =
            arrange_mixed_periodic_face(&second_graph, second_face.clone(), second_operand)
                .unwrap();
        assert_eq!(
            arrange_mixed_periodic_face(&second_graph, second_face, second_operand).unwrap(),
            second
        );

        // Swapping operands reverses the section carrier convention and can
        // therefore exchange forward/reverse cut sides. Compare graph-local
        // identities only through stable branch/source-ordinal lineage.
        assert_eq!(first.source_spans(), second.source_spans());
        let proof_signature = |arrangement: &MixedPeriodicFaceArrangement| {
            let proof = arrangement.proof();
            let mut degrees = proof
                .endpoint_degrees()
                .iter()
                .map(|(_, degree)| (degree.source(), degree.cut()))
                .collect::<Vec<_>>();
            degrees.sort_unstable();
            (
                degrees,
                proof.directed_darts_conserved(),
                proof.source_spans_conserved(),
                proof.opposed_cut_pairs(),
                proof.closed_cycles(),
                proof.exterior_cycles(),
                proof.primal_components(),
                proof.source_boundary_components(),
                proof.dual_connected(),
                proof.surface_euler_characteristic(),
                proof.surface_genus(),
            )
        };
        assert_eq!(proof_signature(&first), proof_signature(&second));
        let cell_signature =
            |graph: &BodySectionGraph, arrangement: &MixedPeriodicFaceArrangement| {
                arrangement
                    .cells()
                    .iter()
                    .map(|cell| {
                        (
                            cell_role(graph, *cell.key()),
                            cell.boundaries().len(),
                            cell.euler_characteristic(),
                            cell.genus(),
                        )
                    })
                    .collect::<BTreeSet<_>>()
            };
        assert_eq!(
            cell_signature(&first_graph, &first),
            cell_signature(&second_graph, &second)
        );

        let first_orientations = component_orientations(&first_graph);
        let second_orientations = component_orientations(&second_graph);
        assert_eq!(first_orientations.len(), second_orientations.len());
        let second_adjacency = second
            .adjacency()
            .iter()
            .map(|adjacency| {
                (
                    fragment_lineage(&second_graph, adjacency.cut().fragment()),
                    adjacency,
                )
            })
            .collect::<BTreeMap<_, _>>();
        for first_edge in first.adjacency() {
            let lineage = fragment_lineage(&first_graph, first_edge.cut().fragment());
            let second_edge = second_adjacency[&lineage];
            let component = component_lineage(&first_graph, first_edge.cut().component());
            let first_sides = [
                cell_role(&first_graph, *first_edge.forward_cell()),
                cell_role(&first_graph, *first_edge.reverse_cell()),
            ];
            let second_sides = [
                cell_role(&second_graph, *second_edge.forward_cell()),
                cell_role(&second_graph, *second_edge.reverse_cell()),
            ];
            if first_orientations[&component] == second_orientations[&component] {
                assert_eq!(first_sides, second_sides);
            } else {
                assert_eq!(
                    first_sides,
                    [second_sides[1].clone(), second_sides[0].clone()]
                );
            }
        }
        assert_eq!(first.cells().len(), 3);
        assert_eq!(first.proof().surface_euler_characteristic(), 0);
        assert_eq!(first.proof().surface_genus(), 0);
        assert!(first.proof().dual_connected());
        assert_eq!(first.proof().opposed_cut_pairs(), 8);
    }

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
            assert!(
                arrangement
                    .cut_fragments()
                    .iter()
                    .all(|fragment| { fragment.key().cylinder_period_shift().unsigned_abs() <= 1 })
            );
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
