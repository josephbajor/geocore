//! Certified cylinder-side arrangements from periodic section evidence.
//!
//! This adapter admits one representation theorem, not a Boolean layout: the
//! source face is an annulus with two topology-owned whole-loop boundaries,
//! and every section component carried by it is a certified simple,
//! contractible, pairwise-disjoint, nonnested cycle in one lifted cylinder
//! chart. Exact section endpoint indices own incidence. The lifted-cycle
//! orientation owns which directed cut side is the disk. No rounded UV or
//! model-space representative is used for an arrangement decision.

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
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct PeriodicCutFragmentKey {
    component: usize,
    ordinal: usize,
    fragment: usize,
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

    let expected = carried_components(graph, &face, operand)?;
    let components = adapt_components(graph, evidence.components())?;
    let actual = components
        .iter()
        .map(|component| component.component)
        .collect::<BTreeSet<_>>();
    if let Some(component) = expected.difference(&actual).next() {
        return Err(MixedPeriodicArrangementError::MissingComponentEvidence(
            *component,
        ));
    }
    if let Some(component) = actual.difference(&expected).next() {
        return Err(MixedPeriodicArrangementError::UnexpectedComponentEvidence(
            *component,
        ));
    }

    let source_directions = evidence.source_loop_windings().map(|winding| {
        if winding.is_positive() {
            ArrangementDirection::Forward
        } else {
            ArrangementDirection::Reverse
        }
    });
    arrange_periodic_spec_with_source_directions(components, source_directions)
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

fn carried_components(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
) -> Result<BTreeSet<usize>, MixedPeriodicArrangementError> {
    let mut carried_components = BTreeSet::new();
    for (component_index, component) in graph.curve_components().iter().enumerate() {
        let mut carried = 0;
        for &fragment_index in component.fragments() {
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
            carried += usize::from(branch.faces()[operand] == *face);
        }
        if carried == 0 {
            continue;
        }
        if carried != component.fragments().len() {
            return Err(MixedPeriodicArrangementError::ComponentLeavesFace(
                component_index,
            ));
        }
        carried_components.insert(component_index);
    }
    Ok(carried_components)
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
                    },
                    endpoints: [endpoints[ordinal], endpoints[(ordinal + 1) % 3]],
                })
                .collect(),
            orientation,
        }
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
