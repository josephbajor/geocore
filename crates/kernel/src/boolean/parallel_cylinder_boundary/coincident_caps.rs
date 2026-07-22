//! Proof-fed boundary cells for strict cylinder lenses with shared cap planes.
//!
//! The operation-local relation has already accounted for every global
//! Section gap.  Each cylinder side is therefore arranged only from its
//! sealed periodic embedding.  Cap cells retain the two physical boundary
//! pieces induced at an overlap end: either two source-ring arcs, or one
//! source-ring arc and one ordinary Section arc.  No coincident SSI circle is
//! synthesized.

use std::collections::BTreeMap;

use ktopo::entity::EdgeId as RawEdgeId;

use super::*;
use crate::boolean::boundary_select::{
    BoundaryFragmentClassification, BoundarySelectionError, CoincidentBoundaryPairEvidence,
    CoincidentSourceOrientation, OperandSide, RegularizedBooleanOperation,
    SelectedBoundaryFragment, SelectedOrientation, select_boundary_fragments_with_coincident_pairs,
};
use crate::boolean::face_arrangement::ArrangementEdgeKey;
use crate::boolean::mixed_boundary::{
    classify_periodic_face_from_source_point, periodic_source_span_point,
};
use crate::boolean::mixed_cap_boundary::bind_cylinder_cap_ring_from_embedding;
use crate::boolean::mixed_periodic_arrangement::{
    MixedPeriodicArrangementError, PeriodicArrangementCellKey, PeriodicSourceLoopKey,
    PeriodicSourceRootKey, arrange_mixed_periodic_face_from_embedding,
};
use crate::boolean::mixed_shell_plan::MixedSourceFaceKey;
use crate::boolean::parallel_cylinder_relation::CertifiedParallelCylinderCoincidentCapRelation;
use crate::boolean::parallel_cylinder_relation::ParallelCylinderSourceRootWitness;
use crate::section::{certify_periodic_face_fragment_subset, periodic_face_fragment_subset_work};
use crate::{
    CertifiedSectionPeriodicFaceEmbedding, SectionCarrier, SectionCompletion,
    SectionCurveFragmentSpan,
};

/// Canonical selector key for an arranged side cell or one physical cap end.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum ParallelCoincidentBoundaryKey {
    Arranged(MixedShellCellKey),
    CapEnd(usize),
    CapRemainder(usize),
}

/// One exact physical boundary piece of a derived planar cap cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CoincidentCapBoundaryPiece {
    SourceArc {
        operand: usize,
        edge: RawEdgeId,
        roots: [ParallelCylinderSourceRootWitness; 2],
    },
    SectionArc {
        fragment: usize,
        endpoints: [usize; 2],
    },
}

/// Relation-bound derived cap cell awaiting canonical truth selection.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct PreparedCoincidentCapCell {
    physical_end: usize,
    target_operand: usize,
    target_boundary: usize,
    target_source: MixedSourceFaceKey,
    target_face: FaceId,
    boundary: [CoincidentCapBoundaryPiece; 2],
}

impl PreparedCoincidentCapCell {
    pub(crate) const fn physical_end(&self) -> usize {
        self.physical_end
    }

    pub(crate) const fn target_operand(&self) -> usize {
        self.target_operand
    }

    pub(crate) const fn target_boundary(&self) -> usize {
        self.target_boundary
    }

    pub(crate) const fn target_source(&self) -> MixedSourceFaceKey {
        self.target_source
    }

    pub(crate) const fn target_face(&self) -> &FaceId {
        &self.target_face
    }

    pub(crate) const fn boundary(&self) -> &[CoincidentCapBoundaryPiece; 2] {
        &self.boundary
    }
}

/// Opaque payload preserved through representation-independent selection.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum ParallelCoincidentBoundaryPayload {
    Arranged,
    Cap(PreparedCoincidentCapCell),
}

/// Exact selected side use that bounds one coalesced planar cap.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SelectedCoincidentCapBoundaryUse {
    SourceSpan {
        operand: usize,
        edge: RawEdgeId,
        roots: [ParallelCylinderSourceRootWitness; 2],
        side_cell: MixedShellCellKey,
        span: PeriodicSourceLoopKey,
        side_orientation: SelectedOrientation,
    },
    SectionArc {
        fragment: usize,
        endpoints: [usize; 2],
        side_cell: MixedShellCellKey,
        side_orientation: SelectedOrientation,
    },
}

/// One canonical operation-selected planar cap and its exact side boundary.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SelectedCoincidentCapPlan {
    owner_key: ParallelCoincidentBoundaryKey,
    target: PreparedCoincidentCapCell,
    orientation: SelectedOrientation,
    boundary: [SelectedCoincidentCapBoundaryUse; 2],
}

impl SelectedCoincidentCapPlan {
    pub(crate) const fn owner_key(&self) -> ParallelCoincidentBoundaryKey {
        self.owner_key
    }

    pub(crate) const fn target(&self) -> &PreparedCoincidentCapCell {
        &self.target
    }

    pub(crate) const fn orientation(&self) -> SelectedOrientation {
        self.orientation
    }

    pub(crate) const fn boundary(&self) -> &[SelectedCoincidentCapBoundaryUse; 2] {
        &self.boundary
    }
}

/// Canonical ordinary cells and one coalesced cap plan per physical end.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SelectedParallelCylinderCoincidentBoundary {
    arranged: Vec<
        SelectedBoundaryFragment<ParallelCoincidentBoundaryKey, ParallelCoincidentBoundaryPayload>,
    >,
    caps: Vec<SelectedCoincidentCapPlan>,
}

impl SelectedParallelCylinderCoincidentBoundary {
    pub(crate) fn arranged(
        &self,
    ) -> &[SelectedBoundaryFragment<
        ParallelCoincidentBoundaryKey,
        ParallelCoincidentBoundaryPayload,
    >] {
        &self.arranged
    }

    pub(crate) fn caps(&self) -> &[SelectedCoincidentCapPlan] {
        &self.caps
    }

    pub(crate) fn into_parts(
        self,
    ) -> (
        Vec<
            SelectedBoundaryFragment<
                ParallelCoincidentBoundaryKey,
                ParallelCoincidentBoundaryPayload,
            >,
        >,
        Vec<SelectedCoincidentCapPlan>,
    ) {
        (self.arranged, self.caps)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PreparedCoincidentSourceSpan {
    cell: MixedShellCellKey,
    span: PeriodicSourceLoopKey,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct PreparedCoincidentSourcePartition {
    physical_end: usize,
    piece: CoincidentCapBoundaryPiece,
    spans: [PreparedCoincidentSourceSpan; 2],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PreparedPeriodicCutOwner {
    fragment: usize,
    cell: MixedShellCellKey,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct PreparedCoincidentCapProfile {
    physical_end: usize,
    boundary: [CoincidentCapBoundaryPiece; 2],
}

/// Owned periodic arrangements, cap candidates, and exact pair evidence.
pub(crate) struct PreparedParallelCylinderCoincidentBoundary {
    periodic: Vec<PreparedPeriodicFace>,
    caps: Vec<MixedCylinderCapRing>,
    classified: Vec<
        ClassifiedBoundaryFragment<
            ParallelCoincidentBoundaryKey,
            ParallelCoincidentBoundaryPayload,
        >,
    >,
    coincident_pairs: Vec<CoincidentBoundaryPairEvidence<ParallelCoincidentBoundaryKey>>,
    source_partitions: Vec<PreparedCoincidentSourcePartition>,
    periodic_cut_owners: Vec<PreparedPeriodicCutOwner>,
    cap_profiles: Vec<PreparedCoincidentCapProfile>,
}

impl PreparedParallelCylinderCoincidentBoundary {
    pub(crate) fn bindings(&self) -> Vec<MixedArrangementBinding<'_>> {
        self.periodic
            .iter()
            .map(|face| MixedArrangementBinding::Periodic {
                face: face.face.clone(),
                operand: face.operand,
                arrangement: &face.arrangement,
                embedding: face.embedding.as_ref(),
            })
            .chain(
                self.caps
                    .iter()
                    .map(|ring| MixedArrangementBinding::CylinderCap { ring }),
            )
            .collect()
    }

    pub(crate) fn classified(
        &self,
    ) -> Vec<
        ClassifiedBoundaryFragment<
            ParallelCoincidentBoundaryKey,
            ParallelCoincidentBoundaryPayload,
        >,
    > {
        self.classified.clone()
    }

    pub(crate) fn coincident_pairs(
        &self,
    ) -> Vec<CoincidentBoundaryPairEvidence<ParallelCoincidentBoundaryKey>> {
        self.coincident_pairs.clone()
    }

    /// Apply regularized set truth, then coalesce the selected planar
    /// partitions into one exact boundary plan per physical overlap end.
    pub(crate) fn select(
        &self,
        operation: RegularizedBooleanOperation,
    ) -> Result<SelectedParallelCylinderCoincidentBoundary, BoundarySelectionError> {
        let selected = select_boundary_fragments_with_coincident_pairs(
            operation,
            self.classified(),
            self.coincident_pairs(),
        )?;
        select_coincident_cap_plans(
            selected,
            &self.source_partitions,
            &self.periodic_cut_owners,
            &self.cap_profiles,
        )
    }
}

/// Arrange both side annuli and publish one proof-backed cap candidate per
/// source contributor.  Shared contributors are paired for canonical dedup.
#[allow(clippy::too_many_arguments)]
pub(crate) fn prepare_parallel_cylinder_coincident_boundary(
    part: &Part<'_>,
    graph: &BodySectionGraph,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderCoincidentCapRelation,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PreparedParallelCylinderCoincidentBoundary, MixedBoundaryError> {
    if graph.completion() != SectionCompletion::Indeterminate
        || graph.gaps().is_empty()
        || relation.overlap_ends().len() != 2
        || relation.rulings().len() != 2
    {
        return Err(MixedBoundaryError::IncompleteSection);
    }
    let fragment_subsets = [
        relation.periodic_fragment_subset(0),
        relation.periodic_fragment_subset(1),
    ];
    let projection_work = fragment_subsets.iter().try_fold(0_u64, |total, subset| {
        total.checked_add(periodic_face_fragment_subset_work(subset.len())?)
    });
    let work = parallel_boundary_work(
        6,
        graph.curve_fragments().len(),
        graph.curve_endpoints().len(),
        graph.curve_components().len(),
    )
    .and_then(|work| work.checked_add(projection_work?))
    .ok_or(MixedBoundaryError::SourceTopology)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, work)
        .map_err(Error::from)?;

    let store = &part.state.store;
    let mut periodic = Vec::with_capacity(2);
    let mut classified = Vec::new();
    let mut source_partitions = Vec::new();
    let mut periodic_cut_owners = Vec::new();
    for operand in 0..2 {
        let face = FaceId::new(
            bodies[operand].part().clone(),
            cylinders[operand].side_face(),
        );
        let certified = certify_periodic_face_fragment_subset(
            store,
            face.part(),
            graph,
            operand,
            face.clone(),
            &fragment_subsets[operand],
            linear,
        )
        .map_err(|gap| {
            MixedBoundaryError::PeriodicArrangement(
                MixedPeriodicArrangementError::EmbeddingIndeterminate(gap),
            )
        })?;
        let arrangement = arrange_mixed_periodic_face_from_embedding(graph, &certified)
            .map_err(MixedBoundaryError::PeriodicArrangement)?;
        validate_coincident_periodic_fragments(&arrangement, relation, operand)?;
        let source = source_face_key(store, graph, &face, operand)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        let (anchor_source, anchor_point) =
            coincident_periodic_anchor(store, graph, relation, &certified, &arrangement)?;
        let classes = classify_periodic_face_from_source_point(
            part,
            &bodies[1 - operand],
            &arrangement,
            anchor_source,
            anchor_point,
            linear,
            scope,
        )?;
        source_partitions.extend(prepare_cap_source_partitions(
            cylinders[operand],
            relation,
            operand,
            source,
            &certified,
            &arrangement,
            &classes,
        )?);
        periodic_cut_owners.extend(collect_periodic_cut_owners(source, &arrangement)?);
        classified.extend(arrangement.cells().iter().map(|cell| {
            ClassifiedBoundaryFragment::new(
                ParallelCoincidentBoundaryKey::Arranged(MixedShellCellKey::periodic(
                    source,
                    *cell.key(),
                )),
                operand_side(operand),
                ParallelCoincidentBoundaryPayload::Arranged,
                as_boundary_classification(classes[cell.key()]),
            )
        }));
        periodic.push(PreparedPeriodicFace {
            face,
            operand,
            arrangement,
            embedding: Some(certified),
        });
    }

    let mut coincident_pairs = Vec::new();
    let mut cap_profiles = Vec::with_capacity(relation.overlap_ends().len());
    for (physical_end, end) in relation.overlap_ends().iter().enumerate() {
        let boundary = cap_boundary_pieces(end)?;
        cap_profiles.push(PreparedCoincidentCapProfile {
            physical_end,
            boundary,
        });
        let mut contributors = 0_usize;
        for operand in 0..2 {
            let Some(source) = end.source(operand) else {
                continue;
            };
            contributors += 1;
            let target_face = FaceId::new(bodies[operand].part().clone(), source.cap_face());
            let target_source = source_face_key(store, graph, &target_face, operand)
                .map_err(|_| MixedBoundaryError::SourceTopology)?;
            let cap = PreparedCoincidentCapCell {
                physical_end,
                target_operand: operand,
                target_boundary: source.boundary(),
                target_source,
                target_face,
                boundary,
            };
            let classification = if end.is_shared() {
                BoundaryFragmentClassification::TwoSided {
                    other_on_source_interior: true,
                    other_on_source_exterior: false,
                }
            } else {
                BoundaryFragmentClassification::Interior
            };
            classified.push(ClassifiedBoundaryFragment::new(
                ParallelCoincidentBoundaryKey::CapEnd(physical_end),
                operand_side(operand),
                ParallelCoincidentBoundaryPayload::Cap(cap.clone()),
                classification,
            ));
            classified.push(ClassifiedBoundaryFragment::new(
                ParallelCoincidentBoundaryKey::CapRemainder(physical_end),
                operand_side(operand),
                ParallelCoincidentBoundaryPayload::Cap(cap),
                BoundaryFragmentClassification::Exterior,
            ));
        }
        if contributors != if end.is_shared() { 2 } else { 1 } {
            return Err(MixedBoundaryError::SourceTopology);
        }
        if end.is_shared() {
            coincident_pairs.push(CoincidentBoundaryPairEvidence::new(
                ParallelCoincidentBoundaryKey::CapEnd(physical_end),
                ParallelCoincidentBoundaryKey::CapEnd(physical_end),
                CoincidentSourceOrientation::Aligned,
            ));
        }
    }

    let mut caps = Vec::new();
    for operand in 0..2 {
        let periodic_face = periodic
            .iter()
            .find(|face| face.operand == operand)
            .ok_or(MixedBoundaryError::MissingPeriodicFaceEvidence)?;
        let certified = periodic_face
            .embedding
            .as_ref()
            .ok_or(MixedBoundaryError::MissingPeriodicFaceEvidence)?;
        for (boundary, source) in cylinders[operand].boundaries().iter().enumerate() {
            let belongs_to_overlap = relation.overlap_ends().iter().any(|end| {
                end.source(operand)
                    .is_some_and(|value| value.boundary() == boundary)
            });
            if belongs_to_overlap {
                continue;
            }
            if classify_anchor(part, &bodies[1 - operand], source.center(), linear, scope)? {
                return Err(MixedBoundaryError::CylinderCapNotExterior);
            }
            let ring = bind_cylinder_cap_ring_from_embedding(
                store,
                graph,
                cylinders[operand],
                operand,
                boundary,
                &periodic_face.face,
                &periodic_face.arrangement,
                certified,
            )
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
            classified.push(ClassifiedBoundaryFragment::new(
                ParallelCoincidentBoundaryKey::Arranged(MixedShellCellKey::cylinder_cap(
                    ring.cap_source(),
                    ring.boundary(),
                )),
                operand_side(operand),
                ParallelCoincidentBoundaryPayload::Arranged,
                BoundaryFragmentClassification::Exterior,
            ));
            caps.push(ring);
        }
    }
    if caps.len() != relation.unique_end_count() {
        return Err(MixedBoundaryError::SourceTopology);
    }

    Ok(PreparedParallelCylinderCoincidentBoundary {
        periodic,
        caps,
        classified,
        coincident_pairs,
        source_partitions,
        periodic_cut_owners,
        cap_profiles,
    })
}

const CAP_SELECTION_PAYLOAD_MISMATCH: &str =
    "coincident-cap truth selection returned an incompatible payload";
const CAP_SELECTION_OWNER_MISMATCH: &str =
    "coincident-cap truth selection has no canonical physical-end owner";
const CAP_SELECTION_SIDE_SPAN_MISMATCH: &str =
    "coincident-cap source arc has no unique selected periodic side span";
const CAP_SELECTION_SECTION_USE_MISMATCH: &str =
    "coincident-cap Section arc has no unique selected periodic side use";

#[derive(Debug, Clone, PartialEq, Eq)]
struct SelectedCoincidentCapTarget {
    key: ParallelCoincidentBoundaryKey,
    operand: OperandSide,
    target: PreparedCoincidentCapCell,
    orientation: SelectedOrientation,
}

fn selection_gap(reason: &'static str) -> BoundarySelectionError {
    BoundarySelectionError::Unsupported { reason }
}

fn select_coincident_cap_plans(
    selected: Vec<
        SelectedBoundaryFragment<ParallelCoincidentBoundaryKey, ParallelCoincidentBoundaryPayload>,
    >,
    source_partitions: &[PreparedCoincidentSourcePartition],
    periodic_cut_owners: &[PreparedPeriodicCutOwner],
    cap_profiles: &[PreparedCoincidentCapProfile],
) -> Result<SelectedParallelCylinderCoincidentBoundary, BoundarySelectionError> {
    let mut arranged = Vec::new();
    let mut side_cells = BTreeMap::new();
    let mut targets: BTreeMap<usize, Vec<SelectedCoincidentCapTarget>> = BTreeMap::new();
    for fragment in selected {
        match (fragment.key(), fragment.fragment()) {
            (
                ParallelCoincidentBoundaryKey::Arranged(cell),
                ParallelCoincidentBoundaryPayload::Arranged,
            ) if fragment.operand() == operand_side(cell.source().operand()) => {
                if side_cells.insert(*cell, fragment.orientation()).is_some() {
                    return Err(selection_gap(CAP_SELECTION_PAYLOAD_MISMATCH));
                }
                arranged.push(fragment);
            }
            (
                key @ (ParallelCoincidentBoundaryKey::CapEnd(physical_end)
                | ParallelCoincidentBoundaryKey::CapRemainder(physical_end)),
                ParallelCoincidentBoundaryPayload::Cap(cap),
            ) if *physical_end == cap.physical_end()
                && fragment.operand() == operand_side(cap.target_operand()) =>
            {
                targets
                    .entry(*physical_end)
                    .or_default()
                    .push(SelectedCoincidentCapTarget {
                        key: *key,
                        operand: fragment.operand(),
                        target: cap.clone(),
                        orientation: fragment.orientation(),
                    });
            }
            _ => return Err(selection_gap(CAP_SELECTION_PAYLOAD_MISMATCH)),
        }
    }

    let mut caps = Vec::with_capacity(cap_profiles.len());
    for profile in cap_profiles {
        let mut candidates = targets
            .remove(&profile.physical_end)
            .ok_or_else(|| selection_gap(CAP_SELECTION_OWNER_MISMATCH))?;
        if candidates.iter().any(|candidate| {
            candidate.target.boundary() != &profile.boundary
                || candidate.target.physical_end() != profile.physical_end
        }) {
            return Err(selection_gap(CAP_SELECTION_OWNER_MISMATCH));
        }
        candidates.sort_unstable_by_key(|candidate| (candidate.operand, candidate.key));
        let owner = candidates
            .first()
            .ok_or_else(|| selection_gap(CAP_SELECTION_OWNER_MISMATCH))?;
        if candidates
            .iter()
            .any(|candidate| candidate.orientation != owner.orientation)
        {
            return Err(selection_gap(CAP_SELECTION_OWNER_MISMATCH));
        }
        let boundary = profile
            .boundary
            .map(|piece| {
                select_cap_boundary_use(
                    profile.physical_end,
                    piece,
                    source_partitions,
                    periodic_cut_owners,
                    &side_cells,
                )
            })
            .into_iter()
            .collect::<Result<Vec<_>, _>>()?
            .try_into()
            .map_err(|_| selection_gap(CAP_SELECTION_SIDE_SPAN_MISMATCH))?;
        caps.push(SelectedCoincidentCapPlan {
            owner_key: owner.key,
            target: owner.target.clone(),
            orientation: owner.orientation,
            boundary,
        });
    }
    if !targets.is_empty() {
        return Err(selection_gap(CAP_SELECTION_OWNER_MISMATCH));
    }
    Ok(SelectedParallelCylinderCoincidentBoundary { arranged, caps })
}

fn select_cap_boundary_use(
    physical_end: usize,
    piece: CoincidentCapBoundaryPiece,
    source_partitions: &[PreparedCoincidentSourcePartition],
    periodic_cut_owners: &[PreparedPeriodicCutOwner],
    side_cells: &BTreeMap<MixedShellCellKey, SelectedOrientation>,
) -> Result<SelectedCoincidentCapBoundaryUse, BoundarySelectionError> {
    match piece {
        CoincidentCapBoundaryPiece::SourceArc {
            operand,
            edge,
            roots,
        } => {
            let mut partitions = source_partitions.iter().filter(|partition| {
                partition.physical_end == physical_end && partition.piece == piece
            });
            let partition = partitions
                .next()
                .filter(|_| partitions.next().is_none())
                .ok_or_else(|| selection_gap(CAP_SELECTION_SIDE_SPAN_MISMATCH))?;
            let mut selected = partition.spans.iter().filter_map(|span| {
                side_cells
                    .get(&span.cell)
                    .map(|orientation| (*span, *orientation))
            });
            let (span, side_orientation) = selected
                .next()
                .filter(|_| selected.next().is_none())
                .ok_or_else(|| selection_gap(CAP_SELECTION_SIDE_SPAN_MISMATCH))?;
            Ok(SelectedCoincidentCapBoundaryUse::SourceSpan {
                operand,
                edge,
                roots,
                side_cell: span.cell,
                span: span.span,
                side_orientation,
            })
        }
        CoincidentCapBoundaryPiece::SectionArc {
            fragment,
            endpoints,
        } => {
            let owners = periodic_cut_owners
                .iter()
                .filter(|owner| owner.fragment == fragment)
                .collect::<Vec<_>>();
            if owners.len() != 2 {
                return Err(selection_gap(CAP_SELECTION_SECTION_USE_MISMATCH));
            }
            let mut selected = owners.iter().filter_map(|owner| {
                side_cells
                    .get(&owner.cell)
                    .map(|orientation| (owner.cell, *orientation))
            });
            let (side_cell, side_orientation) = selected
                .next()
                .filter(|_| selected.next().is_none())
                .ok_or_else(|| selection_gap(CAP_SELECTION_SECTION_USE_MISMATCH))?;
            Ok(SelectedCoincidentCapBoundaryUse::SectionArc {
                fragment,
                endpoints,
                side_cell,
                side_orientation,
            })
        }
    }
}

fn coincident_periodic_anchor(
    store: &ktopo::store::Store,
    graph: &BodySectionGraph,
    relation: &CertifiedParallelCylinderCoincidentCapRelation,
    certified: &CertifiedSectionPeriodicFaceEmbedding,
    arrangement: &MixedPeriodicFaceArrangement,
) -> Result<(PeriodicSourceLoopKey, kgeom::vec::Point3), MixedBoundaryError> {
    let source = arrangement
        .source_spans()
        .iter()
        .find(|span| span.key().terminal_roots().is_some())
        .ok_or(MixedBoundaryError::AnchorUnavailable)?;
    let source_loop = certified
        .source_loops()
        .get(source.key().topology_ordinal())
        .ok_or(MixedBoundaryError::SourceTopology)?;
    let cap_point = periodic_source_span_point(store, source_loop.raw(), *source.key())?;
    let endpoint = source
        .key()
        .terminal_roots()
        .ok_or(MixedBoundaryError::SourceTopology)?[0]
        .endpoint();
    let mut matching_rulings = relation
        .rulings()
        .iter()
        .filter(|ruling| ruling.endpoints().contains(&endpoint));
    let ruling = matching_rulings
        .next()
        .filter(|_| matching_rulings.next().is_none())
        .ok_or(MixedBoundaryError::SourceTopology)?;
    let fragment = graph
        .curve_fragments()
        .get(ruling.fragment())
        .ok_or(MixedBoundaryError::SourceTopology)?;
    let SectionCurveFragmentSpan::LineSegment { endpoints } = fragment.span() else {
        return Err(MixedBoundaryError::SourceTopology);
    };
    let actual_endpoints = endpoints.each_ref().map(|end| end.endpoint());
    let expected_endpoints = ruling.endpoints();
    if fragment.branch() != ruling.branch()
        || (actual_endpoints != expected_endpoints
            && actual_endpoints != [expected_endpoints[1], expected_endpoints[0]])
    {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let matching_end = endpoints
        .iter()
        .find(|end| end.endpoint() == endpoint)
        .ok_or(MixedBoundaryError::SourceTopology)?;
    let parameters = endpoints.each_ref().map(|end| end.carrier_parameter());
    let midpoint = parameters[0] * 0.5 + parameters[1] * 0.5;
    let branch = graph
        .branches()
        .get(ruling.branch())
        .ok_or(MixedBoundaryError::SourceTopology)?;
    let SectionCarrier::Line { direction, .. } = branch.carrier() else {
        return Err(MixedBoundaryError::SourceTopology);
    };
    let offset = direction * (midpoint - matching_end.carrier_parameter());
    let point = cap_point + offset;
    if !midpoint.is_finite()
        || midpoint <= parameters[0].min(parameters[1])
        || midpoint >= parameters[0].max(parameters[1])
        || ![point.x, point.y, point.z].into_iter().all(f64::is_finite)
    {
        return Err(MixedBoundaryError::AnchorUnavailable);
    }
    Ok((*source.key(), point))
}

fn prepare_cap_source_partitions(
    cylinder: &CertifiedCylinderSource,
    relation: &CertifiedParallelCylinderCoincidentCapRelation,
    operand: usize,
    source_face: MixedSourceFaceKey,
    certified: &CertifiedSectionPeriodicFaceEmbedding,
    arrangement: &MixedPeriodicFaceArrangement,
    classes: &BTreeMap<PeriodicArrangementCellKey, bool>,
) -> Result<Vec<PreparedCoincidentSourcePartition>, MixedBoundaryError> {
    let mut partitions = Vec::new();
    for (physical_end, source) in relation
        .overlap_ends()
        .iter()
        .enumerate()
        .filter_map(|(physical_end, end)| end.source(operand).map(|source| (physical_end, source)))
    {
        let source_loop = cylinder.boundaries()[source.boundary()].side_loop();
        let loop_ordinal = certified
            .source_loops()
            .iter()
            .position(|loop_id| loop_id.raw() == source_loop)
            .ok_or(MixedBoundaryError::SourceTopology)?;
        let spans = arrangement
            .source_spans()
            .iter()
            .filter(|span| {
                span.key().topology_ordinal() == loop_ordinal
                    && span
                        .key()
                        .terminal_roots()
                        .is_some_and(|roots| same_periodic_roots(roots, source.roots()))
            })
            .collect::<Vec<_>>();
        if spans.len() != 2 {
            return Err(MixedBoundaryError::SourceTopology);
        }
        let mut owned_spans = Vec::with_capacity(2);
        for span in spans {
            let mut matching = arrangement.cells().iter().filter(|cell| {
                cell.boundaries().iter().any(|boundary| {
                    boundary.uses().iter().any(|use_| {
                        matches!(
                            use_.edge(),
                            super::super::face_arrangement::ArrangementEdgeKey::Source(key)
                                if key == span.key()
                        )
                    })
                })
            });
            let owner = matching
                .next()
                .filter(|_| matching.next().is_none())
                .ok_or(MixedBoundaryError::SourceTopology)?;
            let class = *classes
                .get(owner.key())
                .ok_or(MixedBoundaryError::SourceTopology)?;
            owned_spans.push((
                PreparedCoincidentSourceSpan {
                    cell: MixedShellCellKey::periodic(source_face, *owner.key()),
                    span: *span.key(),
                },
                class,
            ));
        }
        owned_spans.sort_unstable_by_key(|(span, _)| span.span);
        let classes = owned_spans
            .iter()
            .map(|(_, class)| *class)
            .collect::<Vec<_>>();
        if classes.iter().filter(|class| **class).count() != 1
            || classes.iter().filter(|class| !**class).count() != 1
        {
            return Err(MixedBoundaryError::ContradictoryDual);
        }
        let spans = owned_spans
            .into_iter()
            .map(|(span, _)| span)
            .collect::<Vec<_>>()
            .try_into()
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        partitions.push(PreparedCoincidentSourcePartition {
            physical_end,
            piece: CoincidentCapBoundaryPiece::SourceArc {
                operand,
                edge: source.edge(),
                roots: source.roots(),
            },
            spans,
        });
    }
    Ok(partitions)
}

fn collect_periodic_cut_owners(
    source: MixedSourceFaceKey,
    arrangement: &MixedPeriodicFaceArrangement,
) -> Result<Vec<PreparedPeriodicCutOwner>, MixedBoundaryError> {
    let mut owners = Vec::new();
    for cell in arrangement.cells() {
        for boundary in cell.boundaries() {
            for use_ in boundary.uses() {
                let ArrangementEdgeKey::Cut(cut) = use_.edge() else {
                    continue;
                };
                let owner = PreparedPeriodicCutOwner {
                    fragment: cut.fragment(),
                    cell: MixedShellCellKey::periodic(source, *cell.key()),
                };
                if owners.contains(&owner) {
                    return Err(MixedBoundaryError::SourceTopology);
                }
                owners.push(owner);
            }
        }
    }
    owners.sort_unstable_by_key(|owner| (owner.fragment, owner.cell));
    Ok(owners)
}

fn same_periodic_roots(
    actual: [PeriodicSourceRootKey; 2],
    expected: [ParallelCylinderSourceRootWitness; 2],
) -> bool {
    let matches = |actual: PeriodicSourceRootKey, expected: ParallelCylinderSourceRootWitness| {
        actual.endpoint() == expected.endpoint()
            && actual.source_root_ordinal() == expected.root_ordinal()
            && actual.root_parameter().to_bits() == expected.parameter().to_bits()
            && actual.root_enclosure().map(f64::to_bits) == expected.enclosure().map(f64::to_bits)
    };
    matches(actual[0], expected[0]) && matches(actual[1], expected[1])
        || matches(actual[0], expected[1]) && matches(actual[1], expected[0])
}

fn cap_boundary_pieces(
    end: &crate::boolean::parallel_cylinder_relation::ParallelCylinderCoincidentCapEndWitness,
) -> Result<[CoincidentCapBoundaryPiece; 2], MixedBoundaryError> {
    let mut pieces = end
        .sources()
        .iter()
        .flatten()
        .map(|source| CoincidentCapBoundaryPiece::SourceArc {
            operand: source.operand(),
            edge: source.edge(),
            roots: source.roots(),
        })
        .collect::<Vec<_>>();
    if let Some(arc) = end.cap_arc() {
        pieces.push(CoincidentCapBoundaryPiece::SectionArc {
            fragment: arc.fragment(),
            endpoints: arc.endpoints(),
        });
    }
    pieces
        .try_into()
        .map_err(|_| MixedBoundaryError::SourceTopology)
}

fn validate_coincident_periodic_fragments(
    arrangement: &MixedPeriodicFaceArrangement,
    relation: &CertifiedParallelCylinderCoincidentCapRelation,
    operand: usize,
) -> Result<(), MixedBoundaryError> {
    let mut actual = arrangement
        .cut_fragments()
        .iter()
        .map(|fragment| {
            if fragment.key().source_component().is_some() {
                return Err(MixedBoundaryError::SourceTopology);
            }
            Ok(fragment.key().fragment())
        })
        .collect::<Result<Vec<_>, _>>()?;
    let mut expected = relation
        .rulings()
        .iter()
        .map(|ruling| ruling.fragment())
        .collect::<Vec<_>>();
    expected.extend(relation.overlap_ends().iter().filter_map(|end| {
        (end.source(operand).is_none())
            .then(|| end.cap_arc().map(|arc| arc.fragment()))
            .flatten()
    }));
    actual.sort_unstable();
    expected.sort_unstable();
    if actual != expected || actual.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(MixedBoundaryError::SourceTopology);
    }

    let mut actual_roots = arrangement
        .source_spans()
        .iter()
        .filter_map(|span| span.key().terminal_roots())
        .flatten()
        .map(|root| (root.endpoint(), root.source_root_ordinal()))
        .collect::<Vec<_>>();
    let mut expected_roots = relation
        .overlap_ends()
        .iter()
        .filter_map(|end| end.source(operand))
        .flat_map(|source| {
            source
                .roots()
                .map(|root| (root.endpoint(), root.root_ordinal()))
        })
        .collect::<Vec<_>>();
    actual_roots.sort_unstable();
    actual_roots.dedup();
    expected_roots.sort_unstable();
    expected_roots.dedup();
    if actual_roots != expected_roots {
        return Err(MixedBoundaryError::SourceTopology);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationScope};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;

    use super::*;
    use crate::boolean::curved_source::{CylinderSourceOutcome, extract_cylinder_source};
    use crate::boolean::mixed_shell_plan::MixedShellCellKind;
    use crate::boolean::parallel_cylinder_relation::{
        ParallelCylinderRelationOutcome, certify_parallel_cylinder_relation,
    };
    use crate::{BodyId, CylinderRequest, Kernel, SectionBodiesRequest};

    fn extract_source(part: &Part<'_>, body: &BodyId) -> CertifiedCylinderSource {
        let context = OperationContext::new(part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(crate::boolean::BooleanBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        match extract_cylinder_source(&part.state.store, body.raw(), &mut scope).unwrap() {
            CylinderSourceOutcome::Ready(source) => source,
            other => panic!("unexpected cylinder source outcome: {other:?}"),
        }
    }

    fn prepared_case(
        first: (f64, f64),
        second: (f64, f64),
        reverse_second_axis: bool,
    ) -> (
        CertifiedParallelCylinderCoincidentCapRelation,
        PreparedParallelCylinderCoincidentBoundary,
    ) {
        let frame = Frame::world();
        let second_frame = if reverse_second_axis {
            Frame::new(
                frame.point_at(0.5, 0.0, second.0 + second.1),
                -frame.z(),
                frame.x(),
            )
            .unwrap()
        } else {
            frame.with_origin(frame.point_at(0.5, 0.0, second.0))
        };
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let bodies = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            [
                edit.create_cylinder(CylinderRequest::new(
                    frame.with_origin(frame.point_at(-0.5, 0.0, first.0)),
                    1.0,
                    first.1,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body(),
                edit.create_cylinder(CylinderRequest::new(second_frame, 1.0, second.1))
                    .unwrap()
                    .into_result()
                    .unwrap()
                    .body(),
            ]
        };
        let part = session.part(part_id).unwrap();
        let graph = part
            .section_bodies(SectionBodiesRequest::new(
                bodies[0].clone(),
                bodies[1].clone(),
            ))
            .unwrap()
            .into_result()
            .unwrap();
        let cylinders = [
            extract_source(&part, &bodies[0]),
            extract_source(&part, &bodies[1]),
        ];
        let context = OperationContext::new(part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(crate::boolean::BooleanBudgetProfile::v1_defaults());
        let relation = {
            let mut scope = OperationScope::new(&context);
            match certify_parallel_cylinder_relation(
                &part.state.store,
                &graph,
                [&cylinders[0], &cylinders[1]],
                &mut scope,
            )
            .unwrap()
            {
                ParallelCylinderRelationOutcome::CertifiedCoincidentCaps(relation) => *relation,
                other => panic!("unexpected parallel-cylinder relation: {other:?}"),
            }
        };
        let prepared = {
            let mut scope = OperationScope::new(&context);
            prepare_parallel_cylinder_coincident_boundary(
                &part,
                &graph,
                &bodies,
                [&cylinders[0], &cylinders[1]],
                &relation,
                Tolerances::default().linear(),
                &mut scope,
            )
            .unwrap()
        };
        (relation, prepared)
    }

    fn selected_side_matches(
        selected: &SelectedParallelCylinderCoincidentBoundary,
        cell: MixedShellCellKey,
        orientation: SelectedOrientation,
    ) -> bool {
        selected.arranged().iter().any(|fragment| {
            matches!(fragment.key(), ParallelCoincidentBoundaryKey::Arranged(key) if *key == cell)
                && fragment.orientation() == orientation
        })
    }

    fn assert_cap_plan(
        operation: RegularizedBooleanOperation,
        physical_end: usize,
        end: &crate::boolean::parallel_cylinder_relation::ParallelCylinderCoincidentCapEndWitness,
        plan: &SelectedCoincidentCapPlan,
        selected: &SelectedParallelCylinderCoincidentBoundary,
    ) {
        let source_operand = end.sources().iter().position(Option::is_some);
        let expected_operand = if end.is_shared() {
            0
        } else {
            source_operand.expect("a unique end has one source")
        };
        let expected_key = match (operation, end.is_shared(), expected_operand) {
            (RegularizedBooleanOperation::Intersect, _, _) => {
                ParallelCoincidentBoundaryKey::CapEnd(physical_end)
            }
            (RegularizedBooleanOperation::Unite, true, _) => {
                ParallelCoincidentBoundaryKey::CapEnd(physical_end)
            }
            (RegularizedBooleanOperation::Subtract, false, 1) => {
                ParallelCoincidentBoundaryKey::CapEnd(physical_end)
            }
            _ => ParallelCoincidentBoundaryKey::CapRemainder(physical_end),
        };
        let expected_orientation = if operation == RegularizedBooleanOperation::Subtract
            && !end.is_shared()
            && expected_operand == 1
        {
            SelectedOrientation::Reversed
        } else {
            SelectedOrientation::Preserved
        };
        assert_eq!(plan.owner_key(), expected_key);
        assert_eq!(plan.target().physical_end(), physical_end);
        assert_eq!(plan.target().target_operand(), expected_operand);
        assert_eq!(plan.orientation(), expected_orientation);

        for use_ in plan.boundary() {
            let (cell, orientation) = match *use_ {
                SelectedCoincidentCapBoundaryUse::SourceSpan {
                    operand,
                    edge,
                    roots,
                    side_cell,
                    span,
                    side_orientation,
                } => {
                    let source = end
                        .source(operand)
                        .expect("source use must be relation-owned");
                    assert_eq!(edge, source.edge());
                    assert_eq!(roots, source.roots());
                    assert!(same_periodic_roots(
                        span.terminal_roots()
                            .expect("selected source span is bounded"),
                        roots,
                    ));
                    (side_cell, side_orientation)
                }
                SelectedCoincidentCapBoundaryUse::SectionArc {
                    fragment,
                    endpoints,
                    side_cell,
                    side_orientation,
                } => {
                    let arc = end.cap_arc().expect("Section use must be relation-owned");
                    assert_eq!((fragment, endpoints), (arc.fragment(), arc.endpoints()));
                    (side_cell, side_orientation)
                }
            };
            let expected_side_orientation = if operation == RegularizedBooleanOperation::Subtract
                && cell.source().operand() == 1
            {
                SelectedOrientation::Reversed
            } else {
                SelectedOrientation::Preserved
            };
            assert_eq!(orientation, expected_side_orientation);
            assert!(selected_side_matches(selected, cell, orientation));
        }
    }

    #[test]
    fn coincident_cap_selection_is_operation_driven_for_shared_end_proofs() {
        let cases = [
            ((-1.0, 2.0), (-1.0, 2.0), None),
            ((-1.0, 3.0), (-1.0, 2.0), Some(0)),
            ((-1.0, 2.0), (-1.0, 3.0), Some(1)),
            ((-2.0, 3.0), (-1.0, 2.0), Some(0)),
        ];
        for ((first, second, far_operand), reverse_second_axis) in cases
            .into_iter()
            .flat_map(|case| [(case, false), (case, true)])
        {
            let (relation, prepared) = prepared_case(first, second, reverse_second_axis);
            for operation in [
                RegularizedBooleanOperation::Unite,
                RegularizedBooleanOperation::Intersect,
                RegularizedBooleanOperation::Subtract,
            ] {
                let selected = prepared.select(operation).unwrap();
                assert_eq!(selected, prepared.select(operation).unwrap());
                assert_eq!(selected.caps().len(), relation.overlap_ends().len());
                for (physical_end, (end, plan)) in relation
                    .overlap_ends()
                    .iter()
                    .zip(selected.caps())
                    .enumerate()
                {
                    assert_cap_plan(operation, physical_end, end, plan, &selected);
                }

                let selected_whole_caps = selected
                    .arranged()
                    .iter()
                    .filter_map(|fragment| match fragment.key() {
                        ParallelCoincidentBoundaryKey::Arranged(cell)
                            if matches!(cell.cell(), MixedShellCellKind::CylinderCap(_)) =>
                        {
                            Some(cell.source().operand())
                        }
                        _ => None,
                    })
                    .collect::<Vec<_>>();
                let expected_whole_caps = match operation {
                    RegularizedBooleanOperation::Unite => far_operand.into_iter().collect(),
                    RegularizedBooleanOperation::Intersect => Vec::new(),
                    RegularizedBooleanOperation::Subtract => far_operand
                        .filter(|operand| *operand == 0)
                        .into_iter()
                        .collect(),
                };
                assert_eq!(selected_whole_caps, expected_whole_caps);
            }
        }
    }
}
