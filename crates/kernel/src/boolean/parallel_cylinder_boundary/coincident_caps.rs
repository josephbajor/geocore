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
    BoundaryFragmentClassification, CoincidentBoundaryPairEvidence, CoincidentSourceOrientation,
};
use crate::boolean::mixed_boundary::{
    classify_periodic_face_from_source_point, periodic_source_span_point,
};
use crate::boolean::mixed_periodic_arrangement::arrange_mixed_periodic_face_from_certified_embedding;
use crate::boolean::mixed_periodic_arrangement::{
    PeriodicArrangementCellKey, PeriodicSourceLoopKey, PeriodicSourceRootKey,
};
use crate::boolean::mixed_shell_plan::MixedSourceFaceKey;
use crate::boolean::parallel_cylinder_relation::CertifiedParallelCylinderCoincidentCapRelation;
use crate::boolean::parallel_cylinder_relation::ParallelCylinderSourceRootWitness;
use crate::{SectionCarrier, SectionCompletion, SectionCurveFragmentSpan};

/// Canonical selector key for an arranged side cell or one physical cap end.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum ParallelCoincidentBoundaryKey {
    Arranged(MixedShellCellKey),
    CapEnd(usize),
    CapRemainder(usize),
    ExteriorCap(usize),
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
    OmittedCap,
}

/// Owned periodic arrangements, cap candidates, and exact pair evidence.
pub(crate) struct PreparedParallelCylinderCoincidentBoundary {
    periodic: Vec<PreparedPeriodicFace>,
    classified: Vec<
        ClassifiedBoundaryFragment<
            ParallelCoincidentBoundaryKey,
            ParallelCoincidentBoundaryPayload,
        >,
    >,
    coincident_pairs: Vec<CoincidentBoundaryPairEvidence<ParallelCoincidentBoundaryKey>>,
}

impl PreparedParallelCylinderCoincidentBoundary {
    pub(crate) fn bindings(&self) -> Vec<MixedArrangementBinding<'_>> {
        self.periodic
            .iter()
            .map(|face| MixedArrangementBinding::Periodic {
                face: face.face.clone(),
                operand: face.operand,
                arrangement: &face.arrangement,
            })
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
    let work = parallel_boundary_work(
        6,
        graph.curve_fragments().len(),
        graph.curve_endpoints().len(),
        graph.curve_components().len(),
    )
    .ok_or(MixedBoundaryError::SourceTopology)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, work)
        .map_err(Error::from)?;

    let store = &part.state.store;
    let mut periodic = Vec::with_capacity(2);
    let mut classified = Vec::new();
    for operand in 0..2 {
        let face = unique_periodic_face(graph, cylinders[operand], operand)?;
        let arrangement =
            arrange_mixed_periodic_face_from_certified_embedding(graph, face.clone(), operand)
                .map_err(MixedBoundaryError::PeriodicArrangement)?;
        validate_coincident_periodic_fragments(&arrangement, relation, operand)?;
        let source = source_face_key(store, graph, &face, operand)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        let (anchor_source, anchor_point) = coincident_periodic_anchor(
            store,
            graph,
            cylinders[operand],
            relation,
            operand,
            &arrangement,
        )?;
        let classes = classify_periodic_face_from_source_point(
            part,
            &bodies[1 - operand],
            &arrangement,
            anchor_source,
            anchor_point,
            linear,
            scope,
        )?;
        validate_cap_partition_classes(
            graph,
            cylinders[operand],
            relation,
            operand,
            &arrangement,
            &classes,
        )?;
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
        });
    }

    let mut coincident_pairs = Vec::new();
    for (physical_end, end) in relation.overlap_ends().iter().enumerate() {
        let boundary = cap_boundary_pieces(end)?;
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
                ParallelCoincidentBoundaryPayload::Cap(cap),
                classification,
            ));
            classified.push(ClassifiedBoundaryFragment::new(
                ParallelCoincidentBoundaryKey::CapRemainder(physical_end),
                operand_side(operand),
                ParallelCoincidentBoundaryPayload::OmittedCap,
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

    for operand in 0..2 {
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
            classified.push(ClassifiedBoundaryFragment::new(
                ParallelCoincidentBoundaryKey::ExteriorCap(boundary),
                operand_side(operand),
                ParallelCoincidentBoundaryPayload::OmittedCap,
                BoundaryFragmentClassification::Exterior,
            ));
        }
    }

    Ok(PreparedParallelCylinderCoincidentBoundary {
        periodic,
        classified,
        coincident_pairs,
    })
}

fn coincident_periodic_anchor(
    store: &ktopo::store::Store,
    graph: &BodySectionGraph,
    cylinder: &CertifiedCylinderSource,
    relation: &CertifiedParallelCylinderCoincidentCapRelation,
    operand: usize,
    arrangement: &MixedPeriodicFaceArrangement,
) -> Result<(PeriodicSourceLoopKey, kgeom::vec::Point3), MixedBoundaryError> {
    let certified = graph
        .periodic_face_embeddings()
        .iter()
        .find_map(|evidence| match evidence {
            SectionPeriodicFaceEmbeddingEvidence::Certified(value)
                if value.operand() == operand && value.face().raw() == cylinder.side_face() =>
            {
                Some(value)
            }
            _ => None,
        })
        .ok_or(MixedBoundaryError::MissingPeriodicFaceEvidence)?;
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

fn validate_cap_partition_classes(
    graph: &BodySectionGraph,
    cylinder: &CertifiedCylinderSource,
    relation: &CertifiedParallelCylinderCoincidentCapRelation,
    operand: usize,
    arrangement: &MixedPeriodicFaceArrangement,
    classes: &BTreeMap<PeriodicArrangementCellKey, bool>,
) -> Result<(), MixedBoundaryError> {
    let certified = graph
        .periodic_face_embeddings()
        .iter()
        .find_map(|evidence| match evidence {
            SectionPeriodicFaceEmbeddingEvidence::Certified(value)
                if value.operand() == operand && value.face().raw() == cylinder.side_face() =>
            {
                Some(value)
            }
            _ => None,
        })
        .ok_or(MixedBoundaryError::MissingPeriodicFaceEvidence)?;
    for source in relation
        .overlap_ends()
        .iter()
        .filter_map(|end| end.source(operand))
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
        let mut owners = Vec::with_capacity(2);
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
            owners.push(
                *classes
                    .get(owner.key())
                    .ok_or(MixedBoundaryError::SourceTopology)?,
            );
        }
        owners.sort_unstable();
        if owners != [false, true] {
            return Err(MixedBoundaryError::ContradictoryDual);
        }
    }
    Ok(())
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
