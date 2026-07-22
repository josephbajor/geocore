//! Topology-owned identity for uncut finite-cylinder cap rings.
//!
//! A certified finite cylinder owns two endpoint-free circle edges, each used
//! once by the periodic side and once by one planar cap. This adapter binds
//! each cap to the corresponding whole-loop identity already published by the
//! periodic arrangement. The repeated seam key is proof-only: no point or
//! physical vertex is chosen or synthesized.

use ktopo::entity::{EdgeId as RawEdgeId, FinId as RawFinId, LoopId as RawLoopId};
use ktopo::store::Store;

use super::boundary_select::{
    BoundaryFragmentClassification, ClassifiedBoundaryFragment, OperandSide,
};
use super::curved_source::CertifiedCylinderSource;
use super::mixed_periodic_arrangement::{MixedPeriodicFaceArrangement, PeriodicSourceLoopKey};
use super::mixed_shell_plan::{MixedSourceFaceKey, source_face_key};
use crate::{BodySectionGraph, FaceId, SectionPeriodicFaceEmbeddingEvidence};

/// One exact cap/side use pair of an endpoint-free source circle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedCylinderCapRing {
    boundary: usize,
    operand: usize,
    cap_face: FaceId,
    cap_source: MixedSourceFaceKey,
    side_source: MixedSourceFaceKey,
    side_loop_key: PeriodicSourceLoopKey,
    edge: RawEdgeId,
    cap_loop: RawLoopId,
    cap_fin: RawFinId,
    side_loop: RawLoopId,
    side_fin: RawFinId,
}

impl MixedCylinderCapRing {
    pub(crate) const fn boundary(&self) -> usize {
        self.boundary
    }

    pub(crate) const fn operand(&self) -> usize {
        self.operand
    }

    pub(crate) const fn cap_face(&self) -> &FaceId {
        &self.cap_face
    }

    pub(crate) const fn cap_source(&self) -> MixedSourceFaceKey {
        self.cap_source
    }

    pub(crate) const fn side_source(&self) -> MixedSourceFaceKey {
        self.side_source
    }

    pub(crate) const fn side_loop_key(&self) -> PeriodicSourceLoopKey {
        self.side_loop_key
    }

    pub(crate) const fn edge(&self) -> RawEdgeId {
        self.edge
    }

    pub(crate) const fn cap_loop(&self) -> RawLoopId {
        self.cap_loop
    }

    pub(crate) const fn cap_fin(&self) -> RawFinId {
        self.cap_fin
    }

    pub(crate) const fn side_loop(&self) -> RawLoopId {
        self.side_loop
    }

    pub(crate) const fn side_fin(&self) -> RawFinId {
        self.side_fin
    }
}

/// Fail-closed gap while relating source-cylinder evidence to Section's
/// periodic whole-loop identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MixedCylinderCapRingGap {
    MissingPeriodicEvidence,
    PeriodicFaceMismatch,
    PeriodicSourceMismatch,
    CapSourceMismatch,
    SourceLoopMismatch,
    CapLoopMismatch,
    BoundaryIncidenceMismatch,
}

/// Bind both certified cap edges to the side arrangement's exact loop keys.
pub(crate) fn bind_cylinder_cap_rings(
    store: &Store,
    graph: &BodySectionGraph,
    cylinder: &CertifiedCylinderSource,
    cylinder_operand: usize,
    periodic_face: &FaceId,
    periodic_arrangement: &MixedPeriodicFaceArrangement,
) -> Result<[MixedCylinderCapRing; 2], MixedCylinderCapRingGap> {
    let source_spans = periodic_arrangement.source_spans();
    if source_spans.len() != 2 || source_spans.iter().any(|span| !span.is_whole_loop()) {
        return Err(MixedCylinderCapRingGap::SourceLoopMismatch);
    }
    let rings = [
        bind_cylinder_cap_ring(
            store,
            graph,
            cylinder,
            cylinder_operand,
            0,
            periodic_face,
            periodic_arrangement,
        )?,
        bind_cylinder_cap_ring(
            store,
            graph,
            cylinder,
            cylinder_operand,
            1,
            periodic_face,
            periodic_arrangement,
        )?,
    ];
    if rings[0].edge == rings[1].edge
        || rings[0].cap_source == rings[1].cap_source
        || rings[0].side_loop_key == rings[1].side_loop_key
    {
        return Err(MixedCylinderCapRingGap::BoundaryIncidenceMismatch);
    }
    Ok(rings)
}

/// Bind one uncut certified cap edge even when the sibling source loop is cut.
#[allow(clippy::too_many_arguments)]
pub(crate) fn bind_cylinder_cap_ring(
    store: &Store,
    graph: &BodySectionGraph,
    cylinder: &CertifiedCylinderSource,
    cylinder_operand: usize,
    boundary: usize,
    periodic_face: &FaceId,
    periodic_arrangement: &MixedPeriodicFaceArrangement,
) -> Result<MixedCylinderCapRing, MixedCylinderCapRingGap> {
    if periodic_face.raw() != cylinder.side_face() {
        return Err(MixedCylinderCapRingGap::PeriodicFaceMismatch);
    }
    let evidence = graph
        .periodic_face_embeddings()
        .iter()
        .find_map(|evidence| match evidence {
            SectionPeriodicFaceEmbeddingEvidence::Certified(evidence)
                if evidence.operand() == cylinder_operand && evidence.face() == *periodic_face =>
            {
                Some(evidence)
            }
            _ => None,
        })
        .ok_or(MixedCylinderCapRingGap::MissingPeriodicEvidence)?;
    bind_cylinder_cap_ring_from_embedding(
        store,
        graph,
        cylinder,
        cylinder_operand,
        boundary,
        periodic_face,
        periodic_arrangement,
        evidence,
    )
}

/// Bind one source cap to an operation-local periodic projection that retains
/// original graph fragment and topology identities.
#[allow(clippy::too_many_arguments)]
pub(crate) fn bind_cylinder_cap_ring_from_embedding(
    store: &Store,
    graph: &BodySectionGraph,
    cylinder: &CertifiedCylinderSource,
    cylinder_operand: usize,
    boundary: usize,
    periodic_face: &FaceId,
    periodic_arrangement: &MixedPeriodicFaceArrangement,
    evidence: &crate::CertifiedSectionPeriodicFaceEmbedding,
) -> Result<MixedCylinderCapRing, MixedCylinderCapRingGap> {
    if periodic_face.raw() != cylinder.side_face()
        || evidence.operand() != cylinder_operand
        || evidence.face() != *periodic_face
    {
        return Err(MixedCylinderCapRingGap::PeriodicFaceMismatch);
    }
    let side_source = source_face_key(store, graph, periodic_face, cylinder_operand)
        .map_err(|_| MixedCylinderCapRingGap::PeriodicSourceMismatch)?;
    if evidence.source_loops()[0] == evidence.source_loops()[1] {
        return Err(MixedCylinderCapRingGap::SourceLoopMismatch);
    }

    let source_boundary = cylinder
        .boundaries()
        .get(boundary)
        .ok_or(MixedCylinderCapRingGap::BoundaryIncidenceMismatch)?;
    let cap_face = FaceId::new(periodic_face.part().clone(), source_boundary.cap_face());
    let cap_source = source_face_key(store, graph, &cap_face, cylinder_operand)
        .map_err(|_| MixedCylinderCapRingGap::CapSourceMismatch)?;
    let raw_cap_face = store
        .get(source_boundary.cap_face())
        .map_err(|_| MixedCylinderCapRingGap::CapLoopMismatch)?;
    let [cap_loop] = raw_cap_face.loops() else {
        return Err(MixedCylinderCapRingGap::CapLoopMismatch);
    };
    let raw_cap_loop = store
        .get(*cap_loop)
        .map_err(|_| MixedCylinderCapRingGap::CapLoopMismatch)?;
    let [cap_fin] = raw_cap_loop.fins() else {
        return Err(MixedCylinderCapRingGap::CapLoopMismatch);
    };
    let raw_cap_fin = store
        .get(*cap_fin)
        .map_err(|_| MixedCylinderCapRingGap::CapLoopMismatch)?;
    if raw_cap_loop.face() != source_boundary.cap_face()
        || raw_cap_fin.parent() != *cap_loop
        || raw_cap_fin.edge() != source_boundary.edge()
    {
        return Err(MixedCylinderCapRingGap::CapLoopMismatch);
    }

    let edge = store
        .get(source_boundary.edge())
        .map_err(|_| MixedCylinderCapRingGap::BoundaryIncidenceMismatch)?;
    if edge.vertices() != [None, None]
        || edge.bounds().is_some()
        || edge.tolerance().is_some()
        || edge.fins().len() != 2
        || !edge.fins().contains(cap_fin)
    {
        return Err(MixedCylinderCapRingGap::BoundaryIncidenceMismatch);
    }

    let mut side_match = None;
    for (topology_ordinal, loop_id) in evidence.source_loops().iter().enumerate() {
        let raw_loop = store
            .get(loop_id.raw())
            .map_err(|_| MixedCylinderCapRingGap::SourceLoopMismatch)?;
        let [side_fin] = raw_loop.fins() else {
            return Err(MixedCylinderCapRingGap::SourceLoopMismatch);
        };
        let raw_side_fin = store
            .get(*side_fin)
            .map_err(|_| MixedCylinderCapRingGap::SourceLoopMismatch)?;
        if raw_loop.face() != cylinder.side_face() || raw_side_fin.parent() != loop_id.raw() {
            return Err(MixedCylinderCapRingGap::SourceLoopMismatch);
        }
        if raw_side_fin.edge() == source_boundary.edge()
            && side_match
                .replace((
                    topology_ordinal,
                    loop_id.raw(),
                    *side_fin,
                    raw_side_fin.sense(),
                ))
                .is_some()
        {
            return Err(MixedCylinderCapRingGap::SourceLoopMismatch);
        }
    }
    let Some((topology_ordinal, side_loop, side_fin, side_sense)) = side_match else {
        return Err(MixedCylinderCapRingGap::BoundaryIncidenceMismatch);
    };
    if side_sense == raw_cap_fin.sense() || !edge.fins().contains(&side_fin) {
        return Err(MixedCylinderCapRingGap::BoundaryIncidenceMismatch);
    }
    let spans = periodic_arrangement
        .source_spans()
        .iter()
        .filter(|span| span.key().topology_ordinal() == topology_ordinal)
        .collect::<Vec<_>>();
    let [span] = spans.as_slice() else {
        return Err(MixedCylinderCapRingGap::SourceLoopMismatch);
    };
    if !span.is_whole_loop() {
        return Err(MixedCylinderCapRingGap::SourceLoopMismatch);
    }

    Ok(MixedCylinderCapRing {
        boundary,
        operand: cylinder_operand,
        cap_face,
        cap_source,
        side_source,
        side_loop_key: *span.key(),
        edge: source_boundary.edge(),
        cap_loop: *cap_loop,
        cap_fin: *cap_fin,
        side_loop,
        side_fin,
    })
}

/// Present one certified exterior cap to representation-independent truth.
pub(crate) const fn classified_exterior_cap<K>(
    key: K,
    operand: usize,
) -> ClassifiedBoundaryFragment<K, ()> {
    ClassifiedBoundaryFragment::new(
        key,
        if operand == 0 {
            OperandSide::Left
        } else {
            OperandSide::Right
        },
        (),
        BoundaryFragmentClassification::Exterior,
    )
}

#[cfg(test)]
mod tests {
    use super::super::boundary_select::{
        RegularizedBooleanOperation, SelectedOrientation, select_boundary_fragments,
    };
    use super::*;

    fn selected_caps(
        operation: RegularizedBooleanOperation,
        cylinder_operand: usize,
    ) -> Vec<super::super::boundary_select::SelectedBoundaryFragment<usize, ()>> {
        select_boundary_fragments(
            operation,
            [
                classified_exterior_cap(0, cylinder_operand),
                classified_exterior_cap(1, cylinder_operand),
            ],
        )
        .unwrap()
    }

    #[test]
    fn exterior_cap_truth_is_operation_and_operand_order_independent() {
        for cylinder_operand in [0, 1] {
            let united = selected_caps(RegularizedBooleanOperation::Unite, cylinder_operand);
            assert_eq!(united.len(), 2);
            assert!(
                united
                    .iter()
                    .all(|cap| cap.orientation() == SelectedOrientation::Preserved)
            );
            assert!(
                selected_caps(RegularizedBooleanOperation::Intersect, cylinder_operand).is_empty()
            );
        }

        let cylinder_minus_planar = selected_caps(RegularizedBooleanOperation::Subtract, 0);
        assert_eq!(cylinder_minus_planar.len(), 2);
        assert!(
            cylinder_minus_planar
                .iter()
                .all(|cap| cap.orientation() == SelectedOrientation::Preserved)
        );
        assert!(selected_caps(RegularizedBooleanOperation::Subtract, 1).is_empty());
    }
}
