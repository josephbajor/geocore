//! Property-facing projection of the already-resolved lobe theorem.
//!
//! This module deliberately owns no recognizer. It only projects identities
//! after the parent theorem has resolved the complete finite-window family and
//! both oriented Cylinder loops.

use super::{
    LobeTopology, PersistentBoundary, cylinder_loop_orientations, loop_for_face,
    recognize_lobe_topology, resolve_complete_family,
};
use crate::entity::{EdgeId, FaceId, LoopId, ShellId};
use crate::store::Store;
use kcore::error::Result;
use kgraph::VerifiedSkewCylinderOpenSpanCurveDescriptor;

/// Exact identities authorized for lobe-specific property integration.
#[derive(Debug, Clone, Copy)]
pub(crate) struct BoundedSkewLobePropertyWitness {
    faces: [FaceId; 4],
    cylinder_faces: [FaceId; 2],
    cylinder_loops: [LoopId; 2],
    persistent_edges: [EdgeId; 2],
    persistent_descriptors: [VerifiedSkewCylinderOpenSpanCurveDescriptor; 2],
}

impl BoundedSkewLobePropertyWitness {
    pub(crate) fn owns_exact_faces(self, faces: &[FaceId]) -> bool {
        faces.len() == self.faces.len()
            && self
                .faces
                .into_iter()
                .all(|candidate| faces.contains(&candidate))
    }

    pub(crate) fn cylinder_source_slot(self, face: FaceId, loop_id: LoopId) -> Option<usize> {
        self.cylinder_faces
            .iter()
            .copied()
            .zip(self.cylinder_loops)
            .position(|(candidate_face, candidate_loop)| {
                candidate_face == face && candidate_loop == loop_id
            })
    }

    pub(crate) fn persistent_descriptor(
        self,
        edge: EdgeId,
    ) -> Option<VerifiedSkewCylinderOpenSpanCurveDescriptor> {
        self.persistent_edges
            .iter()
            .position(|candidate| *candidate == edge)
            .map(|index| self.persistent_descriptors[index])
    }
}

/// Reissue the existing complete-family theorem as a read-only property input.
pub(crate) fn certify_bounded_skew_lobe_property_witness(
    store: &Store,
    shell_id: ShellId,
) -> Result<Option<BoundedSkewLobePropertyWitness>> {
    let Some(topology) = recognize_lobe_topology(store, shell_id)? else {
        return Ok(None);
    };
    let Some(family) = resolve_complete_family(store, &topology)? else {
        return Ok(None);
    };
    if cylinder_loop_orientations(store, &topology, family)?.is_none() {
        return Ok(None);
    }
    project_witness(topology, family.source_faces, family.ordered)
}

fn project_witness(
    topology: LobeTopology,
    cylinder_faces: [FaceId; 2],
    persistent: [PersistentBoundary; 2],
) -> Result<Option<BoundedSkewLobePropertyWitness>> {
    let cylinder_loops = [
        loop_for_face(&topology, cylinder_faces[0])?,
        loop_for_face(&topology, cylinder_faces[1])?,
    ];
    let faces = [
        topology.caps[0].face,
        topology.caps[1].face,
        cylinder_faces[0],
        cylinder_faces[1],
    ];
    Ok(Some(BoundedSkewLobePropertyWitness {
        faces,
        cylinder_faces,
        cylinder_loops,
        persistent_edges: persistent.map(|boundary| boundary.edge),
        persistent_descriptors: persistent.map(|boundary| boundary.descriptor),
    }))
}
