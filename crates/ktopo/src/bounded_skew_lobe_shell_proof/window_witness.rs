use super::{
    CapSlab, LobeTopology, certify_periodic_u_lift, cylinder_face_domain_box, cylinder_loop_box,
    loop_for_face, periodic_member_box_outside_face,
};
use crate::entity::FaceId;
use crate::loop_proof::certify_periodic_aabb2_window_lift;
use crate::store::Store;
use kcore::error::Result;
use kgraph::PersistentSkewCylinderFiniteWindowFamilyCertificate;

pub(super) fn complete_family_window_witness(
    store: &Store,
    topology: &LobeTopology,
    family: PersistentSkewCylinderFiniteWindowFamilyCertificate,
    source_faces: [FaceId; 2],
    cap_slab: CapSlab,
    retained_ordinals: [usize; 2],
) -> Result<bool> {
    let loop_boxes = [
        cylinder_loop_box(
            store,
            source_faces[0],
            loop_for_face(topology, source_faces[0])?,
        )?,
        cylinder_loop_box(
            store,
            source_faces[1],
            loop_for_face(topology, source_faces[1])?,
        )?,
    ];
    let [Some(first_box), Some(second_box)] = loop_boxes else {
        return Ok(false);
    };
    let source_windows = family.source_windows();
    for (source_slot, (face_id, loop_box)) in source_faces
        .into_iter()
        .zip([first_box, second_box])
        .enumerate()
    {
        let Some(domain_box) = cylinder_face_domain_box(store, face_id)? else {
            return Ok(false);
        };
        let (loop_lift, domain_lift) = if source_slot == cap_slab.source_slot {
            (
                certify_periodic_u_lift(loop_box, source_windows[source_slot][0]),
                certify_periodic_u_lift(domain_box, source_windows[source_slot][0]),
            )
        } else {
            (
                certify_periodic_aabb2_window_lift(
                    loop_box,
                    source_windows[source_slot],
                    core::f64::consts::TAU,
                ),
                certify_periodic_aabb2_window_lift(
                    domain_box,
                    source_windows[source_slot],
                    core::f64::consts::TAU,
                ),
            )
        };
        if loop_lift.is_none() || loop_lift != domain_lift {
            return Ok(false);
        }
    }
    for ordinal in 0..family.member_count() {
        if retained_ordinals.contains(&ordinal) {
            continue;
        }
        let Some(member) = family.member(ordinal) else {
            return Ok(false);
        };
        let boxes = member.pcurve_boxes();
        if !periodic_member_box_outside_face(boxes[0], first_box)
            || !periodic_member_box_outside_face(boxes[1], second_box)
        {
            return Ok(false);
        }
    }
    Ok(true)
}
