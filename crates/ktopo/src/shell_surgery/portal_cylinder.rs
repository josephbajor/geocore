//! Untrusted discovery for periodic hosts with rectangular surgery portals.
//!
//! This module proposes only live geometry and topology.  All incidence,
//! secancy, sweep-support, separation, base, and orientation predicates are
//! rerun by the shared theorem.

use super::*;

pub(super) fn discover(
    store: &Store,
    shell_id: ShellId,
) -> Result<Vec<PeriodicHostSurgeryEvidence>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 6 || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(Vec::new());
    }

    let mut cylinders = Vec::new();
    let mut planar_faces = Vec::new();
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(Vec::new());
        }
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(cylinder) => cylinders.push((face_id, *cylinder)),
            SurfaceGeom::Plane(_) => planar_faces.push(face_id),
            _ => return Ok(Vec::new()),
        }
    }

    let mut proposals = Vec::new();
    for (host_face, cylinder) in cylinders {
        if raw_host_shape(store, host_face)? {
            proposals.push(PeriodicHostSurgeryEvidence {
                shell: shell_id,
                host_face,
                cylinder,
                planar_faces: planar_faces.clone(),
            });
        }
    }
    Ok(proposals)
}

fn raw_host_shape(store: &Store, host_face: FaceId) -> Result<bool> {
    let face = store.get(host_face)?;
    let mut rings = 0_usize;
    let mut portals = 0_usize;
    for &loop_id in &face.loops {
        let loop_ = store.get(loop_id)?;
        match loop_.fins.as_slice() {
            [fin] => {
                let edge = store.get(store.get(*fin)?.edge)?;
                if edge.bounds.is_none() && edge.vertices == [None, None] {
                    rings += 1;
                }
            }
            [_, _, _, _] => portals += 1,
            _ => {}
        }
    }
    Ok(rings == 2 && portals > 0)
}
