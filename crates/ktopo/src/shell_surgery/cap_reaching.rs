//! Untrusted discovery for a terminal strict-secant cylinder sweep.
//!
//! Discovery records only live face identities and analytic carriers.  It
//! does not invoke incidence, loop, secancy, sweep, base, or orientation
//! predicates and carries no proof booleans or shell verdicts.

use super::*;

pub(super) fn discover(
    store: &Store,
    shell_id: ShellId,
) -> Result<Vec<CapReachingSurgeryEvidence>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 5 || !shell.edges.is_empty() || shell.vertex.is_some() {
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
    if cylinders.len() < 2 || planar_faces.len() < 3 {
        return Ok(Vec::new());
    }

    let mut proposals = Vec::new();
    for &(host_face, cylinder) in &cylinders {
        if raw_host_shape(store, host_face)? {
            proposals.push(CapReachingSurgeryEvidence {
                shell: shell_id,
                host_face,
                cylinder,
                cylinders: cylinders.clone(),
                planar_faces: planar_faces.clone(),
            });
        }
    }
    Ok(proposals)
}

fn raw_host_shape(store: &Store, host_face: FaceId) -> Result<bool> {
    let face = store.get(host_face)?;
    let mut rings = 0_usize;
    let mut bounded = 0_usize;
    for &loop_id in &face.loops {
        let loop_ = store.get(loop_id)?;
        if loop_.face != host_face {
            return Ok(false);
        }
        match loop_.fins.as_slice() {
            [fin] => {
                let edge = store.get(store.get(*fin)?.edge)?;
                if edge.bounds.is_none() && edge.vertices == [None, None] {
                    rings += 1;
                }
            }
            fins if fins.len() >= 4 => bounded += 1,
            _ => {}
        }
    }
    Ok(rings == 1 && bounded == 1)
}
