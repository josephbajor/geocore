//! Untrusted discovery for two-host axial and exact-contact surgery.
//!
//! Evidence is limited to the live analytic face inventory. Discovery uses
//! only raw topology shape and carries no predicate result, proof boolean,
//! orientation, contact classification, or shell verdict.

use super::*;

pub(super) fn discover(store: &Store, shell_id: ShellId) -> Result<Vec<TwoHostSurgeryEvidence>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 3 || !shell.edges.is_empty() || shell.vertex.is_some() {
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
    if !matches!(cylinders.len(), 2 | 3)
        || !matches!(planar_faces.len(), 1..=4)
        || cylinders
            .iter()
            .any(|(face, _)| store.get(*face).is_ok_and(|face| face.loops.len() != 2))
    {
        return Ok(Vec::new());
    }
    Ok(vec![TwoHostSurgeryEvidence {
        shell: shell_id,
        cylinders,
        planar_faces,
    }])
}
