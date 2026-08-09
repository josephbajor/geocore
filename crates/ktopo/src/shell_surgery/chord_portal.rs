//! Discovery-only adapter for translated mixed line/arc portal sweeps.
//!
//! The adapter copies topology identities and polygon vertices. It does not
//! call proof predicates, classify orientation, or construct a shell verdict.

use super::*;
use crate::entity::FinId;

pub(super) fn discover(
    store: &Store,
    shell_id: ShellId,
) -> Result<Vec<ChordPortalSurgeryEvidence>> {
    let shell = store.get(shell_id)?;
    if !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(Vec::new());
    }

    let mut facets = Vec::new();
    let mut portals = Vec::new();
    let mut feature_faces = Vec::new();
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(Vec::new());
        }
        if !matches!(store.get(face.surface)?, SurfaceGeom::Plane(_)) {
            feature_faces.push(face_id);
            continue;
        }
        let mut outer = Vec::new();
        for &loop_id in &face.loops {
            if let Some(vertices) = raw_host_outer_loop(store, face_id, loop_id)? {
                outer.push((loop_id, vertices));
            }
        }
        let [(outer_loop, vertices)] = outer.as_slice() else {
            feature_faces.push(face_id);
            continue;
        };
        facets.push(PlanarFacetEvidence {
            face: face_id,
            outer_loop: *outer_loop,
            vertices: vertices.clone(),
        });
        for &loop_id in &face.loops {
            if loop_id != *outer_loop && raw_line_loop(store, face_id, loop_id, 4)? {
                portals.push((face_id, loop_id));
            }
        }
    }
    if facets.len() < 4 || portals.is_empty() {
        return Ok(Vec::new());
    }

    let host_faces = facets.iter().map(|facet| facet.face).collect::<Vec<_>>();
    let mut claimed = Vec::new();
    let mut features = Vec::with_capacity(portals.len());
    for (portal_face, portal_loop) in portals {
        let mut local = Vec::new();
        for &fin_id in &store.get(portal_loop)?.fins {
            let Some(peer) = raw_peer_face(store, fin_id)? else {
                return Ok(Vec::new());
            };
            if peer == portal_face || host_faces.contains(&peer) {
                return Ok(Vec::new());
            }
            if !local.contains(&peer) {
                local.push(peer);
            }
        }
        if local.len() != 3 || local.iter().any(|face| claimed.contains(face)) {
            return Ok(Vec::new());
        }
        claimed.extend(local.iter().copied());
        features.push(ChordPortalFeatureEvidence {
            portal_face,
            portal_loop,
            feature_faces: local,
        });
    }
    if feature_faces.len() != claimed.len()
        || feature_faces.iter().any(|face| !claimed.contains(face))
    {
        return Ok(Vec::new());
    }
    Ok(vec![ChordPortalSurgeryEvidence {
        shell: shell_id,
        base: PlanarBaseEvidence { facets },
        features,
    }])
}

fn raw_host_outer_loop(
    store: &Store,
    face_id: FaceId,
    loop_id: LoopId,
) -> Result<Option<Vec<VertexId>>> {
    let loop_ = store.get(loop_id)?;
    if loop_.face != face_id || loop_.fins.len() < 3 {
        return Ok(None);
    }
    let mut vertices = Vec::with_capacity(loop_.fins.len());
    for &fin_id in &loop_.fins {
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let (Some(_), Some(_), Some(tail), Some(peer)) = (
            edge.curve,
            edge.bounds,
            store.fin_tail(fin_id)?,
            raw_peer_face(store, fin_id)?,
        ) else {
            return Ok(None);
        };
        let peer_is_plane = matches!(store.get(store.get(peer)?.surface)?, SurfaceGeom::Plane(_));
        if edge.tolerance.is_some() || !peer_is_plane || vertices.contains(&tail) {
            return Ok(None);
        }
        vertices.push(tail);
    }
    Ok(Some(vertices))
}

fn raw_line_loop(store: &Store, face_id: FaceId, loop_id: LoopId, count: usize) -> Result<bool> {
    let loop_ = store.get(loop_id)?;
    if loop_.face != face_id || loop_.fins.len() != count {
        return Ok(false);
    }
    for &fin_id in &loop_.fins {
        let edge = store.get(store.get(fin_id)?.edge)?;
        let Some(_) = edge.curve else {
            return Ok(false);
        };
        if edge.tolerance.is_some() || edge.bounds.is_none() {
            return Ok(false);
        }
    }
    Ok(true)
}

fn raw_peer_face(store: &Store, fin_id: FinId) -> Result<Option<FaceId>> {
    let fin = store.get(fin_id)?;
    let edge = store.get(fin.edge)?;
    let [first, second] = edge.fins.as_slice() else {
        return Ok(None);
    };
    let peer = if *first == fin_id {
        *second
    } else if *second == fin_id {
        *first
    } else {
        return Ok(None);
    };
    Ok(Some(store.get(store.get(peer)?.parent)?.face))
}
