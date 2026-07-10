//! Conservative shell embedding and orientation proofs.
//!
//! A closed manifold whose faces are strict convex planar facets, with every
//! facet a supporting plane of the complete vertex set, is the boundary of
//! its convex hull. That gives a compact proof of both global
//! non-self-intersection and outward orientation. A single planar sheet face
//! is embedded when its sole loop is proven simple.

use crate::entity::{BodyKind, FaceId, RegionKind, Sense, ShellId, VertexId};
use crate::geom::{CurveGeom, SurfaceGeom};
use crate::loop_proof::{LoopSimplicity, certify_loop_simplicity};
use crate::store::Store;
use kcore::error::Result;
use kcore::predicates::{Orientation as PredicateOrientation, orient2d, orient3d};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::vec::Point2;

/// Proof state for global shell self-intersection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ShellEmbedding {
    /// The shell belongs to a proven embedded representation class.
    Certified,
    /// The current proof slice cannot establish global embedding.
    Indeterminate,
}

/// Proof state for a solid shell's global outward orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ShellOrientation {
    /// Every facet normal points away from the convex interior.
    Certified,
    /// At least one supporting facet normal provably points into the
    /// convex interior.
    Invalid,
    /// The current proof slice cannot establish an interior half-space.
    Indeterminate,
}

/// Independent embedding and orientation results for one shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ShellCertification {
    pub(crate) embedding: ShellEmbedding,
    pub(crate) orientation: ShellOrientation,
}

/// Attempt to certify one shell in the context of its owning body/region.
pub(crate) fn certify_shell(
    store: &Store,
    shell_id: ShellId,
    body_kind: BodyKind,
    region_kind: RegionKind,
) -> Result<ShellCertification> {
    let shell = store.get(shell_id)?;
    if body_kind == BodyKind::Sheet && shell.faces.len() == 1 {
        let face = store.get(shell.faces[0])?;
        let planar = matches!(store.get(face.surface)?, SurfaceGeom::Plane(_));
        let simple = face.loops.len() == 1
            && certify_loop_simplicity(store, face.loops[0])? == LoopSimplicity::Certified;
        return Ok(ShellCertification {
            embedding: if planar && simple {
                ShellEmbedding::Certified
            } else {
                ShellEmbedding::Indeterminate
            },
            orientation: ShellOrientation::Indeterminate,
        });
    }
    if body_kind != BodyKind::Solid || region_kind != RegionKind::Solid {
        return Ok(indeterminate());
    }
    certify_convex_planar_shell(store, shell_id)
}

fn indeterminate() -> ShellCertification {
    ShellCertification {
        embedding: ShellEmbedding::Indeterminate,
        orientation: ShellOrientation::Indeterminate,
    }
}

fn certify_convex_planar_shell(store: &Store, shell_id: ShellId) -> Result<ShellCertification> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 4 {
        return Ok(indeterminate());
    }
    let mut shell_vertices = Vec::new();
    let mut facets = Vec::with_capacity(shell.faces.len());
    for &face_id in &shell.faces {
        let Some(vertices) = convex_planar_face_vertices(store, face_id)? else {
            return Ok(indeterminate());
        };
        for &vertex in &vertices {
            if !shell_vertices.contains(&vertex) {
                shell_vertices.push(vertex);
            }
        }
        facets.push((face_id, vertices));
    }
    if shell_vertices.len() < 4 {
        return Ok(indeterminate());
    }

    let mut orientation_invalid = false;
    for (face_id, loop_vertices) in facets {
        let face = store.get(face_id)?;
        let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
            return Ok(indeterminate());
        };
        let frame = plane.frame();
        let a = frame.origin();
        let b = a + frame.x();
        let c = a + frame.y();
        let mut positive = false;
        let mut negative = false;
        let mut coplanar = Vec::new();
        for &vertex in &shell_vertices {
            let point = store.vertex_position(vertex)?;
            let signed_distance = (point - a).dot(frame.z());
            let side = if signed_distance.abs() <= LINEAR_RESOLUTION {
                PredicateOrientation::Zero
            } else {
                orient3d(a.to_array(), b.to_array(), c.to_array(), point.to_array())
            };
            match side {
                PredicateOrientation::Positive => positive = true,
                PredicateOrientation::Negative => negative = true,
                PredicateOrientation::Zero => coplanar.push(vertex),
            }
        }
        if positive == negative {
            // Both sides occupied, or the whole shell is coplanar.
            return Ok(indeterminate());
        }
        if coplanar.len() != loop_vertices.len()
            || coplanar
                .iter()
                .any(|vertex| !loop_vertices.contains(vertex))
        {
            // This conservative slice accepts one strict convex facet per
            // supporting plane; coplanar facet partitions remain future work.
            return Ok(indeterminate());
        }
        let expected = if positive {
            // orient3d is positive below the frame's +z plane, so +z points
            // away from vertices on that side.
            Sense::Forward
        } else {
            Sense::Reversed
        };
        orientation_invalid |= face.sense != expected;
    }

    Ok(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if orientation_invalid {
            ShellOrientation::Invalid
        } else {
            ShellOrientation::Certified
        },
    })
}

fn convex_planar_face_vertices(store: &Store, face_id: FaceId) -> Result<Option<Vec<VertexId>>> {
    let face = store.get(face_id)?;
    let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
        return Ok(None);
    };
    if face.loops.len() != 1 {
        return Ok(None);
    }
    let loop_id = face.loops[0];
    if certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified {
        return Ok(None);
    }
    let loop_ = store.get(loop_id)?;
    if loop_.fins.len() < 3 {
        return Ok(None);
    }
    let mut vertices = Vec::with_capacity(loop_.fins.len());
    let mut points = Vec::with_capacity(loop_.fins.len());
    for &fin_id in &loop_.fins {
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let Some(curve_id) = edge.curve else {
            return Ok(None);
        };
        if edge.tolerance.is_some() || !matches!(store.get(curve_id)?, CurveGeom::Line(_)) {
            return Ok(None);
        }
        let Some(vertex) = store.fin_tail(fin_id)? else {
            return Ok(None);
        };
        if vertices.contains(&vertex) {
            return Ok(None);
        }
        let local = plane.frame().to_local(store.vertex_position(vertex)?);
        vertices.push(vertex);
        points.push(Point2::new(local.x, local.y));
    }
    if !strictly_convex(&points) {
        return Ok(None);
    }
    Ok(Some(vertices))
}

fn strictly_convex(points: &[Point2]) -> bool {
    let mut winding = None;
    for index in 0..points.len() {
        let a = points[index];
        let b = points[(index + 1) % points.len()];
        let c = points[(index + 2) % points.len()];
        let turn = orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]);
        if turn == PredicateOrientation::Zero {
            return false;
        }
        if let Some(winding) = winding {
            if turn != winding {
                return false;
            }
        } else {
            winding = Some(turn);
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::make::{block, sphere};
    use crate::store::Store;
    use kgeom::frame::Frame;

    fn solid_shell(store: &Store, body: crate::entity::BodyId) -> ShellId {
        let solid = store
            .get(body)
            .unwrap()
            .regions
            .iter()
            .copied()
            .find(|&region| store.get(region).unwrap().kind == RegionKind::Solid)
            .unwrap();
        store.get(solid).unwrap().shells[0]
    }

    #[test]
    fn convex_block_shell_is_embedded_and_orientation_is_decidable() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        let shell = solid_shell(&store, body);
        assert_eq!(
            certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid).unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Certified,
            }
        );

        let face = store.get(shell).unwrap().faces[0];
        store.get_mut(face).unwrap().sense = store.get(face).unwrap().sense.flipped();
        assert_eq!(
            certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid)
                .unwrap()
                .orientation,
            ShellOrientation::Invalid
        );
    }

    #[test]
    fn curved_shell_remains_indeterminate() {
        let mut store = Store::new();
        let body = sphere(&mut store, &Frame::world(), 1.0).unwrap();
        let shell = solid_shell(&store, body);
        assert_eq!(
            certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid).unwrap(),
            indeterminate()
        );
    }
}
