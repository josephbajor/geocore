//! The entity store: typed generational arenas for all topology and
//! attached geometry, with uniform access and deterministic traversal.

use crate::entity::{
    Body, BodyId, Edge, EdgeId, Face, FaceId, Fin, FinId, Loop, Region, Shell, Vertex, VertexId,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use kcore::arena::{Arena, Handle};
use kcore::error::{Error, Result};
use kgeom::vec::Point3;

/// Implemented by every type the [`Store`] can hold; maps the type to its
/// arena so access is uniform: `store.add(entity)`, `store.get(handle)?`.
pub trait Entity: Sized {
    /// The arena holding this entity type.
    fn arena(store: &Store) -> &Arena<Self>;
    /// The arena holding this entity type, mutably.
    fn arena_mut(store: &mut Store) -> &mut Arena<Self>;
}

macro_rules! entity_arena {
    ($ty:ty, $field:ident) => {
        impl Entity for $ty {
            fn arena(store: &Store) -> &Arena<Self> {
                &store.$field
            }
            fn arena_mut(store: &mut Store) -> &mut Arena<Self> {
                &mut store.$field
            }
        }
    };
}

/// Holds every entity of a modeling session part. All cross-references are
/// handles into these arenas; iteration order is slot order (deterministic).
#[derive(Clone, Default)]
pub struct Store {
    bodies: Arena<Body>,
    regions: Arena<Region>,
    shells: Arena<Shell>,
    faces: Arena<Face>,
    loops: Arena<Loop>,
    fins: Arena<Fin>,
    edges: Arena<Edge>,
    vertices: Arena<Vertex>,
    curves: Arena<CurveGeom>,
    surfaces: Arena<SurfaceGeom>,
    points: Arena<Point3>,
    curves_2d: Arena<Curve2dGeom>,
}

entity_arena!(Body, bodies);
entity_arena!(Region, regions);
entity_arena!(Shell, shells);
entity_arena!(Face, faces);
entity_arena!(Loop, loops);
entity_arena!(Fin, fins);
entity_arena!(Edge, edges);
entity_arena!(Vertex, vertices);
entity_arena!(CurveGeom, curves);
entity_arena!(SurfaceGeom, surfaces);
entity_arena!(Point3, points);
entity_arena!(Curve2dGeom, curves_2d);

impl Store {
    /// Empty store.
    pub fn new() -> Store {
        Store::default()
    }

    /// Insert an entity, returning its handle.
    pub fn add<T: Entity>(&mut self, entity: T) -> Handle<T> {
        T::arena_mut(self).insert(entity)
    }

    /// Borrow an entity; [`Error::StaleHandle`] if removed or unknown.
    pub fn get<T: Entity>(&self, handle: Handle<T>) -> Result<&T> {
        T::arena(self).get(handle).ok_or(Error::StaleHandle)
    }

    /// Mutably borrow an entity; [`Error::StaleHandle`] if removed or
    /// unknown.
    pub fn get_mut<T: Entity>(&mut self, handle: Handle<T>) -> Result<&mut T> {
        T::arena_mut(self).get_mut(handle).ok_or(Error::StaleHandle)
    }

    /// Remove an entity, returning it; [`Error::StaleHandle`] if already
    /// gone. Removal never fixes up references *to* the entity — that is
    /// the caller's job (Euler operators do this correctly).
    pub fn remove<T: Entity>(&mut self, handle: Handle<T>) -> Result<T> {
        T::arena_mut(self).remove(handle).ok_or(Error::StaleHandle)
    }

    /// True if the handle refers to a live entity.
    pub fn contains<T: Entity>(&self, handle: Handle<T>) -> bool {
        T::arena(self).contains(handle)
    }

    /// Number of live entities of one type.
    pub fn count<T: Entity>(&self) -> usize {
        T::arena(self).len()
    }

    /// Iterate live entities of one type in slot order (deterministic).
    pub fn iter<'a, T: Entity + 'a>(&'a self) -> impl Iterator<Item = (Handle<T>, &'a T)> + 'a {
        T::arena(self).iter()
    }

    /// All faces of a body, in region → shell → face stored order.
    pub fn faces_of_body(&self, body: BodyId) -> Result<Vec<FaceId>> {
        let mut out = Vec::new();
        for &region in &self.get(body)?.regions {
            for &shell in &self.get(region)?.shells {
                out.extend_from_slice(&self.get(shell)?.faces);
            }
        }
        Ok(out)
    }

    /// All edges of a body: face-loop edges first (deduplicated, first-
    /// traversal order), then shell wireframe edges. Deterministic.
    pub fn edges_of_body(&self, body: BodyId) -> Result<Vec<EdgeId>> {
        let mut out = Vec::new();
        let push = |e: EdgeId, out: &mut Vec<EdgeId>| {
            if !out.contains(&e) {
                out.push(e);
            }
        };
        for face in self.faces_of_body(body)? {
            for &lp in &self.get(face)?.loops {
                for &fin in &self.get(lp)?.fins {
                    push(self.get(fin)?.edge, &mut out);
                }
            }
        }
        for &region in &self.get(body)?.regions {
            for &shell in &self.get(region)?.shells {
                for &e in &self.get(shell)?.edges {
                    push(e, &mut out);
                }
            }
        }
        Ok(out)
    }

    /// All vertices of a body, deduplicated, in [`Self::edges_of_body`]
    /// order (plus any acorn shell vertex). Deterministic.
    pub fn vertices_of_body(&self, body: BodyId) -> Result<Vec<VertexId>> {
        let mut out = Vec::new();
        for edge in self.edges_of_body(body)? {
            for v in self.get(edge)?.vertices.into_iter().flatten() {
                if !out.contains(&v) {
                    out.push(v);
                }
            }
        }
        for &region in &self.get(body)?.regions {
            for &shell in &self.get(region)?.shells {
                if let Some(v) = self.get(shell)?.vertex
                    && !out.contains(&v)
                {
                    out.push(v);
                }
            }
        }
        Ok(out)
    }

    /// The vertex a fin starts from: the edge's start vertex for a
    /// `Forward` fin, its end vertex for a `Reversed` one. `None` on a
    /// ring edge.
    pub fn fin_tail(&self, fin: FinId) -> Result<Option<VertexId>> {
        let fin = self.get(fin)?;
        let edge = self.get(fin.edge)?;
        Ok(if fin.sense.is_forward() {
            edge.vertices[0]
        } else {
            edge.vertices[1]
        })
    }

    /// The vertex a fin ends at (see [`Self::fin_tail`]).
    pub fn fin_head(&self, fin: FinId) -> Result<Option<VertexId>> {
        let fin = self.get(fin)?;
        let edge = self.get(fin.edge)?;
        Ok(if fin.sense.is_forward() {
            edge.vertices[1]
        } else {
            edge.vertices[0]
        })
    }

    /// Position of a vertex.
    pub fn vertex_position(&self, vertex: VertexId) -> Result<Point3> {
        let v = self.get(vertex)?;
        Ok(*self.get(v.point)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::{BodyKind, RegionKind};

    #[test]
    fn add_get_roundtrip_and_stale_handles() {
        let mut store = Store::new();
        let body = store.add(Body {
            kind: BodyKind::Wire,
            regions: Vec::new(),
        });
        let region = store.add(Region {
            body,
            kind: RegionKind::Void,
            shells: Vec::new(),
        });
        store.get_mut(body).unwrap().regions.push(region);
        assert_eq!(store.get(body).unwrap().regions, vec![region]);
        assert_eq!(store.count::<Region>(), 1);
        store.remove(region).unwrap();
        assert_eq!(store.get(region), Err(Error::StaleHandle));
    }
}
