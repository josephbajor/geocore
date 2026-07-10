//! The entity store: typed generational arenas for all topology and
//! attached geometry, with uniform access, deterministic traversal, and
//! copy-on-write transaction entry points.

use crate::entity::{
    Body, BodyId, Curve2dId, CurveId, Edge, EdgeId, EntityRef, Face, FaceId, Fin, FinId, Loop,
    PointId, Region, Shell, SurfaceId, Vertex, VertexId,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::transaction::{Mutation, MutationKind, Transaction};
use kcore::arena::{Arena, ArenaChangeKind, Handle};
use kcore::error::{Error, Result};
use kcore::tolerance::check_in_size_box;
use kgeom::vec::Point3;

/// Implemented by every type the [`Store`] can hold; maps the type to its
/// arena so access is uniform: `store.add(entity)`, `store.get(handle)?`.
pub trait Entity: Sized + Clone {
    /// The arena holding this entity type.
    fn arena(store: &Store) -> &Arena<Self>;
    /// The arena holding this entity type, mutably.
    fn arena_mut(store: &mut Store) -> &mut Arena<Self>;
    /// Erase a typed handle for diagnostics and journaling.
    fn entity_ref(handle: Handle<Self>) -> EntityRef;
}

macro_rules! entity_arena {
    ($ty:ty, $field:ident, $variant:ident) => {
        impl Entity for $ty {
            fn arena(store: &Store) -> &Arena<Self> {
                &store.$field
            }
            fn arena_mut(store: &mut Store) -> &mut Arena<Self> {
                &mut store.$field
            }
            fn entity_ref(handle: Handle<Self>) -> EntityRef {
                EntityRef::$variant(handle)
            }
        }
    };
}

/// Holds every entity of a modeling session part. All cross-references are
/// handles into these arenas; iteration order is slot order (deterministic).
///
/// Generic entity mutation is deliberately not public. Use checked body
/// builders or [`crate::transaction::Transaction`] methods; low-level import
/// reconstruction uses transaction-scoped [`crate::transaction::AssemblyStore`].
///
/// ```compile_fail
/// use ktopo::entity::{Body, BodyKind};
/// use ktopo::store::Store;
///
/// let mut store = Store::new();
/// // Direct topology insertion is not part of the public API.
/// store.add(Body { kind: BodyKind::Wire, regions: Vec::new() });
/// ```
///
/// ```compile_fail
/// use kgeom::frame::Frame;
/// use ktopo::{make, store::Store};
///
/// let mut store = Store::new();
/// let body = make::block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
/// // Reads are public, mutable entity borrows are not.
/// store.get_mut(body).unwrap().regions.clear();
/// ```
///
/// ```compile_fail
/// use ktopo::store::Store;
///
/// let mut store = Store::new();
/// let transaction = store.transaction().unwrap();
/// // Unchecked commit is reserved for topology internals.
/// transaction.commit().unwrap();
/// ```
#[derive(Default)]
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
    transaction_active: bool,
}

impl Clone for Store {
    fn clone(&self) -> Self {
        Self {
            bodies: self.bodies.clone(),
            regions: self.regions.clone(),
            shells: self.shells.clone(),
            faces: self.faces.clone(),
            loops: self.loops.clone(),
            fins: self.fins.clone(),
            edges: self.edges.clone(),
            vertices: self.vertices.clone(),
            curves: self.curves.clone(),
            surfaces: self.surfaces.clone(),
            points: self.points.clone(),
            curves_2d: self.curves_2d.clone(),
            transaction_active: false,
        }
    }
}

entity_arena!(Body, bodies, Body);
entity_arena!(Region, regions, Region);
entity_arena!(Shell, shells, Shell);
entity_arena!(Face, faces, Face);
entity_arena!(Loop, loops, Loop);
entity_arena!(Fin, fins, Fin);
entity_arena!(Edge, edges, Edge);
entity_arena!(Vertex, vertices, Vertex);
entity_arena!(CurveGeom, curves, Curve);
entity_arena!(SurfaceGeom, surfaces, Surface);
entity_arena!(Point3, points, Point);
entity_arena!(Curve2dGeom, curves_2d, Curve2d);

impl Store {
    /// Empty store.
    pub fn new() -> Store {
        Store::default()
    }

    /// Begin a scoped failure-atomic modeling transaction.
    pub fn transaction(&mut self) -> Result<Transaction<'_>> {
        if self.transaction_active {
            return Err(Error::TransactionActive);
        }
        self.bodies.begin_undo_frame();
        self.regions.begin_undo_frame();
        self.shells.begin_undo_frame();
        self.faces.begin_undo_frame();
        self.loops.begin_undo_frame();
        self.fins.begin_undo_frame();
        self.edges.begin_undo_frame();
        self.vertices.begin_undo_frame();
        self.curves.begin_undo_frame();
        self.surfaces.begin_undo_frame();
        self.points.begin_undo_frame();
        self.curves_2d.begin_undo_frame();
        self.transaction_active = true;
        Ok(Transaction::new(self))
    }

    pub(crate) fn commit_transaction(&mut self) -> Result<Vec<Mutation>> {
        if !self.transaction_active {
            return Err(Error::TransactionInactive);
        }
        let mut out = Vec::new();
        append_changes::<Body>(&mut self.bodies, &mut out)?;
        append_changes::<Region>(&mut self.regions, &mut out)?;
        append_changes::<Shell>(&mut self.shells, &mut out)?;
        append_changes::<Face>(&mut self.faces, &mut out)?;
        append_changes::<Loop>(&mut self.loops, &mut out)?;
        append_changes::<Fin>(&mut self.fins, &mut out)?;
        append_changes::<Edge>(&mut self.edges, &mut out)?;
        append_changes::<Vertex>(&mut self.vertices, &mut out)?;
        append_changes::<CurveGeom>(&mut self.curves, &mut out)?;
        append_changes::<SurfaceGeom>(&mut self.surfaces, &mut out)?;
        append_changes::<Point3>(&mut self.points, &mut out)?;
        append_changes::<Curve2dGeom>(&mut self.curves_2d, &mut out)?;
        self.transaction_active = false;
        Ok(out)
    }

    pub(crate) fn rollback_transaction(&mut self) -> Result<()> {
        if !self.transaction_active {
            return Err(Error::TransactionInactive);
        }
        self.curves_2d.rollback_undo_frame()?;
        self.points.rollback_undo_frame()?;
        self.surfaces.rollback_undo_frame()?;
        self.curves.rollback_undo_frame()?;
        self.vertices.rollback_undo_frame()?;
        self.edges.rollback_undo_frame()?;
        self.fins.rollback_undo_frame()?;
        self.loops.rollback_undo_frame()?;
        self.faces.rollback_undo_frame()?;
        self.shells.rollback_undo_frame()?;
        self.regions.rollback_undo_frame()?;
        self.bodies.rollback_undo_frame()?;
        self.transaction_active = false;
        Ok(())
    }

    /// Insert an entity for topology-internal construction or a scoped
    /// [`crate::transaction::AssemblyStore`].
    pub(crate) fn add<T: Entity>(&mut self, entity: T) -> Handle<T> {
        T::arena_mut(self).insert(entity)
    }

    /// Insert immutable point geometry after size-box validation.
    pub fn insert_point(&mut self, point: Point3) -> Result<PointId> {
        check_in_size_box(point.to_array())?;
        Ok(self.add(point))
    }

    /// Insert immutable 3D curve geometry.
    pub fn insert_curve(&mut self, curve: CurveGeom) -> CurveId {
        self.add(curve)
    }

    /// Insert immutable supporting-surface geometry.
    pub fn insert_surface(&mut self, surface: SurfaceGeom) -> SurfaceId {
        self.add(surface)
    }

    /// Insert immutable parameter-space curve geometry.
    pub fn insert_pcurve(&mut self, curve: Curve2dGeom) -> Curve2dId {
        self.add(curve)
    }

    /// Borrow an entity; [`Error::StaleHandle`] if removed or unknown.
    pub fn get<T: Entity>(&self, handle: Handle<T>) -> Result<&T> {
        T::arena(self).get(handle).ok_or(Error::StaleHandle)
    }

    /// Mutably borrow an entity; [`Error::StaleHandle`] if removed or
    /// unknown.
    pub(crate) fn get_mut<T: Entity>(&mut self, handle: Handle<T>) -> Result<&mut T> {
        T::arena_mut(self).get_mut(handle).ok_or(Error::StaleHandle)
    }

    /// Remove an entity, returning it; [`Error::StaleHandle`] if already
    /// gone. Removal never fixes up references *to* the entity — that is
    /// the caller's job (Euler operators do this correctly).
    pub(crate) fn remove<T: Entity>(&mut self, handle: Handle<T>) -> Result<T> {
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

fn append_changes<T: Entity>(arena: &mut Arena<T>, out: &mut Vec<Mutation>) -> Result<()> {
    out.extend(
        arena
            .commit_undo_frame()?
            .into_iter()
            .map(|change| Mutation {
                entity: T::entity_ref(change.handle()),
                kind: match change.kind() {
                    ArenaChangeKind::Created => MutationKind::Created,
                    ArenaChangeKind::Modified => MutationKind::Modified,
                    ArenaChangeKind::Deleted => MutationKind::Deleted,
                },
            }),
    );
    Ok(())
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

    #[test]
    fn nested_store_transaction_is_rejected() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        assert_eq!(
            transaction.store_mut().transaction().err(),
            Some(Error::TransactionActive)
        );
    }
}
