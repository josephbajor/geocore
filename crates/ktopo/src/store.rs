//! The entity store: typed generational arenas for topology and points plus
//! one authoritative geometry graph, with uniform compatibility reads,
//! deterministic traversal, and copy-on-write transaction entry points.
//!
//! # Stability boundary
//!
//! `Store` is lower-layer kernel infrastructure. Its deterministic reads and
//! checked transaction contracts are documented here for kernel implementors,
//! but its entity representation, handle vocabulary, and generic storage API
//! are not the supported application interface. Ordinary clients should use
//! the `kernel` facade. Trusted interchange code may currently read this store
//! and may assemble through [`crate::transaction::AssemblyStore`]; those seams
//! are explicitly subject to a later breaking encapsulation pass.

use crate::entity::{
    Body, BodyId, Curve2dId, CurveId, Edge, EdgeId, EntityRef, Face, FaceId, Fin, FinId, Loop,
    PointId, Region, Shell, SurfaceId, Vertex, VertexId,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::index::StoreIndex;
use crate::transaction::{Mutation, MutationKind, Transaction};
use kcore::arena::{Arena, ArenaChangeKind, Handle};
use kcore::error::{Error, Result};
use kcore::tolerance::{Tolerances, check_in_size_box};
use kgeom::vec::Point3;
use kgraph::{
    Curve2dHandle, GeometryGraph, GeometryGraphError, GeometryRef,
    PairedPlaneLineResidualCertificate, PairedPlaneSphereCircleResidualCertificate, SurfaceHandle,
    TransmittedNurbsIntersectionCertificate, TransmittedPlaneIntersectionCertificate,
};

pub(crate) mod sealed {
    use super::{EntityRef, Handle, Result, Store};

    pub trait Storage: Sized + Clone {
        fn get(store: &Store, handle: Handle<Self>) -> Option<&Self>;
        fn contains(store: &Store, handle: Handle<Self>) -> bool;
        fn count(store: &Store) -> usize;
        fn iter(store: &Store) -> Box<dyn Iterator<Item = (Handle<Self>, &Self)> + '_>;
        fn entity_ref(handle: Handle<Self>) -> EntityRef;
    }

    pub trait ArenaStorage: Storage {
        fn insert(store: &mut Store, value: Self) -> Handle<Self>;
        fn remove(store: &mut Store, handle: Handle<Self>) -> Result<Self>;
    }

    pub trait MutableStorage: ArenaStorage {
        fn get_mut(store: &mut Store, handle: Handle<Self>) -> Option<&mut Self>;
    }
}

/// Sealed marker implemented by every value readable through [`Store`].
#[allow(private_bounds)]
pub trait Entity: sealed::Storage {}

/// Sealed marker for topology entities and points owned by Store arenas.
#[allow(private_bounds)]
pub trait ArenaEntity: Entity + sealed::ArenaStorage {}

/// Store values whose representation may be mutated in place.
///
/// Geometry graph descriptors intentionally do not implement this trait;
/// replacement is fallible and must update dependency bookkeeping atomically.
#[allow(private_bounds)]
pub trait MutableEntity: ArenaEntity + sealed::MutableStorage {}

macro_rules! entity_arena {
    ($ty:ty, $field:ident, $variant:ident) => {
        impl sealed::Storage for $ty {
            fn get(store: &Store, handle: Handle<Self>) -> Option<&Self> {
                store.$field.get(handle)
            }
            fn contains(store: &Store, handle: Handle<Self>) -> bool {
                store.$field.contains(handle)
            }
            fn count(store: &Store) -> usize {
                store.$field.len()
            }
            fn iter(store: &Store) -> Box<dyn Iterator<Item = (Handle<Self>, &Self)> + '_> {
                Box::new(store.$field.iter())
            }
            fn entity_ref(handle: Handle<Self>) -> EntityRef {
                EntityRef::$variant(handle)
            }
        }
        impl Entity for $ty {}
        impl sealed::ArenaStorage for $ty {
            fn insert(store: &mut Store, value: Self) -> Handle<Self> {
                store.$field.insert(value)
            }
            fn remove(store: &mut Store, handle: Handle<Self>) -> Result<Self> {
                store.$field.remove(handle).ok_or(Error::StaleHandle)
            }
        }
        impl ArenaEntity for $ty {}
        impl sealed::MutableStorage for $ty {
            fn get_mut(store: &mut Store, handle: Handle<Self>) -> Option<&mut Self> {
                store.$field.get_mut(handle)
            }
        }
        impl MutableEntity for $ty {}
    };
}

macro_rules! geometry_entity {
    (
        $ty:ty,
        $get:ident,
        $count:ident,
        $iter:ident,
        $ref_variant:ident,
        $geom_variant:ident
    ) => {
        impl sealed::Storage for $ty {
            fn get(store: &Store, handle: Handle<Self>) -> Option<&Self> {
                store.geometry.$get(handle)
            }
            fn contains(store: &Store, handle: Handle<Self>) -> bool {
                store.geometry.contains(GeometryRef::$geom_variant(handle))
            }
            fn count(store: &Store) -> usize {
                store.geometry.$count()
            }
            fn iter(store: &Store) -> Box<dyn Iterator<Item = (Handle<Self>, &Self)> + '_> {
                Box::new(store.geometry.$iter())
            }
            fn entity_ref(handle: Handle<Self>) -> EntityRef {
                EntityRef::$ref_variant(handle)
            }
        }
        impl Entity for $ty {}
    };
}

/// Holds every entity of a modeling session part. All cross-references are
/// handles into these arenas; iteration order is slot order (deterministic).
///
/// Generic entity mutation is deliberately not public. Use checked body
/// builders or [`crate::transaction::Transaction`] methods; low-level import
/// reconstruction uses transaction-scoped [`crate::transaction::AssemblyStore`].
/// The concrete store/entity boundary is intentionally unstable for ordinary
/// external clients even where a lower-layer read method is currently public.
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
    geometry: GeometryGraph,
    points: Arena<Point3>,
    index: StoreIndex,
    index_dirty: bool,
    full_validation_required: bool,
    transaction_active: bool,
    #[cfg(feature = "benchmark-internals")]
    benchmark_observation: Option<crate::benchmark::CommitObservation>,
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
            geometry: self.geometry.clone(),
            points: self.points.clone(),
            index: self.index.clone(),
            // Arena clones snapshot the transaction's current state rather
            // than its entry state, so an active source transaction cannot
            // reuse the source's committed index without rebuilding it.
            index_dirty: self.index_dirty || self.transaction_active,
            full_validation_required: self.full_validation_required || self.transaction_active,
            transaction_active: false,
            #[cfg(feature = "benchmark-internals")]
            benchmark_observation: self.benchmark_observation,
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
entity_arena!(Point3, points, Point);
geometry_entity!(CurveGeom, curve, curve_count, curves, Curve, Curve);
geometry_entity!(
    SurfaceGeom,
    surface,
    surface_count,
    surfaces,
    Surface,
    Surface
);
geometry_entity!(
    Curve2dGeom,
    curve2d,
    curve2d_count,
    curves_2d,
    Curve2d,
    Curve2d
);

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
        if self.index_dirty {
            self.index = StoreIndex::build(self);
            self.index_dirty = false;
        }
        self.bodies.begin_undo_frame();
        self.regions.begin_undo_frame();
        self.shells.begin_undo_frame();
        self.faces.begin_undo_frame();
        self.loops.begin_undo_frame();
        self.fins.begin_undo_frame();
        self.edges.begin_undo_frame();
        self.vertices.begin_undo_frame();
        self.geometry.begin_undo_frame();
        self.points.begin_undo_frame();
        self.transaction_active = true;
        Ok(Transaction::new(self))
    }

    /// Inspect the active transaction's deterministic net mutations without
    /// consuming any undo frame.
    pub(crate) fn pending_transaction_mutations(&self) -> Result<Vec<Mutation>> {
        if !self.transaction_active {
            return Err(Error::TransactionInactive);
        }
        let mut out = Vec::new();
        append_pending_changes::<Body>(&self.bodies, &mut out)?;
        append_pending_changes::<Region>(&self.regions, &mut out)?;
        append_pending_changes::<Shell>(&self.shells, &mut out)?;
        append_pending_changes::<Face>(&self.faces, &mut out)?;
        append_pending_changes::<Loop>(&self.loops, &mut out)?;
        append_pending_changes::<Fin>(&self.fins, &mut out)?;
        append_pending_changes::<Edge>(&self.edges, &mut out)?;
        append_pending_changes::<Vertex>(&self.vertices, &mut out)?;
        let changes = self.geometry.pending_undo_frame_changes()?;
        append_geometry_changes::<CurveGeom>(changes.curves, &mut out);
        append_geometry_changes::<SurfaceGeom>(changes.surfaces, &mut out);
        append_pending_changes::<Point3>(&self.points, &mut out)?;
        append_geometry_changes::<Curve2dGeom>(changes.curves_2d, &mut out);
        Ok(out)
    }

    pub(crate) fn committed_index(&self) -> &StoreIndex {
        debug_assert!(!self.index_dirty);
        &self.index
    }

    pub(crate) fn full_validation_required(&self) -> bool {
        self.full_validation_required
    }

    pub(crate) fn install_committed_index(&mut self, index: StoreIndex) {
        self.index = index;
        self.index_dirty = false;
        self.full_validation_required = false;
    }

    #[cfg(feature = "benchmark-internals")]
    pub(crate) fn set_benchmark_observation(
        &mut self,
        observation: crate::benchmark::CommitObservation,
    ) {
        self.benchmark_observation = Some(observation);
    }

    #[cfg(feature = "benchmark-internals")]
    pub(crate) fn benchmark_observation(&self) -> Option<crate::benchmark::CommitObservation> {
        self.benchmark_observation
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
        let changes = self.geometry.commit_undo_frame()?;
        append_geometry_changes::<CurveGeom>(changes.curves, &mut out);
        append_geometry_changes::<SurfaceGeom>(changes.surfaces, &mut out);
        append_changes::<Point3>(&mut self.points, &mut out)?;
        append_geometry_changes::<Curve2dGeom>(changes.curves_2d, &mut out);
        self.transaction_active = false;
        Ok(out)
    }

    pub(crate) fn rollback_transaction(&mut self) -> Result<()> {
        if !self.transaction_active {
            return Err(Error::TransactionInactive);
        }
        self.geometry.rollback_undo_frame()?;
        self.points.rollback_undo_frame()?;
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
    pub(crate) fn add<T: ArenaEntity>(&mut self, entity: T) -> Handle<T> {
        let handle = <T as sealed::ArenaStorage>::insert(self, entity);
        if !self.transaction_active && is_topology(<T as sealed::Storage>::entity_ref(handle)) {
            self.index_dirty = true;
            self.full_validation_required = true;
        }
        handle
    }

    /// Insert immutable point geometry after size-box validation.
    pub fn insert_point(&mut self, point: Point3) -> Result<PointId> {
        Self::validate_point(point)?;
        Ok(self.add(point))
    }

    pub(crate) fn validate_point(point: Point3) -> Result<()> {
        check_in_size_box(point.to_array())
    }

    /// Insert immutable 3D curve geometry.
    pub fn insert_curve(&mut self, curve: CurveGeom) -> Result<CurveId> {
        self.geometry.insert_curve(curve).map_err(map_graph_error)
    }

    /// Insert a certified finite Plane/Plane intersection line with graph-owned
    /// source and pcurve proof bindings.
    pub fn insert_verified_plane_intersection_curve(
        &mut self,
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: PairedPlaneLineResidualCertificate,
    ) -> Result<CurveId> {
        self.geometry
            .insert_verified_plane_intersection_curve(source_surfaces, pcurves, certificate)
            .map_err(map_graph_error)
    }

    /// Insert a certified finite Plane/Sphere intersection circle with
    /// graph-owned exact-field source and pcurve proof bindings.
    pub fn insert_verified_plane_sphere_intersection_curve(
        &mut self,
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: PairedPlaneSphereCircleResidualCertificate,
    ) -> Result<CurveId> {
        self.geometry
            .insert_verified_plane_sphere_intersection_curve(source_surfaces, pcurves, certificate)
            .map_err(map_graph_error)
    }

    /// Insert a certified transmitted exact-plane-field intersection with
    /// graph-owned source and pcurve proof bindings.
    pub fn insert_verified_transmitted_plane_intersection_curve(
        &mut self,
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: TransmittedPlaneIntersectionCertificate,
    ) -> Result<CurveId> {
        self.geometry
            .insert_verified_transmitted_plane_intersection_curve(
                source_surfaces,
                pcurves,
                certificate,
            )
            .map_err(map_graph_error)
    }

    /// Insert a certified transmitted chart containing one or two original
    /// NURBS traces with graph-owned ordered source and pcurve proof bindings.
    pub fn insert_verified_transmitted_nurbs_intersection_curve(
        &mut self,
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: TransmittedNurbsIntersectionCertificate,
    ) -> Result<CurveId> {
        self.geometry
            .insert_verified_transmitted_nurbs_intersection_curve(
                source_surfaces,
                pcurves,
                certificate,
            )
            .map_err(map_graph_error)
    }

    /// Compatibility insertion name for the original mixed Plane/NURBS arm.
    pub fn insert_verified_transmitted_plane_nurbs_intersection_curve(
        &mut self,
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: TransmittedNurbsIntersectionCertificate,
    ) -> Result<CurveId> {
        self.insert_verified_transmitted_nurbs_intersection_curve(
            source_surfaces,
            pcurves,
            certificate,
        )
    }

    /// Insert immutable supporting-surface geometry.
    pub fn insert_surface(&mut self, surface: SurfaceGeom) -> Result<SurfaceId> {
        self.geometry
            .insert_surface(surface)
            .map_err(map_graph_error)
    }

    /// Insert immutable parameter-space curve geometry.
    pub fn insert_pcurve(&mut self, curve: Curve2dGeom) -> Result<Curve2dId> {
        self.geometry.insert_curve2d(curve).map_err(map_graph_error)
    }

    /// Borrow the authoritative geometry graph.
    pub fn geometry(&self) -> &GeometryGraph {
        &self.geometry
    }

    /// Construct a bounded evaluator borrowing this store's geometry graph.
    pub fn eval_context(
        &self,
        limits: kgraph::EvalLimits,
        tolerances: Tolerances,
    ) -> kgraph::EvalContext<'_> {
        kgraph::EvalContext::new(&self.geometry, limits, tolerances)
    }

    /// Borrow a live 3D curve descriptor explicitly.
    pub fn curve(&self, handle: CurveId) -> Result<&CurveGeom> {
        self.geometry.curve(handle).ok_or(Error::StaleHandle)
    }

    /// Borrow a live supporting-surface descriptor explicitly.
    pub fn surface(&self, handle: SurfaceId) -> Result<&SurfaceGeom> {
        self.geometry.surface(handle).ok_or(Error::StaleHandle)
    }

    /// Borrow a live parameter-space curve descriptor explicitly.
    pub fn pcurve(&self, handle: Curve2dId) -> Result<&Curve2dGeom> {
        self.geometry.curve2d(handle).ok_or(Error::StaleHandle)
    }

    pub(crate) fn replace_curve(&mut self, handle: CurveId, curve: CurveGeom) -> Result<CurveGeom> {
        self.require_active_transaction()?;
        self.geometry
            .replace_curve(handle, curve)
            .map_err(map_graph_error)
    }

    pub(crate) fn replace_surface(
        &mut self,
        handle: SurfaceId,
        surface: SurfaceGeom,
    ) -> Result<SurfaceGeom> {
        self.require_active_transaction()?;
        self.geometry
            .replace_surface(handle, surface)
            .map_err(map_graph_error)
    }

    pub(crate) fn replace_pcurve(
        &mut self,
        handle: Curve2dId,
        curve: Curve2dGeom,
    ) -> Result<Curve2dGeom> {
        self.require_active_transaction()?;
        self.geometry
            .replace_curve2d(handle, curve)
            .map_err(map_graph_error)
    }

    pub(crate) fn remove_curve(&mut self, handle: CurveId) -> Result<CurveGeom> {
        self.require_active_transaction()?;
        self.geometry.remove_curve(handle).map_err(map_graph_error)
    }

    pub(crate) fn remove_surface(&mut self, handle: SurfaceId) -> Result<SurfaceGeom> {
        self.require_active_transaction()?;
        self.geometry
            .remove_surface(handle)
            .map_err(map_graph_error)
    }

    pub(crate) fn remove_pcurve(&mut self, handle: Curve2dId) -> Result<Curve2dGeom> {
        self.require_active_transaction()?;
        self.geometry
            .remove_curve2d(handle)
            .map_err(map_graph_error)
    }

    fn require_active_transaction(&self) -> Result<()> {
        if self.transaction_active {
            Ok(())
        } else {
            Err(Error::TransactionInactive)
        }
    }

    pub(crate) fn validate_geometry(&self) -> Result<()> {
        self.geometry.validate().map_err(map_graph_error)
    }

    /// Borrow an entity; [`Error::StaleHandle`] if removed or unknown.
    pub fn get<T: Entity>(&self, handle: Handle<T>) -> Result<&T> {
        <T as sealed::Storage>::get(self, handle).ok_or(Error::StaleHandle)
    }

    /// Mutably borrow an entity; [`Error::StaleHandle`] if removed or
    /// unknown.
    pub(crate) fn get_mut<T: MutableEntity>(&mut self, handle: Handle<T>) -> Result<&mut T> {
        if !self.transaction_active {
            self.index_dirty = true;
            self.full_validation_required = true;
        }
        <T as sealed::MutableStorage>::get_mut(self, handle).ok_or(Error::StaleHandle)
    }

    /// Remove an entity, returning it; [`Error::StaleHandle`] if already
    /// gone. Removal never fixes up references *to* the entity — that is
    /// the caller's job (Euler operators do this correctly).
    pub(crate) fn remove<T: ArenaEntity>(&mut self, handle: Handle<T>) -> Result<T> {
        if !self.transaction_active {
            self.index_dirty = true;
            self.full_validation_required = true;
        }
        <T as sealed::ArenaStorage>::remove(self, handle)
    }

    /// True if the handle refers to a live entity.
    pub fn contains<T: Entity>(&self, handle: Handle<T>) -> bool {
        <T as sealed::Storage>::contains(self, handle)
    }

    /// Number of live entities of one type.
    pub fn count<T: Entity>(&self) -> usize {
        <T as sealed::Storage>::count(self)
    }

    /// Iterate live entities of one type in slot order (deterministic).
    pub fn iter<'a, T: Entity + 'a>(&'a self) -> impl Iterator<Item = (Handle<T>, &'a T)> + 'a {
        <T as sealed::Storage>::iter(self)
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
            .map(change_to_mutation::<T>),
    );
    Ok(())
}

fn append_pending_changes<T: Entity>(arena: &Arena<T>, out: &mut Vec<Mutation>) -> Result<()> {
    out.extend(
        arena
            .pending_undo_frame_changes()?
            .into_iter()
            .map(change_to_mutation::<T>),
    );
    Ok(())
}

fn append_geometry_changes<T: Entity>(
    changes: Vec<kcore::arena::ArenaChange<T>>,
    out: &mut Vec<Mutation>,
) {
    out.extend(changes.into_iter().map(change_to_mutation::<T>));
}

fn change_to_mutation<T: Entity>(change: kcore::arena::ArenaChange<T>) -> Mutation {
    Mutation {
        entity: <T as sealed::Storage>::entity_ref(change.handle()),
        kind: match change.kind() {
            ArenaChangeKind::Created => MutationKind::Created,
            ArenaChangeKind::Modified => MutationKind::Modified,
            ArenaChangeKind::Deleted => MutationKind::Deleted,
        },
    }
}

fn is_topology(entity: EntityRef) -> bool {
    matches!(
        entity,
        EntityRef::Body(_)
            | EntityRef::Region(_)
            | EntityRef::Shell(_)
            | EntityRef::Face(_)
            | EntityRef::Loop(_)
            | EntityRef::Fin(_)
            | EntityRef::Edge(_)
            | EntityRef::Vertex(_)
    )
}

fn map_graph_error(error: GeometryGraphError) -> Error {
    match error {
        GeometryGraphError::StaleGeometryHandle { .. } => Error::StaleHandle,
        GeometryGraphError::InvalidDescriptor { reason, .. } => Error::InvalidGeometry { reason },
        GeometryGraphError::HasDependents { .. } => Error::InvalidGeometry {
            reason: "geometry node still has graph dependents",
        },
        GeometryGraphError::DependencyCycle { .. } => Error::InvalidGeometry {
            reason: "geometry graph contains a dependency cycle",
        },
        GeometryGraphError::ReverseDependencyMismatch { .. } => Error::InvalidGeometry {
            reason: "geometry graph reverse dependency index is inconsistent",
        },
        _ => Error::InvalidGeometry {
            reason: "geometry graph validation failed",
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::{BodyKind, RegionKind};
    use crate::geom::CurveGeom;
    use crate::transaction::MutationKind;
    use kcore::operation::{OperationContext, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::curve::Line;
    use kgeom::frame::Frame;
    use kgraph::EvalBudgetProfile;

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

    #[test]
    fn topology_internal_out_of_transaction_mutation_forces_a_full_audit() {
        let mut store = Store::new();
        let invalid = crate::make::block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
        let unchanged = crate::make::block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
        store.get_mut(invalid).unwrap().regions.clear();

        let transaction = store.transaction().unwrap();
        assert!(matches!(
            transaction.commit_checked_body(unchanged),
            Err(Error::TopologyCheckFailed { fault_count }) if fault_count > 0
        ));
        assert!(store.get(invalid).unwrap().regions.is_empty());
    }

    #[test]
    fn contextual_checked_commit_limit_rolls_back_arena_and_allocator_exactly() {
        let mut store = Store::new();
        let body = crate::make::block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
        let before = store.count::<Point3>();
        let mut transaction = store.transaction().unwrap();
        let rolled_back = transaction.assembly().add(Point3::new(7.0, 8.0, 9.0));

        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_budget_overrides(EvalBudgetProfile::for_limits(64, 8).with_total_work_limit(1));
        let outcome = transaction
            .commit_checked_body_with_context(body, &context)
            .unwrap();
        let expected = kcore::operation::LimitSnapshot {
            stage: kcore::operation::TOTAL_WORK_STAGE,
            resource: kcore::operation::ResourceKind::Work,
            consumed: 2,
            allowed: 1,
        };
        assert_eq!(
            outcome.result().as_ref().unwrap_err().limit(),
            Some(expected)
        );
        assert_eq!(outcome.report().limit_events(), &[expected]);
        assert_eq!(store.count::<Point3>(), before);
        let reused = store.add(Point3::new(7.0, 8.0, 9.0));
        assert_eq!(reused, rolled_back);
    }

    #[test]
    fn graph_owned_geometry_replacement_is_journaled_and_rollback_exact() {
        let mut store = Store::new();
        let curve = store
            .insert_curve(CurveGeom::Line(
                Line::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)).unwrap(),
            ))
            .unwrap();
        assert_eq!(store.geometry().curve_count(), 1);
        assert_eq!(store.count::<CurveGeom>(), 1);
        assert!(core::ptr::eq(
            store.curve(curve).unwrap(),
            store.geometry().curve(curve).unwrap()
        ));

        let original = store.curve(curve).unwrap().clone();
        let mut transaction = store.transaction().unwrap();
        transaction
            .assembly()
            .replace_curve(
                curve,
                CurveGeom::Line(
                    Line::new(Point3::new(2.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)).unwrap(),
                ),
            )
            .unwrap();
        transaction.rollback().unwrap();
        assert_eq!(store.curve(curve).unwrap(), &original);

        let mut transaction = store.transaction().unwrap();
        transaction
            .assembly()
            .replace_curve(
                curve,
                CurveGeom::Line(
                    Line::new(Point3::new(3.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)).unwrap(),
                ),
            )
            .unwrap();
        let journal = transaction.commit_checked(&[]).unwrap();
        assert!(journal.mutations().iter().any(|mutation| {
            mutation.entity == EntityRef::Curve(curve) && mutation.kind == MutationKind::Modified
        }));
        store.geometry().validate().unwrap();
    }

    #[test]
    fn invalid_geometry_insertion_is_fallible_and_leaves_graph_unchanged() {
        let mut store = Store::new();
        let invalid =
            Line::new(Point3::new(f64::NAN, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)).unwrap();

        assert!(matches!(
            store.insert_curve(CurveGeom::Line(invalid)),
            Err(Error::InvalidGeometry { .. })
        ));
        assert_eq!(store.geometry().curve_count(), 0);
        store.geometry().validate().unwrap();
    }

    #[test]
    fn graph_dependent_removal_error_retains_its_meaning_at_topology_boundary() {
        let mut graph = GeometryGraph::new();
        let basis = graph
            .insert_curve(Line::new(Point3::default(), Point3::new(1.0, 0.0, 0.0)).unwrap())
            .unwrap();
        let dependent = graph
            .insert_curve(
                Line::new(Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 0.0, 0.0)).unwrap(),
            )
            .unwrap();
        let mapped = map_graph_error(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Curve(basis),
            dependents: vec![GeometryRef::Curve(dependent)],
        });
        assert_eq!(
            mapped,
            Error::InvalidGeometry {
                reason: "geometry node still has graph dependents"
            }
        );
    }
}
