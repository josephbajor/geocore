//! Committed topology ownership and geometry-to-body dependency index.
//!
//! The index is deliberately private to the topology layer. It turns a
//! transaction's deterministic raw mutations into the body roots whose Fast
//! invariants may have changed. Building an index also audits the global
//! ownership closure: every live non-body topology entity must be reachable
//! from exactly one live body. Geometry may be shared by any number of bodies.

use crate::entity::{
    Body, BodyId, Curve2dId, CurveId, Edge, EdgeId, EntityRef, Face, FaceId, Fin, FinId, Loop,
    LoopId, PointId, Region, RegionId, Shell, ShellId, SurfaceId, Vertex, VertexId,
};
use crate::store::{Entity, Store};
use crate::transaction::Mutation;
use kcore::arena::Handle;
use kgraph::{GeometryGraph, GeometryRef};
use std::collections::HashMap;

/// Snapshot of body ownership and shared geometry dependencies for one Store
/// state. Values preserve deterministic body slot order even though lookup is
/// hash-based.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct BodyFootprint {
    regions: Vec<RegionId>,
    shells: Vec<ShellId>,
    faces: Vec<FaceId>,
    loops: Vec<LoopId>,
    fins: Vec<FinId>,
    edges: Vec<EdgeId>,
    vertices: Vec<VertexId>,
    curves: Vec<CurveId>,
    surfaces: Vec<SurfaceId>,
    points: Vec<PointId>,
    pcurves: Vec<Curve2dId>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct StoreIndex {
    regions: HashMap<RegionId, BodyId>,
    shells: HashMap<ShellId, BodyId>,
    faces: HashMap<FaceId, BodyId>,
    loops: HashMap<LoopId, BodyId>,
    fins: HashMap<FinId, BodyId>,
    edges: HashMap<EdgeId, BodyId>,
    vertices: HashMap<VertexId, BodyId>,
    curves: HashMap<CurveId, Vec<BodyId>>,
    surfaces: HashMap<SurfaceId, Vec<BodyId>>,
    points: HashMap<PointId, Vec<BodyId>>,
    pcurves: HashMap<Curve2dId, Vec<BodyId>>,
    footprints: HashMap<BodyId, BodyFootprint>,
    body_order: Vec<BodyId>,
    body_ranks: HashMap<BodyId, usize>,
    ownership_fault_count: usize,
}

#[cfg(feature = "benchmark-internals")]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct CandidateIndexObservation {
    pub(crate) clone_starts: usize,
    pub(crate) cloned_body_footprints: usize,
    pub(crate) cloned_body_order_entries: usize,
    pub(crate) refresh_body_starts: usize,
    pub(crate) body_order_refresh_entries: usize,
    pub(crate) affected_selection_starts: usize,
    pub(crate) affected_selection_mutation_items: usize,
}

#[cfg(feature = "benchmark-internals")]
impl CandidateIndexObservation {
    pub(crate) fn observe_affected_selection(&mut self, mutations: &[Mutation]) {
        self.affected_selection_starts += 1;
        self.affected_selection_mutation_items += mutations.len();
    }
}

impl StoreIndex {
    /// Build an index for the Store's current state and audit ownership
    /// closure. Stale geometry references are indexed so the affected body is
    /// still selected and the body checker can report the actual fault.
    pub(crate) fn build(store: &Store) -> Self {
        let mut index = Self::default();
        index.set_body_order(store);
        for (body_id, body) in store.iter::<Body>() {
            index.rebuild_body(store, body_id, body);
        }

        index.ownership_fault_count += unclaimed::<Region>(store, &index.regions)
            + unclaimed::<Shell>(store, &index.shells)
            + unclaimed::<Face>(store, &index.faces)
            + unclaimed::<Loop>(store, &index.loops)
            + unclaimed::<Fin>(store, &index.fins)
            + unclaimed::<Edge>(store, &index.edges)
            + unclaimed::<Vertex>(store, &index.vertices);
        index
    }

    pub(crate) fn ownership_fault_count(&self) -> usize {
        self.ownership_fault_count
    }

    /// Bodies affected by the supplied raw mutations, in mutation order with
    /// old owners/dependents before new ones and duplicates removed.
    pub(crate) fn affected_bodies(&self, previous: &Self, mutations: &[Mutation]) -> Vec<BodyId> {
        let mut bodies = Vec::new();
        for mutation in mutations {
            match mutation.entity {
                EntityRef::Body(body) => push_body(&mut bodies, body),
                EntityRef::Region(region) => {
                    push_optional(&mut bodies, previous.regions.get(&region).copied());
                    push_optional(&mut bodies, self.regions.get(&region).copied());
                }
                EntityRef::Shell(shell) => {
                    push_optional(&mut bodies, previous.shells.get(&shell).copied());
                    push_optional(&mut bodies, self.shells.get(&shell).copied());
                }
                EntityRef::Face(face) => {
                    push_optional(&mut bodies, previous.faces.get(&face).copied());
                    push_optional(&mut bodies, self.faces.get(&face).copied());
                }
                EntityRef::Loop(loop_) => {
                    push_optional(&mut bodies, previous.loops.get(&loop_).copied());
                    push_optional(&mut bodies, self.loops.get(&loop_).copied());
                }
                EntityRef::Fin(fin) => {
                    push_optional(&mut bodies, previous.fins.get(&fin).copied());
                    push_optional(&mut bodies, self.fins.get(&fin).copied());
                }
                EntityRef::Edge(edge) => {
                    push_optional(&mut bodies, previous.edges.get(&edge).copied());
                    push_optional(&mut bodies, self.edges.get(&edge).copied());
                }
                EntityRef::Vertex(vertex) => {
                    push_optional(&mut bodies, previous.vertices.get(&vertex).copied());
                    push_optional(&mut bodies, self.vertices.get(&vertex).copied());
                }
                EntityRef::Curve(curve) => {
                    push_dependencies(&mut bodies, previous.curves.get(&curve));
                    push_dependencies(&mut bodies, self.curves.get(&curve));
                }
                EntityRef::Surface(surface) => {
                    push_dependencies(&mut bodies, previous.surfaces.get(&surface));
                    push_dependencies(&mut bodies, self.surfaces.get(&surface));
                }
                EntityRef::Point(point) => {
                    push_dependencies(&mut bodies, previous.points.get(&point));
                    push_dependencies(&mut bodies, self.points.get(&point));
                }
                EntityRef::Curve2d(pcurve) => {
                    push_dependencies(&mut bodies, previous.pcurves.get(&pcurve));
                    push_dependencies(&mut bodies, self.pcurves.get(&pcurve));
                }
            }
        }
        bodies
    }

    /// Incrementally replace only body footprints implicated by the pending
    /// mutations. The committed index is assumed ownership-clean; callers use
    /// [`Self::build`] after topology-internal out-of-transaction mutation.
    #[cfg(not(feature = "benchmark-internals"))]
    pub(crate) fn candidate(store: &Store, previous: &Self, mutations: &[Mutation]) -> Self {
        Self::candidate_with_stats(store, previous, mutations).0
    }

    #[cfg(any(not(feature = "benchmark-internals"), test))]
    pub(crate) fn candidate_with_stats(
        store: &Store,
        previous: &Self,
        mutations: &[Mutation],
    ) -> (Self, usize) {
        #[cfg(feature = "benchmark-internals")]
        {
            Self::candidate_impl(store, previous, mutations, None)
        }
        #[cfg(not(feature = "benchmark-internals"))]
        {
            Self::candidate_impl(store, previous, mutations)
        }
    }

    #[cfg(feature = "benchmark-internals")]
    pub(crate) fn candidate_with_benchmark_observation(
        store: &Store,
        previous: &Self,
        mutations: &[Mutation],
    ) -> (Self, usize, CandidateIndexObservation) {
        let mut observation = CandidateIndexObservation::default();
        let (candidate, refreshed_bodies) =
            Self::candidate_impl(store, previous, mutations, Some(&mut observation));
        (candidate, refreshed_bodies, observation)
    }

    fn candidate_impl(
        store: &Store,
        previous: &Self,
        mutations: &[Mutation],
        #[cfg(feature = "benchmark-internals")] mut observation: Option<
            &mut CandidateIndexObservation,
        >,
    ) -> (Self, usize) {
        #[cfg(feature = "benchmark-internals")]
        if let Some(observation) = observation.as_deref_mut() {
            observation.observe_affected_selection(mutations);
            observation.clone_starts += 1;
            observation.cloned_body_footprints += previous.footprints.len();
            observation.cloned_body_order_entries += previous.body_order.len();
        }
        let affected = previous.affected_bodies(previous, mutations);
        let mut candidate = previous.clone();
        candidate.ownership_fault_count = 0;
        if affected.is_empty() {
            candidate.audit_mutated_topology(store, mutations, &[]);
            return (candidate, 0);
        }

        candidate.set_body_order(store);
        #[cfg(feature = "benchmark-internals")]
        if let Some(observation) = observation.as_deref_mut() {
            observation.body_order_refresh_entries += candidate.body_order.len();
        }

        let mut removed = Vec::new();
        for &body in &affected {
            if let Some(footprint) = candidate.remove_body(body) {
                removed.push(footprint);
            }
        }
        let body_order = candidate.body_order.clone();
        let mut rebuilt_bodies = 0usize;
        for body_id in body_order {
            if affected.contains(&body_id)
                && let Ok(body) = store.get(body_id)
            {
                #[cfg(feature = "benchmark-internals")]
                if let Some(observation) = observation.as_deref_mut() {
                    observation.refresh_body_starts += 1;
                }
                candidate.rebuild_body(store, body_id, body);
                rebuilt_bodies += 1;
            }
        }
        candidate.audit_mutated_topology(store, mutations, &removed);
        (candidate, rebuilt_bodies)
    }

    #[cfg(feature = "benchmark-internals")]
    pub(crate) fn benchmark_snapshot(&self, store: &Store) -> crate::benchmark::IndexSnapshot {
        use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
        use std::collections::HashMap;

        fn ordinals<T: Entity>(store: &Store) -> HashMap<Handle<T>, u64> {
            store
                .iter::<T>()
                .enumerate()
                .map(|(ordinal, (handle, _))| (handle, ordinal as u64))
                .collect()
        }
        fn write_handles<T>(
            digest: &mut crate::benchmark::StableHasher,
            handles: &[Handle<T>],
            ordinals: &HashMap<Handle<T>, u64>,
        ) {
            digest.write_count(handles.len());
            for handle in handles {
                digest.write_ordinal(ordinals.get(handle).copied());
            }
        }

        let body_ordinals = ordinals::<Body>(store);
        let region_ordinals = ordinals::<Region>(store);
        let shell_ordinals = ordinals::<Shell>(store);
        let face_ordinals = ordinals::<Face>(store);
        let loop_ordinals = ordinals::<Loop>(store);
        let fin_ordinals = ordinals::<Fin>(store);
        let edge_ordinals = ordinals::<Edge>(store);
        let vertex_ordinals = ordinals::<Vertex>(store);
        let curve_ordinals = ordinals::<CurveGeom>(store);
        let surface_ordinals = ordinals::<SurfaceGeom>(store);
        let point_ordinals = ordinals::<kgeom::vec::Point3>(store);
        let pcurve_ordinals = ordinals::<Curve2dGeom>(store);
        let mut digest = crate::benchmark::StableHasher::new();
        digest.write_tag(0x51);
        digest.write_count(self.body_order.len());
        for &body in &self.body_order {
            digest.write_ordinal(body_ordinals.get(&body).copied());
            if let Some(footprint) = self.footprints.get(&body) {
                digest.write_tag(1);
                write_handles(&mut digest, &footprint.regions, &region_ordinals);
                write_handles(&mut digest, &footprint.shells, &shell_ordinals);
                write_handles(&mut digest, &footprint.faces, &face_ordinals);
                write_handles(&mut digest, &footprint.loops, &loop_ordinals);
                write_handles(&mut digest, &footprint.fins, &fin_ordinals);
                write_handles(&mut digest, &footprint.edges, &edge_ordinals);
                write_handles(&mut digest, &footprint.vertices, &vertex_ordinals);
                write_handles(&mut digest, &footprint.curves, &curve_ordinals);
                write_handles(&mut digest, &footprint.surfaces, &surface_ordinals);
                write_handles(&mut digest, &footprint.points, &point_ordinals);
                write_handles(&mut digest, &footprint.pcurves, &pcurve_ordinals);
            } else {
                digest.write_tag(0);
            }
        }
        digest.write_count(self.ownership_fault_count);
        crate::benchmark::IndexSnapshot {
            bodies: self.body_order.len(),
            ownership_entries: self.regions.len()
                + self.shells.len()
                + self.faces.len()
                + self.loops.len()
                + self.fins.len()
                + self.edges.len()
                + self.vertices.len(),
            dependency_entries: self
                .body_order
                .iter()
                .filter_map(|body| self.footprints.get(body))
                .map(|footprint| {
                    footprint.curves.len()
                        + footprint.surfaces.len()
                        + footprint.points.len()
                        + footprint.pcurves.len()
                })
                .sum(),
            ownership_faults: self.ownership_fault_count,
            digest: digest.finish_stable(),
        }
    }

    /// Debug/test oracle: clean incremental candidates must equal a full
    /// deterministic rebuild; invalid candidates must at least agree that the
    /// ownership closure is not valid.
    pub(crate) fn debug_assert_full_rebuild_parity(&self, _store: &Store) {
        #[cfg(debug_assertions)]
        {
            let rebuilt = Self::build(_store);
            debug_assert_eq!(
                self.ownership_fault_count == 0,
                rebuilt.ownership_fault_count == 0,
                "incremental and full ownership audits disagree"
            );
            if self.ownership_fault_count == 0 {
                debug_assert_eq!(self, &rebuilt);
            }
        }
    }

    fn set_body_order(&mut self, store: &Store) {
        self.body_order = store.iter::<Body>().map(|(body, _)| body).collect();
        self.body_ranks.clear();
        for (rank, &body) in self.body_order.iter().enumerate() {
            self.body_ranks.insert(body, rank);
        }
    }

    fn rebuild_body(&mut self, store: &Store, body_id: BodyId, body: &Body) {
        let mut footprint = BodyFootprint::default();
        for &region_id in &body.regions {
            if !claim(
                &mut self.regions,
                region_id,
                body_id,
                &mut self.ownership_fault_count,
            ) {
                continue;
            }
            footprint.regions.push(region_id);
            let Ok(region) = store.get(region_id) else {
                continue;
            };
            for &shell_id in &region.shells {
                self.walk_shell(store, body_id, shell_id, &mut footprint);
            }
        }
        self.footprints.insert(body_id, footprint);
    }

    fn remove_body(&mut self, body: BodyId) -> Option<BodyFootprint> {
        let footprint = self.footprints.remove(&body)?;
        remove_owned(&mut self.regions, &footprint.regions, body);
        remove_owned(&mut self.shells, &footprint.shells, body);
        remove_owned(&mut self.faces, &footprint.faces, body);
        remove_owned(&mut self.loops, &footprint.loops, body);
        remove_owned(&mut self.fins, &footprint.fins, body);
        remove_owned(&mut self.edges, &footprint.edges, body);
        remove_owned(&mut self.vertices, &footprint.vertices, body);
        remove_dependencies(&mut self.curves, &footprint.curves, body);
        remove_dependencies(&mut self.surfaces, &footprint.surfaces, body);
        remove_dependencies(&mut self.points, &footprint.points, body);
        remove_dependencies(&mut self.pcurves, &footprint.pcurves, body);
        Some(footprint)
    }

    fn audit_mutated_topology(
        &mut self,
        store: &Store,
        mutations: &[Mutation],
        removed: &[BodyFootprint],
    ) {
        let mut unowned = Vec::new();
        for footprint in removed {
            collect_live_unowned(store, &self.regions, &footprint.regions, &mut unowned);
            collect_live_unowned(store, &self.shells, &footprint.shells, &mut unowned);
            collect_live_unowned(store, &self.faces, &footprint.faces, &mut unowned);
            collect_live_unowned(store, &self.loops, &footprint.loops, &mut unowned);
            collect_live_unowned(store, &self.fins, &footprint.fins, &mut unowned);
            collect_live_unowned(store, &self.edges, &footprint.edges, &mut unowned);
            collect_live_unowned(store, &self.vertices, &footprint.vertices, &mut unowned);
        }
        for mutation in mutations {
            if self.live_unowned_topology(store, mutation.entity)
                && !unowned.contains(&mutation.entity)
            {
                unowned.push(mutation.entity);
            }
        }
        self.ownership_fault_count += unowned.len();
    }

    fn live_unowned_topology(&self, store: &Store, entity: EntityRef) -> bool {
        match entity {
            EntityRef::Region(id) => store.contains(id) && !self.regions.contains_key(&id),
            EntityRef::Shell(id) => store.contains(id) && !self.shells.contains_key(&id),
            EntityRef::Face(id) => store.contains(id) && !self.faces.contains_key(&id),
            EntityRef::Loop(id) => store.contains(id) && !self.loops.contains_key(&id),
            EntityRef::Fin(id) => store.contains(id) && !self.fins.contains_key(&id),
            EntityRef::Edge(id) => store.contains(id) && !self.edges.contains_key(&id),
            EntityRef::Vertex(id) => store.contains(id) && !self.vertices.contains_key(&id),
            EntityRef::Body(_)
            | EntityRef::Curve(_)
            | EntityRef::Surface(_)
            | EntityRef::Point(_)
            | EntityRef::Curve2d(_) => false,
        }
    }

    fn walk_shell(
        &mut self,
        store: &Store,
        body: BodyId,
        shell_id: ShellId,
        footprint: &mut BodyFootprint,
    ) {
        if !claim(
            &mut self.shells,
            shell_id,
            body,
            &mut self.ownership_fault_count,
        ) {
            return;
        }
        footprint.shells.push(shell_id);
        let Ok(shell) = store.get(shell_id) else {
            return;
        };
        for &face_id in &shell.faces {
            self.walk_face(store, body, face_id, footprint);
        }
        for &edge_id in &shell.edges {
            self.walk_edge(store, body, edge_id, footprint);
        }
        if let Some(vertex_id) = shell.vertex {
            self.walk_vertex(store, body, vertex_id, footprint);
        }
    }

    fn walk_face(
        &mut self,
        store: &Store,
        body: BodyId,
        face_id: FaceId,
        footprint: &mut BodyFootprint,
    ) {
        if !claim(
            &mut self.faces,
            face_id,
            body,
            &mut self.ownership_fault_count,
        ) {
            return;
        }
        footprint.faces.push(face_id);
        let Ok(face) = store.get(face_id) else {
            return;
        };
        self.add_geometry_dependency(
            store.geometry(),
            GeometryRef::Surface(face.surface),
            body,
            footprint,
        );
        for &loop_id in &face.loops {
            self.walk_loop(store, body, loop_id, footprint);
        }
    }

    fn walk_loop(
        &mut self,
        store: &Store,
        body: BodyId,
        loop_id: LoopId,
        footprint: &mut BodyFootprint,
    ) {
        if !claim(
            &mut self.loops,
            loop_id,
            body,
            &mut self.ownership_fault_count,
        ) {
            return;
        }
        footprint.loops.push(loop_id);
        let Ok(loop_) = store.get(loop_id) else {
            return;
        };
        for &fin_id in &loop_.fins {
            if !claim(
                &mut self.fins,
                fin_id,
                body,
                &mut self.ownership_fault_count,
            ) {
                continue;
            }
            footprint.fins.push(fin_id);
            if let Ok(fin) = store.get(fin_id) {
                if let Some(pcurve) = fin.pcurve {
                    self.add_geometry_dependency(
                        store.geometry(),
                        GeometryRef::Curve2d(pcurve.curve()),
                        body,
                        footprint,
                    );
                }
                self.walk_edge(store, body, fin.edge, footprint);
            }
        }
    }

    fn walk_edge(
        &mut self,
        store: &Store,
        body: BodyId,
        edge_id: EdgeId,
        footprint: &mut BodyFootprint,
    ) {
        if !claim(
            &mut self.edges,
            edge_id,
            body,
            &mut self.ownership_fault_count,
        ) {
            return;
        }
        footprint.edges.push(edge_id);
        let Ok(edge) = store.get(edge_id) else {
            return;
        };
        if let Some(curve) = edge.curve {
            self.add_geometry_dependency(
                store.geometry(),
                GeometryRef::Curve(curve),
                body,
                footprint,
            );
        }
        for vertex_id in edge.vertices.into_iter().flatten() {
            self.walk_vertex(store, body, vertex_id, footprint);
        }
    }

    fn walk_vertex(
        &mut self,
        store: &Store,
        body: BodyId,
        vertex_id: VertexId,
        footprint: &mut BodyFootprint,
    ) {
        if !claim(
            &mut self.vertices,
            vertex_id,
            body,
            &mut self.ownership_fault_count,
        ) {
            return;
        }
        footprint.vertices.push(vertex_id);
        if let Ok(vertex) = store.get(vertex_id) {
            add_dependency(
                &mut self.points,
                &mut footprint.points,
                vertex.point,
                body,
                &self.body_ranks,
            );
        }
    }

    fn add_geometry_dependency(
        &mut self,
        graph: &GeometryGraph,
        root: GeometryRef,
        body: BodyId,
        footprint: &mut BodyFootprint,
    ) {
        let closure = match graph.dependency_closure(root) {
            Ok(closure) => closure,
            Err(_) => {
                // Retain the directly attached identity so invalid-model
                // indexing can still select the owning body, but make the
                // ownership audit fail rather than hiding graph corruption.
                self.ownership_fault_count += 1;
                vec![root]
            }
        };
        for geometry in closure {
            match geometry {
                GeometryRef::Curve(handle) => add_dependency(
                    &mut self.curves,
                    &mut footprint.curves,
                    handle,
                    body,
                    &self.body_ranks,
                ),
                GeometryRef::Surface(handle) => add_dependency(
                    &mut self.surfaces,
                    &mut footprint.surfaces,
                    handle,
                    body,
                    &self.body_ranks,
                ),
                GeometryRef::Curve2d(handle) => add_dependency(
                    &mut self.pcurves,
                    &mut footprint.pcurves,
                    handle,
                    body,
                    &self.body_ranks,
                ),
            }
        }
    }
}

fn claim<T>(
    claims: &mut HashMap<Handle<T>, BodyId>,
    handle: Handle<T>,
    body: BodyId,
    faults: &mut usize,
) -> bool {
    if let Some(owner) = claims.get(&handle) {
        if *owner != body {
            *faults += 1;
        }
        return false;
    }
    claims.insert(handle, body);
    true
}

fn add_dependency<T>(
    dependencies: &mut HashMap<Handle<T>, Vec<BodyId>>,
    footprint: &mut Vec<Handle<T>>,
    handle: Handle<T>,
    body: BodyId,
    body_ranks: &HashMap<BodyId, usize>,
) {
    let bodies = dependencies.entry(handle).or_default();
    if !bodies.contains(&body) {
        let rank = body_ranks.get(&body).copied().unwrap_or(usize::MAX);
        let position = bodies
            .iter()
            .position(|candidate| body_ranks.get(candidate).copied().unwrap_or(usize::MAX) > rank)
            .unwrap_or(bodies.len());
        bodies.insert(position, body);
    }
    if !footprint.contains(&handle) {
        footprint.push(handle);
    }
}

fn remove_owned<T>(owners: &mut HashMap<Handle<T>, BodyId>, handles: &[Handle<T>], body: BodyId) {
    for &handle in handles {
        if owners.get(&handle) == Some(&body) {
            owners.remove(&handle);
        }
    }
}

fn remove_dependencies<T>(
    dependencies: &mut HashMap<Handle<T>, Vec<BodyId>>,
    handles: &[Handle<T>],
    body: BodyId,
) {
    for &handle in handles {
        let remove_entry = if let Some(bodies) = dependencies.get_mut(&handle) {
            bodies.retain(|candidate| *candidate != body);
            bodies.is_empty()
        } else {
            false
        };
        if remove_entry {
            dependencies.remove(&handle);
        }
    }
}

fn collect_live_unowned<T: Entity>(
    store: &Store,
    owners: &HashMap<Handle<T>, BodyId>,
    handles: &[Handle<T>],
    out: &mut Vec<EntityRef>,
) {
    for &handle in handles {
        let entity = <T as crate::store::sealed::Storage>::entity_ref(handle);
        if store.contains(handle) && !owners.contains_key(&handle) && !out.contains(&entity) {
            out.push(entity);
        }
    }
}

fn unclaimed<T: Entity>(store: &Store, claims: &HashMap<Handle<T>, BodyId>) -> usize {
    store
        .iter::<T>()
        .filter(|(handle, _)| !claims.contains_key(handle))
        .count()
}

fn push_optional(bodies: &mut Vec<BodyId>, body: Option<BodyId>) {
    if let Some(body) = body {
        push_body(bodies, body);
    }
}

fn push_dependencies(bodies: &mut Vec<BodyId>, dependencies: Option<&Vec<BodyId>>) {
    if let Some(dependencies) = dependencies {
        for &body in dependencies {
            push_body(bodies, body);
        }
    }
}

fn push_body(bodies: &mut Vec<BodyId>, body: BodyId) {
    if !bodies.contains(&body) {
        bodies.push(body);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::make::{acorn, block};
    use crate::tolerance::EntityTolerance;
    use crate::transaction::MutationKind;
    use kgeom::frame::Frame;
    use kgeom::vec::Point3;

    #[test]
    fn ownership_and_shared_geometry_dependencies_select_affected_roots() {
        let mut store = Store::new();
        let first = block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
        let second = block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
        let first_face = store.faces_of_body(first).unwrap()[0];
        let second_face = store.faces_of_body(second).unwrap()[0];
        let shared_surface = store.get(first_face).unwrap().surface;
        let first_edge = store.edges_of_body(first).unwrap()[0];
        let edge = store.get(first_edge).unwrap();
        let curve = edge.curve.unwrap();
        let point = store.get(edge.vertices[0].unwrap()).unwrap().point;
        let pcurve = store.get(edge.fins[0]).unwrap().pcurve.unwrap().curve();

        let mut share = store.transaction().unwrap();
        share.assembly().get_mut(second_face).unwrap().surface = shared_surface;
        share.commit_checked_body(second).unwrap();

        let current = store.committed_index();
        assert_eq!(current.ownership_fault_count(), 0);
        assert_eq!(
            current.affected_bodies(
                current,
                &[Mutation {
                    entity: EntityRef::Face(second_face),
                    kind: MutationKind::Modified,
                }],
            ),
            vec![second]
        );
        assert_eq!(
            current.affected_bodies(
                current,
                &[Mutation {
                    entity: EntityRef::Surface(shared_surface),
                    kind: MutationKind::Modified,
                }],
            ),
            vec![first, second]
        );
        for entity in [
            EntityRef::Curve(curve),
            EntityRef::Point(point),
            EntityRef::Curve2d(pcurve),
        ] {
            assert_eq!(
                current.affected_bodies(
                    current,
                    &[Mutation {
                        entity,
                        kind: MutationKind::Modified,
                    }],
                ),
                vec![first]
            );
        }
    }

    #[test]
    fn multi_body_candidate_rebuilds_only_the_affected_footprint() {
        let mut store = Store::new();
        let bodies: Vec<_> = (0..64)
            .map(|index| acorn(&mut store, Point3::new(f64::from(index) * 0.01, 0.0, 0.0)).unwrap())
            .collect();
        let target = bodies[31];
        let vertex = store.vertices_of_body(target).unwrap()[0];
        let mut transaction = store.transaction().unwrap();
        transaction.assembly().get_mut(vertex).unwrap().tolerance =
            Some(EntityTolerance::operation(1.0e-8, "index-scope-test").unwrap());
        let pending = transaction.store().pending_transaction_mutations().unwrap();
        let (candidate, rebuilt_bodies) = StoreIndex::candidate_with_stats(
            transaction.store(),
            transaction.store().committed_index(),
            &pending,
        );
        assert_eq!(rebuilt_bodies, 1);
        assert_eq!(
            candidate.affected_bodies(transaction.store().committed_index(), &pending),
            vec![target]
        );
        candidate.debug_assert_full_rebuild_parity(transaction.store());
        transaction.commit_checked(&[]).unwrap();
    }
}
