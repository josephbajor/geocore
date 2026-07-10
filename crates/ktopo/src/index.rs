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
use std::collections::HashMap;

/// Snapshot of body ownership and shared geometry dependencies for one Store
/// state. Values preserve deterministic body slot order even though lookup is
/// hash-based.
#[derive(Debug, Clone, Default)]
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
    ownership_fault_count: usize,
}

impl StoreIndex {
    /// Build an index for the Store's current state and audit ownership
    /// closure. Stale geometry references are indexed so the affected body is
    /// still selected and the body checker can report the actual fault.
    pub(crate) fn build(store: &Store) -> Self {
        let mut index = Self::default();
        for (body_id, body) in store.iter::<Body>() {
            for &region_id in &body.regions {
                if !claim(
                    &mut index.regions,
                    region_id,
                    body_id,
                    &mut index.ownership_fault_count,
                ) {
                    continue;
                }
                let Ok(region) = store.get(region_id) else {
                    continue;
                };
                for &shell_id in &region.shells {
                    index.walk_shell(store, body_id, shell_id);
                }
            }
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

    fn walk_shell(&mut self, store: &Store, body: BodyId, shell_id: ShellId) {
        if !claim(
            &mut self.shells,
            shell_id,
            body,
            &mut self.ownership_fault_count,
        ) {
            return;
        }
        let Ok(shell) = store.get(shell_id) else {
            return;
        };
        for &face_id in &shell.faces {
            self.walk_face(store, body, face_id);
        }
        for &edge_id in &shell.edges {
            self.walk_edge(store, body, edge_id);
        }
        if let Some(vertex_id) = shell.vertex {
            self.walk_vertex(store, body, vertex_id);
        }
    }

    fn walk_face(&mut self, store: &Store, body: BodyId, face_id: FaceId) {
        if !claim(
            &mut self.faces,
            face_id,
            body,
            &mut self.ownership_fault_count,
        ) {
            return;
        }
        let Ok(face) = store.get(face_id) else {
            return;
        };
        add_dependency(&mut self.surfaces, face.surface, body);
        for &loop_id in &face.loops {
            self.walk_loop(store, body, loop_id);
        }
    }

    fn walk_loop(&mut self, store: &Store, body: BodyId, loop_id: LoopId) {
        if !claim(
            &mut self.loops,
            loop_id,
            body,
            &mut self.ownership_fault_count,
        ) {
            return;
        }
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
            if let Ok(fin) = store.get(fin_id) {
                if let Some(pcurve) = fin.pcurve {
                    add_dependency(&mut self.pcurves, pcurve.curve(), body);
                }
                self.walk_edge(store, body, fin.edge);
            }
        }
    }

    fn walk_edge(&mut self, store: &Store, body: BodyId, edge_id: EdgeId) {
        if !claim(
            &mut self.edges,
            edge_id,
            body,
            &mut self.ownership_fault_count,
        ) {
            return;
        }
        let Ok(edge) = store.get(edge_id) else {
            return;
        };
        if let Some(curve) = edge.curve {
            add_dependency(&mut self.curves, curve, body);
        }
        for vertex_id in edge.vertices.into_iter().flatten() {
            self.walk_vertex(store, body, vertex_id);
        }
    }

    fn walk_vertex(&mut self, store: &Store, body: BodyId, vertex_id: VertexId) {
        if !claim(
            &mut self.vertices,
            vertex_id,
            body,
            &mut self.ownership_fault_count,
        ) {
            return;
        }
        if let Ok(vertex) = store.get(vertex_id) {
            add_dependency(&mut self.points, vertex.point, body);
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
    handle: Handle<T>,
    body: BodyId,
) {
    let bodies = dependencies.entry(handle).or_default();
    if !bodies.contains(&body) {
        bodies.push(body);
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
    use crate::make::block;
    use crate::transaction::MutationKind;
    use kgeom::frame::Frame;

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
}
