//! Kernel, session, and independent-part lifecycle.

use std::sync::Arc;

use kcore::arena::Arena;
use kcore::operation::SessionPolicy;
use ktopo::store::Store;

use crate::error::{Error, Result};
use crate::id::SessionIdentity;
use crate::{PartId, PartIds};

/// Cheap configuration root used to create independent sessions.
#[derive(Debug, Clone)]
pub struct Kernel {
    default_policy: Arc<SessionPolicy>,
}

impl Kernel {
    /// Construct a kernel using the validated production-v1 session policy.
    pub fn new() -> Self {
        Self::default()
    }

    /// Construct a kernel using an already validated immutable policy.
    pub fn with_default_policy(policy: SessionPolicy) -> Self {
        Self {
            default_policy: Arc::new(policy),
        }
    }

    /// Create an empty, independently owned session.
    pub fn create_session(&self) -> Session {
        Session {
            policy: Arc::clone(&self.default_policy),
            identity: SessionIdentity::new(),
            parts: Arena::new(),
        }
    }
}

impl Default for Kernel {
    fn default() -> Self {
        Self {
            default_policy: Arc::new(SessionPolicy::v1()),
        }
    }
}

/// Non-cloneable owner of one policy and zero or more independent parts.
pub struct Session {
    policy: Arc<SessionPolicy>,
    identity: SessionIdentity,
    parts: Arena<PartState>,
}

impl Session {
    /// Immutable policy shared by every operation in this session.
    pub fn policy(&self) -> &SessionPolicy {
        &self.policy
    }

    /// Create an empty independent part.
    pub fn create_part(&mut self) -> PartId {
        let handle = self.parts.insert(PartState::default());
        PartId::new(self.identity.clone(), handle)
    }

    /// Remove a part and invalidate its ID.
    pub fn remove_part(&mut self, id: PartId) -> Result<()> {
        self.validate_part_id(&id)?;
        self.parts
            .remove(id.handle())
            .map(|_| ())
            .ok_or(Error::UnknownPart)
    }

    /// Enumerate live parts in deterministic arena-slot order.
    pub fn parts(&self) -> PartIds<'_> {
        PartIds::new(
            self.parts
                .iter()
                .map(|(handle, _)| PartId::new(self.identity.clone(), handle)),
            self.parts.len(),
        )
    }

    /// Borrow one part for immutable inspection.
    pub fn part(&self, id: PartId) -> Result<Part<'_>> {
        self.validate_part_id(&id)?;
        let state = self.parts.get(id.handle()).ok_or(Error::UnknownPart)?;
        Ok(Part {
            policy: &self.policy,
            id,
            state,
        })
    }

    /// Borrow one part as the exclusive capability for future mutations.
    pub fn edit_part(&mut self, id: PartId) -> Result<PartEdit<'_>> {
        self.validate_part_id(&id)?;
        let state = self.parts.get_mut(id.handle()).ok_or(Error::UnknownPart)?;
        Ok(PartEdit {
            policy: &self.policy,
            id,
            state,
        })
    }

    fn validate_part_id(&self, id: &PartId) -> Result<()> {
        if !id.belongs_to(&self.identity) {
            return Err(Error::UnknownPart);
        }
        Ok(())
    }
}

#[derive(Clone, Default)]
pub(crate) struct PartState {
    pub(crate) store: Store,
}

/// Immutable borrowed capability for one part.
pub struct Part<'session> {
    pub(crate) policy: &'session SessionPolicy,
    pub(crate) id: PartId,
    pub(crate) state: &'session PartState,
}

impl Part<'_> {
    /// Opaque identity of this part.
    pub fn id(&self) -> PartId {
        self.id.clone()
    }

    /// Session policy inherited by future contextual operations.
    pub fn policy(&self) -> &SessionPolicy {
        self.policy
    }
}

/// Exclusive borrowed capability for one part.
///
/// K1 exposes reads through [`PartEdit::as_part`]. Semantic operations and
/// edit transactions are added in later façade stages without exposing Store.
pub struct PartEdit<'session> {
    pub(crate) policy: &'session SessionPolicy,
    pub(crate) id: PartId,
    pub(crate) state: &'session mut PartState,
}

impl PartEdit<'_> {
    /// Opaque identity of this part.
    pub fn id(&self) -> PartId {
        self.id.clone()
    }

    /// Session policy inherited by future contextual operations.
    pub fn policy(&self) -> &SessionPolicy {
        self.policy
    }

    /// Temporarily inspect this exclusively borrowed part.
    pub fn as_part(&self) -> Part<'_> {
        Part {
            policy: self.policy,
            id: self.id.clone(),
            state: &*self.state,
        }
    }

    #[cfg(test)]
    pub(crate) fn store_mut_for_test(&mut self) -> &mut Store {
        &mut self.state.store
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BodyId, EdgeId, EntityKind, Error, FaceId, FinId, LoopId, RegionId, ShellId, VertexId,
    };
    use kgeom::frame::Frame;
    use kgeom::surface::Plane;
    use kgeom::vec::Point3;
    use ktopo::entity::{
        Body as RawBody, Edge as RawEdge, Face as RawFace, Fin as RawFin, Loop as RawLoop,
        Region as RawRegion, Sense, Shell as RawShell, Vertex as RawVertex,
    };
    use ktopo::geom::SurfaceGeom;

    fn add_block(session: &mut Session, part: &PartId, offset: f64) -> BodyId {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let raw = ktopo::make::block(
            edit.store_mut_for_test(),
            &Frame::from_z(Point3::new(offset, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0)).unwrap(),
            [1.0, 2.0, 3.0],
        )
        .unwrap();
        BodyId::new(part.clone(), raw)
    }

    #[test]
    fn views_cover_the_hierarchy_and_preserve_documented_orders() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body_id = add_block(&mut session, &part_id, 0.0);
        let second_body_id = add_block(&mut session, &part_id, 5.0);

        let (
            expected_bodies,
            expected_regions,
            expected_shells,
            expected_faces,
            expected_loops,
            expected_fins,
            expected_edges,
            expected_vertices,
            expected_body_regions,
            expected_body_faces,
            expected_body_edges,
            expected_body_vertices,
        ) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let store = edit.store_mut_for_test();
            let ids = |raw| BodyId::new(part_id.clone(), raw);
            let expected_bodies = store
                .iter::<RawBody>()
                .map(|(raw, _)| ids(raw))
                .collect::<Vec<_>>();
            let expected_regions = store
                .iter::<RawRegion>()
                .map(|(raw, _)| RegionId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            let expected_shells = store
                .iter::<RawShell>()
                .map(|(raw, _)| ShellId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            let expected_faces = store
                .iter::<RawFace>()
                .map(|(raw, _)| FaceId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            let expected_loops = store
                .iter::<RawLoop>()
                .map(|(raw, _)| LoopId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            let expected_fins = store
                .iter::<RawFin>()
                .map(|(raw, _)| FinId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            let expected_edges = store
                .iter::<RawEdge>()
                .map(|(raw, _)| EdgeId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            let expected_vertices = store
                .iter::<RawVertex>()
                .map(|(raw, _)| VertexId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            let expected_body_regions = store
                .get(body_id.raw())
                .unwrap()
                .regions
                .iter()
                .map(|&raw| RegionId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            let expected_body_faces = store
                .faces_of_body(body_id.raw())
                .unwrap()
                .into_iter()
                .map(|raw| FaceId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            let expected_body_edges = store
                .edges_of_body(body_id.raw())
                .unwrap()
                .into_iter()
                .map(|raw| EdgeId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            let expected_body_vertices = store
                .vertices_of_body(body_id.raw())
                .unwrap()
                .into_iter()
                .map(|raw| VertexId::new(part_id.clone(), raw))
                .collect::<Vec<_>>();
            (
                expected_bodies,
                expected_regions,
                expected_shells,
                expected_faces,
                expected_loops,
                expected_fins,
                expected_edges,
                expected_vertices,
                expected_body_regions,
                expected_body_faces,
                expected_body_edges,
                expected_body_vertices,
            )
        };
        let part = session.part(part_id).unwrap();

        assert_eq!(expected_bodies, vec![body_id.clone(), second_body_id]);
        assert_eq!(part.bodies().collect::<Vec<_>>(), expected_bodies);
        assert_eq!(part.regions().collect::<Vec<_>>(), expected_regions);
        assert_eq!(part.shells().collect::<Vec<_>>(), expected_shells);
        assert_eq!(part.faces().collect::<Vec<_>>(), expected_faces);
        assert_eq!(part.loops().collect::<Vec<_>>(), expected_loops);
        assert_eq!(part.fins().collect::<Vec<_>>(), expected_fins);
        assert_eq!(part.edges().collect::<Vec<_>>(), expected_edges);
        assert_eq!(part.vertices().collect::<Vec<_>>(), expected_vertices);
        let body = part.body(body_id.clone()).unwrap();
        assert_eq!(body.id(), body_id);
        assert_eq!(format!("{:?}", body.id()), "BodyId(<opaque>)");

        let regions = body.regions().collect::<Vec<_>>();
        let faces = body.faces().unwrap().collect::<Vec<_>>();
        let edges = body.edges().unwrap().collect::<Vec<_>>();
        let vertices = body.vertices().unwrap().collect::<Vec<_>>();
        assert_eq!(regions, expected_body_regions);
        assert_eq!(faces, expected_body_faces);
        assert_eq!(edges, expected_body_edges);
        assert_eq!(vertices, expected_body_vertices);

        let mut ownership_faces = Vec::new();
        let mut first_traversal_edges = Vec::new();
        for region_id in &regions {
            let region = part.region(region_id.clone()).unwrap();
            assert_eq!(region.body(), body_id);
            for shell_id in region.shells() {
                let shell = part.shell(shell_id).unwrap();
                assert_eq!(shell.region(), region.id());
                for face_id in shell.faces() {
                    ownership_faces.push(face_id.clone());
                    let face = part.face(face_id).unwrap();
                    assert_eq!(face.shell(), shell.id());
                    for loop_id in face.loops() {
                        let loop_view = part.loop_(loop_id).unwrap();
                        assert_eq!(loop_view.face(), face.id());
                        for fin_id in loop_view.fins() {
                            let fin = part.fin(fin_id).unwrap();
                            assert_eq!(fin.loop_(), loop_view.id());
                            let edge_id = fin.edge();
                            if !first_traversal_edges.contains(&edge_id) {
                                first_traversal_edges.push(edge_id.clone());
                            }
                            let edge = part.edge(edge_id).unwrap();
                            assert!(edge.fins().any(|candidate| candidate == fin.id()));
                            let [start, end] = edge.vertices();
                            let expected = if fin.sense().is_forward() {
                                [start, end]
                            } else {
                                [end, start]
                            };
                            assert_eq!(fin.tail().unwrap(), expected[0]);
                            assert_eq!(fin.head().unwrap(), expected[1]);
                        }
                    }
                }
                for edge_id in shell.edges() {
                    if !first_traversal_edges.contains(&edge_id) {
                        first_traversal_edges.push(edge_id);
                    }
                }
                if let Some(vertex_id) = shell.vertex() {
                    assert!(vertices.contains(&vertex_id));
                }
            }
        }
        assert_eq!(faces, ownership_faces);
        assert_eq!(edges, first_traversal_edges);

        for vertex_id in vertices {
            let position = part.vertex(vertex_id).unwrap().position().unwrap();
            assert!(position.to_array().into_iter().all(f64::is_finite));
        }
        assert!(part.regions().len() > regions.len());
        assert!(part.faces().len() > faces.len());
        assert!(part.edges().len() > edges.len());
    }

    #[test]
    fn wrong_part_is_rejected_before_equal_raw_handles_can_resolve() {
        let mut session = Kernel::new().create_session();
        let first = session.create_part();
        let second = session.create_part();
        let first_body = add_block(&mut session, &first, 0.0);
        let second_body = add_block(&mut session, &second, 5.0);
        assert_eq!(first_body.raw(), second_body.raw());

        let second_view = session.part(second.clone()).unwrap();
        assert!(matches!(
            second_view.body(first_body),
            Err(Error::WrongPart { expected, actual })
                if expected == second && actual == first
        ));
    }

    #[test]
    fn removed_lower_identity_is_reported_as_stale() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let stale = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let store = edit.store_mut_for_test();
            let surface = store
                .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
                .unwrap();
            let point = store.insert_point(Point3::new(0.0, 0.0, 0.0)).unwrap();
            let mut transaction = store.transaction().unwrap();
            let made = transaction
                .make_minimal_body(surface, Sense::Forward, point)
                .unwrap();
            let facade = BodyId::new(part_id.clone(), made.body);
            transaction.kill_minimal_body(made.body).unwrap();
            transaction.commit_checked(&[]).unwrap();
            facade
        };

        assert!(matches!(
            session.part(part_id).unwrap().body(stale),
            Err(Error::StaleEntity {
                kind: EntityKind::Body
            })
        ));
    }
}
