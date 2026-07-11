//! Read-only geometry identity and stable metadata views.

use kgraph::{Curve2dDescriptor, CurveDescriptor, GeometryDependencies, SurfaceDescriptor};
use ktopo::store::Store;

use crate::{CurveId, GeometryClassKey, PcurveId, SurfaceId};

fn direct_dependency_count(node: &impl GeometryDependencies) -> usize {
    let mut count = 0;
    node.visit_dependencies(&mut |_| count += 1);
    count
}

/// Read-only metadata view of one authoritative 3D curve graph node.
pub struct CurveView<'part> {
    store: &'part Store,
    id: CurveId,
}

impl<'part> CurveView<'part> {
    pub(crate) fn new(store: &'part Store, id: CurveId) -> Self {
        Self { store, id }
    }

    fn node(&self) -> &CurveDescriptor {
        self.store
            .geometry()
            .curve(self.id.raw())
            .expect("an immutable validated curve view remains live")
    }

    /// Opaque curve identity.
    pub fn id(&self) -> CurveId {
        self.id.clone()
    }

    /// Stable F1 class identity.
    pub fn class_key(&self) -> GeometryClassKey {
        self.node().class_key()
    }

    /// Number of direct procedural dependencies.
    ///
    /// The accessor remains metadata-only and does not expose graph
    /// descriptors or dependency handles.
    pub fn direct_dependency_count(&self) -> usize {
        direct_dependency_count(self.node())
    }
}

/// Read-only metadata view of one authoritative surface graph node.
pub struct SurfaceView<'part> {
    store: &'part Store,
    id: SurfaceId,
}

impl<'part> SurfaceView<'part> {
    pub(crate) fn new(store: &'part Store, id: SurfaceId) -> Self {
        Self { store, id }
    }

    fn node(&self) -> &SurfaceDescriptor {
        self.store
            .geometry()
            .surface(self.id.raw())
            .expect("an immutable validated surface view remains live")
    }

    /// Opaque surface identity.
    pub fn id(&self) -> SurfaceId {
        self.id.clone()
    }

    /// Stable F1 class identity.
    pub fn class_key(&self) -> GeometryClassKey {
        self.node().class_key()
    }

    /// Number of direct procedural dependencies.
    pub fn direct_dependency_count(&self) -> usize {
        direct_dependency_count(self.node())
    }

    /// Basis surface for a constant-normal offset, or `None` for leaf
    /// surfaces.
    ///
    /// The returned ID resolves the existing graph node; no descriptor or
    /// facade mirror is allocated.
    pub fn offset_basis(&self) -> Option<SurfaceId> {
        self.node()
            .as_offset()
            .map(|offset| SurfaceId::new(self.id.part().clone(), offset.basis()))
    }

    /// Signed offset distance for a constant-normal offset surface.
    pub fn signed_offset_distance(&self) -> Option<f64> {
        self.node()
            .as_offset()
            .map(|offset| offset.signed_distance())
    }
}

/// Read-only metadata view of one authoritative parameter-space curve node.
pub struct PcurveView<'part> {
    store: &'part Store,
    id: PcurveId,
}

impl<'part> PcurveView<'part> {
    pub(crate) fn new(store: &'part Store, id: PcurveId) -> Self {
        Self { store, id }
    }

    fn node(&self) -> &Curve2dDescriptor {
        self.store
            .geometry()
            .curve2d(self.id.raw())
            .expect("an immutable validated pcurve view remains live")
    }

    /// Opaque pcurve identity.
    pub fn id(&self) -> PcurveId {
        self.id.clone()
    }

    /// Stable F1 class identity.
    pub fn class_key(&self) -> GeometryClassKey {
        self.node().class_key()
    }

    /// Number of direct procedural dependencies.
    ///
    /// The accessor does not expose graph descriptors or dependency handles.
    pub fn direct_dependency_count(&self) -> usize {
        direct_dependency_count(self.node())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyId, EntityKind, Error, Kernel, PartId, Session};
    use kgeom::curve::Line;
    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::surface::Plane;
    use kgeom::vec::{Point2, Point3, Vec2, Vec3};
    use kgraph::OffsetSurfaceDescriptor;
    use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};

    const OFFSET_DISTANCE: f64 = 0.25;

    fn add_offset_sheet(session: &mut Session, part: &PartId) -> BodyId {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let store = edit.store_mut_for_test();
        let world = Frame::world();
        let translated = Frame::new(
            world.origin() + Vec3::new(0.0, 0.0, OFFSET_DISTANCE),
            world.z(),
            world.x(),
        )
        .unwrap();
        let raw_body = ktopo::make::planar_sheet(
            store,
            &translated,
            &[
                Point2::new(-1.0, -1.0),
                Point2::new(1.0, -1.0),
                Point2::new(1.0, 1.0),
                Point2::new(-1.0, 1.0),
            ],
        )
        .unwrap();
        let raw_face = store.faces_of_body(raw_body).unwrap()[0];
        let old_surface = store.get(raw_face).unwrap().surface;

        let mut transaction = store.transaction().unwrap();
        {
            let mut assembly = transaction.assembly();
            let basis = assembly
                .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
                .unwrap();
            let offset = assembly
                .insert_surface(OffsetSurfaceDescriptor::new(basis, OFFSET_DISTANCE).into())
                .unwrap();
            assembly.get_mut(raw_face).unwrap().surface = offset;
            assembly.remove_surface(old_surface).unwrap();
        }
        transaction.commit_checked_body(raw_body).unwrap();
        BodyId::new(part.clone(), raw_body)
    }

    #[test]
    fn topology_attachments_share_facade_geometry_identity_and_iteration_is_stable() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body_id = add_offset_sheet(&mut session, &part_id);
        let part = session.part(part_id).unwrap();

        let curves = part.curves().collect::<Vec<_>>();
        let surfaces = part.surfaces().collect::<Vec<_>>();
        let pcurves = part.pcurves().collect::<Vec<_>>();
        assert_eq!(part.curves().len(), curves.len());
        assert_eq!(part.surfaces().len(), surfaces.len());
        assert_eq!(part.pcurves().len(), pcurves.len());
        assert_eq!(part.curves().collect::<Vec<_>>(), curves);
        assert_eq!(part.surfaces().collect::<Vec<_>>(), surfaces);
        assert_eq!(part.pcurves().collect::<Vec<_>>(), pcurves);

        let faces = part
            .body(body_id.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>();
        assert_eq!(faces.len(), 1);
        let offset_id = part.face(faces[0].clone()).unwrap().surface();
        assert!(surfaces.contains(&offset_id));
        let offset = part.surface(offset_id.clone()).unwrap();
        assert_eq!(offset.id(), offset_id);
        assert_eq!(offset.class_key().as_str(), "kernel.surface.offset.v1");
        assert_eq!(offset.direct_dependency_count(), 1);
        assert_eq!(offset.signed_offset_distance(), Some(OFFSET_DISTANCE));

        let basis_id = offset.offset_basis().unwrap();
        assert!(surfaces.contains(&basis_id));
        let basis = part.surface(basis_id.clone()).unwrap();
        assert_eq!(basis.id(), basis_id);
        assert_eq!(basis.class_key().as_str(), "kernel.surface.plane.v1");
        assert_eq!(basis.direct_dependency_count(), 0);
        assert_eq!(basis.offset_basis(), None);

        for edge_id in part.body(body_id).unwrap().edges().unwrap() {
            let curve_id = part.edge(edge_id).unwrap().curve().unwrap();
            assert!(curves.contains(&curve_id));
            let curve = part.curve(curve_id.clone()).unwrap();
            assert_eq!(curve.id(), curve_id);
            assert_eq!(curve.class_key().as_str(), "kernel.curve.line.v1");
            assert_eq!(curve.direct_dependency_count(), 0);
        }
        for face_id in faces {
            for loop_id in part.face(face_id).unwrap().loops() {
                for fin_id in part.loop_(loop_id).unwrap().fins() {
                    let pcurve_id = part.fin(fin_id).unwrap().pcurve().unwrap();
                    assert!(pcurves.contains(&pcurve_id));
                    let pcurve = part.pcurve(pcurve_id.clone()).unwrap();
                    assert_eq!(pcurve.id(), pcurve_id);
                    assert_eq!(pcurve.class_key().as_str(), "kernel.curve2d.line.v1");
                    assert_eq!(pcurve.direct_dependency_count(), 0);
                }
            }
        }
    }

    #[test]
    fn wrong_part_is_rejected_before_an_equal_graph_handle_can_resolve() {
        let mut session = Kernel::new().create_session();
        let first = session.create_part();
        let second = session.create_part();
        let first_surface = {
            let mut edit = session.edit_part(first.clone()).unwrap();
            let raw = edit
                .store_mut_for_test()
                .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
                .unwrap();
            SurfaceId::new(first.clone(), raw)
        };
        let second_surface = {
            let mut edit = session.edit_part(second.clone()).unwrap();
            let raw = edit
                .store_mut_for_test()
                .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
                .unwrap();
            SurfaceId::new(second.clone(), raw)
        };
        assert_eq!(first_surface.raw(), second_surface.raw());

        assert!(matches!(
            session.part(second.clone()).unwrap().surface(first_surface),
            Err(Error::WrongPart { expected, actual })
                if expected == second && actual == first
        ));
    }

    #[test]
    fn removed_graph_identities_are_reported_as_stale_by_kind() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (curve_id, surface_id, pcurve_id) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let store = edit.store_mut_for_test();
            let curve = store
                .insert_curve(CurveGeom::Line(
                    Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
                ))
                .unwrap();
            let surface = store
                .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
                .unwrap();
            let pcurve = store
                .insert_pcurve(Curve2dGeom::Line(
                    Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
                ))
                .unwrap();
            let ids = (
                CurveId::new(part_id.clone(), curve),
                SurfaceId::new(part_id.clone(), surface),
                PcurveId::new(part_id.clone(), pcurve),
            );
            let mut transaction = store.transaction().unwrap();
            {
                let mut assembly = transaction.assembly();
                assembly.remove_curve(curve).unwrap();
                assembly.remove_surface(surface).unwrap();
                assembly.remove_pcurve(pcurve).unwrap();
            }
            transaction.commit_checked(&[]).unwrap();
            ids
        };

        let part = session.part(part_id).unwrap();
        assert!(matches!(
            part.curve(curve_id),
            Err(Error::StaleEntity {
                kind: EntityKind::Curve
            })
        ));
        assert!(matches!(
            part.surface(surface_id),
            Err(Error::StaleEntity {
                kind: EntityKind::Surface
            })
        ));
        assert!(matches!(
            part.pcurve(pcurve_id),
            Err(Error::StaleEntity {
                kind: EntityKind::Pcurve
            })
        ));
    }
}
