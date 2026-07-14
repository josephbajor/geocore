//! Facade-safe, operation-accounted whole-body tessellation.

use core::ops::Range;

use kcore::operation::OperationScope;

use crate::error::{Error, Result};
use crate::operation::{OperationOutcome, OperationSettings};
use crate::session::Part;
use crate::{BodyId, EdgeId, FaceId, PartId, Point3, TessOptions};

/// Typed request for one conforming whole-body tessellation.
#[derive(Debug, Clone, PartialEq)]
pub struct TessellateBodyRequest {
    body: BodyId,
    options: TessOptions,
    settings: OperationSettings,
}

impl TessellateBodyRequest {
    /// Construct a request with explicit output-quality controls.
    pub fn new(body: BodyId, options: TessOptions) -> Self {
        Self {
            body,
            options,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Body being tessellated.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Requested output-quality controls.
    pub const fn options(&self) -> TessOptions {
        self.options
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Triangle interval contributed by one opaque facade face.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FaceTriangleRange {
    face: FaceId,
    range: Range<usize>,
}

impl FaceTriangleRange {
    /// Face that contributed this interval.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// Half-open interval into [`BodyMesh::triangles`].
    pub fn range(&self) -> Range<usize> {
        self.range.clone()
    }
}

/// Shared mesh polyline belonging to one opaque topological edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EdgePolyline {
    edge: EdgeId,
    vertex_indices: Vec<u32>,
}

impl EdgePolyline {
    /// Edge represented by this shared polyline.
    pub fn edge(&self) -> EdgeId {
        self.edge.clone()
    }

    /// Mesh-vertex indices in deterministic edge order.
    pub fn vertex_indices(&self) -> &[u32] {
        &self.vertex_indices
    }
}

/// Conforming whole-body mesh using only facade-safe identities and values.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyMesh {
    body: BodyId,
    positions: Vec<Point3>,
    triangles: Vec<[u32; 3]>,
    face_triangle_ranges: Vec<FaceTriangleRange>,
    edge_polylines: Vec<EdgePolyline>,
}

impl BodyMesh {
    /// Exact body identity used to produce this mesh.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Vertex positions in model space.
    pub fn positions(&self) -> &[Point3] {
        &self.positions
    }

    /// Triangle vertex indices oriented by face sense: outward for solids and
    /// consistently across each sheet.
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.triangles
    }

    /// Per-face triangle intervals in deterministic body-face order.
    pub fn face_triangle_ranges(&self) -> &[FaceTriangleRange] {
        &self.face_triangle_ranges
    }

    /// Shared edge polylines in deterministic body-edge order.
    pub fn edge_polylines(&self) -> &[EdgePolyline] {
        &self.edge_polylines
    }

    /// Serialize positions and triangles as Wavefront OBJ.
    pub fn to_obj(&self) -> String {
        let mut output = String::new();
        for point in &self.positions {
            output.push_str(&format!("v {:?} {:?} {:?}\n", point.x, point.y, point.z));
        }
        for triangle in &self.triangles {
            output.push_str(&format!(
                "f {} {} {}\n",
                triangle[0] + 1,
                triangle[1] + 1,
                triangle[2] + 1
            ));
        }
        output
    }
}

impl Part<'_> {
    /// Tessellate one body through a facade-owned operation scope.
    ///
    /// Wrong-part and stale identities, invalid settings, and incomplete
    /// body-tessellation budget profiles are rejected before the scope starts.
    /// Once started, graph queries, projection fallbacks, refinement, retained
    /// items, the result, and any classified failure share the returned report.
    pub fn tessellate_body(
        &self,
        request: TessellateBodyRequest,
    ) -> Result<OperationOutcome<BodyMesh>> {
        let TessellateBodyRequest {
            body,
            options,
            settings,
        } = request;
        self.body(body.clone())?;
        let context =
            settings.context(self.policy)?.with_family_budget_defaults(
                ktopo::btess::BodyTessellationBudgetProfile::v1_defaults(),
            );
        let effective = context.effective_budget();
        for required in ktopo::btess::BodyTessellationBudgetProfile::v1_defaults().limits() {
            effective.require_limit(required.stage, required.resource, required.mode)?;
        }

        let mut scope = OperationScope::new(&context);
        let lower = ktopo::btess::tessellate_body_in_scope(
            &self.state.store,
            body.raw(),
            &options,
            &mut scope,
        );
        let result = lower
            .map(|mesh| adapt_body_mesh(&self.id, body, mesh))
            .map_err(Error::from_tessellation);
        Ok(scope.finish_typed(result))
    }
}

fn adapt_body_mesh(part: &PartId, body: BodyId, mesh: ktopo::btess::BodyMesh) -> BodyMesh {
    BodyMesh {
        body,
        positions: mesh.positions,
        triangles: mesh.triangles,
        face_triangle_ranges: mesh
            .face_ranges
            .into_iter()
            .map(|(face, range)| FaceTriangleRange {
                face: FaceId::new(part.clone(), face),
                range,
            })
            .collect(),
        edge_polylines: mesh
            .edge_polylines
            .into_iter()
            .map(|(edge, vertex_indices)| EdgePolyline {
                edge: EdgeId::new(part.clone(), edge),
                vertex_indices,
            })
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use std::error::Error as _;

    use kcore::error::ErrorClass;
    use kcore::operation::{AccountingMode, LimitSpec, OperationContext, ResourceKind};
    use kgeom::surface::Plane;
    use ktopo::entity::Sense;
    use ktopo::geom::SurfaceGeom;
    use ktopo::store::Store;

    use super::*;
    use crate::{
        BlockRequest, BodyTessellationBudgetProfile, BodyTessellationError, BudgetPlan, Frame,
        Kernel, KernelError, Tolerances,
    };

    fn options() -> TessOptions {
        TessOptions {
            chord_tol: 0.25,
            max_edge_len: Some(0.75),
        }
    }

    #[test]
    fn bounded_facade_mesh_matches_direct_contextual_bits_identity_and_report() {
        let policy = crate::SessionPolicy::v1();
        let mut direct_store = Store::new();
        let direct_body =
            ktopo::make::block(&mut direct_store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        let bounded = BodyTessellationBudgetProfile::bounded_v1();
        let direct_context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_budget_overrides(bounded.clone());
        let direct = ktopo::btess::tessellate_body_with_context(
            &direct_store,
            direct_body,
            &options(),
            &direct_context,
        )
        .unwrap();

        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let part = session.part(part_id.clone()).unwrap();
        let facade = part
            .tessellate_body(
                TessellateBodyRequest::new(body.clone(), options())
                    .with_settings(OperationSettings::new().with_budget_overrides(bounded.clone())),
            )
            .unwrap();

        let (direct_result, direct_report) = direct.into_parts();
        let direct_mesh = direct_result.unwrap();
        let expected = adapt_body_mesh(&part_id, body.clone(), direct_mesh.clone());
        assert_eq!(facade.result(), Ok(&expected));
        assert_eq!(facade.report(), &direct_report);
        let mesh = facade.result().unwrap();
        assert_eq!(mesh.body(), body);
        assert_eq!(mesh.positions(), direct_mesh.positions);
        assert_eq!(mesh.triangles(), direct_mesh.triangles);
        assert_eq!(mesh.face_triangle_ranges().len(), 6);
        assert_eq!(mesh.edge_polylines().len(), 12);
        assert!(
            mesh.face_triangle_ranges()
                .iter()
                .all(|range| part.face(range.face()).is_ok())
        );
        assert!(
            mesh.edge_polylines().iter().all(|line| {
                part.edge(line.edge()).is_ok() && !line.vertex_indices().is_empty()
            })
        );
        assert_eq!(mesh.to_obj(), direct_mesh.to_obj());

        let repeated = part
            .tessellate_body(
                TessellateBodyRequest::new(body, options())
                    .with_settings(OperationSettings::new().with_budget_overrides(bounded)),
            )
            .unwrap();
        assert_eq!(repeated, facade);
    }

    #[test]
    fn structural_item_boundary_preserves_report_and_classified_source() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();

        let run = |allowed| {
            let overrides = BodyTessellationBudgetProfile::bounded_v1().overlaid(
                &BudgetPlan::new([LimitSpec::new(
                    ktopo::btess::BODY_TESSELLATION_STRUCTURAL_ITEMS,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
            session
                .part(part_id.clone())
                .unwrap()
                .tessellate_body(
                    TessellateBodyRequest::new(body.clone(), options())
                        .with_settings(OperationSettings::new().with_budget_overrides(overrides)),
                )
                .unwrap()
        };
        assert!(run(84).result().is_ok());
        let limited = run(83);
        let source = match limited.result().unwrap_err() {
            KernelError::BodyTessellation { source } => source,
            other => panic!("unexpected facade error: {other:?}"),
        };
        let snapshot = source.limit().unwrap();
        assert_eq!(source.class(), ErrorClass::ResourceLimit);
        assert_eq!(source.code(), kcore::error::code::RESOURCE_LIMIT);
        assert_eq!(source.capability(), None);
        assert_eq!(
            snapshot.stage,
            ktopo::btess::BODY_TESSELLATION_STRUCTURAL_ITEMS
        );
        assert_eq!(snapshot.resource, ResourceKind::Items);
        assert_eq!(snapshot.allowed, 83);
        assert_eq!(limited.report().limit_events(), &[snapshot]);

        let typed = BodyTessellationError::new(ktopo::btess::TessellationError::Kernel(
            kcore::error::Error::ResourceLimit { snapshot },
        ));
        assert!(typed.source().is_some());
        assert!(typed.source().unwrap().source().is_some());
    }

    #[test]
    fn body_identity_precedes_options_and_invalid_options_remain_in_outcome() {
        let mut session = Kernel::new().create_session();
        let first = session.create_part();
        let second = session.create_part();
        let body = session
            .edit_part(first.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let invalid = TessOptions {
            chord_tol: 0.0,
            max_edge_len: None,
        };
        assert!(matches!(
            session
                .part(second)
                .unwrap()
                .tessellate_body(TessellateBodyRequest::new(body.clone(), invalid)),
            Err(KernelError::WrongPart { .. })
        ));

        let stale = {
            let mut edit = session.edit_part(first.clone()).unwrap();
            let store = edit.store_mut_for_test();
            let surface = store
                .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
                .unwrap();
            let point = store.insert_point(Point3::new(0.0, 0.0, 0.0)).unwrap();
            let mut transaction = store.transaction().unwrap();
            let made = transaction
                .make_minimal_body(surface, Sense::Forward, point)
                .unwrap();
            let stale = BodyId::new(first.clone(), made.body);
            transaction.kill_minimal_body(made.body).unwrap();
            transaction.commit_checked(&[]).unwrap();
            stale
        };
        assert!(matches!(
            session
                .part(first.clone())
                .unwrap()
                .tessellate_body(TessellateBodyRequest::new(stale, invalid)),
            Err(KernelError::StaleEntity {
                kind: crate::EntityKind::Body
            })
        ));

        let outcome = session
            .part(first)
            .unwrap()
            .tessellate_body(TessellateBodyRequest::new(body, invalid))
            .unwrap();
        let error = outcome.result().unwrap_err();
        assert_eq!(error.class(), ErrorClass::InvalidInput);
        assert_eq!(error.code(), kcore::error::code::INVALID_TOLERANCE);
        assert!(matches!(error, KernelError::BodyTessellation { .. }));
    }
}
