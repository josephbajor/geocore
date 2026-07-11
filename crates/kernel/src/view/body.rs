//! Body, region, and shell ownership views.

use ktopo::entity::{Body as RawBody, Region as RawRegion, Shell as RawShell};
use ktopo::store::Store;

use crate::error::{Error, Result};
use crate::{
    BodyId, BodyKind, EdgeId, EdgeIds, FaceId, FaceIds, RegionId, RegionIds, RegionKind, ShellId,
    ShellIds, VertexId, VertexIds,
};

/// Read-only body view.
pub struct BodyView<'part> {
    store: &'part Store,
    id: BodyId,
}

impl<'part> BodyView<'part> {
    pub(crate) fn new(store: &'part Store, id: BodyId) -> Self {
        Self { store, id }
    }

    fn entity(&self) -> &RawBody {
        self.store
            .get(self.id.raw())
            .expect("an immutable validated body view remains live")
    }

    /// Body identity.
    pub fn id(&self) -> BodyId {
        self.id.clone()
    }

    /// Point-set kind.
    pub fn kind(&self) -> BodyKind {
        self.entity().kind
    }

    /// Regions in stored ownership order.
    pub fn regions(&self) -> RegionIds<'_> {
        let regions = &self.entity().regions;
        RegionIds::new(
            regions
                .iter()
                .map(|&raw| RegionId::new(self.id.part().clone(), raw)),
            regions.len(),
        )
    }

    /// Faces in region → shell → face stored order.
    pub fn faces(&self) -> Result<FaceIds<'_>> {
        let body = self.entity();
        let mut count = 0;
        for &region in &body.regions {
            let region = self
                .store
                .get(region)
                .map_err(|source| Error::InconsistentTopology { source })?;
            for &shell in &region.shells {
                let shell = self
                    .store
                    .get(shell)
                    .map_err(|source| Error::InconsistentTopology { source })?;
                count += shell.faces.len();
            }
        }

        let store = self.store;
        let part = self.id.part().clone();
        let faces = body.regions.iter().flat_map(move |region| {
            store
                .get(*region)
                .expect("validated body region remains live during immutable traversal")
                .shells
                .iter()
                .flat_map(move |shell| {
                    store
                        .get(*shell)
                        .expect("validated region shell remains live during immutable traversal")
                        .faces
                        .iter()
                        .copied()
                })
        });
        Ok(FaceIds::new(
            faces.map(move |raw| FaceId::new(part.clone(), raw)),
            count,
        ))
    }

    /// Edges in deterministic deduplicated first-traversal order.
    pub fn edges(&self) -> Result<EdgeIds<'_>> {
        let values = self
            .store
            .edges_of_body(self.id.raw())
            .map_err(|source| Error::InconsistentTopology { source })?;
        let count = values.len();
        let part = self.id.part().clone();
        Ok(EdgeIds::new(
            values
                .into_iter()
                .map(move |raw| EdgeId::new(part.clone(), raw)),
            count,
        ))
    }

    /// Vertices in body-edge order, deduplicated by first traversal.
    pub fn vertices(&self) -> Result<VertexIds<'_>> {
        let values = self
            .store
            .vertices_of_body(self.id.raw())
            .map_err(|source| Error::InconsistentTopology { source })?;
        let count = values.len();
        let part = self.id.part().clone();
        Ok(VertexIds::new(
            values
                .into_iter()
                .map(move |raw| VertexId::new(part.clone(), raw)),
            count,
        ))
    }
}

/// Read-only region view.
pub struct RegionView<'part> {
    store: &'part Store,
    id: RegionId,
}

impl<'part> RegionView<'part> {
    pub(crate) fn new(store: &'part Store, id: RegionId) -> Self {
        Self { store, id }
    }

    fn entity(&self) -> &RawRegion {
        self.store
            .get(self.id.raw())
            .expect("an immutable validated region view remains live")
    }

    /// Region identity.
    pub fn id(&self) -> RegionId {
        self.id.clone()
    }

    /// Owning body.
    pub fn body(&self) -> BodyId {
        BodyId::new(self.id.part().clone(), self.entity().body)
    }

    /// Whether this region contains material.
    pub fn kind(&self) -> RegionKind {
        self.entity().kind
    }

    /// Shells in stored ownership order.
    pub fn shells(&self) -> ShellIds<'_> {
        let shells = &self.entity().shells;
        ShellIds::new(
            shells
                .iter()
                .map(|&raw| ShellId::new(self.id.part().clone(), raw)),
            shells.len(),
        )
    }
}

/// Read-only shell view.
pub struct ShellView<'part> {
    store: &'part Store,
    id: ShellId,
}

impl<'part> ShellView<'part> {
    pub(crate) fn new(store: &'part Store, id: ShellId) -> Self {
        Self { store, id }
    }

    fn entity(&self) -> &RawShell {
        self.store
            .get(self.id.raw())
            .expect("an immutable validated shell view remains live")
    }

    /// Shell identity.
    pub fn id(&self) -> ShellId {
        self.id.clone()
    }

    /// Owning region.
    pub fn region(&self) -> RegionId {
        RegionId::new(self.id.part().clone(), self.entity().region)
    }

    /// Faces in stored ownership order.
    pub fn faces(&self) -> FaceIds<'_> {
        let faces = &self.entity().faces;
        FaceIds::new(
            faces
                .iter()
                .map(|&raw| FaceId::new(self.id.part().clone(), raw)),
            faces.len(),
        )
    }

    /// Wireframe edges in stored ownership order.
    pub fn edges(&self) -> EdgeIds<'_> {
        let edges = &self.entity().edges;
        EdgeIds::new(
            edges
                .iter()
                .map(|&raw| EdgeId::new(self.id.part().clone(), raw)),
            edges.len(),
        )
    }

    /// Acorn vertex, when present.
    pub fn vertex(&self) -> Option<VertexId> {
        self.entity()
            .vertex
            .map(|raw| VertexId::new(self.id.part().clone(), raw))
    }
}
