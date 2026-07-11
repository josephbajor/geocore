//! Part-level deterministic enumeration and typed ID resolution.

use ktopo::entity::{
    Body as RawBody, Edge as RawEdge, Face as RawFace, Fin as RawFin, Loop as RawLoop,
    Region as RawRegion, Shell as RawShell, Vertex as RawVertex,
};

use super::{BodyView, EdgeView, FaceView, FinView, LoopView, RegionView, ShellView, VertexView};
use crate::error::{EntityKind, Error, Result};
use crate::session::Part;
use crate::{
    BodyId, BodyIds, EdgeId, EdgeIds, FaceId, FaceIds, FinId, FinIds, LoopId, LoopIds, PartId,
    RegionId, RegionIds, ShellId, ShellIds, VertexId, VertexIds,
};

impl Part<'_> {
    /// Enumerate live bodies in deterministic Store slot order.
    pub fn bodies(&self) -> BodyIds<'_> {
        BodyIds::new(
            self.state
                .store
                .iter::<RawBody>()
                .map(|(raw, _)| BodyId::new(self.id.clone(), raw)),
            self.state.store.count::<RawBody>(),
        )
    }

    /// Enumerate live regions in deterministic Store slot order.
    pub fn regions(&self) -> RegionIds<'_> {
        RegionIds::new(
            self.state
                .store
                .iter::<RawRegion>()
                .map(|(raw, _)| RegionId::new(self.id.clone(), raw)),
            self.state.store.count::<RawRegion>(),
        )
    }

    /// Enumerate live shells in deterministic Store slot order.
    pub fn shells(&self) -> ShellIds<'_> {
        ShellIds::new(
            self.state
                .store
                .iter::<RawShell>()
                .map(|(raw, _)| ShellId::new(self.id.clone(), raw)),
            self.state.store.count::<RawShell>(),
        )
    }

    /// Enumerate live faces in deterministic Store slot order.
    pub fn faces(&self) -> FaceIds<'_> {
        FaceIds::new(
            self.state
                .store
                .iter::<RawFace>()
                .map(|(raw, _)| FaceId::new(self.id.clone(), raw)),
            self.state.store.count::<RawFace>(),
        )
    }

    /// Enumerate live loops in deterministic Store slot order.
    pub fn loops(&self) -> LoopIds<'_> {
        LoopIds::new(
            self.state
                .store
                .iter::<RawLoop>()
                .map(|(raw, _)| LoopId::new(self.id.clone(), raw)),
            self.state.store.count::<RawLoop>(),
        )
    }

    /// Enumerate live fins in deterministic Store slot order.
    pub fn fins(&self) -> FinIds<'_> {
        FinIds::new(
            self.state
                .store
                .iter::<RawFin>()
                .map(|(raw, _)| FinId::new(self.id.clone(), raw)),
            self.state.store.count::<RawFin>(),
        )
    }

    /// Enumerate live edges in deterministic Store slot order.
    pub fn edges(&self) -> EdgeIds<'_> {
        EdgeIds::new(
            self.state
                .store
                .iter::<RawEdge>()
                .map(|(raw, _)| EdgeId::new(self.id.clone(), raw)),
            self.state.store.count::<RawEdge>(),
        )
    }

    /// Enumerate live vertices in deterministic Store slot order.
    pub fn vertices(&self) -> VertexIds<'_> {
        VertexIds::new(
            self.state
                .store
                .iter::<RawVertex>()
                .map(|(raw, _)| VertexId::new(self.id.clone(), raw)),
            self.state.store.count::<RawVertex>(),
        )
    }

    /// Resolve a body ID into an immutable semantic view.
    pub fn body(&self, id: BodyId) -> Result<BodyView<'_>> {
        self.validate_id(id.part())?;
        self.require_live(self.state.store.get(id.raw()), EntityKind::Body)?;
        Ok(BodyView::new(&self.state.store, id))
    }

    /// Resolve a region ID into an immutable semantic view.
    pub fn region(&self, id: RegionId) -> Result<RegionView<'_>> {
        self.validate_id(id.part())?;
        self.require_live(self.state.store.get(id.raw()), EntityKind::Region)?;
        Ok(RegionView::new(&self.state.store, id))
    }

    /// Resolve a shell ID into an immutable semantic view.
    pub fn shell(&self, id: ShellId) -> Result<ShellView<'_>> {
        self.validate_id(id.part())?;
        self.require_live(self.state.store.get(id.raw()), EntityKind::Shell)?;
        Ok(ShellView::new(&self.state.store, id))
    }

    /// Resolve a face ID into an immutable semantic view.
    pub fn face(&self, id: FaceId) -> Result<FaceView<'_>> {
        self.validate_id(id.part())?;
        self.require_live(self.state.store.get(id.raw()), EntityKind::Face)?;
        Ok(FaceView::new(&self.state.store, id))
    }

    /// Resolve a loop ID into an immutable semantic view.
    pub fn loop_(&self, id: LoopId) -> Result<LoopView<'_>> {
        self.validate_id(id.part())?;
        self.require_live(self.state.store.get(id.raw()), EntityKind::Loop)?;
        Ok(LoopView::new(&self.state.store, id))
    }

    /// Resolve a fin ID into an immutable semantic view.
    pub fn fin(&self, id: FinId) -> Result<FinView<'_>> {
        self.validate_id(id.part())?;
        self.require_live(self.state.store.get(id.raw()), EntityKind::Fin)?;
        Ok(FinView::new(&self.state.store, id))
    }

    /// Resolve an edge ID into an immutable semantic view.
    pub fn edge(&self, id: EdgeId) -> Result<EdgeView<'_>> {
        self.validate_id(id.part())?;
        self.require_live(self.state.store.get(id.raw()), EntityKind::Edge)?;
        Ok(EdgeView::new(&self.state.store, id))
    }

    /// Resolve a vertex ID into an immutable semantic view.
    pub fn vertex(&self, id: VertexId) -> Result<VertexView<'_>> {
        self.validate_id(id.part())?;
        self.require_live(self.state.store.get(id.raw()), EntityKind::Vertex)?;
        Ok(VertexView::new(&self.state.store, id))
    }

    fn validate_id(&self, actual: &PartId) -> Result<()> {
        if actual != &self.id {
            return Err(Error::WrongPart {
                expected: self.id.clone(),
                actual: actual.clone(),
            });
        }
        Ok(())
    }

    fn require_live<T>(&self, result: kcore::error::Result<&T>, kind: EntityKind) -> Result<()> {
        result.map(|_| ()).map_err(|_| Error::StaleEntity { kind })
    }
}
