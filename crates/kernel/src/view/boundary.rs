//! Face, loop, and fin boundary views.

use ktopo::entity::{Face as RawFace, Fin as RawFin, Loop as RawLoop};
use ktopo::store::Store;

use crate::error::{Error, Result};
use crate::{
    EdgeId, EntityTolerance, FaceDomain, FaceId, FinId, FinIds, LoopId, LoopIds, Sense, ShellId,
    VertexId,
};

/// Read-only face view. Geometry identity is intentionally deferred to K3.
pub struct FaceView<'part> {
    store: &'part Store,
    id: FaceId,
}

impl<'part> FaceView<'part> {
    pub(crate) fn new(store: &'part Store, id: FaceId) -> Self {
        Self { store, id }
    }

    fn entity(&self) -> &RawFace {
        self.store
            .get(self.id.raw())
            .expect("an immutable validated face view remains live")
    }

    /// Face identity.
    pub fn id(&self) -> FaceId {
        self.id.clone()
    }

    /// Owning shell.
    pub fn shell(&self) -> ShellId {
        ShellId::new(self.id.part().clone(), self.entity().shell)
    }

    /// Boundary loops in stored topological order.
    pub fn loops(&self) -> LoopIds<'_> {
        let loops = &self.entity().loops;
        LoopIds::new(
            loops
                .iter()
                .map(|&raw| LoopId::new(self.id.part().clone(), raw)),
            loops.len(),
        )
    }

    /// Face orientation relative to its supporting surface.
    pub fn sense(&self) -> Sense {
        self.entity().sense
    }

    /// Conservative finite parameter domain, when known.
    pub fn domain(&self) -> Option<FaceDomain> {
        self.entity().domain
    }

    /// Imported or operation tolerance, when present.
    pub fn tolerance(&self) -> Option<EntityTolerance> {
        self.entity().tolerance
    }
}

/// Read-only loop view.
pub struct LoopView<'part> {
    store: &'part Store,
    id: LoopId,
}

impl<'part> LoopView<'part> {
    pub(crate) fn new(store: &'part Store, id: LoopId) -> Self {
        Self { store, id }
    }

    fn entity(&self) -> &RawLoop {
        self.store
            .get(self.id.raw())
            .expect("an immutable validated loop view remains live")
    }

    /// Loop identity.
    pub fn id(&self) -> LoopId {
        self.id.clone()
    }

    /// Owning face.
    pub fn face(&self) -> FaceId {
        FaceId::new(self.id.part().clone(), self.entity().face)
    }

    /// Fins in stored loop traversal order.
    pub fn fins(&self) -> FinIds<'_> {
        let fins = &self.entity().fins;
        FinIds::new(
            fins.iter()
                .map(|&raw| FinId::new(self.id.part().clone(), raw)),
            fins.len(),
        )
    }
}

/// Read-only fin view. Pcurve geometry identity is deferred to K3.
pub struct FinView<'part> {
    store: &'part Store,
    id: FinId,
}

impl<'part> FinView<'part> {
    pub(crate) fn new(store: &'part Store, id: FinId) -> Self {
        Self { store, id }
    }

    fn entity(&self) -> &RawFin {
        self.store
            .get(self.id.raw())
            .expect("an immutable validated fin view remains live")
    }

    /// Fin identity.
    pub fn id(&self) -> FinId {
        self.id.clone()
    }

    /// Owning loop.
    pub fn loop_(&self) -> LoopId {
        LoopId::new(self.id.part().clone(), self.entity().parent)
    }

    /// Edge used by this fin.
    pub fn edge(&self) -> EdgeId {
        EdgeId::new(self.id.part().clone(), self.entity().edge)
    }

    /// Traversal direction relative to the edge.
    pub fn sense(&self) -> Sense {
        self.entity().sense
    }

    /// Tail vertex in fin traversal direction, or `None` for a ring edge.
    pub fn tail(&self) -> Result<Option<VertexId>> {
        self.store
            .fin_tail(self.id.raw())
            .map(|value| value.map(|raw| VertexId::new(self.id.part().clone(), raw)))
            .map_err(|source| Error::InconsistentTopology { source })
    }

    /// Head vertex in fin traversal direction, or `None` for a ring edge.
    pub fn head(&self) -> Result<Option<VertexId>> {
        self.store
            .fin_head(self.id.raw())
            .map(|value| value.map(|raw| VertexId::new(self.id.part().clone(), raw)))
            .map_err(|source| Error::InconsistentTopology { source })
    }
}
