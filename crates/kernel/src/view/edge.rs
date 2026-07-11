//! Edge and vertex views.

use ktopo::entity::{Edge as RawEdge, Vertex as RawVertex};
use ktopo::store::Store;

use crate::error::{Error, Result};
use crate::{EdgeId, EntityTolerance, FinId, FinIds, Point3, VertexId};

/// Read-only edge view. Supporting curve identity is deferred to K3.
pub struct EdgeView<'part> {
    store: &'part Store,
    id: EdgeId,
}

impl<'part> EdgeView<'part> {
    pub(crate) fn new(store: &'part Store, id: EdgeId) -> Self {
        Self { store, id }
    }

    fn entity(&self) -> &RawEdge {
        self.store
            .get(self.id.raw())
            .expect("an immutable validated edge view remains live")
    }

    /// Edge identity.
    pub fn id(&self) -> EdgeId {
        self.id.clone()
    }

    /// Start/end vertices in edge direction; both are absent for a ring edge.
    pub fn vertices(&self) -> [Option<VertexId>; 2] {
        let [start, end] = self.entity().vertices;
        [
            start.map(|raw| VertexId::new(self.id.part().clone(), raw)),
            end.map(|raw| VertexId::new(self.id.part().clone(), raw)),
        ]
    }

    /// Active edge parameter interval, absent for a full-period ring edge.
    pub fn bounds(&self) -> Option<(f64, f64)> {
        self.entity().bounds
    }

    /// Fins in stored creation order.
    pub fn fins(&self) -> FinIds<'_> {
        let fins = &self.entity().fins;
        FinIds::new(
            fins.iter()
                .map(|&raw| FinId::new(self.id.part().clone(), raw)),
            fins.len(),
        )
    }

    /// Tolerant-edge metric data, when present.
    pub fn tolerance(&self) -> Option<EntityTolerance> {
        self.entity().tolerance
    }
}

/// Read-only vertex view.
pub struct VertexView<'part> {
    store: &'part Store,
    id: VertexId,
}

impl<'part> VertexView<'part> {
    pub(crate) fn new(store: &'part Store, id: VertexId) -> Self {
        Self { store, id }
    }

    fn entity(&self) -> &RawVertex {
        self.store
            .get(self.id.raw())
            .expect("an immutable validated vertex view remains live")
    }

    /// Vertex identity.
    pub fn id(&self) -> VertexId {
        self.id.clone()
    }

    /// Model-space position.
    pub fn position(&self) -> Result<Point3> {
        self.store
            .vertex_position(self.id.raw())
            .map_err(|source| Error::InconsistentTopology { source })
    }

    /// Tolerant-vertex metric data, when present.
    pub fn tolerance(&self) -> Option<EntityTolerance> {
        self.entity().tolerance
    }
}
