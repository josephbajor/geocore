//! Opaque, part-qualified façade identities.

use core::fmt;
use core::hash::{Hash, Hasher};
use std::sync::Arc;

use kcore::arena::Handle;
use kgraph::{
    Curve2dHandle as RawPcurveId, CurveHandle as RawCurveId, SurfaceHandle as RawSurfaceId,
};
use ktopo::entity::{
    BodyId as RawBodyId, EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId,
    LoopId as RawLoopId, RegionId as RawRegionId, ShellId as RawShellId, VertexId as RawVertexId,
};

use crate::session::PartState;

#[derive(Debug)]
pub(crate) struct SessionMarker;

#[derive(Clone)]
pub(crate) struct SessionIdentity(Arc<SessionMarker>);

impl SessionIdentity {
    pub(crate) fn new() -> Self {
        Self(Arc::new(SessionMarker))
    }

    pub(crate) fn same(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

impl PartialEq for SessionIdentity {
    fn eq(&self, other: &Self) -> bool {
        self.same(other)
    }
}

impl Eq for SessionIdentity {}

impl Hash for SessionIdentity {
    fn hash<H: Hasher>(&self, state: &mut H) {
        Arc::as_ptr(&self.0).hash(state);
    }
}

/// Opaque identity of one session-owned part.
#[derive(Clone)]
pub struct PartId {
    session: SessionIdentity,
    handle: Handle<PartState>,
}

impl PartId {
    pub(crate) fn new(session: SessionIdentity, handle: Handle<PartState>) -> Self {
        Self { session, handle }
    }

    pub(crate) fn belongs_to(&self, session: &SessionIdentity) -> bool {
        self.session.same(session)
    }

    pub(crate) fn handle(&self) -> Handle<PartState> {
        self.handle
    }
}

impl PartialEq for PartId {
    fn eq(&self, other: &Self) -> bool {
        self.session == other.session && self.handle == other.handle
    }
}

impl Eq for PartId {}

impl Hash for PartId {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.session.hash(state);
        self.handle.hash(state);
    }
}

impl fmt::Debug for PartId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("PartId(<opaque>)")
    }
}

macro_rules! facade_id {
    ($name:ident, $raw:ty, $label:literal) => {
        #[doc = concat!("Opaque, part-qualified identity of one ", $label, ".")]
        #[derive(Clone, PartialEq, Eq, Hash)]
        pub struct $name {
            part: PartId,
            raw: $raw,
        }

        impl $name {
            pub(crate) fn new(part: PartId, raw: $raw) -> Self {
                Self { part, raw }
            }

            pub(crate) fn part(&self) -> &PartId {
                &self.part
            }

            pub(crate) fn raw(&self) -> $raw {
                self.raw
            }
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(concat!(stringify!($name), "(<opaque>)"))
            }
        }
    };
}

facade_id!(BodyId, RawBodyId, "body");
facade_id!(RegionId, RawRegionId, "region");
facade_id!(ShellId, RawShellId, "shell");
facade_id!(FaceId, RawFaceId, "face");
facade_id!(LoopId, RawLoopId, "loop");
facade_id!(FinId, RawFinId, "fin");
facade_id!(EdgeId, RawEdgeId, "edge");
facade_id!(VertexId, RawVertexId, "vertex");

facade_id!(CurveId, RawCurveId, "3D curve geometry node");
facade_id!(SurfaceId, RawSurfaceId, "surface geometry node");
facade_id!(PcurveId, RawPcurveId, "parameter-space curve geometry node");
