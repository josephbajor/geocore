//! Face, loop, and fin boundary views.

use ktopo::entity::{Face as RawFace, FaceDomain as RawFaceDomain, Fin as RawFin, Loop as RawLoop};
use ktopo::store::Store;

use crate::error::{Error, Result};
use crate::{
    EdgeId, EntityTolerance, FaceId, FinId, FinIds, LoopId, LoopIds, ParamRange, PcurveId,
    PcurveParameterMap, Sense, ShellId, SurfaceId, VertexId,
};

/// Conservative finite parameter-space work box exposed without a supporting
/// graph descriptor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceDomain {
    u: ParamRange,
    v: ParamRange,
}

impl FaceDomain {
    pub(crate) const fn from_raw(domain: RawFaceDomain) -> Self {
        Self {
            u: domain.u,
            v: domain.v,
        }
    }

    /// Conservative range in the surface's first parameter.
    pub const fn u(self) -> ParamRange {
        self.u
    }

    /// Conservative range in the surface's second parameter.
    pub const fn v(self) -> ParamRange {
        self.v
    }

    /// Midpoint of this finite conservative parameter-space work box.
    ///
    /// This is a deterministic interior sample for application inspection;
    /// it does not prove that the point lies inside the face's trimmed region.
    pub fn center(self) -> [f64; 2] {
        [self.u.lerp(0.5), self.v.lerp(0.5)]
    }

    /// Whether a parameter lies in this conservative box.
    pub fn contains(self, uv: [f64; 2]) -> bool {
        self.u.contains(uv[0]) && self.v.contains(uv[1])
    }
}

/// Read-only face view.
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
        ShellId::new(self.id.part().clone(), self.entity().shell())
    }

    /// Boundary loops in stored topological order.
    pub fn loops(&self) -> LoopIds<'_> {
        let loops = self.entity().loops();
        LoopIds::new(
            loops
                .iter()
                .map(|&raw| LoopId::new(self.id.part().clone(), raw)),
            loops.len(),
        )
    }

    /// Authoritative supporting-surface identity.
    pub fn surface(&self) -> SurfaceId {
        SurfaceId::new(self.id.part().clone(), self.entity().surface())
    }

    /// Face orientation relative to its supporting surface.
    pub fn sense(&self) -> Sense {
        self.entity().sense()
    }

    /// Conservative finite parameter domain, when known.
    pub fn domain(&self) -> Option<FaceDomain> {
        self.entity().domain().map(FaceDomain::from_raw)
    }

    /// Imported or operation tolerance, when present.
    pub fn tolerance(&self) -> Option<EntityTolerance> {
        self.entity().tolerance()
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
        FaceId::new(self.id.part().clone(), self.entity().face())
    }

    /// Fins in stored loop traversal order.
    pub fn fins(&self) -> FinIds<'_> {
        let fins = self.entity().fins();
        FinIds::new(
            fins.iter()
                .map(|&raw| FinId::new(self.id.part().clone(), raw)),
            fins.len(),
        )
    }
}

/// Read-only fin view.
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
        LoopId::new(self.id.part().clone(), self.entity().parent())
    }

    /// Edge used by this fin.
    pub fn edge(&self) -> EdgeId {
        EdgeId::new(self.id.part().clone(), self.entity().edge())
    }

    /// Traversal direction relative to the edge.
    pub fn sense(&self) -> Sense {
        self.entity().sense()
    }

    /// Attached parameter-space curve identity, when authored.
    pub fn pcurve(&self) -> Option<PcurveId> {
        self.entity()
            .pcurve()
            .map(|use_| PcurveId::new(self.id.part().clone(), use_.curve()))
    }

    /// Active pcurve parameter interval, when authored.
    pub fn pcurve_range(&self) -> Option<ParamRange> {
        self.entity().pcurve().map(|use_| use_.range())
    }

    /// Edge-to-pcurve affine correspondence, when authored.
    pub fn pcurve_parameter_map(&self) -> Option<PcurveParameterMap> {
        self.entity()
            .pcurve()
            .map(|use_| PcurveParameterMap::from_raw(use_.edge_to_pcurve()))
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_domain_center_is_deterministic_and_contained() {
        let domain = FaceDomain {
            u: ParamRange::new(-2.0, 6.0),
            v: ParamRange::new(3.0, 5.0),
        };

        assert_eq!(domain.center(), [2.0, 4.0]);
        assert!(domain.contains(domain.center()));
    }
}
