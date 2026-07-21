//! Public facade evidence for topology-clipped affine ruling fragments.

use kcore::interval::Interval;

use super::{SectionEdgeParameterInterval, SectionSourceParameterKey};
use crate::{FaceId, FinId, LoopId, Point3};

/// Conservative enclosure of one affine carrier parameter.
///
/// For a tolerance-certified fin, the enclosure is the hull of the pcurve
/// crossing and the associated analytic source-root projection. It is
/// ordering and consistency evidence only: source-edge root identity, not
/// overlap or proximity of metric bounds, owns fragment endpoint joins.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionCarrierParameterInterval {
    pub(super) lo: f64,
    pub(super) hi: f64,
}

impl SectionCarrierParameterInterval {
    pub(super) fn from_interval(interval: Interval) -> Self {
        Self {
            lo: interval.lo(),
            hi: interval.hi(),
        }
    }

    /// Lower bound of the closed carrier-parameter enclosure.
    pub const fn lo(self) -> f64 {
        self.lo
    }

    /// Upper bound of the closed carrier-parameter enclosure.
    pub const fn hi(self) -> f64 {
        self.hi
    }
}

/// Topology and parameter provenance for one ruling-fragment trim event.
///
/// [`SectionSourceParameterKey`] is the exact identity used for joins. The
/// edge and carrier parameter enclosures are supporting consistency and
/// ordering evidence and never substitute for that identity.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionRulingTrimProvenance {
    pub(super) operand: usize,
    pub(super) face: FaceId,
    pub(super) loop_id: LoopId,
    pub(super) fin: FinId,
    pub(super) source_parameter: SectionSourceParameterKey,
    pub(super) edge_parameter: SectionEdgeParameterInterval,
    pub(super) carrier_parameter: SectionCarrierParameterInterval,
}

impl SectionRulingTrimProvenance {
    /// Operand slot whose face trim contributed this event.
    pub const fn operand(&self) -> usize {
        self.operand
    }

    /// Trimmed source face.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// Source boundary loop.
    pub fn loop_id(&self) -> LoopId {
        self.loop_id.clone()
    }

    /// Source fin whose pcurve supplied the crossing equation.
    pub fn fin(&self) -> FinId {
        self.fin.clone()
    }

    /// Stable source-edge/root identity that owns endpoint joins.
    pub const fn source_parameter(&self) -> &SectionSourceParameterKey {
        &self.source_parameter
    }

    /// Conservative hull of the pcurve observation and analytic source root
    /// in the intrinsic edge parameterization.
    pub const fn edge_parameter(&self) -> SectionEdgeParameterInterval {
        self.edge_parameter
    }

    /// Conservative pcurve/root-projection hull used for ordering.
    pub const fn carrier_parameter(&self) -> SectionCarrierParameterInterval {
        self.carrier_parameter
    }
}

/// One directed occurrence of a stitched ruling-fragment endpoint.
///
/// The point and scalar carrier parameter are metric representatives. The
/// optional operand-local trim provenance carries the exact source-root
/// identities that own joins.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionRulingFragmentEnd {
    pub(super) endpoint: usize,
    pub(super) point: Point3,
    pub(super) carrier_parameter: f64,
    pub(super) trims: [Option<SectionRulingTrimProvenance>; 2],
}

impl SectionRulingFragmentEnd {
    /// Index into the section graph's proof-keyed endpoint collection.
    pub const fn endpoint(&self) -> usize {
        self.endpoint
    }

    /// Numeric model-space representative (evidence, not join authority).
    pub const fn point(&self) -> Point3 {
        self.point
    }

    /// Numeric representative in the source branch's canonical parameter.
    pub const fn carrier_parameter(&self) -> f64 {
        self.carrier_parameter
    }

    /// Operand-local exact trim provenance.
    pub const fn trims(&self) -> &[Option<SectionRulingTrimProvenance>; 2] {
        &self.trims
    }
}
