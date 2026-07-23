//! Public value wrappers for certified procedural skew-cylinder branches.
//!
//! The graph certificate owns the nonlinear evaluator. Section owns only the
//! operation-local traversal composition needed to put that evaluator in its
//! canonical orientation.

use kgeom::curve::{Curve, CurveDerivs};
use kgeom::curve2d::{Curve2d, Curve2dDerivs};
use kgeom::param::ParamRange;
use kgeom::vec::{Point2, Point3};
use kgraph::{SkewCylinderBranchCarrier, SkewCylinderBranchPcurve};

use super::{SectionEdgeParameterInterval, SectionSourceParameterKey};
use crate::{FaceId, FinId, LoopId};

/// Projective chart retaining one exact skew-cylinder axial root enclosure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SectionSkewCylinderRootChart {
    /// Tangent half-angle coordinate `tan(u / 2)`.
    TangentHalfAngle,
    /// Cotangent half-angle coordinate `cot(u / 2)`.
    CotangentHalfAngle,
}

/// Exact root corridor in the graph carrier's canonical longitude chart.
///
/// This is deliberately separate from the bounded branch's public carrier
/// interval. The graph residual proof begins at a representable point strictly
/// inside the retained component, while this enclosure owns the physical
/// source-boundary root immediately outside that guarded endpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionSkewCylinderCarrierRootEnclosure {
    pub(super) chart: SectionSkewCylinderRootChart,
    pub(super) lo: f64,
    pub(super) hi: f64,
}

impl SectionSkewCylinderCarrierRootEnclosure {
    /// Projective chart in which the exact isolating enclosure is expressed.
    pub const fn chart(self) -> SectionSkewCylinderRootChart {
        self.chart
    }

    /// Lower projective coordinate of the exact isolating enclosure.
    pub const fn lo(self) -> f64 {
        self.lo
    }

    /// Upper projective coordinate of the exact isolating enclosure.
    pub const fn hi(self) -> f64 {
        self.hi
    }
}

/// Topology-owned source-ring root bounding one procedural fragment end.
///
/// `source_parameter` owns exact endpoint identity in the source edge's
/// intrinsic order. `carrier_root` independently retains the graph solver's
/// exact axial-root corridor; neither identity is inferred from the guarded
/// interior representative exposed by
/// [`SectionBoundedProceduralFragmentEnd`].
#[derive(Debug, Clone, PartialEq)]
pub struct SectionBoundedProceduralTrimProvenance {
    pub(super) operand: usize,
    pub(super) face: FaceId,
    pub(super) loop_id: LoopId,
    pub(super) fin: FinId,
    pub(super) source_parameter: SectionSourceParameterKey,
    pub(super) edge_parameter: SectionEdgeParameterInterval,
    pub(super) carrier_root: SectionSkewCylinderCarrierRootEnclosure,
}

impl SectionBoundedProceduralTrimProvenance {
    /// Operand slot whose axial source boundary contributed this root.
    pub const fn operand(&self) -> usize {
        self.operand
    }

    /// Trimmed cylinder side face.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// Topology-owned cap-ring loop.
    pub fn loop_id(&self) -> LoopId {
        self.loop_id.clone()
    }

    /// Source fin whose whole-ring pcurve owns the intrinsic root.
    pub fn fin(&self) -> FinId {
        self.fin.clone()
    }

    /// Stable source-edge/root identity and scalar materialization authority.
    pub const fn source_parameter(&self) -> &SectionSourceParameterKey {
        &self.source_parameter
    }

    /// Intrinsic source-edge parameter enclosure of the physical root.
    pub const fn edge_parameter(&self) -> SectionEdgeParameterInterval {
        self.edge_parameter
    }

    /// Exact root corridor in the graph carrier's canonical longitude chart.
    pub const fn carrier_root(&self) -> SectionSkewCylinderCarrierRootEnclosure {
        self.carrier_root
    }
}

/// One directed end of a bounded procedural Section fragment.
///
/// The physical root and the residual-certified carrier endpoint are distinct:
/// `root_point` is the canonical source-edge materialization of the exact trim,
/// while `inside_point` and `inside_carrier_parameter` lie strictly inside the
/// retained component and delimit the graph certificate's active range.
/// Combinatorial stitching uses only [`Self::endpoint`].
#[derive(Debug, Clone, PartialEq)]
pub struct SectionBoundedProceduralFragmentEnd {
    pub(super) endpoint: usize,
    pub(super) root_point: Point3,
    pub(super) inside_point: Point3,
    pub(super) inside_carrier_parameter: f64,
    pub(super) trim: SectionBoundedProceduralTrimProvenance,
}

impl SectionBoundedProceduralFragmentEnd {
    /// Index into `BodySectionGraph::curve_endpoints`.
    pub const fn endpoint(&self) -> usize {
        self.endpoint
    }

    /// Canonical source-edge materialization of the physical trim root.
    ///
    /// This point is metric evidence only; exact joins use [`Self::endpoint`].
    pub const fn root_point(&self) -> Point3 {
        self.root_point
    }

    /// Graph-certified model-space representative on the retained inside side.
    pub const fn inside_point(&self) -> Point3 {
        self.inside_point
    }

    /// Section-oriented carrier parameter delimiting the certified interior.
    pub const fn inside_carrier_parameter(&self) -> f64 {
        self.inside_carrier_parameter
    }

    /// Exact topology, source-root, and graph-root provenance.
    pub const fn trim(&self) -> &SectionBoundedProceduralTrimProvenance {
        &self.trim
    }
}

/// Section-oriented facade for one certified skew-cylinder sheet carrier.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionSkewCylinderBranchCarrier {
    source: SkewCylinderBranchCarrier,
    range: ParamRange,
    reversed: bool,
}

impl SectionSkewCylinderBranchCarrier {
    pub(super) const fn new(
        source: SkewCylinderBranchCarrier,
        range: ParamRange,
        reversed: bool,
    ) -> Self {
        Self {
            source,
            range,
            reversed,
        }
    }

    /// Graph-certified procedural carrier before Section orientation.
    pub const fn source(self) -> SkewCylinderBranchCarrier {
        self.source
    }

    /// Complete finite carrier interval.
    pub const fn range(self) -> ParamRange {
        self.range
    }

    /// Whether Section traverses the graph carrier in reverse.
    pub const fn reversed(self) -> bool {
        self.reversed
    }

    /// Evaluate the Section-oriented carrier position.
    pub fn eval(self, parameter: f64) -> Point3 {
        self.eval_derivs(parameter, 0).d[0]
    }

    /// Evaluate position and derivatives through order three.
    pub fn eval_derivs(self, parameter: f64, order: usize) -> CurveDerivs {
        let parameter = composed_parameter(self.range, parameter, self.reversed);
        let mut derivatives = self.source.eval_derivs(parameter, order.min(3));
        apply_reversal(&mut derivatives.d, self.reversed);
        derivatives
    }
}

/// Section-oriented facade for one certified skew-cylinder sheet pcurve.
///
/// Unlike the periodic spatial carrier, this chart trace is bounded. Inputs
/// are therefore clamped to [`Self::range`] before the optional reversal is
/// composed, preserving the source evaluator's bounded-curve contract.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionSkewCylinderBranchPcurve {
    source: SkewCylinderBranchPcurve,
    range: ParamRange,
    reversed: bool,
}

impl SectionSkewCylinderBranchPcurve {
    pub(super) const fn new(
        source: SkewCylinderBranchPcurve,
        range: ParamRange,
        reversed: bool,
    ) -> Self {
        Self {
            source,
            range,
            reversed,
        }
    }

    /// Graph-certified procedural pcurve before Section orientation.
    pub const fn source(self) -> SkewCylinderBranchPcurve {
        self.source
    }

    /// Complete finite carrier interval accepted by this bounded trace.
    pub const fn range(self) -> ParamRange {
        self.range
    }

    /// Whether Section traverses the graph pcurve in reverse.
    pub const fn reversed(self) -> bool {
        self.reversed
    }

    /// Evaluate the Section-oriented parameter-space position.
    pub fn eval(self, parameter: f64) -> Point2 {
        self.eval_derivs(parameter, 0).d[0]
    }

    /// Evaluate position and derivatives through order three.
    pub fn eval_derivs(self, parameter: f64, order: usize) -> Curve2dDerivs {
        let bounded = self.range.clamp_param(parameter);
        let parameter = composed_parameter(self.range, bounded, self.reversed)
            .clamp(self.range.lo, self.range.hi);
        let mut derivatives = self.source.eval_derivs(parameter, order.min(3));
        apply_reversal(&mut derivatives.d, self.reversed);
        derivatives
    }
}

fn composed_parameter(range: ParamRange, parameter: f64, reversed: bool) -> f64 {
    if !reversed {
        return parameter;
    }
    if parameter == range.lo {
        return range.hi;
    }
    if parameter == range.hi {
        return range.lo;
    }
    range.lo + range.hi - parameter
}

fn apply_reversal<const N: usize, Vector>(derivatives: &mut [Vector; N], reversed: bool)
where
    Vector: Copy + core::ops::Neg<Output = Vector>,
{
    if !reversed {
        return;
    }
    for order in (1..N).step_by(2) {
        derivatives[order] = -derivatives[order];
    }
}
