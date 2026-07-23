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
