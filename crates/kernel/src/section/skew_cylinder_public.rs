//! Public value wrappers for certified procedural skew-cylinder branches.
//!
//! The graph certificate owns the nonlinear evaluator. Section owns only the
//! operation-local traversal composition needed to put that evaluator in its
//! canonical orientation.

use kcore::interval::Interval;
use kgeom::curve::{Curve, CurveDerivs};
use kgeom::curve2d::{Curve2d, Curve2dDerivs};
use kgeom::param::ParamRange;
use kgeom::vec::{Point2, Point3};
use kgraph::{
    SKEW_CYLINDER_BRANCH_PCURVE_ALL_CELLS_WORK, SKEW_CYLINDER_BRANCH_PCURVE_CELL_WORK,
    SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK, SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS,
    SkewCylinderBranchCarrier, SkewCylinderBranchPcurve, SkewCylinderBranchPcurveCellCertificate,
    SkewCylinderBranchPcurveEnclosure, SkewCylinderBranchPcurveRootCorridorCertificate,
};
use kops::intersect::SkewCylinderOpenSpanBranchCertificate;

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

/// Outward interval retained by a skew-cylinder embedding certificate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionSkewCylinderInterval {
    lo: f64,
    hi: f64,
}

impl SectionSkewCylinderInterval {
    fn from_interval(value: Interval) -> Self {
        Self {
            lo: value.lo(),
            hi: value.hi(),
        }
    }

    /// Lower outward endpoint.
    pub const fn lo(self) -> f64 {
        self.lo
    }

    /// Upper outward endpoint.
    pub const fn hi(self) -> f64 {
        self.hi
    }

    /// Whether this interval contains `value`.
    pub const fn contains(self, value: f64) -> bool {
        self.lo <= value && value <= self.hi
    }
}

/// Stored and exact-source pcurve enclosures for one Section operand.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionSkewCylinderPcurveEnclosure {
    stored_uv: [SectionSkewCylinderInterval; 2],
    stored_derivative: [SectionSkewCylinderInterval; 2],
    source_uv: [SectionSkewCylinderInterval; 2],
    source_derivative: [SectionSkewCylinderInterval; 2],
}

impl SectionSkewCylinderPcurveEnclosure {
    fn from_source(source: SkewCylinderBranchPcurveEnclosure, reversed: bool) -> Self {
        Self {
            stored_uv: source
                .stored_uv()
                .map(SectionSkewCylinderInterval::from_interval),
            stored_derivative: orient_derivatives(source.stored_derivative(), reversed),
            source_uv: source
                .source_uv()
                .map(SectionSkewCylinderInterval::from_interval),
            source_derivative: orient_derivatives(source.source_derivative(), reversed),
        }
    }

    /// Procedural-evaluator longitude/height enclosure.
    pub const fn stored_uv(&self) -> &[SectionSkewCylinderInterval; 2] {
        &self.stored_uv
    }

    /// Exact-source longitude/height enclosure.
    pub const fn source_uv(&self) -> &[SectionSkewCylinderInterval; 2] {
        &self.source_uv
    }

    /// Procedural-evaluator derivative with respect to Section parameter.
    pub const fn stored_derivative(&self) -> &[SectionSkewCylinderInterval; 2] {
        &self.stored_derivative
    }

    /// Exact-source derivative with respect to Section parameter.
    pub const fn source_derivative(&self) -> &[SectionSkewCylinderInterval; 2] {
        &self.source_derivative
    }

    /// Whether the stored derivative box excludes the zero vector.
    pub fn stored_is_strictly_regular(&self) -> bool {
        derivative_box_is_regular(self.stored_derivative)
    }

    /// Whether the exact-source derivative box excludes the zero vector.
    pub fn source_is_strictly_regular(&self) -> bool {
        derivative_box_is_regular(self.source_derivative)
    }
}

/// Reissued pcurve proof over one closed Section-oriented carrier interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionSkewCylinderPcurveCellCertificate {
    parameter: SectionSkewCylinderInterval,
    pcurves: [SectionSkewCylinderPcurveEnclosure; 2],
}

impl SectionSkewCylinderPcurveCellCertificate {
    /// Closed carrier interval in Section traversal orientation.
    pub const fn parameter(self) -> SectionSkewCylinderInterval {
        self.parameter
    }

    /// Pcurve enclosures in current Section operand order.
    pub const fn pcurves(&self) -> &[SectionSkewCylinderPcurveEnclosure; 2] {
        &self.pcurves
    }

    /// Fixed logical work for reissuing this cell.
    pub const fn work(self) -> u64 {
        SKEW_CYLINDER_BRANCH_PCURVE_CELL_WORK
    }
}

/// Physical-root and closed root-to-guard pcurve evidence for one Section end.
///
/// `root_pcurves` encloses the physical topology root. `corridor` includes the
/// continuation to the guarded residual range; neither is the numeric guarded
/// representative retained by [`SectionBoundedProceduralFragmentEnd`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionSkewCylinderRootCorridorCertificate {
    section_end: usize,
    root_parameter: SectionSkewCylinderInterval,
    root_pcurves: [SectionSkewCylinderPcurveEnclosure; 2],
    corridor: SectionSkewCylinderPcurveCellCertificate,
}

impl SectionSkewCylinderRootCorridorCertificate {
    /// Directed Section end (`0` start or `1` end).
    pub const fn section_end(self) -> usize {
        self.section_end
    }

    /// Physical root enclosure in Section-oriented carrier parameter.
    pub const fn root_parameter(self) -> SectionSkewCylinderInterval {
        self.root_parameter
    }

    /// Physical-root pcurve enclosures in current Section operand order.
    pub const fn root_pcurves(&self) -> &[SectionSkewCylinderPcurveEnclosure; 2] {
        &self.root_pcurves
    }

    /// Closed physical-root-to-guard continuation certificate.
    pub const fn corridor(self) -> SectionSkewCylinderPcurveCellCertificate {
        self.corridor
    }

    /// Fixed logical work for root and corridor recertification together.
    pub const fn work(self) -> u64 {
        SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK
    }
}

/// Sealed Section-oriented nonlinear pcurve embedding authority.
///
/// The compact graph residual certificate is retained once per branch. The
/// two physical-root corridors stay attached in directed Section end order.
/// Indexed guarded cells are reissued on demand and are never replaced by an
/// aggregate bounding box or point samples.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionSkewCylinderEmbeddingCertificate {
    source: SkewCylinderOpenSpanBranchCertificate,
    range: ParamRange,
    reversed: bool,
}

impl SectionSkewCylinderEmbeddingCertificate {
    pub(super) fn new(
        source: SkewCylinderOpenSpanBranchCertificate,
        range: ParamRange,
        reversed: bool,
    ) -> Option<Self> {
        (range.is_finite() && source.residual_certificate().carrier_range() == range).then_some(
            Self {
                source,
                range,
                reversed,
            },
        )
    }

    /// Complete guarded carrier range in Section parameter.
    pub const fn range(&self) -> ParamRange {
        self.range
    }

    /// Whether Section traversal reverses the graph carrier.
    pub const fn reversed(&self) -> bool {
        self.reversed
    }

    /// Number of fixed indexed guarded cells.
    pub const fn guarded_cell_count(&self) -> usize {
        SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS
    }

    /// Fixed logical work for one guarded cell.
    pub const fn guarded_cell_work(&self) -> u64 {
        SKEW_CYLINDER_BRANCH_PCURVE_CELL_WORK
    }

    /// Fixed logical work for all guarded cells.
    pub const fn all_guarded_cells_work(&self) -> u64 {
        SKEW_CYLINDER_BRANCH_PCURVE_ALL_CELLS_WORK
    }

    /// Fixed logical work for one physical-root corridor.
    pub const fn root_corridor_work(&self) -> u64 {
        SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK
    }

    /// Fixed logical work for all guarded cells and both root corridors.
    pub const fn total_work(&self) -> u64 {
        SKEW_CYLINDER_BRANCH_PCURVE_ALL_CELLS_WORK
            + 2 * SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK
    }

    /// Reissue one guarded cell by increasing Section parameter index.
    pub fn guarded_cell(
        &self,
        section_index: usize,
    ) -> Option<SectionSkewCylinderPcurveCellCertificate> {
        if section_index >= SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
            return None;
        }
        let graph_index = if self.reversed {
            SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS - 1 - section_index
        } else {
            section_index
        };
        let source = self.source.certify_pcurve_cell(graph_index).ok()?;
        Some(self.orient_cell(source, true))
    }

    /// Reissue physical-root evidence for directed Section end `0` or `1`.
    pub fn root_corridor(
        &self,
        section_end: usize,
    ) -> Option<SectionSkewCylinderRootCorridorCertificate> {
        if section_end > 1 {
            return None;
        }
        let graph_end = if self.reversed {
            1 - section_end
        } else {
            section_end
        };
        let source = self.source.root_corridors()[graph_end];
        Some(SectionSkewCylinderRootCorridorCertificate {
            section_end,
            root_parameter: orient_parameter_interval(
                self.range,
                source.root_parameter(),
                self.reversed,
            ),
            root_pcurves: source.root_pcurves().map(|pcurve| {
                SectionSkewCylinderPcurveEnclosure::from_source(pcurve, self.reversed)
            }),
            corridor: self.orient_cell(source.corridor(), false),
        })
    }

    pub(super) fn source_root_corridor(
        &self,
        section_end: usize,
    ) -> Option<SkewCylinderBranchPcurveRootCorridorCertificate> {
        if section_end > 1 {
            return None;
        }
        let graph_end = if self.reversed {
            1 - section_end
        } else {
            section_end
        };
        Some(self.source.root_corridors()[graph_end])
    }

    fn orient_cell(
        &self,
        source: SkewCylinderBranchPcurveCellCertificate,
        clamp_to_guarded_range: bool,
    ) -> SectionSkewCylinderPcurveCellCertificate {
        let mut parameter =
            orient_parameter_interval(self.range, source.parameter(), self.reversed);
        if clamp_to_guarded_range {
            parameter.lo = parameter.lo.max(self.range.lo);
            parameter.hi = parameter.hi.min(self.range.hi);
        }
        SectionSkewCylinderPcurveCellCertificate {
            parameter,
            pcurves: source.pcurves().map(|pcurve| {
                SectionSkewCylinderPcurveEnclosure::from_source(pcurve, self.reversed)
            }),
        }
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

pub(super) fn orient_parameter_interval(
    range: ParamRange,
    source: Interval,
    reversed: bool,
) -> SectionSkewCylinderInterval {
    let oriented = if reversed {
        Interval::point(range.lo) + Interval::point(range.hi) - source
    } else {
        source
    };
    SectionSkewCylinderInterval::from_interval(oriented)
}

fn orient_derivatives(source: [Interval; 2], reversed: bool) -> [SectionSkewCylinderInterval; 2] {
    source.map(|value| {
        SectionSkewCylinderInterval::from_interval(if reversed { -value } else { value })
    })
}

fn derivative_box_is_regular(derivative: [SectionSkewCylinderInterval; 2]) -> bool {
    derivative
        .into_iter()
        .any(|coordinate| coordinate.lo() > 0.0 || coordinate.hi() < 0.0)
}
