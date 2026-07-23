//! Persistent normalized composite for one bounded skew-cylinder open span.
//!
//! The source proof remains expressed in canonical cylinder longitude.  This
//! module composes its lower root enclosure/corridor, guarded residual range,
//! and upper corridor/root enclosure behind one nonperiodic logical `[0, 1]`
//! evaluator.  The two representable longitude values used internally at the
//! logical endpoints are metric evaluation coordinates only.  They are never
//! exposed as, or usable as, physical-root scalar authority.

use super::*;
use crate::{Curve2dHandle, GeometryRef, SurfaceHandle};

const LOGICAL_RANGE: ParamRange = ParamRange { lo: 0.0, hi: 1.0 };

/// Exact logical work represented by one persistent open-span proof.
///
/// The guarded residual certificate consumes 256 fixed cells and each of the
/// two sealed root corridors consumes two units (root enclosure plus corridor).
pub const PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK: u64 =
    SKEW_CYLINDER_BRANCH_CERTIFICATE_WORK + 2 * SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK;

/// Logical traversal orientation of a persistent open span.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderOpenSpanOrientation {
    /// Logical zero starts in the lower canonical-longitude root enclosure.
    Forward,
    /// Logical zero starts in the upper canonical-longitude root enclosure.
    Reversed,
}

impl PersistentSkewCylinderOpenSpanOrientation {
    fn orient_pair<T: Copy>(self, pair: [T; 2]) -> [T; 2] {
        match self {
            Self::Forward => pair,
            Self::Reversed => [pair[1], pair[0]],
        }
    }
}

/// Sealed persistent proof for one bounded skew Cylinder/Cylinder component.
///
/// The physical endpoint points are metric topology evidence supplied by the
/// root-owning operation.  `required_edge_tolerance` is derived, never chosen
/// by a later topology caller: it outwardly covers both logical endpoint
/// evaluation representatives versus those points and every paired
/// carrier/pcurve residual over the complete composite domain.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderOpenSpanCertificate {
    residual: PairedSkewCylinderBranchResidualCertificate,
    root_corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
    orientation: PersistentSkewCylinderOpenSpanOrientation,
    carrier: PersistentSkewCylinderOpenSpanCarrier,
    pcurves: [PersistentSkewCylinderOpenSpanPcurve; 2],
    endpoint_points: [Vec3; 2],
    residual_bounds: [f64; 2],
    required_edge_tolerance: f64,
}

impl PersistentSkewCylinderOpenSpanCertificate {
    /// Compact guarded paired residual proof.
    pub const fn residual_certificate(self) -> PairedSkewCylinderBranchResidualCertificate {
        self.residual
    }

    /// Root corridors in canonical `[lower, upper]` longitude order.
    pub const fn root_corridors(self) -> [SkewCylinderBranchPcurveRootCorridorCertificate; 2] {
        self.root_corridors
    }

    /// Logical traversal orientation.
    pub const fn orientation(self) -> PersistentSkewCylinderOpenSpanOrientation {
        self.orientation
    }

    /// Nonperiodic logical evaluator domain.
    pub const fn logical_range(self) -> ParamRange {
        LOGICAL_RANGE
    }

    /// Persistent normalized spatial evaluator.
    pub const fn carrier(self) -> PersistentSkewCylinderOpenSpanCarrier {
        self.carrier
    }

    /// Persistent normalized pcurves in current source order.
    pub const fn pcurves(self) -> [PersistentSkewCylinderOpenSpanPcurve; 2] {
        self.pcurves
    }

    /// Topology metric endpoint points in logical `[0, 1]` order.
    pub const fn endpoint_points(self) -> [Vec3; 2] {
        self.endpoint_points
    }

    /// Complete-domain paired residual bounds in current source order.
    pub const fn residual_bounds(self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Minimum metric tolerance required by a topology edge using this proof.
    ///
    /// A topology layer may floor this at its session linear resolution, but
    /// must not author a smaller value.
    pub const fn required_edge_tolerance(self) -> f64 {
        self.required_edge_tolerance
    }

    /// Exact logical proof work represented by this certificate.
    pub const fn work(self) -> u64 {
        PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK
    }
}

/// Certify one persistent logical open-span composite.
///
/// `root_corridors` and `physical_endpoint_points` are supplied in canonical
/// `[lower, upper]` longitude order.  Each corridor must be sealed by exactly
/// `residual`; mixing evidence from another sheet, source order, chart, or
/// tolerance is rejected.  The returned endpoint points are oriented with the
/// logical domain.
///
/// The certifier chooses deterministic representable longitude coordinates
/// inside the two root enclosures solely to evaluate the normalized curve and
/// pcurves.  Those hidden coordinates are not physical roots and are not trim
/// scalars.
pub fn certify_persistent_skew_cylinder_open_span(
    residual: PairedSkewCylinderBranchResidualCertificate,
    root_corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
    physical_endpoint_points: [Vec3; 2],
    orientation: PersistentSkewCylinderOpenSpanOrientation,
) -> Result<PersistentSkewCylinderOpenSpanCertificate, IntersectionCertificateError> {
    let canonical_representatives =
        validate_composite_inputs(residual, root_corridors, physical_endpoint_points)?;
    let representatives = orientation.orient_pair(canonical_representatives);
    let parameter_map = PersistentSkewCylinderLogicalMap {
        affine: AffineParamMap1d::new(representatives[1] - representatives[0], representatives[0])?,
        endpoint_representatives: representatives,
    };
    let (carrier, pcurves, residual_bounds) =
        build_composite_evaluators(residual, root_corridors, parameter_map)?;
    let endpoint_points = orientation.orient_pair(physical_endpoint_points);
    let required_edge_tolerance = certify_required_edge_tolerance(
        residual,
        carrier,
        pcurves,
        endpoint_points,
        residual_bounds,
    )?;

    Ok(PersistentSkewCylinderOpenSpanCertificate {
        residual,
        root_corridors,
        orientation,
        carrier,
        pcurves,
        endpoint_points,
        residual_bounds,
        required_edge_tolerance,
    })
}

fn validate_composite_inputs(
    residual: PairedSkewCylinderBranchResidualCertificate,
    root_corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
    physical_endpoint_points: [Vec3; 2],
) -> Result<[f64; 2], IntersectionCertificateError> {
    let [lower, upper] = root_corridors;
    let guarded = residual.carrier_range();
    let lower_root = lower.root_parameter();
    let upper_root = upper.root_parameter();
    let reissued_corridors = [
        residual.certify_lower_pcurve_root_corridor(lower_root)?,
        residual.certify_upper_pcurve_root_corridor(upper_root)?,
    ];
    let expected_operands = residual.traces().map(|trace| trace.pcurve().operand());
    let corridors_match = root_corridors.iter().all(|corridor| {
        corridor.root_pcurves().map(|pcurve| pcurve.operand()) == expected_operands
            && corridor.corridor().pcurves().map(|pcurve| pcurve.operand()) == expected_operands
    });
    if lower.guarded_end() != SkewCylinderBranchGuardedEnd::Lower
        || upper.guarded_end() != SkewCylinderBranchGuardedEnd::Upper
        || lower_root.hi() >= guarded.lo
        || upper_root.lo() <= guarded.hi
        || lower.corridor().parameter() != Interval::new(lower_root.lo(), guarded.lo)
        || upper.corridor().parameter() != Interval::new(guarded.hi, upper_root.hi())
        || root_corridors != reissued_corridors
        || !corridors_match
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    if !physical_endpoint_points.into_iter().all(finite3) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    Ok([
        interval_midpoint(lower_root)?,
        interval_midpoint(upper_root)?,
    ])
}

fn build_composite_evaluators(
    residual: PairedSkewCylinderBranchResidualCertificate,
    root_corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
    parameter_map: PersistentSkewCylinderLogicalMap,
) -> Result<
    (
        PersistentSkewCylinderOpenSpanCarrier,
        [PersistentSkewCylinderOpenSpanPcurve; 2],
        [f64; 2],
    ),
    IntersectionCertificateError,
> {
    let [lower, upper] = root_corridors;
    let guarded = residual.carrier_range();
    let carrier_box = residual
        .carrier()
        .bounding_box(guarded)
        .union(lower.corridor().carrier_box())
        .union(upper.corridor().carrier_box());
    if !carrier_box.is_finite() {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let traces = residual.traces();
    let mut pcurve_boxes = traces.map(|trace| trace.pcurve().bounding_box(guarded));
    for corridor in root_corridors {
        for (index, enclosure) in corridor.corridor().pcurves().into_iter().enumerate() {
            pcurve_boxes[index] = pcurve_boxes[index].union(pcurve_box(enclosure.stored_uv()));
        }
    }
    if pcurve_boxes.iter().any(|bounds| {
        !bounds.min.x.is_finite()
            || !bounds.min.y.is_finite()
            || !bounds.max.x.is_finite()
            || !bounds.max.y.is_finite()
    }) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let carrier = PersistentSkewCylinderOpenSpanCarrier {
        algebra: residual.carrier.algebra,
        parameter_map,
        bounding_box: carrier_box,
    };
    let pcurves = core::array::from_fn(|index| PersistentSkewCylinderOpenSpanPcurve {
        algebra: residual.carrier.algebra,
        operand: traces[index].pcurve().operand() as u8,
        parameter_map,
        bounding_box: pcurve_boxes[index],
    });

    let mut residual_bounds = residual.residual_bounds();
    for corridor in root_corridors {
        for (bound, corridor_bound) in residual_bounds
            .iter_mut()
            .zip(corridor.corridor().residual_bounds())
        {
            *bound = bound.max(corridor_bound);
        }
    }
    if residual_bounds
        .into_iter()
        .any(|bound| !bound.is_finite() || bound < 0.0 || bound > residual.tolerance())
    {
        return Err(IntersectionCertificateError::NonFiniteResidualBound {
            trace: PairedTrace::Second,
        });
    }
    Ok((carrier, pcurves, residual_bounds))
}

fn certify_required_edge_tolerance(
    residual: PairedSkewCylinderBranchResidualCertificate,
    carrier: PersistentSkewCylinderOpenSpanCarrier,
    pcurves: [PersistentSkewCylinderOpenSpanPcurve; 2],
    endpoint_points: [Vec3; 2],
    residual_bounds: [f64; 2],
) -> Result<f64, IntersectionCertificateError> {
    let traces = residual.traces();
    let mut required_edge_tolerance = residual_bounds.into_iter().fold(0.0, f64::max);
    for (endpoint, logical) in [LOGICAL_RANGE.lo, LOGICAL_RANGE.hi].into_iter().enumerate() {
        required_edge_tolerance = required_edge_tolerance.max(outward_distance(
            carrier.eval(logical),
            endpoint_points[endpoint],
        )?);
        for (trace, pcurve) in traces.into_iter().zip(pcurves) {
            let uv = pcurve.eval(logical);
            let lifted = trace.surface().eval([uv.x, uv.y]);
            required_edge_tolerance =
                required_edge_tolerance.max(outward_distance(lifted, endpoint_points[endpoint])?);
        }
    }
    if !required_edge_tolerance.is_finite() {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    if required_edge_tolerance > residual.tolerance() {
        return Err(unsupported(
            "skew Cylinder/Cylinder endpoint metric envelope exceeds the certified tolerance",
        ));
    }
    Ok(required_edge_tolerance)
}

/// Persistent nonperiodic logical spatial evaluator for one open span.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderOpenSpanCarrier {
    algebra: BranchAlgebra,
    parameter_map: PersistentSkewCylinderLogicalMap,
    bounding_box: Aabb3,
}

impl PersistentSkewCylinderOpenSpanCarrier {
    /// Canonical source cylinders used by this evaluator.
    pub const fn cylinders(self) -> [Cylinder; 2] {
        self.algebra.cylinders
    }

    /// Ordered quadratic sheet.
    pub const fn sheet(self) -> SkewCylinderSheet {
        self.algebra.sheet
    }
}

impl Curve for PersistentSkewCylinderOpenSpanCarrier {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn eval_derivs(&self, parameter: f64, order: usize) -> CurveDerivs {
        debug_assert!(parameter.is_nan() || LOGICAL_RANGE.contains(parameter));
        let logical = LOGICAL_RANGE.clamp_param(parameter);
        let carrier_parameter = self.parameter_map.map(logical);
        let mut result = self
            .algebra
            .authored_carrier_derivs(carrier_parameter, order.min(3));
        scale_curve_derivatives(&mut result, self.parameter_map.scale(), order.min(3));
        result
    }

    fn param_range(&self) -> ParamRange {
        LOGICAL_RANGE
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, _range: ParamRange) -> Aabb3 {
        self.bounding_box
    }
}

/// Persistent nonperiodic logical pcurve for one source cylinder.
///
/// Logical endpoints evaluate at hidden coordinates inside the sealed physical
/// root enclosures.  Those coordinates provide a continuous metric
/// representation only and are not physical-root or trim-scalar authority.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderOpenSpanPcurve {
    algebra: BranchAlgebra,
    operand: u8,
    parameter_map: PersistentSkewCylinderLogicalMap,
    bounding_box: Aabb2,
}

impl PersistentSkewCylinderOpenSpanPcurve {
    /// Canonical source operand represented by this pcurve.
    pub const fn operand(self) -> usize {
        self.operand as usize
    }

    /// Ordered quadratic sheet.
    pub const fn sheet(self) -> SkewCylinderSheet {
        self.algebra.sheet
    }
}

impl Curve2d for PersistentSkewCylinderOpenSpanPcurve {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn eval_derivs(&self, parameter: f64, order: usize) -> Curve2dDerivs {
        debug_assert!(parameter.is_nan() || LOGICAL_RANGE.contains(parameter));
        let logical = LOGICAL_RANGE.clamp_param(parameter);
        let carrier_parameter = self.parameter_map.map(logical);
        let mut result = self.algebra.authored_pcurve_derivs(
            self.operand as usize,
            carrier_parameter,
            order.min(3),
        );
        scale_curve2d_derivatives(&mut result, self.parameter_map.scale(), order.min(3));
        result
    }

    fn param_range(&self) -> ParamRange {
        LOGICAL_RANGE
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, _range: ParamRange) -> Aabb2 {
        self.bounding_box
    }

    fn source_affine_range(&self, range: ParamRange, linear: Vec2, bias: f64) -> Option<Interval> {
        if !range.is_finite()
            || range.lo < LOGICAL_RANGE.lo
            || range.hi > LOGICAL_RANGE.hi
            || !linear.x.is_finite()
            || !linear.y.is_finite()
            || !bias.is_finite()
        {
            return None;
        }
        finite_interval(
            Interval::point(bias)
                + Interval::point(linear.x)
                    * Interval::new(self.bounding_box.min.x, self.bounding_box.max.x)
                + Interval::point(linear.y)
                    * Interval::new(self.bounding_box.min.y, self.bounding_box.max.y),
        )
    }
}

/// Graph-bound verified curve descriptor for a persistent skew open span.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VerifiedSkewCylinderOpenSpanCurveDescriptor {
    source_surfaces: [SurfaceHandle; 2],
    pcurves: [Curve2dHandle; 2],
    certificate: PersistentSkewCylinderOpenSpanCertificate,
}

impl VerifiedSkewCylinderOpenSpanCurveDescriptor {
    pub(crate) const fn new(
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: PersistentSkewCylinderOpenSpanCertificate,
    ) -> Self {
        Self {
            source_surfaces,
            pcurves,
            certificate,
        }
    }

    /// Exact ordered live source identities.
    pub const fn source_surfaces(self) -> [SurfaceHandle; 2] {
        self.source_surfaces
    }

    /// Exact ordered persistent pcurve identities.
    pub const fn pcurves(self) -> [Curve2dHandle; 2] {
        self.pcurves
    }

    /// Complete normalized composite proof.
    pub const fn certificate(self) -> PersistentSkewCylinderOpenSpanCertificate {
        self.certificate
    }

    pub(crate) fn visit_dependencies(self, visit: &mut dyn FnMut(GeometryRef)) {
        for source in self.source_surfaces {
            visit(GeometryRef::Surface(source));
        }
        for pcurve in self.pcurves {
            visit(GeometryRef::Curve2d(pcurve));
        }
    }
}

impl Curve for VerifiedSkewCylinderOpenSpanCurveDescriptor {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn eval_derivs(&self, parameter: f64, order: usize) -> CurveDerivs {
        self.certificate.carrier.eval_derivs(parameter, order)
    }

    fn param_range(&self) -> ParamRange {
        LOGICAL_RANGE
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb3 {
        self.certificate.carrier.bounding_box(range)
    }
}

fn interval_midpoint(interval: Interval) -> Result<f64, IntersectionCertificateError> {
    let midpoint = 0.5 * interval.lo() + 0.5 * interval.hi();
    if midpoint.is_finite() && interval.contains(midpoint) {
        Ok(midpoint)
    } else {
        Err(IntersectionCertificateError::NonFiniteGeometry)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PersistentSkewCylinderLogicalMap {
    affine: AffineParamMap1d,
    endpoint_representatives: [f64; 2],
}

impl PersistentSkewCylinderLogicalMap {
    fn map(self, logical: f64) -> f64 {
        if logical == LOGICAL_RANGE.lo {
            self.endpoint_representatives[0]
        } else if logical == LOGICAL_RANGE.hi {
            self.endpoint_representatives[1]
        } else {
            self.affine.map(logical)
        }
    }

    const fn scale(self) -> f64 {
        self.affine.scale()
    }
}

fn pcurve_box(uv: [Interval; 2]) -> Aabb2 {
    Aabb2 {
        min: Vec2::new(uv[0].lo(), uv[1].lo()),
        max: Vec2::new(uv[0].hi(), uv[1].hi()),
    }
}

fn outward_distance(first: Vec3, second: Vec3) -> Result<f64, IntersectionCertificateError> {
    let delta = [
        Interval::point(first.x) - Interval::point(second.x),
        Interval::point(first.y) - Interval::point(second.y),
        Interval::point(first.z) - Interval::point(second.z),
    ];
    finite_interval(delta[0].square() + delta[1].square() + delta[2].square())
        .and_then(Interval::sqrt)
        .map(Interval::hi)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)
}

fn scale_curve_derivatives(result: &mut CurveDerivs, scale: f64, order: usize) {
    let mut factor = scale;
    for derivative in 1..=order {
        result.d[derivative] = result.d[derivative] * factor;
        factor *= scale;
    }
}

fn scale_curve2d_derivatives(result: &mut Curve2dDerivs, scale: f64, order: usize) {
    let mut factor = scale;
    for derivative in 1..=order {
        result.d[derivative] = result.d[derivative] * factor;
        factor *= scale;
    }
}
