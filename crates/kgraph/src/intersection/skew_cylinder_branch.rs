//! Certified operation-local carriers for a finite skew Cylinder/Cylinder sheet.
//!
//! The carrier parameter is the longitude of the first (canonical) cylinder.
//! Substitution of that cylinder ruling in the second cylinder's dual frame
//! gives two roots
//! `v = (-M(u) ± sqrt(K - L(u)^2)) / A`.  The certifier admits one root only
//! after a fixed whole-cycle interval proof establishes:
//!
//! - exact-predicate nonparallel axes and regular exact-source/evaluator dual divisors;
//! - strictly positive exact-source and stored radicands in all 256 proof cells;
//! - strict axial containment of both exact-source and stored trace enclosures;
//! - a common exact-source/evaluator seam-free opposite longitude half-plane;
//! - one lift placing both raw longitude enclosures inside the opposite window; and
//! - an analytic, outward-rounded forward-error bound for both surface traces.
//!
//! The cells are interval proof domains, not samples.  Root ordering is
//! structural (`Lower` always uses the negative square root), while interval
//! enclosures prove the numeric conditioning, windows, boxes, and chart lift.
//! These values deliberately have no persistent graph contract yet: graph
//! insertion rejects their descriptor variants before allocating a node.

use core::any::Any;

use kcore::interval::Interval;
use kcore::math;
use kcore::predicates::{Orientation, orient3d};
use kgeom::aabb::{Aabb2, Aabb3};
use kgeom::curve::{Curve, CurveDerivs};
use kgeom::curve2d::{Curve2d, Curve2dDerivs};
use kgeom::param::{ParamRange, wrap_periodic};
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Vec2, Vec3};

use crate::{AffineParamMap1d, IntersectionCertificateError, PairedTrace};

#[path = "skew_cylinder_jet.rs"]
mod jet;
use jet::Jet;

#[cfg(test)]
#[path = "skew_cylinder_branch_tests.rs"]
mod tests;

/// Fixed whole-cycle interval cells consumed by one sheet certificate.
pub const SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS: usize = 256;

/// Deterministic logical work consumed by one sheet certificate.
pub const SKEW_CYLINDER_BRANCH_CERTIFICATE_WORK: u64 = SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS as u64;

const TAU: f64 = core::f64::consts::TAU;
const EVALUATOR_ROUNDING_OPS: f64 = 4096.0;

/// Ordered root of the canonical cylinder-ruling quadratic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderSheet {
    /// `(-M - sqrt(K-L²))/A`.
    Lower,
    /// `(-M + sqrt(K-L²))/A`.
    Upper,
}

impl SkewCylinderSheet {
    const fn sign(self) -> f64 {
        match self {
            Self::Lower => -1.0,
            Self::Upper => 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Harmonic {
    constant: f64,
    cosine: f64,
    sine: f64,
}

#[derive(Debug, Clone, Copy)]
struct IntervalHarmonic {
    constant: Interval,
    cosine: Interval,
    sine: Interval,
}

impl IntervalHarmonic {
    fn interval(self, cosine: Interval, sine: Interval) -> Option<Interval> {
        finite_interval(self.constant + self.cosine * cosine + self.sine * sine)
    }
}

impl Harmonic {
    fn interval(self, cosine: Interval, sine: Interval) -> Option<Interval> {
        finite_interval(
            Interval::point(self.constant)
                + Interval::point(self.cosine) * cosine
                + Interval::point(self.sine) * sine,
        )
    }

    fn jet(self, parameter: f64) -> Jet {
        let (sine, cosine) = math::sincos(parameter);
        Jet {
            d: [
                self.constant + self.cosine * cosine + self.sine * sine,
                -self.cosine * sine + self.sine * cosine,
                -self.cosine * cosine - self.sine * sine,
                self.cosine * sine - self.sine * cosine,
            ],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct BranchAlgebra {
    cylinders: [Cylinder; 2],
    carrier_range: ParamRange,
    sheet: SkewCylinderSheet,
    e: f64,
    dx: f64,
    dy: f64,
    dz: f64,
    a: f64,
    k: f64,
    x0: Harmonic,
    y0: Harmonic,
    z0: Harmonic,
    m: Harmonic,
    l: Harmonic,
    longitude_offset: f64,
}

impl BranchAlgebra {
    fn parameter(self, parameter: f64) -> f64 {
        if parameter.is_nan() {
            self.carrier_range.lo
        } else if self.carrier_range.contains(parameter) {
            parameter
        } else {
            wrap_periodic(parameter, self.carrier_range.lo, TAU)
        }
    }

    fn bounded_parameter(self, parameter: f64) -> f64 {
        debug_assert!(
            parameter.is_nan() || self.carrier_range.contains(parameter),
            "bounded skew-cylinder pcurve parameter is outside its authored range"
        );
        self.carrier_range.clamp_param(parameter)
    }

    fn v_jet(self, parameter: f64) -> Jet {
        let m = self.m.jet(parameter);
        let l = self.l.jet(parameter);
        let radicand = Jet::constant(self.k) - l * l;
        (-m + radicand.sqrt() * self.sheet.sign()) / self.a
    }

    fn dual_jets(self, parameter: f64) -> [Jet; 3] {
        let v = self.v_jet(parameter);
        [
            self.x0.jet(parameter) + v * self.dx,
            self.y0.jet(parameter) + v * self.dy,
            self.z0.jet(parameter) + v * self.dz,
        ]
    }

    fn carrier_derivs(self, parameter: f64, order: usize) -> CurveDerivs {
        let parameter = self.parameter(parameter);
        let v = self.v_jet(parameter);
        let cylinder = self.cylinders[0];
        let frame = cylinder.frame();
        let (sine, cosine) = math::sincos(parameter);
        let radial = frame.x() * cosine + frame.y() * sine;
        let tangential = frame.y() * cosine - frame.x() * sine;
        let mut result = CurveDerivs::default();

        // Keeping order zero structurally identical to the first trace makes
        // its paired residual exactly zero in the evaluator contract.
        result.d[0] = cylinder.eval([parameter, v.d[0]]);
        if order >= 1 {
            result.d[1] = tangential * cylinder.radius() + frame.z() * v.d[1];
        }
        if order >= 2 {
            result.d[2] = -radial * cylinder.radius() + frame.z() * v.d[2];
        }
        if order >= 3 {
            result.d[3] = -tangential * cylinder.radius() + frame.z() * v.d[3];
        }
        result
    }

    fn pcurve_derivs(self, operand: usize, parameter: f64, order: usize) -> Curve2dDerivs {
        let parameter = self.bounded_parameter(parameter);
        let mut result = Curve2dDerivs::default();
        if operand == 0 {
            let v = self.v_jet(parameter);
            result.d[0] = Vec2::new(parameter, v.d[0]);
            if order >= 1 {
                result.d[1] = Vec2::new(1.0, v.d[1]);
            }
            if order >= 2 {
                result.d[2] = Vec2::new(0.0, v.d[2]);
            }
            if order >= 3 {
                result.d[3] = Vec2::new(0.0, v.d[3]);
            }
            return result;
        }

        let [x, y, z] = self.dual_jets(parameter);
        let x_normalized = x / self.e;
        let y_normalized = y / self.e;
        let height = z / self.e;
        let numerator = x * y.derivative() - y * x.derivative();
        let denominator = x * x + y * y;
        let longitude_derivative = numerator * denominator.reciprocal();
        result.d[0] = Vec2::new(
            math::atan2(y_normalized.d[0], x_normalized.d[0]) + self.longitude_offset,
            height.d[0],
        );
        if order >= 1 {
            result.d[1] = Vec2::new(longitude_derivative.d[0], height.d[1]);
        }
        if order >= 2 {
            result.d[2] = Vec2::new(longitude_derivative.d[1], height.d[2]);
        }
        if order >= 3 {
            result.d[3] = Vec2::new(longitude_derivative.d[2], height.d[3]);
        }
        result
    }
}

/// Certifier-minted procedural carrier for one complete skew-cylinder sheet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderBranchCarrier {
    algebra: BranchAlgebra,
    bounding_box: Aabb3,
}

impl SkewCylinderBranchCarrier {
    /// Ordered sheet represented by this carrier.
    pub const fn sheet(self) -> SkewCylinderSheet {
        self.algebra.sheet
    }

    /// Canonical source cylinders used by the procedural definition.
    pub const fn cylinders(self) -> [Cylinder; 2] {
        self.algebra.cylinders
    }
}

impl Curve for SkewCylinderBranchCarrier {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn eval_derivs(&self, parameter: f64, order: usize) -> CurveDerivs {
        self.algebra.carrier_derivs(parameter, order.min(3))
    }

    fn param_range(&self) -> ParamRange {
        self.algebra.carrier_range
    }

    fn periodicity(&self) -> Option<f64> {
        Some(TAU)
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb3 {
        debug_assert!(range.is_finite());
        self.bounding_box
    }
}

/// Certifier-minted pcurve of one skew-cylinder sheet on one source cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderBranchPcurve {
    algebra: BranchAlgebra,
    operand: u8,
    bounding_box: Aabb2,
}

impl SkewCylinderBranchPcurve {
    /// Canonical source operand (`0` or `1`) whose chart this pcurve uses.
    pub const fn operand(self) -> usize {
        self.operand as usize
    }

    /// Ordered sheet represented by this pcurve.
    pub const fn sheet(self) -> SkewCylinderSheet {
        self.algebra.sheet
    }
}

impl Curve2d for SkewCylinderBranchPcurve {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn eval_derivs(&self, parameter: f64, order: usize) -> Curve2dDerivs {
        self.algebra
            .pcurve_derivs(self.operand as usize, parameter, order.min(3))
    }

    fn param_range(&self) -> ParamRange {
        self.algebra.carrier_range
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb2 {
        debug_assert!(range.is_finite());
        self.bounding_box
    }

    fn source_affine_range(&self, range: ParamRange, linear: Vec2, bias: f64) -> Option<Interval> {
        if !range.is_finite()
            || range.lo < self.algebra.carrier_range.lo
            || range.hi > self.algebra.carrier_range.hi
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

/// One source cylinder and its certified sheet pcurve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderBranchTrace {
    surface: Cylinder,
    pcurve: SkewCylinderBranchPcurve,
    parameter_map: AffineParamMap1d,
}

impl SkewCylinderBranchTrace {
    /// Source cylinder represented by this trace.
    pub const fn surface(self) -> Cylinder {
        self.surface
    }

    /// Certified parameter-space curve.
    pub const fn pcurve(self) -> SkewCylinderBranchPcurve {
        self.pcurve
    }

    /// Identity carrier-to-pcurve parameter map.
    pub const fn parameter_map(self) -> AffineParamMap1d {
        self.parameter_map
    }
}

/// Whole-range paired residual proof for one finite skew-cylinder sheet.
///
/// Private fields bind the procedural carrier, source-ordered pcurves, chart
/// lift, interval boxes, and analytic evaluator error envelope.  It is an
/// operation-local proof object; persistence awaits a separate descriptor
/// contract.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PairedSkewCylinderBranchResidualCertificate {
    carrier: SkewCylinderBranchCarrier,
    carrier_range: ParamRange,
    traces: [SkewCylinderBranchTrace; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
    sheet: SkewCylinderSheet,
}

impl PairedSkewCylinderBranchResidualCertificate {
    /// Verified procedural carrier.
    pub const fn carrier(self) -> SkewCylinderBranchCarrier {
        self.carrier
    }

    /// Complete canonical longitude cycle covered by the proof.
    pub const fn carrier_range(self) -> ParamRange {
        self.carrier_range
    }

    /// Verified cylinder traces in current source order.
    pub const fn traces(self) -> [SkewCylinderBranchTrace; 2] {
        self.traces
    }

    /// Identity carrier-to-pcurve parameter maps in current source order.
    pub const fn parameter_maps(self) -> [AffineParamMap1d; 2] {
        [self.traces[0].parameter_map, self.traces[1].parameter_map]
    }

    /// Conservative whole-range model-space residual bounds.
    pub const fn residual_bounds(self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Requested model-space certification tolerance.
    pub const fn tolerance(self) -> f64 {
        self.tolerance
    }

    /// Ordered quadratic sheet represented by the certificate.
    pub const fn sheet(self) -> SkewCylinderSheet {
        self.sheet
    }

    /// Reverse only source trace provenance, retaining the canonical carrier.
    pub const fn swapped(mut self) -> Self {
        self.traces.swap(0, 1);
        self.residual_bounds.swap(0, 1);
        self
    }
}

/// Certify one complete, finite-window skew Cylinder/Cylinder sheet.
///
/// `cylinders[0]` is the canonical ruling parameterization. Both longitude
/// windows must be exactly one complete period, and both axial windows must
/// contain the certified sheet strictly. The opposite longitude is accepted
/// only when one constant `2π` lift places both the exact-source and stored
/// evaluator longitude enclosures strictly inside the authored chart window.
pub fn certify_paired_skew_cylinder_branch_residuals(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    sheet: SkewCylinderSheet,
    tolerance: f64,
) -> Result<PairedSkewCylinderBranchResidualCertificate, IntersectionCertificateError> {
    validate_inputs(cylinders, ranges, tolerance)?;
    if !axes_are_exactly_nonparallel(cylinders) {
        return Err(unsupported(
            "skew Cylinder/Cylinder branch requires exact-predicate nonparallel axes",
        ));
    }

    let mut algebra = build_algebra(cylinders, ranges[0][0], sheet)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let proof = prove_complete_sheet(algebra, ranges)?;
    algebra.longitude_offset = proof.longitude_offset;

    let second_residual = paired_residual_bound(algebra, proof).ok_or(
        IntersectionCertificateError::NonFiniteResidualBound {
            trace: PairedTrace::Second,
        },
    )?;
    if second_residual > tolerance {
        return Err(IntersectionCertificateError::ResidualExceedsTolerance {
            trace: PairedTrace::Second,
            residual_bound: second_residual,
            tolerance,
        });
    }
    let axis_norm_lower = interval_norm_lower(cylinders[0].frame().z())
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let separation = proof.sheet_separation_lower * axis_norm_lower;
    let paired_tube_width =
        finite_interval(Interval::point(2.0) * Interval::point(second_residual))
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound {
                trace: PairedTrace::Second,
            })?
            .hi();
    if separation <= paired_tube_width {
        return Err(unsupported(
            "skew Cylinder/Cylinder sheet separation is not wider than both residual tubes",
        ));
    }

    let carrier = SkewCylinderBranchCarrier {
        algebra,
        bounding_box: proof.carrier_box,
    };
    let pcurves = [
        SkewCylinderBranchPcurve {
            algebra,
            operand: 0,
            bounding_box: proof.pcurve_boxes[0],
        },
        SkewCylinderBranchPcurve {
            algebra,
            operand: 1,
            bounding_box: proof.pcurve_boxes[1],
        },
    ];
    let identity = AffineParamMap1d::new(1.0, 0.0)?;
    let traces = [
        SkewCylinderBranchTrace {
            surface: cylinders[0],
            pcurve: pcurves[0],
            parameter_map: identity,
        },
        SkewCylinderBranchTrace {
            surface: cylinders[1],
            pcurve: pcurves[1],
            parameter_map: identity,
        },
    ];
    Ok(PairedSkewCylinderBranchResidualCertificate {
        carrier,
        carrier_range: ranges[0][0],
        traces,
        residual_bounds: [0.0, second_residual],
        tolerance,
        sheet,
    })
}

#[derive(Debug, Clone, Copy)]
struct SheetProof {
    carrier_box: Aabb3,
    pcurve_boxes: [Aabb2; 2],
    longitude_offset: f64,
    radicand_lower: f64,
    sheet_separation_lower: f64,
    max_v: f64,
    max_x: f64,
    max_y: f64,
    max_z: f64,
    max_intermediate: f64,
}

fn validate_inputs(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    tolerance: f64,
) -> Result<(), IntersectionCertificateError> {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if !cylinders.into_iter().all(finite_cylinder) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    if !ranges
        .into_iter()
        .flatten()
        .all(|range| range.is_finite() && range.width() > 0.0)
    {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if ranges[0][0].width() != TAU || ranges[1][0].width() != TAU {
        return Err(unsupported(
            "skew Cylinder/Cylinder branch requires exact complete-period longitude windows",
        ));
    }
    Ok(())
}

fn finite_cylinder(cylinder: Cylinder) -> bool {
    let frame = cylinder.frame();
    [frame.origin(), frame.x(), frame.y(), frame.z()]
        .into_iter()
        .all(finite3)
        && cylinder.radius().is_finite()
        && cylinder.radius() > 0.0
}

fn finite3(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

fn axes_are_exactly_nonparallel(cylinders: [Cylinder; 2]) -> bool {
    let first = cylinders[0].frame().z().to_array();
    let second = cylinders[1].frame().z().to_array();
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .any(|basis| orient3d(first, second, basis, [0.0; 3]) != Orientation::Zero)
}

fn build_algebra(
    cylinders: [Cylinder; 2],
    carrier_range: ParamRange,
    sheet: SkewCylinderSheet,
) -> Option<BranchAlgebra> {
    let first = cylinders[0];
    let second = cylinders[1];
    let first_frame = first.frame();
    let second_frame = second.frame();
    let offset = first_frame.origin() - second_frame.origin();
    let [bx, by, bz] = [second_frame.x(), second_frame.y(), second_frame.z()];
    let e = determinant(bx, by, bz);
    let dx = determinant(first_frame.z(), by, bz);
    let dy = determinant(bx, first_frame.z(), bz);
    let dz = determinant(bx, by, first_frame.z());
    let x0 = Harmonic {
        constant: determinant(offset, by, bz),
        cosine: first.radius() * determinant(first_frame.x(), by, bz),
        sine: first.radius() * determinant(first_frame.y(), by, bz),
    };
    let y0 = Harmonic {
        constant: determinant(bx, offset, bz),
        cosine: first.radius() * determinant(bx, first_frame.x(), bz),
        sine: first.radius() * determinant(bx, first_frame.y(), bz),
    };
    let z0 = Harmonic {
        constant: determinant(bx, by, offset),
        cosine: first.radius() * determinant(bx, by, first_frame.x()),
        sine: first.radius() * determinant(bx, by, first_frame.y()),
    };
    let a = dx * dx + dy * dy;
    let m = harmonic_linear_combination(dx, x0, dy, y0);
    let l = harmonic_linear_combination(dx, y0, -dy, x0);
    let radius_determinant = second.radius() * e;
    let k = a * radius_determinant * radius_determinant;
    let values = [
        e,
        dx,
        dy,
        dz,
        a,
        k,
        x0.constant,
        x0.cosine,
        x0.sine,
        y0.constant,
        y0.cosine,
        y0.sine,
        z0.constant,
        z0.cosine,
        z0.sine,
        m.constant,
        m.cosine,
        m.sine,
        l.constant,
        l.cosine,
        l.sine,
    ];
    values
        .into_iter()
        .all(f64::is_finite)
        .then_some(BranchAlgebra {
            cylinders,
            carrier_range,
            sheet,
            e,
            dx,
            dy,
            dz,
            a,
            k,
            x0,
            y0,
            z0,
            m,
            l,
            longitude_offset: 0.0,
        })
}

fn harmonic_linear_combination(
    first_scale: f64,
    first: Harmonic,
    second_scale: f64,
    second: Harmonic,
) -> Harmonic {
    Harmonic {
        constant: first_scale * first.constant + second_scale * second.constant,
        cosine: first_scale * first.cosine + second_scale * second.cosine,
        sine: first_scale * first.sine + second_scale * second.sine,
    }
}

fn interval_harmonic_linear_combination(
    first_scale: Interval,
    first: IntervalHarmonic,
    second_scale: Interval,
    second: IntervalHarmonic,
) -> Option<IntervalHarmonic> {
    Some(IntervalHarmonic {
        constant: finite_interval(first_scale * first.constant + second_scale * second.constant)?,
        cosine: finite_interval(first_scale * first.cosine + second_scale * second.cosine)?,
        sine: finite_interval(first_scale * first.sine + second_scale * second.sine)?,
    })
}

#[derive(Debug, Clone, Copy)]
struct CellRootEnclosures {
    stored_m: Interval,
    stored_l: Interval,
    stored_radicand: Interval,
    stored_h: Interval,
    stored_v: Interval,
    exact_radicand: Interval,
    exact_h: Interval,
    exact_v: Interval,
}

fn cell_root_enclosures(
    algebra: BranchAlgebra,
    coefficients: CoefficientProof,
    cosine: Interval,
    sine: Interval,
) -> Option<CellRootEnclosures> {
    let stored_m = algebra.m.interval(cosine, sine)?;
    let stored_l = algebra.l.interval(cosine, sine)?;
    let stored_radicand = finite_interval(Interval::point(algebra.k) - stored_l.square())?;
    if stored_radicand.lo() <= 0.0 {
        return None;
    }
    let stored_h = stored_radicand.sqrt().and_then(finite_interval)?;
    let stored_v = finite_interval(
        (Interval::point(-1.0) * stored_m + Interval::point(algebra.sheet.sign()) * stored_h)
            .checked_div(Interval::point(algebra.a))?,
    )?;

    let exact_m = coefficients.m_true.interval(cosine, sine)?;
    let exact_l = coefficients.l_true.interval(cosine, sine)?;
    let exact_radicand = finite_interval(coefficients.k_true - exact_l.square())?;
    if coefficients.a_true.lo() <= 0.0 || exact_radicand.lo() <= 0.0 {
        return None;
    }
    let exact_h = exact_radicand.sqrt().and_then(finite_interval)?;
    let exact_v = finite_interval(
        (Interval::point(-1.0) * exact_m + Interval::point(algebra.sheet.sign()) * exact_h)
            .checked_div(coefficients.a_true)?,
    )?;
    Some(CellRootEnclosures {
        stored_m,
        stored_l,
        stored_radicand,
        stored_h,
        stored_v,
        exact_radicand,
        exact_h,
        exact_v,
    })
}

fn prove_complete_sheet(
    algebra: BranchAlgebra,
    ranges: [[ParamRange; 2]; 2],
) -> Result<SheetProof, IntersectionCertificateError> {
    // This proof pass intentionally remains cohesive: the same outward cell
    // enclosure must feed radicand, both windows, seam sign, boxes, and the
    // forward-error maxima without any recomputation using different bounds.
    let coefficient_proof =
        coefficient_proof(algebra).ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    if coefficient_proof.a_true.lo() <= 0.0 || coefficient_proof.e_true.contains_zero() {
        return Err(unsupported(
            "skew Cylinder/Cylinder dual-frame divisors lack a strict numeric margin",
        ));
    }
    if algebra.a <= 0.0 || algebra.e == 0.0 {
        return Err(unsupported(
            "skew Cylinder/Cylinder evaluator divisors are singular",
        ));
    }

    let mut carrier_min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut carrier_max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    let mut longitude_min = f64::INFINITY;
    let mut longitude_max = f64::NEG_INFINITY;
    let mut exact_longitude_min = f64::INFINITY;
    let mut exact_longitude_max = f64::NEG_INFINITY;
    let mut height_min = f64::INFINITY;
    let mut height_max = f64::NEG_INFINITY;
    let mut radicand_lower = f64::INFINITY;
    let mut separation_lower = f64::INFINITY;
    let mut max_x: f64 = 0.0;
    let mut max_y: f64 = 0.0;
    let mut max_z: f64 = 0.0;
    let mut max_intermediate: f64 = 1.0;
    let mut y_positive = true;
    let mut y_negative = true;
    let mut x_positive = true;
    let mut exact_y_positive = true;
    let mut exact_y_negative = true;
    let mut exact_x_positive = true;
    let width = algebra.carrier_range.width();

    for index in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        let lo = algebra.carrier_range.lo
            + width * index as f64 / SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS as f64;
        let hi = if index + 1 == SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
            algebra.carrier_range.hi
        } else {
            algebra.carrier_range.lo
                + width * (index + 1) as f64 / SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS as f64
        };
        let cosine = trig_interval(lo, hi, false);
        let sine = trig_interval(lo, hi, true);
        let roots = cell_root_enclosures(algebra, coefficient_proof, cosine, sine).ok_or_else(
            || {
                unsupported(
                    "skew Cylinder/Cylinder source/evaluator radicand lacks a strict whole-cycle numeric margin",
                )
            },
        )?;
        let v = roots.stored_v;
        if v.lo() <= ranges[0][1].lo || v.hi() >= ranges[0][1].hi {
            return Err(unsupported(
                "skew Cylinder/Cylinder canonical axial trace escapes its strict finite window",
            ));
        }
        if roots.exact_v.lo() <= ranges[0][1].lo || roots.exact_v.hi() >= ranges[0][1].hi {
            return Err(unsupported(
                "skew Cylinder/Cylinder exact-source canonical axial trace escapes its strict finite window",
            ));
        }

        let x = finite_interval(
            algebra
                .x0
                .interval(cosine, sine)
                .ok_or(IntersectionCertificateError::NonFiniteGeometry)?
                + Interval::point(algebra.dx) * v,
        )
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let y = finite_interval(
            algebra
                .y0
                .interval(cosine, sine)
                .ok_or(IntersectionCertificateError::NonFiniteGeometry)?
                + Interval::point(algebra.dy) * v,
        )
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let z = finite_interval(
            algebra
                .z0
                .interval(cosine, sine)
                .ok_or(IntersectionCertificateError::NonFiniteGeometry)?
                + Interval::point(algebra.dz) * v,
        )
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let normalized_x = x
            .checked_div(Interval::point(algebra.e))
            .and_then(finite_interval)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let normalized_y = y
            .checked_div(Interval::point(algebra.e))
            .and_then(finite_interval)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let height = z
            .checked_div(Interval::point(algebra.e))
            .and_then(finite_interval)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        if height.lo() <= ranges[1][1].lo || height.hi() >= ranges[1][1].hi {
            return Err(unsupported(
                "skew Cylinder/Cylinder opposite axial trace escapes its strict finite window",
            ));
        }
        let exact_coordinates = [0, 1, 2].map(|coordinate| {
            coefficient_proof.harmonics_true[coordinate]
                .interval(cosine, sine)
                .and_then(|harmonic| {
                    finite_interval(
                        harmonic + coefficient_proof.directions_true[coordinate] * roots.exact_v,
                    )
                })
        });
        let [Some(exact_x), Some(exact_y), Some(exact_z)] = exact_coordinates else {
            return Err(IntersectionCertificateError::NonFiniteGeometry);
        };
        let exact_normalized_x = exact_x
            .checked_div(coefficient_proof.e_true)
            .and_then(finite_interval)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let exact_normalized_y = exact_y
            .checked_div(coefficient_proof.e_true)
            .and_then(finite_interval)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let exact_height = exact_z
            .checked_div(coefficient_proof.e_true)
            .and_then(finite_interval)
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        if exact_height.lo() <= ranges[1][1].lo || exact_height.hi() >= ranges[1][1].hi {
            return Err(unsupported(
                "skew Cylinder/Cylinder exact-source opposite axial trace escapes its strict finite window",
            ));
        }

        y_positive &= normalized_y.lo() > 0.0;
        y_negative &= normalized_y.hi() < 0.0;
        x_positive &= normalized_x.lo() > 0.0;
        exact_y_positive &= exact_normalized_y.lo() > 0.0;
        exact_y_negative &= exact_normalized_y.hi() < 0.0;
        exact_x_positive &= exact_normalized_x.lo() > 0.0;
        let longitude = longitude_interval(normalized_x, normalized_y);
        let exact_longitude = longitude_interval(exact_normalized_x, exact_normalized_y);
        longitude_min = longitude_min.min(longitude.lo());
        longitude_max = longitude_max.max(longitude.hi());
        exact_longitude_min = exact_longitude_min.min(exact_longitude.lo());
        exact_longitude_max = exact_longitude_max.max(exact_longitude.hi());
        height_min = height_min.min(height.lo());
        height_max = height_max.max(height.hi());
        v_min = v_min.min(v.lo());
        v_max = v_max.max(v.hi());
        radicand_lower = radicand_lower
            .min(roots.stored_radicand.lo())
            .min(roots.exact_radicand.lo());
        let stored_separation = finite_interval(
            (Interval::point(2.0) * roots.stored_h)
                .checked_div(Interval::point(algebra.a))
                .ok_or(IntersectionCertificateError::NonFiniteGeometry)?,
        )
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        let exact_separation = finite_interval(
            (Interval::point(2.0) * roots.exact_h)
                .checked_div(coefficient_proof.a_true)
                .ok_or(IntersectionCertificateError::NonFiniteGeometry)?,
        )
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
        separation_lower = separation_lower
            .min(stored_separation.lo())
            .min(exact_separation.lo());
        max_x = max_x.max(max_abs(x));
        max_y = max_y.max(max_abs(y));
        max_z = max_z.max(max_abs(z));
        max_intermediate = max_intermediate
            .max(max_abs(roots.stored_m))
            .max(max_abs(roots.stored_l))
            .max(max_abs(roots.stored_h))
            .max(max_abs(roots.exact_h))
            .max(max_abs(v))
            .max(max_abs(roots.exact_v))
            .max(max_abs(x))
            .max(max_abs(y))
            .max(max_abs(z));

        let first = algebra.cylinders[0];
        let frame = first.frame();
        for axis in 0..3 {
            let coordinate = finite_interval(
                Interval::point(frame.origin().to_array()[axis])
                    + Interval::point(first.radius() * frame.x().to_array()[axis]) * cosine
                    + Interval::point(first.radius() * frame.y().to_array()[axis]) * sine
                    + Interval::point(frame.z().to_array()[axis]) * v,
            )
            .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
            match axis {
                0 => {
                    carrier_min.x = carrier_min.x.min(coordinate.lo());
                    carrier_max.x = carrier_max.x.max(coordinate.hi());
                }
                1 => {
                    carrier_min.y = carrier_min.y.min(coordinate.lo());
                    carrier_max.y = carrier_max.y.max(coordinate.hi());
                }
                _ => {
                    carrier_min.z = carrier_min.z.min(coordinate.lo());
                    carrier_max.z = carrier_max.z.max(coordinate.hi());
                }
            }
        }
    }

    if !((y_positive && exact_y_positive)
        || (y_negative && exact_y_negative)
        || (x_positive && exact_x_positive))
    {
        return Err(unsupported(
            "skew Cylinder/Cylinder source/evaluator longitude has no common proven constant seam lift",
        ));
    }
    let longitude_offset = choose_longitude_offset(
        longitude_min.min(exact_longitude_min),
        longitude_max.max(exact_longitude_max),
        ranges[1][0],
    )
    .ok_or_else(|| {
        unsupported(
            "skew Cylinder/Cylinder source/evaluator longitude escapes its common strict chart window",
        )
    })?;
    Ok(SheetProof {
        carrier_box: Aabb3 {
            min: carrier_min,
            max: carrier_max,
        },
        pcurve_boxes: [
            Aabb2 {
                min: Vec2::new(algebra.carrier_range.lo, v_min),
                max: Vec2::new(algebra.carrier_range.hi, v_max),
            },
            Aabb2 {
                min: Vec2::new(longitude_min + longitude_offset, height_min),
                max: Vec2::new(longitude_max + longitude_offset, height_max),
            },
        ],
        longitude_offset,
        radicand_lower,
        sheet_separation_lower: separation_lower.max(0.0),
        max_v: v_min.abs().max(v_max.abs()),
        max_x,
        max_y,
        max_z,
        max_intermediate,
    })
}

#[derive(Debug, Clone, Copy)]
struct CoefficientProof {
    e_true: Interval,
    a_true: Interval,
    a_stored_exact: Interval,
    directions_true: [Interval; 3],
    harmonics_true: [IntervalHarmonic; 3],
    m_true: IntervalHarmonic,
    l_true: IntervalHarmonic,
    k_true: Interval,
    e_error: f64,
    direction_errors: [f64; 3],
    harmonic_errors: [[f64; 3]; 3],
    algebra_errors: [f64; 4],
}

fn coefficient_proof(algebra: BranchAlgebra) -> Option<CoefficientProof> {
    let first = algebra.cylinders[0];
    let second = algebra.cylinders[1];
    let first_frame = first.frame();
    let second_frame = second.frame();
    let [bx, by, bz] = [second_frame.x(), second_frame.y(), second_frame.z()];
    let offset = interval_vec(first_frame.origin()) - interval_vec(second_frame.origin());
    let e_true = interval_determinant(interval_vec(bx), interval_vec(by), interval_vec(bz))?;
    let directions = [
        interval_determinant(
            interval_vec(first_frame.z()),
            interval_vec(by),
            interval_vec(bz),
        )?,
        interval_determinant(
            interval_vec(bx),
            interval_vec(first_frame.z()),
            interval_vec(bz),
        )?,
        interval_determinant(
            interval_vec(bx),
            interval_vec(by),
            interval_vec(first_frame.z()),
        )?,
    ];
    let true_harmonics = [
        [
            interval_determinant(offset, interval_vec(by), interval_vec(bz))?,
            finite_interval(
                Interval::point(first.radius())
                    * interval_determinant(
                        interval_vec(first_frame.x()),
                        interval_vec(by),
                        interval_vec(bz),
                    )?,
            )?,
            finite_interval(
                Interval::point(first.radius())
                    * interval_determinant(
                        interval_vec(first_frame.y()),
                        interval_vec(by),
                        interval_vec(bz),
                    )?,
            )?,
        ],
        [
            interval_determinant(interval_vec(bx), offset, interval_vec(bz))?,
            finite_interval(
                Interval::point(first.radius())
                    * interval_determinant(
                        interval_vec(bx),
                        interval_vec(first_frame.x()),
                        interval_vec(bz),
                    )?,
            )?,
            finite_interval(
                Interval::point(first.radius())
                    * interval_determinant(
                        interval_vec(bx),
                        interval_vec(first_frame.y()),
                        interval_vec(bz),
                    )?,
            )?,
        ],
        [
            interval_determinant(interval_vec(bx), interval_vec(by), offset)?,
            finite_interval(
                Interval::point(first.radius())
                    * interval_determinant(
                        interval_vec(bx),
                        interval_vec(by),
                        interval_vec(first_frame.x()),
                    )?,
            )?,
            finite_interval(
                Interval::point(first.radius())
                    * interval_determinant(
                        interval_vec(bx),
                        interval_vec(by),
                        interval_vec(first_frame.y()),
                    )?,
            )?,
        ],
    ];
    let stored_harmonics = [
        [algebra.x0.constant, algebra.x0.cosine, algebra.x0.sine],
        [algebra.y0.constant, algebra.y0.cosine, algebra.y0.sine],
        [algebra.z0.constant, algebra.z0.cosine, algebra.z0.sine],
    ];
    let mut harmonic_errors = [[0.0; 3]; 3];
    for coordinate in 0..3 {
        for coefficient in 0..3 {
            harmonic_errors[coordinate][coefficient] = deviation(
                true_harmonics[coordinate][coefficient],
                stored_harmonics[coordinate][coefficient],
            );
        }
    }
    let direction_errors = [
        deviation(directions[0], algebra.dx),
        deviation(directions[1], algebra.dy),
        deviation(directions[2], algebra.dz),
    ];
    let a_stored_exact = finite_interval(
        Interval::point(algebra.dx).square() + Interval::point(algebra.dy).square(),
    )?;
    let a_true = finite_interval(directions[0].square() + directions[1].square())?;
    let harmonics_true = true_harmonics.map(|coefficients| IntervalHarmonic {
        constant: coefficients[0],
        cosine: coefficients[1],
        sine: coefficients[2],
    });
    let m_true = interval_harmonic_linear_combination(
        directions[0],
        harmonics_true[0],
        directions[1],
        harmonics_true[1],
    )?;
    let l_true = interval_harmonic_linear_combination(
        directions[0],
        harmonics_true[1],
        -directions[1],
        harmonics_true[0],
    )?;
    let k_true = finite_interval(a_true * (Interval::point(second.radius()) * e_true).square())?;
    let m_exact = stored_identity_harmonic(algebra.dx, algebra.x0, algebra.dy, algebra.y0)?;
    let l_exact = stored_identity_harmonic(algebra.dx, algebra.y0, -algebra.dy, algebra.x0)?;
    let m_error = harmonic_deviation(m_exact, algebra.m)?;
    let l_error = harmonic_deviation(l_exact, algebra.l)?;
    let k_exact = finite_interval(
        a_stored_exact * (Interval::point(second.radius()) * Interval::point(algebra.e)).square(),
    )?;
    Some(CoefficientProof {
        e_true,
        a_true,
        a_stored_exact,
        directions_true: directions,
        harmonics_true,
        m_true,
        l_true,
        k_true,
        e_error: deviation(e_true, algebra.e),
        direction_errors,
        harmonic_errors,
        algebra_errors: [
            deviation(a_stored_exact, algebra.a),
            m_error,
            l_error,
            deviation(k_exact, algebra.k),
        ],
    })
}

fn paired_residual_bound(algebra: BranchAlgebra, proof: SheetProof) -> Option<f64> {
    let coefficients = coefficient_proof(algebra)?;
    let a_exact_lower = coefficients.a_stored_exact.lo();
    let conditioning_a_lower = a_exact_lower.min(coefficients.a_true.lo());
    let e_true_lower = strict_abs_lower(coefficients.e_true)?;
    let e_stored = algebra.e.abs();
    if conditioning_a_lower <= 0.0 || e_stored == 0.0 {
        return None;
    }

    // The stored root satisfies `(a v + M)^2 + L^2 = k`.  Compare that
    // identity with the exact products of the stored coefficients; this
    // avoids interval dependency across a complete trigonometric cell.
    let [a_error, m_error, l_error, k_error] = coefficients.algebra_errors;
    let delta_s = outward_sum(outward_product(a_error, proof.max_v)?, m_error)?;
    let s_max = algebra.k.max(0.0).sqrt().next_up();
    let l_max = outward_sum_many(&[
        algebra.l.constant.abs(),
        algebra.l.cosine.abs(),
        algebra.l.sine.abs(),
    ])?;
    let numerator_error = outward_sum_many(&[
        outward_product(2.0, outward_product(s_max, delta_s)?)?,
        outward_product(delta_s, delta_s)?,
        outward_product(2.0, outward_product(l_max, l_error)?)?,
        outward_product(l_error, l_error)?,
        k_error,
    ])?;
    let stored_radial_residual = outward_quotient(numerator_error, a_exact_lower)?;
    let radial_denominator = finite_interval(
        Interval::point(e_stored).square() * Interval::point(algebra.cylinders[1].radius()),
    )?;
    let radial_normalization =
        finite_interval(Interval::point(stored_radial_residual).checked_div(radial_denominator)?)?
            .hi();

    let coordinate_errors = [0, 1, 2].map(|coordinate| {
        let harmonic_error = outward_sum_many(&coefficients.harmonic_errors[coordinate])?;
        outward_sum(
            harmonic_error,
            outward_product(coefficients.direction_errors[coordinate], proof.max_v)?,
        )
    });
    let [Some(x_error), Some(y_error), Some(z_error)] = coordinate_errors else {
        return None;
    };
    let normalized_error = |coordinate_error: f64, stored_max: f64| {
        let first = outward_quotient(coordinate_error, e_true_lower)?;
        let second_numerator = outward_product(stored_max, coefficients.e_error)?;
        let second_denominator =
            finite_interval(Interval::point(e_true_lower) * Interval::point(e_stored))?.lo();
        outward_sum(
            first,
            outward_quotient(second_numerator, second_denominator)?,
        )
    };
    let normalized_errors = [
        normalized_error(x_error, proof.max_x)?,
        normalized_error(y_error, proof.max_y)?,
        normalized_error(z_error, proof.max_z)?,
    ];
    let second_frame = algebra.cylinders[1].frame();
    let basis_norms = [
        interval_norm_upper(second_frame.x())?,
        interval_norm_upper(second_frame.y())?,
        interval_norm_upper(second_frame.z())?,
    ];
    let coefficient_residual = outward_sum_many(&[
        outward_product(basis_norms[0], normalized_errors[0])?,
        outward_product(basis_norms[1], normalized_errors[1])?,
        outward_product(basis_norms[2], normalized_errors[2])?,
        outward_product(
            outward_sum(basis_norms[0], basis_norms[1])?,
            radial_normalization,
        )?,
    ])?;

    // Fixed evaluator expressions have far fewer than this many rounded
    // elementary operations.  `kcore::math` trigonometric functions are
    // below one ulp; charging each as one scale unit keeps this standard
    // gamma-N forward bound conservative. Conditioning by A, E, and the
    // square-root margin makes near-degenerate cases fail closed.
    let epsilon_work = EVALUATOR_ROUNDING_OPS * f64::EPSILON;
    if epsilon_work >= 1.0 {
        return None;
    }
    let gamma = outward_quotient(epsilon_work, (1.0 - epsilon_work).next_down())?;
    let radicand_root_lower = proof.radicand_lower.sqrt().next_down();
    let conditioning = 1.0_f64
        .max(outward_quotient(1.0, conditioning_a_lower)?)
        .max(outward_quotient(1.0, e_true_lower)?)
        .max(outward_quotient(1.0, radicand_root_lower)?);
    let model_scale = proof
        .max_intermediate
        .max(max_cylinder_scale(algebra.cylinders[0]))
        .max(max_cylinder_scale(algebra.cylinders[1]))
        .max(basis_norms.into_iter().fold(1.0, f64::max));
    let evaluator_roundoff = outward_product(outward_product(gamma, conditioning)?, model_scale)?;
    outward_sum(coefficient_residual, evaluator_roundoff).map(f64::next_up)
}

fn max_cylinder_scale(cylinder: Cylinder) -> f64 {
    let frame = cylinder.frame();
    [
        frame.origin().x.abs(),
        frame.origin().y.abs(),
        frame.origin().z.abs(),
        frame.x().x.abs(),
        frame.x().y.abs(),
        frame.x().z.abs(),
        frame.y().x.abs(),
        frame.y().y.abs(),
        frame.y().z.abs(),
        frame.z().x.abs(),
        frame.z().y.abs(),
        frame.z().z.abs(),
        cylinder.radius().abs(),
        1.0,
    ]
    .into_iter()
    .fold(1.0, f64::max)
}

fn stored_identity_harmonic(
    first_scale: f64,
    first: Harmonic,
    second_scale: f64,
    second: Harmonic,
) -> Option<[Interval; 3]> {
    let first = [first.constant, first.cosine, first.sine];
    let second = [second.constant, second.cosine, second.sine];
    let mut result = [Interval::point(0.0); 3];
    for index in 0..3 {
        result[index] = finite_interval(
            Interval::point(first_scale) * Interval::point(first[index])
                + Interval::point(second_scale) * Interval::point(second[index]),
        )?;
    }
    Some(result)
}

fn harmonic_deviation(exact: [Interval; 3], stored: Harmonic) -> Option<f64> {
    let stored = [stored.constant, stored.cosine, stored.sine];
    let deviations: [f64; 3] = core::array::from_fn(|index| deviation(exact[index], stored[index]));
    outward_sum_many(&deviations)
}

fn choose_longitude_offset(minimum: f64, maximum: f64, window: ParamRange) -> Option<f64> {
    let raw_center = 0.5 * minimum + 0.5 * maximum;
    let window_center = 0.5 * window.lo + 0.5 * window.hi;
    let base_turn = ((window_center - raw_center) / TAU).round();
    for delta in [0.0, -1.0, 1.0] {
        let offset = (base_turn + delta) * TAU;
        if minimum + offset > window.lo && maximum + offset < window.hi {
            return Some(offset);
        }
    }
    None
}

fn longitude_interval(x: Interval, y: Interval) -> Interval {
    let mut minimum = f64::INFINITY;
    let mut maximum = f64::NEG_INFINITY;
    for x_value in [x.lo(), x.hi()] {
        for y_value in [y.lo(), y.hi()] {
            let value = math::atan2(y_value, x_value);
            minimum = minimum.min(value);
            maximum = maximum.max(value);
        }
    }
    Interval::new(minimum.next_down(), maximum.next_up())
}

fn trig_interval(lo: f64, hi: f64, sine: bool) -> Interval {
    let evaluate = if sine { math::sin } else { math::cos };
    let midpoint = 0.5 * lo + 0.5 * hi;
    let value = evaluate(midpoint);
    // Sine and cosine are 1-Lipschitz. A full outward cell width (rather
    // than the half width) absorbs midpoint formation and endpoint rounding
    // without relying on rounded representations of π critical points.
    let radius = (hi - lo).abs().next_up();
    let enclosure =
        Interval::new(value.next_down(), value.next_up()) + Interval::new(-radius, radius);
    Interval::new(enclosure.lo().max(-1.0), enclosure.hi().min(1.0))
}

fn determinant(first: Vec3, second: Vec3, third: Vec3) -> f64 {
    first.x * (second.y * third.z - second.z * third.y)
        - first.y * (second.x * third.z - second.z * third.x)
        + first.z * (second.x * third.y - second.y * third.x)
}

#[derive(Debug, Clone, Copy)]
struct IntervalVec3([Interval; 3]);

impl core::ops::Sub for IntervalVec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self([
            self.0[0] - rhs.0[0],
            self.0[1] - rhs.0[1],
            self.0[2] - rhs.0[2],
        ])
    }
}

fn interval_vec(value: Vec3) -> IntervalVec3 {
    IntervalVec3(value.to_array().map(Interval::point))
}

fn interval_determinant(
    first: IntervalVec3,
    second: IntervalVec3,
    third: IntervalVec3,
) -> Option<Interval> {
    let [a, b, c] = [first.0, second.0, third.0];
    finite_interval(
        a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0]),
    )
}

fn deviation(interval: Interval, value: f64) -> f64 {
    (value - interval.lo())
        .abs()
        .max((interval.hi() - value).abs())
        .next_up()
}

fn strict_abs_lower(interval: Interval) -> Option<f64> {
    if interval.lo() > 0.0 {
        Some(interval.lo())
    } else if interval.hi() < 0.0 {
        Some((-interval.hi()).max(0.0))
    } else {
        None
    }
}

fn max_abs(interval: Interval) -> f64 {
    interval.lo().abs().max(interval.hi().abs())
}

fn interval_norm_lower(vector: Vec3) -> Option<f64> {
    finite_interval(
        Interval::point(vector.x).square()
            + Interval::point(vector.y).square()
            + Interval::point(vector.z).square(),
    )?
    .sqrt()
    .map(Interval::lo)
}

fn interval_norm_upper(vector: Vec3) -> Option<f64> {
    finite_interval(
        Interval::point(vector.x).square()
            + Interval::point(vector.y).square()
            + Interval::point(vector.z).square(),
    )?
    .sqrt()
    .map(Interval::hi)
}

fn outward_sum(first: f64, second: f64) -> Option<f64> {
    let value = finite_interval(Interval::point(first) + Interval::point(second))?.hi();
    value.is_finite().then_some(value)
}

fn outward_product(first: f64, second: f64) -> Option<f64> {
    let value = finite_interval(Interval::point(first) * Interval::point(second))?.hi();
    value.is_finite().then_some(value)
}

fn outward_quotient(numerator: f64, denominator_lower: f64) -> Option<f64> {
    if denominator_lower <= 0.0 {
        return None;
    }
    finite_interval(Interval::point(numerator).checked_div(Interval::point(denominator_lower))?)
        .map(Interval::hi)
}

fn outward_sum_many(values: &[f64]) -> Option<f64> {
    values.iter().copied().try_fold(0.0, outward_sum)
}

fn finite_interval(interval: Interval) -> Option<Interval> {
    (interval.lo().is_finite() && interval.hi().is_finite()).then_some(interval)
}

fn unsupported(reason: &'static str) -> IntersectionCertificateError {
    IntersectionCertificateError::UnsupportedCarrierParameterization { reason }
}
