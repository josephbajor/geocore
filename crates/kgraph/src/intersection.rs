//! Verified persistent intersection-curve building blocks.
//!
//! The first descriptor families retain finite line and circle carriers with
//! ordered source/pcurve identity and whole-interval paired trace proofs.
//! Plane/plane lines use affine interval residuals. Axis-aligned or
//! axis-antialigned plane/sphere circles retain a constant-latitude sphere
//! trace. Other finite plane/sphere secants use a certifier-minted nonlinear
//! inverse spherical chart with continuous seam unwrapping and whole-branch
//! pole/window proof.

use core::fmt;

use kcore::interval::Interval;
use kcore::math;
use kgeom::aabb::{Aabb2, Aabb3};
use kgeom::curve::{Circle, Curve, CurveDerivs, Line};
use kgeom::curve2d::{Circle2d, Curve2d, Curve2dDerivs, Line2d, NurbsCurve2d};
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::{ParamRange, wrap_periodic};
use kgeom::surface::{Dir, Plane, Sphere, Surface};
use kgeom::vec::{Vec2, Vec3};

use crate::{Curve2dHandle, GeometryRef, SurfaceHandle};

/// Fixed interval subdivisions consumed by one nonlinear spherical-circle
/// whole-branch chart proof.
pub const SPHERICAL_CIRCLE_PROOF_SEGMENTS: usize = 128;

/// An invertible affine correspondence from a carrier parameter `t` to a
/// dependent curve parameter `s`: `s = scale * t + offset`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AffineParamMap1d {
    scale: f64,
    offset: f64,
}

impl AffineParamMap1d {
    /// Constructs a finite, invertible affine parameter map.
    pub fn new(scale: f64, offset: f64) -> Result<Self, IntersectionCertificateError> {
        if !scale.is_finite() || !offset.is_finite() {
            return Err(IntersectionCertificateError::InvalidParameterMap {
                reason: "affine parameter-map coefficients must be finite",
            });
        }
        if scale == 0.0 {
            return Err(IntersectionCertificateError::InvalidParameterMap {
                reason: "affine parameter-map scale must be nonzero",
            });
        }
        Ok(Self { scale, offset })
    }

    /// Multiplicative coefficient in `s = scale * t + offset`.
    pub const fn scale(self) -> f64 {
        self.scale
    }

    /// Additive coefficient in `s = scale * t + offset`.
    pub const fn offset(self) -> f64 {
        self.offset
    }

    /// Maps a carrier parameter to the dependent parameter.
    pub fn map(self, carrier_parameter: f64) -> f64 {
        self.scale * carrier_parameter + self.offset
    }

    /// Maps a dependent parameter back to the carrier parameter.
    pub fn inverse(self, dependent_parameter: f64) -> f64 {
        (dependent_parameter - self.offset) / self.scale
    }

    /// Maps an ordered carrier interval, preserving ordered output bounds
    /// even when the parameter correspondence reverses orientation.
    pub fn map_range(self, carrier_range: ParamRange) -> ParamRange {
        let first = self.map(carrier_range.lo);
        let second = self.map(carrier_range.hi);
        ParamRange {
            lo: first.min(second),
            hi: first.max(second),
        }
    }
}

/// Which plane/pcurve trace failed paired residual certification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PairedTrace {
    /// First surface trace.
    First,
    /// Second surface trace.
    Second,
}

/// Failure to construct an affine parameter map or certify paired traces.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum IntersectionCertificateError {
    /// An affine map is non-finite or non-invertible.
    InvalidParameterMap {
        /// Stable validation reason.
        reason: &'static str,
    },
    /// A nonlinear paired certificate did not contain exactly one trace of
    /// each required surface family.
    InvalidTraceFamily,
    /// A nonlinear trace uses a parameterization outside the exact certified
    /// subfamily.
    UnsupportedTraceParameterization {
        /// Trace whose parameterization is unsupported.
        trace: PairedTrace,
        /// Stable proof-boundary explanation.
        reason: &'static str,
    },
    /// A retained carrier uses a representation outside the certifier's exact
    /// whole-range proof family.
    UnsupportedCarrierParameterization {
        /// Stable proof-boundary explanation.
        reason: &'static str,
    },
    /// The inverse spherical chart cannot be regular over the complete
    /// carrier interval because pole clearance was not proved positive.
    SingularSphereChart {
        /// Conservative lower bound for squared distance from the sphere axis.
        squared_pole_clearance: f64,
    },
    /// A certified inverse spherical trace could not be enclosed by the
    /// requested finite chart window.
    SphereTraceOutsideWindow {
        /// Parameter coordinate whose enclosure crossed its window.
        coordinate: &'static str,
    },
    /// The carrier interval is non-finite or reversed.
    InvalidCarrierRange,
    /// The requested model-space tolerance is non-finite or negative.
    InvalidTolerance,
    /// Input geometry contains a non-finite coordinate.
    NonFiniteGeometry,
    /// A finite input overflowed during conservative interval evaluation.
    NonFiniteResidualBound {
        /// Trace whose interval evaluation overflowed.
        trace: PairedTrace,
    },
    /// An offset-NURBS trace did not prove a regular whole-rectangle normal.
    SingularOffsetNormal {
        /// Trace whose basis normal could not be normalized.
        trace: PairedTrace,
        /// Outward lower bound for the squared basis-normal magnitude.
        squared_norm_lower_bound: f64,
    },
    /// A whole-interval trace residual is above the supplied tolerance.
    ResidualExceedsTolerance {
        /// Trace that failed certification.
        trace: PairedTrace,
        /// Conservative Euclidean residual upper bound.
        residual_bound: f64,
        /// Requested model-space tolerance.
        tolerance: f64,
    },
}

impl fmt::Display for IntersectionCertificateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameterMap { reason } => write!(f, "invalid parameter map: {reason}"),
            Self::InvalidTraceFamily => {
                f.write_str("paired nonlinear certificate requires one plane and one sphere trace")
            }
            Self::UnsupportedTraceParameterization { trace, reason } => {
                write!(f, "unsupported {trace:?} trace parameterization: {reason}")
            }
            Self::UnsupportedCarrierParameterization { reason } => {
                write!(f, "unsupported carrier parameterization: {reason}")
            }
            Self::SingularSphereChart {
                squared_pole_clearance,
            } => write!(
                f,
                "inverse sphere chart lacks positive whole-branch pole clearance (squared lower bound {squared_pole_clearance})"
            ),
            Self::SphereTraceOutsideWindow { coordinate } => write!(
                f,
                "inverse sphere trace escapes its requested {coordinate} window"
            ),
            Self::InvalidCarrierRange => f.write_str("carrier range must be finite and ordered"),
            Self::InvalidTolerance => {
                f.write_str("residual tolerance must be finite and nonnegative")
            }
            Self::NonFiniteGeometry => f.write_str("certified geometry must be finite"),
            Self::NonFiniteResidualBound { trace } => {
                write!(f, "{trace:?} trace produced a non-finite residual bound")
            }
            Self::SingularOffsetNormal {
                trace,
                squared_norm_lower_bound,
            } => write!(
                f,
                "{trace:?} offset-NURBS trace lacks a positive whole-range normal bound (squared lower bound {squared_norm_lower_bound})"
            ),
            Self::ResidualExceedsTolerance {
                trace,
                residual_bound,
                tolerance,
            } => write!(
                f,
                "{trace:?} trace residual bound {residual_bound} exceeds tolerance {tolerance}"
            ),
        }
    }
}

impl std::error::Error for IntersectionCertificateError {}

/// Proof that a finite line carrier and two lifted line pcurves agree over a
/// complete carrier interval within a supplied model-space tolerance.
///
/// Fields are private so certificates can only be minted by
/// [`certify_paired_plane_line_residuals`]. The verified geometry is retained
/// in the certificate, preventing the proof bounds from being reassociated
/// with different carriers or traces.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PairedPlaneLineResidualCertificate {
    carrier: Line,
    carrier_range: ParamRange,
    surfaces: [Plane; 2],
    pcurves: [Line2d; 2],
    parameter_maps: [AffineParamMap1d; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
}

/// Exact plane trace used by an axis-aligned plane/sphere circle proof.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneCircleTrace {
    surface: Plane,
    pcurve: Circle2d,
    parameter_map: AffineParamMap1d,
}

impl PlaneCircleTrace {
    /// Construct an ordered plane-circle trace candidate.
    pub const fn new(surface: Plane, pcurve: Circle2d, parameter_map: AffineParamMap1d) -> Self {
        Self {
            surface,
            pcurve,
            parameter_map,
        }
    }

    /// Exact plane field.
    pub const fn surface(self) -> Plane {
        self.surface
    }

    /// Circular plane pcurve.
    pub const fn pcurve(self) -> Circle2d {
        self.pcurve
    }

    /// Carrier-to-pcurve parameter map.
    pub const fn parameter_map(self) -> AffineParamMap1d {
        self.parameter_map
    }
}

/// Exact constant-latitude sphere trace used by an axis-aligned circle proof.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphereLatitudeTrace {
    surface: Sphere,
    pcurve: Line2d,
    parameter_map: AffineParamMap1d,
}

impl SphereLatitudeTrace {
    /// Construct an ordered sphere-latitude trace candidate.
    pub const fn new(surface: Sphere, pcurve: Line2d, parameter_map: AffineParamMap1d) -> Self {
        Self {
            surface,
            pcurve,
            parameter_map,
        }
    }

    /// Exact sphere field.
    pub const fn surface(self) -> Sphere {
        self.surface
    }

    /// Constant-latitude pcurve.
    pub const fn pcurve(self) -> Line2d {
        self.pcurve
    }

    /// Carrier-to-pcurve parameter map.
    pub const fn parameter_map(self) -> AffineParamMap1d {
        self.parameter_map
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct SphericalLongitudeSeam {
    parameter: f64,
    turn_delta: f64,
    longitude: f64,
}

/// Finite inverse sphere-chart pcurve for a certified spatial circle branch.
///
/// Fields are private and instances are minted only by
/// [`certify_paired_plane_sphere_oblique_circle_residuals`]. Longitude is
/// continuously unwrapped through at most two retained canonical seam
/// crossings. The represented branch is deliberately nonperiodic and cannot
/// cross a sphere pole.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphericalCirclePcurve {
    carrier: Circle,
    sphere: Sphere,
    carrier_range: ParamRange,
    chart_window: [ParamRange; 2],
    initial_turn: f64,
    seams: [SphericalLongitudeSeam; 2],
    seam_count: u8,
    endpoint_longitudes: [f64; 2],
    solver_endpoint_longitudes: [f64; 2],
}

impl SphericalCirclePcurve {
    /// Spatial circle whose inverse sphere chart this pcurve represents.
    pub const fn carrier(self) -> Circle {
        self.carrier
    }

    /// Exact sphere field used by the inverse chart.
    pub const fn sphere(self) -> Sphere {
        self.sphere
    }

    /// Complete finite parameter range of the represented branch.
    pub const fn carrier_range(self) -> ParamRange {
        self.carrier_range
    }

    /// Conservative finite longitude/latitude enclosure retained at minting.
    pub const fn chart_window(self) -> [ParamRange; 2] {
        self.chart_window
    }

    fn local_derivative_jets(self, parameter: f64) -> [DerivativeJet; 3] {
        let derivatives = self.carrier.eval_derivs(parameter, 3);
        let frame = self.sphere.frame();
        let mut out = [DerivativeJet::default(); 3];
        for order in 0..=3 {
            let value = if order == 0 {
                derivatives.d[0] - frame.origin()
            } else {
                derivatives.d[order]
            };
            out[0].d[order] = value.dot(frame.x());
            out[1].d[order] = value.dot(frame.y());
            out[2].d[order] = value.dot(frame.z());
        }
        out
    }

    fn unwrapped_longitude(self, parameter: f64, raw: f64) -> f64 {
        if parameter == self.carrier_range.lo {
            return self.endpoint_longitudes[0];
        }
        if parameter == self.carrier_range.hi {
            return self.endpoint_longitudes[1];
        }
        let mut turn = self.initial_turn;
        for seam in self.seams.iter().take(usize::from(self.seam_count)) {
            if parameter == seam.parameter {
                return seam.longitude;
            }
            if parameter > seam.parameter {
                turn += seam.turn_delta;
            }
        }
        raw + turn * core::f64::consts::TAU
    }
}

impl Curve2d for SphericalCirclePcurve {
    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn eval_derivs(&self, parameter: f64, order: usize) -> Curve2dDerivs {
        let parameter = parameter.clamp(self.carrier_range.lo, self.carrier_range.hi);
        let [x, y, z] = self.local_derivative_jets(parameter);
        let rho = (x * x + y * y).sqrt();
        let mut longitude = DerivativeJet::atan2(y, x);
        longitude.d[0] = self.unwrapped_longitude(parameter, longitude.d[0]);
        let latitude = DerivativeJet::atan2(z, rho);
        let mut out = Curve2dDerivs::default();
        for derivative in 0..=order.min(3) {
            out.d[derivative] = Vec2::new(longitude.d[derivative], latitude.d[derivative]);
        }
        out
    }

    fn param_range(&self) -> ParamRange {
        self.carrier_range
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, _: ParamRange) -> Aabb2 {
        Aabb2 {
            min: Vec2::new(self.chart_window[0].lo, self.chart_window[1].lo),
            max: Vec2::new(self.chart_window[0].hi, self.chart_window[1].hi),
        }
    }
}

/// Exact inverse-chart sphere trace used by a general plane/sphere circle
/// proof.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObliqueSphereCircleTrace {
    surface: Sphere,
    pcurve: SphericalCirclePcurve,
    parameter_map: AffineParamMap1d,
}

impl ObliqueSphereCircleTrace {
    /// Exact sphere field.
    pub const fn surface(self) -> Sphere {
        self.surface
    }

    /// Certifier-minted nonlinear inverse sphere-chart pcurve.
    pub const fn pcurve(self) -> SphericalCirclePcurve {
        self.pcurve
    }

    /// Carrier-to-pcurve parameter map.
    pub const fn parameter_map(self) -> AffineParamMap1d {
        self.parameter_map
    }
}

/// One operand-ordered trace of a plane/sphere circle proof.
// Keeping complete exact trace payloads inline preserves the established Copy
// certificate contract and avoids identity-bearing shared proof allocations.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlaneSphereCircleTrace {
    /// Plane trace carried by a parameter-space circle.
    Plane(PlaneCircleTrace),
    /// Sphere trace carried by a longitude line at constant latitude.
    Sphere(SphereLatitudeTrace),
    /// General inverse sphere-chart trace for an oblique spatial circle.
    SphereOblique(ObliqueSphereCircleTrace),
}

impl PlaneSphereCircleTrace {
    /// Carrier-to-pcurve parameter map.
    pub const fn parameter_map(self) -> AffineParamMap1d {
        match self {
            Self::Plane(trace) => trace.parameter_map(),
            Self::Sphere(trace) => trace.parameter_map(),
            Self::SphereOblique(trace) => trace.parameter_map(),
        }
    }
}

/// Whole-interval paired residual proof for one axis-aligned plane/sphere
/// circle branch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PairedPlaneSphereCircleResidualCertificate {
    carrier: Circle,
    carrier_range: ParamRange,
    traces: [PlaneSphereCircleTrace; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
}

/// Exact carrier families currently supported by persistent intersection
/// descriptors.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VerifiedIntersectionCarrier {
    /// Finite line branch.
    Line(Line),
    /// Finite circular branch.
    Circle(Circle),
}

impl VerifiedIntersectionCarrier {
    /// Borrow this carrier as a line when its family matches.
    pub const fn as_line(self) -> Option<Line> {
        if let Self::Line(carrier) = self {
            Some(carrier)
        } else {
            None
        }
    }

    /// Borrow this carrier as a circle when its family matches.
    pub const fn as_circle(self) -> Option<Circle> {
        if let Self::Circle(carrier) = self {
            Some(carrier)
        } else {
            None
        }
    }
}

/// Whole-interval proof families retained by persistent intersection curves.
// The proof families intentionally remain inline so the public certificate is
// an immutable Copy value with structural equality across persistence checks.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VerifiedIntersectionCertificate {
    /// Plane/plane line proof.
    PlaneLine(PairedPlaneLineResidualCertificate),
    /// Axis-aligned or axis-antialigned plane/sphere circle proof.
    PlaneSphereCircle(PairedPlaneSphereCircleResidualCertificate),
}

impl VerifiedIntersectionCertificate {
    /// Exact carrier geometry.
    pub const fn carrier(self) -> VerifiedIntersectionCarrier {
        match self {
            Self::PlaneLine(certificate) => {
                VerifiedIntersectionCarrier::Line(certificate.carrier())
            }
            Self::PlaneSphereCircle(certificate) => {
                VerifiedIntersectionCarrier::Circle(certificate.carrier())
            }
        }
    }

    /// Complete finite parameter range covered by the proof.
    pub const fn carrier_range(self) -> ParamRange {
        match self {
            Self::PlaneLine(certificate) => certificate.carrier_range(),
            Self::PlaneSphereCircle(certificate) => certificate.carrier_range(),
        }
    }

    /// Conservative paired residual bounds in operand order.
    pub const fn residual_bounds(self) -> [f64; 2] {
        match self {
            Self::PlaneLine(certificate) => certificate.residual_bounds(),
            Self::PlaneSphereCircle(certificate) => certificate.residual_bounds(),
        }
    }

    /// Carrier-to-pcurve parameter maps in operand order.
    pub const fn parameter_maps(self) -> [AffineParamMap1d; 2] {
        match self {
            Self::PlaneLine(certificate) => certificate.parameter_maps(),
            Self::PlaneSphereCircle(certificate) => certificate.parameter_maps(),
        }
    }

    /// Model-space tolerance used by the proof.
    pub const fn tolerance(self) -> f64 {
        match self {
            Self::PlaneLine(certificate) => certificate.tolerance(),
            Self::PlaneSphereCircle(certificate) => certificate.tolerance(),
        }
    }

    /// Borrow the line proof when its family matches.
    pub const fn as_plane_line(self) -> Option<PairedPlaneLineResidualCertificate> {
        if let Self::PlaneLine(certificate) = self {
            Some(certificate)
        } else {
            None
        }
    }

    /// Borrow the plane/sphere circle proof when its family matches.
    pub const fn as_plane_sphere_circle(
        self,
    ) -> Option<PairedPlaneSphereCircleResidualCertificate> {
        if let Self::PlaneSphereCircle(certificate) = self {
            Some(certificate)
        } else {
            None
        }
    }
}

/// Persistent graph descriptor for one verified finite intersection branch.
///
/// Source and pcurve handles are retained in operand order and become graph
/// dependencies. Fields are private: descriptors can only be minted by the
/// graph after it verifies that the referenced source fields and pcurves are
/// exactly the geometry bound into `certificate`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VerifiedIntersectionCurveDescriptor {
    source_surfaces: [SurfaceHandle; 2],
    pcurves: [Curve2dHandle; 2],
    certificate: VerifiedIntersectionCertificate,
}

/// Whole-range proof retained for an imported chordal exact-plane-field
/// intersection chart.
///
/// The transmitted 3D and paired parameter-space curves remain degree-1,
/// polynomial, open, and clamped. Their identical knot vectors make every
/// lifted residual affine on each complete knot span, so the maximum norm is
/// bounded by the certified control-point residuals.
#[derive(Debug, Clone, PartialEq)]
pub struct TransmittedPlaneIntersectionCertificate {
    carrier: NurbsCurve,
    carrier_range: ParamRange,
    surfaces: [Plane; 2],
    pcurves: [NurbsCurve2d; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
    metadata: TransmittedIntersectionChartMetadata,
}

/// Fixed binary depth of the source-NURBS trace proof on every transmitted
/// degree-1 carrier span.
pub const TRANSMITTED_NURBS_TRACE_PROOF_DEPTH: usize = 10;

/// One constant signed normal offset of an original NURBS basis retained by
/// a transmitted whole-range trace proof.
#[derive(Debug, Clone, PartialEq)]
pub struct TransmittedOffsetNurbsTrace {
    basis: NurbsSurface,
    signed_distance: f64,
}

impl TransmittedOffsetNurbsTrace {
    /// Construct a trace candidate. Finiteness and whole-range regularity are
    /// checked by the certifier.
    pub const fn new(basis: NurbsSurface, signed_distance: f64) -> Self {
        Self {
            basis,
            signed_distance,
        }
    }

    /// Original NURBS basis descriptor.
    pub const fn basis(&self) -> &NurbsSurface {
        &self.basis
    }

    /// Signed displacement along the basis natural unit normal.
    pub const fn signed_distance(&self) -> f64 {
        self.signed_distance
    }
}

/// Ordered exact source trace retained by a whole-range NURBS branch proof.
#[derive(Debug, Clone, PartialEq)]
pub enum NurbsIntersectionTrace {
    /// Exact direct or effective plane field.
    Plane(Plane),
    /// Exact direct sphere field.
    Sphere(Sphere),
    /// Original source NURBS surface descriptor.
    Nurbs(NurbsSurface),
    /// Constant normal offset of an original NURBS basis.
    OffsetNurbs(TransmittedOffsetNurbsTrace),
}

/// Compatibility name for transmitted-chart callers.
pub type TransmittedNurbsIntersectionTrace = NurbsIntersectionTrace;
/// Compatibility name for the original mixed Plane/NURBS trace API.
pub type TransmittedPlaneNurbsTrace = NurbsIntersectionTrace;

impl NurbsIntersectionTrace {
    /// Borrow the trace as a plane when its family matches.
    pub const fn as_plane(&self) -> Option<Plane> {
        if let Self::Plane(plane) = self {
            Some(*plane)
        } else {
            None
        }
    }

    /// Borrow the trace as a sphere when its family matches.
    pub const fn as_sphere(&self) -> Option<Sphere> {
        if let Self::Sphere(sphere) = self {
            Some(*sphere)
        } else {
            None
        }
    }

    /// Borrow the trace as its original NURBS source when its family matches.
    pub const fn as_nurbs(&self) -> Option<&NurbsSurface> {
        if let Self::Nurbs(surface) = self {
            Some(surface)
        } else {
            None
        }
    }

    /// Borrow the trace as a constant offset of a NURBS basis.
    pub const fn as_offset_nurbs(&self) -> Option<&TransmittedOffsetNurbsTrace> {
        if let Self::OffsetNurbs(trace) = self {
            Some(trace)
        } else {
            None
        }
    }
}

/// Exact three-sample chart tuples retained as interpolation witnesses.
///
/// These values determine the canonical quadratic carrier and pcurves. They
/// are not themselves whole-range proof geometry; acceptance still requires
/// the original-source interval residual certificate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransmittedQuadraticInterpolationWitnesses {
    positions: [Vec3; 3],
    canonicalized_pcurve_points: [[Vec2; 3]; 2],
}

/// Exact four-sample chart tuples retained as interpolation witnesses.
///
/// These values determine the canonical cubic carrier and pcurves. They are
/// not themselves whole-range proof geometry; acceptance still requires the
/// original-source interval residual certificate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransmittedCubicInterpolationWitnesses {
    positions: [Vec3; 4],
    canonicalized_pcurve_points: [[Vec2; 4]; 2],
}

impl TransmittedCubicInterpolationWitnesses {
    /// Exact transmitted model-space positions in chart order.
    pub const fn positions(self) -> [Vec3; 4] {
        self.positions
    }

    /// Canonicalized transmitted paired UV tuples in operand order.
    pub const fn canonicalized_pcurve_points(self) -> [[Vec2; 4]; 2] {
        self.canonicalized_pcurve_points
    }
}

impl TransmittedQuadraticInterpolationWitnesses {
    /// Exact transmitted model-space positions in chart order.
    pub const fn positions(self) -> [Vec3; 3] {
        self.positions
    }

    /// Canonicalized transmitted paired UV tuples in operand order.
    pub const fn canonicalized_pcurve_points(self) -> [[Vec2; 3]; 2] {
        self.canonicalized_pcurve_points
    }
}

/// Whole-range proof retained for a transmitted exact-plane-field/NURBS,
/// NURBS/NURBS, direct Offset(NURBS)/NURBS,
/// exact-plane-field/Offset(NURBS), or bounded dual-Offset(NURBS) chart.
///
/// The NURBS trace is bounded on every binary carrier subdivision with an
/// original-source point/partial interval enclosure and a centered
/// mean-value residual. This preserves the shared affine carrier/pcurve
/// parameter correlation and supports polynomial and rational non-planar
/// surfaces without point sampling or spatial-intersection recomputation.
#[derive(Debug, Clone, PartialEq)]
pub struct TransmittedNurbsIntersectionCertificate {
    carrier: NurbsCurve,
    carrier_range: ParamRange,
    carrier_period: Option<f64>,
    traces: [TransmittedNurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
    metadata: TransmittedIntersectionChartMetadata,
    proof_depth: usize,
    quadratic_witnesses: Option<TransmittedQuadraticInterpolationWitnesses>,
    cubic_witnesses: Option<TransmittedCubicInterpolationWitnesses>,
}

/// Compatibility name for the original mixed Plane/NURBS certificate API.
pub type TransmittedPlaneNurbsIntersectionCertificate = TransmittedNurbsIntersectionCertificate;

impl TransmittedNurbsIntersectionCertificate {
    /// Retained certified model-space chart carrier.
    pub fn carrier(&self) -> &NurbsCurve {
        &self.carrier
    }

    /// Complete canonical chart interval covered by the proof.
    pub const fn carrier_range(&self) -> ParamRange {
        self.carrier_range
    }

    /// Certified full-chart carrier period, when this transmitted chart is a
    /// one-seam closed ring.
    pub const fn carrier_periodicity(&self) -> Option<f64> {
        self.carrier_period
    }

    /// Promote a closed transmitted chart to a periodic carrier only when its
    /// retained proof crosses exactly one complete certified NURBS seam.
    ///
    /// Ordinary finite-open transmitted charts remain nonperiodic. The
    /// promotion additionally requires the spatial carrier endpoints to close
    /// within the certificate tolerance and the paired pcurve endpoints to
    /// differ by exactly one source-domain period, modulo a bounded floating
    /// point seam slack.
    pub fn with_certified_carrier_periodicity(
        mut self,
    ) -> Result<Self, IntersectionCertificateError> {
        let first = self.carrier.points()[0];
        let last = self.carrier.points()[self.carrier.points().len() - 1];
        if first.dist(last) > self.tolerance {
            return Err(
                IntersectionCertificateError::UnsupportedCarrierParameterization {
                    reason: "periodic transmitted carrier endpoints do not close within tolerance",
                },
            );
        }

        let mut periodic_axes = 0_usize;
        for (trace_index, trace) in self.traces.iter().enumerate() {
            let surface = match trace {
                TransmittedNurbsIntersectionTrace::Plane(_)
                | TransmittedNurbsIntersectionTrace::Sphere(_) => continue,
                TransmittedNurbsIntersectionTrace::Nurbs(surface) => surface,
                TransmittedNurbsIntersectionTrace::OffsetNurbs(offset) => offset.basis(),
            };
            let first_uv = self.pcurves[trace_index].points()[0];
            let last_uv =
                self.pcurves[trace_index].points()[self.pcurves[trace_index].points().len() - 1];
            for (axis, period) in surface.periodicity().into_iter().enumerate() {
                let Some(period) = period else {
                    continue;
                };
                periodic_axes += 1;
                let domain = surface
                    .knots(if axis == 0 { Dir::U } else { Dir::V })
                    .domain();
                if period != domain.width() {
                    return Err(
                        IntersectionCertificateError::UnsupportedCarrierParameterization {
                            reason: "periodic transmitted trace period is not its complete source domain",
                        },
                    );
                }
                let (first_coordinate, last_coordinate) = if axis == 0 {
                    (first_uv.x, last_uv.x)
                } else {
                    (first_uv.y, last_uv.y)
                };
                let scale = domain
                    .lo
                    .abs()
                    .max(domain.hi.abs())
                    .max(period.abs())
                    .max(1.0);
                let seam_slack = 16_384.0 * f64::EPSILON * scale;
                if ((last_coordinate - first_coordinate).abs() - period).abs() > seam_slack {
                    return Err(
                        IntersectionCertificateError::UnsupportedCarrierParameterization {
                            reason: "periodic transmitted pcurve does not cross one complete certified source seam",
                        },
                    );
                }
            }
        }
        if periodic_axes != 1 {
            return Err(
                IntersectionCertificateError::UnsupportedCarrierParameterization {
                    reason: "periodic transmitted carrier requires exactly one certified NURBS seam axis",
                },
            );
        }

        self.carrier_period = Some(self.carrier_range.width());
        Ok(self)
    }

    /// Exact ordered proof sources.
    pub const fn traces(&self) -> &[TransmittedNurbsIntersectionTrace; 2] {
        &self.traces
    }

    /// Retained paired pcurves in operand order.
    pub const fn pcurves(&self) -> &[NurbsCurve2d; 2] {
        &self.pcurves
    }

    /// Conservative whole-range lifted residual bounds.
    pub const fn residual_bounds(&self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Explicit source/declaration tolerance used by import certification.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// Exact transmitted affine/error metadata.
    pub const fn metadata(&self) -> TransmittedIntersectionChartMetadata {
        self.metadata
    }

    /// Binary proof depth used on each carrier knot span.
    pub const fn proof_depth(&self) -> usize {
        self.proof_depth
    }

    /// Exact three-sample witnesses for the bounded quadratic dual-offset
    /// family, or `None` for every historical transmitted family.
    pub const fn quadratic_interpolation_witnesses(
        &self,
    ) -> Option<TransmittedQuadraticInterpolationWitnesses> {
        self.quadratic_witnesses
    }

    /// Exact four-sample witnesses for the bounded cubic dual-offset family,
    /// or `None` for every other transmitted family.
    pub const fn cubic_interpolation_witnesses(
        &self,
    ) -> Option<TransmittedCubicInterpolationWitnesses> {
        self.cubic_witnesses
    }
}

/// Source chart declaration retained alongside a transmitted intersection
/// proof. These values describe the published affine parameter convention and
/// its declared approximation/error metadata; they are not recomputed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransmittedIntersectionChartMetadata {
    base_parameter: f64,
    base_scale: f64,
    chordal_error: f64,
    angular_error: f64,
    parameter_error: [Option<f64>; 2],
}

impl TransmittedIntersectionChartMetadata {
    /// Construct finite, nonnegative transmitted chart metadata.
    pub fn new(
        base_parameter: f64,
        base_scale: f64,
        chordal_error: f64,
        angular_error: f64,
        parameter_error: [Option<f64>; 2],
    ) -> Result<Self, IntersectionCertificateError> {
        if !base_parameter.is_finite()
            || !base_scale.is_finite()
            || base_scale <= 0.0
            || !chordal_error.is_finite()
            || chordal_error < 0.0
            || !angular_error.is_finite()
            || angular_error < 0.0
            || parameter_error
                .iter()
                .flatten()
                .any(|value| !value.is_finite() || *value < 0.0)
        {
            return Err(IntersectionCertificateError::NonFiniteGeometry);
        }
        Ok(Self {
            base_parameter,
            base_scale,
            chordal_error,
            angular_error,
            parameter_error,
        })
    }

    /// Transmitted affine parameter origin.
    pub const fn base_parameter(self) -> f64 {
        self.base_parameter
    }

    /// Transmitted affine parameter scale.
    pub const fn base_scale(self) -> f64 {
        self.base_scale
    }

    /// Published chordal error.
    pub const fn chordal_error(self) -> f64 {
        self.chordal_error
    }

    /// Published angular error.
    pub const fn angular_error(self) -> f64 {
        self.angular_error
    }

    /// Published per-operand parameter errors.
    pub const fn parameter_error(self) -> [Option<f64>; 2] {
        self.parameter_error
    }
}

impl TransmittedPlaneIntersectionCertificate {
    /// Retained degree-1 model-space chart carrier.
    pub fn carrier(&self) -> &NurbsCurve {
        &self.carrier
    }

    /// Complete canonical chart interval covered by the proof.
    pub const fn carrier_range(&self) -> ParamRange {
        self.carrier_range
    }

    /// Exact source planes in operand order.
    pub const fn surfaces(&self) -> [Plane; 2] {
        self.surfaces
    }

    /// Retained paired degree-1 pcurves in operand order.
    pub fn pcurves(&self) -> &[NurbsCurve2d; 2] {
        &self.pcurves
    }

    /// Conservative whole-range lifted residual bounds.
    pub const fn residual_bounds(&self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Explicit source/declaration tolerance used by import certification.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// Exact transmitted affine/error metadata.
    pub const fn metadata(&self) -> TransmittedIntersectionChartMetadata {
        self.metadata
    }
}

/// Persistent graph descriptor for a verified transmitted chordal
/// intersection of two effective plane fields.
#[derive(Debug, Clone, PartialEq)]
pub struct TransmittedIntersectionCurveDescriptor {
    source_surfaces: [SurfaceHandle; 2],
    pcurves: [Curve2dHandle; 2],
    certificate: TransmittedPlaneIntersectionCertificate,
}

/// Persistent graph descriptor for a verified transmitted chordal chart with
/// one or two original NURBS traces.
#[derive(Debug, Clone, PartialEq)]
pub struct TransmittedNurbsIntersectionCurveDescriptor {
    source_surfaces: [SurfaceHandle; 2],
    pcurves: [Curve2dHandle; 2],
    certificate: TransmittedNurbsIntersectionCertificate,
}

impl TransmittedNurbsIntersectionCurveDescriptor {
    pub(crate) const fn new(
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: TransmittedNurbsIntersectionCertificate,
    ) -> Self {
        Self {
            source_surfaces,
            pcurves,
            certificate,
        }
    }

    /// Source surface identities in transmitted operand order.
    pub const fn source_surfaces(&self) -> [SurfaceHandle; 2] {
        self.source_surfaces
    }

    /// Paired pcurve identities in transmitted operand order.
    pub const fn pcurves(&self) -> [Curve2dHandle; 2] {
        self.pcurves
    }

    /// Exact carrier/source/pcurve proof payload.
    pub const fn certificate(&self) -> &TransmittedNurbsIntersectionCertificate {
        &self.certificate
    }

    pub(crate) fn visit_dependencies(&self, visit: &mut dyn FnMut(GeometryRef)) {
        for surface in self.source_surfaces {
            visit(GeometryRef::Surface(surface));
        }
        for pcurve in self.pcurves {
            visit(GeometryRef::Curve2d(pcurve));
        }
    }
}

impl Curve for TransmittedNurbsIntersectionCurveDescriptor {
    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn eval_derivs(&self, parameter: f64, order: usize) -> CurveDerivs {
        let parameter = self.certificate.carrier_period.map_or(parameter, |period| {
            wrap_periodic(parameter, self.certificate.carrier_range.lo, period)
        });
        self.certificate.carrier.eval_derivs(parameter, order)
    }

    fn param_range(&self) -> ParamRange {
        self.certificate.carrier_range
    }

    fn periodicity(&self) -> Option<f64> {
        self.certificate.carrier_period
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb3 {
        if self.certificate.carrier_period.is_some() {
            // A caller may request an interval outside the retained base
            // chart. The complete base-chart hull is conservative for every
            // periodic interval and exact for the full-period ring domain.
            self.certificate
                .carrier
                .bounding_box(self.certificate.carrier_range)
        } else {
            self.certificate.carrier.bounding_box(range)
        }
    }
}

impl TransmittedIntersectionCurveDescriptor {
    pub(crate) const fn new(
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: TransmittedPlaneIntersectionCertificate,
    ) -> Self {
        Self {
            source_surfaces,
            pcurves,
            certificate,
        }
    }

    /// Source surface identities in transmitted operand order.
    pub const fn source_surfaces(&self) -> [SurfaceHandle; 2] {
        self.source_surfaces
    }

    /// Paired pcurve identities in transmitted operand order.
    pub const fn pcurves(&self) -> [Curve2dHandle; 2] {
        self.pcurves
    }

    /// Exact carrier/source/pcurve proof payload.
    pub const fn certificate(&self) -> &TransmittedPlaneIntersectionCertificate {
        &self.certificate
    }

    pub(crate) fn visit_dependencies(&self, visit: &mut dyn FnMut(GeometryRef)) {
        for surface in self.source_surfaces {
            visit(GeometryRef::Surface(surface));
        }
        for pcurve in self.pcurves {
            visit(GeometryRef::Curve2d(pcurve));
        }
    }
}

impl Curve for TransmittedIntersectionCurveDescriptor {
    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn eval_derivs(&self, parameter: f64, order: usize) -> CurveDerivs {
        self.certificate.carrier.eval_derivs(parameter, order)
    }

    fn param_range(&self) -> ParamRange {
        self.certificate.carrier_range
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb3 {
        self.certificate.carrier.bounding_box(range)
    }
}

impl VerifiedIntersectionCurveDescriptor {
    pub(crate) const fn new(
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: VerifiedIntersectionCertificate,
    ) -> Self {
        Self {
            source_surfaces,
            pcurves,
            certificate,
        }
    }

    /// Source surface identities in operand order.
    pub const fn source_surfaces(self) -> [SurfaceHandle; 2] {
        self.source_surfaces
    }

    /// Persistent paired pcurve identities in operand order.
    pub const fn pcurves(self) -> [Curve2dHandle; 2] {
        self.pcurves
    }

    /// Whole-interval paired trace proof retained by this descriptor.
    pub const fn certificate(self) -> VerifiedIntersectionCertificate {
        self.certificate
    }

    /// Exact model-space carrier.
    pub const fn carrier(self) -> VerifiedIntersectionCarrier {
        self.certificate.carrier()
    }

    /// Complete finite parameter interval covered by the descriptor proof.
    pub const fn carrier_range(self) -> ParamRange {
        self.certificate.carrier_range()
    }

    pub(crate) fn visit_dependencies(self, visit: &mut dyn FnMut(GeometryRef)) {
        for surface in self.source_surfaces {
            visit(GeometryRef::Surface(surface));
        }
        for pcurve in self.pcurves {
            visit(GeometryRef::Curve2d(pcurve));
        }
    }
}

impl Curve for VerifiedIntersectionCurveDescriptor {
    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn eval_derivs(&self, parameter: f64, order: usize) -> CurveDerivs {
        debug_assert!(self.carrier_range().contains(parameter));
        let parameter = parameter.clamp(self.carrier_range().lo, self.carrier_range().hi);
        match self.carrier() {
            VerifiedIntersectionCarrier::Line(carrier) => carrier.eval_derivs(parameter, order),
            VerifiedIntersectionCarrier::Circle(carrier) => carrier.eval_derivs(parameter, order),
        }
    }

    fn param_range(&self) -> ParamRange {
        self.carrier_range()
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb3 {
        debug_assert!(range.is_finite());
        debug_assert!(range.lo >= self.carrier_range().lo && range.hi <= self.carrier_range().hi);
        let range = ParamRange::new(
            range
                .lo
                .clamp(self.carrier_range().lo, self.carrier_range().hi),
            range
                .hi
                .clamp(self.carrier_range().lo, self.carrier_range().hi),
        );
        match self.carrier() {
            VerifiedIntersectionCarrier::Line(carrier) => carrier.bounding_box(range),
            VerifiedIntersectionCarrier::Circle(carrier) => {
                if carrier.param_range().contains(range.lo)
                    && carrier.param_range().contains(range.hi)
                {
                    carrier.bounding_box(range)
                } else {
                    // Shifted longitude charts may place a finite branch
                    // outside the circle's canonical [0, 2pi] interval. The
                    // current analytic circle bound enumerates a fixed set of
                    // canonical extrema, so use its complete-turn box for
                    // those equivalent shifted parameters.
                    carrier.bounding_box(carrier.param_range())
                }
            }
        }
    }
}

impl PairedPlaneLineResidualCertificate {
    /// Verified model-space carrier.
    pub const fn carrier(self) -> Line {
        self.carrier
    }

    /// Complete carrier interval covered by the proof.
    pub const fn carrier_range(self) -> ParamRange {
        self.carrier_range
    }

    /// Verified plane surfaces, in operand order.
    pub const fn surfaces(self) -> [Plane; 2] {
        self.surfaces
    }

    /// Verified parameter-space lines, in operand order.
    pub const fn pcurves(self) -> [Line2d; 2] {
        self.pcurves
    }

    /// Carrier-to-pcurve parameter maps, in operand order.
    pub const fn parameter_maps(self) -> [AffineParamMap1d; 2] {
        self.parameter_maps
    }

    /// Conservative whole-interval residual bounds, in operand order.
    pub const fn residual_bounds(self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Model-space tolerance against which both traces were certified.
    pub const fn tolerance(self) -> f64 {
        self.tolerance
    }
}

impl PairedPlaneSphereCircleResidualCertificate {
    /// Verified model-space circle carrier.
    pub const fn carrier(self) -> Circle {
        self.carrier
    }

    /// Complete carrier interval covered by the proof.
    pub const fn carrier_range(self) -> ParamRange {
        self.carrier_range
    }

    /// Verified traces in source operand order.
    pub const fn traces(self) -> [PlaneSphereCircleTrace; 2] {
        self.traces
    }

    /// Carrier-to-pcurve parameter maps in operand order.
    pub const fn parameter_maps(self) -> [AffineParamMap1d; 2] {
        [
            self.traces[0].parameter_map(),
            self.traces[1].parameter_map(),
        ]
    }

    /// Conservative whole-interval residual bounds in operand order.
    pub const fn residual_bounds(self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Model-space tolerance against which both traces were certified.
    pub const fn tolerance(self) -> f64 {
        self.tolerance
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct DerivativeJet {
    d: [f64; 4],
}

impl DerivativeJet {
    fn derivative(self) -> Self {
        Self {
            d: [self.d[1], self.d[2], self.d[3], 0.0],
        }
    }

    fn reciprocal(self) -> Self {
        let mut out = Self::default();
        out.d[0] = 1.0 / self.d[0];
        out.d[1] = -(self.d[1] * out.d[0]) / self.d[0];
        out.d[2] = -(self.d[2] * out.d[0] + 2.0 * self.d[1] * out.d[1]) / self.d[0];
        out.d[3] =
            -(self.d[3] * out.d[0] + 3.0 * self.d[2] * out.d[1] + 3.0 * self.d[1] * out.d[2])
                / self.d[0];
        out
    }

    fn sqrt(self) -> Self {
        let mut out = Self::default();
        out.d[0] = self.d[0].max(0.0).sqrt();
        let denominator = 2.0 * out.d[0];
        out.d[1] = self.d[1] / denominator;
        out.d[2] = (self.d[2] - 2.0 * out.d[1] * out.d[1]) / denominator;
        out.d[3] = (self.d[3] - 6.0 * out.d[1] * out.d[2]) / denominator;
        out
    }

    fn atan2(y: Self, x: Self) -> Self {
        let angular_rate = (x * y.derivative() - y * x.derivative()) * (x * x + y * y).reciprocal();
        Self {
            d: [
                math::atan2(y.d[0], x.d[0]),
                angular_rate.d[0],
                angular_rate.d[1],
                angular_rate.d[2],
            ],
        }
    }
}

impl core::ops::Add for DerivativeJet {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self {
            d: core::array::from_fn(|index| self.d[index] + rhs.d[index]),
        }
    }
}

impl core::ops::Sub for DerivativeJet {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self {
            d: core::array::from_fn(|index| self.d[index] - rhs.d[index]),
        }
    }
}

impl core::ops::Mul for DerivativeJet {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self {
            d: [
                self.d[0] * rhs.d[0],
                self.d[1] * rhs.d[0] + self.d[0] * rhs.d[1],
                self.d[2] * rhs.d[0] + 2.0 * self.d[1] * rhs.d[1] + self.d[0] * rhs.d[2],
                self.d[3] * rhs.d[0]
                    + 3.0 * self.d[2] * rhs.d[1]
                    + 3.0 * self.d[1] * rhs.d[2]
                    + self.d[0] * rhs.d[3],
            ],
        }
    }
}

/// Construct and certify the nonlinear inverse sphere-chart trace for a
/// general finite plane/sphere circle branch.
///
/// `sphere_longitudes` are the authoritative analytic solver's fitted
/// longitudes at `carrier_range.lo/hi`. The returned pcurve has private fields
/// and is bound into the returned whole-branch certificate. `plane_position`
/// retains source operand order without changing the canonical carrier.
#[allow(clippy::too_many_arguments)]
pub fn certify_paired_plane_sphere_oblique_circle_residuals(
    carrier: Circle,
    carrier_range: ParamRange,
    plane: Plane,
    plane_pcurve: Circle2d,
    sphere: Sphere,
    sphere_window: [ParamRange; 2],
    sphere_longitudes: [f64; 2],
    plane_position: PairedTrace,
    tolerance: f64,
) -> Result<
    (
        SphericalCirclePcurve,
        PairedPlaneSphereCircleResidualCertificate,
    ),
    IntersectionCertificateError,
> {
    if !carrier_range.is_finite() || carrier_range.width() <= 0.0 {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if sphere_window
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
        || !sphere_longitudes.into_iter().all(f64::is_finite)
    {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    let identity = AffineParamMap1d::new(1.0, 0.0)?;
    let pcurve = build_spherical_circle_pcurve(
        carrier,
        carrier_range,
        sphere,
        sphere_window,
        sphere_longitudes,
        tolerance,
    )?;
    let plane_trace =
        PlaneSphereCircleTrace::Plane(PlaneCircleTrace::new(plane, plane_pcurve, identity));
    let sphere_trace = PlaneSphereCircleTrace::SphereOblique(ObliqueSphereCircleTrace {
        surface: sphere,
        pcurve,
        parameter_map: identity,
    });
    let traces = match plane_position {
        PairedTrace::First => [plane_trace, sphere_trace],
        PairedTrace::Second => [sphere_trace, plane_trace],
    };
    let certificate =
        certify_paired_plane_sphere_circle_residuals(carrier, carrier_range, traces, tolerance)?;
    Ok((pcurve, certificate))
}

fn build_spherical_circle_pcurve(
    carrier: Circle,
    carrier_range: ParamRange,
    sphere: Sphere,
    chart_window: [ParamRange; 2],
    endpoint_longitudes: [f64; 2],
    tolerance: f64,
) -> Result<SphericalCirclePcurve, IntersectionCertificateError> {
    let frame = sphere.frame();
    let center = frame.to_local(carrier.frame().origin());
    let x = carrier.frame().x() * carrier.radius();
    let y = carrier.frame().y() * carrier.radius();
    let cosine = Vec3::new(x.dot(frame.x()), x.dot(frame.y()), x.dot(frame.z()));
    let sine = Vec3::new(y.dot(frame.x()), y.dot(frame.y()), y.dot(frame.z()));
    let angular_tolerance = (tolerance / sphere.radius()).max(64.0 * f64::EPSILON);
    let probe = carrier_range.lerp(1.0e-6);
    let probe_local = harmonic_local_point(center, cosine, sine, probe);
    let raw_probe = math::atan2(probe_local.y, probe_local.x);
    let initial_turn = ((endpoint_longitudes[0] - raw_probe) / core::f64::consts::TAU).round();
    if !initial_turn.is_finite() {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let mut seams = [SphericalLongitudeSeam::default(); 2];
    let mut seam_count = 0_usize;
    let mut current_turn = initial_turn;
    for root in trig_linear_roots(sine.y, cosine.y, center.y, carrier_range, angular_tolerance) {
        if root <= carrier_range.lo + angular_tolerance
            || root >= carrier_range.hi - angular_tolerance
        {
            continue;
        }
        let local = harmonic_local_point(center, cosine, sine, root);
        if local.x >= 0.0 {
            continue;
        }
        let (sin, cos) = math::sincos(root);
        let y_derivative = -cosine.y * sin + sine.y * cos;
        if y_derivative.abs() <= angular_tolerance {
            continue;
        }
        if seam_count == seams.len() {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace: PairedTrace::Second,
                    reason: "inverse sphere chart exceeded its finite seam-crossing capacity",
                },
            );
        }
        let turn_delta = if y_derivative < 0.0 { 1.0 } else { -1.0 };
        let longitude = if turn_delta > 0.0 {
            core::f64::consts::PI + current_turn * core::f64::consts::TAU
        } else {
            -core::f64::consts::PI + current_turn * core::f64::consts::TAU
        };
        seams[seam_count] = SphericalLongitudeSeam {
            parameter: root,
            turn_delta,
            longitude,
        };
        seam_count += 1;
        current_turn += turn_delta;
    }
    seams[..seam_count].sort_by(|first, second| first.parameter.total_cmp(&second.parameter));

    current_turn = initial_turn;
    for seam in seams.iter().take(seam_count) {
        current_turn += seam.turn_delta;
    }
    let end_local = harmonic_local_point(center, cosine, sine, carrier_range.hi);
    let continuous_end =
        math::atan2(end_local.y, end_local.x) + current_turn * core::f64::consts::TAU;
    let start_local = harmonic_local_point(center, cosine, sine, carrier_range.lo);
    let continuous_start =
        math::atan2(start_local.y, start_local.x) + initial_turn * core::f64::consts::TAU;
    let fitted_start = periodic_representative_near(
        endpoint_longitudes[0],
        continuous_start,
        chart_window[0],
        angular_tolerance,
    )
    .unwrap_or_else(|| continuous_start.clamp(chart_window[0].lo, chart_window[0].hi));
    let fitted_end = periodic_representative_near(
        endpoint_longitudes[1],
        continuous_end,
        chart_window[0],
        angular_tolerance,
    )
    .unwrap_or_else(|| continuous_end.clamp(chart_window[0].lo, chart_window[0].hi));

    Ok(SphericalCirclePcurve {
        carrier,
        sphere,
        carrier_range,
        chart_window,
        initial_turn,
        seams,
        seam_count: seam_count as u8,
        endpoint_longitudes: [fitted_start, fitted_end],
        solver_endpoint_longitudes: endpoint_longitudes,
    })
}

fn harmonic_local_point(center: Vec3, cosine: Vec3, sine: Vec3, parameter: f64) -> Vec3 {
    let (sin, cos) = math::sincos(parameter);
    center + cosine * cos + sine * sin
}

fn trig_linear_roots(
    sine: f64,
    cosine: f64,
    constant: f64,
    range: ParamRange,
    tolerance: f64,
) -> Vec<f64> {
    let q2 = constant - cosine;
    let q1 = 2.0 * sine;
    let q0 = cosine + constant;
    let mut roots = Vec::new();
    for half_tangent in quadratic_roots(q2, q1, q0, tolerance) {
        push_periodic_root(
            &mut roots,
            2.0 * math::atan2(half_tangent, 1.0),
            range,
            tolerance,
        );
    }
    if q2.abs() <= tolerance {
        push_periodic_root(&mut roots, core::f64::consts::PI, range, tolerance);
    }
    roots.sort_by(f64::total_cmp);
    roots.dedup_by(|first, second| (*first - *second).abs() <= tolerance);
    roots
}

fn quadratic_roots(a: f64, b: f64, c: f64, tolerance: f64) -> Vec<f64> {
    if a.abs() <= tolerance {
        if b.abs() <= tolerance {
            Vec::new()
        } else {
            vec![-c / b]
        }
    } else {
        let discriminant = b * b - 4.0 * a * c;
        if discriminant < -tolerance {
            Vec::new()
        } else if discriminant.abs() <= tolerance {
            vec![-b / (2.0 * a)]
        } else {
            let root = discriminant.max(0.0).sqrt();
            vec![(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)]
        }
    }
}

fn push_periodic_root(roots: &mut Vec<f64>, candidate: f64, range: ParamRange, tolerance: f64) {
    let tau = core::f64::consts::TAU;
    let first = ((range.lo - tolerance - candidate) / tau).ceil();
    let last = ((range.hi + tolerance - candidate) / tau).floor();
    for turn in [first, last] {
        if turn.is_finite() {
            let root = (candidate + turn * tau).clamp(range.lo, range.hi);
            if !roots
                .iter()
                .any(|existing| (existing - root).abs() <= tolerance)
            {
                roots.push(root);
            }
        }
    }
}

fn periodic_representative_near(
    value: f64,
    target: f64,
    window: ParamRange,
    tolerance: f64,
) -> Option<f64> {
    let tau = core::f64::consts::TAU;
    let first_turn = ((window.lo - tolerance - value) / tau).ceil();
    let last_turn = ((window.hi + tolerance - value) / tau).floor();
    if first_turn > last_turn {
        return None;
    }
    let turn = ((target - value) / tau)
        .round()
        .clamp(first_turn, last_turn);
    let result = (value + turn * tau).clamp(window.lo, window.hi);
    result.is_finite().then_some(result)
}

/// Certify an axis-aligned or axis-antialigned plane/sphere circle over a
/// complete finite range.
///
/// The sphere trace must be `(u, v) = (t, constant)` with an identity map, so
/// its lift and the carrier share the exact same deterministic sine/cosine
/// evaluation even for shifted longitude intervals. The plane trace may map
/// the carrier parameter as `s=t` or `s=-t`; deterministic sine/cosine is
/// bitwise even/odd under that reversal. A nonzero phase belongs in the
/// plane-circle axis, not in either affine map. General oblique sphere traces
/// are admitted only through the private-field nonlinear spherical pcurve
/// produced by [`certify_paired_plane_sphere_oblique_circle_residuals`].
pub fn certify_paired_plane_sphere_circle_residuals(
    carrier: Circle,
    carrier_range: ParamRange,
    traces: [PlaneSphereCircleTrace; 2],
    tolerance: f64,
) -> Result<PairedPlaneSphereCircleResidualCertificate, IntersectionCertificateError> {
    if !carrier_range.is_finite() || carrier_range.lo > carrier_range.hi {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if !finite_circle(carrier) {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    if !matches!(
        traces,
        [
            PlaneSphereCircleTrace::Plane(_),
            PlaneSphereCircleTrace::Sphere(_)
        ] | [
            PlaneSphereCircleTrace::Sphere(_),
            PlaneSphereCircleTrace::Plane(_)
        ] | [
            PlaneSphereCircleTrace::Plane(_),
            PlaneSphereCircleTrace::SphereOblique(_)
        ] | [
            PlaneSphereCircleTrace::SphereOblique(_),
            PlaneSphereCircleTrace::Plane(_)
        ]
    ) {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }

    let mut residual_bounds = [0.0; 2];
    for (index, trace) in traces.into_iter().enumerate() {
        let trace_id = if index == 0 {
            PairedTrace::First
        } else {
            PairedTrace::Second
        };
        let bound = match trace {
            PlaneSphereCircleTrace::Plane(trace) => {
                let parameter_map = trace.parameter_map();
                if !matches!(parameter_map.scale(), -1.0 | 1.0)
                    || parameter_map.offset() != 0.0
                {
                    return Err(
                        IntersectionCertificateError::UnsupportedTraceParameterization {
                            trace: trace_id,
                            reason: "plane-circle trace must map the longitude as t or -t with no phase offset",
                        },
                    );
                }
                if !finite_plane(trace.surface()) || !finite_circle2d(trace.pcurve()) {
                    return Err(IntersectionCertificateError::NonFiniteGeometry);
                }
                plane_circle_residual_bound(
                    carrier,
                    trace.surface(),
                    trace.pcurve(),
                    parameter_map.scale(),
                )
            }
            PlaneSphereCircleTrace::Sphere(trace) => {
                if trace.parameter_map().scale() != 1.0
                    || trace.parameter_map().offset() != 0.0
                {
                    return Err(
                        IntersectionCertificateError::UnsupportedTraceParameterization {
                            trace: trace_id,
                            reason: "sphere trace must use the carrier longitude without an affine transform",
                        },
                    );
                }
                if !finite_sphere(trace.surface())
                    || !finite_vec2(trace.pcurve().origin())
                    || !finite_vec2(trace.pcurve().dir())
                {
                    return Err(IntersectionCertificateError::NonFiniteGeometry);
                }
                let pcurve = trace.pcurve();
                if pcurve.origin().x != 0.0 || pcurve.dir().x != 1.0 || pcurve.dir().y != 0.0 {
                    return Err(
                        IntersectionCertificateError::UnsupportedTraceParameterization {
                            trace: trace_id,
                            reason: "sphere trace must be u=t at one constant latitude",
                        },
                    );
                }
                sphere_latitude_residual_bound(carrier, trace.surface(), pcurve.origin().y)
            }
            PlaneSphereCircleTrace::SphereOblique(trace) => {
                if trace.parameter_map().scale() != 1.0
                    || trace.parameter_map().offset() != 0.0
                {
                    return Err(
                        IntersectionCertificateError::UnsupportedTraceParameterization {
                            trace: trace_id,
                            reason: "inverse sphere-chart trace must use the carrier parameter exactly",
                        },
                    );
                }
                let pcurve = trace.pcurve();
                if trace.surface() != pcurve.sphere()
                    || pcurve.carrier() != carrier
                    || pcurve.carrier_range() != carrier_range
                {
                    return Err(
                        IntersectionCertificateError::UnsupportedTraceParameterization {
                            trace: trace_id,
                            reason: "inverse sphere-chart trace is not bound to this carrier, range, and sphere",
                        },
                    );
                }
                Some(oblique_sphere_trace_residual_bound(
                    pcurve, tolerance, trace_id,
                )?)
            }
        }
        .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace: trace_id })?;
        if bound > tolerance {
            return Err(IntersectionCertificateError::ResidualExceedsTolerance {
                trace: trace_id,
                residual_bound: bound,
                tolerance,
            });
        }
        residual_bounds[index] = bound;
    }

    Ok(PairedPlaneSphereCircleResidualCertificate {
        carrier,
        carrier_range,
        traces,
        residual_bounds,
        tolerance,
    })
}

fn plane_circle_residual_bound(
    carrier: Circle,
    surface: Plane,
    pcurve: Circle2d,
    orientation: f64,
) -> Option<f64> {
    let carrier_coefficients = circle_coefficients(carrier)?;
    let frame = surface.frame();
    let center = pcurve.center();
    let x = pcurve.x_dir();
    let y = x.perp();
    let radius = Interval::point(pcurve.radius());
    let surface_origin = frame.origin().to_array();
    let surface_x = frame.x().to_array();
    let surface_y = frame.y().to_array();
    let mut lifted = [[Interval::point(0.0); 3]; 3];
    for axis in 0..3 {
        lifted[0][axis] = finite_interval(
            finite_interval(
                Interval::point(surface_origin[axis])
                    + finite_interval(
                        Interval::point(surface_x[axis]) * Interval::point(center.x),
                    )?,
            )? + finite_interval(Interval::point(surface_y[axis]) * Interval::point(center.y))?,
        )?;
        lifted[1][axis] = finite_interval(
            finite_interval(
                Interval::point(surface_x[axis]) * finite_interval(radius * Interval::point(x.x))?,
            )? + finite_interval(
                Interval::point(surface_y[axis]) * finite_interval(radius * Interval::point(x.y))?,
            )?,
        )?;
        lifted[2][axis] = finite_interval(
            Interval::point(orientation)
                * finite_interval(
                    finite_interval(
                        Interval::point(surface_x[axis])
                            * finite_interval(radius * Interval::point(y.x))?,
                    )? + finite_interval(
                        Interval::point(surface_y[axis])
                            * finite_interval(radius * Interval::point(y.y))?,
                    )?,
                )?,
        )?;
    }
    harmonic_residual_bound(carrier_coefficients, lifted)
}

fn sphere_latitude_residual_bound(carrier: Circle, surface: Sphere, latitude: f64) -> Option<f64> {
    let carrier_coefficients = circle_coefficients(carrier)?;
    let (sin_latitude, cos_latitude) = math::sincos(latitude);
    let frame = surface.frame();
    let radius = Interval::point(surface.radius());
    let surface_origin = frame.origin().to_array();
    let surface_x = frame.x().to_array();
    let surface_y = frame.y().to_array();
    let surface_z = frame.z().to_array();
    let radial_radius = finite_interval(radius * Interval::point(cos_latitude))?;
    let height = finite_interval(radius * Interval::point(sin_latitude))?;
    let mut lifted = [[Interval::point(0.0); 3]; 3];
    for axis in 0..3 {
        lifted[0][axis] = finite_interval(
            Interval::point(surface_origin[axis])
                + finite_interval(Interval::point(surface_z[axis]) * height)?,
        )?;
        lifted[1][axis] = finite_interval(Interval::point(surface_x[axis]) * radial_radius)?;
        lifted[2][axis] = finite_interval(Interval::point(surface_y[axis]) * radial_radius)?;
    }
    harmonic_residual_bound(carrier_coefficients, lifted)
}

fn oblique_sphere_trace_residual_bound(
    pcurve: SphericalCirclePcurve,
    tolerance: f64,
    trace: PairedTrace,
) -> Result<f64, IntersectionCertificateError> {
    let sphere = pcurve.sphere();
    let frame = sphere.frame();
    let carrier = pcurve.carrier();
    let center_local = frame.to_local(carrier.frame().origin());
    let carrier_x = carrier.frame().x() * carrier.radius();
    let carrier_y = carrier.frame().y() * carrier.radius();
    let cosine = Vec3::new(
        carrier_x.dot(frame.x()),
        carrier_x.dot(frame.y()),
        carrier_x.dot(frame.z()),
    );
    let sine = Vec3::new(
        carrier_y.dot(frame.x()),
        carrier_y.dot(frame.y()),
        carrier_y.dot(frame.z()),
    );
    let range = pcurve.carrier_range();
    let chart = pcurve.chart_window();
    let angular_tolerance = (tolerance / sphere.radius()).max(64.0 * f64::EPSILON);
    let scale = (center_local.norm() + carrier.radius() + sphere.radius()).max(1.0);
    let clearance_floor = tolerance.max(128.0 * f64::EPSILON * scale);
    let mut squared_pole_clearance = f64::INFINITY;
    let mut outside_coordinate = None;
    let residual_bound =
        sphere_radial_residual_bound(center_local, cosine, sine, sphere.radius(), trace)?;

    for segment in 0..SPHERICAL_CIRCLE_PROOF_SEGMENTS {
        let lo = range.lerp(segment as f64 / SPHERICAL_CIRCLE_PROOF_SEGMENTS as f64);
        let hi = range.lerp((segment + 1) as f64 / SPHERICAL_CIRCLE_PROOF_SEGMENTS as f64);
        let cosine_range = trig_interval(lo, hi, false);
        let sine_range = trig_interval(lo, hi, true);
        let coordinates = [
            harmonic_coordinate_interval(
                center_local.x,
                cosine.x,
                sine.x,
                cosine_range,
                sine_range,
            ),
            harmonic_coordinate_interval(
                center_local.y,
                cosine.y,
                sine.y,
                cosine_range,
                sine_range,
            ),
            harmonic_coordinate_interval(
                center_local.z,
                cosine.z,
                sine.z,
                cosine_range,
                sine_range,
            ),
        ];
        if coordinates
            .iter()
            .any(|interval| !interval.lo().is_finite() || !interval.hi().is_finite())
        {
            return Err(IntersectionCertificateError::NonFiniteResidualBound { trace });
        }
        let squared_xy = finite_interval(coordinates[0].square() + coordinates[1].square())
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
        squared_pole_clearance = squared_pole_clearance.min(squared_xy.lo().max(0.0));
        if squared_xy.lo() <= clearance_floor * clearance_floor {
            continue;
        }
        let midpoint = (lo + hi) / 2.0;
        let midpoint_uv = pcurve.eval(midpoint);
        let longitude = longitude_interval(coordinates[0], coordinates[1], midpoint_uv.x);
        if longitude.lo() < chart[0].lo - angular_tolerance
            || longitude.hi() > chart[0].hi + angular_tolerance
        {
            outside_coordinate.get_or_insert("longitude");
        }
        let rho = squared_xy
            .sqrt()
            .and_then(finite_interval)
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
        let latitude = latitude_interval(coordinates[2], rho);
        if latitude.lo() < chart[1].lo - angular_tolerance
            || latitude.hi() > chart[1].hi + angular_tolerance
        {
            outside_coordinate.get_or_insert("latitude");
        }
    }

    if squared_pole_clearance <= clearance_floor * clearance_floor {
        return Err(IntersectionCertificateError::SingularSphereChart {
            squared_pole_clearance,
        });
    }
    if let Some(coordinate) = outside_coordinate {
        return Err(IntersectionCertificateError::SphereTraceOutsideWindow { coordinate });
    }
    for (solver, certified) in pcurve
        .solver_endpoint_longitudes
        .into_iter()
        .zip(pcurve.endpoint_longitudes)
    {
        let fitted = periodic_representative_near(solver, certified, chart[0], angular_tolerance)
            .ok_or(
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace,
                reason: "solver endpoint longitude is inconsistent with continuous seam unwrapping",
            },
        )?;
        if (fitted - certified).abs() > angular_tolerance {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "solver endpoint longitude is inconsistent with continuous seam unwrapping",
                },
            );
        }
    }
    Ok(residual_bound)
}

fn sphere_radial_residual_bound(
    center: Vec3,
    cosine: Vec3,
    sine: Vec3,
    radius: f64,
    trace: PairedTrace,
) -> Result<f64, IntersectionCertificateError> {
    let dot = |first: Vec3, second: Vec3| {
        finite_interval(
            finite_interval(
                Interval::point(first.x) * Interval::point(second.x)
                    + Interval::point(first.y) * Interval::point(second.y),
            )? + Interval::point(first.z) * Interval::point(second.z),
        )
    };
    let half = Interval::point(0.5);
    let two = Interval::point(2.0);
    let center_sq = dot(center, center)
        .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    let cosine_sq = dot(cosine, cosine)
        .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    let sine_sq =
        dot(sine, sine).ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    let constant = finite_interval(
        center_sq
            + finite_interval((cosine_sq + sine_sq) * half)
                .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?
            - Interval::point(radius) * Interval::point(radius),
    )
    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    let first_cosine = finite_interval(
        dot(center, cosine)
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?
            * two,
    )
    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    let first_sine = finite_interval(
        dot(center, sine).ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?
            * two,
    )
    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    let second_cosine = finite_interval((cosine_sq - sine_sq) * half)
        .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    let second_sine =
        dot(cosine, sine).ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    let maximum_absolute = |interval: Interval| interval.lo().abs().max(interval.hi().abs());
    let squared_residual = maximum_absolute(constant)
        + maximum_absolute(first_cosine)
        + maximum_absolute(first_sine)
        + maximum_absolute(second_cosine)
        + maximum_absolute(second_sine);
    let bound = squared_residual / radius;
    if bound.is_finite() {
        Ok(bound.next_up())
    } else {
        Err(IntersectionCertificateError::NonFiniteResidualBound { trace })
    }
}

fn harmonic_coordinate_interval(
    center: f64,
    cosine: f64,
    sine: f64,
    cosine_range: Interval,
    sine_range: Interval,
) -> Interval {
    Interval::point(center)
        + Interval::point(cosine) * cosine_range
        + Interval::point(sine) * sine_range
}

fn trig_interval(lo: f64, hi: f64, sine: bool) -> Interval {
    let evaluate = if sine { math::sin } else { math::cos };
    let mut minimum = evaluate(lo).min(evaluate(hi));
    let mut maximum = evaluate(lo).max(evaluate(hi));
    let base = if sine {
        core::f64::consts::FRAC_PI_2
    } else {
        0.0
    };
    let mut multiple = ((lo - base) / core::f64::consts::PI).ceil();
    for _ in 0..=2 {
        let parameter = base + multiple * core::f64::consts::PI;
        if parameter > hi {
            break;
        }
        if parameter >= lo {
            let value = evaluate(parameter);
            minimum = minimum.min(value);
            maximum = maximum.max(value);
        }
        multiple += 1.0;
    }
    Interval::new(minimum.next_down(), maximum.next_up())
}

fn longitude_interval(x: Interval, y: Interval, reference: f64) -> Interval {
    let mut minimum = f64::INFINITY;
    let mut maximum = f64::NEG_INFINITY;
    for x_value in [x.lo(), x.hi()] {
        for y_value in [y.lo(), y.hi()] {
            let raw = math::atan2(y_value, x_value);
            let lifted =
                raw + ((reference - raw) / core::f64::consts::TAU).round() * core::f64::consts::TAU;
            minimum = minimum.min(lifted);
            maximum = maximum.max(lifted);
        }
    }
    Interval::new(minimum.next_down(), maximum.next_up())
}

fn latitude_interval(z: Interval, rho: Interval) -> Interval {
    let mut minimum = f64::INFINITY;
    let mut maximum = f64::NEG_INFINITY;
    for z_value in [z.lo(), z.hi()] {
        for rho_value in [rho.lo(), rho.hi()] {
            let value = math::atan2(z_value, rho_value);
            minimum = minimum.min(value);
            maximum = maximum.max(value);
        }
    }
    Interval::new(minimum.next_down(), maximum.next_up())
}

fn circle_coefficients(carrier: Circle) -> Option<[[Interval; 3]; 3]> {
    let frame = carrier.frame();
    let origin = frame.origin().to_array();
    let x = frame.x().to_array();
    let y = frame.y().to_array();
    let radius = Interval::point(carrier.radius());
    let mut coefficients = [[Interval::point(0.0); 3]; 3];
    for axis in 0..3 {
        coefficients[0][axis] = Interval::point(origin[axis]);
        coefficients[1][axis] = finite_interval(Interval::point(x[axis]) * radius)?;
        coefficients[2][axis] = finite_interval(Interval::point(y[axis]) * radius)?;
    }
    Some(coefficients)
}

fn harmonic_residual_bound(carrier: [[Interval; 3]; 3], lifted: [[Interval; 3]; 3]) -> Option<f64> {
    let harmonic = Interval::new(-1.0, 1.0);
    let mut squared_norm = Interval::point(0.0);
    for axis in 0..3 {
        let residual = finite_interval(
            finite_interval(carrier[0][axis] - lifted[0][axis])?
                + finite_interval(finite_interval(carrier[1][axis] - lifted[1][axis])? * harmonic)?
                + finite_interval(finite_interval(carrier[2][axis] - lifted[2][axis])? * harmonic)?,
        )?;
        squared_norm = finite_interval(squared_norm + finite_interval(residual.square())?)?;
    }
    Some(finite_interval(squared_norm.sqrt()?)?.hi())
}

/// Certifies two plane/pcurve traces against one finite line carrier.
///
/// For every carrier parameter `t` in `carrier_range`, each pcurve is
/// evaluated at `parameter_maps[i](t)` and lifted through `surfaces[i]`.
/// Outward-rounded interval arithmetic bounds the Euclidean distance between
/// that complete lifted trace and the carrier. Certification succeeds only
/// when both bounds are finite and no greater than `tolerance`.
pub fn certify_paired_plane_line_residuals(
    carrier: Line,
    carrier_range: ParamRange,
    surfaces: [Plane; 2],
    pcurves: [Line2d; 2],
    parameter_maps: [AffineParamMap1d; 2],
    tolerance: f64,
) -> Result<PairedPlaneLineResidualCertificate, IntersectionCertificateError> {
    if !carrier_range.is_finite() || carrier_range.lo > carrier_range.hi {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if !finite_vec3(carrier.origin())
        || !finite_vec3(carrier.dir())
        || surfaces.iter().any(|surface| !finite_plane(*surface))
        || pcurves
            .iter()
            .any(|pcurve| !finite_vec2(pcurve.origin()) || !finite_vec2(pcurve.dir()))
    {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let mut residual_bounds = [0.0; 2];
    for index in 0..2 {
        let trace = if index == 0 {
            PairedTrace::First
        } else {
            PairedTrace::Second
        };
        let Some(bound) = trace_residual_bound(
            carrier,
            carrier_range,
            surfaces[index],
            pcurves[index],
            parameter_maps[index],
        ) else {
            return Err(IntersectionCertificateError::NonFiniteResidualBound { trace });
        };
        if bound > tolerance {
            return Err(IntersectionCertificateError::ResidualExceedsTolerance {
                trace,
                residual_bound: bound,
                tolerance,
            });
        }
        residual_bounds[index] = bound;
    }

    Ok(PairedPlaneLineResidualCertificate {
        carrier,
        carrier_range,
        surfaces,
        pcurves,
        parameter_maps,
        residual_bounds,
        tolerance,
    })
}

/// Certifies a transmitted open degree-1 chart against two exact planes.
///
/// The carrier and both pcurves must be polynomial, clamped degree-1 NURBS
/// with identical knot vectors and control counts over their complete natural
/// range. Because the carrier and lifted plane traces share every linear basis
/// function, their residual is affine on each knot span. Convexity of the
/// Euclidean norm therefore makes the outward-rounded control residuals a
/// complete whole-range bound; no spatial intersection is recomputed.
pub fn certify_transmitted_plane_intersection_residuals(
    carrier: NurbsCurve,
    surfaces: [Plane; 2],
    pcurves: [NurbsCurve2d; 2],
    metadata: TransmittedIntersectionChartMetadata,
    tolerance: f64,
) -> Result<TransmittedPlaneIntersectionCertificate, IntersectionCertificateError> {
    let carrier_range = carrier.param_range();
    if !carrier_range.is_finite() || carrier_range.width() <= 0.0 {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if carrier.degree() != 1
        || carrier.weights().is_some()
        || !carrier.knots().is_clamped()
        || carrier.points().len() < 2
    {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "transmitted chart carrier must be open clamped polynomial degree 1",
            },
        );
    }
    if surfaces.iter().any(|surface| !finite_plane(*surface))
        || carrier
            .points()
            .iter()
            .copied()
            .any(|point| !finite_vec3(point))
    {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let carrier_knots = carrier.knots().as_slice();
    let mut residual_bounds = [0.0; 2];
    for index in 0..2 {
        let trace = if index == 0 {
            PairedTrace::First
        } else {
            PairedTrace::Second
        };
        let pcurve = &pcurves[index];
        if pcurve.degree() != 1
            || pcurve.weights().is_some()
            || !pcurve.knots().is_clamped()
            || pcurve.knots().as_slice() != carrier_knots
            || pcurve.points().len() != carrier.points().len()
            || pcurve.param_range() != carrier_range
        {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "transmitted pcurve must share the carrier's open clamped polynomial degree-1 basis",
                },
            );
        }
        if pcurve
            .points()
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
        {
            return Err(IntersectionCertificateError::NonFiniteGeometry);
        }
        let mut bound = 0.0_f64;
        for (&point, &uv) in carrier.points().iter().zip(pcurve.points()) {
            let control_bound = transmitted_plane_control_residual_bound(
                point,
                surfaces[index],
                Vec2::new(uv.x, uv.y),
            )
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
            bound = bound.max(control_bound);
        }
        if bound > tolerance {
            return Err(IntersectionCertificateError::ResidualExceedsTolerance {
                trace,
                residual_bound: bound,
                tolerance,
            });
        }
        residual_bounds[index] = bound;
    }

    Ok(TransmittedPlaneIntersectionCertificate {
        carrier,
        carrier_range,
        surfaces,
        pcurves,
        residual_bounds,
        tolerance,
        metadata,
    })
}

/// Certify a transmitted open degree-1 chart against one exact plane field
/// and one source NURBS surface in either operand order.
///
/// The plane trace uses the exact shared-control proof. The NURBS trace uses a
/// fixed, finite binary subdivision of each complete carrier span. On every
/// subinterval, original-source homogeneous interval evaluation encloses the
/// surface point and first partials, while a centered mean-value residual
/// keeps the carrier and affine pcurve parameter correlated. No sample is used
/// as proof evidence and the spatial intersection is never recomputed.
pub fn certify_transmitted_plane_nurbs_intersection_residuals(
    carrier: NurbsCurve,
    traces: [TransmittedNurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    metadata: TransmittedIntersectionChartMetadata,
    tolerance: f64,
) -> Result<TransmittedNurbsIntersectionCertificate, IntersectionCertificateError> {
    if !matches!(
        &traces,
        [
            TransmittedNurbsIntersectionTrace::Plane(_),
            TransmittedNurbsIntersectionTrace::Nurbs(_)
        ] | [
            TransmittedNurbsIntersectionTrace::Nurbs(_),
            TransmittedNurbsIntersectionTrace::Plane(_)
        ]
    ) {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    certify_transmitted_nurbs_intersection_residuals_impl(
        carrier, traces, pcurves, metadata, tolerance,
    )
}

/// Certify a transmitted open degree-1 chart against two ordered original
/// source NURBS surfaces.
///
/// Each trace independently uses the same fixed finite whole-range binary
/// subdivision and centered mean-value enclosure as the Plane/NURBS arm. The
/// proof therefore binds both original polynomial or rational sources without
/// sampling or spatial-intersection recomputation.
pub fn certify_transmitted_nurbs_nurbs_intersection_residuals(
    carrier: NurbsCurve,
    traces: [TransmittedNurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    metadata: TransmittedIntersectionChartMetadata,
    tolerance: f64,
) -> Result<TransmittedNurbsIntersectionCertificate, IntersectionCertificateError> {
    if !matches!(
        &traces,
        [
            TransmittedNurbsIntersectionTrace::Nurbs(_),
            TransmittedNurbsIntersectionTrace::Nurbs(_)
        ]
    ) {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    certify_transmitted_nurbs_intersection_residuals_impl(
        carrier, traces, pcurves, metadata, tolerance,
    )
}

/// Certify a transmitted chart between one constant normal offset of an
/// original NURBS basis and either one exact plane field or one direct
/// original NURBS source, in either operand order.
///
/// The offset trace uses the same fixed source point/first-partial enclosure
/// as the direct NURBS proof. Each proof rectangle additionally encloses
/// `du x dv`, proves its norm strictly positive, and outwardly divides to a
/// unit-normal interval before applying the signed displacement. No sampled
/// normal or second-partial estimate is proof evidence.
pub fn certify_transmitted_offset_nurbs_intersection_residuals(
    carrier: NurbsCurve,
    traces: [TransmittedNurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    metadata: TransmittedIntersectionChartMetadata,
    tolerance: f64,
) -> Result<TransmittedNurbsIntersectionCertificate, IntersectionCertificateError> {
    if !matches!(
        &traces,
        [
            TransmittedNurbsIntersectionTrace::OffsetNurbs(_),
            TransmittedNurbsIntersectionTrace::Nurbs(_)
        ] | [
            TransmittedNurbsIntersectionTrace::Nurbs(_),
            TransmittedNurbsIntersectionTrace::OffsetNurbs(_)
        ] | [
            TransmittedNurbsIntersectionTrace::OffsetNurbs(_),
            TransmittedNurbsIntersectionTrace::Plane(_)
        ] | [
            TransmittedNurbsIntersectionTrace::Plane(_),
            TransmittedNurbsIntersectionTrace::OffsetNurbs(_)
        ]
    ) {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    certify_transmitted_nurbs_intersection_residuals_impl(
        carrier, traces, pcurves, metadata, tolerance,
    )
}

/// Certify the strictly bounded canonical finite-open three-sample
/// Offset(NURBS)/Offset(NURBS) transmitted family.
///
/// The degree-2 carrier and pcurves are deterministic common-parameter
/// interpolants through the exact transmitted tuples. Those rounded curves
/// define the candidate geometry only; two independent whole-range
/// original-source interval residuals are the proof evidence.
pub fn certify_transmitted_quadratic_dual_offset_nurbs_intersection_residuals(
    carrier: NurbsCurve,
    traces: [TransmittedNurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    positions: [Vec3; 3],
    canonicalized_pcurve_points: [[Vec2; 3]; 2],
    metadata: TransmittedIntersectionChartMetadata,
    tolerance: f64,
) -> Result<TransmittedNurbsIntersectionCertificate, IntersectionCertificateError> {
    let carrier_range = carrier.param_range();
    let expected_knots = [0.0, 0.0, 0.0, 2.0, 2.0, 2.0];
    if !carrier_range.is_finite() || carrier_range != ParamRange::new(0.0, 2.0) {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if !matches!(
        &traces,
        [
            TransmittedNurbsIntersectionTrace::OffsetNurbs(_),
            TransmittedNurbsIntersectionTrace::OffsetNurbs(_)
        ]
    ) {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    if carrier.degree() != 2
        || carrier.weights().is_some()
        || carrier.points().len() != 3
        || carrier.knots().as_slice() != expected_knots
    {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "dual-offset quadratic carrier must be the canonical three-sample clamped interpolant",
            },
        );
    }
    if carrier
        .points()
        .iter()
        .copied()
        .any(|point| !finite_vec3(point))
        || positions.into_iter().any(|point| !finite_vec3(point))
    {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    let expected_carrier_points = quadratic_interpolant_controls3(positions);
    if carrier.points() != expected_carrier_points {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "dual-offset quadratic carrier must be the canonical three-sample clamped interpolant",
            },
        );
    }
    if positions[0] == positions[1] || positions[0] == positions[2] || positions[1] == positions[2]
    {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "dual-offset quadratic carrier samples must be pairwise distinct",
            },
        );
    }
    let witness_slack = 16_384.0
        * f64::EPSILON
        * positions
            .iter()
            .flat_map(|point| point.to_array())
            .map(f64::abs)
            .fold(1.0_f64, f64::max);
    if quadratic_interpolation_witness_bound3(&carrier, positions) > witness_slack {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "quadratic carrier does not interpolate the exact transmitted positions",
            },
        );
    }

    let offsets = traces.each_ref().map(|trace| {
        trace
            .as_offset_nurbs()
            .expect("dual-offset family was checked before quadratic proof")
    });
    let mut residual_bounds = [0.0; 2];
    for index in 0..2 {
        let trace = paired_trace(index);
        let pcurve = &pcurves[index];
        if pcurve.degree() != 2
            || pcurve.weights().is_some()
            || pcurve.points().len() != 3
            || pcurve.knots().as_slice() != expected_knots
            || pcurve.param_range() != carrier_range
        {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "dual-offset quadratic pcurve must share the canonical carrier basis",
                },
            );
        }
        if pcurve
            .points()
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
            || !offsets[index].signed_distance().is_finite()
            || canonicalized_pcurve_points[index]
                .into_iter()
                .any(|point| !point.x.is_finite() || !point.y.is_finite())
            || offsets[index]
                .basis()
                .points()
                .iter()
                .copied()
                .any(|point| !finite_vec3(point))
            || offsets[index]
                .basis()
                .weights()
                .is_some_and(|weights| weights.iter().any(|weight| !weight.is_finite()))
        {
            return Err(IntersectionCertificateError::NonFiniteGeometry);
        }
        let expected_pcurve_points =
            quadratic_interpolant_controls2(canonicalized_pcurve_points[index]);
        if pcurve.points() != expected_pcurve_points {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "dual-offset quadratic pcurve must share the canonical carrier basis",
                },
            );
        }
        if canonicalized_pcurve_points[index][0] == canonicalized_pcurve_points[index][1]
            || canonicalized_pcurve_points[index][0] == canonicalized_pcurve_points[index][2]
            || canonicalized_pcurve_points[index][1] == canonicalized_pcurve_points[index][2]
        {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "dual-offset quadratic pcurve samples must be pairwise distinct",
                },
            );
        }
        let parameter_scale = canonicalized_pcurve_points[index]
            .iter()
            .flat_map(|point| [point.x.abs(), point.y.abs()])
            .fold(1.0_f64, f64::max);
        if quadratic_interpolation_witness_bound2(pcurve, canonicalized_pcurve_points[index])
            > 16_384.0 * f64::EPSILON * parameter_scale
        {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "quadratic pcurve does not interpolate the exact transmitted UV tuples",
                },
            );
        }
        let bound = transmitted_quadratic_offset_trace_residual_bound(
            &carrier,
            offsets[index],
            pcurve,
            trace,
        )?;
        if bound > tolerance {
            return Err(IntersectionCertificateError::ResidualExceedsTolerance {
                trace,
                residual_bound: bound,
                tolerance,
            });
        }
        residual_bounds[index] = bound;
    }

    Ok(TransmittedNurbsIntersectionCertificate {
        carrier,
        carrier_range,
        carrier_period: None,
        traces,
        pcurves,
        residual_bounds,
        tolerance,
        metadata,
        proof_depth: TRANSMITTED_NURBS_TRACE_PROOF_DEPTH,
        quadratic_witnesses: Some(TransmittedQuadraticInterpolationWitnesses {
            positions,
            canonicalized_pcurve_points,
        }),
        cubic_witnesses: None,
    })
}

/// Certify the strictly bounded canonical finite-open four-sample
/// Offset(NURBS)/Offset(NURBS) transmitted family.
///
/// The degree-3 carrier and pcurves are the unique common-parameter cubic
/// interpolants through the exact transmitted tuples at parameters
/// `0, 1, 2, 3`. Those rounded curves define the candidate geometry only;
/// two independent whole-range original-source interval residuals are the
/// proof evidence.
pub fn certify_transmitted_cubic_dual_offset_nurbs_intersection_residuals(
    carrier: NurbsCurve,
    traces: [TransmittedNurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    positions: [Vec3; 4],
    canonicalized_pcurve_points: [[Vec2; 4]; 2],
    metadata: TransmittedIntersectionChartMetadata,
    tolerance: f64,
) -> Result<TransmittedNurbsIntersectionCertificate, IntersectionCertificateError> {
    let carrier_range = carrier.param_range();
    let expected_knots = [0.0, 0.0, 0.0, 0.0, 3.0, 3.0, 3.0, 3.0];
    if !carrier_range.is_finite() || carrier_range != ParamRange::new(0.0, 3.0) {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if !matches!(
        &traces,
        [
            TransmittedNurbsIntersectionTrace::OffsetNurbs(_),
            TransmittedNurbsIntersectionTrace::OffsetNurbs(_)
        ]
    ) {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    if carrier.degree() != 3
        || carrier.weights().is_some()
        || carrier.points().len() != 4
        || carrier.knots().as_slice() != expected_knots
    {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "dual-offset cubic carrier must be the canonical four-sample clamped interpolant",
            },
        );
    }
    if carrier
        .points()
        .iter()
        .copied()
        .any(|point| !finite_vec3(point))
        || positions.into_iter().any(|point| !finite_vec3(point))
    {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }
    let expected_carrier_points = cubic_interpolant_controls3(positions);
    if carrier.points() != expected_carrier_points {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "dual-offset cubic carrier must be the canonical four-sample clamped interpolant",
            },
        );
    }
    if (0..4).any(|first| (first + 1..4).any(|second| positions[first] == positions[second])) {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "dual-offset cubic carrier samples must be pairwise distinct",
            },
        );
    }
    let witness_slack = 16_384.0
        * f64::EPSILON
        * positions
            .iter()
            .flat_map(|point| point.to_array())
            .map(f64::abs)
            .fold(1.0_f64, f64::max);
    if cubic_interpolation_witness_bound3(&carrier, positions) > witness_slack {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "cubic carrier does not interpolate the exact transmitted positions",
            },
        );
    }

    let offsets = traces.each_ref().map(|trace| {
        trace
            .as_offset_nurbs()
            .expect("dual-offset family was checked before cubic proof")
    });
    let mut residual_bounds = [0.0; 2];
    for index in 0..2 {
        let trace = paired_trace(index);
        let pcurve = &pcurves[index];
        if pcurve.degree() != 3
            || pcurve.weights().is_some()
            || pcurve.points().len() != 4
            || pcurve.knots().as_slice() != expected_knots
            || pcurve.param_range() != carrier_range
        {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "dual-offset cubic pcurve must share the canonical carrier basis",
                },
            );
        }
        if pcurve
            .points()
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
            || !offsets[index].signed_distance().is_finite()
            || canonicalized_pcurve_points[index]
                .into_iter()
                .any(|point| !point.x.is_finite() || !point.y.is_finite())
            || offsets[index]
                .basis()
                .points()
                .iter()
                .copied()
                .any(|point| !finite_vec3(point))
            || offsets[index]
                .basis()
                .weights()
                .is_some_and(|weights| weights.iter().any(|weight| !weight.is_finite()))
        {
            return Err(IntersectionCertificateError::NonFiniteGeometry);
        }
        let expected_pcurve_points =
            cubic_interpolant_controls2(canonicalized_pcurve_points[index]);
        if pcurve.points() != expected_pcurve_points {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "dual-offset cubic pcurve must share the canonical carrier basis",
                },
            );
        }
        if (0..4).any(|first| {
            (first + 1..4).any(|second| {
                canonicalized_pcurve_points[index][first]
                    == canonicalized_pcurve_points[index][second]
            })
        }) {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "dual-offset cubic pcurve samples must be pairwise distinct",
                },
            );
        }
        let parameter_scale = canonicalized_pcurve_points[index]
            .iter()
            .flat_map(|point| [point.x.abs(), point.y.abs()])
            .fold(1.0_f64, f64::max);
        if cubic_interpolation_witness_bound2(pcurve, canonicalized_pcurve_points[index])
            > 16_384.0 * f64::EPSILON * parameter_scale
        {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "cubic pcurve does not interpolate the exact transmitted UV tuples",
                },
            );
        }
        let bound =
            transmitted_cubic_offset_trace_residual_bound(&carrier, offsets[index], pcurve, trace)?;
        if bound > tolerance {
            return Err(IntersectionCertificateError::ResidualExceedsTolerance {
                trace,
                residual_bound: bound,
                tolerance,
            });
        }
        residual_bounds[index] = bound;
    }

    Ok(TransmittedNurbsIntersectionCertificate {
        carrier,
        carrier_range,
        carrier_period: None,
        traces,
        pcurves,
        residual_bounds,
        tolerance,
        metadata,
        proof_depth: TRANSMITTED_NURBS_TRACE_PROOF_DEPTH,
        quadratic_witnesses: None,
        cubic_witnesses: Some(TransmittedCubicInterpolationWitnesses {
            positions,
            canonicalized_pcurve_points,
        }),
    })
}

fn quadratic_interpolant_controls3(samples: [Vec3; 3]) -> [Vec3; 3] {
    [
        samples[0],
        samples[1] * 2.0 - (samples[0] + samples[2]) * 0.5,
        samples[2],
    ]
}

fn quadratic_interpolant_controls2(samples: [Vec2; 3]) -> [Vec2; 3] {
    [
        samples[0],
        samples[1] * 2.0 - (samples[0] + samples[2]) * 0.5,
        samples[2],
    ]
}

fn cubic_interpolant_controls3(samples: [Vec3; 4]) -> [Vec3; 4] {
    let first = samples[1] * 27.0 - samples[0] * 8.0 - samples[3];
    let second = samples[2] * 27.0 - samples[0] - samples[3] * 8.0;
    [
        samples[0],
        (first * 2.0 - second) / 18.0,
        (second * 2.0 - first) / 18.0,
        samples[3],
    ]
}

fn cubic_interpolant_controls2(samples: [Vec2; 4]) -> [Vec2; 4] {
    let first = samples[1] * 27.0 - samples[0] * 8.0 - samples[3];
    let second = samples[2] * 27.0 - samples[0] - samples[3] * 8.0;
    [
        samples[0],
        (first * 2.0 - second) / 18.0,
        (second * 2.0 - first) / 18.0,
        samples[3],
    ]
}

fn quadratic_interval_coefficients3(points: &[Vec3]) -> [[Interval; 3]; 3] {
    let point = |value: Vec3| value.to_array().map(Interval::point);
    let first = point(points[0]);
    let middle = point(points[1]);
    let last = point(points[2]);
    [
        first,
        core::array::from_fn(|axis| (middle[axis] - first[axis]) * Interval::point(2.0)),
        core::array::from_fn(|axis| first[axis] - middle[axis] * Interval::point(2.0) + last[axis]),
    ]
}

fn quadratic_interval_coefficients2(points: &[Vec2]) -> [[Interval; 2]; 3] {
    let point = |value: Vec2| [Interval::point(value.x), Interval::point(value.y)];
    let first = point(points[0]);
    let middle = point(points[1]);
    let last = point(points[2]);
    [
        first,
        core::array::from_fn(|axis| (middle[axis] - first[axis]) * Interval::point(2.0)),
        core::array::from_fn(|axis| first[axis] - middle[axis] * Interval::point(2.0) + last[axis]),
    ]
}

fn cubic_interval_coefficients3(points: &[Vec3]) -> [[Interval; 3]; 4] {
    let point = |value: Vec3| value.to_array().map(Interval::point);
    let points = [
        point(points[0]),
        point(points[1]),
        point(points[2]),
        point(points[3]),
    ];
    [
        points[0],
        core::array::from_fn(|axis| (points[1][axis] - points[0][axis]) * Interval::point(3.0)),
        core::array::from_fn(|axis| {
            (points[0][axis] - points[1][axis] * Interval::point(2.0) + points[2][axis])
                * Interval::point(3.0)
        }),
        core::array::from_fn(|axis| {
            -points[0][axis] + points[1][axis] * Interval::point(3.0)
                - points[2][axis] * Interval::point(3.0)
                + points[3][axis]
        }),
    ]
}

fn cubic_interval_coefficients2(points: &[Vec2]) -> [[Interval; 2]; 4] {
    let point = |value: Vec2| [Interval::point(value.x), Interval::point(value.y)];
    let points = [
        point(points[0]),
        point(points[1]),
        point(points[2]),
        point(points[3]),
    ];
    [
        points[0],
        core::array::from_fn(|axis| (points[1][axis] - points[0][axis]) * Interval::point(3.0)),
        core::array::from_fn(|axis| {
            (points[0][axis] - points[1][axis] * Interval::point(2.0) + points[2][axis])
                * Interval::point(3.0)
        }),
        core::array::from_fn(|axis| {
            -points[0][axis] + points[1][axis] * Interval::point(3.0)
                - points[2][axis] * Interval::point(3.0)
                + points[3][axis]
        }),
    ]
}

fn quadratic_interpolation_witness_bound3(curve: &NurbsCurve, samples: [Vec3; 3]) -> f64 {
    let coefficients = quadratic_interval_coefficients3(curve.points());
    [0.0, 0.5, 1.0]
        .into_iter()
        .zip(samples)
        .map(|(parameter, sample)| {
            let parameter = Interval::point(parameter);
            let sample = sample.to_array();
            let squared = (0..3).fold(Interval::point(0.0), |sum, axis| {
                let value = coefficients[0][axis]
                    + coefficients[1][axis] * parameter
                    + coefficients[2][axis] * parameter.square();
                sum + (value - Interval::point(sample[axis])).square()
            });
            squared.sqrt().map_or(f64::INFINITY, |value| value.hi())
        })
        .fold(0.0_f64, f64::max)
}

fn quadratic_interpolation_witness_bound2(curve: &NurbsCurve2d, samples: [Vec2; 3]) -> f64 {
    let coefficients = quadratic_interval_coefficients2(curve.points());
    [0.0, 0.5, 1.0]
        .into_iter()
        .zip(samples)
        .map(|(parameter, sample)| {
            let parameter = Interval::point(parameter);
            let sample = [sample.x, sample.y];
            (0..2)
                .map(|axis| {
                    let value = coefficients[0][axis]
                        + coefficients[1][axis] * parameter
                        + coefficients[2][axis] * parameter.square();
                    let residual = value - Interval::point(sample[axis]);
                    residual.lo().abs().max(residual.hi().abs())
                })
                .fold(0.0_f64, f64::max)
        })
        .fold(0.0_f64, f64::max)
}

fn cubic_interpolation_witness_bound3(curve: &NurbsCurve, samples: [Vec3; 4]) -> f64 {
    let coefficients = cubic_interval_coefficients3(curve.points());
    let third = Interval::point(1.0)
        .checked_div(Interval::point(3.0))
        .expect("three is nonzero");
    [
        Interval::point(0.0),
        third,
        third * Interval::point(2.0),
        Interval::point(1.0),
    ]
    .into_iter()
    .zip(samples)
    .map(|(parameter, sample)| {
        let parameter_squared = parameter.square();
        let parameter_cubed = parameter_squared * parameter;
        let sample = sample.to_array();
        let squared = (0..3).fold(Interval::point(0.0), |sum, axis| {
            let value = coefficients[0][axis]
                + coefficients[1][axis] * parameter
                + coefficients[2][axis] * parameter_squared
                + coefficients[3][axis] * parameter_cubed;
            sum + (value - Interval::point(sample[axis])).square()
        });
        squared.sqrt().map_or(f64::INFINITY, |value| value.hi())
    })
    .fold(0.0_f64, f64::max)
}

fn cubic_interpolation_witness_bound2(curve: &NurbsCurve2d, samples: [Vec2; 4]) -> f64 {
    let coefficients = cubic_interval_coefficients2(curve.points());
    let third = Interval::point(1.0)
        .checked_div(Interval::point(3.0))
        .expect("three is nonzero");
    [
        Interval::point(0.0),
        third,
        third * Interval::point(2.0),
        Interval::point(1.0),
    ]
    .into_iter()
    .zip(samples)
    .map(|(parameter, sample)| {
        let parameter_squared = parameter.square();
        let parameter_cubed = parameter_squared * parameter;
        let sample = [sample.x, sample.y];
        (0..2)
            .map(|axis| {
                let value = coefficients[0][axis]
                    + coefficients[1][axis] * parameter
                    + coefficients[2][axis] * parameter_squared
                    + coefficients[3][axis] * parameter_cubed;
                let residual = value - Interval::point(sample[axis]);
                residual.lo().abs().max(residual.hi().abs())
            })
            .fold(0.0_f64, f64::max)
    })
    .fold(0.0_f64, f64::max)
}

fn intersect_polynomial_pcurve_enclosures(
    taylor: Interval,
    control_hull: ParamRange,
    trace: PairedTrace,
    inconsistent_reason: &'static str,
) -> Result<ParamRange, IntersectionCertificateError> {
    let lo = taylor.lo().max(control_hull.lo);
    let hi = taylor.hi().min(control_hull.hi);
    if !lo.is_finite() || !hi.is_finite() {
        return Err(IntersectionCertificateError::NonFiniteResidualBound { trace });
    }
    if lo > hi {
        return Err(
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace,
                reason: inconsistent_reason,
            },
        );
    }
    Ok(ParamRange::new(lo, hi))
}

fn polynomial_pcurve_control_hull(pcurve: &NurbsCurve2d) -> [ParamRange; 2] {
    core::array::from_fn(|axis| {
        let coordinate = |point: &Vec2| if axis == 0 { point.x } else { point.y };
        let lo = pcurve
            .points()
            .iter()
            .map(coordinate)
            .fold(f64::INFINITY, f64::min);
        let hi = pcurve
            .points()
            .iter()
            .map(coordinate)
            .fold(f64::NEG_INFINITY, f64::max);
        ParamRange::new(lo, hi)
    })
}

#[cfg(test)]
mod polynomial_enclosure_tests {
    use super::*;

    #[test]
    fn disjoint_taylor_and_control_hull_enclosures_fail_typed_without_panicking() {
        assert_eq!(
            intersect_polynomial_pcurve_enclosures(
                Interval::new(2.0, 3.0),
                ParamRange::new(0.0, 1.0),
                PairedTrace::Second,
                "dual-offset cubic pcurve enclosure is inconsistent with its polynomial control hull",
            ),
            Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace: PairedTrace::Second,
                    reason: "dual-offset cubic pcurve enclosure is inconsistent with its polynomial control hull",
                }
            )
        );
    }
}

fn transmitted_cubic_offset_trace_residual_bound(
    carrier: &NurbsCurve,
    offset: &TransmittedOffsetNurbsTrace,
    pcurve: &NurbsCurve2d,
    trace: PairedTrace,
) -> Result<f64, IntersectionCertificateError> {
    let carrier_coefficients = cubic_interval_coefficients3(carrier.points());
    let pcurve_coefficients = cubic_interval_coefficients2(pcurve.points());
    let pcurve_control_hull = polynomial_pcurve_control_hull(pcurve);
    let domains = [
        offset.basis().knots(Dir::U).domain(),
        offset.basis().knots(Dir::V).domain(),
    ];
    let subdivisions = 1_usize << TRANSMITTED_NURBS_TRACE_PROOF_DEPTH;
    let finite = |interval| {
        finite_interval(interval)
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })
    };
    let mut bound = 0.0_f64;
    for chart_span in 0..3 {
        for subdivision in 0..subdivisions {
            let fraction_lo = (chart_span as f64 + subdivision as f64 / subdivisions as f64) / 3.0;
            let fraction_hi =
                (chart_span as f64 + (subdivision + 1) as f64 / subdivisions as f64) / 3.0;
            let fraction_mid = fraction_lo + 0.5 * (fraction_hi - fraction_lo);
            let delta = Interval::new(fraction_lo - fraction_mid, fraction_hi - fraction_mid);
            let delta_squared = delta.square();
            let delta_cubed = delta_squared * delta;
            let mid = Interval::point(fraction_mid);
            let mid_squared = mid.square();
            let mid_cubed = mid_squared * mid;
            let carrier_center: [Interval; 3] = core::array::from_fn(|axis| {
                carrier_coefficients[0][axis]
                    + carrier_coefficients[1][axis] * mid
                    + carrier_coefficients[2][axis] * mid_squared
                    + carrier_coefficients[3][axis] * mid_cubed
            });
            let carrier_direction: [Interval; 3] = core::array::from_fn(|axis| {
                carrier_coefficients[1][axis]
                    + carrier_coefficients[2][axis] * Interval::point(2.0 * fraction_mid)
                    + carrier_coefficients[3][axis]
                        * Interval::point(3.0 * fraction_mid * fraction_mid)
            });
            let carrier_quadratic: [Interval; 3] = core::array::from_fn(|axis| {
                carrier_coefficients[2][axis]
                    + carrier_coefficients[3][axis] * Interval::point(3.0 * fraction_mid)
            });
            let uv_center: [Interval; 2] = core::array::from_fn(|axis| {
                pcurve_coefficients[0][axis]
                    + pcurve_coefficients[1][axis] * mid
                    + pcurve_coefficients[2][axis] * mid_squared
                    + pcurve_coefficients[3][axis] * mid_cubed
            });
            let uv_direction: [Interval; 2] = core::array::from_fn(|axis| {
                pcurve_coefficients[1][axis]
                    + pcurve_coefficients[2][axis] * Interval::point(2.0 * fraction_mid)
                    + pcurve_coefficients[3][axis]
                        * Interval::point(3.0 * fraction_mid * fraction_mid)
            });
            let uv_quadratic: [Interval; 2] = core::array::from_fn(|axis| {
                pcurve_coefficients[2][axis]
                    + pcurve_coefficients[3][axis] * Interval::point(3.0 * fraction_mid)
            });
            let mut uv_box = [ParamRange::new(0.0, 0.0); 2];
            for axis in 0..2 {
                let taylor = uv_center[axis]
                    + uv_direction[axis] * delta
                    + uv_quadratic[axis] * delta_squared
                    + pcurve_coefficients[3][axis] * delta_cubed;
                uv_box[axis] = intersect_polynomial_pcurve_enclosures(
                    taylor,
                    pcurve_control_hull[axis],
                    trace,
                    "dual-offset cubic pcurve enclosure is inconsistent with its polynomial control hull",
                )?;
            }
            if (0..2).any(|axis| {
                uv_box[axis].lo < domains[axis].lo || uv_box[axis].hi > domains[axis].hi
            }) {
                return Err(
                    IntersectionCertificateError::UnsupportedTraceParameterization {
                        trace,
                        reason: "dual-offset cubic pcurve leaves the original source domain",
                    },
                );
            }
            let source_center = uv_box.map(|range| range.lo + 0.5 * range.width());
            let enclosure = offset
                .basis()
                .source_differential_enclosure(uv_box, source_center)
                .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
            let position = enclosure.position();
            let derivative_u = enclosure.derivative_u();
            let derivative_v = enclosure.derivative_v();
            let normal = interval_unit_normal(derivative_u, derivative_v, trace)?;
            let uv_center_delta = [
                uv_center[0] - Interval::point(source_center[0]),
                uv_center[1] - Interval::point(source_center[1]),
            ];
            let mut squared_norm = Interval::point(0.0);
            for axis in 0..3 {
                let residual_center = finite(
                    finite(
                        finite(carrier_center[axis] - position[axis])?
                            - finite(derivative_u[axis] * uv_center_delta[0])?,
                    )? - finite(derivative_v[axis] * uv_center_delta[1])?,
                )?;
                let residual_direction = finite(
                    finite(
                        carrier_direction[axis] - finite(derivative_u[axis] * uv_direction[0])?,
                    )? - finite(derivative_v[axis] * uv_direction[1])?,
                )?;
                let residual_quadratic = finite(
                    finite(
                        carrier_quadratic[axis] - finite(derivative_u[axis] * uv_quadratic[0])?,
                    )? - finite(derivative_v[axis] * uv_quadratic[1])?,
                )?;
                let residual_cubic = finite(
                    finite(
                        carrier_coefficients[3][axis]
                            - finite(derivative_u[axis] * pcurve_coefficients[3][0])?,
                    )? - finite(derivative_v[axis] * pcurve_coefficients[3][1])?,
                )?;
                let residual = finite(
                    finite(
                        finite(residual_center + finite(residual_direction * delta)?)?
                            + finite(residual_quadratic * delta_squared)?,
                    )? + finite(residual_cubic * delta_cubed)?,
                )?;
                let residual = finite(
                    residual - finite(Interval::point(offset.signed_distance()) * normal[axis])?,
                )?;
                squared_norm = finite(squared_norm + finite(residual.square())?)?;
            }
            let local = finite(
                squared_norm
                    .sqrt()
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
            )?
            .hi();
            bound = bound.max(local);
        }
    }
    Ok(bound)
}

fn transmitted_quadratic_offset_trace_residual_bound(
    carrier: &NurbsCurve,
    offset: &TransmittedOffsetNurbsTrace,
    pcurve: &NurbsCurve2d,
    trace: PairedTrace,
) -> Result<f64, IntersectionCertificateError> {
    let carrier_coefficients = quadratic_interval_coefficients3(carrier.points());
    let pcurve_coefficients = quadratic_interval_coefficients2(pcurve.points());
    let domains = [
        offset.basis().knots(Dir::U).domain(),
        offset.basis().knots(Dir::V).domain(),
    ];
    let subdivisions = 1_usize << TRANSMITTED_NURBS_TRACE_PROOF_DEPTH;
    let finite = |interval| {
        finite_interval(interval)
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })
    };
    let mut bound = 0.0_f64;
    for chart_span in 0..2 {
        for subdivision in 0..subdivisions {
            let fraction_lo = (chart_span as f64 + subdivision as f64 / subdivisions as f64) * 0.5;
            let fraction_hi =
                (chart_span as f64 + (subdivision + 1) as f64 / subdivisions as f64) * 0.5;
            let fraction_mid = fraction_lo + 0.5 * (fraction_hi - fraction_lo);
            let delta = Interval::new(fraction_lo - fraction_mid, fraction_hi - fraction_mid);
            let delta_squared = delta.square();
            let mid = Interval::point(fraction_mid);
            let mid_squared = mid.square();
            let carrier_center: [Interval; 3] = core::array::from_fn(|axis| {
                carrier_coefficients[0][axis]
                    + carrier_coefficients[1][axis] * mid
                    + carrier_coefficients[2][axis] * mid_squared
            });
            let carrier_direction: [Interval; 3] = core::array::from_fn(|axis| {
                carrier_coefficients[1][axis]
                    + carrier_coefficients[2][axis] * Interval::point(2.0 * fraction_mid)
            });
            let uv_center: [Interval; 2] = core::array::from_fn(|axis| {
                pcurve_coefficients[0][axis]
                    + pcurve_coefficients[1][axis] * mid
                    + pcurve_coefficients[2][axis] * mid_squared
            });
            let uv_direction: [Interval; 2] = core::array::from_fn(|axis| {
                pcurve_coefficients[1][axis]
                    + pcurve_coefficients[2][axis] * Interval::point(2.0 * fraction_mid)
            });
            let uv_box = core::array::from_fn(|axis| {
                let enclosure = uv_center[axis]
                    + uv_direction[axis] * delta
                    + pcurve_coefficients[2][axis] * delta_squared;
                ParamRange::new(enclosure.lo(), enclosure.hi())
            });
            if (0..2).any(|axis| {
                uv_box[axis].lo < domains[axis].lo || uv_box[axis].hi > domains[axis].hi
            }) {
                return Err(
                    IntersectionCertificateError::UnsupportedTraceParameterization {
                        trace,
                        reason: "dual-offset quadratic pcurve leaves the original source domain",
                    },
                );
            }
            let source_center = uv_box.map(|range| range.lo + 0.5 * range.width());
            let enclosure = offset
                .basis()
                .source_differential_enclosure(uv_box, source_center)
                .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
            let position = enclosure.position();
            let derivative_u = enclosure.derivative_u();
            let derivative_v = enclosure.derivative_v();
            let normal = interval_unit_normal(derivative_u, derivative_v, trace)?;
            let uv_center_delta = [
                uv_center[0] - Interval::point(source_center[0]),
                uv_center[1] - Interval::point(source_center[1]),
            ];
            let mut squared_norm = Interval::point(0.0);
            for axis in 0..3 {
                let residual_center = finite(
                    finite(
                        finite(carrier_center[axis] - position[axis])?
                            - finite(derivative_u[axis] * uv_center_delta[0])?,
                    )? - finite(derivative_v[axis] * uv_center_delta[1])?,
                )?;
                let residual_direction = finite(
                    finite(
                        carrier_direction[axis] - finite(derivative_u[axis] * uv_direction[0])?,
                    )? - finite(derivative_v[axis] * uv_direction[1])?,
                )?;
                let residual_quadratic = finite(
                    finite(
                        carrier_coefficients[2][axis]
                            - finite(derivative_u[axis] * pcurve_coefficients[2][0])?,
                    )? - finite(derivative_v[axis] * pcurve_coefficients[2][1])?,
                )?;
                let residual = finite(
                    finite(residual_center + finite(residual_direction * delta)?)?
                        + finite(residual_quadratic * delta_squared)?,
                )?;
                let residual = finite(
                    residual - finite(Interval::point(offset.signed_distance()) * normal[axis])?,
                )?;
                squared_norm = finite(squared_norm + finite(residual.square())?)?;
            }
            let local = finite(
                squared_norm
                    .sqrt()
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
            )?
            .hi();
            bound = bound.max(local);
        }
    }
    Ok(bound)
}

fn certify_transmitted_nurbs_intersection_residuals_impl(
    carrier: NurbsCurve,
    traces: [TransmittedNurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    metadata: TransmittedIntersectionChartMetadata,
    tolerance: f64,
) -> Result<TransmittedNurbsIntersectionCertificate, IntersectionCertificateError> {
    let carrier_range = carrier.param_range();
    if !carrier_range.is_finite() || carrier_range.width() <= 0.0 {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if carrier.degree() != 1
        || carrier.weights().is_some()
        || !carrier.knots().is_clamped()
        || carrier.points().len() < 2
    {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "transmitted chart carrier must be open clamped polynomial degree 1",
            },
        );
    }
    if carrier
        .points()
        .iter()
        .copied()
        .any(|point| !finite_vec3(point))
        || traces.iter().any(|trace| match trace {
            TransmittedNurbsIntersectionTrace::Plane(plane) => !finite_plane(*plane),
            TransmittedNurbsIntersectionTrace::Sphere(_) => true,
            TransmittedNurbsIntersectionTrace::Nurbs(surface) => {
                surface
                    .points()
                    .iter()
                    .copied()
                    .any(|point| !finite_vec3(point))
                    || surface
                        .weights()
                        .is_some_and(|weights| weights.iter().any(|weight| !weight.is_finite()))
            }
            TransmittedNurbsIntersectionTrace::OffsetNurbs(trace) => {
                !trace.signed_distance.is_finite()
                    || trace
                        .basis
                        .points()
                        .iter()
                        .copied()
                        .any(|point| !finite_vec3(point))
                    || trace
                        .basis
                        .weights()
                        .is_some_and(|weights| weights.iter().any(|weight| !weight.is_finite()))
            }
        })
    {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let carrier_knots = carrier.knots().as_slice();
    let mut residual_bounds = [0.0; 2];
    for index in 0..2 {
        let trace = paired_trace(index);
        let pcurve = &pcurves[index];
        if pcurve.degree() != 1
            || pcurve.weights().is_some()
            || !pcurve.knots().is_clamped()
            || pcurve.knots().as_slice() != carrier_knots
            || pcurve.points().len() != carrier.points().len()
            || pcurve.param_range() != carrier_range
        {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "transmitted pcurve must share the carrier's open clamped polynomial degree-1 basis",
                },
            );
        }
        if pcurve
            .points()
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
        {
            return Err(IntersectionCertificateError::NonFiniteGeometry);
        }

        let bound = match &traces[index] {
            TransmittedNurbsIntersectionTrace::Plane(surface) => {
                let mut bound = 0.0_f64;
                for (&point, &uv) in carrier.points().iter().zip(pcurve.points()) {
                    let control_bound = transmitted_plane_control_residual_bound(
                        point,
                        *surface,
                        Vec2::new(uv.x, uv.y),
                    )
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
                    bound = bound.max(control_bound);
                }
                bound
            }
            TransmittedNurbsIntersectionTrace::Sphere(_) => {
                return Err(IntersectionCertificateError::InvalidTraceFamily);
            }
            TransmittedNurbsIntersectionTrace::Nurbs(surface) => {
                transmitted_nurbs_trace_residual_bound(&carrier, surface, None, pcurve, trace)?
            }
            TransmittedNurbsIntersectionTrace::OffsetNurbs(offset) => {
                transmitted_nurbs_trace_residual_bound(
                    &carrier,
                    &offset.basis,
                    Some(offset.signed_distance),
                    pcurve,
                    trace,
                )?
            }
        };
        if bound > tolerance {
            return Err(IntersectionCertificateError::ResidualExceedsTolerance {
                trace,
                residual_bound: bound,
                tolerance,
            });
        }
        residual_bounds[index] = bound;
    }

    Ok(TransmittedNurbsIntersectionCertificate {
        carrier,
        carrier_range,
        carrier_period: None,
        traces,
        pcurves,
        residual_bounds,
        tolerance,
        metadata,
        proof_depth: TRANSMITTED_NURBS_TRACE_PROOF_DEPTH,
        quadratic_witnesses: None,
        cubic_witnesses: None,
    })
}

fn transmitted_nurbs_trace_residual_bound(
    carrier: &NurbsCurve,
    surface: &NurbsSurface,
    signed_offset: Option<f64>,
    pcurve: &NurbsCurve2d,
    trace: PairedTrace,
) -> Result<f64, IntersectionCertificateError> {
    let domains = [
        surface.knots(Dir::U).domain(),
        surface.knots(Dir::V).domain(),
    ];
    let periodicity = surface.periodicity();
    let last_point = pcurve.points().len() - 1;
    // Equal-limit import may unwrap only its first or last seam value by one
    // exact period. Admit the resulting decimal-roundoff overhang, while an
    // interior or material out-of-domain value remains a noncanonical range.
    for (point_index, point) in pcurve.points().iter().enumerate() {
        for axis in 0..2 {
            let value = if axis == 0 { point.x } else { point.y };
            if domains[axis].contains(value) {
                continue;
            }
            let Some(period) = periodicity[axis] else {
                return Err(
                    IntersectionCertificateError::UnsupportedTraceParameterization {
                        trace,
                        reason: "transmitted NURBS pcurve leaves the source surface domain",
                    },
                );
            };
            let scale = domains[axis]
                .lo
                .abs()
                .max(domains[axis].hi.abs())
                .max(period.abs())
                .max(1.0);
            let seam_slack = 16_384.0 * f64::EPSILON * scale;
            let outside = (domains[axis].lo - value)
                .max(value - domains[axis].hi)
                .max(0.0);
            if (point_index != 0 && point_index != last_point) || outside > seam_slack {
                return Err(
                    IntersectionCertificateError::UnsupportedTraceParameterization {
                        trace,
                        reason: "transmitted periodic NURBS pcurve has a noncanonical trace range",
                    },
                );
            }
        }
    }
    let subdivisions = 1_usize << TRANSMITTED_NURBS_TRACE_PROOF_DEPTH;
    let mut bound = 0.0_f64;
    for span in 0..carrier.points().len() - 1 {
        let carrier_start = carrier.points()[span];
        let carrier_end = carrier.points()[span + 1];
        let uv_start = pcurve.points()[span];
        let uv_end = pcurve.points()[span + 1];
        let carrier_direction = interval_vec3_difference(carrier_end, carrier_start);
        let uv_direction = [
            Interval::point(uv_end.x) - Interval::point(uv_start.x),
            Interval::point(uv_end.y) - Interval::point(uv_start.y),
        ];
        for subdivision in 0..subdivisions {
            let fraction_lo = subdivision as f64 / subdivisions as f64;
            let fraction_hi = (subdivision + 1) as f64 / subdivisions as f64;
            let mut cuts = vec![fraction_lo, fraction_hi];
            for axis in 0..2 {
                let Some(_) = periodicity[axis] else {
                    continue;
                };
                let start = if axis == 0 { uv_start.x } else { uv_start.y };
                let end = if axis == 0 { uv_end.x } else { uv_end.y };
                let direction = end - start;
                if direction == 0.0 {
                    continue;
                }
                for seam in [domains[axis].lo, domains[axis].hi] {
                    let crossing = (seam - start) / direction;
                    if crossing > fraction_lo && crossing < fraction_hi {
                        cuts.push(crossing);
                    }
                }
            }
            cuts.sort_by(f64::total_cmp);
            cuts.dedup();
            for pair in cuts.windows(2) {
                let piece_lo = pair[0];
                let piece_hi = pair[1];
                let piece_mid = piece_lo + 0.5 * (piece_hi - piece_lo);
                let parameter_shift = core::array::from_fn(|axis| {
                    // Split at every crossed certified seam and prove the
                    // wrapped piece against its original-source interval.
                    periodicity[axis].map_or(0.0, |period| {
                        let start = if axis == 0 { uv_start.x } else { uv_start.y };
                        let end = if axis == 0 { uv_end.x } else { uv_end.y };
                        let raw = start + (end - start) * piece_mid;
                        wrap_periodic(raw, domains[axis].lo, period) - raw
                    })
                });
                let local = transmitted_nurbs_trace_piece_residual_bound(
                    carrier_start,
                    carrier_direction,
                    uv_start,
                    uv_direction,
                    piece_lo,
                    piece_hi,
                    parameter_shift,
                    surface,
                    signed_offset,
                    domains,
                    trace,
                )?;
                bound = bound.max(local);
            }
        }
    }
    Ok(bound)
}

#[allow(clippy::too_many_arguments)]
fn transmitted_nurbs_trace_piece_residual_bound(
    carrier_start: Vec3,
    carrier_direction: [Interval; 3],
    uv_start: Vec2,
    uv_direction: [Interval; 2],
    fraction_lo: f64,
    fraction_hi: f64,
    parameter_shift: [f64; 2],
    surface: &NurbsSurface,
    signed_offset: Option<f64>,
    domains: [ParamRange; 2],
    trace: PairedTrace,
) -> Result<f64, IntersectionCertificateError> {
    let fraction_mid = fraction_lo + 0.5 * (fraction_hi - fraction_lo);
    let delta = Interval::new(fraction_lo - fraction_mid, fraction_hi - fraction_mid);
    let carrier_mid = interval_affine_vec3(carrier_start, carrier_direction, fraction_mid);
    let uv_mid = [
        Interval::point(uv_start.x)
            + uv_direction[0] * Interval::point(fraction_mid)
            + Interval::point(parameter_shift[0]),
        Interval::point(uv_start.y)
            + uv_direction[1] * Interval::point(fraction_mid)
            + Interval::point(parameter_shift[1]),
    ];
    let uv_box = core::array::from_fn(|axis| {
        let enclosure = uv_mid[axis] + uv_direction[axis] * delta;
        ParamRange::new(
            enclosure.lo().max(domains[axis].lo),
            enclosure.hi().min(domains[axis].hi),
        )
    });
    if uv_box.iter().any(|range| range.lo > range.hi) {
        return Err(
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace,
                reason: "transmitted NURBS pcurve interval leaves the source surface domain",
            },
        );
    }
    let center = uv_box.map(|range| range.lo + 0.5 * range.width());
    let enclosure = surface
        .source_differential_enclosure(uv_box, center)
        .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    let position = enclosure.position();
    let derivative_u = enclosure.derivative_u();
    let derivative_v = enclosure.derivative_v();
    let offset_normal = signed_offset
        .map(|distance| {
            interval_unit_normal(derivative_u, derivative_v, trace)
                .map(|normal| (Interval::point(distance), normal))
        })
        .transpose()?;
    let uv_center_delta = [
        uv_mid[0] - Interval::point(center[0]),
        uv_mid[1] - Interval::point(center[1]),
    ];
    let finite = |interval| {
        finite_interval(interval)
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })
    };
    let mut squared_norm = Interval::point(0.0);
    for axis in 0..3 {
        let carrier_minus_position = finite(carrier_mid[axis] - position[axis])?;
        let center_u = finite(derivative_u[axis] * uv_center_delta[0])?;
        let center_v = finite(derivative_v[axis] * uv_center_delta[1])?;
        let residual_center = finite(finite(carrier_minus_position - center_u)? - center_v)?;
        let direction_u = finite(derivative_u[axis] * uv_direction[0])?;
        let direction_v = finite(derivative_v[axis] * uv_direction[1])?;
        let residual_direction =
            finite(finite(carrier_direction[axis] - direction_u)? - direction_v)?;
        let mut residual = finite(residual_center + finite(residual_direction * delta)?)?;
        if let Some((distance, normal)) = offset_normal {
            residual = finite(residual - finite(distance * normal[axis])?)?;
        }
        squared_norm = finite(squared_norm + finite(residual.square())?)?;
    }
    let square_root = squared_norm
        .sqrt()
        .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    Ok(finite(square_root)?.hi())
}

fn interval_unit_normal(
    derivative_u: [Interval; 3],
    derivative_v: [Interval; 3],
    trace: PairedTrace,
) -> Result<[Interval; 3], IntersectionCertificateError> {
    let finite = |interval: Interval| {
        finite_interval(interval)
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })
    };
    let cross = [
        finite(
            finite(derivative_u[1] * derivative_v[2])? - finite(derivative_u[2] * derivative_v[1])?,
        )?,
        finite(
            finite(derivative_u[2] * derivative_v[0])? - finite(derivative_u[0] * derivative_v[2])?,
        )?,
        finite(
            finite(derivative_u[0] * derivative_v[1])? - finite(derivative_u[1] * derivative_v[0])?,
        )?,
    ];
    let squared_norm = finite(
        finite(finite(cross[0].square())? + finite(cross[1].square())?)?
            + finite(cross[2].square())?,
    )?;
    if squared_norm.lo() <= 0.0 {
        return Err(IntersectionCertificateError::SingularOffsetNormal {
            trace,
            squared_norm_lower_bound: squared_norm.lo(),
        });
    }
    let norm = squared_norm
        .sqrt()
        .and_then(finite_interval)
        .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    if norm.lo() <= 0.0 {
        return Err(IntersectionCertificateError::SingularOffsetNormal {
            trace,
            squared_norm_lower_bound: squared_norm.lo(),
        });
    }
    let mut normal = [Interval::point(0.0); 3];
    for axis in 0..3 {
        normal[axis] = cross[axis]
            .checked_div(norm)
            .and_then(finite_interval)
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
    }
    Ok(normal)
}

fn paired_trace(index: usize) -> PairedTrace {
    if index == 0 {
        PairedTrace::First
    } else {
        PairedTrace::Second
    }
}

fn interval_vec3_difference(end: Vec3, start: Vec3) -> [Interval; 3] {
    let end = end.to_array();
    let start = start.to_array();
    core::array::from_fn(|axis| Interval::point(end[axis]) - Interval::point(start[axis]))
}

fn interval_affine_vec3(start: Vec3, direction: [Interval; 3], fraction: f64) -> [Interval; 3] {
    let start = start.to_array();
    core::array::from_fn(|axis| {
        Interval::point(start[axis]) + direction[axis] * Interval::point(fraction)
    })
}

fn transmitted_plane_control_residual_bound(
    carrier: Vec3,
    surface: Plane,
    uv: Vec2,
) -> Option<f64> {
    let frame = surface.frame();
    let carrier = carrier.to_array();
    let origin = frame.origin().to_array();
    let axis_u = frame.x().to_array();
    let axis_v = frame.y().to_array();
    let mut squared_norm = Interval::point(0.0);
    for axis in 0..3 {
        let lifted = finite_interval(
            finite_interval(
                Interval::point(origin[axis])
                    + finite_interval(Interval::point(axis_u[axis]) * Interval::point(uv.x))?,
            )? + finite_interval(Interval::point(axis_v[axis]) * Interval::point(uv.y))?,
        )?;
        let residual = finite_interval(Interval::point(carrier[axis]) - lifted)?;
        squared_norm = finite_interval(squared_norm + finite_interval(residual.square())?)?;
    }
    finite_interval(squared_norm.sqrt()?).map(Interval::hi)
}

fn trace_residual_bound(
    carrier: Line,
    carrier_range: ParamRange,
    surface: Plane,
    pcurve: Line2d,
    parameter_map: AffineParamMap1d,
) -> Option<f64> {
    let t = finite_interval(Interval::new(carrier_range.lo, carrier_range.hi))?;
    let map_scale = Interval::point(parameter_map.scale);
    let map_offset = Interval::point(parameter_map.offset);
    let u_origin = finite_interval(
        Interval::point(pcurve.origin().x)
            + finite_interval(Interval::point(pcurve.dir().x) * map_offset)?,
    )?;
    let u_direction = finite_interval(Interval::point(pcurve.dir().x) * map_scale)?;
    let v_origin = finite_interval(
        Interval::point(pcurve.origin().y)
            + finite_interval(Interval::point(pcurve.dir().y) * map_offset)?,
    )?;
    let v_direction = finite_interval(Interval::point(pcurve.dir().y) * map_scale)?;

    let frame = surface.frame();
    let carrier_origin = carrier.origin().to_array();
    let carrier_direction = carrier.dir().to_array();
    let surface_origin = frame.origin().to_array();
    let surface_u = frame.x().to_array();
    let surface_v = frame.y().to_array();

    let mut squared_norm = Interval::point(0.0);
    for axis in 0..3 {
        // Form the affine residual coefficients before evaluating over `t`.
        // Evaluating the carrier and lifted trace as independent intervals
        // would discard their shared-parameter correlation and produce a
        // bound as wide as the complete carrier range.
        let lifted_origin_u = finite_interval(Interval::point(surface_u[axis]) * u_origin)?;
        let lifted_origin_v = finite_interval(Interval::point(surface_v[axis]) * v_origin)?;
        let lifted_origin = finite_interval(
            finite_interval(Interval::point(surface_origin[axis]) + lifted_origin_u)?
                + lifted_origin_v,
        )?;
        let lifted_direction_u = finite_interval(Interval::point(surface_u[axis]) * u_direction)?;
        let lifted_direction_v = finite_interval(Interval::point(surface_v[axis]) * v_direction)?;
        let lifted_direction = finite_interval(lifted_direction_u + lifted_direction_v)?;
        let residual_origin =
            finite_interval(Interval::point(carrier_origin[axis]) - lifted_origin)?;
        let residual_direction =
            finite_interval(Interval::point(carrier_direction[axis]) - lifted_direction)?;
        let residual = finite_interval(residual_origin + finite_interval(residual_direction * t)?)?;
        squared_norm = finite_interval(squared_norm + finite_interval(residual.square())?)?;
    }
    let norm = finite_interval(squared_norm.sqrt()?)?;
    Some(norm.hi())
}

fn finite_interval(interval: Interval) -> Option<Interval> {
    (interval.lo().is_finite() && interval.hi().is_finite()).then_some(interval)
}

fn finite_vec2(value: Vec2) -> bool {
    value.x.is_finite() && value.y.is_finite()
}

fn finite_vec3(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

fn finite_plane(surface: Plane) -> bool {
    let frame = surface.frame();
    finite_vec3(frame.origin())
        && finite_vec3(frame.x())
        && finite_vec3(frame.y())
        && finite_vec3(frame.z())
}

fn finite_sphere(surface: Sphere) -> bool {
    finite_frame(surface.frame()) && surface.radius().is_finite()
}

fn finite_circle(carrier: Circle) -> bool {
    finite_frame(carrier.frame()) && carrier.radius().is_finite()
}

fn finite_circle2d(pcurve: Circle2d) -> bool {
    finite_vec2(pcurve.center()) && finite_vec2(pcurve.x_dir()) && pcurve.radius().is_finite()
}

fn finite_frame(frame: &kgeom::frame::Frame) -> bool {
    finite_vec3(frame.origin())
        && finite_vec3(frame.x())
        && finite_vec3(frame.y())
        && finite_vec3(frame.z())
}

/// Whole-range proof for one operation-generated degree-1 analytic/NURBS
/// branch.
///
/// Unlike [`TransmittedNurbsIntersectionCertificate`], this contract carries
/// no interchange metadata. It binds only the operation-generated carrier,
/// its paired degree-1 marching traces, the ordered exact graph fields, and the
/// complete residual proof.
#[derive(Debug, Clone, PartialEq)]
pub struct VerifiedNurbsIntersectionCertificate {
    carrier: NurbsCurve,
    carrier_range: ParamRange,
    traces: [NurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
    proof_depth: usize,
}

impl VerifiedNurbsIntersectionCertificate {
    /// Operation-generated degree-1 model-space carrier.
    pub const fn carrier(&self) -> &NurbsCurve {
        &self.carrier
    }

    /// Complete finite carrier interval covered by the proof.
    pub const fn carrier_range(&self) -> ParamRange {
        self.carrier_range
    }

    /// Exact ordered source-field traces.
    pub const fn traces(&self) -> &[NurbsIntersectionTrace; 2] {
        &self.traces
    }

    /// Paired degree-1 pcurves in source operand order.
    pub const fn pcurves(&self) -> &[NurbsCurve2d; 2] {
        &self.pcurves
    }

    /// Conservative whole-range lifted residual bounds.
    pub const fn residual_bounds(&self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Model-space tolerance used by the proof.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// Binary proof depth used on every carrier knot span.
    pub const fn proof_depth(&self) -> usize {
        self.proof_depth
    }
}

/// Persistent descriptor for an operation-generated verified analytic/NURBS
/// branch.
#[derive(Debug, Clone, PartialEq)]
pub struct VerifiedNurbsIntersectionCurveDescriptor {
    source_surfaces: [SurfaceHandle; 2],
    pcurves: [Curve2dHandle; 2],
    certificate: VerifiedNurbsIntersectionCertificate,
}

impl VerifiedNurbsIntersectionCurveDescriptor {
    pub(crate) const fn new(
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: VerifiedNurbsIntersectionCertificate,
    ) -> Self {
        Self {
            source_surfaces,
            pcurves,
            certificate,
        }
    }

    /// Ordered live source identities.
    pub const fn source_surfaces(&self) -> [SurfaceHandle; 2] {
        self.source_surfaces
    }

    /// Ordered persistent pcurve identities.
    pub const fn pcurves(&self) -> [Curve2dHandle; 2] {
        self.pcurves
    }

    /// Immutable whole-range proof payload.
    pub const fn certificate(&self) -> &VerifiedNurbsIntersectionCertificate {
        &self.certificate
    }

    pub(crate) fn visit_dependencies(&self, visit: &mut dyn FnMut(GeometryRef)) {
        for surface in self.source_surfaces {
            visit(GeometryRef::Surface(surface));
        }
        for pcurve in self.pcurves {
            visit(GeometryRef::Curve2d(pcurve));
        }
    }
}

impl Curve for VerifiedNurbsIntersectionCurveDescriptor {
    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn eval_derivs(&self, parameter: f64, order: usize) -> CurveDerivs {
        self.certificate.carrier.eval_derivs(parameter, order)
    }

    fn param_range(&self) -> ParamRange {
        self.certificate.carrier_range
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, range: ParamRange) -> Aabb3 {
        self.certificate.carrier.bounding_box(range)
    }
}

/// Deterministic logical work required by
/// [`certify_verified_plane_nurbs_intersection_residuals`].
///
/// Plane controls cost one unit each. Every NURBS carrier span costs one unit
/// for each fixed proof subdivision and each logical source differential-
/// enclosure operation (`6 * source tensor span slots + 1`).
pub fn verified_plane_nurbs_intersection_certificate_work(
    carrier: &NurbsCurve,
    traces: &[NurbsIntersectionTrace; 2],
) -> Option<u64> {
    if !matches!(
        traces,
        [
            NurbsIntersectionTrace::Plane(_),
            NurbsIntersectionTrace::Nurbs(_)
        ] | [
            NurbsIntersectionTrace::Nurbs(_),
            NurbsIntersectionTrace::Plane(_)
        ]
    ) {
        return None;
    }
    let control_count = u64::try_from(carrier.points().len()).ok()?;
    let carrier_spans = control_count.checked_sub(1)?;
    let subdivisions = 1_u64.checked_shl(TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u32)?;
    traces.iter().try_fold(0_u64, |total, trace| {
        let work = match trace {
            NurbsIntersectionTrace::Plane(_) => control_count,
            NurbsIntersectionTrace::Sphere(_) => return None,
            NurbsIntersectionTrace::Nurbs(surface) => {
                let (control_u, control_v) = surface.net_size();
                let span_u = u64::try_from(control_u.checked_sub(surface.degree_u())?).ok()?;
                let span_v = u64::try_from(control_v.checked_sub(surface.degree_v())?).ok()?;
                let source_slots = span_u.checked_mul(span_v)?;
                let enclosure_work = source_slots.checked_mul(6)?.checked_add(1)?;
                carrier_spans
                    .checked_mul(subdivisions)?
                    .checked_mul(enclosure_work)?
            }
            NurbsIntersectionTrace::OffsetNurbs(_) => return None,
        };
        total.checked_add(work)
    })
}

/// Certify one operation-generated Plane/NURBS marching branch over its whole
/// finite degree-1 carrier range.
///
/// The proof is the same original-source interval enclosure used by imported
/// NURBS traces, without constructing or retaining transmitted chart metadata.
pub fn certify_verified_plane_nurbs_intersection_residuals(
    carrier: NurbsCurve,
    traces: [NurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    tolerance: f64,
) -> Result<VerifiedNurbsIntersectionCertificate, IntersectionCertificateError> {
    if verified_plane_nurbs_intersection_certificate_work(&carrier, &traces).is_none() {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    let carrier_range = carrier.param_range();
    if !carrier_range.is_finite() || carrier_range.width() <= 0.0 {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if carrier.degree() != 1
        || carrier.weights().is_some()
        || !carrier.knots().is_clamped()
        || carrier.points().len() < 2
    {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "verified NURBS branch carrier must be open clamped polynomial degree 1",
            },
        );
    }
    if carrier
        .points()
        .iter()
        .copied()
        .any(|point| !finite_vec3(point))
        || traces.iter().any(|trace| match trace {
            NurbsIntersectionTrace::Plane(plane) => !finite_plane(*plane),
            NurbsIntersectionTrace::Sphere(_) => true,
            NurbsIntersectionTrace::Nurbs(surface) => {
                surface
                    .points()
                    .iter()
                    .copied()
                    .any(|point| !finite_vec3(point))
                    || surface
                        .weights()
                        .is_some_and(|weights| weights.iter().any(|weight| !weight.is_finite()))
            }
            NurbsIntersectionTrace::OffsetNurbs(_) => true,
        })
    {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let carrier_knots = carrier.knots().as_slice();
    let mut residual_bounds = [0.0; 2];
    for index in 0..2 {
        let trace = paired_trace(index);
        let pcurve = &pcurves[index];
        if pcurve.degree() != 1
            || pcurve.weights().is_some()
            || !pcurve.knots().is_clamped()
            || pcurve.knots().as_slice() != carrier_knots
            || pcurve.points().len() != carrier.points().len()
            || pcurve.param_range() != carrier_range
        {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "verified NURBS pcurve must share the carrier's open clamped polynomial degree-1 basis",
                },
            );
        }
        if pcurve
            .points()
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
        {
            return Err(IntersectionCertificateError::NonFiniteGeometry);
        }

        let bound = match &traces[index] {
            NurbsIntersectionTrace::Plane(surface) => {
                let mut bound = 0.0_f64;
                for (&point, &uv) in carrier.points().iter().zip(pcurve.points()) {
                    let control_bound = transmitted_plane_control_residual_bound(
                        point,
                        *surface,
                        Vec2::new(uv.x, uv.y),
                    )
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
                    bound = bound.max(control_bound);
                }
                bound
            }
            NurbsIntersectionTrace::Sphere(_) => {
                return Err(IntersectionCertificateError::InvalidTraceFamily);
            }
            NurbsIntersectionTrace::Nurbs(surface) => {
                transmitted_nurbs_trace_residual_bound(&carrier, surface, None, pcurve, trace)?
            }
            NurbsIntersectionTrace::OffsetNurbs(_) => {
                return Err(IntersectionCertificateError::InvalidTraceFamily);
            }
        };
        if bound > tolerance {
            return Err(IntersectionCertificateError::ResidualExceedsTolerance {
                trace,
                residual_bound: bound,
                tolerance,
            });
        }
        residual_bounds[index] = bound;
    }

    Ok(VerifiedNurbsIntersectionCertificate {
        carrier,
        carrier_range,
        traces,
        pcurves,
        residual_bounds,
        tolerance,
        proof_depth: TRANSMITTED_NURBS_TRACE_PROOF_DEPTH,
    })
}

/// Exact logical resources consumed by one operation-generated direct
/// NURBS/NURBS whole-range certificate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerifiedNurbsNurbsCertificateCost {
    work: u64,
    items: u64,
    depth: u64,
}

impl VerifiedNurbsNurbsCertificateCost {
    /// Paired original-source interval-enclosure work.
    pub const fn work(self) -> u64 {
        self.work
    }

    /// Paired carrier subdivision cells proved over the complete trace.
    pub const fn items(self) -> u64 {
        self.items
    }

    /// Fixed binary subdivision depth used within every carrier span.
    pub const fn depth(self) -> u64 {
        self.depth
    }
}

/// Deterministic logical resources required by
/// [`certify_verified_nurbs_nurbs_intersection_residuals`].
///
/// Every paired subdivision cell costs the sum of the two original-source
/// differential enclosures: `6 * source tensor span slots + 1` per trace.
/// One paired cell is observed as one proof item, and every carrier span uses
/// the fixed depth retained by the certificate.
pub fn verified_nurbs_nurbs_intersection_certificate_cost(
    carrier: &NurbsCurve,
    traces: &[NurbsIntersectionTrace; 2],
) -> Option<VerifiedNurbsNurbsCertificateCost> {
    let [
        NurbsIntersectionTrace::Nurbs(surface_a),
        NurbsIntersectionTrace::Nurbs(surface_b),
    ] = traces
    else {
        return None;
    };
    verified_paired_nurbs_intersection_certificate_cost(carrier, surface_a, surface_b)
}

/// Deterministic logical resources required by
/// [`certify_verified_offset_nurbs_nurbs_intersection_residuals`].
///
/// The offset trace retains the original basis differential enclosure and
/// proves its effective unit-normal lift within the same per-trace logical
/// work unit used by transmitted Offset(NURBS) charts.
pub fn verified_offset_nurbs_nurbs_intersection_certificate_cost(
    carrier: &NurbsCurve,
    traces: &[NurbsIntersectionTrace; 2],
) -> Option<VerifiedNurbsNurbsCertificateCost> {
    let (basis, direct) = match traces {
        [
            NurbsIntersectionTrace::OffsetNurbs(offset),
            NurbsIntersectionTrace::Nurbs(direct),
        ]
        | [
            NurbsIntersectionTrace::Nurbs(direct),
            NurbsIntersectionTrace::OffsetNurbs(offset),
        ] => (offset.basis(), direct),
        _ => return None,
    };
    verified_paired_nurbs_intersection_certificate_cost(carrier, basis, direct)
}

fn verified_paired_nurbs_intersection_certificate_cost(
    carrier: &NurbsCurve,
    surface_a: &NurbsSurface,
    surface_b: &NurbsSurface,
) -> Option<VerifiedNurbsNurbsCertificateCost> {
    let control_count = u64::try_from(carrier.points().len()).ok()?;
    let carrier_spans = control_count.checked_sub(1)?;
    let subdivisions = 1_u64.checked_shl(TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u32)?;
    let items = carrier_spans.checked_mul(subdivisions)?;
    let trace_work = |surface: &NurbsSurface| {
        let (control_u, control_v) = surface.net_size();
        let span_u = u64::try_from(control_u.checked_sub(surface.degree_u())?).ok()?;
        let span_v = u64::try_from(control_v.checked_sub(surface.degree_v())?).ok()?;
        span_u.checked_mul(span_v)?.checked_mul(6)?.checked_add(1)
    };
    let work_per_item = trace_work(surface_a)?.checked_add(trace_work(surface_b)?)?;
    Some(VerifiedNurbsNurbsCertificateCost {
        work: items.checked_mul(work_per_item)?,
        items,
        depth: TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
    })
}

/// Certify one operation-generated direct NURBS/NURBS marching branch over
/// its complete finite degree-1 carrier range.
///
/// Both ordered traces use original-source interval differential enclosures
/// and centered mean-value residuals. No transmitted interchange metadata,
/// point sample, or recomputed spatial intersection is proof evidence.
pub fn certify_verified_nurbs_nurbs_intersection_residuals(
    carrier: NurbsCurve,
    traces: [NurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    tolerance: f64,
) -> Result<VerifiedNurbsIntersectionCertificate, IntersectionCertificateError> {
    if verified_nurbs_nurbs_intersection_certificate_cost(&carrier, &traces).is_none() {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    certify_verified_paired_nurbs_intersection_residuals_impl(carrier, traces, pcurves, tolerance)
}

/// Certify one operation-generated direct Offset(NURBS)/NURBS marching branch
/// over its complete finite degree-1 carrier range.
///
/// The offset trace retains its original basis and signed distance. Every
/// proof cell encloses the basis point and partials, proves a regular unit
/// normal, and certifies the effective displaced lift; the direct trace is
/// certified independently against its original source.
pub fn certify_verified_offset_nurbs_nurbs_intersection_residuals(
    carrier: NurbsCurve,
    traces: [NurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    tolerance: f64,
) -> Result<VerifiedNurbsIntersectionCertificate, IntersectionCertificateError> {
    if verified_offset_nurbs_nurbs_intersection_certificate_cost(&carrier, &traces).is_none() {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    certify_verified_paired_nurbs_intersection_residuals_impl(carrier, traces, pcurves, tolerance)
}

fn certify_verified_paired_nurbs_intersection_residuals_impl(
    carrier: NurbsCurve,
    traces: [NurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    tolerance: f64,
) -> Result<VerifiedNurbsIntersectionCertificate, IntersectionCertificateError> {
    let carrier_range = carrier.param_range();
    if !carrier_range.is_finite() || carrier_range.width() <= 0.0 {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if carrier.degree() != 1
        || carrier.weights().is_some()
        || !carrier.knots().is_clamped()
        || carrier.points().len() < 2
    {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "verified NURBS branch carrier must be open clamped polynomial degree 1",
            },
        );
    }
    if carrier
        .points()
        .iter()
        .copied()
        .any(|point| !finite_vec3(point))
        || traces.iter().any(|trace| match trace {
            NurbsIntersectionTrace::Nurbs(surface) => {
                surface
                    .points()
                    .iter()
                    .copied()
                    .any(|point| !finite_vec3(point))
                    || surface
                        .weights()
                        .is_some_and(|weights| weights.iter().any(|weight| !weight.is_finite()))
            }
            NurbsIntersectionTrace::OffsetNurbs(offset) => {
                !offset.signed_distance().is_finite()
                    || offset
                        .basis()
                        .points()
                        .iter()
                        .copied()
                        .any(|point| !finite_vec3(point))
                    || offset
                        .basis()
                        .weights()
                        .is_some_and(|weights| weights.iter().any(|weight| !weight.is_finite()))
            }
            NurbsIntersectionTrace::Plane(_) | NurbsIntersectionTrace::Sphere(_) => true,
        })
    {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let carrier_knots = carrier.knots().as_slice();
    let mut residual_bounds = [0.0; 2];
    for index in 0..2 {
        let trace = paired_trace(index);
        let pcurve = &pcurves[index];
        if pcurve.degree() != 1
            || pcurve.weights().is_some()
            || !pcurve.knots().is_clamped()
            || pcurve.knots().as_slice() != carrier_knots
            || pcurve.points().len() != carrier.points().len()
            || pcurve.param_range() != carrier_range
        {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "verified NURBS pcurve must share the carrier's open clamped polynomial degree-1 basis",
                },
            );
        }
        if pcurve
            .points()
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
        {
            return Err(IntersectionCertificateError::NonFiniteGeometry);
        }

        let bound = match &traces[index] {
            NurbsIntersectionTrace::Nurbs(surface) => {
                transmitted_nurbs_trace_residual_bound(&carrier, surface, None, pcurve, trace)?
            }
            NurbsIntersectionTrace::OffsetNurbs(offset) => transmitted_nurbs_trace_residual_bound(
                &carrier,
                offset.basis(),
                Some(offset.signed_distance()),
                pcurve,
                trace,
            )?,
            NurbsIntersectionTrace::Plane(_) | NurbsIntersectionTrace::Sphere(_) => {
                return Err(IntersectionCertificateError::InvalidTraceFamily);
            }
        };
        if bound > tolerance {
            return Err(IntersectionCertificateError::ResidualExceedsTolerance {
                trace,
                residual_bound: bound,
                tolerance,
            });
        }
        residual_bounds[index] = bound;
    }

    Ok(VerifiedNurbsIntersectionCertificate {
        carrier,
        carrier_range,
        traces,
        pcurves,
        residual_bounds,
        tolerance,
        proof_depth: TRANSMITTED_NURBS_TRACE_PROOF_DEPTH,
    })
}

/// Exact logical resources consumed by one operation-generated Sphere/NURBS
/// whole-range certificate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerifiedSphereNurbsCertificateCost {
    work: u64,
    items: u64,
    depth: u64,
}

impl VerifiedSphereNurbsCertificateCost {
    /// Paired interval-enclosure work.
    pub const fn work(self) -> u64 {
        self.work
    }

    /// Paired carrier subdivision cells proved over the complete trace.
    pub const fn items(self) -> u64 {
        self.items
    }

    /// Fixed binary subdivision depth used within every carrier span.
    pub const fn depth(self) -> u64 {
        self.depth
    }
}

/// Deterministic logical resources required by
/// [`certify_verified_sphere_nurbs_intersection_residuals`].
///
/// Each paired subdivision cell costs one analytic-sphere mean-value
/// enclosure plus `6 * source tensor span slots + 1` original-NURBS
/// differential enclosure operations. One paired cell is observed as one
/// proof item, and every carrier span uses the fixed depth retained by the
/// certificate.
pub fn verified_sphere_nurbs_intersection_certificate_cost(
    carrier: &NurbsCurve,
    traces: &[NurbsIntersectionTrace; 2],
) -> Option<VerifiedSphereNurbsCertificateCost> {
    let surface = match traces {
        [
            NurbsIntersectionTrace::Sphere(_),
            NurbsIntersectionTrace::Nurbs(surface),
        ]
        | [
            NurbsIntersectionTrace::Nurbs(surface),
            NurbsIntersectionTrace::Sphere(_),
        ] => surface,
        _ => return None,
    };
    let control_count = u64::try_from(carrier.points().len()).ok()?;
    let carrier_spans = control_count.checked_sub(1)?;
    let subdivisions = 1_u64.checked_shl(TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u32)?;
    let items = carrier_spans.checked_mul(subdivisions)?;
    let (control_u, control_v) = surface.net_size();
    let span_u = u64::try_from(control_u.checked_sub(surface.degree_u())?).ok()?;
    let span_v = u64::try_from(control_v.checked_sub(surface.degree_v())?).ok()?;
    let source_slots = span_u.checked_mul(span_v)?;
    let work_per_item = source_slots.checked_mul(6)?.checked_add(2)?;
    Some(VerifiedSphereNurbsCertificateCost {
        work: items.checked_mul(work_per_item)?,
        items,
        depth: TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
    })
}

/// Certify one operation-generated Sphere/NURBS marching branch over its
/// complete finite degree-1 carrier range.
///
/// The sphere lift uses a centered mean-value interval residual on every
/// fixed-depth carrier subdivision. The NURBS lift reuses the correlated
/// original-source interval enclosure. Sphere traces crossing a latitude
/// pole fail closed because their longitude pcurve is not a regular chart.
pub fn certify_verified_sphere_nurbs_intersection_residuals(
    carrier: NurbsCurve,
    traces: [NurbsIntersectionTrace; 2],
    pcurves: [NurbsCurve2d; 2],
    tolerance: f64,
) -> Result<VerifiedNurbsIntersectionCertificate, IntersectionCertificateError> {
    if verified_sphere_nurbs_intersection_certificate_cost(&carrier, &traces).is_none() {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    let carrier_range = carrier.param_range();
    if !carrier_range.is_finite() || carrier_range.width() <= 0.0 {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if carrier.degree() != 1
        || carrier.weights().is_some()
        || !carrier.knots().is_clamped()
        || carrier.points().len() < 2
    {
        return Err(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "verified NURBS branch carrier must be open clamped polynomial degree 1",
            },
        );
    }
    if carrier
        .points()
        .iter()
        .copied()
        .any(|point| !finite_vec3(point))
        || traces.iter().any(|trace| match trace {
            NurbsIntersectionTrace::Sphere(sphere) => !finite_sphere(*sphere),
            NurbsIntersectionTrace::Nurbs(surface) => {
                surface
                    .points()
                    .iter()
                    .copied()
                    .any(|point| !finite_vec3(point))
                    || surface
                        .weights()
                        .is_some_and(|weights| weights.iter().any(|weight| !weight.is_finite()))
            }
            NurbsIntersectionTrace::Plane(_) | NurbsIntersectionTrace::OffsetNurbs(_) => true,
        })
    {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    }

    let carrier_knots = carrier.knots().as_slice();
    let mut residual_bounds = [0.0; 2];
    for index in 0..2 {
        let trace = paired_trace(index);
        let pcurve = &pcurves[index];
        if pcurve.degree() != 1
            || pcurve.weights().is_some()
            || !pcurve.knots().is_clamped()
            || pcurve.knots().as_slice() != carrier_knots
            || pcurve.points().len() != carrier.points().len()
            || pcurve.param_range() != carrier_range
        {
            return Err(
                IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace,
                    reason: "verified NURBS pcurve must share the carrier's open clamped polynomial degree-1 basis",
                },
            );
        }
        if pcurve
            .points()
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
        {
            return Err(IntersectionCertificateError::NonFiniteGeometry);
        }

        let bound = match &traces[index] {
            NurbsIntersectionTrace::Sphere(surface) => {
                verified_sphere_trace_residual_bound(&carrier, *surface, pcurve, tolerance, trace)?
            }
            NurbsIntersectionTrace::Nurbs(surface) => {
                transmitted_nurbs_trace_residual_bound(&carrier, surface, None, pcurve, trace)?
            }
            NurbsIntersectionTrace::Plane(_) | NurbsIntersectionTrace::OffsetNurbs(_) => {
                return Err(IntersectionCertificateError::InvalidTraceFamily);
            }
        };
        if bound > tolerance {
            return Err(IntersectionCertificateError::ResidualExceedsTolerance {
                trace,
                residual_bound: bound,
                tolerance,
            });
        }
        residual_bounds[index] = bound;
    }

    Ok(VerifiedNurbsIntersectionCertificate {
        carrier,
        carrier_range,
        traces,
        pcurves,
        residual_bounds,
        tolerance,
        proof_depth: TRANSMITTED_NURBS_TRACE_PROOF_DEPTH,
    })
}

fn verified_sphere_trace_residual_bound(
    carrier: &NurbsCurve,
    sphere: Sphere,
    pcurve: &NurbsCurve2d,
    tolerance: f64,
    trace: PairedTrace,
) -> Result<f64, IntersectionCertificateError> {
    let latitude_domain = sphere.param_range()[1];
    if pcurve
        .points()
        .iter()
        .any(|point| point.y < latitude_domain.lo || point.y > latitude_domain.hi)
    {
        return Err(IntersectionCertificateError::SphereTraceOutsideWindow {
            coordinate: "latitude",
        });
    }

    let radius = sphere.radius();
    let radius_sq = radius * radius;
    let mut squared_pole_clearance = f64::INFINITY;
    for points in pcurve.points().windows(2) {
        let lo = points[0].y.min(points[1].y);
        let hi = points[0].y.max(points[1].y);
        let cosine = trig_interval(lo, hi, false);
        let squared = finite_interval(cosine.square() * Interval::point(radius_sq))
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
        squared_pole_clearance = squared_pole_clearance.min(squared.lo().max(0.0));
    }
    if squared_pole_clearance <= tolerance * tolerance {
        return Err(IntersectionCertificateError::SingularSphereChart {
            squared_pole_clearance,
        });
    }

    let subdivisions = 1_usize << TRANSMITTED_NURBS_TRACE_PROOF_DEPTH;
    let frame = sphere.frame();
    let origin = frame.origin().to_array();
    let axes = [
        frame.x().to_array(),
        frame.y().to_array(),
        frame.z().to_array(),
    ];
    let mut maximum = 0.0_f64;
    for (carrier_points, pcurve_points) in
        carrier.points().windows(2).zip(pcurve.points().windows(2))
    {
        let carrier_start = carrier_points[0];
        let carrier_delta = interval_vec3_difference(carrier_points[1], carrier_start);
        let uv_start = pcurve_points[0];
        let uv_delta = [
            finite_interval(Interval::point(pcurve_points[1].x) - Interval::point(uv_start.x))
                .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
            finite_interval(Interval::point(pcurve_points[1].y) - Interval::point(uv_start.y))
                .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
        ];
        for subdivision in 0..subdivisions {
            let lo = subdivision as f64 / subdivisions as f64;
            let hi = (subdivision + 1) as f64 / subdivisions as f64;
            let midpoint = (lo + hi) * 0.5;
            let midpoint_fraction = Interval::point(midpoint);
            let carrier_midpoint = interval_affine_vec3(carrier_start, carrier_delta, midpoint);
            let uv_midpoint = [
                finite_interval(Interval::point(uv_start.x) + uv_delta[0] * midpoint_fraction)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
                finite_interval(Interval::point(uv_start.y) + uv_delta[1] * midpoint_fraction)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
            ];
            let midpoint_sine_u = trig_interval(uv_midpoint[0].lo(), uv_midpoint[0].hi(), true);
            let midpoint_cosine_u = trig_interval(uv_midpoint[0].lo(), uv_midpoint[0].hi(), false);
            let midpoint_sine_v = trig_interval(uv_midpoint[1].lo(), uv_midpoint[1].hi(), true);
            let midpoint_cosine_v = trig_interval(uv_midpoint[1].lo(), uv_midpoint[1].hi(), false);
            let radius = Interval::point(radius);
            let midpoint_local = [
                finite_interval(midpoint_cosine_v * midpoint_cosine_u)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
                finite_interval(midpoint_cosine_v * midpoint_sine_u)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
                midpoint_sine_v,
            ];
            let residual_midpoint: [Interval; 3] = core::array::from_fn(|axis| {
                let mut local_to_world = Interval::point(0.0);
                for local_axis in 0..3 {
                    local_to_world = local_to_world
                        + Interval::point(axes[local_axis][axis]) * midpoint_local[local_axis];
                }
                carrier_midpoint[axis] - (Interval::point(origin[axis]) + radius * local_to_world)
            });

            let fraction_range = Interval::new(lo, hi);
            let u_range =
                finite_interval(Interval::point(uv_start.x) + uv_delta[0] * fraction_range)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
            let v_range =
                finite_interval(Interval::point(uv_start.y) + uv_delta[1] * fraction_range)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
            let sine_u = trig_interval(u_range.lo(), u_range.hi(), true);
            let cosine_u = trig_interval(u_range.lo(), u_range.hi(), false);
            let sine_v = trig_interval(v_range.lo(), v_range.hi(), true);
            let cosine_v = trig_interval(v_range.lo(), v_range.hi(), false);
            let du_local = [
                finite_interval(Interval::point(-1.0) * radius * cosine_v * sine_u)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
                finite_interval(radius * cosine_v * cosine_u)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
                Interval::point(0.0),
            ];
            let dv_local = [
                finite_interval(Interval::point(-1.0) * radius * sine_v * cosine_u)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
                finite_interval(Interval::point(-1.0) * radius * sine_v * sine_u)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
                finite_interval(radius * cosine_v)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
            ];
            let mut squared_norm = Interval::point(0.0);
            let centered_fraction =
                Interval::new((lo - midpoint).next_down(), (hi - midpoint).next_up());
            for (axis, carrier_component) in carrier_delta.iter().copied().enumerate() {
                let mut derivative_u = Interval::point(0.0);
                let mut derivative_v = Interval::point(0.0);
                for local_axis in 0..3 {
                    derivative_u = finite_interval(
                        derivative_u
                            + finite_interval(
                                Interval::point(axes[local_axis][axis]) * du_local[local_axis],
                            )
                            .ok_or(
                                IntersectionCertificateError::NonFiniteResidualBound { trace },
                            )?,
                    )
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
                    derivative_v = finite_interval(
                        derivative_v
                            + finite_interval(
                                Interval::point(axes[local_axis][axis]) * dv_local[local_axis],
                            )
                            .ok_or(
                                IntersectionCertificateError::NonFiniteResidualBound { trace },
                            )?,
                    )
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
                }
                let lifted_derivative = finite_interval(
                    finite_interval(derivative_u * uv_delta[0])
                        .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?
                        + finite_interval(derivative_v * uv_delta[1]).ok_or(
                            IntersectionCertificateError::NonFiniteResidualBound { trace },
                        )?,
                )
                .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
                let residual_derivative = finite_interval(carrier_component - lifted_derivative)
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
                let residual = finite_interval(
                    residual_midpoint[axis]
                        + finite_interval(residual_derivative * centered_fraction).ok_or(
                            IntersectionCertificateError::NonFiniteResidualBound { trace },
                        )?,
                )
                .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
                squared_norm = finite_interval(
                    squared_norm
                        + finite_interval(residual.square()).ok_or(
                            IntersectionCertificateError::NonFiniteResidualBound { trace },
                        )?,
                )
                .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?;
            }
            let bound = finite_interval(
                squared_norm
                    .sqrt()
                    .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?,
            )
            .ok_or(IntersectionCertificateError::NonFiniteResidualBound { trace })?
            .hi();
            maximum = maximum.max(bound);
        }
    }
    Ok(maximum)
}
