//! Constant-time relationships between graph-bound persistent skew spans.
//!
//! Each input descriptor was already bound to its own exact ordered live source
//! handles by graph insertion. Exact source geometry may use distinct handles
//! across the two spans. The per-span certifier retains the interval
//! aggregates produced by its fixed 256-cell pass, so this module compares
//! sealed family/root data and exposes those aggregates without sampling,
//! recertification, or another work charge.

use core::fmt;

use kcore::interval::Interval;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgeom::vec::Vec3;

use super::super::{BranchAlgebra, Harmonic, SkewCylinderBranchDirectedChartIntegral, TAU};
use super::*;

/// Explicit canonical-longitude order for two disjoint spans.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderSpanRangeOrder {
    /// The first span ends strictly before the second span starts.
    FirstBeforeSecond,
    /// The second span ends strictly before the first span starts.
    SecondBeforeFirst,
}

/// Relationship theorem requested by the caller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderSpanRelationshipRequest {
    /// Two spans occupy strictly disjoint non-wrapping canonical ranges.
    DisjointRange {
        /// Caller-declared canonical-longitude order.
        order: PersistentSkewCylinderSpanRangeOrder,
    },
}

/// Retained enclosure of one directed pcurve line integral.
///
/// Both intervals enclose `integral(u dv - v du)` over the span's authored
/// logical traversal. The source interval is independently derived from exact
/// source coefficients; neither interval alone asserts a closed-loop
/// orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderDirectedChartIntegralCertificate {
    stored: Interval,
    source: Interval,
}

impl PersistentSkewCylinderDirectedChartIntegralCertificate {
    /// Procedural-evaluator integral enclosure.
    pub const fn stored_enclosure(self) -> Interval {
        self.stored
    }

    /// Independently exact-source integral enclosure.
    pub const fn source_enclosure(self) -> Interval {
        self.source
    }
}

/// Certified relationship-specific payload.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PersistentSkewCylinderSpanRelationshipKind {
    /// Two spans are separated everywhere on their common exact canonical
    /// Cylinder chart, regardless of sheet.
    DisjointRange {
        /// Verified canonical-longitude order.
        order: PersistentSkewCylinderSpanRangeOrder,
        /// Conservative minimum cyclic angular gap between the full
        /// root-enclosure ranges.
        angular_gap_lower: f64,
        /// Conservative radial chord lower bound
        /// `2 r sin(angular_gap_lower / 2)`.
        radial_chord_lower: f64,
        /// Chord bound after subtracting both persistent edge envelopes.
        metric_clearance_lower: f64,
    },
}

/// Sealed relationship between two graph-bound persistent skew spans.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderSpanRelationshipCertificate {
    spans: [VerifiedSkewCylinderOpenSpanCurveDescriptor; 2],
    request: PersistentSkewCylinderSpanRelationshipRequest,
    kind: PersistentSkewCylinderSpanRelationshipKind,
    span_directed_chart_integrals: [[PersistentSkewCylinderDirectedChartIntegralCertificate; 2]; 2],
}

impl PersistentSkewCylinderSpanRelationshipCertificate {
    /// Exact graph-bound descriptors in caller order.
    pub const fn spans(self) -> [VerifiedSkewCylinderOpenSpanCurveDescriptor; 2] {
        self.spans
    }

    /// Exact ordered live source identities retained independently per span.
    pub const fn span_source_surfaces(self) -> [[SurfaceHandle; 2]; 2] {
        [
            self.spans[0].source_surfaces(),
            self.spans[1].source_surfaces(),
        ]
    }

    /// Caller request verified by this certificate.
    pub const fn request(self) -> PersistentSkewCylinderSpanRelationshipRequest {
        self.request
    }

    /// Relationship-specific certified payload.
    pub const fn kind(self) -> PersistentSkewCylinderSpanRelationshipKind {
        self.kind
    }

    /// Per-span directed integral enclosures in caller span order, then exact
    /// ordered source-surface order.
    pub const fn span_directed_chart_integrals(
        self,
    ) -> [[PersistentSkewCylinderDirectedChartIntegralCertificate; 2]; 2] {
        self.span_directed_chart_integrals
    }
}

/// Fail-closed reason for rejecting a persistent skew-span relationship.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum PersistentSkewCylinderSpanRelationshipError {
    /// Sealed trace surfaces do not retain the same ordered source family.
    SourceOrderMismatch,
    /// Exact Cylinder algebra differs beyond the requested sheet/range relation.
    AlgebraMismatch,
    /// Authored source chart windows or their required lifts differ.
    ChartMismatch,
    /// Model-space certification tolerances differ.
    ToleranceMismatch,
    /// A supposedly sealed span is internally inconsistent.
    InvalidSealedSpan,
    /// Canonical ranges do not satisfy the explicit requested relation.
    RangeRelationMismatch,
    /// Exact endpoint equality/distinctness does not match the request.
    EndpointRelationMismatch,
    /// A retained separation or allowance does not yield positive clearance.
    SeparationIndeterminate,
}

impl fmt::Display for PersistentSkewCylinderSpanRelationshipError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SourceOrderMismatch => {
                formatter.write_str("persistent skew spans use different ordered source algebra")
            }
            Self::AlgebraMismatch => {
                formatter.write_str("persistent skew spans do not share exact branch algebra")
            }
            Self::ChartMismatch => {
                formatter.write_str("persistent skew spans do not share exact authored charts")
            }
            Self::ToleranceMismatch => {
                formatter.write_str("persistent skew spans use different certification tolerances")
            }
            Self::InvalidSealedSpan => {
                formatter.write_str("persistent skew span has inconsistent sealed evidence")
            }
            Self::RangeRelationMismatch => formatter
                .write_str("persistent skew span ranges do not match the requested relation"),
            Self::EndpointRelationMismatch => formatter
                .write_str("persistent skew span endpoints do not match the requested relation"),
            Self::SeparationIndeterminate => {
                formatter.write_str("persistent skew span separation has no positive clearance")
            }
        }
    }
}

impl std::error::Error for PersistentSkewCylinderSpanRelationshipError {}

/// Certify a constant-time relationship between two graph-bound persistent
/// skew-cylinder spans.
///
/// Each descriptor's ordered handle array labels that span's own exact source
/// slots. Cross-span equivalence comes from exact ordered trace geometry rather
/// than handle identity. Pcurve operand numbers and numeric handle order never
/// select a source, sheet, range order, or orientation.
pub fn certify_persistent_skew_cylinder_span_relationship(
    first: VerifiedSkewCylinderOpenSpanCurveDescriptor,
    second: VerifiedSkewCylinderOpenSpanCurveDescriptor,
    request: PersistentSkewCylinderSpanRelationshipRequest,
) -> Result<
    PersistentSkewCylinderSpanRelationshipCertificate,
    PersistentSkewCylinderSpanRelationshipError,
> {
    let first_certificate = first.certificate();
    let second_certificate = second.certificate();
    let first_span = validate_sealed_span(first_certificate)?;
    let second_span = validate_sealed_span(second_certificate)?;
    let first_residual = first_certificate.residual;
    let second_residual = second_certificate.residual;
    if !exact_ordered_trace_surfaces(first_residual, second_residual) {
        return Err(PersistentSkewCylinderSpanRelationshipError::SourceOrderMismatch);
    }
    if !same_exact_algebra_family(
        first_residual.carrier.algebra,
        second_residual.carrier.algebra,
    ) {
        return Err(PersistentSkewCylinderSpanRelationshipError::AlgebraMismatch);
    }
    if !exact_param_ranges(first_residual.chart_windows, second_residual.chart_windows) {
        return Err(PersistentSkewCylinderSpanRelationshipError::ChartMismatch);
    }
    if !exact_f64(first_residual.tolerance, second_residual.tolerance) {
        return Err(PersistentSkewCylinderSpanRelationshipError::ToleranceMismatch);
    }

    let span_directed_chart_integrals = [
        public_integrals(first_certificate.directed_chart_integrals),
        public_integrals(second_certificate.directed_chart_integrals),
    ];
    let kind = match request {
        PersistentSkewCylinderSpanRelationshipRequest::DisjointRange { order } => {
            certify_disjoint_range(
                first_certificate,
                second_certificate,
                first_span,
                second_span,
                order,
            )?
        }
    };

    Ok(PersistentSkewCylinderSpanRelationshipCertificate {
        spans: [first, second],
        request,
        kind,
        span_directed_chart_integrals,
    })
}

#[derive(Debug, Clone, Copy)]
struct SealedSpan {
    full_root_enclosure_range: ParamRange,
    residual_range: ParamRange,
}

fn validate_sealed_span(
    certificate: PersistentSkewCylinderOpenSpanCertificate,
) -> Result<SealedSpan, PersistentSkewCylinderSpanRelationshipError> {
    let residual = certificate.residual;
    let [lower, upper] = certificate.root_corridors;
    let roots = [lower.root_parameter(), upper.root_parameter()];
    let canonical_representatives = match roots.map(interval_midpoint) {
        [Some(lower), Some(upper)] => [lower, upper],
        _ => {
            return Err(PersistentSkewCylinderSpanRelationshipError::InvalidSealedSpan);
        }
    };
    let logical_representatives = certificate
        .orientation
        .orient_pair(canonical_representatives);
    let residual_range = residual.carrier_range;
    let expected_parameter_map = certificate.carrier.parameter_map;
    let maps_match = certificate
        .pcurves
        .iter()
        .all(|pcurve| pcurve.parameter_map == expected_parameter_map);
    let algebra_match =
        exact_complete_algebra(certificate.carrier.algebra, residual.carrier.algebra)
            && certificate
                .pcurves
                .iter()
                .all(|pcurve| exact_complete_algebra(pcurve.algebra, residual.carrier.algebra));
    let root_structure_matches = lower.guarded_end() == SkewCylinderBranchGuardedEnd::Lower
        && upper.guarded_end() == SkewCylinderBranchGuardedEnd::Upper
        && roots[0].hi() < residual_range.lo
        && roots[1].lo() > residual_range.hi
        && exact_interval(
            lower.corridor().parameter(),
            Interval::new(roots[0].lo(), residual_range.lo),
        )
        && exact_interval(
            upper.corridor().parameter(),
            Interval::new(residual_range.hi, roots[1].hi()),
        );
    let parameter_map_matches = exact_f64(
        expected_parameter_map.endpoint_representatives[0],
        logical_representatives[0],
    ) && exact_f64(
        expected_parameter_map.endpoint_representatives[1],
        logical_representatives[1],
    );
    let integral_witness_is_finite = certificate
        .directed_chart_integrals
        .iter()
        .all(finite_directed_integral);
    if !maps_match
        || !algebra_match
        || !root_structure_matches
        || !parameter_map_matches
        || !integral_witness_is_finite
        || residual.sheet != residual.carrier.algebra.sheet
        || certificate.carrier.algebra.sheet != residual.sheet
    {
        return Err(PersistentSkewCylinderSpanRelationshipError::InvalidSealedSpan);
    }
    Ok(SealedSpan {
        full_root_enclosure_range: ParamRange {
            lo: roots[0].lo(),
            hi: roots[1].hi(),
        },
        residual_range,
    })
}

fn certify_disjoint_range(
    first: PersistentSkewCylinderOpenSpanCertificate,
    second: PersistentSkewCylinderOpenSpanCertificate,
    first_span: SealedSpan,
    second_span: SealedSpan,
    order: PersistentSkewCylinderSpanRangeOrder,
) -> Result<PersistentSkewCylinderSpanRelationshipKind, PersistentSkewCylinderSpanRelationshipError>
{
    let (earlier, later) = match order {
        PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond => (first_span, second_span),
        PersistentSkewCylinderSpanRangeOrder::SecondBeforeFirst => (second_span, first_span),
    };
    if earlier.full_root_enclosure_range.hi >= later.full_root_enclosure_range.lo
        || earlier.residual_range.hi >= later.residual_range.lo
    {
        return Err(PersistentSkewCylinderSpanRelationshipError::RangeRelationMismatch);
    }
    if !four_exact_distinct_endpoints(first.endpoint_points, second.endpoint_points) {
        return Err(PersistentSkewCylinderSpanRelationshipError::EndpointRelationMismatch);
    }

    let direct_gap = positive_difference(
        later.full_root_enclosure_range.lo,
        earlier.full_root_enclosure_range.hi,
    )?;
    let wrap_gap = finite_interval(
        Interval::point(earlier.full_root_enclosure_range.lo) + Interval::point(TAU)
            - Interval::point(later.full_root_enclosure_range.hi),
    )
    .map(Interval::lo)
    .filter(|gap| *gap > 0.0)
    .ok_or(PersistentSkewCylinderSpanRelationshipError::RangeRelationMismatch)?;
    let angular_gap_lower = direct_gap.min(wrap_gap);
    if angular_gap_lower > core::f64::consts::PI {
        return Err(PersistentSkewCylinderSpanRelationshipError::RangeRelationMismatch);
    }
    let half_gap = finite_interval(Interval::point(0.5) * Interval::point(angular_gap_lower))
        .map(Interval::lo)
        .filter(|half| *half > 0.0)
        .ok_or(PersistentSkewCylinderSpanRelationshipError::SeparationIndeterminate)?;
    let sine_lower = super::super::trig_interval(half_gap, half_gap, true).lo();
    let radius = first.residual.carrier.algebra.cylinders[0].radius();
    let radial_chord_lower = finite_interval(
        Interval::point(2.0) * Interval::point(radius) * Interval::point(sine_lower),
    )
    .map(Interval::lo)
    .filter(|bound| *bound > 0.0)
    .ok_or(PersistentSkewCylinderSpanRelationshipError::SeparationIndeterminate)?;
    let metric_clearance_lower = finite_interval(
        Interval::point(radial_chord_lower)
            - Interval::point(first.required_edge_tolerance)
            - Interval::point(second.required_edge_tolerance),
    )
    .map(Interval::lo)
    .filter(|clearance| *clearance > 0.0)
    .ok_or(PersistentSkewCylinderSpanRelationshipError::SeparationIndeterminate)?;

    Ok(PersistentSkewCylinderSpanRelationshipKind::DisjointRange {
        order,
        angular_gap_lower,
        radial_chord_lower,
        metric_clearance_lower,
    })
}

fn public_integrals(
    integrals: [SkewCylinderBranchDirectedChartIntegral; 2],
) -> [PersistentSkewCylinderDirectedChartIntegralCertificate; 2] {
    integrals.map(public_integral)
}

const fn public_integral(
    integral: SkewCylinderBranchDirectedChartIntegral,
) -> PersistentSkewCylinderDirectedChartIntegralCertificate {
    PersistentSkewCylinderDirectedChartIntegralCertificate {
        stored: integral.stored,
        source: integral.source,
    }
}

fn finite_directed_integral(integral: &SkewCylinderBranchDirectedChartIntegral) -> bool {
    finite_interval(integral.stored).is_some()
        && finite_interval(integral.source).is_some()
        && finite_interval(integral.stored_ordinate_delta).is_some()
        && finite_interval(integral.source_ordinate_delta).is_some()
}

fn four_exact_distinct_endpoints(first: [Vec3; 2], second: [Vec3; 2]) -> bool {
    let endpoints = [first[0], first[1], second[0], second[1]];
    (0..endpoints.len()).all(|first_index| {
        (first_index + 1..endpoints.len())
            .all(|second_index| !exact_vec3(endpoints[first_index], endpoints[second_index]))
    })
}

fn interval_midpoint(interval: Interval) -> Option<f64> {
    let midpoint = 0.5 * interval.lo() + 0.5 * interval.hi();
    (midpoint.is_finite() && interval.contains(midpoint)).then_some(midpoint)
}

fn positive_difference(
    high: f64,
    low: f64,
) -> Result<f64, PersistentSkewCylinderSpanRelationshipError> {
    finite_interval(Interval::point(high) - Interval::point(low))
        .map(Interval::lo)
        .filter(|difference| *difference > 0.0)
        .ok_or(PersistentSkewCylinderSpanRelationshipError::RangeRelationMismatch)
}

fn exact_ordered_trace_surfaces(
    first: PairedSkewCylinderBranchResidualCertificate,
    second: PairedSkewCylinderBranchResidualCertificate,
) -> bool {
    first
        .traces
        .into_iter()
        .zip(second.traces)
        .all(|(first, second)| exact_cylinder(first.surface, second.surface))
}

fn same_exact_algebra_family(first: BranchAlgebra, second: BranchAlgebra) -> bool {
    first
        .cylinders
        .into_iter()
        .zip(second.cylinders)
        .all(|(first, second)| exact_cylinder(first, second))
        && exact_f64(first.e, second.e)
        && exact_f64(first.dx, second.dx)
        && exact_f64(first.dy, second.dy)
        && exact_f64(first.dz, second.dz)
        && exact_f64(first.a, second.a)
        && exact_f64(first.k, second.k)
        && exact_harmonic(first.x0, second.x0)
        && exact_harmonic(first.y0, second.y0)
        && exact_harmonic(first.z0, second.z0)
        && exact_harmonic(first.m, second.m)
        && exact_harmonic(first.l, second.l)
}

fn exact_complete_algebra(first: BranchAlgebra, second: BranchAlgebra) -> bool {
    same_exact_algebra_family(first, second)
        && exact_param_range(first.carrier_range, second.carrier_range)
        && first.sheet == second.sheet
        && exact_f64(first.longitude_offset, second.longitude_offset)
}

fn exact_harmonic(first: Harmonic, second: Harmonic) -> bool {
    exact_f64(first.constant, second.constant)
        && exact_f64(first.cosine, second.cosine)
        && exact_f64(first.sine, second.sine)
}

fn exact_cylinder(first: Cylinder, second: Cylinder) -> bool {
    exact_frame(*first.frame(), *second.frame()) && exact_f64(first.radius(), second.radius())
}

fn exact_frame(first: Frame, second: Frame) -> bool {
    exact_vec3(first.origin(), second.origin())
        && exact_vec3(first.x(), second.x())
        && exact_vec3(first.y(), second.y())
        && exact_vec3(first.z(), second.z())
}

fn exact_vec3(first: Vec3, second: Vec3) -> bool {
    exact_f64(first.x, second.x) && exact_f64(first.y, second.y) && exact_f64(first.z, second.z)
}

fn exact_param_ranges(first: [ParamRange; 2], second: [ParamRange; 2]) -> bool {
    exact_param_range(first[0], second[0]) && exact_param_range(first[1], second[1])
}

fn exact_param_range(first: ParamRange, second: ParamRange) -> bool {
    exact_f64(first.lo, second.lo) && exact_f64(first.hi, second.hi)
}

fn exact_interval(first: Interval, second: Interval) -> bool {
    exact_f64(first.lo(), second.lo()) && exact_f64(first.hi(), second.hi())
}

fn exact_f64(first: f64, second: f64) -> bool {
    first.to_bits() == second.to_bits()
}

fn finite_interval(interval: Interval) -> Option<Interval> {
    (interval.lo().is_finite() && interval.hi().is_finite() && interval.lo() <= interval.hi())
        .then_some(interval)
}

#[cfg(test)]
#[path = "skew_cylinder_branch_persistent_relationship_tests.rs"]
mod tests;
