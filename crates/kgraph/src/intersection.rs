//! Verified affine intersection-curve building blocks.
//!
//! This module intentionally does not add a persistent intersection-curve
//! descriptor. It provides the small, independently verifiable substrate
//! needed by operation-local branch graphs: an invertible affine parameter
//! correspondence and a certificate that a finite line carrier agrees with
//! two plane pcurves over a complete parameter interval.

use core::fmt;

use kcore::interval::Interval;
use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Vec2, Vec3};

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
            Self::InvalidCarrierRange => f.write_str("carrier range must be finite and ordered"),
            Self::InvalidTolerance => {
                f.write_str("residual tolerance must be finite and nonnegative")
            }
            Self::NonFiniteGeometry => f.write_str("certified geometry must be finite"),
            Self::NonFiniteResidualBound { trace } => {
                write!(f, "{trace:?} trace produced a non-finite residual bound")
            }
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
