//! Shared bounded-parameter validation and fitting semantics.

use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::param::ParamRange;

/// Rejects non-finite or reversed ranges while leaving the owner-specific
/// public error message unchanged.
pub(super) fn validate_finite_ranges(ranges: &[ParamRange], reason: &'static str) -> Result<()> {
    if ranges
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        Err(Error::InvalidGeometry { reason })
    } else {
        Ok(())
    }
}

/// Validates a bounded curve range before its paired surface window while
/// preserving the owning solver's public error reasons and precedence.
pub(super) fn validate_curve_surface_ranges(
    curve_range: ParamRange,
    surface_range: [ParamRange; 2],
    curve_reason: &'static str,
    surface_reason: &'static str,
) -> Result<()> {
    validate_finite_ranges(&[curve_range], curve_reason)?;
    validate_finite_ranges(&surface_range, surface_reason)
}

/// Rejects a bounded periodic range spanning more than one period.
pub(super) fn validate_period_span(
    range: ParamRange,
    period: f64,
    tolerance: f64,
    reason: &'static str,
) -> Result<()> {
    debug_assert!(period.is_finite() && period > 0.0);
    if range.width() > period + tolerance {
        Err(Error::InvalidGeometry { reason })
    } else {
        Ok(())
    }
}

/// Fits a scalar parameter to a bounded range, accepting and clamping values
/// within the supplied parameter tolerance of either endpoint.
pub(super) fn fit_scalar_parameter(
    candidate: f64,
    range: ParamRange,
    tolerance: f64,
) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

/// Fits a two-dimensional parameter to a bounded surface window using the
/// same inclusive endpoint and tolerance-spill semantics as scalar fitting.
pub(super) fn fit_parameter_pair(
    candidate: [f64; 2],
    range: [ParamRange; 2],
    tolerance: f64,
) -> Option<[f64; 2]> {
    Some([
        fit_scalar_parameter(candidate[0], range[0], tolerance)?,
        fit_scalar_parameter(candidate[1], range[1], tolerance)?,
    ])
}

/// Selects the earliest periodic representative accepted by `range`, then
/// clamps endpoint-tolerance spill to the bounded range.
pub(super) fn fit_periodic_parameter(
    candidate: f64,
    range: ParamRange,
    period: f64,
    tolerance: f64,
) -> Option<f64> {
    debug_assert!(period.is_finite() && period > 0.0);
    let k_min = ((range.lo - tolerance - candidate) / period).ceil() as i64;
    let k_max = ((range.hi + tolerance - candidate) / period).floor() as i64;
    if k_min > k_max {
        return None;
    }
    Some((candidate + k_min as f64 * period).clamp(range.lo, range.hi))
}

/// Converts model-space linear tolerance to angular parameter tolerance for a
/// circular scale, preserving the session angular floor.
pub(super) fn angular_parameter_tolerance(radius: f64, tolerances: Tolerances) -> f64 {
    (tolerances.linear() / radius).max(tolerances.angular())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_fitting_is_inclusive_and_clamps_only_tolerance_spill() {
        let range = ParamRange::new(2.0, 4.0);
        assert_eq!(fit_scalar_parameter(2.0 - 1e-6, range, 1e-6), Some(2.0));
        assert_eq!(fit_scalar_parameter(4.0 + 1e-6, range, 1e-6), Some(4.0));
        assert_eq!(fit_scalar_parameter(3.0, range, 0.0), Some(3.0));
        assert_eq!(fit_scalar_parameter(2.0 - 2e-6, range, 1e-6), None);
    }

    #[test]
    fn paired_fitting_is_scalar_equivalent_and_axis_ordered() {
        let ranges = [ParamRange::new(2.0, 4.0), ParamRange::new(-1.0, 1.0)];
        assert_eq!(
            fit_parameter_pair([2.0 - 1e-6, 1.0 + 1e-6], ranges, 1e-6),
            Some([2.0, 1.0])
        );
        assert_eq!(fit_parameter_pair([3.0, 1.0 + 2e-6], ranges, 1e-6), None);
    }

    #[test]
    fn periodic_fitting_uses_the_first_accepted_representative() {
        let tau = core::f64::consts::TAU;
        let range = ParamRange::new(1.5 * tau, 2.0 * tau);
        assert_eq!(
            fit_periodic_parameter(0.0, range, tau, 0.0),
            Some(2.0 * tau)
        );
        assert_eq!(
            fit_periodic_parameter(-0.5 * tau, range, tau, 0.0),
            Some(1.5 * tau)
        );
        assert_eq!(fit_periodic_parameter(0.25 * tau, range, tau, 0.0), None);

        let range = ParamRange::new(15.0, 25.0);
        assert_eq!(fit_periodic_parameter(2.0, range, 10.0, 0.0), Some(22.0));
    }

    #[test]
    fn validation_preserves_owner_error_reasons() {
        let reason = "owner-specific finite range requirement";
        assert_eq!(
            validate_finite_ranges(
                &[ParamRange {
                    lo: f64::NAN,
                    hi: 1.0,
                }],
                reason,
            ),
            Err(Error::InvalidGeometry { reason })
        );
        assert_eq!(
            validate_period_span(
                ParamRange::new(0.0, 2.0 * core::f64::consts::TAU),
                core::f64::consts::TAU,
                1e-12,
                "one period"
            ),
            Err(Error::InvalidGeometry {
                reason: "one period"
            })
        );

        let curve_reason = "owner curve range";
        let surface_reason = "owner surface range";
        let invalid = ParamRange {
            lo: f64::NAN,
            hi: 1.0,
        };
        assert_eq!(
            validate_curve_surface_ranges(
                invalid,
                [invalid, ParamRange::new(0.0, 1.0)],
                curve_reason,
                surface_reason,
            ),
            Err(Error::InvalidGeometry {
                reason: curve_reason,
            })
        );
        assert_eq!(
            validate_curve_surface_ranges(
                ParamRange::new(0.0, 1.0),
                [invalid, ParamRange::new(0.0, 1.0)],
                curve_reason,
                surface_reason,
            ),
            Err(Error::InvalidGeometry {
                reason: surface_reason,
            })
        );
    }
}
