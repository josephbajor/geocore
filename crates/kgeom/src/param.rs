//! Parameter ranges and periodicity.

/// A closed parameter interval `[lo, hi]`; bounds may be infinite for
/// unbounded geometry (lines, plane axes, cylinder axes). Consumers that
/// need finite work (sampling, tessellation, bounding) must clamp via
/// [`ParamRange::clamped`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamRange {
    /// Lower bound (may be `-inf`).
    pub lo: f64,
    /// Upper bound (may be `+inf`).
    pub hi: f64,
}

impl ParamRange {
    /// Construct; panics if `lo > hi` or either bound is NaN.
    pub fn new(lo: f64, hi: f64) -> Self {
        assert!(lo <= hi, "invalid parameter range [{lo}, {hi}]");
        ParamRange { lo, hi }
    }

    /// The whole real line (unbounded parameterization).
    pub fn unbounded() -> Self {
        ParamRange {
            lo: f64::NEG_INFINITY,
            hi: f64::INFINITY,
        }
    }

    /// True if both bounds are finite.
    pub fn is_finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }

    /// Width `hi - lo` (may be infinite).
    pub fn width(self) -> f64 {
        self.hi - self.lo
    }

    /// True if `t` lies within the range.
    pub fn contains(self, t: f64) -> bool {
        self.lo <= t && t <= self.hi
    }

    /// Intersect with a finite fallback window, producing a finite range.
    /// Panics if the intersection is empty.
    pub fn clamped(self, fallback: ParamRange) -> ParamRange {
        debug_assert!(fallback.is_finite());
        ParamRange::new(self.lo.max(fallback.lo), self.hi.min(fallback.hi))
    }

    /// Linear interpolation across the range (requires finite bounds).
    pub fn lerp(self, s: f64) -> f64 {
        debug_assert!(self.is_finite());
        self.lo + (self.hi - self.lo) * s
    }
}

/// Wrap a parameter of a periodic curve/surface direction into
/// `[base, base + period)`.
pub fn wrap_periodic(t: f64, base: f64, period: f64) -> f64 {
    debug_assert!(period > 0.0);
    let mut s = (t - base) % period;
    if s < 0.0 {
        s += period;
    }
    base + s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamping_unbounded_ranges() {
        let r = ParamRange::unbounded().clamped(ParamRange::new(-10.0, 10.0));
        assert_eq!(r, ParamRange::new(-10.0, 10.0));
        let r = ParamRange::new(0.0, 4.0).clamped(ParamRange::new(-10.0, 10.0));
        assert_eq!(r, ParamRange::new(0.0, 4.0));
    }

    #[test]
    fn periodic_wrapping() {
        let tau = core::f64::consts::TAU;
        assert!((wrap_periodic(-0.5, 0.0, tau) - (tau - 0.5)).abs() < 1e-15);
        assert!((wrap_periodic(tau + 0.25, 0.0, tau) - 0.25).abs() < 1e-12);
        assert_eq!(wrap_periodic(0.25, 0.0, tau), 0.25);
    }
}
