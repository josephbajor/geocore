//! Conservative interval arithmetic.
//!
//! Intervals here are *filters*: an operation's true real-valued result is
//! guaranteed to lie within the returned interval, so a sign or range test
//! that succeeds on the interval is certain, and an inconclusive test routes
//! to an exact path. Conservatism is achieved by widening every computed
//! bound one ulp outward rather than by directed rounding modes (which Rust
//! cannot portably control); this over-widens by at most one ulp per side
//! per operation, which is irrelevant for filtering.

/// A closed interval `[lo, hi]` of finite `f64`s with `lo <= hi`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Interval {
    lo: f64,
    hi: f64,
}

impl Interval {
    /// Interval from explicit bounds. Panics if `lo > hi` or either bound is NaN.
    pub fn new(lo: f64, hi: f64) -> Self {
        assert!(lo <= hi, "invalid interval [{lo}, {hi}]");
        Interval { lo, hi }
    }

    /// Degenerate interval containing exactly `x`.
    pub fn point(x: f64) -> Self {
        assert!(!x.is_nan());
        Interval { lo: x, hi: x }
    }

    /// Lower bound.
    pub fn lo(self) -> f64 {
        self.lo
    }

    /// Upper bound.
    pub fn hi(self) -> f64 {
        self.hi
    }

    /// Width `hi - lo`.
    pub fn width(self) -> f64 {
        self.hi - self.lo
    }

    /// True if `x` lies in the interval.
    pub fn contains(self, x: f64) -> bool {
        self.lo <= x && x <= self.hi
    }

    /// True if the interval contains zero, i.e. the sign of the represented
    /// value is not certified.
    pub fn contains_zero(self) -> bool {
        self.contains(0.0)
    }

    /// Certified sign: `Some(1)` / `Some(-1)` when the whole interval is
    /// strictly one-signed, `Some(0)` for the degenerate zero interval,
    /// `None` when the sign is undecided.
    pub fn sign(self) -> Option<i8> {
        if self.lo > 0.0 {
            Some(1)
        } else if self.hi < 0.0 {
            Some(-1)
        } else if self.lo == 0.0 && self.hi == 0.0 {
            Some(0)
        } else {
            None
        }
    }

    /// True if the two intervals share at least one point.
    pub fn intersects(self, other: Interval) -> bool {
        self.lo <= other.hi && other.lo <= self.hi
    }

    /// Widen each computed bound one ulp outward to absorb rounding error.
    #[inline]
    fn widened(lo: f64, hi: f64) -> Self {
        Interval {
            lo: lo.next_down(),
            hi: hi.next_up(),
        }
    }

    /// Conservative square: tighter than `self * self` because the two
    /// factors are correlated.
    pub fn square(self) -> Self {
        if self.lo >= 0.0 {
            Self::widened(self.lo * self.lo, self.hi * self.hi)
        } else if self.hi <= 0.0 {
            Self::widened(self.hi * self.hi, self.lo * self.lo)
        } else {
            let m = (-self.lo).max(self.hi);
            Self::widened(0.0, m * m)
        }
    }

    /// Conservative square root over the non-negative part of the interval.
    ///
    /// A slightly negative lower bound is clamped to zero, which is useful
    /// when an algebraically non-negative expression was widened below zero.
    /// Returns `None` only when the complete interval is negative or either
    /// endpoint is NaN.
    pub fn sqrt(self) -> Option<Self> {
        if self.lo.is_nan() || self.hi.is_nan() || self.hi < 0.0 {
            return None;
        }
        let lo = if self.lo <= 0.0 {
            0.0
        } else {
            self.lo.sqrt().next_down()
        };
        let hi = self.hi.sqrt().next_up();
        Some(Interval { lo, hi })
    }
}

impl core::ops::Add for Interval {
    type Output = Interval;
    /// Conservative sum.
    fn add(self, rhs: Interval) -> Interval {
        Self::widened(self.lo + rhs.lo, self.hi + rhs.hi)
    }
}

impl core::ops::Sub for Interval {
    type Output = Interval;
    /// Conservative difference.
    fn sub(self, rhs: Interval) -> Interval {
        Self::widened(self.lo - rhs.hi, self.hi - rhs.lo)
    }
}

impl core::ops::Neg for Interval {
    type Output = Interval;
    /// Exact negation.
    fn neg(self) -> Interval {
        Interval {
            lo: -self.hi,
            hi: -self.lo,
        }
    }
}

impl core::ops::Mul for Interval {
    type Output = Interval;
    /// Conservative product (min/max over the four endpoint products).
    fn mul(self, rhs: Interval) -> Interval {
        let p = [
            self.lo * rhs.lo,
            self.lo * rhs.hi,
            self.hi * rhs.lo,
            self.hi * rhs.hi,
        ];
        let mut lo = p[0];
        let mut hi = p[0];
        for &v in &p[1..] {
            lo = lo.min(v);
            hi = hi.max(v);
        }
        Self::widened(lo, hi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_arithmetic_brackets_true_result() {
        let a = Interval::point(0.1);
        let b = Interval::point(0.2);
        let s = a + b;
        // The true real sum 0.3 is not an f64; the interval must contain it,
        // which it does iff it contains both neighbors of the rounding.
        assert!(s.contains(0.1 + 0.2));
        assert!(s.contains(0.3));
    }

    #[test]
    fn endpoint_samples_stay_inside() {
        // Deterministic pseudo-random check that samples of x op y land in
        // X op Y for endpoint and midpoint samples.
        let mut state = 0x853C_49E6_748F_EA9B_u64;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state as f64 / u64::MAX as f64) * 1000.0 - 500.0
        };
        for _ in 0..5_000 {
            let (a, b) = (next(), next());
            let (c, d) = (next(), next());
            let x = Interval::new(a.min(b), a.max(b));
            let y = Interval::new(c.min(d), c.max(d));
            for &u in &[x.lo(), x.hi(), (x.lo() + x.hi()) / 2.0] {
                for &v in &[y.lo(), y.hi(), (y.lo() + y.hi()) / 2.0] {
                    assert!((x + y).contains(u + v));
                    assert!((x - y).contains(u - v));
                    assert!((x * y).contains(u * v));
                    assert!(x.square().contains(u * u));
                    assert!((-x).contains(-u));
                }
            }
        }
    }

    #[test]
    fn sign_certification() {
        assert_eq!(Interval::new(1.0, 2.0).sign(), Some(1));
        assert_eq!(Interval::new(-2.0, -1.0).sign(), Some(-1));
        assert_eq!(Interval::point(0.0).sign(), Some(0));
        assert_eq!(Interval::new(-1.0, 1.0).sign(), None);
    }

    #[test]
    fn square_straddling_zero_is_nonnegative() {
        let s = Interval::new(-3.0, 2.0).square();
        assert!(s.lo() <= 0.0);
        assert!(s.contains(9.0));
        assert!(s.contains(0.0));
    }

    #[test]
    fn square_root_is_outward_rounded_and_domain_aware() {
        let root = Interval::new(2.0, 9.0).sqrt().unwrap();
        assert!(root.lo() < 2.0_f64.sqrt());
        assert!(root.contains(3.0));

        let widened_nonnegative = Interval::new(-f64::EPSILON, 4.0).sqrt().unwrap();
        assert_eq!(widened_nonnegative.lo(), 0.0);
        assert!(widened_nonnegative.contains(2.0));
        assert!(Interval::new(-4.0, -1.0).sqrt().is_none());
    }
}
