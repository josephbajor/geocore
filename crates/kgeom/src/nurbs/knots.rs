//! Validated knot vectors.

use crate::param::ParamRange;
use kcore::error::{Error, Result};

/// A validated B-spline knot vector bound to a degree.
///
/// Invariants established at construction and relied on everywhere else in
/// the engine:
/// - `degree >= 1`;
/// - `knots.len() >= 2 * degree + 2` (at least `degree + 1` control points);
/// - all knots finite and non-decreasing;
/// - positive-width domain `[knots[p], knots[m - p]]` (`m = len - 1`);
/// - multiplicity of any interior knot value ≤ `degree`, of any knot value
///   ≤ `degree + 1`.
///
/// Multiplicity queries use exact (`==`) comparison: knot values inserted by
/// the engine are stored bit-exactly, and XT files carry exact doubles, so
/// tolerance-based knot comparison is deliberately avoided (it would make
/// multiplicity depend on model scale).
#[derive(Debug, Clone, PartialEq)]
pub struct KnotVector {
    degree: usize,
    knots: Vec<f64>,
}

impl KnotVector {
    /// Validated construction from a degree and raw knots.
    pub fn new(degree: usize, knots: Vec<f64>) -> Result<KnotVector> {
        if degree < 1 {
            return Err(Error::InvalidGeometry {
                reason: "knot vector degree must be at least 1",
            });
        }
        if knots.len() < 2 * degree + 2 {
            return Err(Error::InvalidGeometry {
                reason: "knot vector too short for degree",
            });
        }
        if knots.iter().any(|k| !k.is_finite()) {
            return Err(Error::InvalidGeometry {
                reason: "knot values must be finite",
            });
        }
        if knots.windows(2).any(|w| w[0] > w[1]) {
            return Err(Error::InvalidGeometry {
                reason: "knot values must be non-decreasing",
            });
        }
        let m = knots.len() - 1;
        let (lo, hi) = (knots[degree], knots[m - degree]);
        if lo >= hi {
            return Err(Error::InvalidGeometry {
                reason: "knot vector domain has zero width",
            });
        }
        // Multiplicity limits: ≤ degree + 1 anywhere, ≤ degree strictly
        // inside the domain.
        let mut i = 0;
        while i < knots.len() {
            let mut j = i;
            while j + 1 < knots.len() && knots[j + 1] == knots[i] {
                j += 1;
            }
            let mult = j - i + 1;
            if mult > degree + 1 {
                return Err(Error::InvalidGeometry {
                    reason: "knot multiplicity exceeds degree + 1",
                });
            }
            if mult > degree && knots[i] > lo && knots[i] < hi {
                return Err(Error::InvalidGeometry {
                    reason: "interior knot multiplicity exceeds degree",
                });
            }
            i = j + 1;
        }
        Ok(KnotVector { degree, knots })
    }

    /// Degree this knot vector is bound to.
    pub fn degree(&self) -> usize {
        self.degree
    }

    /// Raw knot values.
    pub fn as_slice(&self) -> &[f64] {
        &self.knots
    }

    /// Number of control points the vector supports
    /// (`len - degree - 1`).
    pub fn control_count(&self) -> usize {
        self.knots.len() - self.degree - 1
    }

    /// Parameter domain `[knots[p], knots[m - p]]`.
    pub fn domain(&self) -> ParamRange {
        let m = self.knots.len() - 1;
        ParamRange::new(self.knots[self.degree], self.knots[m - self.degree])
    }

    /// True if the vector is clamped (first and last knots repeated
    /// `degree + 1` times).
    pub fn is_clamped(&self) -> bool {
        let p = self.degree;
        let n = self.knots.len();
        self.knots[..=p].iter().all(|&k| k == self.knots[0])
            && self.knots[n - p - 1..]
                .iter()
                .all(|&k| k == self.knots[n - 1])
    }

    /// Multiplicity of the exact value `u` among the knots.
    pub fn multiplicity(&self, u: f64) -> usize {
        self.knots.iter().filter(|&&k| k == u).count()
    }

    /// Knot span index containing `u` (A2.1 `FindSpan`): the unique `i` with
    /// `knots[i] <= u < knots[i + 1]`, special-cased to the last span at the
    /// domain end. `u` must lie in the domain.
    pub fn find_span(&self, u: f64) -> usize {
        let p = self.degree;
        let n = self.control_count() - 1;
        let k = &self.knots;
        debug_assert!(self.domain().contains(u), "parameter {u} outside domain");
        if u >= k[n + 1] {
            return n;
        }
        if u <= k[p] {
            return p;
        }
        let (mut low, mut high) = (p, n + 1);
        let mut mid = (low + high) / 2;
        while u < k[mid] || u >= k[mid + 1] {
            if u < k[mid] {
                high = mid;
            } else {
                low = mid;
            }
            mid = (low + high) / 2;
        }
        mid
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cubic() -> KnotVector {
        KnotVector::new(3, vec![0.0, 0.0, 0.0, 0.0, 0.35, 0.65, 1.0, 1.0, 1.0, 1.0]).unwrap()
    }

    #[test]
    fn spans_bracket_their_parameter() {
        let kv = cubic();
        let k = kv.as_slice();
        for i in 0..=100 {
            let u = i as f64 / 100.0;
            let s = kv.find_span(u);
            assert!(k[s] <= u, "u = {u}, span = {s}");
            assert!(u < k[s + 1] || u == kv.domain().hi, "u = {u}, span = {s}");
        }
        assert_eq!(kv.find_span(1.0), kv.control_count() - 1);
    }

    #[test]
    fn properties() {
        let kv = cubic();
        assert_eq!(kv.control_count(), 6);
        assert_eq!(kv.domain(), ParamRange::new(0.0, 1.0));
        assert!(kv.is_clamped());
        assert_eq!(kv.multiplicity(0.0), 4);
        assert_eq!(kv.multiplicity(0.35), 1);
        assert_eq!(kv.multiplicity(0.5), 0);
        let unclamped = KnotVector::new(2, vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]).unwrap();
        assert!(!unclamped.is_clamped());
    }

    #[test]
    fn invalid_vectors_rejected() {
        // Too short.
        assert!(KnotVector::new(3, vec![0.0, 0.0, 1.0, 1.0]).is_err());
        // Decreasing.
        assert!(KnotVector::new(1, vec![0.0, 0.5, 0.4, 1.0]).is_err());
        // Non-finite.
        assert!(KnotVector::new(1, vec![0.0, 0.0, f64::NAN, 1.0]).is_err());
        // Zero-width domain.
        assert!(KnotVector::new(1, vec![0.0, 0.0, 0.0, 0.0]).is_err());
        // Interior multiplicity above degree.
        assert!(KnotVector::new(2, vec![0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0]).is_err());
        // Degree zero.
        assert!(KnotVector::new(0, vec![0.0, 1.0]).is_err());
    }
}
