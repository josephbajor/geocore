//! Shared knot-operation machinery: homogeneous points, convex combination,
//! and knot insertion on raw control-point slices.
//!
//! Insertion is written once, generically over the control-point type, and
//! reused by curves (3D or homogeneous 4D points) and by surfaces
//! (row/column-wise application).

use super::knots::KnotVector;
use crate::vec::{Point3, Vec2, Vec3};
use kcore::error::{Error, Result};

/// Types that support the convex combination used by knot insertion.
pub(crate) trait Comb: Copy {
    /// `(1 - alpha) * a + alpha * b`.
    fn comb(a: Self, b: Self, alpha: f64) -> Self;
}

impl Comb for Vec3 {
    fn comb(a: Self, b: Self, alpha: f64) -> Self {
        a * (1.0 - alpha) + b * alpha
    }
}

impl Comb for Vec2 {
    fn comb(a: Self, b: Self, alpha: f64) -> Self {
        a * (1.0 - alpha) + b * alpha
    }
}

/// A homogeneous control point `(w·x, w·y, w·z, w)`. Rational knot
/// operations act on these so that the projective (rational) geometry is
/// preserved exactly.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct Hpt {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Hpt {
    /// Lift a Euclidean control point with weight `w`.
    pub fn lift(p: Point3, w: f64) -> Hpt {
        Hpt {
            x: p.x * w,
            y: p.y * w,
            z: p.z * w,
            w,
        }
    }

    /// Project back to a Euclidean control point and weight.
    pub fn project(self) -> (Point3, f64) {
        (
            Point3::new(self.x / self.w, self.y / self.w, self.z / self.w),
            self.w,
        )
    }
}

impl Comb for Hpt {
    fn comb(a: Self, b: Self, alpha: f64) -> Self {
        let s = 1.0 - alpha;
        Hpt {
            x: a.x * s + b.x * alpha,
            y: a.y * s + b.y * alpha,
            z: a.z * s + b.z * alpha,
            w: a.w * s + b.w * alpha,
        }
    }
}

/// Insert the knot `u`, `r` times, into `(kv, points)` (A5.1
/// `CurveKnotIns`). `u` must lie strictly inside the domain; the resulting
/// multiplicity must not exceed the degree. Returns the refined knots and
/// control points; the curve's point set is unchanged.
pub(crate) fn insert_knot<T: Comb>(
    kv: &KnotVector,
    points: &[T],
    u: f64,
    r: usize,
) -> Result<(Vec<f64>, Vec<T>)> {
    let p = kv.degree();
    let domain = kv.domain();
    if !(domain.lo < u && u < domain.hi) {
        return Err(Error::InvalidGeometry {
            reason: "inserted knot must lie strictly inside the domain",
        });
    }
    if r == 0 {
        return Err(Error::InvalidGeometry {
            reason: "knot insertion count must be positive",
        });
    }
    let s = kv.multiplicity(u);
    if r + s > p {
        return Err(Error::InvalidGeometry {
            reason: "knot insertion would exceed degree multiplicity",
        });
    }
    let knots = kv.as_slice();
    let np = points.len() - 1;
    let k = kv.find_span(u);

    // New knot vector.
    let mut uq = Vec::with_capacity(knots.len() + r);
    uq.extend_from_slice(&knots[..=k]);
    uq.extend(core::iter::repeat_n(u, r));
    uq.extend_from_slice(&knots[k + 1..]);

    // New control points.
    let mut qw: Vec<Option<T>> = vec![None; np + 1 + r];
    for i in 0..=k - p {
        qw[i] = Some(points[i]);
    }
    for i in k - s..=np {
        qw[i + r] = Some(points[i]);
    }
    let mut rw: Vec<T> = points[k - p..=k - s].to_vec();
    let mut l = 0;
    for j in 1..=r {
        l = k - p + j;
        for i in 0..=p - j - s {
            let alpha = (u - knots[l + i]) / (knots[i + k + 1] - knots[l + i]);
            rw[i] = T::comb(rw[i], rw[i + 1], alpha);
        }
        qw[l] = Some(rw[0]);
        qw[k + r - j - s] = Some(rw[p - j - s]);
    }
    for i in l + 1..k.saturating_sub(s) {
        qw[i] = Some(rw[i - l]);
    }
    let qw: Vec<T> = qw
        .into_iter()
        .map(|q| q.expect("A5.1 must assign every control point"))
        .collect();
    Ok((uq, qw))
}

/// Refine by inserting every value of `xs` (each once per occurrence).
/// Semantically A5.4 `RefineKnotVectCurve`; implemented as repeated A5.1 for
/// a smaller trusted core — refinement is O(n·r) instead of O(n + r), which
/// is irrelevant off the hot path (fitting, splitting, extraction).
pub(crate) fn refine_knots<T: Comb>(
    degree: usize,
    kv: &KnotVector,
    points: &[T],
    xs: &[f64],
) -> Result<(Vec<f64>, Vec<T>)> {
    let mut knots = kv.as_slice().to_vec();
    let mut pts = points.to_vec();
    for &x in xs {
        let kv = KnotVector::new(degree, knots)?;
        let (nk, np) = insert_knot(&kv, &pts, x, 1)?;
        knots = nk;
        pts = np;
    }
    Ok((knots, pts))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn homogeneous_roundtrip() {
        let p = Point3::new(1.0, -2.0, 3.0);
        let (q, w) = Hpt::lift(p, 2.5).project();
        assert!((q - p).norm() < 1e-15);
        assert_eq!(w, 2.5);
    }
}
