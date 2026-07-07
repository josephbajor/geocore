//! Robust geometric predicates.
//!
//! Each predicate runs a fast floating-point evaluation guarded by a forward
//! error bound (Shewchuk's stage-A filter). When the filter cannot certify
//! the sign, it falls back to exact expansion arithmetic, so the returned
//! [`Orientation`] is always the true sign of the underlying determinant.
//!
//! Performance note: the fallback here is the *fully exact* computation
//! rather than Shewchuk's staged B/C/D adaptivity. That trades speed on
//! near-degenerate inputs for a much smaller trusted core. Staged adaptivity
//! is a planned optimization (roadmap M8) once profiling justifies it.

use crate::expansion;

/// Sign of a predicate's underlying determinant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Orientation {
    /// Determinant is negative.
    Negative,
    /// Determinant is exactly zero (degenerate configuration).
    Zero,
    /// Determinant is positive.
    Positive,
}

impl Orientation {
    #[inline]
    fn from_scalar(det: f64) -> Self {
        if det > 0.0 {
            Orientation::Positive
        } else if det < 0.0 {
            Orientation::Negative
        } else {
            Orientation::Zero
        }
    }

    #[inline]
    fn from_sign(s: i8) -> Self {
        match s.cmp(&0) {
            core::cmp::Ordering::Greater => Orientation::Positive,
            core::cmp::Ordering::Less => Orientation::Negative,
            core::cmp::Ordering::Equal => Orientation::Zero,
        }
    }

    /// Compact integer sign: -1, 0, or 1.
    #[inline]
    pub fn as_i8(self) -> i8 {
        match self {
            Orientation::Negative => -1,
            Orientation::Zero => 0,
            Orientation::Positive => 1,
        }
    }
}

/// Machine epsilon in Shewchuk's convention: 2^-53, half a ulp of 1.0.
const EPS: f64 = f64::EPSILON / 2.0;
/// Stage-A error bound coefficient for `orient2d` (Shewchuk `ccwerrboundA`).
const CCW_ERRBOUND_A: f64 = (3.0 + 16.0 * EPS) * EPS;
/// Stage-A error bound coefficient for `orient3d` (Shewchuk `o3derrboundA`).
const O3D_ERRBOUND_A: f64 = (7.0 + 56.0 * EPS) * EPS;

/// Orientation of point `c` relative to the directed line `a -> b`.
///
/// Returns [`Orientation::Positive`] when `a`, `b`, `c` wind counterclockwise
/// (i.e. `c` lies to the left of `a -> b`), the exact sign of
/// `det[[ax - cx, ay - cy], [bx - cx, by - cy]]`.
pub fn orient2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Orientation {
    let det_left = (a[0] - c[0]) * (b[1] - c[1]);
    let det_right = (a[1] - c[1]) * (b[0] - c[0]);
    let det = det_left - det_right;

    let det_sum = if det_left > 0.0 {
        if det_right <= 0.0 {
            return Orientation::from_scalar(det);
        }
        det_left + det_right
    } else if det_left < 0.0 {
        if det_right >= 0.0 {
            return Orientation::from_scalar(det);
        }
        -det_left - det_right
    } else {
        return Orientation::from_scalar(det);
    };

    let errbound = CCW_ERRBOUND_A * det_sum;
    if det >= errbound || -det >= errbound {
        return Orientation::from_scalar(det);
    }
    orient2d_exact(a, b, c)
}

/// Exact 2D orientation via the identity
/// `det = ax(by - cy) + bx(cy - ay) + cx(ay - by)`.
fn orient2d_exact(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Orientation {
    let (x, y) = expansion::two_diff(b[1], c[1]);
    let t1 = expansion::scale(&expansion::from_two(x, y), a[0]);
    let (x, y) = expansion::two_diff(c[1], a[1]);
    let t2 = expansion::scale(&expansion::from_two(x, y), b[0]);
    let (x, y) = expansion::two_diff(a[1], b[1]);
    let t3 = expansion::scale(&expansion::from_two(x, y), c[0]);
    let det = expansion::sum(&expansion::sum(&t1, &t2), &t3);
    Orientation::from_sign(expansion::sign(&det))
}

/// Orientation of point `d` relative to the plane through `a`, `b`, `c`.
///
/// Returns the exact sign of
/// `det[[a - d], [b - d], [c - d]]` (rows are 3-vectors):
/// [`Orientation::Positive`] when `d` lies on the side of the plane from
/// which `a`, `b`, `c` appear in clockwise order (equivalently, "below" a
/// counterclockwise triangle).
pub fn orient3d(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> Orientation {
    let adx = a[0] - d[0];
    let ady = a[1] - d[1];
    let adz = a[2] - d[2];
    let bdx = b[0] - d[0];
    let bdy = b[1] - d[1];
    let bdz = b[2] - d[2];
    let cdx = c[0] - d[0];
    let cdy = c[1] - d[1];
    let cdz = c[2] - d[2];

    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;
    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;

    let det = adz * (bdxcdy - cdxbdy) + bdz * (cdxady - adxcdy) + cdz * (adxbdy - bdxady);

    let permanent = (bdxcdy.abs() + cdxbdy.abs()) * adz.abs()
        + (cdxady.abs() + adxcdy.abs()) * bdz.abs()
        + (adxbdy.abs() + bdxady.abs()) * cdz.abs();
    let errbound = O3D_ERRBOUND_A * permanent;
    if det > errbound || -det > errbound {
        return Orientation::from_scalar(det);
    }
    orient3d_exact(a, b, c, d)
}

/// Exact 3D orientation: coordinate differences are computed exactly as
/// two-component expansions, then the determinant is expanded with exact
/// expansion algebra.
fn orient3d_exact(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> Orientation {
    let diff = |p: f64, q: f64| {
        let (x, y) = expansion::two_diff(p, q);
        expansion::from_two(x, y)
    };
    let adx = diff(a[0], d[0]);
    let ady = diff(a[1], d[1]);
    let adz = diff(a[2], d[2]);
    let bdx = diff(b[0], d[0]);
    let bdy = diff(b[1], d[1]);
    let bdz = diff(b[2], d[2]);
    let cdx = diff(c[0], d[0]);
    let cdy = diff(c[1], d[1]);
    let cdz = diff(c[2], d[2]);

    // (p*q - r*s) * z, all exact.
    let term = |p: &[f64], q: &[f64], r: &[f64], s: &[f64], z: &[f64]| {
        let pq = expansion::mul(p, q);
        let rs = expansion::mul(r, s);
        expansion::mul(&expansion::sum(&pq, &expansion::negate(&rs)), z)
    };
    let t1 = term(&bdx, &cdy, &cdx, &bdy, &adz);
    let t2 = term(&cdx, &ady, &adx, &cdy, &bdz);
    let t3 = term(&adx, &bdy, &bdx, &ady, &cdz);
    let det = expansion::sum(&expansion::sum(&t1, &t2), &t3);
    Orientation::from_sign(expansion::sign(&det))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic xorshift64 PRNG so tests never depend on external crates
    /// or platform randomness.
    struct Rng(u64);

    impl Rng {
        fn new(seed: u64) -> Self {
            Rng(seed.max(1))
        }
        fn next(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x
        }
        /// Uniform integer in `[-bound, bound]`.
        fn int(&mut self, bound: i64) -> i64 {
            let span = (2 * bound + 1) as u64;
            (self.next() % span) as i64 - bound
        }
    }

    /// Exact oracle: integer coordinates make the 2x2 determinant exactly
    /// computable in i128.
    fn orient2d_oracle(a: [i64; 2], b: [i64; 2], c: [i64; 2]) -> i8 {
        let det = (a[0] - c[0]) as i128 * (b[1] - c[1]) as i128
            - (a[1] - c[1]) as i128 * (b[0] - c[0]) as i128;
        det.signum() as i8
    }

    fn orient3d_oracle(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> i8 {
        let v = |p: [i64; 3]| {
            [
                (p[0] - d[0]) as i128,
                (p[1] - d[1]) as i128,
                (p[2] - d[2]) as i128,
            ]
        };
        let (r0, r1, r2) = (v(a), v(b), v(c));
        let det = r0[0] * (r1[1] * r2[2] - r1[2] * r2[1]) - r0[1] * (r1[0] * r2[2] - r1[2] * r2[0])
            + r0[2] * (r1[0] * r2[1] - r1[1] * r2[0]);
        det.signum() as i8
    }

    fn to2(p: [i64; 2]) -> [f64; 2] {
        [p[0] as f64, p[1] as f64]
    }
    fn to3(p: [i64; 3]) -> [f64; 3] {
        [p[0] as f64, p[1] as f64, p[2] as f64]
    }

    #[test]
    fn orient2d_matches_oracle_on_random_points() {
        let mut rng = Rng::new(0x9E37_79B9_7F4A_7C15);
        for _ in 0..20_000 {
            let a = [rng.int(1 << 20), rng.int(1 << 20)];
            let b = [rng.int(1 << 20), rng.int(1 << 20)];
            let c = [rng.int(1 << 20), rng.int(1 << 20)];
            assert_eq!(
                orient2d(to2(a), to2(b), to2(c)).as_i8(),
                orient2d_oracle(a, b, c),
                "a={a:?} b={b:?} c={c:?}"
            );
        }
    }

    #[test]
    fn orient2d_matches_oracle_near_degeneracy() {
        // Collinear triples with unit-scale perturbations: large coordinates,
        // tiny (or zero) determinants — exactly where the filter must punt to
        // the exact path and the exact path must be right.
        let mut rng = Rng::new(0xDEAD_BEEF_CAFE_F00D);
        for _ in 0..20_000 {
            let a = [rng.int(1 << 20), rng.int(1 << 20)];
            let dir = [rng.int(1 << 10), rng.int(1 << 10)];
            let b = [a[0] + dir[0] * rng.int(8), a[1] + dir[1] * rng.int(8)];
            let mut c = [a[0] + dir[0] * rng.int(8), a[1] + dir[1] * rng.int(8)];
            c[0] += rng.int(1);
            c[1] += rng.int(1);
            assert_eq!(
                orient2d(to2(a), to2(b), to2(c)).as_i8(),
                orient2d_oracle(a, b, c),
                "a={a:?} b={b:?} c={c:?}"
            );
        }
    }

    #[test]
    fn orient3d_matches_oracle_on_random_points() {
        let mut rng = Rng::new(0x0123_4567_89AB_CDEF);
        for _ in 0..20_000 {
            let p = |rng: &mut Rng| [rng.int(1 << 18), rng.int(1 << 18), rng.int(1 << 18)];
            let (a, b, c, d) = (p(&mut rng), p(&mut rng), p(&mut rng), p(&mut rng));
            assert_eq!(
                orient3d(to3(a), to3(b), to3(c), to3(d)).as_i8(),
                orient3d_oracle(a, b, c, d),
                "a={a:?} b={b:?} c={c:?} d={d:?}"
            );
        }
    }

    #[test]
    fn orient3d_matches_oracle_near_coplanarity() {
        // d starts in the plane spanned by (a, b, c) via integer combinations,
        // then gets a perturbation in {-1, 0, 1}^3.
        let mut rng = Rng::new(0xFEED_FACE_0BAD_F00D);
        for _ in 0..20_000 {
            let p = |rng: &mut Rng| [rng.int(1 << 16), rng.int(1 << 16), rng.int(1 << 16)];
            let (a, b, c) = (p(&mut rng), p(&mut rng), p(&mut rng));
            let (u, v) = (rng.int(4), rng.int(4));
            let mut d = [0_i64; 3];
            for k in 0..3 {
                d[k] = a[k] + u * (b[k] - a[k]) + v * (c[k] - a[k]) + rng.int(1);
            }
            assert_eq!(
                orient3d(to3(a), to3(b), to3(c), to3(d)).as_i8(),
                orient3d_oracle(a, b, c, d),
                "a={a:?} b={b:?} c={c:?} d={d:?}"
            );
        }
    }

    #[test]
    fn orient2d_exact_zero_on_collinear() {
        assert_eq!(
            orient2d([0.0, 0.0], [1e10, 1e10], [2e10, 2e10]),
            Orientation::Zero
        );
    }

    #[test]
    fn orient3d_sign_convention() {
        // Unit triangle in z = 0, counterclockwise seen from +z;
        // d below the plane (negative z) must be Positive.
        let a = [0.0, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        let c = [0.0, 1.0, 0.0];
        assert_eq!(orient3d(a, b, c, [0.3, 0.3, -1.0]), Orientation::Positive);
        assert_eq!(orient3d(a, b, c, [0.3, 0.3, 1.0]), Orientation::Negative);
        assert_eq!(orient3d(a, b, c, [5.0, -3.0, 0.0]), Orientation::Zero);
    }
}
