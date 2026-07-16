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
/// Stage-A error bound coefficient for `incircle` (Shewchuk `iccerrboundA`).
const ICC_ERRBOUND_A: f64 = (10.0 + 96.0 * EPS) * EPS;

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

/// Exact orientation of a closed 2D polygon.
///
/// Returns the sign of the exact cyclic shoelace sum
/// `sum_i(x_i * y_(i+1) - y_i * x_(i+1))`: positive for counterclockwise
/// winding and negative for clockwise winding. The result is invariant under
/// cyclic rotation of the vertices, and reversing their order reverses every
/// nonzero result.
///
/// Fewer than three vertices, any non-finite coordinate, or an exactly zero
/// shoelace sum returns [`Orientation::Zero`]. Repeated vertices are allowed;
/// self-intersecting polygons retain the sign of their algebraic area.
pub fn polygon_orientation2d(points: &[[f64; 2]]) -> Orientation {
    polygon_orientation2d_iter(points.iter().copied())
}

/// Streaming form of [`polygon_orientation2d`].
///
/// This evaluates the same exact cyclic shoelace expansion without retaining
/// or allocating a copy of the input. It has the same winding convention and
/// returns [`Orientation::Zero`] for fewer than three vertices, any non-finite
/// coordinate, or an exactly zero shoelace sum.
pub fn polygon_orientation2d_iter(points: impl IntoIterator<Item = [f64; 2]>) -> Orientation {
    let mut twice_area = vec![0.0];
    let mut first = None;
    let mut previous = None;
    let mut count = 0_usize;
    for point in points {
        if point.iter().any(|coordinate| !coordinate.is_finite()) {
            return Orientation::Zero;
        }
        if let Some(previous) = previous {
            accumulate_shoelace_cross(&mut twice_area, previous, point);
        } else {
            first = Some(point);
        }
        previous = Some(point);
        count = count.saturating_add(1);
    }
    if count < 3 {
        return Orientation::Zero;
    }
    let (Some(previous), Some(first)) = (previous, first) else {
        return Orientation::Zero;
    };
    accumulate_shoelace_cross(&mut twice_area, previous, first);
    Orientation::from_sign(expansion::sign(&twice_area))
}

fn accumulate_shoelace_cross(twice_area: &mut Vec<f64>, point: [f64; 2], next: [f64; 2]) {
    let (product, error) = expansion::two_product(point[0], next[1]);
    let positive = expansion::from_two(product, error);
    let (product, error) = expansion::two_product(point[1], next[0]);
    let negative = expansion::from_two(product, error);
    let cross = expansion::sum(&positive, &expansion::negate(&negative));
    *twice_area = expansion::sum(twice_area, &cross);
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

/// Position of point `d` relative to the oriented circumcircle through
/// `a`, `b`, and `c`.
///
/// For counterclockwise `a`, `b`, `c`, returns [`Orientation::Positive`] when
/// `d` lies inside the circumcircle, [`Orientation::Negative`] when it lies
/// outside, and [`Orientation::Zero`] when the four points are cocircular.
/// Reversing the orientation of `a`, `b`, `c` reverses the returned sign.
/// Degenerate inputs retain the exact sign of the same lifted determinant.
pub fn incircle(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> Orientation {
    let adx = a[0] - d[0];
    let ady = a[1] - d[1];
    let bdx = b[0] - d[0];
    let bdy = b[1] - d[1];
    let cdx = c[0] - d[0];
    let cdy = c[1] - d[1];

    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;
    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;

    let alift = adx * adx + ady * ady;
    let blift = bdx * bdx + bdy * bdy;
    let clift = cdx * cdx + cdy * cdy;
    let det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);

    let permanent = (bdxcdy.abs() + cdxbdy.abs()) * alift
        + (cdxady.abs() + adxcdy.abs()) * blift
        + (adxbdy.abs() + bdxady.abs()) * clift;
    let errbound = ICC_ERRBOUND_A * permanent;
    if det > errbound || -det > errbound {
        return Orientation::from_scalar(det);
    }
    incircle_exact(a, b, c, d)
}

/// Exact 2D in-circle determinant. Coordinate differences, squared lifts,
/// oriented areas, and their products all remain expansions, so no rounded
/// intermediate decides the sign.
fn incircle_exact(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> Orientation {
    let diff = |p: f64, q: f64| {
        let (x, y) = expansion::two_diff(p, q);
        expansion::from_two(x, y)
    };
    let adx = diff(a[0], d[0]);
    let ady = diff(a[1], d[1]);
    let bdx = diff(b[0], d[0]);
    let bdy = diff(b[1], d[1]);
    let cdx = diff(c[0], d[0]);
    let cdy = diff(c[1], d[1]);

    let cross = |p: &[f64], q: &[f64], r: &[f64], s: &[f64]| {
        let pq = expansion::mul(p, q);
        let rs = expansion::mul(r, s);
        expansion::sum(&pq, &expansion::negate(&rs))
    };
    let lift = |x: &[f64], y: &[f64]| expansion::sum(&expansion::mul(x, x), &expansion::mul(y, y));

    let alift = lift(&adx, &ady);
    let blift = lift(&bdx, &bdy);
    let clift = lift(&cdx, &cdy);
    let bcdet = cross(&bdx, &cdy, &cdx, &bdy);
    let cadet = cross(&cdx, &ady, &adx, &cdy);
    let abdet = cross(&adx, &bdy, &bdx, &ady);

    let adet = expansion::mul(&alift, &bcdet);
    let bdet = expansion::mul(&blift, &cadet);
    let cdet = expansion::mul(&clift, &abdet);
    let det = expansion::sum(&expansion::sum(&adet, &bdet), &cdet);
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

    fn incircle_oracle(a: [i64; 2], b: [i64; 2], c: [i64; 2], d: [i64; 2]) -> i8 {
        let v = |p: [i64; 2]| [(p[0] - d[0]) as i128, (p[1] - d[1]) as i128];
        let (a, b, c) = (v(a), v(b), v(c));
        let cross = |p: [i128; 2], q: [i128; 2]| p[0] * q[1] - q[0] * p[1];
        let lift = |p: [i128; 2]| p[0] * p[0] + p[1] * p[1];
        let det = lift(a) * cross(b, c) + lift(b) * cross(c, a) + lift(c) * cross(a, b);
        det.signum() as i8
    }

    fn polygon_orientation2d_oracle(points: &[[i64; 2]]) -> i8 {
        if points.len() < 3 {
            return 0;
        }
        points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .map(|(point, next)| {
                i128::from(point[0]) * i128::from(next[1])
                    - i128::from(point[1]) * i128::from(next[0])
            })
            .sum::<i128>()
            .signum() as i8
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
    fn polygon_orientation2d_matches_random_integer_oracle() {
        let mut rng = Rng::new(0xA11C_E5E7_0ACE_2D00);
        for _ in 0..20_000 {
            let len = 3 + (rng.next() % 10) as usize;
            let points = (0..len)
                .map(|_| [rng.int(1 << 20), rng.int(1 << 20)])
                .collect::<Vec<_>>();
            let coordinates = points.iter().copied().map(to2).collect::<Vec<_>>();
            assert_eq!(
                polygon_orientation2d(&coordinates).as_i8(),
                polygon_orientation2d_oracle(&points),
                "points={points:?}"
            );
        }
    }

    #[test]
    fn polygon_orientation2d_resolves_large_cancellation_deterministically() {
        let magnitude = 2f64.powi(52);
        let points = vec![
            [magnitude, magnitude],
            [magnitude + 1.0, magnitude],
            [magnitude + 1.0, magnitude + 1.0],
            [magnitude, magnitude + 1.0],
        ];
        let naive_twice_area = points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .map(|(point, next)| point[0] * next[1] - point[1] * next[0])
            .sum::<f64>();
        assert_eq!(naive_twice_area, 0.0);

        for rotation in 0..points.len() {
            let mut rotated = points.clone();
            rotated.rotate_left(rotation);
            for _ in 0..32 {
                assert_eq!(polygon_orientation2d(&rotated), Orientation::Positive);
                assert_eq!(
                    polygon_orientation2d_iter(rotated.iter().copied()),
                    Orientation::Positive
                );
            }

            rotated.reverse();
            assert_eq!(polygon_orientation2d(&rotated), Orientation::Negative);
        }
    }

    #[test]
    fn polygon_orientation2d_fails_closed_on_invalid_or_exact_zero_input() {
        assert_eq!(polygon_orientation2d(&[]), Orientation::Zero);
        assert_eq!(
            polygon_orientation2d(&[[0.0, 0.0], [1.0, 0.0]]),
            Orientation::Zero
        );
        assert_eq!(
            polygon_orientation2d(&[[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]),
            Orientation::Zero
        );
        assert_eq!(
            polygon_orientation2d(&[[0.0, 0.0], [f64::NAN, 1.0], [1.0, 0.0]]),
            Orientation::Zero
        );
        assert_eq!(
            polygon_orientation2d(&[[0.0, 0.0], [f64::INFINITY, 1.0], [1.0, 0.0]]),
            Orientation::Zero
        );
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
    fn incircle_matches_oracle_on_random_integer_points() {
        let mut rng = Rng::new(0xC1AC_1E00_51A7_0BAD);
        for _ in 0..20_000 {
            let p = |rng: &mut Rng| [rng.int(1 << 15), rng.int(1 << 15)];
            let (a, b, c, d) = (p(&mut rng), p(&mut rng), p(&mut rng), p(&mut rng));
            assert_eq!(
                incircle(to2(a), to2(b), to2(c), to2(d)).as_i8(),
                incircle_oracle(a, b, c, d),
                "a={a:?} b={b:?} c={c:?} d={d:?}"
            );
        }
    }

    #[test]
    fn incircle_sign_convention_and_permutations() {
        let a = [0.0, 0.0];
        let b = [4.0, 0.0];
        let c = [0.0, 4.0];
        let inside = [1.0, 1.0];
        let outside = [5.0, 5.0];
        let boundary = [4.0, 4.0];

        assert_eq!(orient2d(a, b, c), Orientation::Positive);
        for ([a, b, c], winding) in [
            ([a, b, c], Orientation::Positive),
            ([b, c, a], Orientation::Positive),
            ([c, a, b], Orientation::Positive),
            ([a, c, b], Orientation::Negative),
            ([c, b, a], Orientation::Negative),
            ([b, a, c], Orientation::Negative),
        ] {
            let opposite = match winding {
                Orientation::Positive => Orientation::Negative,
                Orientation::Negative => Orientation::Positive,
                Orientation::Zero => unreachable!("the fixture triangle is not degenerate"),
            };
            assert_eq!(orient2d(a, b, c), winding);
            assert_eq!(incircle(a, b, c, inside), winding);
            assert_eq!(incircle(a, b, c, outside), opposite);
            assert_eq!(incircle(a, b, c, boundary), Orientation::Zero);
        }
    }

    #[test]
    fn incircle_exact_fallback_resolves_near_cocircular_points() {
        // At this scale, moving the fourth point by one changes the exact
        // determinant, but the stage-A bound deliberately cannot certify the
        // rounded determinant. The full expansion path must retain the sign.
        let radius = 2f64.powi(51);
        let a = [radius, 0.0];
        let b = [0.0, radius];
        let c = [-radius, 0.0];
        let stage_a_is_uncertain = |d: [f64; 2]| {
            let adx = a[0] - d[0];
            let ady = a[1] - d[1];
            let bdx = b[0] - d[0];
            let bdy = b[1] - d[1];
            let cdx = c[0] - d[0];
            let cdy = c[1] - d[1];
            let bdxcdy = bdx * cdy;
            let cdxbdy = cdx * bdy;
            let cdxady = cdx * ady;
            let adxcdy = adx * cdy;
            let adxbdy = adx * bdy;
            let bdxady = bdx * ady;
            let alift = adx * adx + ady * ady;
            let blift = bdx * bdx + bdy * bdy;
            let clift = cdx * cdx + cdy * cdy;
            let det =
                alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
            let permanent = (bdxcdy.abs() + cdxbdy.abs()) * alift
                + (cdxady.abs() + adxcdy.abs()) * blift
                + (adxbdy.abs() + bdxady.abs()) * clift;
            det.abs() <= ICC_ERRBOUND_A * permanent
        };
        for d in [[0.0, -radius + 1.0], [0.0, -radius], [0.0, -radius - 1.0]] {
            assert!(
                stage_a_is_uncertain(d),
                "fixture must exercise exact fallback"
            );
        }
        assert_eq!(
            incircle(a, b, c, [0.0, -radius + 1.0]),
            Orientation::Positive
        );
        assert_eq!(incircle(a, b, c, [0.0, -radius]), Orientation::Zero);
        assert_eq!(
            incircle(a, b, c, [0.0, -radius - 1.0]),
            Orientation::Negative
        );
    }

    #[test]
    fn incircle_degenerate_and_nonfinite_inputs_do_not_panic() {
        let a = [0.0, 0.0];
        let b = [1.0, 0.0];
        let d = [0.0, 1.0];
        assert_eq!(incircle(a, a, b, d), Orientation::Zero);
        assert_eq!(incircle(a, b, [2.0, 0.0], [3.0, 0.0]), Orientation::Zero);
        // A collinear defining triple has no geometric circumcircle, but the
        // predicate still returns the exact sign of its lifted determinant.
        assert_eq!(incircle(a, b, [2.0, 0.0], d), Orientation::Positive);
        assert_eq!(incircle(a, [2.0, 0.0], b, d), Orientation::Negative);

        // Like the existing orientation predicates, invalid non-finite input
        // is not ordered and therefore maps to the neutral sign without a
        // panic; callers validate finite geometry at their API boundary.
        assert_eq!(incircle([f64::NAN, 0.0], a, b, d), Orientation::Zero);
        assert_eq!(incircle([f64::INFINITY, 0.0], a, b, d), Orientation::Zero);
        assert_eq!(orient2d([f64::NAN, 0.0], a, b), Orientation::Zero);
        assert_eq!(
            orient3d(
                [f64::INFINITY, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0]
            ),
            Orientation::Zero
        );
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
