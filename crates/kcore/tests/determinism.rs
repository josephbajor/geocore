//! Determinism harness.
//!
//! Computes a fixed pseudo-random batch of predicate and interval results and
//! folds every output bit into one FNV-1a hash, pinned to a golden constant.
//! CI runs this on Linux, macOS, and Windows: any platform-, codegen-, or
//! refactor-induced drift in numeric results fails loudly here.
//!
//! If a change *intentionally* alters numeric behavior, that is a reviewed
//! event: update the golden value in the same commit and say why.

use kcore::interval::Interval;
use kcore::predicates::{
    affine_dot3, harmonic_half_angle_roots, incircle, orient2d, orient3d, polygon_orientation2d,
    quadratic_discriminant,
};

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    /// Uniform-ish f64 in the session size box [-500, 500].
    fn coord(&mut self) -> f64 {
        (self.next() as f64 / u64::MAX as f64) * 1000.0 - 500.0
    }
}

struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Fnv(0xcbf2_9ce4_8422_2325)
    }
    fn write_u64(&mut self, v: u64) {
        for byte in v.to_le_bytes() {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    fn write_f64(&mut self, v: f64) {
        self.write_u64(v.to_bits());
    }
}

#[test]
fn golden_hash_of_numeric_results() {
    let mut rng = Rng(0x51ED_270B_9A1F_3C4D);
    let mut hash = Fnv::new();

    for _ in 0..10_000 {
        let p2 = |rng: &mut Rng| [rng.coord(), rng.coord()];
        let p3 = |rng: &mut Rng| [rng.coord(), rng.coord(), rng.coord()];

        let (a2, b2, c2) = (p2(&mut rng), p2(&mut rng), p2(&mut rng));
        let d2 = [b2[0], c2[1]];
        hash.write_u64(orient2d(a2, b2, c2).as_i8() as u64);
        hash.write_u64(incircle(a2, b2, c2, d2).as_i8() as u64);
        hash.write_u64(polygon_orientation2d(&[a2, b2, c2, d2]).as_i8() as u64);
        // Degenerate-by-construction: c collinear with a and b.
        let mid = [(a2[0] + b2[0]) / 2.0, (a2[1] + b2[1]) / 2.0];
        hash.write_u64(orient2d(a2, b2, mid).as_i8() as u64);

        let (a3, b3, c3, d3) = (p3(&mut rng), p3(&mut rng), p3(&mut rng), p3(&mut rng));
        hash.write_u64(orient3d(a3, b3, c3, d3).as_i8() as u64);

        let affine = affine_dot3(a3, b3, c3, a2[0]).unwrap();
        hash.write_u64(affine.sign().as_i8() as u64);
        hash.write_f64(affine.approximation());
        hash.write_u64(affine.used_exact_fallback() as u64);

        let discriminant = quadratic_discriminant(a2[0], a2[1], b2[0]).unwrap();
        hash.write_u64(discriminant.sign().as_i8() as u64);
        hash.write_f64(discriminant.approximation());
        hash.write_u64(discriminant.used_exact_fallback() as u64);
        let harmonic = harmonic_half_angle_roots(a2[0], a2[1], b2[0]).unwrap();
        hash.write_u64(harmonic.discriminant().as_i8() as u64);
        hash.write_u64(harmonic.has_infinity_root() as u64);
        hash.write_u64(harmonic.is_identity() as u64);
        hash.write_u64(harmonic.used_exact_fallback() as u64);
        hash.write_u64(harmonic.finite_roots().len() as u64);
        for &root in harmonic.finite_roots() {
            hash.write_f64(root);
        }

        let x = Interval::new(a2[0].min(b2[0]), a2[0].max(b2[0]));
        let y = Interval::new(a2[1].min(b2[1]), a2[1].max(b2[1]));
        for iv in [x + y, x - y, x * y, x.square()] {
            hash.write_f64(iv.lo());
            hash.write_f64(iv.hi());
        }

        // Deterministic transcendental math, across magnitudes including
        // the Payne–Hanek huge-argument reduction path.
        let t = a2[0];
        for scale in [1.0, 1e4, 1e12, 1e100] {
            let (s, c) = kcore::math::sincos(t * scale);
            hash.write_f64(s);
            hash.write_f64(c);
        }
        hash.write_f64(kcore::math::atan2(a2[1], b2[1]));
        hash.write_f64(kcore::math::atan(a2[0] * 1e3));
    }

    let subtraction_fallback = affine_dot3(
        [1.0, 1.0, 0.0],
        [1.0, -1.0, 0.0],
        [-(f64::EPSILON / 2.0), 0.0, 0.0],
        0.0,
    )
    .unwrap();
    assert!(subtraction_fallback.used_exact_fallback());
    hash.write_u64(subtraction_fallback.sign().as_i8() as u64);
    hash.write_f64(subtraction_fallback.approximation());
    hash.write_u64(subtraction_fallback.used_exact_fallback() as u64);

    // Golden value. Changing it is a reviewed, intentional event.
    // (Re-pinned when exact affine-dot classification was added.)
    assert_eq!(hash.0, 0xC253_830A_E2CB_2D65, "numeric results drifted");
}
