//! Exact floating-point expansion arithmetic.
//!
//! An *expansion* is a sequence of `f64` components, ordered by increasing
//! magnitude, mutually nonoverlapping, whose exact (infinite-precision) sum
//! is the value represented. Sums, differences, and products of `f64` values
//! can be computed *exactly* in this representation. This is the machinery
//! behind the exact fallback paths of the robust predicates.
//!
//! The algorithms are from J. R. Shewchuk, *Adaptively Robust Floating-Point
//! Arithmetic and Fast Robust Geometric Predicates* (1997). The exact paths
//! here favor clarity over the paper's hand-staged adaptivity; see
//! `predicates.rs` for the performance note.
//!
//! Invariants assumed by every function taking expansion slices:
//! components ascend in magnitude, are nonoverlapping, and the slice is
//! non-empty (a zero expansion is the singleton `[0.0]`). All functions
//! producing expansions maintain these invariants and eliminate interior
//! zero components.

/// Exact sum of `a` and `b` under the precondition `|a| >= |b|` (or `a == 0`).
/// Returns `(x, y)` with `x = fl(a + b)` and `x + y == a + b` exactly.
#[inline]
pub fn fast_two_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    let b_virtual = x - a;
    (x, b - b_virtual)
}

/// Exact sum of two `f64`s. Returns `(x, y)` with `x = fl(a + b)` and
/// `x + y == a + b` exactly. No magnitude precondition.
#[inline]
pub fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    let b_virtual = x - a;
    let a_virtual = x - b_virtual;
    let b_round = b - b_virtual;
    let a_round = a - a_virtual;
    (x, a_round + b_round)
}

/// Exact difference of two `f64`s. Returns `(x, y)` with `x = fl(a - b)` and
/// `x + y == a - b` exactly.
#[inline]
pub fn two_diff(a: f64, b: f64) -> (f64, f64) {
    let x = a - b;
    let b_virtual = a - x;
    let a_virtual = x + b_virtual;
    let b_round = b_virtual - b;
    let a_round = a - a_virtual;
    (x, a_round + b_round)
}

/// 2^27 + 1, Dekker's splitting constant for 53-bit significands.
const SPLITTER: f64 = 134_217_729.0;

/// Split `a` into `(hi, lo)` with `hi + lo == a`, each half representable in
/// 26 bits of significand, so products of halves are exact.
#[inline]
fn split(a: f64) -> (f64, f64) {
    let c = SPLITTER * a;
    let a_big = c - a;
    let hi = c - a_big;
    (hi, a - hi)
}

/// Exact product of two `f64`s. Returns `(x, y)` with `x = fl(a * b)` and
/// `x + y == a * b` exactly.
#[inline]
pub fn two_product(a: f64, b: f64) -> (f64, f64) {
    let (b_hi, b_lo) = split(b);
    two_product_presplit(a, b, b_hi, b_lo)
}

/// [`two_product`] with `b` already split; used when multiplying many values
/// by the same `b`.
#[inline]
fn two_product_presplit(a: f64, b: f64, b_hi: f64, b_lo: f64) -> (f64, f64) {
    let x = a * b;
    let (a_hi, a_lo) = split(a);
    let err1 = x - a_hi * b_hi;
    let err2 = err1 - a_lo * b_hi;
    let err3 = err2 - a_hi * b_lo;
    (x, a_lo * b_lo - err3)
}

/// Build a canonical expansion from the `(x, y)` result of a `two_*` routine.
#[inline]
pub fn from_two(x: f64, y: f64) -> Vec<f64> {
    if y == 0.0 { vec![x] } else { vec![y, x] }
}

/// Exact sum of two expansions (`fast_expansion_sum_zeroelim`).
pub fn sum(e: &[f64], f: &[f64]) -> Vec<f64> {
    debug_assert!(!e.is_empty() && !f.is_empty());
    // Merge components by ascending magnitude (ties take from `e` first).
    let mut g = Vec::with_capacity(e.len() + f.len());
    let (mut i, mut j) = (0, 0);
    while i < e.len() && j < f.len() {
        if e[i].abs() <= f[j].abs() {
            g.push(e[i]);
            i += 1;
        } else {
            g.push(f[j]);
            j += 1;
        }
    }
    g.extend_from_slice(&e[i..]);
    g.extend_from_slice(&f[j..]);

    if g.len() == 1 {
        return g;
    }
    let mut h = Vec::with_capacity(g.len());
    // First combination may use fast_two_sum because g[1] dominates g[0];
    // afterwards q can outgrow later components, so plain two_sum is required.
    let (mut q, y) = fast_two_sum(g[1], g[0]);
    if y != 0.0 {
        h.push(y);
    }
    for &gi in &g[2..] {
        let (q_new, y) = two_sum(q, gi);
        q = q_new;
        if y != 0.0 {
            h.push(y);
        }
    }
    if q != 0.0 || h.is_empty() {
        h.push(q);
    }
    h
}

/// Exact product of an expansion and a single `f64`
/// (`scale_expansion_zeroelim`).
pub fn scale(e: &[f64], b: f64) -> Vec<f64> {
    debug_assert!(!e.is_empty());
    let (b_hi, b_lo) = split(b);
    let mut h = Vec::with_capacity(2 * e.len());
    let (mut q, y) = two_product_presplit(e[0], b, b_hi, b_lo);
    if y != 0.0 {
        h.push(y);
    }
    for &ei in &e[1..] {
        let (p1, p0) = two_product_presplit(ei, b, b_hi, b_lo);
        let (s, y1) = two_sum(q, p0);
        if y1 != 0.0 {
            h.push(y1);
        }
        let (q_new, y2) = fast_two_sum(p1, s);
        q = q_new;
        if y2 != 0.0 {
            h.push(y2);
        }
    }
    if q != 0.0 || h.is_empty() {
        h.push(q);
    }
    h
}

/// Exact product of two expansions (distributes [`scale`] over `f` and sums).
/// Quadratic in component count; used only on exact fallback paths where the
/// operands are small.
pub fn mul(e: &[f64], f: &[f64]) -> Vec<f64> {
    debug_assert!(!e.is_empty() && !f.is_empty());
    let mut acc = scale(e, f[0]);
    for &fi in &f[1..] {
        acc = sum(&acc, &scale(e, fi));
    }
    acc
}

/// Exact negation of an expansion.
pub fn negate(e: &[f64]) -> Vec<f64> {
    e.iter().map(|c| -c).collect()
}

/// One-`f64` approximation of an expansion's value (sums components in
/// ascending-magnitude order).
pub fn approx(e: &[f64]) -> f64 {
    e.iter().sum()
}

/// Exact sign of the value represented by an expansion. Because components
/// are nonoverlapping and ascend in magnitude, the largest component alone
/// determines the sign.
pub fn sign(e: &[f64]) -> i8 {
    let last = *e.last().expect("expansion must be non-empty");
    if last > 0.0 {
        1
    } else if last < 0.0 {
        -1
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_sum_exact_on_representable_cases() {
        // 1 + 2^-60 is not representable in f64; the residue must capture it.
        let (x, y) = two_sum(1.0, 2f64.powi(-60));
        assert_eq!(x, 1.0);
        assert_eq!(y, 2f64.powi(-60));
    }

    #[test]
    fn two_product_captures_rounding_error() {
        let a = 1.0 + 2f64.powi(-30);
        let (x, y) = two_product(a, a);
        // a^2 = 1 + 2^-29 + 2^-60; the 2^-60 term is exactly the residue.
        assert_eq!(x, 1.0 + 2f64.powi(-29));
        assert_eq!(y, 2f64.powi(-60));
    }

    #[test]
    fn sum_and_scale_agree_with_integer_arithmetic() {
        // Integer-valued doubles keep everything exact and checkable in i128.
        let e = from_two(two_sum(1_048_576.0, 3.0).0, two_sum(1_048_576.0, 3.0).1);
        let f = vec![7.0, 65_536.0];
        let s = sum(&e, &f);
        assert_eq!(approx(&s), 1_048_576.0 + 3.0 + 7.0 + 65_536.0);
        let sc = scale(&e, -12.0);
        assert_eq!(approx(&sc), (1_048_576.0 + 3.0) * -12.0);
    }

    #[test]
    fn mul_is_exact_where_f64_is_not() {
        // (2^30 + 1)^2 = 2^60 + 2^31 + 1 needs 61 bits: inexact in one f64,
        // exact as an expansion.
        let a = vec![2f64.powi(30) + 1.0];
        let p = mul(&a, &a);
        let exact: i128 = (1_i128 << 60) + (1 << 31) + 1;
        let total: i128 = p.iter().map(|&c| c as i128).sum();
        assert_eq!(total, exact);
    }

    #[test]
    fn sign_of_tiny_residue_dominated_value() {
        // 1 + tiny - 1 == tiny, invisible to a plain float sum at 1.0's scale.
        let tiny = vec![2f64.powi(-70)];
        let one = vec![1.0];
        let v = sum(&sum(&one, &tiny), &negate(&one));
        assert_eq!(sign(&v), 1);
        assert_eq!(approx(&v), 2f64.powi(-70));
    }

    #[test]
    fn zero_expansion_is_canonical() {
        let z = sum(&[1.0], &[-1.0]);
        assert_eq!(z, vec![0.0]);
        assert_eq!(sign(&z), 0);
    }
}
