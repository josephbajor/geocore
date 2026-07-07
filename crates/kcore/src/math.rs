//! Deterministic transcendental math.
//!
//! **Why this module exists:** the kernel's determinism contract requires
//! bit-identical results on every platform and build mode, but platform libm
//! `sin`/`cos`/`atan2` are *not* correctly rounded and differ (by an ulp or
//! more) across libms, OS versions, and even compile-time constant folding
//! vs. runtime evaluation — a divergence our golden-hash harness caught in
//! practice. All kernel code must therefore call these functions instead of
//! the `f64` inherent methods (enforced by clippy `disallowed-methods`).
//! `sqrt`, `abs`, `floor`, and arithmetic are exempt: IEEE 754 fully
//! specifies them.
//!
//! The implementations are a faithful Rust port of musl libc's math routines
//! (themselves derived from FreeBSD msun / Sun's fdlibm):
//!
//! ```text
//! Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
//! Developed at SunSoft, a Sun Microsystems, Inc. business.
//! Permission to use, copy, modify, and distribute this
//! software is freely granted, provided that this notice
//! is preserved.
//! ```
//!
//! musl as a whole is MIT-licensed. The port uses only `f64` arithmetic and
//! bit manipulation — no `long double`, no platform intrinsics — so results
//! are identical wherever IEEE 754 binary64 holds. Accuracy is < 1 ulp for
//! sin/cos/atan/atan2 over the full domain, including exact Payne–Hanek
//! argument reduction for astronomically large angles.
//!
//! Deliberate deviations from musl: floating-point exception side effects
//! (`FORCE_EVAL` raising inexact/underflow flags) are dropped — the kernel
//! never inspects the FP status word — and only binary64 (`prec = 1`) paths
//! are ported.

/// NaN for inf/NaN inputs, propagating an input NaN payload (musl's
/// `x - x` idiom, isolated here to carry the lint allowance once).
#[allow(clippy::eq_op)]
#[inline]
fn invalid(x: f64) -> f64 {
    x - x
}

/// Bit pattern of the high 32 bits of an `f64`.
#[inline]
fn hi(x: f64) -> u32 {
    (x.to_bits() >> 32) as u32
}

/// Exact power of two as `f64` (`n` must keep the result normal).
#[inline]
fn exp2i(n: i32) -> f64 {
    debug_assert!((-1022..=1023).contains(&n));
    f64::from_bits(((1023 + n) as u64) << 52)
}

/// `x * 2^n` (exact; `n` bounded as in [`exp2i`]).
#[inline]
fn scalbn(x: f64, n: i32) -> f64 {
    x * exp2i(n)
}

// ---------------------------------------------------------------------------
// Kernel sin/cos on [-pi/4, pi/4] (musl __sin.c / __cos.c)
// ---------------------------------------------------------------------------

const S1: f64 = f64::from_bits(0xBFC5_5555_5555_5549); // -1.66666666666666324348e-01
const S2: f64 = f64::from_bits(0x3F81_1111_1110_F8A6); //  8.33333333332248946124e-03
const S3: f64 = f64::from_bits(0xBF2A_01A0_19C1_61D5); // -1.98412698298579493134e-04
const S4: f64 = f64::from_bits(0x3EC7_1DE3_57B1_FE7D); //  2.75573137070700676789e-06
const S5: f64 = f64::from_bits(0xBE5A_E5E6_8A2B_9CEB); // -2.50507602534068634195e-08
const S6: f64 = f64::from_bits(0x3DE5_D93A_5ACF_D57C); //  1.58969099521155010221e-10

/// Kernel sine on ~[-pi/4, pi/4]; `y` is the tail of `x`, `has_tail` says
/// whether `y` is meaningful.
fn k_sin(x: f64, y: f64, has_tail: bool) -> f64 {
    let z = x * x;
    let w = z * z;
    let r = S2 + z * (S3 + z * S4) + z * w * (S5 + z * S6);
    let v = z * x;
    if !has_tail {
        x + v * (S1 + z * r)
    } else {
        x - ((z * (0.5 * y - v * r) - y) - v * S1)
    }
}

const C1: f64 = f64::from_bits(0x3FA5_5555_5555_554C); //  4.16666666666666019037e-02
const C2: f64 = f64::from_bits(0xBF56_C16C_16C1_5177); // -1.38888888888741095749e-03
const C3: f64 = f64::from_bits(0x3EFA_01A0_19CB_1590); //  2.48015872894767294178e-05
const C4: f64 = f64::from_bits(0xBE92_7E4F_809C_52AD); // -2.75573143513906633035e-07
const C5: f64 = f64::from_bits(0x3E21_EE9E_BDB4_B1C4); //  2.08757232129817482790e-09
const C6: f64 = f64::from_bits(0xBDA8_FAE9_BE88_38D4); // -1.13596475577881948265e-11

/// Kernel cosine on ~[-pi/4, pi/4]; `y` is the tail of `x`.
fn k_cos(x: f64, y: f64) -> f64 {
    let z = x * x;
    let w = z * z;
    let r = z * (C1 + z * (C2 + z * C3)) + w * w * (C4 + z * (C5 + z * C6));
    let hz = 0.5 * z;
    let w = 1.0 - hz;
    w + (((1.0 - w) - hz) + (z * r - x * y))
}

// ---------------------------------------------------------------------------
// Argument reduction (musl __rem_pio2.c / __rem_pio2_large.c)
// ---------------------------------------------------------------------------

const TOINT: f64 = 1.5 / f64::EPSILON;
const PIO4: f64 = f64::from_bits(0x3FE9_21FB_5444_2D18);
const INVPIO2: f64 = f64::from_bits(0x3FE4_5F30_6DC9_C883); // 53 bits of 2/pi
const PIO2_1: f64 = f64::from_bits(0x3FF9_21FB_5440_0000); // first 33 bits of pi/2
const PIO2_1T: f64 = f64::from_bits(0x3DD0_B461_1A62_6331); // pi/2 - PIO2_1
const PIO2_2: f64 = f64::from_bits(0x3DD0_B461_1A60_0000); // second 33 bits
const PIO2_2T: f64 = f64::from_bits(0x3BA3_198A_2E03_7073);
const PIO2_3: f64 = f64::from_bits(0x3BA3_198A_2E00_0000); // third 33 bits
const PIO2_3T: f64 = f64::from_bits(0x397B_839A_2520_49C1);

/// 24-bit chunks of 2/pi after the binary point: entry `i` holds bits
/// `24i .. 24i+23`. 66 entries suffice for binary64 arguments
/// (`(e0-3)/24 + jk` with `e0 <= 1000`, `jk = 4`).
const IPIO2: [i32; 66] = [
    0xA2F983, 0x6E4E44, 0x1529FC, 0x2757D1, 0xF534DD, 0xC0DB62, 0x95993C, 0x439041, 0xFE5163,
    0xABDEBB, 0xC561B7, 0x246E3A, 0x424DD2, 0xE00649, 0x2EEA09, 0xD1921C, 0xFE1DEB, 0x1CB129,
    0xA73EE8, 0x8235F5, 0x2EBB44, 0x84E99C, 0x7026B4, 0x5F7E41, 0x3991D6, 0x398353, 0x39F49C,
    0x845F8B, 0xBDF928, 0x3B1FF8, 0x97FFDE, 0x05980F, 0xEF2F11, 0x8B5A0A, 0x6D1F6D, 0x367ECF,
    0x27CB09, 0xB74F46, 0x3F669E, 0x5FEA2D, 0x7527BA, 0xC7EBE5, 0xF17B3D, 0x0739F7, 0x8A5292,
    0xEA6BFB, 0x5FB11F, 0x8D5D08, 0x560330, 0x46FC7B, 0x6BABF0, 0xCFBC20, 0x9AF436, 0x1DA9E3,
    0x91615E, 0xE61B08, 0x659985, 0x5F14A0, 0x68408D, 0xFFD880, 0x4D7327, 0x310606, 0x1556CA,
    0x73A8C9, 0x60E27B, 0xC08C6B,
];

/// pi/2 cut into 24-bit chunks.
const PIO2S: [f64; 8] = [
    f64::from_bits(0x3FF9_21FB_4000_0000),
    f64::from_bits(0x3E74_442D_0000_0000),
    f64::from_bits(0x3CF8_4698_8000_0000),
    f64::from_bits(0x3B78_CC51_6000_0000),
    f64::from_bits(0x39F0_1B83_8000_0000),
    f64::from_bits(0x387A_2520_4000_0000),
    f64::from_bits(0x36E3_8222_8000_0000),
    f64::from_bits(0x3569_F31D_0000_0000),
];

/// Payne–Hanek reduction for huge arguments (musl `__rem_pio2_large`,
/// binary64 precision only: `prec = 1`, `jk = 4`).
///
/// `x` holds the positive input split into 24-bit chunks scaled by `2^e0`;
/// `nx` is the chunk count. Returns `(n, y0, y1)` with the reduced angle
/// `y0 + y1` and octant count `n` (mod 8).
fn rem_pio2_large(x: &[f64; 3], e0: i32, nx: usize) -> (i32, f64, f64) {
    const JK: usize = 4; // init_jk[1], binary64
    let jp = JK;
    let mut iq = [0_i32; 20];
    let mut f = [0.0_f64; 20];
    let mut fq = [0.0_f64; 20];
    let mut q = [0.0_f64; 20];

    let jx = nx - 1;
    let jv = ((e0 - 3) / 24).max(0);
    let mut q0 = e0 - 24 * (jv + 1);

    // f[0..=jx+jk] = ipio2[jv-jx ..], zero-padded on the left.
    let mut j = jv as i64 - jx as i64;
    for fi in f.iter_mut().take(jx + JK + 1) {
        *fi = if j < 0 {
            0.0
        } else {
            f64::from(IPIO2[j as usize])
        };
        j += 1;
    }
    for (i, qi) in q.iter_mut().enumerate().take(JK + 1) {
        let mut fw = 0.0;
        for j in 0..=jx {
            fw += x[j] * f[jx + i - j];
        }
        *qi = fw;
    }

    let mut jz = JK;
    let mut n: i32;
    let mut ih: i32;
    let mut z: f64;
    loop {
        // Distill q[] into 24-bit integer chunks iq[], reversed.
        z = q[jz];
        let mut jj = jz;
        for iqi in iq.iter_mut().take(jz) {
            let fw = f64::from((exp2i(-24) * z) as i32);
            *iqi = (z - exp2i(24) * fw) as i32;
            z = q[jj - 1] + fw;
            jj -= 1;
        }

        // Compute n.
        z = scalbn(z, q0);
        z -= 8.0 * (z * 0.125).floor(); // trim off integer >= 8
        n = z as i32;
        z -= f64::from(n);
        ih = 0;
        if q0 > 0 {
            // Need iq[jz-1] to determine n.
            let i = iq[jz - 1] >> (24 - q0);
            n += i;
            iq[jz - 1] -= i << (24 - q0);
            ih = iq[jz - 1] >> (23 - q0);
        } else if q0 == 0 {
            ih = iq[jz - 1] >> 23;
        } else if z >= 0.5 {
            ih = 2;
        }

        if ih > 0 {
            // q > 0.5: compute 1 - q.
            n += 1;
            let mut carry = 0;
            for iqi in iq.iter_mut().take(jz) {
                let j = *iqi;
                if carry == 0 {
                    if j != 0 {
                        carry = 1;
                        *iqi = 0x100_0000 - j;
                    }
                } else {
                    *iqi = 0xFF_FFFF - j;
                }
            }
            if q0 == 1 {
                iq[jz - 1] &= 0x7F_FFFF;
            } else if q0 == 2 {
                iq[jz - 1] &= 0x3F_FFFF;
            }
            if ih == 2 {
                z = 1.0 - z;
                if carry != 0 {
                    z -= scalbn(1.0, q0);
                }
            }
        }

        // Check if recomputation is needed (cancellation ate our bits).
        if z == 0.0 {
            let mut j = 0;
            for &iqi in iq.iter().take(jz).skip(JK) {
                j |= iqi;
            }
            if j == 0 {
                let mut k = 1;
                while iq[JK - k] == 0 {
                    k += 1;
                }
                for i in (jz + 1)..=(jz + k) {
                    f[jx + i] = f64::from(IPIO2[(jv as usize) + i]);
                    let mut fw = 0.0;
                    for j in 0..=jx {
                        fw += x[j] * f[jx + i - j];
                    }
                    q[i] = fw;
                }
                jz += k;
                continue;
            }
        }
        break;
    }

    // Chop off zero terms / break z into 24-bit chunks.
    if z == 0.0 {
        jz -= 1;
        q0 -= 24;
        while iq[jz] == 0 {
            jz -= 1;
            q0 -= 24;
        }
    } else {
        z = scalbn(z, -q0);
        if z >= exp2i(24) {
            let fw = f64::from((exp2i(-24) * z) as i32);
            iq[jz] = (z - exp2i(24) * fw) as i32;
            jz += 1;
            q0 += 24;
            iq[jz] = fw as i32;
        } else {
            iq[jz] = z as i32;
        }
    }

    // Convert integer chunks to floating point.
    let mut fw = scalbn(1.0, q0);
    for i in (0..=jz).rev() {
        q[i] = fw * f64::from(iq[i]);
        fw *= exp2i(-24);
    }

    // fq[] = PIo2[] * q[], accumulated most-significant first.
    for i in (0..=jz).rev() {
        let mut fw = 0.0;
        let mut k = 0;
        while k <= jp && k <= jz - i {
            fw += PIO2S[k] * q[i + k];
            k += 1;
        }
        fq[jz - i] = fw;
    }

    // Compress fq[] into (y0, y1) — binary64 (prec = 1) branch.
    let mut fw = 0.0;
    for &fqi in fq.iter().take(jz + 1).rev() {
        fw += fqi;
    }
    let y0 = if ih == 0 { fw } else { -fw };
    let mut fw2 = fq[0] - fw;
    for &fqi in fq.iter().take(jz + 1).skip(1) {
        fw2 += fqi;
    }
    let y1 = if ih == 0 { fw2 } else { -fw2 };
    (n & 7, y0, y1)
}

/// Reduce `x` to `y0 + y1` with `|y0 + y1| <= pi/4`, returning the quadrant
/// count `n` such that `x = n·(pi/2) + y0 + y1` (musl `__rem_pio2`). The
/// caller handles `|x| <= pi/4` itself.
fn rem_pio2(x: f64) -> (i32, f64, f64) {
    let bits = x.to_bits();
    let sign = bits >> 63 != 0;
    let ix = (bits >> 32) as u32 & 0x7fff_ffff;

    // Fast paths: |x| within a few multiples of pi/2 — subtract a
    // double-double multiple directly.
    fn near_multiple(x: f64, m: f64, negate: bool) -> (f64, f64) {
        if !negate {
            let z = x - m * PIO2_1;
            let y0 = z - m * PIO2_1T;
            let y1 = (z - y0) - m * PIO2_1T;
            (y0, y1)
        } else {
            let z = x + m * PIO2_1;
            let y0 = z + m * PIO2_1T;
            let y1 = (z - y0) + m * PIO2_1T;
            (y0, y1)
        }
    }

    if ix <= 0x400f_6a7a {
        // |x| ~<= 5pi/4
        if (ix & 0xf_ffff) == 0x9_21fb {
            // |x| ~= pi/2 or pi: cancellation — use the medium path.
        } else if ix <= 0x4002_d97c {
            // |x| ~<= 3pi/4
            let (y0, y1) = near_multiple(x, 1.0, sign);
            return (if sign { -1 } else { 1 }, y0, y1);
        } else {
            let (y0, y1) = near_multiple(x, 2.0, sign);
            return (if sign { -2 } else { 2 }, y0, y1);
        }
    } else if ix <= 0x401c_463b {
        // |x| ~<= 9pi/4
        if ix <= 0x4015_fdbc {
            // |x| ~<= 7pi/4
            if ix == 0x4012_d97c {
                // |x| ~= 3pi/2 — medium path.
            } else {
                let (y0, y1) = near_multiple(x, 3.0, sign);
                return (if sign { -3 } else { 3 }, y0, y1);
            }
        } else if ix == 0x4019_21fb {
            // |x| ~= 2pi — medium path.
        } else {
            let (y0, y1) = near_multiple(x, 4.0, sign);
            return (if sign { -4 } else { 4 }, y0, y1);
        }
    }

    if ix < 0x4139_21fb {
        // |x| ~< 2^20·(pi/2): medium size. n = rint(x/(pi/2)) via the
        // round-to-nearest trick, then up to three double-double rounds.
        let mut fn_ = x * INVPIO2 + TOINT - TOINT;
        let mut n = fn_ as i32;
        let mut r = x - fn_ * PIO2_1;
        let mut w = fn_ * PIO2_1T; // 1st round, good to 85 bits
        // Matters with directed rounding (kept for fidelity).
        if r - w < -PIO4 {
            n -= 1;
            fn_ -= 1.0;
            r = x - fn_ * PIO2_1;
            w = fn_ * PIO2_1T;
        } else if r - w > PIO4 {
            n += 1;
            fn_ += 1.0;
            r = x - fn_ * PIO2_1;
            w = fn_ * PIO2_1T;
        }
        let mut y0 = r - w;
        let ey = (y0.to_bits() >> 52) as i32 & 0x7ff;
        let ex = (ix >> 20) as i32;
        if ex - ey > 16 {
            // 2nd round, good to 118 bits.
            let t = r;
            w = fn_ * PIO2_2;
            r = t - w;
            w = fn_ * PIO2_2T - ((t - r) - w);
            y0 = r - w;
            let ey = (y0.to_bits() >> 52) as i32 & 0x7ff;
            if ex - ey > 49 {
                // 3rd round, good to 151 bits: covers all cases.
                let t = r;
                w = fn_ * PIO2_3;
                r = t - w;
                w = fn_ * PIO2_3T - ((t - r) - w);
                y0 = r - w;
            }
        }
        let y1 = (r - y0) - w;
        return (n, y0, y1);
    }

    // Inf or NaN.
    if ix >= 0x7ff0_0000 {
        let y = invalid(x);
        return (0, y, y);
    }

    // Huge arguments: split |x| into 24-bit chunks and run Payne–Hanek.
    let mut u = bits & (u64::MAX >> 12);
    u |= u64::from(0x3ffu32 + 23) << 52;
    let mut z = f64::from_bits(u);
    let mut tx = [0.0_f64; 3];
    for txi in tx.iter_mut().take(2) {
        *txi = f64::from(z as i32);
        z = (z - *txi) * exp2i(24);
    }
    tx[2] = z;
    let mut nx = 3;
    while tx[nx - 1] == 0.0 {
        nx -= 1; // skip zero terms (first term is nonzero)
    }
    let e0 = (ix >> 20) as i32 - (0x3ff + 23);
    let (n, y0, y1) = rem_pio2_large(&tx, e0, nx);
    if sign { (-n, -y0, -y1) } else { (n, y0, y1) }
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

/// Deterministic `sin(x)` (musl `sin.c`). Accuracy < 1 ulp; bit-identical on
/// every IEEE 754 platform.
pub fn sin(x: f64) -> f64 {
    let ix = hi(x) & 0x7fff_ffff;
    if ix <= 0x3fe9_21fb {
        // |x| ~<= pi/4
        if ix < 0x3e50_0000 {
            // |x| < 2^-26: sin(x) rounds to x.
            return x;
        }
        return k_sin(x, 0.0, false);
    }
    if ix >= 0x7ff0_0000 {
        return invalid(x); // NaN for inf and NaN
    }
    let (n, y0, y1) = rem_pio2(x);
    match n & 3 {
        0 => k_sin(y0, y1, true),
        1 => k_cos(y0, y1),
        2 => -k_sin(y0, y1, true),
        _ => -k_cos(y0, y1),
    }
}

/// Deterministic `cos(x)` (musl `cos.c`). Accuracy < 1 ulp; bit-identical on
/// every IEEE 754 platform.
pub fn cos(x: f64) -> f64 {
    let ix = hi(x) & 0x7fff_ffff;
    if ix <= 0x3fe9_21fb {
        // |x| ~<= pi/4
        if ix < 0x3e46_a09e {
            // |x| < 2^-27·sqrt(2): cos(x) rounds to 1.
            return 1.0;
        }
        return k_cos(x, 0.0);
    }
    if ix >= 0x7ff0_0000 {
        return invalid(x);
    }
    let (n, y0, y1) = rem_pio2(x);
    match n & 3 {
        0 => k_cos(y0, y1),
        1 => -k_sin(y0, y1, true),
        2 => -k_cos(y0, y1),
        _ => k_sin(y0, y1, true),
    }
}

/// Deterministic `(sin(x), cos(x))` with a single argument reduction.
/// Bit-identical to `(sin(x), cos(x))`.
pub fn sincos(x: f64) -> (f64, f64) {
    let ix = hi(x) & 0x7fff_ffff;
    if ix <= 0x3fe9_21fb {
        let s = if ix < 0x3e50_0000 {
            x
        } else {
            k_sin(x, 0.0, false)
        };
        let c = if ix < 0x3e46_a09e { 1.0 } else { k_cos(x, 0.0) };
        return (s, c);
    }
    if ix >= 0x7ff0_0000 {
        let n = invalid(x);
        return (n, n);
    }
    let (n, y0, y1) = rem_pio2(x);
    let s = k_sin(y0, y1, true);
    let c = k_cos(y0, y1);
    match n & 3 {
        0 => (s, c),
        1 => (c, -s),
        2 => (-s, -c),
        _ => (-c, s),
    }
}

const ATANHI: [f64; 4] = [
    f64::from_bits(0x3FDD_AC67_0561_BB4F), // atan(0.5) hi
    f64::from_bits(0x3FE9_21FB_5444_2D18), // atan(1.0) hi
    f64::from_bits(0x3FEF_730B_D281_F69B), // atan(1.5) hi
    f64::from_bits(0x3FF9_21FB_5444_2D18), // atan(inf) hi
];
const ATANLO: [f64; 4] = [
    f64::from_bits(0x3C7A_2B7F_222F_65E2),
    f64::from_bits(0x3C81_A626_3314_5C07),
    f64::from_bits(0x3C70_0788_7AF0_CBBD),
    f64::from_bits(0x3C91_A626_3314_5C07),
];
const AT: [f64; 11] = [
    f64::from_bits(0x3FD5_5555_5555_550D),
    f64::from_bits(0xBFC9_9999_9998_EBC4),
    f64::from_bits(0x3FC2_4924_9200_83FF),
    f64::from_bits(0xBFBC_71C6_FE23_1671),
    f64::from_bits(0x3FB7_45CD_C54C_206E),
    f64::from_bits(0xBFB3_B0F2_AF74_9A6D),
    f64::from_bits(0x3FB1_0D66_A0D0_3D51),
    f64::from_bits(0xBFAD_DE2D_52DE_FD9A),
    f64::from_bits(0x3FA9_7B4B_2476_0DEB),
    f64::from_bits(0xBFA2_B444_2C6A_6C2F),
    f64::from_bits(0x3F90_AD3A_E322_DA11),
];

/// Deterministic `atan(x)` (musl `atan.c`). Accuracy < 1 ulp.
pub fn atan(x: f64) -> f64 {
    let ix_full = hi(x);
    let sign = ix_full >> 31 != 0;
    let ix = ix_full & 0x7fff_ffff;
    if ix >= 0x4410_0000 {
        // |x| >= 2^66
        if x.is_nan() {
            return x;
        }
        let z = ATANHI[3] + f64::from_bits(1); // nudge for inexactness parity
        return if sign { -z } else { z };
    }
    let (x, id): (f64, i32) = if ix < 0x3fdc_0000 {
        // |x| < 0.4375
        if ix < 0x3e40_0000 {
            // |x| < 2^-27: atan(x) rounds to x.
            return x;
        }
        (x, -1)
    } else {
        let ax = x.abs();
        if ix < 0x3ff3_0000 {
            if ix < 0x3fe6_0000 {
                (
                    (2.0 * ax - 1.0) / (2.0 + ax), // 7/16 <= |x| < 11/16
                    0,
                )
            } else {
                ((ax - 1.0) / (ax + 1.0), 1) // 11/16 <= |x| < 19/16
            }
        } else if ix < 0x4003_8000 {
            ((ax - 1.5) / (1.0 + 1.5 * ax), 2) // 19/16 <= |x| < 39/16
        } else {
            (-1.0 / ax, 3) // 39/16 <= |x| < 2^66
        }
    };
    // Polynomial in odd/even halves.
    let z = x * x;
    let w = z * z;
    let s1 = z * (AT[0] + w * (AT[2] + w * (AT[4] + w * (AT[6] + w * (AT[8] + w * AT[10])))));
    let s2 = w * (AT[1] + w * (AT[3] + w * (AT[5] + w * (AT[7] + w * AT[9]))));
    if id < 0 {
        return x - x * (s1 + s2);
    }
    let id = id as usize;
    let z = ATANHI[id] - (x * (s1 + s2) - ATANLO[id] - x);
    if sign { -z } else { z }
}

const PI: f64 = f64::from_bits(0x4009_21FB_5444_2D18);
const PI_LO: f64 = f64::from_bits(0x3CA1_A626_3314_5C07);

/// Deterministic `atan2(y, x)` (musl `atan2.c`), with the full IEEE special-
/// case table (signed zeros, infinities, NaN propagation).
pub fn atan2(y: f64, x: f64) -> f64 {
    if x.is_nan() || y.is_nan() {
        return x + y;
    }
    let xb = x.to_bits();
    let yb = y.to_bits();
    if xb == 0x3FF0_0000_0000_0000 {
        return atan(y); // x exactly 1.0
    }
    let m = ((yb >> 63) as u32 & 1) | ((xb >> 62) as u32 & 2); // 2*sign(x) + sign(y)
    let ix = (xb >> 32) as u32 & 0x7fff_ffff;
    let lx = xb as u32;
    let iy = (yb >> 32) as u32 & 0x7fff_ffff;
    let ly = yb as u32;

    if iy | ly == 0 {
        // y = 0
        match m {
            0 | 1 => return y, // atan(+-0, +anything) = +-0
            2 => return PI,
            _ => return -PI,
        }
    }
    if ix | lx == 0 {
        // x = 0
        return if m & 1 != 0 { -PI / 2.0 } else { PI / 2.0 };
    }
    if ix == 0x7ff0_0000 {
        // x infinite
        if iy == 0x7ff0_0000 {
            return match m {
                0 => PI / 4.0,
                1 => -PI / 4.0,
                2 => 3.0 * PI / 4.0,
                _ => -3.0 * PI / 4.0,
            };
        }
        return match m {
            0 => 0.0,
            1 => -0.0,
            2 => PI,
            _ => -PI,
        };
    }
    // |y/x| > 2^64: result is +-pi/2.
    if ix.wrapping_add(64 << 20) < iy || iy == 0x7ff0_0000 {
        return if m & 1 != 0 { -PI / 2.0 } else { PI / 2.0 };
    }

    // z = atan(|y/x|) without spurious underflow.
    let z = if (m & 2 != 0) && iy.wrapping_add(64 << 20) < ix {
        0.0 // |y/x| < 2^-64 with x < 0
    } else {
        atan((y / x).abs())
    };
    match m {
        0 => z,
        1 => -z,
        2 => PI - (z - PI_LO),
        _ => (z - PI_LO) - PI,
    }
}

#[cfg(test)]
#[allow(clippy::disallowed_methods)] // tests compare against platform libm
mod tests {
    use super::*;

    /// ulp distance between two finite same-order values.
    fn within_ulps(a: f64, b: f64, ulps: u64) -> bool {
        if a == b {
            return true;
        }
        if a.is_nan() || b.is_nan() || a.signum() != b.signum() {
            return false;
        }
        let (ba, bb) = (a.abs().to_bits(), b.abs().to_bits());
        ba.abs_diff(bb) <= ulps
    }

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
        fn f64_in(&mut self, lo: f64, hi: f64) -> f64 {
            lo + (self.next() as f64 / u64::MAX as f64) * (hi - lo)
        }
    }

    #[test]
    fn trig_agrees_with_platform_libm_across_scales() {
        let mut rng = Rng(0x1234_5678_9ABC_DEF1);
        // Sweep magnitudes from subnormal-adjacent to astronomically large:
        // both implementations do exact reduction, so results stay within
        // 2 ulp of each other everywhere.
        for e in -40..300 {
            for _ in 0..40 {
                let x = rng.f64_in(-1.0, 1.0) * 2f64.powi(e);
                assert!(
                    within_ulps(sin(x), x.sin(), 2),
                    "sin({x:e}): ours {:e} libm {:e}",
                    sin(x),
                    x.sin()
                );
                assert!(
                    within_ulps(cos(x), x.cos(), 2),
                    "cos({x:e}): ours {:e} libm {:e}",
                    cos(x),
                    x.cos()
                );
                assert!(
                    within_ulps(atan(x), x.atan(), 2),
                    "atan({x:e}): ours {:e} libm {:e}",
                    atan(x),
                    x.atan()
                );
            }
        }
    }

    #[test]
    fn atan2_agrees_with_platform_libm() {
        let mut rng = Rng(0xFEDC_BA98_7654_3211);
        for _ in 0..20_000 {
            let y = rng.f64_in(-1.0, 1.0) * 2f64.powi((rng.next() % 80) as i32 - 40);
            let x = rng.f64_in(-1.0, 1.0) * 2f64.powi((rng.next() % 80) as i32 - 40);
            assert!(
                within_ulps(atan2(y, x), y.atan2(x), 2),
                "atan2({y:e}, {x:e}): ours {:e} libm {:e}",
                atan2(y, x),
                y.atan2(x)
            );
        }
    }

    #[test]
    fn sincos_is_bitwise_consistent_with_sin_and_cos() {
        let mut rng = Rng(0x0F0F_F0F0_1234_4321);
        for _ in 0..20_000 {
            let x = rng.f64_in(-1.0, 1.0) * 2f64.powi((rng.next() % 60) as i32 - 20);
            let (s, c) = sincos(x);
            assert_eq!(s.to_bits(), sin(x).to_bits());
            assert_eq!(c.to_bits(), cos(x).to_bits());
        }
    }

    #[test]
    fn symmetries_hold_bitwise() {
        let mut rng = Rng(0xAAAA_BBBB_CCCC_DDD1);
        for _ in 0..10_000 {
            let x = rng.f64_in(0.0, 1.0) * 2f64.powi((rng.next() % 40) as i32 - 10);
            assert_eq!(sin(-x).to_bits(), (-sin(x)).to_bits());
            assert_eq!(cos(-x).to_bits(), cos(x).to_bits());
            assert_eq!(atan(-x).to_bits(), (-atan(x)).to_bits());
        }
    }

    #[test]
    fn pythagorean_identity() {
        let mut rng = Rng(0x1111_2222_3333_4441);
        for _ in 0..10_000 {
            let x = rng.f64_in(-700.0, 700.0);
            let (s, c) = sincos(x);
            assert!((s * s + c * c - 1.0).abs() < 4.0 * f64::EPSILON);
        }
    }

    #[test]
    fn special_values() {
        assert_eq!(sin(0.0).to_bits(), 0.0f64.to_bits());
        assert_eq!(sin(-0.0).to_bits(), (-0.0f64).to_bits());
        assert_eq!(cos(0.0), 1.0);
        assert!(sin(f64::INFINITY).is_nan());
        assert!(cos(f64::NEG_INFINITY).is_nan());
        assert!(sin(f64::NAN).is_nan());
        assert_eq!(atan2(0.0, 1.0).to_bits(), 0.0f64.to_bits());
        assert_eq!(atan2(-0.0, 1.0).to_bits(), (-0.0f64).to_bits());
        assert_eq!(atan2(0.0, -1.0), PI);
        assert_eq!(atan2(-0.0, -1.0), -PI);
        assert_eq!(atan2(1.0, 0.0), PI / 2.0);
        assert_eq!(atan2(-1.0, 0.0), -PI / 2.0);
        assert_eq!(atan2(f64::INFINITY, f64::INFINITY), PI / 4.0);
        assert_eq!(atan2(f64::INFINITY, f64::NEG_INFINITY), 3.0 * PI / 4.0);
        assert!(atan2(f64::NAN, 1.0).is_nan());
    }

    #[test]
    fn huge_argument_reduction_is_exact() {
        // Values whose reduction requires the Payne–Hanek path; the platform
        // libm on this class of host also reduces exactly, so 2 ulp holds.
        for &x in &[1e10, 1e50, 1e100, 1e200, 1e300, 2f64.powi(1000)] {
            assert!(within_ulps(sin(x), x.sin(), 2), "sin({x:e})");
            assert!(within_ulps(cos(x), x.cos(), 2), "cos({x:e})");
            assert!(within_ulps(sin(-x), (-x).sin(), 2), "sin(-{x:e})");
        }
    }
}
