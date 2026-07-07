//! Evaluator conformance harness.
//!
//! Every geometry class must pass [`check_curve`] / [`check_surface`] in its
//! tests (M1 exit criterion). The checks are:
//!
//! - analytic derivatives match central finite differences of the position
//!   (orders 1 and 2; order 3 for curves), with tolerances scaled to the
//!   geometry's magnitude;
//! - periodic classes actually repeat: `P(t) == P(t + T)` to near machine
//!   precision;
//! - declared degeneracies are real (the iso-line collapses to a point) and
//!   the normal is reported absent there;
//! - away from degeneracies the normal exists, is unit, and is orthogonal
//!   to both first partials.
//!
//! This module is a public part of the crate so downstream geometry classes
//! (procedural surfaces in later milestones) can reuse it in their tests;
//! it is not intended for production call paths.

use crate::curve::Curve;
use crate::param::ParamRange;
use crate::surface::{Dir, Surface};
use crate::vec::Vec3;

/// Finite-difference step. With central differences (error `O(h²)`) this
/// puts derivative agreement near 1e-7 relative — the check tolerances
/// below leave headroom above that floor.
const FD_STEP: f64 = 1e-5;
/// Finite-difference step for the *third*-derivative stencil only. The
/// third-order stencil divides by `2h³`, so evaluation roundoff is amplified
/// by `ε/h³`: at `h = 1e-5` that is ~0.1 relative — larger than any sensible
/// tolerance — for evaluators whose rounding is uncorrelated across the
/// stencil (de Boor recursion, rational division). At `h = 2e-4` noise falls
/// to ~1e-5 relative while truncation `O(h²)` stays ~1e-7; the stencil
/// half-width (4e-4) remains far below typical knot spacing, so piecewise
/// evaluators are not straddled at their continuity breaks.
const FD_STEP_3: f64 = 2e-4;
/// Relative tolerance for first/second-derivative agreement.
const FD_TOL: f64 = 5e-5;
/// Relative tolerance for third-derivative agreement (one more division
/// by `h` amplifies noise).
const FD_TOL_3: f64 = 5e-3;

/// Number of sample parameters per direction.
const SAMPLES: usize = 17;

fn finite_window(range: ParamRange) -> ParamRange {
    range.clamped(ParamRange::new(-10.0, 10.0))
}

fn assert_close(actual: Vec3, expected: Vec3, scale: f64, tol: f64, what: &str) {
    let err = (actual - expected).norm();
    let bound = tol * scale.max(1.0);
    assert!(
        err <= bound,
        "{what}: |analytic - FD| = {err:.3e} exceeds {bound:.3e}\n  analytic {actual:?}\n  FD       {expected:?}"
    );
}

/// Assert the full curve evaluator contract. Panics with a diagnostic on the
/// first violation.
pub fn check_curve(curve: &dyn Curve) {
    let range = finite_window(curve.param_range());
    let h = FD_STEP * range.width().max(1.0);
    // Keep FD stencils inside the range for aperiodic curves.
    let (s_lo, s_hi) = if curve.periodicity().is_some() {
        (0.0, 1.0)
    } else {
        (0.02, 0.98)
    };

    for i in 0..SAMPLES {
        let s = s_lo + (s_hi - s_lo) * i as f64 / (SAMPLES - 1) as f64;
        let t = range.lerp(s);
        let d = curve.eval_derivs(t, 3);
        assert_eq!(d.d[0], curve.eval(t), "eval must agree with eval_derivs");

        let scale = d.d[0].norm().max(d.d[1].norm());
        let p = |t: f64| curve.eval(t);
        let fd1 = (p(t + h) - p(t - h)) / (2.0 * h);
        assert_close(d.d[1], fd1, scale, FD_TOL, "curve 1st derivative");
        let fd2 = (p(t + h) - d.d[0] * 2.0 + p(t - h)) / (h * h);
        assert_close(d.d[2], fd2, scale, FD_TOL, "curve 2nd derivative");
        let h3 = FD_STEP_3 * range.width().max(1.0);
        let fd3 = (p(t + 2.0 * h3) - p(t + h3) * 2.0 + p(t - h3) * 2.0 - p(t - 2.0 * h3))
            / (2.0 * h3 * h3 * h3);
        assert_close(d.d[3], fd3, scale, FD_TOL_3, "curve 3rd derivative");

        // Lower-order calls must zero the higher entries.
        let d1 = curve.eval_derivs(t, 1);
        assert_eq!(d1.d[2], Vec3::default());
        assert_eq!(d1.d[3], Vec3::default());
    }

    if let Some(period) = curve.periodicity() {
        for i in 0..SAMPLES {
            let t = range.lo + range.width() * i as f64 / SAMPLES as f64;
            let a = curve.eval(t);
            let b = curve.eval(t + period);
            assert!(
                a.dist(b) <= 1e-9 * a.norm().max(1.0),
                "periodicity violated at t = {t}: {a:?} vs {b:?}"
            );
        }
    }
}

/// Assert the full surface evaluator contract. Panics with a diagnostic on
/// the first violation.
pub fn check_surface(surface: &dyn Surface) {
    let [ur, vr] = surface.param_range();
    let (ur, vr) = (finite_window(ur), finite_window(vr));
    let [pu, pv] = surface.periodicity();
    let hu = FD_STEP * ur.width().max(1.0);
    let hv = FD_STEP * vr.width().max(1.0);
    // Sample interior for aperiodic/degenerate directions.
    let margin = |periodic: bool| if periodic { (0.0, 1.0) } else { (0.02, 0.98) };
    let (ul, uh) = margin(pu.is_some());
    let (vl, vh) = margin(pv.is_some());

    let degeneracies = surface.degeneracies();
    let near_degenerate = |u: f64, v: f64| {
        degeneracies.iter().any(|dg| {
            let (val, width) = match dg.dir {
                Dir::U => (u, ur.width()),
                Dir::V => (v, vr.width()),
            };
            (val - dg.at).abs() < 0.05 * width
        })
    };

    for i in 0..SAMPLES {
        for j in 0..SAMPLES {
            let u = ur.lerp(ul + (uh - ul) * i as f64 / (SAMPLES - 1) as f64);
            let v = vr.lerp(vl + (vh - vl) * j as f64 / (SAMPLES - 1) as f64);
            if near_degenerate(u, v) {
                continue;
            }
            let d = surface.eval_derivs([u, v], 2);
            assert_eq!(d.p, surface.eval([u, v]));

            let scale = d.p.norm().max(d.du.norm()).max(d.dv.norm());
            let p = |u: f64, v: f64| surface.eval([u, v]);
            let fdu = (p(u + hu, v) - p(u - hu, v)) / (2.0 * hu);
            assert_close(d.du, fdu, scale, FD_TOL, "surface du");
            let fdv = (p(u, v + hv) - p(u, v - hv)) / (2.0 * hv);
            assert_close(d.dv, fdv, scale, FD_TOL, "surface dv");
            let fduu = (p(u + hu, v) - d.p * 2.0 + p(u - hu, v)) / (hu * hu);
            assert_close(d.duu, fduu, scale, FD_TOL, "surface duu");
            let fdvv = (p(u, v + hv) - d.p * 2.0 + p(u, v - hv)) / (hv * hv);
            assert_close(d.dvv, fdvv, scale, FD_TOL, "surface dvv");
            let fduv = (p(u + hu, v + hv) - p(u + hu, v - hv) - p(u - hu, v + hv)
                + p(u - hu, v - hv))
                / (4.0 * hu * hv);
            assert_close(d.duv, fduv, scale, FD_TOL, "surface duv");

            let n = surface
                .normal([u, v])
                .expect("normal must exist away from degeneracies");
            assert!((n.norm() - 1.0).abs() < 1e-12, "normal must be unit");
            assert!(
                n.dot(d.du).abs() <= 1e-9 * scale.max(1.0),
                "normal not orthogonal to du"
            );
            assert!(
                n.dot(d.dv).abs() <= 1e-9 * scale.max(1.0),
                "normal not orthogonal to dv"
            );
        }
    }

    // Periodicity.
    for (dir, period) in [(Dir::U, pu), (Dir::V, pv)] {
        let Some(period) = period else { continue };
        for i in 0..SAMPLES {
            let s = i as f64 / SAMPLES as f64;
            let (u, v) = (ur.lerp(s), vr.lerp(1.0 - s));
            let (u2, v2) = match dir {
                Dir::U => (u + period, v),
                Dir::V => (u, v + period),
            };
            let a = surface.eval([u, v]);
            let b = surface.eval([u2, v2]);
            assert!(
                a.dist(b) <= 1e-9 * a.norm().max(1.0),
                "surface periodicity violated at ({u}, {v})"
            );
        }
    }

    // Declared degeneracies must be real: the iso-line collapses to a point
    // and the normal reports absent.
    for dg in &degeneracies {
        let mut first: Option<Vec3> = None;
        for i in 0..SAMPLES {
            let s = i as f64 / (SAMPLES - 1) as f64;
            let uv = match dg.dir {
                Dir::U => [dg.at, vr.lerp(s)],
                Dir::V => [ur.lerp(s), dg.at],
            };
            let p = surface.eval(uv);
            let anchor = *first.get_or_insert(p);
            assert!(
                p.dist(anchor) <= 1e-9 * anchor.norm().max(1.0),
                "declared degeneracy at {dg:?} is not a point"
            );
            assert!(
                surface.normal(uv).is_none(),
                "normal must be None at degeneracy {dg:?}"
            );
        }
    }
}
