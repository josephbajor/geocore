//! Closest-point projection onto curves and surfaces.
//!
//! Projection minimizes the squared distance `f(t) = |C(t) − p|²` (resp.
//! `f(u,v) = |S(u,v) − p|²`) over a caller-supplied **finite** search window:
//! callers own bounding (pass one period for periodic geometry, a clamped
//! window for unbounded geometry). The algorithm is deliberately simple and
//! deterministic for M1:
//!
//! 1. coarse sampling on a fixed grid (64 intervals for curves, 24×24 for
//!    surfaces — adaptive densification is deferred until profiling or
//!    robustness data demands it),
//! 2. every sampled local minimum (plateau-inclusive) becomes a candidate,
//!    ranked by value with index-order tie-breaking,
//! 3. each of the best candidates is polished by damped Newton iteration
//!    with backtracking (guarding indefinite Hessians with a
//!    gradient-descent fallback step), clamped to the window,
//! 4. the global best is returned, with parameters of periodic directions
//!    wrapped into the base range.
//!
//! Everything is deterministic: fixed sample counts, index-ordered candidate
//! selection, and total-order comparisons for the final choice.

use crate::curve::Curve;
use crate::param::{ParamRange, wrap_periodic};
use crate::surface::Surface;
use crate::vec::Point3;

/// Result of projecting a point onto a curve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveProjection {
    /// Parameter of the closest point (wrapped into the base range for
    /// periodic curves).
    pub t: f64,
    /// The closest point `C(t)`.
    pub point: Point3,
    /// Distance from the query point to `point`.
    pub dist: f64,
}

/// Result of projecting a point onto a surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceProjection {
    /// Parameters of the closest point (periodic directions wrapped into
    /// their base ranges).
    pub uv: [f64; 2],
    /// The closest point `S(u, v)`.
    pub point: Point3,
    /// Distance from the query point to `point`.
    pub dist: f64,
}

/// Coarse sample intervals along a curve window.
const CURVE_SAMPLES: usize = 64;
/// Coarse sample intervals per surface direction.
const SURFACE_SAMPLES: usize = 24;
/// Candidates polished per curve projection.
const CURVE_CANDIDATES: usize = 8;
/// Candidates polished per surface projection.
const SURFACE_CANDIDATES: usize = 6;
/// Newton iteration cap (curve).
const MAX_ITER_CURVE: usize = 50;
/// Newton iteration cap (surface).
const MAX_ITER_SURFACE: usize = 60;
/// Backtracking halvings per Newton step.
const MAX_HALVINGS: usize = 30;

/// Project `p` onto `curve`, searching within `window`.
///
/// `window` must be finite (callers bound unbounded curves; pass one period
/// for periodic curves). For periodic curves the returned `t` is wrapped
/// into the curve's base range, which may lie outside `window` if the window
/// was offset from the base range.
///
/// Returns `None` only if no sample can be evaluated (never happens for a
/// valid finite window; a zero-width window returns its single point).
///
/// # Panics
/// Panics if `window` is not finite.
pub fn project_to_curve(
    curve: &dyn Curve,
    p: Point3,
    window: ParamRange,
) -> Option<CurveProjection> {
    assert!(
        window.is_finite(),
        "projection window must be finite; clamp unbounded curves first"
    );
    let n = CURVE_SAMPLES;
    let mut fs = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = window.lerp(i as f64 / n as f64);
        fs.push((curve.eval(t) - p).norm_sq());
    }
    // Plateau-inclusive local minima of the coarse samples.
    let mut candidates: Vec<(f64, usize)> = Vec::new();
    for (i, &f) in fs.iter().enumerate() {
        let left_ok = i == 0 || f <= fs[i - 1];
        let right_ok = i == n || f <= fs[i + 1];
        if left_ok && right_ok {
            candidates.push((f, i));
        }
    }
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    candidates.truncate(CURVE_CANDIDATES);

    let mut best: Option<(f64, f64)> = None; // (dist_sq, t)
    for &(_, i) in &candidates {
        let t0 = window.lerp(i as f64 / n as f64);
        let (t, f) = polish_curve(curve, p, t0, window);
        let better = match best {
            None => true,
            Some((bf, bt)) => (f, t) < (bf, bt),
        };
        if better {
            best = Some((f, t));
        }
    }
    let (f, mut t) = best?;
    if let Some(period) = curve.periodicity() {
        t = wrap_periodic(t, curve.param_range().lo, period);
    }
    let point = curve.eval(t);
    Some(CurveProjection {
        t,
        point,
        dist: f.sqrt(),
    })
}

/// Damped Newton polish of a curve-projection candidate. Returns the
/// improved `(t, f(t))` with `f` the squared distance.
fn polish_curve(curve: &dyn Curve, p: Point3, t0: f64, window: ParamRange) -> (f64, f64) {
    let fval = |t: f64| (curve.eval(t) - p).norm_sq();
    let conv = 1e-12 * window.width().max(1.0);
    let max_step = window.width() / 4.0;
    let fallback_step = window.width() / CURVE_SAMPLES as f64;
    let mut t = t0;
    let mut f_curr = fval(t);
    'newton: for _ in 0..MAX_ITER_CURVE {
        let d = curve.eval_derivs(t, 2);
        let diff = d.d[0] - p;
        let g = 2.0 * d.d[1].dot(diff);
        // Stationarity at the floating-point noise floor of g.
        let g_scale = 2.0 * d.d[1].norm() * diff.norm();
        if g.abs() <= 1e-15 * (1.0 + g_scale) {
            break;
        }
        let h = 2.0 * (d.d[2].dot(diff) + d.d[1].norm_sq());
        let mut step = if h > 0.0 && h.is_finite() {
            -g / h
        } else {
            -g.signum() * fallback_step
        };
        step = step.clamp(-max_step, max_step);
        if step.abs() <= conv {
            break;
        }
        // Near the minimum f(t) plateaus at floating-point precision, so a
        // decrease test would stall at |Δt| ~ √ε. Small Newton steps are
        // locally convergent on the gradient — take them unconditionally.
        if h > 0.0 && step.abs() <= 1e-6 * window.width().max(1.0) {
            t = (t + step).clamp(window.lo, window.hi);
            f_curr = fval(t);
            continue;
        }
        let mut halvings = 0;
        loop {
            let t_new = (t + step).clamp(window.lo, window.hi);
            if t_new != t {
                let f_new = fval(t_new);
                if f_new <= f_curr {
                    t = t_new;
                    f_curr = f_new;
                    break;
                }
            }
            step *= 0.5;
            halvings += 1;
            if halvings >= MAX_HALVINGS || step.abs() <= conv {
                break 'newton;
            }
        }
    }
    (t, f_curr)
}

/// Project `p` onto `surface`, searching within `window` (both directions
/// finite; pass one period for periodic directions). Periodic parameters in
/// the result are wrapped into their base ranges.
///
/// # Panics
/// Panics if either window direction is not finite.
pub fn project_to_surface(
    surface: &dyn Surface,
    p: Point3,
    window: [ParamRange; 2],
) -> Option<SurfaceProjection> {
    assert!(
        window[0].is_finite() && window[1].is_finite(),
        "projection window must be finite; clamp unbounded surfaces first"
    );
    let n = SURFACE_SAMPLES;
    let sample = |i: usize, j: usize| {
        [
            window[0].lerp(i as f64 / n as f64),
            window[1].lerp(j as f64 / n as f64),
        ]
    };
    // Row-major (v-major) sample grid of squared distances.
    let mut fs = vec![0.0; (n + 1) * (n + 1)];
    for j in 0..=n {
        for i in 0..=n {
            fs[j * (n + 1) + i] = (surface.eval(sample(i, j)) - p).norm_sq();
        }
    }
    let at = |i: usize, j: usize| fs[j * (n + 1) + i];
    // Plateau-inclusive local minima against the 4-neighborhood.
    let mut candidates: Vec<(f64, usize, usize)> = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            let f = at(i, j);
            let ok = (i == 0 || f <= at(i - 1, j))
                && (i == n || f <= at(i + 1, j))
                && (j == 0 || f <= at(i, j - 1))
                && (j == n || f <= at(i, j + 1));
            if ok {
                candidates.push((f, i, j));
            }
        }
    }
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
    candidates.truncate(SURFACE_CANDIDATES);

    let mut best: Option<(f64, [f64; 2])> = None; // (dist_sq, uv)
    for &(_, i, j) in &candidates {
        let (uv, f) = polish_surface(surface, p, sample(i, j), window);
        let better = match best {
            None => true,
            Some((bf, buv)) => (f, uv[0], uv[1]) < (bf, buv[0], buv[1]),
        };
        if better {
            best = Some((f, uv));
        }
    }
    let (f, mut uv) = best?;
    let base = surface.param_range();
    for (k, period) in surface.periodicity().iter().enumerate() {
        if let Some(period) = period {
            uv[k] = wrap_periodic(uv[k], base[k].lo, *period);
        }
    }
    let point = surface.eval(uv);
    Some(SurfaceProjection {
        uv,
        point,
        dist: f.sqrt(),
    })
}

/// Damped Newton polish of a surface-projection candidate. Returns the
/// improved `(uv, f(uv))` with `f` the squared distance.
fn polish_surface(
    surface: &dyn Surface,
    p: Point3,
    uv0: [f64; 2],
    window: [ParamRange; 2],
) -> ([f64; 2], f64) {
    let fval = |uv: [f64; 2]| (surface.eval(uv) - p).norm_sq();
    let (wu, wv) = (window[0].width(), window[1].width());
    let conv_u = 1e-12 * wu.max(1.0);
    let conv_v = 1e-12 * wv.max(1.0);
    let (cell_u, cell_v) = (
        (wu / SURFACE_SAMPLES as f64).max(1e-12),
        (wv / SURFACE_SAMPLES as f64).max(1e-12),
    );
    let mut uv = uv0;
    let mut f_curr = fval(uv);
    'newton: for _ in 0..MAX_ITER_SURFACE {
        let d = surface.eval_derivs(uv, 2);
        let diff = d.p - p;
        let g0 = 2.0 * d.du.dot(diff);
        let g1 = 2.0 * d.dv.dot(diff);
        let g_scale = 2.0 * (d.du.norm() + d.dv.norm()) * diff.norm();
        if g0.abs().max(g1.abs()) <= 1e-15 * (1.0 + g_scale) {
            break;
        }
        let h00 = 2.0 * (d.duu.dot(diff) + d.du.norm_sq());
        let h01 = 2.0 * (d.duv.dot(diff) + d.du.dot(d.dv));
        let h11 = 2.0 * (d.dvv.dot(diff) + d.dv.norm_sq());
        let det = h00 * h11 - h01 * h01;
        // Newton step for a positive-definite Hessian; otherwise a
        // cell-scaled gradient-descent step.
        let (mut su, mut sv) = if h00 > 0.0 && det > 0.0 && det.is_finite() {
            (-(h11 * g0 - h01 * g1) / det, -(h00 * g1 - h01 * g0) / det)
        } else {
            let gn = (g0 * g0 + g1 * g1).sqrt();
            if gn == 0.0 {
                break;
            }
            (-g0 / gn * cell_u, -g1 / gn * cell_v)
        };
        su = su.clamp(-wu / 4.0, wu / 4.0);
        sv = sv.clamp(-wv / 4.0, wv / 4.0);
        if su.abs() <= conv_u && sv.abs() <= conv_v {
            break;
        }
        // See polish_curve: small PD-Newton steps bypass the f-decrease test
        // to converge past the f(uv) floating-point plateau.
        let newton_ok = h00 > 0.0 && det > 0.0 && det.is_finite();
        if newton_ok && su.abs() <= 1e-6 * wu.max(1.0) && sv.abs() <= 1e-6 * wv.max(1.0) {
            uv = [
                (uv[0] + su).clamp(window[0].lo, window[0].hi),
                (uv[1] + sv).clamp(window[1].lo, window[1].hi),
            ];
            f_curr = fval(uv);
            continue;
        }
        let mut halvings = 0;
        loop {
            let cand = [
                (uv[0] + su).clamp(window[0].lo, window[0].hi),
                (uv[1] + sv).clamp(window[1].lo, window[1].hi),
            ];
            if cand != uv {
                let f_new = fval(cand);
                if f_new <= f_curr {
                    uv = cand;
                    f_curr = f_new;
                    break;
                }
            }
            su *= 0.5;
            sv *= 0.5;
            halvings += 1;
            if halvings >= MAX_HALVINGS || (su.abs() <= conv_u && sv.abs() <= conv_v) {
                break 'newton;
            }
        }
    }
    (uv, f_curr)
}

#[cfg(test)]
#[allow(clippy::disallowed_methods)] // tests may cross-check against platform libm
mod tests {
    use super::*;
    use crate::curve::{Circle, Line};
    use crate::frame::Frame;
    use crate::surface::{Cylinder, Plane};
    use crate::vec::Vec3;
    use core::f64::consts::TAU;

    /// Deterministic xorshift64 PRNG (mirrors kcore's test RNGs; no deps).
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
        fn f(&mut self, lo: f64, hi: f64) -> f64 {
            lo + (self.next() as f64 / u64::MAX as f64) * (hi - lo)
        }
        fn point(&mut self, half: f64) -> Point3 {
            Point3::new(
                self.f(-half, half),
                self.f(-half, half),
                self.f(-half, half),
            )
        }
    }

    fn tilted_frame() -> Frame {
        Frame::new(
            Point3::new(0.5, 1.0, -2.0),
            Vec3::new(1.0, 2.0, 2.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap()
    }

    /// Angular distance modulo 2π.
    fn ang_diff(a: f64, b: f64) -> f64 {
        let d = (a - b).abs() % TAU;
        d.min(TAU - d)
    }

    #[test]
    fn line_projection_matches_closed_form() {
        let l = Line::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(-1.0, 0.5, 2.0)).unwrap();
        let mut rng = Rng(0x1357_9BDF_2468_ACE0);
        let window = ParamRange::new(-300.0, 300.0);
        for _ in 0..200 {
            let p = rng.point(100.0);
            let t_exp = (p - l.origin()).dot(l.dir());
            let pr = project_to_curve(&l, p, window).unwrap();
            let tol = 1e-12 * t_exp.abs().max(1.0);
            assert!(
                (pr.t - t_exp).abs() <= tol,
                "t = {}, expected {}",
                pr.t,
                t_exp
            );
            use crate::curve::Curve;
            let d_exp = p.dist(l.eval(t_exp));
            assert!((pr.dist - d_exp).abs() <= 1e-12 * d_exp.max(1.0));
        }
    }

    #[test]
    fn circle_projection_matches_closed_form() {
        let c = Circle::new(tilted_frame(), 2.5).unwrap();
        let mut rng = Rng(0xFEDC_BA98_7654_3211);
        let window = ParamRange::new(0.0, TAU);
        let mut tested = 0;
        while tested < 200 {
            let p = rng.point(20.0);
            let local = c.frame().to_local(p);
            let radial = (local.x * local.x + local.y * local.y).sqrt();
            if radial < 0.5 {
                continue; // near-axis: projection ambiguous, tested separately
            }
            tested += 1;
            let mut t_exp = local.y.atan2(local.x);
            if t_exp < 0.0 {
                t_exp += TAU;
            }
            use crate::curve::Curve;
            let q_exp = c.eval(t_exp);
            let pr = project_to_curve(&c, p, window).unwrap();
            assert!(ang_diff(pr.t, t_exp) <= 1e-9, "t = {}, exp {}", pr.t, t_exp);
            assert!(
                (0.0..TAU + 1e-15).contains(&pr.t),
                "t not wrapped: {}",
                pr.t
            );
            assert!(pr.point.dist(q_exp) <= 1e-9);
            let d_exp = p.dist(q_exp);
            assert!((pr.dist - d_exp).abs() <= 1e-12 * d_exp.max(1.0));
        }
    }

    #[test]
    fn circle_projection_near_seam_wraps_correctly() {
        let c = Circle::new(tilted_frame(), 2.5).unwrap();
        let window = ParamRange::new(0.0, TAU);
        for angle in [TAU - 1e-3, 1e-3, TAU - 0.4, 0.4] {
            // Radially outward point at this angle, with an axial offset.
            let (s, co) = angle.sin_cos();
            let p = c.frame().point_at(1.7 * 2.5 * co, 1.7 * 2.5 * s, 0.3);
            let pr = project_to_curve(&c, p, window).unwrap();
            assert!(
                ang_diff(pr.t, angle) <= 1e-9,
                "seam wrap: got {}, expected {}",
                pr.t,
                angle
            );
            assert!((0.0..TAU).contains(&pr.t) || (pr.t - TAU).abs() < 1e-12);
        }
    }

    #[test]
    fn circle_center_projection_is_ambiguous_but_valid() {
        let c = Circle::new(tilted_frame(), 2.5).unwrap();
        let center = c.frame().origin();
        let pr = project_to_curve(&c, center, ParamRange::new(0.0, TAU)).unwrap();
        assert!((pr.dist - 2.5).abs() <= 1e-12);
        assert!((pr.point.dist(center) - 2.5).abs() <= 1e-12);
    }

    #[test]
    fn circle_projection_extreme_distances_converge() {
        let c = Circle::new(tilted_frame(), 2.5).unwrap();
        let window = ParamRange::new(0.0, TAU);
        // Nearly on the curve (radial offset 1e-9).
        let (s, co) = 1.0f64.sin_cos();
        let near = c.frame().point_at((2.5 + 1e-9) * co, (2.5 + 1e-9) * s, 0.0);
        let pr = project_to_curve(&c, near, window).unwrap();
        assert!(pr.dist <= 2e-9, "near-point dist = {}", pr.dist);
        assert!(ang_diff(pr.t, 1.0) <= 1e-6);
        // Far away (within the size box).
        let (s, co) = 2.0f64.sin_cos();
        let far = c.frame().point_at(450.0 * co, 450.0 * s, 0.0);
        let pr = project_to_curve(&c, far, window).unwrap();
        assert!(ang_diff(pr.t, 2.0) <= 1e-9);
        assert!((pr.dist - 447.5).abs() <= 1e-9 * 447.5);
    }

    #[test]
    fn plane_projection_matches_closed_form() {
        let pl = Plane::new(tilted_frame());
        let mut rng = Rng(0x0F1E_2D3C_4B5A_6978);
        let window = [
            ParamRange::new(-500.0, 500.0),
            ParamRange::new(-500.0, 500.0),
        ];
        for _ in 0..200 {
            let p = rng.point(100.0);
            let local = pl.frame().to_local(p);
            let pr = project_to_surface(&pl, p, window).unwrap();
            assert!((pr.uv[0] - local.x).abs() <= 1e-9);
            assert!((pr.uv[1] - local.y).abs() <= 1e-9);
            assert!((pr.dist - local.z.abs()).abs() <= 1e-9);
            let q_exp = pl.frame().point_at(local.x, local.y, 0.0);
            assert!(pr.point.dist(q_exp) <= 1e-9);
        }
    }

    #[test]
    fn cylinder_projection_matches_closed_form() {
        let cyl = Cylinder::new(tilted_frame(), 1.75).unwrap();
        let mut rng = Rng(0xC0FF_EE00_DEAD_BEEF);
        let window = [ParamRange::new(0.0, TAU), ParamRange::new(-50.0, 50.0)];
        let mut tested = 0;
        while tested < 200 {
            let p = rng.point(30.0);
            let local = cyl.frame().to_local(p);
            let radial = (local.x * local.x + local.y * local.y).sqrt();
            if radial < 0.3 {
                continue; // near-axis ambiguity
            }
            tested += 1;
            let mut u_exp = local.y.atan2(local.x);
            if u_exp < 0.0 {
                u_exp += TAU;
            }
            let pr = project_to_surface(&cyl, p, window).unwrap();
            assert!(
                ang_diff(pr.uv[0], u_exp) <= 1e-9,
                "u = {}, expected {}",
                pr.uv[0],
                u_exp
            );
            assert!((pr.uv[1] - local.z).abs() <= 1e-9);
            let d_exp = (radial - 1.75).abs();
            assert!(
                (pr.dist - d_exp).abs() <= 1e-9 * d_exp.max(1.0),
                "dist = {}, expected {}",
                pr.dist,
                d_exp
            );
        }
    }

    #[test]
    fn cylinder_projection_far_point_converges() {
        let cyl = Cylinder::new(tilted_frame(), 1.75).unwrap();
        let window = [ParamRange::new(0.0, TAU), ParamRange::new(-50.0, 50.0)];
        let (s, co) = 0.7f64.sin_cos();
        let p = cyl.frame().point_at(400.0 * co, 400.0 * s, 12.0);
        let pr = project_to_surface(&cyl, p, window).unwrap();
        assert!(ang_diff(pr.uv[0], 0.7) <= 1e-9);
        assert!((pr.uv[1] - 12.0).abs() <= 1e-9);
        assert!((pr.dist - (400.0 - 1.75)).abs() <= 1e-6);
    }

    #[test]
    fn zero_width_window_returns_the_single_point() {
        let l = Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let pr =
            project_to_curve(&l, Point3::new(5.0, 1.0, 0.0), ParamRange::new(2.0, 2.0)).unwrap();
        assert_eq!(pr.t, 2.0);
        assert!(
            (pr.dist - Point3::new(5.0, 1.0, 0.0).dist(Point3::new(2.0, 0.0, 0.0))).abs() < 1e-15
        );
    }
}
