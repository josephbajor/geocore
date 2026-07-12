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
use kcore::error::{CapabilityId, ClassifiedError, ErrorClass, ErrorCode};
use kcore::operation::{
    AccountingMode, DiagnosticKind, OperationContext, OperationOutcome, OperationPolicyError,
    OperationScope, ResourceKind,
};

mod policy;

pub use policy::{
    CURVE_PROJECTION_CANDIDATES, CURVE_PROJECTION_HALVINGS, CURVE_PROJECTION_NEWTON_ITERATIONS,
    CURVE_PROJECTION_QUERIES, CURVE_PROJECTION_SAMPLES, PROJECTION_LIMIT_REACHED,
    ProjectionBudgetProfile, SURFACE_PROJECTION_CANDIDATES, SURFACE_PROJECTION_HALVINGS,
    SURFACE_PROJECTION_NEWTON_ITERATIONS, SURFACE_PROJECTION_QUERIES, SURFACE_PROJECTION_SAMPLES,
};

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

/// Why a contextual closest-point projection could not produce a result.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum ProjectionError {
    /// The query point contains a non-finite component.
    InvalidQueryPoint,
    /// A curve window, or the named surface-window direction, is invalid.
    InvalidWindow {
        /// Zero for curves and the u direction; one for the surface v direction.
        direction: usize,
    },
    /// No sampled local-minimum candidate was available.
    NoCandidate,
    /// A geometry evaluator produced a non-finite point, derivative, or objective.
    NonFiniteEvaluation,
    /// The active operation scope did not satisfy or exceeded its accounting contract.
    Policy(OperationPolicyError),
}

const fn known_error_code(value: &'static str) -> ErrorCode {
    match ErrorCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in projection error code"),
    }
}

/// Stable machine-readable identities returned by [`ProjectionError`].
pub mod error_code {
    use super::{ErrorCode, known_error_code};

    /// The supplied query point was non-finite.
    pub const INVALID_QUERY_POINT: ErrorCode =
        known_error_code("kgeom.project.invalid-query-point");
    /// The supplied projection window was non-finite or reversed.
    pub const INVALID_WINDOW: ErrorCode = known_error_code("kgeom.project.invalid-window");
    /// Sampling did not retain a projection candidate.
    pub const NO_CANDIDATE: ErrorCode = known_error_code("kgeom.project.no-candidate");
    /// A geometry evaluator produced a non-finite numerical result.
    pub const NON_FINITE_EVALUATION: ErrorCode =
        known_error_code("kgeom.project.non-finite-evaluation");
}

impl ProjectionError {
    /// Returns the broad semantic class of this failure.
    pub fn class(&self) -> ErrorClass {
        match self {
            Self::InvalidQueryPoint | Self::InvalidWindow { .. } => ErrorClass::InvalidInput,
            Self::NoCandidate | Self::NonFiniteEvaluation => ErrorClass::InternalInvariant,
            Self::Policy(error) => error.class(),
        }
    }

    /// Returns the stable machine-readable identity of this failure.
    pub fn code(&self) -> ErrorCode {
        match self {
            Self::InvalidQueryPoint => error_code::INVALID_QUERY_POINT,
            Self::InvalidWindow { .. } => error_code::INVALID_WINDOW,
            Self::NoCandidate => error_code::NO_CANDIDATE,
            Self::NonFiniteEvaluation => error_code::NON_FINITE_EVALUATION,
            Self::Policy(error) => error.code(),
        }
    }

    /// Returns structured deterministic-limit data when accounting stopped the query.
    pub fn limit(&self) -> Option<kcore::operation::LimitSnapshot> {
        match self {
            Self::Policy(error) => error.limit(),
            _ => None,
        }
    }
}

impl core::fmt::Display for ProjectionError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidQueryPoint => formatter.write_str("projection query point is not finite"),
            Self::InvalidWindow { direction } => {
                write!(
                    formatter,
                    "projection window direction {direction} is invalid"
                )
            }
            Self::NoCandidate => formatter.write_str("projection produced no candidate"),
            Self::NonFiniteEvaluation => {
                formatter.write_str("projection geometry evaluation is not finite")
            }
            Self::Policy(error) => write!(formatter, "projection policy failed: {error}"),
        }
    }
}

impl std::error::Error for ProjectionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Policy(error) => Some(error),
            _ => None,
        }
    }
}

impl ClassifiedError for ProjectionError {
    fn class(&self) -> ErrorClass {
        self.class()
    }

    fn code(&self) -> ErrorCode {
        self.code()
    }

    fn capability(&self) -> Option<CapabilityId> {
        None
    }

    fn limit(&self) -> Option<kcore::operation::LimitSnapshot> {
        self.limit()
    }
}

impl From<OperationPolicyError> for ProjectionError {
    fn from(error: OperationPolicyError) -> Self {
        Self::Policy(error)
    }
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

/// Project onto a curve with deterministic resource accounting.
///
/// Projection family defaults have the lowest precedence, followed by
/// matching session entries and then request overrides. Budget configuration
/// and input validity are checked before the first geometry evaluation.
pub fn project_to_curve_with_context(
    curve: &dyn Curve,
    p: Point3,
    window: ParamRange,
    context: &OperationContext<'_>,
) -> core::result::Result<OperationOutcome<CurveProjection, ProjectionError>, OperationPolicyError>
{
    let context = context
        .clone()
        .with_family_budget_defaults(ProjectionBudgetProfile::curve_defaults());
    validate_curve_budget(|stage, resource, mode| {
        context
            .effective_budget()
            .require_limit(stage, resource, mode)
    })?;
    let mut scope = OperationScope::new(&context);
    let result = project_to_curve_in_scope(curve, p, window, &mut scope);
    Ok(scope.finish_typed(result))
}

/// Project onto a curve using the caller's existing operation scope.
///
/// High-water sample, candidate, Newton, and backtracking usage is shared
/// without becoming cumulative across queries. The query stage itself is
/// cumulative, so callers planning multiple projections must reserve that
/// count explicitly.
pub fn project_to_curve_in_scope(
    curve: &dyn Curve,
    p: Point3,
    window: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<CurveProjection, ProjectionError> {
    validate_curve_budget(|stage, resource, mode| {
        scope.ledger().require_limit(stage, resource, mode)
    })?;
    validate_point(p)?;
    validate_window(window, 0)?;
    charge_projection(scope, CURVE_PROJECTION_QUERIES, 1)?;

    let n = CURVE_SAMPLES;
    let mut fs = Vec::with_capacity(n + 1);
    for i in 0..=n {
        observe_projection(
            scope,
            CURVE_PROJECTION_SAMPLES,
            ResourceKind::Items,
            (i + 1) as u64,
        )?;
        let t = window.lerp(i as f64 / n as f64);
        fs.push(curve_objective(curve, p, t)?);
    }
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

    let mut best: Option<(f64, f64)> = None;
    for (candidate, &(_, i)) in candidates.iter().enumerate() {
        observe_projection(
            scope,
            CURVE_PROJECTION_CANDIDATES,
            ResourceKind::Items,
            (candidate + 1) as u64,
        )?;
        let t0 = window.lerp(i as f64 / n as f64);
        let (t, f) = polish_curve_in_scope(curve, p, t0, window, scope)?;
        let better = match best {
            None => true,
            Some((bf, bt)) => (f, t) < (bf, bt),
        };
        if better {
            best = Some((f, t));
        }
    }
    let (f, mut t) = best.ok_or(ProjectionError::NoCandidate)?;
    if let Some(period) = curve.periodicity() {
        if !period.is_finite() || period <= 0.0 || !curve.param_range().lo.is_finite() {
            return Err(ProjectionError::NonFiniteEvaluation);
        }
        t = wrap_periodic(t, curve.param_range().lo, period);
    }
    let point = curve.eval(t);
    if !finite_point(point) || !t.is_finite() || !f.is_finite() {
        return Err(ProjectionError::NonFiniteEvaluation);
    }
    Ok(CurveProjection {
        t,
        point,
        dist: f.sqrt(),
    })
}

fn polish_curve_in_scope(
    curve: &dyn Curve,
    p: Point3,
    t0: f64,
    window: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<(f64, f64), ProjectionError> {
    let conv = 1e-12 * window.width().max(1.0);
    let max_step = window.width() / 4.0;
    let fallback_step = window.width() / CURVE_SAMPLES as f64;
    let mut t = t0;
    let mut f_curr = curve_objective(curve, p, t)?;
    for iteration in 0..MAX_ITER_CURVE {
        observe_projection(
            scope,
            CURVE_PROJECTION_NEWTON_ITERATIONS,
            ResourceKind::Depth,
            (iteration + 1) as u64,
        )?;
        let d = curve.eval_derivs(t, 2);
        if !d.d[..=2].iter().copied().all(finite_point) {
            return Err(ProjectionError::NonFiniteEvaluation);
        }
        let diff = d.d[0] - p;
        let g = 2.0 * d.d[1].dot(diff);
        let g_scale = 2.0 * d.d[1].norm() * diff.norm();
        if !g.is_finite() || !g_scale.is_finite() {
            return Err(ProjectionError::NonFiniteEvaluation);
        }
        if g.abs() <= 1e-15 * (1.0 + g_scale) {
            return Ok((t, f_curr));
        }
        let h = 2.0 * (d.d[2].dot(diff) + d.d[1].norm_sq());
        if !h.is_finite() {
            return Err(ProjectionError::NonFiniteEvaluation);
        }
        let mut step = if h > 0.0 && h.is_finite() {
            -g / h
        } else {
            -g.signum() * fallback_step
        };
        step = step.clamp(-max_step, max_step);
        if step.abs() <= conv {
            return Ok((t, f_curr));
        }
        if h > 0.0 && step.abs() <= 1e-6 * window.width().max(1.0) {
            t = (t + step).clamp(window.lo, window.hi);
            f_curr = curve_objective(curve, p, t)?;
            continue;
        }
        let mut halvings = 0;
        loop {
            let t_new = (t + step).clamp(window.lo, window.hi);
            if t_new != t {
                let f_new = curve_objective(curve, p, t_new)?;
                if f_new <= f_curr {
                    t = t_new;
                    f_curr = f_new;
                    break;
                }
            }
            step *= 0.5;
            halvings += 1;
            observe_projection(
                scope,
                CURVE_PROJECTION_HALVINGS,
                ResourceKind::Depth,
                halvings,
            )?;
            if halvings >= MAX_HALVINGS as u64 || step.abs() <= conv {
                if halvings >= MAX_HALVINGS as u64 {
                    observe_projection(
                        scope,
                        CURVE_PROJECTION_HALVINGS,
                        ResourceKind::Depth,
                        MAX_HALVINGS as u64 + 1,
                    )?;
                }
                return Ok((t, f_curr));
            }
        }
    }
    observe_projection(
        scope,
        CURVE_PROJECTION_NEWTON_ITERATIONS,
        ResourceKind::Depth,
        MAX_ITER_CURVE as u64 + 1,
    )?;
    unreachable!("the v1 Newton ceiling must reject its canonical crossing")
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

/// Project onto a surface with deterministic resource accounting.
///
/// Budget configuration and both window directions are validated before the
/// first surface evaluation. Invalid public `ParamRange` values are returned
/// as typed errors rather than reaching `lerp` or `clamp` panics.
pub fn project_to_surface_with_context(
    surface: &dyn Surface,
    p: Point3,
    window: [ParamRange; 2],
    context: &OperationContext<'_>,
) -> core::result::Result<OperationOutcome<SurfaceProjection, ProjectionError>, OperationPolicyError>
{
    let context = compose_surface_projection_context(context)?;
    let mut scope = OperationScope::new(&context);
    let result = project_to_surface_in_scope(surface, p, window, &mut scope);
    Ok(scope.finish_typed(result))
}

/// Composes and validates the surface-projection family profile for a
/// top-level contextual operation that may delegate to projection.
pub(crate) fn compose_surface_projection_context<'session>(
    context: &OperationContext<'session>,
) -> core::result::Result<OperationContext<'session>, OperationPolicyError> {
    let context = context
        .clone()
        .with_family_budget_defaults(ProjectionBudgetProfile::surface_defaults());
    validate_surface_budget(|stage, resource, mode| {
        context
            .effective_budget()
            .require_limit(stage, resource, mode)
    })?;
    Ok(context)
}

/// Project onto a surface using the caller's existing operation scope.
pub fn project_to_surface_in_scope(
    surface: &dyn Surface,
    p: Point3,
    window: [ParamRange; 2],
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<SurfaceProjection, ProjectionError> {
    validate_surface_budget(|stage, resource, mode| {
        scope.ledger().require_limit(stage, resource, mode)
    })?;
    validate_point(p)?;
    validate_window(window[0], 0)?;
    validate_window(window[1], 1)?;
    charge_projection(scope, SURFACE_PROJECTION_QUERIES, 1)?;

    let n = SURFACE_SAMPLES;
    let sample = |i: usize, j: usize| {
        [
            window[0].lerp(i as f64 / n as f64),
            window[1].lerp(j as f64 / n as f64),
        ]
    };
    let mut fs = vec![0.0; (n + 1) * (n + 1)];
    let mut sample_count = 0_u64;
    for j in 0..=n {
        for i in 0..=n {
            sample_count += 1;
            observe_projection(
                scope,
                SURFACE_PROJECTION_SAMPLES,
                ResourceKind::Items,
                sample_count,
            )?;
            fs[j * (n + 1) + i] = surface_objective(surface, p, sample(i, j))?;
        }
    }
    let at = |i: usize, j: usize| fs[j * (n + 1) + i];
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

    let mut best: Option<(f64, [f64; 2])> = None;
    for (candidate, &(_, i, j)) in candidates.iter().enumerate() {
        observe_projection(
            scope,
            SURFACE_PROJECTION_CANDIDATES,
            ResourceKind::Items,
            (candidate + 1) as u64,
        )?;
        let (uv, f) = polish_surface_in_scope(surface, p, sample(i, j), window, scope)?;
        let better = match best {
            None => true,
            Some((bf, buv)) => (f, uv[0], uv[1]) < (bf, buv[0], buv[1]),
        };
        if better {
            best = Some((f, uv));
        }
    }
    let (f, mut uv) = best.ok_or(ProjectionError::NoCandidate)?;
    let base = surface.param_range();
    for (k, period) in surface.periodicity().iter().enumerate() {
        if let Some(period) = period {
            if !period.is_finite() || *period <= 0.0 || !base[k].lo.is_finite() {
                return Err(ProjectionError::NonFiniteEvaluation);
            }
            uv[k] = wrap_periodic(uv[k], base[k].lo, *period);
        }
    }
    let point = surface.eval(uv);
    if !finite_point(point) || !f.is_finite() || !uv.into_iter().all(f64::is_finite) {
        return Err(ProjectionError::NonFiniteEvaluation);
    }
    Ok(SurfaceProjection {
        uv,
        point,
        dist: f.sqrt(),
    })
}

fn polish_surface_in_scope(
    surface: &dyn Surface,
    p: Point3,
    uv0: [f64; 2],
    window: [ParamRange; 2],
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<([f64; 2], f64), ProjectionError> {
    let (wu, wv) = (window[0].width(), window[1].width());
    let conv_u = 1e-12 * wu.max(1.0);
    let conv_v = 1e-12 * wv.max(1.0);
    let (cell_u, cell_v) = (
        (wu / SURFACE_SAMPLES as f64).max(1e-12),
        (wv / SURFACE_SAMPLES as f64).max(1e-12),
    );
    let mut uv = uv0;
    let mut f_curr = surface_objective(surface, p, uv)?;
    for iteration in 0..MAX_ITER_SURFACE {
        observe_projection(
            scope,
            SURFACE_PROJECTION_NEWTON_ITERATIONS,
            ResourceKind::Depth,
            (iteration + 1) as u64,
        )?;
        let d = surface.eval_derivs(uv, 2);
        if ![d.p, d.du, d.dv, d.duu, d.duv, d.dvv]
            .into_iter()
            .all(finite_point)
        {
            return Err(ProjectionError::NonFiniteEvaluation);
        }
        let diff = d.p - p;
        let g0 = 2.0 * d.du.dot(diff);
        let g1 = 2.0 * d.dv.dot(diff);
        let g_scale = 2.0 * (d.du.norm() + d.dv.norm()) * diff.norm();
        if !g0.is_finite() || !g1.is_finite() || !g_scale.is_finite() {
            return Err(ProjectionError::NonFiniteEvaluation);
        }
        if g0.abs().max(g1.abs()) <= 1e-15 * (1.0 + g_scale) {
            return Ok((uv, f_curr));
        }
        let h00 = 2.0 * (d.duu.dot(diff) + d.du.norm_sq());
        let h01 = 2.0 * (d.duv.dot(diff) + d.du.dot(d.dv));
        let h11 = 2.0 * (d.dvv.dot(diff) + d.dv.norm_sq());
        let det = h00 * h11 - h01 * h01;
        if !h00.is_finite() || !h01.is_finite() || !h11.is_finite() || !det.is_finite() {
            return Err(ProjectionError::NonFiniteEvaluation);
        }
        let (mut su, mut sv) = if h00 > 0.0 && det > 0.0 && det.is_finite() {
            (-(h11 * g0 - h01 * g1) / det, -(h00 * g1 - h01 * g0) / det)
        } else {
            let gn = (g0 * g0 + g1 * g1).sqrt();
            if gn == 0.0 {
                return Ok((uv, f_curr));
            }
            (-g0 / gn * cell_u, -g1 / gn * cell_v)
        };
        su = su.clamp(-wu / 4.0, wu / 4.0);
        sv = sv.clamp(-wv / 4.0, wv / 4.0);
        if su.abs() <= conv_u && sv.abs() <= conv_v {
            return Ok((uv, f_curr));
        }
        let newton_ok = h00 > 0.0 && det > 0.0 && det.is_finite();
        if newton_ok && su.abs() <= 1e-6 * wu.max(1.0) && sv.abs() <= 1e-6 * wv.max(1.0) {
            uv = [
                (uv[0] + su).clamp(window[0].lo, window[0].hi),
                (uv[1] + sv).clamp(window[1].lo, window[1].hi),
            ];
            f_curr = surface_objective(surface, p, uv)?;
            continue;
        }
        let mut halvings = 0_u64;
        loop {
            let cand = [
                (uv[0] + su).clamp(window[0].lo, window[0].hi),
                (uv[1] + sv).clamp(window[1].lo, window[1].hi),
            ];
            if cand != uv {
                let f_new = surface_objective(surface, p, cand)?;
                if f_new <= f_curr {
                    uv = cand;
                    f_curr = f_new;
                    break;
                }
            }
            su *= 0.5;
            sv *= 0.5;
            halvings += 1;
            observe_projection(
                scope,
                SURFACE_PROJECTION_HALVINGS,
                ResourceKind::Depth,
                halvings,
            )?;
            if halvings >= MAX_HALVINGS as u64 || (su.abs() <= conv_u && sv.abs() <= conv_v) {
                if halvings >= MAX_HALVINGS as u64 {
                    observe_projection(
                        scope,
                        SURFACE_PROJECTION_HALVINGS,
                        ResourceKind::Depth,
                        MAX_HALVINGS as u64 + 1,
                    )?;
                }
                return Ok((uv, f_curr));
            }
        }
    }
    observe_projection(
        scope,
        SURFACE_PROJECTION_NEWTON_ITERATIONS,
        ResourceKind::Depth,
        MAX_ITER_SURFACE as u64 + 1,
    )?;
    unreachable!("the v1 Newton ceiling must reject its canonical crossing")
}

fn finite_point(point: Point3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

fn curve_objective(
    curve: &dyn Curve,
    point: Point3,
    t: f64,
) -> core::result::Result<f64, ProjectionError> {
    let evaluated = curve.eval(t);
    let objective = (evaluated - point).norm_sq();
    if finite_point(evaluated) && objective.is_finite() {
        Ok(objective)
    } else {
        Err(ProjectionError::NonFiniteEvaluation)
    }
}

fn surface_objective(
    surface: &dyn Surface,
    point: Point3,
    uv: [f64; 2],
) -> core::result::Result<f64, ProjectionError> {
    let evaluated = surface.eval(uv);
    let objective = (evaluated - point).norm_sq();
    if finite_point(evaluated) && objective.is_finite() {
        Ok(objective)
    } else {
        Err(ProjectionError::NonFiniteEvaluation)
    }
}

fn validate_point(point: Point3) -> core::result::Result<(), ProjectionError> {
    if finite_point(point) {
        Ok(())
    } else {
        Err(ProjectionError::InvalidQueryPoint)
    }
}

fn validate_window(
    window: ParamRange,
    direction: usize,
) -> core::result::Result<(), ProjectionError> {
    if window.is_finite() && window.lo <= window.hi {
        Ok(())
    } else {
        Err(ProjectionError::InvalidWindow { direction })
    }
}

fn projection_limits(
    curve: bool,
) -> [(kcore::operation::StageId, ResourceKind, AccountingMode); 5] {
    if curve {
        [
            (
                CURVE_PROJECTION_QUERIES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
            ),
            (
                CURVE_PROJECTION_SAMPLES,
                ResourceKind::Items,
                AccountingMode::HighWater,
            ),
            (
                CURVE_PROJECTION_CANDIDATES,
                ResourceKind::Items,
                AccountingMode::HighWater,
            ),
            (
                CURVE_PROJECTION_NEWTON_ITERATIONS,
                ResourceKind::Depth,
                AccountingMode::HighWater,
            ),
            (
                CURVE_PROJECTION_HALVINGS,
                ResourceKind::Depth,
                AccountingMode::HighWater,
            ),
        ]
    } else {
        [
            (
                SURFACE_PROJECTION_QUERIES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
            ),
            (
                SURFACE_PROJECTION_SAMPLES,
                ResourceKind::Items,
                AccountingMode::HighWater,
            ),
            (
                SURFACE_PROJECTION_CANDIDATES,
                ResourceKind::Items,
                AccountingMode::HighWater,
            ),
            (
                SURFACE_PROJECTION_NEWTON_ITERATIONS,
                ResourceKind::Depth,
                AccountingMode::HighWater,
            ),
            (
                SURFACE_PROJECTION_HALVINGS,
                ResourceKind::Depth,
                AccountingMode::HighWater,
            ),
        ]
    }
}

fn validate_curve_budget(
    mut require: impl FnMut(
        kcore::operation::StageId,
        ResourceKind,
        AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError>,
) -> core::result::Result<(), OperationPolicyError> {
    for (stage, resource, mode) in projection_limits(true) {
        require(stage, resource, mode)?;
    }
    Ok(())
}

fn validate_surface_budget(
    mut require: impl FnMut(
        kcore::operation::StageId,
        ResourceKind,
        AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError>,
) -> core::result::Result<(), OperationPolicyError> {
    for (stage, resource, mode) in projection_limits(false) {
        require(stage, resource, mode)?;
    }
    Ok(())
}

fn charge_projection(
    scope: &mut OperationScope<'_, '_>,
    stage: kcore::operation::StageId,
    amount: u64,
) -> core::result::Result<(), ProjectionError> {
    let result = scope.ledger_mut().charge(stage, amount);
    projection_accounting_result(scope, stage, result)
}

fn observe_projection(
    scope: &mut OperationScope<'_, '_>,
    stage: kcore::operation::StageId,
    resource: ResourceKind,
    value: u64,
) -> core::result::Result<(), ProjectionError> {
    let result = scope.ledger_mut().observe(stage, resource, value);
    projection_accounting_result(scope, stage, result)
}

fn projection_accounting_result(
    scope: &mut OperationScope<'_, '_>,
    stage: kcore::operation::StageId,
    result: core::result::Result<(), OperationPolicyError>,
) -> core::result::Result<(), ProjectionError> {
    if let Err(OperationPolicyError::LimitReached(snapshot)) = &result {
        scope.diagnose(
            stage,
            PROJECTION_LIMIT_REACHED,
            DiagnosticKind::LimitReached(*snapshot),
            "closest-point projection resource limit reached",
        );
    }
    result.map_err(ProjectionError::Policy)
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
    use kcore::operation::{
        BudgetPlan, DiagnosticLevel, ExecutionPolicy, LimitSpec, NumericalPolicy, PolicyVersion,
        SessionPolicy, SessionPrecision, TOTAL_WORK_STAGE,
    };
    use kcore::tolerance::Tolerances;

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

    fn session(budget: BudgetPlan) -> SessionPolicy {
        SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            budget,
            PolicyVersion::V1,
        )
    }

    fn override_limit(
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        mode: AccountingMode,
        allowed: u64,
    ) -> BudgetPlan {
        BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap()
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

    #[test]
    fn contextual_curve_and_surface_are_bit_equivalent_to_v1() {
        let line = Line::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(-1.0, 0.5, 2.0)).unwrap();
        let point = Point3::new(8.0, -4.0, 2.5);
        let curve_window = ParamRange::new(-20.0, 20.0);
        let legacy_curve = project_to_curve(&line, point, curve_window).unwrap();
        let curve_session = session(BudgetPlan::empty());
        let curve_context = OperationContext::new(&curve_session, Tolerances::default()).unwrap();
        let contextual_curve =
            project_to_curve_with_context(&line, point, curve_window, &curve_context).unwrap();
        assert_eq!(contextual_curve.result(), Ok(&legacy_curve));
        assert_eq!(contextual_curve.report().usage().len(), 5);

        let plane = Plane::new(tilted_frame());
        let surface_window = [ParamRange::new(-20.0, 20.0), ParamRange::new(-20.0, 20.0)];
        let legacy_surface = project_to_surface(&plane, point, surface_window).unwrap();
        let surface_session = session(BudgetPlan::empty());
        let surface_context =
            OperationContext::new(&surface_session, Tolerances::default()).unwrap();
        let contextual_surface =
            project_to_surface_with_context(&plane, point, surface_window, &surface_context)
                .unwrap();
        assert_eq!(contextual_surface.result(), Ok(&legacy_surface));
        assert_eq!(contextual_surface.report().usage().len(), 5);
    }

    #[test]
    fn contextual_invalid_inputs_and_nonfinite_evaluation_never_panic() {
        let line = Line::new(Point3::default(), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let policy = session(BudgetPlan::empty());
        let context = OperationContext::new(&policy, Tolerances::default()).unwrap();
        let reversed = ParamRange { lo: 2.0, hi: 1.0 };
        let unbounded = ParamRange::unbounded();

        assert!(matches!(
            project_to_curve_with_context(&line, Point3::default(), reversed, &context)
                .unwrap()
                .result(),
            Err(ProjectionError::InvalidWindow { direction: 0 })
        ));
        assert!(matches!(
            project_to_curve_with_context(
                &line,
                Point3::new(f64::NAN, 0.0, 0.0),
                ParamRange::new(0.0, 1.0),
                &context,
            )
            .unwrap()
            .result(),
            Err(ProjectionError::InvalidQueryPoint)
        ));
        assert!(matches!(
            project_to_surface_with_context(
                &Plane::new(tilted_frame()),
                Point3::default(),
                [ParamRange::new(0.0, 1.0), unbounded],
                &context,
            )
            .unwrap()
            .result(),
            Err(ProjectionError::InvalidWindow { direction: 1 })
        ));
        assert!(matches!(
            project_to_curve_with_context(
                &line,
                Point3::new(f64::MAX, 0.0, 0.0),
                ParamRange::new(0.0, 1.0),
                &context,
            )
            .unwrap()
            .result(),
            Err(ProjectionError::NonFiniteEvaluation)
        ));
    }

    #[test]
    fn repeated_shared_scope_queries_accumulate_only_query_work() {
        let line = Line::new(Point3::default(), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let policy = session(BudgetPlan::empty());
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(ProjectionBudgetProfile::curve_defaults())
            .with_budget_overrides(override_limit(
                CURVE_PROJECTION_QUERIES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                2,
            ));
        let mut scope = OperationScope::new(&context);
        let window = ParamRange::new(2.0, 2.0);
        let point = Point3::new(5.0, 1.0, 0.0);

        let first = project_to_curve_in_scope(&line, point, window, &mut scope).unwrap();
        let second = project_to_curve_in_scope(&line, point, window, &mut scope).unwrap();
        assert_eq!(first, second);
        let query = scope
            .ledger()
            .snapshots()
            .into_iter()
            .find(|snapshot| snapshot.stage == CURVE_PROJECTION_QUERIES)
            .unwrap();
        let samples = scope
            .ledger()
            .snapshots()
            .into_iter()
            .find(|snapshot| snapshot.stage == CURVE_PROJECTION_SAMPLES)
            .unwrap();
        assert_eq!(query.consumed, 2);
        assert_eq!(samples.consumed, 65);
    }

    #[test]
    fn family_session_request_and_root_precedence_is_enforced() {
        let line = Line::new(Point3::default(), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let point = Point3::new(0.25, 1.0, 0.0);
        let window = ParamRange::new(0.0, 1.0);
        let session_budget = override_limit(
            CURVE_PROJECTION_SAMPLES,
            ResourceKind::Items,
            AccountingMode::HighWater,
            64,
        );
        let policy = session(session_budget);
        let denied_context = OperationContext::new(&policy, Tolerances::default()).unwrap();
        let denied = project_to_curve_with_context(&line, point, window, &denied_context).unwrap();
        assert!(matches!(
            denied.result(),
            Err(ProjectionError::Policy(OperationPolicyError::LimitReached(snapshot)))
                if snapshot.stage == CURVE_PROJECTION_SAMPLES
                    && snapshot.consumed == 65
                    && snapshot.allowed == 64
        ));

        let request_context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_budget_overrides(override_limit(
                CURVE_PROJECTION_SAMPLES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65,
            ));
        assert!(
            project_to_curve_with_context(&line, point, window, &request_context)
                .unwrap()
                .result()
                .is_ok()
        );

        let root_policy = session(BudgetPlan::empty().with_total_work_limit(0));
        let root_context = OperationContext::new(&root_policy, Tolerances::default()).unwrap();
        let root = project_to_curve_with_context(&line, point, window, &root_context).unwrap();
        assert!(matches!(
            root.result(),
            Err(ProjectionError::Policy(OperationPolicyError::LimitReached(snapshot)))
                if snapshot.stage == TOTAL_WORK_STAGE
                    && snapshot.consumed == 1
                    && snapshot.allowed == 0
        ));
    }

    #[test]
    fn limit_diagnostics_and_reports_are_deterministic() {
        let run = || {
            let line = Line::new(Point3::default(), Vec3::new(1.0, 0.0, 0.0)).unwrap();
            let policy = session(override_limit(
                CURVE_PROJECTION_SAMPLES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                64,
            ));
            let context = OperationContext::new(&policy, Tolerances::default())
                .unwrap()
                .with_diagnostics(DiagnosticLevel::Summary, 4);
            project_to_curve_with_context(
                &line,
                Point3::new(0.25, 1.0, 0.0),
                ParamRange::new(0.0, 1.0),
                &context,
            )
            .unwrap()
        };

        let first = run();
        let second = run();
        assert_eq!(first.report(), second.report());
        assert_eq!(first.report().limit_events().len(), 1);
        assert_eq!(first.report().diagnostics().len(), 1);
        assert_eq!(
            first.report().diagnostics()[0].code,
            PROJECTION_LIMIT_REACHED
        );
        assert_eq!(
            first.report().diagnostics()[0].stage,
            CURVE_PROJECTION_SAMPLES
        );
    }

    #[test]
    fn public_projection_errors_classify_and_chain_policy_sources() {
        let invalid = ProjectionError::InvalidWindow { direction: 0 };
        assert_eq!(invalid.class(), ErrorClass::InvalidInput);
        assert_eq!(invalid.code(), error_code::INVALID_WINDOW);
        assert!(std::error::Error::source(&invalid).is_none());

        let policy = ProjectionError::Policy(OperationPolicyError::InvalidOperationTolerance);
        assert_eq!(policy.class(), ErrorClass::InvalidInput);
        assert!(std::error::Error::source(&policy).is_some());
    }
}
