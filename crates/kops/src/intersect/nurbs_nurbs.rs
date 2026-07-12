use super::error::{IntersectionError, IntersectionResult};
use super::numerical::{
    directional_gradients_are_numerically_zero, nonnegative_values_are_numerically_equal,
    normalized_cross_magnitude, parameter_progress_step, solve_symmetric_2x2,
    ternary_interval_has_no_progress,
};
use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint, ParamOrientation,
    accept_curve_curve_candidate,
};
use kcore::error::{CapabilityId, Error, Result};
use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, DiagnosticKind, LimitSnapshot, LimitSpec,
    NumericalPolicy, OperationContext, OperationOutcome, OperationPolicyError, OperationScope,
    ResourceKind, SessionPolicy, StageId,
};
use kcore::proof::{IncompleteCause, IncompleteEvidence};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::{
    ContextCurvePairIsolationError, CurvePairCandidateCell, CurvePairIsolationLimits, NurbsCurve,
    NurbsCurvePairBudgetProfile, isolate_curve_pair_candidates_in_scope,
};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

const MIN_STEPS: usize = 96;
const MAX_STEPS: usize = 384;
const MAX_POLISH_STEPS: usize = 32;
const MAX_MINIMIZE_STEPS: usize = 80;
const OVERLAP_SAMPLES: usize = 32;
const DEFAULT_SEED_ATTEMPTS: u64 = 4_096;
const CURVE_PAIR_COMPLETION_REASON: &str =
    "NURBS curve-pair candidate cells do not yet have complete root and overlap coverage";

const fn stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid NURBS curve-pair solve stage"),
    }
}

/// Cumulative bounded cell-local seed and polish attempts.
pub const NURBS_CURVE_PAIR_SEED_ATTEMPTS: StageId =
    stage("kops.intersect.nurbs-curve-pair-seed-attempts");

const fn diagnostic(value: &'static str) -> DiagnosticCode {
    match DiagnosticCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid NURBS curve-pair diagnostic"),
    }
}

const fn capability(value: &'static str) -> CapabilityId {
    match CapabilityId::new(value) {
        Ok(capability) => capability,
        Err(_) => panic!("invalid NURBS curve-pair capability"),
    }
}

/// Complete root and overlap coverage for retained NURBS curve-pair cells.
pub const NURBS_CURVE_PAIR_COMPLETE_COVERAGE: CapabilityId =
    capability("kops.intersect.nurbs-curve-pair-complete-coverage");
/// Exact isolation subdivision work stopped before full requested coverage.
pub const NURBS_CURVE_PAIR_ISOLATION_SUBDIVISION_LIMIT: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-isolation-subdivision-limit");
/// Exact isolation retained-cell capacity stopped before full coverage.
pub const NURBS_CURVE_PAIR_ISOLATION_CANDIDATE_LIMIT: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-isolation-candidate-limit");
/// Exact isolation depth stopped before full requested coverage.
pub const NURBS_CURVE_PAIR_ISOLATION_DEPTH_LIMIT: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-isolation-depth-limit");
/// Exact isolation stopped at arithmetic parameter resolution.
pub const NURBS_CURVE_PAIR_ISOLATION_PARAMETER_RESOLUTION: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-isolation-parameter-resolution");
/// Exact subdivision was unavailable for a valid retained cell.
pub const NURBS_CURVE_PAIR_ISOLATION_METHOD_UNAVAILABLE: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-isolation-method-unavailable");
/// Cell-local seed work stopped before every retained cell was attempted.
pub const NURBS_CURVE_PAIR_SEED_LIMIT: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-seed-limit");
/// Retained cells still lack a complete root/overlap proof method.
pub const NURBS_CURVE_PAIR_COVERAGE_INCOMPLETE: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-coverage-incomplete");

/// Newton stopped at a numerically stationary directional gradient without a witness.
pub const NURBS_CURVE_PAIR_POLISH_STATIONARY: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-polish-stationary");
/// Newton's symmetric system was too ill-conditioned to solve safely.
pub const NURBS_CURVE_PAIR_POLISH_ILL_CONDITIONED: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-polish-ill-conditioned");
/// Damped Newton found no non-increasing step.
pub const NURBS_CURVE_PAIR_POLISH_NO_DESCENT: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-polish-no-descent");
/// Accepted parameter displacement reached arithmetic resolution without a witness.
pub const NURBS_CURVE_PAIR_POLISH_PARAMETER_RESOLUTION: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-polish-parameter-resolution");
/// Newton consumed its fixed iteration bound without a witness.
pub const NURBS_CURVE_PAIR_POLISH_ITERATION_LIMIT: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-polish-iteration-limit");
/// The bounded local minimization fallback was selected.
pub const NURBS_CURVE_PAIR_POLISH_FALLBACK: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-polish-fallback");
/// A fallback minimizer reached arithmetic parameter resolution.
pub const NURBS_CURVE_PAIR_MINIMIZER_PARAMETER_RESOLUTION: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-minimizer-parameter-resolution");
/// A fallback minimizer observed a non-finite or negative objective.
pub const NURBS_CURVE_PAIR_MINIMIZER_INVALID_OBJECTIVE: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-minimizer-invalid-objective");
/// A fallback minimizer consumed its fixed iteration bound.
pub const NURBS_CURVE_PAIR_MINIMIZER_ITERATION_LIMIT: DiagnosticCode =
    diagnostic("kops.intersect.nurbs-curve-pair-minimizer-iteration-limit");

/// Every diagnostic identity owned by NURBS curve-pair polishing.
pub const NURBS_CURVE_PAIR_POLISH_DIAGNOSTICS: &[DiagnosticCode] = &[
    NURBS_CURVE_PAIR_POLISH_STATIONARY,
    NURBS_CURVE_PAIR_POLISH_ILL_CONDITIONED,
    NURBS_CURVE_PAIR_POLISH_NO_DESCENT,
    NURBS_CURVE_PAIR_POLISH_PARAMETER_RESOLUTION,
    NURBS_CURVE_PAIR_POLISH_ITERATION_LIMIT,
    NURBS_CURVE_PAIR_POLISH_FALLBACK,
];

/// Every diagnostic identity owned by NURBS curve-pair fallback minimization.
pub const NURBS_CURVE_PAIR_MINIMIZER_DIAGNOSTICS: &[DiagnosticCode] = &[
    NURBS_CURVE_PAIR_MINIMIZER_PARAMETER_RESOLUTION,
    NURBS_CURVE_PAIR_MINIMIZER_INVALID_OBJECTIVE,
    NURBS_CURVE_PAIR_MINIMIZER_ITERATION_LIMIT,
];

/// Every stable incomplete-proof identity owned by NURBS curve-pair solving.
pub const NURBS_CURVE_PAIR_PROOF_DIAGNOSTICS: &[DiagnosticCode] = &[
    NURBS_CURVE_PAIR_ISOLATION_SUBDIVISION_LIMIT,
    NURBS_CURVE_PAIR_ISOLATION_CANDIDATE_LIMIT,
    NURBS_CURVE_PAIR_ISOLATION_DEPTH_LIMIT,
    NURBS_CURVE_PAIR_ISOLATION_PARAMETER_RESOLUTION,
    NURBS_CURVE_PAIR_ISOLATION_METHOD_UNAVAILABLE,
    NURBS_CURVE_PAIR_SEED_LIMIT,
    NURBS_CURVE_PAIR_COVERAGE_INCOMPLETE,
];

/// Version-1 composed profile for exact isolation plus cell-local discovery.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct NurbsCurvePairSolveBudgetProfile;

impl NurbsCurvePairSolveBudgetProfile {
    /// At most one statically bounded polish attempt per retained isolation
    /// cell. The isolation profile itself caps that cover at 4,096 cells.
    pub fn v1_defaults() -> BudgetPlan {
        let isolation = NurbsCurvePairBudgetProfile::v1_defaults();
        BudgetPlan::new(isolation.limits().iter().copied().chain([LimitSpec::new(
            NURBS_CURVE_PAIR_SEED_ATTEMPTS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            DEFAULT_SEED_ATTEMPTS,
        )]))
        .expect("built-in NURBS curve-pair solve profile is valid")
    }

    /// Require isolation and discovery stages with canonical accounting.
    pub fn validate(plan: &BudgetPlan) -> core::result::Result<(), OperationPolicyError> {
        NurbsCurvePairBudgetProfile::validate(plan)?;
        plan.require_limit(
            NURBS_CURVE_PAIR_SEED_ATTEMPTS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
struct Sample {
    t: f64,
    point: Point3,
}

#[derive(Debug, Clone, Copy)]
struct PolishPolicy {
    range_a: ParamRange,
    range_b: ParamRange,
    tolerances: Tolerances,
    numerical: NumericalPolicy,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct VerifiedCurvePairSeed {
    t_a: f64,
    t_b: f64,
    point_a: Point3,
    point_b: Point3,
    residual: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NewtonPolishStop {
    GradientStationary,
    IllConditioned,
    NoDescent,
    ParameterResolution,
    IterationLimit,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct NewtonPolishOutcome {
    t_a: f64,
    t_b: f64,
    stop: NewtonPolishStop,
}

impl NewtonPolishOutcome {
    const fn parameters(self) -> (f64, f64) {
        (self.t_a, self.t_b)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PolishCandidateOutcome {
    t_a: f64,
    t_b: f64,
    stop: NewtonPolishStop,
    fallback_selected: bool,
    minimizer_stops: MinimizerStopSet,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MinimizerStop {
    ParameterResolution,
    InvalidObjective,
    IterationLimit,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct MinimizerStopSet(u8);

impl MinimizerStopSet {
    const PARAMETER_RESOLUTION: u8 = 1 << 0;
    const INVALID_OBJECTIVE: u8 = 1 << 1;
    const ITERATION_LIMIT: u8 = 1 << 2;

    const fn one(stop: MinimizerStop) -> Self {
        Self(match stop {
            MinimizerStop::ParameterResolution => Self::PARAMETER_RESOLUTION,
            MinimizerStop::InvalidObjective => Self::INVALID_OBJECTIVE,
            MinimizerStop::IterationLimit => Self::ITERATION_LIMIT,
        })
    }

    fn insert(&mut self, stop: MinimizerStop) {
        self.0 |= Self::one(stop).0;
    }

    fn extend(&mut self, other: Self) {
        self.0 |= other.0;
    }

    const fn contains(self, stop: MinimizerStop) -> bool {
        self.0 & Self::one(stop).0 != 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct MinimizeOutcome {
    parameter: f64,
    stop: MinimizerStop,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct NestedMinimizeOutcome {
    parameter: f64,
    stops: MinimizerStopSet,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct LocalRefinementOutcome {
    t_a: f64,
    t_b: f64,
    stops: MinimizerStopSet,
}

impl VerifiedCurvePairSeed {
    fn verify(
        a: &NurbsCurve,
        range_a: ParamRange,
        t_a: f64,
        b: &NurbsCurve,
        range_b: ParamRange,
        t_b: f64,
        tolerance: f64,
    ) -> Option<Self> {
        if !t_a.is_finite() || !t_b.is_finite() || !range_a.contains(t_a) || !range_b.contains(t_b)
        {
            return None;
        }
        let point_a = a.eval(t_a);
        let point_b = b.eval(t_b);
        if [
            point_a.x, point_a.y, point_a.z, point_b.x, point_b.y, point_b.z,
        ]
        .into_iter()
        .any(|value| !value.is_finite())
        {
            return None;
        }
        let residual = point_a.dist(point_b);
        (residual.is_finite() && residual <= tolerance).then_some(Self {
            t_a,
            t_b,
            point_a,
            point_b,
            residual,
        })
    }

    fn into_point(self, kind: ContactKind) -> CurveCurvePoint {
        CurveCurvePoint {
            point: (self.point_a + self.point_b) / 2.0,
            t_a: self.t_a,
            t_b: self.t_b,
            residual: self.residual,
            kind,
        }
    }
}

/// Intersect two clamped NURBS curves restricted to finite ranges.
///
/// This is the first general NURBS/NURBS curve bridge: it isolates exact
/// conservative subcurve-pair cells, chooses one deterministic local seed per
/// cell, polishes it by safeguarded Newton iteration, and emits only
/// re-evaluated tolerance witnesses. It also reports simple contained spans
/// when the first curve range is provisionally found to lie on the second.
pub fn intersect_bounded_nurbs_nurbs(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances)
        .expect("validated Tolerances always satisfy v1 session precision");
    intersect_bounded_nurbs_nurbs_with_context(a, range_a, b, range_b, &context).into_result()
}

/// Context-aware bounded NURBS/NURBS curve intersection.
///
/// The operation's numerical policy controls the Newton system conditioning
/// guard, normalized directional-gradient stop, collapsed-parameter detection,
/// Newton parameter-progress stop, and minimizer progress/value guards.
/// These guards never grant candidate or overlap acceptance: candidates retain
/// their model-space residual checks, while overlap and input parameter slack
/// retain their legacy v1 semantics. Segment degeneracy and parameter-based
/// candidate deduplication remain separate migrations.
pub fn intersect_bounded_nurbs_nurbs_with_context(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
    context: &OperationContext<'_>,
) -> OperationOutcome<CurveCurveIntersections> {
    let context = context
        .clone()
        .with_family_budget_defaults(NurbsCurvePairSolveBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    let result = intersect_bounded_nurbs_nurbs_contextual_impl(a, range_a, b, range_b, &mut scope);
    scope.finish(result)
}

pub(super) fn intersect_bounded_nurbs_nurbs_in_scope(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> IntersectionResult<CurveCurveIntersections> {
    intersect_bounded_nurbs_nurbs_contextual_impl(a, range_a, b, range_b, scope)
        .map_err(IntersectionError::from)
}

fn intersect_bounded_nurbs_nurbs_contextual_impl(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<CurveCurveIntersections> {
    let tolerances = scope.context().tolerances();
    let numerical = scope.context().session().numerical();
    NurbsCurvePairSolveBudgetProfile::validate(&scope.context().effective_budget())?;
    validate_ranges(a, range_a, b, range_b, tolerances)?;
    let range_a = clamp_to_domain(range_a, a.param_range());
    let range_b = clamp_to_domain(range_b, b.param_range());
    let collapsed_a = range_has_no_parameter_progress(range_a, tolerances, numerical);
    let collapsed_b = range_has_no_parameter_progress(range_b, tolerances, numerical);
    if !collapsed_a && !collapsed_b {
        let isolation = isolate_curve_pair_candidates_in_scope(
            a,
            range_a,
            b,
            range_b,
            tolerances.linear(),
            NurbsCurvePairBudgetProfile::default_depth(),
            scope,
        )
        .map_err(|error| match error {
            ContextCurvePairIsolationError::Kernel(error) => error,
            ContextCurvePairIsolationError::Policy(error) => Error::from(error),
        })?;
        let incomplete_evidence = diagnose_curve_pair_isolation_limits(scope, isolation.limits());
        if isolation.is_proven_empty() && incomplete_evidence.is_empty() {
            return Ok(CurveCurveIntersections::complete_empty());
        }
        return intersect_bounded_nurbs_nurbs_candidates_impl(
            a,
            b,
            isolation.candidates(),
            PolishPolicy {
                range_a,
                range_b,
                tolerances,
                numerical,
            },
            scope,
            incomplete_evidence,
        );
    }
    degenerate_range_intersections(a, range_a, collapsed_a, b, range_b, tolerances, numerical)
}

fn curve_pair_coverage_incomplete_evidence() -> IncompleteEvidence {
    IncompleteEvidence {
        code: NURBS_CURVE_PAIR_COVERAGE_INCOMPLETE,
        stage: NURBS_CURVE_PAIR_SEED_ATTEMPTS,
        cause: IncompleteCause::ProofMethodUnavailable {
            capability: NURBS_CURVE_PAIR_COMPLETE_COVERAGE,
        },
        message: CURVE_PAIR_COMPLETION_REASON,
    }
}

/// Evidence order follows the proof pipeline: subdivision work, retained
/// candidates, depth, arithmetic resolution, then method availability.
fn diagnose_curve_pair_isolation_limits(
    scope: &mut OperationScope<'_, '_>,
    limits: CurvePairIsolationLimits,
) -> Vec<IncompleteEvidence> {
    let mut evidence = Vec::new();
    if let Some(snapshot) = limits.subdivision_work() {
        evidence.push(diagnose_curve_pair_limit(
            scope,
            snapshot,
            NURBS_CURVE_PAIR_ISOLATION_SUBDIVISION_LIMIT,
            "NURBS curve-pair isolation subdivision limit reached",
        ));
    }
    if let Some(snapshot) = limits.candidate_cells() {
        evidence.push(diagnose_curve_pair_limit(
            scope,
            snapshot,
            NURBS_CURVE_PAIR_ISOLATION_CANDIDATE_LIMIT,
            "NURBS curve-pair isolation candidate-cover limit reached",
        ));
    }
    if let Some(snapshot) = limits.subdivision_depth() {
        evidence.push(diagnose_curve_pair_limit(
            scope,
            snapshot,
            NURBS_CURVE_PAIR_ISOLATION_DEPTH_LIMIT,
            "NURBS curve-pair isolation depth limit reached",
        ));
    }
    if limits.parameter_resolution() {
        const MESSAGE: &str =
            "NURBS curve-pair isolation stopped at floating-point parameter resolution";
        scope.diagnose(
            kgeom::nurbs::NURBS_CURVE_PAIR_DEPTH,
            NURBS_CURVE_PAIR_ISOLATION_PARAMETER_RESOLUTION,
            DiagnosticKind::NumericResolution,
            MESSAGE,
        );
        evidence.push(IncompleteEvidence {
            code: NURBS_CURVE_PAIR_ISOLATION_PARAMETER_RESOLUTION,
            stage: kgeom::nurbs::NURBS_CURVE_PAIR_DEPTH,
            cause: IncompleteCause::NumericResolution,
            message: MESSAGE,
        });
    }
    if limits.subdivision_unavailable() {
        const MESSAGE: &str = "NURBS curve-pair exact subdivision is unavailable for this cell";
        scope.diagnose(
            kgeom::nurbs::NURBS_CURVE_PAIR_DEPTH,
            NURBS_CURVE_PAIR_ISOLATION_METHOD_UNAVAILABLE,
            DiagnosticKind::ProofIncomplete,
            MESSAGE,
        );
        evidence.push(IncompleteEvidence {
            code: NURBS_CURVE_PAIR_ISOLATION_METHOD_UNAVAILABLE,
            stage: kgeom::nurbs::NURBS_CURVE_PAIR_DEPTH,
            cause: IncompleteCause::ProofMethodUnavailable {
                capability: NURBS_CURVE_PAIR_COMPLETE_COVERAGE,
            },
            message: MESSAGE,
        });
    }
    evidence
}

fn diagnose_curve_pair_limit(
    scope: &mut OperationScope<'_, '_>,
    snapshot: LimitSnapshot,
    code: DiagnosticCode,
    message: &'static str,
) -> IncompleteEvidence {
    scope.diagnose(
        snapshot.stage,
        code,
        DiagnosticKind::LimitReached(snapshot),
        message,
    );
    IncompleteEvidence {
        code,
        stage: snapshot.stage,
        cause: IncompleteCause::Limit { snapshot },
        message,
    }
}

fn intersect_bounded_nurbs_nurbs_candidates_impl(
    a: &NurbsCurve,
    b: &NurbsCurve,
    candidates: &[CurvePairCandidateCell],
    policy: PolishPolicy,
    scope: &mut OperationScope<'_, '_>,
    mut incomplete_evidence: Vec<IncompleteEvidence>,
) -> Result<CurveCurveIntersections> {
    if let Some(overlap) = contained_overlap(
        a,
        policy.range_a,
        b,
        policy.range_b,
        policy.tolerances,
        policy.numerical,
    ) {
        incomplete_evidence.push(curve_pair_coverage_incomplete_evidence());
        return CurveCurveIntersections::canonicalized_with_incomplete_evidence(
            Vec::new(),
            vec![overlap],
            CURVE_PAIR_COMPLETION_REASON,
            incomplete_evidence,
        );
    }

    let mut points = Vec::new();
    for cell in candidates {
        match scope.ledger_mut().charge(NURBS_CURVE_PAIR_SEED_ATTEMPTS, 1) {
            Ok(()) => {}
            Err(OperationPolicyError::LimitReached(snapshot)) => {
                incomplete_evidence.push(diagnose_curve_pair_limit(
                    scope,
                    snapshot,
                    NURBS_CURVE_PAIR_SEED_LIMIT,
                    "NURBS curve-pair seed-attempt limit reached",
                ));
                break;
            }
            Err(error) => return Err(error.into()),
        }
        let cell_range_a = cell.first_range();
        let cell_range_b = cell.second_range();
        let (seed_a, seed_b) = seed_for_candidate_cell(a, cell_range_a, b, cell_range_b);
        let polish = PolishPolicy {
            range_a: cell_range_a,
            range_b: cell_range_b,
            ..policy
        };
        let polished = polish_candidate(a, b, seed_a, seed_b, polish);
        if polished.fallback_selected {
            scope.diagnose(
                NURBS_CURVE_PAIR_SEED_ATTEMPTS,
                NURBS_CURVE_PAIR_POLISH_FALLBACK,
                DiagnosticKind::FallbackSelected,
                "NURBS curve-pair polishing selected bounded local minimization",
            );
        }
        let Some(seed) = VerifiedCurvePairSeed::verify(
            a,
            cell_range_a,
            polished.t_a,
            b,
            cell_range_b,
            polished.t_b,
            policy.tolerances.linear(),
        ) else {
            diagnose_minimizer_stops(scope, polished.minimizer_stops);
            diagnose_polish_stop(scope, polished.stop);
            continue;
        };
        let kind = contact_kind(a, seed.t_a, b, seed.t_b, policy.tolerances);
        push_distinct_point(&mut points, seed.into_point(kind), policy.tolerances);
    }
    if !candidates.is_empty() {
        incomplete_evidence.push(curve_pair_coverage_incomplete_evidence());
    }
    CurveCurveIntersections::canonicalized_with_incomplete_evidence(
        points,
        Vec::new(),
        CURVE_PAIR_COMPLETION_REASON,
        incomplete_evidence,
    )
}

fn seed_for_candidate_cell(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
) -> (f64, f64) {
    let a_lo = a.eval(range_a.lo);
    let a_hi = a.eval(range_a.hi);
    let b_lo = b.eval(range_b.lo);
    let b_hi = b.eval(range_b.hi);
    let (s, t, chord_distance) = closest_segment_parameters(a_lo, a_hi, b_lo, b_hi);
    let chord_seed = (range_a.lerp(s), range_b.lerp(t));
    let midpoint_seed = (range_a.lerp(0.5), range_b.lerp(0.5));
    let midpoint_distance = a.eval(midpoint_seed.0).dist(b.eval(midpoint_seed.1));
    if midpoint_distance < chord_distance {
        midpoint_seed
    } else {
        chord_seed
    }
}

fn degenerate_range_intersections(
    a: &NurbsCurve,
    range_a: ParamRange,
    collapsed_a: bool,
    b: &NurbsCurve,
    range_b: ParamRange,
    tolerances: Tolerances,
    numerical: NumericalPolicy,
) -> Result<CurveCurveIntersections> {
    let (t_a, t_b) = if collapsed_a {
        let t_a = range_a.lo;
        let t_b = closest_parameter_to_point(b, range_b, a.eval(t_a), numerical);
        (t_a, t_b)
    } else {
        let t_b = range_b.lo;
        let t_a = closest_parameter_to_point(a, range_a, b.eval(t_b), numerical);
        (t_a, t_b)
    };
    let mut points = Vec::new();
    push_root_candidate(a, t_a, b, t_b, &mut points, tolerances);
    CurveCurveIntersections::canonicalized_with_incomplete_evidence(
        points,
        Vec::new(),
        CURVE_PAIR_COMPLETION_REASON,
        vec![curve_pair_coverage_incomplete_evidence()],
    )
}

fn contained_overlap(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
    tolerances: Tolerances,
    numerical: NumericalPolicy,
) -> Option<CurveCurveOverlap> {
    let mut mapped = Vec::with_capacity(OVERLAP_SAMPLES + 1);
    for i in 0..=OVERLAP_SAMPLES {
        let t_a = range_a.lerp(i as f64 / OVERLAP_SAMPLES as f64);
        let point = a.eval(t_a);
        let t_b = closest_parameter_to_point(b, range_b, point, numerical);
        if point.dist(b.eval(t_b)) > tolerances.linear() {
            return None;
        }
        mapped.push(t_b);
    }

    let parameter_tol = legacy_parameter_slack(range_b, tolerances);
    let increasing = mapped
        .windows(2)
        .all(|pair| pair[1] + parameter_tol >= pair[0]);
    let decreasing = mapped
        .windows(2)
        .all(|pair| pair[0] + parameter_tol >= pair[1]);
    if !increasing && !decreasing {
        return None;
    }

    let first = snap_to_range_bounds(mapped[0], range_b, parameter_tol);
    let last = snap_to_range_bounds(mapped[mapped.len() - 1], range_b, parameter_tol);
    if (last - first).abs() <= parameter_tol {
        return None;
    }
    Some(CurveCurveOverlap {
        a: range_a,
        b: ParamRange::new(first.min(last), first.max(last)),
        orientation: if last >= first {
            ParamOrientation::Same
        } else {
            ParamOrientation::Reversed
        },
    })
}

fn sample_curve(curve: &NurbsCurve, range: ParamRange) -> Vec<Sample> {
    let span_hint = curve
        .knots()
        .control_count()
        .saturating_sub(curve.degree())
        .max(1);
    let steps = (span_hint * curve.degree().max(1) * 32).clamp(MIN_STEPS, MAX_STEPS);
    (0..=steps)
        .map(|i| {
            let t = range.lerp(i as f64 / steps as f64);
            Sample {
                t,
                point: curve.eval(t),
            }
        })
        .collect()
}

fn closest_segment_parameters(p0: Point3, p1: Point3, q0: Point3, q1: Point3) -> (f64, f64, f64) {
    let d1 = p1 - p0;
    let d2 = q1 - q0;
    let r = p0 - q0;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let (s, t) = if a <= 1e-30 && e <= 1e-30 {
        (0.0, 0.0)
    } else if a <= 1e-30 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= 1e-30 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s = if denom.abs() > 1e-30 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    let p = p0 + d1 * s;
    let q = q0 + d2 * t;
    (s, t, p.dist(q))
}

fn polish_candidate(
    a: &NurbsCurve,
    b: &NurbsCurve,
    t_a: f64,
    t_b: f64,
    policy: PolishPolicy,
) -> PolishCandidateOutcome {
    let mut outcome = newton_polish_pair_outcome(a, b, t_a, t_b, policy);
    let (mut t_a, mut t_b) = outcome.parameters();
    let distance = a.eval(t_a).dist(b.eval(t_b));
    let mut fallback_selected = false;
    let mut minimizer_stops = MinimizerStopSet::default();
    if needs_local_refinement(distance, policy.tolerances.linear()) {
        fallback_selected = true;
        let refined = refine_local_pair(
            a,
            b,
            t_a,
            t_b,
            policy.range_a,
            policy.range_b,
            policy.numerical,
        );
        minimizer_stops = refined.stops;
        if a.eval(refined.t_a).dist(b.eval(refined.t_b)) < distance {
            outcome = newton_polish_pair_outcome(a, b, refined.t_a, refined.t_b, policy);
            (t_a, t_b) = outcome.parameters();
        }
    }
    PolishCandidateOutcome {
        t_a,
        t_b,
        stop: outcome.stop,
        fallback_selected,
        minimizer_stops,
    }
}

fn needs_local_refinement(distance: f64, tolerance: f64) -> bool {
    distance > tolerance && distance <= tolerance * 16.0
}

#[cfg(test)]
fn newton_polish_pair(
    a: &NurbsCurve,
    b: &NurbsCurve,
    t_a: f64,
    t_b: f64,
    policy: PolishPolicy,
) -> (f64, f64) {
    newton_polish_pair_outcome(a, b, t_a, t_b, policy).parameters()
}

fn newton_polish_pair_outcome(
    a: &NurbsCurve,
    b: &NurbsCurve,
    mut t_a: f64,
    mut t_b: f64,
    policy: PolishPolicy,
) -> NewtonPolishOutcome {
    for _ in 0..MAX_POLISH_STEPS {
        let da = a.eval_derivs(t_a, 2);
        let db = b.eval_derivs(t_b, 2);
        let r = da.d[0] - db.d[0];
        let g0 = r.dot(da.d[1]);
        let g1 = -r.dot(db.d[1]);
        if directional_gradients_are_numerically_zero(policy.numerical, r, da.d[1], db.d[1]) {
            return NewtonPolishOutcome {
                t_a,
                t_b,
                stop: NewtonPolishStop::GradientStationary,
            };
        }

        let h00 = da.d[1].dot(da.d[1]) + r.dot(da.d[2]);
        let h01 = -da.d[1].dot(db.d[1]);
        let h11 = db.d[1].dot(db.d[1]) - r.dot(db.d[2]);
        let Some((step_a, step_b)) = solve_symmetric_2x2(policy.numerical, h00, h01, h11, -g0, -g1)
        else {
            return NewtonPolishOutcome {
                t_a,
                t_b,
                stop: NewtonPolishStop::IllConditioned,
            };
        };

        let old_residual = r.norm_sq();
        let old_t_a = t_a;
        let old_t_b = t_b;
        let mut scale = 1.0;
        let mut accepted = false;
        for _ in 0..16 {
            let next_a = (t_a + step_a * scale).clamp(policy.range_a.lo, policy.range_a.hi);
            let next_b = (t_b + step_b * scale).clamp(policy.range_b.lo, policy.range_b.hi);
            let next = a.eval(next_a).dist(b.eval(next_b));
            if next * next <= old_residual {
                accepted = true;
                t_a = next_a;
                t_b = next_b;
                break;
            }
            scale *= 0.5;
        }
        if !accepted {
            return NewtonPolishOutcome {
                t_a,
                t_b,
                stop: NewtonPolishStop::NoDescent,
            };
        }
        let stopped_a = parameter_step_has_no_progress(
            t_a - old_t_a,
            policy.range_a,
            policy.tolerances,
            policy.numerical,
        );
        let stopped_b = parameter_step_has_no_progress(
            t_b - old_t_b,
            policy.range_b,
            policy.tolerances,
            policy.numerical,
        );
        if stopped_a && stopped_b {
            return NewtonPolishOutcome {
                t_a,
                t_b,
                stop: NewtonPolishStop::ParameterResolution,
            };
        }
    }
    NewtonPolishOutcome {
        t_a,
        t_b,
        stop: NewtonPolishStop::IterationLimit,
    }
}

fn diagnose_polish_stop(scope: &mut OperationScope<'_, '_>, stop: NewtonPolishStop) {
    let (code, kind, message) = match stop {
        NewtonPolishStop::GradientStationary => (
            NURBS_CURVE_PAIR_POLISH_STATIONARY,
            DiagnosticKind::ProofIncomplete,
            "NURBS curve-pair polish was stationary without a tolerance witness",
        ),
        NewtonPolishStop::IllConditioned => (
            NURBS_CURVE_PAIR_POLISH_ILL_CONDITIONED,
            DiagnosticKind::IllConditioned,
            "NURBS curve-pair polish was too ill-conditioned for a safe Newton step",
        ),
        NewtonPolishStop::NoDescent => (
            NURBS_CURVE_PAIR_POLISH_NO_DESCENT,
            DiagnosticKind::ProofIncomplete,
            "NURBS curve-pair polish found no non-increasing damped step",
        ),
        NewtonPolishStop::ParameterResolution => {
            scope.record_numeric_resolution(NURBS_CURVE_PAIR_SEED_ATTEMPTS);
            (
                NURBS_CURVE_PAIR_POLISH_PARAMETER_RESOLUTION,
                DiagnosticKind::NumericResolution,
                "NURBS curve-pair polish stopped at parameter resolution without a witness",
            )
        }
        NewtonPolishStop::IterationLimit => (
            NURBS_CURVE_PAIR_POLISH_ITERATION_LIMIT,
            DiagnosticKind::ProofIncomplete,
            "NURBS curve-pair polish reached its fixed iteration bound without a witness",
        ),
    };
    scope.diagnose(NURBS_CURVE_PAIR_SEED_ATTEMPTS, code, kind, message);
}

fn diagnose_minimizer_stops(scope: &mut OperationScope<'_, '_>, stops: MinimizerStopSet) {
    for stop in [
        MinimizerStop::ParameterResolution,
        MinimizerStop::InvalidObjective,
        MinimizerStop::IterationLimit,
    ] {
        if !stops.contains(stop) {
            continue;
        }
        let (code, kind, message) = match stop {
            MinimizerStop::ParameterResolution => {
                scope.record_numeric_resolution(NURBS_CURVE_PAIR_SEED_ATTEMPTS);
                (
                    NURBS_CURVE_PAIR_MINIMIZER_PARAMETER_RESOLUTION,
                    DiagnosticKind::NumericResolution,
                    "NURBS curve-pair fallback minimization reached parameter resolution",
                )
            }
            MinimizerStop::InvalidObjective => (
                NURBS_CURVE_PAIR_MINIMIZER_INVALID_OBJECTIVE,
                DiagnosticKind::ProofIncomplete,
                "NURBS curve-pair fallback minimization observed an invalid objective",
            ),
            MinimizerStop::IterationLimit => (
                NURBS_CURVE_PAIR_MINIMIZER_ITERATION_LIMIT,
                DiagnosticKind::ProofIncomplete,
                "NURBS curve-pair fallback minimization reached its fixed iteration bound",
            ),
        };
        scope.diagnose(NURBS_CURVE_PAIR_SEED_ATTEMPTS, code, kind, message);
    }
}

fn refine_local_pair(
    a: &NurbsCurve,
    b: &NurbsCurve,
    t_a: f64,
    t_b: f64,
    range_a: ParamRange,
    range_b: ParamRange,
    numerical: NumericalPolicy,
) -> LocalRefinementOutcome {
    let width_a = range_a.width() / MIN_STEPS as f64 * 2.0;
    let width_b = range_b.width() / MIN_STEPS as f64 * 2.0;

    let a0 = minimize_curve_to_curve_distance(
        a,
        b,
        ParamRange::new(
            (t_a - width_a).max(range_a.lo),
            (t_a + width_a).min(range_a.hi),
        ),
        range_b,
        numerical,
    );
    let b0 = closest_parameter_to_point_outcome(b, range_b, a.eval(a0.parameter), numerical);

    let b1 = minimize_curve_to_curve_distance(
        b,
        a,
        ParamRange::new(
            (t_b - width_b).max(range_b.lo),
            (t_b + width_b).min(range_b.hi),
        ),
        range_a,
        numerical,
    );
    let a1 = closest_parameter_to_point_outcome(a, range_a, b.eval(b1.parameter), numerical);

    let mut stops = a0.stops;
    stops.insert(b0.stop);
    stops.extend(b1.stops);
    stops.insert(a1.stop);
    if a.eval(a0.parameter).dist(b.eval(b0.parameter))
        <= a.eval(a1.parameter).dist(b.eval(b1.parameter))
    {
        LocalRefinementOutcome {
            t_a: a0.parameter,
            t_b: b0.parameter,
            stops,
        }
    } else {
        LocalRefinementOutcome {
            t_a: a1.parameter,
            t_b: b1.parameter,
            stops,
        }
    }
}

fn minimize_curve_to_curve_distance(
    curve: &NurbsCurve,
    other: &NurbsCurve,
    mut range: ParamRange,
    other_range: ParamRange,
    numerical: NumericalPolicy,
) -> NestedMinimizeOutcome {
    let original_span = range.width();
    let mut stops = MinimizerStopSet::default();
    let mut outer_stop = MinimizerStop::IterationLimit;
    for _ in 0..MAX_MINIMIZE_STEPS {
        let third = range.width() / 3.0;
        let left = range.lo + third;
        let right = range.hi - third;
        if ternary_interval_has_no_progress(
            numerical,
            original_span,
            range.lo,
            left,
            right,
            range.hi,
        ) {
            outer_stop = MinimizerStop::ParameterResolution;
            break;
        }
        let f_left =
            distance_from_point_to_curve_outcome(curve.eval(left), other, other_range, numerical);
        let f_right =
            distance_from_point_to_curve_outcome(curve.eval(right), other, other_range, numerical);
        stops.extend(f_left.stops);
        stops.extend(f_right.stops);
        let Some(equal) =
            nonnegative_values_are_numerically_equal(numerical, f_left.distance, f_right.distance)
        else {
            outer_stop = MinimizerStop::InvalidObjective;
            break;
        };
        if equal {
            range = ParamRange::new(left, right);
        } else if f_left.distance < f_right.distance {
            range = ParamRange::new(range.lo, right);
        } else {
            range = ParamRange::new(left, range.hi);
        }
    }
    stops.insert(outer_stop);
    NestedMinimizeOutcome {
        parameter: range.lerp(0.5),
        stops,
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct DistanceMinimizeOutcome {
    distance: f64,
    stops: MinimizerStopSet,
}

fn distance_from_point_to_curve_outcome(
    point: Point3,
    curve: &NurbsCurve,
    range: ParamRange,
    numerical: NumericalPolicy,
) -> DistanceMinimizeOutcome {
    let outcome = closest_parameter_to_point_outcome(curve, range, point, numerical);
    DistanceMinimizeOutcome {
        distance: point.dist(curve.eval(outcome.parameter)),
        stops: MinimizerStopSet::one(outcome.stop),
    }
}

fn push_root_candidate(
    a: &NurbsCurve,
    t_a: f64,
    b: &NurbsCurve,
    t_b: f64,
    points: &mut Vec<CurveCurvePoint>,
    tolerances: Tolerances,
) {
    if a.eval(t_a).dist(b.eval(t_b)) > tolerances.linear() {
        return;
    }
    let Some(point) = accept_curve_curve_candidate(
        a,
        t_a,
        b,
        t_b,
        contact_kind(a, t_a, b, t_b, tolerances),
        tolerances,
    ) else {
        return;
    };
    push_distinct_point(points, point, tolerances);
}

fn contact_kind(
    a: &NurbsCurve,
    t_a: f64,
    b: &NurbsCurve,
    t_b: f64,
    tolerances: Tolerances,
) -> ContactKind {
    let da = a.eval_derivs(t_a, 1).d[1];
    let db = b.eval_derivs(t_b, 1).d[1];
    match normalized_cross_magnitude(da, db) {
        None => ContactKind::Singular,
        Some(sine) if sine > working_angular_tolerance(tolerances) => ContactKind::Transverse,
        Some(_) => ContactKind::Tangent,
    }
}

fn push_distinct_point(
    points: &mut Vec<CurveCurvePoint>,
    candidate: CurveCurvePoint,
    tolerances: Tolerances,
) {
    if let Some(point) = points
        .iter_mut()
        .find(|point| duplicate_point(point, &candidate, tolerances))
    {
        if better_representative(&candidate, point, tolerances) {
            *point = candidate;
        }
    } else {
        points.push(candidate);
    }
}

fn duplicate_point(
    point: &CurveCurvePoint,
    candidate: &CurveCurvePoint,
    tolerances: Tolerances,
) -> bool {
    let spatial_tol =
        if point.kind == ContactKind::Tangent || candidate.kind == ContactKind::Tangent {
            tolerances.linear().sqrt()
        } else {
            tolerances.linear()
        };
    point.point.dist(candidate.point) <= spatial_tol
        || (point.t_a - candidate.t_a).abs() <= working_angular_tolerance(tolerances)
            && (point.t_b - candidate.t_b).abs() <= working_angular_tolerance(tolerances)
}

fn better_representative(
    candidate: &CurveCurvePoint,
    point: &CurveCurvePoint,
    tolerances: Tolerances,
) -> bool {
    candidate.residual + tolerances.linear() * 1e-6 < point.residual
        || candidate.kind > point.kind && candidate.residual <= point.residual + tolerances.linear()
}

fn working_angular_tolerance(tolerances: Tolerances) -> f64 {
    tolerances.angular().max(tolerances.linear().sqrt())
}

fn closest_parameter_to_point(
    curve: &NurbsCurve,
    range: ParamRange,
    point: Point3,
    numerical: NumericalPolicy,
) -> f64 {
    closest_parameter_to_point_outcome(curve, range, point, numerical).parameter
}

fn closest_parameter_to_point_outcome(
    curve: &NurbsCurve,
    range: ParamRange,
    point: Point3,
    numerical: NumericalPolicy,
) -> MinimizeOutcome {
    let samples = sample_curve(curve, range);
    let (best_idx, _) = samples
        .iter()
        .enumerate()
        .map(|(i, sample)| (i, sample.point.dist(point)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .expect("sample_curve always returns at least one sample");
    let lo = samples[best_idx.saturating_sub(1)].t;
    let hi = samples[(best_idx + 1).min(samples.len() - 1)].t;
    minimize_point_distance_outcome(curve, lo, hi, point, numerical)
}

fn minimize_point_distance_outcome(
    curve: &NurbsCurve,
    mut lo: f64,
    mut hi: f64,
    point: Point3,
    numerical: NumericalPolicy,
) -> MinimizeOutcome {
    let original_span = hi - lo;
    let mut stop = MinimizerStop::IterationLimit;
    for _ in 0..MAX_MINIMIZE_STEPS {
        let third = (hi - lo) / 3.0;
        let left = lo + third;
        let right = hi - third;
        if ternary_interval_has_no_progress(numerical, original_span, lo, left, right, hi) {
            stop = MinimizerStop::ParameterResolution;
            break;
        }
        let f_left = curve.eval(left).dist(point);
        let f_right = curve.eval(right).dist(point);
        let Some(equal) = nonnegative_values_are_numerically_equal(numerical, f_left, f_right)
        else {
            stop = MinimizerStop::InvalidObjective;
            break;
        };
        if equal {
            lo = left;
            hi = right;
        } else if f_left < f_right {
            hi = right;
        } else {
            lo = left;
        }
    }
    MinimizeOutcome {
        parameter: lo + (hi - lo) * 0.5,
        stop,
    }
}

fn clamp_to_domain(range: ParamRange, domain: ParamRange) -> ParamRange {
    ParamRange::new(
        range.lo.clamp(domain.lo, domain.hi),
        range.hi.clamp(domain.lo, domain.hi),
    )
}

fn snap_to_range_bounds(t: f64, range: ParamRange, tolerance: f64) -> f64 {
    if (t - range.lo).abs() <= tolerance {
        range.lo
    } else if (t - range.hi).abs() <= tolerance {
        range.hi
    } else {
        t.clamp(range.lo, range.hi)
    }
}

fn range_has_no_parameter_progress(
    range: ParamRange,
    tolerances: Tolerances,
    numerical: NumericalPolicy,
) -> bool {
    let span = range.width();
    if !span.is_finite() || span <= 0.0 {
        return true;
    }
    let midpoint = range.lo + span * 0.5;
    if !(midpoint.is_finite() && range.lo < midpoint && midpoint < range.hi) {
        return true;
    }
    parameter_progress_step(numerical, 1.0, 1.0, tolerances.linear()).is_none_or(|step| 1.0 <= step)
}

fn parameter_step_has_no_progress(
    step: f64,
    range: ParamRange,
    tolerances: Tolerances,
    numerical: NumericalPolicy,
) -> bool {
    let span = range.width();
    if !step.is_finite() || !span.is_finite() || span <= 0.0 {
        return true;
    }
    let normalized_step = step.abs() / span;
    parameter_progress_step(numerical, 1.0, 1.0, tolerances.linear())
        .is_none_or(|threshold| !normalized_step.is_finite() || normalized_step <= threshold)
}

/// Legacy parameter slack retained for overlap and input semantics. It is
/// deliberately not represented as a numerical-policy guard: migrating these
/// uses requires a separate proof-compatibility review.
fn legacy_parameter_slack(range: ParamRange, tolerances: Tolerances) -> f64 {
    (range.width().abs() * 1e-10)
        .max(tolerances.angular())
        .max(1e-12)
}

fn validate_ranges(
    a: &NurbsCurve,
    range_a: ParamRange,
    b: &NurbsCurve,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<()> {
    let width_a = range_a.width();
    let width_b = range_b.width();
    if !range_a.is_finite()
        || !range_b.is_finite()
        || !width_a.is_finite()
        || !width_b.is_finite()
        || width_a < 0.0
        || width_b < 0.0
    {
        return Err(Error::InvalidGeometry {
            reason: "nurbs/nurbs intersection requires finite non-reversed ranges",
        });
    }
    if !a.knots().is_clamped() || !b.knots().is_clamped() {
        return Err(Error::InvalidGeometry {
            reason: "nurbs/nurbs intersection requires clamped NURBS curves",
        });
    }
    let domain_a = a.param_range();
    let domain_b = b.param_range();
    let parameter_tol_a = legacy_parameter_slack(domain_a, tolerances);
    let parameter_tol_b = legacy_parameter_slack(domain_b, tolerances);
    if range_a.lo < domain_a.lo - parameter_tol_a
        || range_a.hi > domain_a.hi + parameter_tol_a
        || range_b.lo < domain_b.lo - parameter_tol_b
        || range_b.hi > domain_b.hi + parameter_tol_b
    {
        return Err(Error::InvalidGeometry {
            reason: "nurbs/nurbs intersection ranges must lie within the NURBS domains",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use kcore::operation::DiagnosticLevel;

    use super::*;

    fn line_with_domain(start: Point3, end: Point3, hi: f64) -> NurbsCurve {
        NurbsCurve::new(1, vec![0.0, 0.0, hi, hi], vec![start, end], None).unwrap()
    }

    fn tangent_parabola_with_domain(hi: f64) -> NurbsCurve {
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, hi, hi, hi],
            vec![
                Point3::new(-1.0, 1.0, 0.0),
                Point3::new(0.0, -1.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            None,
        )
        .unwrap()
    }

    #[test]
    fn solve_profile_matches_the_isolation_cover_ceiling() {
        let profile = NurbsCurvePairSolveBudgetProfile::v1_defaults();
        NurbsCurvePairSolveBudgetProfile::validate(&profile).unwrap();
        let seeds = profile
            .limits()
            .iter()
            .find(|limit| limit.stage == NURBS_CURVE_PAIR_SEED_ATTEMPTS)
            .unwrap();
        let cells = profile
            .limits()
            .iter()
            .find(|limit| limit.stage == kgeom::nurbs::NURBS_CURVE_PAIR_CANDIDATES)
            .unwrap();
        assert_eq!(seeds.resource, ResourceKind::Work);
        assert_eq!(seeds.mode, AccountingMode::Cumulative);
        assert_eq!(seeds.allowed, cells.allowed);
        assert_eq!(profile.limits().len(), 4);
    }

    #[test]
    fn accepted_witnesses_skip_fallback_refinement_at_the_exact_boundary() {
        let tolerance = Tolerances::default().linear();
        assert!(!needs_local_refinement(0.0, tolerance));
        assert!(!needs_local_refinement(tolerance, tolerance));
        assert!(needs_local_refinement(tolerance * 2.0, tolerance));
        assert!(needs_local_refinement(tolerance * 16.0, tolerance));
        assert!(!needs_local_refinement(
            tolerance * 16.0 + tolerance,
            tolerance
        ));
    }

    #[test]
    fn polish_diagnostics_are_unique_typed_and_bounded_by_context() {
        let unique = NURBS_CURVE_PAIR_POLISH_DIAGNOSTICS
            .iter()
            .map(|code| code.as_str())
            .collect::<BTreeSet<_>>();
        assert_eq!(unique.len(), NURBS_CURVE_PAIR_POLISH_DIAGNOSTICS.len());
        assert!(
            unique
                .iter()
                .all(|code| code.starts_with("kops.intersect.nurbs-curve-pair-polish-"))
        );

        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_diagnostics(DiagnosticLevel::Summary, 5);
        let mut scope = OperationScope::new(&context);
        for stop in [
            NewtonPolishStop::GradientStationary,
            NewtonPolishStop::IllConditioned,
            NewtonPolishStop::NoDescent,
            NewtonPolishStop::ParameterResolution,
            NewtonPolishStop::IterationLimit,
        ] {
            diagnose_polish_stop(&mut scope, stop);
        }
        let outcome = scope.finish(Ok(()));
        let diagnostics = outcome.report().diagnostics();
        assert_eq!(diagnostics.len(), 5);
        assert_eq!(diagnostics[0].code, NURBS_CURVE_PAIR_POLISH_STATIONARY);
        assert_eq!(diagnostics[0].kind, DiagnosticKind::ProofIncomplete);
        assert_eq!(diagnostics[1].code, NURBS_CURVE_PAIR_POLISH_ILL_CONDITIONED);
        assert_eq!(diagnostics[1].kind, DiagnosticKind::IllConditioned);
        assert_eq!(diagnostics[2].code, NURBS_CURVE_PAIR_POLISH_NO_DESCENT);
        assert_eq!(diagnostics[2].kind, DiagnosticKind::ProofIncomplete);
        assert_eq!(
            diagnostics[3].code,
            NURBS_CURVE_PAIR_POLISH_PARAMETER_RESOLUTION
        );
        assert_eq!(diagnostics[3].kind, DiagnosticKind::NumericResolution);
        assert_eq!(diagnostics[4].code, NURBS_CURVE_PAIR_POLISH_ITERATION_LIMIT);
        assert_eq!(diagnostics[4].kind, DiagnosticKind::ProofIncomplete);
        assert_eq!(
            outcome.report().numeric_resolution_stages(),
            &[NURBS_CURVE_PAIR_SEED_ATTEMPTS]
        );
        assert_eq!(outcome.report().dropped_diagnostics(), 0);
    }

    #[test]
    fn fallback_minimizer_stops_are_exact_typed_and_reportable() {
        let unique = NURBS_CURVE_PAIR_MINIMIZER_DIAGNOSTICS
            .iter()
            .map(|code| code.as_str())
            .collect::<BTreeSet<_>>();
        assert_eq!(unique.len(), NURBS_CURVE_PAIR_MINIMIZER_DIAGNOSTICS.len());
        assert!(
            unique
                .iter()
                .all(|code| code.starts_with("kops.intersect.nurbs-curve-pair-minimizer-"))
        );

        let line = line_with_domain(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), 1.0);
        let resolved = minimize_point_distance_outcome(
            &line,
            0.0,
            1.0,
            Point3::new(0.0, 1.0, 0.0),
            NumericalPolicy::v1(),
        );
        assert_eq!(resolved.stop, MinimizerStop::ParameterResolution);
        assert!((resolved.parameter - 0.5).abs() <= 64.0 * f64::EPSILON);

        let invalid = minimize_point_distance_outcome(
            &line,
            0.0,
            1.0,
            Point3::new(f64::NAN, 0.0, 0.0),
            NumericalPolicy::v1(),
        );
        assert_eq!(invalid.stop, MinimizerStop::InvalidObjective);

        let tiny_progress = NumericalPolicy::try_new(1.0e-300, 1.0e-300, f64::EPSILON).unwrap();
        let limited = minimize_point_distance_outcome(
            &line,
            0.0,
            1.0,
            Point3::new(-10.0, 1.0, 0.0),
            tiny_progress,
        );
        assert_eq!(limited.stop, MinimizerStop::IterationLimit);

        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_diagnostics(DiagnosticLevel::Summary, 3);
        let mut scope = OperationScope::new(&context);
        let mut stops = MinimizerStopSet::default();
        stops.insert(MinimizerStop::ParameterResolution);
        stops.insert(MinimizerStop::InvalidObjective);
        stops.insert(MinimizerStop::IterationLimit);
        diagnose_minimizer_stops(&mut scope, stops);
        let outcome = scope.finish(Ok(()));
        let diagnostics = outcome.report().diagnostics();
        assert_eq!(diagnostics.len(), 3);
        assert_eq!(
            diagnostics[0].code,
            NURBS_CURVE_PAIR_MINIMIZER_PARAMETER_RESOLUTION
        );
        assert_eq!(diagnostics[0].kind, DiagnosticKind::NumericResolution);
        assert_eq!(
            diagnostics[1].code,
            NURBS_CURVE_PAIR_MINIMIZER_INVALID_OBJECTIVE
        );
        assert_eq!(diagnostics[1].kind, DiagnosticKind::ProofIncomplete);
        assert_eq!(
            diagnostics[2].code,
            NURBS_CURVE_PAIR_MINIMIZER_ITERATION_LIMIT
        );
        assert_eq!(diagnostics[2].kind, DiagnosticKind::ProofIncomplete);
        assert_eq!(
            outcome.report().numeric_resolution_stages(),
            &[NURBS_CURVE_PAIR_SEED_ATTEMPTS]
        );
        assert_eq!(outcome.report().dropped_diagnostics(), 0);

        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&context);
        diagnose_minimizer_stops(
            &mut scope,
            MinimizerStopSet::one(MinimizerStop::ParameterResolution),
        );
        let diagnostics_off = scope.finish(Ok(()));
        assert!(diagnostics_off.report().diagnostics().is_empty());
        assert_eq!(
            diagnostics_off.report().numeric_resolution_stages(),
            &[NURBS_CURVE_PAIR_SEED_ATTEMPTS]
        );
    }

    #[test]
    fn curve_pair_proof_diagnostics_are_unique_and_namespaced() {
        let unique = NURBS_CURVE_PAIR_PROOF_DIAGNOSTICS
            .iter()
            .map(|code| code.as_str())
            .collect::<BTreeSet<_>>();
        assert_eq!(unique.len(), NURBS_CURVE_PAIR_PROOF_DIAGNOSTICS.len());
        assert!(
            unique
                .iter()
                .all(|code| code.starts_with("kops.intersect.nurbs-curve-pair-"))
        );
        assert_eq!(
            NURBS_CURVE_PAIR_COMPLETE_COVERAGE.as_str(),
            "kops.intersect.nurbs-curve-pair-complete-coverage"
        );
    }

    #[test]
    fn missing_seed_stage_is_rejected_before_a_separated_early_exit() {
        let first = line_with_domain(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), 1.0);
        let second = line_with_domain(
            Point3::new(-1.0, 10.0, 0.0),
            Point3::new(1.0, 10.0, 0.0),
            1.0,
        );
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(NurbsCurvePairBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        let error = intersect_bounded_nurbs_nurbs_contextual_impl(
            &first,
            first.param_range(),
            &second,
            second.param_range(),
            &mut scope,
        )
        .unwrap_err();
        assert_eq!(
            error,
            Error::OperationPolicy {
                source: OperationPolicyError::UnknownLimit {
                    stage: NURBS_CURVE_PAIR_SEED_ATTEMPTS,
                    resource: ResourceKind::Work,
                },
            }
        );
    }

    #[test]
    fn newton_conditioning_is_invariant_under_large_parameter_rescaling() {
        let parameter_scale = 1.0e8;
        let horizontal = line_with_domain(
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            parameter_scale,
        );
        let vertical = line_with_domain(
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            parameter_scale,
        );
        let range = ParamRange::new(0.0, parameter_scale);
        let start_a = 0.4 * parameter_scale;
        let start_b = 0.6 * parameter_scale;
        let da = horizontal.eval_derivs(start_a, 1).d[1];
        let db = vertical.eval_derivs(start_b, 1).d[1];
        let old_absolute_determinant = da.dot(da) * db.dot(db) - da.dot(db) * da.dot(db);
        assert!(old_absolute_determinant.abs() < 1.0e-24);

        let (polished_a, polished_b) = newton_polish_pair(
            &horizontal,
            &vertical,
            start_a,
            start_b,
            PolishPolicy {
                range_a: range,
                range_b: range,
                tolerances: Tolerances::default(),
                numerical: NumericalPolicy::v1(),
            },
        );
        assert!((polished_a / parameter_scale - 0.5).abs() <= f64::EPSILON);
        assert!((polished_b / parameter_scale - 0.5).abs() <= f64::EPSILON);
        assert!(horizontal.eval(polished_a).dist(vertical.eval(polished_b)) <= f64::EPSILON);
    }

    #[test]
    fn newton_polish_honors_the_supplied_numerical_policy() {
        let horizontal =
            line_with_domain(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), 1.0);
        let shallow = line_with_domain(
            Point3::new(-1.0, -0.2, 0.0),
            Point3::new(1.0, 0.2, 0.0),
            1.0,
        );
        let range = ParamRange::new(0.0, 1.0);
        let policy = |numerical| PolishPolicy {
            range_a: range,
            range_b: range,
            tolerances: Tolerances::default(),
            numerical,
        };

        let v1 = newton_polish_pair(
            &horizontal,
            &shallow,
            0.4,
            0.6,
            policy(NumericalPolicy::v1()),
        );
        assert!((v1.0 - 0.5).abs() <= 4.0 * f64::EPSILON);
        assert!((v1.1 - 0.5).abs() <= 4.0 * f64::EPSILON);

        let strict = NumericalPolicy::try_new(32.0, 64.0, 0.5).unwrap();
        let stopped = newton_polish_pair_outcome(&horizontal, &shallow, 0.4, 0.6, policy(strict));
        assert_eq!(stopped.parameters(), (0.4, 0.6));
        assert_eq!(stopped.stop, NewtonPolishStop::IllConditioned);
    }

    #[test]
    fn newton_progress_stop_honors_the_supplied_numerical_policy() {
        let parabola = tangent_parabola_with_domain(1.0);
        let horizontal =
            line_with_domain(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), 1.0);
        let range = ParamRange::new(0.0, 1.0);
        let policy = |numerical| PolishPolicy {
            range_a: range,
            range_b: range,
            tolerances: Tolerances::default(),
            numerical,
        };

        let v1 = newton_polish_pair(
            &parabola,
            &horizontal,
            0.75,
            0.75,
            policy(NumericalPolicy::v1()),
        );
        let v1_residual = parabola.eval(v1.0).dist(horizontal.eval(v1.1));
        assert!(
            v1_residual <= Tolerances::default().linear(),
            "{v1:?}: {v1_residual}"
        );

        let coarse_progress = NumericalPolicy::try_new(32.0, 1.0e15, 128.0 * f64::EPSILON).unwrap();
        let stopped =
            newton_polish_pair_outcome(&parabola, &horizontal, 0.75, 0.75, policy(coarse_progress));
        assert!(
            parabola
                .eval(stopped.t_a)
                .dist(horizontal.eval(stopped.t_b))
                > 1.0e-4
        );
        assert_eq!(stopped.stop, NewtonPolishStop::ParameterResolution);

        let mut accepted = Vec::new();
        push_root_candidate(
            &parabola,
            stopped.t_a,
            &horizontal,
            stopped.t_b,
            &mut accepted,
            Tolerances::default(),
        );
        assert!(accepted.is_empty());
    }

    #[test]
    fn clamped_newton_progress_uses_the_actual_accepted_displacement() {
        let range = ParamRange::new(0.0, 1.0);
        let old = range.hi;
        let proposed_step = 0.25;
        let accepted = (old + proposed_step).clamp(range.lo, range.hi);
        assert_eq!(accepted, old);
        assert!(parameter_step_has_no_progress(
            accepted - old,
            range,
            Tolerances::default(),
            NumericalPolicy::v1(),
        ));
        assert!(!parameter_step_has_no_progress(
            proposed_step,
            range,
            Tolerances::default(),
            NumericalPolicy::v1(),
        ));
    }

    #[test]
    fn newton_normalized_gradient_stop_honors_policy_without_accepting_contact() {
        let parabola = tangent_parabola_with_domain(1.0);
        let horizontal =
            line_with_domain(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), 1.0);
        let range = ParamRange::new(0.0, 1.0);
        let policy = |numerical| PolishPolicy {
            range_a: range,
            range_b: range,
            tolerances: Tolerances::default(),
            numerical,
        };

        let v1 = newton_polish_pair(
            &parabola,
            &horizontal,
            0.75,
            0.75,
            policy(NumericalPolicy::v1()),
        );
        assert!(parabola.eval(v1.0).dist(horizontal.eval(v1.1)) <= Tolerances::default().linear());

        let coarse_rounding = NumericalPolicy::try_new(1.0e16, 64.0, 128.0 * f64::EPSILON).unwrap();
        let stopped =
            newton_polish_pair_outcome(&parabola, &horizontal, 0.75, 0.75, policy(coarse_rounding));
        assert_eq!(stopped.parameters(), (0.75, 0.75));
        assert_eq!(stopped.stop, NewtonPolishStop::GradientStationary);

        let mut accepted = Vec::new();
        push_root_candidate(
            &parabola,
            stopped.t_a,
            &horizontal,
            stopped.t_b,
            &mut accepted,
            Tolerances::default(),
        );
        assert!(accepted.is_empty());
    }

    #[test]
    fn collapsed_second_range_routes_to_the_first_curve_symmetrically() {
        let horizontal =
            line_with_domain(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), 1.0);
        let vertical =
            line_with_domain(Point3::new(0.0, -1.0, 0.0), Point3::new(0.0, 1.0, 0.0), 1.0);
        let full = ParamRange::new(0.0, 1.0);
        let point = ParamRange::new(0.5, 0.5);
        let tolerances = Tolerances::default();

        let forward = degenerate_range_intersections(
            &horizontal,
            full,
            false,
            &vertical,
            point,
            tolerances,
            NumericalPolicy::v1(),
        )
        .unwrap();
        let swapped = degenerate_range_intersections(
            &vertical,
            point,
            true,
            &horizontal,
            full,
            tolerances,
            NumericalPolicy::v1(),
        )
        .unwrap();

        assert_eq!(forward.points.len(), 1);
        assert_eq!(swapped.points.len(), 1);
        assert_eq!(forward.points[0].point, swapped.points[0].point);
        assert_eq!(forward.points[0].t_a, swapped.points[0].t_b);
        assert_eq!(forward.points[0].t_b, swapped.points[0].t_a);
    }

    #[test]
    fn collapsed_range_decision_is_scale_invariant_and_detects_affine_offset_limits() {
        let tolerances = Tolerances::default();
        let numerical = NumericalPolicy::v1();
        for scale in [1.0e-200, 1.0, 1.0e200] {
            assert!(!range_has_no_parameter_progress(
                ParamRange::new(0.0, scale),
                tolerances,
                numerical,
            ));
        }
        assert!(!range_has_no_parameter_progress(
            ParamRange::new(1.0e16, 1.0e16 + 4.0),
            tolerances,
            numerical,
        ));
        assert!(range_has_no_parameter_progress(
            ParamRange::new(1.0e16, 1.0e16 + 2.0),
            tolerances,
            numerical,
        ));

        let coarse = NumericalPolicy::try_new(32.0, 1.0e16, 128.0 * f64::EPSILON).unwrap();
        assert!(range_has_no_parameter_progress(
            ParamRange::new(0.0, 1.0),
            tolerances,
            coarse,
        ));
    }

    #[test]
    fn contact_classification_is_directly_invariant_across_parameter_scales() {
        let tolerances = Tolerances::default();
        for parameter_scale in [1.0e-13, 1.0, 1.0e13] {
            let tangent = tangent_parabola_with_domain(parameter_scale);
            let horizontal = line_with_domain(
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                parameter_scale,
            );
            let vertical = line_with_domain(
                Point3::new(0.0, -1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                parameter_scale,
            );
            let t = parameter_scale * 0.5;
            assert_eq!(
                contact_kind(&tangent, t, &horizontal, t, tolerances),
                ContactKind::Tangent,
                "parameter scale {parameter_scale:e}",
            );
            assert_eq!(
                contact_kind(&horizontal, t, &vertical, t, tolerances),
                ContactKind::Transverse,
                "parameter scale {parameter_scale:e}",
            );
        }
    }

    #[test]
    fn zero_derivative_contact_is_singular() {
        let stationary = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            None,
        )
        .unwrap();
        let vertical =
            line_with_domain(Point3::new(0.0, -1.0, 0.0), Point3::new(0.0, 1.0, 0.0), 1.0);
        assert_eq!(
            contact_kind(&stationary, 0.0, &vertical, 0.5, Tolerances::default()),
            ContactKind::Singular,
        );
    }

    #[test]
    fn finite_ranges_with_overflowing_width_are_rejected() {
        let a = line_with_domain(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), 1.0);
        let overflowing = ParamRange {
            lo: -f64::MAX,
            hi: f64::MAX,
        };
        assert_eq!(
            validate_ranges(&a, overflowing, &a, a.param_range(), Tolerances::default(),),
            Err(Error::InvalidGeometry {
                reason: "nurbs/nurbs intersection requires finite non-reversed ranges",
            }),
        );
    }

    #[test]
    fn conditioning_stop_cannot_accept_a_model_residual() {
        let parameter_scale = 1.0e8;
        let a = line_with_domain(
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            parameter_scale,
        );
        let b = line_with_domain(
            Point3::new(-1.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            parameter_scale,
        );
        let range = ParamRange::new(0.0, parameter_scale);
        let start_a = 0.4 * parameter_scale;
        let start_b = 0.6 * parameter_scale;
        let (stopped_a, stopped_b) = newton_polish_pair(
            &a,
            &b,
            start_a,
            start_b,
            PolishPolicy {
                range_a: range,
                range_b: range,
                tolerances: Tolerances::default(),
                numerical: NumericalPolicy::v1(),
            },
        );
        let mut accepted = Vec::new();
        push_root_candidate(
            &a,
            stopped_a,
            &b,
            stopped_b,
            &mut accepted,
            Tolerances::default(),
        );
        assert!(accepted.is_empty());
    }
}
