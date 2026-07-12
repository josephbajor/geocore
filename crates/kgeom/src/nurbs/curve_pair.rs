//! Exact adaptive control-hull isolation for NURBS curve pairs.

use super::NurbsCurve;
use crate::aabb::Aabb3;
use crate::curve::Curve;
use crate::param::ParamRange;
use kcore::error::{Error, Result};
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSnapshot, LimitSpec, OperationPolicyError, OperationScope,
    ResourceKind, StageId,
};

const DEFAULT_DEPTH: u32 = 6;
const DEFAULT_CANDIDATES: u64 = 4_096;
const DEFAULT_SUBDIVISIONS: u64 = 1_366;

const fn stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid NURBS curve-pair stage"),
    }
}

/// Cumulative exact curve-pair setup and subdivision attempts.
pub const NURBS_CURVE_PAIR_SUBDIVISIONS: StageId = stage("kgeom.nurbs.curve-pair-subdivisions");
/// High-water retained conservative curve-pair candidate cells.
pub const NURBS_CURVE_PAIR_CANDIDATES: StageId = stage("kgeom.nurbs.curve-pair-candidates");
/// High-water exact binary subdivision depth per curve in a pair cell.
pub const NURBS_CURVE_PAIR_DEPTH: StageId = stage("kgeom.nurbs.curve-pair-depth");

/// Version-1 bounded profile for exact NURBS curve-pair isolation.
#[derive(Debug, Clone, Copy, Default)]
pub struct NurbsCurvePairBudgetProfile;

impl NurbsCurvePairBudgetProfile {
    /// Exact ceilings for one root pair through six four-way rounds: at most
    /// 4,096 retained cells and 1,366 setup/subdivision charges.
    pub fn v1_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                NURBS_CURVE_PAIR_SUBDIVISIONS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                DEFAULT_SUBDIVISIONS,
            ),
            LimitSpec::new(
                NURBS_CURVE_PAIR_CANDIDATES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                DEFAULT_CANDIDATES,
            ),
            LimitSpec::new(
                NURBS_CURVE_PAIR_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                u64::from(DEFAULT_DEPTH),
            ),
        ])
        .expect("built-in curve-pair isolation profile is valid")
    }

    /// Exact default subdivision depth.
    pub const fn default_depth() -> u32 {
        DEFAULT_DEPTH
    }

    /// Require all curve-pair stages with their canonical accounting modes.
    pub fn validate(plan: &BudgetPlan) -> core::result::Result<(), OperationPolicyError> {
        plan.require_limit(
            NURBS_CURVE_PAIR_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        plan.require_limit(
            NURBS_CURVE_PAIR_CANDIDATES,
            ResourceKind::Items,
            AccountingMode::HighWater,
        )?;
        plan.require_limit(
            NURBS_CURVE_PAIR_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
        )?;
        Ok(())
    }
}

/// One exact subcurve pair whose tolerance-inflated control hulls overlap.
#[derive(Debug, Clone, PartialEq)]
pub struct CurvePairCandidateCell {
    first: NurbsCurve,
    second: NurbsCurve,
    first_bounds: Aabb3,
    second_bounds: Aabb3,
    depth: u32,
}

impl CurvePairCandidateCell {
    /// Exact first subcurve.
    pub const fn first_curve(&self) -> &NurbsCurve {
        &self.first
    }

    /// Exact second subcurve.
    pub const fn second_curve(&self) -> &NurbsCurve {
        &self.second
    }

    /// First source parameter interval.
    pub fn first_range(&self) -> ParamRange {
        self.first.param_range()
    }

    /// Second source parameter interval.
    pub fn second_range(&self) -> ParamRange {
        self.second.param_range()
    }

    /// Conservative first control-hull box.
    pub const fn first_bounds(&self) -> Aabb3 {
        self.first_bounds
    }

    /// Conservative second control-hull box.
    pub const fn second_bounds(&self) -> Aabb3 {
        self.second_bounds
    }

    /// Number of exact pair-subdivision rounds from the requested root pair.
    pub const fn depth(&self) -> u32 {
        self.depth
    }
}

/// Structured reasons a conservative pair cover stopped early.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CurvePairIsolationLimits {
    subdivision_work: Option<LimitSnapshot>,
    candidate_cells: Option<LimitSnapshot>,
    subdivision_depth: Option<LimitSnapshot>,
    parameter_resolution: bool,
    subdivision_unavailable: bool,
}

impl CurvePairIsolationLimits {
    /// Exact subdivision-work crossing, if reached.
    pub const fn subdivision_work(self) -> Option<LimitSnapshot> {
        self.subdivision_work
    }

    /// Exact retained-candidate crossing, if reached.
    pub const fn candidate_cells(self) -> Option<LimitSnapshot> {
        self.candidate_cells
    }

    /// Exact subdivision-depth crossing, if reached.
    pub const fn subdivision_depth(self) -> Option<LimitSnapshot> {
        self.subdivision_depth
    }

    /// Whether a mathematical midpoint rounded to an existing endpoint.
    pub const fn parameter_resolution(self) -> bool {
        self.parameter_resolution
    }

    /// Whether a valid degree-zero curve prevented binary subdivision.
    pub const fn subdivision_unavailable(self) -> bool {
        self.subdivision_unavailable
    }

    /// True when no configured, numeric, or method stop occurred.
    pub const fn is_empty(self) -> bool {
        self.subdivision_work.is_none()
            && self.candidate_cells.is_none()
            && self.subdivision_depth.is_none()
            && !self.parameter_resolution
            && !self.subdivision_unavailable
    }
}

/// Conservative cover of every possible tolerance-level NURBS curve contact.
#[derive(Debug, Clone, PartialEq)]
pub struct CurvePairIsolation {
    candidates: Vec<CurvePairCandidateCell>,
    requested_depth: u32,
    limits: CurvePairIsolationLimits,
}

impl CurvePairIsolation {
    /// Retained cells in deterministic first-range/second-range order.
    pub fn candidates(&self) -> &[CurvePairCandidateCell] {
        &self.candidates
    }

    /// Requested exact pair-subdivision depth.
    pub const fn requested_depth(&self) -> u32 {
        self.requested_depth
    }

    /// Structured early-stop evidence.
    pub const fn limits(&self) -> CurvePairIsolationLimits {
        self.limits
    }

    /// True when every retained cell reached the requested depth.
    pub fn is_complete(&self) -> bool {
        self.limits.is_empty()
            && self
                .candidates
                .iter()
                .all(|candidate| candidate.depth == self.requested_depth)
    }

    /// True only when complete control-hull pruning excluded every pair.
    pub fn is_proven_empty(&self) -> bool {
        self.is_complete() && self.candidates.is_empty()
    }
}

/// Failure to construct or account a contextual curve-pair cover.
#[derive(Debug, Clone, PartialEq)]
pub enum ContextCurvePairIsolationError {
    /// Invalid geometry or exact NURBS processing failure.
    Kernel(Error),
    /// Invalid or exhausted operation policy before a conservative cover exists.
    Policy(OperationPolicyError),
}

impl From<Error> for ContextCurvePairIsolationError {
    fn from(error: Error) -> Self {
        Self::Kernel(error)
    }
}

impl From<OperationPolicyError> for ContextCurvePairIsolationError {
    fn from(error: OperationPolicyError) -> Self {
        Self::Policy(error)
    }
}

#[derive(Debug)]
struct WorkCell {
    cell: CurvePairCandidateCell,
    blocked: bool,
}

/// Isolate a conservative exact-subcurve cover of every possible contact.
pub fn isolate_curve_pair_candidates_in_scope(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    margin: f64,
    requested_depth: u32,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<CurvePairIsolation, ContextCurvePairIsolationError> {
    validate_inputs(first, first_range, second, second_range, margin)?;
    require_profile(scope)?;
    scope
        .ledger_mut()
        .charge(NURBS_CURVE_PAIR_SUBDIVISIONS, 1)?;

    let first = first.restricted_to(first_range)?;
    let second = second.restricted_to(second_range)?;
    let mut cells = initial_cells(first, second, margin);
    let subdivision_unavailable = requested_depth > 0
        && !cells.is_empty()
        && (cells[0].cell.first.degree() == 0 || cells[0].cell.second.degree() == 0);
    let mut limits = CurvePairIsolationLimits {
        subdivision_unavailable,
        ..CurvePairIsolationLimits::default()
    };
    if subdivision_unavailable {
        return Ok(isolation_result(cells, requested_depth, limits));
    }
    if observe_limit(
        scope,
        NURBS_CURVE_PAIR_CANDIDATES,
        ResourceKind::Items,
        usize_to_u64(
            cells.len(),
            NURBS_CURVE_PAIR_CANDIDATES,
            ResourceKind::Items,
        )?,
    )?
    .is_some_and(|snapshot| {
        limits.candidate_cells = Some(snapshot);
        true
    }) {
        return Ok(isolation_result(cells, requested_depth, limits));
    }
    if observe_limit(scope, NURBS_CURVE_PAIR_DEPTH, ResourceKind::Depth, 0)?.is_some_and(
        |snapshot| {
            limits.subdivision_depth = Some(snapshot);
            true
        },
    ) {
        return Ok(isolation_result(cells, requested_depth, limits));
    }

    for _ in 0..requested_depth {
        if cells.is_empty() || cells.iter().all(|work| work.blocked) {
            break;
        }
        let previous = core::mem::take(&mut cells);
        let previous_len = previous.len();
        let mut next = Vec::with_capacity(previous_len.saturating_mul(4));
        for (position, mut work) in previous.into_iter().enumerate() {
            if work.blocked
                || limits.subdivision_work.is_some()
                || limits.candidate_cells.is_some()
                || limits.subdivision_depth.is_some()
            {
                work.blocked = true;
                next.push(work);
                continue;
            }
            let attempted_depth = u64::from(work.cell.depth).saturating_add(1);
            if let Some(snapshot) = observe_limit(
                scope,
                NURBS_CURVE_PAIR_DEPTH,
                ResourceKind::Depth,
                attempted_depth,
            )? {
                limits.subdivision_depth = Some(snapshot);
                work.blocked = true;
                next.push(work);
                continue;
            }
            match scope.ledger_mut().charge(NURBS_CURVE_PAIR_SUBDIVISIONS, 1) {
                Ok(()) => {}
                Err(OperationPolicyError::LimitReached(snapshot)) => {
                    limits.subdivision_work = Some(snapshot);
                    work.blocked = true;
                    next.push(work);
                    continue;
                }
                Err(error) => return Err(error.into()),
            }
            let Some(children) = split_children(&work.cell, margin)? else {
                limits.parameter_resolution = true;
                scope.record_numeric_resolution(NURBS_CURVE_PAIR_DEPTH);
                work.blocked = true;
                next.push(work);
                continue;
            };
            let remaining = previous_len - position - 1;
            let attempted = next
                .len()
                .checked_add(children.len())
                .and_then(|count| count.checked_add(remaining))
                .ok_or(OperationPolicyError::AccountingOverflow {
                    stage: NURBS_CURVE_PAIR_CANDIDATES,
                    resource: ResourceKind::Items,
                })?;
            if let Some(snapshot) = observe_limit(
                scope,
                NURBS_CURVE_PAIR_CANDIDATES,
                ResourceKind::Items,
                usize_to_u64(attempted, NURBS_CURVE_PAIR_CANDIDATES, ResourceKind::Items)?,
            )? {
                limits.candidate_cells = Some(snapshot);
                work.blocked = true;
                next.push(work);
            } else {
                next.extend(children.into_iter().map(|cell| WorkCell {
                    cell,
                    blocked: false,
                }));
            }
        }
        cells = next;
    }
    Ok(isolation_result(cells, requested_depth, limits))
}

fn validate_inputs(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    margin: f64,
) -> Result<()> {
    if !margin.is_finite() || margin < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "curve-pair isolation margin must be finite and nonnegative",
        });
    }
    for (curve, range) in [(first, first_range), (second, second_range)] {
        let domain = curve.param_range();
        if !range.is_finite()
            || range.width() <= 0.0
            || range.lo < domain.lo
            || range.hi > domain.hi
        {
            return Err(Error::InvalidGeometry {
                reason: "curve-pair isolation ranges must be finite, positive, and inside the curve domains",
            });
        }
    }
    Ok(())
}

fn require_profile(
    scope: &OperationScope<'_, '_>,
) -> core::result::Result<(), OperationPolicyError> {
    NurbsCurvePairBudgetProfile::validate(&scope.context().effective_budget())
}

fn initial_cells(first: NurbsCurve, second: NurbsCurve, margin: f64) -> Vec<WorkCell> {
    candidate_cell(first, second, 0, margin)
        .map(|cell| {
            vec![WorkCell {
                cell,
                blocked: false,
            }]
        })
        .unwrap_or_default()
}

fn candidate_cell(
    first: NurbsCurve,
    second: NurbsCurve,
    depth: u32,
    margin: f64,
) -> Option<CurvePairCandidateCell> {
    let first_bounds = first.bounding_box(first.param_range());
    let second_bounds = second.bounding_box(second.param_range());
    first_bounds
        .inflated(margin)
        .intersects(second_bounds)
        .then_some(CurvePairCandidateCell {
            first,
            second,
            first_bounds,
            second_bounds,
            depth,
        })
}

fn split_children(
    parent: &CurvePairCandidateCell,
    margin: f64,
) -> Result<Option<Vec<CurvePairCandidateCell>>> {
    let first_range = parent.first.param_range();
    let second_range = parent.second.param_range();
    let first_mid = first_range.lo + first_range.width() / 2.0;
    let second_mid = second_range.lo + second_range.width() / 2.0;
    if !(first_range.lo < first_mid
        && first_mid < first_range.hi
        && second_range.lo < second_mid
        && second_mid < second_range.hi)
    {
        return Ok(None);
    }
    let (first_low, first_high) = parent.first.split_at(first_mid)?;
    let (second_low, second_high) = parent.second.split_at(second_mid)?;
    let first = [first_low, first_high];
    let second = [second_low, second_high];
    let mut children = Vec::with_capacity(4);
    for first in first {
        for second in &second {
            if let Some(cell) =
                candidate_cell(first.clone(), second.clone(), parent.depth + 1, margin)
            {
                children.push(cell);
            }
        }
    }
    Ok(Some(children))
}

fn isolation_result(
    mut cells: Vec<WorkCell>,
    requested_depth: u32,
    limits: CurvePairIsolationLimits,
) -> CurvePairIsolation {
    let mut candidates = cells.drain(..).map(|work| work.cell).collect::<Vec<_>>();
    candidates.sort_by(|a, b| {
        a.first_range()
            .lo
            .total_cmp(&b.first_range().lo)
            .then(a.second_range().lo.total_cmp(&b.second_range().lo))
    });
    CurvePairIsolation {
        candidates,
        requested_depth,
        limits,
    }
}

fn observe_limit(
    scope: &mut OperationScope<'_, '_>,
    stage: StageId,
    resource: ResourceKind,
    value: u64,
) -> core::result::Result<Option<LimitSnapshot>, OperationPolicyError> {
    match scope.ledger_mut().observe(stage, resource, value) {
        Ok(()) => Ok(None),
        Err(OperationPolicyError::LimitReached(snapshot)) => Ok(Some(snapshot)),
        Err(error) => Err(error),
    }
}

fn usize_to_u64(
    value: usize,
    stage: StageId,
    resource: ResourceKind,
) -> core::result::Result<u64, OperationPolicyError> {
    u64::try_from(value).map_err(|_| OperationPolicyError::AccountingOverflow { stage, resource })
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationOutcome, OperationReport, SessionPolicy};
    use kcore::tolerance::Tolerances;

    use super::*;
    use crate::vec::Point3;

    fn line(y: f64) -> NurbsCurve {
        NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(-1.0, y, 0.0), Point3::new(1.0, y, 0.0)],
            None,
        )
        .unwrap()
    }

    fn arch() -> NurbsCurve {
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(0.0, 2.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            None,
        )
        .unwrap()
    }

    fn run(
        first: &NurbsCurve,
        second: &NurbsCurve,
        depth: u32,
        overrides: BudgetPlan,
    ) -> OperationOutcome<CurvePairIsolation, ContextCurvePairIsolationError> {
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(NurbsCurvePairBudgetProfile::v1_defaults())
            .with_budget_overrides(overrides);
        let mut scope = OperationScope::new(&context);
        let result = isolate_curve_pair_candidates_in_scope(
            first,
            first.param_range(),
            second,
            second.param_range(),
            Tolerances::default().linear(),
            depth,
            &mut scope,
        );
        scope.finish_typed(result)
    }

    fn usage(report: &OperationReport, stage: StageId, resource: ResourceKind) -> LimitSnapshot {
        *report
            .usage()
            .iter()
            .find(|usage| usage.stage == stage && usage.resource == resource)
            .unwrap()
    }

    #[test]
    fn adaptive_cover_proves_hidden_miss_and_retains_crossing_candidates() {
        let first = arch();
        let separated = line(1.5);
        assert!(
            first
                .bounding_box(first.param_range())
                .intersects(separated.bounding_box(separated.param_range()))
        );
        let miss = run(&first, &separated, 3, BudgetPlan::empty());
        assert!(miss.result().unwrap().is_proven_empty());

        let crossing = line(0.5);
        let first_run = run(&first, &crossing, 3, BudgetPlan::empty());
        let second_run = run(&first, &crossing, 3, BudgetPlan::empty());
        assert_eq!(first_run, second_run);
        let isolation = first_run.result().unwrap();
        assert!(isolation.is_complete());
        assert!(!isolation.candidates().is_empty());
        assert!(
            isolation
                .candidates()
                .iter()
                .all(|candidate| candidate.depth() == 3)
        );
        assert!(isolation.candidates().windows(2).all(|pair| {
            (pair[0].first_range().lo, pair[0].second_range().lo)
                <= (pair[1].first_range().lo, pair[1].second_range().lo)
        }));

        let swapped = run(&crossing, &first, 3, BudgetPlan::empty());
        let swapped = swapped.result().unwrap();
        assert!(swapped.is_complete());
        let forward_ranges = isolation
            .candidates()
            .iter()
            .map(|cell| (cell.first_range(), cell.second_range()))
            .collect::<Vec<_>>();
        let mut swapped_ranges = swapped
            .candidates()
            .iter()
            .map(|cell| (cell.second_range(), cell.first_range()))
            .collect::<Vec<_>>();
        swapped_ranges.sort_by(|a, b| a.0.lo.total_cmp(&b.0.lo).then(a.1.lo.total_cmp(&b.1.lo)));
        assert_eq!(swapped_ranges, forward_ranges);
    }

    #[test]
    fn profile_is_an_exact_stable_contract() {
        let profile = NurbsCurvePairBudgetProfile::v1_defaults();
        assert_eq!(profile.limits().len(), 3);
        assert_eq!(
            profile
                .limits()
                .iter()
                .map(|limit| (limit.stage, limit.resource, limit.mode, limit.allowed))
                .collect::<Vec<_>>(),
            vec![
                (
                    NURBS_CURVE_PAIR_CANDIDATES,
                    ResourceKind::Items,
                    AccountingMode::HighWater,
                    4_096,
                ),
                (
                    NURBS_CURVE_PAIR_DEPTH,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    6,
                ),
                (
                    NURBS_CURVE_PAIR_SUBDIVISIONS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    1_366,
                ),
            ]
        );
    }

    #[test]
    fn work_candidate_and_depth_boundaries_retain_conservative_cover() {
        let first = arch();
        let second = line(0.5);
        let baseline = run(&first, &second, 2, BudgetPlan::empty());
        let work = usage(
            baseline.report(),
            NURBS_CURVE_PAIR_SUBDIVISIONS,
            ResourceKind::Work,
        );
        let candidates = usage(
            baseline.report(),
            NURBS_CURVE_PAIR_CANDIDATES,
            ResourceKind::Items,
        );
        let depth = usage(
            baseline.report(),
            NURBS_CURVE_PAIR_DEPTH,
            ResourceKind::Depth,
        );
        assert!(work.consumed > 1);
        assert!(candidates.consumed > 1);
        assert_eq!(depth.consumed, 2);

        for (stage, resource, mode, consumed) in [
            (
                NURBS_CURVE_PAIR_SUBDIVISIONS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                work.consumed,
            ),
            (
                NURBS_CURVE_PAIR_CANDIDATES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                candidates.consumed,
            ),
            (
                NURBS_CURVE_PAIR_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                depth.consumed,
            ),
        ] {
            let exact = BudgetPlan::new([LimitSpec::new(stage, resource, mode, consumed)]).unwrap();
            let exact = run(&first, &second, 2, exact);
            assert!(exact.result().unwrap().is_complete());
            assert!(exact.report().limit_events().is_empty());

            let low =
                BudgetPlan::new([LimitSpec::new(stage, resource, mode, consumed - 1)]).unwrap();
            let low = run(&first, &second, 2, low);
            let isolation = low.result().unwrap();
            assert!(!isolation.is_complete());
            assert!(!isolation.candidates().is_empty());
            let crossing = *low.report().limit_events().last().unwrap();
            assert_eq!(crossing.stage, stage);
            assert_eq!(
                (crossing.consumed, crossing.allowed),
                (consumed, consumed - 1)
            );
        }
    }

    #[test]
    fn missing_profile_is_rejected_before_a_root_level_empty_proof() {
        let first = line(0.0);
        let second = line(10.0);
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&context);
        let result = isolate_curve_pair_candidates_in_scope(
            &first,
            first.param_range(),
            &second,
            second.param_range(),
            Tolerances::default().linear(),
            2,
            &mut scope,
        );
        assert!(matches!(
            result,
            Err(ContextCurvePairIsolationError::Policy(
                OperationPolicyError::UnknownLimit { .. }
            ))
        ));
    }

    #[test]
    fn unrepresentable_midpoint_retains_cover_and_records_numeric_resolution() {
        let lo = 1.0e16_f64;
        let hi = lo.next_up();
        let first = NurbsCurve::new(
            1,
            vec![lo, lo, hi, hi],
            vec![Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            None,
        )
        .unwrap();
        let second = NurbsCurve::new(
            1,
            vec![lo, lo, hi, hi],
            vec![Point3::new(0.0, -1.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            None,
        )
        .unwrap();
        let outcome = run(&first, &second, 1, BudgetPlan::empty());
        let isolation = outcome.result().unwrap();
        assert!(!isolation.is_complete());
        assert!(isolation.limits().parameter_resolution());
        assert_eq!(isolation.candidates().len(), 1);
        assert_eq!(
            outcome.report().numeric_resolution_stages(),
            &[NURBS_CURVE_PAIR_DEPTH]
        );
    }
}
