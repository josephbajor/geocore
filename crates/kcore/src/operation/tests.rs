use crate::error::Error;
use crate::tolerance::{ANGULAR_RESOLUTION, LINEAR_RESOLUTION, SIZE_BOX_HALF, Tolerances};

use super::*;

const STAGE_A: StageId = match StageId::new("kcore.test.a") {
    Ok(value) => value,
    Err(_) => panic!("valid test stage"),
};
const STAGE_B: StageId = match StageId::new("kcore.test.b") {
    Ok(value) => value,
    Err(_) => panic!("valid test stage"),
};
const CODE_A: DiagnosticCode = match DiagnosticCode::new("kcore.test.notice-a") {
    Ok(value) => value,
    Err(_) => panic!("valid test code"),
};

fn plan() -> BudgetPlan {
    BudgetPlan::new([
        LimitSpec::new(STAGE_B, ResourceKind::Depth, AccountingMode::HighWater, 8),
        LimitSpec::new(STAGE_A, ResourceKind::Work, AccountingMode::Cumulative, 10),
    ])
    .expect("valid plan")
}

#[test]
fn validates_precision_numerical_policy_and_ids() {
    assert!(SessionPrecision::try_new(1.0, 2.0, 3.0).is_ok());
    assert!(SessionPrecision::try_new(0.0, 2.0, 3.0).is_err());
    assert!(SessionPrecision::try_new(1.0, f64::NAN, 3.0).is_err());
    assert!(NumericalPolicy::try_new(1.0, 2.0, 0.5).is_ok());
    assert!(NumericalPolicy::try_new(-1.0, 2.0, 0.5).is_err());
    assert!(StageId::new("not-namespaced").is_err());
    assert!(StageId::new("Bad.name").is_err());
    assert!(DiagnosticCode::new("kcore.good-code").is_ok());
}

#[test]
fn rejects_duplicate_and_invalid_limits() {
    let duplicate = LimitSpec::new(STAGE_A, ResourceKind::Work, AccountingMode::Cumulative, 1);
    assert!(matches!(
        BudgetPlan::new([duplicate, duplicate]),
        Err(OperationPolicyError::DuplicateLimit { .. })
    ));
    assert!(matches!(
        BudgetPlan::new([LimitSpec::new(
            STAGE_A,
            ResourceKind::Depth,
            AccountingMode::Cumulative,
            1,
        )]),
        Err(OperationPolicyError::InvalidLimitMode { .. })
    ));
}

#[test]
fn cumulative_and_high_water_accounting_use_inclusive_boundaries() {
    let mut ledger = WorkLedger::new(plan());
    ledger.charge(STAGE_A, 4).expect("inside limit");
    ledger.charge(STAGE_A, 6).expect("exact boundary");
    assert!(matches!(
        ledger.charge(STAGE_A, 1),
        Err(OperationPolicyError::LimitReached(LimitSnapshot {
            consumed: 11,
            allowed: 10,
            ..
        }))
    ));
    ledger
        .observe(STAGE_B, ResourceKind::Depth, 8)
        .expect("exact boundary");
    ledger
        .observe(STAGE_B, ResourceKind::Depth, 3)
        .expect("lower observation");
    assert!(matches!(
        ledger.observe(STAGE_B, ResourceKind::Depth, 9),
        Err(OperationPolicyError::LimitReached(_))
    ));
    assert_eq!(ledger.snapshots()[0].consumed, 10);
    assert_eq!(ledger.snapshots()[1].consumed, 8);
}

#[test]
fn overflow_is_rejected_without_mutating_usage() {
    let overflow_plan = BudgetPlan::new([LimitSpec::new(
        STAGE_A,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        u64::MAX,
    )])
    .expect("valid plan");
    let mut ledger = WorkLedger::new(overflow_plan);
    ledger.charge(STAGE_A, u64::MAX).expect("exact maximum");
    assert!(matches!(
        ledger.charge(STAGE_A, 1),
        Err(OperationPolicyError::AccountingOverflow { .. })
    ));
    assert_eq!(ledger.snapshots()[0].consumed, u64::MAX);
}

#[test]
fn child_merge_is_ordinal_ordered_and_input_order_independent() {
    let child_plan = BudgetPlan::new([LimitSpec::new(
        STAGE_A,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        5,
    )])
    .expect("valid child plan")
    .with_total_work_limit(5);
    let parent_plan = BudgetPlan::new([LimitSpec::new(
        STAGE_A,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        10,
    )])
    .expect("valid parent plan")
    .with_total_work_limit(10);
    let mut parent = WorkLedger::new(parent_plan);
    let mut first = parent
        .reserve_child(2, child_plan.clone())
        .expect("first reservation");
    let mut second = parent
        .reserve_child(9, child_plan)
        .expect("second reservation");
    first.ledger_mut().charge(STAGE_A, 2).expect("child work");
    second.ledger_mut().charge(STAGE_A, 3).expect("child work");
    parent
        .merge_children(vec![second, first])
        .expect("ordinal-sorted merge");
    assert_eq!(parent.snapshots()[0].consumed, 5);
}

#[test]
fn child_reservations_protect_stage_and_root_capacity() {
    let child_plan = BudgetPlan::new([LimitSpec::new(
        STAGE_A,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        4,
    )])
    .expect("valid child plan")
    .with_total_work_limit(4);
    let parent_plan = BudgetPlan::new([
        LimitSpec::new(STAGE_A, ResourceKind::Work, AccountingMode::Cumulative, 10),
        LimitSpec::new(STAGE_B, ResourceKind::Work, AccountingMode::Cumulative, 10),
    ])
    .expect("valid parent plan")
    .with_total_work_limit(10);
    let mut parent = WorkLedger::new(parent_plan);
    let child = parent
        .reserve_child(1, child_plan)
        .expect("child reservation");
    parent.charge(STAGE_B, 6).expect("unreserved root capacity");
    assert!(matches!(
        parent.charge(STAGE_B, 1),
        Err(OperationPolicyError::LimitReached(LimitSnapshot {
            consumed: 7,
            allowed: 6,
            ..
        }))
    ));
    parent.merge_children(vec![child]).expect("join child");
}

#[test]
fn report_is_ordered_bounded_and_retained_on_both_outcomes() {
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        plan(),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, Tolerances::default())
        .expect("valid context")
        .with_diagnostics(DiagnosticLevel::Summary, 1);
    let mut scope = OperationScope::new(&context);
    scope.ledger_mut().charge(STAGE_A, 3).expect("work");
    scope.diagnose(
        STAGE_B,
        CODE_A,
        DiagnosticKind::FallbackSelected,
        "selected test fallback",
    );
    scope.diagnose(
        STAGE_A,
        CODE_A,
        DiagnosticKind::ProofIncomplete,
        "dropped by bound",
    );
    let success = scope.finish(Ok(42));
    assert_eq!(success.result(), Ok(&42));
    assert_eq!(success.report().policy_version(), PolicyVersion::V1);
    assert_eq!(success.report().diagnostics()[0].ordinal, 0);
    assert_eq!(success.report().dropped_diagnostics(), 1);
    assert_eq!(success.report().usage()[0].stage, STAGE_A);

    let failure_scope = OperationScope::new(&context);
    let failure: OperationOutcome<()> = failure_scope.finish(Err(Error::StaleHandle));
    assert_eq!(failure.result(), Err(&Error::StaleHandle));
    let (result, report) = failure.into_parts();
    assert_eq!(result, Err(Error::StaleHandle));
    assert_eq!(report.policy_version(), PolicyVersion::V1);
}

#[test]
fn v1_precision_and_parameter_policy_are_stable() {
    let policy = SessionPolicy::v1();
    assert_eq!(policy.policy_version(), PolicyVersion::V1);
    assert_eq!(policy.precision().linear_resolution(), LINEAR_RESOLUTION);
    assert_eq!(policy.precision().angular_resolution(), ANGULAR_RESOLUTION);
    assert_eq!(policy.precision().size_box_half(), SIZE_BOX_HALF);

    let tolerance = policy
        .numerical()
        .parameter_tolerance(
            ParameterScale {
                coordinate_magnitude: 2.0,
                span: 4.0,
                output_rate_upper: Some(2.0),
            },
            1e-8,
        )
        .expect("valid scale");
    assert_eq!(tolerance.metric_driven_step, Some(5e-9));
    assert!(tolerance.termination_step >= tolerance.rounding_floor);
}

#[test]
fn context_rejects_tolerance_below_custom_session_precision() {
    let session = SessionPolicy::new(
        SessionPrecision::try_new(1e-4, ANGULAR_RESOLUTION, SIZE_BOX_HALF)
            .expect("valid custom precision"),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        BudgetPlan::empty(),
        PolicyVersion::V1,
    );
    assert!(matches!(
        OperationContext::new(&session, Tolerances::default()),
        Err(OperationPolicyError::InvalidOperationTolerance)
    ));
}
