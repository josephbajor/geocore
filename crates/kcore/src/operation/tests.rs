use std::collections::BTreeSet;

use crate::error::{ClassifiedError, Error, ErrorClass};
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
fn policy_error_variants_have_exhaustive_stable_classification() {
    let snapshot = LimitSnapshot {
        stage: STAGE_A,
        resource: ResourceKind::Work,
        consumed: 11,
        allowed: 10,
    };
    let cases = [
        (
            OperationPolicyError::InvalidIdentifier,
            ErrorClass::InvalidInput,
            code::INVALID_IDENTIFIER,
        ),
        (
            OperationPolicyError::InvalidSessionPrecision,
            ErrorClass::InvalidInput,
            code::INVALID_SESSION_PRECISION,
        ),
        (
            OperationPolicyError::InvalidNumericalPolicy,
            ErrorClass::InvalidInput,
            code::INVALID_NUMERICAL_POLICY,
        ),
        (
            OperationPolicyError::InvalidOperationTolerance,
            ErrorClass::InvalidInput,
            code::INVALID_OPERATION_TOLERANCE,
        ),
        (
            OperationPolicyError::DuplicateLimit {
                stage: STAGE_A,
                resource: ResourceKind::Work,
            },
            ErrorClass::InvalidInput,
            code::DUPLICATE_LIMIT,
        ),
        (
            OperationPolicyError::InvalidLimitMode {
                stage: STAGE_A,
                resource: ResourceKind::Depth,
            },
            ErrorClass::InvalidInput,
            code::INVALID_LIMIT_MODE,
        ),
        (
            OperationPolicyError::UnknownLimit {
                stage: STAGE_A,
                resource: ResourceKind::Work,
            },
            ErrorClass::InvalidInput,
            code::UNKNOWN_LIMIT,
        ),
        (
            OperationPolicyError::AccountingModeMismatch {
                stage: STAGE_A,
                resource: ResourceKind::Depth,
            },
            ErrorClass::InvalidInput,
            code::ACCOUNTING_MODE_MISMATCH,
        ),
        (
            OperationPolicyError::LimitReached(snapshot),
            ErrorClass::ResourceLimit,
            code::LIMIT_REACHED,
        ),
        (
            OperationPolicyError::AccountingOverflow {
                stage: STAGE_A,
                resource: ResourceKind::Work,
            },
            ErrorClass::InvalidInput,
            code::ACCOUNTING_OVERFLOW,
        ),
        (
            OperationPolicyError::InvalidChildOrdinal,
            ErrorClass::InvalidState,
            code::INVALID_CHILD_ORDINAL,
        ),
        (
            OperationPolicyError::ChildReservationExceeded {
                stage: STAGE_A,
                resource: ResourceKind::Work,
            },
            ErrorClass::InvalidState,
            code::CHILD_RESERVATION_EXCEEDED,
        ),
        (
            OperationPolicyError::UnknownChildReservation,
            ErrorClass::InvalidState,
            code::UNKNOWN_CHILD_RESERVATION,
        ),
    ];

    assert_eq!(cases.len(), code::ALL.len());
    for (error, class, expected_code) in cases {
        assert_eq!(error.class(), class, "wrong class for {error:?}");
        assert_eq!(error.code(), expected_code, "wrong code for {error:?}");
        assert_eq!(ClassifiedError::class(&error), class);
        assert_eq!(ClassifiedError::code(&error), expected_code);
        assert_eq!(error.capability(), None);
        assert!(std::error::Error::source(&error).is_none());
        let expected_limit =
            (error == OperationPolicyError::LimitReached(snapshot)).then_some(snapshot);
        assert_eq!(error.limit(), expected_limit);
        assert_eq!(ClassifiedError::limit(&error), expected_limit);
        assert!(!error.to_string().is_empty());
    }
}

#[test]
fn policy_error_code_inventory_has_one_intentional_canonical_delegation() {
    let codes: BTreeSet<_> = code::ALL.iter().map(|code| code.as_str()).collect();
    assert_eq!(codes.len(), code::ALL.len());
    assert_eq!(code::OWNED.len() + 1, code::ALL.len());
    assert!(
        code::OWNED
            .iter()
            .all(|code| code.as_str().starts_with("kcore.operation."))
    );
    let shared: Vec<_> = crate::error::code::ALL
        .iter()
        .filter(|legacy| codes.contains(legacy.as_str()))
        .copied()
        .collect();
    assert_eq!(shared, [crate::error::code::RESOURCE_LIMIT]);
    assert_eq!(code::LIMIT_REACHED, crate::error::code::RESOURCE_LIMIT);
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
fn required_limit_validation_is_consistent_and_read_only_for_plans_and_ledgers() {
    let plan = plan();
    let ledger = WorkLedger::new(plan.clone());
    let before = ledger.clone();

    for result in [
        plan.require_limit(STAGE_A, ResourceKind::Work, AccountingMode::Cumulative),
        ledger.require_limit(STAGE_A, ResourceKind::Work, AccountingMode::Cumulative),
    ] {
        assert_eq!(result, Ok(()));
    }
    for result in [
        plan.require_limit(STAGE_A, ResourceKind::Work, AccountingMode::HighWater),
        ledger.require_limit(STAGE_A, ResourceKind::Work, AccountingMode::HighWater),
    ] {
        assert_eq!(
            result,
            Err(OperationPolicyError::AccountingModeMismatch {
                stage: STAGE_A,
                resource: ResourceKind::Work,
            })
        );
    }
    for result in [
        plan.require_limit(STAGE_B, ResourceKind::Work, AccountingMode::Cumulative),
        ledger.require_limit(STAGE_B, ResourceKind::Work, AccountingMode::Cumulative),
    ] {
        assert_eq!(
            result,
            Err(OperationPolicyError::UnknownLimit {
                stage: STAGE_B,
                resource: ResourceKind::Work,
            })
        );
    }
    assert_eq!(ledger, before);
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
fn charge_preflight_is_strictly_read_only_with_reservations_and_diagnostics() {
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        plan().with_total_work_limit(10),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, Tolerances::default())
        .unwrap()
        .with_diagnostics(DiagnosticLevel::Summary, 2);
    let mut scope = OperationScope::new(&context);
    let child_plan = BudgetPlan::new([LimitSpec::new(
        STAGE_A,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        2,
    )])
    .unwrap()
    .with_total_work_limit(2);
    let _child = scope.ledger_mut().reserve_child(0, child_plan).unwrap();
    scope.diagnose(
        STAGE_A,
        CODE_A,
        DiagnosticKind::ProofIncomplete,
        "pre-existing diagnostic",
    );
    let before = scope.ledger().clone();

    scope
        .ledger()
        .check_charge(STAGE_A, 8)
        .expect("exact unreserved allowance remains available");
    assert!(matches!(
        scope.ledger().check_charge(STAGE_A, 9),
        Err(OperationPolicyError::LimitReached(LimitSnapshot {
            stage: STAGE_A,
            consumed: 9,
            allowed: 8,
            ..
        }))
    ));
    assert_eq!(scope.ledger(), &before);

    let outcome = scope.finish(Ok(()));
    assert_eq!(outcome.report().usage()[0].consumed, 0);
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(outcome.report().diagnostics().len(), 1);
    assert_eq!(
        outcome.report().diagnostics()[0].message,
        "pre-existing diagnostic"
    );
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
    let mut child = parent
        .reserve_child(1, child_plan)
        .expect("child reservation");
    child
        .ledger_mut()
        .charge(STAGE_A, 4)
        .expect("child allowance");
    let child_limit = match child.ledger_mut().charge(STAGE_A, 1) {
        Err(OperationPolicyError::LimitReached(snapshot)) => snapshot,
        other => panic!("unexpected child limit result: {other:?}"),
    };
    parent.charge(STAGE_B, 6).expect("unreserved root capacity");
    let parent_limit = match parent.charge(STAGE_B, 1) {
        Err(OperationPolicyError::LimitReached(snapshot)) => snapshot,
        other => panic!("unexpected parent limit result: {other:?}"),
    };
    assert_eq!(parent_limit.stage, TOTAL_WORK_STAGE);
    assert_eq!(parent_limit.consumed, 7);
    assert_eq!(parent_limit.allowed, 6);
    parent.merge_children(vec![child]).expect("join child");
    assert_eq!(parent.limit_events(), &[parent_limit, child_limit]);
}

#[test]
fn omitted_child_root_is_inferred_and_prevents_late_merge_exhaustion() {
    let parent_plan = BudgetPlan::new([
        LimitSpec::new(STAGE_A, ResourceKind::Work, AccountingMode::Cumulative, 5),
        LimitSpec::new(STAGE_B, ResourceKind::Work, AccountingMode::Cumulative, 10),
    ])
    .unwrap()
    .with_total_work_limit(10);
    let child_plan = BudgetPlan::new([LimitSpec::new(
        STAGE_A,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        5,
    )])
    .unwrap();
    let mut parent = WorkLedger::new(parent_plan);
    let mut child = parent.reserve_child(1, child_plan).unwrap();
    child.ledger_mut().charge(STAGE_A, 5).unwrap();
    parent.charge(STAGE_B, 5).unwrap();

    let protected = match parent.charge(STAGE_B, 1) {
        Err(OperationPolicyError::LimitReached(snapshot)) => snapshot,
        other => panic!("unexpected protected-capacity result: {other:?}"),
    };
    assert_eq!(protected.stage, TOTAL_WORK_STAGE);
    assert_eq!(protected.consumed, 6);
    assert_eq!(protected.allowed, 5);

    parent.merge_children(vec![child]).unwrap();
    assert_eq!(parent.total_work_consumed(), 10);
    assert_eq!(
        parent
            .snapshots()
            .into_iter()
            .find(|snapshot| snapshot.stage == STAGE_B)
            .unwrap()
            .consumed,
        5
    );
    assert_eq!(parent.limit_events(), &[protected]);
}

#[test]
fn inferred_child_root_rejects_insufficient_capacity_and_respects_explicit_stricter_total() {
    let parent_plan = BudgetPlan::new([
        LimitSpec::new(STAGE_A, ResourceKind::Work, AccountingMode::Cumulative, 5),
        LimitSpec::new(STAGE_B, ResourceKind::Work, AccountingMode::Cumulative, 10),
    ])
    .unwrap()
    .with_total_work_limit(4);
    let child_plan = BudgetPlan::new([LimitSpec::new(
        STAGE_A,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        5,
    )])
    .unwrap();
    let mut parent = WorkLedger::new(parent_plan);
    assert_eq!(
        parent.reserve_child(1, child_plan.clone()).unwrap_err(),
        OperationPolicyError::ChildReservationExceeded {
            stage: TOTAL_WORK_STAGE,
            resource: ResourceKind::Work,
        }
    );

    let parent_plan = BudgetPlan::new([
        LimitSpec::new(STAGE_A, ResourceKind::Work, AccountingMode::Cumulative, 5),
        LimitSpec::new(STAGE_B, ResourceKind::Work, AccountingMode::Cumulative, 10),
    ])
    .unwrap()
    .with_total_work_limit(10);
    let mut parent = WorkLedger::new(parent_plan);
    let mut child = parent
        .reserve_child(1, child_plan.with_total_work_limit(3))
        .unwrap();
    parent.charge(STAGE_B, 7).unwrap();
    child.ledger_mut().charge(STAGE_A, 3).unwrap();
    parent.merge_children(vec![child]).unwrap();
    assert_eq!(parent.total_work_consumed(), 10);
}

#[test]
fn numeric_resolution_events_merge_by_child_ordinal_and_rollback_cleanly() {
    let mut parent = WorkLedger::new(BudgetPlan::empty());
    let mut first = parent.reserve_child(2, BudgetPlan::empty()).unwrap();
    let mut second = parent.reserve_child(9, BudgetPlan::empty()).unwrap();
    first.ledger_mut().record_numeric_resolution(STAGE_B);
    first.ledger_mut().record_numeric_resolution(STAGE_A);
    second.ledger_mut().record_numeric_resolution(STAGE_A);

    let mut reversed_parent = parent.clone();
    let normal_children = vec![first.clone(), second.clone()];
    reversed_parent.merge_children(vec![second, first]).unwrap();
    parent.merge_children(normal_children).unwrap();
    assert_eq!(parent.numeric_resolution_stages(), &[STAGE_B, STAGE_A]);
    assert_eq!(
        reversed_parent.numeric_resolution_stages(),
        parent.numeric_resolution_stages()
    );

    let mut rollback_parent = WorkLedger::new(BudgetPlan::empty());
    rollback_parent.record_numeric_resolution(STAGE_A);
    let mut child = rollback_parent
        .reserve_child(1, BudgetPlan::empty())
        .unwrap();
    child.ledger_mut().record_numeric_resolution(STAGE_B);
    assert_eq!(
        rollback_parent.merge_children(Vec::new()),
        Err(OperationPolicyError::UnknownChildReservation)
    );
    assert_eq!(rollback_parent.numeric_resolution_stages(), &[STAGE_A]);
    rollback_parent.merge_children(vec![child]).unwrap();
    assert_eq!(
        rollback_parent.numeric_resolution_stages(),
        &[STAGE_A, STAGE_B]
    );
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
    let attempted = match scope.ledger_mut().charge(STAGE_A, 8) {
        Err(OperationPolicyError::LimitReached(snapshot)) => snapshot,
        other => panic!("unexpected limit result: {other:?}"),
    };
    assert!(matches!(
        scope.ledger_mut().charge(STAGE_A, 9),
        Err(OperationPolicyError::LimitReached(_))
    ));
    scope.record_numeric_resolution(STAGE_B);
    scope.record_numeric_resolution(STAGE_B);
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
    // The error type remains inferable for this legacy Ok-only call.
    let success = scope.finish(Ok(42));
    assert_eq!(success.result(), Ok(&42));
    assert_eq!(success.report().policy_version(), PolicyVersion::V1);
    assert_eq!(success.report().diagnostics()[0].ordinal, 0);
    assert_eq!(success.report().dropped_diagnostics(), 1);
    assert_eq!(success.report().usage()[0].stage, STAGE_A);
    assert_eq!(success.report().limit_events(), &[attempted]);
    assert_eq!(success.report().numeric_resolution_stages(), &[STAGE_B]);

    let failure_scope = OperationScope::new(&context);
    let failure: OperationOutcome<()> = failure_scope.finish(Err(Error::StaleHandle));
    assert_eq!(failure.result(), Err(&Error::StaleHandle));
    let (result, report) = failure.into_parts();
    assert_eq!(result, Err(Error::StaleHandle));
    assert_eq!(report.policy_version(), PolicyVersion::V1);
    assert!(report.limit_events().is_empty());
    assert!(report.numeric_resolution_stages().is_empty());
}

#[test]
fn structured_limit_events_survive_diagnostics_off() {
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        plan(),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, Tolerances::default()).expect("valid context");
    let mut scope = OperationScope::new(&context);
    let snapshot = match scope.ledger_mut().charge(STAGE_A, 11) {
        Err(OperationPolicyError::LimitReached(snapshot)) => snapshot,
        other => panic!("unexpected limit result: {other:?}"),
    };
    scope.record_numeric_resolution(STAGE_B);
    scope.diagnose(
        snapshot.stage,
        CODE_A,
        DiagnosticKind::LimitReached(snapshot),
        "filtered diagnostic",
    );
    let outcome = scope.finish(Ok(()));
    assert_eq!(outcome.report().limit_events(), &[snapshot]);
    assert_eq!(outcome.report().numeric_resolution_stages(), &[STAGE_B]);
    assert!(outcome.report().diagnostics().is_empty());
}

#[test]
fn numeric_resolution_does_not_require_a_budget_stage() {
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let mut scope = OperationScope::new(&context);
    scope.record_numeric_resolution(STAGE_A);
    scope.record_numeric_resolution(STAGE_A);
    let outcome = scope.finish(Ok(()));
    assert_eq!(outcome.report().numeric_resolution_stages(), &[STAGE_A]);
    assert!(outcome.report().usage().is_empty());
    assert!(outcome.report().diagnostics().is_empty());
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
