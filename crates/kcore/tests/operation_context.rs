//! Public-contract tests for operation policy, accounting, and outcomes.

use kcore::error::Error;
use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, DiagnosticKind, DiagnosticLevel, LimitSpec,
    OperationContext, OperationOutcome, OperationScope, PolicyVersion, ResourceKind, SessionPolicy,
    StageId,
};
use kcore::tolerance::Tolerances;

const SOLVE: StageId = match StageId::new("test.operation.solve") {
    Ok(stage) => stage,
    Err(_) => panic!("valid stage identifier"),
};
const FALLBACK: DiagnosticCode = match DiagnosticCode::new("test.operation.fallback") {
    Ok(code) => code,
    Err(_) => panic!("valid diagnostic identifier"),
};

#[test]
fn public_context_scope_and_outcome_preserve_error_reports() {
    let session = SessionPolicy::v1();
    let budget = BudgetPlan::new([LimitSpec::new(
        SOLVE,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        4,
    )])
    .expect("valid budget");
    let context = OperationContext::new(&session, Tolerances::default())
        .expect("valid operation context")
        .with_budget_overrides(budget)
        .with_diagnostics(DiagnosticLevel::Summary, 1);
    let mut scope = OperationScope::new(&context);
    scope.ledger_mut().charge(SOLVE, 4).expect("exact limit");
    scope.diagnose(
        SOLVE,
        FALLBACK,
        DiagnosticKind::FallbackSelected,
        "test fallback",
    );

    let outcome: OperationOutcome<()> = scope.finish(Err(Error::StaleHandle));
    assert_eq!(outcome.result(), Err(&Error::StaleHandle));
    assert_eq!(outcome.report().policy_version(), PolicyVersion::V1);
    assert_eq!(outcome.report().usage()[0].consumed, 4);
    assert_eq!(outcome.report().diagnostics()[0].ordinal, 0);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LayerError {
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AdaptedError {
    Rejected,
}

#[test]
fn typed_outcomes_retain_layer_errors_and_maps_preserve_the_exact_report() {
    let session = SessionPolicy::v1();
    let budget = BudgetPlan::new([LimitSpec::new(
        SOLVE,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        4,
    )])
    .expect("valid budget");
    let context = OperationContext::new(&session, Tolerances::default())
        .expect("valid operation context")
        .with_budget_overrides(budget)
        .with_diagnostics(DiagnosticLevel::Summary, 1);

    let mut success_scope = OperationScope::new(&context);
    success_scope
        .ledger_mut()
        .charge(SOLVE, 3)
        .expect("accounted work");
    success_scope.diagnose(
        SOLVE,
        FALLBACK,
        DiagnosticKind::FallbackSelected,
        "test fallback",
    );
    let success: OperationOutcome<u32, LayerError> = success_scope.finish_typed(Ok(21));
    let success_report = success.report().clone();
    let mapped = success.map(|value| value * 2);
    assert_eq!(mapped.result(), Ok(&42));
    assert_eq!(mapped.report(), &success_report);
    let (result, report) = mapped.into_parts();
    assert_eq!(result, Ok(42));
    assert_eq!(report, success_report);

    let mut failure_scope = OperationScope::new(&context);
    failure_scope
        .ledger_mut()
        .charge(SOLVE, 2)
        .expect("accounted work");
    let failure: OperationOutcome<(), LayerError> =
        failure_scope.finish_typed(Err(LayerError::Rejected));
    let failure_report = failure.report().clone();
    let mapped_error = failure.map_err(|LayerError::Rejected| AdaptedError::Rejected);
    assert_eq!(mapped_error.result(), Err(&AdaptedError::Rejected));
    assert_eq!(mapped_error.report(), &failure_report);
    assert_eq!(
        mapped_error.into_parts(),
        (Err(AdaptedError::Rejected), failure_report)
    );

    let direct_failure: OperationOutcome<(), LayerError> =
        OperationScope::new(&context).finish_typed(Err(LayerError::Rejected));
    assert_eq!(direct_failure.into_result(), Err(LayerError::Rejected));
}

#[test]
fn legacy_ok_only_finish_infers_the_default_kernel_error() {
    let session = SessionPolicy::v1();
    let context =
        OperationContext::new(&session, Tolerances::default()).expect("valid operation context");
    let scope = OperationScope::new(&context);

    // No result annotation or `Ok::<_, Error>` hint: this is the legacy
    // source shape that a fully generic `finish` would make ambiguous.
    let outcome = scope.finish(Ok(7_u32));
    let _: &OperationOutcome<u32> = &outcome;
    assert_eq!(outcome.into_result(), Ok(7));
}

#[test]
fn public_child_ledgers_merge_by_stable_ordinal() {
    let child_budget = BudgetPlan::new([LimitSpec::new(
        SOLVE,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        3,
    )])
    .expect("valid child budget");
    let parent_budget = BudgetPlan::new([LimitSpec::new(
        SOLVE,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        6,
    )])
    .expect("valid parent budget");
    let mut parent = kcore::operation::WorkLedger::new(parent_budget);
    let mut early = parent
        .reserve_child(10, child_budget.clone())
        .expect("first reservation");
    let mut late = parent
        .reserve_child(20, child_budget)
        .expect("second reservation");
    early.ledger_mut().charge(SOLVE, 1).expect("child work");
    late.ledger_mut().charge(SOLVE, 2).expect("child work");

    parent
        .merge_children(vec![late, early])
        .expect("deterministic merge");
    assert_eq!(parent.snapshots()[0].consumed, 3);
}
