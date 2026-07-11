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
