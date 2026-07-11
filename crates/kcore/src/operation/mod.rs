//! Deterministic operation policy, resource accounting, and diagnostics.
//!
//! This module is deliberately payload-agnostic. Higher layers define stable
//! stage and diagnostic identifiers, then use these primitives without making
//! `kcore` aware of geometry, topology, or a particular algorithm.

mod budget;
mod context;
mod id;
mod policy;

pub use budget::{
    AccountingMode, BudgetPlan, ChildWorkLedger, LimitSnapshot, LimitSpec, ResourceKind,
    TOTAL_WORK_STAGE, WorkLedger,
};
pub use context::{
    DiagnosticKind, DiagnosticLevel, OperationContext, OperationDiagnostic, OperationOutcome,
    OperationReport, OperationScope,
};
pub use id::{DiagnosticCode, OperationPolicyError, PolicyVersion, StageId, code};
pub use policy::{
    ExecutionPolicy, NumericGuardKind, NumericalPolicy, ParameterScale, ParameterTolerance,
    SessionPolicy, SessionPrecision,
};

#[cfg(test)]
mod tests;
