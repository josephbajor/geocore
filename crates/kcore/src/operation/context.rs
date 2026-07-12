//! Operation contexts, scopes, diagnostics, reports, and outcomes.

use crate::error::Error;
use crate::tolerance::Tolerances;

use super::budget::{BudgetPlan, LimitSnapshot, WorkLedger};
use super::id::{DiagnosticCode, OperationPolicyError, PolicyVersion, StageId};
use super::policy::SessionPolicy;

/// Whether semantic diagnostics should be retained.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DiagnosticLevel {
    /// Do not retain diagnostics. Accounting is still performed.
    #[default]
    Off,
    /// Retain bounded semantic summary diagnostics.
    Summary,
}

/// Immutable configuration borrowed by one operation.
#[derive(Debug, Clone)]
pub struct OperationContext<'session> {
    session: &'session SessionPolicy,
    tolerances: Tolerances,
    family_budget_defaults: BudgetPlan,
    budget_overrides: BudgetPlan,
    diagnostic_level: DiagnosticLevel,
    diagnostic_capacity: usize,
}

impl<'session> OperationContext<'session> {
    /// Creates a validated context using session defaults and no diagnostics.
    pub fn new(
        session: &'session SessionPolicy,
        tolerances: Tolerances,
    ) -> core::result::Result<Self, OperationPolicyError> {
        if tolerances.linear() < session.precision().linear_resolution()
            || tolerances.angular() < session.precision().angular_resolution()
        {
            return Err(OperationPolicyError::InvalidOperationTolerance);
        }
        Ok(Self {
            session,
            tolerances,
            family_budget_defaults: BudgetPlan::empty(),
            budget_overrides: BudgetPlan::empty(),
            diagnostic_level: DiagnosticLevel::Off,
            diagnostic_capacity: 0,
        })
    }

    /// Installs the owning operation family's default budget profile.
    ///
    /// Family defaults have the lowest precedence: session entries replace
    /// matching defaults, and explicit per-operation overrides replace both.
    /// Omitted entries and root total-work ceilings flow through unchanged.
    /// The three layers remain separate, so builder call order does not affect
    /// the composed plan.
    pub fn with_family_budget_defaults(mut self, defaults: BudgetPlan) -> Self {
        self.family_budget_defaults = defaults;
        self
    }

    /// Replaces per-operation budget overrides.
    pub fn with_budget_overrides(mut self, overrides: BudgetPlan) -> Self {
        self.budget_overrides = overrides;
        self
    }

    /// Enables bounded semantic diagnostics.
    pub fn with_diagnostics(mut self, level: DiagnosticLevel, capacity: usize) -> Self {
        self.diagnostic_level = level;
        self.diagnostic_capacity = capacity;
        self
    }

    /// Returns the borrowed session policy.
    pub const fn session(&self) -> &'session SessionPolicy {
        self.session
    }

    /// Returns model-space acceptance tolerances for this operation.
    pub const fn tolerances(&self) -> Tolerances {
        self.tolerances
    }

    /// Returns per-operation budget overrides.
    pub const fn budget_overrides(&self) -> &BudgetPlan {
        &self.budget_overrides
    }

    /// Returns the effective family, session, and request budget.
    pub fn effective_budget(&self) -> BudgetPlan {
        self.family_budget_defaults
            .overlaid(self.session.default_budget())
            .overlaid(&self.budget_overrides)
    }
}

/// Stable semantic diagnostic category.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum DiagnosticKind {
    /// A deterministic resource limit was reached.
    LimitReached(LimitSnapshot),
    /// Arithmetic resolution prevented further meaningful progress.
    NumericResolution,
    /// A solve was too ill-conditioned for its primary path.
    IllConditioned,
    /// A deterministic fallback path was selected.
    FallbackSelected,
    /// The operation could not complete a requested proof.
    ProofIncomplete,
    /// Reserved for future external cancellation support.
    Cancelled,
}

/// One bounded, deterministically ordered semantic diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OperationDiagnostic {
    /// Operation-local ordinal assigned at insertion.
    pub ordinal: u64,
    /// Stage that emitted the diagnostic.
    pub stage: StageId,
    /// Stable machine-readable code.
    pub code: DiagnosticCode,
    /// Stable diagnostic category and structured data.
    pub kind: DiagnosticKind,
    /// Static human-readable context; not a control-flow contract.
    pub message: &'static str,
}

/// Mutable per-operation accounting and diagnostic state.
#[derive(Debug)]
pub struct OperationScope<'context, 'session> {
    context: &'context OperationContext<'session>,
    ledger: WorkLedger,
    diagnostics: Vec<OperationDiagnostic>,
    next_diagnostic_ordinal: u64,
    dropped_diagnostics: u64,
}

impl<'context, 'session> OperationScope<'context, 'session> {
    /// Starts a fresh scope from an immutable context snapshot.
    pub fn new(context: &'context OperationContext<'session>) -> Self {
        Self {
            context,
            ledger: WorkLedger::new(context.effective_budget()),
            diagnostics: Vec::with_capacity(context.diagnostic_capacity),
            next_diagnostic_ordinal: 0,
            dropped_diagnostics: 0,
        }
    }

    /// Returns the immutable operation context.
    pub const fn context(&self) -> &'context OperationContext<'session> {
        self.context
    }

    /// Returns mutable deterministic work accounting.
    pub fn ledger_mut(&mut self) -> &mut WorkLedger {
        &mut self.ledger
    }

    /// Returns current work accounting.
    pub const fn ledger(&self) -> &WorkLedger {
        &self.ledger
    }

    /// Retains a numeric-resolution stop independently of diagnostic level.
    ///
    /// Each validated stage is retained once in first-observed order. This
    /// evidence does not require a budget entry for the stage.
    pub fn record_numeric_resolution(&mut self, stage: StageId) {
        self.ledger.record_numeric_resolution(stage);
    }

    /// Records a semantic diagnostic if diagnostics are enabled and capacity remains.
    pub fn diagnose(
        &mut self,
        stage: StageId,
        code: DiagnosticCode,
        kind: DiagnosticKind,
        message: &'static str,
    ) {
        if self.context.diagnostic_level == DiagnosticLevel::Off {
            return;
        }
        if self.diagnostics.len() == self.context.diagnostic_capacity {
            self.dropped_diagnostics = self.dropped_diagnostics.saturating_add(1);
            return;
        }
        let ordinal = self.next_diagnostic_ordinal;
        self.next_diagnostic_ordinal = self.next_diagnostic_ordinal.saturating_add(1);
        self.diagnostics.push(OperationDiagnostic {
            ordinal,
            stage,
            code,
            kind,
            message,
        });
    }

    /// Finishes a kernel-error operation and preserves its report.
    ///
    /// This compatibility entry point intentionally fixes the error type to
    /// [`Error`], which keeps `scope.finish(Ok(value))` inference ergonomic.
    /// Use [`Self::finish_typed`] when a layer owns a more specific error.
    pub fn finish<T>(self, result: core::result::Result<T, Error>) -> OperationOutcome<T> {
        self.finish_typed(result)
    }

    /// Finishes an operation with a caller-defined error and preserves its report.
    pub fn finish_typed<T, E>(self, result: core::result::Result<T, E>) -> OperationOutcome<T, E> {
        let limit_events = self.ledger.limit_events().to_vec();
        let numeric_resolution_stages = self.ledger.numeric_resolution_stages().to_vec();
        OperationOutcome {
            result,
            report: OperationReport {
                policy_version: self.context.session.policy_version(),
                usage: self.ledger.snapshots(),
                limit_events,
                numeric_resolution_stages,
                diagnostics: self.diagnostics,
                dropped_diagnostics: self.dropped_diagnostics,
            },
        }
    }
}

/// Deterministic metadata retained after an operation succeeds or fails.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OperationReport {
    policy_version: PolicyVersion,
    usage: Vec<LimitSnapshot>,
    limit_events: Vec<LimitSnapshot>,
    numeric_resolution_stages: Vec<StageId>,
    diagnostics: Vec<OperationDiagnostic>,
    dropped_diagnostics: u64,
}

impl OperationReport {
    /// Returns the policy defaults version used by the operation.
    pub const fn policy_version(&self) -> PolicyVersion {
        self.policy_version
    }

    /// Returns canonical stage/resource usage order.
    pub fn usage(&self) -> &[LimitSnapshot] {
        &self.usage
    }

    /// Returns attempted limit crossings in deterministic observation order.
    ///
    /// Unlike prose diagnostics, these events are always retained when a
    /// limit affects operation control flow, including when diagnostics are
    /// disabled.
    pub fn limit_events(&self) -> &[LimitSnapshot] {
        &self.limit_events
    }

    /// Returns stages where arithmetic resolution stopped proof/refinement.
    ///
    /// These structured stops are retained even when diagnostics are off.
    pub fn numeric_resolution_stages(&self) -> &[StageId] {
        &self.numeric_resolution_stages
    }

    /// Returns operation-local diagnostic insertion order.
    pub fn diagnostics(&self) -> &[OperationDiagnostic] {
        &self.diagnostics
    }

    /// Returns how many enabled diagnostics were omitted by the capacity bound.
    pub const fn dropped_diagnostics(&self) -> u64 {
        self.dropped_diagnostics
    }
}

/// An operation result paired with its deterministic operation report.
///
/// The error parameter defaults to [`Error`] so existing kernel-layer APIs
/// can continue to spell `OperationOutcome<T>`. Higher layers can retain
/// their own typed errors without converting them into a `kcore` error.
///
/// ```
/// use kcore::operation::{OperationContext, OperationOutcome, OperationScope, SessionPolicy};
/// use kcore::tolerance::Tolerances;
///
/// #[derive(Debug, PartialEq)]
/// struct LayerError;
///
/// let session = SessionPolicy::v1();
/// let context = OperationContext::new(&session, Tolerances::default()).unwrap();
/// let scope = OperationScope::new(&context);
/// let outcome: OperationOutcome<(), LayerError> = scope.finish_typed(Err(LayerError));
/// assert_eq!(outcome.result(), Err(&LayerError));
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct OperationOutcome<T, E = Error> {
    result: core::result::Result<T, E>,
    report: OperationReport,
}

impl<T, E> OperationOutcome<T, E> {
    /// Borrows the successful value or operation error.
    pub const fn result(&self) -> core::result::Result<&T, &E> {
        self.result.as_ref()
    }

    /// Returns the report retained on both success and failure.
    pub const fn report(&self) -> &OperationReport {
        &self.report
    }

    /// Discards the report and returns the compatibility result.
    pub fn into_result(self) -> core::result::Result<T, E> {
        self.result
    }

    /// Separates the operation result from its report.
    pub fn into_parts(self) -> (core::result::Result<T, E>, OperationReport) {
        (self.result, self.report)
    }

    /// Maps a successful value without changing the operation report.
    pub fn map<U, F>(self, op: F) -> OperationOutcome<U, E>
    where
        F: FnOnce(T) -> U,
    {
        OperationOutcome {
            result: self.result.map(op),
            report: self.report,
        }
    }

    /// Maps an operation error without changing the operation report.
    pub fn map_err<F, O>(self, op: O) -> OperationOutcome<T, F>
    where
        O: FnOnce(E) -> F,
    {
        OperationOutcome {
            result: self.result.map_err(op),
            report: self.report,
        }
    }
}
