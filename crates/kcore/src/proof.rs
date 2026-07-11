//! Shared proof-completion evidence for bounded kernel algorithms.

use crate::error::CapabilityId;
use crate::operation::{DiagnosticCode, LimitSnapshot, StageId};

/// Structured cause of an unresolved proof obligation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum IncompleteCause {
    /// A valid feature needed to complete the operation is unsupported.
    Unsupported {
        /// Smallest stable unavailable support-matrix feature.
        capability: CapabilityId,
    },
    /// A deterministic configured allowance was reached.
    Limit {
        /// Exact stage, resource, attempted usage, and allowance.
        snapshot: LimitSnapshot,
    },
    /// Floating-point resolution stopped progress without proving the
    /// outstanding obligation.
    NumericResolution,
    /// External cancellation stopped the proof.
    Cancelled,
    /// The implementation has no complete proof method for this valid case.
    ProofMethodUnavailable {
        /// Smallest stable unavailable proof capability.
        capability: CapabilityId,
    },
}

/// One stable, machine-readable explanation for incomplete proof evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IncompleteEvidence {
    /// Stable identity of the incomplete-proof observation.
    pub code: DiagnosticCode,
    /// Deterministic operation stage where the obligation remained open.
    pub stage: StageId,
    /// Structured reason the obligation could not be discharged.
    pub cause: IncompleteCause,
    /// Non-stable human-readable context; callers must not parse it.
    pub message: &'static str,
}

/// Whether an algorithm established a complete result over its requested
/// domain or returned only verified partial evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Completion {
    /// All obligations over the requested domain were discharged. An empty
    /// result carrying this status is a proven miss.
    Complete,
    /// Returned entities are individually verified, but the algorithm did
    /// not prove that no additional entities exist.
    Indeterminate {
        /// Stable diagnostic describing the missing completion evidence.
        reason: &'static str,
    },
}

impl Completion {
    /// True only when the complete requested domain was covered by proof.
    pub fn is_complete(self) -> bool {
        matches!(self, Self::Complete)
    }

    /// Diagnostic reason when completion remains indeterminate.
    pub fn indeterminate_reason(self) -> Option<&'static str> {
        match self {
            Self::Complete => None,
            Self::Indeterminate { reason } => Some(reason),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn completion_keeps_unknown_distinct_from_proven_empty() {
        assert!(Completion::Complete.is_complete());
        let unknown = Completion::Indeterminate {
            reason: "candidate isolation is incomplete",
        };
        assert!(!unknown.is_complete());
        assert_eq!(
            unknown.indeterminate_reason(),
            Some("candidate isolation is incomplete")
        );
    }

    #[test]
    fn incomplete_evidence_keeps_limit_data_structured() {
        use crate::operation::{ResourceKind, TOTAL_WORK_STAGE};

        let snapshot = LimitSnapshot {
            stage: TOTAL_WORK_STAGE,
            resource: ResourceKind::Work,
            consumed: 3,
            allowed: 2,
        };
        let code = DiagnosticCode::new("kcore.test.proof-incomplete").unwrap();
        let evidence = IncompleteEvidence {
            code,
            stage: snapshot.stage,
            cause: IncompleteCause::Limit { snapshot },
            message: "display-only context",
        };
        assert_eq!(evidence.cause, IncompleteCause::Limit { snapshot });
    }
}
