//! Shared proof-completion evidence for bounded kernel algorithms.

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
}
