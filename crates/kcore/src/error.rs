//! The kernel's typed error model.
//!
//! Every public kernel operation returns [`Result`]. Errors are data, not
//! strings: callers (and eventually the PK-style C API, which maps these to
//! error codes) can branch on them. The enum is `#[non_exhaustive]` and grows
//! as layers land; it never carries panics across the API boundary.

use core::fmt;

/// Errors produced by kernel operations.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// A handle referred to an entity that was removed (or never existed).
    StaleHandle,
    /// A coordinate fell outside the ±500 m session size box or was not finite.
    OutsideSizeBox {
        /// The offending coordinate value.
        coordinate: f64,
    },
    /// A tolerance was below session resolution or not finite.
    InvalidTolerance {
        /// The offending tolerance value.
        value: f64,
    },
    /// A tolerance-growth budget limit was negative or not finite.
    InvalidToleranceBudget {
        /// The offending total-growth limit.
        limit: f64,
    },
    /// An operation attempted to enlarge entity tolerances beyond its
    /// declared aggregate growth budget.
    ToleranceBudgetExceeded {
        /// Growth requested by the current change.
        requested_growth: f64,
        /// Growth still available before the current change.
        remaining_growth: f64,
    },
    /// Geometry construction received degenerate or inconsistent inputs
    /// (zero-length axis, parallel basis hint, non-positive radius, …).
    InvalidGeometry {
        /// What made the inputs unusable.
        reason: &'static str,
    },
    /// An algorithm exhausted a deterministic work or refinement limit
    /// before it could establish the requested result.
    AlgorithmLimit {
        /// The operation and stage that reached its limit.
        operation: &'static str,
        /// The configured limit.
        limit: usize,
    },
    /// A modeling transaction was requested while another transaction is
    /// already active on the same store. Nested modeling transactions are
    /// deliberately rejected until their journal-composition semantics are
    /// part of the public contract.
    TransactionActive,
    /// A transaction commit or rollback was requested without a matching
    /// active transaction frame.
    TransactionInactive,
    /// A checked modeling transaction produced one or more body-checker or
    /// store topology-ownership faults. The transaction is rolled back before
    /// this error is returned.
    TopologyCheckFailed {
        /// Total proven faults across all live topology.
        fault_count: usize,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::StaleHandle => write!(f, "handle refers to a removed or unknown entity"),
            Error::OutsideSizeBox { coordinate } => write!(
                f,
                "coordinate {coordinate} lies outside the ±500 m session size box"
            ),
            Error::InvalidTolerance { value } => write!(
                f,
                "tolerance {value} is below session resolution or not finite"
            ),
            Error::InvalidToleranceBudget { limit } => write!(
                f,
                "tolerance-growth budget {limit} is negative or not finite"
            ),
            Error::ToleranceBudgetExceeded {
                requested_growth,
                remaining_growth,
            } => write!(
                f,
                "tolerance growth {requested_growth} exceeds remaining budget {remaining_growth}"
            ),
            Error::InvalidGeometry { reason } => {
                write!(f, "invalid geometry construction: {reason}")
            }
            Error::AlgorithmLimit { operation, limit } => {
                write!(f, "{operation} exceeded its limit of {limit}")
            }
            Error::TransactionActive => {
                write!(f, "a modeling transaction is already active on this store")
            }
            Error::TransactionInactive => write!(f, "no transaction is active"),
            Error::TopologyCheckFailed { fault_count } => {
                write!(
                    f,
                    "checked topology commit rejected {fault_count} invariant fault(s)"
                )
            }
        }
    }
}

impl std::error::Error for Error {}

/// Result alias used by all kernel operations.
pub type Result<T> = core::result::Result<T, Error>;
