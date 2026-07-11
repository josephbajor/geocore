//! The kernel's typed error model.
//!
//! Every public kernel operation returns [`Result`]. Errors are data, not
//! strings: callers (and eventually the PK-style C API, which maps these to
//! error codes) can branch on them. The enum is `#[non_exhaustive]` and grows
//! as layers land; it never carries panics across the API boundary.

use core::fmt;

use crate::identifier::valid_identifier;
use crate::operation::{LimitSnapshot, OperationPolicyError};

/// A stable identifier failed lower-case dotted-path syntax validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IdentifierError;

impl fmt::Display for IdentifierError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("invalid namespaced identifier")
    }
}

impl std::error::Error for IdentifierError {}

/// A stable, machine-readable identity for one public failure branch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ErrorCode(&'static str);

impl ErrorCode {
    /// Validates and constructs a dotted identifier such as
    /// `kcore.input.invalid-tolerance`.
    pub const fn new(value: &'static str) -> core::result::Result<Self, IdentifierError> {
        if valid_identifier(value) {
            Ok(Self(value))
        } else {
            Err(IdentifierError)
        }
    }

    /// Returns the stable identifier text.
    pub const fn as_str(self) -> &'static str {
        self.0
    }
}

impl fmt::Display for ErrorCode {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.0)
    }
}

/// A stable identity for one finite support-matrix feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CapabilityId(&'static str);

impl CapabilityId {
    /// Validates and constructs a dotted identifier such as
    /// `xt.read.general-bodies`.
    pub const fn new(value: &'static str) -> core::result::Result<Self, IdentifierError> {
        if valid_identifier(value) {
            Ok(Self(value))
        } else {
            Err(IdentifierError)
        }
    }

    /// Returns the stable identifier text.
    pub const fn as_str(self) -> &'static str {
        self.0
    }
}

impl fmt::Display for CapabilityId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.0)
    }
}

/// Broad semantic class used for generic reporting and future ABI mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ErrorClass {
    /// The caller supplied data that violates a documented precondition.
    InvalidInput,
    /// The request is valid, but this kernel does not implement a required
    /// capability.
    Unsupported,
    /// A deterministic resource allowance was reached.
    ResourceLimit,
    /// The request is illegal in the current operation or transaction state.
    InvalidState,
    /// A checked model was rejected after a proven invariant violation.
    ModelRejected,
    /// Kernel-owned state violated an invariant callers could not establish.
    InternalInvariant,
    /// The operation was cancelled without claiming complete evidence.
    Cancelled,
}

/// Common machine-readable classification exposed by layer-local errors.
///
/// Implementations retain their concrete payloads and source chains. Wrappers
/// should delegate this view unless their public boundary changes the meaning.
pub trait ClassifiedError {
    /// Returns the broad semantic class.
    fn class(&self) -> ErrorClass;

    /// Returns the stable reason this call failed.
    fn code(&self) -> ErrorCode;

    /// Returns the unavailable capability, when unsupported work caused the
    /// failure.
    fn capability(&self) -> Option<CapabilityId> {
        None
    }

    /// Returns the structured deterministic limit, when the error uses F2's
    /// shared limit vocabulary.
    fn limit(&self) -> Option<LimitSnapshot> {
        None
    }
}

const fn known_error_code(value: &'static str) -> ErrorCode {
    match ErrorCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in error code"),
    }
}

/// Stable codes owned by existing [`Error`] branches.
pub mod code {
    use super::{ErrorCode, known_error_code};

    /// A supplied handle is stale or unknown.
    pub const STALE_HANDLE: ErrorCode = known_error_code("kcore.handle.stale");
    /// A coordinate is non-finite or outside the session size box.
    pub const OUTSIDE_SIZE_BOX: ErrorCode = known_error_code("kcore.input.outside-size-box");
    /// A tolerance violates session precision.
    pub const INVALID_TOLERANCE: ErrorCode = known_error_code("kcore.input.invalid-tolerance");
    /// A tolerance-growth budget is malformed.
    pub const INVALID_TOLERANCE_BUDGET: ErrorCode =
        known_error_code("kcore.input.invalid-tolerance-budget");
    /// Aggregate tolerance growth exceeded its allowance.
    pub const TOLERANCE_BUDGET_EXCEEDED: ErrorCode =
        known_error_code("kcore.limit.tolerance-growth");
    /// Geometry constructor inputs violate a precondition.
    pub const INVALID_GEOMETRY: ErrorCode = known_error_code("kcore.input.invalid-geometry");
    /// A legacy algorithm work/refinement allowance was reached.
    pub const ALGORITHM_LIMIT: ErrorCode = known_error_code("kcore.limit.algorithm");
    /// A deterministic resource allowance was reached with structured F2 data.
    pub const RESOURCE_LIMIT: ErrorCode = known_error_code("kcore.limit.resource");
    /// A nested modeling transaction was requested.
    pub const TRANSACTION_ACTIVE: ErrorCode = known_error_code("kcore.state.transaction-active");
    /// A transaction operation was requested without an active transaction.
    pub const TRANSACTION_INACTIVE: ErrorCode =
        known_error_code("kcore.state.transaction-inactive");
    /// A checked topology transaction was rejected.
    ///
    /// This identity is semantically owned by `ktopo` even though the legacy
    /// compatibility variant currently lives in `kcore::Error`.
    pub const TOPOLOGY_CHECK_FAILED: ErrorCode = known_error_code("ktopo.transaction.check-failed");

    /// Every code owned by the legacy shared error, in deterministic order.
    ///
    /// Operation-policy-owned codes are inventoried by
    /// [`crate::operation::code::OWNED`] and are not duplicated here. Policy
    /// limit failures delegate to [`RESOURCE_LIMIT`], so that same canonical
    /// identity intentionally appears in both variants' returned-code
    /// inventories.
    pub const ALL: &[ErrorCode] = &[
        STALE_HANDLE,
        OUTSIDE_SIZE_BOX,
        INVALID_TOLERANCE,
        INVALID_TOLERANCE_BUDGET,
        TOLERANCE_BUDGET_EXCEEDED,
        INVALID_GEOMETRY,
        ALGORITHM_LIMIT,
        RESOURCE_LIMIT,
        TRANSACTION_ACTIVE,
        TRANSACTION_INACTIVE,
        TOPOLOGY_CHECK_FAILED,
    ];
}

/// Errors produced by kernel operations.
#[derive(Debug, Clone, PartialEq)]
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
    /// A deterministic resource allowance was reached with exact F2 stage,
    /// resource, consumed, and allowed data.
    ResourceLimit {
        /// Structured snapshot of the attempted usage that crossed the limit.
        snapshot: LimitSnapshot,
    },
    /// An operation policy, configuration, or deterministic accounting request
    /// failed. This compatibility bridge retains and delegates the typed source
    /// instead of erasing it into a legacy prose or geometry error.
    OperationPolicy {
        /// The original operation-policy failure.
        source: OperationPolicyError,
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
            Error::ResourceLimit { snapshot } => write!(
                f,
                "{} {:?} usage {} exceeds {}",
                snapshot.stage.as_str(),
                snapshot.resource,
                snapshot.consumed,
                snapshot.allowed
            ),
            Error::OperationPolicy { source } => write!(f, "operation policy failed: {source}"),
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

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::OperationPolicy { source } => Some(source),
            _ => None,
        }
    }
}

impl From<OperationPolicyError> for Error {
    fn from(source: OperationPolicyError) -> Self {
        Self::OperationPolicy { source }
    }
}

impl Error {
    /// Returns this error's broad semantic class.
    pub const fn class(&self) -> ErrorClass {
        match self {
            Self::StaleHandle
            | Self::OutsideSizeBox { .. }
            | Self::InvalidTolerance { .. }
            | Self::InvalidToleranceBudget { .. }
            | Self::InvalidGeometry { .. } => ErrorClass::InvalidInput,
            Self::ToleranceBudgetExceeded { .. }
            | Self::AlgorithmLimit { .. }
            | Self::ResourceLimit { .. } => ErrorClass::ResourceLimit,
            Self::OperationPolicy { source } => source.class(),
            Self::TransactionActive | Self::TransactionInactive => ErrorClass::InvalidState,
            Self::TopologyCheckFailed { .. } => ErrorClass::ModelRejected,
        }
    }

    /// Returns this error's stable machine-readable identity.
    pub const fn code(&self) -> ErrorCode {
        match self {
            Self::StaleHandle => code::STALE_HANDLE,
            Self::OutsideSizeBox { .. } => code::OUTSIDE_SIZE_BOX,
            Self::InvalidTolerance { .. } => code::INVALID_TOLERANCE,
            Self::InvalidToleranceBudget { .. } => code::INVALID_TOLERANCE_BUDGET,
            Self::ToleranceBudgetExceeded { .. } => code::TOLERANCE_BUDGET_EXCEEDED,
            Self::InvalidGeometry { .. } => code::INVALID_GEOMETRY,
            Self::AlgorithmLimit { .. } => code::ALGORITHM_LIMIT,
            Self::ResourceLimit { .. } => code::RESOURCE_LIMIT,
            Self::OperationPolicy { source } => source.code(),
            Self::TransactionActive => code::TRANSACTION_ACTIVE,
            Self::TransactionInactive => code::TRANSACTION_INACTIVE,
            Self::TopologyCheckFailed { .. } => code::TOPOLOGY_CHECK_FAILED,
        }
    }

    /// Returns the unavailable capability, if this error is unsupported.
    ///
    /// No existing shared error variant represents unsupported work. Layer
    /// owners add typed variants during later F4 migration phases.
    pub const fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::OperationPolicy { source } => source.capability(),
            _ => None,
        }
    }

    /// Returns F2 structured limit data when present.
    ///
    /// Legacy limit variants cannot reconstruct a truthful snapshot and return
    /// `None`; the additive structured variant returns its exact F2 data.
    pub const fn limit(&self) -> Option<LimitSnapshot> {
        match self {
            Self::ResourceLimit { snapshot } => Some(*snapshot),
            Self::OperationPolicy { source } => source.limit(),
            _ => None,
        }
    }
}

impl ClassifiedError for Error {
    fn class(&self) -> ErrorClass {
        self.class()
    }

    fn code(&self) -> ErrorCode {
        self.code()
    }

    fn capability(&self) -> Option<CapabilityId> {
        self.capability()
    }

    fn limit(&self) -> Option<LimitSnapshot> {
        self.limit()
    }
}

/// Result alias used by all kernel operations.
pub type Result<T> = core::result::Result<T, Error>;

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;
    use std::error::Error as _;

    use super::*;

    #[test]
    fn stable_identifier_wrappers_validate_syntax() {
        assert_eq!(
            ErrorCode::new("kcore.input.invalid-tolerance")
                .expect("valid code")
                .as_str(),
            "kcore.input.invalid-tolerance"
        );
        assert_eq!(
            CapabilityId::new("xt.read.general-bodies")
                .expect("valid capability")
                .as_str(),
            "xt.read.general-bodies"
        );
        for invalid in [
            "not-namespaced",
            "Kcore.input.bad",
            "kcore..bad",
            "kcore.-bad",
            "kcore.bad_thing",
            "kcore.bad-",
        ] {
            assert!(ErrorCode::new(invalid).is_err(), "accepted {invalid}");
            assert!(CapabilityId::new(invalid).is_err(), "accepted {invalid}");
        }
        assert_eq!(ErrorCode::new("Bad.code").unwrap_err(), IdentifierError);
        assert_eq!(
            CapabilityId::new("bad-code").unwrap_err().to_string(),
            "invalid namespaced identifier"
        );
    }

    #[test]
    fn built_in_error_codes_are_valid_unique_and_ordered() {
        let codes: Vec<_> = code::ALL.iter().map(|code| code.as_str()).collect();
        assert_eq!(
            codes.len(),
            codes.iter().copied().collect::<BTreeSet<_>>().len()
        );
        assert_eq!(codes[0], "kcore.handle.stale");
        assert_eq!(codes.last(), Some(&"ktopo.transaction.check-failed"));
    }

    #[test]
    fn existing_error_variants_have_stable_semantic_classes() {
        let message_a = Error::InvalidGeometry { reason: "first" };
        let message_b = Error::InvalidGeometry { reason: "second" };
        assert_ne!(message_a.to_string(), message_b.to_string());
        assert_eq!(message_a.code(), message_b.code());
        assert_eq!(message_a.class(), ErrorClass::InvalidInput);

        let limit = Error::AlgorithmLimit {
            operation: "legacy",
            limit: 10,
        };
        assert_eq!(limit.class(), ErrorClass::ResourceLimit);
        assert_eq!(limit.code(), code::ALGORITHM_LIMIT);
        assert_eq!(limit.limit(), None);

        let snapshot = LimitSnapshot {
            stage: match crate::operation::StageId::new("kcore.test.structured-limit") {
                Ok(stage) => stage,
                Err(_) => panic!("valid stage"),
            },
            resource: crate::operation::ResourceKind::Work,
            consumed: 11,
            allowed: 10,
        };
        let structured = Error::ResourceLimit { snapshot };
        assert_eq!(structured.class(), ErrorClass::ResourceLimit);
        assert_eq!(structured.code(), code::RESOURCE_LIMIT);
        assert_eq!(structured.limit(), Some(snapshot));

        assert_eq!(Error::TransactionActive.class(), ErrorClass::InvalidState);
        assert_eq!(
            Error::TopologyCheckFailed { fault_count: 1 }.class(),
            ErrorClass::ModelRejected
        );
        assert_eq!(Error::StaleHandle.capability(), None);
    }

    #[test]
    fn operation_policy_bridge_preserves_source_classification_and_limit() {
        let snapshot = LimitSnapshot {
            stage: crate::operation::StageId::new("kcore.test.bridge-limit").unwrap(),
            resource: crate::operation::ResourceKind::Work,
            consumed: 9,
            allowed: 8,
        };
        let source = OperationPolicyError::LimitReached(snapshot);
        let wrapped: Error = source.clone().into();
        let structured = Error::ResourceLimit { snapshot };

        assert_eq!(wrapped.class(), source.class());
        assert_eq!(wrapped.code(), source.code());
        assert_eq!(source.code(), structured.code());
        assert_eq!(wrapped.capability(), source.capability());
        assert_eq!(wrapped.limit(), Some(snapshot));
        assert!(wrapped.to_string().contains(&source.to_string()));
        assert_eq!(
            wrapped
                .source()
                .and_then(|error| error.downcast_ref::<OperationPolicyError>()),
            Some(&source)
        );

        let validation: Error = OperationPolicyError::InvalidNumericalPolicy.into();
        assert_eq!(validation.class(), ErrorClass::InvalidInput);
        assert_eq!(validation.limit(), None);
        assert!(validation.source().is_some());
    }
}
