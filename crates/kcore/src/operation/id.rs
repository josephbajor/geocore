//! Stable operation identifiers and configuration/accounting errors.

use std::{error, fmt};

use crate::{
    error::{CapabilityId, ClassifiedError, ErrorClass, ErrorCode},
    identifier::valid_identifier,
};

use super::budget::{LimitSnapshot, ResourceKind};

/// The version of policy defaults that can affect deterministic output.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum PolicyVersion {
    /// Initial operation-policy defaults.
    V1,
}

/// Why a policy, identifier, or accounting request was rejected.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum OperationPolicyError {
    /// A stable identifier was not a valid namespaced identifier.
    InvalidIdentifier,
    /// Session precision contained a non-finite or non-positive value.
    InvalidSessionPrecision,
    /// A numerical-policy factor was non-finite or outside its valid range.
    InvalidNumericalPolicy,
    /// Operation tolerances were below the selected session precision.
    InvalidOperationTolerance,
    /// A plan contains two limits for the same stage and resource.
    DuplicateLimit {
        /// The duplicated stage.
        stage: StageId,
        /// The duplicated resource.
        resource: ResourceKind,
    },
    /// A limit uses an accounting mode that does not match its resource.
    InvalidLimitMode {
        /// The affected stage.
        stage: StageId,
        /// The affected resource.
        resource: ResourceKind,
    },
    /// A requested stage/resource pair is not present in the plan.
    UnknownLimit {
        /// The requested stage.
        stage: StageId,
        /// The requested resource.
        resource: ResourceKind,
    },
    /// The requested accounting operation did not match the configured mode.
    AccountingModeMismatch {
        /// The affected stage.
        stage: StageId,
        /// The affected resource.
        resource: ResourceKind,
    },
    /// A deterministic limit was exceeded.
    LimitReached(LimitSnapshot),
    /// Unsigned accounting overflowed before a meaningful usage value existed.
    AccountingOverflow {
        /// The affected stage.
        stage: StageId,
        /// The affected resource.
        resource: ResourceKind,
    },
    /// Child work ordinals must be unique and reserved in increasing order.
    InvalidChildOrdinal,
    /// A child reservation did not fit in the parent's remaining allowance.
    ChildReservationExceeded {
        /// The affected stage.
        stage: StageId,
        /// The affected resource.
        resource: ResourceKind,
    },
    /// A returned child ledger has no matching active reservation.
    UnknownChildReservation,
}

const fn known_error_code(value: &'static str) -> ErrorCode {
    match ErrorCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in operation-policy error code"),
    }
}

/// Stable machine-readable identities returned by [`OperationPolicyError`].
pub mod code {
    use super::{ErrorCode, known_error_code};

    /// A stable operation identifier failed syntax validation.
    pub const INVALID_IDENTIFIER: ErrorCode =
        known_error_code("kcore.operation.invalid-identifier");
    /// Session precision contains an invalid value.
    pub const INVALID_SESSION_PRECISION: ErrorCode =
        known_error_code("kcore.operation.invalid-session-precision");
    /// Numerical policy contains an invalid factor.
    pub const INVALID_NUMERICAL_POLICY: ErrorCode =
        known_error_code("kcore.operation.invalid-numerical-policy");
    /// Operation tolerance is incompatible with session precision.
    pub const INVALID_OPERATION_TOLERANCE: ErrorCode =
        known_error_code("kcore.operation.invalid-tolerance");
    /// A budget plan declares a stage/resource limit more than once.
    pub const DUPLICATE_LIMIT: ErrorCode = known_error_code("kcore.operation.duplicate-limit");
    /// A budget limit uses the wrong accounting mode for its resource.
    pub const INVALID_LIMIT_MODE: ErrorCode =
        known_error_code("kcore.operation.invalid-limit-mode");
    /// A requested stage/resource limit is not configured.
    pub const UNKNOWN_LIMIT: ErrorCode = known_error_code("kcore.operation.unknown-limit");
    /// An accounting request does not match the configured resource mode.
    pub const ACCOUNTING_MODE_MISMATCH: ErrorCode =
        known_error_code("kcore.operation.accounting-mode-mismatch");
    /// A deterministic operation-policy limit was reached.
    ///
    /// This delegates to the one canonical structured-resource-limit identity
    /// shared with [`crate::error::Error::ResourceLimit`].
    pub const LIMIT_REACHED: ErrorCode = crate::error::code::RESOURCE_LIMIT;
    /// Unsigned work accounting overflowed.
    pub const ACCOUNTING_OVERFLOW: ErrorCode =
        known_error_code("kcore.operation.accounting-overflow");
    /// Child-ledger reservations were not made in valid ordinal order.
    pub const INVALID_CHILD_ORDINAL: ErrorCode =
        known_error_code("kcore.operation.invalid-child-ordinal");
    /// A child reservation is incompatible with the parent's remaining capacity.
    pub const CHILD_RESERVATION_EXCEEDED: ErrorCode =
        known_error_code("kcore.operation.child-reservation-exceeded");
    /// A returned child ledger has no active reservation.
    pub const UNKNOWN_CHILD_RESERVATION: ErrorCode =
        known_error_code("kcore.operation.unknown-child-reservation");

    /// Codes owned by operation policy, excluding delegated shared identities.
    pub const OWNED: &[ErrorCode] = &[
        INVALID_IDENTIFIER,
        INVALID_SESSION_PRECISION,
        INVALID_NUMERICAL_POLICY,
        INVALID_OPERATION_TOLERANCE,
        DUPLICATE_LIMIT,
        INVALID_LIMIT_MODE,
        UNKNOWN_LIMIT,
        ACCOUNTING_MODE_MISMATCH,
        ACCOUNTING_OVERFLOW,
        INVALID_CHILD_ORDINAL,
        CHILD_RESERVATION_EXCEEDED,
        UNKNOWN_CHILD_RESERVATION,
    ];

    /// Every code returned by operation policy, in variant declaration order.
    ///
    /// This includes [`LIMIT_REACHED`], the intentionally delegated canonical
    /// structured-resource-limit code owned by [`crate::error::code`].
    pub const ALL: &[ErrorCode] = &[
        INVALID_IDENTIFIER,
        INVALID_SESSION_PRECISION,
        INVALID_NUMERICAL_POLICY,
        INVALID_OPERATION_TOLERANCE,
        DUPLICATE_LIMIT,
        INVALID_LIMIT_MODE,
        UNKNOWN_LIMIT,
        ACCOUNTING_MODE_MISMATCH,
        LIMIT_REACHED,
        ACCOUNTING_OVERFLOW,
        INVALID_CHILD_ORDINAL,
        CHILD_RESERVATION_EXCEEDED,
        UNKNOWN_CHILD_RESERVATION,
    ];
}

impl OperationPolicyError {
    /// Returns this policy failure's broad semantic class.
    ///
    /// Reservation failures are state errors: they describe an invalid child
    /// ledger lifecycle or a reservation that is illegal given capacity
    /// already held by the current ledger. They deliberately do not claim a
    /// resource-limit event because they carry no [`LimitSnapshot`]. An
    /// accounting overflow is invalid input at the current public
    /// [`super::WorkLedger`] boundary because callers directly select charge
    /// amounts and budget allowances that can trigger it.
    pub const fn class(&self) -> ErrorClass {
        match self {
            Self::InvalidIdentifier
            | Self::InvalidSessionPrecision
            | Self::InvalidNumericalPolicy
            | Self::InvalidOperationTolerance
            | Self::DuplicateLimit { .. }
            | Self::InvalidLimitMode { .. }
            | Self::UnknownLimit { .. }
            | Self::AccountingModeMismatch { .. } => ErrorClass::InvalidInput,
            Self::LimitReached(_) => ErrorClass::ResourceLimit,
            Self::AccountingOverflow { .. } => ErrorClass::InvalidInput,
            Self::InvalidChildOrdinal
            | Self::ChildReservationExceeded { .. }
            | Self::UnknownChildReservation => ErrorClass::InvalidState,
        }
    }

    /// Returns this policy failure's stable machine-readable identity.
    pub const fn code(&self) -> ErrorCode {
        match self {
            Self::InvalidIdentifier => code::INVALID_IDENTIFIER,
            Self::InvalidSessionPrecision => code::INVALID_SESSION_PRECISION,
            Self::InvalidNumericalPolicy => code::INVALID_NUMERICAL_POLICY,
            Self::InvalidOperationTolerance => code::INVALID_OPERATION_TOLERANCE,
            Self::DuplicateLimit { .. } => code::DUPLICATE_LIMIT,
            Self::InvalidLimitMode { .. } => code::INVALID_LIMIT_MODE,
            Self::UnknownLimit { .. } => code::UNKNOWN_LIMIT,
            Self::AccountingModeMismatch { .. } => code::ACCOUNTING_MODE_MISMATCH,
            Self::LimitReached(_) => code::LIMIT_REACHED,
            Self::AccountingOverflow { .. } => code::ACCOUNTING_OVERFLOW,
            Self::InvalidChildOrdinal => code::INVALID_CHILD_ORDINAL,
            Self::ChildReservationExceeded { .. } => code::CHILD_RESERVATION_EXCEEDED,
            Self::UnknownChildReservation => code::UNKNOWN_CHILD_RESERVATION,
        }
    }

    /// Returns the exact deterministic-limit snapshot when one exists.
    pub const fn limit(&self) -> Option<LimitSnapshot> {
        match self {
            Self::LimitReached(snapshot) => Some(*snapshot),
            _ => None,
        }
    }

    /// Operation-policy failures do not represent unsupported capabilities.
    pub const fn capability(&self) -> Option<CapabilityId> {
        None
    }
}

impl fmt::Display for OperationPolicyError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidIdentifier => formatter.write_str("invalid namespaced identifier"),
            Self::InvalidSessionPrecision => formatter.write_str("invalid session precision"),
            Self::InvalidNumericalPolicy => formatter.write_str("invalid numerical policy"),
            Self::InvalidOperationTolerance => {
                formatter.write_str("operation tolerance is below session precision")
            }
            Self::DuplicateLimit { stage, resource } => write!(
                formatter,
                "duplicate {:?} limit for {}",
                resource,
                stage.as_str()
            ),
            Self::InvalidLimitMode { stage, resource } => write!(
                formatter,
                "invalid accounting mode for {:?} at {}",
                resource,
                stage.as_str()
            ),
            Self::UnknownLimit { stage, resource } => write!(
                formatter,
                "unknown {:?} limit at {}",
                resource,
                stage.as_str()
            ),
            Self::AccountingModeMismatch { stage, resource } => write!(
                formatter,
                "accounting mode mismatch for {:?} at {}",
                resource,
                stage.as_str()
            ),
            Self::LimitReached(snapshot) => write!(
                formatter,
                "{} {:?} usage {} exceeds {}",
                snapshot.stage.as_str(),
                snapshot.resource,
                snapshot.consumed,
                snapshot.allowed
            ),
            Self::AccountingOverflow { stage, resource } => write!(
                formatter,
                "{:?} accounting overflow at {}",
                resource,
                stage.as_str()
            ),
            Self::InvalidChildOrdinal => {
                formatter.write_str("child work ordinals must be unique and increasing")
            }
            Self::ChildReservationExceeded { stage, resource } => write!(
                formatter,
                "child reservation exceeds {:?} allowance at {}",
                resource,
                stage.as_str()
            ),
            Self::UnknownChildReservation => {
                formatter.write_str("child ledger has no matching reservation")
            }
        }
    }
}

impl error::Error for OperationPolicyError {}

impl ClassifiedError for OperationPolicyError {
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

/// A stable, namespaced identifier for one deterministic work stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct StageId(&'static str);

impl StageId {
    /// Validates and constructs an identifier such as `kgeom.project.newton`.
    pub const fn new(value: &'static str) -> core::result::Result<Self, OperationPolicyError> {
        if valid_identifier(value) {
            Ok(Self(value))
        } else {
            Err(OperationPolicyError::InvalidIdentifier)
        }
    }

    /// Returns the stable identifier text.
    pub const fn as_str(self) -> &'static str {
        self.0
    }
}

/// A stable, namespaced identifier for a semantic diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DiagnosticCode(&'static str);

impl DiagnosticCode {
    /// Validates and constructs an identifier such as `kgeom.project.fallback`.
    pub const fn new(value: &'static str) -> core::result::Result<Self, OperationPolicyError> {
        if valid_identifier(value) {
            Ok(Self(value))
        } else {
            Err(OperationPolicyError::InvalidIdentifier)
        }
    }

    /// Returns the stable identifier text.
    pub const fn as_str(self) -> &'static str {
        self.0
    }
}
