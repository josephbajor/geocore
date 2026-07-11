//! Stable operation identifiers and configuration/accounting errors.

use std::{error, fmt};

use super::budget::{LimitSnapshot, ResourceKind};

/// The version of policy defaults that can affect deterministic output.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum PolicyVersion {
    /// Initial operation-policy defaults.
    V1,
}

/// Why a policy, identifier, or accounting request was rejected.
#[derive(Debug, Clone, Copy, PartialEq)]
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

const fn valid_identifier(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() < 3 {
        return false;
    }
    let mut index = 0;
    let mut has_namespace_separator = false;
    let mut previous_was_separator = true;
    while index < bytes.len() {
        let byte = bytes[index];
        let is_alphanumeric = byte.is_ascii_lowercase() || byte.is_ascii_digit();
        if is_alphanumeric {
            previous_was_separator = false;
        } else if byte == b'.' {
            if previous_was_separator {
                return false;
            }
            has_namespace_separator = true;
            previous_was_separator = true;
        } else if byte == b'-' {
            if previous_was_separator {
                return false;
            }
            previous_was_separator = true;
        } else {
            return false;
        }
        index += 1;
    }
    has_namespace_separator && !previous_was_separator
}
