//! Façade-owned lifecycle and identity failures.

use core::fmt;

use kcore::error::{CapabilityId, ClassifiedError, ErrorClass, ErrorCode};
use kcore::operation::LimitSnapshot;

use crate::PartId;

const fn known_error_code(value: &'static str) -> ErrorCode {
    match ErrorCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in kernel facade error code"),
    }
}

/// Stable machine-readable identities owned by the native façade.
pub mod code {
    use super::{ErrorCode, known_error_code};

    /// A part ID does not resolve in the receiving session.
    pub const UNKNOWN_PART: ErrorCode = known_error_code("kernel.part.unknown");
    /// An entity ID belongs to a different part.
    pub const WRONG_PART: ErrorCode = known_error_code("kernel.entity.wrong-part");
    /// An entity ID no longer resolves to a live lower-layer entity.
    pub const STALE_ENTITY: ErrorCode = known_error_code("kernel.entity.stale");
    /// A stored relationship was inconsistent during a façade read.
    pub const INCONSISTENT_TOPOLOGY: ErrorCode = known_error_code("kernel.topology.inconsistent");

    /// Every code currently owned by the façade, in deterministic order.
    pub const ALL: &[ErrorCode] = &[
        UNKNOWN_PART,
        WRONG_PART,
        STALE_ENTITY,
        INCONSISTENT_TOPOLOGY,
    ];
}

/// Facade identity kind used in stale-ID diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum EntityKind {
    /// Body.
    Body,
    /// Region.
    Region,
    /// Shell.
    Shell,
    /// Face.
    Face,
    /// Loop.
    Loop,
    /// Fin.
    Fin,
    /// Edge.
    Edge,
    /// Vertex.
    Vertex,
    /// Three-dimensional curve geometry.
    Curve,
    /// Supporting-surface geometry.
    Surface,
    /// Parameter-space curve geometry.
    Pcurve,
}

/// Façade lifecycle, identity, or read failure.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// A part ID belongs to another session or no longer resolves.
    UnknownPart,
    /// An entity ID was presented to a different part.
    WrongPart {
        /// Part receiving the request.
        expected: PartId,
        /// Part embedded in the entity ID.
        actual: PartId,
    },
    /// The embedded lower-layer generation no longer identifies a live entity.
    StaleEntity {
        /// Entity kind that failed to resolve.
        kind: EntityKind,
    },
    /// A deterministic read traversal encountered an invalid stored relationship.
    InconsistentTopology {
        /// Exact lower-layer failure encountered while following the relationship.
        source: kcore::error::Error,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownPart => f.write_str("part does not belong to this session or is stale"),
            Self::WrongPart { .. } => f.write_str("entity ID belongs to a different part"),
            Self::StaleEntity { kind } => write!(f, "stale {kind:?} identity"),
            Self::InconsistentTopology { .. } => {
                f.write_str("stored topology is inconsistent during deterministic traversal")
            }
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InconsistentTopology { source } => Some(source),
            Self::UnknownPart | Self::WrongPart { .. } | Self::StaleEntity { .. } => None,
        }
    }
}

impl Error {
    /// Returns this façade error's broad semantic class.
    pub const fn class(&self) -> ErrorClass {
        match self {
            Self::UnknownPart | Self::WrongPart { .. } | Self::StaleEntity { .. } => {
                ErrorClass::InvalidInput
            }
            Self::InconsistentTopology { .. } => ErrorClass::InternalInvariant,
        }
    }

    /// Returns this façade error's stable machine-readable identity.
    pub const fn code(&self) -> ErrorCode {
        match self {
            Self::UnknownPart => code::UNKNOWN_PART,
            Self::WrongPart { .. } => code::WRONG_PART,
            Self::StaleEntity { .. } => code::STALE_ENTITY,
            Self::InconsistentTopology { .. } => code::INCONSISTENT_TOPOLOGY,
        }
    }

    /// Returns the unavailable capability when unsupported work caused the failure.
    pub const fn capability(&self) -> Option<CapabilityId> {
        None
    }

    /// Returns structured deterministic-limit data when applicable.
    pub const fn limit(&self) -> Option<LimitSnapshot> {
        None
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

/// K1 façade result.
pub type Result<T> = core::result::Result<T, Error>;

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;
    use std::error::Error as _;

    use super::*;

    #[test]
    fn facade_codes_are_valid_unique_and_classified() {
        let unique = code::ALL
            .iter()
            .map(|code| code.as_str())
            .collect::<BTreeSet<_>>();
        assert_eq!(unique.len(), code::ALL.len());
        assert!(unique.iter().all(|value| value.starts_with("kernel.")));

        assert_eq!(Error::UnknownPart.class(), ErrorClass::InvalidInput);
        assert_eq!(Error::UnknownPart.code(), code::UNKNOWN_PART);
        assert_eq!(
            Error::StaleEntity {
                kind: EntityKind::Face,
            }
            .code(),
            code::STALE_ENTITY
        );
    }

    #[test]
    fn inconsistent_topology_retains_the_lower_failure_as_its_source() {
        let error = Error::InconsistentTopology {
            source: kcore::error::Error::StaleHandle,
        };
        assert_eq!(error.class(), ErrorClass::InternalInvariant);
        assert_eq!(error.code(), code::INCONSISTENT_TOPOLOGY);
        assert!(matches!(
            error.source().and_then(|source| source.downcast_ref()),
            Some(kcore::error::Error::StaleHandle)
        ));
    }
}
