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

/// Classified graph-evaluation failure with no raw graph types in its public
/// representation.
///
/// The exact lower-layer error remains available through the standard error
/// source chain, while stable classification accessors delegate unchanged.
#[derive(Debug, Clone, PartialEq)]
pub struct GeometryEvaluationError {
    source: kgraph::EvalError,
}

impl GeometryEvaluationError {
    pub(crate) const fn new(source: kgraph::EvalError) -> Self {
        Self { source }
    }

    /// Returns the lower failure's broad semantic class.
    pub const fn class(&self) -> ErrorClass {
        self.source.class()
    }

    /// Returns the lower failure's stable machine-readable identity.
    pub const fn code(&self) -> ErrorCode {
        self.source.code()
    }

    /// Returns the unavailable capability when applicable.
    pub const fn capability(&self) -> Option<CapabilityId> {
        self.source.capability()
    }

    /// Returns the exact graph-recursion limit crossing when applicable.
    pub const fn limit(&self) -> Option<LimitSnapshot> {
        self.source.limit()
    }
}

impl fmt::Display for GeometryEvaluationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("geometry evaluation failed")
    }
}

impl std::error::Error for GeometryEvaluationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.source)
    }
}

impl ClassifiedError for GeometryEvaluationError {
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

/// Classified X_T interchange failure with transport details kept out of its
/// public representation.
///
/// Stable capability/classification data is available directly, while the
/// exact parse, reconstruction, or writer failure remains in the standard
/// error source chain.
#[derive(Debug, Clone, PartialEq)]
pub struct XtInterchangeError {
    source: kxt::XtError,
}

impl XtInterchangeError {
    pub(crate) const fn new(source: kxt::XtError) -> Self {
        Self { source }
    }

    /// Returns the lower failure's broad semantic class.
    pub const fn class(&self) -> ErrorClass {
        self.source.class()
    }

    /// Returns the lower failure's stable machine-readable identity.
    pub const fn code(&self) -> ErrorCode {
        self.source.code()
    }

    /// Returns the unavailable interchange capability when applicable.
    pub const fn capability(&self) -> Option<CapabilityId> {
        self.source.capability_id()
    }

    /// Returns a delegated deterministic limit crossing when applicable.
    pub const fn limit(&self) -> Option<LimitSnapshot> {
        self.source.limit()
    }
}

impl fmt::Display for XtInterchangeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("X_T interchange failed")
    }
}

impl std::error::Error for XtInterchangeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.source)
    }
}

impl ClassifiedError for XtInterchangeError {
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

/// Classified façade failure that retains any lower-layer source.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum KernelError {
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
    /// A lower kernel layer rejected an operation.
    ///
    /// Classification, stable identity, capability, and structured limit
    /// data are delegated unchanged to `source`.
    Core {
        /// Exact lower-layer failure.
        source: kcore::error::Error,
    },
    /// A bounded geometry-graph query failed.
    GeometryEvaluation {
        /// Facade-safe classified adapter retaining the exact source chain.
        source: GeometryEvaluationError,
    },
    /// X_T parsing, reconstruction, or deterministic writing failed.
    Interchange {
        /// Facade-safe classified adapter retaining the exact source chain.
        source: XtInterchangeError,
    },
}

impl fmt::Display for KernelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownPart => f.write_str("part does not belong to this session or is stale"),
            Self::WrongPart { .. } => f.write_str("entity ID belongs to a different part"),
            Self::StaleEntity { kind } => write!(f, "stale {kind:?} identity"),
            Self::InconsistentTopology { .. } => {
                f.write_str("stored topology is inconsistent during deterministic traversal")
            }
            Self::Core { source } => write!(f, "kernel operation failed: {source}"),
            Self::GeometryEvaluation { source } => source.fmt(f),
            Self::Interchange { source } => source.fmt(f),
        }
    }
}

impl std::error::Error for KernelError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InconsistentTopology { source } | Self::Core { source } => Some(source),
            Self::GeometryEvaluation { source } => Some(source),
            Self::Interchange { source } => Some(source),
            Self::UnknownPart | Self::WrongPart { .. } | Self::StaleEntity { .. } => None,
        }
    }
}

impl KernelError {
    pub(crate) const fn from_graph(source: kgraph::EvalError) -> Self {
        Self::GeometryEvaluation {
            source: GeometryEvaluationError::new(source),
        }
    }

    pub(crate) const fn from_xt(source: kxt::XtError) -> Self {
        Self::Interchange {
            source: XtInterchangeError::new(source),
        }
    }

    /// Returns this façade error's broad semantic class.
    pub const fn class(&self) -> ErrorClass {
        match self {
            Self::UnknownPart | Self::WrongPart { .. } | Self::StaleEntity { .. } => {
                ErrorClass::InvalidInput
            }
            Self::InconsistentTopology { .. } => ErrorClass::InternalInvariant,
            Self::Core { source } => source.class(),
            Self::GeometryEvaluation { source } => source.class(),
            Self::Interchange { source } => source.class(),
        }
    }

    /// Returns this façade error's stable machine-readable identity.
    pub const fn code(&self) -> ErrorCode {
        match self {
            Self::UnknownPart => code::UNKNOWN_PART,
            Self::WrongPart { .. } => code::WRONG_PART,
            Self::StaleEntity { .. } => code::STALE_ENTITY,
            Self::InconsistentTopology { .. } => code::INCONSISTENT_TOPOLOGY,
            Self::Core { source } => source.code(),
            Self::GeometryEvaluation { source } => source.code(),
            Self::Interchange { source } => source.code(),
        }
    }

    /// Returns the unavailable capability when unsupported work caused the failure.
    pub const fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::Core { source } => source.capability(),
            Self::GeometryEvaluation { source } => source.capability(),
            Self::Interchange { source } => source.capability(),
            Self::UnknownPart
            | Self::WrongPart { .. }
            | Self::StaleEntity { .. }
            | Self::InconsistentTopology { .. } => None,
        }
    }

    /// Returns structured deterministic-limit data when applicable.
    pub const fn limit(&self) -> Option<LimitSnapshot> {
        match self {
            Self::Core { source } => source.limit(),
            Self::GeometryEvaluation { source } => source.limit(),
            Self::Interchange { source } => source.limit(),
            Self::UnknownPart
            | Self::WrongPart { .. }
            | Self::StaleEntity { .. }
            | Self::InconsistentTopology { .. } => None,
        }
    }
}

impl ClassifiedError for KernelError {
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

impl From<kcore::error::Error> for KernelError {
    fn from(source: kcore::error::Error) -> Self {
        Self::Core { source }
    }
}

impl From<kcore::operation::OperationPolicyError> for KernelError {
    fn from(source: kcore::operation::OperationPolicyError) -> Self {
        kcore::error::Error::from(source).into()
    }
}

/// Backward-compatible short name for [`KernelError`].
pub type Error = KernelError;

/// Façade result for lifecycle and pre-operation failures.
pub type Result<T> = core::result::Result<T, KernelError>;

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

    #[test]
    fn core_sources_delegate_every_shared_classification_accessor() {
        let snapshot = LimitSnapshot {
            stage: kcore::operation::TOTAL_WORK_STAGE,
            resource: kcore::operation::ResourceKind::Work,
            consumed: 2,
            allowed: 1,
        };
        let source = kcore::error::Error::ResourceLimit { snapshot };
        let error = KernelError::from(source.clone());
        assert_eq!(error.class(), source.class());
        assert_eq!(error.code(), source.code());
        assert_eq!(error.capability(), source.capability());
        assert_eq!(error.limit(), source.limit());
        assert!(matches!(
            error
                .source()
                .and_then(|source| source.downcast_ref::<kcore::error::Error>()),
            Some(found) if found == &source
        ));
    }

    #[test]
    fn interchange_sources_delegate_nested_kernel_classification_and_limit() {
        let snapshot = LimitSnapshot {
            stage: kcore::operation::TOTAL_WORK_STAGE,
            resource: kcore::operation::ResourceKind::Work,
            consumed: 3,
            allowed: 2,
        };
        let nested = kcore::error::Error::ResourceLimit { snapshot };
        let source = kxt::XtError::Kernel(nested.clone());
        let error = KernelError::from_xt(source.clone());
        assert_eq!(error.class(), source.class());
        assert_eq!(error.code(), source.code());
        assert_eq!(error.capability(), source.capability_id());
        assert_eq!(error.limit(), Some(snapshot));
        let interchange = error
            .source()
            .and_then(|source| source.downcast_ref::<XtInterchangeError>())
            .unwrap();
        let xt = interchange
            .source()
            .and_then(|source| source.downcast_ref::<kxt::XtError>())
            .unwrap();
        assert_eq!(xt, &source);
        assert!(matches!(
            xt.source()
                .and_then(|source| source.downcast_ref::<kcore::error::Error>()),
            Some(found) if found == &nested
        ));
    }
}
