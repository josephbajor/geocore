//! Typed whole-body tessellation failure boundary.

use core::fmt;

use crate::entity::SurfaceId;
use kcore::error::{CapabilityId, ClassifiedError, Error, ErrorClass, ErrorCode};
use kcore::operation::LimitSnapshot;
use kgeom::surface_point::SurfacePointContextError;
use kgraph::EvalError;

const fn error_code(value: &'static str) -> ErrorCode {
    match ErrorCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in tessellation error code"),
    }
}

const fn capability_id(value: &'static str) -> CapabilityId {
    match CapabilityId::new(value) {
        Ok(capability) => capability,
        Err(_) => panic!("invalid built-in tessellation capability identifier"),
    }
}

/// Stable identity for graph evaluation failures during tessellation.
pub const EVALUATION_FAILED: ErrorCode = error_code("ktopo.tessellation.evaluation-failed");
/// Stable identity for a valid representation outside the finite support matrix.
pub const UNSUPPORTED_TESSELLATION: ErrorCode = error_code("ktopo.tessellation.unsupported");
/// Stable identity for an unresolved whole-cell regularity proof.
pub const REGULARITY_INDETERMINATE: ErrorCode =
    error_code("ktopo.tessellation.regularity-indeterminate");

/// Leaf-only legacy tessellation algorithm capability.
pub const PROCEDURAL_LEAF_ALGORITHM: CapabilityId =
    capability_id("ktopo.tessellation.procedural-leaf-algorithm");
/// Offset loops that wind a periodic parameter direction.
pub const OFFSET_PERIODIC_WINDING: CapabilityId =
    capability_id("ktopo.tessellation.offset-periodic-winding");
/// Whole-cell surface regularity certification.
pub const SURFACE_REGULARITY_PROOF: CapabilityId =
    capability_id("ktopo.tessellation.surface-regularity-proof");

/// A topology tessellation failure that preserves graph evaluation outcomes.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum TessellationError {
    /// Existing topology, trim, or tessellation input failure.
    Kernel(Error),
    /// Exact graph evaluation failed, including singular and ill-conditioned offsets.
    Evaluation(EvalError),
    /// Contextual point-to-surface inversion or projection failed.
    SurfacePoint(SurfacePointContextError),
    /// This tessellation path does not implement the requested valid representation.
    Unsupported {
        /// Stable finite-support capability.
        capability: CapabilityId,
    },
    /// Tessellation could not certify the surface regular over the full cell.
    Indeterminate {
        /// Surface whose regularity proof is absent.
        surface: SurfaceId,
        /// Pointwise conditioning result that caused the gap, when present.
        source: Option<EvalError>,
    },
}

impl TessellationError {
    /// Broad failure class without erasing the concrete payload.
    pub fn class(&self) -> ErrorClass {
        match self {
            Self::Kernel(error) => error.class(),
            Self::SurfacePoint(error) => error.class(),
            Self::Evaluation(EvalError::DependencyDepthExceeded { .. })
            | Self::Evaluation(EvalError::NodeVisitLimitExceeded { .. }) => {
                ErrorClass::ResourceLimit
            }
            Self::Unsupported { .. } | Self::Indeterminate { .. } => ErrorClass::Unsupported,
            Self::Evaluation(EvalError::InvalidParameter)
            | Self::Evaluation(EvalError::ParameterOutsideDomain)
            | Self::Evaluation(EvalError::InvalidRange)
            | Self::Evaluation(EvalError::StaleGeometryHandle { .. }) => ErrorClass::InvalidInput,
            Self::Evaluation(EvalError::DerivativeUnavailable { .. })
            | Self::Evaluation(EvalError::IllConditionedSurface { .. }) => ErrorClass::Unsupported,
            Self::Evaluation(EvalError::DependencyCycle { .. }) => ErrorClass::InternalInvariant,
            Self::Evaluation(EvalError::SingularSurface { .. })
            | Self::Evaluation(EvalError::NonFiniteResult { .. }) => ErrorClass::ModelRejected,
            Self::Evaluation(_) => ErrorClass::InternalInvariant,
        }
    }

    /// Stable failure identity.
    pub fn code(&self) -> ErrorCode {
        match self {
            Self::Kernel(error) => error.code(),
            Self::SurfacePoint(error) => error.code(),
            Self::Evaluation(EvalError::IllConditionedSurface { .. }) => REGULARITY_INDETERMINATE,
            Self::Evaluation(_) => EVALUATION_FAILED,
            Self::Unsupported { .. } => UNSUPPORTED_TESSELLATION,
            Self::Indeterminate { .. } => REGULARITY_INDETERMINATE,
        }
    }

    /// Finite capability whose absence caused this failure, when applicable.
    pub fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::Kernel(error) => error.capability(),
            Self::SurfacePoint(error) => error.capability(),
            Self::Unsupported { capability } => Some(*capability),
            Self::Indeterminate { .. } => Some(SURFACE_REGULARITY_PROOF),
            Self::Evaluation(EvalError::IllConditionedSurface { .. }) => {
                Some(SURFACE_REGULARITY_PROOF)
            }
            Self::Evaluation(_) => None,
        }
    }

    /// Structured limit data, delegated from kernel sources.
    ///
    /// `EvalError` predates the shared F2 limit snapshot and therefore cannot
    /// reconstruct a truthful stage/resource record yet.
    pub fn limit(&self) -> Option<LimitSnapshot> {
        match self {
            Self::Kernel(error) => error.limit(),
            Self::SurfacePoint(error) => error.limit(),
            _ => None,
        }
    }
}

impl fmt::Display for TessellationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Kernel(error) => error.fmt(formatter),
            Self::SurfacePoint(error) => error.fmt(formatter),
            Self::Evaluation(error) => error.fmt(formatter),
            Self::Unsupported { capability } => {
                write!(
                    formatter,
                    "unsupported tessellation capability: {capability}"
                )
            }
            Self::Indeterminate { surface, source } => {
                if let Some(source) = source {
                    write!(
                        formatter,
                        "surface regularity is indeterminate for {surface:?}: {source}"
                    )
                } else {
                    write!(
                        formatter,
                        "surface regularity is indeterminate for {surface:?}"
                    )
                }
            }
        }
    }
}

impl std::error::Error for TessellationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Kernel(error) => Some(error),
            Self::SurfacePoint(error) => Some(error),
            Self::Evaluation(error) => Some(error),
            Self::Indeterminate {
                source: Some(error),
                ..
            } => Some(error),
            Self::Unsupported { .. } | Self::Indeterminate { source: None, .. } => None,
        }
    }
}

impl ClassifiedError for TessellationError {
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

impl From<Error> for TessellationError {
    fn from(error: Error) -> Self {
        Self::Kernel(error)
    }
}

impl From<EvalError> for TessellationError {
    fn from(error: EvalError) -> Self {
        if let EvalError::IllConditionedSurface { surface, .. } = error {
            Self::Indeterminate {
                surface,
                source: Some(error),
            }
        } else {
            Self::Evaluation(error)
        }
    }
}

impl From<SurfacePointContextError> for TessellationError {
    fn from(error: SurfacePointContextError) -> Self {
        Self::SurfacePoint(error)
    }
}

/// Result returned by whole-body tessellation.
pub type TessellationResult<T> = core::result::Result<T, TessellationError>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::store::Store;
    use kgeom::frame::Frame;
    use kgeom::surface::Plane;
    use std::error::Error as _;

    #[test]
    fn preserves_classification_capability_and_source() {
        let kernel = TessellationError::from(Error::StaleHandle);
        assert_eq!(ClassifiedError::code(&kernel), Error::StaleHandle.code());
        assert!(kernel.source().is_some());

        let evaluation = TessellationError::from(EvalError::InvalidParameter);
        assert_eq!(ClassifiedError::code(&evaluation), EVALUATION_FAILED);
        assert_eq!(
            ClassifiedError::class(&evaluation),
            ErrorClass::InvalidInput
        );
        assert!(evaluation.source().is_some());

        let surface_point =
            TessellationError::from(SurfacePointContextError::UnboundedProjectionWindow);
        assert_eq!(
            ClassifiedError::class(&surface_point),
            ErrorClass::Unsupported
        );
        assert_eq!(
            ClassifiedError::capability(&surface_point),
            Some(kgeom::surface_point::capability::FINITE_PROJECTION_WINDOW)
        );
        assert!(surface_point.source().is_some());

        let unsupported = TessellationError::Unsupported {
            capability: PROCEDURAL_LEAF_ALGORITHM,
        };
        assert_eq!(
            ClassifiedError::code(&unsupported),
            UNSUPPORTED_TESSELLATION
        );
        assert_eq!(
            ClassifiedError::capability(&unsupported),
            Some(PROCEDURAL_LEAF_ALGORITHM)
        );
        assert!(unsupported.source().is_none());

        let mut store = Store::new();
        let surface = store
            .insert_surface(Plane::new(Frame::world()).into())
            .unwrap();
        let regularity = TessellationError::Indeterminate {
            surface,
            source: None,
        };
        assert_eq!(ClassifiedError::code(&regularity), REGULARITY_INDETERMINATE);
        assert_eq!(
            ClassifiedError::capability(&regularity),
            Some(SURFACE_REGULARITY_PROOF)
        );
        assert_eq!(ClassifiedError::class(&regularity), ErrorClass::Unsupported);

        let conditioning = TessellationError::from(EvalError::IllConditionedSurface {
            surface,
            uv: [0.25, 0.75],
        });
        assert_eq!(conditioning.class(), ErrorClass::Unsupported);
        assert_eq!(conditioning.code(), REGULARITY_INDETERMINATE);
        assert_eq!(conditioning.capability(), Some(SURFACE_REGULARITY_PROOF));
        assert!(conditioning.source().is_some());

        let direct = TessellationError::Evaluation(EvalError::IllConditionedSurface {
            surface,
            uv: [0.0, 0.0],
        });
        assert_eq!(direct.class(), ErrorClass::Unsupported);
        assert_eq!(direct.code(), REGULARITY_INDETERMINATE);
        assert_eq!(direct.capability(), Some(SURFACE_REGULARITY_PROOF));
    }
}
