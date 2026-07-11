//! Typed graph construction and evaluation failures.

use core::fmt;

use kcore::error::{CapabilityId, ClassifiedError, ErrorClass, ErrorCode};
use kcore::operation::{LimitSnapshot, ResourceKind, StageId};

use crate::{GeometryClassKey, GeometryRef, SurfaceHandle};

const fn known_error_code(value: &'static str) -> ErrorCode {
    match ErrorCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in kgraph error code"),
    }
}

const fn known_capability(value: &'static str) -> CapabilityId {
    match CapabilityId::new(value) {
        Ok(capability) => capability,
        Err(_) => panic!("invalid built-in kgraph capability identifier"),
    }
}

const fn known_stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid built-in kgraph stage identifier"),
    }
}

/// Stable identities for bounded graph-evaluation failure branches.
pub mod code {
    use super::{ErrorCode, known_error_code};

    /// The requested graph handle is stale.
    pub const STALE_GEOMETRY_HANDLE: ErrorCode =
        known_error_code("kgraph.eval.stale-geometry-handle");
    /// A scalar or vector query parameter is non-finite.
    pub const INVALID_PARAMETER: ErrorCode = known_error_code("kgraph.eval.invalid-parameter");
    /// A finite query parameter is outside the leaf domain.
    pub const PARAMETER_OUTSIDE_DOMAIN: ErrorCode =
        known_error_code("kgraph.eval.parameter-outside-domain");
    /// A requested parameter range is malformed or outside the leaf domain.
    pub const INVALID_RANGE: ErrorCode = known_error_code("kgraph.eval.invalid-range");
    /// Evaluation observed a dependency cycle after graph validation.
    pub const DEPENDENCY_CYCLE: ErrorCode = known_error_code("kgraph.eval.dependency-cycle");
    /// The configured per-query dependency-depth allowance was crossed.
    pub const DEPENDENCY_DEPTH: ErrorCode = known_error_code("kgraph.eval.dependency-depth-limit");
    /// The configured per-query node-visit allowance was crossed.
    pub const NODE_VISITS: ErrorCode = known_error_code("kgraph.eval.node-visit-limit");
    /// The requested surface point is singular.
    pub const SINGULAR_SURFACE: ErrorCode = known_error_code("kgraph.eval.singular-surface");
    /// Surface regularity cannot be classified robustly at the query point.
    pub const ILL_CONDITIONED_SURFACE: ErrorCode =
        known_error_code("kgraph.eval.ill-conditioned-surface");
    /// The requested exact derivative order is unavailable.
    pub const DERIVATIVE_ORDER: ErrorCode = known_error_code("kgraph.eval.derivative-order");
    /// An evaluator produced a non-finite value despite validated inputs.
    pub const NON_FINITE_RESULT: ErrorCode = known_error_code("kgraph.eval.non-finite-result");

    /// Every stable graph-evaluation error code in deterministic order.
    pub const ALL: &[ErrorCode] = &[
        STALE_GEOMETRY_HANDLE,
        INVALID_PARAMETER,
        PARAMETER_OUTSIDE_DOMAIN,
        INVALID_RANGE,
        DEPENDENCY_CYCLE,
        DEPENDENCY_DEPTH,
        NODE_VISITS,
        SINGULAR_SURFACE,
        ILL_CONDITIONED_SURFACE,
        DERIVATIVE_ORDER,
        NON_FINITE_RESULT,
    ];
}

/// Stable finite support-matrix features owned by graph evaluation.
pub mod capability {
    use super::{CapabilityId, known_capability};

    /// Exact derivative orders supported by a geometry class.
    pub const DERIVATIVE_ORDER: CapabilityId = known_capability("kgraph.eval.derivative-order");

    /// Every graph-evaluation capability in deterministic order.
    pub const ALL: &[CapabilityId] = &[DERIVATIVE_ORDER];
}

/// Stable work stages owned by bounded graph evaluation.
pub mod stage {
    use super::{StageId, known_stage};

    /// Per-query dependency-stack depth.
    pub const DEPENDENCY_DEPTH: StageId = known_stage("kgraph.eval.dependency-depth");
    /// Per-query graph-node visits.
    pub const NODE_VISITS: StageId = known_stage("kgraph.eval.node-visits");

    /// Every graph-evaluation stage in deterministic order.
    pub const ALL: &[StageId] = &[DEPENDENCY_DEPTH, NODE_VISITS];
}

/// Result returned by graph ownership and dependency operations.
pub type GeometryGraphResult<T> = Result<T, GeometryGraphError>;

/// A geometry graph invariant or ownership failure.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum GeometryGraphError {
    /// A descriptor failed graph-boundary validation.
    InvalidDescriptor {
        /// Exact descriptor class.
        class: GeometryClassKey,
        /// Stable human-readable validation reason.
        reason: &'static str,
    },
    /// A handle does not identify a live node of the requested kind.
    StaleGeometryHandle {
        /// Stale reference.
        geometry: GeometryRef,
    },
    /// Removal was attempted while graph nodes still depend on this node.
    HasDependents {
        /// Referenced node.
        geometry: GeometryRef,
        /// Deterministically ordered direct dependents.
        dependents: Vec<GeometryRef>,
    },
    /// A dependency cycle was found during validation or reconstruction.
    DependencyCycle {
        /// Deterministic cycle path, including the repeated endpoint.
        path: Vec<GeometryRef>,
    },
    /// The reverse dependency index disagrees with descriptor dependencies.
    ReverseDependencyMismatch {
        /// Node whose reverse-index entry is inconsistent.
        geometry: GeometryRef,
    },
}

impl fmt::Display for GeometryGraphError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDescriptor { class, reason } => {
                write!(f, "invalid {class} descriptor: {reason}")
            }
            Self::StaleGeometryHandle { geometry } => {
                write!(f, "stale geometry handle: {geometry:?}")
            }
            Self::HasDependents { geometry, .. } => {
                write!(f, "geometry still has graph dependents: {geometry:?}")
            }
            Self::DependencyCycle { path } => write!(f, "geometry dependency cycle: {path:?}"),
            Self::ReverseDependencyMismatch { geometry } => {
                write!(f, "reverse dependency mismatch at {geometry:?}")
            }
        }
    }
}

impl std::error::Error for GeometryGraphError {}

/// Result returned by bounded graph evaluation.
pub type EvalResult<T> = Result<T, EvalError>;

/// A typed geometry-graph evaluation failure.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum EvalError {
    /// The requested handle is not live.
    StaleGeometryHandle {
        /// Stale reference.
        geometry: GeometryRef,
    },
    /// A scalar or vector parameter was not finite.
    InvalidParameter,
    /// A finite parameter lies outside a non-periodic bounded leaf domain.
    ParameterOutsideDomain,
    /// A bounding range was non-finite, reversed, or outside the leaf domain.
    InvalidRange,
    /// Defense-in-depth evaluation cycle detection fired.
    DependencyCycle {
        /// Deterministic active-stack cycle path.
        path: Vec<GeometryRef>,
    },
    /// The per-query dependency-depth reservation was exhausted.
    DependencyDepthExceeded {
        /// Depth that would have been consumed.
        consumed: usize,
        /// Configured maximum depth.
        limit: usize,
    },
    /// The per-query node-visit reservation was exhausted.
    NodeVisitLimitExceeded {
        /// Visits consumed including the rejected visit.
        consumed: usize,
        /// Configured maximum visits.
        limit: usize,
    },
    /// Surface derivatives are singular at the query parameter.
    SingularSurface {
        /// Surface being evaluated.
        surface: SurfaceHandle,
        /// Query parameter.
        uv: [f64; 2],
    },
    /// Surface derivatives cannot be classified robustly at the query point.
    IllConditionedSurface {
        /// Surface being evaluated.
        surface: SurfaceHandle,
        /// Query parameter.
        uv: [f64; 2],
    },
    /// A geometry class cannot supply the requested exact derivative order.
    DerivativeUnavailable {
        /// Exact geometry class.
        class: GeometryClassKey,
        /// Requested order.
        requested: usize,
    },
    /// A leaf evaluator returned a non-finite value.
    NonFiniteResult {
        /// Exact geometry class.
        class: GeometryClassKey,
    },
}

impl fmt::Display for EvalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::StaleGeometryHandle { geometry } => {
                write!(f, "stale geometry handle: {geometry:?}")
            }
            Self::InvalidParameter => f.write_str("geometry parameter must be finite"),
            Self::ParameterOutsideDomain => f.write_str("parameter lies outside geometry domain"),
            Self::InvalidRange => {
                f.write_str("geometry range must be finite, ordered, and in-domain")
            }
            Self::DependencyCycle { path } => write!(f, "geometry evaluation cycle: {path:?}"),
            Self::DependencyDepthExceeded { consumed, limit } => {
                write!(
                    f,
                    "geometry dependency depth {consumed} exceeds limit {limit}"
                )
            }
            Self::NodeVisitLimitExceeded { consumed, limit } => {
                write!(f, "geometry node visits {consumed} exceed limit {limit}")
            }
            Self::SingularSurface { surface, uv } => {
                write!(f, "surface {surface:?} is singular at {uv:?}")
            }
            Self::IllConditionedSurface { surface, uv } => {
                write!(f, "surface {surface:?} is ill-conditioned at {uv:?}")
            }
            Self::DerivativeUnavailable { class, requested } => {
                write!(f, "{class} does not supply derivative order {requested}")
            }
            Self::NonFiniteResult { class } => {
                write!(f, "{class} evaluation produced a non-finite result")
            }
        }
    }
}

impl std::error::Error for EvalError {}

impl EvalError {
    /// Broad semantic class for generic kernel reporting.
    pub const fn class(&self) -> ErrorClass {
        match self {
            Self::StaleGeometryHandle { .. }
            | Self::InvalidParameter
            | Self::ParameterOutsideDomain
            | Self::InvalidRange => ErrorClass::InvalidInput,
            // Evaluation only sees graph state that passed insertion-time cycle
            // validation. Reaching this defense-in-depth branch is therefore a
            // graph invariant failure, not bad query input.
            Self::DependencyCycle { .. } | Self::NonFiniteResult { .. } => {
                ErrorClass::InternalInvariant
            }
            Self::DependencyDepthExceeded { .. } | Self::NodeVisitLimitExceeded { .. } => {
                ErrorClass::ResourceLimit
            }
            // These are model-local numerical evaluation failures. They do not
            // claim that the descriptor is invalid everywhere; proof-bearing
            // callers may retain them as indeterminate evidence.
            Self::SingularSurface { .. } | Self::IllConditionedSurface { .. } => {
                ErrorClass::ModelRejected
            }
            Self::DerivativeUnavailable { .. } => ErrorClass::Unsupported,
        }
    }

    /// Stable machine-readable reason this evaluation failed.
    pub const fn code(&self) -> ErrorCode {
        match self {
            Self::StaleGeometryHandle { .. } => code::STALE_GEOMETRY_HANDLE,
            Self::InvalidParameter => code::INVALID_PARAMETER,
            Self::ParameterOutsideDomain => code::PARAMETER_OUTSIDE_DOMAIN,
            Self::InvalidRange => code::INVALID_RANGE,
            Self::DependencyCycle { .. } => code::DEPENDENCY_CYCLE,
            Self::DependencyDepthExceeded { .. } => code::DEPENDENCY_DEPTH,
            Self::NodeVisitLimitExceeded { .. } => code::NODE_VISITS,
            Self::SingularSurface { .. } => code::SINGULAR_SURFACE,
            Self::IllConditionedSurface { .. } => code::ILL_CONDITIONED_SURFACE,
            Self::DerivativeUnavailable { .. } => code::DERIVATIVE_ORDER,
            Self::NonFiniteResult { .. } => code::NON_FINITE_RESULT,
        }
    }

    /// Unavailable finite support-matrix feature, when applicable.
    pub const fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::DerivativeUnavailable { .. } => Some(capability::DERIVATIVE_ORDER),
            _ => None,
        }
    }

    /// Structured F2 limit snapshot for bounded evaluator stops.
    pub const fn limit(&self) -> Option<LimitSnapshot> {
        let (stage, resource, consumed, allowed) = match self {
            Self::DependencyDepthExceeded { consumed, limit } => (
                stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
                *consumed,
                *limit,
            ),
            Self::NodeVisitLimitExceeded { consumed, limit } => {
                (stage::NODE_VISITS, ResourceKind::Work, *consumed, *limit)
            }
            _ => return None,
        };
        Some(LimitSnapshot {
            stage,
            resource,
            consumed: consumed as u64,
            allowed: allowed as u64,
        })
    }
}

impl ClassifiedError for EvalError {
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

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;

    #[test]
    fn evaluation_identifiers_are_unique_and_stably_namespaced() {
        const FROZEN_CODES: &[&str] = &[
            "kgraph.eval.stale-geometry-handle",
            "kgraph.eval.invalid-parameter",
            "kgraph.eval.parameter-outside-domain",
            "kgraph.eval.invalid-range",
            "kgraph.eval.dependency-cycle",
            "kgraph.eval.dependency-depth-limit",
            "kgraph.eval.node-visit-limit",
            "kgraph.eval.singular-surface",
            "kgraph.eval.ill-conditioned-surface",
            "kgraph.eval.derivative-order",
            "kgraph.eval.non-finite-result",
        ];
        let codes: BTreeSet<_> = code::ALL.iter().map(|code| code.as_str()).collect();
        assert_eq!(codes.len(), code::ALL.len());
        assert!(codes.iter().all(|code| code.starts_with("kgraph.eval.")));
        assert_eq!(
            code::ALL
                .iter()
                .map(|code| code.as_str())
                .collect::<Vec<_>>(),
            FROZEN_CODES
        );
        assert_eq!(
            capability::ALL
                .iter()
                .map(|capability| capability.as_str())
                .collect::<Vec<_>>(),
            ["kgraph.eval.derivative-order"]
        );
        assert_eq!(
            stage::ALL
                .iter()
                .map(|stage| stage.as_str())
                .collect::<Vec<_>>(),
            ["kgraph.eval.dependency-depth", "kgraph.eval.node-visits"]
        );
    }

    #[test]
    fn bounded_evaluation_errors_expose_exact_shared_snapshots() {
        let depth = EvalError::DependencyDepthExceeded {
            consumed: 3,
            limit: 2,
        };
        assert_eq!(depth.class(), ErrorClass::ResourceLimit);
        assert_eq!(
            depth.limit(),
            Some(LimitSnapshot {
                stage: stage::DEPENDENCY_DEPTH,
                resource: ResourceKind::Depth,
                consumed: 3,
                allowed: 2,
            })
        );
        let visits = EvalError::NodeVisitLimitExceeded {
            consumed: 9,
            limit: 8,
        };
        assert_eq!(visits.class(), ErrorClass::ResourceLimit);
        assert_eq!(
            visits.limit(),
            Some(LimitSnapshot {
                stage: stage::NODE_VISITS,
                resource: ResourceKind::Work,
                consumed: 9,
                allowed: 8,
            })
        );
    }

    #[test]
    fn evaluation_classification_distinguishes_contract_owners() {
        let unavailable = EvalError::DerivativeUnavailable {
            class: crate::SurfaceClass::Offset.key(),
            requested: 2,
        };
        assert_eq!(unavailable.class(), ErrorClass::Unsupported);
        assert_eq!(unavailable.capability(), Some(capability::DERIVATIVE_ORDER));
        assert_eq!(
            EvalError::InvalidParameter.class(),
            ErrorClass::InvalidInput
        );
        assert_eq!(
            EvalError::NonFiniteResult {
                class: crate::SurfaceClass::Offset.key(),
            }
            .class(),
            ErrorClass::InternalInvariant
        );
    }
}
