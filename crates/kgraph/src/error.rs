//! Typed graph construction and evaluation failures.

use core::fmt;

use crate::{GeometryClassKey, GeometryRef, SurfaceHandle};

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
