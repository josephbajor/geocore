//! Layer-local intersection failure boundary and common classification view.

use core::fmt;

use kcore::error::{CapabilityId, ClassifiedError, Error, ErrorClass, ErrorCode};
use kcore::operation::LimitSnapshot;
use kgeom::project::ProjectionError;
use kgraph::GeometryClassKey;

const fn error_code(value: &'static str) -> ErrorCode {
    match ErrorCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in intersection error code"),
    }
}

const fn capability_id(value: &'static str) -> CapabilityId {
    match CapabilityId::new(value) {
        Ok(capability) => capability,
        Err(_) => panic!("invalid built-in intersection capability identifier"),
    }
}

/// Stable failure identity for a valid class pair with no implemented solver.
pub const UNSUPPORTED_CLASS_PAIR: ErrorCode = error_code("kops.intersect.unsupported-class-pair");

/// Finite support-matrix capability for curve/curve class-pair dispatch.
pub const CURVE_CURVE_CLASS_PAIR: CapabilityId =
    capability_id("kops.intersect.curve-curve.class-pair");

/// Finite support-matrix capability for curve/surface class-pair dispatch.
pub const CURVE_SURFACE_CLASS_PAIR: CapabilityId =
    capability_id("kops.intersect.curve-surface.class-pair");

/// Finite support-matrix capability for surface/surface class-pair dispatch.
pub const SURFACE_SURFACE_CLASS_PAIR: CapabilityId =
    capability_id("kops.intersect.surface-surface.class-pair");

/// Failures owned by the generic intersection dispatch boundary.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum IntersectionError {
    /// Both curve inputs are valid, but their class pair has no registered
    /// solver in this kernel version.
    UnsupportedCurvePair {
        /// Canonical class key for the first operand, or `None` when the
        /// valid trait implementation is not in the current registry.
        class_a: Option<GeometryClassKey>,
        /// Canonical class key for the second operand, or `None` when the
        /// valid trait implementation is not in the current registry.
        class_b: Option<GeometryClassKey>,
    },
    /// The curve and surface inputs are valid, but their class pair has no
    /// registered solver in this kernel version.
    UnsupportedCurveSurfacePair {
        /// Canonical class key for the curve, or `None` when the valid trait
        /// implementation is not in the current registry.
        curve_class: Option<GeometryClassKey>,
        /// Canonical class key for the surface, or `None` when the valid trait
        /// implementation is not in the current registry.
        surface_class: Option<GeometryClassKey>,
    },
    /// Both surface inputs are valid, but their class pair has no registered
    /// solver in this kernel version.
    UnsupportedSurfacePair {
        /// Canonical class key for the first operand, or `None` when the
        /// valid trait implementation is not in the current registry.
        class_a: Option<GeometryClassKey>,
        /// Canonical class key for the second operand, or `None` when the
        /// valid trait implementation is not in the current registry.
        class_b: Option<GeometryClassKey>,
    },
    /// A closest-point projection used by a supported specialized solver
    /// failed while preserving its exact classification and source payload.
    Projection(ProjectionError),
    /// A supported specialized solver rejected its input or failed while
    /// preserving the lower-layer classification and source payload.
    Kernel(Error),
}

impl IntersectionError {
    /// Returns the broad semantic class.
    pub const fn class(&self) -> ErrorClass {
        match self {
            Self::UnsupportedCurvePair { .. }
            | Self::UnsupportedCurveSurfacePair { .. }
            | Self::UnsupportedSurfacePair { .. } => ErrorClass::Unsupported,
            Self::Projection(error) => error.class(),
            Self::Kernel(error) => error.class(),
        }
    }

    /// Returns the stable failure identity.
    pub const fn code(&self) -> ErrorCode {
        match self {
            Self::UnsupportedCurvePair { .. }
            | Self::UnsupportedCurveSurfacePair { .. }
            | Self::UnsupportedSurfacePair { .. } => UNSUPPORTED_CLASS_PAIR,
            Self::Projection(error) => error.code(),
            Self::Kernel(error) => error.code(),
        }
    }

    /// Returns the fixed support-matrix capability for unsupported class
    /// pairs, or delegates a wrapped source capability.
    pub const fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::UnsupportedCurvePair { .. } => Some(CURVE_CURVE_CLASS_PAIR),
            Self::UnsupportedCurveSurfacePair { .. } => Some(CURVE_SURFACE_CLASS_PAIR),
            Self::UnsupportedSurfacePair { .. } => Some(SURFACE_SURFACE_CLASS_PAIR),
            Self::Projection(_) => None,
            Self::Kernel(error) => error.capability(),
        }
    }

    /// Returns structured F2 limit data unchanged from a wrapped source.
    pub const fn limit(&self) -> Option<LimitSnapshot> {
        match self {
            Self::Projection(error) => error.limit(),
            Self::Kernel(error) => error.limit(),
            Self::UnsupportedCurvePair { .. }
            | Self::UnsupportedCurveSurfacePair { .. }
            | Self::UnsupportedSurfacePair { .. } => None,
        }
    }
}

impl fmt::Display for IntersectionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedCurvePair { class_a, class_b } => {
                write_class_pair(formatter, "curve/curve", *class_a, *class_b)
            }
            Self::UnsupportedCurveSurfacePair {
                curve_class,
                surface_class,
            } => write_class_pair(formatter, "curve/surface", *curve_class, *surface_class),
            Self::UnsupportedSurfacePair { class_a, class_b } => {
                write_class_pair(formatter, "surface/surface", *class_a, *class_b)
            }
            Self::Projection(error) => {
                write!(formatter, "intersection projection failed: {error}")
            }
            Self::Kernel(error) => write!(formatter, "intersection solver failed: {error}"),
        }
    }
}

fn write_class_pair(
    formatter: &mut fmt::Formatter<'_>,
    family: &str,
    class_a: Option<GeometryClassKey>,
    class_b: Option<GeometryClassKey>,
) -> fmt::Result {
    let class_a = class_a.map_or("unclassified", GeometryClassKey::as_str);
    let class_b = class_b.map_or("unclassified", GeometryClassKey::as_str);
    write!(
        formatter,
        "unsupported {family} intersection class pair ({class_a}, {class_b})"
    )
}

impl std::error::Error for IntersectionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Projection(error) => Some(error),
            Self::Kernel(error) => Some(error),
            Self::UnsupportedCurvePair { .. }
            | Self::UnsupportedCurveSurfacePair { .. }
            | Self::UnsupportedSurfacePair { .. } => None,
        }
    }
}

impl ClassifiedError for IntersectionError {
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

impl From<Error> for IntersectionError {
    fn from(error: Error) -> Self {
        Self::Kernel(error)
    }
}

impl From<ProjectionError> for IntersectionError {
    fn from(error: ProjectionError) -> Self {
        Self::Projection(error)
    }
}

/// Result boundary for generic intersection dispatchers.
pub type IntersectionResult<T> = core::result::Result<T, IntersectionError>;

#[cfg(test)]
mod tests {
    use std::error::Error as _;

    use kcore::operation::{OperationPolicyError, ResourceKind, TOTAL_WORK_STAGE};
    use kgeom::project::error_code as projection_error_code;

    use super::*;

    #[test]
    fn projection_failures_preserve_every_classification_and_source() {
        let snapshot = LimitSnapshot {
            stage: TOTAL_WORK_STAGE,
            resource: ResourceKind::Work,
            consumed: 2,
            allowed: 1,
        };
        let policy = OperationPolicyError::LimitReached(snapshot);
        let cases = [
            (
                ProjectionError::InvalidQueryPoint,
                ErrorClass::InvalidInput,
                projection_error_code::INVALID_QUERY_POINT,
                None,
            ),
            (
                ProjectionError::InvalidWindow { direction: 1 },
                ErrorClass::InvalidInput,
                projection_error_code::INVALID_WINDOW,
                None,
            ),
            (
                ProjectionError::NoCandidate,
                ErrorClass::InternalInvariant,
                projection_error_code::NO_CANDIDATE,
                None,
            ),
            (
                ProjectionError::NonFiniteEvaluation,
                ErrorClass::InternalInvariant,
                projection_error_code::NON_FINITE_EVALUATION,
                None,
            ),
            (
                ProjectionError::Policy(policy.clone()),
                policy.class(),
                policy.code(),
                Some(snapshot),
            ),
        ];

        for (projection, class, code, limit) in cases {
            let error = IntersectionError::from(projection.clone());
            assert_eq!(error, IntersectionError::Projection(projection.clone()));
            assert_eq!(error.class(), class);
            assert_eq!(error.code(), code);
            assert_eq!(error.capability(), None);
            assert_eq!(error.limit(), limit);

            let classified: &dyn ClassifiedError = &error;
            assert_eq!(classified.class(), class);
            assert_eq!(classified.code(), code);
            assert_eq!(classified.capability(), None);
            assert_eq!(classified.limit(), limit);

            let retained = error
                .source()
                .and_then(|source| source.downcast_ref::<ProjectionError>())
                .expect("projection remains the direct intersection source");
            assert_eq!(retained, &projection);
            if let ProjectionError::Policy(policy) = &projection {
                assert!(matches!(
                    retained.source().and_then(|source| {
                        source.downcast_ref::<OperationPolicyError>()
                    }),
                    Some(found) if found == policy
                ));
            } else {
                assert!(retained.source().is_none());
            }
        }
    }
}
