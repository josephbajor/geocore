//! Layer-local intersection failure boundary and common classification view.

use core::fmt;

use kcore::error::{CapabilityId, ClassifiedError, Error, ErrorClass, ErrorCode};
use kcore::operation::LimitSnapshot;
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
    /// A supported specialized solver rejected its input or failed while
    /// preserving the lower-layer classification and source payload.
    Kernel(Error),
}

impl IntersectionError {
    /// Returns the broad semantic class.
    pub const fn class(&self) -> ErrorClass {
        match self {
            Self::UnsupportedCurvePair { .. } | Self::UnsupportedSurfacePair { .. } => {
                ErrorClass::Unsupported
            }
            Self::Kernel(error) => error.class(),
        }
    }

    /// Returns the stable failure identity.
    pub const fn code(&self) -> ErrorCode {
        match self {
            Self::UnsupportedCurvePair { .. } | Self::UnsupportedSurfacePair { .. } => {
                UNSUPPORTED_CLASS_PAIR
            }
            Self::Kernel(error) => error.code(),
        }
    }

    /// Returns the fixed support-matrix capability for unsupported class
    /// pairs, or delegates a wrapped source capability.
    pub const fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::UnsupportedCurvePair { .. } => Some(CURVE_CURVE_CLASS_PAIR),
            Self::UnsupportedSurfacePair { .. } => Some(SURFACE_SURFACE_CLASS_PAIR),
            Self::Kernel(error) => error.capability(),
        }
    }

    /// Returns structured F2 limit data unchanged from a wrapped source.
    pub const fn limit(&self) -> Option<LimitSnapshot> {
        match self {
            Self::Kernel(error) => error.limit(),
            Self::UnsupportedCurvePair { .. } | Self::UnsupportedSurfacePair { .. } => None,
        }
    }
}

impl fmt::Display for IntersectionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedCurvePair { class_a, class_b } => {
                write_class_pair(formatter, "curve/curve", *class_a, *class_b)
            }
            Self::UnsupportedSurfacePair { class_a, class_b } => {
                write_class_pair(formatter, "surface/surface", *class_a, *class_b)
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
            Self::Kernel(error) => Some(error),
            Self::UnsupportedCurvePair { .. } | Self::UnsupportedSurfacePair { .. } => None,
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

/// Result boundary for generic intersection dispatchers.
pub type IntersectionResult<T> = core::result::Result<T, IntersectionError>;
