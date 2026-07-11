//! Typed errors for XT interchange.
//!
//! `kxt` has its own error type: interchange failures (malformed files,
//! unsupported schemas) are a different species from kernel errors, and
//! carry file positions. Kernel errors arising during reconstruction are
//! wrapped in [`XtError::Kernel`].

use core::fmt;

use kcore::error::{CapabilityId, ClassifiedError, ErrorClass, ErrorCode};
use kcore::operation::LimitSnapshot;

/// Stable machine-readable reason that valid XT content is outside the
/// currently declared import/export support matrix.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum XtCapability {
    /// Schema generation is not based on the supported base schema.
    SchemaBase13006,
    /// A schema node type has no usable declaration.
    SchemaNodeType,
    /// Machine-dependent bare-binary encoding.
    MachineDependentBinary,
    /// Assembly or non-body part reconstruction.
    Assemblies,
    /// Partition/world reconstruction.
    Partitions,
    /// General-body topology.
    GeneralBodies,
    /// Face without attached surface geometry.
    SurfaceLessFaces,
    /// Isolated/single-vertex loop topology.
    IsolatedLoops,
    /// Curve-less tolerant ring edge.
    TolerantRingEdges,
    /// Procedural/intersection/foreign curve realization.
    ProceduralCurves,
    /// Procedural swept/spun/blend/foreign surface realization.
    ProceduralSurfaces,
    /// Periodic NURBS curve realization or writing.
    PeriodicNurbsCurves,
    /// Periodic NURBS surface realization or writing.
    PeriodicNurbsSurfaces,
    /// Periodic parameter-space geometry, explicit charts, or seam roles.
    PeriodicPcurves,
    /// Apple/lemon self-intersecting torus realization.
    SelfIntersectingTori,
    /// NURBS pole layout outside the declared 2D/3D rational conventions.
    NonstandardNurbsPoles,
    /// Writer body/region/shell topology outside the declared subset.
    WriterBodyTopology,
    /// Writer edge/fin/curve topology outside the declared subset.
    WriterEdgeTopology,
    /// Curve-less tolerant wire edge writing.
    TolerantWireEdges,
    /// Circular pcurve writing.
    CircularPcurves,
    /// Non-null kernel face tolerance cannot be represented by the
    /// published schema-13006 writer contract.
    FaceTolerances,
    /// Two exported offsets share one basis; canonical ownership awaits a
    /// modern Parasolid oracle.
    SharedOffsetBasisExport,
    /// An exported offset directly uses another offset as its basis.
    NestedOffsetExport,
}

impl XtCapability {
    /// All capability codes known to this crate version, in stable order.
    pub const ALL: &'static [Self] = &[
        Self::SchemaBase13006,
        Self::SchemaNodeType,
        Self::MachineDependentBinary,
        Self::Assemblies,
        Self::Partitions,
        Self::GeneralBodies,
        Self::SurfaceLessFaces,
        Self::IsolatedLoops,
        Self::TolerantRingEdges,
        Self::ProceduralCurves,
        Self::ProceduralSurfaces,
        Self::PeriodicNurbsCurves,
        Self::PeriodicNurbsSurfaces,
        Self::PeriodicPcurves,
        Self::SelfIntersectingTori,
        Self::NonstandardNurbsPoles,
        Self::WriterBodyTopology,
        Self::WriterEdgeTopology,
        Self::TolerantWireEdges,
        Self::CircularPcurves,
        Self::FaceTolerances,
        Self::SharedOffsetBasisExport,
        Self::NestedOffsetExport,
    ];

    /// Stable dotted identifier for manifests, metrics, and API clients.
    pub const fn code(self) -> &'static str {
        match self {
            Self::SchemaBase13006 => "xt.schema.base-13006",
            Self::SchemaNodeType => "xt.schema.node-type",
            Self::MachineDependentBinary => "xt.encoding.machine-binary",
            Self::Assemblies => "xt.read.assemblies",
            Self::Partitions => "xt.read.partitions",
            Self::GeneralBodies => "xt.read.general-bodies",
            Self::SurfaceLessFaces => "xt.read.surface-less-faces",
            Self::IsolatedLoops => "xt.read.isolated-loops",
            Self::TolerantRingEdges => "xt.read.tolerant-ring-edges",
            Self::ProceduralCurves => "xt.geometry.procedural-curves",
            Self::ProceduralSurfaces => "xt.geometry.procedural-surfaces",
            Self::PeriodicNurbsCurves => "xt.geometry.periodic-nurbs-curves",
            Self::PeriodicNurbsSurfaces => "xt.geometry.periodic-nurbs-surfaces",
            Self::PeriodicPcurves => "xt.geometry.periodic-pcurves",
            Self::SelfIntersectingTori => "xt.geometry.self-intersecting-tori",
            Self::NonstandardNurbsPoles => "xt.geometry.nonstandard-nurbs-poles",
            Self::WriterBodyTopology => "xt.write.body-topology",
            Self::WriterEdgeTopology => "xt.write.edge-topology",
            Self::TolerantWireEdges => "xt.write.tolerant-wire-edges",
            Self::CircularPcurves => "xt.write.circular-pcurves",
            Self::FaceTolerances => "xt.write.face-tolerances",
            Self::SharedOffsetBasisExport => "xt.write.shared-offset-basis",
            Self::NestedOffsetExport => "xt.write.nested-offset",
        }
    }

    /// Shared stable capability identity used by generic kernel reporting.
    pub const fn id(self) -> CapabilityId {
        match CapabilityId::new(self.code()) {
            Ok(id) => id,
            Err(_) => panic!("invalid built-in XT capability identifier"),
        }
    }
}

impl From<XtCapability> for CapabilityId {
    fn from(capability: XtCapability) -> Self {
        capability.id()
    }
}

const fn known_error_code(value: &'static str) -> ErrorCode {
    match ErrorCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in XT error code"),
    }
}

/// Stable error identities owned by X_T interchange contracts.
pub mod code {
    use super::{ErrorCode, known_error_code};

    /// The common X_T header is malformed.
    pub const BAD_HEADER: ErrorCode = known_error_code("xt.parse.bad-header");
    /// The declared schema is outside the reader's supported base schema.
    pub const UNSUPPORTED_SCHEMA: ErrorCode = known_error_code("xt.schema.unsupported");
    /// The X_T data stream is malformed.
    pub const PARSE: ErrorCode = known_error_code("xt.parse.invalid-data");
    /// A node type is absent from the applicable schema.
    pub const UNKNOWN_NODE_TYPE: ErrorCode = known_error_code("xt.schema.unknown-node-type");
    /// Valid X_T content requires an unavailable support-matrix capability.
    pub const UNSUPPORTED: ErrorCode = known_error_code("xt.content.unsupported");
    /// A kernel model violates an invariant required for deterministic export.
    pub const INVALID_MODEL: ErrorCode = known_error_code("xt.write.invalid-model");
    /// A required referenced node is missing.
    pub const MISSING_NODE: ErrorCode = known_error_code("xt.parse.missing-node");
    /// A node field has an invalid type or value.
    pub const BAD_FIELD: ErrorCode = known_error_code("xt.parse.bad-field");
    /// A reconstruction coordinate is outside the kernel session size box.
    pub const OUTSIDE_SIZE_BOX: ErrorCode = known_error_code("xt.read.outside-size-box");
    /// Surface references contain a dependency cycle.
    pub const SURFACE_DEPENDENCY_CYCLE: ErrorCode =
        known_error_code("xt.read.surface-dependency-cycle");

    /// Every X_T-owned error code in deterministic order.
    pub const ALL: &[ErrorCode] = &[
        BAD_HEADER,
        UNSUPPORTED_SCHEMA,
        PARSE,
        UNKNOWN_NODE_TYPE,
        UNSUPPORTED,
        INVALID_MODEL,
        MISSING_NODE,
        BAD_FIELD,
        OUTSIDE_SIZE_BOX,
        SURFACE_DEPENDENCY_CYCLE,
    ];
}

/// Errors produced while reading or reconstructing an XT transmit file.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum XtError {
    /// The common header (`**...`) is malformed or truncated.
    BadHeader {
        /// What was wrong.
        what: &'static str,
    },
    /// The file's schema cannot be read by this Tier-0 reader. Supported:
    /// schema 13006 exactly, or any embedded-schema file with base 13006.
    UnsupportedSchema {
        /// The schema key from the file, e.g. `SCH_1000230_10004`.
        schema: String,
    },
    /// The data stream is malformed at `offset` (bytes into the cleaned
    /// data section).
    Parse {
        /// Byte offset into the data section.
        offset: usize,
        /// What was expected or found.
        what: &'static str,
    },
    /// A node type unknown to the base schema appeared in a file that
    /// carries no embedded schema description for it.
    UnknownNodeType {
        /// The numeric node type.
        code: u16,
    },
    /// The file uses a feature outside Tier-0 scope (foreign geometry,
    /// tolerant edges, general bodies, …).
    Unsupported {
        /// Stable support-matrix capability.
        capability: XtCapability,
        /// The feature.
        what: &'static str,
    },
    /// A source model violates an invariant required for XT emission.
    InvalidModel {
        /// What made the model impossible to serialize safely.
        what: &'static str,
    },
    /// A pointer referenced a node index that is absent where required.
    MissingNode {
        /// The referenced index.
        index: u32,
    },
    /// A node's field had an unexpected type or value.
    BadField {
        /// The node index.
        index: u32,
        /// The field (and expectation).
        what: &'static str,
    },
    /// A coordinate exceeded the ±500 m session size box or was not
    /// finite.
    OutsideSizeBox {
        /// The offending value.
        value: f64,
    },
    /// Recursive X_T surface references contain a cycle.
    SurfaceDependencyCycle {
        /// Deterministic transport-node path including the repeated endpoint.
        path: Vec<u32>,
    },
    /// Geometry-graph evaluation failed while validating or emitting data.
    Evaluation(kgraph::EvalError),
    /// A kernel error during geometry or topology reconstruction.
    Kernel(kcore::error::Error),
}

impl fmt::Display for XtError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            XtError::BadHeader { what } => write!(f, "malformed XT header: {what}"),
            XtError::UnsupportedSchema { schema } => {
                write!(
                    f,
                    "unsupported XT schema {schema} (need 13006 or base-13006)"
                )
            }
            XtError::Parse { offset, what } => {
                write!(f, "XT parse error at data offset {offset}: {what}")
            }
            XtError::UnknownNodeType { code } => {
                write!(
                    f,
                    "node type {code} unknown to schema 13006 and not described in file"
                )
            }
            XtError::Unsupported { capability, what } => {
                write!(f, "unsupported XT content [{}]: {what}", capability.code())
            }
            XtError::InvalidModel { what } => write!(f, "invalid model for XT export: {what}"),
            XtError::MissingNode { index } => write!(f, "referenced node {index} is missing"),
            XtError::BadField { index, what } => write!(f, "node {index}: {what}"),
            XtError::OutsideSizeBox { value } => {
                write!(f, "coordinate {value} outside the ±500 m size box")
            }
            XtError::SurfaceDependencyCycle { path } => {
                write!(f, "XT surface dependency cycle: {path:?}")
            }
            XtError::Evaluation(error) => write!(f, "XT geometry evaluation failed: {error}"),
            XtError::Kernel(e) => write!(f, "kernel error during reconstruction: {e}"),
        }
    }
}

impl std::error::Error for XtError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Evaluation(error) => Some(error),
            Self::Kernel(error) => Some(error),
            _ => None,
        }
    }
}

impl XtError {
    /// Stable capability code when this error means "valid but unsupported".
    pub const fn capability(&self) -> Option<XtCapability> {
        match self {
            Self::UnsupportedSchema { .. } => Some(XtCapability::SchemaBase13006),
            Self::UnknownNodeType { .. } => Some(XtCapability::SchemaNodeType),
            Self::Unsupported { capability, .. } => Some(*capability),
            _ => None,
        }
    }

    /// Broad semantic class for generic kernel reporting.
    pub const fn class(&self) -> ErrorClass {
        match self {
            Self::BadHeader { .. }
            | Self::Parse { .. }
            | Self::MissingNode { .. }
            | Self::BadField { .. }
            | Self::OutsideSizeBox { .. }
            | Self::SurfaceDependencyCycle { .. } => ErrorClass::InvalidInput,
            Self::UnsupportedSchema { .. }
            | Self::UnknownNodeType { .. }
            | Self::Unsupported { .. } => ErrorClass::Unsupported,
            Self::InvalidModel { .. } => ErrorClass::ModelRejected,
            Self::Evaluation(error) => error.class(),
            Self::Kernel(error) => error.class(),
        }
    }

    /// Stable machine-readable reason this X_T call failed.
    pub const fn code(&self) -> ErrorCode {
        match self {
            Self::BadHeader { .. } => code::BAD_HEADER,
            Self::UnsupportedSchema { .. } => code::UNSUPPORTED_SCHEMA,
            Self::Parse { .. } => code::PARSE,
            Self::UnknownNodeType { .. } => code::UNKNOWN_NODE_TYPE,
            Self::Unsupported { .. } => code::UNSUPPORTED,
            Self::InvalidModel { .. } => code::INVALID_MODEL,
            Self::MissingNode { .. } => code::MISSING_NODE,
            Self::BadField { .. } => code::BAD_FIELD,
            Self::OutsideSizeBox { .. } => code::OUTSIDE_SIZE_BOX,
            Self::SurfaceDependencyCycle { .. } => code::SURFACE_DEPENDENCY_CYCLE,
            Self::Evaluation(error) => error.code(),
            Self::Kernel(error) => error.code(),
        }
    }

    /// Shared capability identity without changing the existing typed
    /// [`Self::capability`] compatibility accessor.
    pub const fn capability_id(&self) -> Option<CapabilityId> {
        match self.capability() {
            Some(capability) => Some(capability.id()),
            None => match self {
                Self::Evaluation(error) => error.capability(),
                Self::Kernel(error) => error.capability(),
                _ => None,
            },
        }
    }

    /// Structured F2 limit data, delegated unchanged from wrapped kernel
    /// errors.
    pub const fn limit(&self) -> Option<LimitSnapshot> {
        match self {
            Self::Evaluation(error) => error.limit(),
            Self::Kernel(error) => error.limit(),
            _ => None,
        }
    }
}

impl ClassifiedError for XtError {
    fn class(&self) -> ErrorClass {
        self.class()
    }

    fn code(&self) -> ErrorCode {
        self.code()
    }

    fn capability(&self) -> Option<CapabilityId> {
        self.capability_id()
    }

    fn limit(&self) -> Option<LimitSnapshot> {
        self.limit()
    }
}

impl From<kcore::error::Error> for XtError {
    fn from(e: kcore::error::Error) -> Self {
        XtError::Kernel(e)
    }
}

/// Result alias for XT operations.
pub type Result<T> = core::result::Result<T, XtError>;

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;
    use std::error::Error as _;

    #[test]
    fn capability_codes_are_unique_stable_identifiers() {
        const FROZEN_CODES: &[&str] = &[
            "xt.schema.base-13006",
            "xt.schema.node-type",
            "xt.encoding.machine-binary",
            "xt.read.assemblies",
            "xt.read.partitions",
            "xt.read.general-bodies",
            "xt.read.surface-less-faces",
            "xt.read.isolated-loops",
            "xt.read.tolerant-ring-edges",
            "xt.geometry.procedural-curves",
            "xt.geometry.procedural-surfaces",
            "xt.geometry.periodic-nurbs-curves",
            "xt.geometry.periodic-nurbs-surfaces",
            "xt.geometry.periodic-pcurves",
            "xt.geometry.self-intersecting-tori",
            "xt.geometry.nonstandard-nurbs-poles",
            "xt.write.body-topology",
            "xt.write.edge-topology",
            "xt.write.tolerant-wire-edges",
            "xt.write.circular-pcurves",
            "xt.write.face-tolerances",
            "xt.write.shared-offset-basis",
            "xt.write.nested-offset",
        ];
        let codes: BTreeSet<_> = XtCapability::ALL
            .iter()
            .map(|capability| capability.code())
            .collect();
        assert_eq!(codes.len(), XtCapability::ALL.len());
        assert!(codes.iter().all(|code| code.starts_with("xt.")));
        assert_eq!(
            XtCapability::ALL
                .iter()
                .map(|capability| capability.id().as_str())
                .collect::<Vec<_>>(),
            FROZEN_CODES
        );
    }

    #[test]
    fn unsupported_errors_expose_capability_without_parsing_display_text() {
        let error = XtError::Unsupported {
            capability: XtCapability::GeneralBodies,
            what: "context retained for people",
        };
        assert_eq!(error.capability(), Some(XtCapability::GeneralBodies));
        assert_eq!(
            error.capability_id(),
            Some(XtCapability::GeneralBodies.id())
        );
        assert_eq!(error.class(), ErrorClass::Unsupported);
        assert_eq!(error.code(), code::UNSUPPORTED);
        assert!(error.to_string().contains("xt.read.general-bodies"));
        assert_eq!(
            XtError::UnsupportedSchema {
                schema: "SCH_1000230_10004".to_owned(),
            }
            .capability(),
            Some(XtCapability::SchemaBase13006)
        );
    }

    #[test]
    fn xt_codes_are_unique_and_do_not_depend_on_display_context() {
        let codes: BTreeSet<_> = code::ALL.iter().map(|code| code.as_str()).collect();
        assert_eq!(codes.len(), code::ALL.len());

        let first = XtError::BadField {
            index: 1,
            what: "first message",
        };
        let second = XtError::BadField {
            index: 99,
            what: "different message",
        };
        assert_ne!(first.to_string(), second.to_string());
        assert_eq!(first.code(), second.code());
        assert_eq!(first.class(), ErrorClass::InvalidInput);
    }

    #[test]
    fn wrapped_kernel_errors_preserve_classification_and_source_chain() {
        let kernel = kcore::error::Error::TransactionActive;
        let error = XtError::Kernel(kernel.clone());
        assert_eq!(error.class(), kernel.class());
        assert_eq!(error.code(), kernel.code());
        assert_eq!(error.capability_id(), kernel.capability());
        assert_eq!(error.limit(), kernel.limit());
        let source = error.source().expect("kernel source retained");
        assert_eq!(source.to_string(), kernel.to_string());
        assert_eq!(source.downcast_ref::<kcore::error::Error>(), Some(&kernel));

        let classified: &dyn ClassifiedError = &error;
        assert_eq!(classified.class(), ErrorClass::InvalidState);
        assert_eq!(classified.code(), kcore::error::code::TRANSACTION_ACTIVE);
    }
}
