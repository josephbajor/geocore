//! Typed errors for XT interchange.
//!
//! `kxt` has its own error type: interchange failures (malformed files,
//! unsupported schemas) are a different species from kernel errors, and
//! carry file positions. Kernel errors arising during reconstruction are
//! wrapped in [`XtError::Kernel`].

use core::fmt;

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
    /// Procedural/swept/spun/offset/blend surface realization.
    ProceduralSurfaces,
    /// Periodic NURBS curve realization or writing.
    PeriodicNurbsCurves,
    /// Periodic NURBS surface realization or writing.
    PeriodicNurbsSurfaces,
    /// Periodic parameter-space NURBS curve realization.
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
        }
    }
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
            XtError::Kernel(e) => write!(f, "kernel error during reconstruction: {e}"),
        }
    }
}

impl std::error::Error for XtError {}

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

    #[test]
    fn capability_codes_are_unique_stable_identifiers() {
        let codes: BTreeSet<_> = XtCapability::ALL
            .iter()
            .map(|capability| capability.code())
            .collect();
        assert_eq!(codes.len(), XtCapability::ALL.len());
        assert!(codes.iter().all(|code| code.starts_with("xt.")));
    }

    #[test]
    fn unsupported_errors_expose_capability_without_parsing_display_text() {
        let error = XtError::Unsupported {
            capability: XtCapability::GeneralBodies,
            what: "context retained for people",
        };
        assert_eq!(error.capability(), Some(XtCapability::GeneralBodies));
        assert!(error.to_string().contains("xt.read.general-bodies"));
        assert_eq!(
            XtError::UnsupportedSchema {
                schema: "SCH_1000230_10004".to_owned(),
            }
            .capability(),
            Some(XtCapability::SchemaBase13006)
        );
    }
}
