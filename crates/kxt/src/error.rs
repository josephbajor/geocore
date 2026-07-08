//! Typed errors for XT interchange.
//!
//! `kxt` has its own error type: interchange failures (malformed files,
//! unsupported schemas) are a different species from kernel errors, and
//! carry file positions. Kernel errors arising during reconstruction are
//! wrapped in [`XtError::Kernel`].

use core::fmt;

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
            XtError::Unsupported { what } => write!(f, "unsupported XT content: {what}"),
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

impl From<kcore::error::Error> for XtError {
    fn from(e: kcore::error::Error) -> Self {
        XtError::Kernel(e)
    }
}

/// Result alias for XT operations.
pub type Result<T> = core::result::Result<T, XtError>;
