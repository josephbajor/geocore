//! The kernel's typed error model.
//!
//! Every public kernel operation returns [`Result`]. Errors are data, not
//! strings: callers (and eventually the PK-style C API, which maps these to
//! error codes) can branch on them. The enum is `#[non_exhaustive]` and grows
//! as layers land; it never carries panics across the API boundary.

use core::fmt;

/// Errors produced by kernel operations.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// A handle referred to an entity that was removed (or never existed).
    StaleHandle,
    /// A coordinate fell outside the ±500 m session size box or was not finite.
    OutsideSizeBox {
        /// The offending coordinate value.
        coordinate: f64,
    },
    /// A tolerance was below session resolution or not finite.
    InvalidTolerance {
        /// The offending tolerance value.
        value: f64,
    },
    /// Geometry construction received degenerate or inconsistent inputs
    /// (zero-length axis, parallel basis hint, non-positive radius, …).
    InvalidGeometry {
        /// What made the inputs unusable.
        reason: &'static str,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::StaleHandle => write!(f, "handle refers to a removed or unknown entity"),
            Error::OutsideSizeBox { coordinate } => write!(
                f,
                "coordinate {coordinate} lies outside the ±500 m session size box"
            ),
            Error::InvalidTolerance { value } => write!(
                f,
                "tolerance {value} is below session resolution or not finite"
            ),
            Error::InvalidGeometry { reason } => {
                write!(f, "invalid geometry construction: {reason}")
            }
        }
    }
}

impl std::error::Error for Error {}

/// Result alias used by all kernel operations.
pub type Result<T> = core::result::Result<T, Error>;
