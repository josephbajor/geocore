//! Shared deterministic fixtures and contracts for kernel benchmarks.
//!
//! This package is intentionally outside the kernel workspace. Benchmark
//! dependencies must not affect normal kernel builds or become public APIs.

use core::fmt;

pub mod body_tessellation;
pub mod nurbs_isolation;
pub mod topology;
pub mod xt_io;

/// Stable path of the Q1 contract fixture.
pub const CONTRACT_CASE_PATH: &str = "harness/contract/tiny-v1/64/default-v1";

/// Stable fixture version recorded in baseline metadata.
pub const CONTRACT_FIXTURE_VERSION: &str = "tiny-contract.v1";

/// Deterministic seed used by the Q1 contract fixture.
pub const CONTRACT_SEED: u64 = 0x4b45_524e_454c_0001;

/// Number of values in the Q1 contract fixture.
pub const CONTRACT_ELEMENTS: usize = 64;

/// Validate the required `<subsystem>/<operation>/<fixture>/<scale>/<policy>` form.
pub fn validate_case_path(path: &str) -> Result<(), CasePathError> {
    let segments: Vec<_> = path.split('/').collect();
    if segments.len() != 5 {
        return Err(CasePathError::SegmentCount {
            actual: segments.len(),
        });
    }
    for (index, segment) in segments.into_iter().enumerate() {
        let valid = !segment.is_empty()
            && segment
                .bytes()
                .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'-');
        if !valid {
            return Err(CasePathError::InvalidSegment { index });
        }
    }
    Ok(())
}

/// Stable case-path validation failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CasePathError {
    /// The path does not contain exactly five slash-delimited segments.
    SegmentCount {
        /// Observed segment count.
        actual: usize,
    },
    /// A segment is empty or contains a character outside `[a-z0-9-]`.
    InvalidSegment {
        /// Zero-based segment index.
        index: usize,
    },
}

impl fmt::Display for CasePathError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SegmentCount { actual } => {
                write!(f, "benchmark path requires five segments, found {actual}")
            }
            Self::InvalidSegment { index } => {
                write!(f, "benchmark path segment {index} is not canonical")
            }
        }
    }
}

impl std::error::Error for CasePathError {}

/// Fully constructed deterministic input; construction is never benchmarked.
#[derive(Debug, Clone)]
pub struct TinyFixture {
    values: Box<[u64]>,
}

impl TinyFixture {
    /// Construct a fixture without randomness or environment dependence.
    pub fn new(seed: u64, elements: usize) -> Self {
        let mut state = seed;
        let mut values = Vec::with_capacity(elements);
        for index in 0..elements {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            values.push(state ^ index as u64);
        }
        Self {
            values: values.into_boxed_slice(),
        }
    }

    /// Execute the deterministic operation whose result is checked per iteration.
    pub fn execute(&self) -> TinyResult {
        let mut sum = 0_u64;
        let mut digest = 14_695_981_039_346_656_037_u64;
        for &value in &self.values {
            sum = sum.wrapping_add(value);
            digest = (digest ^ value).wrapping_mul(1_099_511_628_211);
        }
        TinyResult {
            elements: self.values.len(),
            sum,
            digest,
        }
    }
}

/// Semantic counters emitted by the Q1 fixture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TinyResult {
    /// Values processed.
    pub elements: usize,
    /// Wrapping sum, retained to prevent dead-code elimination.
    pub sum: u64,
    /// Stable FNV-style output digest.
    pub digest: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn case_paths_have_one_canonical_shape() {
        assert_eq!(validate_case_path(CONTRACT_CASE_PATH), Ok(()));
        assert_eq!(
            validate_case_path("too/few/segments"),
            Err(CasePathError::SegmentCount { actual: 3 })
        );
        assert_eq!(
            validate_case_path("harness/contract/Tiny/64/default"),
            Err(CasePathError::InvalidSegment { index: 2 })
        );
    }

    #[test]
    fn fixture_is_bitwise_reproducible() {
        let a = TinyFixture::new(CONTRACT_SEED, CONTRACT_ELEMENTS).execute();
        let b = TinyFixture::new(CONTRACT_SEED, CONTRACT_ELEMENTS).execute();
        assert_eq!(a, b);
        assert_eq!(a.elements, CONTRACT_ELEMENTS);
        assert_eq!(a.sum, 0xbabf_ef09_cc07_d280);
        assert_eq!(a.digest, 0x1428_9053_7c90_ed65);
    }
}
