//! `kcore` — Layer 0 (foundations) of the modeling kernel.
//!
//! This crate holds everything the geometry and topology layers are built on:
//!
//! - [`expansion`]: exact multi-component floating-point arithmetic
//!   (Shewchuk expansions), the substrate for exact predicate fallbacks.
//! - [`predicates`]: robust geometric predicates (`orient2d`, `orient3d`,
//!   `incircle`)
//!   with a fast floating-point filter and an exact fallback. All sign
//!   decisions in the kernel must route through these — never through raw
//!   float comparisons.
//! - [`interval`]: conservative interval arithmetic for range filtering.
//! - [`math`]: deterministic transcendental functions (sin/cos/atan2, musl
//!   port). Kernel code must use these, never platform libm — enforced via
//!   clippy `disallowed-methods`.
//! - [`operation`]: immutable session policy, per-operation configuration,
//!   deterministic work accounting, and bounded semantic diagnostics.
//! - [`tolerance`]: the session numeric regime (Parasolid-compatible:
//!   meters, 1000 m size box, 1e-8 linear / 1e-11 angular resolution) and
//!   the tolerance policy object threaded through all modeling operations.
//! - [`arena`]: generational arena storage; all kernel entities are integer
//!   handles into typed arenas.
//! - [`parallel`]: deterministic parallel primitives (index-ordered results,
//!   independent of thread count).
//! - [`proof`]: shared completion evidence for algorithms that may establish
//!   a complete answer or conservatively remain indeterminate.
//! - [`error`]: the typed error model shared by every public operation.
//!
//! # Determinism contract
//!
//! Everything in this crate is bit-deterministic: the same inputs produce the
//! same outputs on every platform, thread count, and run. No `fast-math`, no
//! hash-order iteration, no time or randomness. The `determinism` test suite
//! pins golden hashes to enforce this in CI across operating systems.

pub mod arena;
pub mod error;
pub mod expansion;
mod identifier;
pub mod interval;
pub mod math;
pub mod operation;
pub mod parallel;
pub mod plane_triple;
pub mod predicates;
pub mod proof;
pub mod tolerance;
