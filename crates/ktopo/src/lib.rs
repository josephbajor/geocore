//! `ktopo` — Layer 2 (topology) of the modeling kernel.
//!
//! The boundary-representation data model: the Parasolid entity hierarchy
//! (`BODY → REGION → SHELL → FACE → LOOP → FIN → EDGE → VERTEX`) over
//! generational arenas, with geometry attached by handles into `kgraph`.
//!
//! # Stability boundary
//!
//! This crate is a lower kernel layer, not the supported application facade.
//! Ordinary application and product code should use `kernel`. Public raw entity
//! fields, handles, [`store::Store`], and [`transaction::AssemblyStore`] remain
//! available for in-repository kernel development and reviewed trusted adapters,
//! but their representation and assembly shape are not compatibility promises.
//! A separately announced breaking encapsulation pass may make raw fields
//! private or replace the assembly seam without changing facade behavior.
//!
//! The currently reviewed cross-crate assembly consumers are X_T
//! reconstruction and the external-oracle fixture generator in `kxt`. Adding a
//! new consumer is an architecture-boundary change. Every persisted assembly
//! still passes through checked commit; this instability notice does not create
//! an unchecked persistence path.
//!
//! Module map:
//! - [`entity`] — the entity structs, typed handles, senses, and the
//!   orientation/adjacency invariants (documented there, enforced by
//!   [`check`]).
//! - [`geom`] — compatibility names for geometry-graph descriptors.
//!   This includes true 2D pcurve geometry; each [`entity::Fin`] can carry
//!   its own parameter-space curve use and edge-parameter correspondence.
//! - [`store`] — the arena-backed entity store and deterministic
//!   traversals.
//! - `index` — committed topology ownership and shared-geometry dependency
//!   indexing used for affected-root checked commits.
//! - [`euler`] — topology-internal Euler primitives and public result types;
//!   external edits use [`transaction::Transaction`].
//! - [`make`] — primitive body constructors.
//! - [`profile`] — validated planar inputs shared by sheet and feature builders.
//! - [`tolerance`] — entity tolerance provenance and growth contracts.
//! - [`check`] — the body checker (structural + geometric invariants).
//! - [`btess`] — whole-body watertight tessellation.
//! - [`domain`] — certified conservative face UV work-box construction.
//! - `loop_proof` / `shell_proof` — checker-v2 whole-entity certificates
//!   kept private until their representation coverage is production-ready.

#[cfg(feature = "benchmark-internals")]
#[doc(hidden)]
pub mod benchmark;
pub mod btess;
pub mod check;
pub mod domain;
pub mod entity;
pub mod euler;
pub mod geom;
pub(crate) mod incidence;
pub(crate) mod index;
pub(crate) mod loop_proof;
pub mod make;
pub mod profile;
pub(crate) mod shell_proof;
pub mod store;
pub mod tolerance;
pub mod transaction;
