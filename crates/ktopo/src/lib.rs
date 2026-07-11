//! `ktopo` — Layer 2 (topology) of the modeling kernel.
//!
//! The boundary-representation data model: the Parasolid entity hierarchy
//! (`BODY → REGION → SHELL → FACE → LOOP → FIN → EDGE → VERTEX`) over
//! generational arenas, with geometry attached by handles into `kgraph`.
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
