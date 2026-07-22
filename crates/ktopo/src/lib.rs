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
//! - [`analytic_shell`] — allocation-free, keyed preflight plans for bounded
//!   Plane/Cylinder analytic shells.
//! - [`planar`] — keyed, manifold planar-solid assembly for semantic builders.
//! - [`cylindrical_band`] — proof-ready finite cylindrical-band assembly for
//!   semantic builders retaining whole Plane/Cylinder rings.
//! - [`cylindrical_boss`] — proof-ready attachment of one capped cylindrical
//!   boss or blind pocket to a convex planar host face.
//! - [`cylindrical_host`] — proof-ready attachment of cylindrical bands whose
//!   endpoints are convex-host ports or source caps.
//! - [`cylindrical_ports`] — proof-ready reversed cylinder band joining two
//!   convex-host port faces.
//! - [`cylindrical_multishell`] — proof-ready convex planar host with one
//!   closed cylindrical cavity shell.
//! - [`convex_multishell`] — oriented convex whole-shell assembly for mixed
//!   analytic outer/cavity region layouts.
//! - [`profile`] — validated planar inputs shared by sheet and feature builders.
//! - [`tolerance`] — entity tolerance provenance and growth contracts.
//! - [`check`] — the body checker (structural + geometric invariants).
//! - [`btess`] — conforming whole-body solid and sheet tessellation.
//! - [`body_distance`] — Full-gated certified solid material-distance enclosures.
//! - [`body_properties`] — Full-gated analytic solid mass-property enclosures.
//! - [`domain`] — certified conservative face UV work-box construction.
//! - `loop_proof` / `shell_proof` / `planar_shell_proof` — checker-v2 whole-entity certificates
//!   kept private until their representation coverage is production-ready.

pub mod analytic_shell;
#[cfg(feature = "benchmark-internals")]
#[doc(hidden)]
pub mod benchmark;
pub(crate) mod body_copy;
pub mod body_distance;
pub mod body_properties;
pub mod btess;
pub mod check;
pub(crate) mod convex_containment;
pub mod convex_multishell;
pub mod cylindrical_band;
pub mod cylindrical_boss;
pub mod cylindrical_host;
pub mod cylindrical_multishell;
pub mod cylindrical_ports;
pub(crate) mod cylindrical_region_proof;
pub mod domain;
pub mod entity;
pub mod euler;
pub mod geom;
#[doc(hidden)]
pub mod graph_work;
pub(crate) mod incidence;
pub mod incidence_authority;
pub(crate) mod index;
pub(crate) mod loop_proof;
pub mod make;
pub(crate) mod mixed_region_proof;
pub mod planar;
pub mod planar_multishell;
pub(crate) mod planar_shell_proof;
pub mod profile;
pub(crate) mod region_proof;
pub(crate) mod semantic_planar_math;
pub(crate) mod semantic_planar_pair_proof;
pub(crate) mod semantic_planar_region_proof;
pub(crate) mod semantic_planar_shell_proof;
pub(crate) mod shell_proof;
pub mod store;
pub mod tolerance;
pub mod transaction;

pub use body_copy::{BodyCopyError, BodyCopyResult};
pub use transaction::{FullBodyCheck, FullCommitDecision, FullCommitRequirement};
