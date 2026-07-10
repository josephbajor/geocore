//! `ktopo` — Layer 2 (topology) of the modeling kernel.
//!
//! The boundary-representation data model: the Parasolid entity hierarchy
//! (`BODY → REGION → SHELL → FACE → LOOP → FIN → EDGE → VERTEX`) over
//! generational arenas, with geometry attached from `kgeom`.
//!
//! Module map:
//! - [`entity`] — the entity structs, typed handles, senses, and the
//!   orientation/adjacency invariants (documented there, enforced by
//!   [`check`]).
//! - [`geom`] — geometry attachment enums over the L1 classes.
//!   This includes true 2D pcurve geometry; each [`entity::Fin`] can carry
//!   its own parameter-space curve use and edge-parameter correspondence.
//! - [`store`] — the arena-backed entity store and deterministic
//!   traversals.
//! - [`euler`] — Euler operators, the only sanctioned topology edits.
//! - [`make`] — primitive body constructors.
//! - [`check`] — the body checker (structural + geometric invariants).
//! - [`btess`] — whole-body watertight tessellation.

pub mod btess;
pub mod check;
pub mod entity;
pub mod euler;
pub mod geom;
pub mod make;
pub mod store;
