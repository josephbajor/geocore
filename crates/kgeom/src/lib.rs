//! `kgeom` — Layer 1 (geometry) of the modeling kernel.
//!
//! Everything here is *unbounded* geometry: curves and surfaces as pure
//! mathematical objects with a uniform evaluator protocol. Topology (which
//! parts of a surface belong to a face) lives one layer up in `ktopo`; the
//! only trimming notion in this crate is [`tess::TrimmedSurface`], a
//! lightweight surface-plus-loops used for single-face tessellation.
//!
//! Geometry classes mirror the Parasolid/XT taxonomy (spec §L1) and are kept
//! **exact**: analytic classes are never converted to NURBS.
//!
//! Module map:
//! - [`vec`], [`frame`], [`aabb`], [`bvh`], [`param`] — math and deterministic
//!   spatial-index types shared by all of L1+.
//! - [`curve`] — the [`curve::Curve`] evaluator trait and analytic curves
//!   (line, circle, ellipse).
//! - [`curve2d`] — the parameter-space curve protocol used by B-rep pcurves.
//! - [`surface`] — the [`surface::Surface`] evaluator trait and analytic
//!   surfaces (plane, cylinder, cone, sphere, torus).
//! - [`nurbs`] — B-spline/NURBS engine (basis, evaluation, knot operations,
//!   fitting) per Piegl & Tiller, *The NURBS Book*.
//! - [`project`] — closest-point projection onto curves and surfaces.
//! - [`tess`] — deterministic, tolerance-driven tessellation of trimmed faces,
//!   with typed errors when refinement limits prevent meeting the request.
//! - [`conformance`] — the evaluator conformance harness (finite-difference
//!   derivative checks, periodicity, degeneracy); used by tests here and by
//!   every future geometry class.
//!
//! Parameterization conventions follow the XT schema where the spec is
//! explicit; each class documents its convention, and all of them are
//! re-verified empirically against Parasolid during M3 (XT interchange).

pub mod aabb;
pub mod bvh;
pub mod conformance;
pub mod curve;
pub mod curve2d;
pub mod frame;
pub mod nurbs;
pub mod param;
pub mod project;
pub mod surface;
pub mod tess;
pub mod vec;
