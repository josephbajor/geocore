//! B-spline / NURBS engine.
//!
//! Algorithms follow Piegl & Tiller, *The NURBS Book* (2nd ed.); doc
//! comments cite algorithm numbers (`A2.1` etc.). The engine covers, for M1:
//!
//! - validated knot vectors, span lookup, multiplicity queries ([`KnotVector`]);
//! - basis functions and derivatives to order 3 ([`basis`]);
//! - polynomial and rational curves ([`NurbsCurve`]) implementing the
//!   [`crate::curve::Curve`] evaluator protocol exactly;
//! - polynomial and rational tensor-product surfaces ([`NurbsSurface`])
//!   implementing [`crate::surface::Surface`];
//! - homogeneous knot insertion/refinement, exact curve and surface
//!   splitting/restriction, and curve-segment/surface-patch Bezier extraction;
//! - bounded source-provenanced NURBS curve-pair subdivision with conservative
//!   contact covers and structured operation accounting;
//! - global curve interpolation ([`interpolate`]).
//!
//! Deliberately deferred (with rationale):
//! - **general knot removal and degree elevation** — first needed by
//!   loft/surface compatibility in M6. A narrower checked inverse-refinement
//!   proof helper exists for exact curve-pair representation equivalence;
//! - **periodic NURBS** — XT B-geometry can be periodic; support lands with
//!   the XT reader (M3) when real periodic inputs exist to test against.
//!   Until then [`NurbsCurve`] reports `periodicity() == None`.
//! - **degenerate patch detection** (collapsed control-point edges) — the
//!   topology checker's job in M2; `degeneracies()` returns empty for now.

pub mod basis;
mod curve_pair;
mod fit;
mod inverse_refinement;
mod knots;
mod ncurve;
mod nsurface;
pub(crate) mod ops;
mod patch_bvh;
mod source_range_interval;
mod spatial_algebraic_correspondence;
mod spatial_curve_pair;
mod spatial_exact_sample;
mod spatial_interior_root;
mod surface_range_interval;

pub use curve_pair::{
    ContextCurvePairIsolationError, CurvePairAlgebraicSearchConfig,
    CurvePairAlgebraicSearchConfigError, CurvePairCandidateCell, CurvePairIsolation,
    CurvePairIsolationLimits, CurvePairProjectionPlane, CurvePairRootCertificate,
    NURBS_CURVE_PAIR_CANDIDATES, NURBS_CURVE_PAIR_DEPTH, NURBS_CURVE_PAIR_SUBDIVISIONS,
    NurbsCurvePairBudgetProfile, certify_curve_pair_unique_root,
    certify_curve_pair_unique_root_with_config, isolate_curve_pair_candidates_in_scope,
};
pub use fit::interpolate;
pub use inverse_refinement::{CHECKED_REFINEMENT_ANCESTOR_LIMIT, checked_refinement_ancestors};
pub use knots::KnotVector;
pub use ncurve::NurbsCurve;
pub use nsurface::NurbsSurface;
pub use patch_bvh::{
    ContextImplicitIsolationError, ImplicitCandidateCell, ImplicitIsolationLimits,
    ImplicitPatchIsolation, NURBS_IMPLICIT_ISOLATION_CANDIDATE_LIMIT,
    NURBS_IMPLICIT_ISOLATION_CANDIDATES, NURBS_IMPLICIT_ISOLATION_DEPTH,
    NURBS_IMPLICIT_ISOLATION_DEPTH_LIMIT, NURBS_IMPLICIT_ISOLATION_NUMERIC_RESOLUTION,
    NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT, NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
    NurbsSurfaceBvh, PlanePatchRelation,
};
pub use source_range_interval::{PlaneCurveRangeRelation, classify_curve_range_against_plane_slab};
pub use surface_range_interval::NurbsSurfaceSourceDifferentialEnclosure;
