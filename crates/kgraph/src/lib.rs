//! `kgraph` — immutable geometry identity and bounded graph evaluation.
//!
//! Pure leaf mathematics remains in `kgeom`. This crate gives those values
//! stable, typed identity and provides the dependency/evaluation boundary used
//! by future procedural geometry without depending on topology or operations.

mod class;
mod descriptor;
mod error;
mod eval;
mod graph;
mod intersection;

pub use class::{Curve2dClass, CurveClass, GeometryClassKey, SurfaceClass};
pub use descriptor::{
    Curve2dDescriptor, CurveDescriptor, GeometryDependencies, OffsetSurfaceDescriptor,
    SurfaceDescriptor,
};
pub use error::{
    EvalError, EvalResult, GeometryGraphError, GeometryGraphResult, capability as eval_capability,
    code as eval_error_code, stage as eval_stage,
};
pub use eval::{
    EvalBudgetProfile, EvalContext, EvalLimits, EvalUsage, ExactSurfaceField,
    SurfaceDerivativeOrder, SurfaceValidity, ValidityGap,
};
#[cfg(feature = "benchmark-internals")]
#[doc(hidden)]
pub use graph::GraphBuildObservation;
pub use graph::{
    Curve2dHandle, Curve2dNode, CurveHandle, CurveNode, GeometryChanges, GeometryGraph,
    GeometryRef, SurfaceHandle, SurfaceNode,
};
pub use intersection::{
    AffineParamMap1d, IntersectionCertificateError, ObliqueSphereCircleTrace,
    PairedPlaneLineResidualCertificate, PairedPlaneSphereCircleResidualCertificate, PairedTrace,
    PlaneCircleTrace, PlaneSphereCircleTrace, SPHERICAL_CIRCLE_PROOF_SEGMENTS, SphereLatitudeTrace,
    SphericalCirclePcurve, TRANSMITTED_NURBS_TRACE_PROOF_DEPTH,
    TransmittedIntersectionChartMetadata, TransmittedIntersectionCurveDescriptor,
    TransmittedNurbsIntersectionCertificate, TransmittedNurbsIntersectionCurveDescriptor,
    TransmittedNurbsIntersectionTrace, TransmittedOffsetNurbsTrace,
    TransmittedPlaneIntersectionCertificate, TransmittedPlaneNurbsIntersectionCertificate,
    TransmittedPlaneNurbsTrace, VerifiedIntersectionCarrier, VerifiedIntersectionCertificate,
    VerifiedIntersectionCurveDescriptor, certify_paired_plane_line_residuals,
    certify_paired_plane_sphere_circle_residuals,
    certify_paired_plane_sphere_oblique_circle_residuals,
    certify_transmitted_nurbs_nurbs_intersection_residuals,
    certify_transmitted_offset_nurbs_intersection_residuals,
    certify_transmitted_plane_intersection_residuals,
    certify_transmitted_plane_nurbs_intersection_residuals,
};
