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
pub use graph::{
    Curve2dHandle, Curve2dNode, CurveHandle, CurveNode, GeometryChanges, GeometryGraph,
    GeometryRef, SurfaceHandle, SurfaceNode,
};
#[cfg(feature = "benchmark-internals")]
#[doc(hidden)]
pub use graph::{GraphBuildObservation, GraphValidationObservation};
pub use intersection::{
    AffineParamMap1d, IntersectionCertificateError, NurbsIntersectionTrace,
    ObliqueSphereCircleTrace, PairedPlaneLineResidualCertificate,
    PairedPlaneSphereCircleResidualCertificate, PairedTrace, PlaneCircleTrace,
    PlaneSphereCircleTrace, SPHERICAL_CIRCLE_PROOF_SEGMENTS, SphereLatitudeTrace,
    SphericalCirclePcurve, TRANSMITTED_NURBS_TRACE_PROOF_DEPTH,
    TransmittedCubicInterpolationWitnesses, TransmittedIntersectionChartMetadata,
    TransmittedIntersectionCurveDescriptor, TransmittedNurbsIntersectionCertificate,
    TransmittedNurbsIntersectionCurveDescriptor, TransmittedNurbsIntersectionTrace,
    TransmittedOffsetNurbsTrace, TransmittedOffsetPlaneTrace,
    TransmittedPlaneIntersectionCertificate, TransmittedPlaneNurbsIntersectionCertificate,
    TransmittedPlaneNurbsTrace, TransmittedQuadraticInterpolationWitnesses,
    VerifiedIntersectionCarrier, VerifiedIntersectionCertificate,
    VerifiedIntersectionCurveDescriptor, VerifiedNurbsIntersectionCertificate,
    VerifiedNurbsIntersectionCurveDescriptor, VerifiedNurbsNurbsCertificateCost,
    VerifiedSphereNurbsCertificateCost, certify_paired_plane_line_residuals,
    certify_paired_plane_sphere_circle_residuals,
    certify_paired_plane_sphere_oblique_circle_residuals,
    certify_transmitted_cubic_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_five_sample_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_nurbs_nurbs_intersection_residuals,
    certify_transmitted_offset_nurbs_intersection_residuals,
    certify_transmitted_plane_intersection_residuals,
    certify_transmitted_plane_nurbs_intersection_residuals,
    certify_transmitted_quadratic_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_seven_sample_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_two_sample_dual_offset_nurbs_intersection_residuals,
    certify_verified_dual_offset_nurbs_intersection_residuals,
    certify_verified_nurbs_nurbs_intersection_residuals,
    certify_verified_offset_nurbs_nurbs_intersection_residuals,
    certify_verified_offset_nurbs_plane_intersection_residuals,
    certify_verified_plane_nurbs_intersection_residuals,
    certify_verified_sphere_nurbs_intersection_residuals,
    reissue_verified_nurbs_intersection_residuals,
    transmitted_nurbs_intersection_has_rigid_copy_recertifier,
    verified_dual_offset_nurbs_intersection_certificate_cost,
    verified_nurbs_nurbs_intersection_certificate_cost,
    verified_offset_nurbs_nurbs_intersection_certificate_cost,
    verified_offset_nurbs_plane_intersection_certificate_cost,
    verified_plane_nurbs_intersection_certificate_work,
    verified_sphere_nurbs_intersection_certificate_cost,
};
