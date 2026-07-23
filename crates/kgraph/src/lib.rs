//! `kgraph` — immutable geometry identity and bounded graph evaluation.
//!
//! Pure leaf mathematics remains in `kgeom`. This crate gives those values
//! stable, typed identity and provides the dependency/evaluation boundary used
//! by future procedural geometry without depending on topology or operations.

mod class;
#[path = "intersection/cylinder_cylinder_ruling.rs"]
mod cylinder_cylinder_ruling;
mod descriptor;
mod error;
mod eval;
mod graph;
mod intersection;
#[path = "intersection/plane_cylinder_ruling.rs"]
mod plane_cylinder_ruling;
#[path = "intersection/skew_cylinder_branch.rs"]
mod skew_cylinder_branch;

pub use class::{Curve2dClass, CurveClass, GeometryClassKey, SurfaceClass};
pub use cylinder_cylinder_ruling::{
    PairedCylinderCylinderRulingResidualCertificate,
    certify_paired_cylinder_cylinder_ruling_residuals,
};
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
    AffineParamMap1d, CylinderLongitudeTrace, IntersectionCertificateError, NurbsIntersectionTrace,
    ObliqueSphereCircleTrace, PairedPlaneCylinderCircleResidualCertificate,
    PairedPlaneLineResidualCertificate, PairedPlaneSphereCircleResidualCertificate, PairedTrace,
    PlaneCircleTrace, PlaneCylinderCircleTrace, PlaneSphereCircleTrace,
    SPHERICAL_CIRCLE_PROOF_SEGMENTS, SphereLatitudeTrace, SphericalCirclePcurve,
    TRANSMITTED_NURBS_TRACE_PROOF_DEPTH, TransmittedCubicInterpolationWitnesses,
    TransmittedIntersectionChartMetadata, TransmittedIntersectionCurveDescriptor,
    TransmittedNurbsIntersectionCertificate, TransmittedNurbsIntersectionCurveDescriptor,
    TransmittedNurbsIntersectionTrace, TransmittedOffsetNurbsTrace, TransmittedOffsetPlaneTrace,
    TransmittedPlaneIntersectionCertificate, TransmittedPlaneNurbsIntersectionCertificate,
    TransmittedPlaneNurbsTrace, TransmittedQuadraticInterpolationWitnesses,
    VerifiedIntersectionCarrier, VerifiedIntersectionCertificate,
    VerifiedIntersectionCurveDescriptor, VerifiedNurbsIntersectionCertificate,
    VerifiedNurbsIntersectionCurveDescriptor, VerifiedNurbsNurbsCertificateCost,
    VerifiedSphereNurbsCertificateCost, certify_paired_plane_cylinder_circle_residuals,
    certify_paired_plane_line_residuals, certify_paired_plane_sphere_circle_residuals,
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
    certify_verified_sphere_nurbs_intersection_residuals, intersection_certificate_capability,
    intersection_certificate_error_code, reissue_verified_nurbs_intersection_residuals,
    transmitted_nurbs_intersection_has_rigid_copy_recertifier,
    verified_dual_offset_nurbs_intersection_certificate_cost,
    verified_nurbs_nurbs_intersection_certificate_cost,
    verified_offset_nurbs_nurbs_intersection_certificate_cost,
    verified_offset_nurbs_plane_intersection_certificate_cost,
    verified_plane_nurbs_intersection_certificate_work,
    verified_sphere_nurbs_intersection_certificate_cost,
};
pub use plane_cylinder_ruling::{
    CylinderRulingTrace, PairedPlaneCylinderRulingResidualCertificate, PlaneCylinderRulingTrace,
    PlaneRulingTrace, certify_paired_plane_cylinder_ruling_residuals,
};
pub use skew_cylinder_branch::{
    PairedSkewCylinderBranchResidualCertificate, SKEW_CYLINDER_BRANCH_CERTIFICATE_WORK,
    SKEW_CYLINDER_BRANCH_PCURVE_ALL_CELLS_WORK, SKEW_CYLINDER_BRANCH_PCURVE_CELL_WORK,
    SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK, SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS,
    SkewCylinderBranchCarrier, SkewCylinderBranchGuardedEnd, SkewCylinderBranchPcurve,
    SkewCylinderBranchPcurveCellCertificate, SkewCylinderBranchPcurveEnclosure,
    SkewCylinderBranchPcurveRootCorridorCertificate, SkewCylinderBranchTrace, SkewCylinderSheet,
    certify_paired_skew_cylinder_branch_residuals,
    certify_paired_skew_cylinder_branch_subrange_residuals,
};
