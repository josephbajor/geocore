//! Graph-aware, verified surface-intersection adapter.
//!
//! Direct planes/spheres and safe constant-offset chains terminating at those
//! leaves form exact analytic field families. Genuinely non-planar direct NURBS
//! surfaces additionally support scoped exact-plane-field/NURBS, exact
//! sphere-field/NURBS, compatible direct NURBS/NURBS marching, and a narrow
//! constant-normal Offset(NURBS)/NURBS family capped at four offset
//! descriptors. One exact rational quarter-cylinder family additionally
//! supports a single varying-normal Offset(NURBS) root against a canonical
//! direct planar NURBS peer.
//! Strictly separated pairs of compatible constant-normal Offset(NURBS) roots
//! capped at four offset descriptors additionally own a graph-level
//! complete-empty proof. The adapter
//! promotes discovered branches only after
//! constructing both pcurves and proving their paired whole-interval residual
//! contracts. Common-axis circles retain their longitude/latitude fast path;
//! other finite secants use a certified nonlinear spherical pcurve and fail
//! closed at chart singularities or outside windows.

use super::error::IntersectionError;
use super::nurbs_nurbs_surface::{
    intersect_bounded_nurbs_nurbs_surfaces_with_traces_in_scope,
    intersect_bounded_offset_nurbs_nurbs_surfaces_with_traces_in_scope,
    supports_constant_normal_offset_nurbs_nurbs_surface_pair,
    supports_direct_nurbs_nurbs_surface_pair,
    supports_strictly_separated_constant_normal_offset_nurbs_pair,
    supports_varying_normal_offset_nurbs_nurbs_surface_pair,
    varying_normal_offset_window_proof_work,
};
use super::nurbs_surface_march::{
    ContextMarchError, MarchOutput, MarchTrace, NURBS_SURFACE_MARCH_SAMPLE_LIMIT,
    NurbsSurfaceMarchBudgetProfile,
};
use super::plane_nurbs_surface::intersect_bounded_plane_nurbs_surface_with_traces_in_scope;
use super::plane_plane::intersect_bounded_planes;
use super::plane_sphere::intersect_bounded_plane_sphere;
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
};
use super::sphere_nurbs_surface::intersect_bounded_sphere_nurbs_surface_with_traces_in_scope;
use core::fmt;
use kcore::error::{CapabilityId, ClassifiedError, ErrorClass, ErrorCode};
use kcore::interval::Interval;
use kcore::math;
use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticKind, LimitSnapshot, LimitSpec, OperationContext,
    OperationOutcome, OperationPolicyError, OperationScope, ResourceKind, SequentialWorkLedger,
    SessionPolicy, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Line};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere};
use kgeom::vec::{Point3, Vec2};
use kgraph::{
    AffineParamMap1d, Curve2dDescriptor, Curve2dHandle, CurveDescriptor, CurveHandle,
    EvalBudgetProfile, EvalContext, EvalError, EvalLimits, EvalUsage, ExactSurfaceField,
    GeometryGraph, GeometryGraphError, GeometryRef, IntersectionCertificateError,
    NurbsIntersectionTrace, PairedTrace, PlaneCircleTrace, PlaneSphereCircleTrace,
    SphereLatitudeTrace, SurfaceDescriptor, SurfaceHandle, VerifiedIntersectionCertificate,
    VerifiedNurbsIntersectionCertificate, certify_paired_plane_line_residuals,
    certify_paired_plane_sphere_circle_residuals,
    certify_paired_plane_sphere_oblique_circle_residuals,
    certify_verified_nurbs_nurbs_intersection_residuals,
    certify_verified_offset_nurbs_nurbs_intersection_residuals,
    certify_verified_plane_nurbs_intersection_residuals,
    certify_verified_sphere_nurbs_intersection_residuals,
    verified_nurbs_nurbs_intersection_certificate_cost,
    verified_offset_nurbs_nurbs_intersection_certificate_cost,
    verified_plane_nurbs_intersection_certificate_work,
    verified_sphere_nurbs_intersection_certificate_cost,
};

const MAX_SPHERICAL_CIRCLE_PROOFS_PER_QUERY: u64 = 4_096;
const MAX_NURBS_TRACE_CERTIFICATE_WORK_PER_QUERY: u64 = 134_217_728;
const MAX_NURBS_TRACE_CERTIFICATE_ITEMS_PER_QUERY: u64 = 16_777_216;
const MAX_OFFSET_NURBS_INTERSECTION_CHAIN_LENGTH: usize = 4;
const MAX_DUAL_OFFSET_NURBS_EMPTY_CHAIN_LENGTH: usize = 4;

/// Stable work stage for fixed whole-branch inverse sphere-chart subdivisions.
pub const SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS: StageId =
    match StageId::new("kops.intersect.spherical-circle-proof-subdivisions") {
        Ok(stage) => stage,
        Err(_) => panic!("valid spherical-circle proof stage"),
    };

/// Stable resource stage for fixed-depth whole-range analytic/NURBS proofs.
pub const NURBS_TRACE_CERTIFICATE_WORK: StageId =
    match StageId::new("kops.intersect.nurbs-trace-certificate-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid NURBS trace-certificate stage"),
    };

/// Version-1 composed budget for graph-owned surface intersection.
#[derive(Debug, Clone, Copy, Default)]
pub struct GraphSurfaceBudgetProfile;

impl GraphSurfaceBudgetProfile {
    /// Graph evaluation, scoped NURBS-surface marching, and bounded
    /// whole-range branch proofs.
    pub fn v1_defaults() -> BudgetPlan {
        let evaluation = EvalBudgetProfile::v1_defaults();
        let marcher = NurbsSurfaceMarchBudgetProfile::v1_defaults();
        BudgetPlan::new(
            evaluation
                .limits()
                .iter()
                .copied()
                .chain(marcher.limits().iter().copied())
                .chain([
                    LimitSpec::new(
                        SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        kgraph::SPHERICAL_CIRCLE_PROOF_SEGMENTS as u64
                            * MAX_SPHERICAL_CIRCLE_PROOFS_PER_QUERY,
                    ),
                    LimitSpec::new(
                        NURBS_TRACE_CERTIFICATE_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        MAX_NURBS_TRACE_CERTIFICATE_WORK_PER_QUERY,
                    ),
                    LimitSpec::new(
                        NURBS_TRACE_CERTIFICATE_WORK,
                        ResourceKind::Items,
                        AccountingMode::HighWater,
                        MAX_NURBS_TRACE_CERTIFICATE_ITEMS_PER_QUERY,
                    ),
                    LimitSpec::new(
                        NURBS_TRACE_CERTIFICATE_WORK,
                        ResourceKind::Depth,
                        AccountingMode::HighWater,
                        kgraph::TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
                    ),
                ]),
        )
        .expect("built-in graph surface-intersection budget is valid")
    }
}

const fn error_code(value: &'static str) -> ErrorCode {
    match ErrorCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in graph intersection error code"),
    }
}

/// Stable failure identity when a solver discovery cannot be promoted to a
/// verified whole-interval branch.
pub const BRANCH_CERTIFICATE_FAILURE: ErrorCode =
    error_code("kops.intersect.branch-certificate-failure");

/// Stable failure identity when verified branch persistence cannot commit.
pub const PERSISTENT_DESCRIPTOR_FAILURE: ErrorCode =
    error_code("kops.intersect.persistent-descriptor-failure");

/// Failure boundary for graph-owned surface intersection.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum GraphSurfaceIntersectionError {
    /// A graph source handle could not be resolved.
    GeometryEvaluation(EvalError),
    /// The lower solver or support-matrix boundary failed.
    Intersection(IntersectionError),
    /// A discovered branch failed whole-interval promotion.
    BranchCertificate(IntersectionCertificateError),
    /// Context composition or bounded graph work failed.
    OperationPolicy(OperationPolicyError),
    /// A verified branch could not be persisted into the geometry graph.
    GeometryPersistence(GeometryGraphError),
}

impl GraphSurfaceIntersectionError {
    /// Broad semantic error class.
    pub const fn class(&self) -> ErrorClass {
        match self {
            Self::GeometryEvaluation(error) => error.class(),
            Self::Intersection(error) => error.class(),
            Self::BranchCertificate(
                IntersectionCertificateError::SingularSphereChart { .. }
                | IntersectionCertificateError::SphereTraceOutsideWindow { .. },
            ) => ErrorClass::Unsupported,
            Self::BranchCertificate(_) => ErrorClass::InternalInvariant,
            Self::OperationPolicy(error) => error.class(),
            Self::GeometryPersistence(error) => match error {
                GeometryGraphError::StaleGeometryHandle { .. } => ErrorClass::InvalidInput,
                GeometryGraphError::HasDependents { .. } => ErrorClass::InvalidState,
                GeometryGraphError::InvalidDescriptor { .. }
                | GeometryGraphError::DependencyCycle { .. }
                | GeometryGraphError::ReverseDependencyMismatch { .. } => {
                    ErrorClass::InternalInvariant
                }
                _ => ErrorClass::InternalInvariant,
            },
        }
    }

    /// Stable machine-readable failure identity.
    pub const fn code(&self) -> ErrorCode {
        match self {
            Self::GeometryEvaluation(error) => error.code(),
            Self::Intersection(error) => error.code(),
            Self::BranchCertificate(_) => BRANCH_CERTIFICATE_FAILURE,
            Self::OperationPolicy(error) => error.code(),
            Self::GeometryPersistence(_) => PERSISTENT_DESCRIPTOR_FAILURE,
        }
    }
}

impl fmt::Display for GraphSurfaceIntersectionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::GeometryEvaluation(error) => {
                write!(formatter, "geometry graph evaluation failed: {error}")
            }
            Self::Intersection(error) => error.fmt(formatter),
            Self::BranchCertificate(error) => {
                write!(
                    formatter,
                    "intersection branch certification failed: {error}"
                )
            }
            Self::OperationPolicy(error) => {
                write!(formatter, "graph intersection policy failed: {error}")
            }
            Self::GeometryPersistence(error) => {
                write!(
                    formatter,
                    "verified intersection persistence failed: {error}"
                )
            }
        }
    }
}

impl std::error::Error for GraphSurfaceIntersectionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::GeometryEvaluation(error) => Some(error),
            Self::Intersection(error) => Some(error),
            Self::BranchCertificate(error) => Some(error),
            Self::OperationPolicy(error) => Some(error),
            Self::GeometryPersistence(error) => Some(error),
        }
    }
}

impl ClassifiedError for GraphSurfaceIntersectionError {
    fn class(&self) -> ErrorClass {
        self.class()
    }

    fn code(&self) -> ErrorCode {
        self.code()
    }

    fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::GeometryEvaluation(error) => error.capability(),
            Self::Intersection(error) => error.capability(),
            Self::BranchCertificate(_) => None,
            Self::OperationPolicy(error) => error.capability(),
            Self::GeometryPersistence(_) => None,
        }
    }

    fn limit(&self) -> Option<kcore::operation::LimitSnapshot> {
        match self {
            Self::GeometryEvaluation(error) => error.limit(),
            Self::Intersection(error) => error.limit(),
            Self::BranchCertificate(_) => None,
            Self::OperationPolicy(error) => error.limit(),
            Self::GeometryPersistence(_) => None,
        }
    }
}

impl From<IntersectionError> for GraphSurfaceIntersectionError {
    fn from(error: IntersectionError) -> Self {
        Self::Intersection(error)
    }
}

impl From<OperationPolicyError> for GraphSurfaceIntersectionError {
    fn from(error: OperationPolicyError) -> Self {
        Self::OperationPolicy(error)
    }
}

impl From<GeometryGraphError> for GraphSurfaceIntersectionError {
    fn from(error: GeometryGraphError) -> Self {
        Self::GeometryPersistence(error)
    }
}

/// Result boundary for graph-owned surface intersection.
pub type GraphSurfaceIntersectionResult<T> = core::result::Result<T, GraphSurfaceIntersectionError>;

/// Why a vertex exists in the operation-local branch graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntersectionBranchVertexEvent {
    /// A complete solver result contains an isolated contact.
    IsolatedContact,
    /// A positive-length branch meets one or both requested surface windows.
    BoundaryEndpoint {
        /// Which source surface windows clip this endpoint.
        surfaces: [bool; 2],
    },
}

/// End condition for one endpoint of an open intersection branch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntersectionBranchEndpointEvent {
    /// The branch is clipped by at least one requested surface window.
    SurfaceWindowBoundary {
        /// Which source surface windows clip this endpoint.
        surfaces: [bool; 2],
    },
}

/// One deterministic operation-local branch-graph vertex.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntersectionBranchVertex {
    /// Model-space location.
    pub point: Point3,
    /// Parameters on the source surfaces, in operand order.
    pub surface_parameters: [[f64; 2]; 2],
    /// Local contact character.
    pub kind: ContactKind,
    /// Structural reason for retaining this vertex.
    pub event: IntersectionBranchVertexEvent,
}

/// One certified positive-length operation-local intersection branch.
#[derive(Debug, Clone, PartialEq)]
pub struct IntersectionBranchEdge {
    /// Source surface identities, in operand order.
    pub source_surfaces: [SurfaceHandle; 2],
    /// Graph-ready exact carrier descriptor.
    pub carrier: CurveDescriptor,
    /// Active finite interval on the carrier.
    pub carrier_range: ParamRange,
    /// Graph-ready pcurve descriptors, in operand order.
    pub pcurves: [Curve2dDescriptor; 2],
    /// Carrier-to-pcurve parameter maps, in operand order.
    pub parameter_maps: [AffineParamMap1d; 2],
    /// Indices of the low/high carrier-range endpoint vertices.
    pub endpoint_vertices: [usize; 2],
    /// End conditions corresponding to `endpoint_vertices`.
    pub endpoint_events: [IntersectionBranchEndpointEvent; 2],
    /// Local contact character along the branch.
    pub kind: ContactKind,
    /// Whole-interval proof covering both lifted pcurve traces.
    pub certificate: IntersectionBranchCertificate,
}

/// Whole-range proof retained by one operation-local branch.
#[derive(Debug, Clone, PartialEq)]
pub enum IntersectionBranchCertificate {
    /// Existing exact analytic line/circle proof family.
    Analytic(Box<VerifiedIntersectionCertificate>),
    /// Operation-generated degree-1 analytic/NURBS trace proof.
    Nurbs(Box<VerifiedNurbsIntersectionCertificate>),
}

impl IntersectionBranchCertificate {
    /// Conservative paired residual bounds in operand order.
    pub fn residual_bounds(&self) -> [f64; 2] {
        match self {
            Self::Analytic(certificate) => certificate.residual_bounds(),
            Self::Nurbs(certificate) => certificate.residual_bounds(),
        }
    }

    /// Model-space tolerance used by the proof.
    pub fn tolerance(&self) -> f64 {
        match self {
            Self::Analytic(certificate) => certificate.tolerance(),
            Self::Nurbs(certificate) => certificate.tolerance(),
        }
    }

    /// Borrow the analytic plane-line proof when it matches.
    pub fn as_plane_line(&self) -> Option<kgraph::PairedPlaneLineResidualCertificate> {
        match self {
            Self::Analytic(certificate) => certificate.as_plane_line(),
            Self::Nurbs(_) => None,
        }
    }

    /// Borrow the analytic plane/sphere proof when it matches.
    pub fn as_plane_sphere_circle(
        &self,
    ) -> Option<kgraph::PairedPlaneSphereCircleResidualCertificate> {
        match self {
            Self::Analytic(certificate) => certificate.as_plane_sphere_circle(),
            Self::Nurbs(_) => None,
        }
    }

    /// Borrow the operation-generated analytic/NURBS proof when it matches.
    pub fn as_nurbs(&self) -> Option<&VerifiedNurbsIntersectionCertificate> {
        match self {
            Self::Analytic(_) => None,
            Self::Nurbs(certificate) => Some(certificate.as_ref()),
        }
    }
}

/// Deterministic verified branch graph derived from one solver result.
#[derive(Debug, Clone, PartialEq)]
pub struct IntersectionBranchGraph {
    /// Source surfaces for the complete query, including misses.
    pub source_surfaces: [SurfaceHandle; 2],
    /// Isolated contacts and branch endpoints.
    pub vertices: Vec<IntersectionBranchVertex>,
    /// Certified positive-length branches.
    pub edges: Vec<IntersectionBranchEdge>,
}

/// Legacy solver evidence paired with its operation-local verified graph.
#[derive(Debug, Clone, PartialEq)]
pub struct GraphSurfaceSurfaceIntersections {
    /// Unchanged result returned by the authoritative lower solver.
    pub raw: SurfaceSurfaceIntersections,
    /// Verified operation-local branch graph derived from `raw`.
    pub branch_graph: IntersectionBranchGraph,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ResolvedGraphSurfaceField<'a> {
    Plane {
        surface: Plane,
        direct: bool,
    },
    Sphere {
        surface: Sphere,
    },
    Nurbs(&'a NurbsSurface),
    OffsetNurbs {
        signed_distance: f64,
        basis: &'a NurbsSurface,
        chain_length: usize,
    },
}

/// One branch whose carrier and paired pcurves are persistent graph nodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistentIntersectionBranchEdge {
    /// Verified finite intersection-curve node.
    pub curve: CurveHandle,
    /// Paired pcurve nodes in source operand order.
    pub pcurves: [Curve2dHandle; 2],
    /// Indices of the low/high carrier endpoint vertices.
    pub endpoint_vertices: [usize; 2],
    /// End conditions corresponding to `endpoint_vertices`.
    pub endpoint_events: [IntersectionBranchEndpointEvent; 2],
    /// Local contact character along the branch.
    pub kind: ContactKind,
}

/// Verified branch topology paired with persistent graph descriptor handles.
#[derive(Debug, Clone, PartialEq)]
pub struct PersistentIntersectionBranchGraph {
    /// Source surfaces for the complete query, including misses.
    pub source_surfaces: [SurfaceHandle; 2],
    /// Isolated contacts and branch endpoints in deterministic order.
    pub vertices: Vec<IntersectionBranchVertex>,
    /// Persistent positive-length branches in solver order.
    pub edges: Vec<PersistentIntersectionBranchEdge>,
}

/// Intersect graph-owned surfaces using the default operation policy.
pub fn intersect_bounded_graph_surfaces(
    graph: &GeometryGraph,
    surface_a: SurfaceHandle,
    range_a: [ParamRange; 2],
    surface_b: SurfaceHandle,
    range_b: [ParamRange; 2],
    tolerances: Tolerances,
) -> GraphSurfaceIntersectionResult<GraphSurfaceSurfaceIntersections> {
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances)
        .expect("validated Tolerances always satisfy v1 session precision");
    intersect_bounded_graph_surfaces_with_context(
        graph, surface_a, range_a, surface_b, range_b, &context,
    )
    .into_result()
}

/// Intersect graph-owned surfaces with caller-owned policy and reporting.
pub fn intersect_bounded_graph_surfaces_with_context(
    graph: &GeometryGraph,
    surface_a: SurfaceHandle,
    range_a: [ParamRange; 2],
    surface_b: SurfaceHandle,
    range_b: [ParamRange; 2],
    context: &OperationContext<'_>,
) -> OperationOutcome<GraphSurfaceSurfaceIntersections, GraphSurfaceIntersectionError> {
    let context = context
        .clone()
        .with_family_budget_defaults(GraphSurfaceBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    let result = intersect_bounded_graph_surfaces_in_scope(
        graph, surface_a, range_a, surface_b, range_b, &mut scope,
    );
    scope.finish_typed(result)
}

/// Intersect graph-owned surfaces inside an existing owner operation scope.
///
/// This function never creates or finishes a nested scope. Direct descriptor
/// classes are resolved exactly once per operand. Direct planes/spheres and
/// their safe constant-offset chains form exact fields. Plane/plane, finite
/// regular-chart plane/sphere, exact-plane-field/genuinely-non-planar-
/// direct-NURBS, exact-Sphere-field/genuinely-non-planar-direct-NURBS, and
/// compatible genuinely-non-planar direct-NURBS/direct-NURBS branches are
/// supported. Constant-normal Offset(NURBS)/NURBS roots containing at most
/// four offset descriptors additionally reuse the compatible paired marcher
/// across the positive-area overlap of distinct operand windows. Two
/// compatible constant-normal Offset(NURBS) roots containing at most four
/// offset descriptors return a complete miss only from strict outward
/// original-control separation; coincident or intersecting effective sheets
/// and all other pairs remain explicitly unsupported. A single varying-normal
/// rational quarter-cylinder offset additionally marches against one canonical
/// direct planar NURBS peer after a whole-window original-derivative normal
/// proof; nested varying-normal roots remain unsupported.
/// Owners must compose [`GraphSurfaceBudgetProfile::v1_defaults`] before
/// creating `scope` when they may dispatch a scoped proof-bearing branch.
pub fn intersect_bounded_graph_surfaces_in_scope(
    graph: &GeometryGraph,
    surface_a: SurfaceHandle,
    range_a: [ParamRange; 2],
    surface_b: SurfaceHandle,
    range_b: [ParamRange; 2],
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<GraphSurfaceSurfaceIntersections> {
    let descriptor_a = graph.surface(surface_a).ok_or({
        GraphSurfaceIntersectionError::GeometryEvaluation(EvalError::StaleGeometryHandle {
            geometry: GeometryRef::Surface(surface_a),
        })
    })?;
    let descriptor_b = graph.surface(surface_b).ok_or({
        GraphSurfaceIntersectionError::GeometryEvaluation(EvalError::StaleGeometryHandle {
            geometry: GeometryRef::Surface(surface_b),
        })
    })?;
    let classes = [descriptor_a.class_key(), descriptor_b.class_key()];
    let field_a = resolve_exact_surface_field(graph, surface_a, descriptor_a, scope)?;
    let field_b = resolve_exact_surface_field(graph, surface_b, descriptor_b, scope)?;
    let unsupported = || {
        GraphSurfaceIntersectionError::Intersection(IntersectionError::UnsupportedSurfacePair {
            class_a: Some(classes[0]),
            class_b: Some(classes[1]),
        })
    };
    let tolerances = scope.context().tolerances();
    let fields = match (field_a, field_b) {
        (
            Some(ResolvedGraphSurfaceField::Plane {
                surface: plane_a,
                direct: direct_a,
            }),
            Some(ResolvedGraphSurfaceField::Plane {
                surface: plane_b,
                direct: direct_b,
            }),
        ) => [
            ResolvedGraphSurfaceField::Plane {
                surface: plane_a,
                direct: direct_a,
            },
            ResolvedGraphSurfaceField::Plane {
                surface: plane_b,
                direct: direct_b,
            },
        ],
        (
            Some(ResolvedGraphSurfaceField::Plane {
                surface: plane,
                direct: direct_plane,
            }),
            Some(ResolvedGraphSurfaceField::Sphere { surface: sphere }),
        ) => [
            ResolvedGraphSurfaceField::Plane {
                surface: plane,
                direct: direct_plane,
            },
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
        ],
        (
            Some(ResolvedGraphSurfaceField::Sphere { surface: sphere }),
            Some(ResolvedGraphSurfaceField::Plane {
                surface: plane,
                direct: direct_plane,
            }),
        ) => [
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
            ResolvedGraphSurfaceField::Plane {
                surface: plane,
                direct: direct_plane,
            },
        ],
        (
            Some(ResolvedGraphSurfaceField::Plane {
                surface: plane,
                direct,
            }),
            Some(ResolvedGraphSurfaceField::Nurbs(surface)),
        ) if nurbs_control_net_is_nonplanar(surface, tolerances.linear()) => [
            ResolvedGraphSurfaceField::Plane {
                surface: plane,
                direct,
            },
            ResolvedGraphSurfaceField::Nurbs(surface),
        ],
        (
            Some(ResolvedGraphSurfaceField::Nurbs(surface)),
            Some(ResolvedGraphSurfaceField::Plane {
                surface: plane,
                direct,
            }),
        ) if nurbs_control_net_is_nonplanar(surface, tolerances.linear()) => [
            ResolvedGraphSurfaceField::Nurbs(surface),
            ResolvedGraphSurfaceField::Plane {
                surface: plane,
                direct,
            },
        ],
        (
            Some(ResolvedGraphSurfaceField::Sphere { surface: sphere }),
            Some(ResolvedGraphSurfaceField::Nurbs(surface)),
        ) if nurbs_control_net_is_nonplanar(surface, tolerances.linear()) => [
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
            ResolvedGraphSurfaceField::Nurbs(surface),
        ],
        (
            Some(ResolvedGraphSurfaceField::Nurbs(surface)),
            Some(ResolvedGraphSurfaceField::Sphere { surface: sphere }),
        ) if nurbs_control_net_is_nonplanar(surface, tolerances.linear()) => [
            ResolvedGraphSurfaceField::Nurbs(surface),
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
        ],
        (
            Some(ResolvedGraphSurfaceField::Nurbs(surface_a)),
            Some(ResolvedGraphSurfaceField::Nurbs(surface_b)),
        ) if nurbs_control_net_is_nonplanar(surface_a, tolerances.linear())
            && nurbs_control_net_is_nonplanar(surface_b, tolerances.linear())
            && supports_direct_nurbs_nurbs_surface_pair(surface_a, range_a, surface_b, range_b) =>
        {
            [
                ResolvedGraphSurfaceField::Nurbs(surface_a),
                ResolvedGraphSurfaceField::Nurbs(surface_b),
            ]
        }
        (
            Some(ResolvedGraphSurfaceField::OffsetNurbs {
                signed_distance,
                basis,
                chain_length,
            }),
            Some(ResolvedGraphSurfaceField::Nurbs(surface)),
        ) if (nurbs_control_net_is_nonplanar(surface, tolerances.linear())
            && supports_constant_normal_offset_nurbs_nurbs_surface_pair(
                basis,
                signed_distance,
                range_a,
                surface,
                range_b,
            ))
            || (chain_length == 1
                && supports_varying_normal_offset_nurbs_nurbs_surface_pair(
                    basis,
                    signed_distance,
                    range_a,
                    surface,
                    range_b,
                )) =>
        {
            [
                ResolvedGraphSurfaceField::OffsetNurbs {
                    signed_distance,
                    basis,
                    chain_length,
                },
                ResolvedGraphSurfaceField::Nurbs(surface),
            ]
        }
        (
            Some(ResolvedGraphSurfaceField::Nurbs(surface)),
            Some(ResolvedGraphSurfaceField::OffsetNurbs {
                signed_distance,
                basis,
                chain_length,
            }),
        ) if (nurbs_control_net_is_nonplanar(surface, tolerances.linear())
            && supports_constant_normal_offset_nurbs_nurbs_surface_pair(
                basis,
                signed_distance,
                range_b,
                surface,
                range_a,
            ))
            || (chain_length == 1
                && supports_varying_normal_offset_nurbs_nurbs_surface_pair(
                    basis,
                    signed_distance,
                    range_b,
                    surface,
                    range_a,
                )) =>
        {
            [
                ResolvedGraphSurfaceField::Nurbs(surface),
                ResolvedGraphSurfaceField::OffsetNurbs {
                    signed_distance,
                    basis,
                    chain_length,
                },
            ]
        }
        (
            Some(ResolvedGraphSurfaceField::OffsetNurbs {
                signed_distance: signed_distance_a,
                basis: basis_a,
                chain_length: chain_length_a,
            }),
            Some(ResolvedGraphSurfaceField::OffsetNurbs {
                signed_distance: signed_distance_b,
                basis: basis_b,
                chain_length: chain_length_b,
            }),
        ) if chain_length_a <= MAX_DUAL_OFFSET_NURBS_EMPTY_CHAIN_LENGTH
            && chain_length_b <= MAX_DUAL_OFFSET_NURBS_EMPTY_CHAIN_LENGTH
            && supports_strictly_separated_constant_normal_offset_nurbs_pair(
                basis_a,
                signed_distance_a,
                range_a,
                basis_b,
                signed_distance_b,
                range_b,
            ) =>
        {
            [
                ResolvedGraphSurfaceField::OffsetNurbs {
                    signed_distance: signed_distance_a,
                    basis: basis_a,
                    chain_length: chain_length_a,
                },
                ResolvedGraphSurfaceField::OffsetNurbs {
                    signed_distance: signed_distance_b,
                    basis: basis_b,
                    chain_length: chain_length_b,
                },
            ]
        }
        _ => return Err(unsupported()),
    };
    let (raw, march_traces) = match fields {
        [
            ResolvedGraphSurfaceField::Plane {
                surface: plane_a, ..
            },
            ResolvedGraphSurfaceField::Plane {
                surface: plane_b, ..
            },
        ] => (
            intersect_bounded_planes(&plane_a, range_a, &plane_b, range_b, tolerances)
                .map_err(IntersectionError::from)
                .map_err(GraphSurfaceIntersectionError::Intersection)?,
            None,
        ),
        [
            ResolvedGraphSurfaceField::Plane { surface: plane, .. },
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
        ] => (
            intersect_bounded_plane_sphere(&plane, range_a, &sphere, range_b, tolerances)
                .map_err(IntersectionError::from)
                .map_err(GraphSurfaceIntersectionError::Intersection)?,
            None,
        ),
        [
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
            ResolvedGraphSurfaceField::Plane { surface: plane, .. },
        ] => (
            intersect_bounded_plane_sphere(&plane, range_b, &sphere, range_a, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
                .map_err(IntersectionError::from)
                .map_err(GraphSurfaceIntersectionError::Intersection)?,
            None,
        ),
        [
            ResolvedGraphSurfaceField::Plane { surface: plane, .. },
            ResolvedGraphSurfaceField::Nurbs(surface),
        ] => {
            let output =
                plane_nurbs_march_in_scope(&plane, range_a, surface, range_b, tolerances, scope)?;
            (output.result, Some(output.traces))
        }
        [
            ResolvedGraphSurfaceField::Nurbs(surface),
            ResolvedGraphSurfaceField::Plane { surface: plane, .. },
        ] => {
            let output =
                plane_nurbs_march_in_scope(&plane, range_b, surface, range_a, tolerances, scope)?;
            let (raw, traces) = swap_nurbs_march_output(output)?;
            (raw, Some(traces))
        }
        [
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
            ResolvedGraphSurfaceField::Nurbs(surface),
        ] => {
            let output =
                sphere_nurbs_march_in_scope(&sphere, range_a, surface, range_b, tolerances, scope)?;
            (output.result, Some(output.traces))
        }
        [
            ResolvedGraphSurfaceField::Nurbs(surface),
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
        ] => {
            let output =
                sphere_nurbs_march_in_scope(&sphere, range_b, surface, range_a, tolerances, scope)?;
            let (raw, traces) = swap_nurbs_march_output(output)?;
            (raw, Some(traces))
        }
        [
            ResolvedGraphSurfaceField::Nurbs(surface_a),
            ResolvedGraphSurfaceField::Nurbs(surface_b),
        ] => {
            let output = nurbs_nurbs_march_in_scope(
                surface_a, range_a, surface_b, range_b, tolerances, scope,
            )?;
            (output.result, Some(output.traces))
        }
        [
            ResolvedGraphSurfaceField::OffsetNurbs {
                signed_distance,
                basis,
                ..
            },
            ResolvedGraphSurfaceField::Nurbs(surface),
        ] => {
            let output = offset_nurbs_nurbs_march_in_scope(
                basis,
                signed_distance,
                range_a,
                surface,
                range_b,
                tolerances,
                scope,
            )?;
            (output.result, Some(output.traces))
        }
        [
            ResolvedGraphSurfaceField::Nurbs(surface),
            ResolvedGraphSurfaceField::OffsetNurbs {
                signed_distance,
                basis,
                ..
            },
        ] => {
            let output = offset_nurbs_nurbs_march_in_scope(
                basis,
                signed_distance,
                range_b,
                surface,
                range_a,
                tolerances,
                scope,
            )?;
            let (raw, traces) = swap_nurbs_march_output(output)?;
            (raw, Some(traces))
        }
        [
            ResolvedGraphSurfaceField::OffsetNurbs { .. },
            ResolvedGraphSurfaceField::OffsetNurbs { .. },
        ] => (SurfaceSurfaceIntersections::complete_empty(), None),
        _ => unreachable!("supported graph surface fields were preclassified"),
    };
    let branch_graph = build_verified_branch_graph(
        [surface_a, surface_b],
        fields,
        [range_a, range_b],
        &raw,
        march_traces.as_deref(),
        tolerances.linear(),
        scope,
    )?;
    Ok(GraphSurfaceSurfaceIntersections { raw, branch_graph })
}

/// Swap lower analytic/NURBS evidence while keeping every retained trace attached
/// to its original branch position through the swapped result's canonical
/// ordering. This intentionally binds by position before sorting; it never
/// searches for a geometrically equal carrier.
fn swap_nurbs_march_output(
    output: MarchOutput,
) -> GraphSurfaceIntersectionResult<(SurfaceSurfaceIntersections, Vec<MarchTrace>)> {
    if output.result.curves.len() != output.traces.len() {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::InvalidTraceFamily,
        ));
    }
    let mut paired = output
        .result
        .curves
        .iter()
        .zip(output.traces)
        .collect::<Vec<_>>();
    paired.sort_by(|(a, _), (b, _)| {
        a.curve_range
            .lo
            .total_cmp(&b.curve_range.lo)
            .then(a.curve_range.hi.total_cmp(&b.curve_range.hi))
            .then(a.uv_b_start[0].total_cmp(&b.uv_b_start[0]))
            .then(a.uv_b_start[1].total_cmp(&b.uv_b_start[1]))
    });
    let traces = paired.into_iter().map(|(_, trace)| trace).collect();
    Ok((output.result.swapped(), traces))
}

/// Persist every certified positive-length branch into the geometry graph.
///
/// Paired pcurves are inserted in operand order followed by their verified
/// intersection-curve node. The complete batch is transactional: stale or
/// altered sources, mismatched certificates, and allocation-time graph
/// validation failures restore exact pre-call graph state.
pub fn persist_verified_graph_surface_intersections(
    graph: &mut GeometryGraph,
    intersections: &GraphSurfaceSurfaceIntersections,
) -> GraphSurfaceIntersectionResult<PersistentIntersectionBranchGraph> {
    graph.begin_undo_frame();
    let result = persist_verified_graph_surface_intersections_impl(graph, intersections);
    match result {
        Ok(persistent) => {
            graph
                .commit_undo_frame()
                .map_err(IntersectionError::from)
                .map_err(GraphSurfaceIntersectionError::Intersection)?;
            Ok(persistent)
        }
        Err(error) => {
            graph
                .rollback_undo_frame()
                .map_err(IntersectionError::from)
                .map_err(GraphSurfaceIntersectionError::Intersection)?;
            Err(error)
        }
    }
}

fn persist_verified_graph_surface_intersections_impl(
    graph: &mut GeometryGraph,
    intersections: &GraphSurfaceSurfaceIntersections,
) -> GraphSurfaceIntersectionResult<PersistentIntersectionBranchGraph> {
    let mut edges = Vec::with_capacity(intersections.branch_graph.edges.len());
    for edge in &intersections.branch_graph.edges {
        let pcurves = [
            graph.insert_curve2d(edge.pcurves[0].clone())?,
            graph.insert_curve2d(edge.pcurves[1].clone())?,
        ];
        let curve = match &edge.certificate {
            IntersectionBranchCertificate::Analytic(certificate) => match certificate.as_ref() {
                VerifiedIntersectionCertificate::PlaneLine(certificate) => graph
                    .insert_verified_plane_intersection_curve(
                        edge.source_surfaces,
                        pcurves,
                        *certificate,
                    )?,
                VerifiedIntersectionCertificate::PlaneSphereCircle(certificate) => graph
                    .insert_verified_plane_sphere_intersection_curve(
                        edge.source_surfaces,
                        pcurves,
                        *certificate,
                    )?,
            },
            IntersectionBranchCertificate::Nurbs(certificate) => graph
                .insert_verified_nurbs_intersection_curve(
                    edge.source_surfaces,
                    pcurves,
                    certificate.as_ref().clone(),
                )?,
        };
        edges.push(PersistentIntersectionBranchEdge {
            curve,
            pcurves,
            endpoint_vertices: edge.endpoint_vertices,
            endpoint_events: edge.endpoint_events,
            kind: edge.kind,
        });
    }
    Ok(PersistentIntersectionBranchGraph {
        source_surfaces: intersections.branch_graph.source_surfaces,
        vertices: intersections.branch_graph.vertices.clone(),
        edges,
    })
}

fn supports_constant_latitude_plane_sphere_chart(plane: Plane, sphere: Sphere) -> bool {
    let plane_frame = plane.frame();
    let sphere_frame = sphere.frame();
    plane_frame.z() == sphere_frame.z() || plane_frame.z() == -sphere_frame.z()
}

fn plane_nurbs_march_in_scope(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<MarchOutput> {
    match intersect_bounded_plane_nurbs_surface_with_traces_in_scope(
        plane,
        plane_range,
        surface,
        surface_range,
        tolerances,
        scope,
    ) {
        Ok(output) => Ok(output),
        Err(ContextMarchError::Kernel(error)) => Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::from(error),
        )),
        Err(ContextMarchError::Limit(snapshot)) => {
            scope.diagnose(
                snapshot.stage,
                NURBS_SURFACE_MARCH_SAMPLE_LIMIT,
                DiagnosticKind::LimitReached(snapshot),
                "NURBS surface marching grid sample limit reached",
            );
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::from(kcore::error::Error::ResourceLimit { snapshot }),
            ))
        }
        Err(ContextMarchError::Policy(error)) => {
            Err(GraphSurfaceIntersectionError::OperationPolicy(error))
        }
    }
}

fn sphere_nurbs_march_in_scope(
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<MarchOutput> {
    match intersect_bounded_sphere_nurbs_surface_with_traces_in_scope(
        sphere,
        sphere_range,
        surface,
        surface_range,
        tolerances,
        scope,
    ) {
        Ok(output) => Ok(output),
        Err(ContextMarchError::Kernel(error)) => Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::from(error),
        )),
        Err(ContextMarchError::Limit(snapshot)) => {
            scope.diagnose(
                snapshot.stage,
                NURBS_SURFACE_MARCH_SAMPLE_LIMIT,
                DiagnosticKind::LimitReached(snapshot),
                "NURBS surface marching grid sample limit reached",
            );
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::from(kcore::error::Error::ResourceLimit { snapshot }),
            ))
        }
        Err(ContextMarchError::Policy(error)) => {
            Err(GraphSurfaceIntersectionError::OperationPolicy(error))
        }
    }
}

fn nurbs_nurbs_march_in_scope(
    surface_a: &NurbsSurface,
    range_a: [ParamRange; 2],
    surface_b: &NurbsSurface,
    range_b: [ParamRange; 2],
    tolerances: Tolerances,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<MarchOutput> {
    match intersect_bounded_nurbs_nurbs_surfaces_with_traces_in_scope(
        surface_a, range_a, surface_b, range_b, tolerances, scope,
    ) {
        Ok(output) => Ok(output),
        Err(ContextMarchError::Kernel(error)) => Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::from(error),
        )),
        Err(ContextMarchError::Limit(snapshot)) => {
            scope.diagnose(
                snapshot.stage,
                NURBS_SURFACE_MARCH_SAMPLE_LIMIT,
                DiagnosticKind::LimitReached(snapshot),
                "NURBS surface marching grid sample limit reached",
            );
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::from(kcore::error::Error::ResourceLimit { snapshot }),
            ))
        }
        Err(ContextMarchError::Policy(error)) => {
            Err(GraphSurfaceIntersectionError::OperationPolicy(error))
        }
    }
}

fn offset_nurbs_nurbs_march_in_scope(
    basis: &NurbsSurface,
    signed_distance: f64,
    offset_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<MarchOutput> {
    if supports_varying_normal_offset_nurbs_nurbs_surface_pair(
        basis,
        signed_distance,
        offset_range,
        surface,
        surface_range,
    ) {
        let work = varying_normal_offset_window_proof_work(basis).ok_or(
            GraphSurfaceIntersectionError::OperationPolicy(
                OperationPolicyError::AccountingOverflow {
                    stage: NURBS_TRACE_CERTIFICATE_WORK,
                    resource: ResourceKind::Work,
                },
            ),
        )?;
        scope
            .ledger_mut()
            .charge(NURBS_TRACE_CERTIFICATE_WORK, work)?;
        scope
            .ledger_mut()
            .observe(NURBS_TRACE_CERTIFICATE_WORK, ResourceKind::Items, 1)?;
        scope
            .ledger_mut()
            .observe(NURBS_TRACE_CERTIFICATE_WORK, ResourceKind::Depth, 1)?;
    }
    match intersect_bounded_offset_nurbs_nurbs_surfaces_with_traces_in_scope(
        basis,
        signed_distance,
        offset_range,
        surface,
        surface_range,
        tolerances,
        scope,
    ) {
        Ok(output) => Ok(output),
        Err(ContextMarchError::Kernel(error)) => Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::from(error),
        )),
        Err(ContextMarchError::Limit(snapshot)) => {
            scope.diagnose(
                snapshot.stage,
                NURBS_SURFACE_MARCH_SAMPLE_LIMIT,
                DiagnosticKind::LimitReached(snapshot),
                "NURBS surface marching grid sample limit reached",
            );
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::from(kcore::error::Error::ResourceLimit { snapshot }),
            ))
        }
        Err(ContextMarchError::Policy(error)) => {
            Err(GraphSurfaceIntersectionError::OperationPolicy(error))
        }
    }
}

fn resolve_exact_surface_field<'a>(
    graph: &'a GeometryGraph,
    surface: SurfaceHandle,
    descriptor: &'a SurfaceDescriptor,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<Option<ResolvedGraphSurfaceField<'a>>> {
    if let Some(plane) = descriptor.as_plane() {
        return Ok(Some(ResolvedGraphSurfaceField::Plane {
            surface: *plane,
            direct: true,
        }));
    }
    if let Some(sphere) = descriptor.as_sphere() {
        return Ok(Some(ResolvedGraphSurfaceField::Sphere { surface: *sphere }));
    }
    if let Some(surface) = descriptor.as_nurbs() {
        return Ok(Some(ResolvedGraphSurfaceField::Nurbs(surface)));
    }
    if descriptor.as_offset().is_none() {
        return Ok(None);
    }
    let field = query_graph_in_scope(scope, graph, |evaluator| {
        evaluator.surface_exact_field(surface)
    })?
    .map_err(GraphSurfaceIntersectionError::GeometryEvaluation)?;
    if let Some(field) = field {
        return Ok(Some(match field {
            ExactSurfaceField::Plane(surface) => ResolvedGraphSurfaceField::Plane {
                surface,
                direct: false,
            },
            ExactSurfaceField::Sphere(surface) => ResolvedGraphSurfaceField::Sphere { surface },
        }));
    }
    resolve_offset_nurbs_field(graph, surface)
}

fn resolve_offset_nurbs_field(
    graph: &GeometryGraph,
    root: SurfaceHandle,
) -> GraphSurfaceIntersectionResult<Option<ResolvedGraphSurfaceField<'_>>> {
    let mut current = root;
    let mut distances = Vec::new();
    loop {
        let geometry = GeometryRef::Surface(current);
        let descriptor =
            graph
                .surface(current)
                .ok_or(GraphSurfaceIntersectionError::GeometryEvaluation(
                    EvalError::StaleGeometryHandle { geometry },
                ))?;
        match descriptor {
            SurfaceDescriptor::Offset(offset) => {
                distances.push(offset.signed_distance());
                if distances.len() > MAX_OFFSET_NURBS_INTERSECTION_CHAIN_LENGTH {
                    return Ok(None);
                }
                current = offset.basis();
            }
            SurfaceDescriptor::Nurbs(basis) => {
                let Some(signed_distance) = accumulated_regular_offset_distance(basis, &distances)
                else {
                    return Ok(None);
                };
                return Ok(Some(ResolvedGraphSurfaceField::OffsetNurbs {
                    signed_distance,
                    basis,
                    chain_length: distances.len(),
                }));
            }
            _ => return Ok(None),
        }
    }
}

fn accumulated_regular_offset_distance(basis: &NurbsSurface, distances: &[f64]) -> Option<f64> {
    distances.iter().rev().try_fold(0.0, |sum, &distance| {
        let next = sum + distance;
        (next.is_finite()
            && basis.points().iter().all(|point| {
                let lifted = Interval::point(point.z) + Interval::point(next);
                lifted.lo().is_finite() && lifted.hi().is_finite()
            }))
        .then_some(next)
    })
}

/// Returns whether the Euclidean control net proves that the NURBS surface is
/// non-planar at the operation's linear resolution.
///
/// A NURBS surface contained in a plane has every Euclidean control point in
/// that plane. The converse lets contextual Plane/NURBS support reject planar
/// encodings before marching, while accepting only sources whose control net
/// supplies direct non-plane evidence.
fn nurbs_control_net_is_nonplanar(surface: &NurbsSurface, linear_tolerance: f64) -> bool {
    let points = surface.points();
    let Some((&origin, rest)) = points.split_first() else {
        return false;
    };
    let Some(axis) = rest
        .iter()
        .map(|point| *point - origin)
        .max_by(|a, b| a.norm_sq().total_cmp(&b.norm_sq()))
    else {
        return false;
    };
    let axis_length = axis.norm();
    if axis_length <= linear_tolerance {
        return false;
    }
    let Some(normal) = rest
        .iter()
        .map(|point| axis.cross(*point - origin))
        .max_by(|a, b| a.norm_sq().total_cmp(&b.norm_sq()))
    else {
        return false;
    };
    let normal_length = normal.norm();
    if normal_length <= linear_tolerance * axis_length {
        return false;
    }
    let unit_normal = normal / normal_length;
    let scale = points
        .iter()
        .map(|point| (*point - origin).norm())
        .fold(1.0_f64, f64::max);
    let coplanar_tolerance = linear_tolerance + 256.0 * f64::EPSILON * scale;
    points
        .iter()
        .any(|point| (*point - origin).dot(unit_normal).abs() > coplanar_tolerance)
}

fn query_graph_in_scope<T>(
    scope: &mut OperationScope<'_, '_>,
    graph: &GeometryGraph,
    query: impl FnOnce(&mut EvalContext<'_>) -> Result<T, EvalError>,
) -> GraphSurfaceIntersectionResult<Result<T, EvalError>> {
    scope.ledger().require_limit(
        kgraph::eval_stage::NODE_VISITS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
    )?;
    let snapshots = scope.ledger().snapshots();
    let depth = graph_snapshot(
        &snapshots,
        kgraph::eval_stage::DEPENDENCY_DEPTH,
        ResourceKind::Depth,
    )?;
    let defaults = EvalLimits::default();
    let max_node_visits_per_query = usize::try_from(maximum_admissible_graph_visits(
        scope,
        defaults.max_node_visits_per_query as u64,
    )?)
    .map_err(|_| OperationPolicyError::AccountingOverflow {
        stage: kgraph::eval_stage::NODE_VISITS,
        resource: ResourceKind::Work,
    })?;
    let max_dependency_depth =
        usize::try_from(depth.allowed.min(defaults.max_dependency_depth as u64)).map_err(|_| {
            OperationPolicyError::AccountingOverflow {
                stage: depth.stage,
                resource: depth.resource,
            }
        })?;
    let tolerances = scope.context().tolerances();
    let mut ledger = scope
        .ledger_mut()
        .sequential(EvalBudgetProfile::v1_defaults())?;
    let mut evaluator = EvalContext::new(
        graph,
        EvalLimits {
            max_dependency_depth,
            max_node_visits_per_query,
        },
        tolerances,
    );
    let lower = query(&mut evaluator);
    let crossing = account_graph_query(
        &mut ledger,
        evaluator.last_query_usage(),
        lower.as_ref().err(),
    )?;
    if let Some(snapshot) = crossing {
        return Err(GraphSurfaceIntersectionError::OperationPolicy(
            OperationPolicyError::LimitReached(snapshot),
        ));
    }
    Ok(lower)
}

fn maximum_admissible_graph_visits(
    scope: &OperationScope<'_, '_>,
    upper: u64,
) -> Result<u64, OperationPolicyError> {
    let mut accepted = 0_u64;
    let mut rejected = upper.saturating_add(1);
    while accepted + 1 < rejected {
        let candidate = accepted + (rejected - accepted) / 2;
        match scope
            .ledger()
            .check_charge(kgraph::eval_stage::NODE_VISITS, candidate)
        {
            Ok(()) => accepted = candidate,
            Err(OperationPolicyError::LimitReached(_)) => rejected = candidate,
            Err(error) => return Err(error),
        }
    }
    Ok(accepted)
}

fn graph_snapshot(
    snapshots: &[LimitSnapshot],
    stage: kcore::operation::StageId,
    resource: ResourceKind,
) -> Result<LimitSnapshot, OperationPolicyError> {
    snapshots
        .iter()
        .copied()
        .find(|entry| entry.stage == stage && entry.resource == resource)
        .ok_or(OperationPolicyError::UnknownLimit { stage, resource })
}

fn account_graph_query(
    ledger: &mut SequentialWorkLedger<'_>,
    usage: EvalUsage,
    failure: Option<&EvalError>,
) -> Result<Option<LimitSnapshot>, OperationPolicyError> {
    let visits = u64::try_from(usage.node_visits()).map_err(|_| {
        OperationPolicyError::AccountingOverflow {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: ResourceKind::Work,
        }
    })?;
    let depth = u64::try_from(usage.dependency_depth()).map_err(|_| {
        OperationPolicyError::AccountingOverflow {
            stage: kgraph::eval_stage::DEPENDENCY_DEPTH,
            resource: ResourceKind::Depth,
        }
    })?;
    ledger.charge(kgraph::eval_stage::NODE_VISITS, visits)?;
    ledger.observe(
        kgraph::eval_stage::DEPENDENCY_DEPTH,
        ResourceKind::Depth,
        depth,
    )?;
    let Some(snapshot) = failure.and_then(EvalError::limit) else {
        return Ok(None);
    };
    let crossing = match snapshot.resource {
        ResourceKind::Work => ledger.charge_resource(snapshot.stage, snapshot.resource, 1),
        ResourceKind::Depth => ledger.observe(snapshot.stage, snapshot.resource, snapshot.consumed),
        _ => {
            return Err(OperationPolicyError::UnknownLimit {
                stage: snapshot.stage,
                resource: snapshot.resource,
            });
        }
    };
    match crossing {
        Err(OperationPolicyError::LimitReached(actual)) => Ok(Some(actual)),
        Err(error) => Err(error),
        Ok(()) => Err(OperationPolicyError::UnknownLimit {
            stage: snapshot.stage,
            resource: snapshot.resource,
        }),
    }
}

fn build_verified_branch_graph(
    source_surfaces: [SurfaceHandle; 2],
    fields: [ResolvedGraphSurfaceField<'_>; 2],
    surface_ranges: [[ParamRange; 2]; 2],
    raw: &SurfaceSurfaceIntersections,
    march_traces: Option<&[MarchTrace]>,
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<IntersectionBranchGraph> {
    let mut vertices = raw
        .points
        .iter()
        .map(|point| IntersectionBranchVertex {
            point: point.point,
            surface_parameters: [point.uv_a, point.uv_b],
            kind: point.kind,
            event: IntersectionBranchVertexEvent::IsolatedContact,
        })
        .collect::<Vec<_>>();
    let mut edges = Vec::with_capacity(raw.curves.len());
    if let Some(traces) = march_traces
        && traces.len() != raw.curves.len()
    {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::InvalidTraceFamily,
        ));
    }

    for (branch_index, branch) in raw.curves.iter().enumerate() {
        let VerifiedBranchPayload {
            carrier,
            carrier_range,
            pcurves,
            parameter_maps,
            certificate,
        } = match (&branch.curve, fields) {
            (
                SurfaceIntersectionCurve::Line(raw_line),
                [
                    ResolvedGraphSurfaceField::Plane {
                        surface: plane_a, ..
                    },
                    ResolvedGraphSurfaceField::Plane {
                        surface: plane_b, ..
                    },
                ],
            ) => {
                let (carrier, carrier_range) = canonical_line(*raw_line, branch.curve_range)
                    .map_err(IntersectionError::from)
                    .map_err(GraphSurfaceIntersectionError::Intersection)?;
                let (pcurve_a, map_a) = plane_pcurve(carrier, plane_a)?;
                let (pcurve_b, map_b) = plane_pcurve(carrier, plane_b)?;
                let certificate = certify_paired_plane_line_residuals(
                    carrier,
                    carrier_range,
                    [plane_a, plane_b],
                    [pcurve_a, pcurve_b],
                    [map_a, map_b],
                    tolerance,
                )
                .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
                VerifiedBranchPayload {
                    carrier: CurveDescriptor::Line(carrier),
                    carrier_range,
                    pcurves: [
                        Curve2dDescriptor::Line(pcurve_a),
                        Curve2dDescriptor::Line(pcurve_b),
                    ],
                    parameter_maps: [map_a, map_b],
                    certificate: IntersectionBranchCertificate::Analytic(Box::new(
                        VerifiedIntersectionCertificate::PlaneLine(certificate),
                    )),
                }
            }
            (SurfaceIntersectionCurve::Circle(raw_circle), fields) => {
                build_verified_plane_sphere_circle_branch(
                    *raw_circle,
                    branch,
                    fields,
                    surface_ranges,
                    tolerance,
                    scope,
                )?
            }
            (SurfaceIntersectionCurve::Nurbs(raw_carrier), fields) => {
                build_verified_analytic_nurbs_branch(
                    raw_carrier,
                    fields,
                    march_traces
                        .and_then(|traces| traces.get(branch_index))
                        .ok_or(GraphSurfaceIntersectionError::BranchCertificate(
                            IntersectionCertificateError::InvalidTraceFamily,
                        ))?,
                    tolerance,
                    scope,
                )?
            }
            _ => {
                return Err(GraphSurfaceIntersectionError::BranchCertificate(
                    kgraph::IntersectionCertificateError::NonFiniteGeometry,
                ));
            }
        };

        let endpoint_vertices = [vertices.len(), vertices.len() + 1];
        let mut endpoint_events = [
            IntersectionBranchEndpointEvent::SurfaceWindowBoundary {
                surfaces: [false; 2],
            },
            IntersectionBranchEndpointEvent::SurfaceWindowBoundary {
                surfaces: [false; 2],
            },
        ];
        for (endpoint, parameter) in [carrier_range.lo, carrier_range.hi].into_iter().enumerate() {
            let surface_parameters = [
                pcurves[0].as_curve().eval(parameter_maps[0].map(parameter)),
                pcurves[1].as_curve().eval(parameter_maps[1].map(parameter)),
            ];
            let boundary_surfaces = [
                on_window_boundary(surface_parameters[0], surface_ranges[0], tolerance),
                on_window_boundary(surface_parameters[1], surface_ranges[1], tolerance),
            ];
            endpoint_events[endpoint] = IntersectionBranchEndpointEvent::SurfaceWindowBoundary {
                surfaces: boundary_surfaces,
            };
            vertices.push(IntersectionBranchVertex {
                point: carrier.as_curve().eval(parameter),
                surface_parameters: [
                    [surface_parameters[0].x, surface_parameters[0].y],
                    [surface_parameters[1].x, surface_parameters[1].y],
                ],
                kind: branch.kind,
                event: IntersectionBranchVertexEvent::BoundaryEndpoint {
                    surfaces: boundary_surfaces,
                },
            });
        }
        edges.push(IntersectionBranchEdge {
            source_surfaces,
            carrier,
            carrier_range,
            pcurves,
            parameter_maps,
            endpoint_vertices,
            endpoint_events,
            kind: branch.kind,
            certificate,
        });
    }

    Ok(IntersectionBranchGraph {
        source_surfaces,
        vertices,
        edges,
    })
}

struct VerifiedBranchPayload {
    carrier: CurveDescriptor,
    carrier_range: ParamRange,
    pcurves: [Curve2dDescriptor; 2],
    parameter_maps: [AffineParamMap1d; 2],
    certificate: IntersectionBranchCertificate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VerifiedNurbsProofFamily {
    Plane,
    Sphere,
    PairedSources,
    OffsetPair,
}

fn build_verified_analytic_nurbs_branch(
    raw_carrier: &kgeom::nurbs::NurbsCurve,
    fields: [ResolvedGraphSurfaceField<'_>; 2],
    march_trace: &MarchTrace,
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<VerifiedBranchPayload> {
    if march_trace.carrier != *raw_carrier {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::InvalidTraceFamily,
        ));
    }
    let (traces, pcurves, proof_family) = match fields {
        [
            ResolvedGraphSurfaceField::Plane { surface: plane, .. },
            ResolvedGraphSurfaceField::Nurbs(surface),
        ] => (
            [
                NurbsIntersectionTrace::Plane(plane),
                NurbsIntersectionTrace::Nurbs(surface.clone()),
            ],
            [
                march_trace.other_pcurve.clone(),
                march_trace.surface_pcurve.clone(),
            ],
            VerifiedNurbsProofFamily::Plane,
        ),
        [
            ResolvedGraphSurfaceField::Nurbs(surface),
            ResolvedGraphSurfaceField::Plane { surface: plane, .. },
        ] => (
            [
                NurbsIntersectionTrace::Nurbs(surface.clone()),
                NurbsIntersectionTrace::Plane(plane),
            ],
            [
                march_trace.surface_pcurve.clone(),
                march_trace.other_pcurve.clone(),
            ],
            VerifiedNurbsProofFamily::Plane,
        ),
        [
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
            ResolvedGraphSurfaceField::Nurbs(surface),
        ] => (
            [
                NurbsIntersectionTrace::Sphere(sphere),
                NurbsIntersectionTrace::Nurbs(surface.clone()),
            ],
            [
                march_trace.other_pcurve.clone(),
                march_trace.surface_pcurve.clone(),
            ],
            VerifiedNurbsProofFamily::Sphere,
        ),
        [
            ResolvedGraphSurfaceField::Nurbs(surface),
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
        ] => (
            [
                NurbsIntersectionTrace::Nurbs(surface.clone()),
                NurbsIntersectionTrace::Sphere(sphere),
            ],
            [
                march_trace.surface_pcurve.clone(),
                march_trace.other_pcurve.clone(),
            ],
            VerifiedNurbsProofFamily::Sphere,
        ),
        [
            ResolvedGraphSurfaceField::Nurbs(surface_a),
            ResolvedGraphSurfaceField::Nurbs(surface_b),
        ] => (
            [
                NurbsIntersectionTrace::Nurbs(surface_a.clone()),
                NurbsIntersectionTrace::Nurbs(surface_b.clone()),
            ],
            [
                march_trace.other_pcurve.clone(),
                march_trace.surface_pcurve.clone(),
            ],
            VerifiedNurbsProofFamily::PairedSources,
        ),
        [
            ResolvedGraphSurfaceField::OffsetNurbs {
                signed_distance,
                basis,
                ..
            },
            ResolvedGraphSurfaceField::Nurbs(surface),
        ] => (
            [
                NurbsIntersectionTrace::OffsetNurbs(kgraph::TransmittedOffsetNurbsTrace::new(
                    basis.clone(),
                    signed_distance,
                )),
                NurbsIntersectionTrace::Nurbs(surface.clone()),
            ],
            [
                march_trace.other_pcurve.clone(),
                march_trace.surface_pcurve.clone(),
            ],
            VerifiedNurbsProofFamily::OffsetPair,
        ),
        [
            ResolvedGraphSurfaceField::Nurbs(surface),
            ResolvedGraphSurfaceField::OffsetNurbs {
                signed_distance,
                basis,
                ..
            },
        ] => (
            [
                NurbsIntersectionTrace::Nurbs(surface.clone()),
                NurbsIntersectionTrace::OffsetNurbs(kgraph::TransmittedOffsetNurbsTrace::new(
                    basis.clone(),
                    signed_distance,
                )),
            ],
            [
                march_trace.surface_pcurve.clone(),
                march_trace.other_pcurve.clone(),
            ],
            VerifiedNurbsProofFamily::OffsetPair,
        ),
        _ => {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        }
    };
    let certificate = match proof_family {
        VerifiedNurbsProofFamily::Sphere => {
            let cost = verified_sphere_nurbs_intersection_certificate_cost(raw_carrier, &traces)
                .ok_or(GraphSurfaceIntersectionError::OperationPolicy(
                    OperationPolicyError::AccountingOverflow {
                        stage: NURBS_TRACE_CERTIFICATE_WORK,
                        resource: ResourceKind::Work,
                    },
                ))?;
            scope
                .ledger_mut()
                .charge(NURBS_TRACE_CERTIFICATE_WORK, cost.work())?;
            scope.ledger_mut().observe(
                NURBS_TRACE_CERTIFICATE_WORK,
                ResourceKind::Items,
                cost.items(),
            )?;
            scope.ledger_mut().observe(
                NURBS_TRACE_CERTIFICATE_WORK,
                ResourceKind::Depth,
                cost.depth(),
            )?;
            certify_verified_sphere_nurbs_intersection_residuals(
                raw_carrier.clone(),
                traces,
                pcurves.clone(),
                tolerance,
            )
            .map_err(GraphSurfaceIntersectionError::BranchCertificate)?
        }
        VerifiedNurbsProofFamily::PairedSources => {
            let cost = verified_nurbs_nurbs_intersection_certificate_cost(raw_carrier, &traces)
                .ok_or(GraphSurfaceIntersectionError::OperationPolicy(
                    OperationPolicyError::AccountingOverflow {
                        stage: NURBS_TRACE_CERTIFICATE_WORK,
                        resource: ResourceKind::Work,
                    },
                ))?;
            scope
                .ledger_mut()
                .charge(NURBS_TRACE_CERTIFICATE_WORK, cost.work())?;
            scope.ledger_mut().observe(
                NURBS_TRACE_CERTIFICATE_WORK,
                ResourceKind::Items,
                cost.items(),
            )?;
            scope.ledger_mut().observe(
                NURBS_TRACE_CERTIFICATE_WORK,
                ResourceKind::Depth,
                cost.depth(),
            )?;
            certify_verified_nurbs_nurbs_intersection_residuals(
                raw_carrier.clone(),
                traces,
                pcurves.clone(),
                tolerance,
            )
            .map_err(GraphSurfaceIntersectionError::BranchCertificate)?
        }
        VerifiedNurbsProofFamily::OffsetPair => {
            let cost =
                verified_offset_nurbs_nurbs_intersection_certificate_cost(raw_carrier, &traces)
                    .ok_or(GraphSurfaceIntersectionError::OperationPolicy(
                        OperationPolicyError::AccountingOverflow {
                            stage: NURBS_TRACE_CERTIFICATE_WORK,
                            resource: ResourceKind::Work,
                        },
                    ))?;
            scope
                .ledger_mut()
                .charge(NURBS_TRACE_CERTIFICATE_WORK, cost.work())?;
            scope.ledger_mut().observe(
                NURBS_TRACE_CERTIFICATE_WORK,
                ResourceKind::Items,
                cost.items(),
            )?;
            scope.ledger_mut().observe(
                NURBS_TRACE_CERTIFICATE_WORK,
                ResourceKind::Depth,
                cost.depth(),
            )?;
            certify_verified_offset_nurbs_nurbs_intersection_residuals(
                raw_carrier.clone(),
                traces,
                pcurves.clone(),
                tolerance,
            )
            .map_err(GraphSurfaceIntersectionError::BranchCertificate)?
        }
        VerifiedNurbsProofFamily::Plane => {
            let proof_work =
                verified_plane_nurbs_intersection_certificate_work(raw_carrier, &traces).ok_or(
                    GraphSurfaceIntersectionError::OperationPolicy(
                        OperationPolicyError::AccountingOverflow {
                            stage: NURBS_TRACE_CERTIFICATE_WORK,
                            resource: ResourceKind::Work,
                        },
                    ),
                )?;
            scope
                .ledger_mut()
                .charge(NURBS_TRACE_CERTIFICATE_WORK, proof_work)?;
            certify_verified_plane_nurbs_intersection_residuals(
                raw_carrier.clone(),
                traces,
                pcurves.clone(),
                tolerance,
            )
            .map_err(GraphSurfaceIntersectionError::BranchCertificate)?
        }
    };
    let identity = AffineParamMap1d::new(1.0, 0.0)
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    Ok(VerifiedBranchPayload {
        carrier: CurveDescriptor::Nurbs(raw_carrier.clone()),
        carrier_range: raw_carrier.param_range(),
        pcurves: pcurves.map(Curve2dDescriptor::Nurbs),
        parameter_maps: [identity, identity],
        certificate: IntersectionBranchCertificate::Nurbs(Box::new(certificate)),
    })
}

fn build_verified_plane_sphere_circle_branch(
    raw_carrier: Circle,
    raw_branch: &SurfaceSurfaceCurve,
    fields: [ResolvedGraphSurfaceField<'_>; 2],
    surface_ranges: [[ParamRange; 2]; 2],
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<VerifiedBranchPayload> {
    let (plane, sphere, plane_first) = match fields {
        [
            ResolvedGraphSurfaceField::Plane { surface: plane, .. },
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
        ] => (plane, sphere, true),
        [
            ResolvedGraphSurfaceField::Sphere { surface: sphere },
            ResolvedGraphSurfaceField::Plane { surface: plane, .. },
        ] => (plane, sphere, false),
        _ => {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        }
    };
    if !supports_constant_latitude_plane_sphere_chart(plane, sphere) {
        return build_verified_oblique_plane_sphere_circle_branch(
            raw_carrier,
            raw_branch,
            plane,
            sphere,
            plane_first,
            surface_ranges,
            tolerance,
            scope,
        );
    }
    let carrier = Circle::new(
        sphere.frame().with_origin(raw_carrier.frame().origin()),
        raw_carrier.radius(),
    )
    .map_err(IntersectionError::from)
    .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let plane_orientation = if plane.frame().z() == sphere.frame().z() {
        1.0
    } else if plane.frame().z() == -sphere.frame().z() {
        -1.0
    } else {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: kgraph::PairedTrace::First,
                reason: "plane normal must be aligned or anti-aligned with the sphere axis",
            },
        ));
    };
    let local_center = plane.frame().to_local(carrier.frame().origin());
    let sphere_x = sphere.frame().x();
    let plane_pcurve = Circle2d::new(
        Vec2::new(local_center.x, local_center.y),
        carrier.radius(),
        Vec2::new(
            sphere_x.dot(plane.frame().x()),
            sphere_x.dot(plane.frame().y()),
        ),
    )
    .map_err(IntersectionError::from)
    .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let height = (carrier.frame().origin() - sphere.frame().origin()).dot(sphere.frame().z());
    let latitude = math::atan2(height, carrier.radius());
    let sphere_pcurve = Line2d::new(Vec2::new(0.0, latitude), Vec2::new(1.0, 0.0))
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let identity = AffineParamMap1d::new(1.0, 0.0)
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    let plane_map = AffineParamMap1d::new(plane_orientation, 0.0)
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    let sphere_index = usize::from(plane_first);
    let sphere_uv = if plane_first {
        [raw_branch.uv_b_start, raw_branch.uv_b_end]
    } else {
        [raw_branch.uv_a_start, raw_branch.uv_a_end]
    };
    let carrier_range = sphere_longitude_carrier_range(
        raw_branch.curve_range,
        [sphere_uv[0][0], sphere_uv[1][0]],
        surface_ranges[sphere_index][0],
        plane_orientation,
        carrier.radius(),
        tolerance,
    )?;
    let plane_trace =
        PlaneSphereCircleTrace::Plane(PlaneCircleTrace::new(plane, plane_pcurve, plane_map));
    let sphere_trace =
        PlaneSphereCircleTrace::Sphere(SphereLatitudeTrace::new(sphere, sphere_pcurve, identity));
    let (pcurves, traces) = if plane_first {
        (
            [
                Curve2dDescriptor::Circle(plane_pcurve),
                Curve2dDescriptor::Line(sphere_pcurve),
            ],
            [plane_trace, sphere_trace],
        )
    } else {
        (
            [
                Curve2dDescriptor::Line(sphere_pcurve),
                Curve2dDescriptor::Circle(plane_pcurve),
            ],
            [sphere_trace, plane_trace],
        )
    };
    let certificate =
        certify_paired_plane_sphere_circle_residuals(carrier, carrier_range, traces, tolerance)
            .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    Ok(VerifiedBranchPayload {
        carrier: CurveDescriptor::Circle(carrier),
        carrier_range,
        pcurves,
        parameter_maps: if plane_first {
            [plane_map, identity]
        } else {
            [identity, plane_map]
        },
        certificate: IntersectionBranchCertificate::Analytic(Box::new(
            VerifiedIntersectionCertificate::PlaneSphereCircle(certificate),
        )),
    })
}

#[allow(clippy::too_many_arguments)]
fn build_verified_oblique_plane_sphere_circle_branch(
    carrier: Circle,
    raw_branch: &SurfaceSurfaceCurve,
    plane: Plane,
    sphere: Sphere,
    plane_first: bool,
    surface_ranges: [[ParamRange; 2]; 2],
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<VerifiedBranchPayload> {
    scope.ledger_mut().charge(
        SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS,
        kgraph::SPHERICAL_CIRCLE_PROOF_SEGMENTS as u64,
    )?;
    let local_center = plane.frame().to_local(carrier.frame().origin());
    let carrier_x = carrier.frame().x();
    let plane_pcurve = Circle2d::new(
        Vec2::new(local_center.x, local_center.y),
        carrier.radius(),
        Vec2::new(
            carrier_x.dot(plane.frame().x()),
            carrier_x.dot(plane.frame().y()),
        ),
    )
    .map_err(IntersectionError::from)
    .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let identity = AffineParamMap1d::new(1.0, 0.0)
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    let sphere_index = usize::from(plane_first);
    let sphere_uv = if plane_first {
        [raw_branch.uv_b_start, raw_branch.uv_b_end]
    } else {
        [raw_branch.uv_a_start, raw_branch.uv_a_end]
    };
    let plane_position = if plane_first {
        PairedTrace::First
    } else {
        PairedTrace::Second
    };
    let (sphere_pcurve, certificate) = certify_paired_plane_sphere_oblique_circle_residuals(
        carrier,
        raw_branch.curve_range,
        plane,
        plane_pcurve,
        sphere,
        surface_ranges[sphere_index],
        [sphere_uv[0][0], sphere_uv[1][0]],
        plane_position,
        tolerance,
    )
    .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    let pcurves = if plane_first {
        [
            Curve2dDescriptor::Circle(plane_pcurve),
            Curve2dDescriptor::SphericalCircle(sphere_pcurve),
        ]
    } else {
        [
            Curve2dDescriptor::SphericalCircle(sphere_pcurve),
            Curve2dDescriptor::Circle(plane_pcurve),
        ]
    };
    Ok(VerifiedBranchPayload {
        carrier: CurveDescriptor::Circle(carrier),
        carrier_range: raw_branch.curve_range,
        pcurves,
        parameter_maps: [identity, identity],
        certificate: IntersectionBranchCertificate::Analytic(Box::new(
            VerifiedIntersectionCertificate::PlaneSphereCircle(certificate),
        )),
    })
}

fn sphere_longitude_carrier_range(
    raw_range: ParamRange,
    raw_longitudes: [f64; 2],
    longitude_window: ParamRange,
    raw_orientation: f64,
    radius: f64,
    tolerance: f64,
) -> GraphSurfaceIntersectionResult<ParamRange> {
    let tau = core::f64::consts::TAU;
    let angular_tolerance = (tolerance / radius).max(32.0 * f64::EPSILON);
    if raw_range.width() >= tau - angular_tolerance && longitude_window.width() >= tau {
        let hi = longitude_window.lo + tau;
        if hi.is_finite() && hi <= longitude_window.hi {
            return Ok(ParamRange::new(longitude_window.lo, hi));
        }
    }

    let start = raw_longitudes[0];
    let expected_end = start + raw_orientation * raw_range.width();
    let end = periodic_representative_near(raw_longitudes[1], expected_end, longitude_window)
        .ok_or(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: kgraph::PairedTrace::First,
                reason: "sphere longitude branch has no continuous representative in its requested window",
            },
        ))?;
    let range = ParamRange::new(start.min(end), start.max(end));
    if !range.is_finite() || range.width() <= 0.0 {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::InvalidCarrierRange,
        ));
    }
    Ok(range)
}

fn periodic_representative_near(value: f64, target: f64, window: ParamRange) -> Option<f64> {
    let tau = core::f64::consts::TAU;
    if !value.is_finite() || !target.is_finite() || !window.is_finite() || window.width() < 0.0 {
        return None;
    }
    let first_turn = ((window.lo - value) / tau).ceil();
    let last_turn = ((window.hi - value) / tau).floor();
    if first_turn > last_turn {
        return None;
    }
    let nearest_turn = ((target - value) / tau)
        .round()
        .clamp(first_turn, last_turn);
    let representative = value + nearest_turn * tau;
    (representative.is_finite() && representative >= window.lo && representative <= window.hi)
        .then_some(representative)
}

fn on_window_boundary(uv: Vec2, ranges: [ParamRange; 2], tolerance: f64) -> bool {
    (uv.x - ranges[0].lo).abs() <= tolerance
        || (uv.x - ranges[0].hi).abs() <= tolerance
        || (uv.y - ranges[1].lo).abs() <= tolerance
        || (uv.y - ranges[1].hi).abs() <= tolerance
}

fn canonical_line(line: Line, range: ParamRange) -> kcore::error::Result<(Line, ParamRange)> {
    let direction = line.dir();
    let reversed = direction.x < 0.0
        || (direction.x == 0.0 && direction.y < 0.0)
        || (direction.x == 0.0 && direction.y == 0.0 && direction.z < 0.0);
    if reversed {
        Ok((
            Line::new(line.origin(), -direction)?,
            ParamRange::new(-range.hi, -range.lo),
        ))
    } else {
        Ok((line, range))
    }
}

fn plane_pcurve(
    carrier: Line,
    surface: Plane,
) -> GraphSurfaceIntersectionResult<(Line2d, AffineParamMap1d)> {
    let frame = surface.frame();
    let local_origin = frame.to_local(carrier.origin());
    let uv_direction = Vec2::new(carrier.dir().dot(frame.x()), carrier.dir().dot(frame.y()));
    let scale = uv_direction.norm();
    let pcurve = Line2d::new(Vec2::new(local_origin.x, local_origin.y), uv_direction)
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let map = AffineParamMap1d::new(scale, 0.0)
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    Ok((pcurve, map))
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::curve2d::NurbsCurve2d;
    use kgeom::nurbs::NurbsCurve;

    fn trace_branch(
        height: f64,
        uv_a_start: [f64; 2],
        uv_b_start: [f64; 2],
    ) -> (SurfaceSurfaceCurve, MarchTrace) {
        let knots = vec![0.0, 0.0, 1.0, 1.0];
        let carrier = NurbsCurve::new(
            1,
            knots.clone(),
            vec![Point3::new(0.0, height, 0.0), Point3::new(1.0, height, 0.0)],
            None,
        )
        .unwrap();
        let other_pcurve = NurbsCurve2d::new(
            1,
            knots.clone(),
            vec![Vec2::new(0.0, height), Vec2::new(1.0, height)],
            None,
        )
        .unwrap();
        let surface_pcurve = other_pcurve.clone();
        (
            SurfaceSurfaceCurve {
                curve: SurfaceIntersectionCurve::Nurbs(carrier.clone()),
                curve_range: ParamRange::new(0.0, 1.0),
                uv_a_start,
                uv_a_end: [uv_a_start[0] + 1.0, uv_a_start[1]],
                uv_b_start,
                uv_b_end: [uv_b_start[0] + 1.0, uv_b_start[1]],
                kind: ContactKind::Transverse,
            },
            MarchTrace {
                carrier,
                other_pcurve,
                surface_pcurve,
            },
        )
    }

    #[test]
    fn swapped_march_output_reorders_branch_trace_pairs_positionally() {
        let (first, first_trace) = trace_branch(0.0, [0.0, 0.0], [1.0, 0.0]);
        let (second, second_trace) = trace_branch(1.0, [1.0, 0.0], [0.0, 0.0]);
        let result = SurfaceSurfaceIntersections::canonicalized_indeterminate(
            Vec::new(),
            vec![first, second],
            "test marcher evidence",
        )
        .unwrap();
        let (swapped, traces) = swap_nurbs_march_output(MarchOutput {
            result,
            traces: vec![first_trace, second_trace],
        })
        .unwrap();

        assert_eq!(swapped.curves.len(), traces.len());
        for (branch, trace) in swapped.curves.iter().zip(&traces) {
            assert_eq!(
                branch.curve,
                SurfaceIntersectionCurve::Nurbs(trace.carrier.clone())
            );
        }
        assert_eq!(traces[0].carrier.points()[0].y, 1.0);
        assert_eq!(traces[1].carrier.points()[0].y, 0.0);
    }
}
