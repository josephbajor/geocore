//! Graph-aware, verified surface-intersection adapter.
//!
//! Direct planes/spheres and safe constant-offset chains terminating at those
//! leaves form exact field families. The adapter promotes plane/plane lines and
//! plane/sphere circles only after constructing both pcurves and proving their
//! paired whole-interval residual contracts. Common-axis circles retain their
//! longitude/latitude fast path; other finite secants use a certified nonlinear
//! spherical pcurve and fail closed at chart singularities or outside windows.

use super::error::IntersectionError;
use super::plane_plane::intersect_bounded_planes;
use super::plane_sphere::intersect_bounded_plane_sphere;
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
};
use core::fmt;
use kcore::error::{CapabilityId, ClassifiedError, ErrorClass, ErrorCode};
use kcore::math;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSnapshot, LimitSpec, OperationContext, OperationOutcome,
    OperationPolicyError, OperationScope, ResourceKind, SequentialWorkLedger, SessionPolicy,
    StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Line};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere};
use kgeom::vec::{Point3, Vec2};
use kgraph::{
    AffineParamMap1d, Curve2dDescriptor, Curve2dHandle, CurveDescriptor, CurveHandle,
    EvalBudgetProfile, EvalContext, EvalError, EvalLimits, EvalUsage, ExactSurfaceField,
    GeometryGraph, GeometryGraphError, GeometryRef, IntersectionCertificateError, PairedTrace,
    PlaneCircleTrace, PlaneSphereCircleTrace, SphereLatitudeTrace, SurfaceDescriptor,
    SurfaceHandle, VerifiedIntersectionCertificate, certify_paired_plane_line_residuals,
    certify_paired_plane_sphere_circle_residuals,
    certify_paired_plane_sphere_oblique_circle_residuals,
};

const MAX_SPHERICAL_CIRCLE_PROOFS_PER_QUERY: u64 = 4_096;

/// Stable work stage for fixed whole-branch inverse sphere-chart subdivisions.
pub const SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS: StageId =
    match StageId::new("kops.intersect.spherical-circle-proof-subdivisions") {
        Ok(stage) => stage,
        Err(_) => panic!("valid spherical-circle proof stage"),
    };

/// Version-1 composed budget for graph-owned surface intersection.
#[derive(Debug, Clone, Copy, Default)]
pub struct GraphSurfaceBudgetProfile;

impl GraphSurfaceBudgetProfile {
    /// Graph evaluation plus a bounded number of fixed spherical-circle chart
    /// proofs. Each oblique branch consumes exactly
    /// [`kgraph::SPHERICAL_CIRCLE_PROOF_SEGMENTS`] work units.
    pub fn v1_defaults() -> BudgetPlan {
        let evaluation = EvalBudgetProfile::v1_defaults();
        BudgetPlan::new(evaluation.limits().iter().copied().chain([LimitSpec::new(
            SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            kgraph::SPHERICAL_CIRCLE_PROOF_SEGMENTS as u64 * MAX_SPHERICAL_CIRCLE_PROOFS_PER_QUERY,
        )]))
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
    /// The existing analytic solver or support-matrix boundary failed.
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
    pub certificate: VerifiedIntersectionCertificate,
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
    /// Unchanged result returned by the authoritative analytic solver.
    pub raw: SurfaceSurfaceIntersections,
    /// Verified operation-local branch graph derived from `raw`.
    pub branch_graph: IntersectionBranchGraph,
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
/// their safe constant-offset chains form exact fields. Plane/plane and finite
/// regular-chart plane/sphere branches are supported; all other pairs remain
/// explicitly unsupported. Owners that may dispatch oblique plane/sphere
/// branches must compose [`GraphSurfaceBudgetProfile::v1_defaults`] before
/// creating `scope`.
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
        (Some(ExactSurfaceField::Plane(plane_a)), Some(ExactSurfaceField::Plane(plane_b))) => [
            ExactSurfaceField::Plane(plane_a),
            ExactSurfaceField::Plane(plane_b),
        ],
        (Some(ExactSurfaceField::Plane(plane)), Some(ExactSurfaceField::Sphere(sphere))) => [
            ExactSurfaceField::Plane(plane),
            ExactSurfaceField::Sphere(sphere),
        ],
        (Some(ExactSurfaceField::Sphere(sphere)), Some(ExactSurfaceField::Plane(plane))) => [
            ExactSurfaceField::Sphere(sphere),
            ExactSurfaceField::Plane(plane),
        ],
        _ => return Err(unsupported()),
    };
    let raw = match fields {
        [
            ExactSurfaceField::Plane(plane_a),
            ExactSurfaceField::Plane(plane_b),
        ] => intersect_bounded_planes(&plane_a, range_a, &plane_b, range_b, tolerances),
        [
            ExactSurfaceField::Plane(plane),
            ExactSurfaceField::Sphere(sphere),
        ] => intersect_bounded_plane_sphere(&plane, range_a, &sphere, range_b, tolerances),
        [
            ExactSurfaceField::Sphere(sphere),
            ExactSurfaceField::Plane(plane),
        ] => intersect_bounded_plane_sphere(&plane, range_b, &sphere, range_a, tolerances)
            .map(SurfaceSurfaceIntersections::swapped),
        _ => unreachable!("supported graph surface fields were preclassified"),
    }
    .map_err(IntersectionError::from)
    .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let branch_graph = build_verified_branch_graph(
        [surface_a, surface_b],
        fields,
        [range_a, range_b],
        &raw,
        tolerances.linear(),
        scope,
    )?;
    Ok(GraphSurfaceSurfaceIntersections { raw, branch_graph })
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
        let curve = match edge.certificate {
            VerifiedIntersectionCertificate::PlaneLine(certificate) => graph
                .insert_verified_plane_intersection_curve(
                    edge.source_surfaces,
                    pcurves,
                    certificate,
                )?,
            VerifiedIntersectionCertificate::PlaneSphereCircle(certificate) => graph
                .insert_verified_plane_sphere_intersection_curve(
                    edge.source_surfaces,
                    pcurves,
                    certificate,
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

fn resolve_exact_surface_field(
    graph: &GeometryGraph,
    surface: SurfaceHandle,
    descriptor: &SurfaceDescriptor,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<Option<ExactSurfaceField>> {
    if let Some(plane) = descriptor.as_plane() {
        return Ok(Some(ExactSurfaceField::Plane(*plane)));
    }
    if let Some(sphere) = descriptor.as_sphere() {
        return Ok(Some(ExactSurfaceField::Sphere(*sphere)));
    }
    if descriptor.as_offset().is_none() {
        return Ok(None);
    }
    query_graph_in_scope(scope, graph, |evaluator| {
        evaluator.surface_exact_field(surface)
    })?
    .map_err(GraphSurfaceIntersectionError::GeometryEvaluation)
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
    fields: [ExactSurfaceField; 2],
    surface_ranges: [[ParamRange; 2]; 2],
    raw: &SurfaceSurfaceIntersections,
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

    for branch in &raw.curves {
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
                    ExactSurfaceField::Plane(plane_a),
                    ExactSurfaceField::Plane(plane_b),
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
                    certificate: VerifiedIntersectionCertificate::PlaneLine(certificate),
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
    certificate: VerifiedIntersectionCertificate,
}

fn build_verified_plane_sphere_circle_branch(
    raw_carrier: Circle,
    raw_branch: &SurfaceSurfaceCurve,
    fields: [ExactSurfaceField; 2],
    surface_ranges: [[ParamRange; 2]; 2],
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<VerifiedBranchPayload> {
    let (plane, sphere, plane_first) = match fields {
        [
            ExactSurfaceField::Plane(plane),
            ExactSurfaceField::Sphere(sphere),
        ] => (plane, sphere, true),
        [
            ExactSurfaceField::Sphere(sphere),
            ExactSurfaceField::Plane(plane),
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
        certificate: VerifiedIntersectionCertificate::PlaneSphereCircle(certificate),
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
        certificate: VerifiedIntersectionCertificate::PlaneSphereCircle(certificate),
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
