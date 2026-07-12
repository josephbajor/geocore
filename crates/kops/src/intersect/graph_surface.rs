//! Graph-aware, verified surface-intersection adapter.
//!
//! The first slice deliberately supports only direct Plane/Plane descriptors.
//! It delegates discovery and completion to the existing analytic solver, then
//! promotes line branches only after constructing both pcurves and proving the
//! paired whole-interval residual contract.

use super::error::IntersectionError;
use super::plane_plane::intersect_bounded_planes;
use super::result::{ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceIntersections};
use core::fmt;
use kcore::error::{CapabilityId, ClassifiedError, ErrorClass, ErrorCode};
use kcore::operation::{OperationContext, OperationOutcome, OperationScope, SessionPolicy};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Line};
use kgeom::curve2d::{Curve2d, Line2d};
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point3, Vec2};
use kgraph::{
    AffineParamMap1d, Curve2dDescriptor, CurveDescriptor, EvalError, GeometryGraph, GeometryRef,
    IntersectionCertificateError, PairedPlaneLineResidualCertificate, SurfaceHandle,
    certify_paired_plane_line_residuals,
};

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
}

impl GraphSurfaceIntersectionError {
    /// Broad semantic error class.
    pub const fn class(&self) -> ErrorClass {
        match self {
            Self::GeometryEvaluation(error) => error.class(),
            Self::Intersection(error) => error.class(),
            Self::BranchCertificate(_) => ErrorClass::InternalInvariant,
        }
    }

    /// Stable machine-readable failure identity.
    pub const fn code(&self) -> ErrorCode {
        match self {
            Self::GeometryEvaluation(error) => error.code(),
            Self::Intersection(error) => error.code(),
            Self::BranchCertificate(_) => BRANCH_CERTIFICATE_FAILURE,
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
        }
    }
}

impl std::error::Error for GraphSurfaceIntersectionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::GeometryEvaluation(error) => Some(error),
            Self::Intersection(error) => Some(error),
            Self::BranchCertificate(error) => Some(error),
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
        }
    }

    fn limit(&self) -> Option<kcore::operation::LimitSnapshot> {
        match self {
            Self::GeometryEvaluation(error) => error.limit(),
            Self::Intersection(error) => error.limit(),
            Self::BranchCertificate(_) => None,
        }
    }
}

impl From<IntersectionError> for GraphSurfaceIntersectionError {
    fn from(error: IntersectionError) -> Self {
        Self::Intersection(error)
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
    pub certificate: PairedPlaneLineResidualCertificate,
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
    let mut scope = OperationScope::new(context);
    let result = intersect_bounded_graph_surfaces_in_scope(
        graph, surface_a, range_a, surface_b, range_b, &mut scope,
    );
    scope.finish_typed(result)
}

/// Intersect graph-owned surfaces inside an existing owner operation scope.
///
/// This function never creates or finishes a nested scope. Direct descriptor
/// classes are resolved exactly once per operand; procedural offsets and all
/// class pairs other than Plane/Plane remain explicitly unsupported.
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
    let (Some(plane_a), Some(plane_b)) = (descriptor_a.as_plane(), descriptor_b.as_plane()) else {
        return Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::UnsupportedSurfacePair {
                class_a: Some(classes[0]),
                class_b: Some(classes[1]),
            },
        ));
    };

    let tolerances = scope.context().tolerances();
    let raw = intersect_bounded_planes(plane_a, range_a, plane_b, range_b, tolerances)
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let branch_graph = build_verified_plane_branch_graph(
        [surface_a, surface_b],
        [*plane_a, *plane_b],
        [range_a, range_b],
        &raw,
        tolerances.linear(),
    )?;
    Ok(GraphSurfaceSurfaceIntersections { raw, branch_graph })
}

fn build_verified_plane_branch_graph(
    source_surfaces: [SurfaceHandle; 2],
    surfaces: [Plane; 2],
    surface_ranges: [[ParamRange; 2]; 2],
    raw: &SurfaceSurfaceIntersections,
    tolerance: f64,
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
        let SurfaceIntersectionCurve::Line(raw_line) = &branch.curve else {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                kgraph::IntersectionCertificateError::NonFiniteGeometry,
            ));
        };
        let (carrier, carrier_range) = canonical_line(*raw_line, branch.curve_range)
            .map_err(IntersectionError::from)
            .map_err(GraphSurfaceIntersectionError::Intersection)?;
        let (pcurve_a, map_a) = plane_pcurve(carrier, surfaces[0])?;
        let (pcurve_b, map_b) = plane_pcurve(carrier, surfaces[1])?;
        let pcurves = [pcurve_a, pcurve_b];
        let parameter_maps = [map_a, map_b];
        let certificate = certify_paired_plane_line_residuals(
            carrier,
            carrier_range,
            surfaces,
            pcurves,
            parameter_maps,
            tolerance,
        )
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;

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
                pcurve_a.eval(map_a.map(parameter)),
                pcurve_b.eval(map_b.map(parameter)),
            ];
            let boundary_surfaces = [
                on_window_boundary(surface_parameters[0], surface_ranges[0], tolerance),
                on_window_boundary(surface_parameters[1], surface_ranges[1], tolerance),
            ];
            endpoint_events[endpoint] = IntersectionBranchEndpointEvent::SurfaceWindowBoundary {
                surfaces: boundary_surfaces,
            };
            vertices.push(IntersectionBranchVertex {
                point: carrier.eval(parameter),
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
            carrier: CurveDescriptor::Line(carrier),
            carrier_range,
            pcurves: [
                Curve2dDescriptor::Line(pcurve_a),
                Curve2dDescriptor::Line(pcurve_b),
            ],
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
