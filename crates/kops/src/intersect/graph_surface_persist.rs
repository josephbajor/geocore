//! Transactional persistence boundary for verified surface branches.

use kgraph::{GeometryGraph, IntersectionCertificateError, VerifiedIntersectionCertificate};

use super::error::IntersectionError;
use super::graph_surface::{
    GraphSurfaceIntersectionError, GraphSurfaceIntersectionResult,
    GraphSurfaceSurfaceIntersections, IntersectionBranchCertificate,
    PersistentIntersectionBranchEdge, PersistentIntersectionBranchGraph,
};

const CYLINDER_PERSISTENCE_REASON: &str =
    "persistent analytic cylinder branches require a dedicated descriptor contract";

/// Persist every certified positive-length branch into the geometry graph.
///
/// Operation-local cylinder families are rejected before any descriptor
/// insertion. All supported insertions then commit or roll back as one batch.
pub fn persist_verified_graph_surface_intersections(
    graph: &mut GeometryGraph,
    intersections: &GraphSurfaceSurfaceIntersections,
) -> GraphSurfaceIntersectionResult<PersistentIntersectionBranchGraph> {
    if intersections
        .branch_graph
        .edges
        .iter()
        .any(|edge| edge.certificate.is_operation_local_cylinder())
    {
        return Err(operation_local_cylinder_refusal());
    }
    graph.begin_undo_frame();
    let result = persist_impl(graph, intersections);
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

fn persist_impl(
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
            IntersectionBranchCertificate::PlaneCylinderCircle(_)
            | IntersectionBranchCertificate::PlaneCylinderRuling(_)
            | IntersectionBranchCertificate::CylinderCylinderRuling(_)
            | IntersectionBranchCertificate::SkewCylinderTwoSheet(_) => {
                return Err(operation_local_cylinder_refusal());
            }
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

fn operation_local_cylinder_refusal() -> GraphSurfaceIntersectionError {
    GraphSurfaceIntersectionError::BranchCertificate(
        IntersectionCertificateError::UnsupportedCarrierParameterization {
            reason: CYLINDER_PERSISTENCE_REASON,
        },
    )
}
