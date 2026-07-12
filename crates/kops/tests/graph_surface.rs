//! Graph-aware Plane/Plane intersection branch contracts.

use kcore::error::{ClassifiedError, ErrorClass};
use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Surface};
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    Curve2dDescriptor, CurveDescriptor, EvalError, GeometryGraph, GeometryRef,
    OffsetSurfaceDescriptor, SurfaceClass,
};
use kops::intersect::{
    ContactKind, GraphSurfaceIntersectionError, IntersectionBranchVertexEvent, IntersectionError,
    SURFACE_SURFACE_CLASS_PAIR, intersect_bounded_graph_surfaces,
    intersect_bounded_graph_surfaces_in_scope, intersect_bounded_graph_surfaces_with_context,
    intersect_bounded_planes,
};

fn horizontal_plane(z: f64) -> Plane {
    Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, z),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    )
}

fn vertical_plane_x(x: f64) -> Plane {
    Plane::new(
        Frame::new(
            Point3::new(x, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
    )
}

fn window() -> [ParamRange; 2] {
    [ParamRange::new(-1.0, 1.0), ParamRange::new(-1.0, 1.0)]
}

fn assert_edge_lifts_over_complete_range(
    edge: &kops::intersect::IntersectionBranchEdge,
    surfaces: [Plane; 2],
) {
    let CurveDescriptor::Line(carrier) = &edge.carrier else {
        panic!("Plane/Plane carrier must remain an exact line");
    };
    for pcurve in &edge.pcurves {
        assert!(matches!(pcurve, Curve2dDescriptor::Line(_)));
    }
    for t in [
        edge.carrier_range.lo,
        edge.carrier_range.lerp(0.37),
        edge.carrier_range.hi,
    ] {
        let point = carrier.eval(t);
        for (surface_index, surface) in surfaces.iter().enumerate() {
            let uv = edge.pcurves[surface_index]
                .as_curve()
                .eval(edge.parameter_maps[surface_index].map(t));
            let lifted = surface.eval([uv.x, uv.y]);
            assert!(point.dist(lifted) <= edge.certificate.tolerance());
        }
    }
    assert!(
        edge.certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= edge.certificate.tolerance())
    );
}

#[test]
fn plane_pair_preserves_raw_solver_and_builds_certified_graph_in_one_scope() {
    let plane_a = horizontal_plane(0.0);
    let plane_b = vertical_plane_x(0.0);
    let range_a = [ParamRange::new(-2.0, 2.0), ParamRange::new(-0.5, 0.75)];
    let range_b = [ParamRange::new(0.0, 1.0), ParamRange::new(-1.0, 1.0)];
    let tolerances = Tolerances::default();
    let mut graph = GeometryGraph::new();
    let surface_a = graph.insert_surface(plane_a).unwrap();
    let surface_b = graph.insert_surface(plane_b).unwrap();

    let hit = intersect_bounded_graph_surfaces(
        &graph, surface_a, range_a, surface_b, range_b, tolerances,
    )
    .unwrap();
    let legacy =
        intersect_bounded_planes(&plane_a, range_a, &plane_b, range_b, tolerances).unwrap();
    assert_eq!(hit.raw, legacy);
    assert_eq!(hit.branch_graph.source_surfaces, [surface_a, surface_b]);
    assert_eq!(hit.branch_graph.vertices.len(), 2);
    assert_eq!(hit.branch_graph.edges.len(), 1);

    let edge = &hit.branch_graph.edges[0];
    assert_eq!(edge.source_surfaces, [surface_a, surface_b]);
    assert_eq!(edge.endpoint_vertices, [0, 1]);
    assert_eq!(edge.kind, ContactKind::Transverse);
    assert_edge_lifts_over_complete_range(edge, [plane_a, plane_b]);
    for endpoint in edge.endpoint_vertices {
        assert!(matches!(
            hit.branch_graph.vertices[endpoint].event,
            IntersectionBranchVertexEvent::BoundaryEndpoint { .. }
        ));
    }

    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let wrapped = intersect_bounded_graph_surfaces_with_context(
        &graph, surface_a, range_a, surface_b, range_b, &context,
    );
    let mut scope = OperationScope::new(&context);
    let scoped_result = intersect_bounded_graph_surfaces_in_scope(
        &graph, surface_a, range_a, surface_b, range_b, &mut scope,
    );
    let scoped = scope.finish_typed(scoped_result);
    assert_eq!(wrapped, scoped);
}

#[test]
fn operand_swap_preserves_canonical_carrier_and_swaps_trace_provenance() {
    let plane_a = horizontal_plane(0.0);
    let plane_b = vertical_plane_x(0.0);
    let mut graph = GeometryGraph::new();
    let surface_a = graph.insert_surface(plane_a).unwrap();
    let surface_b = graph.insert_surface(plane_b).unwrap();
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        surface_a,
        window(),
        surface_b,
        window(),
        Tolerances::default(),
    )
    .unwrap();
    let reverse = intersect_bounded_graph_surfaces(
        &graph,
        surface_b,
        window(),
        surface_a,
        window(),
        Tolerances::default(),
    )
    .unwrap();

    let first = &forward.branch_graph.edges[0];
    let second = &reverse.branch_graph.edges[0];
    assert_eq!(first.source_surfaces, [surface_a, surface_b]);
    assert_eq!(second.source_surfaces, [surface_b, surface_a]);
    assert_eq!(first.carrier, second.carrier);
    assert_eq!(first.carrier_range, second.carrier_range);
    assert_eq!(first.pcurves[0], second.pcurves[1]);
    assert_eq!(first.pcurves[1], second.pcurves[0]);
    assert_eq!(first.parameter_maps[0], second.parameter_maps[1]);
    assert_eq!(first.parameter_maps[1], second.parameter_maps[0]);
    for vertex in 0..2 {
        assert_eq!(
            forward.branch_graph.vertices[vertex].point,
            reverse.branch_graph.vertices[vertex].point
        );
        assert_eq!(
            forward.branch_graph.vertices[vertex].surface_parameters[0],
            reverse.branch_graph.vertices[vertex].surface_parameters[1]
        );
        assert_eq!(
            forward.branch_graph.vertices[vertex].surface_parameters[1],
            reverse.branch_graph.vertices[vertex].surface_parameters[0]
        );
    }
}

#[test]
fn clipped_point_and_proven_miss_produce_vertex_only_and_empty_graphs() {
    let plane_a = horizontal_plane(0.0);
    let plane_b = vertical_plane_x(0.0);
    let mut graph = GeometryGraph::new();
    let surface_a = graph.insert_surface(plane_a).unwrap();
    let surface_b = graph.insert_surface(plane_b).unwrap();
    let point = intersect_bounded_graph_surfaces(
        &graph,
        surface_a,
        [ParamRange::new(-1.0, 1.0), ParamRange::new(0.0, 1.0)],
        surface_b,
        [ParamRange::new(1.0, 2.0), ParamRange::new(-1.0, 1.0)],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(point.raw.points.len(), 1);
    assert_eq!(point.branch_graph.vertices.len(), 1);
    assert!(point.branch_graph.edges.is_empty());
    assert_eq!(
        point.branch_graph.vertices[0].event,
        IntersectionBranchVertexEvent::IsolatedContact
    );

    let separated = graph.insert_surface(horizontal_plane(2.0)).unwrap();
    let miss = intersect_bounded_graph_surfaces(
        &graph,
        surface_a,
        window(),
        separated,
        window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.raw.is_proven_empty());
    assert!(miss.branch_graph.vertices.is_empty());
    assert!(miss.branch_graph.edges.is_empty());
}

#[test]
fn stale_and_offset_sources_remain_typed_and_never_become_complete_misses() {
    let mut graph = GeometryGraph::new();
    let stale = graph.insert_surface(horizontal_plane(0.0)).unwrap();
    graph.remove_surface(stale).unwrap();
    let live = graph.insert_surface(vertical_plane_x(0.0)).unwrap();
    let stale_error = intersect_bounded_graph_surfaces(
        &graph,
        stale,
        window(),
        live,
        window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(stale_error.class(), ErrorClass::InvalidInput);
    assert!(matches!(
        stale_error,
        GraphSurfaceIntersectionError::GeometryEvaluation(
            EvalError::StaleGeometryHandle {
                geometry: GeometryRef::Surface(handle)
            }
        ) if handle == stale
    ));

    let plane = graph.insert_surface(horizontal_plane(0.0)).unwrap();
    let offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(plane, 0.5))
        .unwrap();
    let unsupported = intersect_bounded_graph_surfaces(
        &graph,
        plane,
        window(),
        offset,
        window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(unsupported.class(), ErrorClass::Unsupported);
    assert_eq!(unsupported.capability(), Some(SURFACE_SURFACE_CLASS_PAIR));
    assert!(matches!(
        unsupported,
        GraphSurfaceIntersectionError::Intersection(
            IntersectionError::UnsupportedSurfacePair {
                class_a: Some(class_a),
                class_b: Some(class_b),
            }
        ) if class_a == SurfaceClass::Plane.key() && class_b == SurfaceClass::Offset.key()
    ));
}
