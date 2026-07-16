//! Graph-aware Plane/Plane intersection branch contracts.

use kcore::error::{ClassifiedError, ErrorClass};
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationScope, ResourceKind,
    SessionPolicy,
};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Line};
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere, Surface};
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    Curve2dDescriptor, CurveClass, CurveDescriptor, EvalContext, EvalError, EvalLimits,
    GeometryGraph, GeometryGraphError, GeometryRef, IntersectionCertificateError,
    OffsetSurfaceDescriptor, PairedTrace, PlaneSphereCircleTrace, SurfaceClass,
};
use kops::intersect::{
    BRANCH_CERTIFICATE_FAILURE, CURVE_CURVE_CLASS_PAIR, ContactKind, GraphSurfaceBudgetProfile,
    GraphSurfaceIntersectionError, IntersectionBranchVertexEvent, IntersectionError,
    SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS, SURFACE_SURFACE_CLASS_PAIR, intersect_bounded_curves,
    intersect_bounded_graph_surfaces, intersect_bounded_graph_surfaces_in_scope,
    intersect_bounded_graph_surfaces_with_context, intersect_bounded_plane_sphere,
    intersect_bounded_planes, persist_verified_graph_surface_intersections,
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

fn rotated_common_axis_plane(z: f64, orientation: f64) -> Plane {
    Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, z),
            Vec3::new(0.0, 0.0, orientation),
            Vec3::new(0.6, 0.8, 0.0),
        )
        .unwrap(),
    )
}

fn oblique_plane(offset: f64) -> Plane {
    let normal = Vec3::new(0.0, 0.6, 0.8);
    Plane::new(Frame::new(normal * offset, normal, Vec3::new(1.0, 0.0, 0.0)).unwrap())
}

fn bilinear_nurbs_surface(z: f64) -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, -1.0, z),
            Point3::new(-1.0, 1.0, z),
            Point3::new(1.0, -1.0, z),
            Point3::new(1.0, 1.0, z),
        ],
        None,
    )
    .unwrap()
}

fn window() -> [ParamRange; 2] {
    [ParamRange::new(-1.0, 1.0), ParamRange::new(-1.0, 1.0)]
}

fn wide_window() -> [ParamRange; 2] {
    [ParamRange::new(-3.0, 3.0), ParamRange::new(-3.0, 3.0)]
}

fn sphere_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ]
}

fn assert_plane_sphere_edge_lifts_over_complete_range(
    edge: &kops::intersect::IntersectionBranchEdge,
    plane: Plane,
    sphere: Sphere,
    plane_first: bool,
) {
    let CurveDescriptor::Circle(carrier) = &edge.carrier else {
        panic!("Plane/Sphere carrier must remain an exact circle");
    };
    let plane_index = usize::from(!plane_first);
    let sphere_index = usize::from(plane_first);
    assert!(matches!(
        edge.pcurves[plane_index],
        Curve2dDescriptor::Circle(_)
    ));
    assert!(matches!(
        edge.pcurves[sphere_index],
        Curve2dDescriptor::Line(_)
    ));
    let certificate = edge.certificate.as_plane_sphere_circle().unwrap();
    assert!(matches!(
        certificate.traces()[plane_index],
        PlaneSphereCircleTrace::Plane(_)
    ));
    assert!(matches!(
        certificate.traces()[sphere_index],
        PlaneSphereCircleTrace::Sphere(_)
    ));

    for t in [
        edge.carrier_range.lo,
        edge.carrier_range.lerp(0.37),
        edge.carrier_range.hi,
    ] {
        let point = carrier.eval(t);
        let plane_uv = edge.pcurves[plane_index]
            .as_curve()
            .eval(edge.parameter_maps[plane_index].map(t));
        let sphere_uv = edge.pcurves[sphere_index]
            .as_curve()
            .eval(edge.parameter_maps[sphere_index].map(t));
        assert!(point.dist(plane.eval([plane_uv.x, plane_uv.y])) <= edge.certificate.tolerance());
        assert!(
            point.dist(sphere.eval([sphere_uv.x, sphere_uv.y])) <= edge.certificate.tolerance()
        );
    }
    assert!(
        edge.certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= edge.certificate.tolerance())
    );
}

fn assert_oblique_plane_sphere_edge_lifts_over_complete_range(
    edge: &kops::intersect::IntersectionBranchEdge,
    plane: Plane,
    sphere: Sphere,
    plane_first: bool,
) {
    let CurveDescriptor::Circle(carrier) = &edge.carrier else {
        panic!("Plane/Sphere carrier must remain an exact circle");
    };
    let plane_index = usize::from(!plane_first);
    let sphere_index = usize::from(plane_first);
    assert!(matches!(
        edge.pcurves[plane_index],
        Curve2dDescriptor::Circle(_)
    ));
    assert!(matches!(
        edge.pcurves[sphere_index],
        Curve2dDescriptor::SphericalCircle(_)
    ));
    let certificate = edge.certificate.as_plane_sphere_circle().unwrap();
    assert!(matches!(
        certificate.traces()[plane_index],
        PlaneSphereCircleTrace::Plane(_)
    ));
    assert!(matches!(
        certificate.traces()[sphere_index],
        PlaneSphereCircleTrace::SphereOblique(_)
    ));
    assert_eq!(edge.parameter_maps[plane_index].scale(), 1.0);
    assert_eq!(edge.parameter_maps[sphere_index].scale(), 1.0);

    let pcurve = edge.pcurves[sphere_index].as_curve();
    let bounds = pcurve.bounding_box(edge.carrier_range);
    for t in [
        edge.carrier_range.lo,
        edge.carrier_range.lerp(0.23),
        edge.carrier_range.lerp(0.61),
        edge.carrier_range.hi,
    ] {
        let point = carrier.eval(t);
        let plane_uv = edge.pcurves[plane_index].as_curve().eval(t);
        let sphere_derivs = pcurve.eval_derivs(t, 3);
        let sphere_uv = sphere_derivs.d[0];
        assert!(bounds.contains(sphere_uv));
        assert!(
            sphere_derivs
                .d
                .iter()
                .all(|value| value.x.is_finite() && value.y.is_finite())
        );
        assert!(point.dist(plane.eval([plane_uv.x, plane_uv.y])) <= edge.certificate.tolerance());
        assert!(
            point.dist(sphere.eval([sphere_uv.x, sphere_uv.y])) <= edge.certificate.tolerance()
        );
    }
    assert!(
        edge.certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= edge.certificate.tolerance())
    );
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
    let composed = context
        .clone()
        .with_family_budget_defaults(GraphSurfaceBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&composed);
    let scoped_result = intersect_bounded_graph_surfaces_in_scope(
        &graph, surface_a, range_a, surface_b, range_b, &mut scope,
    );
    let scoped = scope.finish_typed(scoped_result);
    assert_eq!(wrapped, scoped);
}

#[test]
fn offset_plane_field_is_exact_contextual_and_preserves_graph_source_identity() {
    let basis_plane = horizontal_plane(0.0);
    let effective_plane = horizontal_plane(0.5);
    let vertical_plane = vertical_plane_x(0.0);
    let mut graph = GeometryGraph::new();
    let basis = graph.insert_surface(basis_plane).unwrap();
    let first = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.75))
        .unwrap();
    let offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(first, -0.25))
        .unwrap();
    let vertical = graph.insert_surface(vertical_plane).unwrap();
    let tolerances = Tolerances::default();
    let legacy = intersect_bounded_planes(
        &effective_plane,
        window(),
        &vertical_plane,
        window(),
        tolerances,
    )
    .unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        offset,
        window(),
        vertical,
        window(),
        &context,
    );
    let result = outcome.result().unwrap();

    assert_eq!(result.raw, legacy);
    assert!(result.raw.is_complete());
    assert_eq!(result.branch_graph.source_surfaces, [offset, vertical]);
    let edge = &result.branch_graph.edges[0];
    assert_eq!(edge.source_surfaces, [offset, vertical]);
    assert_eq!(
        edge.certificate.as_plane_line().unwrap().surfaces(),
        [effective_plane, vertical_plane]
    );
    assert_edge_lifts_over_complete_range(edge, [effective_plane, vertical_plane]);
    let visits = outcome
        .report()
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == kgraph::eval_stage::NODE_VISITS)
        .unwrap();
    let depth = outcome
        .report()
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == kgraph::eval_stage::DEPENDENCY_DEPTH)
        .unwrap();
    assert_eq!((visits.resource, visits.consumed), (ResourceKind::Work, 3));
    assert_eq!((depth.resource, depth.consumed), (ResourceKind::Depth, 3));

    let limited = BudgetPlan::new([LimitSpec::new(
        kgraph::eval_stage::NODE_VISITS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        2,
    )])
    .unwrap();
    let limited_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(limited);
    let limited = intersect_bounded_graph_surfaces_with_context(
        &graph,
        offset,
        window(),
        vertical,
        window(),
        &limited_context,
    );
    let crossing = limited.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(crossing.stage, kgraph::eval_stage::NODE_VISITS);
    assert_eq!((crossing.consumed, crossing.allowed), (3, 2));
    assert_eq!(limited.report().limit_events(), &[crossing]);
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
fn persistence_retains_branch_order_operand_provenance_and_fail_closed_curve_class() {
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
    let before = (graph.curve_count(), graph.curve2d_count());
    let persistent_forward =
        persist_verified_graph_surface_intersections(&mut graph, &forward).unwrap();
    let persistent_reverse =
        persist_verified_graph_surface_intersections(&mut graph, &reverse).unwrap();

    assert_eq!(persistent_forward.source_surfaces, [surface_a, surface_b]);
    assert_eq!(persistent_reverse.source_surfaces, [surface_b, surface_a]);
    assert_eq!(persistent_forward.vertices, forward.branch_graph.vertices);
    assert_eq!(persistent_reverse.vertices, reverse.branch_graph.vertices);
    assert_eq!(persistent_forward.edges.len(), 1);
    assert_eq!(persistent_reverse.edges.len(), 1);
    assert_eq!(graph.curve_count(), before.0 + 2);
    assert_eq!(graph.curve2d_count(), before.1 + 4);

    let forward_edge = persistent_forward.edges[0];
    let reverse_edge = persistent_reverse.edges[0];
    let forward_descriptor = graph
        .curve(forward_edge.curve)
        .unwrap()
        .as_intersection()
        .copied()
        .unwrap();
    let reverse_descriptor = graph
        .curve(reverse_edge.curve)
        .unwrap()
        .as_intersection()
        .copied()
        .unwrap();
    assert_eq!(forward_descriptor.source_surfaces(), [surface_a, surface_b]);
    assert_eq!(reverse_descriptor.source_surfaces(), [surface_b, surface_a]);
    assert_eq!(forward_descriptor.pcurves(), forward_edge.pcurves);
    assert_eq!(reverse_descriptor.pcurves(), reverse_edge.pcurves);
    assert_eq!(forward_descriptor.carrier(), reverse_descriptor.carrier());
    assert_eq!(
        forward_descriptor
            .certificate()
            .as_plane_line()
            .unwrap()
            .pcurves()[0],
        reverse_descriptor
            .certificate()
            .as_plane_line()
            .unwrap()
            .pcurves()[1]
    );
    assert_eq!(
        forward_descriptor
            .certificate()
            .as_plane_line()
            .unwrap()
            .pcurves()[1],
        reverse_descriptor
            .certificate()
            .as_plane_line()
            .unwrap()
            .pcurves()[0]
    );
    assert_eq!(
        forward_descriptor.certificate().parameter_maps()[0],
        reverse_descriptor.certificate().parameter_maps()[1]
    );
    assert_eq!(
        forward_descriptor.certificate().parameter_maps()[1],
        reverse_descriptor.certificate().parameter_maps()[0]
    );
    assert_eq!(
        graph
            .direct_dependencies(GeometryRef::Curve(forward_edge.curve))
            .unwrap(),
        vec![
            GeometryRef::Surface(surface_a),
            GeometryRef::Surface(surface_b),
            GeometryRef::Curve2d(forward_edge.pcurves[0]),
            GeometryRef::Curve2d(forward_edge.pcurves[1]),
        ]
    );

    let line = Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let unsupported = intersect_bounded_curves(
        graph.curve(forward_edge.curve).unwrap().as_curve(),
        forward_descriptor.carrier_range(),
        &line,
        ParamRange::new(-1.0, 1.0),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(unsupported.capability(), Some(CURVE_CURVE_CLASS_PAIR));
    assert!(matches!(
        unsupported,
        IntersectionError::UnsupportedCurvePair {
            class_a: Some(class_a),
            class_b: Some(class_b),
        } if class_a == CurveClass::Intersection.key() && class_b == CurveClass::Line.key()
    ));
    graph.validate().unwrap();
}

#[test]
fn persistence_rolls_back_the_whole_batch_on_certificate_or_handle_failure() {
    let build = || {
        let mut graph = GeometryGraph::new();
        let surface_a = graph.insert_surface(horizontal_plane(0.0)).unwrap();
        let surface_b = graph.insert_surface(vertical_plane_x(0.0)).unwrap();
        let intersections = intersect_bounded_graph_surfaces(
            &graph,
            surface_a,
            window(),
            surface_b,
            window(),
            Tolerances::default(),
        )
        .unwrap();
        (graph, surface_a, surface_b, intersections)
    };

    let (mut altered, altered_a, _, altered_intersections) = build();
    altered
        .replace_surface(altered_a, horizontal_plane(0.25))
        .unwrap();
    let counts = (
        altered.curve_count(),
        altered.surface_count(),
        altered.curve2d_count(),
    );
    let order = altered.geometry().collect::<Vec<_>>();
    let mismatch =
        persist_verified_graph_surface_intersections(&mut altered, &altered_intersections)
            .unwrap_err();
    assert!(matches!(
        mismatch,
        GraphSurfaceIntersectionError::GeometryPersistence(
            GeometryGraphError::InvalidDescriptor { class, .. }
        ) if class == CurveClass::Intersection.key()
    ));
    assert_eq!(
        (
            altered.curve_count(),
            altered.surface_count(),
            altered.curve2d_count(),
        ),
        counts
    );
    assert_eq!(altered.geometry().collect::<Vec<_>>(), order);
    altered.validate().unwrap();

    let (mut stale, stale_a, _, stale_intersections) = build();
    stale.remove_surface(stale_a).unwrap();
    let counts = (
        stale.curve_count(),
        stale.surface_count(),
        stale.curve2d_count(),
    );
    let order = stale.geometry().collect::<Vec<_>>();
    let stale_error =
        persist_verified_graph_surface_intersections(&mut stale, &stale_intersections).unwrap_err();
    assert_eq!(
        stale_error,
        GraphSurfaceIntersectionError::GeometryPersistence(
            GeometryGraphError::StaleGeometryHandle {
                geometry: GeometryRef::Surface(stale_a),
            }
        )
    );
    assert_eq!(
        (
            stale.curve_count(),
            stale.surface_count(),
            stale.curve2d_count(),
        ),
        counts
    );
    assert_eq!(stale.geometry().collect::<Vec<_>>(), order);
    stale.validate().unwrap();
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
fn aligned_plane_sphere_secant_clipping_tangent_and_miss_preserve_raw_completion() {
    let tolerances = Tolerances::default();
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let secant_plane = horizontal_plane(0.5);
    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(secant_plane).unwrap();
    let sphere_handle = graph.insert_surface(sphere).unwrap();

    let secant = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        window(),
        sphere_handle,
        sphere_window(),
        tolerances,
    )
    .unwrap();
    let secant_raw = intersect_bounded_plane_sphere(
        &secant_plane,
        window(),
        &sphere,
        sphere_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(secant.raw, secant_raw);
    assert!(secant.raw.is_complete());
    assert_eq!(secant.branch_graph.edges.len(), 1);
    assert_plane_sphere_edge_lifts_over_complete_range(
        &secant.branch_graph.edges[0],
        secant_plane,
        sphere,
        true,
    );

    let clipped_sphere_window = [
        ParamRange::new(0.0, core::f64::consts::PI),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ];
    let clipped = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        window(),
        sphere_handle,
        clipped_sphere_window,
        tolerances,
    )
    .unwrap();
    let clipped_raw = intersect_bounded_plane_sphere(
        &secant_plane,
        window(),
        &sphere,
        clipped_sphere_window,
        tolerances,
    )
    .unwrap();
    assert_eq!(clipped.raw, clipped_raw);
    assert_eq!(clipped.branch_graph.edges.len(), 1);
    assert_eq!(
        clipped.branch_graph.edges[0].carrier_range,
        ParamRange::new(0.0, core::f64::consts::PI)
    );
    assert_plane_sphere_edge_lifts_over_complete_range(
        &clipped.branch_graph.edges[0],
        secant_plane,
        sphere,
        true,
    );

    let tangent_plane = horizontal_plane(1.0);
    let tangent_handle = graph.insert_surface(tangent_plane).unwrap();
    let tangent = intersect_bounded_graph_surfaces(
        &graph,
        tangent_handle,
        window(),
        sphere_handle,
        sphere_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(
        tangent.raw,
        intersect_bounded_plane_sphere(
            &tangent_plane,
            window(),
            &sphere,
            sphere_window(),
            tolerances,
        )
        .unwrap()
    );
    assert_eq!(tangent.raw.points.len(), 1);
    assert_eq!(tangent.raw.points[0].kind, ContactKind::Singular);
    assert_eq!(tangent.branch_graph.vertices.len(), 1);
    assert!(tangent.branch_graph.edges.is_empty());

    let miss_plane = horizontal_plane(2.0);
    let miss_handle = graph.insert_surface(miss_plane).unwrap();
    let miss = intersect_bounded_graph_surfaces(
        &graph,
        miss_handle,
        window(),
        sphere_handle,
        sphere_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(
        miss.raw,
        intersect_bounded_plane_sphere(
            &miss_plane,
            window(),
            &sphere,
            sphere_window(),
            tolerances,
        )
        .unwrap()
    );
    assert!(miss.raw.is_proven_empty());
    assert!(miss.branch_graph.vertices.is_empty());
    assert!(miss.branch_graph.edges.is_empty());
}

#[test]
fn shifted_full_turn_longitude_persists_and_bounds_the_canonical_sphere_parameter() {
    let plane = horizontal_plane(0.5);
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let tau = core::f64::consts::TAU;
    let shifted_sphere_window = [
        ParamRange::new(2.0 * tau, 3.0 * tau),
        ParamRange::new(-2.0, 2.0),
    ];
    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let sphere_handle = graph.insert_surface(sphere).unwrap();
    let tolerances = Tolerances::default();
    let result = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        window(),
        sphere_handle,
        shifted_sphere_window,
        tolerances,
    )
    .unwrap();

    assert_eq!(
        result.raw,
        intersect_bounded_plane_sphere(
            &plane,
            window(),
            &sphere,
            shifted_sphere_window,
            tolerances,
        )
        .unwrap()
    );
    assert_eq!(result.branch_graph.edges.len(), 1);
    let edge = &result.branch_graph.edges[0];
    assert_eq!(edge.carrier_range, shifted_sphere_window[0]);
    assert_plane_sphere_edge_lifts_over_complete_range(edge, plane, sphere, true);
    for t in [edge.carrier_range.lo, edge.carrier_range.hi] {
        let uv = edge.pcurves[1]
            .as_curve()
            .eval(edge.parameter_maps[1].map(t));
        assert!(shifted_sphere_window[0].contains(uv.x));
        assert!(shifted_sphere_window[1].contains(uv.y));
    }

    let persistent = persist_verified_graph_surface_intersections(&mut graph, &result).unwrap();
    let curve = persistent.edges[0].curve;
    let mut evaluator = EvalContext::new(&graph, EvalLimits::default(), tolerances);
    let bounds = evaluator.curve_bounds(curve, edge.carrier_range).unwrap();
    for t in [
        edge.carrier_range.lo,
        edge.carrier_range.lerp(0.25),
        edge.carrier_range.lerp(0.5),
        edge.carrier_range.hi,
    ] {
        assert!(bounds.contains(evaluator.eval_curve(curve, t, 0).unwrap().d[0]));
    }
    graph.validate().unwrap();

    let overwide_sphere_window = [ParamRange::new(1.0, 8.0), ParamRange::new(-2.0, 2.0)];
    let overwide = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        window(),
        sphere_handle,
        overwide_sphere_window,
        tolerances,
    )
    .unwrap();
    assert_eq!(
        overwide.raw,
        intersect_bounded_plane_sphere(
            &plane,
            window(),
            &sphere,
            overwide_sphere_window,
            tolerances,
        )
        .unwrap()
    );
    assert_eq!(overwide.branch_graph.edges.len(), overwide.raw.curves.len());
    assert!(!overwide.branch_graph.edges.is_empty());
    for edge in &overwide.branch_graph.edges {
        assert!(edge.carrier_range.lo >= overwide_sphere_window[0].lo);
        assert!(edge.carrier_range.hi <= overwide_sphere_window[0].hi);
        assert_plane_sphere_edge_lifts_over_complete_range(edge, plane, sphere, true);
    }

    let antialigned_plane = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let antialigned_handle = graph.insert_surface(antialigned_plane).unwrap();
    let antialigned = intersect_bounded_graph_surfaces(
        &graph,
        antialigned_handle,
        window(),
        sphere_handle,
        shifted_sphere_window,
        tolerances,
    )
    .unwrap();
    assert_eq!(
        antialigned.raw,
        intersect_bounded_plane_sphere(
            &antialigned_plane,
            window(),
            &sphere,
            shifted_sphere_window,
            tolerances,
        )
        .unwrap()
    );
    assert_eq!(antialigned.branch_graph.edges.len(), 1);
    let antialigned_edge = &antialigned.branch_graph.edges[0];
    assert_eq!(antialigned_edge.carrier_range, shifted_sphere_window[0]);
    assert_eq!(antialigned_edge.parameter_maps[0].scale(), -1.0);
    assert_plane_sphere_edge_lifts_over_complete_range(
        antialigned_edge,
        antialigned_plane,
        sphere,
        true,
    );
}

#[test]
fn rotated_and_antialigned_common_axis_charts_are_seam_aware_and_swap_stable() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let sphere_ranges = [ParamRange::new(5.5, 7.0), ParamRange::new(-2.0, 2.0)];
    let tolerances = Tolerances::default();

    for orientation in [1.0, -1.0] {
        let plane = rotated_common_axis_plane(0.5, orientation);
        let mut graph = GeometryGraph::new();
        let plane_handle = graph.insert_surface(plane).unwrap();
        let sphere_handle = graph.insert_surface(sphere).unwrap();
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            plane_handle,
            window(),
            sphere_handle,
            sphere_ranges,
            tolerances,
        )
        .unwrap();
        let raw =
            intersect_bounded_plane_sphere(&plane, window(), &sphere, sphere_ranges, tolerances)
                .unwrap();
        assert_eq!(forward.raw, raw);
        assert!(!forward.raw.curves.is_empty());
        assert_eq!(forward.branch_graph.edges.len(), forward.raw.curves.len());
        for edge in &forward.branch_graph.edges {
            assert_eq!(edge.parameter_maps[0].scale(), orientation);
            assert_eq!(edge.parameter_maps[1].scale(), 1.0);
            assert!(edge.carrier_range.lo >= sphere_ranges[0].lo);
            assert!(edge.carrier_range.hi <= sphere_ranges[0].hi);
            assert_plane_sphere_edge_lifts_over_complete_range(edge, plane, sphere, true);
        }

        let reverse = intersect_bounded_graph_surfaces(
            &graph,
            sphere_handle,
            sphere_ranges,
            plane_handle,
            window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(reverse.raw, forward.raw.clone().swapped());
        assert_eq!(
            reverse.branch_graph.edges.len(),
            forward.branch_graph.edges.len()
        );
        for reverse_edge in &reverse.branch_graph.edges {
            assert_eq!(reverse_edge.parameter_maps[0].scale(), 1.0);
            assert_eq!(reverse_edge.parameter_maps[1].scale(), orientation);
            assert_plane_sphere_edge_lifts_over_complete_range(reverse_edge, plane, sphere, false);
            let forward_edge = forward
                .branch_graph
                .edges
                .iter()
                .find(|edge| edge.carrier_range == reverse_edge.carrier_range)
                .unwrap();
            assert_eq!(forward_edge.carrier, reverse_edge.carrier);
            assert_eq!(forward_edge.pcurves[0], reverse_edge.pcurves[1]);
            assert_eq!(forward_edge.pcurves[1], reverse_edge.pcurves[0]);
        }

        let persistent =
            persist_verified_graph_surface_intersections(&mut graph, &forward).unwrap();
        assert_eq!(persistent.edges.len(), forward.branch_graph.edges.len());
        for edge in persistent.edges {
            let descriptor = graph.curve(edge.curve).unwrap().as_intersection().unwrap();
            assert_eq!(descriptor.source_surfaces(), [plane_handle, sphere_handle]);
            assert!(descriptor.certificate().as_plane_sphere_circle().is_some());
        }
        graph.validate().unwrap();
    }
}

#[test]
fn oblique_shifted_seam_branches_are_swap_stable_persistent_and_stale_safe() {
    let plane = oblique_plane(0.5);
    let sphere = Sphere::new(Frame::world(), 2.5).unwrap();
    let sphere_ranges = [
        ParamRange::new(5.5, 7.0),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ];
    let tolerances = Tolerances::default();
    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let sphere_handle = graph.insert_surface(sphere).unwrap();
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        wide_window(),
        sphere_handle,
        sphere_ranges,
        tolerances,
    )
    .unwrap();
    let raw =
        intersect_bounded_plane_sphere(&plane, wide_window(), &sphere, sphere_ranges, tolerances)
            .unwrap();
    assert_eq!(forward.raw, raw);
    assert!(!forward.branch_graph.edges.is_empty());
    assert_eq!(forward.branch_graph.edges.len(), forward.raw.curves.len());
    for edge in &forward.branch_graph.edges {
        assert_oblique_plane_sphere_edge_lifts_over_complete_range(edge, plane, sphere, true);
        for parameter in [edge.carrier_range.lo, edge.carrier_range.hi] {
            let uv = edge.pcurves[1].as_curve().eval(parameter);
            assert!(sphere_ranges[0].contains(uv.x));
            assert!(sphere_ranges[1].contains(uv.y));
        }
    }

    let reverse = intersect_bounded_graph_surfaces(
        &graph,
        sphere_handle,
        sphere_ranges,
        plane_handle,
        wide_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(reverse.raw, forward.raw.clone().swapped());
    assert_eq!(
        reverse.branch_graph.edges.len(),
        forward.branch_graph.edges.len()
    );
    for reverse_edge in &reverse.branch_graph.edges {
        assert_oblique_plane_sphere_edge_lifts_over_complete_range(
            reverse_edge,
            plane,
            sphere,
            false,
        );
        let forward_edge = forward
            .branch_graph
            .edges
            .iter()
            .find(|edge| edge.carrier_range == reverse_edge.carrier_range)
            .unwrap();
        assert_eq!(forward_edge.carrier, reverse_edge.carrier);
        assert_eq!(forward_edge.pcurves[0], reverse_edge.pcurves[1]);
        assert_eq!(forward_edge.pcurves[1], reverse_edge.pcurves[0]);
    }

    let persistent = persist_verified_graph_surface_intersections(&mut graph, &forward).unwrap();
    assert_eq!(persistent.edges.len(), forward.branch_graph.edges.len());
    for (local, persisted) in forward.branch_graph.edges.iter().zip(&persistent.edges) {
        let descriptor = graph
            .curve(persisted.curve)
            .unwrap()
            .as_intersection()
            .unwrap();
        assert_eq!(descriptor.source_surfaces(), [plane_handle, sphere_handle]);
        assert!(
            graph
                .curve2d(persisted.pcurves[1])
                .unwrap()
                .descriptor()
                .as_spherical_circle()
                .is_some()
        );
        let mut evaluator = EvalContext::new(&graph, EvalLimits::default(), tolerances);
        let bounds = evaluator
            .curve_bounds(persisted.curve, local.carrier_range)
            .unwrap();
        for parameter in [
            local.carrier_range.lo,
            local.carrier_range.lerp(0.5),
            local.carrier_range.hi,
        ] {
            assert!(
                bounds.contains(
                    evaluator
                        .eval_curve(persisted.curve, parameter, 3)
                        .unwrap()
                        .d[0]
                )
            );
        }
    }
    graph.validate().unwrap();

    let mut stale_graph = GeometryGraph::new();
    let stale_plane = stale_graph.insert_surface(plane).unwrap();
    let stale_sphere = stale_graph.insert_surface(sphere).unwrap();
    let stale = intersect_bounded_graph_surfaces(
        &stale_graph,
        stale_plane,
        wide_window(),
        stale_sphere,
        sphere_ranges,
        tolerances,
    )
    .unwrap();
    stale_graph
        .replace_surface(stale_sphere, Sphere::new(Frame::world(), 2.75).unwrap())
        .unwrap();
    let before = (
        stale_graph.curve_count(),
        stale_graph.curve2d_count(),
        stale_graph.geometry().collect::<Vec<_>>(),
    );
    assert!(matches!(
        persist_verified_graph_surface_intersections(&mut stale_graph, &stale),
        Err(GraphSurfaceIntersectionError::GeometryPersistence(
            GeometryGraphError::InvalidDescriptor { .. }
        ))
    ));
    assert_eq!(stale_graph.curve_count(), before.0);
    assert_eq!(stale_graph.curve2d_count(), before.1);
    assert_eq!(stale_graph.geometry().collect::<Vec<_>>(), before.2);
    stale_graph.validate().unwrap();
}

#[test]
fn oblique_safe_offsets_charge_exact_whole_branch_proof_work() {
    let tolerances = Tolerances::default();
    let effective_plane = oblique_plane(0.5);
    let effective_sphere = Sphere::new(Frame::world(), 2.5).unwrap();
    let sphere_ranges = [
        ParamRange::new(0.2, 2.8),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ];
    let mut graph = GeometryGraph::new();
    let plane_basis = graph.insert_surface(oblique_plane(0.0)).unwrap();
    let plane_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(plane_basis, 0.5))
        .unwrap();
    let sphere_basis = graph
        .insert_surface(Sphere::new(Frame::world(), 2.0).unwrap())
        .unwrap();
    let sphere_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(sphere_basis, 0.5))
        .unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        plane_offset,
        wide_window(),
        sphere_offset,
        sphere_ranges,
        &context,
    );
    let result = outcome.result().unwrap();
    assert_eq!(
        result.raw,
        intersect_bounded_plane_sphere(
            &effective_plane,
            wide_window(),
            &effective_sphere,
            sphere_ranges,
            tolerances,
        )
        .unwrap()
    );
    assert_eq!(
        result.branch_graph.source_surfaces,
        [plane_offset, sphere_offset]
    );
    assert!(!result.branch_graph.edges.is_empty());
    for edge in &result.branch_graph.edges {
        assert_oblique_plane_sphere_edge_lifts_over_complete_range(
            edge,
            effective_plane,
            effective_sphere,
            true,
        );
    }
    let exact_work =
        result.branch_graph.edges.len() as u64 * kgraph::SPHERICAL_CIRCLE_PROOF_SEGMENTS as u64;
    let proof_usage = outcome
        .report()
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS)
        .unwrap();
    assert_eq!(proof_usage.consumed, exact_work);

    let exact_plan = BudgetPlan::new([LimitSpec::new(
        SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        exact_work,
    )])
    .unwrap();
    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(exact_plan);
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        plane_offset,
        wide_window(),
        sphere_offset,
        sphere_ranges,
        &exact_context,
    );
    assert_eq!(exact.result().unwrap().raw, result.raw);
    assert_eq!(
        exact
            .report()
            .usage()
            .iter()
            .find(|snapshot| snapshot.stage == SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS)
            .unwrap()
            .consumed,
        exact_work
    );

    let lower_plan = BudgetPlan::new([LimitSpec::new(
        SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        exact_work - 1,
    )])
    .unwrap();
    let lower_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(lower_plan);
    let lower = intersect_bounded_graph_surfaces_with_context(
        &graph,
        plane_offset,
        wide_window(),
        sphere_offset,
        sphere_ranges,
        &lower_context,
    );
    let crossing = lower.result().unwrap_err().limit().unwrap();
    assert_eq!(crossing.stage, SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS);
    assert_eq!(
        (crossing.consumed, crossing.allowed),
        (exact_work, exact_work - 1)
    );
}

#[test]
fn oblique_pole_crossing_fails_typed_while_tangent_and_miss_preserve_raw_parity() {
    let tolerances = Tolerances::default();
    let sphere = Sphere::new(Frame::world(), 2.5).unwrap();
    let sphere_ranges = [
        ParamRange::new(-core::f64::consts::PI, core::f64::consts::PI),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ];
    let pole_plane = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
    );
    let pole_raw = intersect_bounded_plane_sphere(
        &pole_plane,
        wide_window(),
        &sphere,
        sphere_ranges,
        tolerances,
    )
    .unwrap();
    assert!(!pole_raw.curves.is_empty());
    let mut graph = GeometryGraph::new();
    let sphere_handle = graph.insert_surface(sphere).unwrap();
    let pole_handle = graph.insert_surface(pole_plane).unwrap();
    let pole_error = intersect_bounded_graph_surfaces(
        &graph,
        pole_handle,
        wide_window(),
        sphere_handle,
        sphere_ranges,
        tolerances,
    )
    .unwrap_err();
    assert_eq!(pole_error.class(), ErrorClass::Unsupported);
    assert_eq!(pole_error.code(), BRANCH_CERTIFICATE_FAILURE);
    assert_eq!(
        pole_error.capability(),
        Some(kgraph::intersection_certificate_capability::REGULAR_SPHERE_CHART)
    );
    assert_eq!(pole_error.limit(), None);
    let pole_source = std::error::Error::source(&pole_error)
        .unwrap()
        .downcast_ref::<IntersectionCertificateError>()
        .unwrap();
    assert!(matches!(
        pole_source,
        IntersectionCertificateError::SingularSphereChart { .. }
    ));
    assert_eq!(
        pole_source.code(),
        kgraph::intersection_certificate_error_code::SINGULAR_SPHERE_CHART
    );
    assert!(matches!(
        pole_error,
        GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::SingularSphereChart { .. }
        )
    ));

    let normal = Vec3::new(0.0, 0.6, 0.8);
    let tangent_plane =
        Plane::new(Frame::new(normal * sphere.radius(), normal, Vec3::new(1.0, 0.0, 0.0)).unwrap());
    let tangent_handle = graph.insert_surface(tangent_plane).unwrap();
    let tangent = intersect_bounded_graph_surfaces(
        &graph,
        tangent_handle,
        window(),
        sphere_handle,
        sphere_ranges,
        tolerances,
    )
    .unwrap();
    assert_eq!(
        tangent.raw,
        intersect_bounded_plane_sphere(
            &tangent_plane,
            window(),
            &sphere,
            sphere_ranges,
            tolerances,
        )
        .unwrap()
    );
    assert_eq!(tangent.raw.points.len(), 1);
    assert_eq!(tangent.raw.points[0].kind, ContactKind::Tangent);
    assert!(tangent.branch_graph.edges.is_empty());

    let miss_plane = Plane::new(
        Frame::new(
            normal * (2.0 * sphere.radius()),
            normal,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let miss_handle = graph.insert_surface(miss_plane).unwrap();
    let miss = intersect_bounded_graph_surfaces(
        &graph,
        miss_handle,
        window(),
        sphere_handle,
        sphere_ranges,
        tolerances,
    )
    .unwrap();
    assert_eq!(
        miss.raw,
        intersect_bounded_plane_sphere(&miss_plane, window(), &sphere, sphere_ranges, tolerances,)
            .unwrap()
    );
    assert!(miss.raw.is_proven_empty());
    assert!(miss.branch_graph.vertices.is_empty());
    assert!(miss.branch_graph.edges.is_empty());
}

#[test]
fn harmonic_certificate_failure_preserves_owner_metadata_and_source() {
    let error = GraphSurfaceIntersectionError::BranchCertificate(
        IntersectionCertificateError::HarmonicRootClassification,
    );
    assert_eq!(error.class(), ErrorClass::Unsupported);
    assert_eq!(error.code(), BRANCH_CERTIFICATE_FAILURE);
    assert_eq!(
        error.capability(),
        Some(kgraph::intersection_certificate_capability::HARMONIC_ROOT_CLASSIFICATION)
    );
    assert_eq!(error.limit(), None);
    let source = std::error::Error::source(&error)
        .unwrap()
        .downcast_ref::<IntersectionCertificateError>()
        .unwrap();
    assert_eq!(
        source,
        &IntersectionCertificateError::HarmonicRootClassification
    );
    assert_eq!(
        source.code(),
        kgraph::intersection_certificate_error_code::HARMONIC_ROOT_CLASSIFICATION
    );
}

#[test]
fn branch_certificate_adapter_preserves_legacy_classes_and_exact_leaf_sources() {
    use kgraph::intersection_certificate_capability as capability;

    let cases = vec![
        (
            IntersectionCertificateError::InvalidParameterMap { reason: "test" },
            ErrorClass::InternalInvariant,
            None,
        ),
        (
            IntersectionCertificateError::InvalidTraceFamily,
            ErrorClass::InternalInvariant,
            None,
        ),
        (
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: PairedTrace::First,
                reason: "test",
            },
            ErrorClass::InternalInvariant,
            None,
        ),
        (
            IntersectionCertificateError::UnsupportedCarrierParameterization { reason: "test" },
            ErrorClass::InternalInvariant,
            None,
        ),
        (
            IntersectionCertificateError::SingularSphereChart {
                squared_pole_clearance: 0.0,
            },
            ErrorClass::Unsupported,
            Some(capability::REGULAR_SPHERE_CHART),
        ),
        (
            IntersectionCertificateError::SphereTraceOutsideWindow {
                coordinate: "longitude",
            },
            ErrorClass::Unsupported,
            Some(capability::SPHERE_CHART_WINDOW),
        ),
        (
            IntersectionCertificateError::InvalidCarrierRange,
            ErrorClass::InternalInvariant,
            None,
        ),
        (
            IntersectionCertificateError::InvalidTolerance,
            ErrorClass::InternalInvariant,
            None,
        ),
        (
            IntersectionCertificateError::NonFiniteGeometry,
            ErrorClass::InternalInvariant,
            None,
        ),
        (
            IntersectionCertificateError::HarmonicRootClassification,
            ErrorClass::Unsupported,
            Some(capability::HARMONIC_ROOT_CLASSIFICATION),
        ),
        (
            IntersectionCertificateError::NonFiniteResidualBound {
                trace: PairedTrace::Second,
            },
            ErrorClass::InternalInvariant,
            None,
        ),
        (
            IntersectionCertificateError::SingularOffsetNormal {
                trace: PairedTrace::First,
                squared_norm_lower_bound: 0.0,
            },
            ErrorClass::InternalInvariant,
            None,
        ),
        (
            IntersectionCertificateError::ResidualExceedsTolerance {
                trace: PairedTrace::Second,
                residual_bound: 2.0,
                tolerance: 1.0,
            },
            ErrorClass::InternalInvariant,
            None,
        ),
    ];

    for (source, class, expected_capability) in cases {
        let expected_leaf_code = source.code();
        let error = GraphSurfaceIntersectionError::BranchCertificate(source.clone());
        assert_eq!(error.class(), class);
        assert_eq!(error.code(), BRANCH_CERTIFICATE_FAILURE);
        assert_eq!(error.capability(), expected_capability);
        assert_eq!(error.limit(), None);
        let retained = std::error::Error::source(&error)
            .unwrap()
            .downcast_ref::<IntersectionCertificateError>()
            .unwrap();
        assert_eq!(retained, &source);
        assert_eq!(retained.code(), expected_leaf_code);
    }
}

#[test]
fn plane_sphere_swap_persistence_and_stale_source_rollback_preserve_provenance() {
    let plane = horizontal_plane(0.5);
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let sphere_handle = graph.insert_surface(sphere).unwrap();
    let tolerances = Tolerances::default();
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        window(),
        sphere_handle,
        sphere_window(),
        tolerances,
    )
    .unwrap();
    let reverse = intersect_bounded_graph_surfaces(
        &graph,
        sphere_handle,
        sphere_window(),
        plane_handle,
        window(),
        tolerances,
    )
    .unwrap();

    assert_eq!(reverse.raw, forward.raw.clone().swapped());
    let forward_edge = &forward.branch_graph.edges[0];
    let reverse_edge = &reverse.branch_graph.edges[0];
    assert_eq!(forward_edge.source_surfaces, [plane_handle, sphere_handle]);
    assert_eq!(reverse_edge.source_surfaces, [sphere_handle, plane_handle]);
    assert_eq!(forward_edge.carrier, reverse_edge.carrier);
    assert_eq!(forward_edge.carrier_range, reverse_edge.carrier_range);
    assert_eq!(forward_edge.pcurves[0], reverse_edge.pcurves[1]);
    assert_eq!(forward_edge.pcurves[1], reverse_edge.pcurves[0]);
    assert_plane_sphere_edge_lifts_over_complete_range(forward_edge, plane, sphere, true);
    assert_plane_sphere_edge_lifts_over_complete_range(reverse_edge, plane, sphere, false);

    let before = (graph.curve_count(), graph.curve2d_count());
    let persistent = persist_verified_graph_surface_intersections(&mut graph, &forward).unwrap();
    assert_eq!(graph.curve_count(), before.0 + 1);
    assert_eq!(graph.curve2d_count(), before.1 + 2);
    let persistent_edge = persistent.edges[0];
    let descriptor = graph
        .curve(persistent_edge.curve)
        .unwrap()
        .as_intersection()
        .copied()
        .unwrap();
    assert_eq!(descriptor.source_surfaces(), [plane_handle, sphere_handle]);
    assert_eq!(descriptor.pcurves(), persistent_edge.pcurves);
    assert!(descriptor.carrier().as_circle().is_some());
    assert!(descriptor.certificate().as_plane_sphere_circle().is_some());
    assert_eq!(
        graph
            .direct_dependencies(GeometryRef::Curve(persistent_edge.curve))
            .unwrap(),
        vec![
            GeometryRef::Surface(plane_handle),
            GeometryRef::Surface(sphere_handle),
            GeometryRef::Curve2d(persistent_edge.pcurves[0]),
            GeometryRef::Curve2d(persistent_edge.pcurves[1]),
        ]
    );
    graph.validate().unwrap();

    let mut stale_graph = GeometryGraph::new();
    let stale_plane = stale_graph.insert_surface(plane).unwrap();
    let stale_sphere = stale_graph.insert_surface(sphere).unwrap();
    let stale_result = intersect_bounded_graph_surfaces(
        &stale_graph,
        stale_plane,
        window(),
        stale_sphere,
        sphere_window(),
        tolerances,
    )
    .unwrap();
    stale_graph
        .replace_surface(stale_sphere, Sphere::new(Frame::world(), 1.25).unwrap())
        .unwrap();
    let counts = (
        stale_graph.curve_count(),
        stale_graph.curve2d_count(),
        stale_graph.geometry().collect::<Vec<_>>(),
    );
    let error =
        persist_verified_graph_surface_intersections(&mut stale_graph, &stale_result).unwrap_err();
    assert!(matches!(
        error,
        GraphSurfaceIntersectionError::GeometryPersistence(
            GeometryGraphError::InvalidDescriptor { class, .. }
        ) if class == CurveClass::Intersection.key()
    ));
    assert_eq!(stale_graph.curve_count(), counts.0);
    assert_eq!(stale_graph.curve2d_count(), counts.1);
    assert_eq!(stale_graph.geometry().collect::<Vec<_>>(), counts.2);
    stale_graph.validate().unwrap();
}

#[test]
fn exact_plane_sphere_offsets_account_limits_and_fail_closed_outside_the_chart_boundary() {
    let tolerances = Tolerances::default();
    let effective_plane = horizontal_plane(0.5);
    let effective_sphere = Sphere::new(Frame::world(), 2.5).unwrap();
    let mut graph = GeometryGraph::new();
    let plane_basis = graph.insert_surface(horizontal_plane(0.0)).unwrap();
    let plane_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(plane_basis, 0.5))
        .unwrap();
    let sphere_basis = graph
        .insert_surface(Sphere::new(Frame::world(), 2.0).unwrap())
        .unwrap();
    let sphere_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(sphere_basis, 0.5))
        .unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        plane_offset,
        wide_window(),
        sphere_offset,
        sphere_window(),
        &context,
    );
    let result = outcome.result().unwrap();
    assert_eq!(
        result.raw,
        intersect_bounded_plane_sphere(
            &effective_plane,
            wide_window(),
            &effective_sphere,
            sphere_window(),
            tolerances,
        )
        .unwrap()
    );
    assert_eq!(
        result.branch_graph.source_surfaces,
        [plane_offset, sphere_offset]
    );
    assert_plane_sphere_edge_lifts_over_complete_range(
        &result.branch_graph.edges[0],
        effective_plane,
        effective_sphere,
        true,
    );
    let visits = outcome
        .report()
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == kgraph::eval_stage::NODE_VISITS)
        .unwrap();
    let depth = outcome
        .report()
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == kgraph::eval_stage::DEPENDENCY_DEPTH)
        .unwrap();
    assert_eq!((visits.resource, visits.consumed), (ResourceKind::Work, 4));
    assert_eq!((depth.resource, depth.consumed), (ResourceKind::Depth, 2));

    let limited = BudgetPlan::new([LimitSpec::new(
        kgraph::eval_stage::NODE_VISITS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        3,
    )])
    .unwrap();
    let limited_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(limited);
    let limited = intersect_bounded_graph_surfaces_with_context(
        &graph,
        plane_offset,
        wide_window(),
        sphere_offset,
        sphere_window(),
        &limited_context,
    );
    let crossing = limited.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(crossing.stage, kgraph::eval_stage::NODE_VISITS);
    assert_eq!((crossing.consumed, crossing.allowed), (4, 3));

    let invalid_sphere = graph
        .insert_surface(OffsetSurfaceDescriptor::new(sphere_basis, -2.0))
        .unwrap();
    let invalid = intersect_bounded_graph_surfaces(
        &graph,
        plane_offset,
        wide_window(),
        invalid_sphere,
        sphere_window(),
        tolerances,
    )
    .unwrap_err();
    assert_eq!(invalid.class(), ErrorClass::Unsupported);
    assert!(matches!(
        invalid,
        GraphSurfaceIntersectionError::Intersection(
            IntersectionError::UnsupportedSurfacePair {
                class_a: Some(class_a),
                class_b: Some(class_b),
            }
        ) if class_a == SurfaceClass::Offset.key() && class_b == SurfaceClass::Offset.key()
    ));

    let oblique_plane = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let oblique_handle = graph.insert_surface(oblique_plane).unwrap();
    let direct_sphere = graph.insert_surface(effective_sphere).unwrap();
    let oblique_chart = intersect_bounded_graph_surfaces(
        &graph,
        oblique_handle,
        wide_window(),
        direct_sphere,
        sphere_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(
        oblique_chart.raw,
        intersect_bounded_plane_sphere(
            &oblique_plane,
            wide_window(),
            &effective_sphere,
            sphere_window(),
            tolerances,
        )
        .unwrap()
    );
    assert_eq!(
        oblique_chart.branch_graph.edges.len(),
        oblique_chart.raw.curves.len()
    );
    assert!(!oblique_chart.branch_graph.edges.is_empty());
    for edge in &oblique_chart.branch_graph.edges {
        assert_oblique_plane_sphere_edge_lifts_over_complete_range(
            edge,
            oblique_plane,
            effective_sphere,
            true,
        );
    }
}

#[test]
fn stale_nurbs_and_nonplane_offset_sources_fail_closed_without_complete_misses() {
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
    let nurbs = graph.insert_surface(bilinear_nurbs_surface(0.0)).unwrap();
    let unsupported_nurbs = intersect_bounded_graph_surfaces(
        &graph,
        plane,
        window(),
        nurbs,
        window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(unsupported_nurbs.class(), ErrorClass::Unsupported);
    assert_eq!(
        unsupported_nurbs.capability(),
        Some(SURFACE_SURFACE_CLASS_PAIR)
    );
    assert!(matches!(
        unsupported_nurbs,
        GraphSurfaceIntersectionError::Intersection(
            IntersectionError::UnsupportedSurfacePair {
                class_a: Some(class_a),
                class_b: Some(class_b),
            }
        ) if class_a == SurfaceClass::Plane.key() && class_b == SurfaceClass::Nurbs.key()
    ));

    let offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(nurbs, 0.5))
        .unwrap();
    let unsupported_offset = intersect_bounded_graph_surfaces(
        &graph,
        plane,
        window(),
        offset,
        window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(unsupported_offset.class(), ErrorClass::Unsupported);
    assert_eq!(
        unsupported_offset.capability(),
        Some(SURFACE_SURFACE_CLASS_PAIR)
    );
    assert!(matches!(
        unsupported_offset,
        GraphSurfaceIntersectionError::Intersection(
            IntersectionError::UnsupportedSurfacePair {
                class_a: Some(class_a),
                class_b: Some(class_b),
            }
        ) if class_a == SurfaceClass::Plane.key() && class_b == SurfaceClass::Offset.key()
    ));
}
