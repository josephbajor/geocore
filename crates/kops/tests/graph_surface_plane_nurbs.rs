//! Contextual graph-owned Plane/NURBS branch contracts.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsSurface;
use kgeom::nurbs::{
    NURBS_IMPLICIT_ISOLATION_CANDIDATES, NURBS_IMPLICIT_ISOLATION_DEPTH,
    NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
};
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Surface};
use kgeom::vec::Point3;
use kgraph::{
    Curve2dDescriptor, CurveDescriptor, GeometryGraph, GeometryGraphError,
    IntersectionCertificateError, OffsetSurfaceDescriptor,
    verified_plane_nurbs_intersection_certificate_work,
};
use kops::intersect::{
    GraphSurfaceIntersectionError, IntersectionError, NURBS_SURFACE_MARCH_SAMPLES,
    NURBS_TRACE_CERTIFICATE_WORK, NurbsSurfaceMarchBudgetProfile, SurfaceIntersectionCurve,
    intersect_bounded_graph_surfaces, intersect_bounded_graph_surfaces_with_context,
    intersect_bounded_plane_nurbs_surface, intersect_bounded_plane_nurbs_surface_with_context,
    persist_verified_graph_surface_intersections,
};

fn curved_surface() -> NurbsSurface {
    curved_surface_with(0.01, 0.0)
}

fn curved_surface_with(bend: f64, height: f64) -> NurbsSurface {
    let coordinates = [0.0, 0.5, 1.0];
    let bend_controls = [0.0, 0.5 * bend, 0.0];
    let mut points = Vec::with_capacity(9);
    for (u_index, &u) in coordinates.iter().enumerate() {
        for (v_index, &v) in coordinates.iter().enumerate() {
            points.push(Point3::new(
                u,
                v,
                height + coordinates[u_index] - 0.5 + bend_controls[v_index],
            ));
        }
    }
    NurbsSurface::new(
        2,
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points,
        None,
    )
    .unwrap()
}

fn usage(report: &kcore::operation::OperationReport, stage: StageId) -> u64 {
    report
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == stage && snapshot.resource == ResourceKind::Work)
        .unwrap()
        .consumed
}

fn observed(
    report: &kcore::operation::OperationReport,
    stage: StageId,
    resource: ResourceKind,
) -> u64 {
    report
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == stage && snapshot.resource == resource)
        .unwrap()
        .consumed
}

fn plane_window() -> [ParamRange; 2] {
    [ParamRange::new(-0.1, 1.1), ParamRange::new(-0.1, 1.1)]
}

fn single_march_segment_plane_window() -> [ParamRange; 2] {
    [ParamRange::new(-0.1, 1.1), ParamRange::new(0.0, 0.025)]
}

#[test]
fn curved_plane_nurbs_march_promotes_only_whole_range_certified_traces() {
    let plane = Plane::new(Frame::world());
    let surface = curved_surface();
    let tolerances = Tolerances::with_linear(1.0e-5).unwrap();
    let lower = intersect_bounded_plane_nurbs_surface(
        &plane,
        plane_window(),
        &surface,
        surface.param_range(),
        tolerances,
    )
    .unwrap();
    assert!(!lower.is_complete());
    assert_eq!(lower.curves.len(), 1);

    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let surface_handle = graph.insert_surface(surface.clone()).unwrap();
    let graph_result = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        plane_window(),
        surface_handle,
        surface.param_range(),
        tolerances,
    )
    .unwrap();
    assert_eq!(graph_result.raw, lower);
    assert!(!graph_result.raw.is_complete());
    assert_eq!(
        graph_result.branch_graph.source_surfaces,
        [plane_handle, surface_handle]
    );
    assert_eq!(graph_result.branch_graph.edges.len(), 1);

    let edge = &graph_result.branch_graph.edges[0];
    let certificate = edge.certificate.as_nurbs().unwrap();
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= tolerances.linear())
    );
    assert!(matches!(edge.carrier, CurveDescriptor::Nurbs(_)));
    assert!(
        edge.pcurves
            .iter()
            .all(|pcurve| matches!(pcurve, Curve2dDescriptor::Nurbs(_)))
    );
    let midpoint = edge.carrier_range.lerp(0.5);
    let plane_uv = edge.pcurves[0].as_curve().eval(midpoint);
    assert!(
        edge.carrier
            .as_curve()
            .eval(midpoint)
            .dist(plane.eval([plane_uv.x, plane_uv.y]))
            <= tolerances.linear()
    );

    let persistent =
        persist_verified_graph_surface_intersections(&mut graph, &graph_result).unwrap();
    assert_eq!(persistent.edges.len(), 1);
    let descriptor = graph.curve(persistent.edges[0].curve).unwrap();
    assert!(descriptor.as_verified_nurbs_intersection().is_some());
    graph.validate().unwrap();
}

#[test]
fn scoped_plane_nurbs_preserves_raw_march_report_and_exact_certificate_boundaries() {
    let plane = Plane::new(Frame::world());
    let surface = curved_surface();
    let plane_range = single_march_segment_plane_window();
    let tolerances = Tolerances::with_linear(1.0e-5).unwrap();
    let session = SessionPolicy::v1();
    let lower_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_family_budget_defaults(NurbsSurfaceMarchBudgetProfile::v1_defaults());
    let lower = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range,
        &surface,
        surface.param_range(),
        &lower_context,
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let surface_handle = graph.insert_surface(surface.clone()).unwrap();
    let graph_context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        plane_handle,
        plane_range,
        surface_handle,
        surface.param_range(),
        &graph_context,
    );
    let result = outcome.result().unwrap();
    assert_eq!(&result.raw, *lower.result().as_ref().unwrap());
    for (stage, resource) in [
        (NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS, ResourceKind::Work),
        (NURBS_IMPLICIT_ISOLATION_CANDIDATES, ResourceKind::Items),
        (NURBS_IMPLICIT_ISOLATION_DEPTH, ResourceKind::Depth),
        (NURBS_SURFACE_MARCH_SAMPLES, ResourceKind::Work),
    ] {
        assert_eq!(
            observed(outcome.report(), stage, resource),
            observed(lower.report(), stage, resource),
            "lower marcher report parity for {stage:?}"
        );
    }

    let certificate = result.branch_graph.edges[0].certificate.as_nurbs().unwrap();
    let exact_work = verified_plane_nurbs_intersection_certificate_work(
        certificate.carrier(),
        certificate.traces(),
    )
    .unwrap();
    assert_eq!(exact_work, 7_170);
    assert_eq!(
        usage(outcome.report(), NURBS_TRACE_CERTIFICATE_WORK),
        exact_work
    );

    let exact_plan = BudgetPlan::new([LimitSpec::new(
        NURBS_TRACE_CERTIFICATE_WORK,
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
        plane_handle,
        plane_range,
        surface_handle,
        surface.param_range(),
        &exact_context,
    );
    assert_eq!(exact.result().unwrap().raw, result.raw);
    assert_eq!(
        usage(exact.report(), NURBS_TRACE_CERTIFICATE_WORK),
        exact_work
    );

    let denied_plan = BudgetPlan::new([LimitSpec::new(
        NURBS_TRACE_CERTIFICATE_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        exact_work - 1,
    )])
    .unwrap();
    let denied_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(denied_plan);
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        plane_handle,
        plane_range,
        surface_handle,
        surface.param_range(),
        &denied_context,
    );
    let GraphSurfaceIntersectionError::OperationPolicy(
        kcore::operation::OperationPolicyError::LimitReached(crossing),
    ) = denied.result().unwrap_err()
    else {
        panic!("N-1 certificate work must stop at its exact stage");
    };
    assert_eq!(crossing.stage, NURBS_TRACE_CERTIFICATE_WORK);
    assert_eq!(crossing.allowed, exact_work - 1);
    assert_eq!(crossing.consumed, exact_work);
}

#[test]
fn failed_whole_range_residual_consumes_attempted_certificate_work() {
    let plane = Plane::new(Frame::world());
    let surface = curved_surface_with(0.1, 0.0);
    let tolerances = Tolerances::with_linear(1.0e-5).unwrap();
    let lower = intersect_bounded_plane_nurbs_surface(
        &plane,
        plane_window(),
        &surface,
        surface.param_range(),
        tolerances,
    )
    .unwrap();
    assert_eq!(lower.curves.len(), 1);
    let SurfaceIntersectionCurve::Nurbs(carrier) = &lower.curves[0].curve else {
        panic!("Plane/NURBS marcher must return a NURBS carrier");
    };
    let expected_work = u64::try_from(carrier.points().len()).unwrap()
        + u64::try_from(carrier.points().len() - 1).unwrap() * (1_u64 << 10) * 7;

    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let surface_handle = graph.insert_surface(surface.clone()).unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        plane_handle,
        plane_window(),
        surface_handle,
        surface.param_range(),
        &context,
    );
    assert!(matches!(
        outcome.result(),
        Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::ResidualExceedsTolerance { .. }
        ))
    ));
    assert_eq!(
        usage(outcome.report(), NURBS_TRACE_CERTIFICATE_WORK),
        expected_work
    );
}

#[test]
fn plane_nurbs_swap_and_complete_miss_preserve_lower_evidence() {
    let plane = Plane::new(Frame::world());
    let surface = curved_surface();
    let tolerances = Tolerances::with_linear(1.0e-5).unwrap();
    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let surface_handle = graph.insert_surface(surface.clone()).unwrap();
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        plane_window(),
        surface_handle,
        surface.param_range(),
        tolerances,
    )
    .unwrap();
    let reverse = intersect_bounded_graph_surfaces(
        &graph,
        surface_handle,
        surface.param_range(),
        plane_handle,
        plane_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(reverse.raw, forward.raw.clone().swapped());
    assert_eq!(
        reverse.branch_graph.source_surfaces,
        [surface_handle, plane_handle]
    );
    assert_eq!(
        reverse.branch_graph.edges.len(),
        forward.branch_graph.edges.len()
    );
    let reverse_certificate = reverse.branch_graph.edges[0]
        .certificate
        .as_nurbs()
        .unwrap();
    assert!(matches!(
        reverse_certificate.traces(),
        [
            kgraph::NurbsIntersectionTrace::Nurbs(_),
            kgraph::NurbsIntersectionTrace::Plane(_)
        ]
    ));

    let miss = curved_surface_with(0.01, 2.0);
    let miss_handle = graph.insert_surface(miss.clone()).unwrap();
    let graph_miss = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        plane_window(),
        miss_handle,
        miss.param_range(),
        tolerances,
    )
    .unwrap();
    let lower_miss = intersect_bounded_plane_nurbs_surface(
        &plane,
        plane_window(),
        &miss,
        miss.param_range(),
        tolerances,
    )
    .unwrap();
    assert_eq!(graph_miss.raw, lower_miss);
    assert!(graph_miss.raw.is_proven_empty());
    assert!(graph_miss.branch_graph.edges.is_empty());
}

#[test]
fn stale_and_altered_sources_roll_back_persistence_atomically() {
    fn local_result(
        graph: &GeometryGraph,
        plane: kgraph::SurfaceHandle,
        surface: kgraph::SurfaceHandle,
        source: &NurbsSurface,
        tolerances: Tolerances,
    ) -> kops::intersect::GraphSurfaceSurfaceIntersections {
        intersect_bounded_graph_surfaces(
            graph,
            plane,
            plane_window(),
            surface,
            source.param_range(),
            tolerances,
        )
        .unwrap()
    }

    let tolerances = Tolerances::with_linear(1.0e-5).unwrap();
    let plane = Plane::new(Frame::world());
    let source = curved_surface();
    for stale in [false, true] {
        let mut graph = GeometryGraph::new();
        let plane_handle = graph.insert_surface(plane).unwrap();
        let surface_handle = graph.insert_surface(source.clone()).unwrap();
        let local = local_result(&graph, plane_handle, surface_handle, &source, tolerances);
        if stale {
            graph.remove_surface(surface_handle).unwrap();
        } else {
            graph
                .replace_surface(surface_handle, curved_surface_with(0.02, 0.0))
                .unwrap();
        }
        let before = (
            graph.curve_count(),
            graph.curve2d_count(),
            graph.geometry().collect::<Vec<_>>(),
        );
        assert!(matches!(
            persist_verified_graph_surface_intersections(&mut graph, &local),
            Err(GraphSurfaceIntersectionError::GeometryPersistence(
                GeometryGraphError::InvalidDescriptor { .. }
                    | GeometryGraphError::StaleGeometryHandle { .. }
            ))
        ));
        assert_eq!(graph.curve_count(), before.0);
        assert_eq!(graph.curve2d_count(), before.1);
        assert_eq!(graph.geometry().collect::<Vec<_>>(), before.2);
        graph.validate().unwrap();
    }
}

#[test]
fn procedural_offsets_and_other_nurbs_pairs_remain_explicitly_unsupported() {
    let plane = Plane::new(Frame::world());
    let source = curved_surface();
    let tolerances = Tolerances::with_linear(1.0e-5).unwrap();
    let mut graph = GeometryGraph::new();
    let plane_basis = graph.insert_surface(plane).unwrap();
    let offset_plane = graph
        .insert_surface(OffsetSurfaceDescriptor::new(plane_basis, 0.1))
        .unwrap();
    let nurbs = graph.insert_surface(source.clone()).unwrap();
    let sphere = graph
        .insert_surface(kgeom::surface::Sphere::new(Frame::world(), 2.0).unwrap())
        .unwrap();
    for (first, first_range, second, second_range) in [
        (offset_plane, plane_window(), nurbs, source.param_range()),
        (sphere, plane_window(), nurbs, source.param_range()),
    ] {
        assert!(matches!(
            intersect_bounded_graph_surfaces(
                &graph,
                first,
                first_range,
                second,
                second_range,
                tolerances,
            ),
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::UnsupportedSurfacePair { .. }
            ))
        ));
    }
}
