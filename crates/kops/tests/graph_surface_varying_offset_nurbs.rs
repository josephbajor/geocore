//! Proof-backed varying-normal Offset(NURBS)/direct-NURBS graph contracts.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use kgraph::{
    GeometryGraph, GeometryGraphError, OffsetSurfaceDescriptor,
    verified_offset_nurbs_nurbs_intersection_certificate_cost,
};
use kops::intersect::{
    GraphSurfaceIntersectionError, IntersectionError, NURBS_TRACE_CERTIFICATE_WORK,
    intersect_bounded_graph_surfaces, intersect_bounded_graph_surfaces_with_context,
    intersect_bounded_offset_nurbs_nurbs_surfaces, persist_verified_graph_surface_intersections,
};

const SIGNED_DISTANCE: f64 = 0.1;
const NORMAL_WINDOW_PROOF_WORK: u64 = 7;

fn rational_quarter_cylinder(radius: f64) -> NurbsSurface {
    NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(radius, 0.0, 0.0),
            Point3::new(radius, 0.0, 1.0),
            Point3::new(radius, radius, 0.0),
            Point3::new(radius, radius, 1.0),
            Point3::new(0.0, radius, 0.0),
            Point3::new(0.0, radius, 1.0),
        ],
        Some(vec![1.0, 1.0, 1.0, 1.0, 2.0, 2.0]),
    )
    .unwrap()
}

fn direct_x_planar_nurbs(plane_x: f64) -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(plane_x, 0.0, 0.0),
            Point3::new(plane_x, 0.0, 1.0),
            Point3::new(plane_x, 1.2, 0.0),
            Point3::new(plane_x, 1.2, 1.0),
        ],
        None,
    )
    .unwrap()
}

fn direct_y_planar_nurbs(plane_y: f64) -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, plane_y, 0.0),
            Point3::new(0.0, plane_y, 1.0),
            Point3::new(1.2, plane_y, 0.0),
            Point3::new(1.2, plane_y, 1.0),
        ],
        None,
    )
    .unwrap()
}

fn offset_window() -> [ParamRange; 2] {
    [ParamRange::new(0.2, 0.8), ParamRange::new(0.1, 0.9)]
}

fn direct_window() -> [ParamRange; 2] {
    [ParamRange::new(0.1, 0.9), ParamRange::new(0.1, 0.9)]
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
        .map_or(0, |snapshot| snapshot.consumed)
}

#[test]
fn varying_normal_offset_promotes_in_both_orders_with_exact_limits_and_persists() {
    let basis = rational_quarter_cylinder(1.0);
    // At source parameter u=0.5, the exact rational cylinder point is
    // (0.6, 0.8, v); its 0.1 outward offset lies on x=0.66.
    let direct = direct_x_planar_nurbs(0.66);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let lower = intersect_bounded_offset_nurbs_nurbs_surfaces(
        &basis,
        SIGNED_DISTANCE,
        offset_window(),
        &direct,
        direct_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(lower.curves.len(), 1);

    let mut graph = GeometryGraph::new();
    let basis_handle = graph.insert_surface(basis.clone()).unwrap();
    let offset_handle = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_handle, SIGNED_DISTANCE))
        .unwrap();
    let direct_handle = graph.insert_surface(direct.clone()).unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        offset_handle,
        offset_window(),
        direct_handle,
        direct_window(),
        &context,
    );
    let result = outcome.result().unwrap();
    assert_eq!(result.raw, lower);
    assert_eq!(result.branch_graph.edges.len(), 1);
    assert_eq!(
        observed(
            outcome.report(),
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
        ),
        2
    );
    assert_eq!(
        observed(
            outcome.report(),
            kgraph::eval_stage::DEPENDENCY_DEPTH,
            ResourceKind::Depth,
        ),
        2
    );

    let certificate = result.branch_graph.edges[0].certificate.as_nurbs().unwrap();
    let offset_trace = certificate.traces()[0].as_offset_nurbs().unwrap();
    assert_eq!(offset_trace.basis(), &basis);
    assert_eq!(offset_trace.signed_distance(), SIGNED_DISTANCE);
    assert_eq!(certificate.traces()[1].as_nurbs(), Some(&direct));
    let cost = verified_offset_nurbs_nurbs_intersection_certificate_cost(
        certificate.carrier(),
        certificate.traces(),
    )
    .unwrap();
    assert_eq!(
        (cost.work(), cost.items(), cost.depth()),
        (14_336, 1_024, 10)
    );
    let total_work = NORMAL_WINDOW_PROOF_WORK + cost.work();
    assert_eq!(
        observed(
            outcome.report(),
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        total_work
    );
    assert_eq!(
        observed(
            outcome.report(),
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Items,
        ),
        cost.items()
    );
    assert_eq!(
        observed(
            outcome.report(),
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Depth,
        ),
        cost.depth()
    );

    let reverse = intersect_bounded_graph_surfaces(
        &graph,
        direct_handle,
        direct_window(),
        offset_handle,
        offset_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(reverse.raw, result.raw.clone().swapped());
    let reverse_certificate = reverse.branch_graph.edges[0]
        .certificate
        .as_nurbs()
        .unwrap();
    assert_eq!(reverse_certificate.traces()[0].as_nurbs(), Some(&direct));
    assert_eq!(
        reverse_certificate.traces()[1].as_offset_nurbs(),
        Some(offset_trace)
    );

    let exact_plan = BudgetPlan::new([
        LimitSpec::new(
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            2,
        ),
        LimitSpec::new(
            kgraph::eval_stage::DEPENDENCY_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            2,
        ),
        LimitSpec::new(
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            total_work,
        ),
        LimitSpec::new(
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Items,
            AccountingMode::HighWater,
            cost.items(),
        ),
        LimitSpec::new(
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            cost.depth(),
        ),
    ])
    .unwrap();
    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(exact_plan);
    assert!(
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            offset_handle,
            offset_window(),
            direct_handle,
            direct_window(),
            &exact_context,
        )
        .result()
        .is_ok()
    );

    for (stage, resource, mode, allowed, consumed) in [
        (
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            1,
            2,
        ),
        (
            kgraph::eval_stage::DEPENDENCY_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            1,
            2,
        ),
        (
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            total_work - 1,
            total_work,
        ),
        (
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Items,
            AccountingMode::HighWater,
            cost.items() - 1,
            cost.items(),
        ),
        (
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            cost.depth() - 1,
            cost.depth(),
        ),
    ] {
        let denied_plan =
            BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap();
        let denied_context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(denied_plan);
        let denied = intersect_bounded_graph_surfaces_with_context(
            &graph,
            offset_handle,
            offset_window(),
            direct_handle,
            direct_window(),
            &denied_context,
        );
        let GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(crossing),
        ) = denied.result().unwrap_err()
        else {
            panic!("N-1 varying-normal resource must stop at {stage:?}");
        };
        assert_eq!(crossing.stage, stage);
        assert_eq!(crossing.resource, resource);
        assert_eq!(crossing.allowed, allowed);
        assert_eq!(crossing.consumed, consumed);
    }

    let persistent = persist_verified_graph_surface_intersections(&mut graph, result).unwrap();
    assert_eq!(persistent.edges.len(), 1);
    for protected in [basis_handle, offset_handle, direct_handle] {
        assert!(matches!(
            graph.remove_surface(protected),
            Err(GeometryGraphError::HasDependents { .. })
        ));
    }
    graph.validate().unwrap();
}

#[test]
fn varying_normal_y_plane_promotes_swaps_persists_and_pins_exact_limits() {
    let basis = rational_quarter_cylinder(1.0);
    // At source parameter u=0.5, the exact point (0.6, 0.8, v) has outward
    // offset (0.66, 0.88, v), so the second canonical orientation is y=0.88.
    let direct = direct_y_planar_nurbs(0.88);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let lower = intersect_bounded_offset_nurbs_nurbs_surfaces(
        &basis,
        SIGNED_DISTANCE,
        offset_window(),
        &direct,
        direct_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(lower.curves.len(), 1);

    let mut graph = GeometryGraph::new();
    let basis_handle = graph.insert_surface(basis.clone()).unwrap();
    let offset_handle = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_handle, SIGNED_DISTANCE))
        .unwrap();
    let direct_handle = graph.insert_surface(direct.clone()).unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        offset_handle,
        offset_window(),
        direct_handle,
        direct_window(),
        &context,
    );
    let result = outcome.result().unwrap();
    assert_eq!(result.raw, lower);
    assert_eq!(result.branch_graph.edges.len(), 1);
    assert_eq!(
        observed(
            outcome.report(),
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
        ),
        2
    );
    assert_eq!(
        observed(
            outcome.report(),
            kgraph::eval_stage::DEPENDENCY_DEPTH,
            ResourceKind::Depth,
        ),
        2
    );

    let certificate = result.branch_graph.edges[0].certificate.as_nurbs().unwrap();
    let offset_trace = certificate.traces()[0].as_offset_nurbs().unwrap();
    assert_eq!(offset_trace.basis(), &basis);
    assert_eq!(offset_trace.signed_distance(), SIGNED_DISTANCE);
    assert_eq!(certificate.traces()[1].as_nurbs(), Some(&direct));
    let cost = verified_offset_nurbs_nurbs_intersection_certificate_cost(
        certificate.carrier(),
        certificate.traces(),
    )
    .unwrap();
    assert_eq!(
        (cost.work(), cost.items(), cost.depth()),
        (14_336, 1_024, 10)
    );
    let total_work = NORMAL_WINDOW_PROOF_WORK + cost.work();
    assert_eq!(
        observed(
            outcome.report(),
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        total_work
    );
    assert_eq!(
        observed(
            outcome.report(),
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Items,
        ),
        cost.items()
    );
    assert_eq!(
        observed(
            outcome.report(),
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Depth,
        ),
        cost.depth()
    );

    let reverse = intersect_bounded_graph_surfaces(
        &graph,
        direct_handle,
        direct_window(),
        offset_handle,
        offset_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(reverse.raw, result.raw.clone().swapped());
    let reverse_certificate = reverse.branch_graph.edges[0]
        .certificate
        .as_nurbs()
        .unwrap();
    assert_eq!(reverse_certificate.traces()[0].as_nurbs(), Some(&direct));
    assert_eq!(
        reverse_certificate.traces()[1].as_offset_nurbs(),
        Some(offset_trace)
    );

    let exact_plan = BudgetPlan::new([
        LimitSpec::new(
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            2,
        ),
        LimitSpec::new(
            kgraph::eval_stage::DEPENDENCY_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            2,
        ),
        LimitSpec::new(
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            total_work,
        ),
        LimitSpec::new(
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Items,
            AccountingMode::HighWater,
            cost.items(),
        ),
        LimitSpec::new(
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            cost.depth(),
        ),
    ])
    .unwrap();
    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(exact_plan);
    assert!(
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            offset_handle,
            offset_window(),
            direct_handle,
            direct_window(),
            &exact_context,
        )
        .result()
        .is_ok()
    );

    for (stage, resource, mode, allowed, consumed) in [
        (
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            1,
            2,
        ),
        (
            kgraph::eval_stage::DEPENDENCY_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            1,
            2,
        ),
        (
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            total_work - 1,
            total_work,
        ),
        (
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Items,
            AccountingMode::HighWater,
            cost.items() - 1,
            cost.items(),
        ),
        (
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            cost.depth() - 1,
            cost.depth(),
        ),
    ] {
        let denied_plan =
            BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap();
        let denied_context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(denied_plan);
        let denied = intersect_bounded_graph_surfaces_with_context(
            &graph,
            offset_handle,
            offset_window(),
            direct_handle,
            direct_window(),
            &denied_context,
        );
        let GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(crossing),
        ) = denied.result().unwrap_err()
        else {
            panic!("N-1 y-oriented varying-normal resource must stop at {stage:?}");
        };
        assert_eq!(crossing.stage, stage);
        assert_eq!(crossing.resource, resource);
        assert_eq!(crossing.allowed, allowed);
        assert_eq!(crossing.consumed, consumed);
    }

    let persistent = persist_verified_graph_surface_intersections(&mut graph, result).unwrap();
    assert_eq!(persistent.edges.len(), 1);
    for protected in [basis_handle, offset_handle, direct_handle] {
        assert!(matches!(
            graph.remove_surface(protected),
            Err(GeometryGraphError::HasDependents { .. })
        ));
    }
    graph.validate().unwrap();
}

#[test]
fn varying_normal_original_control_miss_is_complete_and_allocates_nothing() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for direct in [direct_x_planar_nurbs(1.2), direct_y_planar_nurbs(1.2)] {
        let mut graph = GeometryGraph::new();
        let basis_handle = graph
            .insert_surface(rational_quarter_cylinder(1.0))
            .unwrap();
        let offset_handle = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis_handle, SIGNED_DISTANCE))
            .unwrap();
        let direct_handle = graph.insert_surface(direct).unwrap();
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, tolerances).unwrap();
        let outcome = intersect_bounded_graph_surfaces_with_context(
            &graph,
            offset_handle,
            offset_window(),
            direct_handle,
            direct_window(),
            &context,
        );
        let result = outcome.result().unwrap();
        assert!(result.raw.is_proven_empty());
        assert!(result.branch_graph.edges.is_empty());
        for (resource, consumed) in [
            (ResourceKind::Work, NORMAL_WINDOW_PROOF_WORK),
            (ResourceKind::Items, 1),
            (ResourceKind::Depth, 1),
        ] {
            assert_eq!(
                observed(outcome.report(), NURBS_TRACE_CERTIFICATE_WORK, resource),
                consumed
            );
        }

        let reverse = intersect_bounded_graph_surfaces_with_context(
            &graph,
            direct_handle,
            direct_window(),
            offset_handle,
            offset_window(),
            &context,
        );
        let reverse_result = reverse.result().unwrap();
        assert!(reverse_result.raw.is_proven_empty());
        assert!(reverse_result.branch_graph.edges.is_empty());
        for (resource, consumed) in [
            (ResourceKind::Work, NORMAL_WINDOW_PROOF_WORK),
            (ResourceKind::Items, 1),
            (ResourceKind::Depth, 1),
        ] {
            assert_eq!(
                observed(reverse.report(), NURBS_TRACE_CERTIFICATE_WORK, resource),
                consumed
            );
        }

        let before = graph.geometry().collect::<Vec<_>>();
        let persistent = persist_verified_graph_surface_intersections(&mut graph, result).unwrap();
        assert!(persistent.edges.is_empty());
        assert_eq!(graph.geometry().collect::<Vec<_>>(), before);
    }
}

#[test]
fn singular_nested_and_incompatible_varying_normal_sources_fail_typed() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let direct = direct_x_planar_nurbs(0.66);

    let mut singular = GeometryGraph::new();
    let singular_basis = singular
        .insert_surface(rational_quarter_cylinder(0.0))
        .unwrap();
    let singular_offset = singular
        .insert_surface(OffsetSurfaceDescriptor::new(
            singular_basis,
            SIGNED_DISTANCE,
        ))
        .unwrap();
    let singular_direct = singular.insert_surface(direct.clone()).unwrap();
    assert!(matches!(
        intersect_bounded_graph_surfaces(
            &singular,
            singular_offset,
            offset_window(),
            singular_direct,
            direct_window(),
            tolerances,
        ),
        Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::Kernel(kcore::error::Error::InvalidGeometry { .. })
        ))
    ));

    let mut graph = GeometryGraph::new();
    let basis = graph
        .insert_surface(rational_quarter_cylinder(1.0))
        .unwrap();
    let inner = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.04))
        .unwrap();
    let nested = graph
        .insert_surface(OffsetSurfaceDescriptor::new(inner, 0.06))
        .unwrap();
    let direct_handle = graph.insert_surface(direct.clone()).unwrap();
    for (a, b, a_range, b_range) in [
        (nested, direct_handle, offset_window(), direct_window()),
        (direct_handle, nested, direct_window(), offset_window()),
    ] {
        assert!(matches!(
            intersect_bounded_graph_surfaces(&graph, a, a_range, b, b_range, tolerances),
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::UnsupportedSurfacePair { .. }
            ))
        ));
    }

    let mut points = direct.points().to_vec();
    points[3].x += 0.01;
    let incompatible = NurbsSurface::new(
        direct.degree_u(),
        direct.degree_v(),
        direct.knots(kgeom::surface::Dir::U).as_slice().to_vec(),
        direct.knots(kgeom::surface::Dir::V).as_slice().to_vec(),
        points,
        direct.weights().map(<[f64]>::to_vec),
    )
    .unwrap();
    let incompatible = graph.insert_surface(incompatible).unwrap();
    assert!(matches!(
        intersect_bounded_graph_surfaces(
            &graph,
            inner,
            offset_window(),
            incompatible,
            direct_window(),
            tolerances,
        ),
        Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::UnsupportedSurfacePair { .. }
        ))
    ));

    let direct_y = direct_y_planar_nurbs(0.88);
    let mut points = direct_y.points().to_vec();
    points[3].y += 0.01;
    let incompatible_y = NurbsSurface::new(
        direct_y.degree_u(),
        direct_y.degree_v(),
        direct_y.knots(kgeom::surface::Dir::U).as_slice().to_vec(),
        direct_y.knots(kgeom::surface::Dir::V).as_slice().to_vec(),
        points,
        direct_y.weights().map(<[f64]>::to_vec),
    )
    .unwrap();
    let incompatible_y = graph.insert_surface(incompatible_y).unwrap();
    for (a, b, a_range, b_range) in [
        (inner, incompatible_y, offset_window(), direct_window()),
        (incompatible_y, inner, direct_window(), offset_window()),
    ] {
        assert!(matches!(
            intersect_bounded_graph_surfaces(&graph, a, a_range, b, b_range, tolerances),
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::UnsupportedSurfacePair { .. }
            ))
        ));
    }
}

#[test]
fn altered_or_nested_varying_normal_proof_sources_roll_back_atomically() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for mutation in 0..3 {
        let mut graph = GeometryGraph::new();
        let basis = graph
            .insert_surface(rational_quarter_cylinder(1.0))
            .unwrap();
        let root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, SIGNED_DISTANCE))
            .unwrap();
        let direct = graph.insert_surface(direct_x_planar_nurbs(0.66)).unwrap();
        let local = intersect_bounded_graph_surfaces(
            &graph,
            root,
            offset_window(),
            direct,
            direct_window(),
            tolerances,
        )
        .unwrap();
        match mutation {
            0 => {
                graph
                    .replace_surface(basis, rational_quarter_cylinder(1.01))
                    .unwrap();
            }
            1 => {
                graph
                    .replace_surface(direct, direct_x_planar_nurbs(0.67))
                    .unwrap();
            }
            2 => {
                let inner = graph
                    .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.04))
                    .unwrap();
                graph
                    .replace_surface(root, OffsetSurfaceDescriptor::new(inner, 0.06))
                    .unwrap();
            }
            _ => unreachable!(),
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
            ))
        ));
        assert_eq!(graph.curve_count(), before.0);
        assert_eq!(graph.curve2d_count(), before.1);
        assert_eq!(graph.geometry().collect::<Vec<_>>(), before.2);
        graph.validate().unwrap();
    }
}

#[test]
fn altered_or_stale_y_oriented_peer_rolls_persistence_back_atomically() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for mutation in 0..2 {
        let mut graph = GeometryGraph::new();
        let basis = graph
            .insert_surface(rational_quarter_cylinder(1.0))
            .unwrap();
        let root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, SIGNED_DISTANCE))
            .unwrap();
        let direct = graph.insert_surface(direct_y_planar_nurbs(0.88)).unwrap();
        let local = intersect_bounded_graph_surfaces(
            &graph,
            root,
            offset_window(),
            direct,
            direct_window(),
            tolerances,
        )
        .unwrap();
        match mutation {
            0 => {
                graph
                    .replace_surface(direct, direct_y_planar_nurbs(0.89))
                    .unwrap();
            }
            1 => {
                graph.remove_surface(direct).unwrap();
            }
            _ => unreachable!(),
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
