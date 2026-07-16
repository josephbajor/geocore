//! Proof-backed varying-normal Offset(NURBS)/direct-Plane graph contracts.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::curve2d::NurbsCurve2d;
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    GeometryGraph, GeometryGraphError, IntersectionCertificateError, OffsetSurfaceDescriptor,
    certify_verified_offset_nurbs_plane_intersection_residuals,
    verified_offset_nurbs_plane_intersection_certificate_cost,
};
use kops::intersect::{
    GraphSurfaceIntersectionError, IntersectionError, NURBS_TRACE_CERTIFICATE_WORK,
    intersect_bounded_graph_surfaces, intersect_bounded_graph_surfaces_with_context,
    persist_verified_graph_surface_intersections,
};

const SIGNED_DISTANCE: f64 = 0.1;
const INNER_SIGNED_DISTANCE: f64 = 0.04;
const OUTER_SIGNED_DISTANCE: f64 = 0.06;
const NORMAL_WINDOW_PROOF_WORK: u64 = 7;

#[derive(Clone, Copy)]
struct AxisCase {
    plane: Plane,
    plane_range: [ParamRange; 2],
    carrier_controls: usize,
    certificate_work: u64,
    certificate_items: u64,
}

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

fn canonical_plane(normal_axis: usize, coordinate: f64) -> Plane {
    let (origin, normal, tangent) = match normal_axis {
        0 => (
            Point3::new(coordinate, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ),
        1 => (
            Point3::new(0.0, coordinate, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        2 => (
            Point3::new(0.0, 0.0, coordinate),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        _ => unreachable!(),
    };
    Plane::new(Frame::new(origin, normal, tangent).unwrap())
}

fn axis_cases() -> [AxisCase; 3] {
    [
        AxisCase {
            // u=0.5 maps to the exact offset point (0.66, 0.88, z).
            plane: canonical_plane(0, 0.66),
            plane_range: [ParamRange::new(0.1, 1.0), ParamRange::new(0.1, 0.9)],
            carrier_controls: 2,
            certificate_work: 7_170,
            certificate_items: 1_024,
        },
        AxisCase {
            plane: canonical_plane(1, 0.88),
            // The canonical +Y frame has local v=-z.
            plane_range: [ParamRange::new(0.1, 0.9), ParamRange::new(-0.9, -0.1)],
            carrier_controls: 2,
            certificate_work: 7_170,
            certificate_items: 1_024,
        },
        AxisCase {
            plane: canonical_plane(2, 0.5),
            plane_range: [ParamRange::new(0.0, 1.2), ParamRange::new(0.0, 1.2)],
            carrier_controls: 41,
            certificate_work: 286_761,
            certificate_items: 40_960,
        },
    ]
}

fn offset_window() -> [ParamRange; 2] {
    [ParamRange::new(0.2, 0.8), ParamRange::new(0.1, 0.9)]
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
fn global_axis_planes_promote_swap_and_pin_every_exact_resource() {
    let basis_surface = rational_quarter_cylinder(1.0);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();

    for case in axis_cases() {
        let mut graph = GeometryGraph::new();
        let basis = graph.insert_surface(basis_surface.clone()).unwrap();
        let root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, SIGNED_DISTANCE))
            .unwrap();
        let plane = graph.insert_surface(case.plane).unwrap();
        let context = OperationContext::new(&session, tolerances).unwrap();
        let outcome = intersect_bounded_graph_surfaces_with_context(
            &graph,
            root,
            offset_window(),
            plane,
            case.plane_range,
            &context,
        );
        let local = outcome.result().unwrap();
        assert_eq!(local.raw.curves.len(), 1);
        assert_eq!(local.branch_graph.edges.len(), 1);
        let certificate = local.branch_graph.edges[0].certificate.as_nurbs().unwrap();
        assert_eq!(certificate.carrier().points().len(), case.carrier_controls);
        let offset_trace = certificate.traces()[0].as_offset_nurbs().unwrap();
        assert_eq!(offset_trace.basis(), &basis_surface);
        assert_eq!(offset_trace.signed_distance(), SIGNED_DISTANCE);
        assert_eq!(certificate.traces()[1].as_plane(), Some(case.plane));
        let cost = verified_offset_nurbs_plane_intersection_certificate_cost(
            certificate.carrier(),
            certificate.traces(),
        )
        .unwrap();
        assert_eq!(
            (cost.work(), cost.items(), cost.depth()),
            (case.certificate_work, case.certificate_items, 10)
        );
        let total_work = NORMAL_WINDOW_PROOF_WORK + cost.work();
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
        for (resource, consumed) in [
            (ResourceKind::Work, total_work),
            (ResourceKind::Items, cost.items()),
            (ResourceKind::Depth, cost.depth()),
        ] {
            assert_eq!(
                observed(outcome.report(), NURBS_TRACE_CERTIFICATE_WORK, resource),
                consumed
            );
        }

        let reverse = intersect_bounded_graph_surfaces(
            &graph,
            plane,
            case.plane_range,
            root,
            offset_window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(reverse.raw, local.raw.clone().swapped());
        let reverse_certificate = reverse.branch_graph.edges[0]
            .certificate
            .as_nurbs()
            .unwrap();
        assert_eq!(reverse_certificate.traces()[0].as_plane(), Some(case.plane));
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
                root,
                offset_window(),
                plane,
                case.plane_range,
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
                root,
                offset_window(),
                plane,
                case.plane_range,
                &denied_context,
            );
            let GraphSurfaceIntersectionError::OperationPolicy(
                kcore::operation::OperationPolicyError::LimitReached(crossing),
            ) = denied.result().unwrap_err()
            else {
                panic!("N-1 proof resource must fail closed");
            };
            assert_eq!(crossing.stage, stage);
            assert_eq!(crossing.resource, resource);
            assert_eq!(crossing.allowed, allowed);
            assert_eq!(crossing.consumed, consumed);
        }
    }
}

#[test]
fn nested_global_axis_planes_bind_both_offsets_and_pin_exact_resources() {
    let basis_surface = rational_quarter_cylinder(1.0);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();

    for case in axis_cases() {
        let mut graph = GeometryGraph::new();
        let basis = graph.insert_surface(basis_surface.clone()).unwrap();
        let inner = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, INNER_SIGNED_DISTANCE))
            .unwrap();
        let root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(inner, OUTER_SIGNED_DISTANCE))
            .unwrap();
        let plane = graph.insert_surface(case.plane).unwrap();
        let context = OperationContext::new(&session, tolerances).unwrap();
        let outcome = intersect_bounded_graph_surfaces_with_context(
            &graph,
            root,
            offset_window(),
            plane,
            case.plane_range,
            &context,
        );
        let local = outcome.result().unwrap();
        assert_eq!(local.raw.curves.len(), 1);
        assert_eq!(local.branch_graph.edges.len(), 1);
        let certificate = local.branch_graph.edges[0].certificate.as_nurbs().unwrap();
        assert_eq!(certificate.carrier().points().len(), case.carrier_controls);
        let offset_trace = certificate.traces()[0].as_offset_nurbs().unwrap();
        assert_eq!(offset_trace.basis(), &basis_surface);
        assert_eq!(offset_trace.signed_distance(), SIGNED_DISTANCE);
        assert_eq!(
            offset_trace.descriptor_signed_distances(),
            &[OUTER_SIGNED_DISTANCE, INNER_SIGNED_DISTANCE]
        );
        assert_eq!(certificate.traces()[1].as_plane(), Some(case.plane));
        let cost = verified_offset_nurbs_plane_intersection_certificate_cost(
            certificate.carrier(),
            certificate.traces(),
        )
        .unwrap();
        assert_eq!(
            (cost.work(), cost.items(), cost.depth()),
            (case.certificate_work, case.certificate_items, 10)
        );
        let total_work = NORMAL_WINDOW_PROOF_WORK + cost.work();
        assert_eq!(
            observed(
                outcome.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            ),
            3
        );
        assert_eq!(
            observed(
                outcome.report(),
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
            ),
            3
        );
        for (resource, consumed) in [
            (ResourceKind::Work, total_work),
            (ResourceKind::Items, cost.items()),
            (ResourceKind::Depth, cost.depth()),
        ] {
            assert_eq!(
                observed(outcome.report(), NURBS_TRACE_CERTIFICATE_WORK, resource),
                consumed
            );
        }

        let reverse = intersect_bounded_graph_surfaces(
            &graph,
            plane,
            case.plane_range,
            root,
            offset_window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(reverse.raw, local.raw.clone().swapped());
        let reverse_trace = reverse.branch_graph.edges[0]
            .certificate
            .as_nurbs()
            .unwrap()
            .traces()[1]
            .as_offset_nurbs()
            .unwrap();
        assert_eq!(reverse_trace, offset_trace);

        let exact_plan = BudgetPlan::new([
            LimitSpec::new(
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                3,
            ),
            LimitSpec::new(
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                3,
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
                root,
                offset_window(),
                plane,
                case.plane_range,
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
                2,
                3,
            ),
            (
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                2,
                3,
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
            let before = (
                graph.curve_count(),
                graph.curve2d_count(),
                graph.geometry().collect::<Vec<_>>(),
            );
            let denied = intersect_bounded_graph_surfaces_with_context(
                &graph,
                root,
                offset_window(),
                plane,
                case.plane_range,
                &denied_context,
            );
            let GraphSurfaceIntersectionError::OperationPolicy(
                kcore::operation::OperationPolicyError::LimitReached(crossing),
            ) = denied.result().unwrap_err()
            else {
                panic!("N-1 nested varying-offset resource must fail closed");
            };
            assert_eq!(crossing.stage, stage);
            assert_eq!(crossing.resource, resource);
            assert_eq!(crossing.allowed, allowed);
            assert_eq!(crossing.consumed, consumed);
            assert_eq!(graph.curve_count(), before.0);
            assert_eq!(graph.curve2d_count(), before.1);
            assert_eq!(graph.geometry().collect::<Vec<_>>(), before.2);
        }

        let persistent = persist_verified_graph_surface_intersections(&mut graph, local).unwrap();
        let descriptor = graph
            .curve(persistent.edges[0].curve)
            .unwrap()
            .as_verified_nurbs_intersection()
            .unwrap();
        assert_eq!(descriptor.source_surfaces(), [root, plane]);
        assert_eq!(
            descriptor.certificate().traces()[0]
                .as_offset_nurbs()
                .unwrap()
                .descriptor_signed_distances(),
            &[OUTER_SIGNED_DISTANCE, INNER_SIGNED_DISTANCE]
        );
        for protected in [basis, inner, root, plane] {
            assert!(matches!(
                graph.remove_surface(protected),
                Err(GeometryGraphError::HasDependents { .. })
            ));
        }
        graph.validate().unwrap();
    }
}

#[test]
fn positive_branch_persists_with_ordered_sources_and_strict_validation() {
    let case = axis_cases()[0];
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let mut graph = GeometryGraph::new();
    let basis = graph
        .insert_surface(rational_quarter_cylinder(1.0))
        .unwrap();
    let root = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, SIGNED_DISTANCE))
        .unwrap();
    let plane = graph.insert_surface(case.plane).unwrap();
    let local = intersect_bounded_graph_surfaces(
        &graph,
        root,
        offset_window(),
        plane,
        case.plane_range,
        tolerances,
    )
    .unwrap();
    assert_eq!(local.branch_graph.source_surfaces, [root, plane]);
    let certificate = local.branch_graph.edges[0].certificate.as_nurbs().unwrap();
    let mut pcurves = certificate.pcurves().clone();
    let mut tampered_points = pcurves[1].points().to_vec();
    tampered_points[0].x += 0.01;
    pcurves[1] = NurbsCurve2d::new(
        pcurves[1].degree(),
        pcurves[1].knots().as_slice().to_vec(),
        tampered_points,
        pcurves[1].weights().map(<[f64]>::to_vec),
    )
    .unwrap();
    assert!(matches!(
        certify_verified_offset_nurbs_plane_intersection_residuals(
            certificate.carrier().clone(),
            certificate.traces().clone(),
            pcurves,
            certificate.tolerance(),
        ),
        Err(IntersectionCertificateError::ResidualExceedsTolerance { .. })
    ));
    let persistent = persist_verified_graph_surface_intersections(&mut graph, &local).unwrap();
    assert_eq!(persistent.source_surfaces, [root, plane]);
    assert_eq!(persistent.edges.len(), 1);
    for protected in [basis, root, plane] {
        assert!(matches!(
            graph.remove_surface(protected),
            Err(GeometryGraphError::HasDependents { .. })
        ));
    }
    graph.validate().unwrap();
}

#[test]
fn nested_altered_or_stale_sources_roll_persistence_back_atomically() {
    let case = axis_cases()[0];
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for mutation in 0..5 {
        let mut graph = GeometryGraph::new();
        let basis = graph
            .insert_surface(rational_quarter_cylinder(1.0))
            .unwrap();
        let inner = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, INNER_SIGNED_DISTANCE))
            .unwrap();
        let root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(inner, OUTER_SIGNED_DISTANCE))
            .unwrap();
        let plane = graph.insert_surface(case.plane).unwrap();
        let local = intersect_bounded_graph_surfaces(
            &graph,
            root,
            offset_window(),
            plane,
            case.plane_range,
            tolerances,
        )
        .unwrap();
        match mutation {
            0 => graph
                .replace_surface(
                    inner,
                    OffsetSurfaceDescriptor::new(basis, INNER_SIGNED_DISTANCE + 0.001),
                )
                .unwrap(),
            1 => graph
                .replace_surface(
                    root,
                    OffsetSurfaceDescriptor::new(inner, OUTER_SIGNED_DISTANCE + 0.001),
                )
                .unwrap(),
            2 => {
                graph
                    .replace_surface(
                        inner,
                        OffsetSurfaceDescriptor::new(basis, INNER_SIGNED_DISTANCE + 0.001),
                    )
                    .unwrap();
                graph
                    .replace_surface(
                        root,
                        OffsetSurfaceDescriptor::new(inner, OUTER_SIGNED_DISTANCE - 0.001),
                    )
                    .unwrap()
            }
            3 => graph.remove_surface(root).unwrap(),
            4 => graph
                .replace_surface(plane, canonical_plane(0, 0.67))
                .unwrap(),
            _ => unreachable!(),
        };
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
fn exact_original_control_misses_are_complete_in_both_orders() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();
    for axis in 0..3 {
        let mut graph = GeometryGraph::new();
        let basis = graph
            .insert_surface(rational_quarter_cylinder(1.0))
            .unwrap();
        let root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, SIGNED_DISTANCE))
            .unwrap();
        let plane_surface = canonical_plane(axis, 1.2);
        let plane = graph.insert_surface(plane_surface).unwrap();
        let plane_range = if axis == 1 {
            [ParamRange::new(0.0, 1.2), ParamRange::new(-1.0, 0.0)]
        } else {
            [ParamRange::new(0.0, 1.2), ParamRange::new(0.0, 1.2)]
        };
        for (a, a_range, b, b_range) in [
            (root, offset_window(), plane, plane_range),
            (plane, plane_range, root, offset_window()),
        ] {
            let context = OperationContext::new(&session, tolerances).unwrap();
            let outcome = intersect_bounded_graph_surfaces_with_context(
                &graph, a, a_range, b, b_range, &context,
            );
            let result = outcome.result().unwrap();
            assert!(result.raw.is_complete());
            assert!(result.raw.curves.is_empty());
            assert!(result.branch_graph.edges.is_empty());
            assert_eq!(
                observed(
                    outcome.report(),
                    NURBS_TRACE_CERTIFICATE_WORK,
                    ResourceKind::Work,
                ),
                NORMAL_WINDOW_PROOF_WORK
            );
            assert_eq!(
                observed(
                    outcome.report(),
                    NURBS_TRACE_CERTIFICATE_WORK,
                    ResourceKind::Items,
                ),
                1
            );
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
            assert_eq!(
                observed(
                    outcome.report(),
                    NURBS_TRACE_CERTIFICATE_WORK,
                    ResourceKind::Depth,
                ),
                1
            );
        }
    }
}

#[test]
fn nested_original_control_misses_remain_source_proven_in_both_orders() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();
    for axis in 0..3 {
        let mut graph = GeometryGraph::new();
        let basis = graph
            .insert_surface(rational_quarter_cylinder(1.0))
            .unwrap();
        let inner = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, INNER_SIGNED_DISTANCE))
            .unwrap();
        let root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(inner, OUTER_SIGNED_DISTANCE))
            .unwrap();
        let plane_surface = canonical_plane(axis, 1.2);
        let plane = graph.insert_surface(plane_surface).unwrap();
        let plane_range = if axis == 1 {
            [ParamRange::new(0.0, 1.2), ParamRange::new(-1.0, 0.0)]
        } else {
            [ParamRange::new(0.0, 1.2), ParamRange::new(0.0, 1.2)]
        };
        for (a, a_range, b, b_range) in [
            (root, offset_window(), plane, plane_range),
            (plane, plane_range, root, offset_window()),
        ] {
            let context = OperationContext::new(&session, tolerances).unwrap();
            let outcome = intersect_bounded_graph_surfaces_with_context(
                &graph, a, a_range, b, b_range, &context,
            );
            let result = outcome.result().unwrap();
            assert!(result.raw.is_complete());
            assert!(result.raw.curves.is_empty());
            assert!(result.branch_graph.edges.is_empty());
            assert_eq!(
                observed(
                    outcome.report(),
                    NURBS_TRACE_CERTIFICATE_WORK,
                    ResourceKind::Work,
                ),
                NORMAL_WINDOW_PROOF_WORK
            );
            assert_eq!(
                observed(
                    outcome.report(),
                    NURBS_TRACE_CERTIFICATE_WORK,
                    ResourceKind::Items,
                ),
                1
            );
            assert_eq!(
                observed(
                    outcome.report(),
                    NURBS_TRACE_CERTIFICATE_WORK,
                    ResourceKind::Depth,
                ),
                1
            );
            assert_eq!(
                observed(
                    outcome.report(),
                    kgraph::eval_stage::NODE_VISITS,
                    ResourceKind::Work,
                ),
                3
            );
            assert_eq!(
                observed(
                    outcome.report(),
                    kgraph::eval_stage::DEPENDENCY_DEPTH,
                    ResourceKind::Depth,
                ),
                3
            );
        }
        let before = graph.geometry().collect::<Vec<_>>();
        let miss = intersect_bounded_graph_surfaces(
            &graph,
            root,
            offset_window(),
            plane,
            plane_range,
            tolerances,
        )
        .unwrap();
        let persistent = persist_verified_graph_surface_intersections(&mut graph, &miss).unwrap();
        assert!(persistent.edges.is_empty());
        assert_eq!(graph.geometry().collect::<Vec<_>>(), before);
    }
}

#[test]
fn malformed_peers_deeper_chains_and_singular_intermediates_are_unsupported() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let mut graph = GeometryGraph::new();
    let basis = graph
        .insert_surface(rational_quarter_cylinder(1.0))
        .unwrap();
    let root = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, SIGNED_DISTANCE))
        .unwrap();
    let nested = graph
        .insert_surface(OffsetSurfaceDescriptor::new(root, 0.02))
        .unwrap();
    let too_deep = graph
        .insert_surface(OffsetSurfaceDescriptor::new(nested, 0.01))
        .unwrap();
    let collapsed_inner = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, -1.0))
        .unwrap();
    let recovered_final_radius = graph
        .insert_surface(OffsetSurfaceDescriptor::new(collapsed_inner, 0.2))
        .unwrap();
    let canonical = graph.insert_surface(canonical_plane(0, 0.66)).unwrap();
    let malformed = graph
        .insert_surface(Plane::new(
            Frame::new(
                Point3::new(0.66, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap(),
        ))
        .unwrap();
    assert!(matches!(
        intersect_bounded_graph_surfaces(
            &graph,
            root,
            offset_window(),
            malformed,
            axis_cases()[0].plane_range,
            tolerances,
        ),
        Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::UnsupportedSurfacePair { .. }
        ))
    ));
    assert!(matches!(
        intersect_bounded_graph_surfaces(
            &graph,
            too_deep,
            offset_window(),
            canonical,
            axis_cases()[0].plane_range,
            tolerances,
        ),
        Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::UnsupportedSurfacePair { .. }
        ))
    ));
    assert!(matches!(
        intersect_bounded_graph_surfaces(
            &graph,
            recovered_final_radius,
            offset_window(),
            canonical,
            axis_cases()[0].plane_range,
            tolerances,
        ),
        Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::UnsupportedSurfacePair { .. }
        ))
    ));
    graph.validate().unwrap();
}

#[test]
fn altered_or_stale_exact_sources_roll_persistence_back_atomically() {
    let case = axis_cases()[0];
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for mutation in 0..4 {
        let mut graph = GeometryGraph::new();
        let basis = graph
            .insert_surface(rational_quarter_cylinder(1.0))
            .unwrap();
        let root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, SIGNED_DISTANCE))
            .unwrap();
        let plane = graph.insert_surface(case.plane).unwrap();
        let local = intersect_bounded_graph_surfaces(
            &graph,
            root,
            offset_window(),
            plane,
            case.plane_range,
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
                    .replace_surface(plane, canonical_plane(0, 0.67))
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
            3 => {
                graph.remove_surface(plane).unwrap();
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
