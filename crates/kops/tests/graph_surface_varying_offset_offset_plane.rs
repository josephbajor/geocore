//! Certified varying-normal Offset(NURBS)/one-descriptor Offset(Plane) graphs.

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

const NURBS_OFFSET: f64 = 0.1;
const PLANE_OFFSET: f64 = 0.05;
const NORMAL_WINDOW_PROOF_WORK: u64 = 7;

#[derive(Clone, Copy)]
struct AxisCase {
    axis: usize,
    effective_coordinate: f64,
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

fn canonical_plane(axis: usize, coordinate: f64) -> Plane {
    let (origin, normal, tangent) = match axis {
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
            axis: 0,
            effective_coordinate: 0.66,
            plane_range: [ParamRange::new(0.1, 1.0), ParamRange::new(0.1, 0.9)],
            carrier_controls: 2,
            certificate_work: 7_170,
            certificate_items: 1_024,
        },
        AxisCase {
            axis: 1,
            effective_coordinate: 0.88,
            plane_range: [ParamRange::new(0.1, 0.9), ParamRange::new(-0.9, -0.1)],
            carrier_controls: 2,
            certificate_work: 7_170,
            certificate_items: 1_024,
        },
        AxisCase {
            axis: 2,
            effective_coordinate: 0.5,
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
fn x_y_z_offset_planes_promote_swap_and_pin_exact_resources() {
    let basis_surface = rational_quarter_cylinder(1.0);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();

    for case in axis_cases() {
        let plane_basis_surface =
            canonical_plane(case.axis, case.effective_coordinate - PLANE_OFFSET);
        let mut graph = GeometryGraph::new();
        let basis = graph.insert_surface(basis_surface.clone()).unwrap();
        let nurbs_root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, NURBS_OFFSET))
            .unwrap();
        let plane_basis = graph.insert_surface(plane_basis_surface).unwrap();
        let plane_root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(plane_basis, PLANE_OFFSET))
            .unwrap();
        let context = OperationContext::new(&session, tolerances).unwrap();
        let outcome = intersect_bounded_graph_surfaces_with_context(
            &graph,
            nurbs_root,
            offset_window(),
            plane_root,
            case.plane_range,
            &context,
        );
        let local = outcome.result().unwrap();
        assert_eq!(local.raw.curves.len(), 1);
        assert_eq!(local.branch_graph.edges.len(), 1);
        let certificate = local.branch_graph.edges[0].certificate.as_nurbs().unwrap();
        assert_eq!(certificate.carrier().points().len(), case.carrier_controls);
        assert_eq!(
            certificate.traces()[0].as_offset_nurbs().unwrap().basis(),
            &basis_surface
        );
        let plane_trace = certificate.traces()[1].as_offset_plane().unwrap();
        assert_eq!(plane_trace.basis(), plane_basis_surface);
        assert_eq!(plane_trace.signed_distance(), PLANE_OFFSET);
        assert_eq!(
            plane_trace.effective_plane(),
            Some(canonical_plane(case.axis, case.effective_coordinate))
        );
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
            4
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
            plane_root,
            case.plane_range,
            nurbs_root,
            offset_window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(reverse.raw, local.raw.clone().swapped());
        let reverse_certificate = reverse.branch_graph.edges[0]
            .certificate
            .as_nurbs()
            .unwrap();
        assert_eq!(
            reverse_certificate.traces()[0].as_offset_plane(),
            Some(plane_trace)
        );
        assert_eq!(
            reverse_certificate.traces()[1].as_offset_nurbs(),
            certificate.traces()[0].as_offset_nurbs()
        );

        let exact_plan = BudgetPlan::new([
            LimitSpec::new(
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                4,
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
                nurbs_root,
                offset_window(),
                plane_root,
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
                3,
                4,
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
            let plan = BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap();
            let denied_context = OperationContext::new(&session, tolerances)
                .unwrap()
                .with_budget_overrides(plan);
            let denied = intersect_bounded_graph_surfaces_with_context(
                &graph,
                nurbs_root,
                offset_window(),
                plane_root,
                case.plane_range,
                &denied_context,
            );
            let GraphSurfaceIntersectionError::OperationPolicy(
                kcore::operation::OperationPolicyError::LimitReached(crossing),
            ) = denied.result().unwrap_err()
            else {
                panic!("N-1 resource must fail closed");
            };
            assert_eq!(crossing.stage, stage);
            assert_eq!(crossing.resource, resource);
            assert_eq!(crossing.allowed, allowed);
            assert_eq!(crossing.consumed, consumed);
        }
    }
}

#[test]
fn offset_plane_branch_rejects_tampering_and_persists_transitive_identity() {
    let case = axis_cases()[0];
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let plane_basis_surface = canonical_plane(case.axis, case.effective_coordinate - PLANE_OFFSET);
    let mut graph = GeometryGraph::new();
    let basis = graph
        .insert_surface(rational_quarter_cylinder(1.0))
        .unwrap();
    let nurbs_root = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, NURBS_OFFSET))
        .unwrap();
    let plane_basis = graph.insert_surface(plane_basis_surface).unwrap();
    let plane_root = graph
        .insert_surface(OffsetSurfaceDescriptor::new(plane_basis, PLANE_OFFSET))
        .unwrap();
    let local = intersect_bounded_graph_surfaces(
        &graph,
        nurbs_root,
        offset_window(),
        plane_root,
        case.plane_range,
        tolerances,
    )
    .unwrap();
    assert_eq!(local.branch_graph.source_surfaces, [nurbs_root, plane_root]);
    let certificate = local.branch_graph.edges[0].certificate.as_nurbs().unwrap();
    let mut pcurves = certificate.pcurves().clone();
    let mut points = pcurves[1].points().to_vec();
    points[0].x += 0.01;
    pcurves[1] = NurbsCurve2d::new(
        pcurves[1].degree(),
        pcurves[1].knots().as_slice().to_vec(),
        points,
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

    let reverse = intersect_bounded_graph_surfaces(
        &graph,
        plane_root,
        case.plane_range,
        nurbs_root,
        offset_window(),
        tolerances,
    )
    .unwrap();

    let persistent = persist_verified_graph_surface_intersections(&mut graph, &local).unwrap();
    assert_eq!(persistent.source_surfaces, [nurbs_root, plane_root]);
    assert_eq!(persistent.edges.len(), 1);
    let reverse_persistent =
        persist_verified_graph_surface_intersections(&mut graph, &reverse).unwrap();
    assert_eq!(reverse_persistent.source_surfaces, [plane_root, nurbs_root]);
    assert_eq!(reverse_persistent.edges.len(), 1);
    for protected in [basis, nurbs_root, plane_basis, plane_root] {
        assert!(matches!(
            graph.remove_surface(protected),
            Err(GeometryGraphError::HasDependents { .. })
        ));
    }
    graph.validate().unwrap();
}

#[test]
fn offset_plane_original_control_misses_are_complete_in_both_orders() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();
    for axis in 0..3 {
        let mut graph = GeometryGraph::new();
        let basis = graph
            .insert_surface(rational_quarter_cylinder(1.0))
            .unwrap();
        let nurbs_root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, NURBS_OFFSET))
            .unwrap();
        let plane_basis = graph
            .insert_surface(canonical_plane(axis, 1.2 - PLANE_OFFSET))
            .unwrap();
        let plane_root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(plane_basis, PLANE_OFFSET))
            .unwrap();
        let plane_range = if axis == 1 {
            [ParamRange::new(0.0, 1.2), ParamRange::new(-1.0, 0.0)]
        } else {
            [ParamRange::new(0.0, 1.2), ParamRange::new(0.0, 1.2)]
        };
        for (a, a_range, b, b_range) in [
            (nurbs_root, offset_window(), plane_root, plane_range),
            (plane_root, plane_range, nurbs_root, offset_window()),
        ] {
            let context = OperationContext::new(&session, tolerances).unwrap();
            let outcome = intersect_bounded_graph_surfaces_with_context(
                &graph, a, a_range, b, b_range, &context,
            );
            let result = outcome.result().unwrap();
            assert!(result.raw.is_complete());
            assert!(result.raw.curves.is_empty());
            assert!(result.branch_graph.edges.is_empty());
            for (stage, resource, consumed) in [
                (kgraph::eval_stage::NODE_VISITS, ResourceKind::Work, 4),
                (kgraph::eval_stage::DEPENDENCY_DEPTH, ResourceKind::Depth, 2),
                (
                    NURBS_TRACE_CERTIFICATE_WORK,
                    ResourceKind::Work,
                    NORMAL_WINDOW_PROOF_WORK,
                ),
                (NURBS_TRACE_CERTIFICATE_WORK, ResourceKind::Items, 1),
                (NURBS_TRACE_CERTIFICATE_WORK, ResourceKind::Depth, 1),
            ] {
                assert_eq!(observed(outcome.report(), stage, resource), consumed);
            }
        }
    }
}

#[test]
fn unsafe_noncanonical_and_nested_plane_offsets_fail_closed() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let case = axis_cases()[0];
    let mut graph = GeometryGraph::new();
    let basis = graph
        .insert_surface(rational_quarter_cylinder(1.0))
        .unwrap();
    let nurbs_root = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, NURBS_OFFSET))
        .unwrap();
    let plane_basis = graph
        .insert_surface(canonical_plane(0, case.effective_coordinate - PLANE_OFFSET))
        .unwrap();
    assert!(matches!(
        graph.insert_surface(OffsetSurfaceDescriptor::new(plane_basis, f64::NAN)),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    let plane_root = graph
        .insert_surface(OffsetSurfaceDescriptor::new(plane_basis, PLANE_OFFSET))
        .unwrap();
    let nested = graph
        .insert_surface(OffsetSurfaceDescriptor::new(plane_root, 0.01))
        .unwrap();
    let deeper = graph
        .insert_surface(OffsetSurfaceDescriptor::new(nested, -0.01))
        .unwrap();
    let noncanonical_basis = graph
        .insert_surface(Plane::new(
            Frame::new(
                Point3::new(case.effective_coordinate - PLANE_OFFSET, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap(),
        ))
        .unwrap();
    let noncanonical = graph
        .insert_surface(OffsetSurfaceDescriptor::new(
            noncanonical_basis,
            PLANE_OFFSET,
        ))
        .unwrap();
    for peer in [nested, deeper, noncanonical] {
        assert!(matches!(
            intersect_bounded_graph_surfaces(
                &graph,
                nurbs_root,
                offset_window(),
                peer,
                case.plane_range,
                tolerances,
            ),
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::UnsupportedSurfacePair { .. }
            ))
        ));
    }

    let huge_basis = graph.insert_surface(canonical_plane(0, f64::MAX)).unwrap();
    let overflow = graph
        .insert_surface(OffsetSurfaceDescriptor::new(huge_basis, f64::MAX))
        .unwrap();
    assert!(
        intersect_bounded_graph_surfaces(
            &graph,
            nurbs_root,
            offset_window(),
            overflow,
            case.plane_range,
            tolerances,
        )
        .is_err()
    );
    graph.validate().unwrap();
}

#[test]
fn stale_altered_compensated_and_nested_sources_roll_back_atomically() {
    let case = axis_cases()[0];
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for mutation in 0..7 {
        let plane_basis_surface =
            canonical_plane(case.axis, case.effective_coordinate - PLANE_OFFSET);
        let mut graph = GeometryGraph::new();
        let basis = graph
            .insert_surface(rational_quarter_cylinder(1.0))
            .unwrap();
        let nurbs_root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, NURBS_OFFSET))
            .unwrap();
        let plane_basis = graph.insert_surface(plane_basis_surface).unwrap();
        let plane_root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(plane_basis, PLANE_OFFSET))
            .unwrap();
        let local = intersect_bounded_graph_surfaces(
            &graph,
            nurbs_root,
            offset_window(),
            plane_root,
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
                    .replace_surface(
                        plane_basis,
                        canonical_plane(case.axis, case.effective_coordinate - PLANE_OFFSET + 0.01),
                    )
                    .unwrap();
            }
            2 => {
                graph
                    .replace_surface(
                        plane_root,
                        OffsetSurfaceDescriptor::new(plane_basis, PLANE_OFFSET + 0.01),
                    )
                    .unwrap();
            }
            3 => {
                graph
                    .replace_surface(
                        plane_basis,
                        canonical_plane(case.axis, case.effective_coordinate - PLANE_OFFSET + 0.01),
                    )
                    .unwrap();
                graph
                    .replace_surface(
                        plane_root,
                        OffsetSurfaceDescriptor::new(plane_basis, PLANE_OFFSET - 0.01),
                    )
                    .unwrap();
            }
            4 => {
                let inner = graph
                    .insert_surface(OffsetSurfaceDescriptor::new(plane_basis, 0.02))
                    .unwrap();
                graph
                    .replace_surface(
                        plane_root,
                        OffsetSurfaceDescriptor::new(inner, PLANE_OFFSET - 0.02),
                    )
                    .unwrap();
            }
            5 => {
                let inner = graph
                    .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.04))
                    .unwrap();
                graph
                    .replace_surface(
                        nurbs_root,
                        OffsetSurfaceDescriptor::new(inner, NURBS_OFFSET - 0.04),
                    )
                    .unwrap();
            }
            6 => {
                graph.remove_surface(plane_root).unwrap();
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
