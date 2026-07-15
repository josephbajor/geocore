//! Contextual graph-owned direct Offset(NURBS)/NURBS branch contracts.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::Surface;
use kgeom::vec::Point3;
use kgraph::{
    GeometryGraph, GeometryGraphError, OffsetSurfaceDescriptor,
    verified_dual_offset_nurbs_intersection_certificate_cost,
    verified_offset_nurbs_nurbs_intersection_certificate_cost,
};
use kops::intersect::{
    GraphSurfaceIntersectionError, IntersectionError, NURBS_SURFACE_MARCH_SAMPLES,
    NURBS_TRACE_CERTIFICATE_WORK, NurbsSurfaceMarchBudgetProfile, intersect_bounded_graph_surfaces,
    intersect_bounded_graph_surfaces_with_context, intersect_bounded_offset_nurbs_nurbs_surfaces,
    intersect_bounded_offset_nurbs_nurbs_surfaces_with_context,
    persist_verified_graph_surface_intersections,
};

fn unit_chart_surface(z_controls: [f64; 3], rational: bool) -> NurbsSurface {
    let coordinates = [0.0, 0.5, 1.0];
    let mut points = Vec::with_capacity(6);
    for (u_index, &u) in coordinates.iter().enumerate() {
        for &v in &[0.0, 1.0] {
            points.push(Point3::new(u, v, z_controls[u_index]));
        }
    }
    NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        points,
        rational.then(|| vec![2.0; 6]),
    )
    .unwrap()
}

fn basis(rational: bool) -> NurbsSurface {
    unit_chart_surface([0.25; 3], rational)
}

fn crossing_direct(effective_height: f64, rational: bool) -> NurbsSurface {
    let bend = [0.0, 0.02, 0.0];
    let delta = [-0.2, -0.0148, 0.1704];
    unit_chart_surface(
        std::array::from_fn(|index| effective_height + bend[index] + delta[index]),
        rational,
    )
}

fn crossing_constant_normal_basis(rational: bool) -> NurbsSurface {
    unit_chart_surface([0.05, 0.25, 0.45], rational)
}

fn dual_window_a() -> [ParamRange; 2] {
    [ParamRange::new(0.1, 0.9), ParamRange::new(0.1, 0.8)]
}

fn dual_window_b() -> [ParamRange; 2] {
    [ParamRange::new(0.15, 0.95), ParamRange::new(0.0, 0.9)]
}

fn insert_offset_chain(
    graph: &mut GeometryGraph,
    surface: NurbsSurface,
    distances: &[f64],
) -> (
    kgraph::SurfaceHandle,
    kgraph::SurfaceHandle,
    Vec<kgraph::SurfaceHandle>,
) {
    let basis = graph.insert_surface(surface).unwrap();
    let mut root = basis;
    let mut chain = vec![basis];
    for &distance in distances {
        root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(root, distance))
            .unwrap();
        chain.push(root);
    }
    (basis, root, chain)
}

fn narrow_window() -> [ParamRange; 2] {
    [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 0.0015)]
}

fn overlapping_offset_window() -> [ParamRange; 2] {
    [ParamRange::new(0.0, 0.8), ParamRange::new(0.0, 0.0015)]
}

fn overlapping_direct_window() -> [ParamRange; 2] {
    [ParamRange::new(0.2, 1.0), ParamRange::new(0.0005, 0.0015)]
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
fn overlapping_windows_promote_in_both_orders_and_persist_identities() {
    let signed_distance = 0.05;
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for rational in [false, true] {
        let basis = basis(rational);
        let direct = crossing_direct(0.25 + signed_distance, rational);
        let lower = intersect_bounded_offset_nurbs_nurbs_surfaces(
            &basis,
            signed_distance,
            overlapping_offset_window(),
            &direct,
            overlapping_direct_window(),
            tolerances,
        )
        .unwrap();
        assert!(!lower.is_complete());
        assert_eq!(lower.curves.len(), 1);

        let mut graph = GeometryGraph::new();
        let basis_handle = graph.insert_surface(basis.clone()).unwrap();
        let offset_handle = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis_handle, signed_distance))
            .unwrap();
        let direct_handle = graph.insert_surface(direct.clone()).unwrap();
        let result = intersect_bounded_graph_surfaces(
            &graph,
            offset_handle,
            overlapping_offset_window(),
            direct_handle,
            overlapping_direct_window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(result.raw, lower);
        let raw_branch = &result.raw.curves[0];
        for coordinate in [raw_branch.uv_a_start[1], raw_branch.uv_b_start[1]] {
            assert!((coordinate - 0.0005).abs() <= f64::EPSILON);
        }
        for coordinate in [raw_branch.uv_a_end[1], raw_branch.uv_b_end[1]] {
            assert!((coordinate - 0.0015).abs() <= f64::EPSILON);
        }
        assert_eq!(
            result.branch_graph.source_surfaces,
            [offset_handle, direct_handle]
        );
        assert_eq!(result.branch_graph.edges.len(), 1);
        assert_eq!(
            result.branch_graph.edges[0]
                .endpoint_events
                .map(|event| match event {
                    kops::intersect::IntersectionBranchEndpointEvent::SurfaceWindowBoundary {
                        surfaces,
                    } => surfaces,
                }),
            [[true, true], [true, true]]
        );
        let certificate = result.branch_graph.edges[0].certificate.as_nurbs().unwrap();
        let offset_trace = certificate.traces()[0].as_offset_nurbs().unwrap();
        assert_eq!(offset_trace.basis(), &basis);
        assert_eq!(offset_trace.signed_distance(), signed_distance);
        assert_eq!(certificate.traces()[1].as_nurbs(), Some(&direct));
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= tolerances.linear())
        );

        let reverse = intersect_bounded_graph_surfaces(
            &graph,
            direct_handle,
            overlapping_direct_window(),
            offset_handle,
            overlapping_offset_window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(reverse.raw, result.raw.clone().swapped());
        assert_eq!(
            reverse.branch_graph.source_surfaces,
            [direct_handle, offset_handle]
        );
        let reverse_certificate = reverse.branch_graph.edges[0]
            .certificate
            .as_nurbs()
            .unwrap();
        assert_eq!(reverse_certificate.traces()[0].as_nurbs(), Some(&direct));
        assert_eq!(
            reverse_certificate.traces()[1].as_offset_nurbs().unwrap(),
            offset_trace
        );

        let persistent = persist_verified_graph_surface_intersections(&mut graph, &result).unwrap();
        let descriptor = graph
            .curve(persistent.edges[0].curve)
            .unwrap()
            .as_verified_nurbs_intersection()
            .unwrap();
        assert_eq!(descriptor.source_surfaces(), [offset_handle, direct_handle]);
        assert_eq!(descriptor.certificate(), certificate);
        for protected in [basis_handle, offset_handle, direct_handle] {
            assert!(matches!(
                graph.remove_surface(protected),
                Err(GeometryGraphError::HasDependents { .. })
            ));
        }
        graph.validate().unwrap();
    }
}

#[test]
fn capped_nested_constant_normal_offsets_preserve_chain_proof_and_exact_limits() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();
    for distances in [
        &[0.02, 0.03][..],
        &[0.01, 0.015, 0.025][..],
        &[0.005, 0.01, 0.015, 0.02][..],
    ] {
        let signed_distance = distances.iter().sum::<f64>();
        let basis = basis(false);
        let direct = crossing_direct(0.25 + signed_distance, false);
        let lower = intersect_bounded_offset_nurbs_nurbs_surfaces(
            &basis,
            signed_distance,
            overlapping_offset_window(),
            &direct,
            overlapping_direct_window(),
            tolerances,
        )
        .unwrap();

        let mut graph = GeometryGraph::new();
        let basis_handle = graph.insert_surface(basis.clone()).unwrap();
        let mut chain_handles = vec![basis_handle];
        let mut root = basis_handle;
        for &distance in distances {
            root = graph
                .insert_surface(OffsetSurfaceDescriptor::new(root, distance))
                .unwrap();
            chain_handles.push(root);
        }
        let direct_handle = graph.insert_surface(direct.clone()).unwrap();
        let expected_graph_resource = distances.len() as u64 + 1;
        let context = OperationContext::new(&session, tolerances).unwrap();
        let outcome = intersect_bounded_graph_surfaces_with_context(
            &graph,
            root,
            overlapping_offset_window(),
            direct_handle,
            overlapping_direct_window(),
            &context,
        );
        let result = outcome.result().unwrap();
        assert_eq!(&result.raw, &lower);
        assert_eq!(
            observed(
                outcome.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            ),
            expected_graph_resource
        );
        assert_eq!(
            observed(
                outcome.report(),
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
            ),
            expected_graph_resource
        );

        let certificate = result.branch_graph.edges[0].certificate.as_nurbs().unwrap();
        let offset_trace = certificate.traces()[0].as_offset_nurbs().unwrap();
        assert_eq!(offset_trace.basis(), &basis);
        assert_eq!(offset_trace.signed_distance(), signed_distance);
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

        let reverse = intersect_bounded_graph_surfaces(
            &graph,
            direct_handle,
            overlapping_direct_window(),
            root,
            overlapping_offset_window(),
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
                expected_graph_resource,
            ),
            LimitSpec::new(
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                expected_graph_resource,
            ),
            LimitSpec::new(
                NURBS_TRACE_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                cost.work(),
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
                overlapping_offset_window(),
                direct_handle,
                overlapping_direct_window(),
                &exact_context,
            )
            .result()
            .is_ok()
        );

        if distances.len() == 4 {
            for (resource, mode, allowed, consumed) in [
                (
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    cost.work() - 1,
                    cost.work(),
                ),
                (
                    ResourceKind::Items,
                    AccountingMode::HighWater,
                    cost.items() - 1,
                    cost.items(),
                ),
                (
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    cost.depth() - 1,
                    cost.depth(),
                ),
            ] {
                let denied_plan = BudgetPlan::new([LimitSpec::new(
                    NURBS_TRACE_CERTIFICATE_WORK,
                    resource,
                    mode,
                    allowed,
                )])
                .unwrap();
                let denied_context = OperationContext::new(&session, tolerances)
                    .unwrap()
                    .with_budget_overrides(denied_plan);
                let denied = intersect_bounded_graph_surfaces_with_context(
                    &graph,
                    root,
                    overlapping_offset_window(),
                    direct_handle,
                    overlapping_direct_window(),
                    &denied_context,
                );
                let GraphSurfaceIntersectionError::OperationPolicy(
                    kcore::operation::OperationPolicyError::LimitReached(crossing),
                ) = denied.result().unwrap_err()
                else {
                    panic!("N-1 four-descriptor certificate resource must stop at proof stage");
                };
                assert_eq!(crossing.stage, NURBS_TRACE_CERTIFICATE_WORK);
                assert_eq!(crossing.resource, resource);
                assert_eq!(crossing.allowed, allowed);
                assert_eq!(crossing.consumed, consumed);
            }
        }

        for (stage, resource, mode) in [
            (
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
            ),
            (
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
            ),
        ] {
            let denied_plan = BudgetPlan::new([LimitSpec::new(
                stage,
                resource,
                mode,
                expected_graph_resource - 1,
            )])
            .unwrap();
            let denied_context = OperationContext::new(&session, tolerances)
                .unwrap()
                .with_budget_overrides(denied_plan);
            let denied = intersect_bounded_graph_surfaces_with_context(
                &graph,
                root,
                overlapping_offset_window(),
                direct_handle,
                overlapping_direct_window(),
                &denied_context,
            );
            let GraphSurfaceIntersectionError::OperationPolicy(
                kcore::operation::OperationPolicyError::LimitReached(crossing),
            ) = denied.result().unwrap_err()
            else {
                panic!("N-1 nested graph resource must stop at {stage:?}");
            };
            assert_eq!(crossing.stage, stage);
            assert_eq!(crossing.resource, resource);
            assert_eq!(crossing.allowed, expected_graph_resource - 1);
            assert_eq!(crossing.consumed, expected_graph_resource);
        }

        let persistent = persist_verified_graph_surface_intersections(&mut graph, result).unwrap();
        assert_eq!(
            graph
                .curve(persistent.edges[0].curve)
                .unwrap()
                .as_verified_nurbs_intersection()
                .unwrap()
                .source_surfaces(),
            [root, direct_handle]
        );
        chain_handles.push(direct_handle);
        for protected in chain_handles {
            assert!(matches!(
                graph.remove_surface(protected),
                Err(GeometryGraphError::HasDependents { .. })
            ));
        }
        graph.validate().unwrap();
    }
}

#[test]
fn altered_four_descriptor_offset_rolls_persistence_back_atomically() {
    let inner_distance = 0.01;
    let middle_distance = 0.01;
    let penultimate_distance = 0.015;
    let outer_distance = 0.015;
    let signed_distance = inner_distance + middle_distance + penultimate_distance + outer_distance;
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let mut graph = GeometryGraph::new();
    let basis_handle = graph.insert_surface(basis(false)).unwrap();
    let inner = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_handle, inner_distance))
        .unwrap();
    let middle = graph
        .insert_surface(OffsetSurfaceDescriptor::new(inner, middle_distance))
        .unwrap();
    let penultimate = graph
        .insert_surface(OffsetSurfaceDescriptor::new(middle, penultimate_distance))
        .unwrap();
    let outer = graph
        .insert_surface(OffsetSurfaceDescriptor::new(penultimate, outer_distance))
        .unwrap();
    let direct = graph
        .insert_surface(crossing_direct(0.25 + signed_distance, false))
        .unwrap();
    let local = intersect_bounded_graph_surfaces(
        &graph,
        outer,
        overlapping_offset_window(),
        direct,
        overlapping_direct_window(),
        tolerances,
    )
    .unwrap();

    graph
        .replace_surface(
            penultimate,
            OffsetSurfaceDescriptor::new(middle, penultimate_distance + 0.001),
        )
        .unwrap();
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

#[test]
fn scoped_offset_pair_preserves_marcher_report_and_has_exact_certificate_cost() {
    let signed_distance = 0.05;
    let basis = basis(false);
    let direct = crossing_direct(0.25 + signed_distance, false);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();
    let lower_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_family_budget_defaults(NurbsSurfaceMarchBudgetProfile::v1_defaults());
    let lower = intersect_bounded_offset_nurbs_nurbs_surfaces_with_context(
        &basis,
        signed_distance,
        overlapping_offset_window(),
        &direct,
        overlapping_direct_window(),
        &lower_context,
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let basis_handle = graph.insert_surface(basis).unwrap();
    let offset_handle = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_handle, signed_distance))
        .unwrap();
    let direct_handle = graph.insert_surface(direct).unwrap();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        offset_handle,
        overlapping_offset_window(),
        direct_handle,
        overlapping_direct_window(),
        &context,
    );
    let result = outcome.result().unwrap();
    assert_eq!(&result.raw, *lower.result().as_ref().unwrap());
    for (stage, resource) in [
        (
            kgeom::nurbs::NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            ResourceKind::Work,
        ),
        (
            kgeom::nurbs::NURBS_IMPLICIT_ISOLATION_CANDIDATES,
            ResourceKind::Items,
        ),
        (
            kgeom::nurbs::NURBS_IMPLICIT_ISOLATION_DEPTH,
            ResourceKind::Depth,
        ),
        (NURBS_SURFACE_MARCH_SAMPLES, ResourceKind::Work),
    ] {
        assert_eq!(
            observed(outcome.report(), stage, resource),
            observed(lower.report(), stage, resource),
            "lower marcher report parity for {stage:?}"
        );
    }
    assert_eq!(
        observed(
            outcome.report(),
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work
        ),
        2
    );
    assert_eq!(
        observed(
            outcome.report(),
            kgraph::eval_stage::DEPENDENCY_DEPTH,
            ResourceKind::Depth
        ),
        2
    );

    let certificate = result.branch_graph.edges[0].certificate.as_nurbs().unwrap();
    let cost = verified_offset_nurbs_nurbs_intersection_certificate_cost(
        certificate.carrier(),
        certificate.traces(),
    )
    .unwrap();
    assert_eq!(
        (cost.work(), cost.items(), cost.depth()),
        (14_336, 1_024, 10)
    );
    for (resource, expected) in [
        (ResourceKind::Work, cost.work()),
        (ResourceKind::Items, cost.items()),
        (ResourceKind::Depth, cost.depth()),
    ] {
        assert_eq!(
            observed(outcome.report(), NURBS_TRACE_CERTIFICATE_WORK, resource),
            expected
        );
    }

    let exact_plan = BudgetPlan::new([
        LimitSpec::new(
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            cost.work(),
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
            overlapping_offset_window(),
            direct_handle,
            overlapping_direct_window(),
            &exact_context,
        )
        .result()
        .is_ok()
    );

    for (resource, mode, allowed, consumed) in [
        (
            ResourceKind::Work,
            AccountingMode::Cumulative,
            cost.work() - 1,
            cost.work(),
        ),
        (
            ResourceKind::Items,
            AccountingMode::HighWater,
            cost.items() - 1,
            cost.items(),
        ),
        (
            ResourceKind::Depth,
            AccountingMode::HighWater,
            cost.depth() - 1,
            cost.depth(),
        ),
    ] {
        let denied_plan = BudgetPlan::new([LimitSpec::new(
            NURBS_TRACE_CERTIFICATE_WORK,
            resource,
            mode,
            allowed,
        )])
        .unwrap();
        let denied_context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(denied_plan);
        let denied = intersect_bounded_graph_surfaces_with_context(
            &graph,
            offset_handle,
            overlapping_offset_window(),
            direct_handle,
            overlapping_direct_window(),
            &denied_context,
        );
        let GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(crossing),
        ) = denied.result().unwrap_err()
        else {
            panic!("N-1 resource must stop at the certificate stage");
        };
        assert_eq!(crossing.stage, NURBS_TRACE_CERTIFICATE_WORK);
        assert_eq!(crossing.resource, resource);
        assert_eq!(crossing.allowed, allowed);
        assert_eq!(crossing.consumed, consumed);
    }
}

#[test]
fn intersecting_dual_offsets_cover_the_full_one_through_four_chain_matrix() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();
    let chains_a = [
        &[0.05][..],
        &[0.02, 0.03][..],
        &[0.01, 0.015, 0.025][..],
        &[0.005, 0.01, 0.015, 0.02][..],
    ];
    let chains_b = [
        &[0.02][..],
        &[0.008, 0.012][..],
        &[0.004, 0.006, 0.01][..],
        &[0.002, 0.004, 0.006, 0.008][..],
    ];

    for distances_a in chains_a {
        for distances_b in chains_b {
            let mut graph = GeometryGraph::new();
            let (basis_a_handle, root_a, chain_a) =
                insert_offset_chain(&mut graph, basis(false), distances_a);
            let (basis_b_handle, root_b, chain_b) = insert_offset_chain(
                &mut graph,
                crossing_constant_normal_basis(false),
                distances_b,
            );
            let expected_visits = (distances_a.len() + distances_b.len() + 2) as u64;
            let expected_depth = (distances_a.len().max(distances_b.len()) + 1) as u64;
            let context = OperationContext::new(&session, tolerances).unwrap();
            let outcome = intersect_bounded_graph_surfaces_with_context(
                &graph,
                root_a,
                dual_window_a(),
                root_b,
                dual_window_b(),
                &context,
            );
            let result = outcome.result().unwrap();
            assert!(!result.raw.is_complete());
            assert_eq!(result.raw.curves.len(), 1);
            assert_eq!(result.branch_graph.source_surfaces, [root_a, root_b]);
            assert_eq!(result.branch_graph.edges.len(), 1);
            assert_eq!(
                observed(
                    outcome.report(),
                    kgraph::eval_stage::NODE_VISITS,
                    ResourceKind::Work,
                ),
                expected_visits
            );
            assert_eq!(
                observed(
                    outcome.report(),
                    kgraph::eval_stage::DEPENDENCY_DEPTH,
                    ResourceKind::Depth,
                ),
                expected_depth
            );

            let certificate = result.branch_graph.edges[0].certificate.as_nurbs().unwrap();
            let trace_a = certificate.traces()[0].as_offset_nurbs().unwrap();
            let trace_b = certificate.traces()[1].as_offset_nurbs().unwrap();
            assert_eq!(
                trace_a.basis(),
                graph.surface(basis_a_handle).unwrap().as_nurbs().unwrap()
            );
            assert_eq!(
                trace_b.basis(),
                graph.surface(basis_b_handle).unwrap().as_nurbs().unwrap()
            );
            assert_eq!(trace_a.signed_distance(), 0.05);
            assert_eq!(trace_b.signed_distance(), 0.02);
            assert!(
                certificate
                    .residual_bounds()
                    .into_iter()
                    .all(|bound| bound <= tolerances.linear())
            );
            let cost = verified_dual_offset_nurbs_intersection_certificate_cost(
                certificate.carrier(),
                certificate.traces(),
            )
            .unwrap();
            assert_eq!(
                (cost.work(), cost.items(), cost.depth()),
                (14_336, 1_024, 10)
            );
            for (resource, expected) in [
                (ResourceKind::Work, cost.work()),
                (ResourceKind::Items, cost.items()),
                (ResourceKind::Depth, cost.depth()),
            ] {
                assert_eq!(
                    observed(outcome.report(), NURBS_TRACE_CERTIFICATE_WORK, resource),
                    expected
                );
            }

            if distances_a.len() == 4 && distances_b.len() == 4 {
                assert_eq!((expected_visits, expected_depth), (10, 5));
                let reverse = intersect_bounded_graph_surfaces(
                    &graph,
                    root_b,
                    dual_window_b(),
                    root_a,
                    dual_window_a(),
                    tolerances,
                )
                .unwrap();
                assert_eq!(reverse.raw, result.raw.clone().swapped());
                assert_eq!(reverse.branch_graph.source_surfaces, [root_b, root_a]);
                let reverse_certificate = reverse.branch_graph.edges[0]
                    .certificate
                    .as_nurbs()
                    .unwrap();
                assert_eq!(reverse_certificate.traces()[0], certificate.traces()[1]);
                assert_eq!(reverse_certificate.traces()[1], certificate.traces()[0]);

                let exact_plan = BudgetPlan::new([
                    LimitSpec::new(
                        kgraph::eval_stage::NODE_VISITS,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        expected_visits,
                    ),
                    LimitSpec::new(
                        kgraph::eval_stage::DEPENDENCY_DEPTH,
                        ResourceKind::Depth,
                        AccountingMode::HighWater,
                        expected_depth,
                    ),
                    LimitSpec::new(
                        NURBS_TRACE_CERTIFICATE_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        cost.work(),
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
                        root_a,
                        dual_window_a(),
                        root_b,
                        dual_window_b(),
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
                        expected_visits - 1,
                        expected_visits,
                    ),
                    (
                        kgraph::eval_stage::DEPENDENCY_DEPTH,
                        ResourceKind::Depth,
                        AccountingMode::HighWater,
                        expected_depth - 1,
                        expected_depth,
                    ),
                    (
                        NURBS_TRACE_CERTIFICATE_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        cost.work() - 1,
                        cost.work(),
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
                        root_a,
                        dual_window_a(),
                        root_b,
                        dual_window_b(),
                        &denied_context,
                    );
                    let GraphSurfaceIntersectionError::OperationPolicy(
                        kcore::operation::OperationPolicyError::LimitReached(crossing),
                    ) = denied.result().unwrap_err()
                    else {
                        panic!("N-1 maximum-depth dual-offset resource must stop at {stage:?}");
                    };
                    assert_eq!(crossing.stage, stage);
                    assert_eq!(crossing.resource, resource);
                    assert_eq!(crossing.allowed, allowed);
                    assert_eq!(crossing.consumed, consumed);
                    assert_eq!(graph.curve_count(), before.0);
                    assert_eq!(graph.curve2d_count(), before.1);
                    assert_eq!(graph.geometry().collect::<Vec<_>>(), before.2);
                }

                let persistent =
                    persist_verified_graph_surface_intersections(&mut graph, result).unwrap();
                let descriptor = graph
                    .curve(persistent.edges[0].curve)
                    .unwrap()
                    .as_verified_nurbs_intersection()
                    .unwrap();
                assert_eq!(descriptor.source_surfaces(), [root_a, root_b]);
                assert_eq!(descriptor.certificate().traces(), certificate.traces());
                for protected in chain_a.into_iter().chain(chain_b) {
                    assert!(matches!(
                        graph.remove_surface(protected),
                        Err(GeometryGraphError::HasDependents { .. })
                    ));
                }
                graph.validate().unwrap();
            }
        }
    }
}

#[test]
fn rational_dual_offset_hits_retain_independent_sources_and_swap() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for depths in [(1, 4), (4, 1)] {
        let mut graph = GeometryGraph::new();
        let distances_a = vec![0.05 / depths.0 as f64; depths.0];
        let distances_b = vec![0.02 / depths.1 as f64; depths.1];
        let (_, root_a, _) = insert_offset_chain(&mut graph, basis(true), &distances_a);
        let (_, root_b, _) = insert_offset_chain(
            &mut graph,
            crossing_constant_normal_basis(true),
            &distances_b,
        );
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            root_a,
            dual_window_a(),
            root_b,
            dual_window_b(),
            tolerances,
        )
        .unwrap();
        let reverse = intersect_bounded_graph_surfaces(
            &graph,
            root_b,
            dual_window_b(),
            root_a,
            dual_window_a(),
            tolerances,
        )
        .unwrap();
        assert_eq!(forward.raw.curves.len(), 1);
        assert_eq!(reverse.raw, forward.raw.clone().swapped());
        let forward_certificate = forward.branch_graph.edges[0]
            .certificate
            .as_nurbs()
            .unwrap();
        let reverse_certificate = reverse.branch_graph.edges[0]
            .certificate
            .as_nurbs()
            .unwrap();
        assert_eq!(
            forward_certificate.traces()[0]
                .as_offset_nurbs()
                .unwrap()
                .basis()
                .weights(),
            Some(&[2.0; 6][..])
        );
        assert_eq!(
            reverse_certificate.traces()[0],
            forward_certificate.traces()[1]
        );
        assert_eq!(
            reverse_certificate.traces()[1],
            forward_certificate.traces()[0]
        );
    }
}

#[test]
fn stale_or_altered_dual_offset_roots_roll_persistence_back_atomically() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let distances_a = [0.005, 0.01, 0.015, 0.02];
    let distances_b = [0.002, 0.004, 0.006, 0.008];
    for mutation in 0..4 {
        let mut graph = GeometryGraph::new();
        let (_, root_a, chain_a) = insert_offset_chain(&mut graph, basis(false), &distances_a);
        let (_, root_b, chain_b) = insert_offset_chain(
            &mut graph,
            crossing_constant_normal_basis(false),
            &distances_b,
        );
        let local = intersect_bounded_graph_surfaces(
            &graph,
            root_a,
            dual_window_a(),
            root_b,
            dual_window_b(),
            tolerances,
        )
        .unwrap();
        match mutation {
            0 => graph.remove_surface(root_a).unwrap(),
            1 => graph.remove_surface(root_b).unwrap(),
            2 => graph
                .replace_surface(
                    root_a,
                    OffsetSurfaceDescriptor::new(chain_a[chain_a.len() - 2], 0.021),
                )
                .unwrap(),
            3 => graph
                .replace_surface(
                    root_b,
                    OffsetSurfaceDescriptor::new(chain_b[chain_b.len() - 2], 0.009),
                )
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
fn outward_offset_control_proof_returns_a_complete_miss() {
    let signed_distance = 0.05;
    let basis = basis(true);
    let direct = unit_chart_surface([0.7, 0.72, 0.7], true);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let lower = intersect_bounded_offset_nurbs_nurbs_surfaces(
        &basis,
        signed_distance,
        basis.param_range(),
        &direct,
        direct.param_range(),
        tolerances,
    )
    .unwrap();
    assert!(lower.is_proven_empty());

    let mut graph = GeometryGraph::new();
    let basis_handle = graph.insert_surface(basis.clone()).unwrap();
    let mut offset_handle = basis_handle;
    for distance in [0.005, 0.01, 0.015, 0.02] {
        offset_handle = graph
            .insert_surface(OffsetSurfaceDescriptor::new(offset_handle, distance))
            .unwrap();
    }
    let direct_handle = graph.insert_surface(direct.clone()).unwrap();
    let miss = intersect_bounded_graph_surfaces(
        &graph,
        offset_handle,
        basis.param_range(),
        direct_handle,
        direct.param_range(),
        tolerances,
    )
    .unwrap();
    assert_eq!(miss.raw, lower);
    assert!(miss.raw.is_proven_empty());
    assert!(miss.branch_graph.vertices.is_empty());
    assert!(miss.branch_graph.edges.is_empty());
}

#[test]
fn separated_constant_normal_dual_offsets_prove_a_complete_miss_in_both_orders() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for rational in [false, true] {
        let mut graph = GeometryGraph::new();
        let basis_a = graph
            .insert_surface(unit_chart_surface([0.25; 3], rational))
            .unwrap();
        let offset_a = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis_a, 0.05))
            .unwrap();
        let basis_b = graph
            .insert_surface(unit_chart_surface([0.70; 3], rational))
            .unwrap();
        let offset_b = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis_b, -0.10))
            .unwrap();

        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, tolerances).unwrap();
        let outcome = intersect_bounded_graph_surfaces_with_context(
            &graph,
            offset_a,
            narrow_window(),
            offset_b,
            narrow_window(),
            &context,
        );
        let result = outcome.result().unwrap();
        assert!(result.raw.is_proven_empty());
        assert_eq!(result.branch_graph.source_surfaces, [offset_a, offset_b]);
        assert!(result.branch_graph.vertices.is_empty());
        assert!(result.branch_graph.edges.is_empty());
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

        let reverse = intersect_bounded_graph_surfaces(
            &graph,
            offset_b,
            narrow_window(),
            offset_a,
            narrow_window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(reverse.raw, result.raw.clone().swapped());
        assert_eq!(reverse.branch_graph.source_surfaces, [offset_b, offset_a]);
        assert!(reverse.branch_graph.edges.is_empty());

        let unequal_window = [ParamRange::new(0.0, 0.5), narrow_window()[1]];
        assert!(matches!(
            intersect_bounded_graph_surfaces(
                &graph,
                offset_a,
                narrow_window(),
                offset_b,
                unequal_window,
                tolerances,
            ),
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::UnsupportedSurfacePair { .. }
            ))
        ));
    }
}

#[test]
fn one_to_four_descriptor_dual_offset_misses_pin_matrix_accounting_and_no_artifacts() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();
    let distance_chains_a = [
        &[0.05][..],
        &[0.02, 0.03][..],
        &[0.01, 0.015, 0.025][..],
        &[0.005, 0.01, 0.015, 0.02][..],
    ];
    let distance_chains_b = [
        &[-0.10][..],
        &[-0.04, -0.06][..],
        &[-0.02, -0.03, -0.05][..],
        &[-0.01, -0.02, -0.03, -0.04][..],
    ];
    for distances_a in distance_chains_a {
        for distances_b in distance_chains_b {
            let mut graph = GeometryGraph::new();
            let insert_root =
                |graph: &mut GeometryGraph, surface: NurbsSurface, distances: &[f64]| {
                    let mut root = graph.insert_surface(surface).unwrap();
                    for &distance in distances {
                        root = graph
                            .insert_surface(OffsetSurfaceDescriptor::new(root, distance))
                            .unwrap();
                    }
                    root
                };
            let root_a = insert_root(
                &mut graph,
                unit_chart_surface([0.25; 3], false),
                distances_a,
            );
            let root_b = insert_root(
                &mut graph,
                unit_chart_surface([0.70; 3], false),
                distances_b,
            );
            let expected_visits = (distances_a.len() + distances_b.len() + 2) as u64;
            let expected_depth = (distances_a.len().max(distances_b.len()) + 1) as u64;
            let context = OperationContext::new(&session, tolerances).unwrap();
            let outcome = intersect_bounded_graph_surfaces_with_context(
                &graph,
                root_a,
                narrow_window(),
                root_b,
                narrow_window(),
                &context,
            );
            let result = outcome.result().unwrap();
            assert!(result.raw.is_proven_empty());
            assert_eq!(result.branch_graph.source_surfaces, [root_a, root_b]);
            assert!(result.branch_graph.vertices.is_empty());
            assert!(result.branch_graph.edges.is_empty());
            assert_eq!(
                observed(
                    outcome.report(),
                    kgraph::eval_stage::NODE_VISITS,
                    ResourceKind::Work,
                ),
                expected_visits
            );
            assert_eq!(
                observed(
                    outcome.report(),
                    kgraph::eval_stage::DEPENDENCY_DEPTH,
                    ResourceKind::Depth,
                ),
                expected_depth
            );
            for resource in [ResourceKind::Work, ResourceKind::Items, ResourceKind::Depth] {
                assert_eq!(
                    observed(outcome.report(), NURBS_TRACE_CERTIFICATE_WORK, resource),
                    0
                );
            }

            let reverse_outcome = intersect_bounded_graph_surfaces_with_context(
                &graph,
                root_b,
                narrow_window(),
                root_a,
                narrow_window(),
                &context,
            );
            let reverse = reverse_outcome.result().unwrap();
            assert_eq!(reverse.raw, result.raw.clone().swapped());
            assert_eq!(reverse.branch_graph.source_surfaces, [root_b, root_a]);
            assert!(reverse.branch_graph.edges.is_empty());
            assert_eq!(
                observed(
                    reverse_outcome.report(),
                    kgraph::eval_stage::NODE_VISITS,
                    ResourceKind::Work,
                ),
                expected_visits
            );
            assert_eq!(
                observed(
                    reverse_outcome.report(),
                    kgraph::eval_stage::DEPENDENCY_DEPTH,
                    ResourceKind::Depth,
                ),
                expected_depth
            );
            for resource in [ResourceKind::Work, ResourceKind::Items, ResourceKind::Depth] {
                assert_eq!(
                    observed(
                        reverse_outcome.report(),
                        NURBS_TRACE_CERTIFICATE_WORK,
                        resource,
                    ),
                    0
                );
            }

            if distances_a.len() == 4 && distances_b.len() == 4 {
                assert_eq!((expected_visits, expected_depth), (10, 5));
                let exact_plan = BudgetPlan::new([
                    LimitSpec::new(
                        kgraph::eval_stage::NODE_VISITS,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        expected_visits,
                    ),
                    LimitSpec::new(
                        kgraph::eval_stage::DEPENDENCY_DEPTH,
                        ResourceKind::Depth,
                        AccountingMode::HighWater,
                        expected_depth,
                    ),
                ])
                .unwrap();
                let exact_context = OperationContext::new(&session, tolerances)
                    .unwrap()
                    .with_budget_overrides(exact_plan);
                assert!(
                    intersect_bounded_graph_surfaces_with_context(
                        &graph,
                        root_a,
                        narrow_window(),
                        root_b,
                        narrow_window(),
                        &exact_context,
                    )
                    .result()
                    .unwrap()
                    .raw
                    .is_proven_empty()
                );

                for (stage, resource, mode, allowed, consumed) in [
                    (
                        kgraph::eval_stage::NODE_VISITS,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        expected_visits - 1,
                        expected_visits,
                    ),
                    (
                        kgraph::eval_stage::DEPENDENCY_DEPTH,
                        ResourceKind::Depth,
                        AccountingMode::HighWater,
                        expected_depth - 1,
                        expected_depth,
                    ),
                ] {
                    let denied_plan =
                        BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap();
                    let denied_context = OperationContext::new(&session, tolerances)
                        .unwrap()
                        .with_budget_overrides(denied_plan);
                    let denied = intersect_bounded_graph_surfaces_with_context(
                        &graph,
                        root_a,
                        narrow_window(),
                        root_b,
                        narrow_window(),
                        &denied_context,
                    );
                    let GraphSurfaceIntersectionError::OperationPolicy(
                        kcore::operation::OperationPolicyError::LimitReached(crossing),
                    ) = denied.result().unwrap_err()
                    else {
                        panic!("N-1 four-descriptor dual-offset resource must stop at {stage:?}");
                    };
                    assert_eq!(crossing.stage, stage);
                    assert_eq!(crossing.resource, resource);
                    assert_eq!(crossing.allowed, allowed);
                    assert_eq!(crossing.consumed, consumed);
                }
            }

            let before = (
                graph.curve_count(),
                graph.curve2d_count(),
                graph.geometry().collect::<Vec<_>>(),
            );
            let persistent =
                persist_verified_graph_surface_intersections(&mut graph, result).unwrap();
            assert!(persistent.edges.is_empty());
            assert_eq!(graph.curve_count(), before.0);
            assert_eq!(graph.curve2d_count(), before.1);
            assert_eq!(graph.geometry().collect::<Vec<_>>(), before.2);
            graph.validate().unwrap();
        }
    }
}

#[test]
fn dual_offset_miss_pins_graph_resource_boundaries() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let mut graph = GeometryGraph::new();
    let basis_a = graph.insert_surface(basis(false)).unwrap();
    let offset_a = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_a, 0.05))
        .unwrap();
    let basis_b = graph
        .insert_surface(unit_chart_surface([0.70; 3], false))
        .unwrap();
    let offset_b = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_b, -0.10))
        .unwrap();
    let session = SessionPolicy::v1();

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
    ])
    .unwrap();
    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(exact_plan);
    assert!(
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            offset_a,
            narrow_window(),
            offset_b,
            narrow_window(),
            &exact_context,
        )
        .result()
        .unwrap()
        .raw
        .is_proven_empty()
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
    ] {
        let denied_plan =
            BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap();
        let denied_context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(denied_plan);
        let denied = intersect_bounded_graph_surfaces_with_context(
            &graph,
            offset_a,
            narrow_window(),
            offset_b,
            narrow_window(),
            &denied_context,
        );
        let GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(crossing),
        ) = denied.result().unwrap_err()
        else {
            panic!("N-1 graph resource must stop at {stage:?}");
        };
        assert_eq!(crossing.stage, stage);
        assert_eq!(crossing.resource, resource);
        assert_eq!(crossing.allowed, allowed);
        assert_eq!(crossing.consumed, consumed);
    }
}

#[test]
fn stale_and_altered_offset_sources_roll_back_atomically() {
    let signed_distance = 0.05;
    let original_basis = basis(false);
    let direct = crossing_direct(0.25 + signed_distance, false);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for mutation in 0..4 {
        let mut graph = GeometryGraph::new();
        let basis_handle = graph.insert_surface(original_basis.clone()).unwrap();
        let offset_handle = graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis_handle, signed_distance))
            .unwrap();
        let direct_handle = graph.insert_surface(direct.clone()).unwrap();
        let local = intersect_bounded_graph_surfaces(
            &graph,
            offset_handle,
            overlapping_offset_window(),
            direct_handle,
            overlapping_direct_window(),
            tolerances,
        )
        .unwrap();
        match mutation {
            0 => graph.remove_surface(offset_handle).unwrap(),
            1 => graph
                .replace_surface(
                    offset_handle,
                    OffsetSurfaceDescriptor::new(basis_handle, signed_distance + 0.001),
                )
                .unwrap(),
            2 => graph.remove_surface(direct_handle).unwrap(),
            3 => graph
                .replace_surface(
                    direct_handle,
                    crossing_direct(0.25 + signed_distance + 0.001, false),
                )
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

    // Reuse the operation-local proof in a graph with the same live handles
    // but a different basis value. Persistence must inspect the basis identity,
    // not merely the root offset handle and distance.
    let mut original = GeometryGraph::new();
    let original_basis_handle = original.insert_surface(original_basis.clone()).unwrap();
    let original_offset = original
        .insert_surface(OffsetSurfaceDescriptor::new(
            original_basis_handle,
            signed_distance,
        ))
        .unwrap();
    let original_direct = original.insert_surface(direct.clone()).unwrap();
    let local = intersect_bounded_graph_surfaces(
        &original,
        original_offset,
        overlapping_offset_window(),
        original_direct,
        overlapping_direct_window(),
        tolerances,
    )
    .unwrap();

    let mut altered = GeometryGraph::new();
    let altered_basis = altered
        .insert_surface(unit_chart_surface([0.251; 3], false))
        .unwrap();
    let altered_offset = altered
        .insert_surface(OffsetSurfaceDescriptor::new(altered_basis, signed_distance))
        .unwrap();
    let altered_direct = altered.insert_surface(direct).unwrap();
    assert_eq!(altered_basis, original_basis_handle);
    assert_eq!(altered_offset, original_offset);
    assert_eq!(altered_direct, original_direct);
    let before = altered.geometry().collect::<Vec<_>>();
    assert!(matches!(
        persist_verified_graph_surface_intersections(&mut altered, &local),
        Err(GraphSurfaceIntersectionError::GeometryPersistence(
            GeometryGraphError::InvalidDescriptor { .. }
        ))
    ));
    assert_eq!(altered.curve_count(), 0);
    assert_eq!(altered.curve2d_count(), 0);
    assert_eq!(altered.geometry().collect::<Vec<_>>(), before);
    altered.validate().unwrap();
}

#[test]
fn broader_offset_families_and_unaligned_charts_fail_closed() {
    let signed_distance = 0.05;
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let planar_basis = basis(false);
    let direct = crossing_direct(0.25 + signed_distance, false);
    let curved_basis = unit_chart_surface([0.25, 0.27, 0.25], false);
    let mut warped_points = crossing_constant_normal_basis(false).points().to_vec();
    warped_points[2].z += 1.0e-10;
    let slightly_warped_basis = NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        warped_points,
        None,
    )
    .unwrap();
    let mut unaligned_points = direct.points().to_vec();
    unaligned_points[0].x += 0.01;
    let unaligned = NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        unaligned_points,
        None,
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let planar_basis_handle = graph.insert_surface(planar_basis).unwrap();
    let offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(
            planar_basis_handle,
            signed_distance,
        ))
        .unwrap();
    let nested = graph
        .insert_surface(OffsetSurfaceDescriptor::new(offset, 0.01))
        .unwrap();
    let capped = graph
        .insert_surface(OffsetSurfaceDescriptor::new(nested, 0.01))
        .unwrap();
    let supported_four = graph
        .insert_surface(OffsetSurfaceDescriptor::new(capped, 0.01))
        .unwrap();
    let too_deep = graph
        .insert_surface(OffsetSurfaceDescriptor::new(supported_four, 0.01))
        .unwrap();
    let direct_handle = graph.insert_surface(direct).unwrap();
    let curved_basis_handle = graph.insert_surface(curved_basis).unwrap();
    let curved_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(
            curved_basis_handle,
            signed_distance,
        ))
        .unwrap();
    let curved_nested = graph
        .insert_surface(OffsetSurfaceDescriptor::new(curved_offset, 0.01))
        .unwrap();
    let curved_capped = graph
        .insert_surface(OffsetSurfaceDescriptor::new(curved_nested, 0.01))
        .unwrap();
    let curved_four = graph
        .insert_surface(OffsetSurfaceDescriptor::new(curved_capped, 0.01))
        .unwrap();
    let slightly_warped_basis = graph.insert_surface(slightly_warped_basis).unwrap();
    let slightly_warped_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(slightly_warped_basis, 0.02))
        .unwrap();
    let coincident_basis = graph.insert_surface(basis(false)).unwrap();
    let coincident_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(
            coincident_basis,
            signed_distance,
        ))
        .unwrap();
    let nested_coincident_basis = graph.insert_surface(basis(false)).unwrap();
    let nested_coincident_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(
            nested_coincident_basis,
            signed_distance + 0.01,
        ))
        .unwrap();
    let capped_coincident_basis = graph.insert_surface(basis(false)).unwrap();
    let capped_coincident_inner = graph
        .insert_surface(OffsetSurfaceDescriptor::new(
            capped_coincident_basis,
            signed_distance,
        ))
        .unwrap();
    let capped_coincident_middle = graph
        .insert_surface(OffsetSurfaceDescriptor::new(capped_coincident_inner, 0.01))
        .unwrap();
    let capped_coincident_root = graph
        .insert_surface(OffsetSurfaceDescriptor::new(capped_coincident_middle, 0.01))
        .unwrap();
    let four_coincident_root = graph
        .insert_surface(OffsetSurfaceDescriptor::new(capped_coincident_root, 0.01))
        .unwrap();
    let separated_basis = graph
        .insert_surface(unit_chart_surface([0.70; 3], false))
        .unwrap();
    let separated_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(separated_basis, -0.10))
        .unwrap();
    let rational_basis = graph
        .insert_surface(unit_chart_surface([0.70; 3], true))
        .unwrap();
    let unequal_weight_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(rational_basis, -0.10))
        .unwrap();
    let intersecting_peer_basis = graph
        .insert_surface(crossing_constant_normal_basis(false))
        .unwrap();
    let intersecting_peer = graph
        .insert_surface(OffsetSurfaceDescriptor::new(intersecting_peer_basis, 0.02))
        .unwrap();
    let boundary_peer_basis = graph
        .insert_surface(unit_chart_surface([-0.1, 0.1, 0.3], false))
        .unwrap();
    let boundary_peer = graph
        .insert_surface(OffsetSurfaceDescriptor::new(boundary_peer_basis, 0.0))
        .unwrap();
    let unaligned = graph.insert_surface(unaligned).unwrap();
    for (first, second) in [
        (too_deep, direct_handle),
        (direct_handle, too_deep),
        (curved_offset, direct_handle),
        (direct_handle, curved_offset),
        (curved_four, direct_handle),
        (direct_handle, curved_four),
        (offset, unaligned),
        (unaligned, offset),
        (supported_four, unaligned),
        (unaligned, supported_four),
        (offset, curved_offset),
        (supported_four, curved_four),
        (curved_four, supported_four),
        (offset, slightly_warped_offset),
        (slightly_warped_offset, offset),
        (offset, coincident_offset),
        (coincident_offset, offset),
        (nested, nested_coincident_offset),
        (nested_coincident_offset, nested),
        (capped, capped_coincident_root),
        (capped_coincident_root, capped),
        (supported_four, four_coincident_root),
        (four_coincident_root, supported_four),
        (too_deep, separated_offset),
        (separated_offset, too_deep),
        (too_deep, intersecting_peer),
        (intersecting_peer, too_deep),
        (offset, boundary_peer),
        (boundary_peer, offset),
        (offset, unequal_weight_offset),
        (unequal_weight_offset, offset),
    ] {
        assert!(matches!(
            intersect_bounded_graph_surfaces(
                &graph,
                first,
                narrow_window(),
                second,
                narrow_window(),
                tolerances,
            ),
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::UnsupportedSurfacePair { .. }
            ))
        ));
    }

    for (offset_window, direct_window) in [
        (
            [ParamRange::new(0.0, 0.4), narrow_window()[1]],
            [ParamRange::new(0.6, 1.0), narrow_window()[1]],
        ),
        (
            [ParamRange::new(0.0, 0.5), narrow_window()[1]],
            [ParamRange::new(0.5, 1.0), narrow_window()[1]],
        ),
    ] {
        assert!(matches!(
            intersect_bounded_graph_surfaces(
                &graph,
                offset,
                offset_window,
                direct_handle,
                direct_window,
                tolerances,
            ),
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::UnsupportedSurfacePair { .. }
            ))
        ));
    }
}
