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

fn narrow_window() -> [ParamRange; 2] {
    [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 0.0015)]
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
fn direct_constant_normal_offset_promotes_in_both_orders_and_persists_identities() {
    let signed_distance = 0.05;
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for rational in [false, true] {
        let basis = basis(rational);
        let direct = crossing_direct(0.25 + signed_distance, rational);
        let lower = intersect_bounded_offset_nurbs_nurbs_surfaces(
            &basis,
            signed_distance,
            narrow_window(),
            &direct,
            narrow_window(),
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
            narrow_window(),
            direct_handle,
            narrow_window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(result.raw, lower);
        assert_eq!(
            result.branch_graph.source_surfaces,
            [offset_handle, direct_handle]
        );
        assert_eq!(result.branch_graph.edges.len(), 1);
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
            narrow_window(),
            offset_handle,
            narrow_window(),
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
        narrow_window(),
        &direct,
        narrow_window(),
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
        narrow_window(),
        direct_handle,
        narrow_window(),
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
            narrow_window(),
            direct_handle,
            narrow_window(),
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
            narrow_window(),
            direct_handle,
            narrow_window(),
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
    let offset_handle = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_handle, signed_distance))
        .unwrap();
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
            narrow_window(),
            direct_handle,
            narrow_window(),
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
        narrow_window(),
        original_direct,
        narrow_window(),
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
    let direct_handle = graph.insert_surface(direct).unwrap();
    let curved_basis_handle = graph.insert_surface(curved_basis).unwrap();
    let curved_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(
            curved_basis_handle,
            signed_distance,
        ))
        .unwrap();
    let coincident_basis = graph.insert_surface(basis(false)).unwrap();
    let coincident_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(
            coincident_basis,
            signed_distance,
        ))
        .unwrap();
    let rational_basis = graph
        .insert_surface(unit_chart_surface([0.70; 3], true))
        .unwrap();
    let unequal_weight_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(rational_basis, -0.10))
        .unwrap();
    let unaligned = graph.insert_surface(unaligned).unwrap();
    for (first, second) in [
        (nested, direct_handle),
        (direct_handle, nested),
        (curved_offset, direct_handle),
        (direct_handle, curved_offset),
        (offset, unaligned),
        (unaligned, offset),
        (offset, curved_offset),
        (offset, coincident_offset),
        (coincident_offset, offset),
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
}
