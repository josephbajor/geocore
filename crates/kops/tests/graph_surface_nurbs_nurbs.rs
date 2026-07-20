//! Contextual graph-owned direct NURBS/NURBS branch contracts.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use kgraph::{
    GeometryGraph, GeometryGraphError, NurbsIntersectionTrace, OffsetSurfaceDescriptor,
    verified_nurbs_nurbs_intersection_certificate_cost,
};
use kops::intersect::{
    GraphSurfaceIntersectionError, IntersectionError, NURBS_SURFACE_MARCH_SAMPLES,
    NURBS_TRACE_CERTIFICATE_WORK, NurbsSurfaceMarchBudgetProfile, intersect_bounded_graph_surfaces,
    intersect_bounded_graph_surfaces_with_context, intersect_bounded_nurbs_nurbs_surfaces,
    intersect_bounded_nurbs_nurbs_surfaces_with_context,
    persist_verified_graph_surface_intersections,
};

fn paired_surface(delta_controls: [f64; 3], height: f64, rational: bool) -> NurbsSurface {
    let coordinates = [0.0, 0.5, 1.0];
    let bend = [0.0, 0.02, 0.0];
    let mut points = Vec::with_capacity(6);
    for (u_index, &u) in coordinates.iter().enumerate() {
        for &v in &[0.0, 1.0] {
            points.push(Point3::new(
                u,
                v,
                height + bend[u_index] + delta_controls[u_index],
            ));
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

fn narrow_window() -> [ParamRange; 2] {
    [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 0.0015)]
}

fn overlapping_window_a() -> [ParamRange; 2] {
    [ParamRange::new(0.0, 0.8), ParamRange::new(0.0, 0.0015)]
}

fn overlapping_window_b() -> [ParamRange; 2] {
    [ParamRange::new(0.2, 1.0), ParamRange::new(0.0005, 0.0015)]
}

fn planar_surface() -> NurbsSurface {
    let mut points = Vec::with_capacity(6);
    for &u in &[0.0, 0.5, 1.0] {
        for &v in &[0.0, 1.0] {
            points.push(Point3::new(u, v, 0.25));
        }
    }
    NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        points,
        None,
    )
    .unwrap()
}

#[test]
fn direct_nurbs_nurbs_promotes_a_paired_whole_range_trace() {
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for rational in [false, true] {
        let surface_a = paired_surface([0.0; 3], 0.25, rational);
        let surface_b = paired_surface([-0.2, -0.0148, 0.1704], 0.25, rational);
        let lower = intersect_bounded_nurbs_nurbs_surfaces(
            &surface_a,
            overlapping_window_a(),
            &surface_b,
            overlapping_window_b(),
            tolerances,
        )
        .unwrap();
        assert!(!lower.is_complete());
        assert_eq!(lower.curves.len(), 1);

        let mut graph = GeometryGraph::new();
        let source_a = graph.insert_surface(surface_a.clone()).unwrap();
        let source_b = graph.insert_surface(surface_b.clone()).unwrap();
        let result = intersect_bounded_graph_surfaces(
            &graph,
            source_a,
            overlapping_window_a(),
            source_b,
            overlapping_window_b(),
            tolerances,
        )
        .unwrap();
        assert_eq!(result.raw, lower);
        assert_eq!(result.branch_graph.source_surfaces, [source_a, source_b]);
        assert_eq!(result.branch_graph.edges.len(), 1);
        let raw_branch = &result.raw.curves[0];
        assert_eq!(raw_branch.uv_a_start[1], 0.0005);
        assert_eq!(raw_branch.uv_b_start[1], 0.0005);
        assert_eq!(raw_branch.uv_a_end[1], 0.0015);
        assert_eq!(raw_branch.uv_b_end[1], 0.0015);
        assert_eq!(
            result.branch_graph.edges[0].topology,
            kops::intersect::IntersectionBranchTopology::Open
        );
        assert_eq!(
            result.branch_graph.edges[0]
                .endpoint_events
                .map(|event| match event {
                    kops::intersect::IntersectionBranchEndpointEvent::SurfaceWindowBoundary {
                        surfaces,
                    } => surfaces,
                    kops::intersect::IntersectionBranchEndpointEvent::PeriodSeam { .. } => {
                        panic!("an open NURBS branch cannot end at a periodic seam")
                    }
                }),
            [[true, true], [true, true]]
        );
        let certificate = result.branch_graph.edges[0].certificate.as_nurbs().unwrap();
        assert!(matches!(
            certificate.traces(),
            [
                NurbsIntersectionTrace::Nurbs(_),
                NurbsIntersectionTrace::Nurbs(_)
            ]
        ));
        assert_eq!(certificate.traces()[0].as_nurbs(), Some(&surface_a));
        assert_eq!(certificate.traces()[1].as_nurbs(), Some(&surface_b));
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= tolerances.linear())
        );

        let reverse = intersect_bounded_graph_surfaces(
            &graph,
            source_b,
            overlapping_window_b(),
            source_a,
            overlapping_window_a(),
            tolerances,
        )
        .unwrap();
        assert_eq!(reverse.raw, result.raw.clone().swapped());
        assert_eq!(reverse.branch_graph.source_surfaces, [source_b, source_a]);
        let reverse_certificate = reverse.branch_graph.edges[0]
            .certificate
            .as_nurbs()
            .unwrap();
        assert_eq!(reverse_certificate.traces()[0].as_nurbs(), Some(&surface_b));
        assert_eq!(reverse_certificate.traces()[1].as_nurbs(), Some(&surface_a));

        let identical_lower = intersect_bounded_nurbs_nurbs_surfaces(
            &surface_a,
            narrow_window(),
            &surface_b,
            narrow_window(),
            tolerances,
        )
        .unwrap();
        let identical = intersect_bounded_graph_surfaces(
            &graph,
            source_a,
            narrow_window(),
            source_b,
            narrow_window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(identical.raw, identical_lower);
        assert_eq!(identical.branch_graph.edges.len(), 1);
        let identical_certificate = identical.branch_graph.edges[0]
            .certificate
            .as_nurbs()
            .unwrap();
        assert_eq!(
            identical_certificate.traces()[0].as_nurbs(),
            Some(&surface_a)
        );
        assert_eq!(
            identical_certificate.traces()[1].as_nurbs(),
            Some(&surface_b)
        );

        let persistent = persist_verified_graph_surface_intersections(&mut graph, &result).unwrap();
        let descriptor = graph
            .curve(persistent.edges[0].curve)
            .unwrap()
            .as_verified_nurbs_intersection()
            .unwrap();
        assert_eq!(descriptor.source_surfaces(), [source_a, source_b]);
        assert_eq!(descriptor.certificate(), certificate);
        assert!(matches!(
            graph.remove_surface(source_a),
            Err(GeometryGraphError::HasDependents { .. })
        ));
        assert!(matches!(
            graph.remove_surface(source_b),
            Err(GeometryGraphError::HasDependents { .. })
        ));
        graph.validate().unwrap();
    }
}

#[test]
fn scoped_nurbs_nurbs_preserves_raw_report_and_exact_certificate_boundaries() {
    let surface_a = paired_surface([0.0; 3], 0.25, false);
    let surface_b = paired_surface([-0.2, -0.0148, 0.1704], 0.25, false);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();
    let lower_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_family_budget_defaults(NurbsSurfaceMarchBudgetProfile::v1_defaults());
    let lower = intersect_bounded_nurbs_nurbs_surfaces_with_context(
        &surface_a,
        overlapping_window_a(),
        &surface_b,
        overlapping_window_b(),
        &lower_context,
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let source_a = graph.insert_surface(surface_a).unwrap();
    let source_b = graph.insert_surface(surface_b).unwrap();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        source_a,
        overlapping_window_a(),
        source_b,
        overlapping_window_b(),
        &context,
    );
    let result = outcome.result().unwrap();
    assert_eq!(&result.raw, *lower.result().as_ref().unwrap());
    assert_eq!(
        observed(
            outcome.report(),
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
        ),
        0
    );
    assert_eq!(
        observed(
            outcome.report(),
            kgraph::eval_stage::DEPENDENCY_DEPTH,
            ResourceKind::Depth,
        ),
        0
    );
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

    let certificate = result.branch_graph.edges[0].certificate.as_nurbs().unwrap();
    let cost = verified_nurbs_nurbs_intersection_certificate_cost(
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
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        source_a,
        overlapping_window_a(),
        source_b,
        overlapping_window_b(),
        &exact_context,
    );
    assert!(exact.result().is_ok());
    for (resource, expected) in [
        (ResourceKind::Work, cost.work()),
        (ResourceKind::Items, cost.items()),
        (ResourceKind::Depth, cost.depth()),
    ] {
        assert_eq!(
            observed(exact.report(), NURBS_TRACE_CERTIFICATE_WORK, resource),
            expected
        );
    }

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
            source_a,
            overlapping_window_a(),
            source_b,
            overlapping_window_b(),
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
fn outward_original_control_proof_returns_a_complete_miss() {
    let surface_a = paired_surface([0.0; 3], 0.25, true);
    let surface_b = paired_surface([0.2; 3], 0.25, true);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let lower = intersect_bounded_nurbs_nurbs_surfaces(
        &surface_a,
        overlapping_window_a(),
        &surface_b,
        overlapping_window_b(),
        tolerances,
    )
    .unwrap();
    assert!(lower.is_proven_empty());

    let mut graph = GeometryGraph::new();
    let source_a = graph.insert_surface(surface_a.clone()).unwrap();
    let source_b = graph.insert_surface(surface_b.clone()).unwrap();
    let miss = intersect_bounded_graph_surfaces(
        &graph,
        source_a,
        overlapping_window_a(),
        source_b,
        overlapping_window_b(),
        tolerances,
    )
    .unwrap();
    assert_eq!(miss.raw, lower);
    assert!(miss.raw.is_proven_empty());
    assert_eq!(miss.branch_graph.source_surfaces, [source_a, source_b]);
    assert!(miss.branch_graph.vertices.is_empty());
    assert!(miss.branch_graph.edges.is_empty());
}

#[test]
fn stale_and_altered_nurbs_sources_roll_back_atomically() {
    let surface_a = paired_surface([0.0; 3], 0.25, false);
    let surface_b = paired_surface([-0.2, -0.0148, 0.1704], 0.25, false);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for mutation in 0..4 {
        let mut graph = GeometryGraph::new();
        let source_a = graph.insert_surface(surface_a.clone()).unwrap();
        let source_b = graph.insert_surface(surface_b.clone()).unwrap();
        let local = intersect_bounded_graph_surfaces(
            &graph,
            source_a,
            overlapping_window_a(),
            source_b,
            overlapping_window_b(),
            tolerances,
        )
        .unwrap();
        match mutation {
            0 => graph.remove_surface(source_a).unwrap(),
            1 => graph
                .replace_surface(source_a, paired_surface([0.0; 3], 0.251, false))
                .unwrap(),
            2 => graph.remove_surface(source_b).unwrap(),
            3 => graph
                .replace_surface(
                    source_b,
                    paired_surface([-0.2, -0.0148, 0.1704], 0.251, false),
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
fn clipped_derived_empty_candidate_cannot_claim_original_source_completion() {
    let surface_a = paired_surface([0.0; 3], 0.25, false);
    let surface_b = paired_surface([-0.2, -0.0148, 0.1704], 0.25, false);
    let range_a = [ParamRange::new(0.0, 0.4), narrow_window()[1]];
    let range_b = [ParamRange::new(0.2, 0.45), ParamRange::new(0.0005, 0.0015)];
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let lower = intersect_bounded_nurbs_nurbs_surfaces(
        &surface_a, range_a, &surface_b, range_b, tolerances,
    )
    .unwrap();
    assert!(lower.curves.is_empty());
    assert!(!lower.is_complete());

    let mut graph = GeometryGraph::new();
    let source_a = graph.insert_surface(surface_a).unwrap();
    let source_b = graph.insert_surface(surface_b).unwrap();
    let result =
        intersect_bounded_graph_surfaces(&graph, source_a, range_a, source_b, range_b, tolerances)
            .unwrap();
    assert_eq!(result.raw, lower);
    assert!(result.branch_graph.edges.is_empty());
    assert!(result.branch_graph.vertices.is_empty());

    let reverse =
        intersect_bounded_graph_surfaces(&graph, source_b, range_b, source_a, range_a, tolerances)
            .unwrap();
    assert_eq!(reverse.raw, result.raw.swapped());
    assert_eq!(reverse.branch_graph.source_surfaces, [source_b, source_a]);
}

#[test]
fn offset_planar_unaligned_and_incompatible_basis_pairs_remain_unsupported() {
    let surface_a = paired_surface([0.0; 3], 0.25, false);
    let surface_b = paired_surface([-0.2, -0.0148, 0.1704], 0.25, false);
    let planar = planar_surface();
    let mut unaligned = paired_surface([-0.2, -0.0148, 0.1704], 0.25, false)
        .points()
        .to_vec();
    unaligned[0].x += 0.01;
    let unaligned = NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        unaligned,
        None,
    )
    .unwrap();
    let rational = paired_surface([-0.2, -0.0148, 0.1704], 0.25, true);
    let nonconstant_weights = NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        surface_b.points().to_vec(),
        Some(vec![1.0, 1.0, 2.0, 2.0, 1.0, 1.0]),
    )
    .unwrap();
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let mut graph = GeometryGraph::new();
    let source_a = graph.insert_surface(surface_a.clone()).unwrap();
    let source_b = graph.insert_surface(surface_b.clone()).unwrap();
    let offset_a = graph
        .insert_surface(OffsetSurfaceDescriptor::new(source_a, 0.01))
        .unwrap();
    let planar = graph.insert_surface(planar).unwrap();
    let unaligned = graph.insert_surface(unaligned).unwrap();
    let rational = graph.insert_surface(rational).unwrap();
    let nonconstant_weights = graph.insert_surface(nonconstant_weights).unwrap();
    for (first, second) in [
        (offset_a, source_b),
        (source_b, offset_a),
        (planar, source_b),
        (source_a, unaligned),
        (source_a, rational),
        (source_a, nonconstant_weights),
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

    for ranges in [
        [
            [ParamRange::new(0.0, 0.4), ParamRange::new(0.0, 0.0015)],
            [ParamRange::new(0.6, 1.0), ParamRange::new(0.0, 0.0015)],
        ],
        [
            [ParamRange::new(0.0, 0.5), ParamRange::new(0.0, 0.0015)],
            [ParamRange::new(0.5, 1.0), ParamRange::new(0.0, 0.0015)],
        ],
        [
            [ParamRange::new(0.5, 0.5), ParamRange::new(0.0, 0.0015)],
            [ParamRange::new(0.5, 0.5), ParamRange::new(0.0, 0.0015)],
        ],
    ] {
        assert!(matches!(
            intersect_bounded_graph_surfaces(
                &graph, source_a, ranges[0], source_b, ranges[1], tolerances,
            ),
            Err(GraphSurfaceIntersectionError::Intersection(
                IntersectionError::UnsupportedSurfacePair { .. }
            ))
        ));
    }
}
