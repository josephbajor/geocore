//! Contextual graph-owned Sphere/NURBS branch contracts.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::curve2d::NurbsCurve2d;
use kgeom::frame::Frame;
use kgeom::nurbs::{
    NURBS_IMPLICIT_ISOLATION_CANDIDATES, NURBS_IMPLICIT_ISOLATION_DEPTH,
    NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS, NurbsCurve, NurbsSurface,
};
use kgeom::param::ParamRange;
use kgeom::surface::{Sphere, Surface};
use kgeom::vec::{Point3, Vec2};
use kgraph::{
    GeometryGraph, GeometryGraphError, IntersectionCertificateError, NurbsIntersectionTrace,
    OffsetSurfaceDescriptor, PairedTrace, certify_verified_sphere_nurbs_intersection_residuals,
    verified_sphere_nurbs_intersection_certificate_cost,
};
use kops::intersect::{
    GraphSurfaceIntersectionError, IntersectionError, NURBS_SURFACE_MARCH_SAMPLES,
    NURBS_TRACE_CERTIFICATE_WORK, SurfaceIntersectionCurve, intersect_bounded_graph_surfaces,
    intersect_bounded_graph_surfaces_with_context, intersect_bounded_sphere_nurbs_surface,
    intersect_bounded_sphere_nurbs_surface_with_context,
    persist_verified_graph_surface_intersections,
};

fn curved_patch(height: f64) -> NurbsSurface {
    patch(height, 0.005)
}

fn patch(height: f64, bend_control: f64) -> NurbsSurface {
    let u_coordinates = [0.0, 0.5, 1.0];
    let z_controls = [height, height + bend_control, height];
    let mut points = Vec::with_capacity(6);
    for (u_index, &u) in u_coordinates.iter().enumerate() {
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
        None,
    )
    .unwrap()
}

fn sphere_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
        ParamRange::new(0.3, 0.8),
    ]
}

fn single_march_segment_sphere_window() -> [ParamRange; 2] {
    [ParamRange::new(0.0, 0.06), ParamRange::new(0.3, 0.8)]
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

#[test]
fn curved_sphere_nurbs_trace_certifies_over_the_complete_carrier() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let surface = curved_patch(0.5);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let mut graph = GeometryGraph::new();
    let sphere_handle = graph.insert_surface(sphere).unwrap();
    let surface_handle = graph.insert_surface(surface.clone()).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        sphere_handle,
        sphere_window(),
        surface_handle,
        surface.param_range(),
        &context,
    );
    let result = outcome.result().unwrap();
    assert_eq!(result.branch_graph.edges.len(), 1);
    assert!(matches!(
        result.branch_graph.edges[0]
            .certificate
            .as_nurbs()
            .unwrap()
            .traces(),
        [
            NurbsIntersectionTrace::Sphere(_),
            NurbsIntersectionTrace::Nurbs(_)
        ]
    ));

    let lower_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_family_budget_defaults(
            kops::intersect::NurbsSurfaceMarchBudgetProfile::v1_defaults(),
        );
    let lower = intersect_bounded_sphere_nurbs_surface_with_context(
        &sphere,
        sphere_window(),
        &surface,
        surface.param_range(),
        &lower_context,
    )
    .unwrap();
    assert_eq!(&result.raw, *lower.result().as_ref().unwrap());
    assert!(!result.raw.is_complete());
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
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= tolerances.linear())
    );
    let cost = verified_sphere_nurbs_intersection_certificate_cost(
        certificate.carrier(),
        certificate.traces(),
    )
    .unwrap();
    assert_eq!(
        observed(
            outcome.report(),
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        cost.work()
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
        surface_handle,
        surface.param_range(),
        sphere_handle,
        sphere_window(),
        tolerances,
    )
    .unwrap();
    assert_eq!(reverse.raw, result.raw.clone().swapped());
    assert_eq!(
        reverse.branch_graph.source_surfaces,
        [surface_handle, sphere_handle]
    );
    assert!(matches!(
        reverse.branch_graph.edges[0]
            .certificate
            .as_nurbs()
            .unwrap()
            .traces(),
        [
            NurbsIntersectionTrace::Nurbs(_),
            NurbsIntersectionTrace::Sphere(_)
        ]
    ));

    let persistent = persist_verified_graph_surface_intersections(&mut graph, result).unwrap();
    let descriptor = graph.curve(persistent.edges[0].curve).unwrap();
    let verified = descriptor.as_verified_nurbs_intersection().unwrap();
    assert_eq!(verified.source_surfaces(), [sphere_handle, surface_handle]);
    graph.validate().unwrap();

    let miss_surface = curved_patch(2.0);
    let miss_handle = graph.insert_surface(miss_surface.clone()).unwrap();
    let miss = intersect_bounded_graph_surfaces(
        &graph,
        sphere_handle,
        sphere_window(),
        miss_handle,
        miss_surface.param_range(),
        tolerances,
    )
    .unwrap();
    let lower_miss = intersect_bounded_sphere_nurbs_surface(
        &sphere,
        sphere_window(),
        &miss_surface,
        miss_surface.param_range(),
        tolerances,
    )
    .unwrap();
    assert_eq!(miss.raw, lower_miss);
    assert!(miss.raw.is_proven_empty());
    assert!(miss.branch_graph.edges.is_empty());

    assert!(matches!(
        graph.remove_surface(sphere_handle),
        Err(GeometryGraphError::HasDependents { .. })
    ));
    assert!(matches!(
        graph.replace_surface(sphere_handle, Sphere::new(Frame::world(), 1.1).unwrap()),
        Err(GeometryGraphError::HasDependents { .. })
    ));
    assert!(matches!(
        graph.remove_surface(surface_handle),
        Err(GeometryGraphError::HasDependents { .. })
    ));
    graph.validate().unwrap();
}

#[test]
fn sphere_nurbs_pins_exact_work_items_and_depth_boundaries() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let surface = curved_patch(0.5);
    let ranges = single_march_segment_sphere_window();
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let mut graph = GeometryGraph::new();
    let sphere_handle = graph.insert_surface(sphere).unwrap();
    let surface_handle = graph.insert_surface(surface.clone()).unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let baseline = intersect_bounded_graph_surfaces_with_context(
        &graph,
        sphere_handle,
        ranges,
        surface_handle,
        surface.param_range(),
        &context,
    );
    let certificate = baseline.result().unwrap().branch_graph.edges[0]
        .certificate
        .as_nurbs()
        .unwrap();
    let cost = verified_sphere_nurbs_intersection_certificate_cost(
        certificate.carrier(),
        certificate.traces(),
    )
    .unwrap();
    assert_eq!(
        (cost.work(), cost.items(), cost.depth()),
        (8_192, 1_024, 10)
    );

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
        sphere_handle,
        ranges,
        surface_handle,
        surface.param_range(),
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
        (ResourceKind::Work, AccountingMode::Cumulative, 8_191, 8_192),
        (ResourceKind::Items, AccountingMode::HighWater, 1_023, 1_024),
        (ResourceKind::Depth, AccountingMode::HighWater, 9, 10),
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
            sphere_handle,
            ranges,
            surface_handle,
            surface.param_range(),
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
fn failed_sphere_lift_proof_reports_all_attempted_resources() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let surface = curved_patch(0.5);
    let tolerances = Tolerances::with_linear(1.0e-5).unwrap();
    let lower = intersect_bounded_sphere_nurbs_surface(
        &sphere,
        sphere_window(),
        &surface,
        surface.param_range(),
        tolerances,
    )
    .unwrap();
    let SurfaceIntersectionCurve::Nurbs(carrier) = &lower.curves[0].curve else {
        panic!("Sphere/NURBS marcher must return a NURBS carrier");
    };
    let items = u64::try_from(carrier.points().len() - 1).unwrap() * 1_024;
    let work = items * 8;

    let mut graph = GeometryGraph::new();
    let sphere_handle = graph.insert_surface(sphere).unwrap();
    let surface_handle = graph.insert_surface(surface.clone()).unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_graph_surfaces_with_context(
        &graph,
        sphere_handle,
        sphere_window(),
        surface_handle,
        surface.param_range(),
        &context,
    );
    assert!(matches!(
        outcome.result(),
        Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::ResidualExceedsTolerance {
                trace: PairedTrace::First,
                ..
            }
        ))
    ));
    assert_eq!(
        observed(
            outcome.report(),
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        work
    );
    assert_eq!(
        observed(
            outcome.report(),
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Items,
        ),
        items
    );
    assert_eq!(
        observed(
            outcome.report(),
            NURBS_TRACE_CERTIFICATE_WORK,
            ResourceKind::Depth,
        ),
        10
    );
}

#[test]
fn sphere_nurbs_stale_and_altered_sources_roll_back_atomically() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let surface = curved_patch(0.5);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    for mutation in 0..4 {
        let mut graph = GeometryGraph::new();
        let sphere_handle = graph.insert_surface(sphere).unwrap();
        let surface_handle = graph.insert_surface(surface.clone()).unwrap();
        let local = intersect_bounded_graph_surfaces(
            &graph,
            sphere_handle,
            sphere_window(),
            surface_handle,
            surface.param_range(),
            tolerances,
        )
        .unwrap();
        match mutation {
            0 => {
                graph.remove_surface(sphere_handle).unwrap();
            }
            1 => {
                graph
                    .replace_surface(sphere_handle, Sphere::new(Frame::world(), 1.1).unwrap())
                    .unwrap();
            }
            2 => {
                graph.remove_surface(surface_handle).unwrap();
            }
            3 => {
                graph
                    .replace_surface(surface_handle, curved_patch(0.51))
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
fn procedural_and_nurbs_nurbs_pairs_remain_explicitly_unsupported() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let surface = curved_patch(0.5);
    let other_surface = curved_patch(0.55);
    let planar_surface = patch(0.5, 0.0);
    let tolerances = Tolerances::with_linear(1.0e-3).unwrap();
    let mut graph = GeometryGraph::new();
    let sphere_handle = graph.insert_surface(sphere).unwrap();
    let offset_sphere = graph
        .insert_surface(OffsetSurfaceDescriptor::new(sphere_handle, 0.1))
        .unwrap();
    let nurbs = graph.insert_surface(surface.clone()).unwrap();
    let offset_nurbs = graph
        .insert_surface(OffsetSurfaceDescriptor::new(nurbs, 0.1))
        .unwrap();
    let other_nurbs = graph.insert_surface(other_surface.clone()).unwrap();
    let planar_nurbs = graph.insert_surface(planar_surface.clone()).unwrap();
    for (first, first_range, second, second_range) in [
        (offset_sphere, sphere_window(), nurbs, surface.param_range()),
        (
            sphere_handle,
            sphere_window(),
            offset_nurbs,
            surface.param_range(),
        ),
        (
            offset_nurbs,
            surface.param_range(),
            other_nurbs,
            other_surface.param_range(),
        ),
        (
            nurbs,
            surface.param_range(),
            other_nurbs,
            other_surface.param_range(),
        ),
        (
            sphere_handle,
            sphere_window(),
            planar_nurbs,
            planar_surface.param_range(),
        ),
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

#[test]
fn sphere_trace_at_a_chart_pole_fails_closed() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let surface = curved_patch(0.5);
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let carrier = NurbsCurve::new(
        1,
        knots.clone(),
        vec![Point3::new(0.0, 0.0, 1.0), Point3::new(0.0, 0.0, 1.0)],
        None,
    )
    .unwrap();
    let sphere_pcurve = NurbsCurve2d::new(
        1,
        knots.clone(),
        vec![
            Vec2::new(0.0, core::f64::consts::FRAC_PI_2),
            Vec2::new(0.1, core::f64::consts::FRAC_PI_2),
        ],
        None,
    )
    .unwrap();
    let surface_pcurve = NurbsCurve2d::new(
        1,
        knots,
        vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
        None,
    )
    .unwrap();
    assert!(matches!(
        certify_verified_sphere_nurbs_intersection_residuals(
            carrier,
            [
                NurbsIntersectionTrace::Sphere(sphere),
                NurbsIntersectionTrace::Nurbs(surface),
            ],
            [sphere_pcurve, surface_pcurve],
            1.0e-3,
        ),
        Err(IntersectionCertificateError::SingularSphereChart { .. })
    ));
}
