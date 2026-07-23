//! Verified graph admission for exact parallel rulings and skew discriminants.
//! Wall-time budget: less than 10 seconds for the focused analytic matrix.

use kcore::error::CapabilityId;
use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, LimitSnapshot, LimitSpec, OperationContext,
    OperationScope, ResourceKind, SessionPolicy,
};
use kcore::proof::IncompleteCause;
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Point3, Vec3};
use kgraph::{Curve2dDescriptor, CurveDescriptor, GeometryGraph, IntersectionCertificateError};
use kops::intersect::{
    GraphSurfaceBudgetProfile, GraphSurfaceIntersectionError, IntersectionBranchTopology,
    IntersectionError, SKEW_CYLINDER_CONTACT_ROOT_TOPOLOGY,
    SKEW_CYLINDER_CONTACT_TOPOLOGY_INCOMPLETE, SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK,
    SKEW_CYLINDER_DISCRIMINANT_NUMERIC_RESOLUTION, SKEW_CYLINDER_DISCRIMINANT_WORK,
    SKEW_CYLINDER_TWO_SHEET_BRANCH_CARRIER, SKEW_CYLINDER_TWO_SHEET_INCOMPLETE,
    intersect_bounded_graph_surfaces, intersect_bounded_graph_surfaces_in_scope,
    intersect_bounded_graph_surfaces_with_context,
};

fn range(lo: f64, hi: f64) -> ParamRange {
    ParamRange::new(lo, hi)
}

fn cylinder_window(height: ParamRange) -> [ParamRange; 2] {
    [range(0.0, core::f64::consts::TAU), height]
}

fn graph_pair(
    first: Cylinder,
    second: Cylinder,
) -> (GeometryGraph, kgraph::SurfaceHandle, kgraph::SurfaceHandle) {
    let mut graph = GeometryGraph::new();
    let first_handle = graph.insert_surface(first).unwrap();
    let second_handle = graph.insert_surface(second).unwrap();
    (graph, first_handle, second_handle)
}

fn perpendicular_axis_pair(frame: Frame, offset: f64, second_radius: f64) -> [Cylinder; 2] {
    let first = Cylinder::new(frame, 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin() + frame.y() * offset, frame.x(), frame.y()).unwrap(),
        second_radius,
    )
    .unwrap();
    [first, second]
}

fn non_right_angle_axis_pair(frame: Frame, offset: f64, second_radius: f64) -> [Cylinder; 2] {
    let first = Cylinder::new(frame, 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            frame.origin() + frame.y() * offset,
            frame.x() * 0.6 + frame.z() * 0.8,
            frame.y(),
        )
        .unwrap(),
        second_radius,
    )
    .unwrap();
    [first, second]
}

fn one_sided_envelope_retry_pair() -> [Cylinder; 2] {
    let first = Cylinder::new(Frame::world(), 2.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            Point3::new(0.0, 8.0, 0.0),
            Vec3::new(1.0, 1.0, 2.0_f64.powi(-500)),
            Vec3::new(1.0, -1.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    [first, second]
}

fn skew_windows() -> [[ParamRange; 2]; 2] {
    [
        cylinder_window(range(-2.25, 2.25)),
        cylinder_window(range(-1.25, 1.25)),
    ]
}

fn assert_empty_skew_branch_graph(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
) {
    assert_eq!(result.branch_graph.source_surfaces, sources);
    assert!(result.branch_graph.vertices.is_empty());
    assert!(result.branch_graph.edges.is_empty());
    assert!(result.raw.points.is_empty());
    assert!(result.raw.curves.is_empty());
    assert!(result.raw.regions.is_empty());
    assert!(
        result
            .parallel_cylinder_exterior_radial_separation()
            .is_none(),
        "a skew proof must not mint parallel radial-separation evidence"
    );
}

fn assert_single_skew_incomplete(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
    code: DiagnosticCode,
    capability: CapabilityId,
    fixture: &str,
) {
    assert_empty_skew_branch_graph(result, sources);
    assert!(!result.raw.is_complete());
    assert!(!result.raw.is_proven_empty());
    assert!(
        result.skew_cylinder_strict_discriminant_miss().is_none(),
        "an unresolved skew contact family must not carry a miss witness"
    );
    assert_eq!(result.raw.incomplete_evidence().len(), 1, "{fixture}");
    let evidence = result.raw.incomplete_evidence()[0];
    assert_eq!(evidence.code, code, "{fixture}");
    assert_eq!(evidence.stage, SKEW_CYLINDER_DISCRIMINANT_WORK, "{fixture}");
    assert_eq!(
        evidence.cause,
        IncompleteCause::ProofMethodUnavailable { capability },
        "{fixture}"
    );
}

fn observed_work(
    report: &kcore::operation::OperationReport,
    stage: kcore::operation::StageId,
) -> u64 {
    report
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Work)
        .map_or(0, |usage| usage.consumed)
}

fn assert_ruling_lifts(edge: &kops::intersect::IntersectionBranchEdge, cylinders: [Cylinder; 2]) {
    let CurveDescriptor::Line(carrier) = edge.carrier else {
        panic!("Cylinder/Cylinder ruling must retain an exact line carrier");
    };
    assert_eq!(edge.topology, IntersectionBranchTopology::Open);
    assert!(
        edge.pcurves
            .iter()
            .all(|pcurve| matches!(pcurve, Curve2dDescriptor::Line(_)))
    );
    let certificate = edge.certificate.as_cylinder_cylinder_ruling().unwrap();
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= certificate.tolerance())
    );
    for parameter in [
        edge.carrier_range.lo,
        edge.carrier_range.lerp(0.37),
        edge.carrier_range.hi,
    ] {
        let point = carrier.eval(parameter);
        for (operand, cylinder) in cylinders.iter().enumerate() {
            let uv = edge.pcurves[operand]
                .as_curve()
                .eval(edge.parameter_maps[operand].map(parameter));
            assert!(point.dist(cylinder.eval([uv.x, uv.y])) <= certificate.tolerance());
        }
    }
}

#[test]
fn strict_parallel_secant_promotes_two_deterministic_rulings_in_both_orders() {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.25),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let first_range = cylinder_window(range(-1.0, 2.0));
    let second_range = cylinder_window(range(-0.75, 1.5));
    let (graph, first_handle, second_handle) = graph_pair(first, second);

    let forward = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        first_range,
        second_handle,
        second_range,
        Tolerances::default(),
    )
    .unwrap();
    let replay = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        first_range,
        second_handle,
        second_range,
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        second_handle,
        second_range,
        first_handle,
        first_range,
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(forward, replay);
    assert!(
        forward
            .parallel_cylinder_exterior_radial_separation()
            .is_none()
    );
    assert!(
        reversed
            .parallel_cylinder_exterior_radial_separation()
            .is_none()
    );
    assert_eq!(forward.branch_graph.edges.len(), 2);
    assert_eq!(forward.branch_graph.vertices.len(), 4);
    assert_eq!(reversed.raw, forward.raw.clone().swapped());
    assert_eq!(reversed.branch_graph.edges.len(), 2);
    for edge in &forward.branch_graph.edges {
        assert_eq!(edge.source_surfaces, [first_handle, second_handle]);
        assert_ruling_lifts(edge, [first, second]);
    }
    for edge in &reversed.branch_graph.edges {
        assert_eq!(edge.source_surfaces, [second_handle, first_handle]);
        assert_ruling_lifts(edge, [second, first]);
    }
    assert_eq!(
        forward
            .branch_graph
            .edges
            .iter()
            .map(|edge| (edge.carrier.clone(), edge.carrier_range))
            .collect::<Vec<_>>(),
        reversed
            .branch_graph
            .edges
            .iter()
            .map(|edge| (edge.carrier.clone(), edge.carrier_range))
            .collect::<Vec<_>>()
    );
}

#[test]
fn exact_antiparallel_oblique_axes_retain_operand_ordered_lifts() {
    let first_frame = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(-1.0, -1.0, 0.5),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let first = Cylinder::new(first_frame, 1.25).unwrap();
    let second = Cylinder::new(
        Frame::new(
            first_frame.origin() + first_frame.x(),
            -first_frame.z(),
            first_frame.x(),
        )
        .unwrap(),
        1.25,
    )
    .unwrap();
    let window = cylinder_window(range(-1.5, 2.0));
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let hit = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        window,
        second_handle,
        window,
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.branch_graph.edges.len(), 2);
    assert!(hit.parallel_cylinder_exterior_radial_separation().is_none());
    for edge in &hit.branch_graph.edges {
        assert_ruling_lifts(edge, [first, second]);
        assert!(edge.parameter_maps[0].scale() * edge.parameter_maps[1].scale() < 0.0);
    }
}

fn assert_typed_gap(
    first: Cylinder,
    first_window: [ParamRange; 2],
    second: Cylinder,
    second_window: [ParamRange; 2],
) {
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let error = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        first_window,
        second_handle,
        second_window,
        Tolerances::default(),
    )
    .unwrap_err();
    assert!(matches!(
        error,
        GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::UnsupportedCarrierParameterization { .. }
        )
    ));
}

#[test]
fn exact_exterior_radial_misses_are_complete_witnessed_and_swap_stable() {
    let oblique = Frame::new(
        Point3::new(0.0, -1.0, 3.0),
        Vec3::new(0.0, 0.6, 0.8),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let cases = [
        (
            Cylinder::new(Frame::world(), 1.0).unwrap(),
            Cylinder::new(
                Frame::new(
                    Point3::new(3.0, 0.0, 0.25),
                    Vec3::new(0.0, 0.0, 1.0),
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
                1.0,
            )
            .unwrap(),
        ),
        (
            Cylinder::new(oblique, 1.25).unwrap(),
            Cylinder::new(
                Frame::new(
                    Point3::new(2.0_f64.next_up(), -1.0, 3.0),
                    -oblique.z(),
                    oblique.x(),
                )
                .unwrap(),
                0.75,
            )
            .unwrap(),
        ),
    ];
    let windows = [
        cylinder_window(range(-1.0, 2.0)),
        cylinder_window(range(-0.5, 1.25)),
    ];

    for (first, second) in cases {
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let forward = intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        );
        let reversed = intersect_bounded_graph_surfaces_with_context(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            &context,
        );
        for (outcome, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            let result = outcome.result().unwrap();
            assert!(result.raw.is_proven_empty());
            assert!(result.raw.incomplete_evidence().is_empty());
            assert!(
                result
                    .parallel_cylinder_exterior_radial_separation()
                    .is_some()
            );
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.branch_graph.vertices.is_empty());
            assert!(result.branch_graph.edges.is_empty());
            let visits = outcome
                .report()
                .usage()
                .iter()
                .find(|usage| {
                    usage.stage == kgraph::eval_stage::NODE_VISITS
                        && usage.resource == ResourceKind::Work
                })
                .unwrap();
            assert_eq!(visits.consumed, 0);
        }
        assert_eq!(
            reversed.result().unwrap().raw,
            forward.result().unwrap().raw
        );
    }
}

#[test]
fn exact_exterior_miss_boundary_is_tolerance_independent_and_fails_closed() {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let window = cylinder_window(range(-1.0, 1.0));
    let cylinder_at = |distance: f64, radius: f64| {
        Cylinder::new(
            Frame::new(
                Point3::new(distance, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            radius,
        )
        .unwrap()
    };

    let just_outside = cylinder_at(2.0_f64.next_up(), 1.0);
    let (graph, first_handle, second_handle) = graph_pair(first, just_outside);
    let miss = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        window,
        second_handle,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.raw.is_proven_empty());
    assert!(
        miss.parallel_cylinder_exterior_radial_separation()
            .is_some()
    );

    for distance in [2.0, 2.0_f64.next_down()] {
        assert_typed_gap(first, window, cylinder_at(distance, 1.0), window);
    }

    // This separation is entirely inside the default linear tolerance. The
    // graph proof must use exact source coefficients rather than inheriting the
    // lower solver's near-coincident policy.
    let tiny_first = Cylinder::new(Frame::world(), 1.0e-12).unwrap();
    let tiny_second = cylinder_at(4.0e-12, 2.0e-12);
    let (graph, first_handle, second_handle) = graph_pair(tiny_first, tiny_second);
    let tiny = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        window,
        second_handle,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert!(tiny.raw.is_proven_empty());
    assert!(
        tiny.parallel_cylinder_exterior_radial_separation()
            .is_some()
    );
}

#[test]
fn perpendicular_skew_miss_is_complete_swap_replay_and_rigid_stable() {
    let oblique = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(1.0, -2.0, 3.0),
        Vec3::new(2.0, 1.0, 0.5),
    )
    .unwrap();
    let windows = skew_windows();

    // In the fixture frame, A is the local z-axis and B is the local x-axis
    // through (0, d, 0). Substitution gives
    // v^2 = R^2 - (sin(u) - d)^2. For d=4 and R=2 the right-hand side is
    // strictly negative over the complete cycle. The upper one-ULP neighbor
    // of the d=3 repeated contact is independently strict-negative.
    for (name, frame, offset) in [
        ("world", Frame::world(), 4.0),
        ("rigid-oblique", oblique, 4.0),
        ("one-ulp-strict-miss", Frame::world(), 3.0_f64.next_up()),
    ] {
        let [first, second] = perpendicular_axis_pair(frame, offset, 2.0);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();

        assert_eq!(forward, replay, "{name} replay changed the exact result");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_empty_skew_branch_graph(result, sources);
            assert!(result.raw.is_proven_empty(), "{name}");
            assert!(result.raw.incomplete_evidence().is_empty(), "{name}");
            assert!(
                result.skew_cylinder_strict_discriminant_miss().is_some(),
                "{name}"
            );
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped(), "{name}");
        assert_eq!(
            reversed.skew_cylinder_strict_discriminant_miss(),
            forward.skew_cylinder_strict_discriminant_miss(),
            "{name}"
        );
    }
}

#[test]
fn non_right_angle_skew_miss_matches_axis_distance_oracle_and_is_swap_stable() {
    let [first, second] = non_right_angle_axis_pair(Frame::world(), 4.0, 2.0);
    let axis_cross = first.frame().z().cross(second.frame().z());
    let axis_cosine = first.frame().z().dot(second.frame().z())
        / (first.frame().z().norm() * second.frame().z().norm());
    let axis_distance = ((second.frame().origin() - first.frame().origin()).dot(axis_cross)).abs()
        / axis_cross.norm();
    assert!((axis_cosine - 0.8).abs() < 1.0e-14);
    assert!(axis_cosine != 0.0 && axis_cosine.abs() != 1.0);
    assert!((axis_distance - 4.0).abs() < 1.0e-14);
    assert!(axis_distance > first.radius() + second.radius());

    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let replay = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        second_handle,
        windows[1],
        first_handle,
        windows[0],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(forward, replay);
    for (result, sources) in [
        (&forward, [first_handle, second_handle]),
        (&reversed, [second_handle, first_handle]),
    ] {
        assert_empty_skew_branch_graph(result, sources);
        assert!(result.raw.is_proven_empty());
        assert!(result.raw.incomplete_evidence().is_empty());
        assert!(result.skew_cylinder_strict_discriminant_miss().is_some());
    }
    assert_eq!(reversed.raw, forward.raw.clone().swapped());
    assert_eq!(
        reversed.skew_cylinder_strict_discriminant_miss(),
        forward.skew_cylinder_strict_discriminant_miss()
    );
}

#[test]
fn one_sided_exact_envelope_refusal_retries_reversed_parameterization() {
    let [first, second] = one_sided_envelope_retry_pair();
    let axis_cross = first.frame().z().cross(second.frame().z());
    let axis_distance = ((second.frame().origin() - first.frame().origin()).dot(axis_cross)).abs()
        / axis_cross.norm();
    assert!(axis_distance > first.radius() + second.radius());

    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let forward = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &context,
    );
    let replay = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &context,
    );
    let reversed = intersect_bounded_graph_surfaces_with_context(
        &graph,
        second_handle,
        windows[1],
        first_handle,
        windows[0],
        &context,
    );

    assert_eq!(forward, replay);
    for (outcome, sources) in [
        (&forward, [first_handle, second_handle]),
        (&reversed, [second_handle, first_handle]),
    ] {
        let result = outcome.result().unwrap();
        assert_empty_skew_branch_graph(result, sources);
        assert!(result.raw.is_proven_empty());
        assert!(result.skew_cylinder_strict_discriminant_miss().is_some());
        assert_eq!(
            observed_work(outcome.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
            SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
        );
        assert!(outcome.report().numeric_resolution_stages().is_empty());
        assert!(outcome.report().limit_events().is_empty());
    }
    assert_eq!(
        reversed.result().unwrap().raw,
        forward.result().unwrap().raw.clone().swapped()
    );
}

#[test]
fn perpendicular_skew_positive_root_and_ulp_cases_keep_structured_evidence() {
    #[derive(Clone, Copy)]
    struct Fixture {
        name: &'static str,
        offset: f64,
        code: DiagnosticCode,
        capability: CapabilityId,
    }

    let fixtures = [
        Fixture {
            name: "strict-positive-two-sheet-cover",
            offset: 0.0,
            code: SKEW_CYLINDER_TWO_SHEET_INCOMPLETE,
            capability: SKEW_CYLINDER_TWO_SHEET_BRANCH_CARRIER,
        },
        Fixture {
            name: "exact-repeated-zero",
            offset: 3.0,
            code: SKEW_CYLINDER_CONTACT_TOPOLOGY_INCOMPLETE,
            capability: SKEW_CYLINDER_CONTACT_ROOT_TOPOLOGY,
        },
        Fixture {
            name: "one-ulp-rooted-neighbor",
            offset: 3.0_f64.next_down(),
            code: SKEW_CYLINDER_CONTACT_TOPOLOGY_INCOMPLETE,
            capability: SKEW_CYLINDER_CONTACT_ROOT_TOPOLOGY,
        },
    ];
    let windows = skew_windows();

    for fixture in fixtures {
        let [first, second] = perpendicular_axis_pair(Frame::world(), fixture.offset, 2.0);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();

        assert_eq!(forward, replay, "{} changed across replay", fixture.name);
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_single_skew_incomplete(
                result,
                sources,
                fixture.code,
                fixture.capability,
                fixture.name,
            );
        }
        assert_eq!(
            reversed.raw,
            forward.raw.clone().swapped(),
            "{} changed under operand reversal",
            fixture.name
        );
    }
}

#[test]
fn skew_miss_proof_validates_windows_and_fails_closed_on_unsafe_expansions() {
    let windows = skew_windows();
    let [first, second] = perpendicular_axis_pair(Frame::world(), 4.0, 2.0);
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let mut reversed_window = windows[0];
    reversed_window[1] = ParamRange { lo: 1.0, hi: -1.0 };
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let malformed = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        reversed_window,
        second_handle,
        windows[1],
        &context,
    );
    assert!(matches!(
        malformed.result(),
        Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::Kernel(kcore::error::Error::InvalidGeometry { .. })
        ))
    ));
    assert_eq!(
        observed_work(malformed.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        0,
        "window validation must precede global discriminant certification"
    );
    assert!(malformed.report().limit_events().is_empty());

    let [first, unsafe_second] = perpendicular_axis_pair(Frame::world(), 1.0e200, 2.0);
    let (unsafe_graph, first_handle, second_handle) = graph_pair(first, unsafe_second);
    let unresolved = intersect_bounded_graph_surfaces_with_context(
        &unsafe_graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &context,
    );
    let result = unresolved
        .result()
        .expect("unsafe exact expansion must be incomplete, not a policy error");
    assert_empty_skew_branch_graph(result, [first_handle, second_handle]);
    assert!(!result.raw.is_complete());
    assert!(!result.raw.is_proven_empty());
    assert!(result.skew_cylinder_strict_discriminant_miss().is_none());
    assert_eq!(result.raw.incomplete_evidence().len(), 1);
    let evidence = result.raw.incomplete_evidence()[0];
    assert_eq!(evidence.code, SKEW_CYLINDER_DISCRIMINANT_NUMERIC_RESOLUTION);
    assert_eq!(evidence.stage, SKEW_CYLINDER_DISCRIMINANT_WORK);
    assert_eq!(evidence.cause, IncompleteCause::NumericResolution);
    assert_eq!(
        observed_work(unresolved.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
    );
    assert_eq!(
        unresolved.report().numeric_resolution_stages(),
        &[SKEW_CYLINDER_DISCRIMINANT_WORK]
    );
    assert!(unresolved.report().limit_events().is_empty());
}

#[test]
fn skew_discriminant_work_has_exact_n_and_atomic_n_minus_one_boundary() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 4.0, 2.0);
    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();

    let exact_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_DISCRIMINANT_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK,
    )])
    .unwrap();
    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(exact_plan);
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &exact_context,
    );
    assert!(exact.result().unwrap().raw.is_proven_empty());
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_DISCRIMINANT_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK - 1,
    )])
    .unwrap();
    let denied_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(denied_plan);
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &denied_context,
    );
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_DISCRIMINANT_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK,
        allowed: SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        0,
        "a rejected single-stage debit must not partially consume work"
    );
}

#[test]
fn default_graph_budget_admits_multiple_skew_pairs_in_one_owner_scope() {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let second_at = |offset| perpendicular_axis_pair(Frame::world(), offset, 2.0)[1];
    let mut graph = GeometryGraph::new();
    let first_handle = graph.insert_surface(first).unwrap();
    let second_handles = [
        graph.insert_surface(second_at(4.0)).unwrap(),
        graph.insert_surface(second_at(5.0)).unwrap(),
    ];
    let windows = skew_windows();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(GraphSurfaceBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);

    for second_handle in second_handles {
        let result = intersect_bounded_graph_surfaces_in_scope(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &mut scope,
        )
        .expect("the aggregate graph budget must admit more than one skew face pair");
        assert!(result.raw.is_proven_empty());
        assert!(result.skew_cylinder_strict_discriminant_miss().is_some());
    }

    let outcome = scope.finish_typed::<_, GraphSurfaceIntersectionError>(Ok(()));
    assert_eq!(
        observed_work(outcome.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        2 * SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
    );
    assert!(outcome.report().limit_events().is_empty());
}

#[test]
fn tangent_internal_coincident_and_axially_clipped_secant_remain_typed_gaps() {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let window = cylinder_window(range(-1.0, 1.0));
    let cases = [
        Cylinder::new(
            Frame::new(
                Point3::new(2.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            1.0,
        )
        .unwrap(),
        Cylinder::new(
            Frame::new(
                Point3::new(0.5, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            0.25,
        )
        .unwrap(),
        first,
    ];
    for second in cases {
        assert_typed_gap(first, window, second, window);
    }

    let secant = Cylinder::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    assert_typed_gap(
        first,
        cylinder_window(range(-2.0, -1.0)),
        secant,
        cylinder_window(range(1.0, 2.0)),
    );
}
