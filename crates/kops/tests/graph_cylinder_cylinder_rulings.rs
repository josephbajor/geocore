//! Verified graph promotion for strict parallel Cylinder/Cylinder rulings.
//! Wall-time budget: less than 10 seconds for the focused analytic matrix.

use kcore::operation::{OperationContext, ResourceKind, SessionPolicy};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Point3, Vec3};
use kgraph::{Curve2dDescriptor, CurveDescriptor, GeometryGraph, IntersectionCertificateError};
use kops::intersect::{
    GraphSurfaceIntersectionError, IntersectionBranchTopology, intersect_bounded_graph_surfaces,
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
fn tangent_internal_coincident_skew_and_axially_clipped_secant_remain_typed_gaps() {
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
        Cylinder::new(
            Frame::new(
                Point3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            1.0,
        )
        .unwrap(),
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
