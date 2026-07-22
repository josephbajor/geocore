//! Verified graph promotion for strict parallel Cylinder/Cylinder rulings.
//! Wall-time budget: less than 10 seconds for the focused analytic matrix.

use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Point3, Vec3};
use kgraph::{Curve2dDescriptor, CurveDescriptor, GeometryGraph, IntersectionCertificateError};
use kops::intersect::{
    GraphSurfaceIntersectionError, IntersectionBranchTopology, intersect_bounded_graph_surfaces,
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
    for edge in &hit.branch_graph.edges {
        assert_ruling_lifts(edge, [first, second]);
        assert!(edge.parameter_maps[0].scale() * edge.parameter_maps[1].scale() < 0.0);
    }
}

#[test]
fn tangent_miss_coincident_and_skew_pairs_remain_typed_gaps() {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
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
                Point3::new(3.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            1.0,
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
    let window = cylinder_window(range(-1.0, 1.0));
    for second in cases {
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let error = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            window,
            second_handle,
            window,
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
}
