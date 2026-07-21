//! Verified graph promotion for finite Plane/Cylinder rulings.

use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane, Surface};
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    Curve2dDescriptor, CurveDescriptor, GeometryGraph, IntersectionCertificateError,
    PlaneCylinderRulingTrace,
};
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
    plane: Plane,
    cylinder: Cylinder,
) -> (GeometryGraph, kgraph::SurfaceHandle, kgraph::SurfaceHandle) {
    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let cylinder_handle = graph.insert_surface(cylinder).unwrap();
    (graph, plane_handle, cylinder_handle)
}

fn assert_ruling_lifts(
    edge: &kops::intersect::IntersectionBranchEdge,
    plane: Plane,
    cylinder: Cylinder,
    plane_first: bool,
) {
    let CurveDescriptor::Line(carrier) = &edge.carrier else {
        panic!("Plane/Cylinder ruling must retain an exact line carrier");
    };
    assert_eq!(edge.topology, IntersectionBranchTopology::Open);
    assert!(
        edge.pcurves
            .iter()
            .all(|pcurve| matches!(pcurve, Curve2dDescriptor::Line(_)))
    );
    let certificate = edge.certificate.as_plane_cylinder_ruling().unwrap();
    let plane_index = usize::from(!plane_first);
    let cylinder_index = usize::from(plane_first);
    assert!(matches!(
        certificate.traces()[plane_index],
        PlaneCylinderRulingTrace::Plane(_)
    ));
    assert!(matches!(
        certificate.traces()[cylinder_index],
        PlaneCylinderRulingTrace::Cylinder(_)
    ));
    assert_eq!(certificate.carrier(), *carrier);
    assert_eq!(certificate.carrier_range(), edge.carrier_range);
    assert_eq!(certificate.parameter_maps(), edge.parameter_maps);
    for parameter in [
        edge.carrier_range.lo,
        edge.carrier_range.lerp(0.37),
        edge.carrier_range.hi,
    ] {
        let point = carrier.eval(parameter);
        let plane_uv = edge.pcurves[plane_index]
            .as_curve()
            .eval(edge.parameter_maps[plane_index].map(parameter));
        let cylinder_uv = edge.pcurves[cylinder_index]
            .as_curve()
            .eval(edge.parameter_maps[cylinder_index].map(parameter));
        assert!(point.dist(plane.eval([plane_uv.x, plane_uv.y])) <= certificate.tolerance());
        assert!(
            point.dist(cylinder.eval([cylinder_uv.x, cylinder_uv.y])) <= certificate.tolerance()
        );
    }
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= certificate.tolerance())
    );
}

#[test]
fn world_rulings_promote_in_both_operand_orders_with_range_parity() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let plane = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
    );
    let plane_window = [range(-2.0, 2.0), range(-3.0, 3.0)];
    let cylinder_window = cylinder_window(range(-1.25, 2.5));
    let (graph, plane_handle, cylinder_handle) = graph_pair(plane, cylinder);
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        plane_window,
        cylinder_handle,
        cylinder_window,
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        cylinder_handle,
        cylinder_window,
        plane_handle,
        plane_window,
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(forward.branch_graph.edges.len(), 2);
    assert_eq!(reversed.raw, forward.raw.swapped());
    assert_eq!(reversed.branch_graph.edges.len(), 2);
    for edge in &forward.branch_graph.edges {
        assert_eq!(edge.source_surfaces, [plane_handle, cylinder_handle]);
        assert_ruling_lifts(edge, plane, cylinder, true);
    }
    for edge in &reversed.branch_graph.edges {
        assert_eq!(edge.source_surfaces, [cylinder_handle, plane_handle]);
        assert_ruling_lifts(edge, plane, cylinder, false);
    }
    let forward_ranges = forward
        .branch_graph
        .edges
        .iter()
        .map(|edge| edge.carrier_range)
        .collect::<Vec<_>>();
    let reversed_ranges = reversed
        .branch_graph
        .edges
        .iter()
        .map(|edge| edge.carrier_range)
        .collect::<Vec<_>>();
    assert_eq!(forward_ranges, reversed_ranges);
}

#[test]
fn oblique_frame_rulings_use_the_canonical_signed_axis() {
    let frame = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(-1.0, -1.0, 0.5),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let cylinder = Cylinder::new(frame, 1.25).unwrap();
    let plane = Plane::new(Frame::new(frame.origin(), frame.x(), frame.z()).unwrap());
    let (graph, plane_handle, cylinder_handle) = graph_pair(plane, cylinder);
    let hit = intersect_bounded_graph_surfaces(
        &graph,
        plane_handle,
        [range(-2.0, 2.0), range(-3.0, 3.0)],
        cylinder_handle,
        cylinder_window(range(-1.5, 2.0)),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.branch_graph.edges.len(), 2);
    for edge in &hit.branch_graph.edges {
        let CurveDescriptor::Line(carrier) = edge.carrier else {
            panic!("oblique ruling must retain a line");
        };
        assert!(carrier.dir().dot(frame.z()) < 0.0);
        assert!(carrier.dir().x > 0.0);
        assert_eq!(edge.carrier_range, range(-2.0, 1.5));
        assert_ruling_lifts(edge, plane, cylinder, true);
    }
}

#[test]
fn tangent_and_near_parallel_graph_candidates_fail_closed() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    for plane in [
        Plane::new(
            Frame::new(
                Point3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
        ),
        Plane::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 1e-12),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
        ),
    ] {
        let (graph, plane_handle, cylinder_handle) = graph_pair(plane, cylinder);
        let error = intersect_bounded_graph_surfaces(
            &graph,
            plane_handle,
            [range(-2.0, 2.0), range(-2.0, 2.0)],
            cylinder_handle,
            cylinder_window(range(-1.0, 1.0)),
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
