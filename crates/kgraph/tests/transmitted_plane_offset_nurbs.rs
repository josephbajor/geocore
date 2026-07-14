//! Plane/Offset(NURBS) transmitted-intersection certificate contract.

use kgeom::curve2d::NurbsCurve2d;
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::surface::Plane;
use kgeom::vec::{Vec2, Vec3};
use kgraph::{
    GeometryGraph, GeometryGraphError, GeometryRef, IntersectionCertificateError,
    OffsetSurfaceDescriptor, TransmittedIntersectionChartMetadata, TransmittedOffsetNurbsTrace,
    TransmittedPlaneNurbsTrace, certify_transmitted_offset_nurbs_intersection_residuals,
};

#[test]
fn plane_offset_nurbs_certifies_both_orders_and_rejects_wrong_family() {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let carrier = NurbsCurve::new(
        1,
        knots.clone(),
        vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let pcurve = NurbsCurve2d::new(
        1,
        knots.clone(),
        vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
        None,
    )
    .unwrap();
    let basis = NurbsSurface::new(
        1,
        1,
        knots.clone(),
        knots,
        vec![
            Vec3::new(0.0, 0.0, -0.25),
            Vec3::new(0.0, 1.0, -0.25),
            Vec3::new(1.0, 0.0, -0.25),
            Vec3::new(1.0, 1.0, -0.25),
        ],
        None,
    )
    .unwrap();
    let plane = Plane::new(
        Frame::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let metadata =
        TransmittedIntersectionChartMetadata::new(0.0, 1.0, 0.0, 0.0, [None, None]).unwrap();

    let ordered_traces = [
        [
            TransmittedPlaneNurbsTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(
                basis.clone(),
                0.25,
            )),
            TransmittedPlaneNurbsTrace::Plane(plane),
        ],
        [
            TransmittedPlaneNurbsTrace::Plane(plane),
            TransmittedPlaneNurbsTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(
                basis.clone(),
                0.25,
            )),
        ],
    ];
    for traces in ordered_traces.clone() {
        let certificate = certify_transmitted_offset_nurbs_intersection_residuals(
            carrier.clone(),
            traces,
            [pcurve.clone(), pcurve.clone()],
            metadata,
            1.0e-10,
        )
        .unwrap();
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound < 1.0e-10)
        );
    }
    assert_eq!(
        certify_transmitted_offset_nurbs_intersection_residuals(
            carrier.clone(),
            [
                TransmittedPlaneNurbsTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(
                    basis.clone(),
                    0.25,
                )),
                TransmittedPlaneNurbsTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(
                    basis.clone(),
                    0.25,
                )),
            ],
            [pcurve.clone(), pcurve.clone()],
            metadata,
            1.0e-10,
        ),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );

    let mut graph = GeometryGraph::new();
    let basis_handle = graph.insert_surface(basis).unwrap();
    let offset_handle = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_handle, 0.25))
        .unwrap();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let pcurves = [
        graph.insert_curve2d(pcurve.clone()).unwrap(),
        graph.insert_curve2d(pcurve).unwrap(),
    ];
    for (sources, traces) in [
        ([offset_handle, plane_handle], ordered_traces[0].clone()),
        ([plane_handle, offset_handle], ordered_traces[1].clone()),
    ] {
        let certificate = certify_transmitted_offset_nurbs_intersection_residuals(
            carrier.clone(),
            traces,
            [
                graph
                    .curve2d(pcurves[0])
                    .unwrap()
                    .as_nurbs()
                    .unwrap()
                    .clone(),
                graph
                    .curve2d(pcurves[1])
                    .unwrap()
                    .as_nurbs()
                    .unwrap()
                    .clone(),
            ],
            metadata,
            1.0e-10,
        )
        .unwrap();
        let curve = graph
            .insert_verified_transmitted_nurbs_intersection_curve(sources, pcurves, certificate)
            .unwrap();
        assert_eq!(
            graph
                .direct_dependencies(GeometryRef::Curve(curve))
                .unwrap(),
            vec![
                GeometryRef::Surface(sources[0]),
                GeometryRef::Surface(sources[1]),
                GeometryRef::Curve2d(pcurves[0]),
                GeometryRef::Curve2d(pcurves[1]),
            ]
        );
    }

    let altered = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_handle, 0.5))
        .unwrap();
    let certificate = certify_transmitted_offset_nurbs_intersection_residuals(
        carrier,
        ordered_traces[0].clone(),
        [
            graph
                .curve2d(pcurves[0])
                .unwrap()
                .as_nurbs()
                .unwrap()
                .clone(),
            graph
                .curve2d(pcurves[1])
                .unwrap()
                .as_nurbs()
                .unwrap()
                .clone(),
        ],
        metadata,
        1.0e-10,
    )
    .unwrap();
    assert!(matches!(
        graph.insert_verified_transmitted_nurbs_intersection_curve(
            [altered, plane_handle],
            pcurves,
            certificate,
        ),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    graph.validate().unwrap();
}
