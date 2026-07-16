#![allow(
    deprecated,
    reason = "lower-layer copy integration retains the compatibility tessellation wrapper"
)]

//! Checked deterministic complete-body rigid-copy contracts.

use kcore::error::Error;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Line};
use kgeom::curve2d::{Circle2d, Curve2d, Line2d, NurbsCurve2d};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere, Surface};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};
use kgraph::{
    AffineParamMap1d, EvalLimits, ExactSurfaceField, NurbsIntersectionTrace,
    OffsetSurfaceDescriptor, PairedTrace, PlaneCircleTrace, PlaneSphereCircleTrace,
    SphereLatitudeTrace, SurfaceDerivativeOrder, TransmittedIntersectionChartMetadata,
    TransmittedOffsetNurbsTrace, TransmittedOffsetPlaneTrace, VerifiedIntersectionCertificate,
    VerifiedNurbsIntersectionCertificate, certify_paired_plane_line_residuals,
    certify_paired_plane_sphere_circle_residuals,
    certify_paired_plane_sphere_oblique_circle_residuals,
    certify_transmitted_cubic_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_five_sample_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_nurbs_nurbs_intersection_residuals,
    certify_transmitted_offset_nurbs_intersection_residuals,
    certify_transmitted_plane_intersection_residuals,
    certify_transmitted_plane_nurbs_intersection_residuals,
    certify_transmitted_quadratic_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_seven_sample_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_two_sample_dual_offset_nurbs_intersection_residuals,
    certify_verified_dual_offset_nurbs_intersection_residuals,
    certify_verified_nurbs_nurbs_intersection_residuals,
    certify_verified_offset_nurbs_nurbs_intersection_residuals,
    certify_verified_offset_nurbs_plane_intersection_residuals,
    certify_verified_plane_nurbs_intersection_residuals,
    certify_verified_sphere_nurbs_intersection_residuals,
};
use ktopo::btess::{TessOptions, tessellate_body};
use ktopo::check::{CheckLevel, CheckOutcome, check_body_report};
use ktopo::entity::{Body, BodyKind, Edge, Face, Fin, Loop, Region, RegionKind, Shell, Vertex};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::make;
use ktopo::profile::PlanarProfile;
use ktopo::store::Store;
use ktopo::transaction::LineageEvent;
use std::collections::HashSet;

fn placement() -> Frame {
    Frame::new(
        Point3::new(4.0, -3.0, 2.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

fn oblique_placement() -> Frame {
    let z = Vec3::new(1.0, 2.0, 3.0).normalized().unwrap();
    let x = Vec3::new(2.0, -1.0, 0.0).normalized().unwrap();
    Frame::new(Point3::new(-2.5, 1.25, 3.75), z, x).unwrap()
}

fn map_point(placement: Frame, point: Point3) -> Point3 {
    placement.point_at(point.x, point.y, point.z)
}

fn map_vector(placement: Frame, vector: Vec3) -> Vec3 {
    placement.x() * vector.x + placement.y() * vector.y + placement.z() * vector.z
}

fn assert_transformed_nurbs_trace(
    source: &NurbsIntersectionTrace,
    copied: &NurbsIntersectionTrace,
    placement: Frame,
) {
    let transformed_surface = |source: &NurbsSurface, copied: &NurbsSurface| {
        assert_eq!(copied.degree_u(), source.degree_u());
        assert_eq!(copied.degree_v(), source.degree_v());
        assert_eq!(
            copied.knots(kgeom::surface::Dir::U),
            source.knots(kgeom::surface::Dir::U)
        );
        assert_eq!(
            copied.knots(kgeom::surface::Dir::V),
            source.knots(kgeom::surface::Dir::V)
        );
        assert_eq!(copied.weights(), source.weights());
        assert_eq!(
            copied.points(),
            source
                .points()
                .iter()
                .map(|&point| map_point(placement, point))
                .collect::<Vec<_>>()
        );
    };
    let transformed_frame = |source: &Frame| {
        Frame::new(
            map_point(placement, source.origin()),
            map_vector(placement, source.z()),
            map_vector(placement, source.x()),
        )
        .unwrap()
    };
    match (source, copied) {
        (NurbsIntersectionTrace::Plane(source), NurbsIntersectionTrace::Plane(copied)) => {
            assert_eq!(*copied.frame(), transformed_frame(source.frame()));
        }
        (NurbsIntersectionTrace::Sphere(source), NurbsIntersectionTrace::Sphere(copied)) => {
            assert_eq!(*copied.frame(), transformed_frame(source.frame()));
            assert_eq!(copied.radius(), source.radius());
        }
        (NurbsIntersectionTrace::Nurbs(source), NurbsIntersectionTrace::Nurbs(copied)) => {
            transformed_surface(source, copied);
        }
        (
            NurbsIntersectionTrace::OffsetNurbs(source),
            NurbsIntersectionTrace::OffsetNurbs(copied),
        ) => {
            transformed_surface(source.basis(), copied.basis());
            assert_eq!(copied.signed_distance(), source.signed_distance());
            assert_eq!(
                copied.descriptor_signed_distances(),
                source.descriptor_signed_distances()
            );
        }
        (
            NurbsIntersectionTrace::OffsetPlane(source),
            NurbsIntersectionTrace::OffsetPlane(copied),
        ) => {
            assert_eq!(
                *copied.basis().frame(),
                transformed_frame(source.basis().frame())
            );
            assert_eq!(copied.signed_distance(), source.signed_distance());
        }
        _ => panic!("rigid copy changed the verified NURBS trace family or operand order"),
    }
}

fn copy_checked(
    store: &mut Store,
    source: ktopo::entity::BodyId,
    placement: Frame,
) -> (ktopo::entity::BodyId, ktopo::transaction::Journal) {
    let mut transaction = store.transaction().unwrap();
    let copied = transaction.copy_body_rigid(source, placement).unwrap();
    let journal = transaction.commit_checked_body(copied).unwrap();
    (copied, journal)
}

fn insert_plane_field(
    store: &mut Store,
    effective: Plane,
    offsets: &[f64],
) -> ktopo::entity::SurfaceId {
    let total = offsets.iter().sum::<f64>();
    let frame = effective.frame();
    let basis = Plane::new(frame.with_origin(frame.origin() - frame.z() * total));
    let mut root = store.insert_surface(SurfaceGeom::Plane(basis)).unwrap();
    for &distance in offsets {
        root = store
            .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                root, distance,
            )))
            .unwrap();
    }
    root
}

fn insert_sphere_field(
    store: &mut Store,
    effective: Sphere,
    offsets: &[f64],
) -> ktopo::entity::SurfaceId {
    let base_radius = effective.radius() - offsets.iter().sum::<f64>();
    let basis = Sphere::new(*effective.frame(), base_radius).unwrap();
    let mut root = store.insert_surface(SurfaceGeom::Sphere(basis)).unwrap();
    for &distance in offsets {
        root = store
            .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                root, distance,
            )))
            .unwrap();
    }
    root
}

fn surface_dependency_chain(
    store: &Store,
    root: ktopo::entity::SurfaceId,
) -> Vec<ktopo::entity::SurfaceId> {
    let mut chain = vec![root];
    let mut current = root;
    while let SurfaceGeom::Offset(offset) = store.get(current).unwrap() {
        current = offset.basis();
        chain.push(current);
    }
    chain
}

fn assert_copied_surface_chain(
    store: &Store,
    journal: &ktopo::transaction::Journal,
    copied_root: ktopo::entity::SurfaceId,
    source_root: ktopo::entity::SurfaceId,
) {
    let copied = surface_dependency_chain(store, copied_root);
    let source = surface_dependency_chain(store, source_root);
    assert_eq!(copied.len(), source.len());
    for (&copied, &source) in copied.iter().zip(&source) {
        assert_ne!(copied, source);
        match (store.get(copied).unwrap(), store.get(source).unwrap()) {
            (SurfaceGeom::Offset(copied), SurfaceGeom::Offset(source)) => {
                assert_eq!(copied.signed_distance(), source.signed_distance());
            }
            (SurfaceGeom::Plane(_), SurfaceGeom::Plane(_))
            | (SurfaceGeom::Sphere(_), SurfaceGeom::Sphere(_))
            | (SurfaceGeom::Nurbs(_), SurfaceGeom::Nurbs(_)) => {}
            _ => panic!("copy changed an exact-field dependency descriptor family"),
        }
        assert!(journal.lineage().iter().any(|event| matches!(
            event,
            LineageEvent::DerivedFrom {
                derived: ktopo::entity::EntityRef::Surface(derived),
                source: ktopo::entity::EntityRef::Surface(original),
            } if *derived == copied && *original == source
        )));
    }
}

fn linear_nurbs_curve(points: [Point3; 2]) -> NurbsCurve {
    NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], points.to_vec(), None).unwrap()
}

fn linear_nurbs_pcurve(points: [Point2; 2]) -> NurbsCurve2d {
    NurbsCurve2d::new(1, vec![0.0, 0.0, 1.0, 1.0], points.to_vec(), None).unwrap()
}

fn horizontal_nurbs_surface(z: f64, rational: bool) -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, z),
            Point3::new(0.0, 1.0, z),
            Point3::new(1.0, 0.0, z),
            Point3::new(1.0, 1.0, z),
        ],
        rational.then(|| vec![2.0; 4]),
    )
    .unwrap()
}

fn vertical_nurbs_surface(rational: bool) -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
        ],
        rational.then(|| vec![3.0; 4]),
    )
    .unwrap()
}

fn insert_offset_nurbs_field(
    store: &mut Store,
    basis: NurbsSurface,
    offsets: &[f64],
) -> ktopo::entity::SurfaceId {
    let mut root = store.insert_surface(SurfaceGeom::Nurbs(basis)).unwrap();
    for &distance in offsets {
        root = store
            .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                root, distance,
            )))
            .unwrap();
    }
    root
}

fn verified_nurbs_wire(
    store: &mut Store,
    surfaces: [ktopo::entity::SurfaceId; 2],
    pcurves: [NurbsCurve2d; 2],
    certificate: VerifiedNurbsIntersectionCertificate,
) -> (
    ktopo::entity::BodyId,
    ktopo::entity::CurveId,
    [ktopo::entity::Curve2dId; 2],
) {
    let carrier = certificate.carrier().clone();
    let range = certificate.carrier_range();
    let pcurves = pcurves.map(|pcurve| store.insert_pcurve(Curve2dGeom::Nurbs(pcurve)).unwrap());
    let curve = store
        .insert_verified_nurbs_intersection_curve(surfaces, pcurves, certificate)
        .unwrap();
    let mut transaction = store.transaction().unwrap();
    let body = {
        let mut assembly = transaction.assembly();
        let body = assembly.add(Body {
            kind: BodyKind::Wire,
            regions: Vec::new(),
        });
        let region = assembly.add(Region {
            body,
            kind: RegionKind::Void,
            shells: Vec::new(),
        });
        let shell = assembly.add(Shell {
            region,
            faces: Vec::new(),
            edges: Vec::new(),
            vertex: None,
        });
        let vertices = [carrier.eval(range.lo), carrier.eval(range.hi)].map(|position| {
            let point = assembly.add(position);
            assembly.add(Vertex {
                point,
                tolerance: None,
            })
        });
        let edge = assembly.add(Edge {
            curve: Some(curve),
            vertices: vertices.map(Some),
            bounds: Some((range.lo, range.hi)),
            fins: Vec::new(),
            tolerance: None,
        });
        assembly.get_mut(shell).unwrap().edges.push(edge);
        assembly.get_mut(region).unwrap().shells.push(shell);
        assembly.get_mut(body).unwrap().regions.push(region);
        body
    };
    transaction.commit_checked_body(body).unwrap();
    (body, curve, pcurves)
}

fn transmitted_wire(
    store: &mut Store,
    curve: ktopo::entity::CurveId,
    carrier: &NurbsCurve,
) -> ktopo::entity::BodyId {
    let range = carrier.param_range();
    let mut transaction = store.transaction().unwrap();
    let body = {
        let mut assembly = transaction.assembly();
        let body = assembly.add(Body {
            kind: BodyKind::Wire,
            regions: Vec::new(),
        });
        let region = assembly.add(Region {
            body,
            kind: RegionKind::Void,
            shells: Vec::new(),
        });
        let shell = assembly.add(Shell {
            region,
            faces: Vec::new(),
            edges: Vec::new(),
            vertex: None,
        });
        let vertices = [carrier.eval(range.lo), carrier.eval(range.hi)].map(|position| {
            let point = assembly.add(position);
            assembly.add(Vertex {
                point,
                tolerance: None,
            })
        });
        let edge = assembly.add(Edge {
            curve: Some(curve),
            vertices: vertices.map(Some),
            bounds: Some((range.lo, range.hi)),
            fins: Vec::new(),
            tolerance: None,
        });
        assembly.get_mut(shell).unwrap().edges.push(edge);
        assembly.get_mut(region).unwrap().shells.push(shell);
        assembly.get_mut(body).unwrap().regions.push(region);
        body
    };
    transaction.commit_checked_body(body).unwrap();
    body
}

fn transmitted_metadata() -> TransmittedIntersectionChartMetadata {
    TransmittedIntersectionChartMetadata::new(
        -0.25,
        1.5,
        2.0e-8,
        3.0e-8,
        [Some(4.0e-8), Some(5.0e-8)],
    )
    .unwrap()
}

fn transmitted_plane_wire(
    store: &mut Store,
    offsets: [&[f64]; 2],
) -> (
    ktopo::entity::BodyId,
    ktopo::entity::CurveId,
    [ktopo::entity::SurfaceId; 2],
    [ktopo::entity::Curve2dId; 2],
) {
    let planes = [
        Plane::new(Frame::world()),
        Plane::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        ),
    ];
    let carrier = linear_nurbs_curve([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
    let pcurves = [
        linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]),
        linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]),
    ];
    let certificate = certify_transmitted_plane_intersection_residuals(
        carrier.clone(),
        planes,
        pcurves.clone(),
        transmitted_metadata(),
        1.0e-10,
    )
    .unwrap();
    let surfaces = [
        insert_plane_field(store, planes[0], offsets[0]),
        insert_plane_field(store, planes[1], offsets[1]),
    ];
    let pcurve_handles =
        pcurves.map(|pcurve| store.insert_pcurve(Curve2dGeom::Nurbs(pcurve)).unwrap());
    let curve = store
        .insert_verified_transmitted_plane_intersection_curve(surfaces, pcurve_handles, certificate)
        .unwrap();
    let body = transmitted_wire(store, curve, &carrier);
    (body, curve, surfaces, pcurve_handles)
}

fn transmitted_offset_nurbs_wire(
    store: &mut Store,
    offset_first: bool,
) -> (
    ktopo::entity::BodyId,
    ktopo::entity::CurveId,
    [ktopo::entity::SurfaceId; 2],
    [ktopo::entity::Curve2dId; 2],
) {
    let offset_basis = horizontal_nurbs_surface(-0.25, true);
    let direct = vertical_nurbs_surface(false);
    let offset_root = insert_offset_nurbs_field(store, offset_basis.clone(), &[0.25]);
    let direct_root = store
        .insert_surface(SurfaceGeom::Nurbs(direct.clone()))
        .unwrap();
    let offset_trace = NurbsIntersectionTrace::OffsetNurbs(
        TransmittedOffsetNurbsTrace::from_descriptor_signed_distances(offset_basis, &[0.25])
            .unwrap(),
    );
    let direct_trace = NurbsIntersectionTrace::Nurbs(direct);
    let carrier = linear_nurbs_curve([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
    let common = linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]);
    let (surfaces, traces) = if offset_first {
        ([offset_root, direct_root], [offset_trace, direct_trace])
    } else {
        ([direct_root, offset_root], [direct_trace, offset_trace])
    };
    let pcurves = [common.clone(), common];
    let certificate = certify_transmitted_offset_nurbs_intersection_residuals(
        carrier.clone(),
        traces,
        pcurves.clone(),
        transmitted_metadata(),
        1.0e-8,
    )
    .unwrap();
    let pcurve_handles =
        pcurves.map(|pcurve| store.insert_pcurve(Curve2dGeom::Nurbs(pcurve)).unwrap());
    let curve = store
        .insert_verified_transmitted_nurbs_intersection_curve(surfaces, pcurve_handles, certificate)
        .unwrap();
    let body = transmitted_wire(store, curve, &carrier);
    (body, curve, surfaces, pcurve_handles)
}

fn transmitted_offset_nurbs_plane_wire(
    store: &mut Store,
    offset_first: bool,
    source_offset_distances: &[f64],
    trace_descriptor_distances: &[f64],
    plane_offset: Option<f64>,
) -> (
    ktopo::entity::BodyId,
    ktopo::entity::CurveId,
    [ktopo::entity::SurfaceId; 2],
    [ktopo::entity::Curve2dId; 2],
) {
    let offset_basis = horizontal_nurbs_surface(-0.25, true);
    let offset_root =
        insert_offset_nurbs_field(store, offset_basis.clone(), source_offset_distances);
    let plane = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let plane_root = if let Some(distance) = plane_offset {
        let frame = plane.frame();
        let basis = Plane::new(frame.with_origin(frame.origin() - frame.z() * distance));
        let basis = store.insert_surface(SurfaceGeom::Plane(basis)).unwrap();
        store
            .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                basis, distance,
            )))
            .unwrap()
    } else {
        store.insert_surface(SurfaceGeom::Plane(plane)).unwrap()
    };
    let offset_trace = NurbsIntersectionTrace::OffsetNurbs(
        TransmittedOffsetNurbsTrace::from_descriptor_signed_distances(
            offset_basis,
            trace_descriptor_distances,
        )
        .unwrap(),
    );
    let plane_trace = NurbsIntersectionTrace::Plane(plane);
    let carrier = linear_nurbs_curve([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
    let common = linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]);
    let (surfaces, traces) = if offset_first {
        ([offset_root, plane_root], [offset_trace, plane_trace])
    } else {
        ([plane_root, offset_root], [plane_trace, offset_trace])
    };
    let pcurves = [common.clone(), common];
    let certificate = certify_transmitted_offset_nurbs_intersection_residuals(
        carrier.clone(),
        traces,
        pcurves.clone(),
        transmitted_metadata(),
        1.0e-8,
    )
    .unwrap();
    let pcurve_handles =
        pcurves.map(|pcurve| store.insert_pcurve(Curve2dGeom::Nurbs(pcurve)).unwrap());
    let curve = store
        .insert_verified_transmitted_nurbs_intersection_curve(surfaces, pcurve_handles, certificate)
        .unwrap();
    let body = transmitted_wire(store, curve, &carrier);
    (body, curve, surfaces, pcurve_handles)
}

fn transmitted_dual_offset_wire(
    store: &mut Store,
    source_distances: [&[f64]; 2],
    trace_distances: [&[f64]; 2],
    shared_basis: bool,
    periodic_first: bool,
    sample_count: usize,
    swap_order: bool,
) -> (
    ktopo::entity::BodyId,
    ktopo::entity::CurveId,
    [ktopo::entity::SurfaceId; 2],
    [ktopo::entity::Curve2dId; 2],
) {
    let regular_first = horizontal_nurbs_surface(-0.25, false);
    let vertical = vertical_nurbs_surface(false);
    let regular_second = NurbsSurface::new(
        vertical.degree_u(),
        vertical.degree_v(),
        vertical.knots(kgeom::surface::Dir::U).as_slice().to_vec(),
        vertical.knots(kgeom::surface::Dir::V).as_slice().to_vec(),
        vertical
            .points()
            .iter()
            .map(|point| *point + Vec3::new(0.0, 0.5, 0.0))
            .collect(),
        vertical.weights().map(<[f64]>::to_vec),
    )
    .unwrap();
    let (first_basis, second_basis) = if periodic_first {
        let mut points = Vec::new();
        for point in [
            Point3::new(0.0, 0.25, 0.0),
            Point3::new(1.0, 0.25, 0.0),
            Point3::new(-1.0, 0.25, 0.0),
            Point3::new(0.0, 0.25, 0.0),
        ] {
            points.push(point);
            points.push(point + Vec3::new(0.0, 0.0, 1.0));
        }
        let periodic = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 2.0, 3.0, 3.0],
            vec![0.0, 0.0, 1.0, 1.0],
            points,
            None,
        )
        .unwrap()
        .with_certified_periodicity([true, false], 0.0)
        .unwrap();
        (periodic, horizontal_nurbs_surface(-0.5, false))
    } else if shared_basis {
        (regular_first.clone(), regular_first)
    } else {
        (regular_first, regular_second)
    };
    let first_basis_root = store
        .insert_surface(SurfaceGeom::Nurbs(first_basis.clone()))
        .unwrap();
    let second_basis_root = if shared_basis {
        first_basis_root
    } else {
        store
            .insert_surface(SurfaceGeom::Nurbs(second_basis.clone()))
            .unwrap()
    };
    let mut surfaces = [first_basis_root, second_basis_root];
    for index in 0..2 {
        for &distance in source_distances[index] {
            surfaces[index] = store
                .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                    surfaces[index],
                    distance,
                )))
                .unwrap();
        }
    }
    let mut traces = [
        NurbsIntersectionTrace::OffsetNurbs(
            TransmittedOffsetNurbsTrace::from_descriptor_signed_distances(
                first_basis,
                trace_distances[0],
            )
            .unwrap(),
        ),
        NurbsIntersectionTrace::OffsetNurbs(
            TransmittedOffsetNurbsTrace::from_descriptor_signed_distances(
                second_basis,
                trace_distances[1],
            )
            .unwrap(),
        ),
    ];
    if swap_order {
        surfaces.swap(0, 1);
        traces.swap(0, 1);
    }
    let quadratic_positions = [
        Point3::new(0.1, 0.0, 0.0),
        Point3::new(0.4, 0.0, 0.0),
        Point3::new(0.9, 0.0, 0.0),
    ];
    let quadratic_uv_samples = [[
        Point2::new(0.1, 0.0),
        Point2::new(0.4, 0.0),
        Point2::new(0.9, 0.0),
    ]; 2];
    let cubic_positions = [
        Point3::new(0.1, 0.0, 0.0),
        Point3::new(0.3, 0.0, 0.0),
        Point3::new(0.6, 0.0, 0.0),
        Point3::new(0.9, 0.0, 0.0),
    ];
    let cubic_uv_samples = [cubic_positions.map(|point| Point2::new(point.x, 0.0)); 2];
    let (carrier, common) = match sample_count {
        2 | 5 | 7 => {
            let (knots, coordinates) = match sample_count {
                2 => (vec![0.0, 0.0, 1.0, 1.0], vec![0.1, 0.9]),
                5 => (
                    vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0],
                    vec![0.1, 0.3, 0.5, 0.7, 0.9],
                ),
                7 => (
                    vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0],
                    vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 0.9],
                ),
                _ => unreachable!(),
            };
            (
                NurbsCurve::new(
                    1,
                    knots.clone(),
                    coordinates
                        .iter()
                        .map(|&coordinate| Point3::new(coordinate, 0.0, 0.0))
                        .collect(),
                    None,
                )
                .unwrap(),
                NurbsCurve2d::new(
                    1,
                    knots,
                    coordinates
                        .iter()
                        .map(|&coordinate| Point2::new(coordinate, 0.0))
                        .collect(),
                    None,
                )
                .unwrap(),
            )
        }
        3 => {
            let knots = vec![0.0, 0.0, 0.0, 2.0, 2.0, 2.0];
            let controls3 = vec![
                quadratic_positions[0],
                quadratic_positions[1] * 2.0
                    - (quadratic_positions[0] + quadratic_positions[2]) * 0.5,
                quadratic_positions[2],
            ];
            let samples = quadratic_uv_samples[0];
            let controls2 = vec![
                samples[0],
                samples[1] * 2.0 - (samples[0] + samples[2]) * 0.5,
                samples[2],
            ];
            (
                NurbsCurve::new(2, knots.clone(), controls3, None).unwrap(),
                NurbsCurve2d::new(2, knots, controls2, None).unwrap(),
            )
        }
        4 => {
            let knots = vec![0.0, 0.0, 0.0, 0.0, 3.0, 3.0, 3.0, 3.0];
            let controls3 = |samples: [Point3; 4]| {
                let first = samples[1] * 27.0 - samples[0] * 8.0 - samples[3];
                let second = samples[2] * 27.0 - samples[0] - samples[3] * 8.0;
                vec![
                    samples[0],
                    (first * 2.0 - second) / 18.0,
                    (second * 2.0 - first) / 18.0,
                    samples[3],
                ]
            };
            let controls2 = |samples: [Point2; 4]| {
                let first = samples[1] * 27.0 - samples[0] * 8.0 - samples[3];
                let second = samples[2] * 27.0 - samples[0] - samples[3] * 8.0;
                vec![
                    samples[0],
                    (first * 2.0 - second) / 18.0,
                    (second * 2.0 - first) / 18.0,
                    samples[3],
                ]
            };
            (
                NurbsCurve::new(3, knots.clone(), controls3(cubic_positions), None).unwrap(),
                NurbsCurve2d::new(3, knots, controls2(cubic_uv_samples[0]), None).unwrap(),
            )
        }
        _ => unreachable!(),
    };
    let pcurves = [common.clone(), common];
    let certificate = match sample_count {
        2 => certify_transmitted_two_sample_dual_offset_nurbs_intersection_residuals(
            carrier.clone(),
            traces,
            pcurves.clone(),
            transmitted_metadata(),
            1.0e-8,
        ),
        5 => certify_transmitted_five_sample_dual_offset_nurbs_intersection_residuals(
            carrier.clone(),
            traces,
            pcurves.clone(),
            transmitted_metadata(),
            1.0e-8,
        ),
        3 => certify_transmitted_quadratic_dual_offset_nurbs_intersection_residuals(
            carrier.clone(),
            traces,
            pcurves.clone(),
            quadratic_positions,
            quadratic_uv_samples,
            transmitted_metadata(),
            1.0e-8,
        ),
        4 => certify_transmitted_cubic_dual_offset_nurbs_intersection_residuals(
            carrier.clone(),
            traces,
            pcurves.clone(),
            cubic_positions,
            cubic_uv_samples,
            transmitted_metadata(),
            1.0e-8,
        ),
        7 => certify_transmitted_seven_sample_dual_offset_nurbs_intersection_residuals(
            carrier.clone(),
            traces,
            pcurves.clone(),
            transmitted_metadata(),
            1.0e-8,
        ),
        _ => unreachable!(),
    }
    .unwrap();
    let pcurve_handles =
        pcurves.map(|pcurve| store.insert_pcurve(Curve2dGeom::Nurbs(pcurve)).unwrap());
    let curve = store
        .insert_verified_transmitted_nurbs_intersection_curve(surfaces, pcurve_handles, certificate)
        .unwrap();
    let body = transmitted_wire(store, curve, &carrier);
    (body, curve, surfaces, pcurve_handles)
}

#[derive(Clone, Copy)]
enum TransmittedDirectNurbsFamily {
    PlaneNurbs { plane_first: bool },
    NurbsNurbs,
}

fn transmitted_direct_nurbs_wire(
    store: &mut Store,
    family: TransmittedDirectNurbsFamily,
) -> (
    ktopo::entity::BodyId,
    ktopo::entity::CurveId,
    [ktopo::entity::SurfaceId; 2],
    [ktopo::entity::Curve2dId; 2],
) {
    let carrier = linear_nurbs_curve([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
    let common = linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]);
    let pcurves = [common.clone(), common];
    let (surfaces, traces) = match family {
        TransmittedDirectNurbsFamily::PlaneNurbs { plane_first } => {
            let plane = Plane::new(Frame::world());
            let nurbs = vertical_nurbs_surface(true);
            let plane_root = store.insert_surface(SurfaceGeom::Plane(plane)).unwrap();
            let nurbs_root = store
                .insert_surface(SurfaceGeom::Nurbs(nurbs.clone()))
                .unwrap();
            let plane_trace = NurbsIntersectionTrace::Plane(plane);
            let nurbs_trace = NurbsIntersectionTrace::Nurbs(nurbs);
            if plane_first {
                ([plane_root, nurbs_root], [plane_trace, nurbs_trace])
            } else {
                ([nurbs_root, plane_root], [nurbs_trace, plane_trace])
            }
        }
        TransmittedDirectNurbsFamily::NurbsNurbs => {
            let horizontal = horizontal_nurbs_surface(0.0, true);
            let vertical = vertical_nurbs_surface(false);
            let surfaces = [horizontal.clone(), vertical.clone()]
                .map(|surface| store.insert_surface(SurfaceGeom::Nurbs(surface)).unwrap());
            (
                surfaces,
                [
                    NurbsIntersectionTrace::Nurbs(horizontal),
                    NurbsIntersectionTrace::Nurbs(vertical),
                ],
            )
        }
    };
    let certificate = match family {
        TransmittedDirectNurbsFamily::PlaneNurbs { .. } => {
            certify_transmitted_plane_nurbs_intersection_residuals(
                carrier.clone(),
                traces,
                pcurves.clone(),
                transmitted_metadata(),
                1.0e-8,
            )
        }
        TransmittedDirectNurbsFamily::NurbsNurbs => {
            certify_transmitted_nurbs_nurbs_intersection_residuals(
                carrier.clone(),
                traces,
                pcurves.clone(),
                transmitted_metadata(),
                1.0e-8,
            )
        }
    }
    .unwrap();
    let pcurve_handles =
        pcurves.map(|pcurve| store.insert_pcurve(Curve2dGeom::Nurbs(pcurve)).unwrap());
    let curve = store
        .insert_verified_transmitted_nurbs_intersection_curve(surfaces, pcurve_handles, certificate)
        .unwrap();
    let body = transmitted_wire(store, curve, &carrier);
    (body, curve, surfaces, pcurve_handles)
}

fn assert_verified_nurbs_copy(
    store: &mut Store,
    source: ktopo::entity::BodyId,
    source_curve: ktopo::entity::CurveId,
    source_surfaces: [ktopo::entity::SurfaceId; 2],
    source_pcurves: [ktopo::entity::Curve2dId; 2],
    rigid_placement: Frame,
) {
    let source_certificate = store
        .get(source_curve)
        .unwrap()
        .as_verified_nurbs_intersection()
        .unwrap()
        .certificate()
        .clone();
    let (copied, journal) = copy_checked(store, source, rigid_placement);
    let copied_curve = store
        .get(store.edges_of_body(copied).unwrap()[0])
        .unwrap()
        .curve
        .unwrap();
    let copied_descriptor = store
        .get(copied_curve)
        .unwrap()
        .as_verified_nurbs_intersection()
        .unwrap();
    let copied_certificate = copied_descriptor.certificate();
    assert_ne!(copied_curve, source_curve);
    assert_eq!(
        copied_certificate.carrier_range(),
        source_certificate.carrier_range()
    );
    assert_eq!(
        copied_certificate.carrier().degree(),
        source_certificate.carrier().degree()
    );
    assert_eq!(
        copied_certificate.carrier().knots(),
        source_certificate.carrier().knots()
    );
    assert_eq!(
        copied_certificate.carrier().weights(),
        source_certificate.carrier().weights()
    );
    assert_eq!(
        copied_certificate.carrier().points(),
        source_certificate
            .carrier()
            .points()
            .iter()
            .map(|&point| map_point(rigid_placement, point))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        copied_certificate.pcurves(),
        source_certificate.pcurves(),
        "rigid placement must not alter parameter-space traces"
    );
    assert_eq!(
        copied_certificate.tolerance(),
        source_certificate.tolerance()
    );
    assert_eq!(
        copied_certificate.proof_depth(),
        source_certificate.proof_depth()
    );
    for (source, copied) in source_certificate
        .traces()
        .iter()
        .zip(copied_certificate.traces())
    {
        assert_transformed_nurbs_trace(source, copied, rigid_placement);
    }
    for ((copied_root, source_root), (copied_pcurve, source_pcurve)) in copied_descriptor
        .source_surfaces()
        .into_iter()
        .zip(source_surfaces)
        .zip(copied_descriptor.pcurves().into_iter().zip(source_pcurves))
    {
        assert_ne!(copied_root, source_root);
        assert_ne!(copied_pcurve, source_pcurve);
        assert_copied_surface_chain(store, &journal, copied_root, source_root);
    }
    assert!(journal.lineage().iter().any(|event| matches!(
        event,
        LineageEvent::DerivedFrom {
            derived: ktopo::entity::EntityRef::Curve(derived),
            source: ktopo::entity::EntityRef::Curve(original),
        } if *derived == copied_curve && *original == source_curve
    )));
    store.geometry().validate().unwrap();
}

fn verified_plane_line_wire(
    store: &mut Store,
    offsets: [&[f64]; 2],
) -> (
    ktopo::entity::BodyId,
    ktopo::entity::CurveId,
    [ktopo::entity::SurfaceId; 2],
    [ktopo::entity::Curve2dId; 2],
) {
    let planes = [
        Plane::new(Frame::world()),
        Plane::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        ),
    ];
    let pcurves = [
        Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
        Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
    ];
    let maps = [
        AffineParamMap1d::new(1.0, 0.0).unwrap(),
        AffineParamMap1d::new(1.0, 0.0).unwrap(),
    ];
    let carrier = Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let range = ParamRange::new(-1.0, 2.0);
    let certificate =
        certify_paired_plane_line_residuals(carrier, range, planes, pcurves, maps, 1.0e-12)
            .unwrap();
    let surfaces = [
        insert_plane_field(store, planes[0], offsets[0]),
        insert_plane_field(store, planes[1], offsets[1]),
    ];
    let pcurve_handles =
        pcurves.map(|pcurve| store.insert_pcurve(Curve2dGeom::Line(pcurve)).unwrap());
    let curve = store
        .insert_verified_plane_intersection_curve(surfaces, pcurve_handles, certificate)
        .unwrap();

    let mut transaction = store.transaction().unwrap();
    let body = {
        let mut assembly = transaction.assembly();
        let body = assembly.add(Body {
            kind: BodyKind::Wire,
            regions: Vec::new(),
        });
        let region = assembly.add(Region {
            body,
            kind: RegionKind::Void,
            shells: Vec::new(),
        });
        let shell = assembly.add(Shell {
            region,
            faces: Vec::new(),
            edges: Vec::new(),
            vertex: None,
        });
        let points = [
            assembly.add(Point3::new(-1.0, 0.0, 0.0)),
            assembly.add(Point3::new(2.0, 0.0, 0.0)),
        ];
        let vertices = points.map(|point| {
            assembly.add(Vertex {
                point,
                tolerance: None,
            })
        });
        let edge = assembly.add(Edge {
            curve: Some(curve),
            vertices: vertices.map(Some),
            bounds: Some((range.lo, range.hi)),
            fins: Vec::new(),
            tolerance: None,
        });
        assembly.get_mut(shell).unwrap().edges.push(edge);
        assembly.get_mut(region).unwrap().shells.push(shell);
        assembly.get_mut(body).unwrap().regions.push(region);
        body
    };
    transaction.commit_checked_body(body).unwrap();
    (body, curve, surfaces, pcurve_handles)
}

fn wire_body_for_curve(
    store: &mut Store,
    curve: ktopo::entity::CurveId,
    carrier: Circle,
    range: ParamRange,
) -> ktopo::entity::BodyId {
    let mut transaction = store.transaction().unwrap();
    let body = {
        let mut assembly = transaction.assembly();
        let body = assembly.add(Body {
            kind: BodyKind::Wire,
            regions: Vec::new(),
        });
        let region = assembly.add(Region {
            body,
            kind: RegionKind::Void,
            shells: Vec::new(),
        });
        let shell = assembly.add(Shell {
            region,
            faces: Vec::new(),
            edges: Vec::new(),
            vertex: None,
        });
        let vertices = [carrier.eval(range.lo), carrier.eval(range.hi)].map(|position| {
            let point = assembly.add(position);
            assembly.add(Vertex {
                point,
                tolerance: None,
            })
        });
        let edge = assembly.add(Edge {
            curve: Some(curve),
            vertices: vertices.map(Some),
            bounds: Some((range.lo, range.hi)),
            fins: Vec::new(),
            tolerance: None,
        });
        assembly.get_mut(shell).unwrap().edges.push(edge);
        assembly.get_mut(region).unwrap().shells.push(shell);
        assembly.get_mut(body).unwrap().regions.push(region);
        body
    };
    transaction.commit_checked_body(body).unwrap();
    body
}

fn verified_aligned_plane_sphere_wire(
    store: &mut Store,
    plane_first: bool,
    plane_offsets: &[f64],
    sphere_offsets: &[f64],
) -> (
    ktopo::entity::BodyId,
    ktopo::entity::CurveId,
    [ktopo::entity::SurfaceId; 2],
    [ktopo::entity::Curve2dId; 2],
) {
    let height = 0.5;
    let plane = Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, height)));
    let sphere = Sphere::new(Frame::world(), 2.0).unwrap();
    let radius = (sphere.radius() * sphere.radius() - height * height).sqrt();
    let carrier = Circle::new(
        Frame::world().with_origin(Point3::new(0.0, 0.0, height)),
        radius,
    )
    .unwrap();
    let plane_pcurve = Circle2d::new(Point2::new(0.0, 0.0), radius, Vec2::new(1.0, 0.0)).unwrap();
    let latitude = kcore::math::atan2(height, radius);
    let sphere_pcurve = Line2d::new(Point2::new(0.0, latitude), Vec2::new(1.0, 0.0)).unwrap();
    let identity = AffineParamMap1d::new(1.0, 0.0).unwrap();
    let plane_trace =
        PlaneSphereCircleTrace::Plane(PlaneCircleTrace::new(plane, plane_pcurve, identity));
    let sphere_trace =
        PlaneSphereCircleTrace::Sphere(SphereLatitudeTrace::new(sphere, sphere_pcurve, identity));
    let traces = if plane_first {
        [plane_trace, sphere_trace]
    } else {
        [sphere_trace, plane_trace]
    };
    let range = ParamRange::new(0.25, 4.75);
    let certificate =
        certify_paired_plane_sphere_circle_residuals(carrier, range, traces, 1.0e-10).unwrap();

    let plane_source = insert_plane_field(store, plane, plane_offsets);
    let sphere_source = insert_sphere_field(store, sphere, sphere_offsets);
    let plane_pcurve = store
        .insert_pcurve(Curve2dGeom::Circle(plane_pcurve))
        .unwrap();
    let sphere_pcurve = store
        .insert_pcurve(Curve2dGeom::Line(sphere_pcurve))
        .unwrap();
    let surfaces = if plane_first {
        [plane_source, sphere_source]
    } else {
        [sphere_source, plane_source]
    };
    let pcurves = if plane_first {
        [plane_pcurve, sphere_pcurve]
    } else {
        [sphere_pcurve, plane_pcurve]
    };
    let curve = store
        .insert_verified_plane_sphere_intersection_curve(surfaces, pcurves, certificate)
        .unwrap();
    let body = wire_body_for_curve(store, curve, carrier, range);
    (body, curve, surfaces, pcurves)
}

fn collapsed_sphere_certificate_fixture(
    store: &mut Store,
) -> (
    [ktopo::entity::SurfaceId; 2],
    [ktopo::entity::SurfaceId; 2],
    [ktopo::entity::Curve2dId; 2],
    kgraph::PairedPlaneSphereCircleResidualCertificate,
) {
    let (_, curve, valid_surfaces, pcurves) =
        verified_aligned_plane_sphere_wire(store, true, &[], &[]);
    let certificate = store
        .get(curve)
        .unwrap()
        .as_intersection()
        .unwrap()
        .certificate()
        .as_plane_sphere_circle()
        .unwrap();
    let basis = store
        .insert_surface(SurfaceGeom::Sphere(
            Sphere::new(Frame::world(), 0.5).unwrap(),
        ))
        .unwrap();
    let collapsed = store
        .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
            basis, -0.5,
        )))
        .unwrap();
    let invalid_surfaces = [valid_surfaces[0], collapsed];
    (invalid_surfaces, valid_surfaces, pcurves, certificate)
}

fn verified_oblique_plane_sphere_wire(
    store: &mut Store,
    plane_first: bool,
    plane_offsets: &[f64],
    sphere_offsets: &[f64],
) -> (
    ktopo::entity::BodyId,
    ktopo::entity::CurveId,
    [ktopo::entity::SurfaceId; 2],
    [ktopo::entity::Curve2dId; 2],
) {
    let sphere = Sphere::new(Frame::world(), 2.5).unwrap();
    let normal = Vec3::new(0.0, 0.6, 0.8);
    let center = normal * 0.5;
    let frame = Frame::new(center, normal, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let plane = Plane::new(frame);
    let carrier = Circle::new(frame, (sphere.radius() * sphere.radius() - 0.25).sqrt()).unwrap();
    let plane_pcurve =
        Circle2d::new(Point2::new(0.0, 0.0), carrier.radius(), Vec2::new(1.0, 0.0)).unwrap();
    let range = ParamRange::new(0.2, 2.8);
    let sphere_window = [
        ParamRange::new(-core::f64::consts::PI, core::f64::consts::PI),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ];
    let sphere_longitude = |parameter| {
        let local = sphere.frame().to_local(carrier.eval(parameter));
        kcore::math::atan2(local.y, local.x)
    };
    let (sphere_pcurve, certificate) = certify_paired_plane_sphere_oblique_circle_residuals(
        carrier,
        range,
        plane,
        plane_pcurve,
        sphere,
        sphere_window,
        [sphere_longitude(range.lo), sphere_longitude(range.hi)],
        if plane_first {
            PairedTrace::First
        } else {
            PairedTrace::Second
        },
        1.0e-10,
    )
    .unwrap();
    let plane_source = insert_plane_field(store, plane, plane_offsets);
    let sphere_source = insert_sphere_field(store, sphere, sphere_offsets);
    let plane_pcurve = store
        .insert_pcurve(Curve2dGeom::Circle(plane_pcurve))
        .unwrap();
    let sphere_pcurve = store
        .insert_pcurve(Curve2dGeom::SphericalCircle(sphere_pcurve))
        .unwrap();
    let surfaces = if plane_first {
        [plane_source, sphere_source]
    } else {
        [sphere_source, plane_source]
    };
    let pcurves = if plane_first {
        [plane_pcurve, sphere_pcurve]
    } else {
        [sphere_pcurve, plane_pcurve]
    };
    let curve = store
        .insert_verified_plane_sphere_intersection_curve(surfaces, pcurves, certificate)
        .unwrap();
    let body = wire_body_for_curve(store, curve, carrier, range);
    (body, curve, surfaces, pcurves)
}

#[test]
fn rigid_copy_reissues_plane_and_sphere_nurbs_certificates_in_both_orders() {
    for analytic_first in [false, true] {
        let mut store = Store::new();
        let carrier = linear_nurbs_curve([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
        let pcurve = linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]);
        let plane = Plane::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        );
        let nurbs = vertical_nurbs_surface(true);
        let plane_source = insert_plane_field(&mut store, plane, &[0.125, 0.125]);
        let nurbs_source = store
            .insert_surface(SurfaceGeom::Nurbs(nurbs.clone()))
            .unwrap();
        let traces = if analytic_first {
            [
                NurbsIntersectionTrace::Plane(plane),
                NurbsIntersectionTrace::Nurbs(nurbs),
            ]
        } else {
            [
                NurbsIntersectionTrace::Nurbs(nurbs),
                NurbsIntersectionTrace::Plane(plane),
            ]
        };
        let surfaces = if analytic_first {
            [plane_source, nurbs_source]
        } else {
            [nurbs_source, plane_source]
        };
        let certificate = certify_verified_plane_nurbs_intersection_residuals(
            carrier,
            traces,
            [pcurve.clone(), pcurve.clone()],
            1.0e-10,
        )
        .unwrap();
        let (source, source_curve, source_pcurves) =
            verified_nurbs_wire(&mut store, surfaces, [pcurve.clone(), pcurve], certificate);
        assert_verified_nurbs_copy(
            &mut store,
            source,
            source_curve,
            surfaces,
            source_pcurves,
            oblique_placement(),
        );
    }

    for sphere_first in [false, true] {
        let mut store = Store::new();
        let longitude = 0.2_f64;
        let endpoints = [
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(
                kcore::math::cos(longitude),
                kcore::math::sin(longitude),
                0.0,
            ),
        ];
        let carrier = linear_nurbs_curve(endpoints);
        let sphere_pcurve =
            linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(longitude, 0.0)]);
        let nurbs_pcurve = linear_nurbs_pcurve([
            Point2::new(endpoints[0].x, endpoints[0].y),
            Point2::new(endpoints[1].x, endpoints[1].y),
        ]);
        let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
        let nurbs = horizontal_nurbs_surface(0.0, true);
        let sphere_source = insert_sphere_field(&mut store, sphere, &[0.25, -0.125]);
        let nurbs_source = store
            .insert_surface(SurfaceGeom::Nurbs(nurbs.clone()))
            .unwrap();
        let (surfaces, traces, pcurves) = if sphere_first {
            (
                [sphere_source, nurbs_source],
                [
                    NurbsIntersectionTrace::Sphere(sphere),
                    NurbsIntersectionTrace::Nurbs(nurbs),
                ],
                [sphere_pcurve, nurbs_pcurve],
            )
        } else {
            (
                [nurbs_source, sphere_source],
                [
                    NurbsIntersectionTrace::Nurbs(nurbs),
                    NurbsIntersectionTrace::Sphere(sphere),
                ],
                [nurbs_pcurve, sphere_pcurve],
            )
        };
        let certificate = certify_verified_sphere_nurbs_intersection_residuals(
            carrier,
            traces,
            pcurves.clone(),
            0.05,
        )
        .unwrap();
        let (source, source_curve, source_pcurves) =
            verified_nurbs_wire(&mut store, surfaces, pcurves, certificate);
        assert_verified_nurbs_copy(
            &mut store,
            source,
            source_curve,
            surfaces,
            source_pcurves,
            oblique_placement(),
        );
    }
}

#[test]
fn rigid_copy_reissues_direct_and_nested_offset_nurbs_source_pairs() {
    for offset_first in [false, true] {
        let mut store = Store::new();
        let carrier = linear_nurbs_curve([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
        let pcurve = linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]);
        let offsets = [0.025; 4];
        let basis = horizontal_nurbs_surface(-0.1, true);
        let direct = vertical_nurbs_surface(false);
        let offset_source = insert_offset_nurbs_field(&mut store, basis.clone(), &offsets);
        let direct_source = store
            .insert_surface(SurfaceGeom::Nurbs(direct.clone()))
            .unwrap();
        let offset_trace = NurbsIntersectionTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(
            basis,
            offsets.iter().sum(),
        ));
        let direct_trace = NurbsIntersectionTrace::Nurbs(direct);
        let (surfaces, traces) = if offset_first {
            ([offset_source, direct_source], [offset_trace, direct_trace])
        } else {
            ([direct_source, offset_source], [direct_trace, offset_trace])
        };
        let certificate = certify_verified_offset_nurbs_nurbs_intersection_residuals(
            carrier,
            traces,
            [pcurve.clone(), pcurve.clone()],
            1.0e-10,
        )
        .unwrap();
        let (source, source_curve, source_pcurves) =
            verified_nurbs_wire(&mut store, surfaces, [pcurve.clone(), pcurve], certificate);
        assert_verified_nurbs_copy(
            &mut store,
            source,
            source_curve,
            surfaces,
            source_pcurves,
            placement(),
        );
    }

    let mut store = Store::new();
    let carrier = linear_nurbs_curve([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
    let pcurve = linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]);
    let first = horizontal_nurbs_surface(0.0, true);
    let second = vertical_nurbs_surface(true);
    let surfaces = [
        store
            .insert_surface(SurfaceGeom::Nurbs(first.clone()))
            .unwrap(),
        store
            .insert_surface(SurfaceGeom::Nurbs(second.clone()))
            .unwrap(),
    ];
    let certificate = certify_verified_nurbs_nurbs_intersection_residuals(
        carrier,
        [
            NurbsIntersectionTrace::Nurbs(first),
            NurbsIntersectionTrace::Nurbs(second),
        ],
        [pcurve.clone(), pcurve.clone()],
        1.0e-10,
    )
    .unwrap();
    let (source, source_curve, source_pcurves) =
        verified_nurbs_wire(&mut store, surfaces, [pcurve.clone(), pcurve], certificate);
    assert_verified_nurbs_copy(
        &mut store,
        source,
        source_curve,
        surfaces,
        source_pcurves,
        placement(),
    );
}

#[test]
fn rigid_copy_reissues_offset_nurbs_with_direct_and_offset_plane_traces() {
    for plane_is_offset in [false, true] {
        for plane_first in [false, true] {
            let mut store = Store::new();
            let carrier =
                linear_nurbs_curve([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
            let pcurve = linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]);
            let basis = horizontal_nurbs_surface(-0.1, false);
            let offset_source = insert_offset_nurbs_field(&mut store, basis.clone(), &[0.1]);
            let effective_plane = Plane::new(
                Frame::new(
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            );
            let (plane_source, plane_trace) = if plane_is_offset {
                let distance = 0.25;
                let frame = effective_plane.frame();
                let plane_basis =
                    Plane::new(frame.with_origin(frame.origin() - frame.z() * distance));
                let basis_handle = store
                    .insert_surface(SurfaceGeom::Plane(plane_basis))
                    .unwrap();
                let root = store
                    .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                        basis_handle,
                        distance,
                    )))
                    .unwrap();
                (
                    root,
                    NurbsIntersectionTrace::OffsetPlane(TransmittedOffsetPlaneTrace::new(
                        plane_basis,
                        distance,
                    )),
                )
            } else {
                (
                    store
                        .insert_surface(SurfaceGeom::Plane(effective_plane))
                        .unwrap(),
                    NurbsIntersectionTrace::Plane(effective_plane),
                )
            };
            let offset_trace =
                NurbsIntersectionTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(basis, 0.1));
            let (surfaces, traces) = if plane_first {
                ([plane_source, offset_source], [plane_trace, offset_trace])
            } else {
                ([offset_source, plane_source], [offset_trace, plane_trace])
            };
            let certificate = certify_verified_offset_nurbs_plane_intersection_residuals(
                carrier,
                traces,
                [pcurve.clone(), pcurve.clone()],
                1.0e-10,
            )
            .unwrap();
            let (source, source_curve, source_pcurves) =
                verified_nurbs_wire(&mut store, surfaces, [pcurve.clone(), pcurve], certificate);
            assert_verified_nurbs_copy(
                &mut store,
                source,
                source_curve,
                surfaces,
                source_pcurves,
                placement(),
            );
        }
    }
}

#[test]
fn rigid_copy_reissues_two_independent_nested_offset_nurbs_roots() {
    let mut store = Store::new();
    let carrier = linear_nurbs_curve([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
    let pcurve = linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]);
    let first_offsets = [0.025; 4];
    let second_offsets = [0.05, 0.05];
    let first_basis = horizontal_nurbs_surface(-0.1, true);
    let mut second_basis = vertical_nurbs_surface(true);
    second_basis = NurbsSurface::new(
        second_basis.degree_u(),
        second_basis.degree_v(),
        second_basis
            .knots(kgeom::surface::Dir::U)
            .as_slice()
            .to_vec(),
        second_basis
            .knots(kgeom::surface::Dir::V)
            .as_slice()
            .to_vec(),
        second_basis
            .points()
            .iter()
            .map(|point| *point + Vec3::new(0.0, 0.1, 0.0))
            .collect(),
        second_basis.weights().map(<[f64]>::to_vec),
    )
    .unwrap();
    let first_source = insert_offset_nurbs_field(&mut store, first_basis.clone(), &first_offsets);
    let second_source =
        insert_offset_nurbs_field(&mut store, second_basis.clone(), &second_offsets);
    let surfaces = [first_source, second_source];
    let certificate = certify_verified_dual_offset_nurbs_intersection_residuals(
        carrier,
        [
            NurbsIntersectionTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(
                first_basis,
                first_offsets.iter().sum(),
            )),
            NurbsIntersectionTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(
                second_basis,
                second_offsets.iter().sum(),
            )),
        ],
        [pcurve.clone(), pcurve.clone()],
        1.0e-10,
    )
    .unwrap();
    let (source, source_curve, source_pcurves) =
        verified_nurbs_wire(&mut store, surfaces, [pcurve.clone(), pcurve], certificate);
    assert_verified_nurbs_copy(
        &mut store,
        source,
        source_curve,
        surfaces,
        source_pcurves,
        oblique_placement(),
    );
}

#[test]
fn verified_nurbs_source_binding_rejects_altered_and_overdeep_offset_roots_atomically() {
    let mut store = Store::new();
    let carrier = linear_nurbs_curve([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
    let pcurve = linear_nurbs_pcurve([Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]);
    let basis = horizontal_nurbs_surface(-0.1, true);
    let direct = vertical_nurbs_surface(false);
    let traces = [
        NurbsIntersectionTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(basis.clone(), 0.1)),
        NurbsIntersectionTrace::Nurbs(direct.clone()),
    ];
    let certificate = certify_verified_offset_nurbs_nurbs_intersection_residuals(
        carrier,
        traces,
        [pcurve.clone(), pcurve.clone()],
        1.0e-10,
    )
    .unwrap();
    let basis_handle = store.insert_surface(SurfaceGeom::Nurbs(basis)).unwrap();
    let direct_handle = store.insert_surface(SurfaceGeom::Nurbs(direct)).unwrap();
    let altered = store
        .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
            basis_handle,
            0.2,
        )))
        .unwrap();
    let mut overdeep = basis_handle;
    for _ in 0..5 {
        overdeep = store
            .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                overdeep, 0.02,
            )))
            .unwrap();
    }
    let pcurves = [pcurve.clone(), pcurve]
        .map(|pcurve| store.insert_pcurve(Curve2dGeom::Nurbs(pcurve)).unwrap());
    let before = store.count::<CurveGeom>();
    assert!(
        store
            .insert_verified_nurbs_intersection_curve(
                [altered, direct_handle],
                pcurves,
                certificate.clone(),
            )
            .is_err()
    );
    assert_eq!(store.count::<CurveGeom>(), before);
    assert!(
        store
            .insert_verified_nurbs_intersection_curve(
                [overdeep, direct_handle],
                pcurves,
                certificate.clone(),
            )
            .is_err()
    );
    assert_eq!(store.count::<CurveGeom>(), before);

    let mut valid = basis_handle;
    for _ in 0..4 {
        valid = store
            .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                valid, 0.025,
            )))
            .unwrap();
    }
    let accepted = store
        .insert_verified_nurbs_intersection_curve([valid, direct_handle], pcurves, certificate)
        .unwrap();
    assert_eq!(store.count::<CurveGeom>(), before + 1);
    assert!(
        store
            .get(accepted)
            .unwrap()
            .as_verified_nurbs_intersection()
            .is_some()
    );
    store.geometry().validate().unwrap();
}

#[test]
fn rigid_block_copy_duplicates_complete_ownership_and_records_lineage() {
    let mut store = Store::new();
    let source = make::block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
    let source_vertices = store.vertices_of_body(source).unwrap();
    let source_positions: Vec<_> = source_vertices
        .iter()
        .map(|&vertex| store.vertex_position(vertex).unwrap())
        .collect();
    let source_edges: HashSet<_> = store.edges_of_body(source).unwrap().into_iter().collect();
    let source_faces: HashSet<_> = store.faces_of_body(source).unwrap().into_iter().collect();

    let (copied, journal) = copy_checked(&mut store, source, placement());
    assert_ne!(source, copied);
    assert_eq!(
        check_body_report(&store, copied, CheckLevel::Full)
            .unwrap()
            .outcome(),
        CheckOutcome::Valid
    );
    let copied_vertices = store.vertices_of_body(copied).unwrap();
    let copied_positions: Vec<_> = copied_vertices
        .iter()
        .map(|&vertex| store.vertex_position(vertex).unwrap())
        .collect();
    assert_eq!(
        copied_positions,
        source_positions
            .into_iter()
            .map(|point| map_point(placement(), point))
            .collect::<Vec<_>>()
    );
    assert!(
        store
            .edges_of_body(copied)
            .unwrap()
            .iter()
            .all(|edge| !source_edges.contains(edge))
    );
    assert!(
        store
            .faces_of_body(copied)
            .unwrap()
            .iter()
            .all(|face| !source_faces.contains(face))
    );

    let copied_curves: HashSet<_> = store
        .edges_of_body(copied)
        .unwrap()
        .into_iter()
        .filter_map(|edge| store.get(edge).unwrap().curve)
        .collect();
    let source_curves: HashSet<_> = source_edges
        .iter()
        .filter_map(|&edge| store.get(edge).unwrap().curve)
        .collect();
    assert!(copied_curves.is_disjoint(&source_curves));
    let copied_surfaces: HashSet<_> = store
        .faces_of_body(copied)
        .unwrap()
        .into_iter()
        .map(|face| store.get(face).unwrap().surface)
        .collect();
    let source_surfaces: HashSet<_> = source_faces
        .iter()
        .map(|&face| store.get(face).unwrap().surface)
        .collect();
    assert!(copied_surfaces.is_disjoint(&source_surfaces));

    assert!(
        journal
            .lineage()
            .iter()
            .all(|event| matches!(event, LineageEvent::DerivedFrom { .. }))
    );
    assert!(journal.lineage().iter().any(|event| matches!(
        event,
        LineageEvent::DerivedFrom {
            derived: ktopo::entity::EntityRef::Body(derived),
            source: ktopo::entity::EntityRef::Body(original),
        } if *derived == copied && *original == source
    )));
}

#[test]
fn rigid_holed_sheet_copy_preserves_pcurves_full_proof_and_area() {
    let outer = [
        Point2::new(-2.0, -2.0),
        Point2::new(2.0, -2.0),
        Point2::new(2.0, 2.0),
        Point2::new(-2.0, 2.0),
    ];
    let hole = [
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ];
    let profile = PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&hole]).unwrap();
    let mut store = Store::new();
    let source = make::planar_sheet_from_profile(&mut store, &profile).unwrap();
    let (copied, _) = copy_checked(&mut store, source, placement());
    assert_eq!(
        check_body_report(&store, copied, CheckLevel::Full)
            .unwrap()
            .outcome(),
        CheckOutcome::Valid
    );
    for edge in store.edges_of_body(copied).unwrap() {
        assert!(
            store
                .get(edge)
                .unwrap()
                .fins
                .iter()
                .all(|&fin| { store.get(fin).unwrap().pcurve.is_some() })
        );
    }
    let mesh = tessellate_body(
        &store,
        copied,
        &TessOptions {
            chord_tol: 1.0e-3,
            max_edge_len: Some(0.5),
        },
    )
    .unwrap();
    let area = mesh
        .triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.map(|index| mesh.positions[index as usize]);
            (b - a).cross(c - a).norm() * 0.5
        })
        .sum::<f64>();
    assert!((area - 12.0).abs() <= 1.0e-10);
}

#[test]
fn rigid_copy_reissues_transmitted_plane_proof_metadata_and_is_repeat_deterministic() {
    let run = || {
        let mut store = Store::new();
        let (source, source_curve, source_surfaces, source_pcurves) =
            transmitted_plane_wire(&mut store, [&[0.125, -0.125], &[-0.25, 0.25]]);
        let source_certificate = store
            .get(source_curve)
            .unwrap()
            .as_transmitted_intersection()
            .unwrap()
            .certificate()
            .clone();

        let first_chain = surface_dependency_chain(&store, source_surfaces[0]);
        {
            let mut transaction = store.transaction().unwrap();
            assert!(
                transaction
                    .assembly()
                    .remove_surface(source_surfaces[0])
                    .is_err(),
                "a transmitted curve must keep its ordered root live"
            );
            assert!(
                transaction
                    .assembly()
                    .replace_surface(
                        *first_chain.last().unwrap(),
                        SurfaceGeom::Plane(Plane::new(
                            Frame::world().with_origin(Point3::new(0.0, 0.0, 0.5)),
                        )),
                    )
                    .is_err(),
                "a transmitted curve must protect every transitive basis from alteration"
            );
        }

        let (copied, journal) = copy_checked(&mut store, source, oblique_placement());
        let copied_curve = store
            .get(store.edges_of_body(copied).unwrap()[0])
            .unwrap()
            .curve
            .unwrap();
        let copied_descriptor = store
            .get(copied_curve)
            .unwrap()
            .as_transmitted_intersection()
            .unwrap();
        let copied_certificate = copied_descriptor.certificate();
        assert_eq!(copied_certificate.metadata(), source_certificate.metadata());
        assert_eq!(
            copied_certificate.tolerance(),
            source_certificate.tolerance()
        );
        assert_eq!(copied_certificate.pcurves(), source_certificate.pcurves());
        assert_eq!(
            copied_certificate.carrier().points(),
            source_certificate
                .carrier()
                .points()
                .iter()
                .map(|&point| map_point(oblique_placement(), point))
                .collect::<Vec<_>>()
        );
        for ((copied_root, source_root), (copied_pcurve, source_pcurve)) in copied_descriptor
            .source_surfaces()
            .into_iter()
            .zip(source_surfaces)
            .zip(copied_descriptor.pcurves().into_iter().zip(source_pcurves))
        {
            assert_ne!(copied_root, source_root);
            assert_ne!(copied_pcurve, source_pcurve);
            assert_copied_surface_chain(&store, &journal, copied_root, source_root);
        }
        assert!(journal.lineage().iter().any(|event| matches!(
            event,
            LineageEvent::DerivedFrom {
                derived: ktopo::entity::EntityRef::Curve(derived),
                source: ktopo::entity::EntityRef::Curve(original),
            } if *derived == copied_curve && *original == source_curve
        )));
        store.geometry().validate().unwrap();
        (copied, journal, store.get(copied_curve).unwrap().clone())
    };

    assert_eq!(run(), run());
}

#[test]
fn rigid_copy_reissues_direct_transmitted_nurbs_families_in_both_orders() {
    let families = [
        TransmittedDirectNurbsFamily::PlaneNurbs { plane_first: true },
        TransmittedDirectNurbsFamily::PlaneNurbs { plane_first: false },
        TransmittedDirectNurbsFamily::NurbsNurbs,
    ];
    for family in families {
        let mut store = Store::new();
        let (source, source_curve, source_surfaces, source_pcurves) =
            transmitted_direct_nurbs_wire(&mut store, family);
        let source_certificate = store
            .get(source_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap()
            .certificate()
            .clone();
        let (copied, journal) = copy_checked(&mut store, source, oblique_placement());
        let copied_curve = store
            .get(store.edges_of_body(copied).unwrap()[0])
            .unwrap()
            .curve
            .unwrap();
        let copied_descriptor = store
            .get(copied_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap();
        let copied_certificate = copied_descriptor.certificate();
        assert_eq!(copied_certificate.metadata(), source_certificate.metadata());
        assert_eq!(
            copied_certificate.tolerance(),
            source_certificate.tolerance()
        );
        assert_eq!(copied_certificate.pcurves(), source_certificate.pcurves());
        assert_eq!(
            copied_certificate.carrier().points(),
            source_certificate
                .carrier()
                .points()
                .iter()
                .map(|&point| map_point(oblique_placement(), point))
                .collect::<Vec<_>>()
        );
        for (source_trace, copied_trace) in source_certificate
            .traces()
            .iter()
            .zip(copied_certificate.traces())
        {
            assert_transformed_nurbs_trace(source_trace, copied_trace, oblique_placement());
        }
        for ((copied_root, source_root), (copied_pcurve, source_pcurve)) in copied_descriptor
            .source_surfaces()
            .into_iter()
            .zip(source_surfaces)
            .zip(copied_descriptor.pcurves().into_iter().zip(source_pcurves))
        {
            assert_ne!(copied_root, source_root);
            assert_ne!(copied_pcurve, source_pcurve);
            assert_copied_surface_chain(&store, &journal, copied_root, source_root);
        }
        assert!(journal.lineage().iter().any(|event| matches!(
            event,
            LineageEvent::DerivedFrom {
                derived: ktopo::entity::EntityRef::Curve(derived),
                source: ktopo::entity::EntityRef::Curve(original),
            } if *derived == copied_curve && *original == source_curve
        )));
        store.geometry().validate().unwrap();
    }
}

#[test]
fn rigid_copy_reissues_direct_transmitted_offset_nurbs_in_both_orders() {
    for offset_first in [false, true] {
        let mut store = Store::new();
        let (source, source_curve, source_surfaces, source_pcurves) =
            transmitted_offset_nurbs_wire(&mut store, offset_first);
        let source_certificate = store
            .get(source_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap()
            .certificate()
            .clone();
        let (copied, journal) = copy_checked(&mut store, source, oblique_placement());
        let copied_curve = store
            .get(store.edges_of_body(copied).unwrap()[0])
            .unwrap()
            .curve
            .unwrap();
        let copied_descriptor = store
            .get(copied_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap();
        let copied_certificate = copied_descriptor.certificate();
        assert_eq!(copied_certificate.metadata(), source_certificate.metadata());
        assert_eq!(
            copied_certificate.tolerance(),
            source_certificate.tolerance()
        );
        assert_eq!(copied_certificate.pcurves(), source_certificate.pcurves());
        assert_eq!(
            copied_certificate.carrier().points(),
            source_certificate
                .carrier()
                .points()
                .iter()
                .map(|&point| map_point(oblique_placement(), point))
                .collect::<Vec<_>>()
        );
        assert_eq!(
            matches!(
                copied_certificate.traces()[0],
                NurbsIntersectionTrace::OffsetNurbs(_)
            ),
            offset_first
        );
        for (source_trace, copied_trace) in source_certificate
            .traces()
            .iter()
            .zip(copied_certificate.traces())
        {
            assert_transformed_nurbs_trace(source_trace, copied_trace, oblique_placement());
        }
        for ((copied_root, source_root), (copied_pcurve, source_pcurve)) in copied_descriptor
            .source_surfaces()
            .into_iter()
            .zip(source_surfaces)
            .zip(copied_descriptor.pcurves().into_iter().zip(source_pcurves))
        {
            assert_ne!(copied_root, source_root);
            assert_ne!(copied_pcurve, source_pcurve);
            assert_copied_surface_chain(&store, &journal, copied_root, source_root);
        }
        store.geometry().validate().unwrap();
    }
}

#[test]
fn rigid_copy_reissues_transmitted_offset_nurbs_direct_plane_in_both_orders() {
    let mut copied_certificates = Vec::new();
    for offset_first in [false, true] {
        let mut store = Store::new();
        let (source, source_curve, source_surfaces, source_pcurves) =
            transmitted_offset_nurbs_plane_wire(&mut store, offset_first, &[0.25], &[0.25], None);
        let source_certificate = store
            .get(source_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap()
            .certificate()
            .clone();
        let offset_index = usize::from(!offset_first);
        let source_chain = surface_dependency_chain(&store, source_surfaces[offset_index]);
        {
            let mut transaction = store.transaction().unwrap();
            assert!(
                transaction
                    .assembly()
                    .remove_surface(source_surfaces[offset_index])
                    .is_err()
            );
            assert!(
                transaction
                    .assembly()
                    .replace_surface(
                        *source_chain.last().unwrap(),
                        SurfaceGeom::Nurbs(horizontal_nurbs_surface(-0.2, true)),
                    )
                    .is_err()
            );
        }

        let (copied, journal) = copy_checked(&mut store, source, oblique_placement());
        let copied_curve = store
            .get(store.edges_of_body(copied).unwrap()[0])
            .unwrap()
            .curve
            .unwrap();
        let copied_descriptor = store
            .get(copied_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap();
        let copied_surfaces = copied_descriptor.source_surfaces();
        let copied_pcurves = copied_descriptor.pcurves();
        let copied_certificate = copied_descriptor.certificate().clone();
        assert_eq!(copied_certificate.metadata(), source_certificate.metadata());
        assert_eq!(
            copied_certificate.tolerance(),
            source_certificate.tolerance()
        );
        assert_eq!(copied_certificate.pcurves(), source_certificate.pcurves());
        assert_eq!(
            copied_certificate.carrier().points(),
            source_certificate
                .carrier()
                .points()
                .iter()
                .map(|&point| map_point(oblique_placement(), point))
                .collect::<Vec<_>>()
        );
        assert_eq!(
            matches!(
                copied_certificate.traces()[0],
                NurbsIntersectionTrace::OffsetNurbs(_)
            ),
            offset_first
        );
        for (source_trace, copied_trace) in source_certificate
            .traces()
            .iter()
            .zip(copied_certificate.traces())
        {
            assert_transformed_nurbs_trace(source_trace, copied_trace, oblique_placement());
        }
        let recertified = certify_transmitted_offset_nurbs_intersection_residuals(
            copied_certificate.carrier().clone(),
            copied_certificate.traces().clone(),
            copied_certificate.pcurves().clone(),
            copied_certificate.metadata(),
            copied_certificate.tolerance(),
        )
        .unwrap();
        assert_eq!(recertified, copied_certificate);
        for ((copied_root, source_root), (copied_pcurve, source_pcurve)) in copied_surfaces
            .into_iter()
            .zip(source_surfaces)
            .zip(copied_pcurves.into_iter().zip(source_pcurves))
        {
            assert_ne!(copied_root, source_root);
            assert_ne!(copied_pcurve, source_pcurve);
            assert_copied_surface_chain(&store, &journal, copied_root, source_root);
        }
        let retained = copied_surfaces
            .into_iter()
            .flat_map(|root| surface_dependency_chain(&store, root))
            .collect::<Vec<_>>();
        {
            let mut transaction = store.transaction().unwrap();
            for retained in retained {
                assert!(transaction.assembly().remove_surface(retained).is_err());
            }
        }
        store.geometry().validate().unwrap();
        copied_certificates.push(copied_certificate);
    }
    assert_eq!(
        copied_certificates[0].traces()[0],
        copied_certificates[1].traces()[1]
    );
    assert_eq!(
        copied_certificates[0].traces()[1],
        copied_certificates[1].traces()[0]
    );
}

#[test]
fn unsupported_transmitted_offset_nurbs_plane_sources_roll_back_without_allocation() {
    for (source_distances, trace_distances, plane_offset) in [
        (vec![0.25], vec![0.125, 0.125], None),
        (vec![0.25], vec![0.25], Some(0.05)),
    ] {
        for offset_first in [false, true] {
            let mut attempted = Store::new();
            let (source, _, _, _) = transmitted_offset_nurbs_plane_wire(
                &mut attempted,
                offset_first,
                &source_distances,
                &trace_distances,
                plane_offset,
            );
            let before = (
                attempted.count::<Body>(),
                attempted.count::<Region>(),
                attempted.count::<Shell>(),
                attempted.count::<Edge>(),
                attempted.count::<Vertex>(),
                attempted.count::<CurveGeom>(),
                attempted.count::<SurfaceGeom>(),
                attempted.count::<Curve2dGeom>(),
                attempted.count::<Point3>(),
            );
            let mut control = attempted.clone();
            {
                let mut transaction = attempted.transaction().unwrap();
                assert!(
                    transaction
                        .copy_body_rigid(source, oblique_placement())
                        .is_err()
                );
            }
            assert_eq!(
                before,
                (
                    attempted.count::<Body>(),
                    attempted.count::<Region>(),
                    attempted.count::<Shell>(),
                    attempted.count::<Edge>(),
                    attempted.count::<Vertex>(),
                    attempted.count::<CurveGeom>(),
                    attempted.count::<SurfaceGeom>(),
                    attempted.count::<Curve2dGeom>(),
                    attempted.count::<Point3>(),
                )
            );
            let next = attempted.insert_point(Point3::new(4.0, 5.0, 6.0)).unwrap();
            let control_next = control.insert_point(Point3::new(4.0, 5.0, 6.0)).unwrap();
            assert_eq!(next, control_next);
            attempted.geometry().validate().unwrap();
        }
    }
}

#[test]
fn rigid_copy_reissues_canonical_two_sample_dual_offset_in_both_orders() {
    let mut copied_certificates = Vec::new();
    for swap_order in [false, true] {
        let mut store = Store::new();
        let first = [0.25];
        let second = [0.5];
        let (source, source_curve, source_surfaces, source_pcurves) = transmitted_dual_offset_wire(
            &mut store,
            [&first, &second],
            [&first, &second],
            false,
            false,
            2,
            swap_order,
        );
        let source_certificate = store
            .get(source_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap()
            .certificate()
            .clone();
        assert_ne!(
            *surface_dependency_chain(&store, source_surfaces[0])
                .last()
                .unwrap(),
            *surface_dependency_chain(&store, source_surfaces[1])
                .last()
                .unwrap()
        );
        let (copied, journal) = copy_checked(&mut store, source, oblique_placement());
        let copied_curve = store
            .get(store.edges_of_body(copied).unwrap()[0])
            .unwrap()
            .curve
            .unwrap();
        let copied_descriptor = store
            .get(copied_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap();
        let copied_surfaces = copied_descriptor.source_surfaces();
        let copied_pcurves = copied_descriptor.pcurves();
        let copied_certificate = copied_descriptor.certificate().clone();
        assert_eq!(copied_certificate.metadata(), source_certificate.metadata());
        assert_eq!(
            copied_certificate.tolerance(),
            source_certificate.tolerance()
        );
        assert_eq!(copied_certificate.pcurves(), source_certificate.pcurves());
        assert_eq!(
            copied_certificate.carrier().points(),
            source_certificate
                .carrier()
                .points()
                .iter()
                .map(|&point| map_point(oblique_placement(), point))
                .collect::<Vec<_>>()
        );
        for (source_trace, copied_trace) in source_certificate
            .traces()
            .iter()
            .zip(copied_certificate.traces())
        {
            assert_transformed_nurbs_trace(source_trace, copied_trace, oblique_placement());
        }
        assert_eq!(
            certify_transmitted_two_sample_dual_offset_nurbs_intersection_residuals(
                copied_certificate.carrier().clone(),
                copied_certificate.traces().clone(),
                copied_certificate.pcurves().clone(),
                copied_certificate.metadata(),
                copied_certificate.tolerance(),
            )
            .unwrap(),
            copied_certificate
        );
        for ((copied_root, source_root), (copied_pcurve, source_pcurve)) in copied_surfaces
            .into_iter()
            .zip(source_surfaces)
            .zip(copied_pcurves.into_iter().zip(source_pcurves))
        {
            assert_ne!(copied_root, source_root);
            assert_ne!(copied_pcurve, source_pcurve);
            assert_copied_surface_chain(&store, &journal, copied_root, source_root);
        }
        let copied_basis_roots =
            copied_surfaces.map(|root| *surface_dependency_chain(&store, root).last().unwrap());
        assert_ne!(copied_basis_roots[0], copied_basis_roots[1]);
        let retained = copied_surfaces
            .into_iter()
            .flat_map(|root| surface_dependency_chain(&store, root))
            .collect::<Vec<_>>();
        {
            let mut transaction = store.transaction().unwrap();
            for retained in retained {
                assert!(transaction.assembly().remove_surface(retained).is_err());
            }
        }
        store.geometry().validate().unwrap();
        copied_certificates.push(copied_certificate);
    }
    assert_eq!(
        copied_certificates[0].traces()[0],
        copied_certificates[1].traces()[1]
    );
    assert_eq!(
        copied_certificates[0].traces()[1],
        copied_certificates[1].traces()[0]
    );
}

#[test]
fn rigid_copy_reissues_witnessed_quadratic_dual_offset_in_both_orders() {
    let mut copied_certificates = Vec::new();
    for swap_order in [false, true] {
        let mut store = Store::new();
        let first = [0.25];
        let second = [0.5];
        let (source, source_curve, source_surfaces, source_pcurves) = transmitted_dual_offset_wire(
            &mut store,
            [&first, &second],
            [&first, &second],
            false,
            false,
            3,
            swap_order,
        );
        let source_certificate = store
            .get(source_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap()
            .certificate()
            .clone();
        let source_witnesses = source_certificate
            .quadratic_interpolation_witnesses()
            .unwrap();
        let source_basis_roots =
            source_surfaces.map(|root| *surface_dependency_chain(&store, root).last().unwrap());
        assert_ne!(source_basis_roots[0], source_basis_roots[1]);
        {
            let mut transaction = store.transaction().unwrap();
            assert!(
                transaction
                    .assembly()
                    .remove_surface(source_surfaces[0])
                    .is_err()
            );
            assert!(
                transaction
                    .assembly()
                    .replace_surface(
                        source_basis_roots[1],
                        SurfaceGeom::Nurbs(horizontal_nurbs_surface(-0.75, false)),
                    )
                    .is_err()
            );
        }

        let (copied, journal) = copy_checked(&mut store, source, oblique_placement());
        let copied_curve = store
            .get(store.edges_of_body(copied).unwrap()[0])
            .unwrap()
            .curve
            .unwrap();
        let copied_descriptor = store
            .get(copied_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap();
        let copied_surfaces = copied_descriptor.source_surfaces();
        let copied_pcurves = copied_descriptor.pcurves();
        let copied_certificate = copied_descriptor.certificate().clone();
        let copied_witnesses = copied_certificate
            .quadratic_interpolation_witnesses()
            .unwrap();
        assert_eq!(copied_certificate.metadata(), source_certificate.metadata());
        assert_eq!(
            copied_certificate.tolerance(),
            source_certificate.tolerance()
        );
        assert_eq!(copied_certificate.pcurves(), source_certificate.pcurves());
        assert_eq!(
            copied_witnesses.positions(),
            source_witnesses
                .positions()
                .map(|point| map_point(oblique_placement(), point))
        );
        assert_eq!(
            copied_witnesses.canonicalized_pcurve_points(),
            source_witnesses.canonicalized_pcurve_points()
        );
        assert_eq!(
            copied_certificate.carrier().points(),
            &[
                copied_witnesses.positions()[0],
                copied_witnesses.positions()[1] * 2.0
                    - (copied_witnesses.positions()[0] + copied_witnesses.positions()[2]) * 0.5,
                copied_witnesses.positions()[2],
            ]
        );
        assert!(
            copied_certificate
                .carrier()
                .points()
                .iter()
                .zip(source_certificate.carrier().points())
                .all(|(&copied, &source)| {
                    copied.dist(map_point(oblique_placement(), source)) <= 16.0 * f64::EPSILON
                })
        );
        for (source_trace, copied_trace) in source_certificate
            .traces()
            .iter()
            .zip(copied_certificate.traces())
        {
            assert_transformed_nurbs_trace(source_trace, copied_trace, oblique_placement());
        }
        assert_eq!(
            certify_transmitted_quadratic_dual_offset_nurbs_intersection_residuals(
                copied_certificate.carrier().clone(),
                copied_certificate.traces().clone(),
                copied_certificate.pcurves().clone(),
                copied_witnesses.positions(),
                copied_witnesses.canonicalized_pcurve_points(),
                copied_certificate.metadata(),
                copied_certificate.tolerance(),
            )
            .unwrap(),
            copied_certificate
        );
        let mut altered_positions = copied_witnesses.positions();
        altered_positions[1] += Vec3::new(0.0, 0.0, 1.0e-4);
        assert!(
            certify_transmitted_quadratic_dual_offset_nurbs_intersection_residuals(
                copied_certificate.carrier().clone(),
                copied_certificate.traces().clone(),
                copied_certificate.pcurves().clone(),
                altered_positions,
                copied_witnesses.canonicalized_pcurve_points(),
                copied_certificate.metadata(),
                copied_certificate.tolerance(),
            )
            .is_err()
        );
        for ((copied_root, source_root), (copied_pcurve, source_pcurve)) in copied_surfaces
            .into_iter()
            .zip(source_surfaces)
            .zip(copied_pcurves.into_iter().zip(source_pcurves))
        {
            assert_ne!(copied_root, source_root);
            assert_ne!(copied_pcurve, source_pcurve);
            assert_copied_surface_chain(&store, &journal, copied_root, source_root);
        }
        let copied_basis_roots =
            copied_surfaces.map(|root| *surface_dependency_chain(&store, root).last().unwrap());
        assert_ne!(copied_basis_roots[0], copied_basis_roots[1]);
        store.geometry().validate().unwrap();
        copied_certificates.push(copied_certificate);
    }
    assert_eq!(
        copied_certificates[0].traces()[0],
        copied_certificates[1].traces()[1]
    );
    assert_eq!(
        copied_certificates[0].traces()[1],
        copied_certificates[1].traces()[0]
    );
}

#[test]
fn rigid_copy_reissues_witnessed_cubic_dual_offset_in_both_orders() {
    let mut copied_certificates = Vec::new();
    for swap_order in [false, true] {
        let mut store = Store::new();
        let first = [0.25];
        let second = [0.5];
        let (source, source_curve, source_surfaces, source_pcurves) = transmitted_dual_offset_wire(
            &mut store,
            [&first, &second],
            [&first, &second],
            false,
            false,
            4,
            swap_order,
        );
        let source_certificate = store
            .get(source_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap()
            .certificate()
            .clone();
        let source_witnesses = source_certificate.cubic_interpolation_witnesses().unwrap();
        let source_basis_roots =
            source_surfaces.map(|root| *surface_dependency_chain(&store, root).last().unwrap());
        assert_ne!(source_basis_roots[0], source_basis_roots[1]);
        {
            let mut transaction = store.transaction().unwrap();
            assert!(
                transaction
                    .assembly()
                    .remove_surface(source_surfaces[0])
                    .is_err()
            );
            assert!(
                transaction
                    .assembly()
                    .replace_surface(
                        source_basis_roots[1],
                        SurfaceGeom::Nurbs(horizontal_nurbs_surface(-0.75, false)),
                    )
                    .is_err()
            );
        }

        let (copied, journal) = copy_checked(&mut store, source, oblique_placement());
        let copied_curve = store
            .get(store.edges_of_body(copied).unwrap()[0])
            .unwrap()
            .curve
            .unwrap();
        let copied_descriptor = store
            .get(copied_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap();
        let copied_surfaces = copied_descriptor.source_surfaces();
        let copied_pcurves = copied_descriptor.pcurves();
        let copied_certificate = copied_descriptor.certificate().clone();
        let copied_witnesses = copied_certificate.cubic_interpolation_witnesses().unwrap();
        assert_eq!(copied_certificate.metadata(), source_certificate.metadata());
        assert_eq!(
            copied_certificate.tolerance(),
            source_certificate.tolerance()
        );
        assert_eq!(copied_certificate.pcurves(), source_certificate.pcurves());
        assert_eq!(
            copied_witnesses.positions(),
            source_witnesses
                .positions()
                .map(|point| map_point(oblique_placement(), point))
        );
        assert_eq!(
            copied_witnesses.canonicalized_pcurve_points(),
            source_witnesses.canonicalized_pcurve_points()
        );
        let positions = copied_witnesses.positions();
        let first_control = positions[1] * 27.0 - positions[0] * 8.0 - positions[3];
        let second_control = positions[2] * 27.0 - positions[0] - positions[3] * 8.0;
        assert_eq!(
            copied_certificate.carrier().points(),
            &[
                positions[0],
                (first_control * 2.0 - second_control) / 18.0,
                (second_control * 2.0 - first_control) / 18.0,
                positions[3],
            ]
        );
        assert!(
            copied_certificate
                .carrier()
                .points()
                .iter()
                .zip(source_certificate.carrier().points())
                .all(|(&copied, &source)| {
                    copied.dist(map_point(oblique_placement(), source)) <= 64.0 * f64::EPSILON
                })
        );
        for (source_trace, copied_trace) in source_certificate
            .traces()
            .iter()
            .zip(copied_certificate.traces())
        {
            assert_transformed_nurbs_trace(source_trace, copied_trace, oblique_placement());
        }
        assert_eq!(
            certify_transmitted_cubic_dual_offset_nurbs_intersection_residuals(
                copied_certificate.carrier().clone(),
                copied_certificate.traces().clone(),
                copied_certificate.pcurves().clone(),
                copied_witnesses.positions(),
                copied_witnesses.canonicalized_pcurve_points(),
                copied_certificate.metadata(),
                copied_certificate.tolerance(),
            )
            .unwrap(),
            copied_certificate
        );
        let mut altered_positions = copied_witnesses.positions();
        altered_positions[2] += Vec3::new(0.0, 0.0, 1.0e-4);
        assert!(
            certify_transmitted_cubic_dual_offset_nurbs_intersection_residuals(
                copied_certificate.carrier().clone(),
                copied_certificate.traces().clone(),
                copied_certificate.pcurves().clone(),
                altered_positions,
                copied_witnesses.canonicalized_pcurve_points(),
                copied_certificate.metadata(),
                copied_certificate.tolerance(),
            )
            .is_err()
        );
        for ((copied_root, source_root), (copied_pcurve, source_pcurve)) in copied_surfaces
            .into_iter()
            .zip(source_surfaces)
            .zip(copied_pcurves.into_iter().zip(source_pcurves))
        {
            assert_ne!(copied_root, source_root);
            assert_ne!(copied_pcurve, source_pcurve);
            assert_copied_surface_chain(&store, &journal, copied_root, source_root);
        }
        let copied_basis_roots =
            copied_surfaces.map(|root| *surface_dependency_chain(&store, root).last().unwrap());
        assert_ne!(copied_basis_roots[0], copied_basis_roots[1]);
        store.geometry().validate().unwrap();
        copied_certificates.push(copied_certificate);
    }
    assert_eq!(
        copied_certificates[0].traces()[0],
        copied_certificates[1].traces()[1]
    );
    assert_eq!(
        copied_certificates[0].traces()[1],
        copied_certificates[1].traces()[0]
    );
}

#[test]
fn rigid_copy_reissues_canonical_five_sample_dual_offset_in_both_orders() {
    let mut copied_certificates = Vec::new();
    for swap_order in [false, true] {
        let mut store = Store::new();
        let first = [0.25];
        let second = [0.5];
        let (source, source_curve, source_surfaces, source_pcurves) = transmitted_dual_offset_wire(
            &mut store,
            [&first, &second],
            [&first, &second],
            false,
            false,
            5,
            swap_order,
        );
        let source_certificate = store
            .get(source_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap()
            .certificate()
            .clone();
        assert!(
            source_certificate
                .quadratic_interpolation_witnesses()
                .is_none()
        );
        assert!(source_certificate.cubic_interpolation_witnesses().is_none());
        let source_basis_roots =
            source_surfaces.map(|root| *surface_dependency_chain(&store, root).last().unwrap());
        assert_ne!(source_basis_roots[0], source_basis_roots[1]);

        let (copied, journal) = copy_checked(&mut store, source, oblique_placement());
        let copied_curve = store
            .get(store.edges_of_body(copied).unwrap()[0])
            .unwrap()
            .curve
            .unwrap();
        let copied_descriptor = store
            .get(copied_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap();
        let copied_surfaces = copied_descriptor.source_surfaces();
        let copied_pcurves = copied_descriptor.pcurves();
        let copied_certificate = copied_descriptor.certificate().clone();
        assert_eq!(copied_certificate.metadata(), source_certificate.metadata());
        assert_eq!(
            copied_certificate.tolerance(),
            source_certificate.tolerance()
        );
        assert_eq!(copied_certificate.pcurves(), source_certificate.pcurves());
        assert_eq!(
            copied_certificate.carrier().points(),
            source_certificate
                .carrier()
                .points()
                .iter()
                .map(|&point| map_point(oblique_placement(), point))
                .collect::<Vec<_>>()
        );
        for (source_trace, copied_trace) in source_certificate
            .traces()
            .iter()
            .zip(copied_certificate.traces())
        {
            assert_transformed_nurbs_trace(source_trace, copied_trace, oblique_placement());
        }
        assert_eq!(
            certify_transmitted_five_sample_dual_offset_nurbs_intersection_residuals(
                copied_certificate.carrier().clone(),
                copied_certificate.traces().clone(),
                copied_certificate.pcurves().clone(),
                copied_certificate.metadata(),
                copied_certificate.tolerance(),
            )
            .unwrap(),
            copied_certificate
        );
        for ((copied_root, source_root), (copied_pcurve, source_pcurve)) in copied_surfaces
            .into_iter()
            .zip(source_surfaces)
            .zip(copied_pcurves.into_iter().zip(source_pcurves))
        {
            assert_ne!(copied_root, source_root);
            assert_ne!(copied_pcurve, source_pcurve);
            assert_copied_surface_chain(&store, &journal, copied_root, source_root);
        }
        let copied_basis_roots =
            copied_surfaces.map(|root| *surface_dependency_chain(&store, root).last().unwrap());
        assert_ne!(copied_basis_roots[0], copied_basis_roots[1]);
        let retained = copied_surfaces
            .into_iter()
            .flat_map(|root| surface_dependency_chain(&store, root))
            .collect::<Vec<_>>();
        {
            let mut transaction = store.transaction().unwrap();
            for retained in retained {
                assert!(transaction.assembly().remove_surface(retained).is_err());
            }
        }
        store.geometry().validate().unwrap();
        copied_certificates.push(copied_certificate);
    }
    assert_eq!(
        copied_certificates[0].traces()[0],
        copied_certificates[1].traces()[1]
    );
    assert_eq!(
        copied_certificates[0].traces()[1],
        copied_certificates[1].traces()[0]
    );
}

#[test]
fn rigid_copy_reissues_canonical_seven_sample_dual_offset_in_both_orders() {
    let mut copied_certificates = Vec::new();
    for swap_order in [false, true] {
        let mut store = Store::new();
        let first = [0.25];
        let second = [0.5];
        let (source, source_curve, source_surfaces, source_pcurves) = transmitted_dual_offset_wire(
            &mut store,
            [&first, &second],
            [&first, &second],
            false,
            false,
            7,
            swap_order,
        );
        let source_certificate = store
            .get(source_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap()
            .certificate()
            .clone();
        assert!(
            source_certificate
                .quadratic_interpolation_witnesses()
                .is_none()
        );
        assert!(source_certificate.cubic_interpolation_witnesses().is_none());
        let source_basis_roots =
            source_surfaces.map(|root| *surface_dependency_chain(&store, root).last().unwrap());
        assert_ne!(source_basis_roots[0], source_basis_roots[1]);

        let (copied, journal) = copy_checked(&mut store, source, oblique_placement());
        let copied_curve = store
            .get(store.edges_of_body(copied).unwrap()[0])
            .unwrap()
            .curve
            .unwrap();
        let copied_descriptor = store
            .get(copied_curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap();
        let copied_surfaces = copied_descriptor.source_surfaces();
        let copied_pcurves = copied_descriptor.pcurves();
        let copied_certificate = copied_descriptor.certificate().clone();
        assert_eq!(copied_certificate.metadata(), source_certificate.metadata());
        assert_eq!(
            copied_certificate.tolerance(),
            source_certificate.tolerance()
        );
        assert_eq!(copied_certificate.pcurves(), source_certificate.pcurves());
        assert_eq!(
            copied_certificate.carrier().points(),
            source_certificate
                .carrier()
                .points()
                .iter()
                .map(|&point| map_point(oblique_placement(), point))
                .collect::<Vec<_>>()
        );
        for (source_trace, copied_trace) in source_certificate
            .traces()
            .iter()
            .zip(copied_certificate.traces())
        {
            assert_transformed_nurbs_trace(source_trace, copied_trace, oblique_placement());
        }
        assert_eq!(
            certify_transmitted_seven_sample_dual_offset_nurbs_intersection_residuals(
                copied_certificate.carrier().clone(),
                copied_certificate.traces().clone(),
                copied_certificate.pcurves().clone(),
                copied_certificate.metadata(),
                copied_certificate.tolerance(),
            )
            .unwrap(),
            copied_certificate
        );
        for ((copied_root, source_root), (copied_pcurve, source_pcurve)) in copied_surfaces
            .into_iter()
            .zip(source_surfaces)
            .zip(copied_pcurves.into_iter().zip(source_pcurves))
        {
            assert_ne!(copied_root, source_root);
            assert_ne!(copied_pcurve, source_pcurve);
            assert_copied_surface_chain(&store, &journal, copied_root, source_root);
        }
        let copied_basis_roots =
            copied_surfaces.map(|root| *surface_dependency_chain(&store, root).last().unwrap());
        assert_ne!(copied_basis_roots[0], copied_basis_roots[1]);
        let retained = copied_surfaces
            .into_iter()
            .flat_map(|root| surface_dependency_chain(&store, root))
            .collect::<Vec<_>>();
        {
            let mut transaction = store.transaction().unwrap();
            for retained in retained {
                assert!(transaction.assembly().remove_surface(retained).is_err());
            }
        }
        store.geometry().validate().unwrap();
        copied_certificates.push(copied_certificate);
    }
    assert_eq!(
        copied_certificates[0].traces()[0],
        copied_certificates[1].traces()[1]
    );
    assert_eq!(
        copied_certificates[0].traces()[1],
        copied_certificates[1].traces()[0]
    );
}

#[test]
fn unsupported_dual_offset_transmitted_families_roll_back_without_allocation() {
    let first = [0.25];
    let first_split = [0.125, 0.125];
    let second = [0.5];
    let shared_second = [0.25];
    for (trace_distances, shared_basis, periodic_first, sample_count) in [
        ((&first_split[..], &second[..]), false, false, 2),
        ((&first_split[..], &second[..]), false, false, 3),
        ((&first_split[..], &second[..]), false, false, 4),
        ((&first[..], &shared_second[..]), true, false, 2),
        ((&first[..], &shared_second[..]), true, false, 3),
        ((&first[..], &shared_second[..]), true, false, 4),
        ((&first_split[..], &second[..]), false, false, 5),
        ((&first[..], &shared_second[..]), true, false, 5),
        ((&first[..], &second[..]), false, true, 5),
        ((&first_split[..], &second[..]), false, false, 7),
        ((&first[..], &shared_second[..]), true, false, 7),
        ((&first[..], &second[..]), false, true, 7),
        ((&first[..], &second[..]), false, true, 2),
        ((&first[..], &second[..]), false, true, 3),
        ((&first[..], &second[..]), false, true, 4),
    ] {
        for swap_order in [false, true] {
            let source_distances = if shared_basis {
                [&first[..], &shared_second[..]]
            } else {
                [&first[..], &second[..]]
            };
            let mut attempted = Store::new();
            let (source, _, _, _) = transmitted_dual_offset_wire(
                &mut attempted,
                source_distances,
                [trace_distances.0, trace_distances.1],
                shared_basis,
                periodic_first,
                sample_count,
                swap_order,
            );
            let before = (
                attempted.count::<Body>(),
                attempted.count::<Region>(),
                attempted.count::<Shell>(),
                attempted.count::<Edge>(),
                attempted.count::<Vertex>(),
                attempted.count::<CurveGeom>(),
                attempted.count::<SurfaceGeom>(),
                attempted.count::<Curve2dGeom>(),
                attempted.count::<Point3>(),
            );
            let mut control = attempted.clone();
            {
                let mut transaction = attempted.transaction().unwrap();
                assert!(
                    transaction
                        .copy_body_rigid(source, oblique_placement())
                        .is_err()
                );
            }
            assert_eq!(
                before,
                (
                    attempted.count::<Body>(),
                    attempted.count::<Region>(),
                    attempted.count::<Shell>(),
                    attempted.count::<Edge>(),
                    attempted.count::<Vertex>(),
                    attempted.count::<CurveGeom>(),
                    attempted.count::<SurfaceGeom>(),
                    attempted.count::<Curve2dGeom>(),
                    attempted.count::<Point3>(),
                )
            );
            assert_eq!(
                attempted.insert_point(Point3::new(7.0, 8.0, 9.0)).unwrap(),
                control.insert_point(Point3::new(7.0, 8.0, 9.0)).unwrap()
            );
            attempted.geometry().validate().unwrap();
        }
    }
}

#[test]
fn rejected_overdeep_transmitted_copy_rolls_back_and_reuses_future_identity() {
    let mut attempted = Store::new();
    let overdeep = vec![0.0; EvalLimits::default().max_dependency_depth];
    let (unsupported, _, _, _) = transmitted_plane_wire(&mut attempted, [&overdeep, &[]]);
    let before = (
        attempted.count::<Body>(),
        attempted.count::<Region>(),
        attempted.count::<Shell>(),
        attempted.count::<Edge>(),
        attempted.count::<Vertex>(),
        attempted.count::<CurveGeom>(),
        attempted.count::<SurfaceGeom>(),
        attempted.count::<Curve2dGeom>(),
        attempted.count::<Point3>(),
    );
    {
        let mut transaction = attempted.transaction().unwrap();
        assert!(
            transaction
                .copy_body_rigid(unsupported, placement())
                .is_err()
        );
    }
    assert_eq!(
        before,
        (
            attempted.count::<Body>(),
            attempted.count::<Region>(),
            attempted.count::<Shell>(),
            attempted.count::<Edge>(),
            attempted.count::<Vertex>(),
            attempted.count::<CurveGeom>(),
            attempted.count::<SurfaceGeom>(),
            attempted.count::<Curve2dGeom>(),
            attempted.count::<Point3>(),
        )
    );

    let (valid, _, _, _) = transmitted_plane_wire(&mut attempted, [&[], &[]]);
    let (copied_after, journal_after) = copy_checked(&mut attempted, valid, placement());
    let mut control = Store::new();
    let _ = transmitted_plane_wire(&mut control, [&overdeep, &[]]);
    let (control_source, _, _, _) = transmitted_plane_wire(&mut control, [&[], &[]]);
    let (copied_control, journal_control) = copy_checked(&mut control, control_source, placement());
    assert_eq!(copied_after, copied_control);
    assert_eq!(journal_after, journal_control);
}

#[test]
fn rigid_copy_reissues_plane_line_intersection_certificate() {
    let cases: [(&[f64], &[f64]); 2] = [(&[], &[]), (&[0.25, -0.125], &[-0.5])];
    for (first_offsets, second_offsets) in cases {
        let mut store = Store::new();
        let (source, source_curve, source_surfaces, source_pcurves) =
            verified_plane_line_wire(&mut store, [first_offsets, second_offsets]);

        let (copied, journal) = copy_checked(&mut store, source, placement());
        let copied_edge = store.edges_of_body(copied).unwrap()[0];
        let copied_curve = store.get(copied_edge).unwrap().curve.unwrap();
        assert_ne!(copied_curve, source_curve);
        let copied_intersection = store
            .get(copied_curve)
            .unwrap()
            .as_intersection()
            .copied()
            .unwrap();
        assert!(
            copied_intersection
                .source_surfaces()
                .into_iter()
                .zip(source_surfaces)
                .all(|(copied, source)| copied != source)
        );
        assert!(
            copied_intersection
                .pcurves()
                .into_iter()
                .zip(source_pcurves)
                .all(|(copied, source)| copied != source)
        );
        let VerifiedIntersectionCertificate::PlaneLine(certificate) =
            copied_intersection.certificate()
        else {
            panic!("copy changed the intersection certificate family");
        };
        assert_eq!(
            certificate.carrier(),
            Line::new(
                map_point(placement(), Point3::new(0.0, 0.0, 0.0)),
                placement().x(),
            )
            .unwrap()
        );
        assert_eq!(certificate.carrier_range(), ParamRange::new(-1.0, 2.0));
        for (index, surface) in copied_intersection
            .source_surfaces()
            .into_iter()
            .enumerate()
        {
            let mut evaluator = store.eval_context(EvalLimits::default(), Tolerances::default());
            assert_eq!(
                evaluator.surface_exact_field(surface).unwrap(),
                Some(ExactSurfaceField::Plane(certificate.surfaces()[index]))
            );
        }
        for (index, pcurve) in copied_intersection.pcurves().into_iter().enumerate() {
            assert_eq!(
                store.get(pcurve).unwrap().as_line().copied(),
                Some(certificate.pcurves()[index])
            );
        }
        assert!(journal.lineage().iter().any(|event| matches!(
            event,
            LineageEvent::DerivedFrom {
                derived: ktopo::entity::EntityRef::Curve(derived),
                source: ktopo::entity::EntityRef::Curve(original),
            } if *derived == copied_curve && *original == source_curve
        )));
        for (copied_root, source_root) in copied_intersection
            .source_surfaces()
            .into_iter()
            .zip(source_surfaces)
        {
            assert_copied_surface_chain(&store, &journal, copied_root, source_root);
        }
        store.geometry().validate().unwrap();
    }
}

#[test]
fn rigid_copy_reissues_ordered_aligned_plane_sphere_circle_certificate() {
    let cases: [(&[f64], &[f64]); 4] = [
        (&[], &[]),
        (&[0.25, -0.125], &[]),
        (&[], &[0.5, -0.125]),
        (&[0.25, -0.125], &[0.5, -0.125]),
    ];
    for plane_first in [false, true] {
        for (plane_offsets, sphere_offsets) in cases {
            let mut store = Store::new();
            let (source, source_curve, source_surfaces, source_pcurves) =
                verified_aligned_plane_sphere_wire(
                    &mut store,
                    plane_first,
                    plane_offsets,
                    sphere_offsets,
                );

            let (copied, journal) = copy_checked(&mut store, source, placement());
            let copied_edge = store.edges_of_body(copied).unwrap()[0];
            let copied_curve = store.get(copied_edge).unwrap().curve.unwrap();
            assert_ne!(copied_curve, source_curve);
            let copied_intersection = store
                .get(copied_curve)
                .unwrap()
                .as_intersection()
                .copied()
                .unwrap();
            assert!(
                copied_intersection
                    .source_surfaces()
                    .into_iter()
                    .zip(source_surfaces)
                    .all(|(copied, source)| copied != source)
            );
            assert!(
                copied_intersection
                    .pcurves()
                    .into_iter()
                    .zip(source_pcurves)
                    .all(|(copied, source)| copied != source)
            );
            let VerifiedIntersectionCertificate::PlaneSphereCircle(certificate) =
                copied_intersection.certificate()
            else {
                panic!("copy changed the intersection certificate family");
            };
            assert_eq!(certificate.carrier_range(), ParamRange::new(0.25, 4.75));
            let carrier = certificate.carrier();
            assert_eq!(
                carrier.frame().origin(),
                map_point(placement(), Point3::new(0.0, 0.0, 0.5))
            );
            assert_eq!(carrier.frame().x(), placement().x());
            assert_eq!(carrier.frame().y(), placement().y());
            assert_eq!(
                matches!(certificate.traces()[0], PlaneSphereCircleTrace::Plane(_)),
                plane_first
            );
            assert!(matches!(
                certificate.traces()[usize::from(plane_first)],
                PlaneSphereCircleTrace::Sphere(_)
            ));

            for ((surface, pcurve), trace) in copied_intersection
                .source_surfaces()
                .into_iter()
                .zip(copied_intersection.pcurves())
                .zip(certificate.traces())
            {
                match trace {
                    PlaneSphereCircleTrace::Plane(trace) => {
                        let mut evaluator =
                            store.eval_context(EvalLimits::default(), Tolerances::default());
                        assert_eq!(
                            evaluator.surface_exact_field(surface).unwrap(),
                            Some(ExactSurfaceField::Plane(trace.surface()))
                        );
                        assert_eq!(
                            store.get(pcurve).unwrap().as_circle(),
                            Some(&trace.pcurve())
                        );
                    }
                    PlaneSphereCircleTrace::Sphere(trace) => {
                        let mut evaluator =
                            store.eval_context(EvalLimits::default(), Tolerances::default());
                        assert_eq!(
                            evaluator.surface_exact_field(surface).unwrap(),
                            Some(ExactSurfaceField::Sphere(trace.surface()))
                        );
                        assert_eq!(store.get(pcurve).unwrap().as_line(), Some(&trace.pcurve()));
                    }
                    PlaneSphereCircleTrace::SphereOblique(_) => {
                        panic!("aligned copy changed the sphere trace family");
                    }
                }
            }
            for parameter in [
                certificate.carrier_range().lo,
                certificate.carrier_range().lerp(0.41),
                certificate.carrier_range().hi,
            ] {
                let point = carrier.eval(parameter);
                for ((surface, pcurve), map) in copied_intersection
                    .source_surfaces()
                    .into_iter()
                    .zip(copied_intersection.pcurves())
                    .zip(certificate.parameter_maps())
                {
                    let uv = store
                        .get(pcurve)
                        .unwrap()
                        .as_curve()
                        .eval(map.map(parameter));
                    let lifted = store
                        .eval_context(EvalLimits::default(), Tolerances::default())
                        .eval_surface(surface, [uv.x, uv.y], SurfaceDerivativeOrder::Position)
                        .unwrap()
                        .p;
                    assert!(point.dist(lifted) <= certificate.tolerance());
                }
            }
            assert!(journal.lineage().iter().any(|event| matches!(
                event,
                LineageEvent::DerivedFrom {
                    derived: ktopo::entity::EntityRef::Curve(derived),
                    source: ktopo::entity::EntityRef::Curve(original),
                } if *derived == copied_curve && *original == source_curve
            )));
            for (copied_root, source_root) in copied_intersection
                .source_surfaces()
                .into_iter()
                .zip(source_surfaces)
            {
                assert_copied_surface_chain(&store, &journal, copied_root, source_root);
            }
            store.geometry().validate().unwrap();
        }
    }
}

#[test]
fn rigid_copy_regenerates_oblique_plane_sphere_pcurve_and_proof() {
    let cases: [(&[f64], &[f64]); 2] = [(&[], &[]), (&[0.25, -0.125], &[0.5, -0.125])];
    for plane_first in [false, true] {
        for (plane_offsets, sphere_offsets) in cases {
            let mut store = Store::new();
            let (source, source_curve, source_surfaces, source_pcurves) =
                verified_oblique_plane_sphere_wire(
                    &mut store,
                    plane_first,
                    plane_offsets,
                    sphere_offsets,
                );
            let source_intersection = store
                .get(source_curve)
                .unwrap()
                .as_intersection()
                .copied()
                .unwrap();
            let source_certificate = source_intersection
                .certificate()
                .as_plane_sphere_circle()
                .unwrap();

            let (copied, journal) = copy_checked(&mut store, source, placement());
            let copied_curve = store
                .get(store.edges_of_body(copied).unwrap()[0])
                .unwrap()
                .curve
                .unwrap();
            let copied_intersection = store
                .get(copied_curve)
                .unwrap()
                .as_intersection()
                .copied()
                .unwrap();
            let certificate = copied_intersection
                .certificate()
                .as_plane_sphere_circle()
                .unwrap();
            assert_ne!(copied_curve, source_curve);
            assert!(
                copied_intersection
                    .source_surfaces()
                    .into_iter()
                    .zip(source_surfaces)
                    .all(|(copied, source)| copied != source)
            );
            assert!(
                copied_intersection
                    .pcurves()
                    .into_iter()
                    .zip(source_pcurves)
                    .all(|(copied, source)| copied != source)
            );
            assert_eq!(
                matches!(certificate.traces()[0], PlaneSphereCircleTrace::Plane(_)),
                plane_first
            );
            let sphere_index = usize::from(plane_first);
            let sphere_trace = match certificate.traces()[sphere_index] {
                PlaneSphereCircleTrace::SphereOblique(trace) => trace,
                _ => panic!("copy changed the oblique sphere trace family or order"),
            };
            let copied_sphere = match store
                .eval_context(EvalLimits::default(), Tolerances::default())
                .surface_exact_field(copied_intersection.source_surfaces()[sphere_index])
                .unwrap()
                .unwrap()
            {
                ExactSurfaceField::Sphere(sphere) => sphere,
                ExactSurfaceField::Plane(_) => panic!("copy changed the sphere field family"),
            };
            assert_eq!(sphere_trace.surface(), copied_sphere);
            assert_eq!(sphere_trace.pcurve().sphere(), copied_sphere);
            assert_eq!(sphere_trace.pcurve().carrier(), certificate.carrier());
            assert_eq!(
                store
                    .get(copied_intersection.pcurves()[sphere_index])
                    .unwrap()
                    .as_spherical_circle()
                    .copied(),
                Some(sphere_trace.pcurve())
            );
            let source_sphere_trace = match source_certificate.traces()[sphere_index] {
                PlaneSphereCircleTrace::SphereOblique(trace) => trace,
                _ => unreachable!(),
            };
            assert_ne!(sphere_trace.pcurve(), source_sphere_trace.pcurve());
            assert_eq!(
                sphere_trace.pcurve().chart_window(),
                source_sphere_trace.pcurve().chart_window()
            );
            for parameter in [
                certificate.carrier_range().lo,
                certificate.carrier_range().lerp(0.53),
                certificate.carrier_range().hi,
            ] {
                let uv = sphere_trace.pcurve().eval(parameter);
                assert!(
                    certificate
                        .carrier()
                        .eval(parameter)
                        .dist(copied_sphere.eval([uv.x, uv.y]))
                        <= certificate.tolerance()
                );
            }
            for (copied_root, source_root) in copied_intersection
                .source_surfaces()
                .into_iter()
                .zip(source_surfaces)
            {
                assert_copied_surface_chain(&store, &journal, copied_root, source_root);
            }
            store.geometry().validate().unwrap();
        }
    }
}

#[test]
fn overdeep_offset_backed_plane_sphere_copy_fails_typed_atomic_and_deterministic() {
    for plane_first in [false, true] {
        let mut attempted = Store::new();
        let overdeep = vec![0.0; EvalLimits::default().max_dependency_depth];
        let (unsupported, _, _, _) =
            verified_aligned_plane_sphere_wire(&mut attempted, plane_first, &overdeep, &[]);
        let before = (
            attempted.count::<Body>(),
            attempted.count::<Region>(),
            attempted.count::<Shell>(),
            attempted.count::<Face>(),
            attempted.count::<Loop>(),
            attempted.count::<Fin>(),
            attempted.count::<Edge>(),
            attempted.count::<Vertex>(),
            attempted.count::<CurveGeom>(),
            attempted.count::<SurfaceGeom>(),
            attempted.count::<Curve2dGeom>(),
            attempted.count::<Point3>(),
        );
        {
            let mut transaction = attempted.transaction().unwrap();
            assert_eq!(
                transaction.copy_body_rigid(unsupported, placement()),
                Err(Error::InvalidGeometry {
                    reason: "verified intersection source exceeds the supported safe offset-field boundary",
                })
            );
        }
        assert_eq!(
            before,
            (
                attempted.count::<Body>(),
                attempted.count::<Region>(),
                attempted.count::<Shell>(),
                attempted.count::<Face>(),
                attempted.count::<Loop>(),
                attempted.count::<Fin>(),
                attempted.count::<Edge>(),
                attempted.count::<Vertex>(),
                attempted.count::<CurveGeom>(),
                attempted.count::<SurfaceGeom>(),
                attempted.count::<Curve2dGeom>(),
                attempted.count::<Point3>(),
            )
        );

        let (valid_source, _, _, _) =
            verified_aligned_plane_sphere_wire(&mut attempted, !plane_first, &[], &[]);
        let (copied_after, journal_after) = copy_checked(&mut attempted, valid_source, placement());

        let mut control = Store::new();
        let _ = verified_aligned_plane_sphere_wire(&mut control, plane_first, &overdeep, &[]);
        let (control_source, _, _, _) =
            verified_aligned_plane_sphere_wire(&mut control, !plane_first, &[], &[]);
        let (copied_control, journal_control) =
            copy_checked(&mut control, control_source, placement());
        assert_eq!(copied_after, copied_control);
        assert_eq!(journal_after, journal_control);
        assert_eq!(
            attempted
                .get(
                    attempted
                        .get(attempted.edges_of_body(copied_after).unwrap()[0])
                        .unwrap()
                        .curve
                        .unwrap()
                )
                .unwrap(),
            control
                .get(
                    control
                        .get(control.edges_of_body(copied_control).unwrap()[0])
                        .unwrap()
                        .curve
                        .unwrap()
                )
                .unwrap()
        );
    }
}

#[test]
fn collapsed_sphere_field_proof_insertion_is_atomic_and_reuses_future_identity() {
    let mut attempted = Store::new();
    let (invalid_surfaces, valid_surfaces, pcurves, certificate) =
        collapsed_sphere_certificate_fixture(&mut attempted);
    let before = attempted.count::<CurveGeom>();
    assert!(
        attempted
            .insert_verified_plane_sphere_intersection_curve(
                invalid_surfaces,
                pcurves,
                certificate,
            )
            .is_err()
    );
    assert_eq!(attempted.count::<CurveGeom>(), before);
    let after = attempted
        .insert_verified_plane_sphere_intersection_curve(valid_surfaces, pcurves, certificate)
        .unwrap();

    let mut control = Store::new();
    let (_, valid_surfaces, pcurves, certificate) =
        collapsed_sphere_certificate_fixture(&mut control);
    let expected = control
        .insert_verified_plane_sphere_intersection_curve(valid_surfaces, pcurves, certificate)
        .unwrap();
    assert_eq!(after, expected);
}

#[test]
fn rejected_out_of_box_copy_is_atomic_and_reuses_future_identity() {
    let mut attempted = Store::new();
    let source = make::block(&mut attempted, &Frame::world(), [1.0; 3]).unwrap();
    let before = (
        attempted.count::<Body>(),
        attempted.count::<Region>(),
        attempted.count::<Shell>(),
        attempted.count::<Face>(),
        attempted.count::<Loop>(),
        attempted.count::<Fin>(),
        attempted.count::<Edge>(),
        attempted.count::<Vertex>(),
        attempted.count::<CurveGeom>(),
        attempted.count::<SurfaceGeom>(),
        attempted.count::<Curve2dGeom>(),
        attempted.count::<Point3>(),
    );
    {
        let mut transaction = attempted.transaction().unwrap();
        assert!(
            transaction
                .copy_body_rigid(
                    source,
                    Frame::world().with_origin(Point3::new(600.0, 0.0, 0.0)),
                )
                .is_err()
        );
    }
    assert_eq!(
        before,
        (
            attempted.count::<Body>(),
            attempted.count::<Region>(),
            attempted.count::<Shell>(),
            attempted.count::<Face>(),
            attempted.count::<Loop>(),
            attempted.count::<Fin>(),
            attempted.count::<Edge>(),
            attempted.count::<Vertex>(),
            attempted.count::<CurveGeom>(),
            attempted.count::<SurfaceGeom>(),
            attempted.count::<Curve2dGeom>(),
            attempted.count::<Point3>(),
        )
    );

    let (copied_after, journal_after) = copy_checked(&mut attempted, source, placement());
    let mut control = Store::new();
    let control_source = make::block(&mut control, &Frame::world(), [1.0; 3]).unwrap();
    let (copied_control, journal_control) = copy_checked(&mut control, control_source, placement());
    assert_eq!(copied_after, copied_control);
    assert_eq!(journal_after, journal_control);
}
