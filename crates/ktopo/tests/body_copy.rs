#![allow(
    deprecated,
    reason = "lower-layer copy integration retains the compatibility tessellation wrapper"
)]

//! Checked deterministic complete-body rigid-copy contracts.

use kcore::error::Error;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Line};
use kgeom::curve2d::{Circle2d, Curve2d, Line2d};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere, Surface};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};
use kgraph::{
    AffineParamMap1d, EvalLimits, ExactSurfaceField, OffsetSurfaceDescriptor, PairedTrace,
    PlaneCircleTrace, PlaneSphereCircleTrace, SphereLatitudeTrace, SurfaceDerivativeOrder,
    VerifiedIntersectionCertificate, certify_paired_plane_line_residuals,
    certify_paired_plane_sphere_circle_residuals,
    certify_paired_plane_sphere_oblique_circle_residuals,
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

fn map_point(placement: Frame, point: Point3) -> Point3 {
    placement.point_at(point.x, point.y, point.z)
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
            | (SurfaceGeom::Sphere(_), SurfaceGeom::Sphere(_)) => {}
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
