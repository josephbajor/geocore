#![allow(
    deprecated,
    reason = "lower-layer copy integration retains the compatibility tessellation wrapper"
)]

//! Checked deterministic complete-body rigid-copy contracts.

use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Point3, Vec2, Vec3};
use kgraph::{
    AffineParamMap1d, VerifiedIntersectionCertificate, certify_paired_plane_line_residuals,
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

fn verified_plane_line_wire(
    store: &mut Store,
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
    let surfaces = planes.map(|plane| store.insert_surface(SurfaceGeom::Plane(plane)).unwrap());
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
    let mut store = Store::new();
    let (source, source_curve, source_surfaces, source_pcurves) =
        verified_plane_line_wire(&mut store);

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
    let VerifiedIntersectionCertificate::PlaneLine(certificate) = copied_intersection.certificate()
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
        assert_eq!(
            store.get(surface).unwrap().as_plane().copied(),
            Some(certificate.surfaces()[index])
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
