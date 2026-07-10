//! Public Euler mutation is transaction-owned, pcurve-aware, and journaled.

use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Point3};
use ktopo::check::check_body;
use ktopo::entity::{Body, Edge, EntityRef, Face, Fin, FinPcurve, Loop, ParamMap1d, Sense, Vertex};
use ktopo::euler::FinPcurvePair;
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::make;
use ktopo::store::Store;
use ktopo::transaction::{Journal, LineageEvent};

fn line_inputs(
    store: &mut Store,
    surface: ktopo::entity::SurfaceId,
    start: Point3,
    end: Point3,
) -> (ktopo::entity::CurveId, (f64, f64), FinPcurvePair) {
    let direction = end - start;
    let length = direction.norm();
    let curve = store.add(CurveGeom::Line(Line::new(start, direction).unwrap()));
    let SurfaceGeom::Plane(plane) = *store.get(surface).unwrap() else {
        panic!("test surface must be planar");
    };
    let start_local = plane.frame().to_local(start);
    let end_local = plane.frame().to_local(end);
    let uv_start = Point2::new(start_local.x, start_local.y);
    let uv_end = Point2::new(end_local.x, end_local.y);
    let make_use = |store: &mut Store| {
        let pcurve = store.add(Curve2dGeom::Line(
            Line2d::new(uv_start, uv_end - uv_start).unwrap(),
        ));
        FinPcurve::new(pcurve, ParamRange::new(0.0, length), ParamMap1d::identity()).unwrap()
    };
    (
        curve,
        (0.0, length),
        FinPcurvePair::new(make_use(store), make_use(store)),
    )
}

fn minimal_inverse_journal() -> Journal {
    let mut store = Store::new();
    let surface = store.add(SurfaceGeom::Plane(Plane::new(Frame::world())));
    let start_position = Point3::new(0.0, 0.0, 0.0);
    let end_position = Point3::new(1.0, 0.0, 0.0);
    let start = store.add(start_position);
    let end = store.add(end_position);
    let (curve, bounds, pcurves) = line_inputs(&mut store, surface, start_position, end_position);

    let mut transaction = store.transaction().unwrap();
    let minimal = transaction
        .make_minimal_body(surface, Sense::Forward, start)
        .unwrap();
    let sprout = transaction
        .make_edge_vertex(minimal.ring, 0, curve, bounds, end, pcurves)
        .unwrap();
    transaction.kill_edge_vertex(sprout.edge).unwrap();
    transaction.kill_minimal_body(minimal.body).unwrap();
    let journal = transaction.commit().unwrap();

    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<Face>(), 0);
    assert_eq!(store.count::<Loop>(), 0);
    assert_eq!(store.count::<Fin>(), 0);
    assert_eq!(store.count::<Edge>(), 0);
    assert_eq!(store.count::<Vertex>(), 0);
    journal
}

#[test]
fn minimal_mev_kev_kvfs_sequence_is_atomic_and_deterministic() {
    let first = minimal_inverse_journal();
    let second = minimal_inverse_journal();
    assert_eq!(first, second);
    assert_eq!(first.lineage().len(), 6);
    assert!(matches!(
        first.lineage(),
        [
            LineageEvent::DerivedFrom {
                derived: EntityRef::Vertex(_),
                source: EntityRef::Point(_),
            },
            LineageEvent::DerivedFrom {
                derived: EntityRef::Edge(_),
                source: EntityRef::Loop(_),
            },
            LineageEvent::DerivedFrom {
                derived: EntityRef::Vertex(_),
                source: EntityRef::Point(_),
            },
            LineageEvent::Deleted {
                entity: EntityRef::Edge(_),
            },
            LineageEvent::Deleted {
                entity: EntityRef::Vertex(_),
            },
            LineageEvent::Deleted {
                entity: EntityRef::Body(_),
            },
        ]
    ));
}

#[test]
fn face_ring_genus_operators_round_trip_through_checked_transactions() {
    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let keep = store.faces_of_body(body).unwrap()[0];
    let keep_data = store.get(keep).unwrap().clone();
    let outer = keep_data.loops[0];
    let fins = store.get(outer).unwrap().fins.clone();
    let start = store
        .vertex_position(store.fin_tail(fins[0]).unwrap().unwrap())
        .unwrap();
    let end = store
        .vertex_position(store.fin_tail(fins[2]).unwrap().unwrap())
        .unwrap();
    let (diagonal, diagonal_bounds, diagonal_pcurves) =
        line_inputs(&mut store, keep_data.surface, start, end);

    let mut split = store.transaction().unwrap();
    let made = split
        .split_face(
            outer,
            0,
            2,
            diagonal,
            diagonal_bounds,
            keep_data.surface,
            keep_data.sense,
            diagonal_pcurves,
        )
        .unwrap();
    split.commit_checked_body(body).unwrap();

    let kill = made.face;
    let ring = store.get(kill).unwrap().loops[0];
    let outer_fins = store.get(outer).unwrap().fins.clone();
    let ring_fins = store.get(ring).unwrap().fins.clone();
    let bridge_start = store
        .vertex_position(store.fin_tail(outer_fins[0]).unwrap().unwrap())
        .unwrap();
    let bridge_end = store
        .vertex_position(store.fin_tail(ring_fins[0]).unwrap().unwrap())
        .unwrap();
    let (bridge, bridge_bounds, bridge_pcurves) =
        line_inputs(&mut store, keep_data.surface, bridge_start, bridge_end);

    let mut genus = store.transaction().unwrap();
    let moved_ring = genus.merge_face_as_hole(keep, kill).unwrap();
    assert_eq!(moved_ring, ring);
    let joined = genus
        .make_edge_kill_ring(
            outer,
            0,
            moved_ring,
            0,
            bridge,
            bridge_bounds,
            bridge_pcurves,
        )
        .unwrap();
    let restored_ring = genus.kill_edge_make_ring(joined.edge).unwrap();
    let restored_face = genus
        .split_hole_as_face(restored_ring, keep_data.surface, keep_data.sense)
        .unwrap();
    let genus_journal = genus.commit_checked_body(body).unwrap();
    assert!(check_body(&store, body).unwrap().is_empty());
    assert!(matches!(
        genus_journal.lineage(),
        [
            LineageEvent::Merge { .. },
            LineageEvent::DerivedFrom {
                derived: EntityRef::Edge(_),
                source: EntityRef::Loop(_),
            },
            LineageEvent::Merge { .. },
            LineageEvent::Split {
                source: EntityRef::Loop(_),
                ..
            },
            LineageEvent::Split {
                source: EntityRef::Face(_),
                ..
            },
        ]
    ));
    assert_eq!(store.get(restored_face).unwrap().loops, vec![restored_ring]);

    let mut merge = store.transaction().unwrap();
    merge.merge_faces(made.edge).unwrap();
    merge.commit_checked_body(body).unwrap();
    assert!(check_body(&store, body).unwrap().is_empty());
    assert_eq!(store.faces_of_body(body).unwrap().len(), 6);
}
