//! Failure-atomic Store transactions and deterministic lineage journals.

use kcore::error::{Error, Result};
use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Point3, Vec3};
use ktopo::check::check_body;
use ktopo::entity::{
    Body, BodyId, Edge, EntityRef, Face, Fin, FinPcurve, Loop, ParamMap1d, Region, Sense, Shell,
    Vertex,
};
use ktopo::euler::{FinPcurvePair, mev, mvfs};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::make::block;
use ktopo::store::Store;
use ktopo::transaction::{LineageEvent, MutationKind};

fn seed_geometry(
    store: &mut Store,
) -> (
    ktopo::entity::SurfaceId,
    ktopo::entity::PointId,
    ktopo::entity::CurveId,
    ktopo::entity::PointId,
) {
    let surface = store.add(SurfaceGeom::Plane(Plane::new(Frame::world())));
    let start = store.add(Point3::new(0.0, 0.0, 0.0));
    let end = store.add(Point3::new(1.0, 0.0, 0.0));
    let curve = store.add(CurveGeom::Line(
        Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
    ));
    (surface, start, curve, end)
}

fn first_face_diagonal(
    store: &mut Store,
    body: BodyId,
) -> (
    ktopo::entity::LoopId,
    ktopo::entity::CurveId,
    f64,
    ktopo::entity::SurfaceId,
    Sense,
    FinPcurvePair,
) {
    let face_id = store.faces_of_body(body).unwrap()[0];
    let face = store.get(face_id).unwrap();
    let (lp, surface, sense) = (face.loops[0], face.surface, face.sense);
    let fins = store.get(lp).unwrap().fins.clone();
    let start_vertex = store.fin_tail(fins[0]).unwrap().unwrap();
    let end_vertex = store.fin_tail(fins[2]).unwrap().unwrap();
    let start = store.vertex_position(start_vertex).unwrap();
    let end = store.vertex_position(end_vertex).unwrap();
    let delta = end - start;
    let length = delta.norm();
    let curve = store.add(CurveGeom::Line(Line::new(start, delta).unwrap()));
    let plane = match store.get(surface).unwrap() {
        SurfaceGeom::Plane(plane) => *plane,
        _ => panic!("block face must be planar"),
    };
    let local_start = plane.frame().to_local(start);
    let local_end = plane.frame().to_local(end);
    let uv_start = Point2::new(local_start.x, local_start.y);
    let uv_end = Point2::new(local_end.x, local_end.y);
    let range = ParamRange::new(0.0, length);
    let make_use = |store: &mut Store| {
        let pcurve = store.add(Curve2dGeom::Line(
            Line2d::new(uv_start, uv_end - uv_start).unwrap(),
        ));
        FinPcurve::new(pcurve, range, ParamMap1d::identity()).unwrap()
    };
    let forward = make_use(store);
    let reversed = make_use(store);
    (
        lp,
        curve,
        length,
        surface,
        sense,
        FinPcurvePair::new(forward, reversed),
    )
}

fn failing_multi_step_edit(store: &mut Store) -> Result<()> {
    let (surface, start, curve, end) = seed_geometry(store);
    let mut transaction = store.transaction()?;
    let made = mvfs(transaction.store_mut(), surface, Sense::Forward, start)?;
    // The first step created a complete minimal topology. The second step
    // fails preflight; `?` drops the transaction and must undo the first.
    mev(
        transaction.store_mut(),
        made.ring,
        0,
        curve,
        (1.0, 0.0),
        end,
    )?;
    let _ = transaction.commit();
    Ok(())
}

#[test]
fn failed_multi_step_euler_edit_restores_identity_and_future_allocations() {
    let mut store = Store::new();
    assert!(matches!(
        failing_multi_step_edit(&mut store),
        Err(Error::InvalidGeometry { .. })
    ));
    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<Region>(), 0);
    assert_eq!(store.count::<Shell>(), 0);
    assert_eq!(store.count::<Face>(), 0);
    assert_eq!(store.count::<Loop>(), 0);
    assert_eq!(store.count::<Fin>(), 0);
    assert_eq!(store.count::<Edge>(), 0);
    assert_eq!(store.count::<Vertex>(), 0);

    // Geometry was intentionally authored before the transaction and must
    // remain. A control clone proves every topology arena's next allocation
    // identity, not only its live count.
    let mut control = store.clone();
    let surface = store.iter::<SurfaceGeom>().next().unwrap().0;
    let start = store.iter::<Point3>().next().unwrap().0;
    let made = mvfs(&mut store, surface, Sense::Forward, start).unwrap();
    let control_made = mvfs(&mut control, surface, Sense::Forward, start).unwrap();
    assert_eq!(made.body, control_made.body);
    assert_eq!(made.void_region, control_made.void_region);
    assert_eq!(made.solid_region, control_made.solid_region);
    assert_eq!(made.shell, control_made.shell);
    assert_eq!(made.face, control_made.face);
    assert_eq!(made.ring, control_made.ring);
    assert_eq!(made.vertex, control_made.vertex);
}

#[test]
fn commit_emits_raw_mutations_and_semantic_lineage_deterministically() {
    let mut store = Store::new();
    let (surface, start, _, _) = seed_geometry(&mut store);
    let mut transaction = store.transaction().unwrap();
    let made = mvfs(transaction.store_mut(), surface, Sense::Forward, start).unwrap();
    let lineage = LineageEvent::DerivedFrom {
        derived: EntityRef::Vertex(made.vertex),
        source: EntityRef::Point(start),
    };
    transaction.record_lineage(lineage.clone());
    let journal = transaction.commit().unwrap();

    let entities: Vec<_> = journal
        .mutations()
        .iter()
        .map(|mutation| mutation.entity)
        .collect();
    assert_eq!(
        entities,
        vec![
            EntityRef::Body(made.body),
            EntityRef::Region(made.void_region),
            EntityRef::Region(made.solid_region),
            EntityRef::Shell(made.shell),
            EntityRef::Face(made.face),
            EntityRef::Loop(made.ring),
            EntityRef::Vertex(made.vertex),
        ]
    );
    assert!(
        journal
            .mutations()
            .iter()
            .all(|mutation| mutation.kind == MutationKind::Created)
    );
    assert_eq!(journal.lineage(), &[lineage]);
}

#[test]
fn transaction_is_rollback_on_drop_and_nested_scope_is_rejected() {
    let mut store = Store::new();
    let (surface, start, _, _) = seed_geometry(&mut store);
    {
        let mut transaction = store.transaction().unwrap();
        mvfs(transaction.store_mut(), surface, Sense::Forward, start).unwrap();
        assert_eq!(
            transaction.store_mut().transaction().err(),
            Some(Error::TransactionActive)
        );
    }
    assert_eq!(store.count::<Body>(), 0);
}

#[test]
fn checked_face_split_and_merge_emit_semantic_lineage() {
    let mut store = Store::new();
    let body = block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let (lp, curve, length, surface, sense, pcurves) = first_face_diagonal(&mut store, body);
    let source_face = store.get(lp).unwrap().face;

    let mut split = store.transaction().unwrap();
    let made = split
        .split_face(lp, 0, 2, curve, (0.0, length), surface, sense, pcurves)
        .unwrap();
    let split_journal = split.commit().unwrap();
    assert_eq!(
        split_journal.lineage(),
        &[LineageEvent::Split {
            source: EntityRef::Face(source_face),
            pieces: vec![EntityRef::Face(source_face), EntityRef::Face(made.face)],
        }]
    );
    let faults = check_body(&store, body).unwrap();
    assert!(faults.is_empty(), "split checker faults: {faults:?}");

    let mut merge = store.transaction().unwrap();
    merge.merge_faces(made.edge).unwrap();
    let merge_journal = merge.commit().unwrap();
    assert_eq!(
        merge_journal.lineage(),
        &[LineageEvent::Merge {
            sources: vec![EntityRef::Face(source_face), EntityRef::Face(made.face)],
            result: EntityRef::Face(source_face),
        }]
    );
    let faults = check_body(&store, body).unwrap();
    assert!(faults.is_empty(), "merge checker faults: {faults:?}");
}
