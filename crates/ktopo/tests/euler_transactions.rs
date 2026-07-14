//! Public Euler mutation is transaction-owned, pcurve-aware, and journaled.

use kcore::tolerance::{LINEAR_RESOLUTION, SIZE_BOX_HALF};
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
use ktopo::tolerance::{EntityTolerance, ToleranceOrigin};
use ktopo::transaction::{
    FaceTolerancePropagation, Journal, LineageEvent, ToleranceGrowth, ToleranceGrowthTarget,
};

fn line_inputs(
    store: &mut Store,
    surface: ktopo::entity::SurfaceId,
    start: Point3,
    end: Point3,
) -> (ktopo::entity::CurveId, (f64, f64), FinPcurvePair) {
    let direction = end - start;
    let length = direction.norm();
    let curve = store
        .insert_curve(CurveGeom::Line(Line::new(start, direction).unwrap()))
        .unwrap();
    let SurfaceGeom::Plane(plane) = *store.get(surface).unwrap() else {
        panic!("test surface must be planar");
    };
    let start_local = plane.frame().to_local(start);
    let end_local = plane.frame().to_local(end);
    let uv_start = Point2::new(start_local.x, start_local.y);
    let uv_end = Point2::new(end_local.x, end_local.y);
    let make_use = |store: &mut Store| {
        let pcurve = store
            .insert_pcurve(Curve2dGeom::Line(
                Line2d::new(uv_start, uv_end - uv_start).unwrap(),
            ))
            .unwrap();
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
    let surface = store
        .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
        .unwrap();
    let start_position = Point3::new(0.0, 0.0, 0.0);
    let end_position = Point3::new(1.0, 0.0, 0.0);
    let start = store.insert_point(start_position).unwrap();
    let end = store.insert_point(end_position).unwrap();
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
    let journal = transaction.commit_checked(&[]).unwrap();

    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<Face>(), 0);
    assert_eq!(store.count::<Loop>(), 0);
    assert_eq!(store.count::<Fin>(), 0);
    assert_eq!(store.count::<Edge>(), 0);
    assert_eq!(store.count::<Vertex>(), 0);
    assert_eq!(store.count::<Point3>(), 2);
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
fn position_mvfs_preflights_reuses_ids_and_cleans_its_hidden_point() {
    let mut store = Store::new();
    let surface = store
        .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
        .unwrap();
    let position = Point3::new(1.0, 2.0, 0.0);
    let point_count = store.count::<Point3>();

    let (rolled_back, rolled_back_point) = {
        let mut transaction = store.transaction().unwrap();
        for invalid in [
            Point3::new(f64::NAN, 0.0, 0.0),
            Point3::new(SIZE_BOX_HALF + 1.0, 0.0, 0.0),
        ] {
            assert!(
                transaction
                    .make_minimal_body_at_position(surface, Sense::Forward, invalid)
                    .is_err()
            );
            assert_eq!(transaction.store().count::<Point3>(), point_count);
        }
        let made = transaction
            .make_minimal_body_at_position(surface, Sense::Forward, position)
            .unwrap();
        let point = transaction.store().get(made.vertex).unwrap().point();
        transaction.rollback().unwrap();
        (made, point)
    };
    assert_eq!(store.count::<Point3>(), point_count);

    let mut transaction = store.transaction().unwrap();
    let made = transaction
        .make_minimal_body_at_position(surface, Sense::Forward, position)
        .unwrap();
    let point = transaction.store().get(made.vertex).unwrap().point();
    assert_eq!(made.body, rolled_back.body);
    assert_eq!(made.void_region, rolled_back.void_region);
    assert_eq!(made.solid_region, rolled_back.solid_region);
    assert_eq!(made.shell, rolled_back.shell);
    assert_eq!(made.face, rolled_back.face);
    assert_eq!(made.ring, rolled_back.ring);
    assert_eq!(made.vertex, rolled_back.vertex);
    assert_eq!(point, rolled_back_point);
    transaction
        .kill_position_owned_minimal_body(made.body)
        .unwrap();
    assert_eq!(transaction.store().count::<Point3>(), point_count);
    let journal = transaction.commit_checked(&[]).unwrap();
    assert!(matches!(
        journal.lineage(),
        [
            LineageEvent::DerivedFrom {
                derived: EntityRef::Vertex(vertex),
                source: EntityRef::Point(source),
            },
            LineageEvent::Deleted {
                entity: EntityRef::Body(body),
            },
            LineageEvent::Deleted {
                entity: EntityRef::Point(deleted),
            },
        ] if *vertex == made.vertex
            && *source == point
            && *body == made.body
            && *deleted == point
    ));
    assert_eq!(store.count::<Point3>(), point_count);
}

#[test]
fn position_owned_kvfs_retains_a_point_still_shared_by_a_live_vertex() {
    let mut store = Store::new();
    let surface = store
        .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
        .unwrap();
    let position = Point3::new(1.0, 2.0, 0.0);
    let mut transaction = store.transaction().unwrap();
    let made = transaction
        .make_minimal_body_at_position(surface, Sense::Forward, position)
        .unwrap();
    let point = transaction.store().get(made.vertex).unwrap().point();
    let sharing_vertex = transaction.assembly().add(Vertex {
        point,
        tolerance: None,
    });
    transaction
        .kill_position_owned_minimal_body(made.body)
        .unwrap();
    assert_eq!(
        transaction.store().get(sharing_vertex).unwrap().point(),
        point
    );
    assert_eq!(*transaction.store().get(point).unwrap(), position);
    transaction.rollback().unwrap();
}

#[test]
fn position_mev_preflights_without_consuming_point_identity() {
    let mut store = Store::new();
    let surface = store
        .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
        .unwrap();
    let start_position = Point3::new(0.0, 0.0, 0.0);
    let end_position = Point3::new(1.0, 0.0, 0.0);
    let start = store.insert_point(start_position).unwrap();
    let (curve, bounds, pcurves) = line_inputs(&mut store, surface, start_position, end_position);
    let off_curve = store
        .insert_pcurve(Curve2dGeom::Line(
            Line2d::new(Point2::new(10.0, 10.0), Point2::new(1.0, 0.0)).unwrap(),
        ))
        .unwrap();
    let off_use =
        FinPcurve::new(off_curve, ParamRange::new(0.0, 1.0), ParamMap1d::identity()).unwrap();
    let off_pcurves = FinPcurvePair::new(off_use, off_use);

    let mut transaction = store.transaction().unwrap();
    let minimal = transaction
        .make_minimal_body(surface, Sense::Forward, start)
        .unwrap();
    let point_count = transaction.store().count::<Point3>();
    for invalid in [
        Point3::new(f64::NAN, 0.0, 0.0),
        Point3::new(SIZE_BOX_HALF + 1.0, 0.0, 0.0),
    ] {
        assert!(
            transaction
                .make_edge_vertex_at_position(minimal.ring, 0, curve, bounds, invalid, pcurves,)
                .is_err()
        );
        assert_eq!(transaction.store().count::<Point3>(), point_count);
    }
    assert!(
        transaction
            .make_edge_vertex_at_position(minimal.ring, 1, curve, bounds, end_position, pcurves,)
            .is_err()
    );
    assert_eq!(transaction.store().count::<Point3>(), point_count);
    assert!(
        transaction
            .make_edge_vertex_at_position(
                minimal.ring,
                0,
                curve,
                bounds,
                end_position,
                off_pcurves,
            )
            .is_err()
    );
    assert_eq!(transaction.store().count::<Point3>(), point_count);

    let sprout = transaction
        .make_edge_vertex_at_position(minimal.ring, 0, curve, bounds, end_position, pcurves)
        .unwrap();
    assert_eq!(transaction.store().count::<Point3>(), point_count + 1);
    let inserted_point = transaction.store().get(sprout.vertex).unwrap().point();
    transaction
        .kill_position_owned_edge_vertex(sprout.edge)
        .unwrap();
    assert_eq!(transaction.store().count::<Point3>(), point_count);
    transaction.kill_minimal_body(minimal.body).unwrap();
    let journal = transaction.commit_checked(&[]).unwrap();
    assert!(matches!(
        journal.lineage()[2],
        LineageEvent::DerivedFrom {
            derived: EntityRef::Vertex(vertex),
            source: EntityRef::Point(point),
        } if vertex == sprout.vertex && point == inserted_point
    ));
    assert!(matches!(
        journal.lineage()[5],
        LineageEvent::Deleted {
            entity: EntityRef::Point(point),
        } if point == inserted_point
    ));
    assert_eq!(store.count::<Point3>(), point_count);
}

#[test]
fn position_owned_kev_retains_a_point_still_shared_by_a_live_vertex() {
    let mut store = Store::new();
    let surface = store
        .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
        .unwrap();
    let start_position = Point3::new(0.0, 0.0, 0.0);
    let end_position = Point3::new(1.0, 0.0, 0.0);
    let start = store.insert_point(start_position).unwrap();
    let (curve, bounds, pcurves) = line_inputs(&mut store, surface, start_position, end_position);

    let mut transaction = store.transaction().unwrap();
    let minimal = transaction
        .make_minimal_body(surface, Sense::Forward, start)
        .unwrap();
    let sprout = transaction
        .make_edge_vertex_at_position(minimal.ring, 0, curve, bounds, end_position, pcurves)
        .unwrap();
    let point = transaction.store().get(sprout.vertex).unwrap().point();
    let sharing_vertex = transaction.assembly().add(Vertex {
        point,
        tolerance: None,
    });
    transaction
        .kill_position_owned_edge_vertex(sprout.edge)
        .unwrap();
    assert_eq!(
        transaction.store().get(sharing_vertex).unwrap().point(),
        point
    );
    assert_eq!(*transaction.store().get(point).unwrap(), end_position);
    transaction.rollback().unwrap();
}

#[test]
fn face_split_merge_journals_inherited_and_max_combined_tolerance() {
    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let source = store.faces_of_body(body).unwrap()[0];
    let source_data = store.get(source).unwrap().clone();
    let outer = source_data.loops[0];
    let fins = store.get(outer).unwrap().fins.clone();
    let start = store
        .vertex_position(store.fin_tail(fins[0]).unwrap().unwrap())
        .unwrap();
    let end = store
        .vertex_position(store.fin_tail(fins[2]).unwrap().unwrap())
        .unwrap();
    let (diagonal, bounds, pcurves) = line_inputs(&mut store, source_data.surface, start, end);
    let imported = EntityTolerance::imported_xt(2.0 * LINEAR_RESOLUTION).unwrap();
    let mut setup = store.transaction().unwrap();
    setup.assembly().get_mut(source).unwrap().tolerance = Some(imported);
    setup.commit_checked_body(body).unwrap();

    let mut split = store.transaction().unwrap();
    let made = split
        .split_face(
            outer,
            0,
            2,
            diagonal,
            bounds,
            source_data.surface,
            source_data.sense,
            pcurves,
        )
        .unwrap();
    assert_eq!(
        split.store().get(made.face).unwrap().tolerance,
        Some(imported)
    );
    let split_journal = split.commit_checked_body(body).unwrap();
    assert_eq!(
        split_journal.face_tolerance_propagations(),
        &[FaceTolerancePropagation::Inherited {
            source,
            result: made.face,
            tolerance: Some(imported),
        }]
    );
    assert!(split_journal.tolerance_budgets().is_empty());
    assert!(split_journal.tolerance_events().is_empty());

    // Simulate a later exact child before a modeled repair introduces its own
    // operation-origin tolerance. The merge must retain that selected origin
    // rather than manufacture provenance from the imported survivor.
    let mut reset_child = store.transaction().unwrap();
    reset_child.assembly().get_mut(made.face).unwrap().tolerance = None;
    reset_child.commit_checked_body(body).unwrap();

    let requested = 5.0 * LINEAR_RESOLUTION;
    let mut merge = store.transaction().unwrap();
    merge
        .grow_tolerances(
            "merge-face-maximum",
            requested - LINEAR_RESOLUTION,
            &[ToleranceGrowth::new(
                ToleranceGrowthTarget::Face(made.face),
                requested,
            )],
        )
        .unwrap();
    let absorbed = merge.store().get(made.face).unwrap().tolerance.unwrap();
    merge.merge_faces(made.edge).unwrap();
    assert_eq!(merge.store().get(source).unwrap().tolerance, Some(absorbed));
    let merge_journal = merge.commit_checked_body(body).unwrap();
    assert_eq!(
        merge_journal.face_tolerance_propagations(),
        &[FaceTolerancePropagation::CombinedMax {
            sources: [source, made.face],
            source_tolerances: [Some(imported), Some(absorbed)],
            result: source,
            selected_source: Some(made.face),
            tolerance: Some(absorbed),
        }]
    );
    assert_eq!(merge_journal.tolerance_budgets().len(), 1);
    assert_eq!(merge_journal.tolerance_events().len(), 1);
    assert_eq!(
        absorbed.origin(),
        ToleranceOrigin::Operation("merge-face-maximum")
    );
    assert_eq!(absorbed.origin_value(), requested);
    assert_eq!(absorbed.accumulated_growth(), 0.0);
    assert_eq!(absorbed.last_operation(), Some("merge-face-maximum"));
}

#[test]
fn checked_split_denial_discards_tolerance_propagation_and_reuses_future_ids() {
    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let source = store.faces_of_body(body).unwrap()[0];
    let source_data = store.get(source).unwrap().clone();
    let outer = source_data.loops[0];
    let fins = store.get(outer).unwrap().fins.clone();
    let start = store
        .vertex_position(store.fin_tail(fins[0]).unwrap().unwrap())
        .unwrap();
    let end = store
        .vertex_position(store.fin_tail(fins[2]).unwrap().unwrap())
        .unwrap();
    let (diagonal, bounds, pcurves) = line_inputs(&mut store, source_data.surface, start, end);
    let imported = EntityTolerance::imported_xt(2.0 * LINEAR_RESOLUTION).unwrap();
    let mut setup = store.transaction().unwrap();
    setup.assembly().get_mut(source).unwrap().tolerance = Some(imported);
    setup.commit_checked_body(body).unwrap();
    let original_face_count = store.count::<Face>();

    let mut denied = store.transaction().unwrap();
    let attempted = denied
        .split_face(
            outer,
            0,
            2,
            diagonal,
            bounds,
            source_data.surface,
            source_data.sense,
            pcurves,
        )
        .unwrap();
    denied
        .assembly()
        .get_mut(attempted.face)
        .unwrap()
        .loops
        .clear();
    assert!(denied.commit_checked_body(body).is_err());
    assert_eq!(store.count::<Face>(), original_face_count);
    assert_eq!(store.get(source).unwrap().tolerance, Some(imported));

    let mut repeated = store.transaction().unwrap();
    let repeated_ids = repeated
        .split_face(
            outer,
            0,
            2,
            diagonal,
            bounds,
            source_data.surface,
            source_data.sense,
            pcurves,
        )
        .unwrap();
    assert_eq!(repeated_ids.edge, attempted.edge);
    assert_eq!(repeated_ids.face, attempted.face);
    assert_eq!(repeated_ids.ring, attempted.ring);
    assert_eq!(repeated_ids.fin_old, attempted.fin_old);
    assert_eq!(repeated_ids.fin_new, attempted.fin_new);
    assert_eq!(
        repeated.store().get(repeated_ids.face).unwrap().tolerance,
        Some(imported)
    );
    repeated.rollback().unwrap();
    assert_eq!(store.count::<Face>(), original_face_count);
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
