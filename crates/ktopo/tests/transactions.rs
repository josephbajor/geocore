//! Failure-atomic Store transactions and deterministic lineage journals.

use kcore::error::{Error, Result};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Point3, Vec3};
use ktopo::check::{CheckOutcome, check_body};
use ktopo::entity::{
    Body, BodyId, BodyKind, Edge, EntityRef, Face, Fin, FinPcurve, Loop, ParamMap1d, Region,
    RegionKind, Sense, Shell, Vertex,
};
use ktopo::euler::FinPcurvePair;
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::make::{block, block_with_journal, torus_with_journal};
use ktopo::store::Store;
use ktopo::tolerance::EntityTolerance;
use ktopo::transaction::{
    FullCommitRequirement, LineageEvent, MutationKind, ToleranceGrowth, ToleranceGrowthTarget,
};

fn seed_geometry(
    store: &mut Store,
) -> (
    ktopo::entity::SurfaceId,
    ktopo::entity::PointId,
    ktopo::entity::CurveId,
    ktopo::entity::PointId,
) {
    let surface = store
        .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
        .unwrap();
    let start = store.insert_point(Point3::new(0.0, 0.0, 0.0)).unwrap();
    let end = store.insert_point(Point3::new(1.0, 0.0, 0.0)).unwrap();
    let curve = store
        .insert_curve(CurveGeom::Line(
            Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
        ))
        .unwrap();
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
    let curve = store
        .insert_curve(CurveGeom::Line(Line::new(start, delta).unwrap()))
        .unwrap();
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
        let pcurve = store
            .insert_pcurve(Curve2dGeom::Line(
                Line2d::new(uv_start, uv_end - uv_start).unwrap(),
            ))
            .unwrap();
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
    let make_pcurve = |store: &mut Store| {
        let curve = store
            .insert_pcurve(Curve2dGeom::Line(
                Line2d::new(Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)).unwrap(),
            ))
            .unwrap();
        FinPcurve::new(curve, ParamRange::new(0.0, 1.0), ParamMap1d::identity()).unwrap()
    };
    let pcurves = FinPcurvePair::new(make_pcurve(store), make_pcurve(store));
    let mut transaction = store.transaction()?;
    let made = transaction.make_minimal_body(surface, Sense::Forward, start)?;
    // The first step created a complete minimal topology. The second step
    // fails preflight; `?` drops the transaction and must undo the first.
    transaction.make_edge_vertex(made.ring, 0, curve, (1.0, 0.0), end, pcurves)?;
    let _ = transaction.commit_checked_body(made.body);
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
    let mut transaction = store.transaction().unwrap();
    let made = transaction
        .make_minimal_body(surface, Sense::Forward, start)
        .unwrap();
    let mut control_transaction = control.transaction().unwrap();
    let control_made = control_transaction
        .make_minimal_body(surface, Sense::Forward, start)
        .unwrap();
    assert_eq!(made.body, control_made.body);
    assert_eq!(made.void_region, control_made.void_region);
    assert_eq!(made.solid_region, control_made.solid_region);
    assert_eq!(made.shell, control_made.shell);
    assert_eq!(made.face, control_made.face);
    assert_eq!(made.ring, control_made.ring);
    assert_eq!(made.vertex, control_made.vertex);
    transaction.rollback().unwrap();
    control_transaction.rollback().unwrap();
}

#[test]
fn checked_assembly_commit_emits_raw_mutations_deterministically() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let (body, region, shell) = {
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
        assembly.get_mut(region).unwrap().shells.push(shell);
        assembly.get_mut(body).unwrap().regions.push(region);
        (body, region, shell)
    };
    let journal = transaction.commit_checked_body(body).unwrap();

    let entities: Vec<_> = journal
        .mutations()
        .iter()
        .map(|mutation| mutation.entity)
        .collect();
    assert_eq!(
        entities,
        vec![
            EntityRef::Body(body),
            EntityRef::Region(region),
            EntityRef::Shell(shell),
        ]
    );
    assert!(
        journal
            .mutations()
            .iter()
            .all(|mutation| mutation.kind == MutationKind::Created)
    );
    assert!(journal.lineage().is_empty());
}

#[test]
fn transaction_is_rollback_on_drop() {
    let mut store = Store::new();
    let (surface, start, _, _) = seed_geometry(&mut store);
    {
        let mut transaction = store.transaction().unwrap();
        transaction
            .make_minimal_body(surface, Sense::Forward, start)
            .unwrap();
    }
    assert_eq!(store.count::<Body>(), 0);
}

#[test]
fn checked_face_split_and_merge_emit_semantic_lineage() {
    let mut store = Store::new();
    let body = block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let (lp, curve, length, surface, sense, pcurves) = first_face_diagonal(&mut store, body);
    let source_face = store.get(lp).unwrap().face;
    let source_domain = store.get(source_face).unwrap().domain;
    let source_tolerance = EntityTolerance::operation(1.0e-8, "split-test").unwrap();
    let mut metadata = store.transaction().unwrap();
    metadata.assembly().get_mut(source_face).unwrap().tolerance = Some(source_tolerance);
    metadata.commit_checked_body(body).unwrap();

    let mut split = store.transaction().unwrap();
    let made = split
        .split_face(lp, 0, 2, curve, (0.0, length), surface, sense, pcurves)
        .unwrap();
    let split_journal = split.commit_checked_body(body).unwrap();
    assert_eq!(store.get(made.face).unwrap().domain, source_domain);
    assert_eq!(
        store.get(made.face).unwrap().tolerance,
        Some(source_tolerance)
    );
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
    let merge_journal = merge.commit_checked_body(body).unwrap();
    assert_eq!(store.get(source_face).unwrap().domain, source_domain);
    assert_eq!(
        store.get(source_face).unwrap().tolerance,
        Some(source_tolerance)
    );
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

#[test]
fn checked_commit_rejects_faulted_topology_and_restores_the_body() {
    let mut store = Store::new();
    let body = block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let original_regions = store.get(body).unwrap().regions.clone();

    let mut transaction = store.transaction().unwrap();
    transaction
        .assembly()
        .get_mut(body)
        .unwrap()
        .regions
        .clear();
    assert!(matches!(
        transaction.commit_checked_body(body),
        Err(Error::TopologyCheckFailed { fault_count }) if fault_count > 0
    ));
    assert_eq!(store.get(body).unwrap().regions, original_regions);
    assert!(check_body(&store, body).unwrap().is_empty());
}

#[test]
fn full_commit_distinguishes_valid_indeterminate_and_rejected_candidates() {
    let mut valid_store = Store::new();
    let valid_body = block(&mut valid_store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let valid_face = valid_store.faces_of_body(valid_body).unwrap()[0];
    let mut valid = valid_store.transaction().unwrap();
    valid
        .grow_tolerances(
            "full-valid",
            LINEAR_RESOLUTION,
            &[ToleranceGrowth::new(
                ToleranceGrowthTarget::Face(valid_face),
                2.0 * LINEAR_RESOLUTION,
            )],
        )
        .unwrap();
    let valid_decision = valid
        .commit_full(&[valid_body], FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(valid_decision.is_committed());
    assert_eq!(valid_decision.checks().len(), 1);
    assert_eq!(valid_decision.checks()[0].body(), valid_body);
    assert_eq!(
        valid_decision.checks()[0].report().outcome(),
        CheckOutcome::Valid
    );
    assert_eq!(
        valid_decision.journal().unwrap().tolerance_events().len(),
        1
    );

    let mut store = Store::new();
    let body = block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let face = store.faces_of_body(body).unwrap()[0];
    let original_domain = store.get(face).unwrap().domain;

    let mut rejected = store.transaction().unwrap();
    rejected.assembly().get_mut(face).unwrap().domain = None;
    let rejected = rejected
        .commit_full(&[body], FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(!rejected.is_committed());
    assert!(rejected.journal().is_none());
    assert_eq!(rejected.checks().len(), 1);
    assert_eq!(
        rejected.checks()[0].report().outcome(),
        CheckOutcome::Indeterminate
    );
    assert_eq!(store.get(face).unwrap().domain, original_domain);

    let mut allowed = store.transaction().unwrap();
    allowed.assembly().get_mut(face).unwrap().domain = None;
    let allowed = allowed
        .commit_full(&[body], FullCommitRequirement::AllowIndeterminate)
        .unwrap();
    assert!(allowed.is_committed());
    assert_eq!(
        allowed.checks()[0].report().outcome(),
        CheckOutcome::Indeterminate
    );
    assert!(allowed.journal().is_some());
    assert_eq!(store.get(face).unwrap().domain, None);
}

#[test]
fn full_commit_proof_fault_restores_tolerances_and_future_point_identity() {
    let mut store = Store::new();
    let body = ktopo::make::sphere(&mut store, &Frame::world(), 2.0).unwrap();
    let face = store.faces_of_body(body).unwrap()[0];
    let original_sense = store.get(face).unwrap().sense;
    let mut control = store.clone();

    let mut transaction = store.transaction().unwrap();
    let attempted_point = transaction.assembly().add(Point3::new(4.0, 5.0, 6.0));
    transaction
        .grow_tolerances(
            "full-rejected",
            LINEAR_RESOLUTION,
            &[ToleranceGrowth::new(
                ToleranceGrowthTarget::Face(face),
                2.0 * LINEAR_RESOLUTION,
            )],
        )
        .unwrap();
    transaction.assembly().get_mut(face).unwrap().sense = match original_sense {
        Sense::Forward => Sense::Reversed,
        Sense::Reversed => Sense::Forward,
    };
    let decision = transaction
        .commit_full(&[], FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(!decision.is_committed());
    assert_eq!(decision.checks().len(), 1, "affected root must be checked");
    assert_eq!(decision.checks()[0].body(), body);
    assert_eq!(
        decision.checks()[0].report().outcome(),
        CheckOutcome::Invalid
    );
    assert_eq!(store.get(face).unwrap().sense, original_sense);
    assert_eq!(store.get(face).unwrap().tolerance, None);

    let next = store.insert_point(Point3::new(4.0, 5.0, 6.0)).unwrap();
    let control_next = control.insert_point(Point3::new(4.0, 5.0, 6.0)).unwrap();
    assert_eq!(next, control_next);
    assert_eq!(next, attempted_point);
}

#[test]
fn full_commit_checks_duplicate_explicit_roots_once_before_affected_roots() {
    let mut store = Store::new();
    let explicit = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    let affected = block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let affected_face = store.faces_of_body(affected).unwrap()[0];

    let mut transaction = store.transaction().unwrap();
    transaction
        .assembly()
        .get_mut(affected_face)
        .unwrap()
        .domain = None;
    let decision = transaction
        .commit_full(&[explicit, explicit], FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(!decision.is_committed());
    assert_eq!(
        decision
            .checks()
            .iter()
            .map(|check| check.body())
            .collect::<Vec<_>>(),
        vec![explicit, affected]
    );
    assert_eq!(
        decision.checks()[1].report().outcome(),
        CheckOutcome::Indeterminate
    );
}

#[test]
fn checked_body_creation_is_atomic_and_journaled_deterministically() {
    let mut store = Store::new();
    let mut control = Store::new();
    let made = block_with_journal(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
    let expected = block_with_journal(&mut control, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
    assert_eq!(made.body(), expected.body());
    assert_eq!(made.journal(), expected.journal());
    assert!(
        made.journal()
            .mutations()
            .iter()
            .all(|mutation| mutation.kind == MutationKind::Created)
    );
    assert!(made.journal().lineage().is_empty());
    assert!(check_body(&store, made.body()).unwrap().is_empty());

    let mut failed = Store::new();
    let mut pristine = Store::new();
    // The torus relation is rejected only after its raw constructor has
    // created the body scaffold, so this exercises rollback of partial
    // topology rather than input-only preflight.
    assert!(torus_with_journal(&mut failed, &Frame::world(), 1.0, 2.0).is_err());
    let after_failure = block(&mut failed, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    let pristine_body = block(&mut pristine, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    assert_eq!(
        after_failure, pristine_body,
        "rollback must restore the next body identity"
    );
}
