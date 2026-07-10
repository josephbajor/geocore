//! Generic graph assembly is transaction-scoped; the public Store is read-only.

use kcore::error::Error;
use kgeom::frame::Frame;
use kgeom::surface::Plane;
use kgeom::vec::{Point3, Vec3};
use ktopo::check::check_body;
use ktopo::entity::{Body, BodyKind, Region, RegionKind, Shell};
use ktopo::geom::SurfaceGeom;
use ktopo::make::block;
use ktopo::store::Store;
use ktopo::transaction::{AssemblyStore, MutationKind};

fn empty_wire(assembly: &mut AssemblyStore<'_>) -> ktopo::entity::BodyId {
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
    body
}

fn checked_empty_wire(store: &mut Store) -> (ktopo::entity::BodyId, ktopo::transaction::Journal) {
    let mut transaction = store.transaction().unwrap();
    let body = empty_wire(&mut transaction.assembly());
    let journal = transaction.commit_checked_body(body).unwrap();
    (body, journal)
}

#[test]
fn dropped_assembly_restores_identity_and_future_allocations() {
    let mut store = Store::new();
    {
        let mut transaction = store.transaction().unwrap();
        let _ = empty_wire(&mut transaction.assembly());
        // Drop without commit: every assembled identity must disappear.
    }
    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<Region>(), 0);
    assert_eq!(store.count::<Shell>(), 0);

    let mut control = Store::new();
    let (body, journal) = checked_empty_wire(&mut store);
    let (control_body, control_journal) = checked_empty_wire(&mut control);
    assert_eq!(body, control_body);
    assert_eq!(journal, control_journal);
    assert!(
        journal
            .mutations()
            .iter()
            .all(|mutation| mutation.kind == MutationKind::Created)
    );
}

#[test]
fn checked_commit_rejects_an_invalid_assembled_graph() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let body = transaction.assembly().add(Body {
        kind: BodyKind::Wire,
        regions: Vec::new(),
    });
    assert!(matches!(
        transaction.commit_checked_body(body),
        Err(Error::TopologyCheckFailed { fault_count }) if fault_count > 0
    ));
    assert_eq!(store.count::<Body>(), 0);
}

#[test]
fn checked_commit_cannot_hide_an_invalid_unlisted_body() {
    let mut store = Store::new();
    let (valid_body, _) = checked_empty_wire(&mut store);
    let mut transaction = store.transaction().unwrap();
    transaction.assembly().add(Body {
        kind: BodyKind::Wire,
        regions: Vec::new(),
    });
    assert!(matches!(
        transaction.commit_checked_body(valid_body),
        Err(Error::TopologyCheckFailed { fault_count }) if fault_count > 0
    ));
    assert_eq!(store.count::<Body>(), 1);
}

#[test]
fn checked_commit_rejects_orphan_topology() {
    let mut store = Store::new();
    let (body, _) = checked_empty_wire(&mut store);
    let mut transaction = store.transaction().unwrap();
    transaction.assembly().add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    assert!(matches!(
        transaction.commit_checked_body(body),
        Err(Error::TopologyCheckFailed { fault_count }) if fault_count > 0
    ));
    assert_eq!(store.count::<Region>(), 1);
}

#[test]
fn incremental_index_rejects_cross_body_topology_sharing() {
    let mut store = Store::new();
    let (first, _) = checked_empty_wire(&mut store);
    let (second, _) = checked_empty_wire(&mut store);
    let first_region = store.get(first).unwrap().regions[0];
    let first_shell = store.get(first_region).unwrap().shells[0];
    let second_region = store.get(second).unwrap().regions[0];
    let original_shells = store.get(second_region).unwrap().shells.clone();
    let mut transaction = store.transaction().unwrap();
    transaction
        .assembly()
        .get_mut(second_region)
        .unwrap()
        .shells
        .push(first_shell);
    assert!(matches!(
        transaction.commit_checked_body(second),
        Err(Error::TopologyCheckFailed { fault_count }) if fault_count > 0
    ));
    assert_eq!(store.get(second_region).unwrap().shells, original_shells);
}

#[test]
fn incremental_index_removes_a_committed_body_footprint() {
    let mut store = Store::new();
    let (body, _) = checked_empty_wire(&mut store);
    let region = store.get(body).unwrap().regions[0];
    let shell = store.get(region).unwrap().shells[0];
    let mut transaction = store.transaction().unwrap();
    {
        let mut assembly = transaction.assembly();
        assembly.remove(shell).unwrap();
        assembly.remove(region).unwrap();
        assembly.remove(body).unwrap();
    }
    let journal = transaction.commit_checked(&[]).unwrap();
    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<Region>(), 0);
    assert_eq!(store.count::<Shell>(), 0);
    assert!(
        journal
            .mutations()
            .iter()
            .all(|mutation| mutation.kind == MutationKind::Deleted)
    );
}

#[test]
fn affected_root_selection_finds_topology_without_explicit_hints() {
    let mut store = Store::new();
    let (body, _) = checked_empty_wire(&mut store);
    let original_regions = store.get(body).unwrap().regions.clone();
    let mut transaction = store.transaction().unwrap();
    transaction
        .assembly()
        .get_mut(body)
        .unwrap()
        .regions
        .clear();
    assert!(matches!(
        transaction.commit_checked(&[]),
        Err(Error::TopologyCheckFailed { fault_count }) if fault_count > 0
    ));
    assert_eq!(store.get(body).unwrap().regions, original_regions);
}

#[test]
fn affected_root_selection_tracks_shared_geometry_without_explicit_hints() {
    let mut store = Store::new();
    let first = block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
    let second = block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
    let first_face = store.faces_of_body(first).unwrap()[0];
    let second_face = store.faces_of_body(second).unwrap()[0];
    let shared_surface = store.get(first_face).unwrap().surface;

    let mut share = store.transaction().unwrap();
    share.assembly().get_mut(second_face).unwrap().surface = shared_surface;
    share.commit_checked_body(second).unwrap();
    assert!(check_body(&store, first).unwrap().is_empty());
    assert!(check_body(&store, second).unwrap().is_empty());

    let SurfaceGeom::Plane(original) = store.get(shared_surface).unwrap() else {
        panic!("block face must use a plane");
    };
    let original_origin = original.frame().origin();
    let shifted = SurfaceGeom::Plane(Plane::new(
        Frame::from_z(Point3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 1.0)).unwrap(),
    ));
    let mut transaction = store.transaction().unwrap();
    *transaction.assembly().get_mut(shared_surface).unwrap() = shifted;
    assert!(matches!(
        transaction.commit_checked(&[]),
        Err(Error::TopologyCheckFailed { fault_count }) if fault_count > 0
    ));
    let SurfaceGeom::Plane(restored) = store.get(shared_surface).unwrap() else {
        panic!("rolled-back block face must use a plane");
    };
    assert_eq!(restored.frame().origin(), original_origin);
    assert!(check_body(&store, first).unwrap().is_empty());
    assert!(check_body(&store, second).unwrap().is_empty());
}

#[test]
fn detached_point_insertion_validates_the_size_box() {
    let mut store = Store::new();
    let point = store.insert_point(Point3::new(1.0, 2.0, 3.0)).unwrap();
    assert_eq!(*store.get(point).unwrap(), Point3::new(1.0, 2.0, 3.0));
    assert!(store.insert_point(Point3::new(501.0, 0.0, 0.0)).is_err());
    assert!(store.insert_point(Point3::new(f64::NAN, 0.0, 0.0)).is_err());
    assert_eq!(store.count::<Point3>(), 1);
}
