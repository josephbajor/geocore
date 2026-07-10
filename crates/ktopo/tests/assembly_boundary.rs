//! Generic graph assembly is transaction-scoped; the public Store is read-only.

use kcore::error::Error;
use kgeom::vec::Point3;
use ktopo::entity::{Body, BodyKind, Region, RegionKind, Shell};
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
fn detached_point_insertion_validates_the_size_box() {
    let mut store = Store::new();
    let point = store.insert_point(Point3::new(1.0, 2.0, 3.0)).unwrap();
    assert_eq!(*store.get(point).unwrap(), Point3::new(1.0, 2.0, 3.0));
    assert!(store.insert_point(Point3::new(501.0, 0.0, 0.0)).is_err());
    assert!(store.insert_point(Point3::new(f64::NAN, 0.0, 0.0)).is_err());
    assert_eq!(store.count::<Point3>(), 1);
}
