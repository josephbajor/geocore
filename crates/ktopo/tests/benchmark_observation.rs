//! Exact benchmark-only phase counters on ordinary checked commits.

#![cfg(feature = "benchmark-internals")]

use kgeom::vec::Point3;
use ktopo::benchmark::{CommitObservation, last_commit};
use ktopo::entity::{BodyId, PointId};
use ktopo::{make, store::Store};

fn isolated_acorns(body_count: usize) -> (Store, Vec<BodyId>, Vec<PointId>) {
    let mut store = Store::new();
    let mut bodies = Vec::with_capacity(body_count);
    let mut points = Vec::with_capacity(body_count);
    for ordinal in 0..body_count {
        let body = make::acorn(&mut store, Point3::new(ordinal as f64 * 0.25, 0.0, 0.0)).unwrap();
        let vertex = store.vertices_of_body(body).unwrap()[0];
        bodies.push(body);
        points.push(store.get(vertex).unwrap().point);
    }
    (store, bodies, points)
}

fn observation(store: &Store) -> CommitObservation {
    last_commit(store).expect("checked commit records a benchmark observation")
}

#[test]
fn clean_commit_records_phase_boundaries_without_body_work() {
    let (mut store, _, _) = isolated_acorns(4);

    store.transaction().unwrap().commit_checked(&[]).unwrap();

    let observed = observation(&store);
    assert!(observed.committed);
    assert_eq!(observed.body_count, 4);
    assert_eq!(observed.affected_bodies, 0);
    assert_eq!(observed.refreshed_bodies, 0);
    assert_eq!(observed.checked_bodies, 0);
    assert_eq!(observed.mutations, 0);
    assert_eq!(observed.geometry_graph_validation_starts, 1);
    assert_eq!(observed.geometry_graph_validation_primary_node_starts, 0);
    assert_eq!(observed.candidate_index_clone_starts, 1);
    assert_eq!(observed.candidate_index_cloned_body_footprints, 4);
    assert_eq!(observed.candidate_index_cloned_body_order_entries, 4);
    assert_eq!(observed.candidate_index_refresh_body_starts, 0);
    assert_eq!(observed.candidate_index_body_order_refresh_entries, 0);
    assert_eq!(observed.affected_root_selection_starts, 2);
    assert_eq!(observed.affected_root_selection_mutation_items, 0);
    assert_eq!(observed.fast_body_check_starts, 0);
}

#[test]
fn local_edit_records_one_refresh_and_one_fast_check_start() {
    let (mut store, _, points) = isolated_acorns(2);
    let mut transaction = store.transaction().unwrap();
    transaction.assembly().get_mut(points[0]).unwrap().y = 0.5;

    transaction.commit_checked(&[]).unwrap();

    let observed = observation(&store);
    assert!(observed.committed);
    assert_eq!(observed.body_count, 2);
    assert_eq!(observed.affected_bodies, 1);
    assert_eq!(observed.refreshed_bodies, 1);
    assert_eq!(observed.checked_bodies, 1);
    assert_eq!(observed.mutations, 1);
    assert_eq!(observed.geometry_graph_validation_starts, 1);
    assert_eq!(observed.geometry_graph_validation_primary_node_starts, 0);
    assert_eq!(observed.candidate_index_clone_starts, 1);
    assert_eq!(observed.candidate_index_cloned_body_footprints, 2);
    assert_eq!(observed.candidate_index_cloned_body_order_entries, 2);
    assert_eq!(observed.candidate_index_refresh_body_starts, 1);
    assert_eq!(observed.candidate_index_body_order_refresh_entries, 2);
    assert_eq!(observed.affected_root_selection_starts, 2);
    assert_eq!(observed.affected_root_selection_mutation_items, 2);
    assert_eq!(observed.fast_body_check_starts, 1);
}

#[test]
fn shared_geometry_edit_records_fanout_refresh_and_fast_check_starts() {
    let (mut store, bodies, points) = isolated_acorns(3);
    let shared = points[0];
    let mut setup = store.transaction().unwrap();
    for &body in bodies.iter().skip(1) {
        let vertex = setup.store().vertices_of_body(body).unwrap()[0];
        let old_point = setup.store().get(vertex).unwrap().point;
        setup.assembly().get_mut(vertex).unwrap().point = shared;
        setup.assembly().remove(old_point).unwrap();
    }
    setup.commit_checked(&[]).unwrap();

    let mut transaction = store.transaction().unwrap();
    transaction.assembly().get_mut(shared).unwrap().z = 0.5;
    transaction.commit_checked(&[]).unwrap();

    let observed = observation(&store);
    assert!(observed.committed);
    assert_eq!(observed.body_count, 3);
    assert_eq!(observed.affected_bodies, 3);
    assert_eq!(observed.refreshed_bodies, 3);
    assert_eq!(observed.checked_bodies, 3);
    assert_eq!(observed.mutations, 1);
    assert_eq!(observed.geometry_graph_validation_starts, 1);
    assert_eq!(observed.geometry_graph_validation_primary_node_starts, 0);
    assert_eq!(observed.candidate_index_clone_starts, 1);
    assert_eq!(observed.candidate_index_cloned_body_footprints, 3);
    assert_eq!(observed.candidate_index_cloned_body_order_entries, 3);
    assert_eq!(observed.candidate_index_refresh_body_starts, 3);
    assert_eq!(observed.candidate_index_body_order_refresh_entries, 3);
    assert_eq!(observed.affected_root_selection_starts, 2);
    assert_eq!(observed.affected_root_selection_mutation_items, 2);
    assert_eq!(observed.fast_body_check_starts, 3);
}

#[test]
fn stale_explicit_body_is_selected_before_fast_check_invocation() {
    let (mut store, bodies, _) = isolated_acorns(1);
    let body = bodies[0];
    let mut transaction = store.transaction().unwrap();
    transaction.assembly().remove(body).unwrap();

    assert!(transaction.commit_checked(&[body]).is_err());

    let observed = observation(&store);
    assert!(!observed.committed);
    assert_eq!(observed.affected_bodies, 1);
    assert_eq!(observed.refreshed_bodies, 0);
    assert_eq!(observed.checked_bodies, 1);
    assert_eq!(observed.fast_body_check_starts, 0);
}
