//! Transaction-owned tolerance provenance and aggregate growth budgets.

use kcore::error::Error;
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::frame::Frame;
use ktopo::check::FaultKind;
use ktopo::entity::EntityRef;
use ktopo::make;
use ktopo::store::Store;
use ktopo::tolerance::{EntityTolerance, ToleranceOrigin};
use ktopo::transaction::{Journal, MutationKind};

fn model() -> (Store, ktopo::entity::BodyId, ktopo::entity::EdgeId) {
    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    let edge = store.edges_of_body(body).unwrap()[0];
    (store, body, edge)
}

fn journaled_growth() -> (Store, Journal) {
    let (mut store, body, edge) = model();
    let vertex = store.get(edge).unwrap().vertices[0].unwrap();
    let mut transaction = store.transaction().unwrap();
    let budget = transaction
        .declare_tolerance_budget("sew", LINEAR_RESOLUTION * 5.0)
        .unwrap();
    transaction
        .grow_edge_tolerance(budget, edge, LINEAR_RESOLUTION * 3.0)
        .unwrap();
    transaction
        .grow_vertex_tolerance(budget, vertex, LINEAR_RESOLUTION * 4.0)
        .unwrap();
    let journal = transaction.commit_checked_body(body).unwrap();
    (store, journal)
}

#[test]
fn committed_growth_is_provenanced_budgeted_and_journaled() {
    let (store, journal) = journaled_growth();
    assert_eq!(journal.tolerance_budgets().len(), 1);
    let report = journal.tolerance_budgets()[0];
    assert_eq!(report.operation(), "sew");
    assert_eq!(report.limit(), LINEAR_RESOLUTION * 5.0);
    assert_eq!(report.consumed(), LINEAR_RESOLUTION * 5.0);
    assert_eq!(report.remaining(), 0.0);
    assert_eq!(journal.tolerance_events().len(), 2);
    assert!(journal.tolerance_events().iter().all(|event| {
        event.previous().is_none()
            && event.current().origin() == ToleranceOrigin::Operation("sew")
            && event.current().last_operation() == Some("sew")
            && event.budget().index() == 0
    }));
    assert!(journal.mutations().iter().any(|mutation| {
        mutation.entity == journal.tolerance_events()[0].entity()
            && mutation.kind == MutationKind::Modified
    }));

    for event in journal.tolerance_events() {
        let tolerance = match event.entity() {
            EntityRef::Edge(edge) => store.get(edge).unwrap().tolerance.unwrap(),
            EntityRef::Vertex(vertex) => store.get(vertex).unwrap().tolerance.unwrap(),
            other => panic!("unexpected tolerance event: {other:?}"),
        };
        assert_eq!(tolerance, event.current());
    }
}

#[test]
fn imported_origin_survives_budgeted_growth() {
    let (mut store, body, edge) = model();
    let imported = EntityTolerance::imported_xt(LINEAR_RESOLUTION * 2.0).unwrap();
    store.get_mut(edge).unwrap().tolerance = Some(imported);

    let mut transaction = store.transaction().unwrap();
    let budget = transaction
        .declare_tolerance_budget("heal", LINEAR_RESOLUTION * 3.0)
        .unwrap();
    transaction
        .grow_edge_tolerance(budget, edge, LINEAR_RESOLUTION * 5.0)
        .unwrap();
    let journal = transaction.commit_checked_body(body).unwrap();

    let current = store.get(edge).unwrap().tolerance.unwrap();
    assert_eq!(current.origin(), ToleranceOrigin::ImportedXt);
    assert_eq!(current.origin_value(), LINEAR_RESOLUTION * 2.0);
    assert_eq!(
        current.accumulated_growth(),
        LINEAR_RESOLUTION * 5.0 - LINEAR_RESOLUTION * 2.0
    );
    assert_eq!(current.last_operation(), Some("heal"));
    assert_eq!(journal.tolerance_events()[0].previous(), Some(imported));
    assert_eq!(journal.tolerance_events()[0].current(), current);
}

#[test]
fn budget_exhaustion_is_atomic_and_does_not_record_a_change() {
    let (mut store, body, edge) = model();
    let mut transaction = store.transaction().unwrap();
    let budget = transaction
        .declare_tolerance_budget("sew", LINEAR_RESOLUTION)
        .unwrap();
    let error = transaction
        .grow_edge_tolerance(budget, edge, LINEAR_RESOLUTION * 3.0)
        .unwrap_err();
    assert!(matches!(error, Error::ToleranceBudgetExceeded { .. }));
    assert_eq!(transaction.store().get(edge).unwrap().tolerance, None);
    let journal = transaction.commit_checked_body(body).unwrap();
    assert_eq!(journal.tolerance_budgets()[0].consumed(), 0.0);
    assert!(journal.tolerance_events().is_empty());
    assert_eq!(store.get(edge).unwrap().tolerance, None);
}

#[test]
fn checked_commit_rolls_back_tolerance_growth_with_faulted_topology() {
    let (mut store, body, edge) = model();
    let original = store.get(edge).unwrap().clone();
    let mut transaction = store.transaction().unwrap();
    let budget = transaction
        .declare_tolerance_budget("heal", LINEAR_RESOLUTION * 2.0)
        .unwrap();
    transaction
        .grow_edge_tolerance(budget, edge, LINEAR_RESOLUTION * 3.0)
        .unwrap();
    transaction.store_mut().get_mut(edge).unwrap().vertices = [None, None];
    let error = transaction.commit_checked_body(body).unwrap_err();
    assert!(matches!(error, Error::TopologyCheckFailed { .. }));
    assert_eq!(store.get(edge).unwrap(), &original);
    let faults = ktopo::check::check_body(&store, body).unwrap();
    assert!(
        !faults
            .iter()
            .any(|fault| fault.kind == FaultKind::MissingVertices)
    );
}

#[test]
fn budget_journals_are_deterministic_and_invalid_limits_are_typed() {
    let (_, first) = journaled_growth();
    let (_, second) = journaled_growth();
    assert_eq!(first, second);

    let (mut store, _, _) = model();
    let mut transaction = store.transaction().unwrap();
    assert!(matches!(
        transaction.declare_tolerance_budget("bad", f64::NAN),
        Err(Error::InvalidToleranceBudget { .. })
    ));
    assert!(matches!(
        transaction.declare_tolerance_budget("bad", -1.0),
        Err(Error::InvalidToleranceBudget { .. })
    ));
}
