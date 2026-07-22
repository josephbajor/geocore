use super::AnalyticShellPlanError;
use super::assemble::AnalyticShellAssemblyError;
use super::tests::{full_cylinder_input, half_cylinder_input, shifted_full_cylinder_input};
use crate::check::{CheckLevel, CheckOutcome, check_body_report};
use crate::entity::{Body, Edge, EntityRef, Face, Fin, FinPcurve, Loop, Region, Shell, Vertex};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::incidence::{PcurveIssue, check_pcurve_incidence};
use crate::make;
use crate::store::Store;
use crate::transaction::{FullCommitRequirement, Journal, LineageEvent};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

#[test]
fn genuinely_mixed_shell_is_fast_clean_and_records_exact_shared_incidence() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&half_cylinder_input(), 1.0e-12)
        .unwrap();

    assert_eq!(output.vertices().len(), 4);
    assert_eq!(output.edges().len(), 6);
    assert_eq!(output.faces().len(), 4);
    let report = check_body_report(transaction.store(), output.body(), CheckLevel::Fast).unwrap();
    assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:#?}");

    for &(key, edge_id) in output.edges() {
        let edge = transaction.store().get(edge_id).unwrap();
        assert_eq!(edge.fins().len(), 2, "edge {key:?}");
        let fins = [
            transaction.store().get(edge.fins()[0]).unwrap(),
            transaction.store().get(edge.fins()[1]).unwrap(),
        ];
        assert_eq!(fins[0].sense(), fins[1].sense().flipped());
        assert!(fins.iter().all(|fin| fin.pcurve().is_some()));

        let [Some(tail), Some(head)] = edge.vertices() else {
            panic!("bounded analytic edges retain both vertices")
        };
        let (lo, hi) = edge.bounds().unwrap();
        let tail_position = *transaction
            .store()
            .get(transaction.store().get(tail).unwrap().point())
            .unwrap();
        let head_position = *transaction
            .store()
            .get(transaction.store().get(head).unwrap().point())
            .unwrap();
        let carrier = transaction.store().get(edge.curve().unwrap()).unwrap();
        let endpoints = [carrier.as_curve().eval(lo), carrier.as_curve().eval(hi)];
        assert_point_bits(tail_position, endpoints[0]);
        assert_point_bits(head_position, endpoints[1]);
    }

    let surface_classes = output
        .faces()
        .iter()
        .map(|&(_, face)| transaction.store().get(face).unwrap().surface())
        .map(|surface| transaction.store().get(surface).unwrap())
        .fold((0, 0), |(planes, cylinders), surface| match surface {
            SurfaceGeom::Plane(_) => (planes + 1, cylinders),
            SurfaceGeom::Cylinder(_) => (planes, cylinders + 1),
            _ => (planes, cylinders),
        });
    assert_eq!(surface_classes, (3, 1));

    transaction.rollback().unwrap();
    assert_eq!(counts(&store), [0; 12]);
}

#[test]
fn full_policy_accepts_the_structurally_certified_mixed_shell() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&half_cylinder_input(), 1.0e-12)
        .unwrap();
    let decision = transaction
        .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(decision.is_committed());
    assert_eq!(decision.checks().len(), 1);
    let report = decision.checks()[0].report();
    assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:#?}");
    assert!(report.faults.is_empty());
    assert!(report.gaps.is_empty(), "{report:#?}");
}

#[test]
fn endpoint_free_full_cylinder_assembles_without_vertices_or_bounds() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&full_cylinder_input(), 1.0e-12)
        .unwrap();

    assert!(output.vertices().is_empty());
    assert_eq!(output.edges().len(), 2);
    assert_eq!(output.faces().len(), 3);
    for &(_, edge_id) in output.edges() {
        let edge = transaction.store().get(edge_id).unwrap();
        assert_eq!(edge.vertices(), [None, None]);
        assert!(edge.bounds().is_none());
        assert_eq!(edge.fins().len(), 2);
        let fins = edge
            .fins()
            .iter()
            .map(|fin| transaction.store().get(*fin).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(fins[0].sense(), fins[1].sense().flipped());
        assert!(
            fins.iter()
                .all(|fin| fin.pcurve().unwrap().closure_winding().is_some())
        );
    }

    let decision = transaction
        .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(decision.is_committed());
    let report = decision.checks()[0].report();
    assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:#?}");
    assert!(report.faults.is_empty(), "{report:#?}");
    assert!(report.gaps.is_empty(), "{report:#?}");
}

#[test]
fn shifted_endpoint_free_period_assembles_with_checked_fin_range_authority() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&shifted_full_cylinder_input(), 1.0e-12)
        .unwrap();

    for &(_, edge_id) in output.edges() {
        let edge = transaction.store().get(edge_id).unwrap();
        assert_eq!(edge.vertices(), [None, None]);
        assert!(edge.bounds().is_none());
        let curve = edge.curve().unwrap();
        for &fin_id in edge.fins() {
            let fin = transaction.store().get(fin_id).unwrap();
            let loop_ = transaction.store().get(fin.parent()).unwrap();
            let face = transaction.store().get(loop_.face()).unwrap();
            assert_eq!(
                check_pcurve_incidence(
                    transaction.store(),
                    curve,
                    None,
                    face.surface(),
                    fin.pcurve().unwrap(),
                    1.0e-12,
                ),
                Ok(())
            );
        }
    }

    let decision = transaction
        .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(decision.is_committed());
    assert_eq!(decision.checks()[0].report().outcome(), CheckOutcome::Valid);
}

#[test]
fn shifted_endpoint_free_period_refuses_a_partial_fin_authority() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&shifted_full_cylinder_input(), 1.0e-12)
        .unwrap();
    let edge_id = output.edges()[0].1;
    let edge = transaction.store().get(edge_id).unwrap();
    let curve = edge.curve().unwrap();
    let fin = transaction.store().get(edge.fins()[0]).unwrap();
    let use_ = fin.pcurve().unwrap();
    let range = use_.range();
    let partial = ParamRange::new(range.lo, range.lo + range.width() / 2.0);
    let tampered = FinPcurve::new(use_.curve(), partial, use_.edge_to_pcurve())
        .unwrap()
        .with_chart(use_.chart())
        .with_closure_winding(use_.closure_winding().unwrap());
    let loop_ = transaction.store().get(fin.parent()).unwrap();
    let face = transaction.store().get(loop_.face()).unwrap();
    assert_eq!(
        check_pcurve_incidence(
            transaction.store(),
            curve,
            None,
            face.surface(),
            tampered,
            1.0e-12,
        ),
        Err(PcurveIssue::BadRange)
    );
}

#[test]
fn endpoint_free_batch_tamper_refuses_before_any_allocation() {
    let valid = half_cylinder_input();
    let mut invalid = full_cylinder_input();
    invalid.faces[0].loops[0].fins[0].pcurve.closure_winding = None;
    let mut store = Store::new();
    let before = counts(&store);
    let mut transaction = store.transaction().unwrap();
    let error = transaction
        .assemble_analytic_shell_batch(&[valid, invalid], 1.0e-12)
        .unwrap_err();
    assert!(matches!(
        error,
        AnalyticShellAssemblyError::Preflight(AnalyticShellPlanError::MissingClosureWinding { .. })
    ));
    assert_eq!(counts(transaction.store()), before);
    transaction.rollback().unwrap();
}

#[test]
fn endpoint_free_edge_preserves_live_source_lineage() {
    let mut store = Store::new();
    let source_body = make::cylinder(&mut store, &Frame::world(), 1.0, 1.0).unwrap();
    let source_edge = store
        .edges_of_body(source_body)
        .unwrap()
        .into_iter()
        .find(|edge| store.get(*edge).unwrap().vertices() == [None, None])
        .unwrap();
    let mut input = full_cylinder_input();
    input.closed_edges[0].source = Some(EntityRef::Edge(source_edge));

    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap();
    let derived_edge = output
        .edges()
        .iter()
        .find(|(key, _)| key.value() == 0)
        .unwrap()
        .1;
    let journal = transaction.commit_checked(&[output.body()]).unwrap();
    assert!(journal.lineage().contains(&LineageEvent::DerivedFrom {
        derived: EntityRef::Edge(derived_edge),
        source: EntityRef::Edge(source_edge),
    }));
}

#[test]
fn deterministic_journal_lineage_and_rollback_reuse_the_same_handles() {
    let first = assemble_with_lineage();
    let second = assemble_with_lineage();
    assert_eq!(first, second);

    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let rolled_back = transaction
        .assemble_analytic_shell(&half_cylinder_input(), 1.0e-12)
        .unwrap();
    transaction.rollback().unwrap();
    let mut transaction = store.transaction().unwrap();
    let replayed = transaction
        .assemble_analytic_shell(&half_cylinder_input(), 1.0e-12)
        .unwrap();
    assert_eq!(replayed, rolled_back);
    transaction.rollback().unwrap();
}

#[test]
fn invalid_plan_refuses_before_the_first_transaction_allocation() {
    let mut input = half_cylinder_input();
    input.vertices.push(input.vertices[0]);
    let mut store = Store::new();
    let before = counts(&store);
    let mut transaction = store.transaction().unwrap();
    let error = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap_err();
    assert!(matches!(
        error,
        AnalyticShellAssemblyError::Preflight(AnalyticShellPlanError::DuplicateVertex(_))
    ));
    assert_eq!(counts(transaction.store()), before);
    transaction.rollback().unwrap();
}

#[test]
fn batch_preflights_every_component_before_the_first_allocation() {
    let valid = half_cylinder_input();
    let mut invalid = half_cylinder_input();
    invalid.vertices.push(invalid.vertices[0]);

    let mut store = Store::new();
    let before = counts(&store);
    let mut transaction = store.transaction().unwrap();
    let error = transaction
        .assemble_analytic_shell_batch(&[valid, invalid], 1.0e-12)
        .unwrap_err();
    assert!(matches!(
        error,
        AnalyticShellAssemblyError::Preflight(AnalyticShellPlanError::DuplicateVertex(_))
    ));
    assert_eq!(
        counts(transaction.store()),
        before,
        "a later component's preflight failure allocated an earlier component"
    );
    transaction.rollback().unwrap();
    assert_eq!(counts(&store), before);
}

#[test]
fn batch_outputs_follow_input_order_and_commit_full_together() {
    let inputs = [half_cylinder_input(), half_cylinder_input()];
    let mut store = Store::new();

    let mut transaction = store.transaction().unwrap();
    let first = transaction
        .assemble_analytic_shell_batch(&inputs, 1.0e-12)
        .unwrap();
    assert_eq!(first.len(), inputs.len());
    assert_ne!(first[0].body(), first[1].body());
    assert_ne!(first[0].shell(), first[1].shell());
    transaction.rollback().unwrap();

    let mut transaction = store.transaction().unwrap();
    let replayed = transaction
        .assemble_analytic_shell_batch(&inputs, 1.0e-12)
        .unwrap();
    assert_eq!(
        replayed, first,
        "rollback changed deterministic batch order"
    );
    let bodies = replayed
        .iter()
        .map(|output| output.body())
        .collect::<Vec<_>>();
    let decision = transaction
        .commit_full(&bodies, FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(decision.is_committed());
    assert_eq!(decision.checks().len(), inputs.len());
    assert!(decision.checks().iter().all(|check| {
        check.report().outcome() == CheckOutcome::Valid
            && check.report().faults.is_empty()
            && check.report().gaps.is_empty()
    }));
}

fn assemble_with_lineage() -> (super::AnalyticShellOutput, Journal) {
    let mut store = Store::new();
    let source_body = make::block(&mut store, &Frame::world(), [3.0, 3.0, 3.0]).unwrap();
    let source_faces = store.faces_of_body(source_body).unwrap();
    let source_edges = store.edges_of_body(source_body).unwrap();
    let mut input = half_cylinder_input();
    input.faces[0].source = Some(EntityRef::Face(source_faces[0]));
    input.edges[0].source = Some(EntityRef::Edge(source_edges[0]));

    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap();
    let expected = vec![
        LineageEvent::DerivedFrom {
            derived: EntityRef::Face(output.faces()[0].1),
            source: EntityRef::Face(source_faces[0]),
        },
        LineageEvent::DerivedFrom {
            derived: EntityRef::Edge(output.edges()[0].1),
            source: EntityRef::Edge(source_edges[0]),
        },
    ];
    let journal = transaction.commit_checked(&[output.body()]).unwrap();
    assert_eq!(journal.lineage(), expected);
    (output, journal)
}

fn counts(store: &Store) -> [usize; 12] {
    [
        store.count::<Body>(),
        store.count::<Region>(),
        store.count::<Shell>(),
        store.count::<Face>(),
        store.count::<Loop>(),
        store.count::<Fin>(),
        store.count::<Edge>(),
        store.count::<Vertex>(),
        store.count::<CurveGeom>(),
        store.count::<SurfaceGeom>(),
        store.count::<Point3>(),
        store.count::<Curve2dGeom>(),
    ]
}

fn assert_point_bits(actual: Point3, expected: Point3) {
    assert_eq!(actual.x.to_bits(), expected.x.to_bits());
    assert_eq!(actual.y.to_bits(), expected.y.to_bits());
    assert_eq!(actual.z.to_bits(), expected.z.to_bits());
}
