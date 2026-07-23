use super::assemble::AnalyticShellAssemblyError;
use super::tests::{full_cylinder_input, half_cylinder_input, shifted_full_cylinder_input};
use super::{
    AnalyticEdgeKey, AnalyticEdgeSplitPiece, AnalyticFaceSplitPiece, AnalyticShellClosedEdge,
    AnalyticShellPlanError,
};
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
fn analytic_face_merge_lineage_preserves_caller_source_order() {
    fn assemble(reversed: bool) -> Vec<LineageEvent> {
        let mut store = Store::new();
        let source_body = make::block(&mut store, &Frame::world(), [3.0, 3.0, 3.0]).unwrap();
        let source_faces = store.faces_of_body(source_body).unwrap();
        let ordered = if reversed {
            [source_faces[1], source_faces[0]]
        } else {
            [source_faces[0], source_faces[1]]
        };
        let mut input = full_cylinder_input();
        input.faces[0] = input.faces[0]
            .clone()
            .with_merge_sources([EntityRef::Face(ordered[0]), EntityRef::Face(ordered[1])]);

        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&input, 1.0e-12)
            .unwrap();
        let result = output
            .faces()
            .iter()
            .find_map(|(key, face)| (key.value() == 0).then_some(*face))
            .unwrap();
        let journal = transaction.commit_checked(&[output.body()]).unwrap();
        assert_eq!(
            journal.lineage(),
            [LineageEvent::Merge {
                sources: vec![EntityRef::Face(ordered[0]), EntityRef::Face(ordered[1]),],
                result: EntityRef::Face(result),
            }],
        );
        journal.lineage().to_vec()
    }

    let forward = assemble(false);
    let replayed = assemble(false);
    let reversed = assemble(true);
    assert_eq!(forward, replayed);
    assert_ne!(forward, reversed);
}

#[test]
fn analytic_face_split_lineage_spans_components_in_explicit_semantic_order() {
    fn assemble(reverse_components: bool, reverse_faces: bool) -> Vec<LineageEvent> {
        let mut store = Store::new();
        let source_body = make::block(&mut store, &Frame::world(), [3.0, 3.0, 3.0]).unwrap();
        let source = EntityRef::Face(store.faces_of_body(source_body).unwrap()[0]);
        let mut first = full_cylinder_input();
        let mut second = shifted_full_cylinder_input();
        first.faces[0] = first.faces[0]
            .clone()
            .with_split_lineage(source, AnalyticFaceSplitPiece::First);
        second.faces[1] = second.faces[1]
            .clone()
            .with_split_lineage(source, AnalyticFaceSplitPiece::Second);
        if reverse_faces {
            first.faces.reverse();
            second.faces.reverse();
        }
        let inputs = if reverse_components {
            [second, first]
        } else {
            [first, second]
        };

        let mut transaction = store.transaction().unwrap();
        let outputs = transaction
            .assemble_analytic_shell_batch(&inputs, 1.0e-12)
            .unwrap();
        let result_face = |component: usize, key: u64| {
            EntityRef::Face(
                outputs[component]
                    .faces()
                    .iter()
                    .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
                    .unwrap(),
            )
        };
        let expected = if reverse_components {
            [result_face(1, 0), result_face(0, 1)]
        } else {
            [result_face(0, 0), result_face(1, 1)]
        };
        let bodies = outputs
            .iter()
            .map(|output| output.body())
            .collect::<Vec<_>>();
        let journal = transaction.commit_checked(&bodies).unwrap();
        assert_eq!(
            journal.lineage(),
            [LineageEvent::Split {
                source,
                pieces: expected.to_vec(),
            }]
        );
        journal.lineage().to_vec()
    }

    assert_eq!(assemble(false, false), assemble(false, true));
    assert_eq!(assemble(true, false), assemble(true, true));
}

#[test]
fn invalid_batch_face_split_metadata_refuses_before_any_allocation() {
    fn assert_refused(
        store: &mut Store,
        inputs: &[super::AnalyticShellInput],
        expected: impl FnOnce(&AnalyticShellPlanError) -> bool,
    ) {
        let before = counts(store);
        let mut transaction = store.transaction().unwrap();
        let error = transaction
            .assemble_analytic_shell_batch(inputs, 1.0e-12)
            .unwrap_err();
        let AnalyticShellAssemblyError::Preflight(error) = error else {
            panic!("expected allocation-free preflight refusal")
        };
        assert!(expected(&error), "unexpected preflight error: {error:?}");
        assert_eq!(counts(transaction.store()), before);
        transaction.rollback().unwrap();
    }

    let mut store = Store::new();
    let source_body = make::block(&mut store, &Frame::world(), [3.0, 3.0, 3.0]).unwrap();
    let source_face = EntityRef::Face(store.faces_of_body(source_body).unwrap()[0]);
    let source_edge = EntityRef::Edge(store.edges_of_body(source_body).unwrap()[0]);

    let mut incomplete = full_cylinder_input();
    incomplete.faces[0] = incomplete.faces[0]
        .clone()
        .with_split_lineage(source_face, AnalyticFaceSplitPiece::First);
    assert_refused(
        &mut store,
        &[incomplete],
        |error| matches!(error, AnalyticShellPlanError::InvalidFaceSplitLineage(source) if *source == source_face),
    );

    let mut duplicate_first = full_cylinder_input();
    let mut duplicate_second = shifted_full_cylinder_input();
    duplicate_first.faces[0] = duplicate_first.faces[0]
        .clone()
        .with_split_lineage(source_face, AnalyticFaceSplitPiece::First);
    duplicate_second.faces[0] = duplicate_second.faces[0]
        .clone()
        .with_split_lineage(source_face, AnalyticFaceSplitPiece::First);
    assert_refused(
        &mut store,
        &[duplicate_first, duplicate_second],
        |error| matches!(error, AnalyticShellPlanError::InvalidFaceSplitLineage(source) if *source == source_face),
    );

    let mut wrong_kind_first = full_cylinder_input();
    let mut wrong_kind_second = shifted_full_cylinder_input();
    wrong_kind_first.faces[0] = wrong_kind_first.faces[0]
        .clone()
        .with_split_lineage(source_edge, AnalyticFaceSplitPiece::First);
    wrong_kind_second.faces[0] = wrong_kind_second.faces[0]
        .clone()
        .with_split_lineage(source_edge, AnalyticFaceSplitPiece::Second);
    assert_refused(
        &mut store,
        &[wrong_kind_first, wrong_kind_second],
        |error| matches!(error, AnalyticShellPlanError::InvalidFaceSplitLineage(source) if *source == source_edge),
    );

    let mut conflicting_first = full_cylinder_input();
    let mut conflicting_second = shifted_full_cylinder_input();
    conflicting_first.faces[0] = conflicting_first.faces[0]
        .clone()
        .with_source(source_face)
        .with_split_lineage(source_face, AnalyticFaceSplitPiece::First);
    conflicting_second.faces[0] = conflicting_second.faces[0]
        .clone()
        .with_split_lineage(source_face, AnalyticFaceSplitPiece::Second);
    assert_refused(
        &mut store,
        &[conflicting_first, conflicting_second],
        |error| matches!(error, AnalyticShellPlanError::ConflictingFaceLineage(_)),
    );

    let mut reused_first = full_cylinder_input();
    let mut reused_second = shifted_full_cylinder_input();
    reused_first.faces[0] = reused_first.faces[0]
        .clone()
        .with_split_lineage(source_face, AnalyticFaceSplitPiece::First);
    reused_second.faces[0] = reused_second.faces[0]
        .clone()
        .with_split_lineage(source_face, AnalyticFaceSplitPiece::Second);
    reused_second.faces[1] = reused_second.faces[1].clone().with_source(source_face);
    assert_refused(&mut store, &[reused_first, reused_second], |error| {
        matches!(error, AnalyticShellPlanError::ConflictingFaceLineage(_))
    });

    let stale_face = {
        let mut other = Store::new();
        let body = make::block(&mut other, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        EntityRef::Face(other.faces_of_body(body).unwrap()[0])
    };
    let mut stale_first = full_cylinder_input();
    let mut stale_second = shifted_full_cylinder_input();
    stale_first.faces[0] = stale_first.faces[0]
        .clone()
        .with_split_lineage(stale_face, AnalyticFaceSplitPiece::First);
    stale_second.faces[0] = stale_second.faces[0]
        .clone()
        .with_split_lineage(stale_face, AnalyticFaceSplitPiece::Second);
    let mut empty_store = Store::new();
    assert_refused(
        &mut empty_store,
        &[stale_first, stale_second],
        |error| matches!(error, AnalyticShellPlanError::StaleLineage(source) if *source == stale_face),
    );
}

#[test]
fn endpoint_free_edge_merge_lineage_preserves_caller_source_order() {
    fn assemble(reversed: bool) -> Vec<LineageEvent> {
        let mut store = Store::new();
        let source_body = make::cylinder(&mut store, &Frame::world(), 1.0, 1.0).unwrap();
        let source_edges = store
            .edges_of_body(source_body)
            .unwrap()
            .into_iter()
            .filter(|edge| store.get(*edge).unwrap().vertices() == [None, None])
            .collect::<Vec<_>>();
        let ordered = if reversed {
            [source_edges[1], source_edges[0]]
        } else {
            [source_edges[0], source_edges[1]]
        };
        let mut input = full_cylinder_input();
        input.closed_edges[0] = input.closed_edges[0]
            .with_merge_sources([EntityRef::Edge(ordered[0]), EntityRef::Edge(ordered[1])]);

        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&input, 1.0e-12)
            .unwrap();
        let result = output
            .edges()
            .iter()
            .find_map(|(key, edge)| (key.value() == 0).then_some(*edge))
            .unwrap();
        let journal = transaction.commit_checked(&[output.body()]).unwrap();
        assert_eq!(
            journal.lineage(),
            [LineageEvent::Merge {
                sources: vec![EntityRef::Edge(ordered[0]), EntityRef::Edge(ordered[1])],
                result: EntityRef::Edge(result),
            }]
        );
        journal.lineage().to_vec()
    }

    let forward = assemble(false);
    assert_eq!(forward, assemble(false));
    assert_ne!(forward, assemble(true));
}

#[test]
fn endpoint_free_edge_multi_source_derivation_preserves_caller_source_order() {
    fn assemble(reversed: bool) -> Vec<LineageEvent> {
        let mut store = Store::new();
        let source_body = make::cylinder(&mut store, &Frame::world(), 1.0, 1.0).unwrap();
        let source_faces = store.faces_of_body(source_body).unwrap();
        let ordered = if reversed {
            [source_faces[1], source_faces[0]]
        } else {
            [source_faces[0], source_faces[1]]
        };
        let sources = [EntityRef::Face(ordered[0]), EntityRef::Face(ordered[1])];
        let mut input = full_cylinder_input();
        input.closed_edges[0] = input.closed_edges[0].with_derived_sources(sources);

        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&input, 1.0e-12)
            .unwrap();
        let result = EntityRef::Edge(
            output
                .edges()
                .iter()
                .find_map(|(key, edge)| (key.value() == 0).then_some(*edge))
                .unwrap(),
        );
        let journal = transaction.commit_checked(&[output.body()]).unwrap();
        assert_eq!(
            journal.lineage(),
            [
                LineageEvent::DerivedFrom {
                    derived: result,
                    source: sources[0],
                },
                LineageEvent::DerivedFrom {
                    derived: result,
                    source: sources[1],
                },
            ]
        );
        journal.lineage().to_vec()
    }

    let forward = assemble(false);
    assert_eq!(forward, assemble(false));
    assert_ne!(forward, assemble(true));
}

#[test]
fn bounded_edge_multi_source_derivation_preserves_caller_source_order() {
    fn assemble(reversed: bool) -> Vec<LineageEvent> {
        let mut store = Store::new();
        let source_body = make::cylinder(&mut store, &Frame::world(), 1.0, 1.0).unwrap();
        let source_faces = store.faces_of_body(source_body).unwrap();
        let ordered = if reversed {
            [source_faces[1], source_faces[0]]
        } else {
            [source_faces[0], source_faces[1]]
        };
        let sources = [EntityRef::Face(ordered[0]), EntityRef::Face(ordered[1])];
        let mut input = half_cylinder_input();
        input.edges[0] = input.edges[0].with_derived_sources(sources);

        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&input, 1.0e-12)
            .unwrap();
        let result = EntityRef::Edge(
            output
                .edges()
                .iter()
                .find_map(|(key, edge)| (key == &input.edges[0].key()).then_some(*edge))
                .unwrap(),
        );
        let journal = transaction.commit_checked(&[output.body()]).unwrap();
        assert_eq!(
            journal.lineage(),
            [
                LineageEvent::DerivedFrom {
                    derived: result,
                    source: sources[0],
                },
                LineageEvent::DerivedFrom {
                    derived: result,
                    source: sources[1],
                },
            ]
        );
        journal.lineage().to_vec()
    }

    let forward = assemble(false);
    assert_eq!(forward, assemble(false));
    assert_ne!(forward, assemble(true));
}

#[test]
fn invalid_bounded_edge_multi_source_derivation_refuses_before_any_allocation() {
    fn assert_refused(
        store: &mut Store,
        input: &super::AnalyticShellInput,
        expected: impl FnOnce(&AnalyticShellPlanError) -> bool,
    ) {
        let before = counts(store);
        let mut transaction = store.transaction().unwrap();
        let error = transaction
            .assemble_analytic_shell(input, 1.0e-12)
            .unwrap_err();
        let AnalyticShellAssemblyError::Preflight(error) = error else {
            panic!("expected allocation-free preflight refusal")
        };
        assert!(expected(&error), "unexpected preflight error: {error:?}");
        assert_eq!(counts(transaction.store()), before);
        transaction.rollback().unwrap();
    }

    let mut store = Store::new();
    let source_body = make::cylinder(&mut store, &Frame::world(), 1.0, 1.0).unwrap();
    let source_faces = store.faces_of_body(source_body).unwrap();
    let sources = [
        EntityRef::Face(source_faces[0]),
        EntityRef::Face(source_faces[1]),
    ];

    let mut conflict = half_cylinder_input();
    conflict.edges[0] = conflict.edges[0]
        .with_source(sources[0])
        .with_derived_sources(sources);
    assert_refused(&mut store, &conflict, |error| {
        matches!(error, AnalyticShellPlanError::ConflictingEdgeLineage(_))
    });

    let mut duplicate = half_cylinder_input();
    duplicate.edges[0] = duplicate.edges[0].with_derived_sources([sources[0], sources[0]]);
    assert_refused(&mut store, &duplicate, |error| {
        matches!(error, AnalyticShellPlanError::InvalidEdgeDerivedLineage(_))
    });

    let stale_face = {
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&full_cylinder_input(), 1.0e-12)
            .unwrap();
        let face = output.faces()[0].1;
        transaction.rollback().unwrap();
        EntityRef::Face(face)
    };
    let mut stale = half_cylinder_input();
    stale.edges[0] = stale.edges[0].with_derived_sources([sources[0], stale_face]);
    assert_refused(
        &mut store,
        &stale,
        |error| matches!(error, AnalyticShellPlanError::StaleLineage(source) if *source == stale_face),
    );
}

#[test]
fn invalid_endpoint_free_edge_multi_source_derivation_refuses_before_any_allocation() {
    fn assert_refused(
        store: &mut Store,
        input: &super::AnalyticShellInput,
        expected: impl FnOnce(&AnalyticShellPlanError) -> bool,
    ) {
        let before = counts(store);
        let mut transaction = store.transaction().unwrap();
        let error = transaction
            .assemble_analytic_shell(input, 1.0e-12)
            .unwrap_err();
        let AnalyticShellAssemblyError::Preflight(error) = error else {
            panic!("expected allocation-free preflight refusal")
        };
        assert!(expected(&error), "unexpected preflight error: {error:?}");
        assert_eq!(counts(transaction.store()), before);
        transaction.rollback().unwrap();
    }

    let mut store = Store::new();
    let source_body = make::cylinder(&mut store, &Frame::world(), 1.0, 1.0).unwrap();
    let source_faces = store.faces_of_body(source_body).unwrap();
    let source_edges = store.edges_of_body(source_body).unwrap();
    let sources = [
        EntityRef::Face(source_faces[0]),
        EntityRef::Face(source_faces[1]),
    ];

    let conflicts = [
        full_cylinder_input().with_closed_edges(vec![
            full_cylinder_input().closed_edges()[0]
                .with_source(sources[0])
                .with_derived_sources(sources),
            full_cylinder_input().closed_edges()[1],
        ]),
        full_cylinder_input().with_closed_edges(vec![
            full_cylinder_input().closed_edges()[0]
                .with_merge_sources([
                    EntityRef::Edge(source_edges[0]),
                    EntityRef::Edge(source_edges[1]),
                ])
                .with_derived_sources(sources),
            full_cylinder_input().closed_edges()[1],
        ]),
    ];
    for conflict in &conflicts {
        assert_refused(&mut store, conflict, |error| {
            matches!(error, AnalyticShellPlanError::ConflictingEdgeLineage(_))
        });
    }

    let mut duplicate = full_cylinder_input();
    duplicate.closed_edges[0] =
        duplicate.closed_edges[0].with_derived_sources([sources[0], sources[0]]);
    assert_refused(&mut store, &duplicate, |error| {
        matches!(error, AnalyticShellPlanError::InvalidEdgeDerivedLineage(_))
    });

    let stale_face = {
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&full_cylinder_input(), 1.0e-12)
            .unwrap();
        let face = output.faces()[0].1;
        transaction.rollback().unwrap();
        EntityRef::Face(face)
    };
    let mut stale = full_cylinder_input();
    stale.closed_edges[0] = stale.closed_edges[0].with_derived_sources([sources[0], stale_face]);
    assert_refused(
        &mut store,
        &stale,
        |error| matches!(error, AnalyticShellPlanError::StaleLineage(source) if *source == stale_face),
    );
}

#[test]
fn invalid_endpoint_free_edge_merge_refuses_before_any_allocation() {
    fn assert_refused(
        store: &mut Store,
        input: &super::AnalyticShellInput,
        expected: impl FnOnce(&AnalyticShellPlanError) -> bool,
    ) {
        let before = counts(store);
        let mut transaction = store.transaction().unwrap();
        let error = transaction
            .assemble_analytic_shell(input, 1.0e-12)
            .unwrap_err();
        let AnalyticShellAssemblyError::Preflight(error) = error else {
            panic!("expected allocation-free preflight refusal")
        };
        assert!(expected(&error), "unexpected preflight error: {error:?}");
        assert_eq!(counts(transaction.store()), before);
        transaction.rollback().unwrap();
    }

    let mut store = Store::new();
    let source_body = make::cylinder(&mut store, &Frame::world(), 1.0, 1.0).unwrap();
    let source_edges = store.edges_of_body(source_body).unwrap();
    let source_faces = store.faces_of_body(source_body).unwrap();
    let edge = EntityRef::Edge(source_edges[0]);

    let mut duplicate = full_cylinder_input();
    duplicate.closed_edges[0] = duplicate.closed_edges[0].with_merge_sources([edge, edge]);
    assert_refused(&mut store, &duplicate, |error| {
        matches!(error, AnalyticShellPlanError::InvalidEdgeMergeLineage(_))
    });

    let mut wrong_kind = full_cylinder_input();
    wrong_kind.closed_edges[0] = wrong_kind.closed_edges[0].with_merge_sources([
        EntityRef::Face(source_faces[0]),
        EntityRef::Face(source_faces[1]),
    ]);
    assert_refused(&mut store, &wrong_kind, |error| {
        matches!(error, AnalyticShellPlanError::InvalidEdgeMergeLineage(_))
    });

    let mut conflicting = full_cylinder_input();
    conflicting.closed_edges[0] = conflicting.closed_edges[0]
        .with_source(edge)
        .with_merge_sources([
            EntityRef::Edge(source_edges[0]),
            EntityRef::Edge(source_edges[1]),
        ]);
    assert_refused(&mut store, &conflicting, |error| {
        matches!(error, AnalyticShellPlanError::ConflictingEdgeLineage(_))
    });

    let stale_edges = {
        let mut other = Store::new();
        let body = make::cylinder(&mut other, &Frame::world(), 1.0, 1.0).unwrap();
        other.edges_of_body(body).unwrap()
    };
    let mut stale = full_cylinder_input();
    stale.closed_edges[0] = stale.closed_edges[0].with_merge_sources([
        EntityRef::Edge(stale_edges[0]),
        EntityRef::Edge(stale_edges[1]),
    ]);
    let mut empty_store = Store::new();
    assert_refused(&mut empty_store, &stale, |error| {
        matches!(error, AnalyticShellPlanError::StaleLineage(_))
    });
}

#[test]
fn analytic_edge_split_lineage_is_explicit_binary_ordered_and_deterministic() {
    fn assemble(reverse_declarations: bool) -> Vec<LineageEvent> {
        let mut store = Store::new();
        let source_body = make::block(&mut store, &Frame::world(), [3.0, 3.0, 3.0]).unwrap();
        let source = EntityRef::Edge(store.edges_of_body(source_body).unwrap()[0]);
        let mut input = half_cylinder_input();
        input.edges[0] = input.edges[0].with_split_lineage(source, AnalyticEdgeSplitPiece::Second);
        input.edges[1] = input.edges[1].with_split_lineage(source, AnalyticEdgeSplitPiece::First);
        if reverse_declarations {
            input.edges.reverse();
        }

        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&input, 1.0e-12)
            .unwrap();
        let edge = |value| {
            EntityRef::Edge(
                output
                    .edges()
                    .iter()
                    .find_map(|(key, edge)| (key.value() == value).then_some(*edge))
                    .unwrap(),
            )
        };
        let expected = vec![LineageEvent::Split {
            source,
            pieces: vec![edge(1), edge(0)],
        }];
        let journal = transaction.commit_checked(&[output.body()]).unwrap();
        assert_eq!(journal.lineage(), expected);
        journal.lineage().to_vec()
    }

    let first = assemble(false);
    assert_eq!(assemble(false), first);
    assert_eq!(assemble(true), first);
}

#[test]
fn repeated_edge_derivations_are_not_inferred_as_a_split() {
    let mut store = Store::new();
    let source_body = make::block(&mut store, &Frame::world(), [3.0, 3.0, 3.0]).unwrap();
    let source = EntityRef::Edge(store.edges_of_body(source_body).unwrap()[0]);
    let mut input = half_cylinder_input();
    input.edges[0] = input.edges[0].with_source(source);
    input.edges[1] = input.edges[1].with_source(source);

    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap();
    let journal = transaction.commit_checked(&[output.body()]).unwrap();
    assert_eq!(journal.lineage().len(), 2);
    assert!(journal.lineage().iter().all(|event| matches!(
        event,
        LineageEvent::DerivedFrom {
            source: candidate,
            ..
        } if *candidate == source
    )));
}

#[test]
fn invalid_binary_edge_split_metadata_refuses_before_allocation() {
    let mut store = Store::new();
    let source_body = make::block(&mut store, &Frame::world(), [3.0, 3.0, 3.0]).unwrap();
    let source_edge = EntityRef::Edge(store.edges_of_body(source_body).unwrap()[0]);
    let source_face = EntityRef::Face(store.faces_of_body(source_body).unwrap()[0]);

    let mut incomplete = half_cylinder_input();
    incomplete.edges[0] =
        incomplete.edges[0].with_split_lineage(source_edge, AnalyticEdgeSplitPiece::First);
    assert!(matches!(
        super::prepare_analytic_shell(&incomplete, &store, 1.0e-12),
        Err(AnalyticShellPlanError::InvalidEdgeSplitLineage(source)) if source == source_edge
    ));

    let mut duplicate = half_cylinder_input();
    duplicate.edges[0] =
        duplicate.edges[0].with_split_lineage(source_edge, AnalyticEdgeSplitPiece::First);
    duplicate.edges[1] =
        duplicate.edges[1].with_split_lineage(source_edge, AnalyticEdgeSplitPiece::First);
    assert!(matches!(
        super::prepare_analytic_shell(&duplicate, &store, 1.0e-12),
        Err(AnalyticShellPlanError::InvalidEdgeSplitLineage(source)) if source == source_edge
    ));

    let mut wrong_kind = half_cylinder_input();
    wrong_kind.edges[0] =
        wrong_kind.edges[0].with_split_lineage(source_face, AnalyticEdgeSplitPiece::First);
    wrong_kind.edges[1] =
        wrong_kind.edges[1].with_split_lineage(source_face, AnalyticEdgeSplitPiece::Second);
    assert!(matches!(
        super::prepare_analytic_shell(&wrong_kind, &store, 1.0e-12),
        Err(AnalyticShellPlanError::InvalidEdgeSplitLineage(source)) if source == source_face
    ));

    let closed_template = full_cylinder_input().closed_edges()[0];
    let closed_key = AnalyticEdgeKey::new(u64::MAX);
    for source_slot in 0..2 {
        let mut sources = [source_face, source_face];
        sources[source_slot] = source_edge;
        let closed = AnalyticShellClosedEdge::new(
            closed_key,
            closed_template.carrier(),
            closed_template.logical_range(),
        )
        .with_derived_sources(sources);
        let mut cross_declaration_conflict = half_cylinder_input().with_closed_edges(vec![closed]);
        cross_declaration_conflict.edges[0] = cross_declaration_conflict.edges[0]
            .with_split_lineage(source_edge, AnalyticEdgeSplitPiece::First);
        cross_declaration_conflict.edges[1] = cross_declaration_conflict.edges[1]
            .with_split_lineage(source_edge, AnalyticEdgeSplitPiece::Second);

        let before = counts(&store);
        let mut transaction = store.transaction().unwrap();
        assert!(matches!(
            transaction.assemble_analytic_shell(&cross_declaration_conflict, 1.0e-12),
            Err(AnalyticShellAssemblyError::Preflight(
                AnalyticShellPlanError::ConflictingEdgeLineage(key)
            )) if key == closed_key
        ));
        assert_eq!(counts(transaction.store()), before);
        transaction.rollback().unwrap();
    }

    let mut bounded_cross_declaration_conflict = half_cylinder_input();
    bounded_cross_declaration_conflict.edges[0] = bounded_cross_declaration_conflict.edges[0]
        .with_split_lineage(source_edge, AnalyticEdgeSplitPiece::First);
    bounded_cross_declaration_conflict.edges[1] = bounded_cross_declaration_conflict.edges[1]
        .with_split_lineage(source_edge, AnalyticEdgeSplitPiece::Second);
    let conflict_key = bounded_cross_declaration_conflict.edges[2].key();
    bounded_cross_declaration_conflict.edges[2] = bounded_cross_declaration_conflict.edges[2]
        .with_derived_sources([source_edge, source_face]);
    let before = counts(&store);
    let mut transaction = store.transaction().unwrap();
    assert!(matches!(
        transaction.assemble_analytic_shell(&bounded_cross_declaration_conflict, 1.0e-12),
        Err(AnalyticShellAssemblyError::Preflight(
            AnalyticShellPlanError::ConflictingEdgeLineage(key)
        )) if key == conflict_key
    ));
    assert_eq!(counts(transaction.store()), before);
    transaction.rollback().unwrap();

    let mut conflicting = half_cylinder_input();
    conflicting.edges[0] = conflicting.edges[0]
        .with_source(source_edge)
        .with_split_lineage(source_edge, AnalyticEdgeSplitPiece::First);
    conflicting.edges[1] =
        conflicting.edges[1].with_split_lineage(source_edge, AnalyticEdgeSplitPiece::Second);
    let before = counts(&store);
    let mut transaction = store.transaction().unwrap();
    assert!(matches!(
        transaction.assemble_analytic_shell(&conflicting, 1.0e-12),
        Err(AnalyticShellAssemblyError::Preflight(
            AnalyticShellPlanError::ConflictingEdgeLineage(_)
        ))
    ));
    assert_eq!(counts(transaction.store()), before);
    transaction.rollback().unwrap();
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

#[test]
fn batch_face_split_and_closed_edge_merge_full_commit_and_replay_after_rollback() {
    let mut store = Store::new();
    let source_body = make::cylinder(&mut store, &Frame::world(), 1.0, 1.0).unwrap();
    let source_face = EntityRef::Face(store.faces_of_body(source_body).unwrap()[0]);
    let source_edges = store
        .edges_of_body(source_body)
        .unwrap()
        .into_iter()
        .filter(|edge| store.get(*edge).unwrap().vertices() == [None, None])
        .collect::<Vec<_>>();
    let merge_sources = [
        EntityRef::Edge(source_edges[0]),
        EntityRef::Edge(source_edges[1]),
    ];
    let mut first = full_cylinder_input();
    let mut second = shifted_full_cylinder_input();
    first.faces[0] = first.faces[0]
        .clone()
        .with_split_lineage(source_face, AnalyticFaceSplitPiece::First);
    second.faces[0] = second.faces[0]
        .clone()
        .with_split_lineage(source_face, AnalyticFaceSplitPiece::Second);
    first.closed_edges[0] = first.closed_edges[0].with_merge_sources(merge_sources);
    let inputs = [first, second];

    let mut transaction = store.transaction().unwrap();
    let rolled_back = transaction
        .assemble_analytic_shell_batch(&inputs, 1.0e-12)
        .unwrap();
    transaction.rollback().unwrap();

    let mut transaction = store.transaction().unwrap();
    let outputs = transaction
        .assemble_analytic_shell_batch(&inputs, 1.0e-12)
        .unwrap();
    assert_eq!(outputs, rolled_back);
    let result_edge = EntityRef::Edge(
        outputs[0]
            .edges()
            .iter()
            .find_map(|(key, edge)| (key.value() == 0).then_some(*edge))
            .unwrap(),
    );
    let result_face = |component: usize| {
        EntityRef::Face(
            outputs[component]
                .faces()
                .iter()
                .find_map(|(key, face)| (key.value() == 0).then_some(*face))
                .unwrap(),
        )
    };
    let bodies = outputs
        .iter()
        .map(|output| output.body())
        .collect::<Vec<_>>();
    let decision = transaction
        .commit_full(&bodies, FullCommitRequirement::RequireValid)
        .unwrap();
    assert!(decision.is_committed());
    assert!(decision.checks().iter().all(|check| {
        check.report().outcome() == CheckOutcome::Valid
            && check.report().faults.is_empty()
            && check.report().gaps.is_empty()
    }));
    assert_eq!(
        decision.journal().unwrap().lineage(),
        [
            LineageEvent::Merge {
                sources: merge_sources.to_vec(),
                result: result_edge,
            },
            LineageEvent::Split {
                source: source_face,
                pieces: vec![result_face(0), result_face(1)],
            },
        ]
    );
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
