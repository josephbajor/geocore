use kcore::{
    operation::{OperationContext, OperationScope},
    tolerance::Tolerances,
};

use super::super::boundary_select::{
    BoundaryFragmentClassification, ClassifiedBoundaryFragment, RegularizedBooleanOperation,
    select_boundary_fragments,
};
use super::super::curved_source::{CylinderSourceOutcome, extract_cylinder_source};
use super::super::mixed_face_arrangement::arrange_mixed_planar_face_with_lineage;
use super::super::mixed_periodic_arrangement::{
    arrange_mixed_periodic_face, arrange_mixed_periodic_face_from_embedding,
};
use super::super::parallel_cylinder_relation::{
    ParallelCylinderRelationOutcome, certify_parallel_cylinder_relation,
};
use super::*;
use crate::{BlockRequest, CylinderRequest, Kernel, SectionBodiesRequest};
use kgeom::frame::Frame;

type PlanarArrangementSet = Vec<(
    FaceId,
    super::super::mixed_face_arrangement::MixedPlanarFaceOutput,
)>;
type SelectedMixedCells = Vec<SelectedBoundaryFragment<MixedShellCellKey, ()>>;

fn plan_selected<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    let arrangement = arrange_mixed_shell(store, graph, bindings, selected)?;
    plan_mixed_shell(store, graph, arrangement)
}

fn store_shape(store: &Store) -> [usize; 5] {
    [
        store.count::<ktopo::entity::Face>(),
        store.count::<ktopo::entity::Loop>(),
        store.count::<ktopo::entity::Fin>(),
        store.count::<ktopo::entity::Edge>(),
        store.count::<ktopo::entity::Vertex>(),
    ]
}

#[test]
fn bounded_circle_period_lifts_follow_physical_fin_traversal() {
    assert_eq!(
        intrinsic_circle_period_shifts(Sense::Forward, [0.25, 1.75]),
        Some([0, 0])
    );
    assert_eq!(
        intrinsic_circle_period_shifts(Sense::Forward, [5.75, 0.25]),
        Some([0, 1])
    );
    assert_eq!(
        intrinsic_circle_period_shifts(Sense::Reversed, [5.75, 0.25]),
        Some([0, 0])
    );
    assert_eq!(
        intrinsic_circle_period_shifts(Sense::Reversed, [0.25, 5.75]),
        Some([1, 0])
    );
    assert_eq!(
        intrinsic_circle_period_shifts(Sense::Forward, [1.0, 1.0]),
        None
    );
    assert_eq!(
        intrinsic_circle_period_shifts(Sense::Forward, [f64::NAN, 1.0]),
        None
    );
}

#[test]
fn operation_local_path_ordinals_bind_lineage_without_global_promotion() {
    let frame = Frame::world();
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(-0.5, 0.0, -1.0)),
                1.0,
                3.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.5, 0.0, -1.0)),
                1.0,
                2.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    let part = session.part(part_id.clone()).unwrap();
    let graph = part
        .section_bodies(SectionBodiesRequest::new(first.clone(), second.clone()))
        .unwrap()
        .into_result()
        .unwrap();
    assert!(graph.periodic_face_embeddings().iter().all(|evidence| {
        matches!(
            evidence,
            SectionPeriodicFaceEmbeddingEvidence::Indeterminate {
                gap: crate::SectionPeriodicEmbeddingGap::UnstitchedFragmentPath { .. },
                ..
            }
        )
    }));
    let tolerances = Tolerances::default();
    let context = OperationContext::new(part.policy(), tolerances)
        .unwrap()
        .with_family_budget_defaults(super::super::BooleanBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    let mut extract = |body: &crate::BodyId| match extract_cylinder_source(
        &part.state.store,
        body.raw(),
        &mut scope,
    )
    .unwrap()
    {
        CylinderSourceOutcome::Ready(source) => source,
        other => panic!("fixture lost certified cylinder source: {other:?}"),
    };
    let sources = [extract(&first), extract(&second)];
    let relation = match certify_parallel_cylinder_relation(
        &part.state.store,
        &graph,
        [&sources[0], &sources[1]],
        &mut scope,
    )
    .unwrap()
    {
        ParallelCylinderRelationOutcome::CertifiedCoincidentCaps(relation) => relation,
        other => panic!("fixture lost certified coincident-cap relation: {other:?}"),
    };
    let mut saw_face_local_trace = false;
    for (operand, operand_source) in sources.iter().enumerate() {
        let face = FaceId::new(part_id.clone(), operand_source.side_face());
        let evidence = crate::section::certify_periodic_face_fragment_subset(
            &part.state.store,
            face.clone().part(),
            &graph,
            operand,
            face,
            &relation.periodic_fragment_subset(operand),
            tolerances.linear(),
        )
        .unwrap();
        let mut occurrences = BTreeSet::new();
        for trace in evidence.boundary_traces() {
            assert_eq!(trace.source_component(), None);
            assert_eq!(trace.component_ordinals().len(), trace.fragments().len());
            for (&ordinal, fragment) in trace.component_ordinals().iter().zip(trace.fragments()) {
                assert!(occurrences.insert((trace.component(), ordinal, fragment.fragment(),)));
            }
        }
        saw_face_local_trace |= !occurrences.is_empty();
        let arrangement = arrange_mixed_periodic_face_from_embedding(&graph, &evidence).unwrap();
        assert_eq!(occurrences.len(), arrangement.cut_fragments().len());
        let source = source_face_key(
            &part.state.store,
            &graph,
            &evidence.face(),
            evidence.operand(),
        )
        .unwrap();
        let lineage = periodic_cut_lineage(
            &graph,
            &evidence.face(),
            evidence.operand(),
            &arrangement,
            Some(&evidence),
            source,
        )
        .unwrap();
        assert_eq!(lineage.len(), arrangement.cut_fragments().len());
        for cut in arrangement.cut_fragments() {
            let retained = lineage.get(cut.key()).unwrap();
            assert_eq!(retained.fragment, cut.key().fragment());
            assert_eq!(
                retained.cylinder_period_shift,
                cut.key().cylinder_period_shift()
            );
        }
    }
    assert!(saw_face_local_trace);
}

fn with_fixture(
    frame: Frame,
    test: impl FnOnce(&mut Store, &BodySectionGraph, usize, FaceId, MixedPeriodicFaceArrangement),
) {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let block = edit
            .create_block(BlockRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, 1.0)),
                [2.0, 5.0, 1.0],
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(frame, 1.5, 2.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let graph = session
        .part(part_id.clone())
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(block, cylinder))
        .unwrap()
        .into_result()
        .unwrap();
    let (periodic_operand, periodic_face) = graph
        .periodic_face_embeddings()
        .iter()
        .find_map(|evidence| match evidence {
            SectionPeriodicFaceEmbeddingEvidence::Certified(value) => {
                Some((value.operand(), value.face()))
            }
            _ => None,
        })
        .unwrap();
    let periodic =
        arrange_mixed_periodic_face(&graph, periodic_face.clone(), periodic_operand).unwrap();
    let mut edit = session.edit_part(part_id).unwrap();
    test(
        edit.store_mut_for_test(),
        &graph,
        periodic_operand,
        periodic_face,
        periodic,
    );
}

fn selected_patch(
    store: &Store,
    graph: &BodySectionGraph,
    periodic_operand: usize,
    periodic_face: &FaceId,
    periodic: &MixedPeriodicFaceArrangement,
) -> (PlanarArrangementSet, SelectedMixedCells) {
    let periodic_source = source_face_key(store, graph, periodic_face, periodic_operand).unwrap();
    let periodic_cells = periodic
        .cells()
        .iter()
        .filter(|cell| matches!(cell.key(), PeriodicArrangementCellKey::ComponentDisk(_)))
        .collect::<Vec<_>>();
    assert!(!periodic_cells.is_empty());
    let periodic_lineage = periodic_cut_lineage(
        graph,
        periodic_face,
        periodic_operand,
        periodic,
        None,
        periodic_source,
    )
    .unwrap();
    let target_uses = periodic_cells
        .iter()
        .flat_map(|cell| cell.boundaries())
        .flat_map(ArrangementCycle::uses)
        .filter_map(|use_| match use_.edge() {
            ArrangementEdgeKey::Cut(key) => {
                let lineage = periodic_lineage.get(key).unwrap();
                Some((
                    lineage.fragment,
                    compose_direction(use_.direction(), lineage.arrangement_to_section),
                ))
            }
            ArrangementEdgeKey::Source(_) => None,
        })
        .collect::<Vec<_>>();
    assert!(!target_uses.is_empty());

    let planar_operand = 1 - periodic_operand;
    let mut planar_faces = Vec::<FaceId>::new();
    for (fragment, _) in &target_uses {
        let branch = &graph.branches()[graph.curve_fragments()[*fragment].branch()];
        let face = branch.faces()[planar_operand].clone();
        if !planar_faces.contains(&face) {
            planar_faces.push(face);
        }
    }
    let arrangements = planar_faces
        .into_iter()
        .map(|face| {
            let output =
                arrange_mixed_planar_face_with_lineage(store, graph, face.clone(), planar_operand)
                    .unwrap();
            (face, output)
        })
        .collect::<Vec<_>>();

    let mut selected_keys = periodic_cells
        .iter()
        .map(|cell| MixedShellCellKey::periodic(periodic_source, *cell.key()))
        .collect::<BTreeSet<_>>();
    for (fragment, periodic_direction) in target_uses {
        let mut matched = None;
        for (face, output) in &arrangements {
            let arrangement = output.arrangement();
            let source = source_face_key(store, graph, face, planar_operand).unwrap();
            let lineage =
                planar_cut_lineage(graph, face, planar_operand, arrangement, source).unwrap();
            for cell in arrangement.cells() {
                for use_ in cell
                    .boundaries()
                    .iter()
                    .flat_map(|boundary| boundary.uses())
                {
                    let ArrangementEdgeKey::Cut(key) = use_.edge() else {
                        continue;
                    };
                    let Some(candidate) = lineage.get(key) else {
                        continue;
                    };
                    let direction =
                        compose_direction(use_.direction(), candidate.arrangement_to_section);
                    if candidate.fragment == fragment && direction != periodic_direction {
                        let key = MixedShellCellKey::planar(source, cell.key());
                        assert!(matched.replace(key).is_none());
                    }
                }
            }
        }
        selected_keys.insert(matched.expect("opposed planar cell use"));
    }

    let classified = selected_keys.into_iter().map(|key| {
        ClassifiedBoundaryFragment::new(
            key,
            operand_side(key.source().operand()),
            (),
            BoundaryFragmentClassification::Exterior,
        )
    });
    let selected =
        select_boundary_fragments(RegularizedBooleanOperation::Unite, classified).unwrap();
    (arrangements, selected)
}

#[test]
fn certified_block_cylinder_patch_preserves_shared_identity_and_chart_lifts() {
    let oblique = Frame::new(
        kgeom::vec::Point3::new(3.0, -2.0, 1.25),
        kgeom::vec::Vec3::new(0.48, 0.64, 0.6),
        kgeom::vec::Vec3::new(0.8, -0.6, 0.0),
    )
    .unwrap();
    for frame in [Frame::world(), oblique] {
        with_fixture(
            frame,
            |store, graph, periodic_operand, periodic_face, periodic| {
                let (planar, selected) =
                    selected_patch(store, graph, periodic_operand, &periodic_face, &periodic);
                let bindings = std::iter::once(MixedArrangementBinding::Periodic {
                    face: periodic_face,
                    operand: periodic_operand,
                    arrangement: &periodic,
                    embedding: None,
                })
                .chain(planar.iter().map(|(face, output)| {
                    MixedArrangementBinding::Planar {
                        face: face.clone(),
                        operand: 1 - periodic_operand,
                        arrangement: output.arrangement(),
                        lineage: output.lineage(),
                    }
                }));
                let plan = plan_selected(store, graph, bindings, selected).unwrap();
                for edge in plan.section_edges() {
                    let uses = plan
                        .faces()
                        .iter()
                        .flat_map(MixedShellFacePlan::loops)
                        .flat_map(MixedShellLoopPlan::uses)
                        .filter(|use_| {
                            use_.edge()
                                == &MixedShellEdgeKey::SectionFragment(edge.fragment_index())
                        })
                        .collect::<Vec<_>>();
                    assert_eq!(uses.len(), 2);
                    assert_ne!(uses[0].direction(), uses[1].direction());
                }
                assert!(plan.materialization_gaps().is_empty());
                let blueprint =
                    materialize::prepare_mixed_shell_materialization(&plan, store).unwrap();
                assert!(blueprint.all_edges_have_two_opposed_uses());
                assert_eq!(
                    blueprint.planar_use_count(),
                    blueprint.planar_edge_count() * 2
                );
                let before = store_shape(store);
                let input = materialize::materialize_mixed_shell_input(
                    &plan,
                    store,
                    &materialize::MixedShellScalarInputs::empty(),
                    1.0e-9,
                )
                .unwrap();
                assert_eq!(store_shape(store), before);

                let mut transaction = store.transaction().unwrap();
                let output = transaction.assemble_analytic_shell(&input, 1.0e-9).unwrap();
                let faults = ktopo::check::check_body(transaction.store(), output.body()).unwrap();
                assert!(faults.is_empty(), "{faults:#?}");
                let full = ktopo::check::check_body_report(
                    transaction.store(),
                    output.body(),
                    ktopo::check::CheckLevel::Full,
                )
                .unwrap();
                assert_eq!(
                    full.outcome(),
                    ktopo::check::CheckOutcome::Valid,
                    "{full:#?}"
                );
                transaction.rollback().unwrap();
                assert_eq!(store_shape(store), before);
            },
        );
    }
}

#[test]
fn binding_and_selection_order_do_not_change_the_plan() {
    with_fixture(
        Frame::world(),
        |store, graph, periodic_operand, periodic_face, periodic| {
            let (planar, selected) =
                selected_patch(store, graph, periodic_operand, &periodic_face, &periodic);
            let make_bindings = || {
                let mut bindings = planar
                    .iter()
                    .map(|(face, output)| MixedArrangementBinding::Planar {
                        face: face.clone(),
                        operand: 1 - periodic_operand,
                        arrangement: output.arrangement(),
                        lineage: output.lineage(),
                    })
                    .collect::<Vec<_>>();
                bindings.push(MixedArrangementBinding::Periodic {
                    face: periodic_face.clone(),
                    operand: periodic_operand,
                    arrangement: &periodic,
                    embedding: None,
                });
                bindings
            };
            let expected = plan_selected(store, graph, make_bindings(), selected.clone()).unwrap();
            let mut bindings = make_bindings();
            bindings.reverse();
            let mut reversed_selected = selected;
            reversed_selected.reverse();
            let actual = plan_selected(store, graph, bindings, reversed_selected).unwrap();
            assert_eq!(actual, expected);
            assert_eq!(
                materialize::prepare_mixed_shell_materialization(&actual, store).unwrap(),
                materialize::prepare_mixed_shell_materialization(&expected, store).unwrap()
            );
            assert_eq!(
                materialize::materialize_mixed_shell_input(
                    &actual,
                    store,
                    &materialize::MixedShellScalarInputs::empty(),
                    1.0e-9,
                )
                .unwrap(),
                materialize::materialize_mixed_shell_input(
                    &expected,
                    store,
                    &materialize::MixedShellScalarInputs::empty(),
                    1.0e-9,
                )
                .unwrap()
            );
        },
    );
}

#[test]
fn missing_peer_and_forged_cell_fail_closed_without_metric_matching() {
    with_fixture(
        Frame::world(),
        |store, graph, periodic_operand, periodic_face, periodic| {
            let (planar, mut selected) =
                selected_patch(store, graph, periodic_operand, &periodic_face, &periodic);
            let target = planar
                .iter()
                .position(|(_, output)| {
                    output.lineage().spans().iter().any(|span| {
                        span.range().iter().any(|value| {
                            matches!(value, MixedSourceParameterEvidence::SectionRoot { .. })
                        })
                    })
                })
                .unwrap();
            let mut forged_root = planar[target].1.lineage().clone();
            let root = forged_root
                .spans
                .iter_mut()
                .flat_map(|span| &mut span.range)
                .find_map(|value| match value {
                    MixedSourceParameterEvidence::SectionRoot { enclosure_bits, .. } => {
                        Some(enclosure_bits)
                    }
                    _ => None,
                })
                .unwrap();
            root[0] ^= 1;
            let mut forged_vertex = planar[target].1.lineage().clone();
            forged_vertex.source_vertices.swap(0, 1);
            for forged in [forged_root, forged_vertex] {
                let bindings = std::iter::once(MixedArrangementBinding::Periodic {
                    face: periodic_face.clone(),
                    operand: periodic_operand,
                    arrangement: &periodic,
                    embedding: None,
                })
                .chain(planar.iter().enumerate().map(
                    |(index, (face, output))| MixedArrangementBinding::Planar {
                        face: face.clone(),
                        operand: 1 - periodic_operand,
                        arrangement: output.arrangement(),
                        lineage: if index == target {
                            &forged
                        } else {
                            output.lineage()
                        },
                    },
                ));
                assert!(matches!(
                    plan_selected(store, graph, bindings, selected.clone()),
                    Err(MixedShellPlanError::PlanarLineageMismatch(_))
                ));
            }
            selected.pop();
            let bindings = std::iter::once(MixedArrangementBinding::Periodic {
                face: periodic_face.clone(),
                operand: periodic_operand,
                arrangement: &periodic,
                embedding: None,
            })
            .chain(planar.iter().map(|(face, output)| {
                MixedArrangementBinding::Planar {
                    face: face.clone(),
                    operand: 1 - periodic_operand,
                    arrangement: output.arrangement(),
                    lineage: output.lineage(),
                }
            }));
            assert!(matches!(
                plan_selected(store, graph, bindings, selected),
                Err(MixedShellPlanError::SectionUseCount { actual: 1, .. })
            ));

            let periodic_source =
                source_face_key(store, graph, &periodic_face, periodic_operand).unwrap();
            let forged = ClassifiedBoundaryFragment::new(
                MixedShellCellKey::periodic(
                    periodic_source,
                    PeriodicArrangementCellKey::ComponentDisk(usize::MAX),
                ),
                operand_side(periodic_operand),
                (),
                BoundaryFragmentClassification::Exterior,
            );
            let forged =
                select_boundary_fragments(RegularizedBooleanOperation::Unite, [forged]).unwrap();
            assert!(matches!(
                plan_selected(
                    store,
                    graph,
                    [MixedArrangementBinding::Periodic {
                        face: periodic_face,
                        operand: periodic_operand,
                        arrangement: &periodic,
                        embedding: None,
                    }],
                    forged,
                ),
                Err(MixedShellPlanError::MissingPeriodicCell(_))
            ));
        },
    );
}
