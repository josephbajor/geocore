use std::collections::BTreeSet;

use kcore::{
    operation::{OperationContext, OperationScope},
    tolerance::Tolerances,
};
use kgeom::{frame::Frame, vec::Point3};

use super::super::{
    BooleanBudgetProfile,
    curved_source::{CylinderSourceOutcome, extract_cylinder_source},
    parallel_cylinder_relation::{
        ParallelCylinderRelationOutcome, certify_parallel_cylinder_relation,
    },
};
use super::*;
use crate::{
    BodyId, CylinderRequest, Kernel, PartId, SectionBodiesRequest, Session,
    section::certify_periodic_face_fragment_subset,
};

struct ParallelCylinderFixture {
    session: Session,
    part: PartId,
    bodies: [BodyId; 2],
    graph: BodySectionGraph,
}

fn parallel_cylinder_fixture(first_height: f64, second_height: f64) -> ParallelCylinderFixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                Frame::world().with_origin(Point3::new(-0.5, 0.0, 0.0)),
                1.0,
                first_height,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second = edit
            .create_cylinder(CylinderRequest::new(
                Frame::world().with_origin(Point3::new(0.5, 0.0, 0.0)),
                1.0,
                second_height,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    let graph = session
        .part(part.clone())
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(first.clone(), second.clone()))
        .unwrap()
        .into_result()
        .unwrap();
    ParallelCylinderFixture {
        session,
        part,
        bodies: [first, second],
        graph,
    }
}

fn operation_local_embeddings(
    fixture: &ParallelCylinderFixture,
) -> Vec<crate::CertifiedSectionPeriodicFaceEmbedding> {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let tolerances = Tolerances::default();
    let context = OperationContext::new(part.policy(), tolerances)
        .unwrap()
        .with_family_budget_defaults(BooleanBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    let sources = fixture.bodies.each_ref().map(|body| {
        match extract_cylinder_source(&part.state.store, body.raw(), &mut scope).unwrap() {
            CylinderSourceOutcome::Ready(source) => source,
            other => panic!("fixture lost certified cylinder source: {other:?}"),
        }
    });
    let relation = match certify_parallel_cylinder_relation(
        &part.state.store,
        &fixture.graph,
        [&sources[0], &sources[1]],
        &mut scope,
    )
    .unwrap()
    {
        ParallelCylinderRelationOutcome::CertifiedCoincidentCaps(relation) => relation,
        other => panic!("fixture lost certified coincident-cap relation: {other:?}"),
    };
    (0..2)
        .map(|operand| {
            let face = FaceId::new(fixture.part.clone(), sources[operand].side_face());
            certify_periodic_face_fragment_subset(
                &part.state.store,
                face.clone().part(),
                &fixture.graph,
                operand,
                face,
                &relation.periodic_fragment_subset(operand),
                tolerances.linear(),
            )
            .unwrap()
        })
        .collect()
}

type ArrangementShape = (usize, usize, usize, usize, usize, usize);

fn face_local_shapes(fixture: &ParallelCylinderFixture) -> Vec<ArrangementShape> {
    let graph = &fixture.graph;
    assert_eq!(graph.completion(), SectionCompletion::Indeterminate);
    assert_eq!(graph.curve_components().len(), 0);
    assert!(graph.periodic_face_embeddings().iter().all(|evidence| {
        matches!(
            evidence,
            SectionPeriodicFaceEmbeddingEvidence::Indeterminate {
                gap: SectionPeriodicEmbeddingGap::UnstitchedFragmentPath { .. },
                ..
            }
        )
    }));
    let mut shapes = Vec::new();
    for evidence in operation_local_embeddings(fixture) {
        assert!(
            evidence
                .boundary_traces()
                .iter()
                .all(|trace| trace.source_component().is_none())
        );
        let arrangement = arrange_mixed_periodic_face_from_embedding(graph, &evidence).unwrap();
        assert_eq!(
            arrange_mixed_periodic_face_from_embedding(graph, &evidence).unwrap(),
            arrangement
        );
        assert!(matches!(
            arrange_mixed_periodic_face_from_certified_embedding(
                graph,
                evidence.face(),
                evidence.operand(),
            ),
            Err(MixedPeriodicArrangementError::EmbeddingIndeterminate(
                SectionPeriodicEmbeddingGap::UnstitchedFragmentPath { .. }
            ))
        ));
        assert_eq!(
            arrange_mixed_periodic_face(graph, evidence.face(), evidence.operand()),
            Err(MixedPeriodicArrangementError::IncompleteSectionGraph)
        );
        let expected_keys = evidence
            .boundary_traces()
            .iter()
            .flat_map(|trace| {
                trace
                    .component_ordinals()
                    .iter()
                    .copied()
                    .zip(trace.fragments())
                    .map(|(ordinal, fragment)| {
                        (
                            trace.component(),
                            trace.source_component(),
                            ordinal,
                            fragment.fragment(),
                        )
                    })
            })
            .collect::<BTreeSet<_>>();
        let actual_keys = arrangement
            .cut_fragments()
            .iter()
            .map(|fragment| {
                let key = *fragment.key();
                (
                    key.component(),
                    key.source_component(),
                    key.ordinal(),
                    key.fragment(),
                )
            })
            .collect::<BTreeSet<_>>();
        assert_eq!(actual_keys, expected_keys);
        let fragment_ids = arrangement
            .cut_fragments()
            .iter()
            .map(|fragment| {
                assert_eq!(fragment.key().source_component(), None);
                fragment.key().fragment()
            })
            .collect::<BTreeSet<_>>();
        assert_eq!(fragment_ids.len(), arrangement.cut_fragments().len());
        shapes.push((
            arrangement.source_spans().len(),
            arrangement.cut_fragments().len(),
            arrangement.cells().len(),
            arrangement.adjacency().len(),
            arrangement
                .cells()
                .iter()
                .filter(|cell| cell.key() == &PeriodicArrangementCellKey::AnnularRemainder)
                .count(),
            arrangement
                .cells()
                .iter()
                .filter(|cell| matches!(cell.key(), PeriodicArrangementCellKey::TraceCell(_)))
                .count(),
        ));
    }
    shapes.sort_unstable();
    shapes
}

#[test]
fn certified_face_local_paths_feed_equal_and_shared_end_arrangements() {
    let equal = parallel_cylinder_fixture(2.0, 2.0);
    assert_eq!(
        face_local_shapes(&equal),
        vec![(4, 2, 2, 2, 0, 2), (4, 2, 2, 2, 0, 2)]
    );
    let shared_end = parallel_cylinder_fixture(3.0, 2.0);
    assert_eq!(
        face_local_shapes(&shared_end),
        vec![(3, 3, 2, 3, 1, 1), (4, 2, 2, 2, 0, 2)]
    );
}

#[test]
fn operation_local_embedding_rejects_missing_selected_fragment_and_ignores_unrelated_additions() {
    let fixture = parallel_cylinder_fixture(2.0, 2.0);
    let graph = &fixture.graph;
    let evidence = operation_local_embeddings(&fixture).remove(0);
    let baseline = arrange_mixed_periodic_face_from_embedding(graph, &evidence).unwrap();
    let selected_fragment = evidence
        .boundary_traces()
        .iter()
        .flat_map(|trace| trace.fragments())
        .map(|fragment| fragment.fragment())
        .max()
        .unwrap();

    let mut truncated = graph.clone();
    truncated.curve_fragments.truncate(selected_fragment);
    assert!(matches!(
        arrange_mixed_periodic_face_from_embedding(&truncated, &evidence),
        Err(MixedPeriodicArrangementError::UnknownFragment { fragment, .. })
            if fragment == selected_fragment
    ));

    let mut duplicated = graph.clone();
    duplicated
        .curve_fragments
        .push(duplicated.curve_fragments[selected_fragment].clone());
    assert_eq!(
        arrange_mixed_periodic_face_from_embedding(&duplicated, &evidence).unwrap(),
        baseline
    );
}
