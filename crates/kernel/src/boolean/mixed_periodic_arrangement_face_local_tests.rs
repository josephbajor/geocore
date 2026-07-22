use std::collections::BTreeSet;

use kgeom::{frame::Frame, vec::Point3};

use super::*;
use crate::{CylinderRequest, Kernel, SectionBodiesRequest};

fn parallel_cylinder_graph(first_height: f64, second_height: f64) -> BodySectionGraph {
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
    session
        .part(part)
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(first, second))
        .unwrap()
        .into_result()
        .unwrap()
}

type ArrangementShape = (usize, usize, usize, usize, usize, usize);

fn face_local_shapes(graph: &BodySectionGraph) -> Vec<ArrangementShape> {
    assert_eq!(graph.completion(), SectionCompletion::Indeterminate);
    assert_eq!(graph.curve_components().len(), 0);
    let mut shapes = Vec::new();
    for evidence in graph.periodic_face_embeddings() {
        let SectionPeriodicFaceEmbeddingEvidence::Certified(evidence) = evidence else {
            panic!("fixture lost certified periodic evidence: {evidence:#?}")
        };
        assert!(
            evidence
                .boundary_traces()
                .iter()
                .all(|trace| trace.source_component().is_none())
        );
        let arrangement = arrange_mixed_periodic_face_from_certified_embedding(
            graph,
            evidence.face(),
            evidence.operand(),
        )
        .unwrap();
        assert_eq!(
            arrange_mixed_periodic_face_from_certified_embedding(
                graph,
                evidence.face(),
                evidence.operand(),
            )
            .unwrap(),
            arrangement
        );
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
    assert_eq!(
        face_local_shapes(&parallel_cylinder_graph(2.0, 2.0)),
        vec![(4, 2, 2, 2, 0, 2), (4, 2, 2, 2, 0, 2)]
    );
    assert_eq!(
        face_local_shapes(&parallel_cylinder_graph(3.0, 2.0)),
        vec![(3, 3, 2, 3, 1, 1), (4, 2, 2, 2, 0, 2)]
    );
}

#[test]
fn altered_face_local_path_coverage_fails_closed() {
    let graph = parallel_cylinder_graph(2.0, 2.0);
    let SectionPeriodicFaceEmbeddingEvidence::Certified(evidence) =
        &graph.periodic_face_embeddings()[0]
    else {
        panic!("fixture lost certified periodic evidence")
    };
    let operand = evidence.operand();
    let face = evidence.face();

    let mut truncated = graph.clone();
    truncated.curve_fragments.pop();
    assert!(matches!(
        arrange_mixed_periodic_face_from_certified_embedding(&truncated, face.clone(), operand,),
        Err(MixedPeriodicArrangementError::UnexpectedComponentEvidence(
            _
        ))
    ));

    let mut duplicated = graph;
    duplicated
        .curve_fragments
        .push(duplicated.curve_fragments[0].clone());
    assert!(matches!(
        arrange_mixed_periodic_face_from_certified_embedding(&duplicated, face, operand),
        Err(MixedPeriodicArrangementError::FaceLocalPathUnavailable(_))
    ));
}
