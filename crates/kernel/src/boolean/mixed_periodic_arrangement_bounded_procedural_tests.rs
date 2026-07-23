//! Focused Boolean-adapter evidence for bounded procedural Section fragments.

use std::collections::BTreeSet;

use kgeom::{frame::Frame, vec::Point3};

use super::*;
use crate::{
    BlockRequest, CylinderRequest, Kernel, SectionBodiesRequest, SectionCurveFragmentSpan,
};

fn bounded_skew_graph(swapped: bool) -> BodySectionGraph {
    let frame = Frame::world();
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(Point3::new(0.0, 0.0, 1.8)),
                1.0,
                0.1,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame = Frame::new(Point3::new(-1.25, 0.0, 0.0), frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 2.5))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    let bodies = if swapped {
        [second, first]
    } else {
        [first, second]
    };
    session
        .part(part)
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(
            bodies[0].clone(),
            bodies[1].clone(),
        ))
        .unwrap()
        .into_result()
        .unwrap()
}

fn procedural_endpoint_signature(graph: &BodySectionGraph) -> Vec<(usize, [usize; 2])> {
    graph
        .curve_fragments()
        .iter()
        .enumerate()
        .filter_map(|(fragment_index, fragment)| {
            matches!(
                fragment.span(),
                SectionCurveFragmentSpan::BoundedProcedural { .. }
            )
            .then(|| {
                (
                    fragment_index,
                    fragment_endpoints(fragment_index, fragment).unwrap(),
                )
            })
        })
        .collect()
}

#[test]
fn bounded_procedural_endpoints_are_exact_replay_stable_adapter_keys() {
    for swapped in [false, true] {
        let graph = bounded_skew_graph(swapped);
        let replay = bounded_skew_graph(swapped);
        let signature = procedural_endpoint_signature(&graph);
        assert_eq!(signature, procedural_endpoint_signature(&replay));
        assert_eq!(signature.len(), 4);
        let endpoints = signature
            .iter()
            .flat_map(|(_, endpoints)| *endpoints)
            .collect::<BTreeSet<_>>();
        assert_eq!(endpoints, (0..8).collect());
    }
}

#[test]
fn face_local_path_discovery_retains_a_bounded_procedural_fragment() {
    let mut graph = bounded_skew_graph(false);
    let fragment = graph
        .curve_fragments()
        .iter()
        .find(|fragment| {
            matches!(
                fragment.span(),
                SectionCurveFragmentSpan::BoundedProcedural { .. }
            )
        })
        .unwrap()
        .clone();
    graph.curve_fragments = vec![fragment];
    graph.curve_components.clear();

    let paths = collect_unstitched_fragment_paths(&graph);
    assert_eq!(paths.paths, vec![vec![0]]);
    assert_eq!(paths.assigned, vec![true]);
}

#[test]
fn uncertified_procedural_embeddings_still_fail_closed() {
    for swapped in [false, true] {
        let graph = bounded_skew_graph(swapped);
        assert_eq!(graph.completion(), SectionCompletion::Complete);
        assert_eq!(graph.periodic_face_embeddings().len(), 2);
        for evidence in graph.periodic_face_embeddings() {
            let gap = evidence
                .gap()
                .expect("Section has not yet certified nonlinear periodic embedding")
                .clone();
            assert!(matches!(
                &gap,
                SectionPeriodicEmbeddingGap::NonLinearCylinderPcurve { .. }
            ));
            assert_eq!(
                arrange_mixed_periodic_face(&graph, evidence.face(), evidence.operand()),
                Err(MixedPeriodicArrangementError::EmbeddingIndeterminate(gap))
            );
        }
    }
}

#[test]
fn certified_embedding_endpoint_identity_is_checked_before_arrangement() {
    let frame = Frame::world();
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let block = edit
            .create_block(BlockRequest::new(
                frame.with_origin(Point3::new(0.0, 0.0, 1.0)),
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
    let mut graph = session
        .part(part)
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(block, cylinder))
        .unwrap()
        .into_result()
        .unwrap();
    let [SectionPeriodicFaceEmbeddingEvidence::Certified(evidence)] =
        graph.periodic_face_embeddings()
    else {
        panic!("fixture must retain one certified periodic embedding")
    };
    let operand = evidence.operand();
    let face = evidence.face();
    let fragments = evidence.components()[0].fragments();
    let [first, second, ..] = fragments else {
        panic!("fixture component must retain multiple fragments")
    };
    let (first, second) = (first.fragment(), second.fragment());
    graph.curve_fragments.swap(first, second);

    assert!(matches!(
        arrange_mixed_periodic_face(&graph, face, operand),
        Err(MixedPeriodicArrangementError::FragmentEmbeddingEndpointMismatch {
            fragment,
            ..
        }) if fragment == first
    ));
}
