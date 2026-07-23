//! End-to-end refusal tests for malformed nonlinear proof attachment.

use kgeom::{
    frame::Frame,
    vec::{Point3, Vec3},
};

use super::*;
use crate::{CylinderRequest, Kernel, SectionBodiesRequest};

fn bounded_skew_graph() -> BodySectionGraph {
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
    session
        .part(part)
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(first, second))
        .unwrap()
        .into_result()
        .unwrap()
}

#[test]
fn missing_sealed_procedural_certificate_fails_closed() {
    let mut graph = bounded_skew_graph();
    let fragment_index = graph
        .curve_fragments
        .iter()
        .position(|fragment| {
            matches!(
                fragment.span(),
                SectionCurveFragmentSpan::BoundedProcedural { .. }
            )
        })
        .unwrap();
    let branch_index = graph.curve_fragments[fragment_index].branch();
    graph.branches[branch_index].skew_cylinder_embedding = None;

    assert!(matches!(
        procedural::ProceduralFragmentProof::certify(
            fragment_index,
            &graph.curve_fragments[fragment_index],
            &graph.branches[branch_index],
            0,
        ),
        Err(SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceUnavailable {
            fragment
        }) if fragment == fragment_index
    ));
}

#[test]
fn attached_certificate_with_mismatched_carrier_fails_closed() {
    let mut graph = bounded_skew_graph();
    let fragment_index = graph
        .curve_fragments
        .iter()
        .position(|fragment| {
            matches!(
                fragment.span(),
                SectionCurveFragmentSpan::BoundedProcedural { .. }
            )
        })
        .unwrap();
    let branch_index = graph.curve_fragments[fragment_index].branch();
    assert!(
        graph.branches[branch_index]
            .embedding_certificate()
            .is_some()
    );
    graph.branches[branch_index].carrier = SectionCarrier::Line {
        origin: Point3::new(0.0, 0.0, 0.0),
        direction: Vec3::new(1.0, 0.0, 0.0),
    };

    assert!(matches!(
        procedural::ProceduralFragmentProof::certify(
            fragment_index,
            &graph.curve_fragments[fragment_index],
            &graph.branches[branch_index],
            0,
        ),
        Err(SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
            fragment
        }) if fragment == fragment_index
    ));
}
