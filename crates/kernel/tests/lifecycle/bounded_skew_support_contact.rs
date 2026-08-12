//! Facade-only lifecycle evidence for an exact isolated skew support tangency.
//! Wall-time budget: less than 10 seconds as part of the `lifecycle` target.

use super::*;
use kernel::{
    BodySectionGraph, SectionBranchTopology, SectionCurveEndpointTopology,
    SectionCurveFragmentSpan, SectionIsolatedContactKind, SectionPeriodicEmbeddingGap, SectionSite,
    SectionSkewCylinderAxialBoundary,
};

struct Fixture {
    session: Session,
    part: PartId,
    first: BodyId,
    second: BodyId,
    frame: Frame,
}

fn exact_frames() -> [Frame; 2] {
    [
        Frame::world(),
        Frame::new(
            Point3::new(2.0, -1.0, 3.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap(),
    ]
}

fn folded_exact_frames() -> [Frame; 2] {
    [
        Frame::world(),
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap(),
    ]
}

fn fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, -2.0)),
                1.0,
                4.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame =
            Frame::new(frame.point_at(-2.0, 3.0, 0.0), frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn boundary_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(frame, 1.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame =
            Frame::new(frame.point_at(-2.0, 3.0, 0.0), frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn corner_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(frame, 1.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame = Frame::new(frame.point_at(0.0, 3.0, 0.0), frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn folded_support_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.origin() - frame.z() * 2.25),
                1.0,
                4.5,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_axis_origin = frame.origin() + frame.y() * 3.0_f64.next_down();
        let second_frame =
            Frame::new(second_axis_origin + frame.x() * 1.25, -frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 2.5))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn section(fixture: &Fixture, swapped: bool) -> BodySectionGraph {
    let bodies = if swapped {
        [fixture.second.clone(), fixture.first.clone()]
    } else {
        [fixture.first.clone(), fixture.second.clone()]
    };
    fixture
        .session
        .part(fixture.part.clone())
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(
            bodies[0].clone(),
            bodies[1].clone(),
        ))
        .unwrap()
        .into_result()
        .unwrap()
}

fn assert_support_contact(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.branches().is_empty());
    assert!(graph.curve_fragments().is_empty());
    assert_eq!(graph.curve_endpoints().len(), 1);
    assert_eq!(graph.isolated_contacts().len(), 1);
    assert!(graph.through_contacts().is_empty());
    assert_eq!(graph.curve_components().len(), 1);
    let component = &graph.curve_components()[0];
    assert!(component.closed());
    assert!(component.fragments().is_empty());
    assert_eq!(component.isolated_contacts(), &[0]);

    let contact = &graph.isolated_contacts()[0];
    assert_eq!(contact.kind(), SectionIsolatedContactKind::SupportTangency);
    assert!(contact.roots().is_empty());
    assert_eq!(contact.endpoint(), 0);
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = graph.curve_endpoints()[0].topology()
    else {
        panic!("support tangency acquired a non-trim endpoint")
    };
    assert!(
        sites
            .iter()
            .all(|site| matches!(site, SectionSite::FaceInterior(_)))
    );
    assert!(source_parameters.iter().all(Option::is_none));
    assert!(contact.point().dist(fixture.frame.point_at(0.0, 1.0, 0.0)) <= 1.0e-12);
    assert!((contact.surface_parameters()[0][1] - 2.0).abs() <= 1.0e-12);
    assert!((contact.surface_parameters()[1][1] - 2.0).abs() <= 1.0e-12);

    let part = fixture.session.part(fixture.part.clone()).unwrap();
    for body in [fixture.first.clone(), fixture.second.clone()] {
        let classification = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body, contact.point()))
            .unwrap()
            .into_result()
            .unwrap();
        assert!(matches!(
            classification.verdict(),
            kernel::PointBodyVerdict::Boundary { .. }
        ));
    }
}

fn assert_boundary_support_contact(
    fixture: &Fixture,
    graph: &BodySectionGraph,
    boundary_operand: usize,
) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.branches().is_empty());
    assert!(graph.curve_fragments().is_empty());
    assert_eq!(graph.curve_endpoints().len(), 1);
    assert_eq!(graph.isolated_contacts().len(), 1);
    assert_eq!(graph.curve_components().len(), 1);

    let contact = &graph.isolated_contacts()[0];
    assert_eq!(contact.kind(), SectionIsolatedContactKind::SupportTangency);
    let [root] = contact.roots() else {
        panic!("boundary support contact did not retain one source root")
    };
    assert_eq!(root.operand(), boundary_operand);
    assert_eq!(
        root.axial_boundary(),
        SectionSkewCylinderAxialBoundary::Lower
    );
    assert_eq!(root.authored_bound().to_bits(), 0.0_f64.to_bits());
    assert_eq!(contact.endpoint(), 0);
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = graph.curve_endpoints()[0].topology()
    else {
        panic!("boundary support tangency acquired a non-trim endpoint")
    };
    for operand in 0..2 {
        if operand == boundary_operand {
            assert!(matches!(sites[operand], SectionSite::EdgeInterior(_)));
            assert!(source_parameters[operand].is_some());
        } else {
            assert!(matches!(sites[operand], SectionSite::FaceInterior(_)));
            assert!(source_parameters[operand].is_none());
        }
    }
    assert!(contact.point().dist(fixture.frame.point_at(0.0, 1.0, 0.0)) <= 1.0e-12);
    assert!(contact.surface_parameters()[boundary_operand][1].abs() <= 1.0e-12);
}

fn assert_corner_support_contact(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.branches().is_empty());
    assert!(graph.curve_fragments().is_empty());
    let [contact] = graph.isolated_contacts() else {
        panic!("expected one corner support contact: {graph:#?}")
    };
    assert_eq!(contact.kind(), SectionIsolatedContactKind::SupportTangency);
    assert!(contact.point().dist(fixture.frame.point_at(0.0, 1.0, 0.0)) <= 1.0e-12);
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = graph.curve_endpoints()[contact.endpoint()].topology()
    else {
        panic!("corner support tangency acquired a non-trim endpoint")
    };
    assert!(
        sites
            .iter()
            .all(|site| matches!(site, SectionSite::EdgeInterior(_)))
    );
    assert!(source_parameters.iter().all(Option::is_some));
    assert_eq!(contact.roots().len(), 2);
    assert!(contact.roots().iter().all(|root| {
        root.axial_boundary() == SectionSkewCylinderAxialBoundary::Lower
            && root.authored_bound() == 0.0
    }));
}

fn assert_folded_support_component(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert!(graph.rings().is_empty());
    assert!(graph.isolated_contacts().is_empty());
    assert!(graph.through_contacts().is_empty());
    assert_eq!(graph.branches().len(), 2);
    assert_eq!(graph.curve_fragments().len(), 2);
    assert_eq!(graph.curve_endpoints().len(), 2);
    assert_eq!(graph.curve_components().len(), 1);
    let component = &graph.curve_components()[0];
    assert!(component.closed());
    assert_eq!(component.fragments().len(), 2);
    assert!(component.isolated_contacts().is_empty());

    let mut endpoint_incidence = [0_usize; 2];
    for &fragment_index in component.fragments() {
        let fragment = &graph.curve_fragments()[fragment_index];
        assert_eq!(fragment.source_ordinal(), 0);
        let branch = &graph.branches()[fragment.branch()];
        assert_eq!(branch.topology(), SectionBranchTopology::Open);
        assert_eq!(branch.fragment_sites().len(), 2);
        assert_eq!(branch.endpoint_sites(), [0, 1]);
        assert!(branch.embedding_certificate().is_none());
        let SectionCurveFragmentSpan::FoldedSupport { endpoints } = fragment.span() else {
            panic!("folded support component retained a non-folded fragment")
        };
        assert!(endpoints[0].carrier_parameter() < endpoints[1].carrier_parameter());
        for endpoint in endpoints.iter() {
            endpoint_incidence[endpoint.endpoint()] += 1;
            let local = fixture.frame.to_local(endpoint.point());
            assert!((local.x * local.x + local.y * local.y - 1.0).abs() <= 1.0e-12);
            assert!(local.z.abs() <= 1.0e-12);
            assert!((local.y - (3.0_f64.next_down() - 2.0)).abs() <= 1.0e-12);
        }
    }
    assert_eq!(endpoint_incidence, [2, 2]);
    let mut root_ordinals = graph
        .curve_endpoints()
        .iter()
        .map(|endpoint| match endpoint.topology() {
            SectionCurveEndpointTopology::FoldedSupportJoin {
                faces,
                root_ordinal,
                root_interval,
                ..
            } => {
                assert_eq!(faces, graph.branches()[0].faces());
                assert!(root_interval.lo().is_finite());
                assert!(root_interval.hi().is_finite());
                assert!(root_interval.lo() <= root_interval.hi());
                *root_ordinal
            }
            topology => panic!("folded support acquired an unexpected endpoint: {topology:?}"),
        })
        .collect::<Vec<_>>();
    root_ordinals.sort_unstable();
    assert_eq!(root_ordinals, vec![0, 1]);
    assert_eq!(graph.periodic_face_embeddings().len(), 2);
    let mut embedding_operands = graph
        .periodic_face_embeddings()
        .iter()
        .map(|embedding| {
            assert!(
                matches!(
                embedding.gap(),
                Some(SectionPeriodicEmbeddingGap::NonLinearCylinderPcurve { fragment })
                    if *fragment < graph.curve_fragments().len()
                ),
                "{embedding:?}"
            );
            embedding.operand()
        })
        .collect::<Vec<_>>();
    embedding_operands.sort_unstable();
    assert_eq!(embedding_operands, vec![0, 1]);
}

#[test]
fn isolated_support_contact_section_is_complete_replay_swap_and_frame_stable() {
    for frame in exact_frames() {
        let fixture = fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        assert_support_contact(&fixture, &forward);
        assert_support_contact(&fixture, &swapped);
        assert_eq!(
            swapped.bodies(),
            &[fixture.second.clone(), fixture.first.clone()]
        );
    }
}

#[test]
fn isolated_support_contact_boolean_refuses_distinctly_without_mutation() {
    for frame in exact_frames() {
        for swapped in [false, true] {
            let mut fixture = fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Subtract,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(matches!(
                outcome,
                BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
            ));
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
        }
    }
}

#[test]
fn boundary_support_contact_section_is_complete_replay_swap_and_frame_stable() {
    for frame in exact_frames() {
        let fixture = boundary_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        assert_boundary_support_contact(&fixture, &forward, 0);
        assert_boundary_support_contact(&fixture, &swapped, 1);
    }
}

#[test]
fn boundary_support_contact_boolean_refuses_distinctly_without_mutation() {
    for frame in exact_frames() {
        for swapped in [false, true] {
            let mut fixture = boundary_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(matches!(
                outcome,
                BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
            ));
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            let graph = section(&fixture, swapped);
            assert_boundary_support_contact(&fixture, &graph, usize::from(swapped));
        }
    }
}

#[test]
fn corner_support_contact_section_is_complete_replay_swap_and_frame_stable() {
    for frame in exact_frames() {
        let fixture = corner_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        assert_corner_support_contact(&fixture, &forward);
        assert_corner_support_contact(&fixture, &swapped);
    }
}

#[test]
fn corner_support_contact_boolean_refuses_distinctly_without_mutation() {
    for frame in exact_frames() {
        for swapped in [false, true] {
            let mut fixture = corner_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(matches!(
                outcome,
                BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
            ));
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_corner_support_contact(&fixture, &section(&fixture, swapped));
        }
    }
}

#[test]
fn folded_support_section_is_complete_replay_swap_and_frame_stable() {
    for frame in folded_exact_frames() {
        let fixture = folded_support_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        assert_folded_support_component(&fixture, &forward);
        assert_folded_support_component(&fixture, &swapped);
    }
}

#[test]
fn folded_support_boolean_refuses_distinctly_without_mutation() {
    for frame in folded_exact_frames() {
        for swapped in [false, true] {
            let mut fixture = folded_support_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(
                matches!(
                    outcome,
                    BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
                ),
                "{outcome:?}"
            );
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_folded_support_component(&fixture, &section(&fixture, swapped));
        }
    }
}
