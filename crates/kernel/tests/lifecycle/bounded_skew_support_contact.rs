//! Facade-only lifecycle evidence for an exact isolated skew support tangency.
//! Wall-time budget: less than 10 seconds as part of the `lifecycle` target.

use super::*;
use kernel::{
    BodySectionGraph, SectionCurveEndpointTopology, SectionIsolatedContactKind, SectionSite,
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
