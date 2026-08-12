//! Facade-only lifecycle evidence for contacts inside bounded skew spans.
//! Wall-time budget: less than 10 seconds as part of the `lifecycle` target.

use super::*;
use kernel::{BodySectionGraph, SectionBranchTopology, SectionCurveEndpointTopology};

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
        let first_frame = Frame::new(frame.point_at(0.0, 0.0, -3.0), frame.z(), frame.y()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(first_frame, 1.0, 6.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame =
            Frame::new(frame.point_at(0.25, 0.0, 0.0), frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 0.75))
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

fn assert_clipped_contacts(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert_eq!(graph.through_contacts().len(), 2);
    let mut expected = vec![
        fixture.frame.point_at(1.0, 0.0, -2.0),
        fixture.frame.point_at(1.0, 0.0, 2.0),
    ];
    for contact in graph.through_contacts() {
        assert_eq!(
            graph.branches()[contact.branch()].topology(),
            SectionBranchTopology::Open
        );
        assert_eq!(contact.roots().len(), 1);
        let index = expected
            .iter()
            .position(|point| point.dist(contact.point()) <= 1.0e-8)
            .expect("clipped through-contact escaped the exact oracle");
        expected.remove(index);
    }
    assert!(expected.is_empty());
    let mut endpoint_contacts = graph
        .curve_endpoints()
        .iter()
        .filter_map(|endpoint| match endpoint.topology() {
            SectionCurveEndpointTopology::ThroughContact { contact } => Some(*contact),
            _ => None,
        })
        .collect::<Vec<_>>();
    endpoint_contacts.sort_unstable();
    assert_eq!(endpoint_contacts, vec![0, 1]);
}

#[test]
fn clipped_through_contacts_are_complete_replay_swap_and_frame_stable() {
    for frame in exact_frames() {
        let fixture = fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        assert_clipped_contacts(&fixture, &forward);
        assert_clipped_contacts(&fixture, &swapped);
        assert_eq!(
            swapped.bodies(),
            &[fixture.second.clone(), fixture.first.clone()]
        );
    }
}

#[test]
fn clipped_through_contact_boolean_refuses_distinctly_without_mutation() {
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
