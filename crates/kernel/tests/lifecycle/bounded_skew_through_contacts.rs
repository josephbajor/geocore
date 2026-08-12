//! Facade-only lifecycle evidence for branch-attached skew through-contacts.
//! Wall-time budget: less than 15 seconds as part of the `lifecycle` target.

use super::*;
use kernel::{
    BodySectionGraph, SectionBranchTopology, SectionCurveEndpointTopology, SectionCurveFragmentSpan,
};

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Oblique,
}

struct Fixture {
    session: Session,
    part: PartId,
    first: BodyId,
    second: BodyId,
    frame: Frame,
}

fn shared_frame(placement: Placement) -> Frame {
    match placement {
        Placement::World => Frame::world(),
        Placement::Oblique => Frame::new(
            Point3::new(2.5, -1.75, 0.625),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap(),
    }
}

fn fixture(placement: Placement) -> Fixture {
    let frame = shared_frame(placement);
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, -3.0)),
                1.0,
                6.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame =
            Frame::new(frame.point_at(-1.0, 0.0, 0.0), frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 2.0))
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

fn assert_through_contact_oracle(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert_eq!(graph.through_contacts().len(), 4);
    assert_eq!(graph.branches().len(), 4);
    assert_eq!(graph.curve_fragments().len(), 4);
    assert_eq!(graph.curve_endpoints().len(), 4);
    let mut terminal_contacts = graph
        .curve_endpoints()
        .iter()
        .map(|endpoint| match endpoint.topology() {
            SectionCurveEndpointTopology::ThroughContact { contact } => *contact,
            topology => panic!("tangent ruling acquired a non-contact endpoint: {topology:?}"),
        })
        .collect::<Vec<_>>();
    terminal_contacts.sort_unstable();
    assert_eq!(terminal_contacts, vec![0, 1, 2, 3]);
    assert_eq!(
        graph
            .curve_fragments()
            .iter()
            .filter(|fragment| matches!(fragment.span(), SectionCurveFragmentSpan::Whole))
            .count(),
        2
    );
    assert_eq!(
        graph
            .curve_fragments()
            .iter()
            .filter(|fragment| matches!(
                fragment.span(),
                SectionCurveFragmentSpan::LineSegment { .. }
            ))
            .count(),
        2
    );
    assert_eq!(
        graph
            .curve_components()
            .iter()
            .filter(|component| component.closed())
            .count(),
        2
    );
    assert_eq!(
        graph
            .curve_components()
            .iter()
            .filter(|component| !component.closed())
            .count(),
        2
    );
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let mut expected = vec![
        fixture.frame.point_at(-1.0, 0.0, -2.0),
        fixture.frame.point_at(-1.0, 0.0, 2.0),
        fixture.frame.point_at(1.0, 0.0, -2.0),
        fixture.frame.point_at(1.0, 0.0, 2.0),
    ];
    for contact in graph.through_contacts() {
        let branch = &graph.branches()[contact.branch()];
        assert_eq!(branch.faces(), contact.faces());
        assert_eq!(branch.topology(), SectionBranchTopology::Closed);
        assert_eq!(contact.roots().len(), 1);
        let root = &contact.roots()[0];
        assert_eq!(root.face(), contact.faces()[root.operand()]);
        let index = expected
            .iter()
            .position(|point| point.dist(contact.point()) <= 1.0e-8)
            .expect("through-contact escaped the exact perpendicular-cylinder oracle");
        expected.remove(index);
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
    assert!(expected.is_empty());
}

#[test]
fn through_contact_section_is_complete_deterministic_and_transform_stable() {
    for placement in [Placement::World, Placement::Oblique] {
        let fixture = fixture(placement);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        assert_through_contact_oracle(&fixture, &forward);
        assert_through_contact_oracle(&fixture, &swapped);
        assert_eq!(
            swapped.bodies(),
            &[fixture.second.clone(), fixture.first.clone()]
        );
    }
}

#[test]
fn through_contact_boolean_retains_distinct_topology_refusal_without_mutation() {
    for placement in [Placement::World, Placement::Oblique] {
        for swapped in [false, true] {
            let mut fixture = fixture(placement);
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
