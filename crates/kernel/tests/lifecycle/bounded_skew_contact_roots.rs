//! Facade-only completion fixture for coincident finite-window skew-cylinder roots.
//! Wall-time budget: less than 10 seconds as part of the `lifecycle` target.

use super::*;
use kernel::BodySectionGraph;

const FIRST_RADIUS: f64 = 13.0;
const FIRST_LOWER: f64 = 16.0;
const FIRST_UPPER: f64 = 17.0;
const SECOND_RADIUS: f64 = 20.0;
const SECOND_LOWER: f64 = -14.0;
const SECOND_UPPER: f64 = 5.0;

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
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PartCounts {
    bodies: usize,
    regions: usize,
    shells: usize,
    faces: usize,
    loops: usize,
    fins: usize,
    edges: usize,
    vertices: usize,
    curves: usize,
    surfaces: usize,
    pcurves: usize,
}

impl PartCounts {
    fn from_fixture(fixture: &Fixture) -> Self {
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        Self {
            bodies: part.bodies().len(),
            regions: part.regions().len(),
            shells: part.shells().len(),
            faces: part.faces().len(),
            loops: part.loops().len(),
            fins: part.fins().len(),
            edges: part.edges().len(),
            vertices: part.vertices().len(),
            curves: part.curves().len(),
            surfaces: part.surfaces().len(),
            pcurves: part.pcurves().len(),
        }
    }
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
                frame.with_origin(frame.point_at(0.0, 0.0, FIRST_LOWER)),
                FIRST_RADIUS,
                FIRST_UPPER - FIRST_LOWER,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame =
            Frame::new(frame.point_at(SECOND_LOWER, 0.0, 0.0), frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(
                second_frame,
                SECOND_RADIUS,
                SECOND_UPPER - SECOND_LOWER,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    let fixture = Fixture {
        session,
        part,
        first,
        second,
    };
    assert_eq!(
        PartCounts::from_fixture(&fixture),
        PartCounts {
            bodies: 2,
            regions: 4,
            shells: 2,
            faces: 6,
            loops: 8,
            fins: 8,
            edges: 4,
            vertices: 0,
            curves: 4,
            surfaces: 6,
            pcurves: 8,
        }
    );
    fixture
}

fn section(fixture: &Fixture, swapped: bool) -> BodySectionGraph {
    let (first, second) = if swapped {
        (fixture.second.clone(), fixture.first.clone())
    } else {
        (fixture.first.clone(), fixture.second.clone())
    };
    fixture
        .session
        .part(fixture.part.clone())
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(first, second))
        .unwrap()
        .into_result()
        .unwrap()
}

fn assert_contact_corner_oracle(fixture: &Fixture, frame: Frame) {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    for y in [-12.0, 12.0] {
        let x = SECOND_UPPER;
        let z = FIRST_LOWER;
        assert_eq!(x * x + y * y, FIRST_RADIUS * FIRST_RADIUS);
        assert_eq!(y * y + z * z, SECOND_RADIUS * SECOND_RADIUS);
        assert_eq!(
            (y / (FIRST_RADIUS + x)).abs(),
            2.0 / 3.0,
            "the two coincident roots must retain half-angle magnitude 2/3"
        );
        let corner = frame.point_at(x, y, z);
        for body in [fixture.first.clone(), fixture.second.clone()] {
            let classification = part
                .classify_point_in_body(ClassifyPointInBodyRequest::new(body, corner))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(matches!(
                classification.verdict(),
                kernel::PointBodyVerdict::Boundary { .. }
            ));
        }
    }
}

#[test]
fn finite_window_contact_corner_sections_completely_read_only_in_both_orders() {
    for placement in [Placement::World, Placement::Oblique] {
        let fixture = fixture(placement);
        assert_contact_corner_oracle(&fixture, shared_frame(placement));
        let before = PartCounts::from_fixture(&fixture);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);

        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        for graph in [&forward, &swapped] {
            assert_eq!(
                graph.completion(),
                SectionCompletion::Complete,
                "{placement:?}: {:?}",
                graph.gaps()
            );
            assert!(graph.gaps().is_empty(), "{placement:?}: {:?}", graph.gaps());
            assert_eq!(graph.branches().len(), 10, "{placement:?}");
            assert_eq!(graph.curve_endpoints().len(), 8, "{placement:?}");
            assert_eq!(graph.curve_fragments().len(), 8, "{placement:?}");
            assert_eq!(graph.isolated_contacts().len(), 2, "{placement:?}");
            // Zero-dimensional contacts remain first-class members: neither
            // legacy planar vertices nor degenerate curve edges are created.
            assert!(graph.vertices().is_empty(), "{placement:?}");
            assert!(graph.edges().is_empty(), "{placement:?}");
            assert_eq!(graph.curve_components().len(), 1, "{placement:?}");
            let component = &graph.curve_components()[0];
            assert!(component.closed());
            assert_eq!(component.fragments().len(), 8);
            assert_eq!(component.isolated_contacts().len(), 2);
            assert_eq!(component.isolated_contacts(), &[0, 1]);
            for contact in graph.isolated_contacts() {
                assert_eq!(contact.roots().len(), 2);
                assert!(contact.endpoint() < graph.curve_endpoints().len());
                for operand in 0..2 {
                    assert_eq!(
                        contact
                            .roots()
                            .iter()
                            .filter(|root| root.operand() == operand)
                            .count(),
                        1
                    );
                }
                let point = contact.point();
                assert!([-12.0, 12.0].into_iter().any(|y| {
                    point.dist(shared_frame(placement).point_at(SECOND_UPPER, y, FIRST_LOWER))
                        <= 1.0e-8
                }));
            }
        }
        assert_eq!(
            swapped.bodies(),
            &[fixture.second.clone(), fixture.first.clone()]
        );
        assert_eq!(PartCounts::from_fixture(&fixture), before);
    }
}

#[test]
fn finite_window_contact_corner_boolean_still_refuses_assembly_atomically_in_both_orders() {
    for placement in [Placement::World, Placement::Oblique] {
        for swapped in [false, true] {
            let mut fixture = fixture(placement);
            let before = PartCounts::from_fixture(&fixture);
            let (first, second) = if swapped {
                (fixture.second.clone(), fixture.first.clone())
            } else {
                (fixture.first.clone(), fixture.second.clone())
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Subtract,
                    first,
                    second,
                ))
                .unwrap();
            let result = outcome.into_result().unwrap();
            assert!(
                matches!(
                    result,
                    BooleanOutcome::Refused(BooleanRefusal::AssemblyRejected)
                ),
                "{placement:?} swapped={swapped}: {result:?}"
            );
            assert_eq!(PartCounts::from_fixture(&fixture), before);
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
        }
    }
}
