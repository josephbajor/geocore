//! Facade-only lifecycle evidence for bounded Plane/Cylinder ruling carriers.
//!
//! Wall-time budget: less than 60 seconds for the table-driven matrix.

use super::*;
use kernel::{
    ClassifyPointOnFaceRequest, PointFaceVerdict, SectionBranchTopology, SectionCarrier,
    SectionUvCurve,
};

const RADIUS: f64 = 1.25;
const HALF_BLOCK_X: f64 = 0.75;
const ROOT_Y: f64 = 1.0;
const CARRIER_TOLERANCE: f64 = 1.0e-9;

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Oblique,
}

#[derive(Debug, Clone, Copy)]
struct RulingCase {
    name: &'static str,
    placement: Placement,
    swapped: bool,
}

const CASES: [RulingCase; 4] = [
    RulingCase {
        name: "world_block_first",
        placement: Placement::World,
        swapped: false,
    },
    RulingCase {
        name: "world_cylinder_first",
        placement: Placement::World,
        swapped: true,
    },
    RulingCase {
        name: "oblique_block_first",
        placement: Placement::Oblique,
        swapped: false,
    },
    RulingCase {
        name: "oblique_cylinder_first",
        placement: Placement::Oblique,
        swapped: true,
    },
];

fn ruling_frame(placement: Placement) -> Frame {
    match placement {
        Placement::World => Frame::world(),
        Placement::Oblique => Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap(),
    }
}

fn body_topology_signature(part: &kernel::Part<'_>, body: BodyId) -> [usize; 3] {
    let body = part.body(body).unwrap();
    [
        body.faces().unwrap().len(),
        body.edges().unwrap().len(),
        body.vertices().unwrap().len(),
    ]
}

#[test]
fn facade_exposes_bounded_plane_cylinder_rulings_in_shared_rigid_frames() {
    // x = +/-3/4 cuts x^2 + y^2 = (5/4)^2 at y = +/-1, so the
    // two in-range block faces independently imply exactly four rulings.
    let expected_roots = [
        [-HALF_BLOCK_X, -ROOT_Y],
        [-HALF_BLOCK_X, ROOT_Y],
        [HALF_BLOCK_X, -ROOT_Y],
        [HALF_BLOCK_X, ROOT_Y],
    ];

    for case in CASES {
        let frame = ruling_frame(case.placement);
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(frame, [1.5, 4.0, 6.0]))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(
                    frame.with_origin(frame.point_at(0.0, 0.0, -2.0)),
                    RADIUS,
                    4.0,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (block, cylinder)
        };
        let before = {
            let part = session.part(part_id.clone()).unwrap();
            (
                body_topology_signature(&part, block.clone()),
                body_topology_signature(&part, cylinder.clone()),
                part.bodies().len(),
            )
        };
        assert_eq!(before, ([6, 12, 8], [3, 2, 0], 2), "{}", case.name);

        let (body_a, body_b) = if case.swapped {
            (cylinder.clone(), block.clone())
        } else {
            (block.clone(), cylinder.clone())
        };
        let graph = session
            .part(part_id.clone())
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(body_a.clone(), body_b.clone()))
            .unwrap()
            .into_result()
            .unwrap();

        assert_eq!(graph.bodies(), &[body_a, body_b], "{}", case.name);
        assert_eq!(
            graph.completion(),
            SectionCompletion::Indeterminate,
            "{}: ruling topology trimming is not yet assembled",
            case.name
        );
        let ruling_gaps = graph
            .gaps()
            .iter()
            .filter(|gap| {
                gap.reason()
                    == "Plane/Cylinder ruling-line branches await topology-owned open-interval trimming"
                    && gap.faces().len() == 2
            })
            .count();
        assert_eq!(ruling_gaps, expected_roots.len(), "{}", case.name);

        let branches = graph.branches();
        assert_eq!(branches.len(), expected_roots.len(), "{}", case.name);
        let mut seen = [false; 4];
        let part = session.part(part_id.clone()).unwrap();
        for branch in branches {
            let (origin, direction) = match branch.carrier() {
                SectionCarrier::Line { origin, direction } => (origin, direction),
                other => panic!("{}: expected ruling line, got {other:?}", case.name),
            };
            assert_eq!(
                branch.topology(),
                SectionBranchTopology::Open,
                "{}",
                case.name
            );
            let range = branch.range();
            assert!(range.is_finite() && range.lo < range.hi, "{}", case.name);
            assert_eq!(branch.endpoint_sites(), [0, 1], "{}", case.name);
            assert_eq!(branch.fragment_sites().len(), 2, "{}", case.name);
            assert!(
                (direction.norm() - 1.0).abs() <= CARRIER_TOLERANCE,
                "{}",
                case.name
            );
            for (site, parameter) in branch.fragment_sites().iter().zip([range.lo, range.hi]) {
                assert!(
                    site.point().dist(origin + direction * parameter) <= CARRIER_TOLERANCE,
                    "{}: public endpoint sample escaped its carrier bound",
                    case.name
                );
                assert!(
                    site.surface_window_boundaries().into_iter().any(|hit| hit),
                    "{}: endpoint has no source-window boundary evidence",
                    case.name
                );
                for face in branch.faces() {
                    let verdict = part
                        .classify_point_on_face(ClassifyPointOnFaceRequest::new(
                            face.clone(),
                            site.point(),
                        ))
                        .unwrap()
                        .into_result()
                        .unwrap();
                    assert!(
                        matches!(verdict.verdict(), PointFaceVerdict::On(_)),
                        "{}: finite carrier endpoint is not on both source faces: {:?}",
                        case.name,
                        verdict.verdict()
                    );
                }
            }

            let root = expected_roots
                .iter()
                .enumerate()
                .find(|(_, root)| {
                    let expected = frame.point_at(root[0], root[1], 0.0);
                    (expected - origin).cross(direction).norm() <= CARRIER_TOLERANCE
                })
                .map(|(index, root)| (index, *root))
                .unwrap_or_else(|| panic!("{}: ruling escaped the analytic roots", case.name));
            assert!(!seen[root.0], "{}: duplicate analytic ruling", case.name);
            seen[root.0] = true;

            let expected = frame.point_at(root.1[0], root.1[1], 0.0);
            let parameter = (expected - origin).dot(direction) / direction.norm_sq();
            assert!(range.contains(parameter), "{}", case.name);
            assert!(
                (origin + direction * parameter).dist(expected) <= CARRIER_TOLERANCE,
                "{}",
                case.name
            );

            let evidence = branch.evidence();
            assert!(evidence.tolerance().is_finite() && evidence.tolerance() > 0.0);
            for residual in evidence.residual_bounds() {
                assert!(
                    residual.is_finite() && residual >= 0.0 && residual <= evidence.tolerance(),
                    "{}: residual {residual} exceeds {}",
                    case.name,
                    evidence.tolerance()
                );
            }

            for slot in 0..2 {
                let face = part.face(branch.faces()[slot].clone()).unwrap();
                let class = part.surface(face.surface()).unwrap().class_key().as_str();
                let expected_class = if slot == usize::from(case.swapped) {
                    "kernel.surface.plane.v1"
                } else {
                    "kernel.surface.cylinder.v1"
                };
                assert_eq!(class, expected_class, "{}: operand slot {slot}", case.name);
                let SectionUvCurve::Line(pcurve) = branch.pcurves()[slot] else {
                    panic!("{}: operand slot {slot} lacks a line pcurve", case.name)
                };
                let uv_origin = pcurve.origin();
                let uv_direction = pcurve.direction();
                let evaluated = part
                    .evaluate_surface(SurfaceEvaluationRequest::new(
                        face.surface(),
                        [
                            uv_origin.x + uv_direction.x * parameter,
                            uv_origin.y + uv_direction.y * parameter,
                        ],
                        SurfaceDerivativeOrder::Position,
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert!(
                    evaluated.position().dist(expected) <= CARRIER_TOLERANCE,
                    "{}: operand slot {slot} pcurve",
                    case.name
                );
                let verdict = part
                    .classify_point_on_face(ClassifyPointOnFaceRequest::new(
                        branch.faces()[slot].clone(),
                        expected,
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert!(
                    matches!(verdict.verdict(), PointFaceVerdict::On(_)),
                    "{}: exact boundary point is not on operand slot {slot}: {:?}",
                    case.name,
                    verdict.verdict()
                );
            }
        }
        assert!(seen.into_iter().all(|root| root), "{}", case.name);
        drop(part);

        let after = {
            let part = session.part(part_id.clone()).unwrap();
            (
                body_topology_signature(&part, block.clone()),
                body_topology_signature(&part, cylinder.clone()),
                part.bodies().len(),
            )
        };
        assert_eq!(
            after, before,
            "{}: read-only section mutated sources",
            case.name
        );
    }
}
