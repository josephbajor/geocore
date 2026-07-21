//! Facade-only lifecycle evidence for bounded Plane/Cylinder ruling carriers.
//!
//! Wall-time budget: less than 60 seconds for the table-driven matrix.

use super::*;
use kernel::{
    ClassifyPointOnFaceRequest, PointFaceVerdict, SectionBranch, SectionBranchTopology,
    SectionCarrier, SectionCurveEndpointTopology, SectionCurveFragmentSpan,
    SectionRulingFragmentEnd, SectionSite, SectionUvCurve,
};

const RADIUS: f64 = 1.25;
const HALF_BLOCK_X: f64 = 0.75;
const ROOT_Y: f64 = 1.0;
const HALF_CYLINDER_HEIGHT: f64 = 2.0;
const CARRIER_TOLERANCE: f64 = 1.0e-9;
const EXPECTED_ROOTS: [[f64; 2]; 4] = [
    [-HALF_BLOCK_X, -ROOT_Y],
    [-HALF_BLOCK_X, ROOT_Y],
    [HALF_BLOCK_X, -ROOT_Y],
    [HALF_BLOCK_X, ROOT_Y],
];

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

type SourceSignature = ([usize; 3], [usize; 3], usize);

struct RulingFixture {
    session: Session,
    part_id: PartId,
    block: BodyId,
    cylinder: BodyId,
    frame: Frame,
    before: SourceSignature,
}

#[derive(Debug, Clone)]
struct RootObservation {
    edge: kernel::EdgeId,
    opposing_face: kernel::FaceId,
    ordinal: usize,
}

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

fn source_signature(
    session: &Session,
    part_id: &PartId,
    block: &BodyId,
    cylinder: &BodyId,
) -> SourceSignature {
    let part = session.part(part_id.clone()).unwrap();
    (
        body_topology_signature(&part, block.clone()),
        body_topology_signature(&part, cylinder.clone()),
        part.bodies().len(),
    )
}

fn ruling_fixture(case: RulingCase) -> RulingFixture {
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
                frame.with_origin(frame.point_at(0.0, 0.0, -HALF_CYLINDER_HEIGHT)),
                RADIUS,
                2.0 * HALF_CYLINDER_HEIGHT,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let before = source_signature(&session, &part_id, &block, &cylinder);
    assert_eq!(before, ([6, 12, 8], [3, 2, 0], 2), "{}", case.name);
    RulingFixture {
        session,
        part_id,
        block,
        cylinder,
        frame,
        before,
    }
}

fn assert_on_faces(
    part: &kernel::Part<'_>,
    faces: &[kernel::FaceId; 2],
    point: Point3,
    context: &str,
) {
    for face in faces {
        let verdict = part
            .classify_point_on_face(ClassifyPointOnFaceRequest::new(face.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert!(
            matches!(verdict.verdict(), PointFaceVerdict::On(_)),
            "{context}: point is not on both source faces: {:?}",
            verdict.verdict()
        );
    }
}

fn line_carrier(branch: &SectionBranch, case: RulingCase) -> (Point3, Vec3) {
    match branch.carrier() {
        SectionCarrier::Line { origin, direction } => (origin, direction),
        other => panic!("{}: expected ruling line, got {other:?}", case.name),
    }
}

fn analytic_root(
    frame: Frame,
    origin: Point3,
    direction: Vec3,
    case: RulingCase,
) -> (usize, [f64; 2]) {
    EXPECTED_ROOTS
        .iter()
        .enumerate()
        .find(|(_, root)| {
            let expected = frame.point_at(root[0], root[1], 0.0);
            (expected - origin).cross(direction).norm() <= CARRIER_TOLERANCE
        })
        .map(|(index, root)| (index, *root))
        .unwrap_or_else(|| panic!("{}: ruling escaped the analytic roots", case.name))
}

fn assert_graph_shape(
    graph: &kernel::BodySectionGraph,
    case: RulingCase,
    body_a: &BodyId,
    body_b: &BodyId,
) {
    assert_eq!(
        graph.bodies(),
        &[body_a.clone(), body_b.clone()],
        "{}",
        case.name
    );
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{}",
        case.name
    );
    assert_eq!(
        graph.branches().len(),
        2 * EXPECTED_ROOTS.len(),
        "{}",
        case.name
    );
    assert_eq!(
        graph.curve_fragments().len(),
        2 * EXPECTED_ROOTS.len(),
        "{}",
        case.name
    );
    assert_eq!(graph.curve_endpoints().len(), 8, "{}", case.name);
    assert_eq!(graph.curve_components().len(), 2, "{}", case.name);
    assert!(
        graph
            .curve_components()
            .iter()
            .all(|component| component.closed() && component.fragments().len() == 4),
        "{}",
        case.name
    );
    assert!(graph.rings().is_empty(), "{}", case.name);
    assert!(graph.gaps().is_empty(), "{}", case.name);
}

fn is_ruling_branch(part: &kernel::Part<'_>, branch: &SectionBranch, case: RulingCase) -> bool {
    let cylinder_slot = usize::from(!case.swapped);
    let face = part.face(branch.faces()[cylinder_slot].clone()).unwrap();
    part.surface(face.surface()).unwrap().class_key().as_str() == "kernel.surface.cylinder.v1"
}

fn assert_branch(
    part: &kernel::Part<'_>,
    branch: &SectionBranch,
    frame: Frame,
    case: RulingCase,
) -> usize {
    let (origin, direction) = line_carrier(branch, case);
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
    assert!(
        direction.cross(frame.z()).norm() <= CARRIER_TOLERANCE,
        "{}",
        case.name
    );

    for (site, parameter) in branch.fragment_sites().iter().zip([range.lo, range.hi]) {
        assert!(
            site.point().dist(origin + direction * parameter) <= CARRIER_TOLERANCE,
            "{}: public source-window sample escaped its carrier",
            case.name
        );
        assert!(site.surface_window_boundaries().into_iter().any(|hit| hit));
        assert_on_faces(part, branch.faces(), site.point(), case.name);
    }

    let (root_index, root) = analytic_root(frame, origin, direction, case);
    let expected = frame.point_at(root[0], root[1], 0.0);
    let parameter = (expected - origin).dot(direction) / direction.norm_sq();
    assert!(range.contains(parameter), "{}", case.name);
    assert!(
        (origin + direction * parameter).dist(expected) <= CARRIER_TOLERANCE,
        "{}",
        case.name
    );

    let evidence = branch.evidence();
    assert!(evidence.tolerance().is_finite() && evidence.tolerance() > 0.0);
    assert!(evidence.residual_bounds().into_iter().all(|residual| {
        residual.is_finite() && residual >= 0.0 && residual <= evidence.tolerance()
    }));

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
        for endpoint_parameter in [range.lo, range.hi] {
            let carrier_point = origin + direction * endpoint_parameter;
            let endpoint_evaluation = part
                .evaluate_surface(SurfaceEvaluationRequest::new(
                    face.surface(),
                    [
                        uv_origin.x + uv_direction.x * endpoint_parameter,
                        uv_origin.y + uv_direction.y * endpoint_parameter,
                    ],
                    SurfaceDerivativeOrder::Position,
                ))
                .unwrap()
                .into_result()
                .unwrap();
            let roundoff = 128.0
                * f64::EPSILON
                * (1.0
                    + carrier_point
                        .norm()
                        .max(endpoint_evaluation.position().norm()));
            assert!(
                endpoint_evaluation.position().dist(carrier_point)
                    <= evidence.residual_bounds()[slot] + roundoff,
                "{}: reissued residual bound missed an expanded range endpoint",
                case.name
            );
        }
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
        assert!(evaluated.position().dist(expected) <= CARRIER_TOLERANCE);
    }
    assert_on_faces(part, branch.faces(), expected, case.name);
    root_index
}

fn assert_branches(
    part: &kernel::Part<'_>,
    graph: &kernel::BodySectionGraph,
    frame: Frame,
    case: RulingCase,
) {
    let mut seen = [false; EXPECTED_ROOTS.len()];
    for branch in graph
        .branches()
        .iter()
        .filter(|branch| is_ruling_branch(part, branch, case))
    {
        let root = assert_branch(part, branch, frame, case);
        assert!(!seen[root], "{}: duplicate analytic ruling", case.name);
        seen[root] = true;
    }
    assert!(seen.into_iter().all(|root| root), "{}", case.name);
}

fn assert_fragment_end(
    part: &kernel::Part<'_>,
    graph: &kernel::BodySectionGraph,
    branch: &SectionBranch,
    end: &SectionRulingFragmentEnd,
    frame: Frame,
    root: [f64; 2],
    case: RulingCase,
) -> RootObservation {
    let cylinder_slot = usize::from(!case.swapped);
    let block_slot = 1 - cylinder_slot;
    let (origin, direction) = line_carrier(branch, case);
    assert!(
        end.endpoint() < graph.curve_endpoints().len(),
        "{}",
        case.name
    );
    assert!(end.carrier_parameter().is_finite(), "{}", case.name);
    assert!(
        branch.range().contains(end.carrier_parameter()),
        "{}",
        case.name
    );
    assert!(
        end.point()
            .dist(origin + direction * end.carrier_parameter())
            <= CARRIER_TOLERANCE,
        "{}: ruling endpoint representative escaped its branch",
        case.name
    );
    let local = frame.to_local(end.point());
    assert!(
        (local.x - root[0]).abs() <= CARRIER_TOLERANCE,
        "{}",
        case.name
    );
    assert!(
        (local.y - root[1]).abs() <= CARRIER_TOLERANCE,
        "{}",
        case.name
    );
    assert!(
        (local.z.abs() - HALF_CYLINDER_HEIGHT).abs() <= CARRIER_TOLERANCE,
        "{}: rigid endpoint escaped a cylinder ring",
        case.name
    );

    assert_eq!(end.trims().iter().filter(|trim| trim.is_some()).count(), 1);
    assert!(end.trims()[block_slot].is_none(), "{}", case.name);
    let trim = end.trims()[cylinder_slot]
        .as_ref()
        .unwrap_or_else(|| panic!("{}: cylinder trim is missing", case.name));
    assert_eq!(trim.operand(), cylinder_slot, "{}", case.name);
    assert_eq!(trim.face(), branch.faces()[cylinder_slot], "{}", case.name);
    assert_eq!(part.loop_(trim.loop_id()).unwrap().face(), trim.face());
    assert_eq!(part.fin(trim.fin()).unwrap().loop_(), trim.loop_id());
    assert_eq!(
        part.fin(trim.fin()).unwrap().edge(),
        trim.source_parameter().edge()
    );

    let edge_interval = trim.edge_parameter();
    assert!(
        edge_interval.lo().is_finite()
            && edge_interval.hi().is_finite()
            && edge_interval.lo() <= edge_interval.hi(),
        "{}: empty source-edge enclosure",
        case.name
    );
    let carrier_interval = trim.carrier_parameter();
    let analytic_endpoint =
        frame.point_at(root[0], root[1], local.z.signum() * HALF_CYLINDER_HEIGHT);
    let analytic_parameter = (analytic_endpoint - origin).dot(direction) / direction.norm_sq();
    let analytic_distance = if analytic_parameter < carrier_interval.lo() {
        carrier_interval.lo() - analytic_parameter
    } else if analytic_parameter > carrier_interval.hi() {
        analytic_parameter - carrier_interval.hi()
    } else {
        0.0
    };
    let analytic_roundoff = 32.0 * f64::EPSILON * (1.0 + analytic_parameter.abs());
    assert!(
        carrier_interval.lo().is_finite()
            && carrier_interval.hi().is_finite()
            && carrier_interval.lo() <= end.carrier_parameter()
            && end.carrier_parameter() <= carrier_interval.hi()
            && analytic_distance <= analytic_roundoff
            && branch.range().contains(carrier_interval.lo())
            && branch.range().contains(carrier_interval.hi()),
        "{}: source-derived carrier enclosure {:?} diverged from analytic parameter {analytic_parameter:?} or escaped proof range {:?}",
        case.name,
        carrier_interval,
        branch.range()
    );

    let endpoint = &graph.curve_endpoints()[end.endpoint()];
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = endpoint.topology()
    else {
        panic!(
            "{}: physical ruling endpoint became a chart seam",
            case.name
        )
    };
    assert_eq!(
        sites[cylinder_slot],
        SectionSite::EdgeInterior(trim.source_parameter().edge())
    );
    assert_eq!(
        sites[block_slot],
        SectionSite::FaceInterior(branch.faces()[block_slot].clone())
    );
    assert_eq!(
        source_parameters[cylinder_slot].as_ref(),
        Some(trim.source_parameter())
    );
    assert!(source_parameters[block_slot].is_none(), "{}", case.name);
    let common = endpoint.edge_parameters()[cylinder_slot]
        .unwrap_or_else(|| panic!("{}: shared cap/ruling root lost its enclosure", case.name));
    assert!(
        common.lo() >= edge_interval.lo() && common.hi() <= edge_interval.hi(),
        "{}",
        case.name
    );
    assert!(
        endpoint.edge_parameters()[block_slot].is_none(),
        "{}",
        case.name
    );
    assert_on_faces(part, branch.faces(), end.point(), case.name);

    RootObservation {
        edge: trim.source_parameter().edge(),
        opposing_face: branch.faces()[block_slot].clone(),
        ordinal: trim.source_parameter().root_ordinal(),
    }
}

fn assert_fragments(
    part: &kernel::Part<'_>,
    graph: &kernel::BodySectionGraph,
    frame: Frame,
    case: RulingCase,
) -> Vec<RootObservation> {
    let mut branch_indices = Vec::new();
    let mut endpoint_indices = Vec::new();
    let mut roots = Vec::new();
    let mut saw_expanded_root_enclosure = false;
    for fragment in graph.curve_fragments() {
        let branch = graph
            .branches()
            .get(fragment.branch())
            .unwrap_or_else(|| panic!("{}: fragment escaped its branch table", case.name));
        if !is_ruling_branch(part, branch, case) {
            continue;
        }
        branch_indices.push(fragment.branch());
        assert_eq!(fragment.source_ordinal(), 0, "{}", case.name);
        let (origin, direction) = line_carrier(branch, case);
        let mut discovery = branch
            .fragment_sites()
            .iter()
            .map(|site| (site.point() - origin).dot(direction) / direction.norm_sq());
        let first_discovery = discovery.next().unwrap();
        let (discovery_lo, discovery_hi) = discovery
            .fold((first_discovery, first_discovery), |(lo, hi), parameter| {
                (lo.min(parameter), hi.max(parameter))
            });
        let (_, root) = analytic_root(frame, origin, direction, case);
        let SectionCurveFragmentSpan::LineSegment { endpoints } = fragment.span() else {
            panic!(
                "{}: topology-clipped ruling is not a LineSegment",
                case.name
            )
        };
        assert!(endpoints[0].carrier_parameter() < endpoints[1].carrier_parameter());
        for end in endpoints.iter() {
            saw_expanded_root_enclosure |= end.trims().iter().flatten().any(|trim| {
                let enclosure = trim.carrier_parameter();
                enclosure.lo() < discovery_lo || enclosure.hi() > discovery_hi
            });
            endpoint_indices.push(end.endpoint());
            roots.push(assert_fragment_end(
                part, graph, branch, end, frame, root, case,
            ));
        }
    }
    branch_indices.sort_unstable();
    branch_indices.dedup();
    assert_eq!(branch_indices.len(), EXPECTED_ROOTS.len(), "{}", case.name);
    endpoint_indices.sort_unstable();
    endpoint_indices.dedup();
    assert_eq!(
        endpoint_indices,
        (0..8).collect::<Vec<_>>(),
        "{}",
        case.name
    );
    if matches!(case.placement, Placement::Oblique) {
        assert!(
            saw_expanded_root_enclosure,
            "{}: fixture did not exercise proof-range expansion",
            case.name
        );
    }
    roots
}

fn assert_global_root_ordinals(observations: &[RootObservation], case: RulingCase) {
    let mut ring_edges = Vec::new();
    for observation in observations {
        if !ring_edges.contains(&observation.edge) {
            ring_edges.push(observation.edge.clone());
        }
    }
    assert_eq!(ring_edges.len(), 2, "{}", case.name);
    for edge in ring_edges {
        let mut edge_roots = observations
            .iter()
            .filter(|observation| observation.edge == edge)
            .map(|observation| observation.ordinal)
            .collect::<Vec<_>>();
        edge_roots.sort_unstable();
        assert_eq!(edge_roots, vec![0, 0, 1, 1], "{}", case.name);

        let mut opposing_faces = Vec::new();
        for observation in observations.iter().filter(|item| item.edge == edge) {
            if !opposing_faces.contains(&observation.opposing_face) {
                opposing_faces.push(observation.opposing_face.clone());
            }
        }
        assert_eq!(opposing_faces.len(), 2, "{}", case.name);
        for face in opposing_faces {
            let mut roots = observations
                .iter()
                .filter(|item| item.edge == edge && item.opposing_face == face)
                .map(|item| item.ordinal)
                .collect::<Vec<_>>();
            roots.sort_unstable();
            assert_eq!(roots, vec![0, 1], "{}", case.name);
        }
    }
}

#[test]
fn facade_exposes_bounded_plane_cylinder_rulings_in_shared_rigid_frames() {
    // x = +/-3/4 cuts x^2 + y^2 = (5/4)^2 at y = +/-1, so the
    // two in-range block faces independently imply exactly four rulings.
    for case in CASES {
        let fixture = ruling_fixture(case);
        let (body_a, body_b) = if case.swapped {
            (fixture.cylinder.clone(), fixture.block.clone())
        } else {
            (fixture.block.clone(), fixture.cylinder.clone())
        };
        let graph = fixture
            .session
            .part(fixture.part_id.clone())
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(body_a.clone(), body_b.clone()))
            .unwrap()
            .into_result()
            .unwrap();
        assert_graph_shape(&graph, case, &body_a, &body_b);

        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        assert_branches(&part, &graph, fixture.frame, case);
        let observations = assert_fragments(&part, &graph, fixture.frame, case);
        assert_global_root_ordinals(&observations, case);
        drop(part);

        let after = source_signature(
            &fixture.session,
            &fixture.part_id,
            &fixture.block,
            &fixture.cylinder,
        );
        assert_eq!(
            after, fixture.before,
            "{}: section mutated sources",
            case.name
        );
    }
}
