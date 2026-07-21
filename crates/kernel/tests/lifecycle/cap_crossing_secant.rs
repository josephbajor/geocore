//! Facade-only lifecycle evidence for proof-keyed disk-cap chords joined to
//! bounded cylinder rulings.
//!
//! Wall-time budget: less than 30 seconds for the rigid-frame/order matrix.

use super::*;
use kernel::{
    EdgeId, FaceId, SectionBranch, SectionBranchTopology, SectionCarrier,
    SectionCurveEndpointTopology, SectionSite, SectionSourceParameterKey, SectionUvCurve,
};

const RADIUS: f64 = 1.5;
const OFFSET_X: f64 = 0.5;
const ROOT_Y: f64 = 1.414_213_562_373_095_1;
const CYLINDER_HEIGHT: f64 = 2.0;
const OUTER_X: f64 = 2.5;
const BLOCK_Y: f64 = 6.0;
const BLOCK_Z: f64 = 4.0;
const GEOMETRY_TOLERANCE: f64 = 1.0e-9;
const MESH_RELATIVE_VOLUME_TOLERANCE: f64 = 1.0e-3;

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Oblique,
}

#[derive(Debug, Clone, Copy)]
struct CapCrossingCase {
    name: &'static str,
    placement: Placement,
    swapped: bool,
}

const CASES: [CapCrossingCase; 4] = [
    CapCrossingCase {
        name: "world_prism_first",
        placement: Placement::World,
        swapped: false,
    },
    CapCrossingCase {
        name: "world_cylinder_first",
        placement: Placement::World,
        swapped: true,
    },
    CapCrossingCase {
        name: "oblique_prism_first",
        placement: Placement::Oblique,
        swapped: false,
    },
    CapCrossingCase {
        name: "oblique_cylinder_first",
        placement: Placement::Oblique,
        swapped: true,
    },
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FragmentKind {
    Chord,
    Ruling,
}

#[derive(Debug, Clone)]
struct EndpointOccurrence {
    endpoint: usize,
    kind: FragmentKind,
    point: Point3,
    source_parameter: SectionSourceParameterKey,
    edge_parameter: kernel::SectionEdgeParameterInterval,
}

type SourceSignature = ([usize; 3], [usize; 3], usize);

struct CapCrossingFixture {
    session: Session,
    part_id: PartId,
    prism: BodyId,
    cylinder: BodyId,
    frame: Frame,
    before: SourceSignature,
}

fn shared_frame(placement: Placement) -> Frame {
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

fn body_topology(part: &kernel::Part<'_>, body: BodyId) -> [usize; 3] {
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
    prism: &BodyId,
    cylinder: &BodyId,
) -> SourceSignature {
    let part = session.part(part_id.clone()).unwrap();
    (
        body_topology(&part, prism.clone()),
        body_topology(&part, cylinder.clone()),
        part.bodies().len(),
    )
}

fn cap_crossing_fixture(case: CapCrossingCase) -> CapCrossingFixture {
    let frame = shared_frame(case.placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (prism, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        // Only the prism face x = OFFSET_X meets the cylinder. Its opposite
        // x face and all four remaining faces lie strictly outside the finite
        // cylinder, so admission depends on geometry rather than face labels.
        let prism = edit
            .create_block(BlockRequest::new(
                frame.with_origin(frame.point_at(
                    0.5 * (OFFSET_X + OUTER_X),
                    0.0,
                    0.5 * CYLINDER_HEIGHT,
                )),
                [OUTER_X - OFFSET_X, BLOCK_Y, BLOCK_Z],
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(frame, RADIUS, CYLINDER_HEIGHT))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (prism, cylinder)
    };
    let before = source_signature(&session, &part_id, &prism, &cylinder);
    assert_eq!(before, ([6, 12, 8], [3, 2, 0], 2), "{}", case.name);
    CapCrossingFixture {
        session,
        part_id,
        prism,
        cylinder,
        frame,
        before,
    }
}

fn surface_class(part: &kernel::Part<'_>, face: FaceId) -> String {
    let surface = part.face(face).unwrap().surface();
    part.surface(surface)
        .unwrap()
        .class_key()
        .as_str()
        .to_owned()
}

fn branch_kind(
    part: &kernel::Part<'_>,
    branch: &SectionBranch,
    cylinder_slot: usize,
    case: CapCrossingCase,
) -> FragmentKind {
    let prism_slot = 1 - cylinder_slot;
    assert_eq!(
        surface_class(part, branch.faces()[prism_slot].clone()),
        "kernel.surface.plane.v1",
        "{}",
        case.name
    );
    match surface_class(part, branch.faces()[cylinder_slot].clone()).as_str() {
        "kernel.surface.plane.v1" => FragmentKind::Chord,
        "kernel.surface.cylinder.v1" => FragmentKind::Ruling,
        class => panic!("{}: unexpected cylinder-face surface {class}", case.name),
    }
}

fn assert_local_endpoint(point: Point3, frame: Frame, case: CapCrossingCase) {
    let local = frame.to_local(point);
    assert!(
        (local.x - OFFSET_X).abs() <= GEOMETRY_TOLERANCE,
        "{}",
        case.name
    );
    assert!(
        (local.y.abs() - ROOT_Y).abs() <= GEOMETRY_TOLERANCE,
        "{}",
        case.name
    );
    assert!(
        local.z.abs() <= GEOMETRY_TOLERANCE
            || (local.z - CYLINDER_HEIGHT).abs() <= GEOMETRY_TOLERANCE,
        "{}: endpoint escaped the finite cylinder caps: {local:?}",
        case.name
    );
}

fn assert_branch_geometry(
    branch: &SectionBranch,
    endpoints: &[kernel::SectionRulingFragmentEnd; 2],
    kind: FragmentKind,
    frame: Frame,
    case: CapCrossingCase,
) {
    assert_eq!(
        branch.topology(),
        SectionBranchTopology::Open,
        "{}",
        case.name
    );
    assert!(
        matches!(
            branch.pcurves(),
            [SectionUvCurve::Line(_), SectionUvCurve::Line(_)]
        ),
        "{}",
        case.name
    );
    let SectionCarrier::Line { origin, direction } = branch.carrier() else {
        panic!("{}: cap-crossing fragment lost its line carrier", case.name)
    };
    assert!(
        (direction.norm() - 1.0).abs() <= GEOMETRY_TOLERANCE,
        "{}",
        case.name
    );
    let locals = endpoints.each_ref().map(|end| {
        assert_local_endpoint(end.point(), frame, case);
        assert!(
            end.point()
                .dist(origin + direction * end.carrier_parameter())
                <= GEOMETRY_TOLERANCE,
            "{}",
            case.name
        );
        frame.to_local(end.point())
    });
    match kind {
        FragmentKind::Chord => {
            assert!((locals[0].z - locals[1].z).abs() <= GEOMETRY_TOLERANCE);
            assert!(locals[0].y * locals[1].y < 0.0, "{}", case.name);
        }
        FragmentKind::Ruling => {
            assert!((locals[0].y - locals[1].y).abs() <= GEOMETRY_TOLERANCE);
            assert!((locals[0].z - locals[1].z).abs() > 1.0, "{}", case.name);
        }
    }
}

fn assert_exact_endpoint_incidence(
    graph: &kernel::BodySectionGraph,
    occurrences: &[EndpointOccurrence],
    cylinder_slot: usize,
    frame: Frame,
    case: CapCrossingCase,
) {
    let prism_slot = 1 - cylinder_slot;
    let mut endpoint_keys = Vec::new();
    for (endpoint_index, endpoint) in graph.curve_endpoints().iter().enumerate() {
        let SectionCurveEndpointTopology::Trim {
            sites,
            source_parameters,
        } = endpoint.topology()
        else {
            panic!("{}: physical cap root became a parameter seam", case.name)
        };
        let SectionSite::EdgeInterior(edge) = &sites[cylinder_slot] else {
            panic!("{}: cap root lost its cylinder ring edge", case.name)
        };
        assert!(matches!(sites[prism_slot], SectionSite::FaceInterior(_)));
        let key = source_parameters[cylinder_slot]
            .as_ref()
            .unwrap_or_else(|| panic!("{}: cap root lost exact source authority", case.name));
        assert_eq!(key.edge(), edge.clone(), "{}", case.name);
        assert!(source_parameters[prism_slot].is_none(), "{}", case.name);
        assert!(endpoint.edge_parameters()[prism_slot].is_none());
        let common = endpoint.edge_parameters()[cylinder_slot]
            .expect("cap root lost its common intrinsic enclosure");

        let at_endpoint = occurrences
            .iter()
            .filter(|occurrence| occurrence.endpoint == endpoint_index)
            .collect::<Vec<_>>();
        assert_eq!(
            at_endpoint.len(),
            2,
            "{} endpoint {endpoint_index}",
            case.name
        );
        assert_ne!(at_endpoint[0].kind, at_endpoint[1].kind, "{}", case.name);
        assert_eq!(
            at_endpoint[0].source_parameter, at_endpoint[1].source_parameter,
            "{}: chord/ruling join did not share an exact root key",
            case.name
        );
        assert_eq!(&at_endpoint[0].source_parameter, key, "{}", case.name);
        assert_eq!(
            at_endpoint[0].source_parameter.root_parameter().to_bits(),
            at_endpoint[1].source_parameter.root_parameter().to_bits(),
            "{}",
            case.name
        );
        assert!(
            at_endpoint[0].point.dist(at_endpoint[1].point) <= GEOMETRY_TOLERANCE,
            "{}",
            case.name
        );
        assert_local_endpoint(at_endpoint[0].point, frame, case);
        for occurrence in at_endpoint {
            assert!(
                common.lo() >= occurrence.edge_parameter.lo()
                    && common.hi() <= occurrence.edge_parameter.hi(),
                "{}",
                case.name
            );
        }
        endpoint_keys.push(key.clone());
    }

    let mut ring_edges = Vec::<EdgeId>::new();
    for key in &endpoint_keys {
        if !ring_edges.contains(&key.edge()) {
            ring_edges.push(key.edge());
        }
    }
    assert_eq!(ring_edges.len(), 2, "{}", case.name);
    for edge in ring_edges {
        let mut ordinals = endpoint_keys
            .iter()
            .filter(|key| key.edge() == edge)
            .map(SectionSourceParameterKey::root_ordinal)
            .collect::<Vec<_>>();
        ordinals.sort_unstable();
        assert_eq!(ordinals, vec![0, 1], "{}", case.name);
    }
}

fn assert_graph(
    fixture: &CapCrossingFixture,
    graph: &kernel::BodySectionGraph,
    bodies: &[BodyId; 2],
    case: CapCrossingCase,
) {
    assert_eq!(graph.bodies(), bodies, "{}", case.name);
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{}: {:?}",
        case.name,
        graph.gaps()
    );
    assert!(graph.gaps().is_empty(), "{}", case.name);
    assert!(graph.vertices().is_empty(), "{}", case.name);
    assert!(graph.edges().is_empty(), "{}", case.name);
    assert!(graph.loops().is_empty(), "{}", case.name);
    assert!(graph.rings().is_empty(), "{}", case.name);
    assert_eq!(graph.branches().len(), 4, "{}", case.name);
    assert_eq!(graph.curve_fragments().len(), 4, "{}", case.name);
    assert_eq!(graph.curve_endpoints().len(), 4, "{}", case.name);
    assert_eq!(graph.curve_components().len(), 1, "{}", case.name);
    assert!(graph.periodic_face_embeddings().len() <= 1, "{}", case.name);

    let cylinder_slot = usize::from(!case.swapped);
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let operand_faces = bodies.each_ref().map(|body| {
        part.body(body.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>()
    });
    for branch in graph.branches() {
        for slot in 0..2 {
            assert!(
                operand_faces[slot].contains(&branch.faces()[slot]),
                "{}",
                case.name
            );
        }
    }

    let mut occurrences = Vec::new();
    let mut fragment_kinds = Vec::new();
    let mut branch_uses = vec![0usize; graph.branches().len()];
    for fragment in graph.curve_fragments() {
        let branch = &graph.branches()[fragment.branch()];
        let kind = branch_kind(&part, branch, cylinder_slot, case);
        fragment_kinds.push(kind);
        branch_uses[fragment.branch()] += 1;
        assert_eq!(fragment.source_ordinal(), 0, "{}", case.name);
        let SectionCurveFragmentSpan::LineSegment { endpoints } = fragment.span() else {
            panic!("{}: cap-crossing fragment is not a line segment", case.name)
        };
        assert_branch_geometry(branch, endpoints, kind, fixture.frame, case);
        for end in endpoints.iter() {
            let trim = end.trims()[cylinder_slot]
                .as_ref()
                .unwrap_or_else(|| panic!("{}: cap root lost cylinder trim", case.name));
            assert!(end.trims()[1 - cylinder_slot].is_none(), "{}", case.name);
            assert_eq!(trim.operand(), cylinder_slot, "{}", case.name);
            occurrences.push(EndpointOccurrence {
                endpoint: end.endpoint(),
                kind,
                point: end.point(),
                source_parameter: trim.source_parameter().clone(),
                edge_parameter: trim.edge_parameter(),
            });
        }
    }
    assert_eq!(
        fragment_kinds
            .iter()
            .filter(|&&kind| kind == FragmentKind::Chord)
            .count(),
        2,
        "{}",
        case.name
    );
    assert_eq!(
        fragment_kinds
            .iter()
            .filter(|&&kind| kind == FragmentKind::Ruling)
            .count(),
        2,
        "{}",
        case.name
    );
    assert_eq!(branch_uses, vec![1; 4], "{}", case.name);

    let component = &graph.curve_components()[0];
    assert!(component.closed(), "{}", case.name);
    assert_eq!(component.fragments().len(), 4, "{}", case.name);
    for offset in 0..4 {
        let current = component.fragments()[offset];
        let next = component.fragments()[(offset + 1) % 4];
        let SectionCurveFragmentSpan::LineSegment {
            endpoints: current_ends,
        } = graph.curve_fragments()[current].span()
        else {
            unreachable!()
        };
        let SectionCurveFragmentSpan::LineSegment {
            endpoints: next_ends,
        } = graph.curve_fragments()[next].span()
        else {
            unreachable!()
        };
        assert_eq!(
            current_ends[1].endpoint(),
            next_ends[0].endpoint(),
            "{}",
            case.name
        );
        assert_ne!(
            fragment_kinds[current], fragment_kinds[next],
            "{}",
            case.name
        );
    }
    assert_exact_endpoint_incidence(graph, &occurrences, cylinder_slot, fixture.frame, case);
}

#[test]
fn facade_closes_offset_disk_cap_chords_with_cylinder_rulings() {
    // The only active prism face is x = 1/2. On each radius-3/2 cap its
    // boundary roots are y = +/-sqrt(2), producing two chords. The same two
    // roots bound the cylinder-side rulings and must join by exact ring-edge
    // root identity into one four-fragment cycle in every rigid frame/order.
    for case in CASES {
        let fixture = cap_crossing_fixture(case);
        let bodies = if case.swapped {
            [fixture.cylinder.clone(), fixture.prism.clone()]
        } else {
            [fixture.prism.clone(), fixture.cylinder.clone()]
        };
        let request = || {
            fixture
                .session
                .part(fixture.part_id.clone())
                .unwrap()
                .section_bodies(SectionBodiesRequest::new(
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap()
        };
        let graph = request();
        assert_eq!(
            request(),
            graph,
            "{}: repeated query changed payload",
            case.name
        );
        assert_graph(&fixture, &graph, &bodies, case);
        assert_eq!(
            source_signature(
                &fixture.session,
                &fixture.part_id,
                &fixture.prism,
                &fixture.cylinder,
            ),
            fixture.before,
            "{}: section query mutated its sources",
            case.name
        );
    }
}

fn mesh_volume(positions: &[Point3], triangles: &[[u32; 3]]) -> f64 {
    let six_volume = triangles.iter().fold(0.0, |sum, triangle| {
        let first = positions[triangle[0] as usize];
        let second = positions[triangle[1] as usize];
        let third = positions[triangle[2] as usize];
        sum + first.dot(second.cross(third))
    });
    (six_volume / 6.0).abs()
}

fn cap_crossing_segment_volume() -> f64 {
    let segment_area = RADIUS * RADIUS * (OFFSET_X / RADIUS).acos()
        - OFFSET_X * (RADIUS * RADIUS - OFFSET_X * OFFSET_X).sqrt();
    segment_area * CYLINDER_HEIGHT
}

fn assert_cap_crossing_operation(
    operation: BooleanOperation,
    case: CapCrossingCase,
    expected_topology: [usize; 3],
    expected_volume: f64,
) {
    let mut fixture = cap_crossing_fixture(case);
    let bodies = if case.swapped {
        [fixture.cylinder.clone(), fixture.prism.clone()]
    } else {
        [fixture.prism.clone(), fixture.cylinder.clone()]
    };
    let outcome = fixture
        .session
        .edit_part(fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(BooleanBodiesRequest::new(
            operation,
            bodies[0].clone(),
            bodies[1].clone(),
        ))
        .unwrap()
        .into_result()
        .unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = outcome else {
        assert_eq!(
            source_signature(
                &fixture.session,
                &fixture.part_id,
                &fixture.prism,
                &fixture.cylinder,
            ),
            fixture.before,
            "{} {operation:?}: refusal mutated a source or persisted a candidate",
            case.name,
        );
        panic!("{} {operation:?} did not commit: {outcome:?}", case.name)
    };
    assert_eq!(created.bodies().len(), 1, "{} {operation:?}", case.name);
    assert_eq!(created.reports().len(), 1, "{} {operation:?}", case.name);
    assert_eq!(
        created.reports()[0].report().outcome(),
        CheckOutcome::Valid,
        "{} {operation:?}",
        case.name,
    );

    let result = created.bodies()[0].clone();
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_eq!(
        body_topology(&part, result.clone()),
        expected_topology,
        "{} {operation:?}",
        case.name,
    );
    let full = part
        .check_body(CheckBodyRequest::new(result.clone(), CheckLevel::Full))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(
        full.outcome(),
        CheckOutcome::Valid,
        "{} {operation:?}: {full:?}",
        case.name,
    );
    let mesh = part
        .tessellate_body(TessellateBodyRequest::new(
            result.clone(),
            TessOptions {
                chord_tol: 1.0e-3,
                max_edge_len: None,
            },
        ))
        .unwrap()
        .into_result()
        .unwrap();
    let actual_volume = mesh_volume(mesh.positions(), mesh.triangles());
    assert!(
        (actual_volume - expected_volume).abs() <= expected_volume * MESH_RELATIVE_VOLUME_TOLERANCE,
        "{} {operation:?}: expected volume {expected_volume:.17e}, got {actual_volume:.17e}",
        case.name,
    );
    let bytes = part
        .export_xt(ExportXtRequest::new(result))
        .unwrap()
        .into_result()
        .unwrap()
        .bytes()
        .to_vec();
    assert!(!bytes.is_empty(), "{} {operation:?}", case.name);
    drop(part);

    let after = source_signature(
        &fixture.session,
        &fixture.part_id,
        &fixture.prism,
        &fixture.cylinder,
    );
    assert_eq!(after.0, fixture.before.0, "{} {operation:?}", case.name);
    assert_eq!(after.1, fixture.before.1, "{} {operation:?}", case.name);
    assert_eq!(after.2, fixture.before.2 + 1, "{} {operation:?}", case.name);

    let repeated = fixture
        .session
        .edit_part(fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(BooleanBodiesRequest::new(
            operation,
            bodies[0].clone(),
            bodies[1].clone(),
        ))
        .unwrap()
        .into_result()
        .unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(repeated)) = repeated else {
        panic!("{} {operation:?}: repeated operation refused", case.name)
    };
    let repeated_bytes = fixture
        .session
        .part(fixture.part_id.clone())
        .unwrap()
        .export_xt(ExportXtRequest::new(repeated.bodies()[0].clone()))
        .unwrap()
        .into_result()
        .unwrap()
        .bytes()
        .to_vec();
    assert_eq!(repeated_bytes, bytes, "{} {operation:?}", case.name);
    let repeated_sources = source_signature(
        &fixture.session,
        &fixture.part_id,
        &fixture.prism,
        &fixture.cylinder,
    );
    assert_eq!(
        repeated_sources.0, fixture.before.0,
        "{} {operation:?}",
        case.name
    );
    assert_eq!(
        repeated_sources.1, fixture.before.1,
        "{} {operation:?}",
        case.name
    );
    assert_eq!(
        repeated_sources.2,
        fixture.before.2 + 2,
        "{} {operation:?}",
        case.name
    );

    let imported_part = fixture.session.create_part();
    let imported = fixture
        .session
        .edit_part(imported_part.clone())
        .unwrap()
        .import_xt(ImportXtRequest::new(&bytes))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(imported.bodies().len(), 1, "{} {operation:?}", case.name);
    let fast = fixture
        .session
        .part(imported_part)
        .unwrap()
        .check_body(CheckBodyRequest::new(
            imported.bodies()[0].clone(),
            CheckLevel::Fast,
        ))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(
        fast.outcome(),
        CheckOutcome::Valid,
        "{} {operation:?}",
        case.name
    );
}

#[test]
fn cap_crossing_union_full_commits_in_rigid_frame_and_order_matrix() {
    let intersection_volume = cap_crossing_segment_volume();
    let block_volume = (OUTER_X - OFFSET_X) * BLOCK_Y * BLOCK_Z;
    let cylinder_volume = core::f64::consts::PI * RADIUS * RADIUS * CYLINDER_HEIGHT;
    for case in CASES {
        assert_cap_crossing_operation(
            BooleanOperation::Unite,
            case,
            [9, 18, 12],
            block_volume + cylinder_volume - intersection_volume,
        );
    }
}

#[test]
fn cap_crossing_prism_minus_cylinder_full_commits_in_rigid_frame_matrix() {
    let intersection_volume = cap_crossing_segment_volume();
    let block_volume = (OUTER_X - OFFSET_X) * BLOCK_Y * BLOCK_Z;
    for case in CASES.into_iter().filter(|case| !case.swapped) {
        assert_cap_crossing_operation(
            BooleanOperation::Subtract,
            case,
            [9, 18, 12],
            block_volume - intersection_volume,
        );
    }
}

#[test]
fn cap_crossing_cylinder_minus_prism_full_commits_in_rigid_frame_matrix() {
    let intersection_volume = cap_crossing_segment_volume();
    let cylinder_volume = core::f64::consts::PI * RADIUS * RADIUS * CYLINDER_HEIGHT;
    for case in CASES.into_iter().filter(|case| case.swapped) {
        assert_cap_crossing_operation(
            BooleanOperation::Subtract,
            case,
            [4, 6, 4],
            cylinder_volume - intersection_volume,
        );
    }
}

#[test]
fn cap_crossing_intersection_full_commits_the_circular_segment_prism() {
    let expected_volume = cap_crossing_segment_volume();
    for case in CASES {
        assert_cap_crossing_operation(
            BooleanOperation::Intersect,
            case,
            [4, 6, 4],
            expected_volume,
        );
    }
}
