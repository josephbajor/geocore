//! Facade-only lifecycle evidence for closed mixed Plane/Cylinder section cycles.
//!
//! Wall-time budget: less than 60 seconds for the table-driven matrix.

use super::*;
use kernel::{
    ClassifyPointOnFaceRequest, EdgeId, FaceId, PointFaceVerdict, SectionBranch,
    SectionBranchTopology, SectionCarrier, SectionCurveEndpointTopology, SectionCurveFragment,
    SectionCurveFragmentSpan, SectionEdgeParameterInterval, SectionSite, SectionSourceParameterKey,
    VertexId,
};

const RADIUS: f64 = 1.5;
const HALF_BLOCK_X: f64 = 1.0;
const ROOT_Y: f64 = 1.118_033_988_749_895;
const SLAB_LO: f64 = 0.5;
const SLAB_HI: f64 = 1.5;
const CYLINDER_HEIGHT: f64 = 2.0;
const GEOMETRY_TOLERANCE: f64 = 1.0e-9;

#[derive(Debug, Clone, Copy)]
struct MixedCycleCase {
    name: &'static str,
    swapped: bool,
}

const CASES: [MixedCycleCase; 2] = [
    MixedCycleCase {
        name: "world_block_first",
        swapped: false,
    },
    MixedCycleCase {
        name: "world_cylinder_first",
        swapped: true,
    },
];

#[derive(Debug, Clone, PartialEq, Eq)]
struct BodySignature {
    faces: Vec<FaceId>,
    edges: Vec<EdgeId>,
    vertices: Vec<VertexId>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct SourceSignature {
    block: BodySignature,
    cylinder: BodySignature,
    body_count: usize,
    geometry_counts: [usize; 3],
}

struct MixedCycleFixture {
    session: Session,
    part_id: PartId,
    block: BodyId,
    cylinder: BodyId,
    frame: Frame,
    before: SourceSignature,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FragmentKind {
    Arc,
    Line,
}

#[derive(Debug, Clone)]
struct EndpointOccurrence {
    endpoint: usize,
    kind: FragmentKind,
    point: Point3,
    source_parameter: SectionSourceParameterKey,
    edge_parameter: SectionEdgeParameterInterval,
}

fn body_signature(part: &kernel::Part<'_>, body: BodyId) -> BodySignature {
    let body = part.body(body).unwrap();
    BodySignature {
        faces: body.faces().unwrap().collect(),
        edges: body.edges().unwrap().collect(),
        vertices: body.vertices().unwrap().collect(),
    }
}

fn source_signature(
    session: &Session,
    part_id: &PartId,
    block: &BodyId,
    cylinder: &BodyId,
) -> SourceSignature {
    let part = session.part(part_id.clone()).unwrap();
    SourceSignature {
        block: body_signature(&part, block.clone()),
        cylinder: body_signature(&part, cylinder.clone()),
        body_count: part.bodies().len(),
        geometry_counts: [
            part.curves().len(),
            part.pcurves().len(),
            part.surfaces().len(),
        ],
    }
}

fn mixed_cycle_fixture(case: MixedCycleCase) -> MixedCycleFixture {
    let frame = Frame::world();
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        // The cavity's x faces cut four rulings. The host's top and bottom
        // faces lie strictly inside the cylinder and cut four matching arcs.
        let block = edit
            .extrude_profile(ExtrudeProfileRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, SLAB_LO)),
                vec![
                    Point2::new(-3.0, -3.0),
                    Point2::new(3.0, -3.0),
                    Point2::new(3.0, 3.0),
                    Point2::new(-3.0, 3.0),
                ],
                vec![vec![
                    Point2::new(-HALF_BLOCK_X, -2.5),
                    Point2::new(-HALF_BLOCK_X, 2.5),
                    Point2::new(HALF_BLOCK_X, 2.5),
                    Point2::new(HALF_BLOCK_X, -2.5),
                ]],
                SLAB_HI - SLAB_LO,
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
        (block, cylinder)
    };
    let before = source_signature(&session, &part_id, &block, &cylinder);
    assert_eq!(before.block.faces.len(), 10, "{}", case.name);
    assert_eq!(before.block.edges.len(), 24, "{}", case.name);
    assert_eq!(before.block.vertices.len(), 16, "{}", case.name);
    assert_eq!(before.cylinder.faces.len(), 3, "{}", case.name);
    assert_eq!(before.cylinder.edges.len(), 2, "{}", case.name);
    assert!(before.cylinder.vertices.is_empty(), "{}", case.name);
    assert_eq!(before.body_count, 2, "{}", case.name);
    MixedCycleFixture {
        session,
        part_id,
        block,
        cylinder,
        frame,
        before,
    }
}

fn assert_on_faces(part: &kernel::Part<'_>, faces: &[FaceId; 2], point: Point3, context: &str) {
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

fn fragment_kind(fragment: &SectionCurveFragment) -> FragmentKind {
    match fragment.span() {
        SectionCurveFragmentSpan::Arc { .. } => FragmentKind::Arc,
        SectionCurveFragmentSpan::LineSegment { .. } => FragmentKind::Line,
        SectionCurveFragmentSpan::Whole => {
            panic!("finite mixed-cycle fixture published a whole-period fragment")
        }
        _ => panic!("finite mixed-cycle fixture published an unknown fragment family"),
    }
}

fn fragment_endpoints(fragment: &SectionCurveFragment) -> [usize; 2] {
    match fragment.span() {
        SectionCurveFragmentSpan::Arc { endpoints, .. } => {
            endpoints.each_ref().map(|end| end.endpoint())
        }
        SectionCurveFragmentSpan::LineSegment { endpoints } => {
            endpoints.each_ref().map(|end| end.endpoint())
        }
        SectionCurveFragmentSpan::Whole => {
            panic!("finite mixed-cycle fixture published a whole-period fragment")
        }
        _ => panic!("finite mixed-cycle fixture published an unknown fragment family"),
    }
}

fn assert_analytic_endpoint(point: Point3, frame: Frame, context: &str) {
    let local = frame.to_local(point);
    assert!(
        (local.x.abs() - HALF_BLOCK_X).abs() <= GEOMETRY_TOLERANCE,
        "{context}: endpoint escaped the two in-radius block faces: {local:?}"
    );
    assert!(
        (local.y.abs() - ROOT_Y).abs() <= GEOMETRY_TOLERANCE,
        "{context}: endpoint escaped x^2 + y^2 = r^2: {local:?}"
    );
    assert!(
        (local.z - SLAB_LO).abs() <= GEOMETRY_TOLERANCE
            || (local.z - SLAB_HI).abs() <= GEOMETRY_TOLERANCE,
        "{context}: endpoint escaped the slab caps: {local:?}"
    );
    assert!(
        (local.x * local.x + local.y * local.y - RADIUS * RADIUS).abs() <= GEOMETRY_TOLERANCE,
        "{context}: endpoint escaped the authored cylinder: {local:?}"
    );
}

#[allow(clippy::too_many_arguments)]
fn assert_trim_provenance(
    part: &kernel::Part<'_>,
    graph: &kernel::BodySectionGraph,
    branch: &SectionBranch,
    endpoint: usize,
    point: Point3,
    operand: usize,
    face: FaceId,
    loop_id: kernel::LoopId,
    fin: kernel::FinId,
    source_parameter: &SectionSourceParameterKey,
    edge_parameter: SectionEdgeParameterInterval,
    block_slot: usize,
    block_edges: &[EdgeId],
    frame: Frame,
    context: &str,
) {
    assert!(endpoint < graph.curve_endpoints().len(), "{context}");
    assert_eq!(operand, block_slot, "{context}: trim escaped block slot");
    assert_eq!(face, branch.faces()[operand], "{context}");
    assert_eq!(
        part.loop_(loop_id.clone()).unwrap().face(),
        face,
        "{context}"
    );
    assert_eq!(part.fin(fin.clone()).unwrap().loop_(), loop_id, "{context}");
    assert_eq!(
        part.fin(fin).unwrap().edge(),
        source_parameter.edge(),
        "{context}"
    );
    assert!(block_edges.contains(&source_parameter.edge()), "{context}");
    assert!(
        edge_parameter.lo().is_finite()
            && edge_parameter.hi().is_finite()
            && edge_parameter.lo() < edge_parameter.hi(),
        "{context}: invalid source-edge enclosure"
    );

    let public_endpoint = &graph.curve_endpoints()[endpoint];
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = public_endpoint.topology()
    else {
        panic!("{context}: physical endpoint became a parameter seam")
    };
    assert_eq!(
        sites[block_slot],
        SectionSite::EdgeInterior(source_parameter.edge()),
        "{context}"
    );
    assert_eq!(
        source_parameters[block_slot].as_ref(),
        Some(source_parameter),
        "{context}"
    );
    assert_eq!(
        sites[1 - block_slot],
        SectionSite::FaceInterior(branch.faces()[1 - block_slot].clone()),
        "{context}"
    );
    assert!(source_parameters[1 - block_slot].is_none(), "{context}");
    assert!(
        public_endpoint.edge_parameters()[1 - block_slot].is_none(),
        "{context}"
    );
    let common = public_endpoint.edge_parameters()[block_slot]
        .expect("mixed endpoint must retain common source-edge evidence");
    assert!(
        common.lo() >= edge_parameter.lo() && common.hi() <= edge_parameter.hi(),
        "{context}: interned evidence escaped its occurrence enclosure"
    );
    assert_analytic_endpoint(point, frame, context);
    assert_on_faces(part, branch.faces(), point, context);
}

fn collect_endpoint_occurrences(
    part: &kernel::Part<'_>,
    graph: &kernel::BodySectionGraph,
    block_slot: usize,
    block_edges: &[EdgeId],
    frame: Frame,
    case: MixedCycleCase,
) -> Vec<EndpointOccurrence> {
    let mut occurrences = Vec::with_capacity(2 * graph.curve_fragments().len());
    for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
        let branch = graph
            .branches()
            .get(fragment.branch())
            .unwrap_or_else(|| panic!("{}: fragment escaped branch table", case.name));
        assert!(
            branch.range().is_finite() && branch.range().lo < branch.range().hi,
            "{}",
            case.name
        );
        let evidence = branch.evidence();
        assert!(evidence.tolerance().is_finite() && evidence.tolerance() > 0.0);
        assert!(evidence.residual_bounds().into_iter().all(|residual| {
            residual.is_finite() && residual >= 0.0 && residual <= evidence.tolerance()
        }));

        match fragment.span() {
            SectionCurveFragmentSpan::Arc { endpoints, .. } => {
                assert!(matches!(branch.carrier(), SectionCarrier::Circle { .. }));
                assert_eq!(branch.topology(), SectionBranchTopology::Closed);
                for end in endpoints.iter() {
                    let trim = end.trim();
                    let context = format!("{} arc fragment {fragment_index}", case.name);
                    assert!(
                        branch.range().contains(end.carrier_parameter()),
                        "{context}"
                    );
                    assert!(
                        trim.pcurve_half_angle().lo().is_finite()
                            && trim.pcurve_half_angle().hi().is_finite()
                            && trim.pcurve_half_angle().lo() < trim.pcurve_half_angle().hi(),
                        "{context}"
                    );
                    assert_trim_provenance(
                        part,
                        graph,
                        branch,
                        end.endpoint(),
                        end.point(),
                        trim.operand(),
                        trim.face(),
                        trim.loop_id(),
                        trim.fin(),
                        trim.source_parameter(),
                        trim.edge_parameter(),
                        block_slot,
                        block_edges,
                        frame,
                        &context,
                    );
                    occurrences.push(EndpointOccurrence {
                        endpoint: end.endpoint(),
                        kind: FragmentKind::Arc,
                        point: end.point(),
                        source_parameter: trim.source_parameter().clone(),
                        edge_parameter: trim.edge_parameter(),
                    });
                }
            }
            SectionCurveFragmentSpan::LineSegment { endpoints } => {
                assert!(matches!(branch.carrier(), SectionCarrier::Line { .. }));
                assert_eq!(branch.topology(), SectionBranchTopology::Open);
                for end in endpoints.iter() {
                    let context = format!("{} line fragment {fragment_index}", case.name);
                    assert!(
                        branch.range().contains(end.carrier_parameter()),
                        "{context}"
                    );
                    let trims = end.trims().iter().flatten().collect::<Vec<_>>();
                    assert_eq!(trims.len(), 1, "{context}");
                    let trim = trims[0];
                    let carrier = trim.carrier_parameter();
                    assert!(
                        carrier.lo().is_finite()
                            && carrier.hi().is_finite()
                            && carrier.lo() < carrier.hi()
                            && carrier.lo() <= end.carrier_parameter()
                            && end.carrier_parameter() <= carrier.hi(),
                        "{context}"
                    );
                    assert_trim_provenance(
                        part,
                        graph,
                        branch,
                        end.endpoint(),
                        end.point(),
                        trim.operand(),
                        trim.face(),
                        trim.loop_id(),
                        trim.fin(),
                        trim.source_parameter(),
                        trim.edge_parameter(),
                        block_slot,
                        block_edges,
                        frame,
                        &context,
                    );
                    occurrences.push(EndpointOccurrence {
                        endpoint: end.endpoint(),
                        kind: FragmentKind::Line,
                        point: end.point(),
                        source_parameter: trim.source_parameter().clone(),
                        edge_parameter: trim.edge_parameter(),
                    });
                }
            }
            SectionCurveFragmentSpan::Whole => {
                panic!("{}: finite fixture published a whole fragment", case.name)
            }
            _ => panic!(
                "{}: finite fixture published an unknown fragment",
                case.name
            ),
        }
    }
    occurrences
}

fn assert_components(graph: &kernel::BodySectionGraph, case: MixedCycleCase) {
    assert_eq!(graph.curve_components().len(), 2, "{}", case.name);
    let mut uses = vec![0usize; graph.curve_fragments().len()];
    for component in graph.curve_components() {
        assert!(component.closed(), "{}", case.name);
        assert_eq!(component.fragments().len(), 4, "{}", case.name);
        let mut kinds = Vec::with_capacity(component.fragments().len());
        for &fragment_index in component.fragments() {
            let fragment = graph
                .curve_fragments()
                .get(fragment_index)
                .unwrap_or_else(|| panic!("{}: unknown component fragment", case.name));
            uses[fragment_index] += 1;
            kinds.push(fragment_kind(fragment));
        }
        assert_eq!(
            kinds
                .iter()
                .filter(|&&kind| kind == FragmentKind::Arc)
                .count(),
            2,
            "{}",
            case.name
        );
        assert_eq!(
            kinds
                .iter()
                .filter(|&&kind| kind == FragmentKind::Line)
                .count(),
            2,
            "{}",
            case.name
        );
        for offset in 0..component.fragments().len() {
            let current = component.fragments()[offset];
            let next = component.fragments()[(offset + 1) % component.fragments().len()];
            assert_ne!(
                fragment_kind(&graph.curve_fragments()[current]),
                fragment_kind(&graph.curve_fragments()[next]),
                "{}",
                case.name
            );
            assert_eq!(
                fragment_endpoints(&graph.curve_fragments()[current])[1],
                fragment_endpoints(&graph.curve_fragments()[next])[0],
                "{}: component traversal is not a directed exact-endpoint cycle",
                case.name
            );
        }
    }
    assert_eq!(
        uses,
        vec![1; graph.curve_fragments().len()],
        "{}",
        case.name
    );
}

fn assert_shared_root_identity(
    graph: &kernel::BodySectionGraph,
    occurrences: &[EndpointOccurrence],
    case: MixedCycleCase,
) {
    assert_eq!(occurrences.len(), 16, "{}", case.name);
    let mut endpoint_keys = Vec::new();
    for endpoint in 0..graph.curve_endpoints().len() {
        let at_endpoint = occurrences
            .iter()
            .filter(|occurrence| occurrence.endpoint == endpoint)
            .collect::<Vec<_>>();
        assert_eq!(at_endpoint.len(), 2, "{} endpoint {endpoint}", case.name);
        assert_ne!(at_endpoint[0].kind, at_endpoint[1].kind, "{}", case.name);
        assert_eq!(
            at_endpoint[0].source_parameter, at_endpoint[1].source_parameter,
            "{}: arc/ruling join did not share exact root identity",
            case.name
        );
        assert!(
            at_endpoint[0].point.dist(at_endpoint[1].point) <= GEOMETRY_TOLERANCE,
            "{}",
            case.name
        );
        let common = graph.curve_endpoints()[endpoint]
            .edge_parameters()
            .iter()
            .flatten()
            .next()
            .expect("mixed endpoint lost its common parameter evidence");
        for occurrence in &at_endpoint {
            assert!(
                common.lo() >= occurrence.edge_parameter.lo()
                    && common.hi() <= occurrence.edge_parameter.hi(),
                "{}",
                case.name
            );
        }
        endpoint_keys.push(at_endpoint[0].source_parameter.clone());
    }
    assert_eq!(endpoint_keys.len(), 8, "{}", case.name);
    for (index, key) in endpoint_keys.iter().enumerate() {
        assert!(
            !endpoint_keys[..index].contains(key),
            "{}: distinct physical endpoints reused a root key",
            case.name
        );
    }

    let mut source_edges = Vec::new();
    for key in &endpoint_keys {
        if !source_edges.contains(&key.edge()) {
            source_edges.push(key.edge());
        }
    }
    assert_eq!(source_edges.len(), 4, "{}", case.name);
    for edge in source_edges {
        let mut ordinals = endpoint_keys
            .iter()
            .filter(|key| key.edge() == edge)
            .map(SectionSourceParameterKey::root_ordinal)
            .collect::<Vec<_>>();
        ordinals.sort_unstable();
        assert_eq!(ordinals, vec![0, 1], "{}", case.name);
    }
}

fn assert_graph_contract(
    fixture: &MixedCycleFixture,
    graph: &kernel::BodySectionGraph,
    body_a: &BodyId,
    body_b: &BodyId,
    case: MixedCycleCase,
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
        "{}: {:?}",
        case.name,
        graph.gaps()
    );
    assert!(graph.gaps().is_empty(), "{}: {:?}", case.name, graph.gaps());
    assert!(graph.edges().is_empty(), "{}", case.name);
    assert!(graph.vertices().is_empty(), "{}", case.name);
    assert!(graph.loops().is_empty(), "{}", case.name);
    assert!(graph.rings().is_empty(), "{}", case.name);
    assert_eq!(graph.branches().len(), 6, "{}", case.name);
    assert_eq!(graph.curve_fragments().len(), 8, "{}", case.name);
    assert_eq!(graph.curve_endpoints().len(), 8, "{}", case.name);
    assert_eq!(
        graph
            .curve_fragments()
            .iter()
            .filter(|fragment| fragment_kind(fragment) == FragmentKind::Arc)
            .count(),
        4,
        "{}",
        case.name
    );
    assert_eq!(
        graph
            .curve_fragments()
            .iter()
            .filter(|fragment| fragment_kind(fragment) == FragmentKind::Line)
            .count(),
        4,
        "{}",
        case.name
    );

    let block_slot = usize::from(case.swapped);
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let operand_faces = [
        part.body(body_a.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>(),
        part.body(body_b.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>(),
    ];
    for branch in graph.branches() {
        assert!(
            operand_faces[0].contains(&branch.faces()[0]),
            "{}",
            case.name
        );
        assert!(
            operand_faces[1].contains(&branch.faces()[1]),
            "{}",
            case.name
        );
    }
    assert_components(graph, case);
    let occurrences = collect_endpoint_occurrences(
        &part,
        graph,
        block_slot,
        &fixture.before.block.edges,
        fixture.frame,
        case,
    );
    assert_shared_root_identity(graph, &occurrences, case);
}

#[test]
fn facade_exposes_deterministic_closed_mixed_cycles_in_both_operand_orders() {
    // x = +/-1 cuts x^2 + y^2 = (3/2)^2 at y = +/-sqrt(5)/2. The
    // four rulings and four cap arcs therefore form two alternating cycles.
    for case in CASES {
        let fixture = mixed_cycle_fixture(case);
        let (body_a, body_b) = if case.swapped {
            (fixture.cylinder.clone(), fixture.block.clone())
        } else {
            (fixture.block.clone(), fixture.cylinder.clone())
        };
        let request = || {
            fixture
                .session
                .part(fixture.part_id.clone())
                .unwrap()
                .section_bodies(SectionBodiesRequest::new(body_a.clone(), body_b.clone()))
                .unwrap()
                .into_result()
                .unwrap()
        };
        let graph = request();
        let repeated = request();
        assert_eq!(
            repeated, graph,
            "{}: repeated public query changed its exact payload",
            case.name
        );
        assert_graph_contract(&fixture, &graph, &body_a, &body_b, case);
        assert_eq!(
            source_signature(
                &fixture.session,
                &fixture.part_id,
                &fixture.block,
                &fixture.cylinder,
            ),
            fixture.before,
            "{}: read-only section query mutated its sources",
            case.name
        );
    }
}
