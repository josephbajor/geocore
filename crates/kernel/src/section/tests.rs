//! End-to-end section-graph tests: independent oracles, adversarial
//! configurations, and determinism.
//!
//! The oracle contract (rung 2's external validity): every certified section
//! edge is sampled at five deterministic strictly-interior parameters (1/7,
//! 2/7, 3/7, 4/7, 5/7 of its carrier range) and every sample — plus every
//! graph vertex representative — must classify as `Boundary` on BOTH operand
//! bodies through the rung-1 `classify_point_in_body` oracle, which was
//! itself certified against exact parity witnesses. Structural exact
//! invariants are verified independently by walking endpoint indices, never
//! by trusting the implementation's own bookkeeping: in a `Complete` graph
//! every referenced vertex exists, every vertex has degree exactly two,
//! every loop is closed and chains through shared vertices, every edge lies
//! in exactly one loop, and `gaps()` is empty. Known block/block
//! configurations additionally pin exact hand-derived vertex coordinates,
//! edge counts, and loop counts; degenerate contacts (coincident faces,
//! shared edges, plane-through-vertex) must be certified or honestly
//! refused with stable gap reasons, never fabricated.
//!
//! R7 lane note: these are lib-lane unit tests over tiny analytic models;
//! wall time is trivial (fractions of a second for the whole module).

use std::collections::BTreeSet;

use kgeom::frame::Frame;
use ktopo::geom::CurveGeom;

use super::*;
use crate::classify::{ClassifyPointInBodyRequest, PointBodyVerdict};
use crate::operation::{BlockRequest, ExtrudeProfileRequest};
use crate::{CylinderRequest, Kernel, PartEdit, PartId, Session};

/// Every stable gap-reason constant the section slice may report. Any other
/// string is a contract violation.
const STABLE_GAP_REASONS: [&str; 21] = [
    GAP_PLANAR_ONLY,
    GAP_LINE_EDGES_ONLY,
    GAP_BOUNDED_EDGES_ONLY,
    GAP_NO_LOOPS,
    GAP_SHORT_LOOP,
    GAP_COINCIDENT_FACE_PAIR,
    GAP_TANGENT_CONTACT,
    GAP_UNORDERED_CROSSINGS,
    GAP_DEGENERATE_VERTEX,
    GAP_OPEN_CHAIN,
    GAP_CARRIER_ORIENTATION,
    GAP_PAIR_UNRESOLVED,
    GAP_INCOMPATIBLE_EDGE_PARAMETERS,
    GAP_CURVED_TRIM_UNRESOLVED,
    GAP_CLOSED_STITCH,
    curved_clip::ClosedConicClipGap::UnsupportedTrim.reason(),
    curved_clip::ClosedConicClipGap::MalformedTrim.reason(),
    curved_clip::ClosedConicClipGap::ArithmeticGuard.reason(),
    curved_clip::ClosedConicClipGap::TangentialContact.reason(),
    curved_clip::ClosedConicClipGap::CoincidentBoundary.reason(),
    curved_clip::ClosedConicClipGap::UnorderedCrossings.reason(),
];

/// Exact-coordinate agreement tolerance for hand-derived expected points.
/// Every configuration uses dyadic coordinates, so certified results must
/// land exactly; 1e-12 only absorbs benign last-ulp arithmetic.
const POINT_MATCH_TOL: f64 = 1e-12;

// ---------------------------------------------------------------------------
// Construction helpers
// ---------------------------------------------------------------------------

fn frame_at(origin: [f64; 3]) -> Frame {
    Frame::world().with_origin(Point3::new(origin[0], origin[1], origin[2]))
}

/// Create one checked block centered at `center` with the given extents
/// (blocks are centered on their placement frame origin).
fn create_block_in(edit: &mut PartEdit<'_>, center: [f64; 3], extents: [f64; 3]) -> BodyId {
    edit.create_block(BlockRequest::new(frame_at(center), extents))
        .unwrap()
        .into_result()
        .unwrap()
        .body()
}

fn block_pair(
    a_center: [f64; 3],
    a_extents: [f64; 3],
    b_center: [f64; 3],
    b_extents: [f64; 3],
) -> (Session, PartId, BodyId, BodyId) {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (a, b) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let a = create_block_in(&mut edit, a_center, a_extents);
        let b = create_block_in(&mut edit, b_center, b_extents);
        (a, b)
    };
    (session, part_id, a, b)
}

fn section_graph(
    session: &Session,
    part_id: &PartId,
    body_a: &BodyId,
    body_b: &BodyId,
) -> BodySectionGraph {
    session
        .part(part_id.clone())
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(body_a.clone(), body_b.clone()))
        .unwrap()
        .into_result()
        .unwrap()
}

// ---------------------------------------------------------------------------
// Oracle contract: rung-1 classification is the independent referee
// ---------------------------------------------------------------------------

fn body_verdict(
    session: &Session,
    part_id: &PartId,
    body: &BodyId,
    point: Point3,
) -> PointBodyVerdict {
    session
        .part(part_id.clone())
        .unwrap()
        .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
        .unwrap()
        .into_result()
        .unwrap()
        .verdict()
        .clone()
}

fn assert_boundary_on_both(
    session: &Session,
    part_id: &PartId,
    bodies: &[BodyId; 2],
    point: Point3,
    context: &str,
) {
    for (slot, body) in bodies.iter().enumerate() {
        let verdict = body_verdict(session, part_id, body, point);
        assert!(
            matches!(verdict, PointBodyVerdict::Boundary { .. }),
            "{context}: point {point:?} must classify Boundary on operand {slot}, got {verdict:?}"
        );
    }
}

/// Five deterministic strictly-interior carrier parameters (k/7 fractions).
fn sample_params(edge: &SectionEdge) -> [f64; 5] {
    let range = edge.range();
    core::array::from_fn(|i| range.lo + range.width() * ((i + 1) as f64 / 7.0))
}

/// The oracle contract over every certified vertex and edge of a returned
/// graph, regardless of completion state: returned evidence must be genuine
/// boundary/boundary intersection, endpoints must reference existing
/// vertices, and each edge's carrier evaluation must agree with its endpoint
/// vertex representatives.
fn assert_certified_evidence(session: &Session, part_id: &PartId, graph: &BodySectionGraph) {
    let bodies = graph.bodies();

    for (index, vertex) in graph.vertices().iter().enumerate() {
        assert_boundary_on_both(
            session,
            part_id,
            bodies,
            vertex.point(),
            &format!("vertex {index}"),
        );
    }

    for (index, edge) in graph.edges().iter().enumerate() {
        let range = edge.range();
        assert!(
            range.is_finite() && range.lo < range.hi,
            "edge {index}: carrier range must be finite and non-degenerate, got {range:?}"
        );
        for bound in edge.residual_bounds() {
            assert!(
                bound.is_finite() && bound >= 0.0,
                "edge {index}: residual bound must be a finite non-negative certificate, got {bound}"
            );
        }
        for (end, &endpoint) in edge.endpoints().iter().enumerate() {
            assert!(
                endpoint < graph.vertices().len(),
                "edge {index}: endpoint {end} references missing vertex {endpoint}"
            );
            let param = if end == 0 { range.lo } else { range.hi };
            let on_carrier = edge.origin() + edge.direction() * param;
            let representative = graph.vertices()[endpoint].point();
            assert!(
                on_carrier.dist(representative) <= 1e-6,
                "edge {index}: endpoint {end} vertex {representative:?} does not sit at the \
                 carrier range end {on_carrier:?}"
            );
        }
        for t in sample_params(edge) {
            let point = edge.origin() + edge.direction() * t;
            assert_boundary_on_both(
                session,
                part_id,
                bodies,
                point,
                &format!("edge {index} sample t={t}"),
            );
        }
    }
}

/// Independent source-curve oracle for the edge parameter evidence exposed
/// on section vertices. Straight source edges are arc-length parameterized,
/// so projecting the returned point onto the stored line yields the known
/// intrinsic parameter regardless of either incident fin's traversal sense.
fn assert_source_edge_parameters(session: &Session, part_id: &PartId, graph: &BodySectionGraph) {
    let part = session.part(part_id.clone()).unwrap();
    let store = &part.state.store;
    for (vertex_index, vertex) in graph.vertices().iter().enumerate() {
        for operand in 0..2 {
            match &vertex.sites()[operand] {
                SectionSite::EdgeInterior(edge_id) => {
                    let evidence = vertex.edge_parameters()[operand].unwrap_or_else(|| {
                        panic!(
                            "vertex {vertex_index} operand {operand} edge site lacks parameter evidence"
                        )
                    });
                    let edge = store.get(edge_id.raw()).unwrap();
                    let curve = store.get(edge.curve.unwrap()).unwrap();
                    let CurveGeom::Line(line) = curve else {
                        panic!("certified planar slice source edge must be a line");
                    };
                    let expected = (vertex.point() - line.origin()).dot(line.dir());
                    assert!(
                        evidence.contains(expected),
                        "vertex {vertex_index} operand {operand}: independently projected source \
                         parameter {expected} lies outside [{}, {}]",
                        evidence.lo(),
                        evidence.hi()
                    );
                    assert!(
                        evidence.hi() - evidence.lo() < 1e-9,
                        "vertex {vertex_index} operand {operand}: source parameter enclosure is \
                         unexpectedly wide: [{}, {}]",
                        evidence.lo(),
                        evidence.hi()
                    );
                }
                SectionSite::FaceInterior(_) | SectionSite::AtVertex(_) => assert!(
                    vertex.edge_parameters()[operand].is_none(),
                    "vertex {vertex_index} operand {operand}: non-edge site carried edge parameter evidence"
                ),
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Structural exact invariants
// ---------------------------------------------------------------------------

/// True if the loop's edges chain through shared endpoint vertices and close
/// back onto the starting vertex (both traversal orientations attempted).
fn loop_closes(edges: &[SectionEdge], lp: &SectionLoop) -> bool {
    let indices = lp.edges();
    if indices.is_empty() {
        return false;
    }
    'orientation: for flip in [false, true] {
        let [lo, hi] = edges[indices[0]].endpoints();
        let (start, mut cursor) = if flip { (hi, lo) } else { (lo, hi) };
        for &edge_index in &indices[1..] {
            let [a, b] = edges[edge_index].endpoints();
            cursor = if a == cursor {
                b
            } else if b == cursor {
                a
            } else {
                continue 'orientation;
            };
        }
        if cursor == start {
            return true;
        }
    }
    false
}

/// Exact invariants a `Complete` graph must satisfy: no gaps, every vertex
/// degree exactly two (endpoint incidences counted here, not taken from the
/// implementation), every loop closed and chained, every edge in exactly one
/// loop.
fn assert_complete_section_invariants(graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "complete graph gaps: {:?}",
        graph.gaps()
    );
    assert!(
        graph.gaps().is_empty(),
        "a Complete graph must report no gaps, got {:?}",
        graph.gaps()
    );

    let mut degree = vec![0usize; graph.vertices().len()];
    for (index, edge) in graph.edges().iter().enumerate() {
        for endpoint in edge.endpoints() {
            assert!(
                endpoint < degree.len(),
                "edge {index} references missing vertex {endpoint}"
            );
            degree[endpoint] += 1;
        }
    }
    assert!(
        degree.iter().all(|&d| d == 2),
        "every vertex of a Complete graph must have degree exactly 2, got {degree:?}"
    );

    let mut edge_uses = vec![0usize; graph.edges().len()];
    for (loop_index, lp) in graph.loops().iter().enumerate() {
        assert!(
            lp.closed(),
            "loop {loop_index} of a Complete graph must be closed"
        );
        assert!(
            loop_closes(graph.edges(), lp),
            "loop {loop_index} must chain consecutive edges through shared vertices and close"
        );
        for &edge_index in lp.edges() {
            assert!(
                edge_index < edge_uses.len(),
                "loop {loop_index} references missing edge {edge_index}"
            );
            edge_uses[edge_index] += 1;
        }
    }
    assert!(
        edge_uses.iter().all(|&uses| uses == 1),
        "every edge of a Complete graph must lie in exactly one loop, got uses {edge_uses:?}"
    );
}

fn assert_stable_gap_reasons(graph: &BodySectionGraph) {
    for gap in graph.gaps() {
        assert!(
            STABLE_GAP_REASONS.contains(&gap.reason()),
            "gap reason is not one of the stable constants: {:?}",
            gap.reason()
        );
    }
}

// ---------------------------------------------------------------------------
// Point-set matching against hand-derived expectations
// ---------------------------------------------------------------------------

fn close3(a: Point3, b: Point3) -> bool {
    (a.x - b.x).abs() <= POINT_MATCH_TOL
        && (a.y - b.y).abs() <= POINT_MATCH_TOL
        && (a.z - b.z).abs() <= POINT_MATCH_TOL
}

/// Order-independent bijective match between actual and expected points.
fn point_set_matches(actual: &[Point3], expected: &[Point3]) -> bool {
    if actual.len() != expected.len() {
        return false;
    }
    let mut used = vec![false; actual.len()];
    'expected: for want in expected {
        for (index, got) in actual.iter().enumerate() {
            if !used[index] && close3(*got, *want) {
                used[index] = true;
                continue 'expected;
            }
        }
        return false;
    }
    true
}

fn assert_point_set(actual: &[Point3], expected: &[Point3], context: &str) {
    assert!(
        point_set_matches(actual, expected),
        "{context}: point sets differ\n  got:  {actual:?}\n  want: {expected:?}"
    );
}

fn ring_points(ring: &[[f64; 3]]) -> Vec<Point3> {
    ring.iter().map(|p| Point3::new(p[0], p[1], p[2])).collect()
}

fn graph_vertex_points(graph: &BodySectionGraph) -> Vec<Point3> {
    graph.vertices().iter().map(SectionVertex::point).collect()
}

/// Distinct vertex representatives referenced by one loop's edges.
fn loop_vertex_points(graph: &BodySectionGraph, lp: &SectionLoop) -> Vec<Point3> {
    let indices: BTreeSet<usize> = lp
        .edges()
        .iter()
        .flat_map(|&edge_index| graph.edges()[edge_index].endpoints())
        .collect();
    indices
        .into_iter()
        .map(|vertex_index| graph.vertices()[vertex_index].point())
        .collect()
}

// ---------------------------------------------------------------------------
// Table-driven runner for certified-Complete block/block configurations
// ---------------------------------------------------------------------------

struct CompleteBlockCase {
    label: &'static str,
    a_center: [f64; 3],
    a_extents: [f64; 3],
    b_center: [f64; 3],
    b_extents: [f64; 3],
    /// Expected loops, each as its hand-derived vertex coordinate set.
    expected_loops: &'static [&'static [[f64; 3]]],
    expected_edges: usize,
}

fn run_complete_block_case(
    case: &CompleteBlockCase,
) -> (Session, PartId, BodyId, BodyId, BodySectionGraph) {
    let (session, part_id, a, b) =
        block_pair(case.a_center, case.a_extents, case.b_center, case.b_extents);
    let graph = section_graph(&session, &part_id, &a, &b);

    assert_eq!(graph.bodies(), &[a.clone(), b.clone()], "{}", case.label);
    assert_complete_section_invariants(&graph);
    assert_certified_evidence(&session, &part_id, &graph);
    assert_source_edge_parameters(&session, &part_id, &graph);

    assert_eq!(
        graph.edges().len(),
        case.expected_edges,
        "{}: edge count",
        case.label
    );
    assert_eq!(
        graph.loops().len(),
        case.expected_loops.len(),
        "{}: loop count",
        case.label
    );
    let expected_total: usize = case.expected_loops.iter().map(|ring| ring.len()).sum();
    assert_eq!(
        graph.vertices().len(),
        expected_total,
        "{}: vertex count",
        case.label
    );

    let expected_all: Vec<Point3> = case
        .expected_loops
        .iter()
        .flat_map(|ring| ring_points(ring))
        .collect();
    assert_point_set(&graph_vertex_points(&graph), &expected_all, case.label);

    // Bijection between returned loops and expected rings, matched as sets.
    let mut ring_used = vec![false; case.expected_loops.len()];
    for (loop_index, lp) in graph.loops().iter().enumerate() {
        let points = loop_vertex_points(&graph, lp);
        let matched = case
            .expected_loops
            .iter()
            .enumerate()
            .find(|(ring_index, ring)| {
                !ring_used[*ring_index] && point_set_matches(&points, &ring_points(ring))
            })
            .map(|(ring_index, ring)| (ring_index, ring.len()));
        let Some((ring_index, ring_len)) = matched else {
            panic!(
                "{}: loop {loop_index} matches no unused expected ring; got {points:?}",
                case.label
            );
        };
        ring_used[ring_index] = true;
        assert_eq!(
            lp.edges().len(),
            ring_len,
            "{}: loop {loop_index} edge count",
            case.label
        );
    }

    (session, part_id, a, b, graph)
}

// ---------------------------------------------------------------------------
// Shared corner-overlap configuration (also used by the determinism, swap,
// and budget tests)
// ---------------------------------------------------------------------------

// Blocks are centered on their frame origin, so A = [0,1]^3 places its
// frame at (0.5, 0.5, 0.5) and B = [0.5,1.5]^3 places its frame at (1,1,1).
const CORNER_A_CENTER: [f64; 3] = [0.5, 0.5, 0.5];
const CORNER_B_CENTER: [f64; 3] = [1.0, 1.0, 1.0];
const UNIT_EXTENTS: [f64; 3] = [1.0, 1.0, 1.0];

// Hand geometry for A = [0,1]^3 vs B = [0.5,1.5]^3. Only A's max faces
// (x=1, y=1, z=1) can meet ∂B and only B's min faces (x=0.5, y=0.5, z=0.5)
// can meet ∂A; same-axis pairs are parallel, so the section is the six
// cross-axis segments
//   A x=1 ∩ B y=0.5: {x=1, y=0.5, z ∈ [0.5, 1]}
//   A x=1 ∩ B z=0.5: {x=1, z=0.5, y ∈ [0.5, 1]}
//   A y=1 ∩ B z=0.5: {y=1, z=0.5, x ∈ [0.5, 1]}
//   A y=1 ∩ B x=0.5: {y=1, x=0.5, z ∈ [0.5, 1]}
//   A z=1 ∩ B x=0.5: {z=1, x=0.5, y ∈ [0.5, 1]}
//   A z=1 ∩ B y=0.5: {z=1, y=0.5, x ∈ [0.5, 1]}
// whose pairwise shared endpoints close the hexagon
//   (1,0.5,0.5) → (1,1,0.5) → (0.5,1,0.5) → (0.5,1,1) → (0.5,0.5,1)
//   → (1,0.5,1) → back.
const CORNER_HEXAGON: [[f64; 3]; 6] = [
    [1.0, 0.5, 0.5], // A face x=1 ∩ B edge (y=0.5 ∧ z=0.5)
    [1.0, 1.0, 0.5], // A edge (x=1 ∧ y=1) ∩ B face z=0.5
    [0.5, 1.0, 0.5], // A face y=1 ∩ B edge (x=0.5 ∧ z=0.5)
    [0.5, 1.0, 1.0], // A edge (y=1 ∧ z=1) ∩ B face x=0.5
    [0.5, 0.5, 1.0], // A face z=1 ∩ B edge (x=0.5 ∧ y=0.5)
    [1.0, 0.5, 1.0], // A edge (x=1 ∧ z=1) ∩ B face y=0.5
];

fn corner_overlap_case() -> CompleteBlockCase {
    CompleteBlockCase {
        label: "corner overlap A=[0,1]^3 B=[0.5,1.5]^3",
        a_center: CORNER_A_CENTER,
        a_extents: UNIT_EXTENTS,
        b_center: CORNER_B_CENTER,
        b_extents: UNIT_EXTENTS,
        expected_loops: &[&CORNER_HEXAGON],
        expected_edges: 6,
    }
}

// ---------------------------------------------------------------------------
// 1. Corner overlap: closed hexagonal loop through six face pairs
// ---------------------------------------------------------------------------

#[test]
fn corner_overlap_hexagonal_section() {
    let (_session, _part_id, _a, _b, graph) = run_complete_block_case(&corner_overlap_case());

    // Every hexagon vertex is a transversal crossing of one operand's edge
    // through the other operand's face interior: sites must pair
    // FaceInterior with EdgeInterior (one per operand slot, either order).
    for (index, vertex) in graph.vertices().iter().enumerate() {
        let sites = vertex.sites();
        let face_then_edge = matches!(sites[0], SectionSite::FaceInterior(_))
            && matches!(sites[1], SectionSite::EdgeInterior(_));
        let edge_then_face = matches!(sites[0], SectionSite::EdgeInterior(_))
            && matches!(sites[1], SectionSite::FaceInterior(_));
        assert!(
            face_then_edge || edge_then_face,
            "hexagon vertex {index} must pair FaceInterior with EdgeInterior, got {sites:?}"
        );
    }
}

// ---------------------------------------------------------------------------
// 2. Slab through block: two closed rectangle loops
// ---------------------------------------------------------------------------

// Hand geometry for A = [0,1]^3 vs slab B = [-1,2] × [-1,2] × [0.25,0.75].
// B's huge z=0.25 and z=0.75 faces cross A's four side faces in one segment
// each (B's own side faces lie strictly outside A, and A's z=0/z=1 faces
// are parallel to and distinct from B's z-faces). Each ring's corners sit
// on A's four vertical corner edges:
//   z=0.25 ring: (0,0,0.25), (1,0,0.25), (1,1,0.25), (0,1,0.25)
//   z=0.75 ring: (0,0,0.75), (1,0,0.75), (1,1,0.75), (0,1,0.75)
const SLAB_RING_LOW: [[f64; 3]; 4] = [
    [0.0, 0.0, 0.25],
    [1.0, 0.0, 0.25],
    [1.0, 1.0, 0.25],
    [0.0, 1.0, 0.25],
];
const SLAB_RING_HIGH: [[f64; 3]; 4] = [
    [0.0, 0.0, 0.75],
    [1.0, 0.0, 0.75],
    [1.0, 1.0, 0.75],
    [0.0, 1.0, 0.75],
];

#[test]
fn slab_through_block_two_rectangle_loops() {
    let case = CompleteBlockCase {
        label: "slab B=[-1,2]x[-1,2]x[0.25,0.75] through A=[0,1]^3",
        a_center: [0.5, 0.5, 0.5],
        a_extents: UNIT_EXTENTS,
        b_center: [0.5, 0.5, 0.5],
        b_extents: [3.0, 3.0, 0.5],
        expected_loops: &[&SLAB_RING_LOW, &SLAB_RING_HIGH],
        expected_edges: 8,
    };
    let (_session, _part_id, _a, _b, graph) = run_complete_block_case(&case);

    // Every ring corner crosses one of A's vertical corner edges through
    // the interior of one of B's z-faces.
    for (index, vertex) in graph.vertices().iter().enumerate() {
        let sites = vertex.sites();
        assert!(
            matches!(sites[0], SectionSite::EdgeInterior(_)),
            "ring vertex {index} must sit on an edge of operand A, got {sites:?}"
        );
        assert!(
            matches!(sites[1], SectionSite::FaceInterior(_)),
            "ring vertex {index} must sit inside a face of operand B, got {sites:?}"
        );
    }
}

// ---------------------------------------------------------------------------
// 3./4. Empty sections that are certified complete
// ---------------------------------------------------------------------------

#[test]
fn disjoint_blocks_empty_complete() {
    let case = CompleteBlockCase {
        label: "disjoint blocks A=[0,1]^3 B=[10,11]x[0,1]x[0,1]",
        a_center: [0.5, 0.5, 0.5],
        a_extents: UNIT_EXTENTS,
        b_center: [10.5, 0.5, 0.5],
        b_extents: UNIT_EXTENTS,
        expected_loops: &[],
        expected_edges: 0,
    };
    let (_session, _part_id, _a, _b, graph) = run_complete_block_case(&case);
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert!(graph.gaps().is_empty());
}

#[test]
fn contained_block_empty_complete() {
    // B = [1.5,2.5]^3 strictly inside A = [0,4]^3: the boundaries never meet.
    let case = CompleteBlockCase {
        label: "contained block B=[1.5,2.5]^3 inside A=[0,4]^3",
        a_center: [2.0, 2.0, 2.0],
        a_extents: [4.0, 4.0, 4.0],
        b_center: [2.0, 2.0, 2.0],
        b_extents: UNIT_EXTENTS,
        expected_loops: &[],
        expected_edges: 0,
    };
    let (_session, _part_id, _a, _b, graph) = run_complete_block_case(&case);
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert!(graph.gaps().is_empty());
}

// ---------------------------------------------------------------------------
// 5. Coincident face pair: honest gap, no fabricated section
// ---------------------------------------------------------------------------

#[test]
fn flush_face_pair_is_honest_gap() {
    // A = [0,1]^3 and B = [0,1] × [0,1] × [1,2] share the plane z=1 exactly:
    // A's top face and B's bottom face are coincident, and every point of
    // ∂A ∩ ∂B lies in that plane (A ⊂ {z ≤ 1}, B ⊂ {z ≥ 1}). The honest
    // outcome is a refusal of the two-dimensional contact, never a stitched
    // curve inside the shared plane.
    let (session, part_id, a, b) =
        block_pair([0.5, 0.5, 0.5], UNIT_EXTENTS, [0.5, 0.5, 1.5], UNIT_EXTENTS);
    let graph = section_graph(&session, &part_id, &a, &b);

    assert_eq!(graph.completion(), SectionCompletion::Indeterminate);
    assert!(
        !graph.gaps().is_empty(),
        "an indeterminate graph must explain itself with structured gaps"
    );
    assert_stable_gap_reasons(&graph);
    assert!(
        graph
            .gaps()
            .iter()
            .any(|gap| gap.reason() == GAP_COINCIDENT_FACE_PAIR),
        "the shared-plane face pair must be reported as a coincident-face gap, got {:?}",
        graph.gaps()
    );

    // No fabricated section inside the shared plane: every certified edge
    // must leave z=1 somewhere along its samples. (Because ∂A ∩ ∂B lies
    // entirely in that plane, this — together with the oracle below —
    // forbids invented edges everywhere.)
    for (index, edge) in graph.edges().iter().enumerate() {
        let leaves_plane = sample_params(edge).into_iter().any(|t| {
            let point = edge.origin() + edge.direction() * t;
            (point.z - 1.0).abs() > 1e-9
        });
        assert!(
            leaves_plane,
            "edge {index} lies inside the coincident plane z=1: a fabricated section"
        );
    }
    assert!(
        graph.loops().iter().all(|lp| !lp.closed()),
        "no closed loop may be fabricated across a coincident contact"
    );
    assert_certified_evidence(&session, &part_id, &graph);
}

// ---------------------------------------------------------------------------
// 6. Edge touch: shared line contact is refused, not stitched
// ---------------------------------------------------------------------------

#[test]
fn edge_touch_is_honest() {
    // A = [0,1]^3 and B = [1,2] × [1,2] × [0,1] share exactly the vertical
    // edge {x=1, y=1, z ∈ [0,1]}. ∂A ∩ ∂B is that 1D segment, produced only
    // by coincident-plane pairs (A x=1 vs B x=1, A y=1 vs B y=1 — coplanar,
    // overlapping only along the line) and boundary-grazing perpendicular
    // pairs whose carrier lies along both faces' boundary edges. The honest
    // slice outcome is Indeterminate with coincident/tangent gaps; a
    // fabricated closed loop would be a lie.
    let (session, part_id, a, b) =
        block_pair([0.5, 0.5, 0.5], UNIT_EXTENTS, [1.5, 1.5, 0.5], UNIT_EXTENTS);
    let graph = section_graph(&session, &part_id, &a, &b);

    assert_eq!(graph.completion(), SectionCompletion::Indeterminate);
    assert!(
        !graph.gaps().is_empty(),
        "an indeterminate graph must explain itself with structured gaps"
    );
    assert_stable_gap_reasons(&graph);
    assert!(
        graph.loops().iter().all(|lp| !lp.closed()),
        "no closed loop may be fabricated from an edge-touch contact"
    );
    assert_certified_evidence(&session, &part_id, &graph);
}

// ---------------------------------------------------------------------------
// 7. Plane exactly through prism vertices: certified or honest, never wrong
// ---------------------------------------------------------------------------

#[test]
fn vertex_pierce_prism_is_certified_or_honest() {
    // Prism A: profile triangle (0,0), (2,0), (1,1) extruded along +z by 2,
    // so the apex ridge is the exact segment {x=1, y=1, z ∈ [0,2]} with
    // prism vertices at (1,1,0) and (1,1,2). Block B = [1,3] × [-1,2] ×
    // [-1,3]: its face plane x=1 slices the prism transversally (the slice
    // is the rectangle y ∈ [0,1], z ∈ [0,2]) and passes EXACTLY through
    // both apex vertices; the rectangle's y=1 side is the apex ridge
    // itself, which lies in the boundary of both slanted faces — the
    // certified-degenerate frontier. All coordinates are integer-exact.
    //
    // The pinned contract: either the implementation certifies the full
    // rectangle (then every exact invariant, the oracle, and the AtVertex
    // sites must hold), or it refuses honestly with stable gap reasons.
    // A Complete claim that violates the oracle or the degree invariant
    // fails; a non-constant gap string fails.
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (prism, block) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let prism = edit
            .extrude_profile(ExtrudeProfileRequest::new(
                Frame::world(),
                vec![
                    Point2::new(0.0, 0.0),
                    Point2::new(2.0, 0.0),
                    Point2::new(1.0, 1.0),
                ],
                Vec::new(),
                2.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let block = create_block_in(&mut edit, [2.0, 0.5, 1.0], [2.0, 3.0, 4.0]);
        (prism, block)
    };
    let graph = section_graph(&session, &part_id, &prism, &block);

    assert_stable_gap_reasons(&graph);
    assert_certified_evidence(&session, &part_id, &graph);

    match graph.completion() {
        SectionCompletion::Complete => {
            assert_complete_section_invariants(&graph);
            assert_eq!(graph.edges().len(), 4, "slice rectangle has four sides");
            assert_eq!(graph.vertices().len(), 4);
            assert_eq!(graph.loops().len(), 1);
            let expected = ring_points(&[
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [1.0, 1.0, 2.0],
                [1.0, 0.0, 2.0],
            ]);
            assert_point_set(&graph_vertex_points(&graph), &expected, "prism slice");

            for (index, vertex) in graph.vertices().iter().enumerate() {
                let point = vertex.point();
                let sites = vertex.sites();
                if (point.y - 1.0).abs() <= POINT_MATCH_TOL {
                    // Apex corners (1,1,0) and (1,1,2) are prism vertices.
                    assert!(
                        matches!(sites[0], SectionSite::AtVertex(_)),
                        "vertex {index} at {point:?} must sit AtVertex on the prism, got {sites:?}"
                    );
                } else {
                    // Base-side corners (1,0,0) and (1,0,2) lie strictly
                    // inside prism cap-boundary edges.
                    assert!(
                        point.y.abs() <= POINT_MATCH_TOL,
                        "unexpected slice vertex {point:?}"
                    );
                    assert!(
                        matches!(sites[0], SectionSite::EdgeInterior(_)),
                        "vertex {index} at {point:?} must sit on a prism edge, got {sites:?}"
                    );
                }
                assert!(
                    matches!(sites[1], SectionSite::FaceInterior(_)),
                    "vertex {index} at {point:?} must sit inside the block's x=1 face, got {sites:?}"
                );
            }
        }
        SectionCompletion::Indeterminate => {
            assert!(
                !graph.gaps().is_empty(),
                "an indeterminate prism slice must explain itself with structured gaps"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// 8. Determinism: serial re-execution reproduces the graph bit-identically
// ---------------------------------------------------------------------------

/// Structural fingerprint excluding facade ids (which differ across parts):
/// exact bit patterns of every numeric field, endpoint/loop index wiring,
/// site kinds, completion, and gap reasons.
fn structural_fingerprint(graph: &BodySectionGraph) -> (Vec<u64>, Vec<&'static str>) {
    fn site_kind(site: &SectionSite) -> u64 {
        match site {
            SectionSite::FaceInterior(_) => 0,
            SectionSite::EdgeInterior(_) => 1,
            SectionSite::AtVertex(_) => 2,
        }
    }

    let mut bits: Vec<u64> = Vec::new();
    let mut reasons: Vec<&'static str> = Vec::new();

    bits.push(graph.vertices().len() as u64);
    for vertex in graph.vertices() {
        let point = vertex.point();
        bits.extend([point.x.to_bits(), point.y.to_bits(), point.z.to_bits()]);
        bits.extend(vertex.sites().iter().map(site_kind));
    }

    bits.push(graph.edges().len() as u64);
    for edge in graph.edges() {
        let origin = edge.origin();
        let direction = edge.direction();
        bits.extend([
            origin.x.to_bits(),
            origin.y.to_bits(),
            origin.z.to_bits(),
            direction.x.to_bits(),
            direction.y.to_bits(),
            direction.z.to_bits(),
        ]);
        let range = edge.range();
        bits.extend([range.lo.to_bits(), range.hi.to_bits()]);
        bits.extend(edge.residual_bounds().map(f64::to_bits));
        bits.extend(edge.endpoints().map(|endpoint| endpoint as u64));
        for uv in edge.uv_lines() {
            bits.extend([
                uv.origin().x.to_bits(),
                uv.origin().y.to_bits(),
                uv.direction().x.to_bits(),
                uv.direction().y.to_bits(),
            ]);
        }
    }

    bits.push(graph.loops().len() as u64);
    for lp in graph.loops() {
        bits.push(u64::from(lp.closed()));
        bits.push(lp.edges().len() as u64);
        bits.extend(lp.edges().iter().map(|&edge_index| edge_index as u64));
    }

    bits.push(match graph.completion() {
        SectionCompletion::Complete => 0,
        SectionCompletion::Indeterminate => 1,
    });
    bits.push(graph.gaps().len() as u64);
    for gap in graph.gaps() {
        bits.push(gap.faces().len() as u64);
        reasons.push(gap.reason());
    }

    (bits, reasons)
}

#[test]
fn deterministic_rerun_bit_identical() {
    let build = || {
        let (session, part_id, a, b) =
            block_pair(CORNER_A_CENTER, UNIT_EXTENTS, CORNER_B_CENTER, UNIT_EXTENTS);
        let graph = section_graph(&session, &part_id, &a, &b);
        // Guard: the fingerprint must cover a nontrivial graph.
        assert_eq!(graph.edges().len(), 6);
        structural_fingerprint(&graph)
    };
    let first = build();
    let second = build();
    assert_eq!(
        first, second,
        "two fresh sessions must reproduce the section graph bit-for-bit"
    );
}

// ---------------------------------------------------------------------------
// 9. Operand order swap: mirrored sites, negated carrier directions
// ---------------------------------------------------------------------------

#[test]
fn operand_order_swap_mirrors() {
    let (session, part_id, a, b) =
        block_pair(CORNER_A_CENTER, UNIT_EXTENTS, CORNER_B_CENTER, UNIT_EXTENTS);
    let forward = section_graph(&session, &part_id, &a, &b);
    let swapped = section_graph(&session, &part_id, &b, &a);

    assert_eq!(swapped.bodies(), &[b, a]);
    assert_eq!(forward.edges().len(), swapped.edges().len());
    assert_eq!(forward.loops().len(), swapped.loops().len());
    assert_point_set(
        &graph_vertex_points(&swapped),
        &graph_vertex_points(&forward),
        "operand swap must preserve the section point set",
    );

    // Both graphs live on the same part, so operand sites must swap slots
    // exactly (identical entity ids, opposite order).
    for (index, vertex) in forward.vertices().iter().enumerate() {
        let mirrored = swapped
            .vertices()
            .iter()
            .find(|candidate| close3(candidate.point(), vertex.point()))
            .unwrap_or_else(|| panic!("no swapped vertex matches forward vertex {index}"));
        assert_eq!(mirrored.sites()[0], vertex.sites()[1], "vertex {index}");
        assert_eq!(mirrored.sites()[1], vertex.sites()[0], "vertex {index}");
    }

    // Canonical carrier orientation d = n_first × n_second flips sign under
    // the swap. Match edges structurally by their endpoint point pairs.
    let endpoint_points = |graph: &BodySectionGraph, edge: &SectionEdge| -> [Point3; 2] {
        edge.endpoints()
            .map(|endpoint| graph.vertices()[endpoint].point())
    };
    for (index, edge) in forward.edges().iter().enumerate() {
        let fw = endpoint_points(&forward, edge);
        let mirrored = swapped
            .edges()
            .iter()
            .find(|candidate| {
                let sw = endpoint_points(&swapped, candidate);
                (close3(sw[0], fw[0]) && close3(sw[1], fw[1]))
                    || (close3(sw[0], fw[1]) && close3(sw[1], fw[0]))
            })
            .unwrap_or_else(|| panic!("no swapped edge matches forward edge {index}"));
        let d_forward = edge.direction();
        let d_swapped = mirrored.direction();
        assert!(
            (d_forward.x + d_swapped.x).abs() <= POINT_MATCH_TOL
                && (d_forward.y + d_swapped.y).abs() <= POINT_MATCH_TOL
                && (d_forward.z + d_swapped.z).abs() <= POINT_MATCH_TOL,
            "edge {index}: swap must negate the carrier direction, got {d_forward:?} vs {d_swapped:?}"
        );
    }
}

// ---------------------------------------------------------------------------
// 10. Work budget: the limit is reported, never a wrong graph
// ---------------------------------------------------------------------------

#[test]
fn work_budget_limit_is_reported() {
    let (session, part_id, a, b) =
        block_pair(CORNER_A_CENTER, UNIT_EXTENTS, CORNER_B_CENTER, UNIT_EXTENTS);
    let plan = BudgetPlan::new([LimitSpec::new(
        SECTION_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        2,
    )])
    .unwrap();
    let outcome = session
        .part(part_id)
        .unwrap()
        .section_bodies(
            SectionBodiesRequest::new(a, b)
                .with_settings(OperationSettings::new().with_budget_overrides(plan)),
        )
        .unwrap();
    let error = outcome.into_result().unwrap_err();
    let crossing = error.limit().expect("limit evidence must be preserved");
    assert_eq!(crossing.stage, SECTION_WORK);
}

// ---------------------------------------------------------------------------
// 11. Invalid operand identities are rejected before any scope starts
// ---------------------------------------------------------------------------

#[test]
fn identical_operand_rejected() {
    let (session, part_id, a, _b) =
        block_pair(CORNER_A_CENTER, UNIT_EXTENTS, CORNER_B_CENTER, UNIT_EXTENTS);
    let error = session
        .part(part_id)
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(a.clone(), a))
        .unwrap_err();
    assert!(
        matches!(
            error,
            Error::Core {
                source: kcore::error::Error::InvalidGeometry { .. }
            }
        ),
        "sectioning a body against itself must be rejected, got {error:?}"
    );
}

#[test]
fn wrong_part_rejected() {
    let mut session = Kernel::new().create_session();
    let part_one = session.create_part();
    let part_two = session.create_part();
    let body_one = {
        let mut edit = session.edit_part(part_one.clone()).unwrap();
        create_block_in(&mut edit, [0.5, 0.5, 0.5], UNIT_EXTENTS)
    };
    let body_two = {
        let mut edit = session.edit_part(part_two).unwrap();
        create_block_in(&mut edit, [1.0, 1.0, 1.0], UNIT_EXTENTS)
    };
    let error = session
        .part(part_one)
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(body_one, body_two))
        .unwrap_err();
    assert!(
        matches!(error, Error::WrongPart { .. }),
        "a body id from a different part must be rejected, got {error:?}"
    );
}

// ---------------------------------------------------------------------------
// 12. Exact curved evidence: two full circles become endpoint-free rings
// ---------------------------------------------------------------------------

fn block_slab_and_cylinder() -> (Session, PartId, BodyId, BodyId) {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        // The block's z faces cut complete circles from the smaller cylinder;
        // its four side planes lie strictly outside the cylinder.
        let block = create_block_in(&mut edit, [0.0, 0.0, 1.0], [4.0, 4.0, 1.0]);
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(Frame::world(), 0.75, 2.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    (session, part_id, block, cylinder)
}

#[test]
fn block_slab_through_cylinder_exposes_two_exact_closed_rings() {
    let (session, part_id, block, cylinder) = block_slab_and_cylinder();
    let graph = section_graph(&session, &part_id, &block, &cylinder);

    assert_eq!(graph.completion(), SectionCompletion::Complete);
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert_eq!(graph.branches().len(), 2);
    assert_eq!(graph.rings().len(), 2);
    assert_eq!(graph.curve_endpoints().len(), 0);
    assert_eq!(graph.curve_fragments().len(), 2);
    assert_eq!(graph.curve_components().len(), 2);
    assert!(
        graph
            .curve_fragments()
            .iter()
            .all(|fragment| matches!(fragment.span(), SectionCurveFragmentSpan::Whole))
    );
    assert!(
        graph
            .curve_components()
            .iter()
            .all(SectionCurveComponent::closed)
    );
    assert!(graph.gaps().is_empty());
    let mut ring_branches = graph
        .rings()
        .iter()
        .map(|ring| ring.branch())
        .collect::<Vec<_>>();
    ring_branches.sort_unstable();
    assert_eq!(ring_branches, vec![0, 1]);
    let mut heights = Vec::new();
    for (branch_index, branch) in graph.branches().iter().enumerate() {
        assert_eq!(branch.topology(), SectionBranchTopology::Closed);
        assert_eq!(branch.endpoint_sites(), [0, 0]);
        assert_eq!(branch.fragment_sites().len(), 1);
        assert_eq!(branch.range().width(), core::f64::consts::TAU);
        assert!(matches!(branch.pcurves()[0], SectionUvCurve::Circle(_)));
        assert!(matches!(branch.pcurves()[1], SectionUvCurve::Line(_)));
        let evidence = branch.evidence();
        assert!(
            evidence
                .residual_bounds()
                .into_iter()
                .all(|bound| bound.is_finite() && bound <= evidence.tolerance())
        );
        let (center, normal, x_direction, radius) = match branch.carrier() {
            SectionCarrier::Circle {
                center,
                normal,
                x_direction,
                radius,
            } => (center, normal, x_direction, radius),
        };
        heights.push(center.z);
        assert_eq!(radius, 0.75);
        let y_direction = normal.cross(x_direction);
        for parameter in [0.17, 1.9, 4.8] {
            let (sin, cos) = kcore::math::sincos(parameter);
            let point = center + x_direction * (radius * cos) + y_direction * (radius * sin);
            assert_boundary_on_both(
                &session,
                &part_id,
                graph.bodies(),
                point,
                &format!("closed curved branch {branch_index}"),
            );
        }
        let seam = branch.fragment_sites()[0].point();
        let expected_seam = center + x_direction * radius;
        assert!(seam.dist(expected_seam) <= 1e-9);
    }
    heights.sort_by(f64::total_cmp);
    assert_eq!(heights, vec![0.5, 1.5]);
    assert_stable_gap_reasons(&graph);
}

#[test]
fn clipped_plane_cylinder_circles_retain_exact_public_arc_endpoints() {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let block = edit
            .extrude_profile(ExtrudeProfileRequest::new(
                frame_at([0.0, 0.0, 0.5]),
                vec![
                    Point2::new(-3.0, -3.0),
                    Point2::new(3.0, -3.0),
                    Point2::new(3.0, 3.0),
                    Point2::new(-3.0, 3.0),
                ],
                vec![vec![
                    Point2::new(-1.0, -2.5),
                    Point2::new(-1.0, 2.5),
                    Point2::new(1.0, 2.5),
                    Point2::new(1.0, -2.5),
                ]],
                1.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(Frame::world(), 1.5, 2.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let graph = section_graph(&session, &part_id, &block, &cylinder);

    assert_eq!(graph.branches().len(), 2, "bounded graph: {graph:#?}");
    assert_eq!(graph.curve_fragments().len(), 4);
    assert_eq!(graph.curve_endpoints().len(), 8);
    assert_eq!(graph.curve_components().len(), 4);
    assert!(graph.rings().is_empty());
    assert_eq!(graph.completion(), SectionCompletion::Indeterminate);
    assert!(
        graph
            .curve_components()
            .iter()
            .all(|component| !component.closed())
    );
    assert!(
        graph
            .gaps()
            .iter()
            .all(|gap| gap.reason() != GAP_CURVED_TRIM_UNRESOLVED),
        "public endpoint adaptation must not report the retired facade gap: {:?}",
        graph.gaps()
    );

    let part = session.part(part_id.clone()).unwrap();
    let mut branch_ordinals = vec![Vec::new(); graph.branches().len()];
    for fragment in graph.curve_fragments() {
        branch_ordinals[fragment.branch()].push(fragment.source_ordinal());
        let branch = &graph.branches()[fragment.branch()];
        let SectionCurveFragmentSpan::Arc { endpoints, .. } = fragment.span() else {
            panic!("partially clipped circle must persist as a bounded arc")
        };
        for end in endpoints.iter() {
            assert!(end.endpoint() < graph.curve_endpoints().len());
            assert_boundary_on_both(
                &session,
                &part_id,
                graph.bodies(),
                end.point(),
                "certified curved fragment endpoint",
            );
            assert!(branch.range().contains(end.carrier_parameter()));

            let trim = end.trim();
            assert_eq!(trim.face(), branch.faces()[trim.operand()]);
            assert!(trim.edge_parameter().lo() < trim.edge_parameter().hi());
            assert!(trim.pcurve_half_angle().lo() < trim.pcurve_half_angle().hi());
            assert_eq!(part.loop_(trim.loop_id()).unwrap().face(), trim.face());
            assert_eq!(part.fin(trim.fin()).unwrap().loop_(), trim.loop_id());
            assert_eq!(
                part.fin(trim.fin()).unwrap().edge(),
                trim.source_parameter().edge()
            );

            let endpoint = &graph.curve_endpoints()[end.endpoint()];
            let SectionCurveEndpointTopology::Trim {
                sites,
                source_parameters,
            } = endpoint.topology()
            else {
                panic!("physical curved trim event must not become a chart seam")
            };
            assert_eq!(
                sites[trim.operand()],
                SectionSite::EdgeInterior(trim.source_parameter().edge())
            );
            assert_eq!(
                source_parameters[trim.operand()].as_ref(),
                Some(trim.source_parameter())
            );
            assert_eq!(
                endpoint.edge_parameters()[trim.operand()],
                Some(trim.edge_parameter())
            );
        }
    }
    assert!(branch_ordinals.iter().all(|ordinals| ordinals == &[0, 1]));
    assert_stable_gap_reasons(&graph);

    let swapped = section_graph(&session, &part_id, &cylinder, &block);
    assert_eq!(swapped.curve_fragments().len(), 4);
    assert_eq!(swapped.curve_endpoints().len(), 8);
    assert!(swapped.curve_fragments().iter().all(|fragment| {
        let SectionCurveFragmentSpan::Arc { endpoints, .. } = fragment.span() else {
            return false;
        };
        endpoints.iter().all(|end| {
            end.trim().operand() == 1
                && end.trim().face() == swapped.branches()[fragment.branch()].faces()[1]
        })
    }));
    assert_stable_gap_reasons(&swapped);
}

#[test]
fn closed_ring_operand_swap_reverses_the_canonical_carriers() {
    let (session, part_id, block, cylinder) = block_slab_and_cylinder();
    let forward = section_graph(&session, &part_id, &block, &cylinder);
    let swapped = section_graph(&session, &part_id, &cylinder, &block);
    assert_eq!(forward.completion(), SectionCompletion::Complete);
    assert_eq!(swapped.completion(), SectionCompletion::Complete);
    assert_eq!(forward.rings().len(), 2);
    assert_eq!(swapped.rings().len(), 2);

    for forward_branch in forward.branches() {
        let (center, normal, x_direction) = match forward_branch.carrier() {
            SectionCarrier::Circle {
                center,
                normal,
                x_direction,
                ..
            } => (center, normal, x_direction),
        };
        let swapped_branch = swapped
            .branches()
            .iter()
            .find(|branch| match branch.carrier() {
                SectionCarrier::Circle {
                    center: candidate, ..
                } => candidate.dist(center) <= POINT_MATCH_TOL,
            })
            .expect("swapped graph must retain the same geometric ring");
        let (swapped_normal, swapped_x) = match swapped_branch.carrier() {
            SectionCarrier::Circle {
                normal,
                x_direction,
                ..
            } => (normal, x_direction),
        };
        assert!((normal + swapped_normal).norm() <= POINT_MATCH_TOL);
        assert!((x_direction - swapped_x).norm() <= POINT_MATCH_TOL);

        let (SectionUvCurve::Circle(forward_plane), SectionUvCurve::Circle(swapped_plane)) =
            (forward_branch.pcurves()[0], swapped_branch.pcurves()[1])
        else {
            panic!("operand swap must exchange the plane pcurve slot")
        };
        assert_eq!(
            forward_plane.parameter_scale(),
            -swapped_plane.parameter_scale()
        );
        let (SectionUvCurve::Line(forward_cylinder), SectionUvCurve::Line(swapped_cylinder)) =
            (forward_branch.pcurves()[1], swapped_branch.pcurves()[0])
        else {
            panic!("operand swap must exchange the cylinder pcurve slot")
        };
        let forward_direction = forward_cylinder.direction();
        let swapped_direction = swapped_cylinder.direction();
        assert_eq!(forward_direction.x, -swapped_direction.x);
        assert_eq!(forward_direction.y, -swapped_direction.y);
    }
}

#[test]
fn rigidly_oriented_slab_through_cylinder_keeps_two_exact_rings() {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let base = Point3::new(3.0, -2.0, 1.25);
    let cylinder_frame =
        Frame::new(base, Vec3::new(0.0, 0.6, 0.8), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let block_frame = cylinder_frame.with_origin(base + cylinder_frame.z());
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let block = edit
            .create_block(BlockRequest::new(block_frame, [4.0, 4.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(cylinder_frame, 0.75, 2.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let graph = section_graph(&session, &part_id, &block, &cylinder);
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "rigid graph gaps: {:?}",
        graph.gaps()
    );
    assert_eq!(graph.rings().len(), 2);
    assert!(graph.gaps().is_empty());

    let mut axial_parameters = graph
        .rings()
        .iter()
        .map(|ring| match graph.branches()[ring.branch()].carrier() {
            SectionCarrier::Circle { center, .. } => (center - base).dot(cylinder_frame.z()),
        })
        .collect::<Vec<_>>();
    axial_parameters.sort_by(f64::total_cmp);
    assert!((axial_parameters[0] - 0.5).abs() <= POINT_MATCH_TOL);
    assert!((axial_parameters[1] - 1.5).abs() <= POINT_MATCH_TOL);
}
