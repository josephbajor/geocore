//! Combinatorial stitching of certified section segments into edge graphs.
//!
//! Segment endpoints are identified across face pairs by exact combinatorial
//! keys — the operand entity (edge or vertex) that produced the endpoint
//! together with the opposing operand's site — never by comparing derived
//! floating-point coordinates. Two segments share a graph vertex exactly
//! when their endpoint keys are equal. Every graph invariant (vertex degree
//! two, chain closure) is checked structurally; violations become structured
//! gaps carried alongside the partial graph, never silent repairs.
//!
//! Determinism: input segments arrive in pair-major, along-carrier order;
//! vertices number in first-appearance order over that sequence, edges keep
//! it outright, and loops start from the lowest unused edge index walking
//! each edge in its stored (canonical) direction.

use std::collections::HashMap;
use std::collections::hash_map::Entry;

use ktopo::entity::{EdgeId as RawEdgeId, FaceId as RawFaceId, VertexId as RawVertexId};

/// The boundary entity of one operand that anchors a segment endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum SiteKey {
    /// The carrier stays inside this face at the endpoint.
    Face(RawFaceId),
    /// The carrier crosses the interior of this edge.
    Edge(RawEdgeId),
    /// The carrier passes exactly through this vertex.
    Vertex(RawVertexId),
}

/// Exact combinatorial identity of one segment endpoint across both
/// operands, in operand order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct VertexKey {
    pub a: SiteKey,
    pub b: SiteKey,
}

/// One certified section segment awaiting stitching.
///
/// `start`/`end` follow the segment's canonical traversal direction (the
/// carrier direction after canonical orientation), so a walk leaving through
/// `end` continues into the successor segment's `start`.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct StitchSegment {
    /// Deterministic ordinal of the owning candidate face pair.
    pub pair: usize,
    /// Carrier faces on the two operands, in operand order.
    pub faces: [RawFaceId; 2],
    /// Combinatorial identity of the traversal-start endpoint.
    pub start: VertexKey,
    /// Combinatorial identity of the traversal-end endpoint.
    pub end: VertexKey,
    /// Numeric representative of the traversal-start endpoint.
    pub start_point: [f64; 3],
    /// Numeric representative of the traversal-end endpoint.
    pub end_point: [f64; 3],
}

/// One stitched graph vertex.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct StitchVertex {
    pub key: VertexKey,
    /// Numeric representative from the first segment endpoint that created
    /// this vertex (deterministic).
    pub point: [f64; 3],
    /// Number of segment endpoints landing on this vertex.
    pub degree: usize,
}

/// One stitched graph edge: the segment plus its endpoint vertex indices.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct StitchEdge {
    /// Index into the caller's segment sequence (== this edge's index).
    pub segment: usize,
    /// Vertex indices at the traversal start/end.
    pub endpoints: [usize; 2],
}

/// One maximal stitched chain.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct StitchChain {
    /// Edge indices in traversal order.
    pub edges: Vec<usize>,
    /// Whether the walk returned to its starting vertex.
    pub closed: bool,
}

/// Structural defects found while stitching.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum StitchDefect {
    /// A vertex's degree differs from two (index into `vertices`).
    DegreeNotTwo(usize),
    /// A chain failed to close (index into `chains`).
    OpenChain(usize),
}

/// Deterministic stitched graph with structural evidence.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct StitchResult {
    pub vertices: Vec<StitchVertex>,
    pub edges: Vec<StitchEdge>,
    pub chains: Vec<StitchChain>,
    pub defects: Vec<StitchDefect>,
}

/// Stitch certified segments into a deterministic edge graph.
///
/// Vertices are grouped by exact [`VertexKey`] equality in first-appearance
/// order. Chains walk forward from the lowest unused edge, leaving each
/// vertex through its unique other incident edge; a vertex whose degree is
/// not two ends the walk and is recorded as a defect, as is any chain that
/// fails to close. The input order fully determines the output.
pub(crate) fn stitch_segments(segments: &[StitchSegment]) -> StitchResult {
    let mut vertices: Vec<StitchVertex> = Vec::new();
    let mut index_of: HashMap<VertexKey, usize> = HashMap::new();
    let mut incident: Vec<Vec<Incidence>> = Vec::new();
    let mut edges: Vec<StitchEdge> = Vec::with_capacity(segments.len());

    for (segment, seg) in segments.iter().enumerate() {
        let start = intern(
            &mut vertices,
            &mut index_of,
            &mut incident,
            seg.start,
            seg.start_point,
        );
        incident[start].push(Incidence {
            edge: segment,
            kind: EndKind::Start,
        });
        let end = intern(
            &mut vertices,
            &mut index_of,
            &mut incident,
            seg.end,
            seg.end_point,
        );
        incident[end].push(Incidence {
            edge: segment,
            kind: EndKind::End,
        });
        edges.push(StitchEdge {
            segment,
            endpoints: [start, end],
        });
    }

    let mut used = vec![false; edges.len()];
    let mut chains: Vec<StitchChain> = Vec::new();
    for first in 0..edges.len() {
        if used[first] {
            continue;
        }
        used[first] = true;
        let origin = edges[first].endpoints[0];
        let mut at = edges[first].endpoints[1];
        let mut last = first;
        let mut chain = vec![first];
        let closed = loop {
            if at == origin {
                break true;
            }
            let landings = &incident[at];
            if landings.len() != 2 {
                // Degree defect: the walk cannot pick a unique continuation.
                break false;
            }
            // The arrival incidence is `last`'s end landing on `at`; the
            // continuation is the one remaining incidence.
            let next = if landings[0].edge == last && landings[0].kind == EndKind::End {
                landings[1]
            } else {
                landings[0]
            };
            if next.kind != EndKind::Start {
                // Orientation inconsistency: the continuation edge presents
                // its end here. Reversing it would repair upstream evidence,
                // so the chain ends open instead.
                break false;
            }
            if used[next.edge] {
                // The continuation was already consumed by an earlier chain
                // (possible only downstream of another defect).
                break false;
            }
            used[next.edge] = true;
            chain.push(next.edge);
            last = next.edge;
            at = edges[next.edge].endpoints[1];
        };
        chains.push(StitchChain {
            edges: chain,
            closed,
        });
    }

    let mut defects: Vec<StitchDefect> = Vec::new();
    for (index, vertex) in vertices.iter().enumerate() {
        if vertex.degree != 2 {
            defects.push(StitchDefect::DegreeNotTwo(index));
        }
    }
    for (index, chain) in chains.iter().enumerate() {
        if !chain.closed {
            defects.push(StitchDefect::OpenChain(index));
        }
    }

    StitchResult {
        vertices,
        edges,
        chains,
        defects,
    }
}

/// Which endpoint of a stitched edge lands on a vertex.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EndKind {
    Start,
    End,
}

/// One edge endpoint landing on a vertex, recorded in edge order.
#[derive(Debug, Clone, Copy)]
struct Incidence {
    edge: usize,
    kind: EndKind,
}

/// Returns the vertex index for `key`, allocating the next index on first
/// appearance (with `point` as the representative) and counting one endpoint
/// incidence either way. `incident` grows in lockstep with `vertices`.
fn intern(
    vertices: &mut Vec<StitchVertex>,
    index_of: &mut HashMap<VertexKey, usize>,
    incident: &mut Vec<Vec<Incidence>>,
    key: VertexKey,
    point: [f64; 3],
) -> usize {
    match index_of.entry(key) {
        Entry::Occupied(slot) => {
            let index = *slot.get();
            vertices[index].degree += 1;
            index
        }
        Entry::Vacant(slot) => {
            let index = vertices.len();
            slot.insert(index);
            vertices.push(StitchVertex {
                key,
                point,
                degree: 1,
            });
            incident.push(Vec::new());
            index
        }
    }
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;
    use ktopo::store::Store;

    use super::*;

    /// Real raw handles harvested from one block body. The stitcher treats
    /// them as opaque keys and never dereferences them, but fabricating
    /// handles from integers would bypass the arena's provenance.
    struct Ids {
        faces: Vec<RawFaceId>,
        edges: Vec<RawEdgeId>,
        vertices: Vec<RawVertexId>,
    }

    fn ids() -> Ids {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [2.0, 2.0, 2.0])
            .expect("block construction succeeds");
        Ids {
            faces: store.faces_of_body(body).expect("block has faces"),
            edges: store.edges_of_body(body).expect("block has edges"),
            vertices: store.vertices_of_body(body).expect("block has vertices"),
        }
    }

    /// Distinct combinatorial key per ordinal, cycling through all three
    /// site kinds on operand A (distinct handles per kind keep keys unequal).
    fn key(ids: &Ids, ordinal: usize) -> VertexKey {
        let a = match ordinal % 3 {
            0 => SiteKey::Edge(ids.edges[ordinal]),
            1 => SiteKey::Vertex(ids.vertices[ordinal]),
            _ => SiteKey::Face(ids.faces[ordinal]),
        };
        VertexKey {
            a,
            b: SiteKey::Face(ids.faces[0]),
        }
    }

    /// Segment from key `from` to key `to`; `pair` doubles as a point marker
    /// so representative-point provenance is observable per segment.
    fn seg(ids: &Ids, pair: usize, from: usize, to: usize) -> StitchSegment {
        StitchSegment {
            pair,
            faces: [ids.faces[0], ids.faces[1]],
            start: key(ids, from),
            end: key(ids, to),
            start_point: [from as f64, pair as f64, 0.0],
            end_point: [to as f64, pair as f64, 0.0],
        }
    }

    #[test]
    fn square_loop_stitches_into_one_closed_chain() {
        let ids = ids();
        let segments = [
            seg(&ids, 0, 0, 1),
            seg(&ids, 1, 1, 2),
            seg(&ids, 2, 2, 3),
            seg(&ids, 3, 3, 0),
        ];
        let result = stitch_segments(&segments);

        assert_eq!(result.vertices.len(), 4);
        for (index, vertex) in result.vertices.iter().enumerate() {
            assert_eq!(vertex.key, key(&ids, index));
            assert_eq!(vertex.degree, 2);
        }
        // Representatives come from the first appearance of each key: key 0
        // from segment 0's start, key 3 from segment 2's end — never from
        // the later re-appearances with different pair markers.
        assert_eq!(result.vertices[0].point, [0.0, 0.0, 0.0]);
        assert_eq!(result.vertices[3].point, [3.0, 2.0, 0.0]);

        assert_eq!(result.edges.len(), 4);
        for (index, edge) in result.edges.iter().enumerate() {
            assert_eq!(edge.segment, index);
        }
        let endpoints: Vec<[usize; 2]> = result.edges.iter().map(|e| e.endpoints).collect();
        assert_eq!(endpoints, [[0, 1], [1, 2], [2, 3], [3, 0]]);

        assert_eq!(
            result.chains,
            [StitchChain {
                edges: vec![0, 1, 2, 3],
                closed: true,
            }]
        );
        assert!(result.defects.is_empty());
    }

    #[test]
    fn interleaved_disjoint_loops_discover_chains_from_lowest_edge() {
        let ids = ids();
        // Two triangles with their segments interleaved in the input; chain
        // discovery must start at the lowest unused edge index each time.
        let segments = [
            seg(&ids, 0, 0, 1),
            seg(&ids, 1, 3, 4),
            seg(&ids, 0, 1, 2),
            seg(&ids, 1, 4, 5),
            seg(&ids, 0, 2, 0),
            seg(&ids, 1, 5, 3),
        ];
        let result = stitch_segments(&segments);

        // First-appearance vertex order interleaves the two loops.
        let keys: Vec<VertexKey> = result.vertices.iter().map(|v| v.key).collect();
        let expected: Vec<VertexKey> = [0, 1, 3, 4, 2, 5]
            .map(|ordinal| key(&ids, ordinal))
            .to_vec();
        assert_eq!(keys, expected);
        assert!(result.vertices.iter().all(|v| v.degree == 2));

        let endpoints: Vec<[usize; 2]> = result.edges.iter().map(|e| e.endpoints).collect();
        assert_eq!(endpoints, [[0, 1], [2, 3], [1, 4], [3, 5], [4, 0], [5, 2]]);

        assert_eq!(
            result.chains,
            [
                StitchChain {
                    edges: vec![0, 2, 4],
                    closed: true,
                },
                StitchChain {
                    edges: vec![1, 3, 5],
                    closed: true,
                },
            ]
        );
        assert!(result.defects.is_empty());
    }

    #[test]
    fn open_path_records_endpoint_degree_defects_and_open_chain() {
        let ids = ids();
        let segments = [seg(&ids, 0, 0, 1), seg(&ids, 0, 1, 2), seg(&ids, 0, 2, 3)];
        let result = stitch_segments(&segments);

        let degrees: Vec<usize> = result.vertices.iter().map(|v| v.degree).collect();
        assert_eq!(degrees, [1, 2, 2, 1]);
        assert_eq!(
            result.chains,
            [StitchChain {
                edges: vec![0, 1, 2],
                closed: false,
            }]
        );
        assert_eq!(
            result.defects,
            [
                StitchDefect::DegreeNotTwo(0),
                StitchDefect::DegreeNotTwo(3),
                StitchDefect::OpenChain(0),
            ]
        );
    }

    #[test]
    fn t_junction_records_degree_defects_and_open_chains() {
        let ids = ids();
        let segments = [seg(&ids, 0, 0, 1), seg(&ids, 0, 1, 2), seg(&ids, 0, 1, 3)];
        let result = stitch_segments(&segments);

        let degrees: Vec<usize> = result.vertices.iter().map(|v| v.degree).collect();
        assert_eq!(degrees, [1, 3, 1, 1]);
        assert_eq!(
            result.chains,
            [
                StitchChain {
                    edges: vec![0],
                    closed: false,
                },
                StitchChain {
                    edges: vec![1],
                    closed: false,
                },
                StitchChain {
                    edges: vec![2],
                    closed: false,
                },
            ]
        );
        assert_eq!(
            result.defects,
            [
                StitchDefect::DegreeNotTwo(0),
                StitchDefect::DegreeNotTwo(1),
                StitchDefect::DegreeNotTwo(2),
                StitchDefect::DegreeNotTwo(3),
                StitchDefect::OpenChain(0),
                StitchDefect::OpenChain(1),
                StitchDefect::OpenChain(2),
            ]
        );
    }

    #[test]
    fn self_loop_segment_closes_as_single_edge_chain() {
        let ids = ids();
        let segments = [seg(&ids, 0, 0, 0)];
        let result = stitch_segments(&segments);

        assert_eq!(result.vertices.len(), 1);
        assert_eq!(result.vertices[0].degree, 2);
        assert_eq!(result.edges.len(), 1);
        assert_eq!(result.edges[0].endpoints, [0, 0]);
        assert_eq!(
            result.chains,
            [StitchChain {
                edges: vec![0],
                closed: true,
            }]
        );
        assert!(result.defects.is_empty());
    }

    #[test]
    fn parallel_same_direction_edges_end_chains_open_without_repair() {
        let ids = ids();
        // Both edges run key 0 → key 1: every vertex has degree two, but the
        // continuation at key 1 presents its end, so the walk must not
        // silently reverse it — both chains end open with no degree defects.
        let segments = [seg(&ids, 0, 0, 1), seg(&ids, 1, 0, 1)];
        let result = stitch_segments(&segments);

        let degrees: Vec<usize> = result.vertices.iter().map(|v| v.degree).collect();
        assert_eq!(degrees, [2, 2]);
        assert_eq!(
            result.chains,
            [
                StitchChain {
                    edges: vec![0],
                    closed: false,
                },
                StitchChain {
                    edges: vec![1],
                    closed: false,
                },
            ]
        );
        assert_eq!(
            result.defects,
            [StitchDefect::OpenChain(0), StitchDefect::OpenChain(1)]
        );
    }

    #[test]
    fn empty_input_yields_empty_graph() {
        let result = stitch_segments(&[]);
        assert!(result.vertices.is_empty());
        assert!(result.edges.is_empty());
        assert!(result.chains.is_empty());
        assert!(result.defects.is_empty());
    }

    #[test]
    fn identical_input_reproduces_identical_result() {
        let ids = ids();
        let segments = [
            seg(&ids, 0, 0, 1),
            seg(&ids, 1, 3, 4),
            seg(&ids, 0, 1, 2),
            seg(&ids, 1, 4, 5),
            seg(&ids, 0, 2, 0),
            seg(&ids, 1, 5, 3),
        ];
        assert_eq!(stitch_segments(&segments), stitch_segments(&segments));
    }

    #[test]
    fn rotated_input_renumbers_outputs_correspondingly() {
        let ids = ids();
        let base = [
            seg(&ids, 0, 0, 1),
            seg(&ids, 1, 1, 2),
            seg(&ids, 2, 2, 3),
            seg(&ids, 3, 3, 0),
        ];
        let rotated = [
            base[1].clone(),
            base[2].clone(),
            base[3].clone(),
            base[0].clone(),
        ];
        let result = stitch_segments(&rotated);

        // Vertex numbering follows the rotated first-appearance order …
        let keys: Vec<VertexKey> = result.vertices.iter().map(|v| v.key).collect();
        let expected: Vec<VertexKey> = [1, 2, 3, 0].map(|ordinal| key(&ids, ordinal)).to_vec();
        assert_eq!(keys, expected);
        // … and the graph structure is the same loop under that renumbering.
        let endpoints: Vec<[usize; 2]> = result.edges.iter().map(|e| e.endpoints).collect();
        assert_eq!(endpoints, [[0, 1], [1, 2], [2, 3], [3, 0]]);
        assert_eq!(
            result.chains,
            [StitchChain {
                edges: vec![0, 1, 2, 3],
                closed: true,
            }]
        );
        assert!(result.defects.is_empty());
    }
}
