//! Deterministic Q2b geometry-graph traversal fixtures and contracts.

use core::time::Duration;
use std::collections::HashMap;

use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point3, Vec2, Vec3};
use kgraph::{
    AffineParamMap1d, GeometryGraph, GeometryRef, OffsetSurfaceDescriptor,
    certify_paired_plane_line_residuals,
};

/// Fixture identity shared by every Q2b traversal case.
pub const FIXTURE_VERSION: &str = "graph-traversal.v2";
/// Deterministic fixture seed (construction itself is not randomized).
pub const FIXTURE_SEED: u64 = 0x5154_3254_5241_0009;

/// One graph traversal operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Traversal {
    /// Produce the dependency-first closure of an offset-chain root.
    DependencyClosure,
    /// Exhaust the chain while searching for an unrelated live target.
    MissingDependencyPath,
    /// Produce a dependency-first closure whose two offset branches share one basis.
    DiamondDependencyClosure,
    /// Exhaust the shared-basis diamond while searching for an unrelated target.
    DiamondMissingDependencyPath,
}

/// Stable Q2b case definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GraphTraversalCase {
    /// Five-segment benchmark path.
    pub path: &'static str,
    /// Traversal operation.
    pub traversal: Traversal,
    /// Number of dependency edges in the prepared chain.
    pub scale: usize,
    /// Reviewed digest of the semantic traversal evidence.
    pub expected_output_digest: u64,
}

/// The ten Q2b chain and production-diamond traversal cases.
pub const CASES: [GraphTraversalCase; 10] = [
    case(Traversal::DependencyClosure, 1, 0xd32a_43f8_8885_5691),
    case(Traversal::DependencyClosure, 10, 0x9cf1_e363_1a47_4c8c),
    case(Traversal::DependencyClosure, 100, 0x569e_4ec5_e87b_41e6),
    case(Traversal::DependencyClosure, 1_000, 0x0ef4_22c4_76a5_66ab),
    case(Traversal::MissingDependencyPath, 1, 0x7612_e6bc_420a_1036),
    case(Traversal::MissingDependencyPath, 10, 0x4cea_5b14_1133_07ea),
    case(Traversal::MissingDependencyPath, 100, 0xa6d3_b09a_13ca_0ed6),
    case(
        Traversal::MissingDependencyPath,
        1_000,
        0x4f96_2c80_64e3_d636,
    ),
    case(
        Traversal::DiamondDependencyClosure,
        1,
        0x477d_9f7a_09bd_7d51,
    ),
    case(
        Traversal::DiamondMissingDependencyPath,
        1,
        0x55e4_e90b_1607_4b8d,
    ),
];

const fn case(
    traversal: Traversal,
    scale: usize,
    expected_output_digest: u64,
) -> GraphTraversalCase {
    let path = match (traversal, scale) {
        (Traversal::DependencyClosure, 1) => {
            "graph/traverse/offset-chain-closure-v1/1/dependency-first-v1"
        }
        (Traversal::DependencyClosure, 10) => {
            "graph/traverse/offset-chain-closure-v1/10/dependency-first-v1"
        }
        (Traversal::DependencyClosure, 100) => {
            "graph/traverse/offset-chain-closure-v1/100/dependency-first-v1"
        }
        (Traversal::DependencyClosure, 1_000) => {
            "graph/traverse/offset-chain-closure-v1/1000/dependency-first-v1"
        }
        (Traversal::MissingDependencyPath, 1) => {
            "graph/traverse/offset-chain-miss-v1/1/deterministic-v1"
        }
        (Traversal::MissingDependencyPath, 10) => {
            "graph/traverse/offset-chain-miss-v1/10/deterministic-v1"
        }
        (Traversal::MissingDependencyPath, 100) => {
            "graph/traverse/offset-chain-miss-v1/100/deterministic-v1"
        }
        (Traversal::MissingDependencyPath, 1_000) => {
            "graph/traverse/offset-chain-miss-v1/1000/deterministic-v1"
        }
        (Traversal::DiamondDependencyClosure, 1) => {
            "graph/traverse/verified-intersection-diamond-v1/1/dependency-first-v1"
        }
        (Traversal::DiamondMissingDependencyPath, 1) => {
            "graph/traverse/verified-intersection-diamond-miss-v1/1/deterministic-v1"
        }
        _ => "",
    };
    GraphTraversalCase {
        path,
        traversal,
        scale,
        expected_output_digest,
    }
}

/// Fully prepared graph; construction and ordinal indexing are outside timing.
pub struct GraphTraversalFixture {
    case: GraphTraversalCase,
    graph: GeometryGraph,
    root: GeometryRef,
    unrelated: GeometryRef,
    ordinals: HashMap<GeometryRef, usize>,
}

impl GraphTraversalFixture {
    /// Prepare one registered chain or shared-basis diamond and unrelated target.
    pub fn new(case: GraphTraversalCase) -> Self {
        assert!(case.scale > 0);
        let (graph, root, unrelated) = match case.traversal {
            Traversal::DependencyClosure | Traversal::MissingDependencyPath => {
                prepare_chain(case.scale)
            }
            Traversal::DiamondDependencyClosure | Traversal::DiamondMissingDependencyPath => {
                assert_eq!(case.scale, 1);
                prepare_diamond()
            }
        };
        graph.validate().expect("prepared graph must validate");
        let ordinals = graph
            .geometry()
            .enumerate()
            .map(|(ordinal, geometry)| (geometry, ordinal))
            .collect();
        Self {
            case,
            graph,
            root,
            unrelated,
            ordinals,
        }
    }

    /// Execute one sample, timing only the requested graph traversal.
    pub fn measure_once(&self) -> (Duration, GraphTraversalResult) {
        let started = std::time::Instant::now();
        let result = self.traverse();
        let elapsed = started.elapsed();
        let repeated = self.traverse();
        let stable = result == repeated;
        let result_nodes = result.as_ref().map_or(0, Vec::len);
        let reached = result.is_some();
        let result_digest = self.result_digest(result.as_deref());
        (
            elapsed,
            GraphTraversalResult {
                nodes: self.graph.len(),
                dependency_edges: expected_dependency_edges(self.case),
                result_nodes,
                reached,
                stable,
                result_digest,
            },
        )
    }

    fn traverse(&self) -> Option<Vec<GeometryRef>> {
        match self.case.traversal {
            Traversal::DependencyClosure | Traversal::DiamondDependencyClosure => Some(
                self.graph
                    .dependency_closure(self.root)
                    .expect("prepared closure must succeed"),
            ),
            Traversal::MissingDependencyPath | Traversal::DiamondMissingDependencyPath => self
                .graph
                .dependency_path(self.root, self.unrelated)
                .expect("prepared path query must succeed"),
        }
    }

    fn result_digest(&self, result: Option<&[GeometryRef]>) -> u64 {
        let mut digest = ResultHasher::new();
        digest.tag(0x54);
        match result {
            Some(result) => {
                digest.tag(1);
                digest.count(result.len());
                for geometry in result {
                    digest.count(self.ordinals[geometry]);
                }
            }
            None => digest.tag(0),
        }
        digest.finish()
    }
}

/// Semantic counters and deterministic evidence from one traversal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GraphTraversalResult {
    /// Prepared live nodes, including the unrelated target.
    pub nodes: usize,
    /// Prepared direct dependency edges.
    pub dependency_edges: usize,
    /// Nodes returned by the traversal.
    pub result_nodes: usize,
    /// Whether the path/closure operation returned a node sequence.
    pub reached: bool,
    /// Whether an immediate repeat produced the identical sequence.
    pub stable: bool,
    /// Stable digest of result presence and node ordinals.
    pub result_digest: u64,
}

impl GraphTraversalResult {
    /// Stable digest over counters and correctness evidence.
    pub fn output_digest(self) -> u64 {
        let mut digest = ResultHasher::new();
        digest.tag(0x74);
        digest.count(self.nodes);
        digest.count(self.dependency_edges);
        digest.count(self.result_nodes);
        digest.boolean(self.reached);
        digest.boolean(self.stable);
        digest.u64(self.result_digest);
        digest.finish()
    }
}

/// Verify exact counters and reviewed digests before accepting a sample.
pub fn verify(case: GraphTraversalCase, result: GraphTraversalResult) {
    assert_eq!(result.nodes, expected_nodes(case));
    assert_eq!(result.dependency_edges, expected_dependency_edges(case));
    assert!(result.stable);
    match case.traversal {
        Traversal::DependencyClosure => {
            assert_eq!(result.result_nodes, case.scale + 1);
            assert!(result.reached);
        }
        Traversal::DiamondDependencyClosure => {
            assert_eq!(result.result_nodes, 6);
            assert!(result.reached);
        }
        Traversal::MissingDependencyPath | Traversal::DiamondMissingDependencyPath => {
            assert_eq!(result.result_nodes, 0);
            assert!(!result.reached);
        }
    }
    assert_ne!(case.expected_output_digest, 0);
    assert_eq!(result.output_digest(), case.expected_output_digest);
}

const fn expected_nodes(case: GraphTraversalCase) -> usize {
    match case.traversal {
        Traversal::DependencyClosure | Traversal::MissingDependencyPath => case.scale + 2,
        Traversal::DiamondDependencyClosure | Traversal::DiamondMissingDependencyPath => 7,
    }
}

const fn expected_dependency_edges(case: GraphTraversalCase) -> usize {
    match case.traversal {
        Traversal::DependencyClosure | Traversal::MissingDependencyPath => case.scale,
        Traversal::DiamondDependencyClosure | Traversal::DiamondMissingDependencyPath => 6,
    }
}

fn prepare_chain(scale: usize) -> (GeometryGraph, GeometryRef, GeometryRef) {
    let mut graph = GeometryGraph::new();
    let basis = graph
        .insert_surface(plane(0))
        .expect("chain basis must be valid");
    let mut root = basis;
    for ordinal in 0..scale {
        root = graph
            .insert_surface(OffsetSurfaceDescriptor::new(
                root,
                (ordinal + 1) as f64 * 0.125,
            ))
            .expect("offset-chain descriptor must be valid");
    }
    let unrelated = graph
        .insert_surface(plane(scale + 1))
        .expect("unrelated target must be valid");
    (
        graph,
        GeometryRef::Surface(root),
        GeometryRef::Surface(unrelated),
    )
}

fn prepare_diamond() -> (GeometryGraph, GeometryRef, GeometryRef) {
    let mut graph = GeometryGraph::new();
    let basis_plane = Plane::new(Frame::world());
    let basis = graph
        .insert_surface(basis_plane)
        .expect("diamond basis must be valid");
    let sources = [
        graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.25))
            .expect("first diamond branch must be valid"),
        graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.25))
            .expect("second diamond branch must be valid"),
    ];
    let pcurves = [
        Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0))
            .expect("first diamond pcurve must be valid"),
        Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0))
            .expect("second diamond pcurve must be valid"),
    ];
    let pcurve_handles = [
        graph
            .insert_curve2d(pcurves[0])
            .expect("first diamond pcurve must insert"),
        graph
            .insert_curve2d(pcurves[1])
            .expect("second diamond pcurve must insert"),
    ];
    let carrier = Line::new(Point3::new(0.0, 0.0, 0.25), Vec3::new(1.0, 0.0, 0.0))
        .expect("diamond carrier must be valid");
    let certified_plane = Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, 0.25)));
    let identity = AffineParamMap1d::new(1.0, 0.0).expect("identity map must be valid");
    let certificate = certify_paired_plane_line_residuals(
        carrier,
        ParamRange::new(-8.0, 8.0),
        [certified_plane; 2],
        pcurves,
        [identity; 2],
        1.0e-12,
    )
    .expect("diamond trace must certify");
    let root = graph
        .insert_verified_plane_intersection_curve(sources, pcurve_handles, certificate)
        .expect("diamond merge must be valid");
    let unrelated = graph
        .insert_surface(plane(2))
        .expect("unrelated target must be valid");
    (
        graph,
        GeometryRef::Curve(root),
        GeometryRef::Surface(unrelated),
    )
}

fn plane(ordinal: usize) -> Plane {
    let frame = Frame::from_z(
        Point3::new(ordinal as f64, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .expect("fixture frame must be valid");
    Plane::new(frame)
}

struct ResultHasher(u64);

impl ResultHasher {
    const fn new() -> Self {
        Self(14_695_981_039_346_656_037)
    }

    fn bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 = (self.0 ^ u64::from(byte)).wrapping_mul(1_099_511_628_211);
        }
    }

    fn tag(&mut self, value: u8) {
        self.bytes(&[value]);
    }

    fn boolean(&mut self, value: bool) {
        self.tag(u8::from(value));
    }

    fn u64(&mut self, value: u64) {
        self.bytes(&value.to_le_bytes());
    }

    fn count(&mut self, value: usize) {
        self.u64(value as u64);
    }

    const fn finish(self) -> u64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    #[test]
    fn registry_contains_exactly_ten_unique_canonical_cases() {
        assert_eq!(CASES.len(), 10);
        let unique: BTreeSet<_> = CASES.iter().map(|case| case.path).collect();
        assert_eq!(unique.len(), CASES.len());
        for case in CASES {
            crate::validate_case_path(case.path).unwrap();
            assert_eq!(include_str!("../cases.json").matches(case.path).count(), 1);
        }
    }

    #[test]
    fn generated_evidence_is_repeatable() {
        for case in CASES {
            let fixture = GraphTraversalFixture::new(case);
            let first = fixture.measure_once().1;
            let second = fixture.measure_once().1;
            assert_eq!(first, second);
        }
    }

    #[test]
    fn json_registry_matches_every_rust_case_and_counter() {
        let manifest: serde_json::Value =
            serde_json::from_str(include_str!("../cases.json")).unwrap();
        let registered: Vec<_> = manifest["cases"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|entry| entry["benchmark_target"] == "graph_traversal")
            .collect();
        assert_eq!(registered.len(), CASES.len());

        for case in CASES {
            let entry = registered
                .iter()
                .copied()
                .find(|entry| entry["path"] == case.path)
                .expect("Rust Q2b case must be registered");
            let counters = &entry["expected_result_counters"];
            assert_eq!(entry["fixture_version"], FIXTURE_VERSION);
            assert_eq!(entry["deterministic_seed"], FIXTURE_SEED);
            assert_eq!(counters["nodes"], expected_nodes(case));
            assert_eq!(
                counters["dependency_edges"],
                expected_dependency_edges(case)
            );
            assert_eq!(counters["stable"], true);
            match case.traversal {
                Traversal::DependencyClosure => {
                    assert_eq!(counters["result_nodes"], case.scale + 1);
                    assert_eq!(counters["reached"], true);
                }
                Traversal::DiamondDependencyClosure => {
                    assert_eq!(counters["result_nodes"], 6);
                    assert_eq!(counters["reached"], true);
                }
                Traversal::MissingDependencyPath | Traversal::DiamondMissingDependencyPath => {
                    assert_eq!(counters["result_nodes"], 0);
                    assert_eq!(counters["reached"], false);
                }
            }
            assert_eq!(
                counters["output_digest"],
                format!("{:016x}", case.expected_output_digest)
            );
        }
    }
}
