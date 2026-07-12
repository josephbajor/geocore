//! Deterministic Q2a geometry-graph construction fixtures and contracts.

use core::time::Duration;

use kgeom::frame::Frame;
use kgeom::surface::Plane;
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    GeometryGraph, GeometryRef, GraphBuildObservation, OffsetSurfaceDescriptor, SurfaceDescriptor,
};

/// Fixture identity shared by every implemented Q2a case.
pub const FIXTURE_VERSION: &str = "graph-build.v1";
/// Deterministic fixture seed (construction itself is not randomized).
pub const FIXTURE_SEED: u64 = 0x5154_3241_4752_0006;

/// One implemented Q2a graph shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ladder {
    /// Leaf planes without dependency edges.
    Independent,
    /// One plane followed by a dependency-first offset chain.
    Chain,
    /// Offset surfaces sharing one plane basis.
    Fanout,
    /// A transient dependent chain followed by rejection and rollback.
    Rollback,
}

/// Stable Q2a case definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GraphBuildCase {
    /// Five-segment benchmark path.
    pub path: &'static str,
    /// Graph shape and operation.
    pub ladder: Ladder,
    /// Nodes, depth, dependents, or transient nodes according to the ladder.
    pub scale: usize,
    /// Reviewed digest of the final graph in stable node order.
    pub expected_graph_digest: u64,
    /// Reviewed digest of the final reverse-dependency index.
    pub expected_reverse_index_digest: u64,
    /// Reviewed digest of all semantic result evidence.
    pub expected_output_digest: u64,
}

/// The 17 currently representable Q2a graph-build cases.
///
/// The planned diamond row is intentionally absent: every current procedural
/// descriptor reports at most one dependency, so constructing a real diamond
/// would require a fake benchmark-only descriptor.
pub const CASES: [GraphBuildCase; 17] = [
    case(
        "graph/build/independent-planes-v1/1/default-v1",
        Ladder::Independent,
        1,
    ),
    case(
        "graph/build/independent-planes-v1/10/default-v1",
        Ladder::Independent,
        10,
    ),
    case(
        "graph/build/independent-planes-v1/100/default-v1",
        Ladder::Independent,
        100,
    ),
    case(
        "graph/build/independent-planes-v1/1000/default-v1",
        Ladder::Independent,
        1_000,
    ),
    case(
        "graph/build/independent-planes-v1/10000/default-v1",
        Ladder::Independent,
        10_000,
    ),
    case("graph/build/offset-chain-v1/1/default-v1", Ladder::Chain, 1),
    case(
        "graph/build/offset-chain-v1/10/default-v1",
        Ladder::Chain,
        10,
    ),
    case(
        "graph/build/offset-chain-v1/100/default-v1",
        Ladder::Chain,
        100,
    ),
    case(
        "graph/build/offset-chain-v1/1000/default-v1",
        Ladder::Chain,
        1_000,
    ),
    case(
        "graph/build/shared-offset-basis-v1/1/default-v1",
        Ladder::Fanout,
        1,
    ),
    case(
        "graph/build/shared-offset-basis-v1/10/default-v1",
        Ladder::Fanout,
        10,
    ),
    case(
        "graph/build/shared-offset-basis-v1/100/default-v1",
        Ladder::Fanout,
        100,
    ),
    case(
        "graph/build/shared-offset-basis-v1/1000/default-v1",
        Ladder::Fanout,
        1_000,
    ),
    case(
        "graph/rollback/dependent-chain-v1/1/rejected-v1",
        Ladder::Rollback,
        1,
    ),
    case(
        "graph/rollback/dependent-chain-v1/10/rejected-v1",
        Ladder::Rollback,
        10,
    ),
    case(
        "graph/rollback/dependent-chain-v1/100/rejected-v1",
        Ladder::Rollback,
        100,
    ),
    case(
        "graph/rollback/dependent-chain-v1/1000/rejected-v1",
        Ladder::Rollback,
        1_000,
    ),
];

const fn case(path: &'static str, ladder: Ladder, scale: usize) -> GraphBuildCase {
    let (expected_graph_digest, expected_reverse_index_digest, expected_output_digest) =
        match (ladder, scale) {
            (Ladder::Independent, 1) => (
                0x5c41_4503_e372_ce9b,
                0xa007_3203_cc9c_feb4,
                0x5c97_6e02_709a_9723,
            ),
            (Ladder::Independent, 10) => (
                0xb422_a04c_a195_f688,
                0x37b1_9f46_62c0_705e,
                0xb9c1_9292_2591_5cad,
            ),
            (Ladder::Independent, 100) => (
                0x2fec_b6a8_e55a_1843,
                0x7b34_c7ff_2f41_43f1,
                0xf9b4_eaf5_4768_c8ee,
            ),
            (Ladder::Independent, 1_000) => (
                0x6db7_056d_f18d_982a,
                0xb118_8b03_54e2_e48c,
                0x8f63_44a5_24db_609d,
            ),
            (Ladder::Independent, 10_000) => (
                0x0554_149c_f309_e6ce,
                0xc78b_7b06_64e6_f5a8,
                0x6ad7_6ede_b73d_bbdb,
            ),
            (Ladder::Chain, 1) | (Ladder::Fanout, 1) => (
                0x6a06_790f_bc13_37bd,
                0xe10d_a8c2_67f6_0616,
                0xc9c2_ece5_e620_7867,
            ),
            (Ladder::Chain, 10) => (
                0xe6e1_0cc1_2755_8f99,
                0x471d_b684_3140_01fe,
                0x8a2c_2e80_eab4_193c,
            ),
            (Ladder::Chain, 100) => (
                0x3953_b454_10da_73b7,
                0x8419_f106_b33c_76d0,
                0x50d9_fedc_a303_c57f,
            ),
            (Ladder::Chain, 1_000) => (
                0x5e94_64ea_140c_5b57,
                0xcc7e_6367_42c1_c33d,
                0xe6b9_7295_4665_6269,
            ),
            (Ladder::Fanout, 10) => (
                0x0a28_1e1f_4f83_d9a8,
                0xda26_9a8b_ba2d_5d34,
                0xb007_3103_3276_14f5,
            ),
            (Ladder::Fanout, 100) => (
                0xe73f_8894_acf9_9c63,
                0x520f_ca0b_5739_4374,
                0x3500_aa14_82d3_5fcd,
            ),
            (Ladder::Fanout, 1_000) => (
                0x68e8_6a3e_a2a4_27bb,
                0x3eba_19f1_6511_4864,
                0xe814_f105_1234_775c,
            ),
            (Ladder::Rollback, 1) => (
                0x5c41_4503_e372_ce9b,
                0xa007_3203_cc9c_feb4,
                0xc4cb_e324_74e6_5e74,
            ),
            (Ladder::Rollback, 10) => (
                0x5c41_4503_e372_ce9b,
                0xa007_3203_cc9c_feb4,
                0x0d41_a599_8403_c0b3,
            ),
            (Ladder::Rollback, 100) => (
                0x5c41_4503_e372_ce9b,
                0xa007_3203_cc9c_feb4,
                0x0158_a68e_d000_497d,
            ),
            (Ladder::Rollback, 1_000) => (
                0x5c41_4503_e372_ce9b,
                0xa007_3203_cc9c_feb4,
                0xf4b5_db24_7d34_b4c5,
            ),
            _ => (0, 0, 0),
        };
    GraphBuildCase {
        path,
        ladder,
        scale,
        expected_graph_digest,
        expected_reverse_index_digest,
        expected_output_digest,
    }
}

/// Fully prepared deterministic input. Policy data and rollback control state
/// are constructed outside the timed graph operation.
pub struct GraphBuildFixture {
    case: GraphBuildCase,
    planes: Box<[Plane]>,
    rollback_basis: Option<GeometryGraph>,
}

impl GraphBuildFixture {
    /// Prepare one registered case.
    pub fn new(case: GraphBuildCase) -> Self {
        assert!(case.scale > 0);
        let plane_count = if case.ladder == Ladder::Independent {
            case.scale
        } else {
            1
        };
        let planes: Box<[_]> = (0..plane_count).map(plane).collect();
        let rollback_basis = (case.ladder == Ladder::Rollback).then(|| {
            let mut graph = GeometryGraph::new();
            graph
                .insert_surface(planes[0])
                .expect("rollback basis must be valid");
            graph
        });
        Self {
            case,
            planes,
            rollback_basis,
        }
    }

    /// Execute one sample, timing only graph mutation and dependency-index work.
    pub fn measure_once(&self) -> (Duration, GraphBuildResult) {
        match self.case.ladder {
            Ladder::Rollback => self.measure_rollback(),
            ladder => self.measure_construction(ladder),
        }
    }

    fn measure_construction(&self, ladder: Ladder) -> (Duration, GraphBuildResult) {
        let mut graph = GeometryGraph::new();
        let before_observation = graph.benchmark_observation();
        let started = std::time::Instant::now();
        match ladder {
            Ladder::Independent => {
                for &plane in &self.planes {
                    graph
                        .insert_surface(plane)
                        .expect("plane descriptor must be valid");
                }
            }
            Ladder::Chain => {
                let basis = graph
                    .insert_surface(self.planes[0])
                    .expect("chain basis must be valid");
                let mut dependency = basis;
                for ordinal in 0..self.case.scale {
                    dependency = graph
                        .insert_surface(OffsetSurfaceDescriptor::new(
                            dependency,
                            offset_distance(ordinal),
                        ))
                        .expect("chain descriptor must be valid");
                }
            }
            Ladder::Fanout => {
                let basis = graph
                    .insert_surface(self.planes[0])
                    .expect("fanout basis must be valid");
                for ordinal in 0..self.case.scale {
                    graph
                        .insert_surface(OffsetSurfaceDescriptor::new(
                            basis,
                            offset_distance(ordinal),
                        ))
                        .expect("fanout descriptor must be valid");
                }
            }
            Ladder::Rollback => unreachable!("rollback has a prepared control graph"),
        }
        let elapsed = started.elapsed();
        let observation = graph.benchmark_observation().since(before_observation);
        (elapsed, inspect(graph, observation, false, false, None))
    }

    fn measure_rollback(&self) -> (Duration, GraphBuildResult) {
        let mut graph = self
            .rollback_basis
            .as_ref()
            .expect("rollback fixture has a basis")
            .clone();
        let before_graph_digest = graph_digest(&graph);
        let before_reverse_index_digest = reverse_index_digest(&graph);
        let before_observation = graph.benchmark_observation();
        let basis = match graph.geometry().next().expect("prepared basis") {
            GeometryRef::Surface(handle) => handle,
            _ => unreachable!("rollback basis is a surface"),
        };

        let started = std::time::Instant::now();
        graph.begin_undo_frame();
        let mut dependency = basis;
        for ordinal in 0..self.case.scale {
            dependency = graph
                .insert_surface(OffsetSurfaceDescriptor::new(
                    dependency,
                    offset_distance(ordinal),
                ))
                .expect("transient chain descriptor must be valid");
        }
        let rejected = graph
            .insert_surface(OffsetSurfaceDescriptor::new(dependency, f64::NAN))
            .is_err();
        graph
            .rollback_undo_frame()
            .expect("graph rollback must restore the prepared basis");
        let elapsed = started.elapsed();

        let observation = graph.benchmark_observation().since(before_observation);
        (
            elapsed,
            inspect(
                graph,
                observation,
                rejected,
                true,
                Some((before_graph_digest, before_reverse_index_digest)),
            ),
        )
    }
}

/// Semantic counters and deterministic graph evidence from one sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GraphBuildResult {
    /// Final live nodes.
    pub nodes: usize,
    /// Final direct dependency edges.
    pub dependency_edges: usize,
    /// Nodes registered during the timed operation.
    pub registered_nodes: usize,
    /// Dependency edges registered during the timed operation.
    pub registered_dependency_edges: usize,
    /// Reverse-index key plus edge registrations during the timed operation.
    pub reverse_index_updates: usize,
    /// Complete geometry orders rebuilt during the timed operation.
    pub full_order_rebuilds: usize,
    /// Whether repeated graph iteration produced the same exact order.
    pub stable_order: bool,
    /// Whether the deliberately invalid dependent was rejected.
    pub rejected: bool,
    /// Whether an undo frame was rolled back.
    pub rolled_back: bool,
    /// Stable semantic graph digest.
    pub graph_digest: u64,
    /// Stable reverse-dependency index digest.
    pub reverse_index_digest: u64,
    /// Pre-operation graph digest for rollback cases.
    pub before_graph_digest: Option<u64>,
    /// Pre-operation reverse-index digest for rollback cases.
    pub before_reverse_index_digest: Option<u64>,
}

impl GraphBuildResult {
    /// Stable digest over counters and correctness evidence.
    pub fn output_digest(self) -> u64 {
        let mut digest = ResultHasher::new();
        digest.tag(0x72);
        digest.count(self.nodes);
        digest.count(self.dependency_edges);
        digest.count(self.registered_nodes);
        digest.count(self.registered_dependency_edges);
        digest.count(self.reverse_index_updates);
        digest.count(self.full_order_rebuilds);
        digest.boolean(self.stable_order);
        digest.boolean(self.rejected);
        digest.boolean(self.rolled_back);
        digest.u64(self.graph_digest);
        digest.u64(self.reverse_index_digest);
        digest.optional_u64(self.before_graph_digest);
        digest.optional_u64(self.before_reverse_index_digest);
        digest.finish()
    }
}

/// Verify exact counters and reviewed digests before accepting a sample.
pub fn verify(case: GraphBuildCase, result: GraphBuildResult) {
    let (nodes, edges, registered_nodes, registered_edges, rebuilds) = match case.ladder {
        Ladder::Independent => (case.scale, 0, case.scale, 0, case.scale),
        Ladder::Chain | Ladder::Fanout => (
            case.scale + 1,
            case.scale,
            case.scale + 1,
            case.scale,
            case.scale + 1,
        ),
        Ladder::Rollback => (1, 0, case.scale, case.scale, case.scale),
    };
    assert_eq!(result.nodes, nodes);
    assert_eq!(result.dependency_edges, edges);
    assert_eq!(result.registered_nodes, registered_nodes);
    assert_eq!(result.registered_dependency_edges, registered_edges);
    assert_eq!(
        result.reverse_index_updates,
        registered_nodes + registered_edges
    );
    assert_eq!(result.full_order_rebuilds, rebuilds);
    assert!(result.stable_order);
    assert_eq!(result.rejected, case.ladder == Ladder::Rollback);
    assert_eq!(result.rolled_back, case.ladder == Ladder::Rollback);
    if case.ladder == Ladder::Rollback {
        assert_eq!(result.before_graph_digest, Some(result.graph_digest));
        assert_eq!(
            result.before_reverse_index_digest,
            Some(result.reverse_index_digest)
        );
    } else {
        assert_eq!(result.before_graph_digest, None);
        assert_eq!(result.before_reverse_index_digest, None);
    }
    assert_ne!(case.expected_graph_digest, 0);
    assert_ne!(case.expected_reverse_index_digest, 0);
    assert_ne!(case.expected_output_digest, 0);
    assert_eq!(result.graph_digest, case.expected_graph_digest);
    assert_eq!(
        result.reverse_index_digest,
        case.expected_reverse_index_digest
    );
    assert_eq!(result.output_digest(), case.expected_output_digest);
}

fn inspect(
    graph: GeometryGraph,
    observation: GraphBuildObservation,
    rejected: bool,
    rolled_back: bool,
    before: Option<(u64, u64)>,
) -> GraphBuildResult {
    graph.validate().expect("constructed graph must validate");
    let order: Vec<_> = graph.geometry().collect();
    let stable_order = graph.geometry().eq(order.iter().copied());
    let dependency_edges = order
        .iter()
        .copied()
        .map(|geometry| {
            graph
                .direct_dependencies(geometry)
                .expect("live geometry")
                .len()
        })
        .sum();
    let registered_nodes = observation.registered_nodes();
    let registered_dependency_edges = observation.registered_dependency_edges();
    let (before_graph_digest, before_reverse_index_digest) = before
        .map_or((None, None), |(graph, reverse)| {
            (Some(graph), Some(reverse))
        });
    GraphBuildResult {
        nodes: graph.len(),
        dependency_edges,
        registered_nodes,
        registered_dependency_edges,
        reverse_index_updates: registered_nodes + registered_dependency_edges,
        full_order_rebuilds: observation.full_order_rebuilds(),
        stable_order,
        rejected,
        rolled_back,
        graph_digest: graph_digest(&graph),
        reverse_index_digest: reverse_index_digest(&graph),
        before_graph_digest,
        before_reverse_index_digest,
    }
}

fn plane(ordinal: usize) -> Plane {
    let frame = Frame::from_z(
        Point3::new(ordinal as f64, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .expect("fixture frame must be valid");
    Plane::new(frame)
}

fn offset_distance(ordinal: usize) -> f64 {
    (ordinal + 1) as f64 * 0.125
}

fn graph_digest(graph: &GeometryGraph) -> u64 {
    let order: Vec<_> = graph.geometry().collect();
    let mut digest = ResultHasher::new();
    digest.tag(0x47);
    digest.count(order.len());
    for (ordinal, geometry) in order.iter().copied().enumerate() {
        digest.count(ordinal);
        match geometry {
            GeometryRef::Curve(handle) => {
                digest.tag(0);
                digest.bytes(
                    graph
                        .curve(handle)
                        .expect("live curve")
                        .class_key()
                        .as_str()
                        .as_bytes(),
                );
            }
            GeometryRef::Surface(handle) => {
                digest.tag(1);
                let descriptor = graph.surface(handle).expect("live surface");
                digest.bytes(descriptor.class_key().as_str().as_bytes());
                match descriptor {
                    SurfaceDescriptor::Plane(plane) => {
                        digest.tag(0);
                        let origin = plane.frame().origin();
                        digest.u64(origin.x.to_bits());
                        digest.u64(origin.y.to_bits());
                        digest.u64(origin.z.to_bits());
                    }
                    SurfaceDescriptor::Offset(offset) => {
                        digest.tag(1);
                        digest.u64(offset.signed_distance().to_bits());
                    }
                    _ => unreachable!("Q2a fixtures use planes and offsets only"),
                }
            }
            GeometryRef::Curve2d(handle) => {
                digest.tag(2);
                digest.bytes(
                    graph
                        .curve2d(handle)
                        .expect("live pcurve")
                        .class_key()
                        .as_str()
                        .as_bytes(),
                );
            }
        }
        let dependencies = graph
            .direct_dependencies(geometry)
            .expect("live geometry dependencies");
        digest.count(dependencies.len());
        for dependency in dependencies {
            digest.count(ordinal_of(&order, dependency));
        }
    }
    digest.finish()
}

fn reverse_index_digest(graph: &GeometryGraph) -> u64 {
    let order: Vec<_> = graph.geometry().collect();
    let mut digest = ResultHasher::new();
    digest.tag(0x52);
    digest.count(order.len());
    for (ordinal, geometry) in order.iter().copied().enumerate() {
        digest.count(ordinal);
        let dependents = graph
            .dependents(geometry)
            .expect("live geometry dependents");
        digest.count(dependents.len());
        for dependent in dependents {
            digest.count(ordinal_of(&order, dependent));
        }
    }
    digest.finish()
}

fn ordinal_of(order: &[GeometryRef], geometry: GeometryRef) -> usize {
    order
        .iter()
        .position(|candidate| *candidate == geometry)
        .expect("dependency identity belongs to graph order")
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

    fn optional_u64(&mut self, value: Option<u64>) {
        match value {
            Some(value) => {
                self.tag(1);
                self.u64(value);
            }
            None => self.tag(0),
        }
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
    fn registry_contains_exactly_17_unique_canonical_cases() {
        assert_eq!(CASES.len(), 17);
        let unique: BTreeSet<_> = CASES.iter().map(|case| case.path).collect();
        assert_eq!(unique.len(), CASES.len());
        for case in CASES {
            crate::validate_case_path(case.path).unwrap();
            assert_eq!(include_str!("../cases.json").matches(case.path).count(), 1);
        }
    }

    #[test]
    fn every_case_matches_its_reviewed_contract() {
        for case in CASES {
            let fixture = GraphBuildFixture::new(case);
            let result = fixture.measure_once().1;
            verify(case, result);
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
            .filter(|entry| entry["benchmark_target"] == "graph_build")
            .collect();
        assert_eq!(registered.len(), CASES.len());

        for case in CASES {
            let entry = registered
                .iter()
                .copied()
                .find(|entry| entry["path"] == case.path)
                .expect("Rust Q2a case must be registered");
            let (nodes, dependency_edges, registered_nodes, registered_edges, rebuilds) =
                match case.ladder {
                    Ladder::Independent => (case.scale, 0, case.scale, 0, case.scale),
                    Ladder::Chain | Ladder::Fanout => (
                        case.scale + 1,
                        case.scale,
                        case.scale + 1,
                        case.scale,
                        case.scale + 1,
                    ),
                    Ladder::Rollback => (1, 0, case.scale, case.scale, case.scale),
                };
            let counters = &entry["expected_result_counters"];
            assert_eq!(entry["fixture_version"], FIXTURE_VERSION);
            assert_eq!(entry["deterministic_seed"], FIXTURE_SEED);
            assert_eq!(counters["nodes"], nodes);
            assert_eq!(counters["dependency_edges"], dependency_edges);
            assert_eq!(counters["registered_nodes"], registered_nodes);
            assert_eq!(counters["registered_dependency_edges"], registered_edges);
            assert_eq!(
                counters["reverse_index_updates"],
                registered_nodes + registered_edges
            );
            assert_eq!(counters["full_order_rebuilds"], rebuilds);
            assert_eq!(counters["stable_order"], true);
            assert_eq!(
                counters["graph_digest"],
                format!("{:016x}", case.expected_graph_digest)
            );
            assert_eq!(
                counters["reverse_index_digest"],
                format!("{:016x}", case.expected_reverse_index_digest)
            );
            assert_eq!(
                counters["output_digest"],
                format!("{:016x}", case.expected_output_digest)
            );
        }
    }

    #[test]
    fn rollback_restores_exact_graph_and_reverse_index_state() {
        for scale in [1, 10, 100, 1_000] {
            let case = GraphBuildCase {
                path: "graph/rollback/dependent-chain-v1/1/rejected-v1",
                ladder: Ladder::Rollback,
                scale,
                expected_graph_digest: 0,
                expected_reverse_index_digest: 0,
                expected_output_digest: 0,
            };
            let fixture = GraphBuildFixture::new(case);
            let result = fixture.measure_once().1;
            assert!(result.rejected);
            assert_eq!(result.before_graph_digest, Some(result.graph_digest));
            assert_eq!(
                result.before_reverse_index_digest,
                Some(result.reverse_index_digest)
            );
        }
    }
}
