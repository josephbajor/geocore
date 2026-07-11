//! Deterministic Q3 analytic body-tessellation fixtures and evidence.

use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};
use ktopo::btess::{BodyMesh, TessOptions, check_watertight, signed_volume, tessellate_body};
use ktopo::entity::{BodyId, EdgeId, FaceId};
use ktopo::make;
use ktopo::store::Store;

/// Fixture identity shared by the first Q3 analytic-solid slice.
pub const FIXTURE_VERSION: &str = "body-tessellation.v1";
/// Deterministic fixture seed (construction itself is not randomized).
pub const FIXTURE_SEED: u64 = 0x5154_4553_5300_0003;

/// Analytic closed solid represented by one Q3 case.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Primitive {
    /// Six planar faces and straight edges.
    Block,
    /// Periodic side face with two closed seam boundaries.
    Cylinder,
    /// Periodic conical side face with two closed seam boundaries.
    Cone,
    /// One closed face with a periodic seam and two parameter poles.
    Sphere,
    /// One doubly periodic closed face.
    Torus,
}

/// Stable Q3 case definition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyTessellationCase {
    /// Five-segment benchmark path.
    pub path: &'static str,
    /// Analytic fixture kind.
    pub primitive: Primitive,
    /// Chordal tessellation tolerance.
    pub chord_tol: f64,
    /// Reviewed complete semantic output digest.
    pub expected_output_digest: u64,
    /// Reviewed complete mesh digest.
    pub expected_mesh_digest: u64,
    /// Reviewed output vertex count.
    pub expected_mesh_vertices: usize,
    /// Reviewed output triangle count.
    pub expected_mesh_triangles: usize,
    /// Reviewed source face count.
    pub expected_source_faces: usize,
    /// Reviewed source edge count.
    pub expected_source_edges: usize,
    /// Reviewed source vertex count.
    pub expected_source_vertices: usize,
    /// Reviewed face-range count.
    pub expected_face_ranges: usize,
    /// Reviewed edge-polyline count.
    pub expected_edge_polylines: usize,
}

/// Ten analytic-solid cases: five primitives at coarse and fine tolerances.
pub const CASES: [BodyTessellationCase; 10] = [
    case(
        "topology/body-tessellation/block-v1/1/chord-1e-2-v1",
        Primitive::Block,
        1.0e-2,
        8,
        12,
        0x3773_8ea0_abf2_68a7,
        0x2b93_7370_9976_5cda,
    ),
    case(
        "topology/body-tessellation/block-v1/1/chord-1e-3-v1",
        Primitive::Block,
        1.0e-3,
        8,
        12,
        0x3773_8ea0_abf2_68a7,
        0x2b93_7370_9976_5cda,
    ),
    case(
        "topology/body-tessellation/cylinder-v1/1/chord-1e-2-v1",
        Primitive::Cylinder,
        1.0e-2,
        2_913,
        5_822,
        0x3047_4187_c9d8_a9ce,
        0x1383_c6b7_e587_30b3,
    ),
    case(
        "topology/body-tessellation/cylinder-v1/1/chord-1e-3-v1",
        Primitive::Cylinder,
        1.0e-3,
        85_683,
        171_362,
        0xc18e_8ba3_3c72_5d33,
        0x1300_a8c1_59d7_ab48,
    ),
    case(
        "topology/body-tessellation/cone-v1/1/chord-1e-2-v1",
        Primitive::Cone,
        1.0e-2,
        2_737,
        5_470,
        0x2ce0_b59e_91e2_2400,
        0x7a7c_465b_655f_93c0,
    ),
    case(
        "topology/body-tessellation/cone-v1/1/chord-1e-3-v1",
        Primitive::Cone,
        1.0e-3,
        54_432,
        108_860,
        0x4159_97ae_b0ba_bc82,
        0xb06b_9683_b803_4a21,
    ),
    case(
        "topology/body-tessellation/sphere-v1/1/chord-1e-2-v1",
        Primitive::Sphere,
        1.0e-2,
        2_704,
        5_404,
        0x79f4_2a54_6c49_f36f,
        0x1fd7_f010_353b_6581,
    ),
    case(
        "topology/body-tessellation/sphere-v1/1/chord-1e-3-v1",
        Primitive::Sphere,
        1.0e-3,
        75_430,
        150_856,
        0xf827_bff6_d901_87a7,
        0xc0ef_9392_fad8_0d52,
    ),
    case(
        "topology/body-tessellation/torus-v1/1/chord-1e-2-v1",
        Primitive::Torus,
        1.0e-2,
        11_340,
        22_680,
        0xbec9_d49d_9830_dc7e,
        0x4b23_8374_3df8_8c46,
    ),
    case(
        "topology/body-tessellation/torus-v1/1/chord-1e-3-v1",
        Primitive::Torus,
        1.0e-3,
        148_178,
        296_356,
        0x39d6_eb3f_0319_b7f7,
        0x1f8d_af00_17ea_2cb6,
    ),
];

const fn case(
    path: &'static str,
    primitive: Primitive,
    chord_tol: f64,
    expected_mesh_vertices: usize,
    expected_mesh_triangles: usize,
    expected_mesh_digest: u64,
    expected_output_digest: u64,
) -> BodyTessellationCase {
    let (source_faces, source_edges, source_vertices) = match primitive {
        Primitive::Block => (6, 12, 8),
        Primitive::Cylinder | Primitive::Cone => (3, 2, 0),
        Primitive::Sphere | Primitive::Torus => (1, 0, 0),
    };
    BodyTessellationCase {
        path,
        primitive,
        chord_tol,
        expected_output_digest,
        expected_mesh_digest,
        expected_mesh_vertices,
        expected_mesh_triangles,
        expected_source_faces: source_faces,
        expected_source_edges: source_edges,
        expected_source_vertices: source_vertices,
        expected_face_ranges: source_faces,
        expected_edge_polylines: source_edges,
    }
}

/// Fully constructed immutable Q3 input. Construction is never measured.
pub struct BodyTessellationFixture {
    store: Store,
    body: BodyId,
    exact_volume: f64,
    minimum_volume_ratio: f64,
    source_faces: usize,
    source_edges: usize,
    source_vertices: usize,
    expected_faces: Box<[FaceId]>,
    expected_edges: Box<[EdgeId]>,
}

impl BodyTessellationFixture {
    /// Tessellate once through the ordinary public body entry point.
    pub fn tessellate(&self, chord_tol: f64) -> BodyMesh {
        tessellate_body(
            &self.store,
            self.body,
            &TessOptions {
                chord_tol,
                max_edge_len: None,
            },
        )
        .expect("reviewed Q3 fixture must tessellate")
    }

    /// Validate one mesh and reduce it to stable semantic evidence.
    pub fn evidence(&self, mesh: &BodyMesh) -> BodyTessellationEvidence {
        let positions_finite = mesh
            .positions
            .iter()
            .all(|point| point.x.is_finite() && point.y.is_finite() && point.z.is_finite());
        let triangle_indices_valid = mesh
            .triangles
            .iter()
            .flatten()
            .all(|&index| (index as usize) < mesh.positions.len());
        let edge_indices_valid = mesh
            .edge_polylines
            .iter()
            .flat_map(|(_, indices)| indices)
            .all(|&index| (index as usize) < mesh.positions.len());
        let face_ranges_valid =
            mesh.face_ranges
                .iter()
                .try_fold(0usize, |expected_start, (_, range)| {
                    (range.start == expected_start
                        && range.end >= range.start
                        && range.end <= mesh.triangles.len())
                    .then_some(range.end)
                })
                == Some(mesh.triangles.len());
        let owner_mapping_valid = mesh.face_ranges.len() == self.expected_faces.len()
            && mesh
                .face_ranges
                .iter()
                .zip(&self.expected_faces)
                .all(|((owner, _), expected)| owner == expected)
            && mesh.edge_polylines.len() == self.expected_edges.len()
            && mesh
                .edge_polylines
                .iter()
                .zip(&self.expected_edges)
                .all(|((owner, _), expected)| owner == expected);
        let watertight = check_watertight(mesh).is_empty();
        let volume = signed_volume(mesh);
        let outward = volume.is_finite() && volume > 0.0;
        let volume_within_tolerance = outward
            && volume >= self.exact_volume * self.minimum_volume_ratio
            && volume <= self.exact_volume * (1.0 + 1.0e-9);
        let mesh_digest = self.mesh_digest(mesh);
        let mut evidence = BodyTessellationEvidence {
            source_faces: self.source_faces,
            source_edges: self.source_edges,
            source_vertices: self.source_vertices,
            mesh_vertices: mesh.positions.len(),
            mesh_triangles: mesh.triangles.len(),
            face_ranges: mesh.face_ranges.len(),
            edge_polylines: mesh.edge_polylines.len(),
            positions_finite,
            indices_valid: triangle_indices_valid && edge_indices_valid && face_ranges_valid,
            owner_mapping_valid,
            watertight,
            outward,
            volume_within_tolerance,
            mesh_digest,
            output_digest: 0,
        };
        evidence.output_digest = evidence.digest();
        evidence
    }

    fn mesh_digest(&self, mesh: &BodyMesh) -> u64 {
        fn ordinal<T: PartialEq>(owners: &[T], owner: &T) -> Option<usize> {
            owners.iter().position(|expected| expected == owner)
        }

        let mut digest = StableHasher::new();
        digest.tag(0x80);
        digest.count(mesh.positions.len());
        for point in &mesh.positions {
            digest.f64(point.x);
            digest.f64(point.y);
            digest.f64(point.z);
        }
        digest.count(mesh.triangles.len());
        for triangle in &mesh.triangles {
            for &index in triangle {
                digest.u64(u64::from(index));
            }
        }
        digest.count(mesh.face_ranges.len());
        for (owner, range) in &mesh.face_ranges {
            digest.ordinal(ordinal(&self.expected_faces, owner));
            digest.count(range.start);
            digest.count(range.end);
        }
        digest.count(mesh.edge_polylines.len());
        for (owner, polyline) in &mesh.edge_polylines {
            digest.ordinal(ordinal(&self.expected_edges, owner));
            digest.count(polyline.len());
            for &index in polyline {
                digest.u64(u64::from(index));
            }
        }
        digest.finish()
    }
}

/// Stable counters and correctness evidence for one Q3 output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BodyTessellationEvidence {
    /// Source face count.
    pub source_faces: usize,
    /// Source edge count.
    pub source_edges: usize,
    /// Source vertex count.
    pub source_vertices: usize,
    /// Output position count.
    pub mesh_vertices: usize,
    /// Output triangle count.
    pub mesh_triangles: usize,
    /// Face range count.
    pub face_ranges: usize,
    /// Topological edge-polyline count.
    pub edge_polylines: usize,
    /// Whether every output coordinate is finite.
    pub positions_finite: bool,
    /// Whether triangle, edge-polyline, and face-range indices are valid.
    pub indices_valid: bool,
    /// Whether face and edge outputs retain their exact source-owner order.
    pub owner_mapping_valid: bool,
    /// Whether the closed-solid mesh passes the complete watertightness audit.
    pub watertight: bool,
    /// Whether signed volume proves outward orientation.
    pub outward: bool,
    /// Whether signed volume remains within the fixture's reviewed error bound.
    pub volume_within_tolerance: bool,
    /// Stable digest of every mesh coordinate, triangle, range, and polyline.
    pub mesh_digest: u64,
    /// Stable digest of all source/output counters and correctness evidence.
    pub output_digest: u64,
}

impl BodyTessellationEvidence {
    fn digest(&self) -> u64 {
        let mut digest = StableHasher::new();
        digest.tag(0x81);
        digest.count(self.source_faces);
        digest.count(self.source_edges);
        digest.count(self.source_vertices);
        digest.count(self.mesh_vertices);
        digest.count(self.mesh_triangles);
        digest.count(self.face_ranges);
        digest.count(self.edge_polylines);
        digest.boolean(self.positions_finite);
        digest.boolean(self.indices_valid);
        digest.boolean(self.owner_mapping_valid);
        digest.boolean(self.watertight);
        digest.boolean(self.outward);
        digest.boolean(self.volume_within_tolerance);
        digest.u64(self.mesh_digest);
        digest.finish()
    }
}

/// Construct the immutable input for one case.
pub fn fixture(case: BodyTessellationCase) -> BodyTessellationFixture {
    let frame = Frame::new(
        Point3::new(0.3, -1.2, 2.1),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .expect("valid Q3 fixture frame");
    let mut store = Store::new();
    let (body, exact_volume, minimum_volume_ratio) = match case.primitive {
        Primitive::Block => (
            make::block(&mut store, &frame, [2.0, 3.0, 4.0]).expect("valid block fixture"),
            24.0,
            1.0 - 1.0e-12,
        ),
        Primitive::Cylinder => (
            make::cylinder(&mut store, &frame, 1.3, 2.0).expect("valid cylinder fixture"),
            core::f64::consts::PI * 1.3 * 1.3 * 2.0,
            0.98,
        ),
        Primitive::Cone => (
            make::cone(&mut store, &frame, 1.5, 0.6, 2.0).expect("valid cone fixture"),
            core::f64::consts::PI * 2.0 * (1.5 * 1.5 + 1.5 * 0.6 + 0.6 * 0.6) / 3.0,
            0.98,
        ),
        Primitive::Sphere => (
            make::sphere(&mut store, &frame, 1.1).expect("valid sphere fixture"),
            4.0 / 3.0 * core::f64::consts::PI * 1.1_f64.powi(3),
            0.98,
        ),
        Primitive::Torus => (
            make::torus(&mut store, &frame, 2.0, 0.7).expect("valid torus fixture"),
            2.0 * core::f64::consts::PI * core::f64::consts::PI * 2.0 * 0.7 * 0.7,
            0.98,
        ),
    };
    let expected_faces = store.faces_of_body(body).expect("valid body");
    let expected_edges = store.edges_of_body(body).expect("valid body");
    let source_faces = expected_faces.len();
    let source_edges = expected_edges.len();
    let source_vertices = store.vertices_of_body(body).expect("valid body").len();
    BodyTessellationFixture {
        store,
        body,
        exact_volume,
        minimum_volume_ratio,
        source_faces,
        source_edges,
        source_vertices,
        expected_faces: expected_faces.into_boxed_slice(),
        expected_edges: expected_edges.into_boxed_slice(),
    }
}

/// Verify exact reviewed evidence for one case.
pub fn verify(case: BodyTessellationCase, evidence: BodyTessellationEvidence) {
    assert!(evidence.positions_finite);
    assert!(evidence.indices_valid);
    assert!(evidence.owner_mapping_valid);
    assert!(evidence.watertight);
    assert!(evidence.outward);
    assert!(evidence.volume_within_tolerance);
    assert_ne!(case.expected_mesh_digest, 0);
    assert_ne!(case.expected_output_digest, 0);
    assert_eq!(evidence.mesh_vertices, case.expected_mesh_vertices);
    assert_eq!(evidence.mesh_triangles, case.expected_mesh_triangles);
    assert_eq!(evidence.source_faces, case.expected_source_faces);
    assert_eq!(evidence.source_edges, case.expected_source_edges);
    assert_eq!(evidence.source_vertices, case.expected_source_vertices);
    assert_eq!(evidence.face_ranges, case.expected_face_ranges);
    assert_eq!(evidence.edge_polylines, case.expected_edge_polylines);
    assert_eq!(evidence.mesh_digest, case.expected_mesh_digest);
    assert_eq!(evidence.output_digest, case.expected_output_digest);
}

struct StableHasher(u64);

impl StableHasher {
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

    fn ordinal(&mut self, value: Option<usize>) {
        match value {
            Some(value) => {
                self.tag(1);
                self.count(value);
            }
            None => self.tag(0),
        }
    }

    fn f64(&mut self, value: f64) {
        self.u64(value.to_bits());
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
        }
    }

    #[test]
    fn json_registry_matches_every_rust_case_and_reviewed_counter() {
        let manifest: serde_json::Value =
            serde_json::from_str(include_str!("../cases.json")).unwrap();
        let entries = manifest["cases"].as_array().unwrap();
        let q3_entries: Vec<_> = entries
            .iter()
            .filter(|entry| entry["benchmark_target"] == "body_tessellation")
            .collect();
        assert_eq!(q3_entries.len(), CASES.len());
        for case in CASES {
            let matches: Vec<_> = q3_entries
                .iter()
                .copied()
                .filter(|entry| entry["path"] == case.path)
                .collect();
            assert_eq!(matches.len(), 1, "registry mismatch for {}", case.path);
            let entry = matches[0];
            assert_eq!(entry["fixture_version"], FIXTURE_VERSION);
            assert_eq!(entry["deterministic_seed"].as_u64(), Some(FIXTURE_SEED));
            assert_eq!(entry["size_parameters"]["elements"].as_u64(), Some(1));
            assert_eq!(entry["size_parameters"]["bodies"].as_u64(), Some(1));
            assert_eq!(
                entry["tolerances"]["chord_tol"].as_f64(),
                Some(case.chord_tol)
            );
            assert_eq!(entry["policy_values"]["max_edge_len"], "unbounded");
            assert_eq!(entry["policy_values"]["validation"], "closed-solid");

            let counters = &entry["expected_result_counters"];
            assert_eq!(
                counters["source_faces"].as_u64(),
                Some(case.expected_source_faces as u64)
            );
            assert_eq!(
                counters["source_edges"].as_u64(),
                Some(case.expected_source_edges as u64)
            );
            assert_eq!(
                counters["source_vertices"].as_u64(),
                Some(case.expected_source_vertices as u64)
            );
            assert_eq!(
                counters["mesh_vertices"].as_u64(),
                Some(case.expected_mesh_vertices as u64)
            );
            assert_eq!(
                counters["mesh_triangles"].as_u64(),
                Some(case.expected_mesh_triangles as u64)
            );
            assert_eq!(
                counters["face_ranges"].as_u64(),
                Some(case.expected_face_ranges as u64)
            );
            assert_eq!(
                counters["edge_polylines"].as_u64(),
                Some(case.expected_edge_polylines as u64)
            );
            for field in [
                "positions_finite",
                "indices_valid",
                "owner_mapping_valid",
                "watertight",
                "outward",
                "volume_within_tolerance",
            ] {
                assert_eq!(counters[field].as_bool(), Some(true), "{field}");
            }
            assert_eq!(
                counters["mesh_digest"].as_str(),
                Some(format!("{:016x}", case.expected_mesh_digest).as_str())
            );
            assert_eq!(
                counters["output_digest"].as_str(),
                Some(format!("{:016x}", case.expected_output_digest).as_str())
            );
        }
    }

    #[test]
    fn every_case_is_bitwise_repeatable_and_matches_reviewed_evidence() {
        for case in CASES {
            let fixture = fixture(case);
            let first = fixture.evidence(&fixture.tessellate(case.chord_tol));
            let second = fixture.evidence(&fixture.tessellate(case.chord_tol));
            assert_eq!(first, second, "repeatability drift for {}", case.path);
            verify(case, first);
        }
    }

    #[test]
    fn reversed_face_range_is_rejected() {
        let case = CASES[0];
        let fixture = fixture(case);
        let mut mesh = fixture.tessellate(case.chord_tol);
        assert!(mesh.face_ranges.len() > 1);
        let start = mesh.face_ranges[1].1.start;
        assert!(start > 0);
        mesh.face_ranges[1].1.end = start - 1;
        assert!(!fixture.evidence(&mesh).indices_valid);
    }

    #[test]
    fn wrong_and_duplicate_owner_mappings_are_rejected_and_digested() {
        let case = CASES[0];
        let fixture = fixture(case);
        let mesh = fixture.tessellate(case.chord_tol);
        let reviewed = fixture.evidence(&mesh);

        let mut duplicate_face = mesh.clone();
        duplicate_face.face_ranges[1].0 = duplicate_face.face_ranges[0].0;
        let duplicate_face = fixture.evidence(&duplicate_face);
        assert!(!duplicate_face.owner_mapping_valid);
        assert_ne!(duplicate_face.mesh_digest, reviewed.mesh_digest);

        let mut wrong_edge_order = mesh;
        wrong_edge_order.edge_polylines.swap(0, 1);
        let wrong_edge_order = fixture.evidence(&wrong_edge_order);
        assert!(!wrong_edge_order.owner_mapping_valid);
        assert_ne!(wrong_edge_order.mesh_digest, reviewed.mesh_digest);
    }
}
