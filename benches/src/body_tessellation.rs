//! Deterministic Q3 body-tessellation fixtures and evidence.

use kcore::operation::{
    AccountingMode, ExecutionPolicy, NumericalPolicy, OperationContext, OperationOutcome,
    OperationPolicyError, OperationReport, PolicyVersion, ResourceKind, SessionPolicy,
    SessionPrecision, StageId,
};
#[cfg(test)]
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::tess::{
    FACE_TESSELLATION_BOUNDARY_DEPTH, FACE_TESSELLATION_BOUNDARY_SPLITS,
    FACE_TESSELLATION_MESH_TRIANGLES, FACE_TESSELLATION_MESH_VERTICES,
    FACE_TESSELLATION_REFINEMENT_PASSES,
};
use kgeom::vec::{Point3, Vec3};
use ktopo::btess::{
    BODY_TESSELLATION_EDGE_DEPTH, BODY_TESSELLATION_EDGE_SPLITS,
    BODY_TESSELLATION_EDGE_STORAGE_ITEMS, BODY_TESSELLATION_ISO_ARC_DEPTH,
    BODY_TESSELLATION_ISO_ARC_SPLITS, BODY_TESSELLATION_MESH_VERTICES,
    BODY_TESSELLATION_PREPARED_PATCH_ITEMS, BODY_TESSELLATION_RETAINED_TRIANGLES,
    BODY_TESSELLATION_STRUCTURAL_ITEMS, BodyMesh, BodyTessellationBudgetProfile, TessOptions,
    TessellationError, check_watertight, signed_volume, tessellate_body_with_context,
};
use ktopo::entity::{BodyId, EdgeId, FaceId};
use ktopo::geom::{Curve2dGeom, SurfaceGeom};
use ktopo::make;
use ktopo::store::Store;

/// Fixture identity shared by the first Q3 analytic-solid slice.
pub const FIXTURE_VERSION: &str = "body-tessellation.v2";
/// Deterministic fixture seed (construction itself is not randomized).
pub const FIXTURE_SEED: u64 = 0x5154_4553_5300_0003;
/// Public entry point measured by the Q3 v2 contract.
pub const API_IDENTITY: &str = "tessellate_body_with_context";
/// Complete body-family defaults used by the Q3 v2 contract.
pub const PROFILE_IDENTITY: &str = "body-tessellation.compatibility-v1";
/// Deterministic execution policy used by the Q3 v2 contract.
pub const EXECUTION_IDENTITY: &str = "serial";
/// Canonical number of body, face, graph, and projection usage stages.
pub const USAGE_STAGE_COUNT: usize = 21;
/// Exact licensed-host-certified corpus identity used by the first Q3 NURBS slice.
pub const IMPORTED_NURBS_FACE_IDENTITY: &str =
    "solid_block_nurbs_face.x_t@onshape-cloud-2026-07-11";
/// SHA-256 pinned by `docs/oracle-certification.json` for the imported NURBS fixture.
pub const IMPORTED_NURBS_FACE_SHA256: &str =
    "410831b258864b3f55a221f329a49743b3863c664f1d2a53435f74a72ea5d9db";
/// Portable in-harness byte digest for the exact certified NURBS fixture copy.
pub const IMPORTED_NURBS_FACE_BYTE_DIGEST: u64 = 0x7aaf_75cc_f251_e6b9;
const IMPORTED_NURBS_FACE_BYTES: &[u8] =
    include_bytes!("../testdata/solid_block_nurbs_face.certified.x_t");
/// Exact licensed-host-certified corpus identity for the first tolerant-edge slice.
pub const IMPORTED_TOLERANT_EDGE_IDENTITY: &str =
    "solid_block_tolerant_edge.x_t@onshape-cloud-2026-07-11";
/// SHA-256 pinned by `docs/oracle-certification.json` for the tolerant-edge fixture.
pub const IMPORTED_TOLERANT_EDGE_SHA256: &str =
    "49e1c858c73200f82816b6c352b2a4e92b7af7d45e8f82f10e00abb6d4edf831";
/// Portable in-harness byte digest for the exact certified tolerant-edge fixture.
pub const IMPORTED_TOLERANT_EDGE_BYTE_DIGEST: u64 = 0x1483_6dc5_5d6d_8a71;
const IMPORTED_TOLERANT_EDGE_BYTES: &[u8] =
    include_bytes!("../../oracle/outbox/solid_block_tolerant_edge.x_t");

/// Closed solid represented by one Q3 case.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixtureKind {
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
    /// Certified X_T block with one exact planar B-surface face and no pcurves.
    ImportedNurbsFace,
    /// Certified X_T block with one curve-less tolerant edge and two NURBS pcurves.
    ImportedTolerantEdge,
}

/// Stable Q3 case definition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyTessellationCase {
    /// Five-segment benchmark path.
    pub path: &'static str,
    /// Fixture kind.
    pub fixture_kind: FixtureKind,
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
    /// Reviewed consumed values for all canonical usage stages.
    pub expected_usage: [u64; USAGE_STAGE_COUNT],
    /// Reviewed portable canonical usage-stage digest.
    pub expected_usage_stage_digest: u64,
}

/// Ten analytic-solid cases plus four certified imported corpus rows.
pub const CASES: [BodyTessellationCase; 14] = [
    case(
        "topology/body-tessellation/block-v2/1/chord-1e-2-v2",
        FixtureKind::Block,
        1.0e-2,
        8,
        12,
        0x3773_8ea0_abf2_68a7,
        0xd5d4_e021_7441_32d8,
    ),
    case(
        "topology/body-tessellation/block-v2/1/chord-1e-3-v2",
        FixtureKind::Block,
        1.0e-3,
        8,
        12,
        0x3773_8ea0_abf2_68a7,
        0xd5d4_e021_7441_32d8,
    ),
    case(
        "topology/body-tessellation/cylinder-v2/1/chord-1e-2-v2",
        FixtureKind::Cylinder,
        1.0e-2,
        2_913,
        5_822,
        0x3047_4187_c9d8_a9ce,
        0xb964_2497_18cf_75e6,
    ),
    case(
        "topology/body-tessellation/cylinder-v2/1/chord-1e-3-v2",
        FixtureKind::Cylinder,
        1.0e-3,
        85_683,
        171_362,
        0xc18e_8ba3_3c72_5d33,
        0x5f29_ed51_1dd0_7c8b,
    ),
    case(
        "topology/body-tessellation/cone-v2/1/chord-1e-2-v2",
        FixtureKind::Cone,
        1.0e-2,
        2_737,
        5_470,
        0x2ce0_b59e_91e2_2400,
        0xb602_3ef6_8ee4_c2ee,
    ),
    case(
        "topology/body-tessellation/cone-v2/1/chord-1e-3-v2",
        FixtureKind::Cone,
        1.0e-3,
        54_432,
        108_860,
        0x4159_97ae_b0ba_bc82,
        0x4266_bd85_9caf_2548,
    ),
    case(
        "topology/body-tessellation/sphere-v2/1/chord-1e-2-v2",
        FixtureKind::Sphere,
        1.0e-2,
        2_704,
        5_404,
        0x79f4_2a54_6c49_f36f,
        0x9396_9a9a_2e3e_2b9d,
    ),
    case(
        "topology/body-tessellation/sphere-v2/1/chord-1e-3-v2",
        FixtureKind::Sphere,
        1.0e-3,
        75_430,
        150_856,
        0xf827_bff6_d901_87a7,
        0x65a5_9f05_ace3_e62f,
    ),
    case(
        "topology/body-tessellation/torus-v2/1/chord-1e-2-v2",
        FixtureKind::Torus,
        1.0e-2,
        11_340,
        22_680,
        0xbec9_d49d_9830_dc7e,
        0x49b3_c184_bb15_572c,
    ),
    case(
        "topology/body-tessellation/torus-v2/1/chord-1e-3-v2",
        FixtureKind::Torus,
        1.0e-3,
        148_178,
        296_356,
        0x39d6_eb3f_0319_b7f7,
        0x9492_ef50_35aa_53ed,
    ),
    case(
        "topology/body-tessellation/imported-nurbs-face-v2/1/chord-1e-2-v2",
        FixtureKind::ImportedNurbsFace,
        1.0e-2,
        8,
        12,
        0x226e_dd2a_120c_74b0,
        0x621e_45af_fa38_7187,
    ),
    case(
        "topology/body-tessellation/imported-nurbs-face-v2/1/chord-1e-3-v2",
        FixtureKind::ImportedNurbsFace,
        1.0e-3,
        8,
        12,
        0x226e_dd2a_120c_74b0,
        0x621e_45af_fa38_7187,
    ),
    case(
        "topology/body-tessellation/imported-tolerant-edge-v2/1/chord-1e-2-v2",
        FixtureKind::ImportedTolerantEdge,
        1.0e-2,
        8,
        12,
        0x226e_dd2a_120c_74b0,
        0xe30c_4f87_158c_d78e,
    ),
    case(
        "topology/body-tessellation/imported-tolerant-edge-v2/1/chord-1e-3-v2",
        FixtureKind::ImportedTolerantEdge,
        1.0e-3,
        8,
        12,
        0x226e_dd2a_120c_74b0,
        0xe30c_4f87_158c_d78e,
    ),
];

const fn case(
    path: &'static str,
    fixture_kind: FixtureKind,
    chord_tol: f64,
    expected_mesh_vertices: usize,
    expected_mesh_triangles: usize,
    expected_mesh_digest: u64,
    expected_output_digest: u64,
) -> BodyTessellationCase {
    let (source_faces, source_edges, source_vertices) = match fixture_kind {
        FixtureKind::Block | FixtureKind::ImportedNurbsFace | FixtureKind::ImportedTolerantEdge => {
            (6, 12, 8)
        }
        FixtureKind::Cylinder | FixtureKind::Cone => (3, 2, 0),
        FixtureKind::Sphere | FixtureKind::Torus => (1, 0, 0),
    };
    let (usage, usage_stage_digest) = reviewed_accounting(fixture_kind, chord_tol);
    BodyTessellationCase {
        path,
        fixture_kind,
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
        expected_usage: usage,
        expected_usage_stage_digest: usage_stage_digest,
    }
}

const fn reviewed_accounting(
    fixture_kind: FixtureKind,
    chord_tol: f64,
) -> ([u64; USAGE_STAGE_COUNT], u64) {
    assert!(
        chord_tol.to_bits() == 1.0e-2_f64.to_bits() || chord_tol.to_bits() == 1.0e-3_f64.to_bits()
    );
    let fine = chord_tol < 5.0e-3;
    match (fixture_kind, fine) {
        (FixtureKind::Block, _) => (
            [
                0, 0, 0, 0, 0, 0, 0, 0, 2, 24, 1, 150, 0, 0, 120, 0, 0, 8, 120, 12, 84,
            ],
            0xbf3b_615d_9b62_211b,
        ),
        (FixtureKind::Cylinder, false) => (
            [
                0, 0, 0, 0, 0, 0, 0, 5, 5_762, 2_979, 1, 647, 3, 56, 206, 0, 0, 2_913, 3_497,
                5_822, 23,
            ],
            0x7a3d_7472_ee49_90c7,
        ),
        (FixtureKind::Cylinder, true) => (
            [
                0, 0, 0, 0, 0, 0, 0, 7, 171_110, 85_941, 1, 2_567, 5, 248, 782, 0, 0, 85_683,
                87_995, 171_362, 23,
            ],
            0x39fc_e57f_8d9b_ef9b,
        ),
        (FixtureKind::Cone, false) => (
            [
                0, 0, 0, 0, 0, 0, 0, 5, 5_410, 2_803, 1, 647, 3, 56, 206, 0, 0, 2_737, 3_321,
                5_470, 23,
            ],
            0x5438_4a78_bc47_d770,
        ),
        (FixtureKind::Cone, true) => (
            [
                0, 0, 0, 0, 0, 0, 0, 7, 108_672, 54_626, 1, 1_927, 5, 184, 590, 0, 0, 54_432,
                56_168, 108_860, 23,
            ],
            0x1e56_8d21_5d34_34c3,
        ),
        (FixtureKind::Sphere, false) => (
            [
                0, 0, 0, 0, 0, 0, 0, 10, 2_728, 2_788, 0, 0, 0, 0, 0, 4, 30, 2_704, 3_127, 5_404, 6,
            ],
            0x3406_66f2_a573_7072,
        ),
        (FixtureKind::Sphere, true) => (
            [
                0, 0, 0, 0, 0, 0, 0, 12, 75_506, 75_714, 0, 0, 0, 0, 0, 6, 126, 75_430, 76_893,
                150_856, 6,
            ],
            0x9edf_8608_c712_3093,
        ),
        (FixtureKind::Torus, false) => (
            [
                0, 0, 0, 0, 0, 0, 0, 20, 6_244, 11_504, 0, 0, 0, 0, 0, 5, 152, 11_340, 12_480,
                22_680, 22,
            ],
            0xe7d8_13a7_f1cb_300b,
        ),
        (FixtureKind::Torus, true) => (
            [
                0, 0, 0, 0, 0, 0, 0, 28, 76_164, 148_566, 0, 0, 0, 0, 0, 6, 376, 148_178, 150_886,
                296_356, 22,
            ],
            0x2135_eeba_3fb0_dfa7,
        ),
        (FixtureKind::ImportedNurbsFace, _) => (
            [
                0, 1, 1, 16, 625, 0, 0, 0, 2, 24, 1, 54, 0, 0, 120, 0, 0, 8, 120, 12, 84,
            ],
            0x8e9a_f09b_d104_3a00,
        ),
        (FixtureKind::ImportedTolerantEdge, _) => (
            [
                0, 0, 0, 0, 0, 0, 0, 0, 2, 24, 1, 66, 0, 0, 120, 0, 0, 8, 120, 12, 84,
            ],
            0x0604_73f6_2ba7_442f,
        ),
    }
}

/// Construct the explicit compatibility-v1 policy outside measured work.
pub fn compatibility_session() -> SessionPolicy {
    SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        BodyTessellationBudgetProfile::v1_defaults(),
        PolicyVersion::V1,
    )
}

/// Construct reviewed request data outside the measured interval.
pub const fn tessellation_options(chord_tol: f64) -> TessOptions {
    TessOptions {
        chord_tol,
        max_edge_len: None,
    }
}

/// One contextual tessellation result with its complete operation report.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyTessellationRun {
    /// Tessellated body mesh.
    pub mesh: BodyMesh,
    /// Deterministic operation accounting and diagnostics.
    pub report: OperationReport,
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
    /// Invoke exactly the contextual API measured by Q3.
    pub fn tessellate_outcome(
        &self,
        options: &TessOptions,
        context: &OperationContext<'_>,
    ) -> Result<OperationOutcome<BodyMesh, TessellationError>, OperationPolicyError> {
        tessellate_body_with_context(&self.store, self.body, options, context)
    }

    /// Tessellate once through the contextual public body entry point.
    pub fn tessellate(
        &self,
        chord_tol: f64,
        context: &OperationContext<'_>,
    ) -> BodyTessellationRun {
        let options = tessellation_options(chord_tol);
        let outcome = self
            .tessellate_outcome(&options, context)
            .expect("reviewed Q3 policy must be valid");
        BodyTessellationRun::from_outcome(outcome)
    }

    /// Validate one mesh and reduce it to stable semantic evidence.
    pub fn evidence(&self, run: &BodyTessellationRun) -> BodyTessellationEvidence {
        let mesh = &run.mesh;
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
        let report = report_evidence(&run.report);
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
            api_identity: API_IDENTITY,
            profile_identity: PROFILE_IDENTITY,
            execution_identity: EXECUTION_IDENTITY,
            policy_version_v1: run.report.policy_version() == PolicyVersion::V1,
            face_boundary_depth: report.face[0],
            face_boundary_splits: report.face[1],
            face_refinement_passes: report.face[2],
            face_mesh_triangles: report.face[3],
            face_mesh_vertices: report.face[4],
            body_edge_depth: report.body[0],
            body_edge_splits: report.body[1],
            body_edge_storage_items: report.body[2],
            body_iso_arc_depth: report.body[3],
            body_iso_arc_splits: report.body[4],
            body_mesh_vertices: report.body[5],
            body_prepared_patch_items: report.body[6],
            body_retained_triangles: report.body[7],
            body_structural_items: report.body[8],
            usage_stage_count: report.stage_count,
            usage_consumed: report.consumed,
            usage_stage_digest: report.stage_digest,
            limit_event_count: run.report.limit_events().len(),
            numeric_resolution_stage_count: run.report.numeric_resolution_stages().len(),
            diagnostic_count: run.report.diagnostics().len(),
            dropped_diagnostic_count: run.report.dropped_diagnostics(),
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

impl BodyTessellationRun {
    /// Unpack a completed contextual call outside the measured interval.
    pub fn from_outcome(outcome: OperationOutcome<BodyMesh, TessellationError>) -> Self {
        let (result, report) = outcome.into_parts();
        BodyTessellationRun {
            mesh: result.expect("reviewed Q3 fixture must tessellate"),
            report,
        }
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
    /// Contextual entry-point identity.
    pub api_identity: &'static str,
    /// Complete family-budget identity.
    pub profile_identity: &'static str,
    /// Deterministic execution-policy identity.
    pub execution_identity: &'static str,
    /// Whether the operation report retained policy version v1.
    pub policy_version_v1: bool,
    /// Face boundary-refinement depth high-water.
    pub face_boundary_depth: u64,
    /// Accepted face boundary splits.
    pub face_boundary_splits: u64,
    /// Completed face interior-refinement passes.
    pub face_refinement_passes: u64,
    /// Face triangle-allocation high-water.
    pub face_mesh_triangles: u64,
    /// Cumulative face mesh-vertex allocations.
    pub face_mesh_vertices: u64,
    /// Exact-edge refinement depth high-water.
    pub body_edge_depth: u64,
    /// Accepted exact-edge refinement splits.
    pub body_edge_splits: u64,
    /// Body-owned edge preparation and output-storage items.
    pub body_edge_storage_items: u64,
    /// Iso/seam refinement depth high-water.
    pub body_iso_arc_depth: u64,
    /// Accepted iso/seam refinement splits.
    pub body_iso_arc_splits: u64,
    /// Retained whole-body mesh vertices.
    pub body_mesh_vertices: u64,
    /// Body-owned prepared UV/patch items.
    pub body_prepared_patch_items: u64,
    /// Retained nondegenerate whole-body triangles.
    pub body_retained_triangles: u64,
    /// Body-owned topology, mapping, and container slots.
    pub body_structural_items: u64,
    /// Number of canonical body, face, graph, and projection usage stages.
    pub usage_stage_count: usize,
    /// Consumed values for every canonical stage in profile/report order.
    pub usage_consumed: [u64; USAGE_STAGE_COUNT],
    /// Digest of every canonical stage, including zero graph/projection usage.
    pub usage_stage_digest: u64,
    /// Attempted resource-limit crossing count.
    pub limit_event_count: usize,
    /// Numeric-resolution stop count.
    pub numeric_resolution_stage_count: usize,
    /// Retained semantic diagnostic count.
    pub diagnostic_count: usize,
    /// Diagnostics omitted because of the configured capacity.
    pub dropped_diagnostic_count: u64,
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
        digest.string(self.api_identity);
        digest.string(self.profile_identity);
        digest.string(self.execution_identity);
        digest.boolean(self.policy_version_v1);
        for consumed in self.face_usage().into_iter().chain(self.body_usage()) {
            digest.u64(consumed);
        }
        digest.count(self.usage_stage_count);
        for consumed in self.usage_consumed {
            digest.u64(consumed);
        }
        digest.u64(self.usage_stage_digest);
        digest.count(self.limit_event_count);
        digest.count(self.numeric_resolution_stage_count);
        digest.count(self.diagnostic_count);
        digest.u64(self.dropped_diagnostic_count);
        digest.finish()
    }

    fn face_usage(&self) -> [u64; 5] {
        [
            self.face_boundary_depth,
            self.face_boundary_splits,
            self.face_refinement_passes,
            self.face_mesh_triangles,
            self.face_mesh_vertices,
        ]
    }

    fn body_usage(&self) -> [u64; 9] {
        [
            self.body_edge_depth,
            self.body_edge_splits,
            self.body_edge_storage_items,
            self.body_iso_arc_depth,
            self.body_iso_arc_splits,
            self.body_mesh_vertices,
            self.body_prepared_patch_items,
            self.body_retained_triangles,
            self.body_structural_items,
        ]
    }
}

struct ReportEvidence {
    face: [u64; 5],
    body: [u64; 9],
    consumed: [u64; USAGE_STAGE_COUNT],
    stage_count: usize,
    stage_digest: u64,
}

fn report_evidence(report: &OperationReport) -> ReportEvidence {
    assert_eq!(report.policy_version(), PolicyVersion::V1);
    let profile = BodyTessellationBudgetProfile::v1_defaults();
    assert_eq!(report.usage().len(), profile.limits().len());
    assert_eq!(report.usage().len(), USAGE_STAGE_COUNT);
    let mut digest = StableHasher::new();
    digest.string("q3-usage.v1");
    digest.string(PROFILE_IDENTITY);
    digest.string("policy.v1");
    digest.count(report.usage().len());
    for (snapshot, limit) in report.usage().iter().zip(profile.limits()) {
        assert_eq!(snapshot.stage, limit.stage);
        assert_eq!(snapshot.resource, limit.resource);
        assert_eq!(snapshot.allowed, limit.allowed);
        digest.string(snapshot.stage.as_str());
        digest.tag(resource_tag(snapshot.resource));
        digest.tag(accounting_tag(limit.mode));
        digest.u64(snapshot.consumed);
    }
    ReportEvidence {
        face: [
            consumed(report, FACE_TESSELLATION_BOUNDARY_DEPTH),
            consumed(report, FACE_TESSELLATION_BOUNDARY_SPLITS),
            consumed(report, FACE_TESSELLATION_REFINEMENT_PASSES),
            consumed(report, FACE_TESSELLATION_MESH_TRIANGLES),
            consumed(report, FACE_TESSELLATION_MESH_VERTICES),
        ],
        body: [
            consumed(report, BODY_TESSELLATION_EDGE_DEPTH),
            consumed(report, BODY_TESSELLATION_EDGE_SPLITS),
            consumed(report, BODY_TESSELLATION_EDGE_STORAGE_ITEMS),
            consumed(report, BODY_TESSELLATION_ISO_ARC_DEPTH),
            consumed(report, BODY_TESSELLATION_ISO_ARC_SPLITS),
            consumed(report, BODY_TESSELLATION_MESH_VERTICES),
            consumed(report, BODY_TESSELLATION_PREPARED_PATCH_ITEMS),
            consumed(report, BODY_TESSELLATION_RETAINED_TRIANGLES),
            consumed(report, BODY_TESSELLATION_STRUCTURAL_ITEMS),
        ],
        consumed: report
            .usage()
            .iter()
            .map(|entry| entry.consumed)
            .collect::<Vec<_>>()
            .try_into()
            .expect("canonical Q3 profile has exactly 21 stages"),
        stage_count: report.usage().len(),
        stage_digest: digest.finish(),
    }
}

fn consumed(report: &OperationReport, stage: StageId) -> u64 {
    report
        .usage()
        .iter()
        .find(|entry| entry.stage == stage)
        .unwrap_or_else(|| panic!("missing canonical Q3 usage stage {}", stage.as_str()))
        .consumed
}

const fn resource_tag(resource: ResourceKind) -> u8 {
    match resource {
        ResourceKind::Work => 1,
        ResourceKind::Items => 2,
        ResourceKind::Bytes => 3,
        ResourceKind::Depth => 4,
        _ => panic!("Q3 digest does not define a tag for this resource kind"),
    }
}

const fn accounting_tag(mode: AccountingMode) -> u8 {
    match mode {
        AccountingMode::Cumulative => 1,
        AccountingMode::HighWater => 2,
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
    let (body, exact_volume, minimum_volume_ratio) = match case.fixture_kind {
        FixtureKind::Block => (
            make::block(&mut store, &frame, [2.0, 3.0, 4.0]).expect("valid block fixture"),
            24.0,
            1.0 - 1.0e-12,
        ),
        FixtureKind::Cylinder => (
            make::cylinder(&mut store, &frame, 1.3, 2.0).expect("valid cylinder fixture"),
            core::f64::consts::PI * 1.3 * 1.3 * 2.0,
            0.98,
        ),
        FixtureKind::Cone => (
            make::cone(&mut store, &frame, 1.5, 0.6, 2.0).expect("valid cone fixture"),
            core::f64::consts::PI * 2.0 * (1.5 * 1.5 + 1.5 * 0.6 + 0.6 * 0.6) / 3.0,
            0.98,
        ),
        FixtureKind::Sphere => (
            make::sphere(&mut store, &frame, 1.1).expect("valid sphere fixture"),
            4.0 / 3.0 * core::f64::consts::PI * 1.1_f64.powi(3),
            0.98,
        ),
        FixtureKind::Torus => (
            make::torus(&mut store, &frame, 2.0, 0.7).expect("valid torus fixture"),
            2.0 * core::f64::consts::PI * core::f64::consts::PI * 2.0 * 0.7 * 0.7,
            0.98,
        ),
        FixtureKind::ImportedNurbsFace => {
            assert_eq!(IMPORTED_NURBS_FACE_BYTES.len(), 6_488);
            let mut source_digest = StableHasher::new();
            source_digest.bytes(IMPORTED_NURBS_FACE_BYTES);
            assert_eq!(
                source_digest.finish(),
                IMPORTED_NURBS_FACE_BYTE_DIGEST,
                "certified Q3 NURBS fixture bytes drifted"
            );
            let reconstruction = kxt::import(IMPORTED_NURBS_FACE_BYTES, &mut store)
                .expect("certified Q3 NURBS fixture must import");
            assert!(reconstruction.skipped.is_empty());
            assert_eq!(reconstruction.bodies.len(), 1);
            let body = reconstruction.bodies[0];
            assert_eq!(store.count::<Curve2dGeom>(), 0);
            let nurbs_faces = store
                .faces_of_body(body)
                .expect("certified Q3 NURBS body must be live")
                .into_iter()
                .filter(|&face| {
                    let surface = store.get(face).expect("live face").surface;
                    matches!(
                        store.get(surface).expect("live surface"),
                        SurfaceGeom::Nurbs(_)
                    )
                })
                .count();
            assert_eq!(nurbs_faces, 1);
            (body, 0.024, 1.0 - 1.0e-12)
        }
        FixtureKind::ImportedTolerantEdge => {
            assert_eq!(IMPORTED_TOLERANT_EDGE_BYTES.len(), 7_172);
            let mut source_digest = StableHasher::new();
            source_digest.bytes(IMPORTED_TOLERANT_EDGE_BYTES);
            assert_eq!(
                source_digest.finish(),
                IMPORTED_TOLERANT_EDGE_BYTE_DIGEST,
                "certified Q3 tolerant-edge fixture bytes drifted"
            );
            let reconstruction = kxt::import(IMPORTED_TOLERANT_EDGE_BYTES, &mut store)
                .expect("certified Q3 tolerant-edge fixture must import");
            assert_eq!(
                reconstruction.skipped,
                vec![(kxt::schema::code::GEOMETRIC_OWNER, 4)]
            );
            assert_eq!(reconstruction.bodies.len(), 1);
            let body = reconstruction.bodies[0];
            assert_eq!(store.count::<Curve2dGeom>(), 2);
            assert_eq!(
                store
                    .edges_of_body(body)
                    .expect("certified Q3 tolerant-edge body must be live")
                    .into_iter()
                    .filter(|&edge| {
                        let edge = store.get(edge).expect("live tolerant edge");
                        edge.curve().is_none() && edge.tolerance().is_some()
                    })
                    .count(),
                1
            );
            let pcurves: Vec<_> = store
                .edges_of_body(body)
                .expect("certified Q3 tolerant-edge body must be live")
                .into_iter()
                .flat_map(|edge| store.get(edge).expect("live edge").fins().to_vec())
                .filter_map(|fin| store.get(fin).expect("live fin").pcurve())
                .collect();
            assert_eq!(pcurves.len(), 2);
            assert!(pcurves.into_iter().all(|pcurve| matches!(
                store.get(pcurve.curve()).expect("live NURBS pcurve"),
                Curve2dGeom::Nurbs(_)
            )));
            (body, 0.024, 1.0 - 1.0e-12)
        }
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
    assert_eq!(evidence.api_identity, API_IDENTITY);
    assert_eq!(evidence.profile_identity, PROFILE_IDENTITY);
    assert_eq!(evidence.execution_identity, EXECUTION_IDENTITY);
    assert!(evidence.policy_version_v1);
    assert_eq!(evidence.usage_stage_count, USAGE_STAGE_COUNT);
    assert_eq!(evidence.limit_event_count, 0);
    assert_eq!(evidence.numeric_resolution_stage_count, 0);
    assert_eq!(evidence.diagnostic_count, 0);
    assert_eq!(evidence.dropped_diagnostic_count, 0);
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
    assert_eq!(evidence.usage_consumed, case.expected_usage);
    assert_eq!(
        evidence.face_usage(),
        [
            case.expected_usage[5],
            case.expected_usage[6],
            case.expected_usage[7],
            case.expected_usage[8],
            case.expected_usage[9],
        ]
    );
    assert_eq!(evidence.body_usage(), case.expected_usage[12..]);
    assert_eq!(
        evidence.usage_stage_digest,
        case.expected_usage_stage_digest
    );
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

    fn string(&mut self, value: &str) {
        self.count(value.len());
        self.bytes(value.as_bytes());
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
    fn registry_contains_exactly_fourteen_unique_canonical_cases() {
        assert_eq!(CASES.len(), 14);
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
            assert_eq!(entry["policy_values"]["api"], API_IDENTITY);
            assert_eq!(entry["policy_values"]["budget_profile"], PROFILE_IDENTITY);
            assert_eq!(entry["policy_values"]["execution"], EXECUTION_IDENTITY);
            assert_eq!(entry["policy_values"]["policy_version"], "v1");
            assert_eq!(entry["policy_values"]["usage_contract"], "q3-usage.v1");
            match case.fixture_kind {
                FixtureKind::ImportedNurbsFace => {
                    assert_eq!(
                        entry["size_parameters"]["input_bytes"].as_u64(),
                        Some(IMPORTED_NURBS_FACE_BYTES.len() as u64)
                    );
                    assert_eq!(
                        entry["policy_values"]["source_fixture"],
                        IMPORTED_NURBS_FACE_IDENTITY
                    );
                    assert_eq!(
                        entry["policy_values"]["source_sha256"],
                        IMPORTED_NURBS_FACE_SHA256
                    );
                }
                FixtureKind::ImportedTolerantEdge => {
                    assert_eq!(
                        entry["size_parameters"]["input_bytes"].as_u64(),
                        Some(IMPORTED_TOLERANT_EDGE_BYTES.len() as u64)
                    );
                    assert_eq!(
                        entry["policy_values"]["source_fixture"],
                        IMPORTED_TOLERANT_EDGE_IDENTITY
                    );
                    assert_eq!(
                        entry["policy_values"]["source_sha256"],
                        IMPORTED_TOLERANT_EDGE_SHA256
                    );
                    assert_eq!(
                        entry["expected_result_counters"]["tolerant_edges"].as_u64(),
                        Some(1)
                    );
                    assert_eq!(
                        entry["expected_result_counters"]["pcurve_uses"].as_u64(),
                        Some(2)
                    );
                    assert_eq!(
                        entry["expected_result_counters"]["skipped_geometric_owners"].as_u64(),
                        Some(4)
                    );
                }
                _ => {
                    assert!(entry["size_parameters"]["input_bytes"].is_null());
                    assert!(entry["policy_values"]["source_fixture"].is_null());
                    assert!(entry["policy_values"]["source_sha256"].is_null());
                }
            }

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
                counters["usage_stage_count"].as_u64(),
                Some(USAGE_STAGE_COUNT as u64)
            );
            let consumed = counters["usage_consumed"].as_array().unwrap();
            assert_eq!(consumed.len(), USAGE_STAGE_COUNT);
            for (index, value) in consumed.iter().enumerate() {
                assert_eq!(value.as_u64(), Some(case.expected_usage[index]));
            }
            assert_eq!(
                counters["usage_stage_digest"].as_str(),
                Some(format!("{:016x}", case.expected_usage_stage_digest).as_str())
            );
            for field in [
                "limit_event_count",
                "numeric_resolution_stage_count",
                "diagnostic_count",
                "dropped_diagnostic_count",
            ] {
                assert_eq!(counters[field].as_u64(), Some(0), "{field}");
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
            let session = compatibility_session();
            let context = OperationContext::new(&session, Tolerances::default()).unwrap();
            let first_run = fixture.tessellate(case.chord_tol, &context);
            let second_run = fixture.tessellate(case.chord_tol, &context);
            assert_eq!(
                first_run.mesh, second_run.mesh,
                "mesh drift for {}",
                case.path
            );
            assert_eq!(
                first_run.report, second_run.report,
                "report drift for {}",
                case.path
            );
            let first = fixture.evidence(&first_run);
            let second = fixture.evidence(&second_run);
            assert_eq!(first, second, "repeatability drift for {}", case.path);
            verify(case, first);
        }
    }

    #[test]
    fn reversed_face_range_is_rejected() {
        let case = CASES[0];
        let fixture = fixture(case);
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let run = fixture.tessellate(case.chord_tol, &context);
        let mut mesh = run.mesh.clone();
        assert!(mesh.face_ranges.len() > 1);
        let start = mesh.face_ranges[1].1.start;
        assert!(start > 0);
        mesh.face_ranges[1].1.end = start - 1;
        let drifted = BodyTessellationRun {
            mesh,
            report: run.report,
        };
        assert!(!fixture.evidence(&drifted).indices_valid);
    }

    #[test]
    fn wrong_and_duplicate_owner_mappings_are_rejected_and_digested() {
        let case = CASES[0];
        let fixture = fixture(case);
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let run = fixture.tessellate(case.chord_tol, &context);
        let mesh = &run.mesh;
        let reviewed = fixture.evidence(&run);

        let mut duplicate_face = mesh.clone();
        duplicate_face.face_ranges[1].0 = duplicate_face.face_ranges[0].0;
        let duplicate_face = fixture.evidence(&BodyTessellationRun {
            mesh: duplicate_face,
            report: run.report.clone(),
        });
        assert!(!duplicate_face.owner_mapping_valid);
        assert_ne!(duplicate_face.mesh_digest, reviewed.mesh_digest);

        let mut wrong_edge_order = mesh.clone();
        wrong_edge_order.edge_polylines.swap(0, 1);
        let wrong_edge_order = fixture.evidence(&BodyTessellationRun {
            mesh: wrong_edge_order,
            report: run.report,
        });
        assert!(!wrong_edge_order.owner_mapping_valid);
        assert_ne!(wrong_edge_order.mesh_digest, reviewed.mesh_digest);
    }
}
