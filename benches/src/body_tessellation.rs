//! Deterministic Q3 body-tessellation fixtures and evidence.

#[cfg(test)]
use kcore::operation::TOTAL_WORK_STAGE;
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
use ktopo::entity::{Body, BodyId, BodyKind, EdgeId, FaceId, Sense};
use ktopo::geom::{Curve2dGeom, SurfaceGeom};
use ktopo::make;
use ktopo::store::Store;

/// Fixture identity shared by the Q3 solid/sheet representation matrix.
pub const FIXTURE_VERSION: &str = "body-tessellation.v3";
/// Deterministic fixture seed (construction itself is not randomized).
pub const FIXTURE_SEED: u64 = 0x5154_4553_5300_0003;
/// Public entry point measured by the Q3 v3 contract.
pub const API_IDENTITY: &str = "tessellate_body_with_context";
/// Complete body-family defaults used by the Q3 v3 contract.
pub const PROFILE_IDENTITY: &str = "body-tessellation.compatibility-v1";
/// Deterministic execution policy used by the Q3 v2 contract.
pub const EXECUTION_IDENTITY: &str = "serial";
/// Canonical number of body, face, graph, and projection usage stages.
pub const USAGE_STAGE_COUNT: usize = 21;
/// Source-evidence label shared by the 2026-07-11 licensed-host corpus.
pub const HOST_ACCEPTED_SOURCE_EVIDENCE: &str = "historical-host-accepted:onshape-cloud-2026-07-11";
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
    include_bytes!("../testdata/solid_block_tolerant_edge.certified.x_t");
/// Exact licensed-host-certified cylinder used for broader imported-corpus coverage.
pub const IMPORTED_CYLINDER_IDENTITY: &str = "solid_cylinder.x_t@onshape-cloud-2026-07-11";
/// SHA-256 pinned by `docs/oracle-certification.json` for the imported cylinder.
pub const IMPORTED_CYLINDER_SHA256: &str =
    "f1f2389c98ca323a8e5aef2f19ed2e88f406f8569c2c87a14d86c79111c6a9c4";
/// Portable in-harness byte digest for the exact certified cylinder bytes.
pub const IMPORTED_CYLINDER_BYTE_DIGEST: u64 = 0x57b8_9bfc_e92d_c85a;
const IMPORTED_CYLINDER_BYTES: &[u8] = include_bytes!("../testdata/solid_cylinder.certified.x_t");
/// Licensed-host-certified curved exact-NURBS block.
pub const CURVED_NURBS_BLOCK_IDENTITY: &str =
    "solid_block_curved_nurbs_face.x_t@onshape-cloud-2026-07-20";
/// Exact licensed-host evidence pinned by the current oracle record.
pub const CURVED_NURBS_BLOCK_SOURCE_EVIDENCE: &str =
    "licensed-host-accepted:onshape-cloud-2026-07-20";
/// SHA-256 of the certified curved-NURBS block bytes.
pub const CURVED_NURBS_BLOCK_SHA256: &str =
    "7fad6999a2d2bd0653a3b7558e0460e9ccfe07a43d00f249709ea7aae642829e";
/// Portable in-harness byte digest of the certified curved-NURBS block bytes.
pub const CURVED_NURBS_BLOCK_BYTE_DIGEST: u64 = 0xb8e1_8725_bad5_df39;
const CURVED_NURBS_BLOCK_BYTES: &[u8] =
    include_bytes!("../testdata/solid_block_curved_nurbs_face.local.x_t");
/// Licensed-host-certified concave planar sheet identity.
pub const IMPORTED_PLANE_SHEET_IDENTITY: &str = "sheet_plane_polygon.x_t@onshape-cloud-2026-07-11";
/// SHA-256 pinned by the licensed-host-certified oracle corpus.
pub const IMPORTED_PLANE_SHEET_SHA256: &str =
    "38cec426b656aba55e949d16e50bbf66c1a084941bf333f5f26a2d64f3d9391c";
/// Portable in-harness byte digest of the certified planar-sheet bytes.
pub const IMPORTED_PLANE_SHEET_BYTE_DIGEST: u64 = 0x74e5_96ca_4fb6_fa2a;
const IMPORTED_PLANE_SHEET_BYTES: &[u8] =
    include_bytes!("../testdata/sheet_plane_polygon.certified.x_t");
/// Licensed-host-certified periodic cylindrical sheet identity.
pub const IMPORTED_CYLINDER_SHEET_IDENTITY: &str =
    "sheet_cylinder_seam.x_t@onshape-cloud-2026-07-11";
/// SHA-256 pinned by the licensed-host-certified oracle corpus.
pub const IMPORTED_CYLINDER_SHEET_SHA256: &str =
    "94af58ef0905b5bca7596966510f6da3b1f2832fe50cd07518189d1fd48926d6";
/// Portable in-harness byte digest of the certified cylindrical-sheet bytes.
pub const IMPORTED_CYLINDER_SHEET_BYTE_DIGEST: u64 = 0x2370_4889_3149_88da;
const IMPORTED_CYLINDER_SHEET_BYTES: &[u8] =
    include_bytes!("../testdata/sheet_cylinder_seam.certified.x_t");

/// Body-kind-aware correctness contract for one Q3 representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidationKind {
    /// Closed manifold: incidence, watertightness, outwardness, and volume.
    ClosedSolid,
    /// Open two-manifold: incidence, face-sense alignment, and surface area.
    OrientedSheet,
}

/// Closed solid represented by one Q3 case.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixtureKind {
    /// Six planar faces and straight edges.
    Block,
    /// Periodic side face with two closed seam boundaries.
    Cylinder,
    /// Cylinder tessellated from a store that also owns a block and sphere.
    MixedStoreCylinder,
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
    /// Certified X_T cylinder reconstructed into analytic curved topology.
    ImportedCylinder,
    /// Local X_T block with one genuinely curved exact NURBS face.
    CurvedNurbsBlock,
    /// Certified X_T concave planar sheet.
    ImportedPlaneSheet,
    /// Certified X_T periodic cylindrical sheet with a shared seam.
    ImportedCylinderSheet,
}

impl FixtureKind {
    /// Correctness contract implied by the source body's point-set kind.
    pub const fn validation(self) -> ValidationKind {
        match self {
            Self::ImportedPlaneSheet | Self::ImportedCylinderSheet => ValidationKind::OrientedSheet,
            _ => ValidationKind::ClosedSolid,
        }
    }
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
    /// Reviewed total body count in the prepared store.
    pub expected_store_bodies: usize,
    /// Reviewed face-range count.
    pub expected_face_ranges: usize,
    /// Reviewed edge-polyline count.
    pub expected_edge_polylines: usize,
    /// Reviewed number of triangle-boundary segments matching one-fin topology.
    pub expected_boundary_segments: usize,
    /// Reviewed consumed values for all canonical usage stages.
    pub expected_usage: [u64; USAGE_STAGE_COUNT],
    /// Reviewed portable canonical usage-stage digest.
    pub expected_usage_stage_digest: u64,
}

/// Twenty legacy solid rows plus twelve representation/trim matrix rows.
pub const CASES: [BodyTessellationCase; 32] = [
    case(
        "topology/body-tessellation/block-v3/1/chord-1e-2-v3",
        FixtureKind::Block,
        1.0e-2,
        8,
        12,
        0x3773_8ea0_abf2_68a7,
        0x55b9_3194_7701_ef91,
    ),
    case(
        "topology/body-tessellation/block-v3/1/chord-1e-3-v3",
        FixtureKind::Block,
        1.0e-3,
        8,
        12,
        0x3773_8ea0_abf2_68a7,
        0x55b9_3194_7701_ef91,
    ),
    case(
        "topology/body-tessellation/cylinder-v3/1/chord-1e-2-v3",
        FixtureKind::Cylinder,
        1.0e-2,
        2_913,
        5_822,
        0x3047_4187_c9d8_a9ce,
        0xb294_d366_1cfc_2a6b,
    ),
    case(
        "topology/body-tessellation/cylinder-v3/1/chord-1e-3-v3",
        FixtureKind::Cylinder,
        1.0e-3,
        85_683,
        171_362,
        0xc18e_8ba3_3c72_5d33,
        0x6c55_3f56_87b1_c514,
    ),
    case(
        "topology/body-tessellation/mixed-store-cylinder-v3/1/chord-1e-2-v3",
        FixtureKind::MixedStoreCylinder,
        1.0e-2,
        2_913,
        5_822,
        0x3047_4187_c9d8_a9ce,
        0xb294_d366_1cfc_2a6b,
    ),
    case(
        "topology/body-tessellation/mixed-store-cylinder-v3/1/chord-1e-3-v3",
        FixtureKind::MixedStoreCylinder,
        1.0e-3,
        85_683,
        171_362,
        0xc18e_8ba3_3c72_5d33,
        0x6c55_3f56_87b1_c514,
    ),
    case(
        "topology/body-tessellation/cone-v3/1/chord-1e-2-v3",
        FixtureKind::Cone,
        1.0e-2,
        2_737,
        5_470,
        0x2ce0_b59e_91e2_2400,
        0x6cb0_5b06_6df5_d3c7,
    ),
    case(
        "topology/body-tessellation/cone-v3/1/chord-1e-3-v3",
        FixtureKind::Cone,
        1.0e-3,
        54_432,
        108_860,
        0x4159_97ae_b0ba_bc82,
        0x11e8_589b_f8b7_ad75,
    ),
    case(
        "topology/body-tessellation/sphere-v3/1/chord-1e-2-v3",
        FixtureKind::Sphere,
        1.0e-2,
        2_704,
        5_404,
        0x79f4_2a54_6c49_f36f,
        0x5bd7_82d1_7be2_b032,
    ),
    case(
        "topology/body-tessellation/sphere-v3/1/chord-1e-3-v3",
        FixtureKind::Sphere,
        1.0e-3,
        75_430,
        150_856,
        0xf827_bff6_d901_87a7,
        0x1b2c_3f1a_631f_0fb2,
    ),
    case(
        "topology/body-tessellation/torus-v3/1/chord-1e-2-v3",
        FixtureKind::Torus,
        1.0e-2,
        11_340,
        22_680,
        0xbec9_d49d_9830_dc7e,
        0x0248_4e80_2b50_7779,
    ),
    case(
        "topology/body-tessellation/torus-v3/1/chord-1e-3-v3",
        FixtureKind::Torus,
        1.0e-3,
        148_178,
        296_356,
        0x39d6_eb3f_0319_b7f7,
        0x4276_248d_43e1_4f1a,
    ),
    case(
        "topology/body-tessellation/imported-nurbs-face-v3/1/chord-1e-2-v3",
        FixtureKind::ImportedNurbsFace,
        1.0e-2,
        8,
        12,
        0x226e_dd2a_120c_74b0,
        0x0f5d_a999_70d6_f41a,
    ),
    case(
        "topology/body-tessellation/imported-nurbs-face-v3/1/chord-1e-3-v3",
        FixtureKind::ImportedNurbsFace,
        1.0e-3,
        8,
        12,
        0x226e_dd2a_120c_74b0,
        0x0f5d_a999_70d6_f41a,
    ),
    case(
        "topology/body-tessellation/imported-tolerant-edge-v3/1/chord-1e-2-v3",
        FixtureKind::ImportedTolerantEdge,
        1.0e-2,
        8,
        12,
        0x226e_dd2a_120c_74b0,
        0x3340_2f6c_178b_c67f,
    ),
    case(
        "topology/body-tessellation/imported-tolerant-edge-v3/1/chord-1e-3-v3",
        FixtureKind::ImportedTolerantEdge,
        1.0e-3,
        8,
        12,
        0x226e_dd2a_120c_74b0,
        0x3340_2f6c_178b_c67f,
    ),
    case(
        "topology/body-tessellation/imported-cylinder-v3/1/chord-1e-2-v3",
        FixtureKind::ImportedCylinder,
        1.0e-2,
        202,
        400,
        0xf770_2f5f_5022_0f95,
        0x7e97_894b_e33b_2217,
    ),
    case(
        "topology/body-tessellation/imported-cylinder-v3/1/chord-3e-3-v3",
        FixtureKind::ImportedCylinder,
        3.0e-3,
        540,
        1_076,
        0x57c3_295c_221c_4f73,
        0xc6df_e970_f13e_ceb0,
    ),
    case(
        "topology/body-tessellation/imported-cylinder-v3/1/chord-1e-3-v3",
        FixtureKind::ImportedCylinder,
        1.0e-3,
        2_320,
        4_636,
        0xc4ba_635c_11a1_e117,
        0x3c27_fbc8_e7ff_5a66,
    ),
    case(
        "topology/body-tessellation/imported-cylinder-v3/1/chord-3e-4-v3",
        FixtureKind::ImportedCylinder,
        3.0e-4,
        12_248,
        24_492,
        0x135a_e581_4e82_5469,
        0xe1f9_cb58_e265_b8fb,
    ),
    case(
        "topology/body-tessellation/imported-curved-nurbs-block-v3/1/chord-1e-2-v3",
        FixtureKind::CurvedNurbsBlock,
        1.0e-2,
        8,
        12,
        0x226e_dd2a_120c_74b0,
        0x0f5d_a999_70d6_f41a,
    ),
    case(
        "topology/body-tessellation/imported-curved-nurbs-block-v3/1/chord-3e-3-v3",
        FixtureKind::CurvedNurbsBlock,
        3.0e-3,
        9,
        14,
        0xb245_e9dd_d554_fd49,
        0xe653_f539_fa39_238d,
    ),
    case(
        "topology/body-tessellation/imported-curved-nurbs-block-v3/1/chord-1e-3-v3",
        FixtureKind::CurvedNurbsBlock,
        1.0e-3,
        25,
        46,
        0xdaef_2eb7_4d11_c10a,
        0xbf63_3349_9e20_cbbc,
    ),
    case(
        "topology/body-tessellation/imported-curved-nurbs-block-v3/1/chord-5e-4-v3",
        FixtureKind::CurvedNurbsBlock,
        5.0e-4,
        57,
        110,
        0x6818_abb1_c6ba_fc83,
        0x046a_d9a0_dcda_c765,
    ),
    case(
        "topology/body-tessellation/imported-plane-sheet-v3/1/chord-1e-2-v3",
        FixtureKind::ImportedPlaneSheet,
        1.0e-2,
        6,
        4,
        0x383a_2d10_1866_0156,
        0xec8a_582d_f526_3aa1,
    ),
    case(
        "topology/body-tessellation/imported-plane-sheet-v3/1/chord-3e-3-v3",
        FixtureKind::ImportedPlaneSheet,
        3.0e-3,
        6,
        4,
        0x383a_2d10_1866_0156,
        0xec8a_582d_f526_3aa1,
    ),
    case(
        "topology/body-tessellation/imported-plane-sheet-v3/1/chord-1e-3-v3",
        FixtureKind::ImportedPlaneSheet,
        1.0e-3,
        6,
        4,
        0x383a_2d10_1866_0156,
        0xec8a_582d_f526_3aa1,
    ),
    case(
        "topology/body-tessellation/imported-plane-sheet-v3/1/chord-3e-4-v3",
        FixtureKind::ImportedPlaneSheet,
        3.0e-4,
        6,
        4,
        0x383a_2d10_1866_0156,
        0xec8a_582d_f526_3aa1,
    ),
    case(
        "topology/body-tessellation/imported-cylinder-sheet-v3/1/chord-1e-2-v3",
        FixtureKind::ImportedCylinderSheet,
        1.0e-2,
        36,
        40,
        0x2099_6c44_7801_8a8b,
        0x50c5_4031_bd89_a6e0,
    ),
    case(
        "topology/body-tessellation/imported-cylinder-sheet-v3/1/chord-3e-3-v3",
        FixtureKind::ImportedCylinderSheet,
        3.0e-3,
        52,
        72,
        0x2162_9ed9_1eb7_2c2a,
        0x7007_ceb8_c0a5_ecb2,
    ),
    case(
        "topology/body-tessellation/imported-cylinder-sheet-v3/1/chord-1e-3-v3",
        FixtureKind::ImportedCylinderSheet,
        1.0e-3,
        252,
        440,
        0xf252_7d3b_3582_452a,
        0x92b9_90eb_5e87_65e3,
    ),
    case(
        "topology/body-tessellation/imported-cylinder-sheet-v3/1/chord-3e-4-v3",
        FixtureKind::ImportedCylinderSheet,
        3.0e-4,
        1_492,
        2_856,
        0xe8f3_4bc8_0f77_7a96,
        0x59a4_b369_5b03_e58a,
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
        FixtureKind::Block
        | FixtureKind::ImportedNurbsFace
        | FixtureKind::ImportedTolerantEdge
        | FixtureKind::CurvedNurbsBlock => (6, 12, 8),
        FixtureKind::Cylinder
        | FixtureKind::MixedStoreCylinder
        | FixtureKind::ImportedCylinder
        | FixtureKind::Cone => (3, 2, 0),
        FixtureKind::Sphere | FixtureKind::Torus => (1, 0, 0),
        FixtureKind::ImportedPlaneSheet => (1, 6, 6),
        FixtureKind::ImportedCylinderSheet => (1, 3, 2),
    };
    let store_bodies = match fixture_kind {
        FixtureKind::MixedStoreCylinder => 3,
        _ => 1,
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
        expected_store_bodies: store_bodies,
        expected_face_ranges: source_faces,
        expected_edge_polylines: source_edges,
        expected_boundary_segments: match fixture_kind {
            FixtureKind::ImportedPlaneSheet => 6,
            FixtureKind::ImportedCylinderSheet
                if chord_tol.to_bits() == 1.0e-2_f64.to_bits()
                    || chord_tol.to_bits() == 3.0e-3_f64.to_bits() =>
            {
                32
            }
            FixtureKind::ImportedCylinderSheet if chord_tol.to_bits() == 1.0e-3_f64.to_bits() => 64,
            FixtureKind::ImportedCylinderSheet => 128,
            _ => 0,
        },
        expected_usage: usage,
        expected_usage_stage_digest: usage_stage_digest,
    }
}

const fn reviewed_accounting(
    fixture_kind: FixtureKind,
    chord_tol: f64,
) -> ([u64; USAGE_STAGE_COUNT], u64) {
    assert!(
        chord_tol.to_bits() == 1.0e-2_f64.to_bits()
            || chord_tol.to_bits() == 3.0e-3_f64.to_bits()
            || chord_tol.to_bits() == 1.0e-3_f64.to_bits()
            || chord_tol.to_bits() == 5.0e-4_f64.to_bits()
            || chord_tol.to_bits() == 3.0e-4_f64.to_bits()
    );
    if matches!(fixture_kind, FixtureKind::ImportedCylinder) {
        if chord_tol.to_bits() == 1.0e-2_f64.to_bits() {
            return (
                [
                    0, 0, 0, 0, 0, 0, 0, 3, 372, 236, 1, 131, 2, 24, 110, 0, 0, 202, 498, 400, 23,
                ],
                0x6ee8_a7e7_c94f_6b33,
            );
        }
        if chord_tol.to_bits() == 1.0e-3_f64.to_bits() {
            return (
                [
                    0, 0, 0, 0, 0, 0, 0, 5, 4_576, 2_386, 1, 259, 3, 56, 206, 0, 0, 2_320, 2_904,
                    4_636, 23,
                ],
                0xd07b_2989_d806_bb46,
            );
        }
        if chord_tol.to_bits() == 3.0e-3_f64.to_bits() {
            return (
                [
                    0, 0, 0, 0, 0, 0, 0, 4, 1_048, 574, 1, 131, 2, 24, 110, 0, 0, 540, 836, 1_076,
                    23,
                ],
                0x4331_bcf7_e749_7126,
            );
        }
        return (
            [
                0, 0, 0, 0, 0, 0, 0, 6, 24_368, 12_378, 1, 515, 4, 120, 398, 0, 0, 12_248, 13_408,
                24_492, 23,
            ],
            0x5503_6a96_7093_eaef,
        );
    }
    if matches!(fixture_kind, FixtureKind::CurvedNurbsBlock) {
        if chord_tol.to_bits() == 1.0e-2_f64.to_bits() {
            return (
                [
                    0, 1, 1, 16, 625, 0, 0, 0, 2, 24, 1, 54, 0, 0, 120, 0, 0, 8, 120, 12, 84,
                ],
                0x8e9a_f09b_d104_3a00,
            );
        }
        if chord_tol.to_bits() == 3.0e-3_f64.to_bits() {
            return (
                [
                    0, 1, 1, 16, 625, 0, 0, 1, 4, 25, 1, 54, 0, 0, 120, 0, 0, 9, 121, 14, 84,
                ],
                0xdd87_5541_8cbd_2232,
            );
        }
        if chord_tol.to_bits() == 1.0e-3_f64.to_bits() {
            return (
                [
                    0, 1, 1, 16, 625, 0, 0, 5, 36, 41, 1, 54, 0, 0, 120, 0, 0, 25, 137, 46, 84,
                ],
                0x5d0f_cffa_1463_cb7e,
            );
        }
        return (
            [
                0, 1, 1, 16, 625, 0, 0, 6, 100, 73, 1, 54, 0, 0, 120, 0, 0, 57, 169, 110, 84,
            ],
            0x0e72_cb68_b372_9557,
        );
    }
    if matches!(fixture_kind, FixtureKind::ImportedPlaneSheet) {
        return (
            [
                0, 0, 0, 0, 0, 0, 0, 0, 4, 6, 1, 13, 0, 0, 54, 0, 0, 6, 30, 4, 36,
            ],
            0x627d_bafd_11e0_c984,
        );
    }
    if matches!(fixture_kind, FixtureKind::ImportedCylinderSheet) {
        if chord_tol.to_bits() == 1.0e-2_f64.to_bits() {
            return (
                [
                    0, 0, 0, 0, 0, 0, 0, 4, 10, 44, 1, 69, 2, 24, 118, 0, 0, 36, 276, 40, 23,
                ],
                0xbef1_dd9c_663f_31ae,
            );
        }
        if chord_tol.to_bits() == 3.0e-3_f64.to_bits() {
            return (
                [
                    0, 0, 0, 0, 0, 0, 0, 8, 18, 60, 1, 69, 2, 24, 118, 0, 0, 52, 292, 72, 23,
                ],
                0x598f_284e_b091_01da,
            );
        }
        if chord_tol.to_bits() == 1.0e-3_f64.to_bits() {
            return (
                [
                    0, 0, 0, 0, 0, 0, 0, 12, 110, 260, 1, 133, 3, 56, 214, 0, 0, 252, 684, 440, 23,
                ],
                0x86e6_e8eb_1abb_2b0c,
            );
        }
        return (
            [
                0, 0, 0, 0, 0, 0, 0, 16, 714, 1_500, 1, 261, 4, 120, 406, 0, 0, 1_492, 2_308,
                2_856, 23,
            ],
            0x49dc_a081_ce6d_2499,
        );
    }
    let fine = chord_tol < 5.0e-3;
    match (fixture_kind, fine) {
        (FixtureKind::Block, _) => (
            [
                0, 0, 0, 0, 0, 0, 0, 0, 2, 24, 1, 150, 0, 0, 120, 0, 0, 8, 120, 12, 84,
            ],
            0xbf3b_615d_9b62_211b,
        ),
        (FixtureKind::Cylinder | FixtureKind::MixedStoreCylinder, false) => (
            [
                0, 0, 0, 0, 0, 0, 0, 5, 5_762, 2_979, 1, 647, 3, 56, 206, 0, 0, 2_913, 3_497,
                5_822, 23,
            ],
            0x7a3d_7472_ee49_90c7,
        ),
        (FixtureKind::Cylinder | FixtureKind::MixedStoreCylinder, true) => (
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
        (
            FixtureKind::CurvedNurbsBlock
            | FixtureKind::ImportedPlaneSheet
            | FixtureKind::ImportedCylinderSheet,
            _,
        ) => unreachable!(),
        (FixtureKind::ImportedCylinder, _) => unreachable!(),
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
    validation: ValidationKind,
    exact_measure: f64,
    minimum_measure_ratio: f64,
    maximum_measure_ratio: f64,
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
        let incidence = self.incidence_evidence(mesh);
        let (orientation_valid, measure_within_tolerance) = self.geometric_evidence(mesh);
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
            manifold: incidence.manifold,
            boundary_matches_topology: incidence.boundary_matches_topology,
            boundary_segments: incidence.boundary_segments,
            orientation_valid,
            measure_within_tolerance,
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

    fn incidence_evidence(&self, mesh: &BodyMesh) -> IncidenceEvidence {
        use std::collections::{BTreeMap, BTreeSet};

        let topology_valid = self.expected_edges.iter().all(|&edge_id| {
            let Ok(edge) = self.store.get(edge_id) else {
                return false;
            };
            let expected_fin_count = match self.validation {
                ValidationKind::ClosedSolid => edge.fins().len() == 2,
                ValidationKind::OrientedSheet => matches!(edge.fins().len(), 1 | 2),
            };
            expected_fin_count
                && edge.fins().iter().all(|&fin_id| {
                    let Ok(fin) = self.store.get(fin_id) else {
                        return false;
                    };
                    fin.edge() == edge_id
                        && self
                            .store
                            .get(fin.parent())
                            .is_ok_and(|loop_| self.expected_faces.contains(&loop_.face()))
                })
                && (edge.fins().len() != 2
                    || match (
                        self.store.get(edge.fins()[0]),
                        self.store.get(edge.fins()[1]),
                    ) {
                        (Ok(a), Ok(b)) => a.sense() != b.sense(),
                        _ => false,
                    })
        });

        let mut incidence = BTreeMap::<(u32, u32), [usize; 2]>::new();
        let mut degenerate = false;
        for triangle in &mesh.triangles {
            if triangle[0] == triangle[1]
                || triangle[1] == triangle[2]
                || triangle[2] == triangle[0]
            {
                degenerate = true;
                continue;
            }
            for [a, b] in [
                [triangle[0], triangle[1]],
                [triangle[1], triangle[2]],
                [triangle[2], triangle[0]],
            ] {
                let entry = incidence.entry((a.min(b), a.max(b))).or_default();
                entry[usize::from(a > b)] += 1;
            }
        }
        let actual_boundary: BTreeSet<_> = incidence
            .iter()
            .filter_map(|(&segment, counts)| ((counts[0] + counts[1]) == 1).then_some(segment))
            .collect();
        let directed_manifold = !degenerate
            && !incidence.is_empty()
            && incidence
                .values()
                .all(|counts| matches!(counts, [1, 0] | [0, 1] | [1, 1]));

        let mut expected_boundary = BTreeSet::new();
        let mut unique_expected = true;
        for (edge_id, polyline) in &mesh.edge_polylines {
            let Ok(edge) = self.store.get(*edge_id) else {
                unique_expected = false;
                continue;
            };
            if edge.fins().len() == 1 {
                for pair in polyline.windows(2) {
                    let segment = (pair[0].min(pair[1]), pair[0].max(pair[1]));
                    unique_expected &= segment.0 != segment.1 && expected_boundary.insert(segment);
                }
            }
        }
        let boundary_matches_topology = unique_expected && expected_boundary == actual_boundary;
        let kind_boundary_valid = match self.validation {
            ValidationKind::ClosedSolid => {
                expected_boundary.is_empty()
                    && actual_boundary.is_empty()
                    && check_watertight(mesh).is_empty()
            }
            ValidationKind::OrientedSheet => !expected_boundary.is_empty(),
        };
        IncidenceEvidence {
            manifold: topology_valid && directed_manifold && kind_boundary_valid,
            boundary_matches_topology,
            boundary_segments: actual_boundary.len(),
        }
    }

    fn geometric_evidence(&self, mesh: &BodyMesh) -> (bool, bool) {
        let (orientation_valid, measure) = match self.validation {
            ValidationKind::ClosedSolid => {
                let volume = signed_volume(mesh);
                (volume.is_finite() && volume > 0.0, volume)
            }
            ValidationKind::OrientedSheet => self.sheet_orientation_and_area(mesh),
        };
        let measure_within_tolerance = orientation_valid
            && measure.is_finite()
            && measure >= self.exact_measure * self.minimum_measure_ratio
            && measure <= self.exact_measure * self.maximum_measure_ratio;
        (orientation_valid, measure_within_tolerance)
    }

    fn sheet_orientation_and_area(&self, mesh: &BodyMesh) -> (bool, f64) {
        let dust_threshold = 64.0 * f64::EPSILON * self.exact_measure;
        let mut surface_area = 0.0;
        let valid = mesh.face_ranges.iter().all(|&(face_id, ref range)| {
            let Ok(face) = self.store.get(face_id) else {
                return false;
            };
            let Ok(surface) = self.store.get(face.surface()) else {
                return false;
            };
            let mut signed_alignment = 0.0;
            let mut absolute_alignment = 0.0;
            let mut faceted_area = 0.0;
            let finite = !range.is_empty()
                && mesh.triangles[range.clone()].iter().all(|&triangle| {
                    let [a, b, c] = triangle.map(|index| mesh.positions[index as usize]);
                    let area_vector = (b - a).cross(c - a);
                    let centroid = Point3::new(
                        (a.x + b.x + c.x) / 3.0,
                        (a.y + b.y + c.y) / 3.0,
                        (a.z + b.z + c.z) / 3.0,
                    );
                    let Some(mut expected_normal) = sheet_surface_normal(surface, centroid) else {
                        return false;
                    };
                    if face.sense() == Sense::Reversed {
                        expected_normal = -expected_normal;
                    }
                    let area = 0.5 * area_vector.norm();
                    let alignment = area_vector.dot(expected_normal);
                    signed_alignment += alignment;
                    absolute_alignment += alignment.abs();
                    faceted_area += area;
                    area.is_finite()
                        && alignment.is_finite()
                        && (alignment >= -(2.0 * area) * 1.0e-10 || area <= dust_threshold)
                });
            let aligned = finite
                && absolute_alignment > 0.0
                && signed_alignment > 0.0
                && signed_alignment >= absolute_alignment * (1.0 - 1.0e-10);
            if aligned {
                surface_area += faceted_area;
            }
            aligned
        });
        (valid, surface_area)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct IncidenceEvidence {
    manifold: bool,
    boundary_matches_topology: bool,
    boundary_segments: usize,
}

fn sheet_surface_normal(surface: &SurfaceGeom, point: Point3) -> Option<Vec3> {
    match surface {
        SurfaceGeom::Plane(plane) => Some(plane.frame().z()),
        SurfaceGeom::Cylinder(cylinder) => {
            let frame = cylinder.frame();
            let delta = point - frame.origin();
            (delta - frame.z() * delta.dot(frame.z())).normalized()
        }
        _ => None,
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
    /// Whether every mesh edge has valid directed manifold incidence.
    pub manifold: bool,
    /// Whether triangle boundary segments exactly equal one-fin edge polylines.
    pub boundary_matches_topology: bool,
    /// Exact number of triangle boundary segments.
    pub boundary_segments: usize,
    /// Whether volume or face-sense evidence proves the output orientation.
    pub orientation_valid: bool,
    /// Whether volume or area remains within the fixture's reviewed error bound.
    pub measure_within_tolerance: bool,
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
        digest.boolean(self.manifold);
        digest.boolean(self.boundary_matches_topology);
        digest.count(self.boundary_segments);
        digest.boolean(self.orientation_valid);
        digest.boolean(self.measure_within_tolerance);
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
    let (body, exact_measure, minimum_measure_ratio) = match case.fixture_kind {
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
        FixtureKind::MixedStoreCylinder => {
            let block_frame = Frame::new(
                Point3::new(-12.0, 3.0, -4.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .expect("valid mixed-store block frame");
            make::block(&mut store, &block_frame, [1.0, 2.0, 3.0])
                .expect("valid mixed-store block");
            let body = make::cylinder(&mut store, &frame, 1.3, 2.0)
                .expect("valid mixed-store target cylinder");
            let sphere_frame = Frame::new(
                Point3::new(11.0, -2.0, 5.0),
                Vec3::new(0.0, 1.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .expect("valid mixed-store sphere frame");
            make::sphere(&mut store, &sphere_frame, 0.75).expect("valid mixed-store sphere");
            (body, core::f64::consts::PI * 1.3 * 1.3 * 2.0, 0.98)
        }
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
        FixtureKind::ImportedCylinder => {
            assert_eq!(IMPORTED_CYLINDER_BYTES.len(), 2_309);
            let mut source_digest = StableHasher::new();
            source_digest.bytes(IMPORTED_CYLINDER_BYTES);
            assert_eq!(
                source_digest.finish(),
                IMPORTED_CYLINDER_BYTE_DIGEST,
                "certified Q3 cylinder fixture bytes drifted"
            );
            let reconstruction = kxt::import(IMPORTED_CYLINDER_BYTES, &mut store)
                .expect("certified Q3 cylinder fixture must import");
            assert!(reconstruction.skipped.is_empty());
            assert_eq!(reconstruction.bodies.len(), 1);
            let body = reconstruction.bodies[0];
            assert_eq!(store.faces_of_body(body).expect("live cylinder").len(), 3);
            let cylinder_faces = store
                .faces_of_body(body)
                .expect("certified Q3 cylinder body must be live")
                .into_iter()
                .filter(|&face| {
                    let surface = store.get(face).expect("live face").surface;
                    matches!(
                        store.get(surface).expect("live surface"),
                        SurfaceGeom::Cylinder(_)
                    )
                })
                .count();
            assert_eq!(cylinder_faces, 1);
            let minimum_volume_ratio = match case.chord_tol.to_bits() {
                bits if bits == 1.0e-2_f64.to_bits() => 0.94,
                bits if bits == 3.0e-3_f64.to_bits() => 0.98,
                bits if bits == 1.0e-3_f64.to_bits() => 0.99,
                bits if bits == 3.0e-4_f64.to_bits() => 0.998,
                _ => unreachable!("reviewed imported-cylinder tolerance"),
            };
            (
                body,
                core::f64::consts::PI * 0.13 * 0.13 * 0.2,
                minimum_volume_ratio,
            )
        }
        FixtureKind::CurvedNurbsBlock => {
            assert_eq!(CURVED_NURBS_BLOCK_BYTES.len(), 6_785);
            let mut source_digest = StableHasher::new();
            source_digest.bytes(CURVED_NURBS_BLOCK_BYTES);
            assert_eq!(
                source_digest.finish(),
                CURVED_NURBS_BLOCK_BYTE_DIGEST,
                "local curved-NURBS fixture bytes drifted"
            );
            let reconstruction = kxt::import(CURVED_NURBS_BLOCK_BYTES, &mut store)
                .expect("local curved-NURBS block must import");
            assert!(reconstruction.skipped.is_empty());
            assert_eq!(reconstruction.bodies.len(), 1);
            let body = reconstruction.bodies[0];
            assert_eq!(store.get(body).expect("live body").kind(), BodyKind::Solid);
            let faces = store.faces_of_body(body).expect("live curved block");
            let edges = store.edges_of_body(body).expect("live curved block");
            let vertices = store.vertices_of_body(body).expect("live curved block");
            assert_eq!((faces.len(), edges.len(), vertices.len()), (6, 12, 8));
            let nurbs: Vec<_> = faces
                .into_iter()
                .filter_map(|face| {
                    match store
                        .get(store.get(face).expect("live face").surface)
                        .expect("live surface")
                    {
                        SurfaceGeom::Nurbs(surface) => Some(surface),
                        _ => None,
                    }
                })
                .collect();
            assert_eq!(nurbs.len(), 1);
            let nurbs = nurbs[0];
            assert_eq!((nurbs.degree_u(), nurbs.degree_v()), (2, 2));
            assert_eq!(nurbs.net_size(), (3, 3));
            assert!(!nurbs.is_rational());
            let points = nurbs.points();
            let boundary_normal = (points[2] - points[0])
                .cross(points[6] - points[0])
                .normalized()
                .expect("curved patch boundary spans a plane");
            let center_offset = (points[4] - points[0]).dot(boundary_normal).abs();
            assert!((center_offset - 0.04).abs() <= 1.0e-12);
            for (index, point) in points.iter().enumerate() {
                if index != 4 {
                    assert!(((*point - points[0]).dot(boundary_normal)).abs() <= 1.0e-12);
                }
            }
            let minimum_volume_ratio = match case.chord_tol.to_bits() {
                bits if bits == 1.0e-2_f64.to_bits() => 0.988,
                bits if bits == 3.0e-3_f64.to_bits() => 0.997,
                bits if bits == 1.0e-3_f64.to_bits() => 0.999_7,
                bits if bits == 5.0e-4_f64.to_bits() => 0.999_4,
                _ => unreachable!("reviewed curved-NURBS tolerance"),
            };
            (body, 0.024_266_666_666_666_67, minimum_volume_ratio)
        }
        FixtureKind::ImportedPlaneSheet => {
            assert_eq!(IMPORTED_PLANE_SHEET_BYTES.len(), 3_113);
            let mut source_digest = StableHasher::new();
            source_digest.bytes(IMPORTED_PLANE_SHEET_BYTES);
            assert_eq!(source_digest.finish(), IMPORTED_PLANE_SHEET_BYTE_DIGEST);
            let reconstruction = kxt::import(IMPORTED_PLANE_SHEET_BYTES, &mut store)
                .expect("certified planar sheet must import");
            assert!(reconstruction.skipped.is_empty());
            assert_eq!(reconstruction.bodies.len(), 1);
            let body = reconstruction.bodies[0];
            assert_eq!(store.get(body).expect("live body").kind(), BodyKind::Sheet);
            let faces = store.faces_of_body(body).expect("live planar sheet");
            let edges = store.edges_of_body(body).expect("live planar sheet");
            let vertices = store.vertices_of_body(body).expect("live planar sheet");
            assert_eq!((faces.len(), edges.len(), vertices.len()), (1, 6, 6));
            assert!(matches!(
                store
                    .get(store.get(faces[0]).expect("live face").surface)
                    .expect("live surface"),
                SurfaceGeom::Plane(_)
            ));
            assert!(
                edges.into_iter().all(|edge| store
                    .get(edge)
                    .expect("live boundary edge")
                    .fins()
                    .len()
                    == 1)
            );
            (body, 0.09, 1.0 - 1.0e-12)
        }
        FixtureKind::ImportedCylinderSheet => {
            assert_eq!(IMPORTED_CYLINDER_SHEET_BYTES.len(), 2_209);
            let mut source_digest = StableHasher::new();
            source_digest.bytes(IMPORTED_CYLINDER_SHEET_BYTES);
            assert_eq!(source_digest.finish(), IMPORTED_CYLINDER_SHEET_BYTE_DIGEST);
            let reconstruction = kxt::import(IMPORTED_CYLINDER_SHEET_BYTES, &mut store)
                .expect("certified cylindrical sheet must import");
            assert!(reconstruction.skipped.is_empty());
            assert_eq!(reconstruction.bodies.len(), 1);
            let body = reconstruction.bodies[0];
            assert_eq!(store.get(body).expect("live body").kind(), BodyKind::Sheet);
            let faces = store.faces_of_body(body).expect("live cylindrical sheet");
            let edges = store.edges_of_body(body).expect("live cylindrical sheet");
            let vertices = store
                .vertices_of_body(body)
                .expect("live cylindrical sheet");
            assert_eq!((faces.len(), edges.len(), vertices.len()), (1, 3, 2));
            assert!(matches!(
                store
                    .get(store.get(faces[0]).expect("live face").surface)
                    .expect("live surface"),
                SurfaceGeom::Cylinder(_)
            ));
            let fin_counts: Vec<_> = edges
                .into_iter()
                .map(|edge| store.get(edge).expect("live cylinder edge").fins().len())
                .collect();
            assert_eq!(fin_counts.iter().filter(|&&count| count == 2).count(), 1);
            assert_eq!(fin_counts.iter().filter(|&&count| count == 1).count(), 2);
            let minimum_area_ratio = match case.chord_tol.to_bits() {
                bits if bits == 1.0e-2_f64.to_bits() => 1.001,
                bits if bits == 3.0e-3_f64.to_bits() => 0.994,
                bits if bits == 1.0e-3_f64.to_bits() => 1.0,
                bits if bits == 3.0e-4_f64.to_bits() => 1.0,
                _ => unreachable!("reviewed cylindrical-sheet tolerance"),
            };
            (
                body,
                core::f64::consts::TAU * 0.13 * 0.2,
                minimum_area_ratio,
            )
        }
    };
    assert_eq!(
        store.count::<Body>(),
        case.expected_store_bodies,
        "prepared Q3 store body count drifted"
    );
    let expected_faces = store.faces_of_body(body).expect("valid body");
    let expected_edges = store.edges_of_body(body).expect("valid body");
    let source_faces = expected_faces.len();
    let source_edges = expected_edges.len();
    let source_vertices = store.vertices_of_body(body).expect("valid body").len();
    let maximum_measure_ratio = match case.fixture_kind {
        FixtureKind::ImportedCylinderSheet if case.chord_tol.to_bits() == 1.0e-2_f64.to_bits() => {
            1.002
        }
        FixtureKind::ImportedCylinderSheet if case.chord_tol.to_bits() == 3.0e-3_f64.to_bits() => {
            0.996
        }
        FixtureKind::ImportedCylinderSheet if case.chord_tol.to_bits() == 1.0e-3_f64.to_bits() => {
            1.001
        }
        FixtureKind::ImportedCylinderSheet => 1.002,
        _ => 1.0 + 1.0e-9,
    };
    BodyTessellationFixture {
        store,
        body,
        validation: case.fixture_kind.validation(),
        exact_measure,
        minimum_measure_ratio,
        maximum_measure_ratio,
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
    assert!(evidence.manifold);
    assert!(evidence.boundary_matches_topology);
    assert!(evidence.orientation_valid);
    assert!(evidence.measure_within_tolerance);
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
    assert_eq!(evidence.boundary_segments, case.expected_boundary_segments);
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
    fn registry_contains_exactly_thirty_two_unique_canonical_cases() {
        assert_eq!(CASES.len(), 32);
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
            assert_eq!(
                entry["size_parameters"]["bodies"].as_u64(),
                Some(case.expected_store_bodies as u64)
            );
            assert_eq!(
                entry["tolerances"]["chord_tol"].as_f64(),
                Some(case.chord_tol)
            );
            assert_eq!(entry["policy_values"]["max_edge_len"], "unbounded");
            assert_eq!(
                entry["policy_values"]["validation"],
                match case.fixture_kind.validation() {
                    ValidationKind::ClosedSolid => "closed-solid",
                    ValidationKind::OrientedSheet => "oriented-sheet",
                }
            );
            assert_eq!(entry["policy_values"]["api"], API_IDENTITY);
            assert_eq!(entry["policy_values"]["budget_profile"], PROFILE_IDENTITY);
            assert_eq!(entry["policy_values"]["execution"], EXECUTION_IDENTITY);
            assert_eq!(entry["policy_values"]["policy_version"], "v1");
            assert_eq!(entry["policy_values"]["usage_contract"], "q3-usage.v1");
            let (body_kind, measure, orientation_proof) = match case.fixture_kind.validation() {
                ValidationKind::ClosedSolid => ("solid", "signed-volume", "positive-signed-volume"),
                ValidationKind::OrientedSheet => (
                    "sheet",
                    "faceted-surface-area",
                    "surface-normal-times-face-sense",
                ),
            };
            assert_eq!(entry["policy_values"]["body_kind"], body_kind);
            assert_eq!(entry["policy_values"]["measure"], measure);
            assert_eq!(
                entry["policy_values"]["orientation_proof"],
                orientation_proof
            );
            assert_eq!(
                entry["policy_values"]["incidence_proof"],
                "directed-manifold+exact-topological-boundary"
            );
            match case.fixture_kind.validation() {
                ValidationKind::ClosedSolid => {
                    assert!(entry["policy_values"]["orientation_dust_threshold"].is_null());
                }
                ValidationKind::OrientedSheet => {
                    assert_eq!(
                        entry["policy_values"]["orientation_dust_threshold"],
                        "64*epsilon*exact-measure"
                    );
                }
            }
            let expected_floor = match case.fixture_kind {
                FixtureKind::Block
                | FixtureKind::ImportedNurbsFace
                | FixtureKind::ImportedTolerantEdge
                | FixtureKind::ImportedPlaneSheet => 1.0 - 1.0e-12,
                FixtureKind::Cylinder
                | FixtureKind::MixedStoreCylinder
                | FixtureKind::Cone
                | FixtureKind::Sphere
                | FixtureKind::Torus => 0.98,
                FixtureKind::ImportedCylinder => match case.chord_tol.to_bits() {
                    bits if bits == 1.0e-2_f64.to_bits() => 0.94,
                    bits if bits == 3.0e-3_f64.to_bits() => 0.98,
                    bits if bits == 1.0e-3_f64.to_bits() => 0.99,
                    bits if bits == 3.0e-4_f64.to_bits() => 0.998,
                    _ => unreachable!("reviewed imported-cylinder tolerance"),
                },
                FixtureKind::CurvedNurbsBlock => match case.chord_tol.to_bits() {
                    bits if bits == 1.0e-2_f64.to_bits() => 0.988,
                    bits if bits == 3.0e-3_f64.to_bits() => 0.997,
                    bits if bits == 1.0e-3_f64.to_bits() => 0.999_7,
                    bits if bits == 5.0e-4_f64.to_bits() => 0.999_4,
                    _ => unreachable!("reviewed curved-NURBS tolerance"),
                },
                FixtureKind::ImportedCylinderSheet => match case.chord_tol.to_bits() {
                    bits if bits == 1.0e-2_f64.to_bits() => 1.001,
                    bits if bits == 3.0e-3_f64.to_bits() => 0.994,
                    bits if bits == 1.0e-3_f64.to_bits() => 1.0,
                    bits if bits == 3.0e-4_f64.to_bits() => 1.0,
                    _ => unreachable!("reviewed cylindrical-sheet tolerance"),
                },
            };
            let expected_ceiling = match case.fixture_kind {
                FixtureKind::ImportedCylinderSheet => match case.chord_tol.to_bits() {
                    bits if bits == 1.0e-2_f64.to_bits() => 1.002,
                    bits if bits == 3.0e-3_f64.to_bits() => 0.996,
                    bits if bits == 1.0e-3_f64.to_bits() => 1.001,
                    bits if bits == 3.0e-4_f64.to_bits() => 1.002,
                    _ => unreachable!("reviewed cylindrical-sheet tolerance"),
                },
                _ => 1.0 + 1.0e-9,
            };
            assert_eq!(
                entry["policy_values"]["measure_ratio_floor"].as_f64(),
                Some(expected_floor)
            );
            assert_eq!(
                entry["policy_values"]["measure_ratio_ceiling"].as_f64(),
                Some(expected_ceiling)
            );
            let expected_source_evidence = match case.fixture_kind {
                FixtureKind::ImportedNurbsFace
                | FixtureKind::ImportedTolerantEdge
                | FixtureKind::ImportedCylinder
                | FixtureKind::ImportedPlaneSheet
                | FixtureKind::ImportedCylinderSheet => {
                    serde_json::Value::String(HOST_ACCEPTED_SOURCE_EVIDENCE.into())
                }
                FixtureKind::CurvedNurbsBlock => {
                    serde_json::Value::String(CURVED_NURBS_BLOCK_SOURCE_EVIDENCE.into())
                }
                _ => serde_json::Value::Null,
            };
            assert_eq!(
                entry["policy_values"]["source_evidence"],
                expected_source_evidence
            );
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
                FixtureKind::ImportedCylinder => {
                    assert_eq!(
                        entry["size_parameters"]["input_bytes"].as_u64(),
                        Some(IMPORTED_CYLINDER_BYTES.len() as u64)
                    );
                    assert_eq!(
                        entry["policy_values"]["source_fixture"],
                        IMPORTED_CYLINDER_IDENTITY
                    );
                    assert_eq!(
                        entry["policy_values"]["source_sha256"],
                        IMPORTED_CYLINDER_SHA256
                    );
                }
                FixtureKind::CurvedNurbsBlock => {
                    assert_eq!(
                        entry["size_parameters"]["input_bytes"].as_u64(),
                        Some(CURVED_NURBS_BLOCK_BYTES.len() as u64)
                    );
                    assert_eq!(
                        entry["policy_values"]["source_fixture"],
                        CURVED_NURBS_BLOCK_IDENTITY
                    );
                    assert_eq!(
                        entry["policy_values"]["source_sha256"],
                        CURVED_NURBS_BLOCK_SHA256
                    );
                    assert_eq!(
                        entry["policy_values"]["source_evidence"],
                        CURVED_NURBS_BLOCK_SOURCE_EVIDENCE
                    );
                    assert_eq!(
                        entry["policy_values"]["rejected_finer_tier"],
                        if case.chord_tol.to_bits() == 5.0e-4_f64.to_bits() {
                            serde_json::Value::String(
                                "chord-3e-4;interior-refinement-passes=25;allowed=24".into(),
                            )
                        } else {
                            serde_json::Value::Null
                        }
                    );
                }
                FixtureKind::ImportedPlaneSheet => {
                    assert_eq!(
                        entry["size_parameters"]["input_bytes"].as_u64(),
                        Some(IMPORTED_PLANE_SHEET_BYTES.len() as u64)
                    );
                    assert_eq!(
                        entry["policy_values"]["source_fixture"],
                        IMPORTED_PLANE_SHEET_IDENTITY
                    );
                    assert_eq!(
                        entry["policy_values"]["source_sha256"],
                        IMPORTED_PLANE_SHEET_SHA256
                    );
                }
                FixtureKind::ImportedCylinderSheet => {
                    assert_eq!(
                        entry["size_parameters"]["input_bytes"].as_u64(),
                        Some(IMPORTED_CYLINDER_SHEET_BYTES.len() as u64)
                    );
                    assert_eq!(
                        entry["policy_values"]["source_fixture"],
                        IMPORTED_CYLINDER_SHEET_IDENTITY
                    );
                    assert_eq!(
                        entry["policy_values"]["source_sha256"],
                        IMPORTED_CYLINDER_SHEET_SHA256
                    );
                }
                FixtureKind::MixedStoreCylinder => {
                    assert!(entry["size_parameters"]["input_bytes"].is_null());
                    assert_eq!(
                        entry["policy_values"]["store_shape"],
                        "block-cylinder-sphere; target=cylinder"
                    );
                    assert!(entry["policy_values"]["source_fixture"].is_null());
                    assert!(entry["policy_values"]["source_sha256"].is_null());
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
            assert_eq!(
                counters["boundary_segments"].as_u64(),
                Some(case.expected_boundary_segments as u64)
            );
            for field in [
                "positions_finite",
                "indices_valid",
                "owner_mapping_valid",
                "manifold",
                "boundary_matches_topology",
                "orientation_valid",
                "measure_within_tolerance",
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
    fn bounded_v1_admits_the_complete_matrix_and_pins_the_measured_root_crossing() {
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_budget_overrides(BodyTessellationBudgetProfile::bounded_v1());
        let compatibility = BodyTessellationBudgetProfile::v1_defaults();
        let mut maximum_root_work = 0;
        let mut maximum_path = None;

        for case in CASES {
            let fixture = fixture(case);
            let outcome = fixture
                .tessellate_outcome(&tessellation_options(case.chord_tol), &context)
                .unwrap();
            let (result, report) = outcome.into_parts();
            let mesh = result
                .unwrap_or_else(|error| panic!("bounded preset rejected {}: {error:?}", case.path));
            assert_eq!(
                fixture.mesh_digest(&mesh),
                case.expected_mesh_digest,
                "{}",
                case.path
            );
            assert!(report.limit_events().is_empty(), "{}", case.path);
            for (limit, expected) in compatibility.limits().iter().zip(case.expected_usage) {
                let actual = report
                    .usage()
                    .iter()
                    .find(|snapshot| snapshot.stage == limit.stage)
                    .unwrap_or_else(|| {
                        panic!("missing {} for {}", limit.stage.as_str(), case.path)
                    });
                assert_eq!(actual.consumed, expected, "{}", case.path);
            }
            let expected_root_work = compatibility
                .limits()
                .iter()
                .zip(case.expected_usage)
                .filter(|(limit, _)| limit.resource == ResourceKind::Work)
                .map(|(_, consumed)| consumed)
                .sum::<u64>();
            let root = report
                .usage()
                .iter()
                .find(|snapshot| snapshot.stage == TOTAL_WORK_STAGE)
                .unwrap();
            assert_eq!(root.consumed, expected_root_work, "{}", case.path);
            if root.consumed > maximum_root_work {
                maximum_root_work = root.consumed;
                maximum_path = Some(case.path);
            }
        }

        assert_eq!(maximum_root_work, 2_822);
        assert_eq!(
            maximum_path,
            Some("topology/body-tessellation/cylinder-v3/1/chord-1e-3-v3")
        );

        let case = CASES
            .into_iter()
            .find(|case| case.path == maximum_path.unwrap())
            .unwrap();
        let fixture = fixture(case);
        let run = |allowed| {
            let context = OperationContext::new(&session, Tolerances::default())
                .unwrap()
                .with_budget_overrides(
                    BodyTessellationBudgetProfile::bounded_v1().with_total_work_limit(allowed),
                );
            fixture
                .tessellate_outcome(&tessellation_options(case.chord_tol), &context)
                .unwrap()
        };
        assert!(run(2_822).result().is_ok());
        let denied = run(2_821);
        assert!(denied.result().is_err());
        assert_eq!(denied.report().limit_events().len(), 1);
        let snapshot = denied.report().limit_events()[0];
        assert_eq!(snapshot.stage, TOTAL_WORK_STAGE);
        assert_eq!(snapshot.resource, ResourceKind::Work);
        assert_eq!(snapshot.consumed, 2_822);
        assert_eq!(snapshot.allowed, 2_821);
    }

    #[test]
    fn mixed_store_target_is_isolated_from_unrelated_bodies() {
        let standalone_case = CASES
            .into_iter()
            .find(|case| {
                case.fixture_kind == FixtureKind::Cylinder
                    && case.chord_tol.to_bits() == 1.0e-2_f64.to_bits()
            })
            .unwrap();
        let mixed_case = CASES
            .into_iter()
            .find(|case| {
                case.fixture_kind == FixtureKind::MixedStoreCylinder
                    && case.chord_tol.to_bits() == standalone_case.chord_tol.to_bits()
            })
            .unwrap();
        let standalone = fixture(standalone_case);
        let mixed = fixture(mixed_case);
        assert_eq!(standalone.store.count::<Body>(), 1);
        assert_eq!(mixed.store.count::<Body>(), 3);
        assert_ne!(
            standalone.body, mixed.body,
            "mixed setup must shift identity"
        );

        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let standalone_run = standalone.tessellate(standalone_case.chord_tol, &context);
        let mixed_run = mixed.tessellate(mixed_case.chord_tol, &context);
        assert_eq!(mixed_run.report, standalone_run.report);
        assert_eq!(
            mixed.evidence(&mixed_run),
            standalone.evidence(&standalone_run)
        );
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

    #[test]
    fn missing_solid_triangle_is_neither_manifold_nor_topological_boundary() {
        let case = CASES[0];
        let fixture = fixture(case);
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let run = fixture.tessellate(case.chord_tol, &context);
        let mut mesh = run.mesh;
        mesh.triangles.pop().unwrap();
        mesh.face_ranges.last_mut().unwrap().1.end -= 1;
        let evidence = fixture.evidence(&BodyTessellationRun {
            mesh,
            report: run.report,
        });
        assert!(!evidence.manifold);
        assert!(!evidence.boundary_matches_topology);
        assert_ne!(evidence.boundary_segments, 0);
    }

    #[test]
    fn materially_reversed_sheet_triangle_breaks_direction_and_face_sense() {
        let case = CASES
            .into_iter()
            .find(|case| case.fixture_kind == FixtureKind::ImportedPlaneSheet)
            .unwrap();
        let fixture = fixture(case);
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let run = fixture.tessellate(case.chord_tol, &context);
        let mut mesh = run.mesh;
        let largest = mesh
            .triangles
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| {
                let area = |triangle: &&[u32; 3]| {
                    let [p0, p1, p2] = triangle.map(|index| mesh.positions[index as usize]);
                    (p1 - p0).cross(p2 - p0).norm()
                };
                area(a).total_cmp(&area(b))
            })
            .map(|(index, _)| index)
            .unwrap();
        mesh.triangles[largest].swap(1, 2);
        let evidence = fixture.evidence(&BodyTessellationRun {
            mesh,
            report: run.report,
        });
        assert!(!evidence.manifold);
        assert!(evidence.boundary_matches_topology);
        assert!(!evidence.orientation_valid);
    }

    #[test]
    fn cylinder_seam_is_excluded_from_the_sheet_boundary() {
        let case = CASES
            .into_iter()
            .find(|case| {
                case.fixture_kind == FixtureKind::ImportedCylinderSheet
                    && case.chord_tol.to_bits() == 1.0e-2_f64.to_bits()
            })
            .unwrap();
        let fixture = fixture(case);
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let run = fixture.tessellate(case.chord_tol, &context);
        let seam_segment = run
            .mesh
            .edge_polylines
            .iter()
            .find(|(edge, _)| fixture.store.get(*edge).unwrap().fins().len() == 2)
            .and_then(|(_, polyline)| polyline.windows(2).next())
            .map(|pair| (pair[0].min(pair[1]), pair[0].max(pair[1])))
            .unwrap();
        let mut mesh = run.mesh;
        let adjacent = mesh
            .triangles
            .iter()
            .position(|triangle| {
                [
                    (triangle[0], triangle[1]),
                    (triangle[1], triangle[2]),
                    (triangle[2], triangle[0]),
                ]
                .into_iter()
                .any(|(a, b)| (a.min(b), a.max(b)) == seam_segment)
            })
            .unwrap();
        mesh.triangles.remove(adjacent);
        mesh.face_ranges[0].1.end -= 1;
        let evidence = fixture.evidence(&BodyTessellationRun {
            mesh,
            report: run.report,
        });
        assert!(evidence.manifold);
        assert!(!evidence.boundary_matches_topology);
        assert!(evidence.boundary_segments > case.expected_boundary_segments);
    }

    #[test]
    fn degenerate_and_duplicate_triangle_incidence_are_rejected() {
        let case = CASES[0];
        let fixture = fixture(case);
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let run = fixture.tessellate(case.chord_tol, &context);

        let mut degenerate = run.mesh.clone();
        degenerate.triangles[0][1] = degenerate.triangles[0][0];
        let degenerate = fixture.evidence(&BodyTessellationRun {
            mesh: degenerate,
            report: run.report.clone(),
        });
        assert!(!degenerate.manifold);

        let mut duplicate = run.mesh;
        duplicate.triangles.push(duplicate.triangles[0]);
        duplicate.face_ranges.last_mut().unwrap().1.end += 1;
        let duplicate = fixture.evidence(&BodyTessellationRun {
            mesh: duplicate,
            report: run.report,
        });
        assert!(!duplicate.manifold);
    }

    #[test]
    fn sheet_area_drift_is_rejected_without_invalidating_orientation() {
        let case = CASES
            .into_iter()
            .find(|case| case.fixture_kind == FixtureKind::ImportedPlaneSheet)
            .unwrap();
        let fixture = fixture(case);
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let run = fixture.tessellate(case.chord_tol, &context);
        let mut scaled = run.mesh;
        for point in &mut scaled.positions {
            *point = Point3::new(point.x * 2.0, point.y * 2.0, point.z * 2.0);
        }
        let scaled = fixture.evidence(&BodyTessellationRun {
            mesh: scaled,
            report: run.report,
        });
        assert!(scaled.orientation_valid);
        assert!(!scaled.measure_within_tolerance);
    }
}
