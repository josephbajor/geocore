//! Deterministic Q3 standalone face-tessellation matrix fixtures and evidence.

#[cfg(test)]
use kcore::operation::TOTAL_WORK_STAGE;
use kcore::operation::{
    AccountingMode, ExecutionPolicy, NumericalPolicy, OperationContext, OperationOutcome,
    OperationPolicyError, OperationReport, PolicyVersion, ResourceKind, SessionPolicy,
    SessionPrecision, StageId,
};
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsSurface;
use kgeom::surface::{Cylinder, Plane, Surface};
use kgeom::tess::{
    FACE_TESSELLATION_BOUNDARY_DEPTH, FACE_TESSELLATION_BOUNDARY_SPLITS,
    FACE_TESSELLATION_MESH_TRIANGLES, FACE_TESSELLATION_MESH_VERTICES,
    FACE_TESSELLATION_REFINEMENT_PASSES, FaceMesh, FaceTessellationBudgetProfile, TessOptions,
    TrimLoop, TrimmedSurface, tessellate_with_context,
};
use kgeom::vec::{Point3, Vec2};

/// Fixture identity for the standalone Q3 face matrix.
pub const FIXTURE_VERSION: &str = "face-tessellation.v2";
/// Deterministic fixture seed; construction itself is not randomized.
pub const FIXTURE_SEED: u64 = 0x5154_4641_4345_0007;
/// Public entry point measured by the standalone face contract.
pub const API_IDENTITY: &str = "tessellate_with_context";
/// Complete face-family defaults used by this contract.
pub const PROFILE_IDENTITY: &str = "face-tessellation.compatibility-v1";
/// Deterministic execution policy used by this contract.
pub const EXECUTION_IDENTITY: &str = "serial";
/// Canonical number of standalone face-tessellation usage stages.
pub const USAGE_STAGE_COUNT: usize = 5;
/// Maximum number of trim loops in the matrix.
pub const MAX_TRIM_LOOPS: usize = 4;
const AREA_SCALE: f64 = 1_000_000_000.0;
const CANONICAL_STAGES: [StageId; USAGE_STAGE_COUNT] = [
    FACE_TESSELLATION_BOUNDARY_DEPTH,
    FACE_TESSELLATION_BOUNDARY_SPLITS,
    FACE_TESSELLATION_REFINEMENT_PASSES,
    FACE_TESSELLATION_MESH_TRIANGLES,
    FACE_TESSELLATION_MESH_VERTICES,
];

/// Surface representation exercised by a standalone face case.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceRepresentation {
    /// Exact analytic plane.
    Plane,
    /// Exact analytic half-cylinder parameter range.
    HalfCylinder,
    /// Genuinely curved rational quadratic NURBS patch.
    RationalNurbs,
}

impl SurfaceRepresentation {
    /// Stable representation identity used by paths and evidence.
    pub const fn identity(self) -> &'static str {
        match self {
            Self::Plane => "plane-v2",
            Self::HalfCylinder => "half-cylinder-v2",
            Self::RationalNurbs => "rational-nurbs-v2",
        }
    }
}

/// Parameter-space trim topology exercised by a standalone face case.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrimShape {
    /// Outer rectangle only.
    Outer,
    /// Outer rectangle with one rectangular hole.
    OneHole,
    /// Outer rectangle with three disjoint rectangular holes.
    ThreeHoles,
}

impl TrimShape {
    /// Stable trim identity used by paths and evidence.
    pub const fn identity(self) -> &'static str {
        match self {
            Self::Outer => "outer",
            Self::OneHole => "one-hole",
            Self::ThreeHoles => "three-holes",
        }
    }

    /// Number of loops: one outer loop followed by zero or more holes.
    pub const fn loop_count(self) -> usize {
        match self {
            Self::Outer => 1,
            Self::OneHole => 2,
            Self::ThreeHoles => 4,
        }
    }
}

/// Stable standalone face-tessellation case definition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceTessellationCase {
    /// Five-segment benchmark path.
    pub path: &'static str,
    /// Surface representation.
    pub representation: SurfaceRepresentation,
    /// Parameter-space trim topology.
    pub trim_shape: TrimShape,
    /// Chordal tessellation tolerance.
    pub chord_tol: f64,
    /// Reviewed exact evidence for the completed run.
    pub expected: FaceTessellationExpected,
}

/// Reviewed exact evidence carried by a standalone face case.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaceTessellationExpected {
    /// Reviewed output vertex count.
    pub mesh_vertices: usize,
    /// Reviewed output triangle count.
    pub mesh_triangles: usize,
    /// Reviewed refined vertex count for each loop; unused slots are zero.
    pub boundary_loop_vertices: [usize; MAX_TRIM_LOOPS],
    /// Reviewed digest of the source trim loops.
    pub trim_digest: u64,
    /// Reviewed digest of the refined output boundary loops.
    pub boundary_digest: u64,
    /// Reviewed parameter-space face area in billionths of a square unit.
    pub parameter_area_units: u64,
    /// Reviewed faceted model-space area in billionths of a square meter.
    pub model_area_units: u64,
    /// Reviewed mesh digest.
    pub mesh_digest: u64,
    /// Reviewed complete semantic output digest.
    pub output_digest: u64,
    /// Reviewed consumed values in canonical face-profile order.
    pub usage: [u64; USAGE_STAGE_COUNT],
    /// Reviewed portable usage digest.
    pub usage_digest: u64,
}

macro_rules! case {
    ($representation:ident, $trim:ident, $rep_path:literal, $trim_path:literal, $tol_path:literal, $tol:expr, $expected:expr) => {
        FaceTessellationCase {
            path: concat!(
                "geometry/face-tessellation/",
                $rep_path,
                "/",
                $trim_path,
                "/chord-",
                $tol_path,
                "-v2"
            ),
            representation: SurfaceRepresentation::$representation,
            trim_shape: TrimShape::$trim,
            chord_tol: $tol,
            expected: $expected,
        }
    };
}

#[allow(
    clippy::too_many_arguments,
    reason = "one ordered helper keeps each reviewed matrix golden compact and auditable"
)]
const fn expected(
    mesh_vertices: usize,
    mesh_triangles: usize,
    boundary_loop_vertices: [usize; MAX_TRIM_LOOPS],
    trim_digest: u64,
    boundary_digest: u64,
    parameter_area_units: u64,
    model_area_units: u64,
    mesh_digest: u64,
    output_digest: u64,
    usage: [u64; USAGE_STAGE_COUNT],
    usage_digest: u64,
) -> FaceTessellationExpected {
    FaceTessellationExpected {
        mesh_vertices,
        mesh_triangles,
        boundary_loop_vertices,
        trim_digest,
        boundary_digest,
        parameter_area_units,
        model_area_units,
        mesh_digest,
        output_digest,
        usage,
        usage_digest,
    }
}

const PLANE_OUTER: FaceTessellationExpected = expected(
    4,
    2,
    [4, 0, 0, 0],
    0x4b17_17c9_c0ac_29ea,
    0x2928_d648_6d44_4958,
    6_283_185_307,
    6_283_185_307,
    0xd8c6_7e8d_93ee_d369,
    0x36c7_8470_4e83_6733,
    [0, 0, 0, 2, 4],
    0xfabd_c67a_6ea3_0b26,
);
const PLANE_ONE_HOLE: FaceTessellationExpected = expected(
    8,
    8,
    [4, 4, 0, 0],
    0x9124_5d5e_d35d_d8a5,
    0xd323_1d56_c617_0057,
    5_445_427_266,
    5_445_427_266,
    0x1d5a_3f5d_35c9_b2f6,
    0x54dc_047d_60d7_4377,
    [0, 0, 0, 8, 8],
    0x4f82_14a7_d786_6f2c,
);
const PLANE_THREE_HOLES: FaceTessellationExpected = expected(
    16,
    20,
    [4, 4, 4, 4],
    0x977f_5677_ed7f_9127,
    0x28a7_4f72_e861_3705,
    4_974_188_368,
    4_974_188_368,
    0x29a4_aac4_2455_846d,
    0xa947_9283_6d31_dddd,
    [0, 0, 0, 20, 16],
    0x651f_8ecd_63ea_7e30,
);
const CYLINDER_OUTER_COARSE: FaceTessellationExpected = expected(
    543,
    1_050,
    [34, 0, 0, 0],
    0x4b17_17c9_c0ac_29ea,
    0x84f9_f765_048d_01c2,
    6_283_185_307,
    12_614_511_395,
    0xea72_33bd_2220_5d26,
    0x9763_b1c9_0544_21ed,
    [4, 30, 4, 1_050, 543],
    0x0352_7d7b_7a51_1949,
);
const CYLINDER_OUTER_FINE: FaceTessellationExpected = expected(
    18_841,
    37_550,
    [130, 0, 0, 0],
    0x4b17_17c9_c0ac_29ea,
    0x4356_0349_f265_8192,
    6_283_185_307,
    12_591_608_093,
    0xf31d_11e6_3e2a_fed5,
    0x9fec_c5d7_143b_dbec,
    [6, 126, 6, 37_550, 18_841],
    0x3446_5daa_e185_880c,
);
const CYLINDER_ONE_HOLE_COARSE: FaceTessellationExpected = expected(
    540,
    1_028,
    [34, 18, 0, 0],
    0x9124_5d5e_d35d_d8a5,
    0x7e6f_b9b9_fb75_2f20,
    5_445_427_266,
    11_002_960_078,
    0xfb83_8ad8_558f_55df,
    0xb097_7f26_a716_361f,
    [4, 44, 4, 1_028, 540],
    0x3211_d961_6cba_9492,
);
const CYLINDER_ONE_HOLE_FINE: FaceTessellationExpected = expected(
    14_946,
    29_696,
    [130, 66, 0, 0],
    0x9124_5d5e_d35d_d8a5,
    0x460b_99ee_666e_bfa8,
    5_445_427_266,
    10_930_870_905,
    0xe981_5951_aefd_8117,
    0xebf1_640f_7a76_a8c0,
    [6, 188, 6, 29_696, 14_946],
    0x9e0e_71c4_562d_7d58,
);
const CYLINDER_THREE_HOLES_COARSE: FaceTessellationExpected = expected(
    467,
    874,
    [34, 10, 10, 10],
    0x977f_5677_ed7f_9127,
    0xa1c2_6215_fcac_9465,
    4_974_188_368,
    10_074_975_956,
    0xe6d4_1566_1ea8_6ad5,
    0xc72e_e9c4_63af_83dd,
    [4, 48, 4, 874, 467],
    0x1a5b_f9b0_758d_6bf7,
);
const CYLINDER_THREE_HOLES_FINE: FaceTessellationExpected = expected(
    10_774,
    21_320,
    [130, 34, 34, 34],
    0x977f_5677_ed7f_9127,
    0x6f77_bed3_00ea_04fd,
    4_974_188_368,
    9_984_072_042,
    0xaad9_edf8_610c_c7c5,
    0x7a3f_d998_70f6_27e5,
    [6, 216, 6, 21_320, 10_774],
    0x3f9d_551d_bf48_3bb9,
);
const NURBS_OUTER_COARSE: FaceTessellationExpected = expected(
    155,
    282,
    [26, 0, 0, 0],
    0x4b17_17c9_c0ac_29ea,
    0x1bf4_c49f_084e_ca52,
    6_283_185_307,
    6_290_010_630,
    0x6651_b261_eb20_f12f,
    0x8596_ed7a_62bd_b79b,
    [4, 22, 3, 282, 155],
    0x05a5_aa75_13ab_0d59,
);
const NURBS_OUTER_FINE: FaceTessellationExpected = expected(
    3_015,
    5_962,
    [66, 0, 0, 0],
    0x4b17_17c9_c0ac_29ea,
    0xcd75_dda4_470c_35ca,
    6_283_185_307,
    6_285_978_266,
    0x31ce_831d_8d0c_31e6,
    0x0a5c_562c_9f36_809a,
    [5, 62, 5, 5_962, 3_015],
    0xb3f2_b89c_205e_1f35,
);
const NURBS_ONE_HOLE_COARSE: FaceTessellationExpected = expected(
    110,
    184,
    [26, 10, 0, 0],
    0x9124_5d5e_d35d_d8a5,
    0xb80c_967e_b62a_ad08,
    5_445_427_266,
    5_408_604_575,
    0x390f_4651_06c6_07d4,
    0x1f57_db50_1d7a_15a0,
    [4, 28, 3, 184, 110],
    0x5b03_84a2_e262_7643,
);
const NURBS_ONE_HOLE_FINE: FaceTessellationExpected = expected(
    2_314,
    4_528,
    [66, 34, 0, 0],
    0x9124_5d5e_d35d_d8a5,
    0xdb5c_2780_f857_67e4,
    5_445_427_266,
    5_408_275_586,
    0x7cbc_cfe8_2bad_7745,
    0x587e_4d6f_3229_2a72,
    [5, 92, 5, 4_528, 2_314],
    0xfc3a_50ed_e4b4_627c,
);
const NURBS_THREE_HOLES_COARSE: FaceTessellationExpected = expected(
    155,
    266,
    [26, 6, 10, 6],
    0x977f_5677_ed7f_9127,
    0xf5c0_b5e6_dff4_7169,
    4_974_188_368,
    4_982_128_008,
    0x0f6c_15bb_d3ad_f7e2,
    0x4850_0e03_02b0_b5db,
    [4, 32, 3, 266, 155],
    0x220f_55cd_09ff_75cb,
);
const NURBS_THREE_HOLES_FINE: FaceTessellationExpected = expected(
    1_898,
    3_682,
    [66, 16, 18, 18],
    0x977f_5677_ed7f_9127,
    0xe398_498c_3a7c_f667,
    4_974_188_368,
    4_962_705_823,
    0x4408_d214_e835_ffa4,
    0xe722_9ca0_3cfb_f515,
    [5, 102, 5, 3_682, 1_898],
    0x81a6_4b31_e3ce_2ab7,
);

/// Complete 3 representations × 3 trim topologies × 2 tolerances matrix.
pub const CASES: [FaceTessellationCase; 18] = [
    case!(
        Plane,
        Outer,
        "plane-v2",
        "outer",
        "1e-2",
        1.0e-2,
        PLANE_OUTER
    ),
    case!(
        Plane,
        Outer,
        "plane-v2",
        "outer",
        "1e-3",
        1.0e-3,
        PLANE_OUTER
    ),
    case!(
        Plane,
        OneHole,
        "plane-v2",
        "one-hole",
        "1e-2",
        1.0e-2,
        PLANE_ONE_HOLE
    ),
    case!(
        Plane,
        OneHole,
        "plane-v2",
        "one-hole",
        "1e-3",
        1.0e-3,
        PLANE_ONE_HOLE
    ),
    case!(
        Plane,
        ThreeHoles,
        "plane-v2",
        "three-holes",
        "1e-2",
        1.0e-2,
        PLANE_THREE_HOLES
    ),
    case!(
        Plane,
        ThreeHoles,
        "plane-v2",
        "three-holes",
        "1e-3",
        1.0e-3,
        PLANE_THREE_HOLES
    ),
    case!(
        HalfCylinder,
        Outer,
        "half-cylinder-v2",
        "outer",
        "1e-2",
        1.0e-2,
        CYLINDER_OUTER_COARSE
    ),
    case!(
        HalfCylinder,
        Outer,
        "half-cylinder-v2",
        "outer",
        "1e-3",
        1.0e-3,
        CYLINDER_OUTER_FINE
    ),
    case!(
        HalfCylinder,
        OneHole,
        "half-cylinder-v2",
        "one-hole",
        "1e-2",
        1.0e-2,
        CYLINDER_ONE_HOLE_COARSE
    ),
    case!(
        HalfCylinder,
        OneHole,
        "half-cylinder-v2",
        "one-hole",
        "1e-3",
        1.0e-3,
        CYLINDER_ONE_HOLE_FINE
    ),
    case!(
        HalfCylinder,
        ThreeHoles,
        "half-cylinder-v2",
        "three-holes",
        "1e-2",
        1.0e-2,
        CYLINDER_THREE_HOLES_COARSE
    ),
    case!(
        HalfCylinder,
        ThreeHoles,
        "half-cylinder-v2",
        "three-holes",
        "1e-3",
        1.0e-3,
        CYLINDER_THREE_HOLES_FINE
    ),
    case!(
        RationalNurbs,
        Outer,
        "rational-nurbs-v2",
        "outer",
        "1e-2",
        1.0e-2,
        NURBS_OUTER_COARSE
    ),
    case!(
        RationalNurbs,
        Outer,
        "rational-nurbs-v2",
        "outer",
        "1e-3",
        1.0e-3,
        NURBS_OUTER_FINE
    ),
    case!(
        RationalNurbs,
        OneHole,
        "rational-nurbs-v2",
        "one-hole",
        "1e-2",
        1.0e-2,
        NURBS_ONE_HOLE_COARSE
    ),
    case!(
        RationalNurbs,
        OneHole,
        "rational-nurbs-v2",
        "one-hole",
        "1e-3",
        1.0e-3,
        NURBS_ONE_HOLE_FINE
    ),
    case!(
        RationalNurbs,
        ThreeHoles,
        "rational-nurbs-v2",
        "three-holes",
        "1e-2",
        1.0e-2,
        NURBS_THREE_HOLES_COARSE
    ),
    case!(
        RationalNurbs,
        ThreeHoles,
        "rational-nurbs-v2",
        "three-holes",
        "1e-3",
        1.0e-3,
        NURBS_THREE_HOLES_FINE
    ),
];

/// Construct the explicit compatibility-v1 face policy outside measured work.
pub fn compatibility_session() -> SessionPolicy {
    SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        FaceTessellationBudgetProfile::v1_defaults(),
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

/// One contextual standalone face-tessellation result and report.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceTessellationRun {
    /// Tessellated face mesh.
    pub mesh: FaceMesh,
    /// Deterministic operation accounting and diagnostics.
    pub report: OperationReport,
}

impl FaceTessellationRun {
    /// Separate the completed result from its report outside measured work.
    pub fn from_outcome(outcome: OperationOutcome<FaceMesh>) -> Self {
        let (result, report) = outcome.into_parts();
        Self {
            mesh: result.expect("reviewed Q3 face fixture must tessellate"),
            report,
        }
    }
}

enum FixtureSurface {
    Plane(Plane),
    HalfCylinder(Cylinder),
    RationalNurbs(NurbsSurface),
}

impl FixtureSurface {
    fn as_surface(&self) -> &dyn Surface {
        match self {
            Self::Plane(surface) => surface,
            Self::HalfCylinder(surface) => surface,
            Self::RationalNurbs(surface) => surface,
        }
    }
}

/// Fully constructed immutable standalone-face input.
pub struct FaceTessellationFixture {
    representation: SurfaceRepresentation,
    surface: FixtureSurface,
}

impl FaceTessellationFixture {
    /// Construct the immutable borrowed trim outside measured work.
    pub fn trimmed(&self, trim_shape: TrimShape) -> TrimmedSurface<'_> {
        TrimmedSurface::new(self.surface.as_surface(), trim_loops(trim_shape))
            .expect("reviewed Q3 face trim is valid")
    }

    /// Invoke exactly the contextual API measured by the face matrix.
    pub fn tessellate_outcome(
        &self,
        face: &TrimmedSurface<'_>,
        options: &TessOptions,
        context: &OperationContext<'_>,
    ) -> Result<OperationOutcome<FaceMesh>, OperationPolicyError> {
        tessellate_with_context(face, options, context)
    }

    /// Tessellate once through the contextual public face entry point.
    pub fn tessellate(
        &self,
        trim_shape: TrimShape,
        chord_tol: f64,
        context: &OperationContext<'_>,
    ) -> FaceTessellationRun {
        let face = self.trimmed(trim_shape);
        FaceTessellationRun::from_outcome(
            self.tessellate_outcome(&face, &tessellation_options(chord_tol), context)
                .expect("reviewed Q3 face policy must be valid"),
        )
    }

    /// Reduce one mesh, its source face, and report to stable semantic evidence.
    pub fn evidence(
        &self,
        face: &TrimmedSurface<'_>,
        run: &FaceTessellationRun,
    ) -> FaceTessellationEvidence {
        let mesh = &run.mesh;
        let positions_finite = mesh
            .positions
            .iter()
            .all(|point| point.x.is_finite() && point.y.is_finite() && point.z.is_finite());
        let uvs_finite = mesh
            .uvs
            .iter()
            .all(|uv| uv.x.is_finite() && uv.y.is_finite());
        let indices_valid = mesh
            .triangles
            .iter()
            .flatten()
            .chain(mesh.boundary.iter().flatten())
            .all(|&index| (index as usize) < mesh.positions.len());
        let coordinates_aligned = mesh.positions.len() == mesh.uvs.len();
        let triangles_oriented = coordinates_aligned
            && mesh.triangles.iter().all(|triangle| {
                let [a, b, c] = triangle.map(|index| mesh.uvs[index as usize]);
                (b - a).cross(c - a) > 0.0
            });
        let positions_on_surface = coordinates_aligned
            && mesh
                .positions
                .iter()
                .zip(&mesh.uvs)
                .all(|(&point, &uv)| point == face.surface().eval([uv.x, uv.y]));
        let triangles_follow_surface_orientation = coordinates_aligned
            && mesh.triangles.iter().all(|triangle| {
                let [a, b, c] = triangle.map(|index| mesh.positions[index as usize]);
                let centroid_uv = triangle
                    .map(|index| mesh.uvs[index as usize])
                    .into_iter()
                    .fold(Vec2::default(), |sum, uv| sum + uv)
                    / 3.0;
                let derivatives = face
                    .surface()
                    .eval_derivs([centroid_uv.x, centroid_uv.y], 1);
                (b - a)
                    .cross(c - a)
                    .dot(derivatives.du.cross(derivatives.dv))
                    > 0.0
            });
        let boundary_loop_vertices = boundary_loop_vertices(mesh);
        let boundary_vertices = boundary_loop_vertices.iter().sum();
        let boundary_retains_trim_vertices = boundary_retains_trim_vertices(face, mesh);
        let trim_digest = trim_digest(face);
        let boundary_digest = boundary_digest(mesh);
        let trim_area = trim_parameter_area(face);
        let parameter_area = mesh_parameter_area(mesh);
        let parameter_area_matches_trim =
            (parameter_area - trim_area).abs() <= 1.0e-10 * trim_area.abs().max(1.0);
        let parameter_area_units = quantized_area(parameter_area);
        let model_area = mesh_model_area(mesh);
        let model_area_finite_positive = model_area.is_finite() && model_area > 0.0;
        let model_area_units = quantized_area(model_area);
        let mesh_digest = mesh_digest(mesh);
        let (usage, usage_digest) = report_evidence(&run.report);
        let mut evidence = FaceTessellationEvidence {
            mesh_vertices: mesh.positions.len(),
            mesh_triangles: mesh.triangles.len(),
            boundary_loops: mesh.boundary.len(),
            boundary_loop_vertices,
            boundary_vertices,
            positions_finite,
            uvs_finite,
            indices_valid,
            coordinates_aligned,
            triangles_oriented,
            positions_on_surface,
            triangles_follow_surface_orientation,
            boundary_retains_trim_vertices,
            parameter_area_matches_trim,
            model_area_finite_positive,
            trim_digest,
            boundary_digest,
            parameter_area_units,
            model_area_units,
            mesh_digest,
            representation_identity: self.representation.identity(),
            trim_identity: trim_identity(face.loops().len()),
            api_identity: API_IDENTITY,
            profile_identity: PROFILE_IDENTITY,
            execution_identity: EXECUTION_IDENTITY,
            policy_version_v1: run.report.policy_version() == PolicyVersion::V1,
            usage,
            usage_digest,
            limit_event_count: run.report.limit_events().len(),
            numeric_resolution_stage_count: run.report.numeric_resolution_stages().len(),
            diagnostic_count: run.report.diagnostics().len(),
            dropped_diagnostic_count: run.report.dropped_diagnostics(),
            output_digest: 0,
        };
        evidence.output_digest = evidence.digest();
        evidence
    }
}

/// Stable evidence for one standalone face run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaceTessellationEvidence {
    /// Output vertex count.
    pub mesh_vertices: usize,
    /// Output triangle count.
    pub mesh_triangles: usize,
    /// Boundary loop count.
    pub boundary_loops: usize,
    /// Refined vertex count for each loop; unused slots are zero.
    pub boundary_loop_vertices: [usize; MAX_TRIM_LOOPS],
    /// Total refined boundary vertex count.
    pub boundary_vertices: usize,
    /// Whether all model-space positions are finite.
    pub positions_finite: bool,
    /// Whether all parameter-space positions are finite.
    pub uvs_finite: bool,
    /// Whether all triangle and boundary indices are valid.
    pub indices_valid: bool,
    /// Whether every model-space position has exactly one UV coordinate.
    pub coordinates_aligned: bool,
    /// Whether every triangle is nondegenerate and counterclockwise in UV.
    pub triangles_oriented: bool,
    /// Whether every mesh position exactly re-evaluates from its UV.
    pub positions_on_surface: bool,
    /// Whether every faceted triangle follows the local surface orientation.
    pub triangles_follow_surface_orientation: bool,
    /// Whether every source trim vertex survives in its output loop and order.
    pub boundary_retains_trim_vertices: bool,
    /// Whether triangulated parameter area agrees with the signed trim area.
    pub parameter_area_matches_trim: bool,
    /// Whether faceted model-space area is finite and positive.
    pub model_area_finite_positive: bool,
    /// Stable source-trim digest.
    pub trim_digest: u64,
    /// Stable refined-boundary digest.
    pub boundary_digest: u64,
    /// Parameter-space face area in billionths of a square unit.
    pub parameter_area_units: u64,
    /// Faceted model-space area in billionths of a square meter.
    pub model_area_units: u64,
    /// Stable mesh digest.
    pub mesh_digest: u64,
    /// Surface representation identity.
    pub representation_identity: &'static str,
    /// Trim topology identity.
    pub trim_identity: &'static str,
    /// Measured API identity.
    pub api_identity: &'static str,
    /// Budget profile identity.
    pub profile_identity: &'static str,
    /// Execution identity.
    pub execution_identity: &'static str,
    /// Whether the completed report uses policy v1.
    pub policy_version_v1: bool,
    /// Consumed values in canonical face-profile order.
    pub usage: [u64; USAGE_STAGE_COUNT],
    /// Portable profile/policy/stage usage digest.
    pub usage_digest: u64,
    /// Structured limit-event count.
    pub limit_event_count: usize,
    /// Numeric-resolution stop count.
    pub numeric_resolution_stage_count: usize,
    /// Retained diagnostic count.
    pub diagnostic_count: usize,
    /// Dropped diagnostic count.
    pub dropped_diagnostic_count: u64,
    /// Complete semantic output digest.
    pub output_digest: u64,
}

impl FaceTessellationEvidence {
    fn digest(self) -> u64 {
        let mut digest = StableHasher::new();
        digest.string("q3-face-output.v2");
        for value in [
            self.mesh_vertices,
            self.mesh_triangles,
            self.boundary_loops,
            self.boundary_vertices,
        ] {
            digest.count(value);
        }
        for value in self.boundary_loop_vertices {
            digest.count(value);
        }
        for value in [
            self.positions_finite,
            self.uvs_finite,
            self.indices_valid,
            self.coordinates_aligned,
            self.triangles_oriented,
            self.positions_on_surface,
            self.triangles_follow_surface_orientation,
            self.boundary_retains_trim_vertices,
            self.parameter_area_matches_trim,
            self.model_area_finite_positive,
        ] {
            digest.boolean(value);
        }
        for value in [
            self.trim_digest,
            self.boundary_digest,
            self.parameter_area_units,
            self.model_area_units,
            self.mesh_digest,
        ] {
            digest.u64(value);
        }
        digest.string(self.representation_identity);
        digest.string(self.trim_identity);
        digest.string(self.api_identity);
        digest.string(self.profile_identity);
        digest.string(self.execution_identity);
        digest.boolean(self.policy_version_v1);
        for value in self.usage {
            digest.u64(value);
        }
        digest.u64(self.usage_digest);
        for value in [
            self.limit_event_count,
            self.numeric_resolution_stage_count,
            self.diagnostic_count,
        ] {
            digest.count(value);
        }
        digest.u64(self.dropped_diagnostic_count);
        digest.finish()
    }
}

/// Construct the immutable fixture for one surface representation.
pub fn fixture(representation: SurfaceRepresentation) -> FaceTessellationFixture {
    let surface = match representation {
        SurfaceRepresentation::Plane => FixtureSurface::Plane(Plane::new(Frame::world())),
        SurfaceRepresentation::HalfCylinder => FixtureSurface::HalfCylinder(
            Cylinder::new(Frame::world(), 2.0).expect("valid Q3 cylinder"),
        ),
        SurfaceRepresentation::RationalNurbs => FixtureSurface::RationalNurbs(rational_nurbs()),
    };
    FaceTessellationFixture {
        representation,
        surface,
    }
}

/// Verify exact reviewed evidence for one case.
pub fn verify(case: FaceTessellationCase, evidence: FaceTessellationEvidence) {
    assert!(evidence.positions_finite);
    assert!(evidence.uvs_finite);
    assert!(evidence.indices_valid);
    assert!(evidence.coordinates_aligned);
    assert!(evidence.triangles_oriented);
    assert!(evidence.positions_on_surface);
    assert!(evidence.triangles_follow_surface_orientation);
    assert!(evidence.boundary_retains_trim_vertices);
    assert!(evidence.parameter_area_matches_trim);
    assert!(evidence.model_area_finite_positive);
    assert_eq!(evidence.boundary_loops, case.trim_shape.loop_count());
    assert_eq!(
        evidence.representation_identity,
        case.representation.identity()
    );
    assert_eq!(evidence.trim_identity, case.trim_shape.identity());
    assert_eq!(evidence.api_identity, API_IDENTITY);
    assert_eq!(evidence.profile_identity, PROFILE_IDENTITY);
    assert_eq!(evidence.execution_identity, EXECUTION_IDENTITY);
    assert!(evidence.policy_version_v1);
    assert_eq!(evidence.limit_event_count, 0);
    assert_eq!(evidence.numeric_resolution_stage_count, 0);
    assert_eq!(evidence.diagnostic_count, 0);
    assert_eq!(evidence.dropped_diagnostic_count, 0);
    assert_eq!(evidence.mesh_vertices, case.expected.mesh_vertices);
    assert_eq!(evidence.mesh_triangles, case.expected.mesh_triangles);
    assert_eq!(
        evidence.boundary_loop_vertices,
        case.expected.boundary_loop_vertices
    );
    assert_eq!(evidence.trim_digest, case.expected.trim_digest);
    assert_eq!(evidence.boundary_digest, case.expected.boundary_digest);
    assert_eq!(
        evidence.parameter_area_units,
        case.expected.parameter_area_units
    );
    assert_eq!(evidence.model_area_units, case.expected.model_area_units);
    assert_eq!(evidence.mesh_digest, case.expected.mesh_digest);
    assert_eq!(evidence.output_digest, case.expected.output_digest);
    assert_eq!(evidence.usage, case.expected.usage);
    assert_eq!(evidence.usage_digest, case.expected.usage_digest);
}

fn trim_loops(trim_shape: TrimShape) -> Vec<TrimLoop> {
    let outer = rectangle_loop(0.0, core::f64::consts::PI, 0.0, 2.0, false);
    match trim_shape {
        TrimShape::Outer => vec![outer],
        TrimShape::OneHole => vec![
            outer,
            rectangle_loop(
                core::f64::consts::PI / 3.0,
                2.0 * core::f64::consts::PI / 3.0,
                0.6,
                1.4,
                true,
            ),
        ],
        TrimShape::ThreeHoles => vec![
            outer,
            rectangle_loop(
                core::f64::consts::PI / 12.0,
                core::f64::consts::PI / 4.0,
                0.4,
                0.8,
                true,
            ),
            rectangle_loop(
                core::f64::consts::PI / 3.0,
                7.0 * core::f64::consts::PI / 12.0,
                1.2,
                1.8,
                true,
            ),
            rectangle_loop(
                2.0 * core::f64::consts::PI / 3.0,
                11.0 * core::f64::consts::PI / 12.0,
                0.2,
                1.0,
                true,
            ),
        ],
    }
}

fn rectangle_loop(u0: f64, u1: f64, v0: f64, v1: f64, clockwise: bool) -> TrimLoop {
    let mut points = vec![
        Vec2::new(u0, v0),
        Vec2::new(u1, v0),
        Vec2::new(u1, v1),
        Vec2::new(u0, v1),
    ];
    if clockwise {
        points.reverse();
    }
    TrimLoop::new(points).expect("reviewed rectangle is nondegenerate")
}

fn rational_nurbs() -> NurbsSurface {
    let pi = core::f64::consts::PI;
    let diagonal_weight = core::f64::consts::FRAC_1_SQRT_2;
    NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, pi, pi, pi],
        vec![0.0, 0.0, 2.0, 2.0],
        vec![
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 2.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(2.0, 2.0, 2.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 2.0),
        ],
        Some(vec![1.0, 1.0, diagonal_weight, diagonal_weight, 1.0, 1.0]),
    )
    .expect("valid genuinely curved rational quadratic Q3 NURBS")
}

fn trim_identity(loop_count: usize) -> &'static str {
    match loop_count {
        1 => TrimShape::Outer.identity(),
        2 => TrimShape::OneHole.identity(),
        4 => TrimShape::ThreeHoles.identity(),
        _ => panic!("Q3 matrix has one, two, or four trim loops"),
    }
}

fn boundary_loop_vertices(mesh: &FaceMesh) -> [usize; MAX_TRIM_LOOPS] {
    assert!(mesh.boundary.len() <= MAX_TRIM_LOOPS);
    let mut lengths = [0; MAX_TRIM_LOOPS];
    for (slot, boundary) in lengths.iter_mut().zip(&mesh.boundary) {
        *slot = boundary.len();
    }
    lengths
}

fn boundary_retains_trim_vertices(face: &TrimmedSurface<'_>, mesh: &FaceMesh) -> bool {
    face.loops().len() == mesh.boundary.len()
        && face
            .loops()
            .iter()
            .zip(&mesh.boundary)
            .all(|(source, boundary)| {
                let output: Vec<_> = boundary
                    .iter()
                    .map(|&index| mesh.uvs[index as usize])
                    .collect();
                if output.first() != source.points.first() {
                    return false;
                }
                let mut cursor = 0;
                source.points.iter().all(|point| {
                    let Some(offset) = output[cursor..].iter().position(|output| output == point)
                    else {
                        return false;
                    };
                    cursor += offset + 1;
                    true
                })
            })
}

fn trim_parameter_area(face: &TrimmedSurface<'_>) -> f64 {
    face.loops().iter().map(TrimLoop::signed_area).sum()
}

fn mesh_parameter_area(mesh: &FaceMesh) -> f64 {
    mesh.triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.map(|index| mesh.uvs[index as usize]);
            (b - a).cross(c - a) / 2.0
        })
        .sum()
}

fn mesh_model_area(mesh: &FaceMesh) -> f64 {
    mesh.triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.map(|index| mesh.positions[index as usize]);
            (b - a).cross(c - a).norm() / 2.0
        })
        .sum()
}

fn quantized_area(area: f64) -> u64 {
    assert!(area.is_finite() && area >= 0.0);
    (area * AREA_SCALE).round() as u64
}

fn trim_digest(face: &TrimmedSurface<'_>) -> u64 {
    let mut digest = StableHasher::new();
    digest.string("q3-face-trim.v2");
    digest.count(face.loops().len());
    for loop_ in face.loops() {
        digest.count(loop_.points.len());
        for point in &loop_.points {
            digest.u64(point.x.to_bits());
            digest.u64(point.y.to_bits());
        }
    }
    digest.finish()
}

fn boundary_digest(mesh: &FaceMesh) -> u64 {
    let mut digest = StableHasher::new();
    digest.string("q3-face-boundary.v2");
    digest.count(mesh.boundary.len());
    for boundary in &mesh.boundary {
        digest.count(boundary.len());
        for &index in boundary {
            let uv = mesh.uvs[index as usize];
            digest.u64(uv.x.to_bits());
            digest.u64(uv.y.to_bits());
        }
    }
    digest.finish()
}

fn report_evidence(report: &OperationReport) -> ([u64; USAGE_STAGE_COUNT], u64) {
    assert_eq!(report.policy_version(), PolicyVersion::V1);
    let profile = FaceTessellationBudgetProfile::v1_defaults();
    assert_eq!(report.usage().len(), profile.limits().len());
    assert_eq!(report.usage().len(), USAGE_STAGE_COUNT);
    let mut digest = StableHasher::new();
    digest.string("q3-face-usage.v1");
    digest.string(PROFILE_IDENTITY);
    digest.string("policy.v1");
    digest.count(report.usage().len());
    for ((snapshot, limit), expected_stage) in report
        .usage()
        .iter()
        .zip(profile.limits())
        .zip(CANONICAL_STAGES)
    {
        assert_eq!(snapshot.stage, expected_stage);
        assert_eq!(snapshot.stage, limit.stage);
        assert_eq!(snapshot.resource, limit.resource);
        assert_eq!(snapshot.allowed, limit.allowed);
        digest.string(snapshot.stage.as_str());
        digest.tag(resource_tag(snapshot.resource));
        digest.tag(accounting_tag(limit.mode));
        digest.u64(snapshot.consumed);
    }
    (
        report
            .usage()
            .iter()
            .map(|entry| entry.consumed)
            .collect::<Vec<_>>()
            .try_into()
            .expect("canonical face profile has exactly five stages"),
        digest.finish(),
    )
}

fn mesh_digest(mesh: &FaceMesh) -> u64 {
    let mut digest = StableHasher::new();
    digest.string("q3-face-mesh.v2");
    digest.count(mesh.positions.len());
    for point in &mesh.positions {
        digest.u64(point.x.to_bits());
        digest.u64(point.y.to_bits());
        digest.u64(point.z.to_bits());
    }
    digest.count(mesh.uvs.len());
    for uv in &mesh.uvs {
        digest.u64(uv.x.to_bits());
        digest.u64(uv.y.to_bits());
    }
    digest.count(mesh.triangles.len());
    for triangle in &mesh.triangles {
        for &index in triangle {
            digest.u64(u64::from(index));
        }
    }
    digest.count(mesh.boundary.len());
    for boundary in &mesh.boundary {
        digest.count(boundary.len());
        for &index in boundary {
            digest.u64(u64::from(index));
        }
    }
    digest.finish()
}

const fn resource_tag(resource: ResourceKind) -> u8 {
    match resource {
        ResourceKind::Work => 1,
        ResourceKind::Items => 2,
        ResourceKind::Bytes => 3,
        ResourceKind::Depth => 4,
        _ => panic!("Q3 face digest does not define this resource kind"),
    }
}

const fn accounting_tag(mode: AccountingMode) -> u8 {
    match mode {
        AccountingMode::Cumulative => 1,
        AccountingMode::HighWater => 2,
    }
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

    const fn finish(self) -> u64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kcore::tolerance::Tolerances;
    use std::collections::BTreeSet;

    #[test]
    fn registry_contains_complete_unique_canonical_matrix() {
        assert_eq!(CASES.len(), 18);
        let unique: BTreeSet<_> = CASES.iter().map(|case| case.path).collect();
        assert_eq!(unique.len(), CASES.len());
        for representation in [
            SurfaceRepresentation::Plane,
            SurfaceRepresentation::HalfCylinder,
            SurfaceRepresentation::RationalNurbs,
        ] {
            for trim_shape in [TrimShape::Outer, TrimShape::OneHole, TrimShape::ThreeHoles] {
                for chord_tol in [1.0e-2, 1.0e-3] {
                    assert_eq!(
                        CASES
                            .iter()
                            .filter(|case| case.representation == representation
                                && case.trim_shape == trim_shape
                                && case.chord_tol == chord_tol)
                            .count(),
                        1
                    );
                }
            }
        }
        for case in CASES {
            crate::validate_case_path(case.path).unwrap();
        }
    }

    #[test]
    fn rational_nurbs_fixture_is_rational_and_genuinely_curved() {
        let surface = rational_nurbs();
        let weights = surface.weights().expect("fixture carries rational weights");
        assert!(weights.iter().any(|&weight| weight != weights[0]));

        let points = surface.points();
        let control_tetrahedron = (points[1] - points[0])
            .cross(points[2] - points[0])
            .dot(points[4] - points[0]);
        assert_ne!(control_tetrahedron, 0.0, "control net must not be coplanar");

        let derivatives = surface.eval_derivs([core::f64::consts::FRAC_PI_2, 1.0], 2);
        let normal = derivatives
            .du
            .cross(derivatives.dv)
            .normalized()
            .expect("fixture center is regular");
        assert!(
            derivatives.duu.dot(normal).abs() > 1.0e-6,
            "fixture must retain a nonzero normal-curvature witness"
        );
    }

    #[test]
    fn every_case_is_bitwise_repeatable_and_matches_reviewed_evidence() {
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        for case in CASES {
            let fixture = fixture(case.representation);
            let face = fixture.trimmed(case.trim_shape);
            let first = fixture.tessellate(case.trim_shape, case.chord_tol, &context);
            let repeated = fixture.tessellate(case.trim_shape, case.chord_tol, &context);
            assert_eq!(first, repeated);
            verify(case, fixture.evidence(&face, &first));
        }
    }

    #[test]
    fn bounded_v1_admits_the_complete_matrix_and_pins_the_measured_root_crossing() {
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_budget_overrides(FaceTessellationBudgetProfile::bounded_v1());
        let mut maximum_root_work = 0;
        let mut maximum_path = None;

        for case in CASES {
            let fixture = fixture(case.representation);
            let face = fixture.trimmed(case.trim_shape);
            let outcome = fixture
                .tessellate_outcome(&face, &tessellation_options(case.chord_tol), &context)
                .unwrap();
            let (result, report) = outcome.into_parts();
            let mesh = result
                .unwrap_or_else(|error| panic!("bounded preset rejected {}: {error:?}", case.path));
            assert_eq!(
                mesh_digest(&mesh),
                case.expected.mesh_digest,
                "{}",
                case.path
            );
            assert!(report.limit_events().is_empty(), "{}", case.path);
            for (stage, expected) in CANONICAL_STAGES.into_iter().zip(case.expected.usage) {
                let actual = report
                    .usage()
                    .iter()
                    .find(|snapshot| snapshot.stage == stage)
                    .unwrap_or_else(|| panic!("missing {} for {}", stage.as_str(), case.path));
                assert_eq!(actual.consumed, expected, "{}", case.path);
            }
            let root = report
                .usage()
                .iter()
                .find(|snapshot| snapshot.stage == TOTAL_WORK_STAGE)
                .unwrap();
            assert_eq!(
                root.consumed,
                case.expected.usage[1] + case.expected.usage[2],
                "{}",
                case.path
            );
            if root.consumed > maximum_root_work {
                maximum_root_work = root.consumed;
                maximum_path = Some(case.path);
            }
        }

        assert_eq!(maximum_root_work, 222);
        assert_eq!(
            maximum_path,
            Some("geometry/face-tessellation/half-cylinder-v2/three-holes/chord-1e-3-v2")
        );

        let case = CASES
            .into_iter()
            .find(|case| case.path == maximum_path.unwrap())
            .unwrap();
        let fixture = fixture(case.representation);
        let face = fixture.trimmed(case.trim_shape);
        let run = |allowed| {
            let context = OperationContext::new(&session, Tolerances::default())
                .unwrap()
                .with_budget_overrides(
                    FaceTessellationBudgetProfile::bounded_v1().with_total_work_limit(allowed),
                );
            fixture
                .tessellate_outcome(&face, &tessellation_options(case.chord_tol), &context)
                .unwrap()
        };
        assert!(run(222).result().is_ok());
        let denied = run(221);
        assert!(denied.result().is_err());
        assert_eq!(denied.report().limit_events().len(), 1);
        let snapshot = denied.report().limit_events()[0];
        assert_eq!(snapshot.stage, TOTAL_WORK_STAGE);
        assert_eq!(snapshot.resource, ResourceKind::Work);
        assert_eq!(snapshot.consumed, 222);
        assert_eq!(snapshot.allowed, 221);
    }

    #[test]
    fn json_registry_matches_every_rust_case_and_reviewed_counter() {
        let manifest: serde_json::Value =
            serde_json::from_str(include_str!("../cases.json")).unwrap();
        let entries = manifest["cases"].as_array().unwrap();
        let q3_entries: Vec<_> = entries
            .iter()
            .filter(|entry| entry["benchmark_target"] == "face_tessellation")
            .collect();
        assert_eq!(q3_entries.len(), CASES.len());
        for case in CASES {
            let entry = q3_entries
                .iter()
                .copied()
                .find(|entry| entry["path"] == case.path)
                .unwrap();
            assert_eq!(entry["fixture_version"], FIXTURE_VERSION);
            assert_eq!(entry["deterministic_seed"].as_u64(), Some(FIXTURE_SEED));
            assert_eq!(entry["size_parameters"]["elements"].as_u64(), Some(1));
            assert_eq!(entry["size_parameters"]["faces"].as_u64(), Some(1));
            assert_eq!(
                entry["size_parameters"]["trim_loops"].as_u64(),
                Some(case.trim_shape.loop_count() as u64)
            );
            assert_eq!(
                entry["tolerances"]["chord_tol"].as_f64(),
                Some(case.chord_tol)
            );
            assert_eq!(entry["policy_values"]["api"], API_IDENTITY);
            assert_eq!(entry["policy_values"]["budget_profile"], PROFILE_IDENTITY);
            assert_eq!(entry["policy_values"]["execution"], EXECUTION_IDENTITY);
            assert_eq!(entry["policy_values"]["max_edge_len"], "unbounded");
            assert_eq!(entry["policy_values"]["policy_version"], "v1");
            assert_eq!(
                entry["policy_values"]["representation"],
                case.representation.identity()
            );
            assert_eq!(
                entry["policy_values"]["trim_shape"],
                case.trim_shape.identity()
            );
            assert_eq!(entry["policy_values"]["usage_contract"], "q3-face-usage.v1");
            let counters = &entry["expected_result_counters"];
            assert_eq!(
                counters["mesh_vertices"].as_u64(),
                Some(case.expected.mesh_vertices as u64)
            );
            assert_eq!(
                counters["mesh_triangles"].as_u64(),
                Some(case.expected.mesh_triangles as u64)
            );
            assert_eq!(
                counters["boundary_loops"].as_u64(),
                Some(case.trim_shape.loop_count() as u64)
            );
            let loop_vertices = counters["boundary_loop_vertices"].as_array().unwrap();
            assert_eq!(loop_vertices.len(), case.trim_shape.loop_count());
            for (actual, expected) in loop_vertices
                .iter()
                .zip(case.expected.boundary_loop_vertices)
            {
                assert_eq!(actual.as_u64(), Some(expected as u64));
            }
            assert_eq!(
                counters["boundary_vertices"].as_u64(),
                Some(case.expected.boundary_loop_vertices.iter().sum::<usize>() as u64)
            );
            for field in [
                "positions_finite",
                "uvs_finite",
                "indices_valid",
                "coordinates_aligned",
                "triangles_oriented",
                "positions_on_surface",
                "triangles_follow_surface_orientation",
                "boundary_retains_trim_vertices",
                "parameter_area_matches_trim",
                "model_area_finite_positive",
            ] {
                assert_eq!(counters[field].as_bool(), Some(true));
            }
            assert_eq!(
                counters["parameter_area_units"].as_u64(),
                Some(case.expected.parameter_area_units)
            );
            assert_eq!(
                counters["model_area_units"].as_u64(),
                Some(case.expected.model_area_units)
            );
            assert_eq!(
                counters["usage_stage_count"].as_u64(),
                Some(USAGE_STAGE_COUNT as u64)
            );
            let usage = counters["usage_consumed"].as_array().unwrap();
            assert_eq!(usage.len(), USAGE_STAGE_COUNT);
            for (index, value) in usage.iter().enumerate() {
                assert_eq!(value.as_u64(), Some(case.expected.usage[index]));
            }
            for (field, expected) in [
                ("trim_digest", case.expected.trim_digest),
                ("boundary_digest", case.expected.boundary_digest),
                ("usage_digest", case.expected.usage_digest),
                ("mesh_digest", case.expected.mesh_digest),
                ("output_digest", case.expected.output_digest),
            ] {
                assert_eq!(
                    counters[field].as_str(),
                    Some(format!("{expected:016x}").as_str())
                );
            }
            for field in [
                "limit_event_count",
                "numeric_resolution_stage_count",
                "diagnostic_count",
                "dropped_diagnostic_count",
            ] {
                assert_eq!(counters[field].as_u64(), Some(0));
            }
        }
    }
}
