//! Deterministic Q3 standalone face-tessellation fixtures and evidence.

use kcore::operation::{
    AccountingMode, ExecutionPolicy, NumericalPolicy, OperationContext, OperationOutcome,
    OperationPolicyError, OperationReport, PolicyVersion, ResourceKind, SessionPolicy,
    SessionPrecision, StageId,
};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgeom::tess::{
    FACE_TESSELLATION_BOUNDARY_DEPTH, FACE_TESSELLATION_BOUNDARY_SPLITS,
    FACE_TESSELLATION_MESH_TRIANGLES, FACE_TESSELLATION_MESH_VERTICES,
    FACE_TESSELLATION_REFINEMENT_PASSES, FaceMesh, FaceTessellationBudgetProfile, TessOptions,
    TrimmedSurface, tessellate_with_context,
};

/// Fixture identity for the first standalone Q3 face ladder.
pub const FIXTURE_VERSION: &str = "face-tessellation.v1";
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
const CANONICAL_STAGES: [StageId; USAGE_STAGE_COUNT] = [
    FACE_TESSELLATION_BOUNDARY_DEPTH,
    FACE_TESSELLATION_BOUNDARY_SPLITS,
    FACE_TESSELLATION_REFINEMENT_PASSES,
    FACE_TESSELLATION_MESH_TRIANGLES,
    FACE_TESSELLATION_MESH_VERTICES,
];

/// Stable standalone face-tessellation case definition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceTessellationCase {
    /// Five-segment benchmark path.
    pub path: &'static str,
    /// Chordal tessellation tolerance.
    pub chord_tol: f64,
    /// Reviewed output vertex count.
    pub expected_mesh_vertices: usize,
    /// Reviewed output triangle count.
    pub expected_mesh_triangles: usize,
    /// Reviewed boundary vertex count.
    pub expected_boundary_vertices: usize,
    /// Reviewed mesh digest.
    pub expected_mesh_digest: u64,
    /// Reviewed complete semantic output digest.
    pub expected_output_digest: u64,
    /// Reviewed consumed values in canonical face-profile order.
    pub expected_usage: [u64; USAGE_STAGE_COUNT],
    /// Reviewed portable usage digest.
    pub expected_usage_digest: u64,
}

/// Half-cylinder face at coarse and fine tolerances.
pub const CASES: [FaceTessellationCase; 2] = [
    FaceTessellationCase {
        path: "geometry/face-tessellation/cylinder-half-v1/1/chord-1e-2-v1",
        chord_tol: 1.0e-2,
        expected_mesh_vertices: 543,
        expected_mesh_triangles: 1_050,
        expected_boundary_vertices: 34,
        expected_mesh_digest: 0xd7ae_dd96_5c85_61f7,
        expected_output_digest: 0x1fcc_c2a9_0986_3748,
        expected_usage: [4, 30, 4, 1_050, 543],
        expected_usage_digest: 0x0352_7d7b_7a51_1949,
    },
    FaceTessellationCase {
        path: "geometry/face-tessellation/cylinder-half-v1/1/chord-1e-3-v1",
        chord_tol: 1.0e-3,
        expected_mesh_vertices: 18_841,
        expected_mesh_triangles: 37_550,
        expected_boundary_vertices: 130,
        expected_mesh_digest: 0xf57c_3dd3_8a9f_0da8,
        expected_output_digest: 0xa96b_3eb9_1af5_66d7,
        expected_usage: [6, 126, 6, 37_550, 18_841],
        expected_usage_digest: 0x3446_5daa_e185_880c,
    },
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

/// Fully constructed immutable half-cylinder input.
pub struct FaceTessellationFixture {
    surface: Cylinder,
}

impl FaceTessellationFixture {
    /// Construct the immutable borrowed trim outside measured work.
    pub fn trimmed(&self) -> TrimmedSurface<'_> {
        TrimmedSurface::rectangle(
            &self.surface,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(0.0, 2.0),
            ],
        )
        .expect("reviewed Q3 face trim is valid")
    }

    /// Invoke exactly the contextual API measured by the face ladder.
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
        chord_tol: f64,
        context: &OperationContext<'_>,
    ) -> FaceTessellationRun {
        let face = self.trimmed();
        FaceTessellationRun::from_outcome(
            self.tessellate_outcome(&face, &tessellation_options(chord_tol), context)
                .expect("reviewed Q3 face policy must be valid"),
        )
    }

    /// Reduce one mesh and report to stable semantic evidence.
    pub fn evidence(&self, run: &FaceTessellationRun) -> FaceTessellationEvidence {
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
        let boundary_vertices = mesh.boundary.iter().map(Vec::len).sum();
        let mesh_digest = mesh_digest(mesh);
        let (usage, usage_digest) = report_evidence(&run.report);
        let mut evidence = FaceTessellationEvidence {
            mesh_vertices: mesh.positions.len(),
            mesh_triangles: mesh.triangles.len(),
            boundary_loops: mesh.boundary.len(),
            boundary_vertices,
            positions_finite,
            uvs_finite,
            indices_valid,
            coordinates_aligned,
            triangles_oriented,
            mesh_digest,
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
    /// Stable mesh digest.
    pub mesh_digest: u64,
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
        digest.string("q3-face-output.v1");
        for value in [
            self.mesh_vertices,
            self.mesh_triangles,
            self.boundary_loops,
            self.boundary_vertices,
        ] {
            digest.count(value);
        }
        for value in [
            self.positions_finite,
            self.uvs_finite,
            self.indices_valid,
            self.coordinates_aligned,
            self.triangles_oriented,
        ] {
            digest.boolean(value);
        }
        digest.u64(self.mesh_digest);
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

/// Construct the immutable half-cylinder fixture.
pub fn fixture() -> FaceTessellationFixture {
    FaceTessellationFixture {
        surface: Cylinder::new(Frame::world(), 2.0).expect("valid Q3 cylinder"),
    }
}

/// Verify exact reviewed evidence for one case.
pub fn verify(case: FaceTessellationCase, evidence: FaceTessellationEvidence) {
    assert!(evidence.positions_finite);
    assert!(evidence.uvs_finite);
    assert!(evidence.indices_valid);
    assert!(evidence.coordinates_aligned);
    assert!(evidence.triangles_oriented);
    assert_eq!(evidence.boundary_loops, 1);
    assert_eq!(evidence.api_identity, API_IDENTITY);
    assert_eq!(evidence.profile_identity, PROFILE_IDENTITY);
    assert_eq!(evidence.execution_identity, EXECUTION_IDENTITY);
    assert!(evidence.policy_version_v1);
    assert_eq!(evidence.limit_event_count, 0);
    assert_eq!(evidence.numeric_resolution_stage_count, 0);
    assert_eq!(evidence.diagnostic_count, 0);
    assert_eq!(evidence.dropped_diagnostic_count, 0);
    assert_eq!(evidence.mesh_vertices, case.expected_mesh_vertices);
    assert_eq!(evidence.mesh_triangles, case.expected_mesh_triangles);
    assert_eq!(evidence.boundary_vertices, case.expected_boundary_vertices);
    assert_eq!(evidence.mesh_digest, case.expected_mesh_digest);
    assert_eq!(evidence.output_digest, case.expected_output_digest);
    assert_eq!(evidence.usage, case.expected_usage);
    assert_eq!(evidence.usage_digest, case.expected_usage_digest);
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
    digest.string("q3-face-mesh.v1");
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
    fn registry_contains_exactly_two_unique_canonical_cases() {
        assert_eq!(CASES.len(), 2);
        let unique: BTreeSet<_> = CASES.iter().map(|case| case.path).collect();
        assert_eq!(unique.len(), CASES.len());
        for case in CASES {
            crate::validate_case_path(case.path).unwrap();
        }
    }

    #[test]
    fn every_case_is_bitwise_repeatable_and_matches_reviewed_evidence() {
        let fixture = fixture();
        let session = compatibility_session();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        for case in CASES {
            let first = fixture.tessellate(case.chord_tol, &context);
            let repeated = fixture.tessellate(case.chord_tol, &context);
            assert_eq!(first, repeated);
            verify(case, fixture.evidence(&first));
        }
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
                entry["tolerances"]["chord_tol"].as_f64(),
                Some(case.chord_tol)
            );
            assert_eq!(entry["policy_values"]["api"], API_IDENTITY);
            assert_eq!(entry["policy_values"]["budget_profile"], PROFILE_IDENTITY);
            assert_eq!(entry["policy_values"]["execution"], EXECUTION_IDENTITY);
            assert_eq!(entry["policy_values"]["policy_version"], "v1");
            assert_eq!(entry["policy_values"]["usage_contract"], "q3-face-usage.v1");
            let counters = &entry["expected_result_counters"];
            assert_eq!(
                counters["mesh_vertices"].as_u64(),
                Some(case.expected_mesh_vertices as u64)
            );
            assert_eq!(
                counters["mesh_triangles"].as_u64(),
                Some(case.expected_mesh_triangles as u64)
            );
            assert_eq!(counters["boundary_loops"].as_u64(), Some(1));
            assert_eq!(
                counters["boundary_vertices"].as_u64(),
                Some(case.expected_boundary_vertices as u64)
            );
            for field in [
                "positions_finite",
                "uvs_finite",
                "indices_valid",
                "coordinates_aligned",
                "triangles_oriented",
            ] {
                assert_eq!(counters[field].as_bool(), Some(true));
            }
            let usage = counters["usage_consumed"].as_array().unwrap();
            assert_eq!(usage.len(), USAGE_STAGE_COUNT);
            for (index, value) in usage.iter().enumerate() {
                assert_eq!(value.as_u64(), Some(case.expected_usage[index]));
            }
            assert_eq!(
                counters["usage_digest"].as_str(),
                Some(format!("{:016x}", case.expected_usage_digest).as_str())
            );
            assert_eq!(
                counters["mesh_digest"].as_str(),
                Some(format!("{:016x}", case.expected_mesh_digest).as_str())
            );
            assert_eq!(
                counters["output_digest"].as_str(),
                Some(format!("{:016x}", case.expected_output_digest).as_str())
            );
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
