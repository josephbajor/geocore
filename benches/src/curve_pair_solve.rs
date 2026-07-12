//! Deterministic Q4 NURBS curve-pair solve fixtures and evidence.

use kcore::operation::{
    AccountingMode, BudgetPlan, ExecutionPolicy, LimitSnapshot, LimitSpec, NumericalPolicy,
    OperationContext, OperationReport, PolicyVersion, ResourceKind, SessionPolicy,
    SessionPrecision, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::{
    CurvePairProjectionPlane, NURBS_CURVE_PAIR_CANDIDATES, NURBS_CURVE_PAIR_DEPTH,
    NURBS_CURVE_PAIR_SUBDIVISIONS, NurbsCurve,
};
use kgeom::vec::Point3;
use kops::intersect::{
    ContactKind, CurveCurveIntersections, NURBS_CURVE_PAIR_SEED_ATTEMPTS,
    intersect_bounded_nurbs_nurbs_with_context,
};

/// Fixture identity for the first Q4 curve-pair solve ladder.
pub const FIXTURE_VERSION: &str = "curve-pair-solve.v1";
/// Deterministic construction seed recorded by the registry.
pub const FIXTURE_SEED: u64 = 0x5154_4350_534f_000a;

/// Geometry varied by the solve ladder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolveFixture {
    /// Polynomial transverse line pair.
    PolynomialTransverse,
    /// Rationally parameterized transverse line pair.
    RationalTransverse,
    /// Quadratic tangency.
    Tangent,
    /// Quadratic arch with two transverse contacts.
    MultipleRoots,
    /// Overlapping root hulls whose exact subdivision proves separation.
    HiddenMiss,
}

/// Reviewed structured stop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitKind {
    /// No configured stop.
    None,
    /// Cell-local seed admission stopped.
    SeedAttempts,
}

impl LimitKind {
    /// Stable manifest spelling.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::SeedAttempts => "seed-attempts",
        }
    }
}

/// Stable Q4 solve case.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurvePairSolveCase {
    /// Canonical five-segment path.
    pub path: &'static str,
    /// Geometry fixture.
    pub fixture: SolveFixture,
    /// Inclusive seed-attempt allowance.
    pub seed_attempts_allowed: u64,
    /// Reviewed emitted contact count.
    pub expected_points: usize,
    /// Reviewed complete-domain state.
    pub expected_complete: bool,
    /// Reviewed proven-empty state.
    pub expected_proven_empty: bool,
    /// Reviewed stop.
    pub expected_limit: LimitKind,
    /// Reviewed contact digest.
    pub expected_point_digest: u64,
    /// Reviewed complete evidence digest.
    pub expected_output_digest: u64,
}

/// Six cases covering representation, contact character, multiplicity, proof, and limits.
pub const CASES: [CurvePairSolveCase; 6] = [
    case(
        "geometry/curve-pair-solve/poly-transverse-v1/1/default-v1",
        SolveFixture::PolynomialTransverse,
        4_096,
        expected(
            1,
            true,
            false,
            LimitKind::None,
            0x617e_1b7b_48fd_b84a,
            0x8dfb_ac0d_9e9d_1d0b,
        ),
    ),
    case(
        "geometry/curve-pair-solve/rational-transverse-v1/1/default-v1",
        SolveFixture::RationalTransverse,
        4_096,
        expected(
            1,
            false,
            false,
            LimitKind::None,
            0xd116_a5f6_4d2e_545a,
            0x7fca_16ca_153e_d3c8,
        ),
    ),
    case(
        "geometry/curve-pair-solve/poly-tangent-v1/1/default-v1",
        SolveFixture::Tangent,
        4_096,
        expected(
            1,
            false,
            false,
            LimitKind::None,
            0x617e_1c7b_48fd_b9fd,
            0x9db6_67bc_8bf2_e7cc,
        ),
    ),
    case(
        "geometry/curve-pair-solve/poly-two-root-v1/2/default-v1",
        SolveFixture::MultipleRoots,
        4_096,
        expected(
            2,
            false,
            false,
            LimitKind::None,
            0x3606_10ba_1318_ae30,
            0x289f_9511_588a_2317,
        ),
    ),
    case(
        "geometry/curve-pair-solve/poly-hidden-miss-v1/0/default-v1",
        SolveFixture::HiddenMiss,
        4_096,
        expected(
            0,
            true,
            true,
            LimitKind::None,
            0x6489_db2b_285b_d20f,
            0x302a_fe5b_7cd2_afbf,
        ),
    ),
    case(
        "geometry/curve-pair-solve/poly-tangent-v1/1/seed-denied-v1",
        SolveFixture::Tangent,
        0,
        expected(
            0,
            false,
            false,
            LimitKind::SeedAttempts,
            0x6489_db2b_285b_d20f,
            0x4b70_6ef0_2117_44ef,
        ),
    ),
];

#[derive(Clone, Copy)]
struct Expected {
    points: usize,
    complete: bool,
    proven_empty: bool,
    limit: LimitKind,
    point_digest: u64,
    output_digest: u64,
}

const fn expected(
    points: usize,
    complete: bool,
    proven_empty: bool,
    limit: LimitKind,
    point_digest: u64,
    output_digest: u64,
) -> Expected {
    Expected {
        points,
        complete,
        proven_empty,
        limit,
        point_digest,
        output_digest,
    }
}

const fn case(
    path: &'static str,
    fixture: SolveFixture,
    seed_attempts_allowed: u64,
    expected: Expected,
) -> CurvePairSolveCase {
    CurvePairSolveCase {
        path,
        fixture,
        seed_attempts_allowed,
        expected_points: expected.points,
        expected_complete: expected.complete,
        expected_proven_empty: expected.proven_empty,
        expected_limit: expected.limit,
        expected_point_digest: expected.point_digest,
        expected_output_digest: expected.output_digest,
    }
}

/// Immutable prepared geometry and session policy; neither is timed.
pub struct CurvePairSolveFixture {
    first: NurbsCurve,
    second: NurbsCurve,
    session: SessionPolicy,
}

impl CurvePairSolveFixture {
    /// Time the public contextual solve and return deterministic evidence.
    pub fn measure_once(
        &self,
        case: CurvePairSolveCase,
    ) -> (core::time::Duration, CurvePairSolveEvidence) {
        let overrides = BudgetPlan::new([LimitSpec::new(
            NURBS_CURVE_PAIR_SEED_ATTEMPTS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            case.seed_attempts_allowed,
        )])
        .expect("valid Q4 seed override");
        let context = OperationContext::new(&self.session, Tolerances::default())
            .expect("Q4 curve-pair solve policy is valid")
            .with_budget_overrides(overrides);
        let started = std::time::Instant::now();
        let outcome = intersect_bounded_nurbs_nurbs_with_context(
            &self.first,
            self.first.param_range(),
            &self.second,
            self.second.param_range(),
            &context,
        );
        let elapsed = started.elapsed();
        let (result, report) = outcome.into_parts();
        let result = result.expect("reviewed Q4 curve-pair solve must run");
        (elapsed, self.evidence(case, &result, &report))
    }

    fn evidence(
        &self,
        case: CurvePairSolveCase,
        result: &CurveCurveIntersections,
        report: &OperationReport,
    ) -> CurvePairSolveEvidence {
        let isolation_work = usage(report, NURBS_CURVE_PAIR_SUBDIVISIONS);
        let candidates = usage(report, NURBS_CURVE_PAIR_CANDIDATES);
        let depth = usage(report, NURBS_CURVE_PAIR_DEPTH);
        let seeds = usage(report, NURBS_CURVE_PAIR_SEED_ATTEMPTS);
        let limit = if report
            .limit_events()
            .iter()
            .any(|event| event.stage == NURBS_CURVE_PAIR_SEED_ATTEMPTS)
        {
            LimitKind::SeedAttempts
        } else {
            LimitKind::None
        };
        let (limit_attempted_consumed, limit_attempted_allowed) = report
            .limit_events()
            .first()
            .map_or((0, 0), |event| (event.consumed, event.allowed));
        let point_digest = point_digest(result);
        let incomplete_evidence_digest = incomplete_evidence_digest(result);
        let root_certificate_digest = root_certificate_digest(result);
        let mut evidence = CurvePairSolveEvidence {
            control_points: self.first.points().len() + self.second.points().len(),
            points: result.points.len(),
            overlaps: result.overlaps.len(),
            complete: result.is_complete(),
            proven_empty: result.is_proven_empty(),
            indeterminate: !result.is_complete(),
            verified_witnesses: witnesses_are_verified(&self.first, &self.second, result),
            limit,
            limit_events: report.limit_events().len(),
            limit_attempted_consumed,
            limit_attempted_allowed,
            isolation_work: isolation_work.consumed,
            candidate_high_water: candidates.consumed,
            depth_high_water: depth.consumed,
            seed_attempts: seeds.consumed,
            seed_attempts_allowed: seeds.allowed,
            incomplete_evidence: result.incomplete_evidence().len(),
            incomplete_evidence_digest,
            root_certificates: result.root_certificates().len(),
            root_certificate_digest,
            point_digest,
            output_digest: 0,
        };
        evidence.output_digest = evidence.digest(case);
        evidence
    }
}

/// Stable Q4 curve-pair solve evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurvePairSolveEvidence {
    /// Total source control points.
    pub control_points: usize,
    /// Emitted isolated contacts.
    pub points: usize,
    /// Emitted provisional overlaps.
    pub overlaps: usize,
    /// Complete-domain evidence state.
    pub complete: bool,
    /// Proven-empty state.
    pub proven_empty: bool,
    /// Missing-completion state.
    pub indeterminate: bool,
    /// Whether every emitted contact re-verifies exactly against source geometry.
    pub verified_witnesses: bool,
    /// Structured stop.
    pub limit: LimitKind,
    /// Limit event count.
    pub limit_events: usize,
    /// Attempted usage at the crossing.
    pub limit_attempted_consumed: u64,
    /// Active allowance at the crossing.
    pub limit_attempted_allowed: u64,
    /// Exact isolation setup/subdivision work.
    pub isolation_work: u64,
    /// Retained-cell high water.
    pub candidate_high_water: u64,
    /// Exact isolation depth high water.
    pub depth_high_water: u64,
    /// Committed cell-local attempts.
    pub seed_attempts: u64,
    /// Inclusive cell-local attempt allowance.
    pub seed_attempts_allowed: u64,
    /// Structured unresolved proof obligations.
    pub incomplete_evidence: usize,
    /// Exact ordered incomplete-evidence digest.
    pub incomplete_evidence_digest: u64,
    /// Exact unique-root certificates retained by the solve result.
    pub root_certificates: usize,
    /// Ordered exact root-certificate digest.
    pub root_certificate_digest: u64,
    /// Exact ordered contact digest.
    pub point_digest: u64,
    /// Complete semantic evidence digest.
    pub output_digest: u64,
}

impl CurvePairSolveEvidence {
    fn digest(self, case: CurvePairSolveCase) -> u64 {
        let mut digest = StableHasher::new();
        digest.tag(0xb1);
        digest.count(self.control_points);
        digest.count(self.points);
        digest.count(self.overlaps);
        digest.boolean(self.complete);
        digest.boolean(self.proven_empty);
        digest.boolean(self.indeterminate);
        digest.boolean(self.verified_witnesses);
        digest.tag(match self.limit {
            LimitKind::None => 0,
            LimitKind::SeedAttempts => 1,
        });
        digest.count(self.limit_events);
        digest.u64(self.limit_attempted_consumed);
        digest.u64(self.limit_attempted_allowed);
        digest.u64(self.isolation_work);
        digest.u64(self.candidate_high_water);
        digest.u64(self.depth_high_water);
        digest.u64(self.seed_attempts);
        digest.u64(self.seed_attempts_allowed);
        digest.count(self.incomplete_evidence);
        digest.u64(self.incomplete_evidence_digest);
        digest.count(self.root_certificates);
        digest.u64(self.root_certificate_digest);
        digest.u64(self.point_digest);
        digest.tag(case.fixture as u8);
        digest.finish()
    }
}

/// Construct one prepared solve fixture.
pub fn fixture(case: CurvePairSolveCase) -> CurvePairSolveFixture {
    let (first, second) = curves(case.fixture);
    CurvePairSolveFixture {
        first,
        second,
        session: SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            BudgetPlan::empty(),
            PolicyVersion::V1,
        ),
    }
}

/// Verify reviewed solve evidence without using elapsed time as correctness evidence.
pub fn verify(case: CurvePairSolveCase, evidence: CurvePairSolveEvidence) {
    assert_eq!(evidence.points, case.expected_points);
    assert_eq!(evidence.overlaps, 0);
    assert_eq!(evidence.complete, case.expected_complete);
    assert_eq!(evidence.proven_empty, case.expected_proven_empty);
    assert_eq!(evidence.limit, case.expected_limit);
    assert!(evidence.verified_witnesses);
    let expected_incomplete_evidence = match (case.expected_complete, case.expected_limit) {
        (true, _) => 0,
        (_, LimitKind::SeedAttempts) => 2,
        _ => 1,
    };
    assert_eq!(evidence.incomplete_evidence, expected_incomplete_evidence);
    assert_eq!(evidence.seed_attempts_allowed, case.seed_attempts_allowed);
    if evidence.limit == LimitKind::None {
        assert_eq!(evidence.limit_events, 0);
    } else {
        assert!(evidence.indeterminate);
        assert_eq!(evidence.limit_events, 1);
        assert!(evidence.limit_attempted_consumed > evidence.limit_attempted_allowed);
    }
    assert_ne!(case.expected_point_digest, 0);
    assert_ne!(case.expected_output_digest, 0);
    assert_eq!(evidence.point_digest, case.expected_point_digest);
    assert_eq!(evidence.output_digest, case.expected_output_digest);
}

fn curves(fixture: SolveFixture) -> (NurbsCurve, NurbsCurve) {
    match fixture {
        SolveFixture::PolynomialTransverse => (diagonal(None), horizontal(0.0)),
        SolveFixture::RationalTransverse => (diagonal(Some(vec![1.0, 1.5])), horizontal(0.0)),
        SolveFixture::Tangent => (tangent_parabola(), horizontal(0.0)),
        SolveFixture::MultipleRoots => (arch(), horizontal(0.5)),
        SolveFixture::HiddenMiss => (arch(), horizontal(1.5)),
    }
}

fn diagonal(weights: Option<Vec<f64>>) -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-1.0, -1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        weights,
    )
    .expect("valid Q4 diagonal")
}

fn horizontal(y: f64) -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-2.0, y, 0.0), Point3::new(2.0, y, 0.0)],
        None,
    )
    .expect("valid Q4 horizontal")
}

fn tangent_parabola() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 1.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 tangent")
}

fn arch() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 arch")
}

fn usage(report: &OperationReport, stage: StageId) -> LimitSnapshot {
    *report
        .usage()
        .iter()
        .find(|usage| usage.stage == stage)
        .expect("Q4 curve-pair solve stage is configured")
}

fn witnesses_are_verified(
    first: &NurbsCurve,
    second: &NurbsCurve,
    result: &CurveCurveIntersections,
) -> bool {
    result.points.iter().all(|point| {
        let first_point = first.eval(point.t_a);
        let second_point = second.eval(point.t_b);
        first.param_range().contains(point.t_a)
            && second.param_range().contains(point.t_b)
            && point.residual == first_point.dist(second_point)
            && point.residual <= Tolerances::default().linear()
            && point.point == (first_point + second_point) / 2.0
    })
}

fn point_digest(result: &CurveCurveIntersections) -> u64 {
    let mut digest = StableHasher::new();
    digest.tag(0xb0);
    digest.count(result.points.len());
    for point in &result.points {
        digest.f64(point.t_a);
        digest.f64(point.t_b);
        digest.f64(point.point.x);
        digest.f64(point.point.y);
        digest.f64(point.point.z);
        digest.f64(point.residual);
        digest.tag(match point.kind {
            ContactKind::Transverse => 0,
            ContactKind::Tangent => 1,
            ContactKind::Singular => 2,
            _ => 3,
        });
    }
    digest.finish()
}

fn incomplete_evidence_digest(result: &CurveCurveIntersections) -> u64 {
    let mut digest = StableHasher::new();
    digest.tag(0xb2);
    digest.count(result.incomplete_evidence().len());
    for evidence in result.incomplete_evidence() {
        digest.bytes(evidence.code.as_str().as_bytes());
        digest.bytes(evidence.stage.as_str().as_bytes());
        match evidence.cause {
            kcore::proof::IncompleteCause::Unsupported { capability } => {
                digest.tag(0);
                digest.bytes(capability.as_str().as_bytes());
            }
            kcore::proof::IncompleteCause::Limit { snapshot } => {
                digest.tag(1);
                digest.bytes(snapshot.stage.as_str().as_bytes());
                digest.tag(snapshot.resource as u8);
                digest.u64(snapshot.consumed);
                digest.u64(snapshot.allowed);
            }
            kcore::proof::IncompleteCause::NumericResolution => digest.tag(2),
            kcore::proof::IncompleteCause::Cancelled => digest.tag(3),
            kcore::proof::IncompleteCause::ProofMethodUnavailable { capability } => {
                digest.tag(4);
                digest.bytes(capability.as_str().as_bytes());
            }
            _ => digest.tag(255),
        }
    }
    digest.finish()
}

fn root_certificate_digest(result: &CurveCurveIntersections) -> u64 {
    let mut digest = StableHasher::new();
    digest.tag(0xb3);
    digest.count(result.root_certificates().len());
    for certificate in result.root_certificates() {
        for range in [certificate.first_range(), certificate.second_range()] {
            digest.f64(range.lo);
            digest.f64(range.hi);
        }
        digest.tag(match certificate.projection_plane() {
            CurvePairProjectionPlane::Xy => 0,
            CurvePairProjectionPlane::Xz => 1,
            CurvePairProjectionPlane::Yz => 2,
            _ => 255,
        });
        digest.f64(certificate.determinant_lower_bound());
    }
    digest.finish()
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

    fn f64(&mut self, value: f64) {
        self.u64(value.to_bits());
    }

    const fn finish(self) -> u64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;

    #[test]
    fn registry_is_unique_canonical_and_repeatable() {
        let unique = CASES.iter().map(|case| case.path).collect::<BTreeSet<_>>();
        assert_eq!(unique.len(), CASES.len());
        for case in CASES {
            crate::validate_case_path(case.path).unwrap();
            let fixture = fixture(case);
            let first = fixture.measure_once(case).1;
            assert_eq!(first, fixture.measure_once(case).1);
            verify(case, first);
        }
    }

    #[test]
    fn json_registry_matches_every_rust_case_and_counter() {
        let manifest: serde_json::Value =
            serde_json::from_str(include_str!("../cases.json")).unwrap();
        let entries = manifest["cases"].as_array().unwrap();
        let entries: Vec<_> = entries
            .iter()
            .filter(|entry| entry["benchmark_target"] == "curve_pair_solve")
            .collect();
        assert_eq!(entries.len(), CASES.len());

        for case in CASES {
            let matches: Vec<_> = entries
                .iter()
                .copied()
                .filter(|entry| entry["path"] == case.path)
                .collect();
            assert_eq!(matches.len(), 1, "registry mismatch for {}", case.path);
            let entry = matches[0];
            assert_eq!(entry["fixture_version"], FIXTURE_VERSION);
            assert_eq!(entry["deterministic_seed"].as_u64(), Some(FIXTURE_SEED));
            assert_eq!(
                entry["policy_values"]["seed_attempts_allowed"].as_u64(),
                Some(case.seed_attempts_allowed)
            );
            assert_eq!(entry["policy_values"]["execution"], "serial");
            assert_eq!(entry["policy_values"]["policy_version"], "v1");
            assert_eq!(
                entry["policy_values"]["api"],
                "intersect_bounded_nurbs_nurbs_with_context"
            );

            let evidence = fixture(case).measure_once(case).1;
            verify(case, evidence);
            assert_eq!(
                entry["size_parameters"]["control_points"].as_u64(),
                Some(evidence.control_points as u64)
            );
            let expected_elements = match case.fixture {
                SolveFixture::HiddenMiss => 0,
                SolveFixture::MultipleRoots => 2,
                _ => 1,
            };
            assert_eq!(
                entry["size_parameters"]["elements"].as_u64(),
                Some(expected_elements)
            );
            assert_eq!(
                entry["tolerances"]["linear"].as_f64(),
                Some(Tolerances::default().linear())
            );
            let counters = &entry["expected_result_counters"];
            for (field, actual) in [
                ("points", evidence.points as u64),
                ("overlaps", evidence.overlaps as u64),
                ("limit_events", evidence.limit_events as u64),
                (
                    "limit_attempted_consumed",
                    evidence.limit_attempted_consumed,
                ),
                ("limit_attempted_allowed", evidence.limit_attempted_allowed),
                ("isolation_work", evidence.isolation_work),
                ("candidate_high_water", evidence.candidate_high_water),
                ("depth_high_water", evidence.depth_high_water),
                ("seed_attempts", evidence.seed_attempts),
                ("incomplete_evidence", evidence.incomplete_evidence as u64),
                ("root_certificates", evidence.root_certificates as u64),
            ] {
                assert_eq!(counters[field].as_u64(), Some(actual), "{field}");
            }
            for (field, actual) in [
                ("complete", evidence.complete),
                ("proven_empty", evidence.proven_empty),
                ("indeterminate", evidence.indeterminate),
                ("verified_witnesses", evidence.verified_witnesses),
            ] {
                assert_eq!(counters[field].as_bool(), Some(actual), "{field}");
            }
            assert_eq!(counters["limit_kind"], evidence.limit.as_str());
            assert_eq!(
                counters["root_certificate_digest"].as_str(),
                Some(format!("{:016x}", evidence.root_certificate_digest).as_str())
            );
            assert_eq!(
                counters["incomplete_evidence_digest"].as_str(),
                Some(format!("{:016x}", evidence.incomplete_evidence_digest).as_str())
            );
            assert_eq!(
                counters["point_digest"].as_str(),
                Some(format!("{:016x}", evidence.point_digest).as_str())
            );
            assert_eq!(
                counters["output_digest"].as_str(),
                Some(format!("{:016x}", evidence.output_digest).as_str())
            );
        }
    }
}
