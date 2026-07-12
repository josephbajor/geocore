//! Deterministic Q4 NURBS curve-pair isolation fixtures and evidence.

use kcore::operation::{
    AccountingMode, BudgetPlan, ExecutionPolicy, LimitSnapshot, LimitSpec, NumericalPolicy,
    OperationContext, OperationReport, OperationScope, PolicyVersion, ResourceKind, SessionPolicy,
    SessionPrecision, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::{
    CurvePairIsolation, CurvePairProjectionPlane, NURBS_CURVE_PAIR_CANDIDATES,
    NURBS_CURVE_PAIR_DEPTH, NURBS_CURVE_PAIR_SUBDIVISIONS, NurbsCurve,
    isolate_curve_pair_candidates_in_scope,
};
use kgeom::vec::Point3;

/// Fixture identity for the first Q4 curve-pair ladder.
pub const FIXTURE_VERSION: &str = "curve-pair-isolation.v1";
/// Deterministic construction seed recorded by the benchmark registry.
pub const FIXTURE_SEED: u64 = 0x5154_4350_4149_0009;

/// NURBS representation varied independently from proof outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurveFixture {
    /// Polynomial quadratic arch.
    Polynomial,
    /// Positive-weight rational quadratic arch.
    Rational,
}

/// Exact geometric relation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryRelation {
    /// Endpoint contacts retain a conservative cover.
    Retained,
    /// Root control hulls overlap, but subdivision proves the curves separate.
    Separated,
}

/// Reviewed structured stop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitKind {
    /// No configured stop.
    None,
    /// Subdivision work stop.
    Work,
    /// Candidate high-water stop.
    Candidates,
    /// Subdivision-depth stop.
    Depth,
}

impl LimitKind {
    /// Stable manifest spelling.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Work => "work",
            Self::Candidates => "candidates",
            Self::Depth => "depth",
        }
    }
}

/// Stable Q4 curve-pair case.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurvePairIsolationCase {
    /// Canonical five-segment path.
    pub path: &'static str,
    /// First-curve representation.
    pub fixture: CurveFixture,
    /// Geometric relation.
    pub relation: GeometryRelation,
    /// Requested exact subdivision rounds.
    pub requested_depth: u32,
    /// Inclusive work allowance.
    pub work_allowed: u64,
    /// Inclusive candidate allowance.
    pub candidates_allowed: u64,
    /// Inclusive depth allowance.
    pub depth_allowed: u64,
    /// Reviewed retained candidates.
    pub expected_candidates: usize,
    /// Reviewed completion state.
    pub expected_complete: bool,
    /// Reviewed proven-empty state.
    pub expected_proven_empty: bool,
    /// Reviewed stop.
    pub expected_limit: LimitKind,
    /// Reviewed candidate digest.
    pub expected_candidate_digest: u64,
    /// Reviewed complete evidence digest.
    pub expected_output_digest: u64,
}

/// Six cases covering representation, hidden separation, and all resource stops.
pub const CASES: [CurvePairIsolationCase; 6] = [
    case(
        "geometry/curve-pair-isolation/poly-retained-v1/1/depth-3-v1",
        CurveFixture::Polynomial,
        GeometryRelation::Retained,
        policy(3, 1_366, 4_096, 3),
        expected(
            4,
            true,
            false,
            LimitKind::None,
            0xc6d0_acd6_f471_2a52,
            0x4d6f_b6f5_809c_4b32,
        ),
    ),
    case(
        "geometry/curve-pair-isolation/rational-retained-v1/1/depth-3-v1",
        CurveFixture::Rational,
        GeometryRelation::Retained,
        policy(3, 1_366, 4_096, 3),
        expected(
            2,
            true,
            false,
            LimitKind::None,
            0x4f99_4d43_1116_fb10,
            0x69ec_7d53_00f9_6a42,
        ),
    ),
    case(
        "geometry/curve-pair-isolation/poly-separated-v1/1/depth-3-v1",
        CurveFixture::Polynomial,
        GeometryRelation::Separated,
        policy(3, 1_366, 4_096, 3),
        expected(
            0,
            true,
            true,
            LimitKind::None,
            0x2de4_8551_deac_70df,
            0x218c_ff54_d8fe_44f3,
        ),
    ),
    case(
        "geometry/curve-pair-isolation/poly-retained-v1/1/work-low-v1",
        CurveFixture::Polynomial,
        GeometryRelation::Retained,
        policy(3, 1, 4_096, 3),
        expected(
            1,
            false,
            false,
            LimitKind::Work,
            0xbf04_9d39_485c_be6a,
            0x2d90_9abe_bc52_fd4b,
        ),
    ),
    case(
        "geometry/curve-pair-isolation/poly-retained-v1/1/candidate-low-v1",
        CurveFixture::Polynomial,
        GeometryRelation::Retained,
        policy(3, 1_366, 1, 3),
        expected(
            1,
            false,
            false,
            LimitKind::Candidates,
            0xbf04_9d39_485c_be6a,
            0xa98c_3dae_4ad9_e8e6,
        ),
    ),
    case(
        "geometry/curve-pair-isolation/poly-retained-v1/1/depth-low-v1",
        CurveFixture::Polynomial,
        GeometryRelation::Retained,
        policy(3, 1_366, 4_096, 0),
        expected(
            1,
            false,
            false,
            LimitKind::Depth,
            0xbf04_9d39_485c_be6a,
            0xa461_a399_e70b_0a99,
        ),
    ),
];

#[derive(Clone, Copy)]
struct CasePolicy {
    depth: u32,
    work: u64,
    candidates: u64,
    depth_allowed: u64,
}

const fn policy(depth: u32, work: u64, candidates: u64, depth_allowed: u64) -> CasePolicy {
    CasePolicy {
        depth,
        work,
        candidates,
        depth_allowed,
    }
}

#[derive(Clone, Copy)]
struct Expected {
    candidates: usize,
    complete: bool,
    proven_empty: bool,
    limit: LimitKind,
    candidate_digest: u64,
    output_digest: u64,
}

const fn expected(
    candidates: usize,
    complete: bool,
    proven_empty: bool,
    limit: LimitKind,
    candidate_digest: u64,
    output_digest: u64,
) -> Expected {
    Expected {
        candidates,
        complete,
        proven_empty,
        limit,
        candidate_digest,
        output_digest,
    }
}

const fn case(
    path: &'static str,
    fixture: CurveFixture,
    relation: GeometryRelation,
    policy: CasePolicy,
    expected: Expected,
) -> CurvePairIsolationCase {
    CurvePairIsolationCase {
        path,
        fixture,
        relation,
        requested_depth: policy.depth,
        work_allowed: policy.work,
        candidates_allowed: policy.candidates,
        depth_allowed: policy.depth_allowed,
        expected_candidates: expected.candidates,
        expected_complete: expected.complete,
        expected_proven_empty: expected.proven_empty,
        expected_limit: expected.limit,
        expected_candidate_digest: expected.candidate_digest,
        expected_output_digest: expected.output_digest,
    }
}

/// Immutable prepared input; curve and policy construction are never timed.
pub struct CurvePairIsolationFixture {
    first: NurbsCurve,
    second: NurbsCurve,
    session: SessionPolicy,
}

impl CurvePairIsolationFixture {
    /// Time only exact contextual isolation and return deterministic evidence.
    pub fn measure_once(
        &self,
        case: CurvePairIsolationCase,
    ) -> (core::time::Duration, CurvePairIsolationEvidence) {
        let context = OperationContext::new(&self.session, Tolerances::default())
            .expect("Q4 curve-pair policy is valid");
        let mut scope = OperationScope::new(&context);
        let started = std::time::Instant::now();
        let isolation = isolate_curve_pair_candidates_in_scope(
            &self.first,
            self.first.param_range(),
            &self.second,
            self.second.param_range(),
            Tolerances::default().linear(),
            case.requested_depth,
            &mut scope,
        )
        .expect("reviewed Q4 curve-pair isolation must run");
        let elapsed = started.elapsed();
        let (_, report) = scope.finish(Ok(())).into_parts();
        (elapsed, self.evidence(case, &isolation, &report))
    }

    fn evidence(
        &self,
        case: CurvePairIsolationCase,
        isolation: &CurvePairIsolation,
        report: &OperationReport,
    ) -> CurvePairIsolationEvidence {
        let work = usage(report, NURBS_CURVE_PAIR_SUBDIVISIONS);
        let candidates = usage(report, NURBS_CURVE_PAIR_CANDIDATES);
        let depth = usage(report, NURBS_CURVE_PAIR_DEPTH);
        let limits = isolation.limits();
        let limit = if limits.subdivision_work().is_some() {
            LimitKind::Work
        } else if limits.candidate_cells().is_some() {
            LimitKind::Candidates
        } else if limits.subdivision_depth().is_some() {
            LimitKind::Depth
        } else {
            LimitKind::None
        };
        let (limit_attempted_consumed, limit_attempted_allowed) = report
            .limit_events()
            .first()
            .map_or((0, 0), |snapshot| (snapshot.consumed, snapshot.allowed));
        let candidate_digest = candidate_digest(isolation);
        let root_certificate_digest = root_certificate_digest(isolation);
        let mut evidence = CurvePairIsolationEvidence {
            control_points: self.first.points().len() + self.second.points().len(),
            candidates: isolation.candidates().len(),
            requested_depth: isolation.requested_depth(),
            max_candidate_depth: isolation
                .candidates()
                .iter()
                .map(|candidate| candidate.depth())
                .max()
                .unwrap_or(0),
            complete: isolation.is_complete(),
            proven_empty: isolation.is_proven_empty(),
            indeterminate: !isolation.is_complete(),
            conservative_cover: isolation.is_proven_empty()
                || endpoint_contacts_are_covered(isolation),
            limit,
            limit_events: report.limit_events().len(),
            limit_attempted_consumed,
            limit_attempted_allowed,
            work_consumed: work.consumed,
            work_allowed: work.allowed,
            candidate_high_water: candidates.consumed,
            candidates_allowed: candidates.allowed,
            depth_high_water: depth.consumed,
            depth_allowed: depth.allowed,
            unique_root_cells: isolation
                .candidates()
                .iter()
                .filter(|cell| cell.certify_unique_root().is_some())
                .count(),
            root_certificate_digest,
            candidate_digest,
            output_digest: 0,
        };
        evidence.output_digest = evidence.digest(case);
        evidence
    }
}

/// Stable Q4 curve-pair evidence counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurvePairIsolationEvidence {
    /// Total source control points.
    pub control_points: usize,
    /// Retained conservative cells.
    pub candidates: usize,
    /// Requested subdivision rounds.
    pub requested_depth: u32,
    /// Maximum retained cell depth.
    pub max_candidate_depth: u32,
    /// Whether isolation completed.
    pub complete: bool,
    /// Whether an empty complete cover proves separation.
    pub proven_empty: bool,
    /// Whether proof remains incomplete.
    pub indeterminate: bool,
    /// Whether known endpoint contacts remain covered.
    pub conservative_cover: bool,
    /// Structured stop.
    pub limit: LimitKind,
    /// Limit event count.
    pub limit_events: usize,
    /// Attempted usage at the crossing.
    pub limit_attempted_consumed: u64,
    /// Active allowance at the crossing.
    pub limit_attempted_allowed: u64,
    /// Committed subdivision work.
    pub work_consumed: u64,
    /// Work allowance.
    pub work_allowed: u64,
    /// Candidate high water.
    pub candidate_high_water: u64,
    /// Candidate allowance.
    pub candidates_allowed: u64,
    /// Depth high water.
    pub depth_high_water: u64,
    /// Depth allowance.
    pub depth_allowed: u64,
    /// Retained cells with an exact unique transverse-root certificate.
    pub unique_root_cells: usize,
    /// Ordered exact root-certificate digest.
    pub root_certificate_digest: u64,
    /// Exact candidate digest.
    pub candidate_digest: u64,
    /// Complete semantic evidence digest.
    pub output_digest: u64,
}

impl CurvePairIsolationEvidence {
    fn digest(self, case: CurvePairIsolationCase) -> u64 {
        let mut digest = StableHasher::new();
        digest.tag(0xa1);
        digest.count(self.control_points);
        digest.count(self.candidates);
        digest.u64(u64::from(self.requested_depth));
        digest.u64(u64::from(self.max_candidate_depth));
        digest.boolean(self.complete);
        digest.boolean(self.proven_empty);
        digest.boolean(self.indeterminate);
        digest.boolean(self.conservative_cover);
        digest.tag(match self.limit {
            LimitKind::None => 0,
            LimitKind::Work => 1,
            LimitKind::Candidates => 2,
            LimitKind::Depth => 3,
        });
        digest.count(self.limit_events);
        digest.u64(self.limit_attempted_consumed);
        digest.u64(self.limit_attempted_allowed);
        digest.u64(self.work_consumed);
        digest.u64(self.work_allowed);
        digest.u64(self.candidate_high_water);
        digest.u64(self.candidates_allowed);
        digest.u64(self.depth_high_water);
        digest.u64(self.depth_allowed);
        digest.count(self.unique_root_cells);
        digest.u64(self.root_certificate_digest);
        digest.u64(self.candidate_digest);
        digest.tag(match case.fixture {
            CurveFixture::Polynomial => 0,
            CurveFixture::Rational => 1,
        });
        digest.tag(match case.relation {
            GeometryRelation::Retained => 0,
            GeometryRelation::Separated => 1,
        });
        digest.finish()
    }
}

/// Construct one prepared curve pair and contextual policy.
pub fn fixture(case: CurvePairIsolationCase) -> CurvePairIsolationFixture {
    let first = arch(case.fixture == CurveFixture::Rational);
    let second_y = match case.relation {
        GeometryRelation::Retained => 0.0,
        GeometryRelation::Separated => 1.5,
    };
    let second = line(second_y);
    let budget = BudgetPlan::new([
        LimitSpec::new(
            NURBS_CURVE_PAIR_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            case.work_allowed,
        ),
        LimitSpec::new(
            NURBS_CURVE_PAIR_CANDIDATES,
            ResourceKind::Items,
            AccountingMode::HighWater,
            case.candidates_allowed,
        ),
        LimitSpec::new(
            NURBS_CURVE_PAIR_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            case.depth_allowed,
        ),
    ])
    .expect("valid Q4 curve-pair budget");
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        budget,
        PolicyVersion::V1,
    );
    CurvePairIsolationFixture {
        first,
        second,
        session,
    }
}

/// Verify reviewed evidence and conservative incomplete-proof behavior.
pub fn verify(case: CurvePairIsolationCase, evidence: CurvePairIsolationEvidence) {
    assert_eq!(evidence.candidates, case.expected_candidates);
    assert_eq!(evidence.complete, case.expected_complete);
    assert_eq!(evidence.proven_empty, case.expected_proven_empty);
    assert_eq!(evidence.limit, case.expected_limit);
    assert!(evidence.conservative_cover);
    if evidence.limit == LimitKind::None {
        assert_eq!(evidence.limit_events, 0);
    } else {
        assert!(evidence.indeterminate);
        assert!(!evidence.proven_empty);
        assert_eq!(evidence.limit_events, 1);
        assert!(evidence.limit_attempted_consumed > evidence.limit_attempted_allowed);
    }
    assert_ne!(case.expected_candidate_digest, 0);
    assert_ne!(case.expected_output_digest, 0);
    assert_eq!(evidence.candidate_digest, case.expected_candidate_digest);
    assert_eq!(evidence.output_digest, case.expected_output_digest);
}

fn arch(rational: bool) -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ],
        rational.then_some(vec![1.0, 0.75, 1.25]),
    )
    .expect("valid Q4 arch")
}

fn line(y: f64) -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-1.0, y, 0.0), Point3::new(1.0, y, 0.0)],
        None,
    )
    .expect("valid Q4 line")
}

fn endpoint_contacts_are_covered(isolation: &CurvePairIsolation) -> bool {
    [0.0, 1.0].into_iter().all(|parameter| {
        isolation.candidates().iter().any(|candidate| {
            candidate.first_range().contains(parameter)
                && candidate.second_range().contains(parameter)
        })
    })
}

fn usage(report: &OperationReport, stage: StageId) -> LimitSnapshot {
    *report
        .usage()
        .iter()
        .find(|usage| usage.stage == stage)
        .expect("Q4 curve-pair stage is configured")
}

fn candidate_digest(isolation: &CurvePairIsolation) -> u64 {
    let mut digest = StableHasher::new();
    digest.tag(0xa0);
    digest.count(isolation.candidates().len());
    for candidate in isolation.candidates() {
        digest.u64(u64::from(candidate.depth()));
        for range in [candidate.first_range(), candidate.second_range()] {
            digest.f64(range.lo);
            digest.f64(range.hi);
        }
        for bounds in [candidate.first_bounds(), candidate.second_bounds()] {
            for value in [
                bounds.min.x,
                bounds.min.y,
                bounds.min.z,
                bounds.max.x,
                bounds.max.y,
                bounds.max.z,
            ] {
                digest.f64(value);
            }
        }
        for curve in [candidate.first_curve(), candidate.second_curve()] {
            digest.count(curve.degree());
            digest.count(curve.points().len());
            for point in curve.points() {
                digest.f64(point.x);
                digest.f64(point.y);
                digest.f64(point.z);
            }
            if let Some(weights) = curve.weights() {
                digest.tag(1);
                for &weight in weights {
                    digest.f64(weight);
                }
            } else {
                digest.tag(0);
            }
        }
    }
    digest.finish()
}

fn root_certificate_digest(isolation: &CurvePairIsolation) -> u64 {
    let mut digest = StableHasher::new();
    digest.tag(0xa2);
    let certificates = isolation
        .candidates()
        .iter()
        .filter_map(|candidate| candidate.certify_unique_root())
        .collect::<Vec<_>>();
    digest.count(certificates.len());
    for certificate in certificates {
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
            let repeated = fixture.measure_once(case).1;
            assert_eq!(first, repeated, "repeatability drift for {}", case.path);
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
            .filter(|entry| entry["benchmark_target"] == "curve_pair_isolation")
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
            assert_eq!(entry["size_parameters"]["elements"].as_u64(), Some(1));
            assert_eq!(
                entry["policy_values"]["requested_depth"].as_u64(),
                Some(u64::from(case.requested_depth))
            );
            assert_eq!(
                entry["policy_values"]["work_allowed"].as_u64(),
                Some(case.work_allowed)
            );
            assert_eq!(
                entry["policy_values"]["candidates_allowed"].as_u64(),
                Some(case.candidates_allowed)
            );
            assert_eq!(
                entry["policy_values"]["depth_allowed"].as_u64(),
                Some(case.depth_allowed)
            );
            assert_eq!(entry["policy_values"]["execution"], "serial");

            let evidence = fixture(case).measure_once(case).1;
            verify(case, evidence);
            assert_eq!(
                entry["size_parameters"]["control_points"].as_u64(),
                Some(evidence.control_points as u64)
            );
            assert_eq!(
                entry["tolerances"]["linear"].as_f64(),
                Some(Tolerances::default().linear())
            );
            let counters = &entry["expected_result_counters"];
            for (field, actual) in [
                ("candidates", evidence.candidates as u64),
                (
                    "max_candidate_depth",
                    u64::from(evidence.max_candidate_depth),
                ),
                ("limit_events", evidence.limit_events as u64),
                (
                    "limit_attempted_consumed",
                    evidence.limit_attempted_consumed,
                ),
                ("limit_attempted_allowed", evidence.limit_attempted_allowed),
                ("work_consumed", evidence.work_consumed),
                ("candidate_high_water", evidence.candidate_high_water),
                ("depth_high_water", evidence.depth_high_water),
                ("unique_root_cells", evidence.unique_root_cells as u64),
            ] {
                assert_eq!(counters[field].as_u64(), Some(actual), "{field}");
            }
            for (field, actual) in [
                ("complete", evidence.complete),
                ("proven_empty", evidence.proven_empty),
                ("indeterminate", evidence.indeterminate),
                ("conservative_cover", evidence.conservative_cover),
            ] {
                assert_eq!(counters[field].as_bool(), Some(actual), "{field}");
            }
            assert_eq!(counters["limit_kind"], evidence.limit.as_str());
            assert_eq!(
                counters["root_certificate_digest"].as_str(),
                Some(format!("{:016x}", evidence.root_certificate_digest).as_str())
            );
            assert_eq!(
                counters["candidate_digest"].as_str(),
                Some(format!("{:016x}", evidence.candidate_digest).as_str())
            );
            assert_eq!(
                counters["output_digest"].as_str(),
                Some(format!("{:016x}", evidence.output_digest).as_str())
            );
        }
    }
}
