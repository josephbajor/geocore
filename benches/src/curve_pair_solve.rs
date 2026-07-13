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
    ContactKind, CurveCurveIntersections, NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE,
    NURBS_CURVE_PAIR_SEED_ATTEMPTS, ParamOrientation, intersect_bounded_nurbs_nurbs_with_context,
};

/// Fixture identity for the first Q4 curve-pair solve ladder.
pub const FIXTURE_VERSION: &str = "curve-pair-solve.v18";
/// Deterministic construction seed recorded by the registry.
pub const FIXTURE_SEED: u64 = 0x5154_4350_534f_0018;
const DEFAULT_OVERLAP_EQUIVALENCE_ALLOWANCE: u64 = 1_000_000;

/// Geometry varied by the solve ladder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolveFixture {
    /// Polynomial transverse line pair.
    PolynomialTransverse,
    /// Rationally parameterized transverse line pair.
    RationalTransverse,
    /// Noncoplanar quadratic pair with an algebraic root at normalized 1/3.
    AlgebraicSpatial,
    /// Noncoplanar quadratic pair whose root is lifted by signed linear forms.
    AlgebraicLinearForm,
    /// Noncoplanar quadratic pair whose root requires primitive magnitude-two
    /// carrier and residual coefficients.
    AlgebraicPrimitiveForm,
    /// Noncoplanar quadratic pair whose proof requires a primitive carrier
    /// coefficient of magnitude three.
    AlgebraicPrimitiveMagnitudeThree,
    /// Noncoplanar quadratic pair whose proof requires a primitive carrier
    /// coefficient of magnitude four.
    AlgebraicPrimitiveMagnitudeFour,
    /// Noncoplanar quadratic pair whose proof requires a primitive carrier
    /// coefficient of magnitude five.
    AlgebraicPrimitiveMagnitudeFive,
    /// Noncoplanar quadratic pair whose proof requires a primitive carrier
    /// coefficient of magnitude six.
    AlgebraicPrimitiveMagnitudeSix,
    /// Noncoplanar quadratic pair whose proof requires a primitive carrier
    /// coefficient of magnitude seven.
    AlgebraicPrimitiveMagnitudeSeven,
    /// Noncoplanar quadratic pair whose proof requires a primitive carrier
    /// coefficient of magnitude eight.
    AlgebraicPrimitiveMagnitudeEight,
    /// Noncoplanar quadratic pair whose proof requires a primitive carrier
    /// coefficient of magnitude nine.
    AlgebraicPrimitiveMagnitudeNine,
    /// Noncoplanar quadratic pair whose proof requires a primitive carrier
    /// coefficient of magnitude ten.
    AlgebraicPrimitiveMagnitudeTen,
    /// Noncoplanar quadratic pair whose proof requires a primitive carrier
    /// coefficient of magnitude eleven.
    AlgebraicPrimitiveMagnitudeEleven,
    /// Noncoplanar quadratic pair whose proof requires a primitive carrier
    /// coefficient of magnitude twelve.
    AlgebraicPrimitiveMagnitudeTwelve,
    /// Quadratic tangency.
    Tangent,
    /// Quadratic arch with two transverse contacts.
    MultipleRoots,
    /// Overlapping root hulls whose exact subdivision proves separation.
    HiddenMiss,
    /// Byte-identical NURBS representations over the same full range.
    ExactOverlap,
    /// Equal curves whose stored knot multisets differ by exact refinement.
    CommonRefinementOverlap,
    /// Equal descendants with disjoint knot-insertion histories recovered
    /// through checked inverse refinement.
    InverseHistoryOverlap,
    /// A refinement descendant altered after insertion, which must not inherit
    /// exact overlap completion.
    AlteredInverseHistory,
    /// Tolerance-contained parallel curves without exact representation proof.
    SampledOverlap,
}

/// Reviewed structured stop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitKind {
    /// No configured stop.
    None,
    /// Cell-local seed admission stopped.
    SeedAttempts,
    /// Exact overlap-equivalence work admission stopped.
    OverlapWork,
    /// Exact overlap-equivalence temporary-item admission stopped.
    OverlapItems,
}

impl LimitKind {
    /// Stable manifest spelling.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::SeedAttempts => "seed-attempts",
            Self::OverlapWork => "overlap-work",
            Self::OverlapItems => "overlap-items",
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
    /// Inclusive exact overlap-equivalence work allowance.
    pub overlap_work_allowed: u64,
    /// Inclusive exact overlap-equivalence temporary-item allowance.
    pub overlap_items_allowed: u64,
    /// Reviewed emitted contact count.
    pub expected_points: usize,
    /// Reviewed emitted overlap count.
    pub expected_overlaps: usize,
    /// Reviewed complete-domain state.
    pub expected_complete: bool,
    /// Reviewed proven-empty state.
    pub expected_proven_empty: bool,
    /// Reviewed stop.
    pub expected_limit: LimitKind,
    /// Reviewed contact digest.
    pub expected_point_digest: u64,
    /// Reviewed ordered overlap-extent/orientation digest.
    pub expected_overlap_digest: u64,
    /// Reviewed complete evidence digest.
    pub expected_output_digest: u64,
}

/// Twenty-eight cases covering representation, spatial existence, contact character,
/// overlap proof, and limits.
pub const CASES: [CurvePairSolveCase; 28] = [
    case(
        "geometry/curve-pair-solve/poly-transverse-v1/1/default-v1",
        SolveFixture::PolynomialTransverse,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0x617e_1b7b_48fd_b84a,
                0x16e0_85b4_d5ef_f9c3,
                0x0e8a_0c8f_c288_7e68,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/rational-transverse-v1/1/default-v1",
        SolveFixture::RationalTransverse,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0xd116_a5f6_4d2e_545a,
                0x16e0_85b4_d5ef_f9c3,
                0x2619_4d14_a6ca_e95e,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-spatial-v1/1/default-v1",
        SolveFixture::AlgebraicSpatial,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0xcba7_208e_589b_20a1,
                0x16e0_85b4_d5ef_f9c3,
                0xd4d1_e57b_2449_a796,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-linear-form-v1/1/default-v1",
        SolveFixture::AlgebraicLinearForm,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0xcaee_0659_d4d0_264f,
                0x16e0_85b4_d5ef_f9c3,
                0x4e05_63b1_6632_dfdf,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-form-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveForm,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0x11a8_5af9_7585_8be6,
                0x16e0_85b4_d5ef_f9c3,
                0xf39a_c7cd_8567_2726,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-magnitude-three-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveMagnitudeThree,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0x0d56_1692_2c10_261b,
                0x16e0_85b4_d5ef_f9c3,
                0x831a_92d7_8131_ea32,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-magnitude-four-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveMagnitudeFour,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0xa6c4_0bd1_f119_46a6,
                0x16e0_85b4_d5ef_f9c3,
                0x80fe_0444_7e33_cc5b,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-magnitude-five-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveMagnitudeFive,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0x67ad_1e9d_8a57_f336,
                0x16e0_85b4_d5ef_f9c3,
                0x0b8b_2dfc_eb14_9fa7,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-magnitude-six-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveMagnitudeSix,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0x1ad0_3d1b_3310_10b8,
                0x16e0_85b4_d5ef_f9c3,
                0x8dcb_c69a_dbd3_7afe,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-magnitude-seven-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveMagnitudeSeven,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0xb8e6_2351_1e4e_5be5,
                0x16e0_85b4_d5ef_f9c3,
                0x7094_9ce9_570b_442f,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-magnitude-eight-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveMagnitudeEight,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0xb8e6_2351_1e4e_5be5,
                0x16e0_85b4_d5ef_f9c3,
                0x9b8a_b02c_c82e_4827,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-magnitude-nine-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveMagnitudeNine,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0xea35_3685_4d6b_432c,
                0x16e0_85b4_d5ef_f9c3,
                0x6f2a_90dc_efff_c937,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-magnitude-ten-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveMagnitudeTen,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0x5ea9_8a33_9380_41c5,
                0x16e0_85b4_d5ef_f9c3,
                0x70b8_c902_2d6e_cfbb,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-magnitude-eleven-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveMagnitudeEleven,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0x9497_c3af_110f_59a1,
                0x16e0_85b4_d5ef_f9c3,
                0x36e8_1ec0_e9ec_8719,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/algebraic-primitive-magnitude-twelve-v1/1/default-v1",
        SolveFixture::AlgebraicPrimitiveMagnitudeTwelve,
        policy(4_096),
        expected(
            1,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0x6fbe_da9c_216e_24c3,
                0x16e0_85b4_d5ef_f9c3,
                0x2610_cd86_2dcb_733e,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/poly-tangent-v1/1/default-v1",
        SolveFixture::Tangent,
        policy(4_096),
        expected(
            1,
            0,
            false,
            false,
            LimitKind::None,
            digests(
                0x617e_1c7b_48fd_b9fd,
                0x16e0_85b4_d5ef_f9c3,
                0xee5e_159c_7ab1_60c3,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/poly-two-root-v1/2/default-v1",
        SolveFixture::MultipleRoots,
        policy(4_096),
        expected(
            2,
            0,
            true,
            false,
            LimitKind::None,
            digests(
                0x3606_10ba_1318_ae30,
                0x16e0_85b4_d5ef_f9c3,
                0xd436_d965_d4b6_4777,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/poly-hidden-miss-v1/0/default-v1",
        SolveFixture::HiddenMiss,
        policy(4_096),
        expected(
            0,
            0,
            true,
            true,
            LimitKind::None,
            digests(
                0x6489_db2b_285b_d20f,
                0x16e0_85b4_d5ef_f9c3,
                0x14fb_df45_5f91_91b4,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/exact-overlap-v1/1/default-v1",
        SolveFixture::ExactOverlap,
        policy(4_096),
        expected(
            0,
            1,
            true,
            false,
            LimitKind::None,
            digests(
                0x6489_db2b_285b_d20f,
                0xeebe_95f0_8459_1be6,
                0xfede_9ccb_0a6b_3a25,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/common-refinement-overlap-v1/1/default-v1",
        SolveFixture::CommonRefinementOverlap,
        policy(4_096),
        expected(
            0,
            1,
            true,
            false,
            LimitKind::None,
            digests(
                0x6489_db2b_285b_d20f,
                0xa70a_9e47_b017_2306,
                0xee72_6cb3_a9bf_973b,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/inverse-history-overlap-v1/1/default-v1",
        SolveFixture::InverseHistoryOverlap,
        policy(4_096),
        expected(
            0,
            1,
            true,
            false,
            LimitKind::None,
            digests(
                0x6489_db2b_285b_d20f,
                0xeebe_95f0_8459_1be6,
                0x0ad7_fc6d_2592_2d73,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/inverse-history-overlap-v1/0/work-denied-v1",
        SolveFixture::InverseHistoryOverlap,
        SolvePolicy {
            seed_attempts: 4_096,
            overlap_work: 7_961,
            overlap_items: DEFAULT_OVERLAP_EQUIVALENCE_ALLOWANCE,
        },
        expected(
            0,
            0,
            false,
            false,
            LimitKind::OverlapWork,
            digests(
                0x6489_db2b_285b_d20f,
                0x16e0_85b4_d5ef_f9c3,
                0x16c6_f1c4_f139_c1ed,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/inverse-history-overlap-v1/0/items-denied-v1",
        SolveFixture::InverseHistoryOverlap,
        SolvePolicy {
            seed_attempts: 4_096,
            overlap_work: DEFAULT_OVERLAP_EQUIVALENCE_ALLOWANCE,
            overlap_items: 7_479,
        },
        expected(
            0,
            0,
            false,
            false,
            LimitKind::OverlapItems,
            digests(
                0x6489_db2b_285b_d20f,
                0x16e0_85b4_d5ef_f9c3,
                0x5282_9ff5_7ad5_595a,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/altered-inverse-history-v1/2/default-v1",
        SolveFixture::AlteredInverseHistory,
        policy(4_096),
        expected(
            2,
            0,
            false,
            false,
            LimitKind::None,
            digests(
                0x8465_b61a_a5fb_dbf7,
                0x16e0_85b4_d5ef_f9c3,
                0x5974_1fe9_070b_9bba,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/common-refinement-overlap-v1/0/work-denied-v1",
        SolveFixture::CommonRefinementOverlap,
        SolvePolicy {
            seed_attempts: 4_096,
            overlap_work: 5_157,
            overlap_items: DEFAULT_OVERLAP_EQUIVALENCE_ALLOWANCE,
        },
        expected(
            0,
            0,
            false,
            false,
            LimitKind::OverlapWork,
            digests(
                0x6489_db2b_285b_d20f,
                0x16e0_85b4_d5ef_f9c3,
                0x86af_8ec2_aa1e_3d32,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/common-refinement-overlap-v1/0/items-denied-v1",
        SolveFixture::CommonRefinementOverlap,
        SolvePolicy {
            seed_attempts: 4_096,
            overlap_work: DEFAULT_OVERLAP_EQUIVALENCE_ALLOWANCE,
            overlap_items: 4_849,
        },
        expected(
            0,
            0,
            false,
            false,
            LimitKind::OverlapItems,
            digests(
                0x6489_db2b_285b_d20f,
                0x16e0_85b4_d5ef_f9c3,
                0x84aa_8afb_689a_3758,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/sampled-overlap-v1/1/default-v1",
        SolveFixture::SampledOverlap,
        policy(4_096),
        expected(
            0,
            1,
            false,
            false,
            LimitKind::None,
            digests(
                0x6489_db2b_285b_d20f,
                0xeebe_95f0_8459_1be6,
                0xd795_7920_26df_5106,
            ),
        ),
    ),
    case(
        "geometry/curve-pair-solve/poly-tangent-v1/1/seed-denied-v1",
        SolveFixture::Tangent,
        policy(0),
        expected(
            0,
            0,
            false,
            false,
            LimitKind::SeedAttempts,
            digests(
                0x6489_db2b_285b_d20f,
                0x16e0_85b4_d5ef_f9c3,
                0xf1c5_b32b_c96c_335c,
            ),
        ),
    ),
];

#[derive(Clone, Copy)]
struct Expected {
    points: usize,
    overlaps: usize,
    complete: bool,
    proven_empty: bool,
    limit: LimitKind,
    point_digest: u64,
    overlap_digest: u64,
    output_digest: u64,
}

#[derive(Clone, Copy)]
struct ExpectedDigests {
    point: u64,
    overlap: u64,
    output: u64,
}

const fn digests(point: u64, overlap: u64, output: u64) -> ExpectedDigests {
    ExpectedDigests {
        point,
        overlap,
        output,
    }
}

const fn expected(
    points: usize,
    overlaps: usize,
    complete: bool,
    proven_empty: bool,
    limit: LimitKind,
    digests: ExpectedDigests,
) -> Expected {
    Expected {
        points,
        overlaps,
        complete,
        proven_empty,
        limit,
        point_digest: digests.point,
        overlap_digest: digests.overlap,
        output_digest: digests.output,
    }
}

#[derive(Clone, Copy)]
struct SolvePolicy {
    seed_attempts: u64,
    overlap_work: u64,
    overlap_items: u64,
}

const fn policy(seed_attempts: u64) -> SolvePolicy {
    SolvePolicy {
        seed_attempts,
        overlap_work: DEFAULT_OVERLAP_EQUIVALENCE_ALLOWANCE,
        overlap_items: DEFAULT_OVERLAP_EQUIVALENCE_ALLOWANCE,
    }
}

const fn case(
    path: &'static str,
    fixture: SolveFixture,
    policy: SolvePolicy,
    expected: Expected,
) -> CurvePairSolveCase {
    CurvePairSolveCase {
        path,
        fixture,
        seed_attempts_allowed: policy.seed_attempts,
        overlap_work_allowed: policy.overlap_work,
        overlap_items_allowed: policy.overlap_items,
        expected_points: expected.points,
        expected_overlaps: expected.overlaps,
        expected_complete: expected.complete,
        expected_proven_empty: expected.proven_empty,
        expected_limit: expected.limit,
        expected_point_digest: expected.point_digest,
        expected_overlap_digest: expected.overlap_digest,
        expected_output_digest: expected.output_digest,
    }
}

/// Immutable prepared geometry and session policy; neither is timed.
pub struct CurvePairSolveFixture {
    first: NurbsCurve,
    first_range: kgeom::param::ParamRange,
    second: NurbsCurve,
    second_range: kgeom::param::ParamRange,
    session: SessionPolicy,
}

impl CurvePairSolveFixture {
    /// Time the public contextual solve and return deterministic evidence.
    pub fn measure_once(
        &self,
        case: CurvePairSolveCase,
    ) -> (core::time::Duration, CurvePairSolveEvidence) {
        let overrides = BudgetPlan::new([
            LimitSpec::new(
                NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                case.overlap_work_allowed,
            ),
            LimitSpec::new(
                NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE,
                ResourceKind::Items,
                AccountingMode::Cumulative,
                case.overlap_items_allowed,
            ),
            LimitSpec::new(
                NURBS_CURVE_PAIR_SEED_ATTEMPTS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                case.seed_attempts_allowed,
            ),
        ])
        .expect("valid Q4 seed override");
        let context = OperationContext::new(&self.session, Tolerances::default())
            .expect("Q4 curve-pair solve policy is valid")
            .with_budget_overrides(overrides);
        let started = std::time::Instant::now();
        let outcome = intersect_bounded_nurbs_nurbs_with_context(
            &self.first,
            self.first_range,
            &self.second,
            self.second_range,
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
        let isolation_work = usage(report, NURBS_CURVE_PAIR_SUBDIVISIONS, ResourceKind::Work);
        let candidates = usage(report, NURBS_CURVE_PAIR_CANDIDATES, ResourceKind::Items);
        let depth = usage(report, NURBS_CURVE_PAIR_DEPTH, ResourceKind::Depth);
        let overlap_work = usage(
            report,
            NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE,
            ResourceKind::Work,
        );
        let overlap_items = usage(
            report,
            NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE,
            ResourceKind::Items,
        );
        let seeds = usage(report, NURBS_CURVE_PAIR_SEED_ATTEMPTS, ResourceKind::Work);
        let limit = report
            .limit_events()
            .first()
            .map_or(LimitKind::None, |event| {
                match (event.stage, event.resource) {
                    (NURBS_CURVE_PAIR_SEED_ATTEMPTS, ResourceKind::Work) => LimitKind::SeedAttempts,
                    (NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE, ResourceKind::Work) => {
                        LimitKind::OverlapWork
                    }
                    (NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE, ResourceKind::Items) => {
                        LimitKind::OverlapItems
                    }
                    _ => LimitKind::None,
                }
            });
        let (limit_attempted_consumed, limit_attempted_allowed) = report
            .limit_events()
            .first()
            .map_or((0, 0), |event| (event.consumed, event.allowed));
        let point_digest = point_digest(result);
        let overlap_digest = overlap_digest(result);
        let incomplete_evidence_digest = incomplete_evidence_digest(result);
        let root_certificate_digest = root_certificate_digest(result);
        let mut evidence = CurvePairSolveEvidence {
            control_points: self.first.points().len() + self.second.points().len(),
            points: result.points.len(),
            overlaps: result.overlaps.len(),
            complete: result.is_complete(),
            proven_empty: result.is_proven_empty(),
            indeterminate: !result.is_complete(),
            verified_witnesses: witnesses_are_verified(self, result),
            limit,
            limit_events: report.limit_events().len(),
            limit_attempted_consumed,
            limit_attempted_allowed,
            isolation_work: isolation_work.consumed,
            candidate_high_water: candidates.consumed,
            depth_high_water: depth.consumed,
            overlap_equivalence_work: overlap_work.consumed,
            overlap_equivalence_work_allowed: overlap_work.allowed,
            overlap_equivalence_items: overlap_items.consumed,
            overlap_equivalence_items_allowed: overlap_items.allowed,
            seed_attempts: seeds.consumed,
            seed_attempts_allowed: seeds.allowed,
            incomplete_evidence: result.incomplete_evidence().len(),
            incomplete_evidence_digest,
            root_certificates: result.root_certificates().len(),
            root_certificate_digest,
            point_digest,
            overlap_digest,
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
    /// Exact overlap-equivalence scan/reconstruction work.
    pub overlap_equivalence_work: u64,
    /// Inclusive exact overlap-equivalence work allowance.
    pub overlap_equivalence_work_allowed: u64,
    /// Temporary logical items admitted for overlap equivalence.
    pub overlap_equivalence_items: u64,
    /// Inclusive overlap-equivalence temporary-item allowance.
    pub overlap_equivalence_items_allowed: u64,
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
    /// Ordered overlap ranges and orientation digest.
    pub overlap_digest: u64,
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
            LimitKind::OverlapWork => 2,
            LimitKind::OverlapItems => 3,
        });
        digest.count(self.limit_events);
        digest.u64(self.limit_attempted_consumed);
        digest.u64(self.limit_attempted_allowed);
        digest.u64(self.isolation_work);
        digest.u64(self.candidate_high_water);
        digest.u64(self.depth_high_water);
        digest.u64(self.overlap_equivalence_work);
        digest.u64(self.overlap_equivalence_work_allowed);
        digest.u64(self.overlap_equivalence_items);
        digest.u64(self.overlap_equivalence_items_allowed);
        digest.u64(self.seed_attempts);
        digest.u64(self.seed_attempts_allowed);
        digest.count(self.incomplete_evidence);
        digest.u64(self.incomplete_evidence_digest);
        digest.count(self.root_certificates);
        digest.u64(self.root_certificate_digest);
        digest.u64(self.point_digest);
        digest.u64(self.overlap_digest);
        digest.tag(match case.fixture {
            SolveFixture::PolynomialTransverse => 0,
            SolveFixture::RationalTransverse => 1,
            SolveFixture::Tangent => 2,
            SolveFixture::MultipleRoots => 3,
            SolveFixture::HiddenMiss => 4,
            SolveFixture::ExactOverlap => 5,
            SolveFixture::CommonRefinementOverlap => 6,
            SolveFixture::SampledOverlap => 7,
            SolveFixture::AlgebraicSpatial => 8,
            SolveFixture::AlgebraicLinearForm => 9,
            SolveFixture::InverseHistoryOverlap => 10,
            SolveFixture::AlteredInverseHistory => 11,
            SolveFixture::AlgebraicPrimitiveForm => 12,
            SolveFixture::AlgebraicPrimitiveMagnitudeThree => 13,
            SolveFixture::AlgebraicPrimitiveMagnitudeFour => 14,
            SolveFixture::AlgebraicPrimitiveMagnitudeFive => 15,
            SolveFixture::AlgebraicPrimitiveMagnitudeSix => 16,
            SolveFixture::AlgebraicPrimitiveMagnitudeSeven => 17,
            SolveFixture::AlgebraicPrimitiveMagnitudeEight => 18,
            SolveFixture::AlgebraicPrimitiveMagnitudeNine => 19,
            SolveFixture::AlgebraicPrimitiveMagnitudeTen => 20,
            SolveFixture::AlgebraicPrimitiveMagnitudeEleven => 21,
            SolveFixture::AlgebraicPrimitiveMagnitudeTwelve => 22,
        });
        digest.finish()
    }
}

/// Construct one prepared solve fixture.
pub fn fixture(case: CurvePairSolveCase) -> CurvePairSolveFixture {
    let (first, second) = curves(case.fixture);
    let (first_range, second_range) = match case.fixture {
        SolveFixture::CommonRefinementOverlap => (
            kgeom::param::ParamRange::new(0.25, 0.75),
            kgeom::param::ParamRange::new(0.5, 1.0),
        ),
        SolveFixture::AlgebraicSpatial => (
            kgeom::param::ParamRange::new(0.25, 0.5),
            kgeom::param::ParamRange::new(0.25, 0.5),
        ),
        SolveFixture::AlgebraicLinearForm
        | SolveFixture::AlgebraicPrimitiveForm
        | SolveFixture::AlgebraicPrimitiveMagnitudeThree
        | SolveFixture::AlgebraicPrimitiveMagnitudeFour
        | SolveFixture::AlgebraicPrimitiveMagnitudeFive
        | SolveFixture::AlgebraicPrimitiveMagnitudeSix
        | SolveFixture::AlgebraicPrimitiveMagnitudeSeven
        | SolveFixture::AlgebraicPrimitiveMagnitudeEight
        | SolveFixture::AlgebraicPrimitiveMagnitudeNine
        | SolveFixture::AlgebraicPrimitiveMagnitudeTen
        | SolveFixture::AlgebraicPrimitiveMagnitudeEleven
        | SolveFixture::AlgebraicPrimitiveMagnitudeTwelve => (
            kgeom::param::ParamRange::new(0.0, 1.0),
            kgeom::param::ParamRange::new(0.25, 0.5),
        ),
        _ => (first.param_range(), second.param_range()),
    };
    CurvePairSolveFixture {
        first,
        first_range,
        second,
        second_range,
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
    assert_eq!(evidence.overlaps, case.expected_overlaps);
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
    assert_eq!(
        evidence.overlap_equivalence_work_allowed,
        case.overlap_work_allowed
    );
    assert_eq!(
        evidence.overlap_equivalence_items_allowed,
        case.overlap_items_allowed
    );
    if evidence.limit == LimitKind::None {
        assert_eq!(evidence.limit_events, 0);
    } else {
        assert!(evidence.indeterminate);
        assert_eq!(evidence.limit_events, 1);
        assert!(evidence.limit_attempted_consumed > evidence.limit_attempted_allowed);
    }
    assert_ne!(case.expected_point_digest, 0);
    assert_ne!(case.expected_overlap_digest, 0);
    assert_ne!(case.expected_output_digest, 0);
    assert_eq!(evidence.point_digest, case.expected_point_digest);
    assert_eq!(evidence.overlap_digest, case.expected_overlap_digest);
    assert_eq!(evidence.output_digest, case.expected_output_digest);
}

fn curves(fixture: SolveFixture) -> (NurbsCurve, NurbsCurve) {
    match fixture {
        SolveFixture::PolynomialTransverse => (diagonal(None), horizontal(0.0)),
        SolveFixture::RationalTransverse => (diagonal(Some(vec![1.0, 1.5])), horizontal(0.0)),
        SolveFixture::AlgebraicSpatial => algebraic_spatial_pair(),
        SolveFixture::AlgebraicLinearForm => algebraic_linear_form_pair(),
        SolveFixture::AlgebraicPrimitiveForm => algebraic_primitive_form_pair(),
        SolveFixture::AlgebraicPrimitiveMagnitudeThree => {
            algebraic_primitive_magnitude_three_pair()
        }
        SolveFixture::AlgebraicPrimitiveMagnitudeFour => algebraic_primitive_magnitude_four_pair(),
        SolveFixture::AlgebraicPrimitiveMagnitudeFive => algebraic_primitive_magnitude_five_pair(),
        SolveFixture::AlgebraicPrimitiveMagnitudeSix => algebraic_primitive_magnitude_six_pair(),
        SolveFixture::AlgebraicPrimitiveMagnitudeSeven => {
            algebraic_primitive_magnitude_seven_pair()
        }
        SolveFixture::AlgebraicPrimitiveMagnitudeEight => {
            algebraic_primitive_magnitude_eight_pair()
        }
        SolveFixture::AlgebraicPrimitiveMagnitudeNine => algebraic_primitive_magnitude_nine_pair(),
        SolveFixture::AlgebraicPrimitiveMagnitudeTen => algebraic_primitive_magnitude_ten_pair(),
        SolveFixture::AlgebraicPrimitiveMagnitudeEleven => {
            algebraic_primitive_magnitude_eleven_pair()
        }
        SolveFixture::AlgebraicPrimitiveMagnitudeTwelve => {
            algebraic_primitive_magnitude_twelve_pair()
        }
        SolveFixture::Tangent => (tangent_parabola(), horizontal(0.0)),
        SolveFixture::MultipleRoots => (arch(), horizontal(0.5)),
        SolveFixture::HiddenMiss => (arch(), horizontal(1.5)),
        SolveFixture::ExactOverlap => (horizontal(0.0), horizontal(0.0)),
        SolveFixture::CommonRefinementOverlap => {
            let coarse = arch();
            let refined = coarse
                .with_knot_inserted(0.5, 1)
                .expect("valid Q4 exact common refinement");
            (coarse, refined)
        }
        SolveFixture::InverseHistoryOverlap => inverse_history_pair(false),
        SolveFixture::AlteredInverseHistory => inverse_history_pair(true),
        SolveFixture::SampledOverlap => (horizontal(0.0), horizontal(0.5e-8)),
    }
}

fn inverse_history_pair(altered: bool) -> (NurbsCurve, NurbsCurve) {
    let coarse = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ],
        Some(vec![1.0, 1.5, 2.0]),
    )
    .expect("valid Q4 inverse-history ancestor");
    let left = coarse
        .with_knot_inserted(0.25, 1)
        .expect("valid Q4 left insertion history");
    let mut right = coarse
        .with_knot_inserted(0.75, 1)
        .expect("valid Q4 right insertion history");
    if altered {
        let mut points = right.points().to_vec();
        points[1].y += 1.0e-4;
        right = NurbsCurve::new(
            right.degree(),
            right.knots().as_slice().to_vec(),
            points,
            right.weights().map(<[f64]>::to_vec),
        )
        .expect("valid Q4 altered insertion history");
    }
    (left, right)
}

fn algebraic_spatial_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 1.0),
            Point3::new(1.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 algebraic-spatial first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(0.5, 0.5, 1.0),
            Point3::new(1.0, 2.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 algebraic-spatial second curve");
    (first, second)
}

fn algebraic_linear_form_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 1.0),
            Point3::new(4.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 algebraic-linear-form first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, -1.0, 1.0),
            Point3::new(1.5, 0.5, 0.5),
            Point3::new(2.0, 2.0, -2.0),
        ],
        None,
    )
    .expect("valid Q4 algebraic-linear-form second curve");
    (first, second)
}

fn algebraic_primitive_form_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 algebraic-primitive-form first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(2.0, -4.0, 1.0),
            Point3::new(3.0, 2.0, 0.5),
            Point3::new(4.0, 8.0, -2.0),
        ],
        None,
    )
    .expect("valid Q4 algebraic-primitive-form second curve");
    (first, second)
}

fn algebraic_primitive_magnitude_three_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-three first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, 3.0, 7.0),
            Point3::new(3.5, -1.5, -2.5),
            Point3::new(6.0, -6.0, -14.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-three second curve");
    (first, second)
}

fn algebraic_primitive_magnitude_four_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-four first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, 4.0, 9.0),
            Point3::new(3.5, -2.0, -3.5),
            Point3::new(6.0, -8.0, -18.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-four second curve");
    (first, second)
}

fn algebraic_primitive_magnitude_five_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-five first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, 5.0, 11.0),
            Point3::new(3.5, -2.5, -4.5),
            Point3::new(6.0, -10.0, -22.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-five second curve");
    (first, second)
}

fn algebraic_primitive_magnitude_six_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-six first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, 6.0, 13.0),
            Point3::new(3.5, -3.0, -5.5),
            Point3::new(6.0, -12.0, -26.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-six second curve");
    (first, second)
}

fn algebraic_primitive_magnitude_seven_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-seven first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, 7.0, 15.0),
            Point3::new(3.5, -3.5, -6.5),
            Point3::new(6.0, -14.0, -30.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-seven second curve");
    (first, second)
}

fn algebraic_primitive_magnitude_eight_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-eight first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, 8.0, 17.0),
            Point3::new(3.5, -4.0, -7.5),
            Point3::new(6.0, -16.0, -34.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-eight second curve");
    (first, second)
}

fn algebraic_primitive_magnitude_nine_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-nine first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, 9.0, 19.0),
            Point3::new(3.5, -4.5, -8.5),
            Point3::new(6.0, -18.0, -38.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-nine second curve");
    (first, second)
}

fn algebraic_primitive_magnitude_ten_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-ten first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, 10.0, 21.0),
            Point3::new(3.5, -5.0, -9.5),
            Point3::new(6.0, -20.0, -42.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-ten second curve");
    (first, second)
}

fn algebraic_primitive_magnitude_eleven_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-eleven first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, 11.0, 23.0),
            Point3::new(3.5, -5.5, -10.5),
            Point3::new(6.0, -22.0, -46.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-eleven second curve");
    (first, second)
}

fn algebraic_primitive_magnitude_twelve_pair() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        2,
        knots.clone(),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(8.0, 0.0, 0.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-twelve first curve");
    let second = NurbsCurve::new(
        2,
        knots,
        vec![
            Point3::new(1.0, 12.0, 25.0),
            Point3::new(3.5, -6.0, -11.5),
            Point3::new(6.0, -24.0, -50.0),
        ],
        None,
    )
    .expect("valid Q4 magnitude-twelve second curve");
    (first, second)
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

fn usage(report: &OperationReport, stage: StageId, resource: ResourceKind) -> LimitSnapshot {
    *report
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == resource)
        .expect("Q4 curve-pair solve stage is configured")
}

fn witnesses_are_verified(
    fixture: &CurvePairSolveFixture,
    result: &CurveCurveIntersections,
) -> bool {
    let points_verified = result.points.iter().all(|point| {
        let first_point = fixture.first.eval(point.t_a);
        let second_point = fixture.second.eval(point.t_b);
        fixture.first_range.contains(point.t_a)
            && fixture.second_range.contains(point.t_b)
            && point.residual == first_point.dist(second_point)
            && point.residual <= Tolerances::default().linear()
            && point.point == (first_point + second_point) / 2.0
    });
    let overlaps_verified = result.overlaps.iter().all(|overlap| {
        let (second_start, second_end) = match overlap.orientation {
            ParamOrientation::Same => (overlap.b.lo, overlap.b.hi),
            ParamOrientation::Reversed => (overlap.b.hi, overlap.b.lo),
        };
        overlap.a.width() > 0.0
            && overlap.b.width() > 0.0
            && fixture.first_range.contains(overlap.a.lo)
            && fixture.first_range.contains(overlap.a.hi)
            && fixture.second_range.contains(overlap.b.lo)
            && fixture.second_range.contains(overlap.b.hi)
            && fixture
                .first
                .eval(overlap.a.lo)
                .dist(fixture.second.eval(second_start))
                <= Tolerances::default().linear()
            && fixture
                .first
                .eval(overlap.a.hi)
                .dist(fixture.second.eval(second_end))
                <= Tolerances::default().linear()
    });
    points_verified && overlaps_verified
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

fn overlap_digest(result: &CurveCurveIntersections) -> u64 {
    let mut digest = StableHasher::new();
    digest.tag(0xb4);
    digest.count(result.overlaps.len());
    for overlap in &result.overlaps {
        digest.f64(overlap.a.lo);
        digest.f64(overlap.a.hi);
        digest.f64(overlap.b.lo);
        digest.f64(overlap.b.hi);
        digest.tag(match overlap.orientation {
            ParamOrientation::Same => 0,
            ParamOrientation::Reversed => 1,
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
            assert_eq!(
                entry["policy_values"]["overlap_work_allowed"].as_u64(),
                Some(case.overlap_work_allowed)
            );
            assert_eq!(
                entry["policy_values"]["overlap_items_allowed"].as_u64(),
                Some(case.overlap_items_allowed)
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
                SolveFixture::MultipleRoots | SolveFixture::AlteredInverseHistory => 2,
                _ => 1,
            };
            assert_eq!(
                entry["size_parameters"]["elements"].as_u64(),
                Some(expected_elements as u64)
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
                (
                    "overlap_equivalence_work",
                    evidence.overlap_equivalence_work,
                ),
                (
                    "overlap_equivalence_work_allowed",
                    evidence.overlap_equivalence_work_allowed,
                ),
                (
                    "overlap_equivalence_items",
                    evidence.overlap_equivalence_items,
                ),
                (
                    "overlap_equivalence_items_allowed",
                    evidence.overlap_equivalence_items_allowed,
                ),
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
                counters["overlap_digest"].as_str(),
                Some(format!("{:016x}", evidence.overlap_digest).as_str())
            );
            assert_eq!(
                counters["output_digest"].as_str(),
                Some(format!("{:016x}", evidence.output_digest).as_str())
            );
        }
    }
}
