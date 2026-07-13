//! Deterministic Q4 contextual NURBS implicit-isolation fixtures and evidence.

use kcore::operation::{
    AccountingMode, BudgetPlan, ExecutionPolicy, LimitSnapshot, LimitSpec, NumericalPolicy,
    OperationContext, OperationReport, OperationScope, PolicyVersion, ResourceKind, SessionPolicy,
    SessionPrecision, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::nurbs::{
    ImplicitPatchIsolation, NURBS_IMPLICIT_ISOLATION_CANDIDATES, NURBS_IMPLICIT_ISOLATION_DEPTH,
    NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS, NurbsSurface, NurbsSurfaceBvh,
};
use kgeom::surface::{Plane, Surface};
use kgeom::vec::{Point3, Vec3};

/// Fixture identity shared by the first Q4 isolation slice.
pub const FIXTURE_VERSION: &str = "nurbs-isolation.v3";
/// Deterministic fixture seed (construction itself is not randomized).
pub const FIXTURE_SEED: u64 = 0x5154_4e55_5242_0006;

const ROUNDOFF_CONTACT_Z: f64 = 9_007_199_254_740_991.0;

/// NURBS representation and source-patch scale varied by Q4.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceFixture {
    /// One polynomial bilinear patch.
    PolynomialSingle,
    /// One rational bilinear patch with non-unit positive weights.
    RationalSingle,
    /// Four rational bilinear Bezier patches extracted from a 3×3 net.
    RationalFourPatch,
    /// Cubic extrusion whose rounded child hulls lose an exact plane contact.
    SubdivisionRoundoff,
}

/// Implicit geometry relation varied independently from source representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryRelation {
    /// The NURBS lies on the implicit plane, so a conservative cover remains.
    Retained,
    /// The implicit plane is far away, so interval pruning proves a miss.
    Separated,
}

/// Reviewed limit outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitKind {
    /// Isolation completed without a configured or numeric stop.
    None,
    /// Subdivision work was exhausted.
    Work,
    /// Candidate-cover high water was exhausted.
    Candidates,
}

impl LimitKind {
    /// Stable registry spelling.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Work => "work",
            Self::Candidates => "candidates",
        }
    }
}

/// Stable Q4 case definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NurbsIsolationCase {
    /// Five-segment benchmark path.
    pub path: &'static str,
    /// Source representation and patch scale.
    pub fixture: SurfaceFixture,
    /// Implicit relation.
    pub relation: GeometryRelation,
    /// Requested exact subdivision depth.
    pub requested_depth: u32,
    /// Inclusive subdivision-work allowance.
    pub work_allowed: u64,
    /// Inclusive candidate-cover allowance.
    pub candidates_allowed: u64,
    /// Inclusive subdivision-depth allowance.
    pub depth_allowed: u64,
    /// Reviewed extracted Bezier patch count.
    pub expected_patches: usize,
    /// Reviewed retained candidate count.
    pub expected_candidates: usize,
    /// Reviewed completion state.
    pub expected_complete: bool,
    /// Reviewed complete-miss proof state.
    pub expected_proven_empty: bool,
    /// Reviewed structured limit outcome.
    pub expected_limit: LimitKind,
    /// Reviewed candidate-cover digest.
    pub expected_candidate_digest: u64,
    /// Reviewed complete semantic output digest.
    pub expected_output_digest: u64,
}

/// Eight cases varying representation, source provenance, geometry relation,
/// and exact budgets.
pub const CASES: [NurbsIsolationCase; 8] = [
    case(
        "geometry/nurbs-isolation/poly-single-v1/1/work-exact-v1",
        SurfaceFixture::PolynomialSingle,
        GeometryRelation::Retained,
        policy(1, 29, 2, 1),
        expected(
            1,
            2,
            true,
            false,
            LimitKind::None,
            0x5b02_05cb_3766_1ca0,
            0xaf4f_1f71_b59b_1dd4,
        ),
    ),
    case(
        "geometry/nurbs-isolation/rational-single-v1/1/work-exact-v1",
        SurfaceFixture::RationalSingle,
        GeometryRelation::Retained,
        policy(1, 29, 2, 1),
        expected(
            1,
            2,
            true,
            false,
            LimitKind::None,
            0xe7d7_4902_4af7_bf16,
            0x0a3b_2e64_dca3_daf2,
        ),
    ),
    case(
        "geometry/nurbs-isolation/rational-four-patch-v1/4/retained-v1",
        SurfaceFixture::RationalFourPatch,
        GeometryRelation::Retained,
        policy(1, 404, 8, 1),
        expected(
            4,
            8,
            true,
            false,
            LimitKind::None,
            0x372d_272e_8dfc_face,
            0x799b_8af6_6f20_652a,
        ),
    ),
    case(
        "geometry/nurbs-isolation/rational-four-patch-v1/4/work-low-v2",
        SurfaceFixture::RationalFourPatch,
        GeometryRelation::Retained,
        policy(1, 403, 8, 1),
        expected(
            4,
            7,
            false,
            false,
            LimitKind::Work,
            0x15d7_570c_bf57_a446,
            0x1e80_ec1c_2a9a_7490,
        ),
    ),
    case(
        "geometry/nurbs-isolation/rational-four-patch-v1/4/separated-v1",
        SurfaceFixture::RationalFourPatch,
        GeometryRelation::Separated,
        policy(1, 404, 8, 1),
        expected(
            4,
            0,
            true,
            true,
            LimitKind::None,
            0xd1d4_86dd_bbba_946f,
            0x56ff_6611_015e_7f87,
        ),
    ),
    case(
        "geometry/nurbs-isolation/poly-single-v1/1/work-low-v1",
        SurfaceFixture::PolynomialSingle,
        GeometryRelation::Retained,
        policy(1, 28, 2, 1),
        expected(
            1,
            1,
            false,
            false,
            LimitKind::Work,
            0x4871_aa3f_190d_2d1e,
            0x2d72_5375_64b3_c2c9,
        ),
    ),
    case(
        "geometry/nurbs-isolation/poly-single-v1/1/candidate-low-v1",
        SurfaceFixture::PolynomialSingle,
        GeometryRelation::Retained,
        policy(1, 29, 1, 1),
        expected(
            1,
            1,
            false,
            false,
            LimitKind::Candidates,
            0x4871_aa3f_190d_2d1e,
            0x3bb4_be2b_bfd2_a94d,
        ),
    ),
    case(
        "geometry/nurbs-isolation/poly-subdivision-roundoff-v1/1/depth-1-v1",
        SurfaceFixture::SubdivisionRoundoff,
        GeometryRelation::Retained,
        policy(1, 29, 8, 1),
        expected(
            1,
            2,
            true,
            false,
            LimitKind::None,
            0xc0bc_f78d_8f6f_66bf,
            0x6aa8_538f_10e3_9e78,
        ),
    ),
];

#[derive(Clone, Copy)]
struct CasePolicy {
    requested_depth: u32,
    work_allowed: u64,
    candidates_allowed: u64,
    depth_allowed: u64,
}

const fn policy(
    requested_depth: u32,
    work_allowed: u64,
    candidates_allowed: u64,
    depth_allowed: u64,
) -> CasePolicy {
    CasePolicy {
        requested_depth,
        work_allowed,
        candidates_allowed,
        depth_allowed,
    }
}

#[derive(Clone, Copy)]
struct CaseExpected {
    patches: usize,
    candidates: usize,
    complete: bool,
    proven_empty: bool,
    limit: LimitKind,
    candidate_digest: u64,
    output_digest: u64,
}

const fn expected(
    patches: usize,
    candidates: usize,
    complete: bool,
    proven_empty: bool,
    limit: LimitKind,
    candidate_digest: u64,
    output_digest: u64,
) -> CaseExpected {
    CaseExpected {
        patches,
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
    fixture: SurfaceFixture,
    relation: GeometryRelation,
    policy: CasePolicy,
    expected: CaseExpected,
) -> NurbsIsolationCase {
    NurbsIsolationCase {
        path,
        fixture,
        relation,
        requested_depth: policy.requested_depth,
        work_allowed: policy.work_allowed,
        candidates_allowed: policy.candidates_allowed,
        depth_allowed: policy.depth_allowed,
        expected_patches: expected.patches,
        expected_candidates: expected.candidates,
        expected_complete: expected.complete,
        expected_proven_empty: expected.proven_empty,
        expected_limit: expected.limit,
        expected_candidate_digest: expected.candidate_digest,
        expected_output_digest: expected.output_digest,
    }
}

/// Immutable prepared Q4 input. Surface construction and BVH extraction are never measured.
pub struct NurbsIsolationFixture {
    surface: NurbsSurface,
    hierarchy: NurbsSurfaceBvh,
    plane: Plane,
    session: SessionPolicy,
    control_points: usize,
}

impl NurbsIsolationFixture {
    /// Run one contextual isolation, timing only the isolation method itself.
    pub fn measure_once(
        &self,
        case: NurbsIsolationCase,
    ) -> (core::time::Duration, NurbsIsolationEvidence) {
        let (elapsed, isolation, report) = self.isolate_once(case);
        (elapsed, self.evidence(case, &isolation, &report))
    }

    fn isolate_once(
        &self,
        case: NurbsIsolationCase,
    ) -> (
        core::time::Duration,
        ImplicitPatchIsolation,
        OperationReport,
    ) {
        let context = OperationContext::new(&self.session, Tolerances::default())
            .expect("Q4 operation policy is valid");
        let mut scope = OperationScope::new(&context);
        let started = std::time::Instant::now();
        let isolation = self
            .hierarchy
            .isolate_implicit_candidates_in_scope(
                &self.plane,
                0.0,
                case.requested_depth,
                &mut scope,
            )
            .expect("reviewed Q4 isolation must run");
        let elapsed = started.elapsed();
        let (_, report) = scope.finish(Ok(())).into_parts();
        (elapsed, isolation, report)
    }

    fn evidence(
        &self,
        case: NurbsIsolationCase,
        isolation: &ImplicitPatchIsolation,
        report: &OperationReport,
    ) -> NurbsIsolationEvidence {
        let work = usage(report, NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS);
        let candidates = usage(report, NURBS_IMPLICIT_ISOLATION_CANDIDATES);
        let depth = usage(report, NURBS_IMPLICIT_ISOLATION_DEPTH);
        let limits = isolation.limits();
        let limit = if limits.subdivision_work().is_some() {
            LimitKind::Work
        } else if limits.candidate_cells().is_some() {
            LimitKind::Candidates
        } else {
            LimitKind::None
        };
        let candidate_digest = candidate_digest(isolation);
        let conservative_cover = if isolation.is_proven_empty() {
            true
        } else if case.fixture == SurfaceFixture::SubdivisionRoundoff {
            exact_roundoff_contact_is_covered(isolation)
        } else {
            sampled_cover(&self.surface, isolation)
        };
        let max_candidate_depth = isolation
            .candidates()
            .iter()
            .map(|candidate| candidate.depth())
            .max()
            .unwrap_or(0);
        let (limit_attempted_consumed, limit_attempted_allowed) = report
            .limit_events()
            .first()
            .map_or((0, 0), |snapshot| (snapshot.consumed, snapshot.allowed));
        let mut evidence = NurbsIsolationEvidence {
            control_points: self.control_points,
            extracted_patches: self.hierarchy.patch_count(),
            bvh_nodes: self.hierarchy.node_count(),
            candidates: isolation.candidates().len(),
            requested_depth: isolation.requested_depth(),
            max_candidate_depth,
            complete: isolation.is_complete(),
            proven_empty: isolation.is_proven_empty(),
            indeterminate: !isolation.is_complete(),
            conservative_cover,
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
            candidate_digest,
            output_digest: 0,
        };
        evidence.output_digest = evidence.digest(case);
        evidence
    }
}

/// Stable Q4 counters and conservative proof evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NurbsIsolationEvidence {
    /// Source control-point count.
    pub control_points: usize,
    /// Extracted exact Bezier patch count.
    pub extracted_patches: usize,
    /// Deterministic BVH node count.
    pub bvh_nodes: usize,
    /// Retained conservative candidate-cell count.
    pub candidates: usize,
    /// Requested exact subdivision depth.
    pub requested_depth: u32,
    /// Maximum retained candidate depth.
    pub max_candidate_depth: u32,
    /// Whether isolation completed without a configured or numeric stop.
    pub complete: bool,
    /// Whether complete isolation certified an empty zero-set cover.
    pub proven_empty: bool,
    /// Whether a limit retained an incomplete, non-miss result.
    pub indeterminate: bool,
    /// Whether deterministic source samples remain inside the returned cover.
    pub conservative_cover: bool,
    /// Structured stop category.
    pub limit: LimitKind,
    /// Structured limit-event count.
    pub limit_events: usize,
    /// Attempted usage that crossed the active limit, or zero when complete.
    pub limit_attempted_consumed: u64,
    /// Allowance crossed by the attempted usage, or zero when complete.
    pub limit_attempted_allowed: u64,
    /// Committed subdivision work.
    pub work_consumed: u64,
    /// Configured subdivision-work allowance.
    pub work_allowed: u64,
    /// Committed candidate-cover high water.
    pub candidate_high_water: u64,
    /// Configured candidate allowance.
    pub candidates_allowed: u64,
    /// Committed subdivision-depth high water.
    pub depth_high_water: u64,
    /// Configured depth allowance.
    pub depth_allowed: u64,
    /// Stable digest of exact candidate geometry and source ranges.
    pub candidate_digest: u64,
    /// Stable digest of all counters and proof evidence.
    pub output_digest: u64,
}

impl NurbsIsolationEvidence {
    fn digest(self, case: NurbsIsolationCase) -> u64 {
        let mut digest = StableHasher::new();
        digest.tag(0x91);
        digest.count(self.control_points);
        digest.count(self.extracted_patches);
        digest.count(self.bvh_nodes);
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
        digest.u64(self.candidate_digest);
        digest.tag(match case.relation {
            GeometryRelation::Retained => 0,
            GeometryRelation::Separated => 1,
        });
        digest.tag(match case.fixture {
            SurfaceFixture::PolynomialSingle => 0,
            SurfaceFixture::RationalSingle => 1,
            SurfaceFixture::RationalFourPatch => 2,
            SurfaceFixture::SubdivisionRoundoff => 3,
        });
        digest.finish()
    }
}

/// Construct one immutable surface, hierarchy, implicit plane, and policy.
pub fn fixture(case: NurbsIsolationCase) -> NurbsIsolationFixture {
    let surface = match case.fixture {
        SurfaceFixture::PolynomialSingle => single_patch(false),
        SurfaceFixture::RationalSingle => single_patch(true),
        SurfaceFixture::RationalFourPatch => four_patch(),
        SurfaceFixture::SubdivisionRoundoff => subdivision_roundoff_patch(),
    };
    let hierarchy = NurbsSurfaceBvh::build(&surface).expect("valid Q4 hierarchy");
    let plane = match (case.fixture, case.relation) {
        (SurfaceFixture::SubdivisionRoundoff, GeometryRelation::Retained) => Plane::new(
            Frame::from_z(
                Point3::new(0.0, 0.0, ROUNDOFF_CONTACT_Z),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .expect("valid roundoff-contact plane"),
        ),
        (_, GeometryRelation::Retained) => Plane::new(Frame::world()),
        (_, GeometryRelation::Separated) => Plane::new(
            Frame::from_z(Point3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 1.0))
                .expect("valid separated plane"),
        ),
    };
    let budget = BudgetPlan::new([
        LimitSpec::new(
            NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            case.work_allowed,
        ),
        LimitSpec::new(
            NURBS_IMPLICIT_ISOLATION_CANDIDATES,
            ResourceKind::Items,
            AccountingMode::HighWater,
            case.candidates_allowed,
        ),
        LimitSpec::new(
            NURBS_IMPLICIT_ISOLATION_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            case.depth_allowed,
        ),
    ])
    .expect("valid Q4 budget");
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        budget,
        PolicyVersion::V1,
    );
    let control_points = surface.points().len();
    NurbsIsolationFixture {
        surface,
        hierarchy,
        plane,
        session,
        control_points,
    }
}

/// Verify exact reviewed evidence and the incomplete-proof contract.
pub fn verify(case: NurbsIsolationCase, evidence: NurbsIsolationEvidence) {
    assert_eq!(evidence.extracted_patches, case.expected_patches);
    assert_eq!(evidence.candidates, case.expected_candidates);
    assert_eq!(evidence.complete, case.expected_complete);
    assert_eq!(evidence.proven_empty, case.expected_proven_empty);
    assert_eq!(evidence.limit, case.expected_limit);
    assert!(evidence.conservative_cover);
    if evidence.limit != LimitKind::None {
        assert!(evidence.indeterminate);
        assert!(!evidence.complete);
        assert!(!evidence.proven_empty);
        assert!(evidence.candidates > 0);
        assert_eq!(evidence.limit_events, 1);
        assert_eq!(
            evidence.limit_attempted_consumed,
            evidence.limit_attempted_allowed + 1
        );
    } else {
        assert_eq!(evidence.limit_events, 0);
        assert_eq!(evidence.limit_attempted_consumed, 0);
        assert_eq!(evidence.limit_attempted_allowed, 0);
    }
    assert_ne!(case.expected_candidate_digest, 0);
    assert_ne!(case.expected_output_digest, 0);
    assert_eq!(evidence.candidate_digest, case.expected_candidate_digest);
    assert_eq!(evidence.output_digest, case.expected_output_digest);
}

fn usage(report: &OperationReport, stage: StageId) -> LimitSnapshot {
    *report
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == stage)
        .expect("Q4 stage is configured")
}

fn single_patch(rational: bool) -> NurbsSurface {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    NurbsSurface::new(
        1,
        1,
        knots.clone(),
        knots,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        rational.then_some(vec![0.75, 1.0, 1.25, 0.875]),
    )
    .expect("valid single-patch fixture")
}

fn four_patch() -> NurbsSurface {
    let knots = vec![0.0, 0.0, 0.5, 1.0, 1.0];
    let mut points = Vec::with_capacity(9);
    let mut weights = Vec::with_capacity(9);
    for u in 0..3 {
        for v in 0..3 {
            points.push(Point3::new(f64::from(u), f64::from(v), 0.0));
            weights.push(0.75 + 0.125 * f64::from((u * 3 + v) % 5));
        }
    }
    NurbsSurface::new(1, 1, knots.clone(), knots, points, Some(weights))
        .expect("valid four-patch fixture")
}

fn subdivision_roundoff_patch() -> NurbsSurface {
    let xs = [-1.0, -1.0 / 3.0, 1.0 / 3.0, 1.0];
    let zs = [
        9_007_199_254_740_360.0,
        9_007_199_254_740_978.0,
        9_007_199_254_741_648.0,
        9_007_199_254_739_690.0,
    ];
    let mut points = Vec::with_capacity(8);
    for (x, z) in xs.into_iter().zip(zs) {
        points.push(Point3::new(x, -1.0, z));
        points.push(Point3::new(x, 1.0, z));
    }
    NurbsSurface::new(
        3,
        1,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        points,
        None,
    )
    .expect("valid subdivision-roundoff surface")
}

fn exact_roundoff_contact_is_covered(isolation: &ImplicitPatchIsolation) -> bool {
    let contact = Point3::new(0.0, 0.0, ROUNDOFF_CONTACT_Z);
    isolation.candidates().iter().any(|candidate| {
        let range = candidate.parameter_range();
        range[0].contains(0.5) && range[1].contains(0.5) && candidate.bounds().contains(contact)
    })
}

fn sampled_cover(surface: &NurbsSurface, isolation: &ImplicitPatchIsolation) -> bool {
    let range = surface.param_range();
    (0..=4).all(|u| {
        (0..=4).all(|v| {
            let uv = [
                range[0].lerp(f64::from(u) / 4.0),
                range[1].lerp(f64::from(v) / 4.0),
            ];
            let point = surface.eval(uv);
            isolation.candidates().iter().any(|candidate| {
                let candidate_range = candidate.parameter_range();
                candidate_range[0].contains(uv[0])
                    && candidate_range[1].contains(uv[1])
                    && candidate.bounds().inflated(1.0e-12).contains(point)
            })
        })
    })
}

#[cfg(test)]
fn cover_contains_cover(coarse: &ImplicitPatchIsolation, refined: &ImplicitPatchIsolation) -> bool {
    refined.candidates().iter().all(|fine| {
        coarse.candidates().iter().any(|cover| {
            let coarse_range = cover.parameter_range();
            let fine_range = fine.parameter_range();
            cover.source_patch() == fine.source_patch()
                && coarse_range[0].contains(fine_range[0].lo)
                && coarse_range[0].contains(fine_range[0].hi)
                && coarse_range[1].contains(fine_range[1].lo)
                && coarse_range[1].contains(fine_range[1].hi)
                && cover.bounds().inflated(1.0e-12).contains(fine.bounds().min)
                && cover.bounds().inflated(1.0e-12).contains(fine.bounds().max)
        })
    })
}

fn candidate_digest(isolation: &ImplicitPatchIsolation) -> u64 {
    let mut digest = StableHasher::new();
    digest.tag(0x90);
    digest.count(isolation.candidates().len());
    for candidate in isolation.candidates() {
        digest.count(candidate.source_patch());
        digest.u64(u64::from(candidate.depth()));
        for range in candidate.parameter_range() {
            digest.f64(range.lo);
            digest.f64(range.hi);
        }
        let bounds = candidate.bounds();
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
        let patch = candidate.patch();
        digest.count(patch.degree_u());
        digest.count(patch.degree_v());
        digest.count(patch.points().len());
        for point in patch.points() {
            digest.f64(point.x);
            digest.f64(point.y);
            digest.f64(point.z);
        }
        if let Some(weights) = patch.weights() {
            digest.tag(1);
            digest.count(weights.len());
            for &weight in weights {
                digest.f64(weight);
            }
        } else {
            digest.tag(0);
        }
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
    use super::*;
    use std::collections::BTreeSet;

    #[test]
    fn registry_contains_exactly_eight_unique_canonical_cases() {
        let unique: BTreeSet<_> = CASES.iter().map(|case| case.path).collect();
        assert_eq!(unique.len(), CASES.len());
        for case in CASES {
            crate::validate_case_path(case.path).unwrap();
        }
    }

    #[test]
    fn json_registry_matches_every_rust_case_and_counter() {
        let manifest: serde_json::Value =
            serde_json::from_str(include_str!("../cases.json")).unwrap();
        let entries = manifest["cases"].as_array().unwrap();
        let entries: Vec<_> = entries
            .iter()
            .filter(|entry| entry["benchmark_target"] == "nurbs_isolation")
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
                entry["size_parameters"]["elements"].as_u64(),
                Some(case.expected_patches as u64)
            );
            assert_eq!(entry["tolerances"]["implicit_margin"].as_f64(), Some(0.0));
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
            let counters = &entry["expected_result_counters"];
            for (field, actual) in [
                ("control_points", evidence.control_points as u64),
                ("extracted_patches", evidence.extracted_patches as u64),
                ("bvh_nodes", evidence.bvh_nodes as u64),
                ("candidates", evidence.candidates as u64),
                ("requested_depth", u64::from(evidence.requested_depth)),
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
                ("work_allowed", evidence.work_allowed),
                ("candidate_high_water", evidence.candidate_high_water),
                ("candidates_allowed", evidence.candidates_allowed),
                ("depth_high_water", evidence.depth_high_water),
                ("depth_allowed", evidence.depth_allowed),
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
                counters["candidate_digest"].as_str(),
                Some(format!("{:016x}", evidence.candidate_digest).as_str())
            );
            assert_eq!(
                counters["output_digest"].as_str(),
                Some(format!("{:016x}", evidence.output_digest).as_str())
            );
        }
    }

    #[test]
    fn every_case_is_repeatable_and_reports_reviewed_evidence() {
        for case in CASES {
            let fixture = fixture(case);
            let first = fixture.measure_once(case).1;
            let second = fixture.measure_once(case).1;
            assert_eq!(first, second, "repeatability drift for {}", case.path);
            verify(case, first);
        }
    }

    #[test]
    fn budget_exhaustion_is_the_smallest_crossing_and_retains_the_exact_cover() {
        for (exact_case, case) in [
            (CASES[2], CASES[3]),
            (CASES[0], CASES[5]),
            (CASES[0], CASES[6]),
        ] {
            let exact_fixture = fixture(exact_case);
            let (_, exact, exact_report) = exact_fixture.isolate_once(exact_case);
            assert!(exact.is_complete());
            assert!(exact_report.limit_events().is_empty());
            let limited_fixture = fixture(case);
            assert_eq!(limited_fixture.surface, exact_fixture.surface);
            assert_eq!(limited_fixture.hierarchy, exact_fixture.hierarchy);
            assert_eq!(limited_fixture.plane, exact_fixture.plane);
            let (_, limited, report) = limited_fixture.isolate_once(case);
            let (_, repeated, repeated_report) = limited_fixture.isolate_once(case);
            assert_eq!(limited, repeated);
            assert_eq!(report, repeated_report);
            let evidence = limited_fixture.evidence(case, &limited, &report);
            assert!(evidence.indeterminate);
            assert!(evidence.conservative_cover);
            assert!(!evidence.complete);
            assert!(!evidence.proven_empty);
            assert!(evidence.candidates > 0);
            assert!(evidence.candidates < exact.candidates().len());
            assert!(cover_contains_cover(&limited, &exact));
            assert_eq!(report.limit_events().len(), 1);
            let crossing = report.limit_events()[0];
            assert_eq!(crossing.consumed, crossing.allowed + 1);
            match case.expected_limit {
                LimitKind::Work => {
                    assert_eq!(crossing.stage, NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS);
                    assert_eq!(crossing.resource, ResourceKind::Work);
                }
                LimitKind::Candidates => {
                    assert_eq!(crossing.stage, NURBS_IMPLICIT_ISOLATION_CANDIDATES);
                    assert_eq!(crossing.resource, ResourceKind::Items);
                }
                LimitKind::None => panic!("reviewed boundary case must be limited"),
            }
        }
    }
}
