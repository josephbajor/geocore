//! Bit-pattern, completion, accounting, overlap, and validation contracts for
//! the analytic conic/conic family.

use kcore::error::Error;
use kcore::operation::{OperationContext, ResourceKind, SessionPolicy};
use kcore::proof::Completion;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, CurveCurveIntersections, ParamOrientation, intersect_bounded_circle_ellipse,
    intersect_bounded_circles, intersect_bounded_ellipses, intersect_bounded_ellipses_with_context,
};

const OFFSET: u64 = 0xcbf29ce484222325;
const PRIME: u64 = 0x100000001b3;

fn mix(mut state: u64, word: u64) -> u64 {
    for byte in word.to_le_bytes() {
        state ^= u64::from(byte);
        state = state.wrapping_mul(PRIME);
    }
    state
}

fn mix_str(mut state: u64, value: &str) -> u64 {
    for &byte in value.as_bytes() {
        state ^= u64::from(byte);
        state = state.wrapping_mul(PRIME);
    }
    state
}

fn result_digest(result: &CurveCurveIntersections) -> u64 {
    let mut state = OFFSET;
    state = mix(state, result.points.len() as u64);
    state = mix(state, result.overlaps.len() as u64);
    state = mix(
        state,
        match result.completion() {
            Completion::Complete => 0,
            Completion::Indeterminate { .. } => 1,
            _ => 2,
        },
    );
    for point in &result.points {
        for word in [
            point.point.x.to_bits(),
            point.point.y.to_bits(),
            point.point.z.to_bits(),
            point.t_a.to_bits(),
            point.t_b.to_bits(),
            point.residual.to_bits(),
            match point.kind {
                ContactKind::Transverse => 0,
                ContactKind::Tangent => 1,
                ContactKind::Singular => 2,
                _ => 3,
            },
        ] {
            state = mix(state, word);
        }
    }
    for overlap in &result.overlaps {
        for word in [
            overlap.a.lo.to_bits(),
            overlap.a.hi.to_bits(),
            overlap.b.lo.to_bits(),
            overlap.b.hi.to_bits(),
            match overlap.orientation {
                ParamOrientation::Same => 0,
                ParamOrientation::Reversed => 1,
            },
        ] {
            state = mix(state, word);
        }
    }
    state
}

fn circle(center: [f64; 3], normal: [f64; 3], x_hint: [f64; 3], radius: f64) -> Circle {
    Circle::new(
        Frame::new(
            Point3::from_array(center),
            Vec3::from_array(normal),
            Vec3::from_array(x_hint),
        )
        .unwrap(),
        radius,
    )
    .unwrap()
}

fn ellipse(
    center: [f64; 3],
    normal: [f64; 3],
    x_hint: [f64; 3],
    major: f64,
    minor: f64,
) -> Ellipse {
    Ellipse::new(
        Frame::new(
            Point3::from_array(center),
            Vec3::from_array(normal),
            Vec3::from_array(x_hint),
        )
        .unwrap(),
        major,
        minor,
    )
    .unwrap()
}

fn circle_cases() -> Vec<CurveCurveIntersections> {
    let a = Circle::new(Frame::world(), 1.0).unwrap();
    let full = a.param_range();
    let secant = circle([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 1.0);
    let tangent = circle([2.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 1.0);
    let miss = circle([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 0.5);
    let skew = circle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 1.0);
    let reversed = circle([0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], 1.0);
    let solve = |b: &Circle, range_a, range_b| {
        intersect_bounded_circles(&a, range_a, b, range_b, Tolerances::default()).unwrap()
    };
    vec![
        solve(&secant, full, full),
        solve(&tangent, full, full),
        solve(&miss, full, full),
        solve(&skew, full, full),
        solve(
            &secant,
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, core::f64::consts::PI),
        ),
        solve(&a, ParamRange::new(0.25, 1.25), ParamRange::new(0.75, 1.75)),
        solve(
            &reversed,
            ParamRange::new(0.0, 1.0),
            ParamRange::new(core::f64::consts::TAU - 1.0, core::f64::consts::TAU),
        ),
        solve(&a, ParamRange::new(0.0, 1.0), ParamRange::new(1.0, 2.0)),
    ]
}

fn circle_ellipse_cases() -> Vec<CurveCurveIntersections> {
    let circle2 = Circle::new(Frame::world(), 2.0).unwrap();
    let ellipse31 = Ellipse::new(Frame::world(), 3.0, 1.0).unwrap();
    let tangent = circle([0.0, 2.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 1.0);
    let offset = circle([0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0);
    let unit = Circle::new(Frame::world(), 1.0).unwrap();
    let skew = ellipse([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 1.0, 0.5);
    let unit_ellipse = Ellipse::new(Frame::world(), 1.0, 1.0).unwrap();
    let solve = |a: &Circle, range_a, b: &Ellipse, range_b| {
        intersect_bounded_circle_ellipse(a, range_a, b, range_b, Tolerances::default()).unwrap()
    };
    vec![
        solve(
            &circle2,
            circle2.param_range(),
            &ellipse31,
            ellipse31.param_range(),
        ),
        solve(
            &tangent,
            tangent.param_range(),
            &ellipse31,
            ellipse31.param_range(),
        ),
        solve(
            &offset,
            offset.param_range(),
            &ellipse31,
            ellipse31.param_range(),
        ),
        solve(&unit, unit.param_range(), &skew, skew.param_range()),
        solve(
            &circle2,
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            &ellipse31,
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
        ),
        solve(
            &unit,
            ParamRange::new(0.25, 1.25),
            &unit_ellipse,
            ParamRange::new(0.75, 1.75),
        ),
    ]
}

fn ellipse_cases() -> Vec<CurveCurveIntersections> {
    let a = Ellipse::new(Frame::world(), 3.0, 1.0).unwrap();
    let b = Ellipse::new(Frame::world(), 2.0, 1.5).unwrap();
    let tangent = ellipse([0.0, 2.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 3.0, 1.0);
    let miss = ellipse([0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0, 1.5);
    let skew_a = Ellipse::new(Frame::world(), 2.0, 1.0).unwrap();
    let skew_b = ellipse([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 1.0, 0.5);
    let reversed = ellipse([0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], 3.0, 1.0);
    let circle_a = Ellipse::new(Frame::world(), 1.0, 1.0).unwrap();
    let circle_b = Ellipse::new(Frame::world(), 1.0, 1.0).unwrap();
    let solve = |a: &Ellipse, range_a, b: &Ellipse, range_b| {
        intersect_bounded_ellipses(a, range_a, b, range_b, Tolerances::default()).unwrap()
    };
    vec![
        solve(&a, a.param_range(), &b, b.param_range()),
        solve(&a, a.param_range(), &tangent, tangent.param_range()),
        solve(&a, a.param_range(), &miss, miss.param_range()),
        solve(&skew_a, skew_a.param_range(), &skew_b, skew_b.param_range()),
        solve(
            &a,
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            &b,
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
        ),
        solve(
            &a,
            ParamRange::new(0.25, 1.25),
            &a,
            ParamRange::new(0.75, 1.75),
        ),
        solve(
            &a,
            ParamRange::new(0.0, 1.0),
            &reversed,
            ParamRange::new(core::f64::consts::TAU - 1.0, core::f64::consts::TAU),
        ),
        solve(
            &circle_a,
            ParamRange::new(0.25, 1.25),
            &circle_b,
            ParamRange::new(0.75, 1.75),
        ),
    ]
}

fn report_digest(report: &kcore::operation::OperationReport) -> u64 {
    let mut state = OFFSET;
    for usage in report.usage() {
        state = mix_str(state, usage.stage.as_str());
        state = mix(
            state,
            match usage.resource {
                ResourceKind::Work => 0,
                ResourceKind::Items => 1,
                ResourceKind::Bytes => 2,
                ResourceKind::Depth => 3,
                _ => 4,
            },
        );
        state = mix(state, usage.consumed);
        state = mix(state, usage.allowed);
    }
    state
}

// Captured from the specialized pre-consolidation solvers. Debug and release
// produced identical result and accounting streams.
const CIRCLE_GOLDENS: &[u64] = &[
    4_943_451_449_908_787_037,
    18_164_137_802_848_577_442,
    9_354_609_568_656_401_157,
    14_010_683_829_175_761_732,
    8_947_051_864_999_010_726,
    11_131_968_207_111_907_876,
    3_350_399_014_265_679_972,
    13_731_763_888_342_450_363,
];
const CIRCLE_ELLIPSE_GOLDENS: &[u64] = &[
    5_771_081_056_051_203_534,
    3_152_776_747_965_090_788,
    9_354_609_568_656_401_157,
    8_064_791_522_309_177_226,
    7_512_290_928_093_042_220,
    11_131_968_207_111_907_876,
];
const ELLIPSE_GOLDENS: &[u64] = &[
    17_826_041_413_054_412_823,
    10_274_346_402_497_968_299,
    9_354_609_568_656_401_157,
    12_657_396_070_346_105_917,
    11_545_672_738_980_141_029,
    11_131_968_207_111_907_876,
    3_350_399_014_265_679_972,
    11_131_968_207_111_907_876,
];
const ELLIPSE_REPORT_GOLDEN: u64 = 15_903_753_925_855_650_032;

#[test]
fn conic_pair_streams_and_context_accounting_remain_bit_exact() {
    let circle = circle_cases().iter().map(result_digest).collect::<Vec<_>>();
    let circle_ellipse = circle_ellipse_cases()
        .iter()
        .map(result_digest)
        .collect::<Vec<_>>();
    let ellipse = ellipse_cases()
        .iter()
        .map(result_digest)
        .collect::<Vec<_>>();
    assert_eq!(circle, CIRCLE_GOLDENS);
    assert_eq!(circle_ellipse, CIRCLE_ELLIPSE_GOLDENS);
    assert_eq!(ellipse, ELLIPSE_GOLDENS);

    let a = Ellipse::new(Frame::world(), 3.0, 1.0).unwrap();
    let b = Ellipse::new(Frame::world(), 2.0, 1.5).unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let contextual =
        intersect_bounded_ellipses_with_context(&a, a.param_range(), &b, b.param_range(), &context);
    assert_eq!(report_digest(contextual.report()), ELLIPSE_REPORT_GOLDEN);
    assert_eq!(
        contextual.result().unwrap().completion(),
        Completion::Complete
    );
}

fn invalid_reason<T>(result: Result<T, Error>) -> &'static str {
    match result {
        Err(Error::InvalidGeometry { reason }) => reason,
        Err(error) => panic!("unexpected error: {error:?}"),
        Ok(_) => panic!("expected invalid geometry"),
    }
}

#[test]
fn conic_pair_validation_diagnostics_are_exact() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let ellipse = Ellipse::new(Frame::world(), 2.0, 1.0).unwrap();
    let tolerances = Tolerances::default();
    assert_eq!(
        invalid_reason(intersect_bounded_circles(
            &circle,
            ParamRange::unbounded(),
            &circle,
            circle.param_range(),
            tolerances,
        )),
        "circle/circle intersection requires finite non-reversed ranges"
    );
    assert_eq!(
        invalid_reason(intersect_bounded_circles(
            &circle,
            ParamRange::new(0.0, 2.0 * core::f64::consts::TAU),
            &circle,
            circle.param_range(),
            tolerances,
        )),
        "bounded circle ranges cannot span more than one period"
    );
    assert_eq!(
        invalid_reason(intersect_bounded_circle_ellipse(
            &circle,
            ParamRange::unbounded(),
            &ellipse,
            ellipse.param_range(),
            tolerances,
        )),
        "circle/ellipse intersection requires finite non-reversed ranges"
    );
    assert_eq!(
        invalid_reason(intersect_bounded_circle_ellipse(
            &circle,
            circle.param_range(),
            &ellipse,
            ParamRange::new(0.0, 2.0 * core::f64::consts::TAU),
            tolerances,
        )),
        "bounded circle and ellipse ranges cannot span more than one period"
    );
    assert_eq!(
        invalid_reason(intersect_bounded_ellipses(
            &ellipse,
            ParamRange::unbounded(),
            &ellipse,
            ellipse.param_range(),
            tolerances,
        )),
        "ellipse/ellipse intersection requires finite non-reversed ranges"
    );
    assert_eq!(
        invalid_reason(intersect_bounded_ellipses(
            &ellipse,
            ParamRange::new(0.0, 2.0 * core::f64::consts::TAU),
            &ellipse,
            ellipse.param_range(),
            tolerances,
        )),
        "bounded ellipse ranges cannot span more than one period"
    );
}
