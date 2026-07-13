//! Bit-pattern, completion, overlap, and validation contracts for conic/NURBS.

use kcore::error::Error;
use kcore::proof::Completion;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse};
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use kops::intersect::{
    ContactKind, CurveCurveIntersections, ParamOrientation, intersect_bounded_circle_nurbs,
    intersect_bounded_ellipse_nurbs,
};

fn line(a: Point3, b: Point3) -> NurbsCurve {
    NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![a, b], None).unwrap()
}

fn quarter_nurbs(x_radius: f64, reversed: bool) -> NurbsCurve {
    let mut points = vec![
        Point3::new(x_radius, 0.0, 0.0),
        Point3::new(x_radius, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    if reversed {
        points.reverse();
    }
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points,
        Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
    )
    .unwrap()
}

fn kind_bits(kind: ContactKind) -> u64 {
    match kind {
        ContactKind::Transverse => 0,
        ContactKind::Tangent => 1,
        ContactKind::Singular => 2,
        _ => 3,
    }
}

fn orientation_bits(orientation: ParamOrientation) -> u64 {
    match orientation {
        ParamOrientation::Same => 0,
        ParamOrientation::Reversed => 1,
    }
}

fn signature(result: &CurveCurveIntersections) -> Vec<u64> {
    let mut bits = vec![result.points.len() as u64, result.overlaps.len() as u64];
    for point in &result.points {
        bits.extend([
            point.point.x.to_bits(),
            point.point.y.to_bits(),
            point.point.z.to_bits(),
            point.t_a.to_bits(),
            point.t_b.to_bits(),
            point.residual.to_bits(),
            kind_bits(point.kind),
        ]);
    }
    for overlap in &result.overlaps {
        bits.extend([
            overlap.a.lo.to_bits(),
            overlap.a.hi.to_bits(),
            overlap.b.lo.to_bits(),
            overlap.b.hi.to_bits(),
            orientation_bits(overlap.orientation),
        ]);
    }
    bits
}

const CIRCLE_GOLDENS: &[&[u64]] = &[
    &[
        2,
        0,
        4607182418800077910,
        0,
        0,
        0,
        4604930618986392662,
        4448864564150272000,
        0,
        13830554455654962559,
        4364452196894661639,
        0,
        4614256656552045848,
        4598175219544599044,
        4455375013016509987,
        0,
    ],
    &[
        1,
        0,
        13590505696510988797,
        4607182418800017408,
        0,
        4609753056924675353,
        4602678819172646911,
        4369385459469898237,
        1,
    ],
    &[0, 0],
    &[0, 1, 0, 4609753056924675352, 0, 4607182418800017408, 0],
    &[0, 1, 0, 4605249457297304856, 0, 4602678819172646912, 0],
    &[
        0,
        1,
        0,
        4605249457297304856,
        4602678819172646912,
        4607182418800017408,
        1,
    ],
];

const ELLIPSE_GOLDENS: &[&[u64]] = &[
    &[
        2,
        0,
        4611686018427315064,
        0,
        0,
        0,
        4605681218924178683,
        4450058358900129792,
        0,
        13835058055282203301,
        4364452196894661639,
        0,
        4614256656552045848,
        4595172819793484942,
        4450493902943715561,
        0,
    ],
    &[
        1,
        0,
        4483351868489697677,
        4607182418800017408,
        0,
        4609753056912058094,
        4602678819223115944,
        4368955796537475072,
        1,
    ],
    &[0, 0],
    &[0, 1, 0, 4609753056924675352, 0, 4607182418800017408, 0],
    &[0, 1, 0, 4605249457297304856, 0, 4602678819172646912, 0],
    &[
        0,
        1,
        0,
        4605249457297304856,
        4602678819172646912,
        4607182418800017408,
        1,
    ],
];

fn circle_cases() -> Vec<CurveCurveIntersections> {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let crossing = line(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let tangent = line(Point3::new(-0.5, 1.0, 0.0), Point3::new(0.5, 1.0, 0.0));
    let quarter = quarter_nurbs(1.0, false);
    let reversed = quarter_nurbs(1.0, true);
    let solve = |curve: &NurbsCurve, a, b| {
        intersect_bounded_circle_nurbs(&circle, a, curve, b, Tolerances::default()).unwrap()
    };
    vec![
        solve(&crossing, circle.param_range(), crossing.param_range()),
        solve(&tangent, circle.param_range(), tangent.param_range()),
        solve(&crossing, circle.param_range(), ParamRange::new(0.0, 0.2)),
        solve(&quarter, circle.param_range(), quarter.param_range()),
        solve(
            &quarter,
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_4),
            quarter.param_range(),
        ),
        solve(
            &reversed,
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_4),
            reversed.param_range(),
        ),
    ]
}

fn ellipse_cases() -> Vec<CurveCurveIntersections> {
    let ellipse = Ellipse::new(Frame::world(), 2.0, 1.0).unwrap();
    let crossing = line(Point3::new(-3.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0));
    let tangent = line(Point3::new(-0.5, 1.0, 0.0), Point3::new(0.5, 1.0, 0.0));
    let quarter = quarter_nurbs(2.0, false);
    let reversed = quarter_nurbs(2.0, true);
    let solve = |curve: &NurbsCurve, a, b| {
        intersect_bounded_ellipse_nurbs(&ellipse, a, curve, b, Tolerances::default()).unwrap()
    };
    vec![
        solve(&crossing, ellipse.param_range(), crossing.param_range()),
        solve(&tangent, ellipse.param_range(), tangent.param_range()),
        solve(&crossing, ellipse.param_range(), ParamRange::new(0.0, 0.1)),
        solve(&quarter, ellipse.param_range(), quarter.param_range()),
        solve(
            &quarter,
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_4),
            quarter.param_range(),
        ),
        solve(
            &reversed,
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_4),
            reversed.param_range(),
        ),
    ]
}

fn assert_streams(results: &[CurveCurveIntersections], goldens: &[&[u64]], reason: &'static str) {
    assert_eq!(results.len(), goldens.len());
    for (result, golden) in results.iter().zip(goldens) {
        assert_eq!(signature(result), *golden);
        assert_eq!(result.completion(), Completion::Indeterminate { reason });
    }
}

#[test]
fn conic_nurbs_preserves_pre_consolidation_streams_and_completion() {
    let circle = circle_cases();
    let ellipse = ellipse_cases();
    assert_streams(
        &circle,
        CIRCLE_GOLDENS,
        "fixed-grid circle/NURBS candidate discovery does not prove complete coverage",
    );
    assert_streams(
        &ellipse,
        ELLIPSE_GOLDENS,
        "fixed-grid ellipse/NURBS candidate discovery does not prove complete coverage",
    );
}

fn invalid_reason<T>(result: Result<T, Error>) -> &'static str {
    match result {
        Err(Error::InvalidGeometry { reason }) => reason,
        Err(error) => panic!("unexpected error: {error:?}"),
        Ok(_) => panic!("expected invalid geometry"),
    }
}

fn unclamped_line() -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 1.0, 2.0, 3.0],
        vec![Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap()
}

#[test]
fn conic_nurbs_preserves_exact_validation_diagnostics() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let ellipse = Ellipse::new(Frame::world(), 2.0, 1.0).unwrap();
    let curve = line(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let unclamped = unclamped_line();
    let tolerances = Tolerances::default();

    let circle_solve = |conic_range, curve: &NurbsCurve, curve_range| {
        intersect_bounded_circle_nurbs(&circle, conic_range, curve, curve_range, tolerances)
    };
    assert_eq!(
        invalid_reason(circle_solve(
            ParamRange::unbounded(),
            &curve,
            curve.param_range(),
        )),
        "circle/nurbs intersection requires finite non-reversed ranges"
    );
    assert_eq!(
        invalid_reason(circle_solve(
            ParamRange::new(0.0, core::f64::consts::TAU + 1e-3),
            &curve,
            curve.param_range(),
        )),
        "bounded circle range cannot span more than one period"
    );
    assert_eq!(
        invalid_reason(circle_solve(
            circle.param_range(),
            &unclamped,
            unclamped.param_range(),
        )),
        "circle/nurbs intersection requires a clamped NURBS curve"
    );
    assert_eq!(
        invalid_reason(circle_solve(
            circle.param_range(),
            &curve,
            ParamRange::new(-0.1, 1.0),
        )),
        "circle/nurbs intersection curve range must lie within the NURBS domain"
    );

    let ellipse_solve = |conic_range, curve: &NurbsCurve, curve_range| {
        intersect_bounded_ellipse_nurbs(&ellipse, conic_range, curve, curve_range, tolerances)
    };
    assert_eq!(
        invalid_reason(ellipse_solve(
            ParamRange::unbounded(),
            &curve,
            curve.param_range(),
        )),
        "ellipse/nurbs intersection requires finite non-reversed ranges"
    );
    assert_eq!(
        invalid_reason(ellipse_solve(
            ParamRange::new(0.0, core::f64::consts::TAU + 1e-3),
            &curve,
            curve.param_range(),
        )),
        "bounded ellipse range cannot span more than one period"
    );
    assert_eq!(
        invalid_reason(ellipse_solve(
            ellipse.param_range(),
            &unclamped,
            unclamped.param_range(),
        )),
        "ellipse/nurbs intersection requires a clamped NURBS curve"
    );
    assert_eq!(
        invalid_reason(ellipse_solve(
            ellipse.param_range(),
            &curve,
            ParamRange::new(-0.1, 1.0),
        )),
        "ellipse/nurbs intersection curve range must lie within the NURBS domain"
    );
}
