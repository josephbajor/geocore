//! Bit-pattern and completion contracts for the shared conic/cone driver.

use kcore::error::Error;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Cone;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, CurveSurfaceIntersections, intersect_bounded_circle_cone,
    intersect_bounded_ellipse_cone,
};

fn horizontal_frame(origin: [f64; 3]) -> Frame {
    Frame::new(
        Point3::from_array(origin),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

fn cone() -> Cone {
    Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap()
}

fn cone_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-1.0, 1.0),
    ]
}

fn clipped_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::PI),
        ParamRange::new(-1.0, 1.0),
    ]
}

fn kind_bits(kind: ContactKind) -> u64 {
    match kind {
        ContactKind::Transverse => 0,
        ContactKind::Tangent => 1,
        ContactKind::Singular => 2,
        _ => 3,
    }
}

fn signature(result: &CurveSurfaceIntersections) -> Vec<u64> {
    let mut bits = vec![result.points.len() as u64, result.overlaps.len() as u64];
    for point in &result.points {
        bits.extend([
            point.point.x.to_bits(),
            point.point.y.to_bits(),
            point.point.z.to_bits(),
            point.t_curve.to_bits(),
            point.uv_surface[0].to_bits(),
            point.uv_surface[1].to_bits(),
            point.residual.to_bits(),
            kind_bits(point.kind),
        ]);
    }
    for overlap in &result.overlaps {
        bits.extend([
            overlap.curve.lo.to_bits(),
            overlap.curve.hi.to_bits(),
            overlap.uv_start[0].to_bits(),
            overlap.uv_start[1].to_bits(),
            overlap.uv_end[0].to_bits(),
            overlap.uv_end[1].to_bits(),
        ]);
    }
    bits
}

fn circle_cases() -> Vec<CurveSurfaceIntersections> {
    let cone = cone();
    let secant = Circle::new(horizontal_frame([0.5, 0.0, 0.0]), 1.0).unwrap();
    let tangent = Circle::new(horizontal_frame([1.5, 0.0, 0.0]), 0.5).unwrap();
    let contained = Circle::new(Frame::world(), 1.0).unwrap();
    vec![
        intersect_bounded_circle_cone(
            &secant,
            secant.param_range(),
            &cone,
            cone_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_cone(
            &tangent,
            tangent.param_range(),
            &cone,
            cone_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_cone(
            &contained,
            contained.param_range(),
            &cone,
            cone_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_cone(
            &contained,
            contained.param_range(),
            &cone,
            clipped_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_cone(
            &contained,
            contained.param_range(),
            &cone,
            [
                ParamRange::new(0.0, core::f64::consts::TAU),
                ParamRange::new(0.5, 1.5),
            ],
            Tolerances::default(),
        )
        .unwrap(),
    ]
}

fn contained_ellipse(cone: &Cone) -> Ellipse {
    let slope = 0.5;
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let tan_a = sin_a / cos_a;
    let k = slope * tan_a;
    let axial = 1.0 - k * k;
    let center_x = cone.radius() * k / axial;
    let x_radius = cone.radius() / axial;
    let y_radius = cone.radius() / axial.sqrt();
    let major = x_radius * (1.0_f64 + slope * slope).sqrt();
    let x_axis = Vec3::new(1.0, 0.0, slope).normalized().unwrap();
    let y_axis = Vec3::new(0.0, 1.0, 0.0);
    let frame = Frame::new(
        Point3::new(center_x, 0.0, slope * center_x),
        x_axis.cross(y_axis),
        x_axis,
    )
    .unwrap();
    Ellipse::new(frame, major, y_radius).unwrap()
}

fn ellipse_cases() -> Vec<CurveSurfaceIntersections> {
    let cone = cone();
    let secant = Ellipse::new(horizontal_frame([0.5, 0.0, 0.0]), 1.0, 0.5).unwrap();
    let tangent = Ellipse::new(horizontal_frame([1.5, 0.0, 0.0]), 0.5, 0.25).unwrap();
    let contained = contained_ellipse(&cone);
    vec![
        intersect_bounded_ellipse_cone(
            &secant,
            secant.param_range(),
            &cone,
            cone_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_cone(
            &tangent,
            tangent.param_range(),
            &cone,
            cone_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_cone(
            &contained,
            contained.param_range(),
            &cone,
            cone_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_cone(
            &contained,
            contained.param_range(),
            &cone,
            clipped_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_cone(
            &contained,
            contained.param_range(),
            &cone,
            [
                ParamRange::new(0.0, core::f64::consts::TAU),
                ParamRange::new(0.9, 1.1),
            ],
            Tolerances::default(),
        )
        .unwrap(),
    ]
}

// Captured from the specialized pre-consolidation solvers in both debug and
// release builds. The builds produced identical streams.
const CIRCLE_SIGNATURES: &[&[u64]] = &[
    &[
        2,
        0,
        4598175219545276356,
        4606896402722672345,
        0,
        4610891027627577108,
        4608615086221773612,
        0,
        4382308684314580131,
        0,
        4598175219545276348,
        13830268439577448154,
        0,
        4616707204065683795,
        4617276189417134669,
        0,
        4382796627585783719,
        0,
    ],
    &[
        1,
        0,
        4607182418800017408,
        4364452196894661639,
        0,
        4614256656552045848,
        4364452196894661639,
        0,
        0,
        1,
    ],
    &[0, 1, 0, 4618760256179416344, 0, 0, 0, 0],
    &[0, 1, 0, 4614256656552045848, 0, 0, 4614256656552045848, 0],
    &[0, 0],
];

const ELLIPSE_SIGNATURES: &[&[u64]] = &[
    &[
        2,
        0,
        4606168441330803479,
        4601975364591721094,
        0,
        4607961354897387588,
        4602301703833657906,
        0,
        4407350140328942966,
        0,
        4606168441330803478,
        13825347401446496904,
        0,
        4617439622248231175,
        4618220875934681845,
        0,
        4407393528201608776,
        0,
    ],
    &[
        1,
        0,
        4607182418800017408,
        4359948597267291143,
        0,
        4614256656552045848,
        4359948597267291143,
        0,
        0,
        1,
    ],
    &[
        0,
        1,
        0,
        4618760256179416344,
        0,
        4605485956407268568,
        0,
        4605485956407268568,
    ],
    &[
        0,
        1,
        0,
        4614256656552045848,
        0,
        4605485956407268568,
        4614256656552045848,
        13825114440129581678,
    ],
    &[0, 0],
];

#[test]
fn shared_conic_cone_driver_preserves_legacy_bits_and_is_deterministic() {
    for (cases, repeated, expected) in [
        (circle_cases(), circle_cases(), CIRCLE_SIGNATURES),
        (ellipse_cases(), ellipse_cases(), ELLIPSE_SIGNATURES),
    ] {
        assert_eq!(cases.len(), expected.len());
        for ((result, repeated), expected) in cases.into_iter().zip(repeated).zip(expected) {
            assert!(result.is_complete());
            assert_eq!(result, repeated);
            assert_eq!(signature(&result), *expected);
        }
    }
}

#[test]
fn shared_conic_cone_driver_preserves_variant_validation_diagnostics() {
    let cone = cone();
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let ellipse = Ellipse::new(Frame::world(), 1.0, 0.5).unwrap();
    let reversed = ParamRange { lo: 1.0, hi: 0.0 };
    let over_period = ParamRange::new(0.0, core::f64::consts::TAU + 1.0);
    let reversed_surface = [ParamRange { lo: 1.0, hi: 0.0 }, ParamRange::new(-1.0, 1.0)];

    assert!(matches!(
        intersect_bounded_circle_cone(
            &circle,
            reversed,
            &cone,
            cone_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "circle/cone intersection requires a finite non-reversed curve range"
        })
    ));
    assert!(matches!(
        intersect_bounded_circle_cone(
            &circle,
            over_period,
            &cone,
            cone_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "bounded circle range cannot span more than one period"
        })
    ));
    assert!(matches!(
        intersect_bounded_circle_cone(
            &circle,
            circle.param_range(),
            &cone,
            reversed_surface,
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "circle/cone intersection requires finite non-reversed surface ranges"
        })
    ));

    assert!(matches!(
        intersect_bounded_ellipse_cone(
            &ellipse,
            reversed,
            &cone,
            cone_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "ellipse/cone intersection requires a finite non-reversed curve range"
        })
    ));
    assert!(matches!(
        intersect_bounded_ellipse_cone(
            &ellipse,
            over_period,
            &cone,
            cone_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "bounded ellipse range cannot span more than one period"
        })
    ));
    assert!(matches!(
        intersect_bounded_ellipse_cone(
            &ellipse,
            ellipse.param_range(),
            &cone,
            reversed_surface,
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "ellipse/cone intersection requires finite non-reversed surface ranges"
        })
    ));
}
