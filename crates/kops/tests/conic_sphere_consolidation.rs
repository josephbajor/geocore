//! Bit-pattern and completion contracts for the shared conic/sphere driver.

use kcore::error::Error;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Sphere, Surface};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, CurveSurfaceIntersections, intersect_bounded_circle_sphere,
    intersect_bounded_ellipse_sphere,
};

fn horizontal_frame(origin: [f64; 3]) -> Frame {
    Frame::new(
        Point3::from_array(origin),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

fn sphere() -> Sphere {
    Sphere::new(Frame::world(), 1.0).unwrap()
}

fn clipped_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::PI),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
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
    let sphere = sphere();
    let secant = Circle::new(horizontal_frame([0.5, 0.0, 0.0]), 1.0).unwrap();
    let tangent = Circle::new(horizontal_frame([1.5, 0.0, 0.0]), 0.5).unwrap();
    let contained = Circle::new(Frame::world(), 1.0).unwrap();
    let miss = Circle::new(Frame::world(), 0.5).unwrap();
    vec![
        intersect_bounded_circle_sphere(
            &secant,
            secant.param_range(),
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_sphere(
            &secant,
            secant.param_range(),
            &sphere,
            clipped_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_sphere(
            &tangent,
            tangent.param_range(),
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_sphere(
            &contained,
            contained.param_range(),
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_sphere(
            &contained,
            contained.param_range(),
            &sphere,
            clipped_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_sphere(
            &miss,
            miss.param_range(),
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        )
        .unwrap(),
    ]
}

fn ellipse_cases() -> Vec<CurveSurfaceIntersections> {
    let sphere = sphere();
    let secant = Ellipse::new(horizontal_frame([0.5, 0.0, 0.0]), 1.0, 0.5).unwrap();
    let tangent = Ellipse::new(horizontal_frame([1.5, 0.0, 0.0]), 0.5, 0.25).unwrap();
    let contained = Ellipse::new(Frame::world(), 1.0, 1.0).unwrap();
    let miss = Ellipse::new(Frame::world(), 0.5, 0.25).unwrap();
    vec![
        intersect_bounded_ellipse_sphere(
            &secant,
            secant.param_range(),
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_sphere(
            &secant,
            secant.param_range(),
            &sphere,
            clipped_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_sphere(
            &tangent,
            tangent.param_range(),
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_sphere(
            &contained,
            contained.param_range(),
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_sphere(
            &contained,
            contained.param_range(),
            &sphere,
            clipped_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_sphere(
            &miss,
            miss.param_range(),
            &sphere,
            sphere.param_range(),
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
        4598175219545276414,
        4606896402722672346,
        0,
        4610891027627577100,
        4608615086221773605,
        0,
        4370357090594536397,
        0,
        4598175219545276394,
        13830268439577448154,
        0,
        4616707204065683797,
        4617276189417134670,
        0,
        4379560306371961762,
        0,
    ],
    &[
        1,
        0,
        4598175219545276414,
        4606896402722672346,
        0,
        4610891027627577100,
        4608615086221773605,
        0,
        4370357090594536397,
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
        4606168441330803479,
        4601975364591721094,
        0,
        4607961354897387588,
        4602301703833657906,
        0,
        4407350140328942966,
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
    &[0, 1, 0, 4618760256179416344, 0, 0, 0, 0],
    &[0, 1, 0, 4614256656552045848, 0, 0, 4614256656552045848, 0],
    &[0, 0],
];

#[test]
fn shared_conic_sphere_driver_preserves_legacy_bits_and_is_deterministic() {
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
fn shared_conic_sphere_driver_preserves_variant_validation_diagnostics() {
    let sphere = sphere();
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let ellipse = Ellipse::new(Frame::world(), 1.0, 0.5).unwrap();
    let reversed = ParamRange { lo: 1.0, hi: 0.0 };
    let over_period = ParamRange::new(0.0, core::f64::consts::TAU + 1.0);
    let reversed_surface = [
        ParamRange { lo: 1.0, hi: 0.0 },
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ];

    assert!(matches!(
        intersect_bounded_circle_sphere(
            &circle,
            reversed,
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "circle/sphere intersection requires a finite non-reversed curve range"
        })
    ));
    assert!(matches!(
        intersect_bounded_circle_sphere(
            &circle,
            over_period,
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "bounded circle range cannot span more than one period"
        })
    ));
    assert!(matches!(
        intersect_bounded_circle_sphere(
            &circle,
            circle.param_range(),
            &sphere,
            reversed_surface,
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "circle/sphere intersection requires finite non-reversed surface ranges"
        })
    ));

    assert!(matches!(
        intersect_bounded_ellipse_sphere(
            &ellipse,
            reversed,
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "ellipse/sphere intersection requires a finite non-reversed curve range"
        })
    ));
    assert!(matches!(
        intersect_bounded_ellipse_sphere(
            &ellipse,
            over_period,
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "bounded ellipse range cannot span more than one period"
        })
    ));
    assert!(matches!(
        intersect_bounded_ellipse_sphere(
            &ellipse,
            ellipse.param_range(),
            &sphere,
            reversed_surface,
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "ellipse/sphere intersection requires finite non-reversed surface ranges"
        })
    ));
}
