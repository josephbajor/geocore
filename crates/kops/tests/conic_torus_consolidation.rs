//! Bit-pattern and completion contracts for the shared conic/torus driver.

use kcore::error::Error;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Torus;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, CurveSurfaceIntersections, intersect_bounded_circle_torus,
    intersect_bounded_ellipse_torus,
};

fn horizontal_frame(origin: [f64; 3]) -> Frame {
    Frame::new(
        Point3::from_array(origin),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

fn torus() -> Torus {
    Torus::new(Frame::world(), 2.0, 0.5).unwrap()
}

fn torus_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(0.0, core::f64::consts::TAU),
    ]
}

fn longitude_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::PI),
        ParamRange::new(0.0, core::f64::consts::TAU),
    ]
}

fn tube_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(0.0, core::f64::consts::PI),
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

fn tube_frame() -> Frame {
    Frame::new(
        Point3::new(2.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

fn circle_cases() -> Vec<CurveSurfaceIntersections> {
    let torus = torus();
    let secant = Circle::new(horizontal_frame([1.0, 0.0, 0.0]), 1.0).unwrap();
    let tangent = Circle::new(horizontal_frame([3.0, 0.0, 0.0]), 0.5).unwrap();
    let latitude = Circle::new(Frame::world(), 2.5).unwrap();
    let tube = Circle::new(tube_frame(), 0.5).unwrap();
    vec![
        intersect_bounded_circle_torus(
            &secant,
            secant.param_range(),
            &torus,
            torus_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_torus(
            &tangent,
            tangent.param_range(),
            &torus,
            torus_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_torus(
            &latitude,
            latitude.param_range(),
            &torus,
            torus_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_torus(
            &latitude,
            latitude.param_range(),
            &torus,
            longitude_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_torus(
            &tube,
            tube.param_range(),
            &torus,
            tube_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_circle_torus(
            &latitude,
            latitude.param_range(),
            &torus,
            [
                ParamRange::new(0.0, core::f64::consts::TAU),
                ParamRange::new(core::f64::consts::FRAC_PI_2, core::f64::consts::PI),
            ],
            Tolerances::default(),
        )
        .unwrap(),
    ]
}

fn ellipse_cases() -> Vec<CurveSurfaceIntersections> {
    let torus = torus();
    let secant = Ellipse::new(horizontal_frame([1.0, 0.0, 0.0]), 1.0, 0.5).unwrap();
    let tangent = Ellipse::new(horizontal_frame([3.0, 0.0, 0.0]), 0.5, 0.25).unwrap();
    let latitude = Ellipse::new(Frame::world(), 2.5, 2.5).unwrap();
    let tube = Ellipse::new(tube_frame(), 0.5, 0.5).unwrap();
    vec![
        intersect_bounded_ellipse_torus(
            &secant,
            secant.param_range(),
            &torus,
            torus_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_torus(
            &tangent,
            tangent.param_range(),
            &torus,
            torus_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_torus(
            &latitude,
            latitude.param_range(),
            &torus,
            torus_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_torus(
            &latitude,
            latitude.param_range(),
            &torus,
            longitude_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_torus(
            &tube,
            tube.param_range(),
            &torus,
            tube_window(),
            Tolerances::default(),
        )
        .unwrap(),
        intersect_bounded_ellipse_torus(
            &latitude,
            latitude.param_range(),
            &torus,
            [
                ParamRange::new(0.0, core::f64::consts::TAU),
                ParamRange::new(core::f64::consts::FRAC_PI_2, core::f64::consts::PI),
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
        4607745368753438622,
        4607111773009245624,
        4359948597267291143,
        4609188630550927832,
        4604685030923557336,
        4614256656552045848,
        4401620062023353921,
        0,
        4607745368753438621,
        13830483809864021434,
        4359948597267291143,
        4617132803334846114,
        4617946529757131229,
        4614256656552045848,
        4401719394949030314,
        0,
    ],
    &[
        1,
        0,
        4612811918334230528,
        4364452196894661639,
        0,
        4614256656552045848,
        4358425652199933554,
        0,
        0,
        1,
    ],
    &[0, 1, 0, 4618760256179416344, 0, 0, 0, 0],
    &[0, 1, 0, 4614256656552045848, 0, 0, 4614256656552045848, 0],
    &[0, 1, 0, 4614256656552045848, 0, 0, 0, 4614256656552045848],
    &[0, 0],
];

const ELLIPSE_SIGNATURES: &[&[u64]] = &[
    &[
        2,
        0,
        4609121222375940428,
        4601801429336985717,
        4359948597267291143,
        4607748740569652840,
        4599176781519782428,
        4614256656552045848,
        4395363898707273531,
        0,
        4609121222375940427,
        13825173466191761532,
        4359948597267291143,
        4617492775830164862,
        4618416183579299062,
        4614256656552045848,
        4395161740639163442,
        0,
    ],
    &[
        1,
        0,
        4612811918334230528,
        4359948597267291143,
        0,
        4614256656552045848,
        4353922052572563058,
        0,
        0,
        1,
    ],
    &[0, 1, 0, 4618760256179416344, 0, 0, 0, 0],
    &[0, 1, 0, 4614256656552045848, 0, 0, 4614256656552045848, 0],
    &[0, 1, 0, 4614256656552045848, 0, 0, 0, 4614256656552045848],
    &[0, 0],
];

#[test]
fn shared_conic_torus_driver_preserves_legacy_bits_and_is_deterministic() {
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
fn shared_conic_torus_driver_preserves_variant_validation_diagnostics() {
    let torus = torus();
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let ellipse = Ellipse::new(Frame::world(), 1.0, 0.5).unwrap();
    let reversed = ParamRange { lo: 1.0, hi: 0.0 };
    let over_period = ParamRange::new(0.0, core::f64::consts::TAU + 1.0);
    let reversed_surface = [
        ParamRange { lo: 1.0, hi: 0.0 },
        ParamRange::new(0.0, core::f64::consts::TAU),
    ];

    assert!(matches!(
        intersect_bounded_circle_torus(
            &circle,
            reversed,
            &torus,
            torus_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "circle/torus intersection requires a finite non-reversed curve range"
        })
    ));
    assert!(matches!(
        intersect_bounded_circle_torus(
            &circle,
            over_period,
            &torus,
            torus_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "bounded circle range cannot span more than one period"
        })
    ));
    assert!(matches!(
        intersect_bounded_circle_torus(
            &circle,
            circle.param_range(),
            &torus,
            reversed_surface,
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "circle/torus intersection requires finite non-reversed surface ranges"
        })
    ));

    assert!(matches!(
        intersect_bounded_ellipse_torus(
            &ellipse,
            reversed,
            &torus,
            torus_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "ellipse/torus intersection requires a finite non-reversed curve range"
        })
    ));
    assert!(matches!(
        intersect_bounded_ellipse_torus(
            &ellipse,
            over_period,
            &torus,
            torus_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "bounded ellipse range cannot span more than one period"
        })
    ));
    assert!(matches!(
        intersect_bounded_ellipse_torus(
            &ellipse,
            ellipse.param_range(),
            &torus,
            reversed_surface,
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "ellipse/torus intersection requires finite non-reversed surface ranges"
        })
    ));
}
