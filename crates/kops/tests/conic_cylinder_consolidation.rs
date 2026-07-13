//! Bit-pattern and completion contracts for the shared conic/cylinder driver.

use kcore::error::Error;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, CurveSurfaceIntersections, intersect_bounded_circle_cylinder,
    intersect_bounded_ellipse_cylinder,
};

fn horizontal_frame(origin: [f64; 3]) -> Frame {
    Frame::new(
        Point3::from_array(origin),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

fn cylinder_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
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

#[test]
fn shared_conic_cylinder_driver_preserves_legacy_bits_and_is_deterministic() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let circle = Circle::new(horizontal_frame([0.5, 0.0, 0.25]), 1.0).unwrap();
    let circle_secant = intersect_bounded_circle_cylinder(
        &circle,
        circle.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(circle_secant.is_complete());
    assert_eq!(
        signature(&circle_secant),
        [
            2,
            0,
            4598175219545276356,
            4606896402722672345,
            4598175219545276416,
            4610891027627577108,
            4608615086221773612,
            4598175219545276416,
            4382308684314580131,
            0,
            4598175219545276348,
            13830268439577448154,
            4598175219545276416,
            4616707204065683795,
            4617276189417134669,
            4598175219545276416,
            4382796627585783719,
            0,
        ]
    );
    assert_eq!(
        intersect_bounded_circle_cylinder(
            &circle,
            circle.param_range(),
            &cylinder,
            cylinder_window(),
            Tolerances::default(),
        )
        .unwrap(),
        circle_secant
    );

    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let circle_overlap = intersect_bounded_circle_cylinder(
        &circle,
        circle.param_range(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(circle_overlap.is_complete());
    assert_eq!(
        signature(&circle_overlap),
        [0, 1, 0, 4614256656552045848, 0, 0, 4614256656552045848, 0,]
    );
    assert_eq!(
        intersect_bounded_circle_cylinder(
            &circle,
            circle.param_range(),
            &cylinder,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(-1.0, 1.0),
            ],
            Tolerances::default(),
        )
        .unwrap(),
        circle_overlap
    );

    let ellipse = Ellipse::new(horizontal_frame([0.5, 0.0, 0.25]), 1.0, 0.5).unwrap();
    let ellipse_secant = intersect_bounded_ellipse_cylinder(
        &ellipse,
        ellipse.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(ellipse_secant.is_complete());
    assert_eq!(
        signature(&ellipse_secant),
        [
            2,
            0,
            4606168441330803479,
            4601975364591721094,
            4598175219545276416,
            4607961354897387588,
            4602301703833657906,
            4598175219545276416,
            4407350140328942966,
            0,
            4606168441330803478,
            13825347401446496904,
            4598175219545276416,
            4617439622248231175,
            4618220875934681845,
            4598175219545276416,
            4407393528201608776,
            0,
        ]
    );
    assert_eq!(
        intersect_bounded_ellipse_cylinder(
            &ellipse,
            ellipse.param_range(),
            &cylinder,
            cylinder_window(),
            Tolerances::default(),
        )
        .unwrap(),
        ellipse_secant
    );

    let slope = 0.5;
    let major = (1.0_f64 + slope * slope).sqrt();
    let x_axis = Vec3::new(1.0, 0.0, slope).normalized().unwrap();
    let y_axis = Vec3::new(0.0, 1.0, 0.0);
    let frame = Frame::new(Point3::new(0.0, 0.0, 0.0), x_axis.cross(y_axis), x_axis).unwrap();
    let ellipse = Ellipse::new(frame, major, 1.0).unwrap();
    let ellipse_overlap = intersect_bounded_ellipse_cylinder(
        &ellipse,
        ellipse.param_range(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(ellipse_overlap.is_complete());
    assert_eq!(
        signature(&ellipse_overlap),
        [
            0,
            1,
            0,
            4614256656552045848,
            0,
            4602678819172646913,
            4614256656552045848,
            13826050856027422721,
        ]
    );
    assert_eq!(
        intersect_bounded_ellipse_cylinder(
            &ellipse,
            ellipse.param_range(),
            &cylinder,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(-1.0, 1.0),
            ],
            Tolerances::default(),
        )
        .unwrap(),
        ellipse_overlap
    );
}

#[test]
fn shared_conic_cylinder_driver_preserves_variant_validation_diagnostics() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let ellipse = Ellipse::new(Frame::world(), 1.0, 0.5).unwrap();
    let reversed = ParamRange { lo: 1.0, hi: 0.0 };
    let over_period = ParamRange::new(0.0, core::f64::consts::TAU + 1.0);
    let reversed_surface = [ParamRange { lo: 1.0, hi: 0.0 }, ParamRange::new(-1.0, 1.0)];

    assert!(matches!(
        intersect_bounded_circle_cylinder(
            &circle,
            reversed,
            &cylinder,
            cylinder_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "circle/cylinder intersection requires a finite non-reversed curve range"
        })
    ));
    assert!(matches!(
        intersect_bounded_circle_cylinder(
            &circle,
            over_period,
            &cylinder,
            cylinder_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "bounded circle range cannot span more than one period"
        })
    ));
    assert!(matches!(
        intersect_bounded_circle_cylinder(
            &circle,
            circle.param_range(),
            &cylinder,
            reversed_surface,
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "circle/cylinder intersection requires finite non-reversed surface ranges"
        })
    ));

    assert!(matches!(
        intersect_bounded_ellipse_cylinder(
            &ellipse,
            reversed,
            &cylinder,
            cylinder_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "ellipse/cylinder intersection requires a finite non-reversed curve range"
        })
    ));
    assert!(matches!(
        intersect_bounded_ellipse_cylinder(
            &ellipse,
            over_period,
            &cylinder,
            cylinder_window(),
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "bounded ellipse range cannot span more than one period"
        })
    ));
    assert!(matches!(
        intersect_bounded_ellipse_cylinder(
            &ellipse,
            ellipse.param_range(),
            &cylinder,
            reversed_surface,
            Tolerances::default(),
        ),
        Err(Error::InvalidGeometry {
            reason: "ellipse/cylinder intersection requires finite non-reversed surface ranges"
        })
    ));
}
