//! General bounded curve/curve dispatch over supported analytic classes.

use kcore::error::Error;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{ContactKind, intersect_bounded_curves};

fn line(origin: [f64; 3], direction: [f64; 3]) -> Line {
    Line::new(Point3::from_array(origin), Vec3::from_array(direction)).unwrap()
}

fn full_range(curve: &dyn Curve) -> ParamRange {
    curve.param_range()
}

#[test]
fn dispatches_line_line_and_line_ellipse() {
    let a = line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let b = line([0.5, -1.0, 0.0], [0.0, 1.0, 0.0]);
    let hit = intersect_bounded_curves(
        &a,
        ParamRange::new(0.0, 1.0),
        &b,
        ParamRange::new(0.0, 2.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].point, Point3::new(0.5, 0.0, 0.0));

    let ellipse = Ellipse::new(Frame::world(), 3.0, 1.0).unwrap();
    let hit = intersect_bounded_curves(
        &a,
        ParamRange::new(-4.0, 4.0),
        &ellipse,
        full_range(&ellipse),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);
}

#[test]
fn reversed_dispatch_recanonicalizes_in_first_curve_order() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let line = line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_curves(
        &circle,
        full_range(&circle),
        &line,
        ParamRange::new(0.0, 4.0),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert!(hit.points[0].t_a.abs() < 1e-12);
    assert!((hit.points[0].t_b - 3.0).abs() < 1e-12);
    assert!((hit.points[1].t_a - core::f64::consts::PI).abs() < 1e-12);
    assert!((hit.points[1].t_b - 1.0).abs() < 1e-12);
}

#[test]
fn dispatches_circle_ellipse_and_ellipse_ellipse() {
    let circle = Circle::new(Frame::world(), 2.0).unwrap();
    let ellipse = Ellipse::new(Frame::world(), 3.0, 1.0).unwrap();
    let hit = intersect_bounded_curves(
        &circle,
        full_range(&circle),
        &ellipse,
        full_range(&ellipse),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 4);
    assert!(
        hit.points
            .iter()
            .all(|point| point.kind == ContactKind::Transverse)
    );

    let other = Ellipse::new(Frame::world(), 2.0, 1.5).unwrap();
    let hit = intersect_bounded_curves(
        &ellipse,
        full_range(&ellipse),
        &other,
        full_range(&other),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 4);
}

#[test]
fn unsupported_curve_class_is_explicit_error() {
    let nurbs = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let line = line([0.5, -1.0, 0.0], [0.0, 1.0, 0.0]);
    let err = intersect_bounded_curves(
        &nurbs,
        full_range(&nurbs),
        &line,
        ParamRange::new(0.0, 2.0),
        Tolerances::default(),
    )
    .unwrap_err();

    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "unsupported curve/curve intersection class"
        }
    );
}
