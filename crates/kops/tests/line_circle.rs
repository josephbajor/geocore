//! Bounded analytic line/circle intersection behavior.

use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Line};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{ContactKind, intersect_bounded_line_circle};

fn line(origin: [f64; 3], direction: [f64; 3]) -> Line {
    Line::new(Point3::from_array(origin), Vec3::from_array(direction)).unwrap()
}

fn unit_circle() -> Circle {
    Circle::new(Frame::world(), 1.0).unwrap()
}

#[test]
fn coplanar_secant_returns_two_ordered_contacts() {
    let line = line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_circle(
        &line,
        ParamRange::new(0.0, 4.0),
        &unit_circle(),
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(-1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_a - 1.0).abs() < 1e-12);
    assert!((hit.points[0].t_b - core::f64::consts::PI).abs() < 1e-12);
    assert!(hit.points[1].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[1].t_a - 3.0).abs() < 1e-12);
    assert!(hit.points[1].t_b.abs() < 1e-12);
}

#[test]
fn coplanar_tangent_and_near_tangent_are_single_contacts() {
    for (height, tolerances) in [
        (1.0, Tolerances::default()),
        (1.0 + 5e-7, Tolerances::with_linear(1e-6).unwrap()),
    ] {
        let line = line([-2.0, height, 0.0], [1.0, 0.0, 0.0]);
        let hit = intersect_bounded_line_circle(
            &line,
            ParamRange::new(0.0, 4.0),
            &unit_circle(),
            ParamRange::new(0.0, core::f64::consts::TAU),
            tolerances,
        )
        .unwrap();
        assert_eq!(hit.points.len(), 1);
        assert_eq!(hit.points[0].kind, ContactKind::Tangent);
        assert!(hit.points[0].residual <= tolerances.linear());
    }
}

#[test]
fn transverse_plane_crossing_is_detected() {
    let line = line([1.0, 0.0, -2.0], [0.0, 0.0, 1.0]);
    let hit = intersect_bounded_line_circle(
        &line,
        ParamRange::new(0.0, 4.0),
        &unit_circle(),
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert_eq!(hit.points[0].point, Point3::new(1.0, 0.0, 0.0));
    assert!((hit.points[0].t_a - 2.0).abs() < 1e-12);
}

#[test]
fn tilted_circle_uses_its_local_plane() {
    let frame = Frame::new(
        Point3::new(1.0, 2.0, 3.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, -1.0, 0.0),
    )
    .unwrap();
    let circle = Circle::new(frame, 2.0).unwrap();
    let line = Line::new(frame.origin() - frame.x() * 3.0, frame.x()).unwrap();
    let hit = intersect_bounded_line_circle(
        &line,
        ParamRange::new(0.0, 6.0),
        &circle,
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert!(hit.points[0].point.dist(frame.origin() - frame.x() * 2.0) < 1e-12);
    assert!(hit.points[1].point.dist(frame.origin() + frame.x() * 2.0) < 1e-12);
}

#[test]
fn finite_line_and_periodic_arc_ranges_filter_contacts() {
    let line = line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_circle(
        &line,
        ParamRange::new(2.0, 4.0),
        &unit_circle(),
        ParamRange::new(1.5 * core::f64::consts::PI, 2.5 * core::f64::consts::PI),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert!((hit.points[0].t_a - 3.0).abs() < 1e-12);
    assert!((hit.points[0].t_b - core::f64::consts::TAU).abs() < 1e-12);
}

#[test]
fn offset_parallel_line_and_outside_plane_crossing_miss() {
    for line in [
        line([-2.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        line([2.0, 0.0, -1.0], [0.0, 0.0, 1.0]),
    ] {
        let hit = intersect_bounded_line_circle(
            &line,
            ParamRange::new(0.0, 4.0),
            &unit_circle(),
            ParamRange::new(0.0, core::f64::consts::TAU),
            Tolerances::default(),
        )
        .unwrap();
        assert!(hit.is_empty());
    }
}

#[test]
fn circle_range_longer_than_one_turn_is_rejected() {
    let line = line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let result = intersect_bounded_line_circle(
        &line,
        ParamRange::new(0.0, 4.0),
        &unit_circle(),
        ParamRange::new(0.0, 2.0 * core::f64::consts::TAU),
        Tolerances::default(),
    );
    assert!(result.is_err());
}
