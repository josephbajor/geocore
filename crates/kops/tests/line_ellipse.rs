//! Bounded analytic line/ellipse intersection behavior.

use kcore::tolerance::Tolerances;
use kgeom::curve::{Ellipse, Line};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{ContactKind, intersect_bounded_line_ellipse};

fn line(origin: [f64; 3], direction: [f64; 3]) -> Line {
    Line::new(Point3::from_array(origin), Vec3::from_array(direction)).unwrap()
}

fn world_ellipse() -> Ellipse {
    Ellipse::new(Frame::world(), 3.0, 1.0).unwrap()
}

#[test]
fn coplanar_secant_returns_two_ordered_contacts() {
    let line = line([-4.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_ellipse(
        &line,
        ParamRange::new(0.0, 8.0),
        &world_ellipse(),
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(-3.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_a - 1.0).abs() < 1e-12);
    assert!((hit.points[0].t_b - core::f64::consts::PI).abs() < 1e-12);
    assert!(hit.points[1].point.dist(Point3::new(3.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[1].t_a - 7.0).abs() < 1e-12);
    assert!(hit.points[1].t_b.abs() < 1e-12);
}

#[test]
fn coplanar_tangent_and_near_tangent_are_single_contacts() {
    for (height, tolerances) in [
        (1.0, Tolerances::default()),
        (1.0 + 5e-7, Tolerances::with_linear(1e-6).unwrap()),
    ] {
        let line = line([-4.0, height, 0.0], [1.0, 0.0, 0.0]);
        let hit = intersect_bounded_line_ellipse(
            &line,
            ParamRange::new(0.0, 8.0),
            &world_ellipse(),
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
    let line = line([3.0, 0.0, -2.0], [0.0, 0.0, 1.0]);
    let hit = intersect_bounded_line_ellipse(
        &line,
        ParamRange::new(0.0, 4.0),
        &world_ellipse(),
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert_eq!(hit.points[0].point, Point3::new(3.0, 0.0, 0.0));
    assert!((hit.points[0].t_a - 2.0).abs() < 1e-12);
}

#[test]
fn tilted_ellipse_uses_its_local_plane() {
    let frame = Frame::new(
        Point3::new(1.0, 2.0, 3.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, -1.0, 0.0),
    )
    .unwrap();
    let ellipse = Ellipse::new(frame, 3.0, 1.25).unwrap();
    let line = Line::new(frame.origin() - frame.x() * 4.0, frame.x()).unwrap();
    let hit = intersect_bounded_line_ellipse(
        &line,
        ParamRange::new(0.0, 8.0),
        &ellipse,
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert!(hit.points[0].point.dist(frame.origin() - frame.x() * 3.0) < 1e-12);
    assert!(hit.points[1].point.dist(frame.origin() + frame.x() * 3.0) < 1e-12);
}

#[test]
fn finite_line_and_periodic_arc_ranges_filter_contacts() {
    let line = line([-4.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_ellipse(
        &line,
        ParamRange::new(4.0, 8.0),
        &world_ellipse(),
        ParamRange::new(1.5 * core::f64::consts::PI, 2.5 * core::f64::consts::PI),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert!((hit.points[0].t_a - 7.0).abs() < 1e-12);
    assert!((hit.points[0].t_b - core::f64::consts::TAU).abs() < 1e-12);
}

#[test]
fn offset_parallel_line_and_outside_plane_crossing_miss() {
    for line in [
        line([-4.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        line([4.0, 0.0, -1.0], [0.0, 0.0, 1.0]),
        line([-4.0, 2.0, 0.0], [1.0, 0.0, 0.0]),
    ] {
        let hit = intersect_bounded_line_ellipse(
            &line,
            ParamRange::new(0.0, 8.0),
            &world_ellipse(),
            ParamRange::new(0.0, core::f64::consts::TAU),
            Tolerances::default(),
        )
        .unwrap();
        assert!(hit.is_empty());
    }
}

#[test]
fn ellipse_range_longer_than_one_turn_is_rejected() {
    let line = line([-4.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let result = intersect_bounded_line_ellipse(
        &line,
        ParamRange::new(0.0, 8.0),
        &world_ellipse(),
        ParamRange::new(0.0, 2.0 * core::f64::consts::TAU),
        Tolerances::default(),
    );
    assert!(result.is_err());
}
