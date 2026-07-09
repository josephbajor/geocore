//! Bounded analytic circle/circle intersection behavior.

use kcore::tolerance::Tolerances;
use kgeom::curve::Circle;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{ContactKind, ParamOrientation, intersect_bounded_circles};

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

fn unit_circle() -> Circle {
    Circle::new(Frame::world(), 1.0).unwrap()
}

#[test]
fn coplanar_secant_returns_two_ordered_contacts() {
    let a = unit_circle();
    let b = circle([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 1.0);
    let hit = intersect_bounded_circles(
        &a,
        ParamRange::new(0.0, core::f64::consts::TAU),
        &b,
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(
        hit.points[0]
            .point
            .dist(Point3::new(0.5, 0.75_f64.sqrt(), 0.0))
            < 1e-12
    );
    assert!(
        hit.points[1]
            .point
            .dist(Point3::new(0.5, -0.75_f64.sqrt(), 0.0))
            < 1e-12
    );
    assert!(hit.overlaps.is_empty());
}

#[test]
fn tangent_and_near_tangent_are_single_contacts() {
    let a = unit_circle();
    for (offset, tolerances) in [
        (2.0, Tolerances::default()),
        (2.0 + 5e-7, Tolerances::with_linear(1e-6).unwrap()),
    ] {
        let b = circle([offset, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 1.0);
        let hit = intersect_bounded_circles(
            &a,
            ParamRange::new(0.0, core::f64::consts::TAU),
            &b,
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
fn concentric_and_separate_coplanar_circles_miss() {
    let a = unit_circle();
    for b in [
        circle([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 0.5),
        circle([3.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 1.0),
        circle([0.25, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 0.25),
    ] {
        let hit = intersect_bounded_circles(
            &a,
            ParamRange::new(0.0, core::f64::consts::TAU),
            &b,
            ParamRange::new(0.0, core::f64::consts::TAU),
            Tolerances::default(),
        )
        .unwrap();
        assert!(hit.is_empty());
    }
}

#[test]
fn non_coplanar_plane_crossing_contacts_are_detected() {
    let a = unit_circle();
    let b = circle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 1.0);
    let hit = intersect_bounded_circles(
        &a,
        ParamRange::new(0.0, core::f64::consts::TAU),
        &b,
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 1.0, 0.0)) < 1e-12);
    assert!(hit.points[1].point.dist(Point3::new(0.0, -1.0, 0.0)) < 1e-12);
}

#[test]
fn finite_periodic_arc_ranges_filter_contacts() {
    let a = unit_circle();
    let b = circle([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 1.0);
    let hit = intersect_bounded_circles(
        &a,
        ParamRange::new(0.0, core::f64::consts::PI),
        &b,
        ParamRange::new(0.0, core::f64::consts::PI),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert!(hit.points[0].point.y > 0.0);
}

#[test]
fn coincident_circles_report_same_orientation_overlap() {
    let a = unit_circle();
    let b = unit_circle();
    let hit = intersect_bounded_circles(
        &a,
        ParamRange::new(0.25, 1.25),
        &b,
        ParamRange::new(0.75, 1.75),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].a, ParamRange::new(0.75, 1.25));
    assert_eq!(hit.overlaps[0].b, ParamRange::new(0.75, 1.25));
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Same);
}

#[test]
fn coincident_reversed_circle_reports_reversed_overlap() {
    let a = unit_circle();
    let b = circle([0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], 1.0);
    let hit = intersect_bounded_circles(
        &a,
        ParamRange::new(0.0, 1.0),
        &b,
        ParamRange::new(core::f64::consts::TAU - 1.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].a, ParamRange::new(0.0, 1.0));
    assert_eq!(
        hit.overlaps[0].b,
        ParamRange::new(core::f64::consts::TAU - 1.0, core::f64::consts::TAU)
    );
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Reversed);
}

#[test]
fn coincident_endpoint_only_arc_contact_is_tangent_point() {
    let a = unit_circle();
    let b = unit_circle();
    let hit = intersect_bounded_circles(
        &a,
        ParamRange::new(0.0, 1.0),
        &b,
        ParamRange::new(1.0, 2.0),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!((hit.points[0].t_a - 1.0).abs() < 1e-12);
    assert!(hit.overlaps.is_empty());
}

#[test]
fn circle_range_longer_than_one_turn_is_rejected() {
    let a = unit_circle();
    let b = unit_circle();
    let result = intersect_bounded_circles(
        &a,
        ParamRange::new(0.0, 2.0 * core::f64::consts::TAU),
        &b,
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    );
    assert!(result.is_err());
}
