//! Bounded analytic ellipse/ellipse intersection behavior.

use kcore::tolerance::Tolerances;
use kgeom::curve::Ellipse;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{ContactKind, ParamOrientation, intersect_bounded_ellipses};

fn ellipse(center: [f64; 3], normal: [f64; 3], x_hint: [f64; 3], r1: f64, r2: f64) -> Ellipse {
    Ellipse::new(
        Frame::new(
            Point3::from_array(center),
            Vec3::from_array(normal),
            Vec3::from_array(x_hint),
        )
        .unwrap(),
        r1,
        r2,
    )
    .unwrap()
}

fn world_ellipse(r1: f64, r2: f64) -> Ellipse {
    Ellipse::new(Frame::world(), r1, r2).unwrap()
}

fn assert_contains(points: &[kops::intersect::CurveCurvePoint], expected: Point3) {
    assert!(
        points
            .iter()
            .any(|point| point.point.dist(expected) < 1e-12),
        "missing expected point {expected:?} from {points:?}"
    );
}

#[test]
fn coplanar_secant_returns_four_contacts() {
    let a = world_ellipse(3.0, 1.0);
    let b = world_ellipse(2.0, 1.5);
    let hit = intersect_bounded_ellipses(
        &a,
        ParamRange::new(0.0, core::f64::consts::TAU),
        &b,
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    let x = 6.0 / 13.0_f64.sqrt();
    let y = 3.0 / 13.0_f64.sqrt();
    assert_eq!(hit.points.len(), 4);
    assert!(
        hit.points
            .iter()
            .all(|point| point.kind == ContactKind::Transverse)
    );
    for expected in [
        Point3::new(x, y, 0.0),
        Point3::new(-x, y, 0.0),
        Point3::new(-x, -y, 0.0),
        Point3::new(x, -y, 0.0),
    ] {
        assert_contains(&hit.points, expected);
    }
    assert!(hit.overlaps.is_empty());
}

#[test]
fn coplanar_tangent_and_near_tangent_are_single_contacts() {
    let a = world_ellipse(3.0, 1.0);
    for (center_y, tolerances) in [
        (2.0, Tolerances::default()),
        (2.0 + 5e-7, Tolerances::with_linear(1e-6).unwrap()),
    ] {
        let b = ellipse(
            [0.0, center_y, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            3.0,
            1.0,
        );
        let hit = intersect_bounded_ellipses(
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
fn non_coplanar_plane_crossing_contacts_are_detected() {
    let a = world_ellipse(2.0, 1.0);
    let b = ellipse([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 1.0, 0.5);
    let hit = intersect_bounded_ellipses(
        &a,
        ParamRange::new(0.0, core::f64::consts::TAU),
        &b,
        ParamRange::new(0.0, core::f64::consts::TAU),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert!(
        hit.points
            .iter()
            .all(|point| point.kind == ContactKind::Transverse)
    );
    assert_contains(&hit.points, Point3::new(0.0, 1.0, 0.0));
    assert_contains(&hit.points, Point3::new(0.0, -1.0, 0.0));
}

#[test]
fn finite_periodic_arc_ranges_filter_contacts() {
    let a = world_ellipse(3.0, 1.0);
    let b = world_ellipse(2.0, 1.5);
    let hit = intersect_bounded_ellipses(
        &a,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
        &b,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert!(hit.points[0].point.x > 0.0);
    assert!(hit.points[0].point.y > 0.0);
}

#[test]
fn coincident_ellipses_report_same_and_reversed_overlaps() {
    let a = world_ellipse(3.0, 1.0);
    let same = world_ellipse(3.0, 1.0);
    let reversed = ellipse([0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], 3.0, 1.0);

    let hit = intersect_bounded_ellipses(
        &a,
        ParamRange::new(0.25, 1.25),
        &same,
        ParamRange::new(0.75, 1.75),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].a, ParamRange::new(0.75, 1.25));
    assert_eq!(hit.overlaps[0].b, ParamRange::new(0.75, 1.25));
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Same);

    let hit = intersect_bounded_ellipses(
        &a,
        ParamRange::new(0.0, 1.0),
        &reversed,
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
fn circle_as_ellipse_reports_overlap() {
    let a = world_ellipse(1.0, 1.0);
    let b = world_ellipse(1.0, 1.0);
    let hit = intersect_bounded_ellipses(
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
fn offset_parallel_plane_and_disjoint_coplanar_curves_miss() {
    let a = world_ellipse(3.0, 1.0);
    for b in [
        ellipse([0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0, 1.5),
        ellipse([0.0, 4.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0, 1.0),
    ] {
        let hit = intersect_bounded_ellipses(
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
fn ranges_longer_than_one_turn_are_rejected() {
    let a = world_ellipse(3.0, 1.0);
    let b = world_ellipse(2.0, 1.5);
    for (range_a, range_b) in [
        (
            ParamRange::new(0.0, 2.0 * core::f64::consts::TAU),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ),
        (
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.0, 2.0 * core::f64::consts::TAU),
        ),
    ] {
        let result = intersect_bounded_ellipses(&a, range_a, &b, range_b, Tolerances::default());
        assert!(result.is_err());
    }
}
