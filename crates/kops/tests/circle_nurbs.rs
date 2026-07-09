//! Bounded circle/NURBS curve intersections.

use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use kops::intersect::{ContactKind, ParamOrientation, intersect_bounded_circle_nurbs};

fn unit_circle() -> Circle {
    Circle::new(Frame::world(), 1.0).unwrap()
}

fn crossing_line_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
        None,
    )
    .unwrap()
}

fn tangent_line_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-0.5, 1.0, 0.0), Point3::new(0.5, 1.0, 0.0)],
        None,
    )
    .unwrap()
}

fn quarter_circle_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
    )
    .unwrap()
}

fn reversed_quarter_circle_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ],
        Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
    )
    .unwrap()
}

#[test]
fn circle_nurbs_crossing_tangent_and_range_filtering() {
    let circle = unit_circle();
    let curve = crossing_line_nurbs();
    let hit = intersect_bounded_circle_nurbs(
        &circle,
        circle.param_range(),
        &curve,
        curve.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-8);
    assert!(hit.points[0].t_a.abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.75).abs() < 1e-8);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(-1.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[1].t_a - core::f64::consts::PI).abs() < 1e-8);
    assert!((hit.points[1].t_b - 0.25).abs() < 1e-8);

    let circle_filtered = intersect_bounded_circle_nurbs(
        &circle,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
        &curve,
        curve.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(circle_filtered.points.len(), 1);
    assert!(
        circle_filtered.points[0]
            .point
            .dist(Point3::new(1.0, 0.0, 0.0))
            < 1e-8
    );

    let curve_miss = intersect_bounded_circle_nurbs(
        &circle,
        circle.param_range(),
        &curve,
        ParamRange::new(0.0, 0.2),
        Tolerances::default(),
    )
    .unwrap();
    assert!(curve_miss.is_empty());

    let tangent = tangent_line_nurbs();
    let hit = intersect_bounded_circle_nurbs(
        &circle,
        circle.param_range(),
        &tangent,
        tangent.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 1.0, 0.0)) < 1e-8);
    assert!((hit.points[0].t_a - core::f64::consts::FRAC_PI_2).abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.5).abs() < 1e-8);
}

#[test]
fn circle_nurbs_contained_overlap_clips_to_circle_range() {
    let circle = unit_circle();
    let curve = quarter_circle_nurbs();
    let hit = intersect_bounded_circle_nurbs(
        &circle,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_4),
        &curve,
        curve.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(
        hit.overlaps[0].a,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_4)
    );
    assert!(hit.overlaps[0].b.lo.abs() < 1e-8);
    assert!((hit.overlaps[0].b.hi - 0.5).abs() < 1e-8);
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Same);

    let reversed = reversed_quarter_circle_nurbs();
    let hit = intersect_bounded_circle_nurbs(
        &circle,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_4),
        &reversed,
        reversed.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(
        hit.overlaps[0].a,
        ParamRange::new(0.0, core::f64::consts::FRAC_PI_4)
    );
    assert!((hit.overlaps[0].b.lo - 0.5).abs() < 1e-8);
    assert!((hit.overlaps[0].b.hi - 1.0).abs() < 1e-8);
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Reversed);
}
