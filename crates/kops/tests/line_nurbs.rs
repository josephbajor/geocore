//! Bounded line/NURBS curve intersections.

use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Line};
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{ContactKind, ParamOrientation, intersect_bounded_line_nurbs};

fn x_axis() -> Line {
    Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap()
}

fn transverse_segment() -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-1.0, -1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        None,
    )
    .unwrap()
}

fn tangent_parabola() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 1.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        None,
    )
    .unwrap()
}

fn contained_segment(start_x: f64, end_x: f64) -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(start_x, 0.0, 0.0), Point3::new(end_x, 0.0, 0.0)],
        None,
    )
    .unwrap()
}

#[test]
fn line_nurbs_crossing_tangent_and_range_filtering() {
    let line = x_axis();
    let curve = transverse_segment();
    let hit = intersect_bounded_line_nurbs(
        &line,
        ParamRange::new(-2.0, 2.0),
        &curve,
        curve.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 0.0, 0.0)) < 1e-8);
    assert!(hit.points[0].t_a.abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.5).abs() < 1e-8);

    let line_miss = intersect_bounded_line_nurbs(
        &line,
        ParamRange::new(0.25, 2.0),
        &curve,
        curve.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(line_miss.is_empty());

    let curve_miss = intersect_bounded_line_nurbs(
        &line,
        ParamRange::new(-2.0, 2.0),
        &curve,
        ParamRange::new(0.0, 0.49),
        Tolerances::default(),
    )
    .unwrap();
    assert!(curve_miss.is_empty());

    let tangent = tangent_parabola();
    let hit = intersect_bounded_line_nurbs(
        &line,
        ParamRange::new(-2.0, 2.0),
        &tangent,
        tangent.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 0.0, 0.0)) < 1e-8);
    assert!(hit.points[0].t_a.abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.5).abs() < 1e-8);
}

#[test]
fn line_nurbs_contained_overlap_clips_to_line_range() {
    let line = x_axis();
    let curve = contained_segment(0.0, 3.0);
    let hit = intersect_bounded_line_nurbs(
        &line,
        ParamRange::new(1.0, 2.0),
        &curve,
        curve.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].a, ParamRange::new(1.0, 2.0));
    assert!((hit.overlaps[0].b.lo - 1.0 / 3.0).abs() < 1e-8);
    assert!((hit.overlaps[0].b.hi - 2.0 / 3.0).abs() < 1e-8);
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Same);

    let reversed = contained_segment(3.0, 0.0);
    let hit = intersect_bounded_line_nurbs(
        &line,
        ParamRange::new(1.0, 2.0),
        &reversed,
        reversed.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].a, ParamRange::new(1.0, 2.0));
    assert!((hit.overlaps[0].b.lo - 1.0 / 3.0).abs() < 1e-8);
    assert!((hit.overlaps[0].b.hi - 2.0 / 3.0).abs() < 1e-8);
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Reversed);
}
