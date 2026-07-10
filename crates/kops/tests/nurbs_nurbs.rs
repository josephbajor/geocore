//! Bounded NURBS/NURBS curve intersections.

use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use kops::intersect::{ContactKind, ParamOrientation, intersect_bounded_nurbs_nurbs};

fn line_nurbs(start: Point3, end: Point3) -> NurbsCurve {
    NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end], None).unwrap()
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

#[test]
fn nurbs_nurbs_crossing_tangent_and_range_filtering() {
    let diagonal = line_nurbs(Point3::new(-1.0, -1.0, 0.0), Point3::new(1.0, 1.0, 0.0));
    let horizontal = line_nurbs(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let hit = intersect_bounded_nurbs_nurbs(
        &diagonal,
        diagonal.param_range(),
        &horizontal,
        horizontal.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[0].t_a - 0.5).abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.5).abs() < 1e-8);

    let range_miss = intersect_bounded_nurbs_nurbs(
        &diagonal,
        ParamRange::new(0.0, 0.49),
        &horizontal,
        horizontal.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(range_miss.is_empty());

    let tangent = tangent_parabola();
    let hit = intersect_bounded_nurbs_nurbs(
        &tangent,
        tangent.param_range(),
        &horizontal,
        horizontal.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[0].t_a - 0.5).abs() < 1e-8);
    assert!((hit.points[0].t_b - 0.5).abs() < 1e-8);
}

#[test]
fn nurbs_nurbs_reports_simple_contained_overlaps() {
    let a = line_nurbs(Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0));
    let b = line_nurbs(Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0));
    let hit = intersect_bounded_nurbs_nurbs(
        &a,
        a.param_range(),
        &b,
        b.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].a, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].b, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Same);

    let reversed = line_nurbs(Point3::new(3.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
    let hit = intersect_bounded_nurbs_nurbs(
        &a,
        a.param_range(),
        &reversed,
        reversed.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].a, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].b, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].orientation, ParamOrientation::Reversed);
}
