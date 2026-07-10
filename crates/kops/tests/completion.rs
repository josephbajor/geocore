//! Completion semantics for exact and provisional intersection solvers.

use kcore::proof::Completion;
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere, Surface};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    CurveCurveIntersections, intersect_bounded_line_nurbs, intersect_bounded_line_plane,
    intersect_bounded_lines, intersect_bounded_nurbs_sphere, intersect_bounded_plane_nurbs_surface,
    intersect_bounded_planes, intersect_bounded_sphere_nurbs_surface,
};

fn x_axis(y: f64) -> Line {
    Line::new(Point3::new(0.0, y, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap()
}

fn vertical_nurbs(x: f64, y0: f64, y1: f64) -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(x, y0, 0.0), Point3::new(x, y1, 0.0)],
        None,
    )
    .unwrap()
}

#[test]
fn exact_analytic_results_distinguish_proven_empty_from_contacts() {
    let range = ParamRange::new(-1.0, 1.0);
    let miss = intersect_bounded_lines(
        &x_axis(0.0),
        range,
        &x_axis(2.0),
        range,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(miss.completion(), Completion::Complete);
    assert!(miss.is_empty());
    assert!(miss.is_proven_empty());

    let transverse = Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)).unwrap();
    let hit = intersect_bounded_lines(
        &x_axis(0.0),
        range,
        &transverse,
        range,
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.is_complete());
    assert!(!hit.is_empty());
    assert!(!hit.is_proven_empty());
}

#[test]
fn sampled_nurbs_results_preserve_contacts_without_claiming_completion() {
    let line = x_axis(0.0);
    let line_range = ParamRange::new(-2.0, 2.0);
    let curve_range = ParamRange::new(0.0, 1.0);

    let hit = intersect_bounded_line_nurbs(
        &line,
        line_range,
        &vertical_nurbs(0.0, -1.0, 1.0),
        curve_range,
        Tolerances::default(),
    )
    .unwrap();
    assert!(!hit.is_empty());
    assert!(matches!(hit.completion(), Completion::Indeterminate { .. }));

    let unresolved_empty = intersect_bounded_line_nurbs(
        &line,
        line_range,
        &vertical_nurbs(0.0, 2.0, 3.0),
        curve_range,
        Tolerances::default(),
    )
    .unwrap();
    assert!(unresolved_empty.is_empty());
    assert!(!unresolved_empty.is_proven_empty());
    assert_eq!(
        unresolved_empty.completion().indeterminate_reason(),
        Some("fixed-grid line/NURBS candidate discovery does not prove complete coverage")
    );
}

#[test]
fn constructors_default_to_unknown_until_completion_is_explicit() {
    let default = CurveCurveIntersections::default();
    assert!(default.is_empty());
    assert!(!default.is_proven_empty());

    let partial = CurveCurveIntersections::canonicalized(Vec::new(), Vec::new()).unwrap();
    assert!(!partial.is_complete());
    let proven = CurveCurveIntersections::canonicalized_complete(Vec::new(), Vec::new()).unwrap();
    assert!(proven.is_proven_empty());
}

#[test]
fn curve_surface_and_surface_surface_paths_propagate_completion() {
    let plane = Plane::new(Frame::world());
    let window = [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)];
    let parallel_line = Line::new(Point3::new(0.0, 0.0, 2.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let exact_curve_miss = intersect_bounded_line_plane(
        &parallel_line,
        ParamRange::new(-1.0, 1.0),
        &plane,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert!(exact_curve_miss.is_proven_empty());

    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let sampled_curve_miss = intersect_bounded_nurbs_sphere(
        &vertical_nurbs(10.0, -1.0, 1.0),
        ParamRange::new(0.0, 1.0),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(sampled_curve_miss.is_empty());
    assert!(!sampled_curve_miss.is_proven_empty());

    let offset_frame = Frame::new(
        Point3::new(0.0, 0.0, 2.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let offset_plane = Plane::new(offset_frame);
    let exact_surface_miss =
        intersect_bounded_planes(&plane, window, &offset_plane, window, Tolerances::default())
            .unwrap();
    assert!(exact_surface_miss.is_proven_empty());

    let patch = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, -1.0, 10.0),
            Point3::new(-1.0, 1.0, 10.0),
            Point3::new(1.0, -1.0, 10.0),
            Point3::new(1.0, 1.0, 10.0),
        ],
        None,
    )
    .unwrap();
    let certified_surface_miss = intersect_bounded_plane_nurbs_surface(
        &plane,
        window,
        &patch,
        patch.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(certified_surface_miss.is_empty());
    assert!(certified_surface_miss.is_proven_empty());

    // A positive quadratic height field can have Bernstein control points on
    // both sides of the plane. The interval broad phase must retain it, while
    // the fixed grid discovers no contact. That empty remains unresolved.
    let epsilon = 10.0 * Tolerances::default().linear();
    let unresolved_patch = NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.25 + epsilon),
            Point3::new(0.0, 1.0, 0.25 + epsilon),
            Point3::new(0.5, 0.0, -0.25 + epsilon),
            Point3::new(0.5, 1.0, -0.25 + epsilon),
            Point3::new(1.0, 0.0, 0.25 + epsilon),
            Point3::new(1.0, 1.0, 0.25 + epsilon),
        ],
        None,
    )
    .unwrap();
    let unresolved_surface_miss = intersect_bounded_plane_nurbs_surface(
        &plane,
        window,
        &unresolved_patch,
        unresolved_patch.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(unresolved_surface_miss.is_empty());
    assert!(!unresolved_surface_miss.is_proven_empty());
    assert_eq!(
        unresolved_surface_miss.completion().indeterminate_reason(),
        Some("fixed-grid NURBS surface marching does not prove complete coverage")
    );

    // Exact restriction must allow a bounded subdomain to prove a miss even
    // when another part of the same underlying NURBS surface meets the sphere.
    let wide_patch = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(4.0, -1.0, 0.0),
            Point3::new(4.0, 1.0, 0.0),
        ],
        None,
    )
    .unwrap();
    let restricted_miss = intersect_bounded_sphere_nurbs_surface(
        &sphere,
        sphere.param_range(),
        &wide_patch,
        [ParamRange::new(0.75, 1.0), ParamRange::new(0.0, 1.0)],
        Tolerances::default(),
    )
    .unwrap();
    assert!(restricted_miss.is_proven_empty());
}
