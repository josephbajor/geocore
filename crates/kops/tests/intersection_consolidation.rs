//! Equivalence contracts for the first shared intersection-driver utilities.

use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Line};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, intersect_bounded_curves, intersect_bounded_line_circle,
    intersect_bounded_plane_sphere, intersect_bounded_surfaces,
};

#[test]
fn line_circle_shared_path_is_bit_exact_deterministic_and_complete() {
    let line = Line::new(Point3::new(-2.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let line_range = ParamRange::new(0.0, 4.0);
    let circle_range = ParamRange::new(1.5 * core::f64::consts::PI, 2.5 * core::f64::consts::PI);
    let tolerances = Tolerances::default();

    let specialized =
        intersect_bounded_line_circle(&line, line_range, &circle, circle_range, tolerances)
            .unwrap();
    let dispatched =
        intersect_bounded_curves(&line, line_range, &circle, circle_range, tolerances).unwrap();
    let repeated =
        intersect_bounded_curves(&line, line_range, &circle, circle_range, tolerances).unwrap();

    assert_eq!(dispatched, specialized);
    assert_eq!(repeated, dispatched);
    assert!(dispatched.is_complete());
    assert_eq!(dispatched.points.len(), 1);

    let reverse =
        intersect_bounded_curves(&circle, circle_range, &line, line_range, tolerances).unwrap();
    assert_eq!(reverse, specialized.swapped());
    assert!(reverse.is_complete());
}

#[test]
fn plane_sphere_shared_path_is_bit_exact_deterministic_and_complete() {
    let plane = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let plane_range = [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)];
    let sphere_range = [
        ParamRange::new(0.0, core::f64::consts::PI),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ];
    let tolerances = Tolerances::default();

    let specialized =
        intersect_bounded_plane_sphere(&plane, plane_range, &sphere, sphere_range, tolerances)
            .unwrap();
    let dispatched =
        intersect_bounded_surfaces(&plane, plane_range, &sphere, sphere_range, tolerances).unwrap();
    let repeated =
        intersect_bounded_surfaces(&plane, plane_range, &sphere, sphere_range, tolerances).unwrap();

    assert_eq!(dispatched, specialized);
    assert_eq!(repeated, dispatched);
    assert!(dispatched.is_complete());
    assert_eq!(dispatched.curves.len(), 1);

    let reverse =
        intersect_bounded_surfaces(&sphere, sphere_range, &plane, plane_range, tolerances).unwrap();
    assert_eq!(reverse, specialized.swapped());
    assert!(reverse.is_complete());
}

#[test]
fn surface_result_swapping_restores_canonical_first_surface_order() {
    let branch = Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let result = SurfaceSurfaceIntersections::canonicalized_complete(
        vec![
            SurfaceSurfacePoint {
                point: Point3::new(0.0, 0.0, 0.0),
                uv_a: [0.0, 0.0],
                uv_b: [2.0, 0.0],
                residual: 0.0,
                kind: ContactKind::Transverse,
            },
            SurfaceSurfacePoint {
                point: Point3::new(1.0, 0.0, 0.0),
                uv_a: [1.0, 0.0],
                uv_b: [1.0, 0.0],
                residual: 0.0,
                kind: ContactKind::Tangent,
            },
        ],
        vec![
            SurfaceSurfaceCurve {
                curve: SurfaceIntersectionCurve::Line(branch),
                curve_range: ParamRange::new(0.0, 1.0),
                uv_a_start: [0.0, 0.0],
                uv_a_end: [0.5, 0.0],
                uv_b_start: [2.0, 0.0],
                uv_b_end: [2.5, 0.0],
                kind: ContactKind::Transverse,
            },
            SurfaceSurfaceCurve {
                curve: SurfaceIntersectionCurve::Line(branch),
                curve_range: ParamRange::new(0.0, 1.0),
                uv_a_start: [1.0, 0.0],
                uv_a_end: [1.5, 0.0],
                uv_b_start: [1.0, 0.0],
                uv_b_end: [1.5, 0.0],
                kind: ContactKind::Tangent,
            },
        ],
    )
    .unwrap();

    let swapped = result.clone().swapped();

    assert_eq!(swapped.points[0].uv_a, [1.0, 0.0]);
    assert_eq!(swapped.points[1].uv_a, [2.0, 0.0]);
    assert_eq!(swapped.curves[0].uv_a_start, [1.0, 0.0]);
    assert_eq!(swapped.curves[1].uv_a_start, [2.0, 0.0]);
    assert!(swapped.is_complete());
    assert_eq!(swapped.swapped(), result);
}
