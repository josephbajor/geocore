//! Bounded analytic surface/surface intersection behavior.

use kcore::error::Error;
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane, Sphere, Surface};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceIntersections,
    intersect_bounded_plane_cylinder, intersect_bounded_plane_sphere, intersect_bounded_planes,
    intersect_bounded_spheres, intersect_bounded_surfaces,
};

fn plane_window() -> [ParamRange; 2] {
    [ParamRange::new(-1.0, 1.0), ParamRange::new(-1.0, 1.0)]
}

fn wide_plane_window() -> [ParamRange; 2] {
    [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)]
}

fn cylinder_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-1.0, 1.0),
    ]
}

fn sphere_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ]
}

fn horizontal_plane(z: f64) -> Plane {
    Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, z),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    )
}

fn vertical_plane_x(x: f64) -> Plane {
    Plane::new(
        Frame::new(
            Point3::new(x, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
    )
}

fn vertical_plane_y(y: f64) -> Plane {
    Plane::new(
        Frame::new(
            Point3::new(0.0, y, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    )
}

fn oblique_cylinder_plane() -> Plane {
    Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(-0.5, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
    )
}

fn assert_plane_cylinder_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    plane: &Plane,
    cylinder: &Cylinder,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(plane.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(plane.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(cylinder.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(cylinder.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_plane_plane_branch_endpoints(hit: &SurfaceSurfaceIntersections, a: &Plane, b: &Plane) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(a.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(a.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(b.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(b.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn total_curve_width(hit: &SurfaceSurfaceIntersections) -> f64 {
    hit.curves
        .iter()
        .map(|branch| branch.curve_range.width())
        .sum()
}

fn assert_sphere_sphere_circle_segments(
    hit: &SurfaceSurfaceIntersections,
    a: &Sphere,
    b: &Sphere,
    expected_segments: usize,
    expected_width: f64,
) {
    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), expected_segments);
    let mut total_width = 0.0;
    let mut last_hi = None;
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        assert!(branch.curve_range.width() > 0.0);
        if let Some(last_hi) = last_hi {
            assert!(branch.curve_range.lo >= last_hi - 1e-12);
        }
        last_hi = Some(branch.curve_range.hi);
        total_width += branch.curve_range.width();

        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("sphere/sphere secant should be carried by circle segments");
        };
        assert!(
            a.eval(branch.uv_a_start)
                .dist(circle.eval(branch.curve_range.lo))
                < 1e-12
        );
        assert!(
            a.eval(branch.uv_a_end)
                .dist(circle.eval(branch.curve_range.hi))
                < 1e-12
        );
        assert!(
            b.eval(branch.uv_b_start)
                .dist(circle.eval(branch.curve_range.lo))
                < 1e-12
        );
        assert!(
            b.eval(branch.uv_b_end)
                .dist(circle.eval(branch.curve_range.hi))
                < 1e-12
        );
    }
    assert!((total_width - expected_width).abs() < 1e-12);
}

#[test]
fn plane_plane_transverse_returns_bounded_line_branch() {
    let a = horizontal_plane(0.0);
    let b = vertical_plane_x(0.0);
    let hit = intersect_bounded_planes(
        &a,
        plane_window(),
        &b,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert_eq!(hit.curves[0].kind, ContactKind::Transverse);
    assert!((hit.curves[0].curve_range.lo + 1.0).abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - 1.0).abs() < 1e-12);
    assert_plane_plane_branch_endpoints(&hit, &a, &b);
    let SurfaceIntersectionCurve::Line(line) = &hit.curves[0].curve else {
        panic!("transverse plane/plane cut should be a line");
    };
    assert!(line.origin().dist(Point3::new(0.0, 0.0, 0.0)) < 1e-12);
    assert!(line.dir().cross(Vec3::new(0.0, 1.0, 0.0)).norm() < 1e-12);
}

#[test]
fn plane_plane_windows_clip_line_branch() {
    let a = horizontal_plane(0.0);
    let b = vertical_plane_x(0.0);
    let hit = intersect_bounded_planes(
        &a,
        [ParamRange::new(-1.0, 1.0), ParamRange::new(-0.5, 0.75)],
        &b,
        [ParamRange::new(0.0, 1.0), ParamRange::new(-1.0, 1.0)],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!((hit.curves[0].curve_range.lo - 0.0).abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - 0.75).abs() < 1e-12);
    assert_plane_plane_branch_endpoints(&hit, &a, &b);
}

#[test]
fn plane_plane_window_boundary_contact_is_point() {
    let a = horizontal_plane(0.0);
    let b = vertical_plane_x(0.0);
    let hit = intersect_bounded_planes(
        &a,
        [ParamRange::new(-1.0, 1.0), ParamRange::new(0.0, 1.0)],
        &b,
        [ParamRange::new(1.0, 2.0), ParamRange::new(-1.0, 1.0)],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.curves.is_empty());
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 1.0, 0.0)) < 1e-12);
    assert_eq!(hit.points[0].uv_a, [0.0, 1.0]);
    assert_eq!(hit.points[0].uv_b, [1.0, 0.0]);
}

#[test]
fn plane_plane_parallel_miss_and_coincident_cases() {
    let a = horizontal_plane(0.0);
    let miss = intersect_bounded_planes(
        &a,
        plane_window(),
        &horizontal_plane(2.0),
        plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let err = intersect_bounded_planes(
        &a,
        plane_window(),
        &a,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "coincident plane/plane intersection is a surface overlap"
        }
    );
}

#[test]
fn surface_surface_dispatches_plane_plane_both_orders() {
    let a = horizontal_plane(0.0);
    let b = vertical_plane_x(0.0);
    let hit = intersect_bounded_surfaces(
        &a,
        plane_window(),
        &b,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 1);

    let swapped = intersect_bounded_surfaces(
        &b,
        plane_window(),
        &a,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), 1);
    let same_orientation = swapped.curves[0].uv_a_start == hit.curves[0].uv_b_start
        && swapped.curves[0].uv_b_start == hit.curves[0].uv_a_start;
    let reversed_orientation = swapped.curves[0].uv_a_start == hit.curves[0].uv_b_end
        && swapped.curves[0].uv_b_start == hit.curves[0].uv_a_end;
    assert!(same_orientation || reversed_orientation);
}

#[test]
fn plane_sphere_secant_returns_bounded_circle_branch() {
    let plane = horizontal_plane(0.5);
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_plane_sphere(
        &plane,
        plane_window(),
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();

    let r = 3.0_f64.sqrt() / 2.0;
    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert_eq!(hit.curves[0].kind, ContactKind::Transverse);
    assert!(hit.curves[0].curve_range.lo.abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_eq!(hit.curves[0].uv_a_start, [r, 0.0]);
    assert_eq!(hit.curves[0].uv_a_end[0], r);
    assert!(hit.curves[0].uv_a_end[1].abs() < 1e-12);
    assert_eq!(hit.curves[0].uv_b_start[0], 0.0);
    assert!((hit.curves[0].uv_b_start[1] - core::f64::consts::FRAC_PI_6).abs() < 1e-12);
    assert_eq!(hit.curves[0].uv_b_end[0], 0.0);
    assert!((hit.curves[0].uv_b_end[1] - core::f64::consts::FRAC_PI_6).abs() < 1e-12);

    let SurfaceIntersectionCurve::Circle(circle) = &hit.curves[0].curve else {
        panic!("plane/sphere secant should be carried by a circle");
    };
    assert!(circle.frame().origin().dist(Point3::new(0.0, 0.0, 0.5)) < 1e-12);
    assert!((circle.radius() - r).abs() < 1e-12);
    assert!(circle.eval(0.0).dist(Point3::new(r, 0.0, 0.5)) < 1e-12);
}

#[test]
fn plane_sphere_surface_windows_clip_circle_branch() {
    let plane = horizontal_plane(0.5);
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_plane_sphere(
        &plane,
        plane_window(),
        &sphere,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
        ],
        Tolerances::default(),
    )
    .unwrap();

    let r = 3.0_f64.sqrt() / 2.0;
    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!(hit.curves[0].curve_range.lo.abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(hit.curves[0].uv_a_start, [r, 0.0]);
    assert!((hit.curves[0].uv_a_end[0] + r).abs() < 1e-12);
    assert!(hit.curves[0].uv_a_end[1].abs() < 1e-12);
    assert_eq!(hit.curves[0].uv_b_start[0], 0.0);
    assert!((hit.curves[0].uv_b_end[0] - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn plane_sphere_tangent_and_miss_cases() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let tangent_plane = Plane::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
    );
    let tangent = intersect_bounded_plane_sphere(
        &tangent_plane,
        plane_window(),
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(tangent.curves.is_empty());
    assert_eq!(tangent.points.len(), 1);
    assert_eq!(tangent.points[0].kind, ContactKind::Tangent);
    assert!(tangent.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert_eq!(tangent.points[0].uv_a, [0.0, 0.0]);
    assert_eq!(tangent.points[0].uv_b, [0.0, 0.0]);

    let miss = intersect_bounded_plane_sphere(
        &horizontal_plane(2.0),
        plane_window(),
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn surface_surface_dispatches_plane_sphere_and_rejects_unsupported() {
    let plane = horizontal_plane(0.5);
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_surfaces(
        &plane,
        plane_window(),
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 1);

    let swapped = intersect_bounded_surfaces(
        &sphere,
        sphere_window(),
        &plane,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), 1);
    assert_eq!(swapped.curves[0].uv_a_start, hit.curves[0].uv_b_start);
    assert_eq!(swapped.curves[0].uv_b_start, hit.curves[0].uv_a_start);

    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let err = intersect_bounded_surfaces(
        &sphere,
        sphere_window(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "unsupported surface/surface intersection class"
        }
    );
}

#[test]
fn plane_cylinder_perpendicular_cut_returns_circle_branch() {
    let plane = horizontal_plane(0.5);
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_plane_cylinder(
        &plane,
        wide_plane_window(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert_eq!(hit.curves[0].kind, ContactKind::Transverse);
    assert!(hit.curves[0].curve_range.lo.abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_plane_cylinder_branch_endpoints(&hit, &plane, &cylinder);

    let SurfaceIntersectionCurve::Circle(circle) = &hit.curves[0].curve else {
        panic!("perpendicular plane/cylinder cut should be a circle");
    };
    assert!(circle.frame().origin().dist(Point3::new(0.0, 0.0, 0.5)) < 1e-12);
    assert!((circle.radius() - 1.0).abs() < 1e-12);
}

#[test]
fn plane_cylinder_surface_windows_clip_circle_branch() {
    let plane = horizontal_plane(0.5);
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_plane_cylinder(
        &plane,
        wide_plane_window(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!((total_curve_width(&hit) - core::f64::consts::PI).abs() < 1e-12);
    assert_plane_cylinder_branch_endpoints(&hit, &plane, &cylinder);
}

#[test]
fn plane_cylinder_oblique_cut_returns_ellipse_branch() {
    let plane = oblique_cylinder_plane();
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_plane_cylinder(
        &plane,
        wide_plane_window(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert!(!hit.curves.is_empty());
    assert!((total_curve_width(&hit) - core::f64::consts::TAU).abs() < 1e-12);
    assert_plane_cylinder_branch_endpoints(&hit, &plane, &cylinder);
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Ellipse(ellipse) = &branch.curve else {
            panic!("oblique plane/cylinder cut should be an ellipse");
        };
        assert!(ellipse.frame().origin().dist(Point3::new(0.0, 0.0, 0.0)) < 1e-12);
        assert!((ellipse.major_radius() - 5.0_f64.sqrt() / 2.0).abs() < 1e-12);
        assert!((ellipse.minor_radius() - 1.0).abs() < 1e-12);
    }
}

#[test]
fn plane_cylinder_parallel_cut_returns_ruling_lines() {
    let plane = vertical_plane_y(0.0);
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_plane_cylinder(
        &plane,
        wide_plane_window(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 4.0).abs() < 1e-12);
    assert_plane_cylinder_branch_endpoints(&hit, &plane, &cylinder);
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Line(line) = &branch.curve else {
            panic!("axis-parallel plane/cylinder cut should be ruling lines");
        };
        assert!(line.dir().cross(Vec3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
    }
}

#[test]
fn plane_cylinder_tangent_and_miss_cases() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let tangent_plane = vertical_plane_x(1.0);
    let tangent = intersect_bounded_plane_cylinder(
        &tangent_plane,
        wide_plane_window(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(tangent.points.is_empty());
    assert_eq!(tangent.curves.len(), 1);
    assert_eq!(tangent.curves[0].kind, ContactKind::Tangent);
    assert_plane_cylinder_branch_endpoints(&tangent, &tangent_plane, &cylinder);

    let miss = intersect_bounded_plane_cylinder(
        &vertical_plane_x(2.0),
        wide_plane_window(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn surface_surface_dispatches_plane_cylinder_both_orders() {
    let plane = horizontal_plane(0.5);
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_surfaces(
        &plane,
        wide_plane_window(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 1);

    let swapped = intersect_bounded_surfaces(
        &cylinder,
        cylinder_window(),
        &plane,
        wide_plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), 1);
    assert_eq!(swapped.curves[0].uv_a_start, hit.curves[0].uv_b_start);
    assert_eq!(swapped.curves[0].uv_b_start, hit.curves[0].uv_a_start);
}

#[test]
fn sphere_sphere_secant_returns_bounded_circle_branch() {
    let a = Sphere::new(Frame::world(), 1.0).unwrap();
    let b = Sphere::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let hit = intersect_bounded_spheres(
        &a,
        sphere_window(),
        &b,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();

    let r = 3.0_f64.sqrt() / 2.0;
    assert_sphere_sphere_circle_segments(&hit, &a, &b, 3, core::f64::consts::TAU);

    let SurfaceIntersectionCurve::Circle(circle) = &hit.curves[0].curve else {
        panic!("sphere/sphere secant should be carried by a circle");
    };
    assert!(circle.frame().origin().dist(Point3::new(0.5, 0.0, 0.0)) < 1e-12);
    assert!((circle.radius() - r).abs() < 1e-12);
    assert!(circle.eval(0.0).dist(Point3::new(0.5, r, 0.0)) < 1e-12);
}

#[test]
fn sphere_sphere_surface_windows_clip_circle_branch() {
    let a = Sphere::new(Frame::world(), 1.0).unwrap();
    let b = Sphere::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let hit = intersect_bounded_spheres(
        &a,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
        ],
        &b,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert_sphere_sphere_circle_segments(&hit, &a, &b, 2, core::f64::consts::PI);
    for branch in &hit.curves {
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("sphere/sphere secant should be carried by circle segments");
        };
        let midpoint = circle.eval((branch.curve_range.lo + branch.curve_range.hi) / 2.0);
        assert!(midpoint.z >= -1e-12);
    }
}

#[test]
fn sphere_sphere_tangent_miss_and_coincident_cases() {
    let a = Sphere::new(Frame::world(), 1.0).unwrap();
    let tangent_b = Sphere::new(
        Frame::new(
            Point3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let tangent = intersect_bounded_spheres(
        &a,
        sphere_window(),
        &tangent_b,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(tangent.curves.is_empty());
    assert_eq!(tangent.points.len(), 1);
    assert_eq!(tangent.points[0].kind, ContactKind::Tangent);
    assert!(tangent.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert_eq!(tangent.points[0].uv_a, [0.0, 0.0]);
    assert!((tangent.points[0].uv_b[0] - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(tangent.points[0].uv_b[1], 0.0);

    let miss_b = Sphere::new(
        Frame::new(
            Point3::new(3.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let miss = intersect_bounded_spheres(
        &a,
        sphere_window(),
        &miss_b,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let err = intersect_bounded_spheres(
        &a,
        sphere_window(),
        &a,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "coincident sphere/sphere intersection is a surface overlap"
        }
    );
}

#[test]
fn surface_surface_dispatches_sphere_sphere() {
    let a = Sphere::new(Frame::world(), 1.0).unwrap();
    let b = Sphere::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let hit = intersect_bounded_surfaces(
        &a,
        sphere_window(),
        &b,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 3);
    assert!(hit.points.is_empty());
}
