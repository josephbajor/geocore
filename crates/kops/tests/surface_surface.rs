//! Bounded analytic surface/surface intersection behavior.

use kcore::error::Error;
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere, Surface};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceIntersections,
    intersect_bounded_plane_sphere, intersect_bounded_spheres, intersect_bounded_surfaces,
};

fn plane_window() -> [ParamRange; 2] {
    [ParamRange::new(-1.0, 1.0), ParamRange::new(-1.0, 1.0)]
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

    let err = intersect_bounded_surfaces(
        &plane,
        plane_window(),
        &horizontal_plane(0.25),
        plane_window(),
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
