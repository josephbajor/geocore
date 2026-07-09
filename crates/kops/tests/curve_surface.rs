//! Bounded analytic curve/surface intersection behavior.

use kcore::error::Error;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, intersect_bounded_circle_cone, intersect_bounded_circle_cylinder,
    intersect_bounded_circle_plane, intersect_bounded_circle_sphere,
    intersect_bounded_circle_torus, intersect_bounded_curve_surface,
    intersect_bounded_ellipse_cone, intersect_bounded_ellipse_cylinder,
    intersect_bounded_ellipse_plane, intersect_bounded_ellipse_sphere,
    intersect_bounded_ellipse_torus, intersect_bounded_line_cone, intersect_bounded_line_cylinder,
    intersect_bounded_line_plane, intersect_bounded_line_sphere, intersect_bounded_line_torus,
};

fn make_line(origin: [f64; 3], direction: [f64; 3]) -> Line {
    Line::new(Point3::from_array(origin), Vec3::from_array(direction)).unwrap()
}

fn plane_window() -> [ParamRange; 2] {
    [ParamRange::new(-1.0, 1.0), ParamRange::new(-0.5, 0.5)]
}

fn cylinder_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-1.0, 1.0),
    ]
}

fn cone_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-1.0, 1.0),
    ]
}

fn torus_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(0.0, core::f64::consts::TAU),
    ]
}

fn vertical_conic_frame() -> Frame {
    Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

fn horizontal_frame(origin: [f64; 3]) -> Frame {
    Frame::new(
        Point3::from_array(origin),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

#[test]
fn line_plane_transverse_hit_and_window_miss() {
    let line = make_line([0.0, 0.0, -1.0], [0.0, 0.0, 1.0]);
    let plane = Plane::new(Frame::world());
    let hit = intersect_bounded_line_plane(
        &line,
        ParamRange::new(0.0, 2.0),
        &plane,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert_eq!(hit.points[0].point, Point3::new(0.0, 0.0, 0.0));
    assert!((hit.points[0].t_curve - 1.0).abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface, [0.0, 0.0]);

    let miss = intersect_bounded_line_plane(
        &make_line([2.0, 0.0, -1.0], [0.0, 0.0, 1.0]),
        ParamRange::new(0.0, 2.0),
        &plane,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn line_in_plane_clips_to_surface_window_overlap() {
    let line = make_line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let plane = Plane::new(Frame::world());
    let hit = intersect_bounded_line_plane(
        &line,
        ParamRange::new(0.0, 4.0),
        &plane,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].curve, ParamRange::new(1.0, 3.0));
    assert_eq!(hit.overlaps[0].uv_start, [-1.0, 0.0]);
    assert_eq!(hit.overlaps[0].uv_end, [1.0, 0.0]);
}

#[test]
fn line_sphere_secant_and_tangent_contacts() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let secant = make_line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_sphere(
        &secant,
        ParamRange::new(0.0, 4.0),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(-1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - 1.0).abs() < 1e-12);
    assert!(hit.points[1].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[1].t_curve - 3.0).abs() < 1e-12);

    for (height, tolerances) in [
        (1.0, Tolerances::default()),
        (1.0 + 5e-7, Tolerances::with_linear(1e-6).unwrap()),
    ] {
        let tangent = make_line([-2.0, height, 0.0], [1.0, 0.0, 0.0]);
        let hit = intersect_bounded_line_sphere(
            &tangent,
            ParamRange::new(0.0, 4.0),
            &sphere,
            sphere.param_range(),
            tolerances,
        )
        .unwrap();
        assert_eq!(hit.points.len(), 1);
        assert_eq!(hit.points[0].kind, ContactKind::Tangent);
        assert!(hit.points[0].residual <= tolerances.linear());
    }
}

#[test]
fn line_sphere_surface_range_filters_contacts() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let line = make_line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_sphere(
        &line,
        ParamRange::new(0.0, 4.0),
        &sphere,
        [
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - 3.0).abs() < 1e-12);
}

#[test]
fn circle_sphere_secant_tangent_and_surface_window_filtering() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let secant = Circle::new(horizontal_frame([0.5, 0.0, 0.0]), 1.0).unwrap();
    let hit = intersect_bounded_circle_sphere(
        &secant,
        secant.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    let y = 15.0_f64.sqrt() / 4.0;
    let t = math::atan2(y, -0.25);
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(0.25, y, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - t).abs() < 1e-12);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(0.25, -y, 0.0)) < 1e-12);
    assert!((hit.points[1].t_curve - (core::f64::consts::TAU - t)).abs() < 1e-12);

    let filtered = intersect_bounded_circle_sphere(
        &secant,
        secant.param_range(),
        &sphere,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(0.25, y, 0.0)) < 1e-12);

    let tangent = Circle::new(horizontal_frame([1.5, 0.0, 0.0]), 0.5).unwrap();
    let hit = intersect_bounded_circle_sphere(
        &tangent,
        tangent.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn circle_on_sphere_clips_overlap_to_surface_window() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_circle_sphere(
        &circle,
        circle.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start, [0.0, 0.0]);
    assert_eq!(hit.overlaps[0].uv_end, [0.0, 0.0]);

    let clipped = intersect_bounded_circle_sphere(
        &circle,
        circle.param_range(),
        &sphere,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(clipped.points.is_empty());
    assert_eq!(clipped.overlaps.len(), 1);
    assert!(clipped.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((clipped.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_start, [0.0, 0.0]);
    assert!((clipped.overlaps[0].uv_end[0] - core::f64::consts::PI).abs() < 1e-12);
    assert!(clipped.overlaps[0].uv_end[1].abs() < 1e-12);

    let miss = intersect_bounded_circle_sphere(
        &Circle::new(Frame::world(), 0.5).unwrap(),
        circle.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn ellipse_sphere_secant_tangent_and_surface_window_filtering() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let ellipse = Ellipse::new(horizontal_frame([0.5, 0.0, 0.0]), 1.0, 0.5).unwrap();
    let hit = intersect_bounded_ellipse_sphere(
        &ellipse,
        ellipse.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    let cos_t = (-2.0 + 10.0_f64.sqrt()) / 3.0;
    let sin_t = (1.0 - cos_t * cos_t).sqrt();
    let x = 0.5 + cos_t;
    let y = 0.5 * sin_t;
    let t = math::atan2(sin_t, cos_t);
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(x, y, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - t).abs() < 1e-12);
    assert!((hit.points[0].uv_surface[0] - math::atan2(y, x)).abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface[1], 0.0);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(x, -y, 0.0)) < 1e-12);
    assert!((hit.points[1].t_curve - (core::f64::consts::TAU - t)).abs() < 1e-12);

    let filtered = intersect_bounded_ellipse_sphere(
        &ellipse,
        ellipse.param_range(),
        &sphere,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(x, y, 0.0)) < 1e-12);

    let tangent = Ellipse::new(horizontal_frame([1.5, 0.0, 0.0]), 0.5, 0.25).unwrap();
    let hit = intersect_bounded_ellipse_sphere(
        &tangent,
        tangent.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn circle_as_ellipse_on_sphere_clips_overlap_to_surface_window() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let ellipse = Ellipse::new(Frame::world(), 1.0, 1.0).unwrap();
    let hit = intersect_bounded_ellipse_sphere(
        &ellipse,
        ellipse.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start, [0.0, 0.0]);
    assert_eq!(hit.overlaps[0].uv_end, [0.0, 0.0]);

    let clipped = intersect_bounded_ellipse_sphere(
        &ellipse,
        ellipse.param_range(),
        &sphere,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(clipped.points.is_empty());
    assert_eq!(clipped.overlaps.len(), 1);
    assert!(clipped.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((clipped.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_start, [0.0, 0.0]);
    assert!((clipped.overlaps[0].uv_end[0] - core::f64::consts::PI).abs() < 1e-12);
    assert!(clipped.overlaps[0].uv_end[1].abs() < 1e-12);

    let miss = intersect_bounded_ellipse_sphere(
        &Ellipse::new(Frame::world(), 0.5, 0.25).unwrap(),
        ellipse.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn line_cylinder_secant_and_tangent_contacts() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let secant = make_line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_cylinder(
        &secant,
        ParamRange::new(0.0, 4.0),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(-1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - 1.0).abs() < 1e-12);
    assert!((hit.points[0].uv_surface[0] - core::f64::consts::PI).abs() < 1e-12);
    assert!(hit.points[1].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[1].t_curve - 3.0).abs() < 1e-12);
    assert_eq!(hit.points[1].uv_surface, [0.0, 0.0]);

    for (height, tolerances) in [
        (1.0, Tolerances::default()),
        (1.0 + 5e-7, Tolerances::with_linear(1e-6).unwrap()),
    ] {
        let tangent = make_line([-2.0, height, 0.0], [1.0, 0.0, 0.0]);
        let hit = intersect_bounded_line_cylinder(
            &tangent,
            ParamRange::new(0.0, 4.0),
            &cylinder,
            cylinder_window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(hit.points.len(), 1);
        assert_eq!(hit.points[0].kind, ContactKind::Tangent);
        assert!((hit.points[0].t_curve - 2.0).abs() < 1e-12);
        assert!(hit.points[0].residual <= tolerances.linear());
    }
}

#[test]
fn line_cylinder_surface_range_filters_contacts() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let line = make_line([-2.0, 0.0, 0.25], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_cylinder(
        &line,
        ParamRange::new(0.0, 4.0),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.25)) < 1e-12);
    assert!((hit.points[0].t_curve - 3.0).abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface, [0.0, 0.25]);
}

#[test]
fn line_on_cylinder_ruling_clips_to_surface_window_overlap() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let ruling = make_line([1.0, 0.0, -2.0], [0.0, 0.0, 1.0]);
    let hit = intersect_bounded_line_cylinder(
        &ruling,
        ParamRange::new(0.0, 4.0),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-0.5, 0.75),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!((hit.overlaps[0].curve.lo - 1.5).abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - 2.75).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start[0], 0.0);
    assert!((hit.overlaps[0].uv_start[1] + 0.5).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_end[0], 0.0);
    assert!((hit.overlaps[0].uv_end[1] - 0.75).abs() < 1e-12);

    let endpoint = intersect_bounded_line_cylinder(
        &ruling,
        ParamRange::new(0.0, 1.5),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-0.5, 0.75),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(endpoint.points.len(), 1);
    assert_eq!(endpoint.points[0].kind, ContactKind::Tangent);
    assert_eq!(endpoint.points[0].uv_surface, [0.0, -0.5]);
    assert!(endpoint.overlaps.is_empty());
}

#[test]
fn circle_cylinder_secant_tangent_and_surface_window_filtering() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let secant = Circle::new(horizontal_frame([0.5, 0.0, 0.25]), 1.0).unwrap();
    let hit = intersect_bounded_circle_cylinder(
        &secant,
        secant.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    let y = 15.0_f64.sqrt() / 4.0;
    let t = math::atan2(y, -0.25);
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(0.25, y, 0.25)) < 1e-12);
    assert!((hit.points[0].t_curve - t).abs() < 1e-12);
    assert!((hit.points[0].uv_surface[0] - math::atan2(y, 0.25)).abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface[1], 0.25);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(0.25, -y, 0.25)) < 1e-12);
    assert!((hit.points[1].t_curve - (core::f64::consts::TAU - t)).abs() < 1e-12);

    let filtered = intersect_bounded_circle_cylinder(
        &secant,
        secant.param_range(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(0.25, y, 0.25)) < 1e-12);

    let tangent = Circle::new(horizontal_frame([1.5, 0.0, 0.0]), 0.5).unwrap();
    let hit = intersect_bounded_circle_cylinder(
        &tangent,
        tangent.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn circle_on_cylinder_clips_overlap_to_surface_window() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_circle_cylinder(
        &circle,
        circle.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start, [0.0, 0.0]);
    assert_eq!(hit.overlaps[0].uv_end, [0.0, 0.0]);

    let clipped = intersect_bounded_circle_cylinder(
        &circle,
        circle.param_range(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(clipped.points.is_empty());
    assert_eq!(clipped.overlaps.len(), 1);
    assert!(clipped.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((clipped.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_start, [0.0, 0.0]);
    assert!((clipped.overlaps[0].uv_end[0] - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_end[1], 0.0);

    let miss = intersect_bounded_circle_cylinder(
        &circle,
        circle.param_range(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(1.0, 2.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn ellipse_cylinder_secant_tangent_and_surface_window_filtering() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let ellipse = Ellipse::new(horizontal_frame([0.5, 0.0, 0.25]), 1.0, 0.5).unwrap();
    let hit = intersect_bounded_ellipse_cylinder(
        &ellipse,
        ellipse.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    let cos_t = (-2.0 + 10.0_f64.sqrt()) / 3.0;
    let sin_t = (1.0 - cos_t * cos_t).sqrt();
    let x = 0.5 + cos_t;
    let y = 0.5 * sin_t;
    let t = math::atan2(sin_t, cos_t);
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(x, y, 0.25)) < 1e-12);
    assert!((hit.points[0].t_curve - t).abs() < 1e-12);
    assert!((hit.points[0].uv_surface[0] - math::atan2(y, x)).abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface[1], 0.25);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(x, -y, 0.25)) < 1e-12);
    assert!((hit.points[1].t_curve - (core::f64::consts::TAU - t)).abs() < 1e-12);

    let filtered = intersect_bounded_ellipse_cylinder(
        &ellipse,
        ellipse.param_range(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(x, y, 0.25)) < 1e-12);

    let tangent = Ellipse::new(horizontal_frame([1.5, 0.0, 0.0]), 0.5, 0.25).unwrap();
    let hit = intersect_bounded_ellipse_cylinder(
        &tangent,
        tangent.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn ellipse_on_cylinder_clips_oblique_overlap_to_surface_window() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let slope = 0.5;
    let major = (1.0_f64 + slope * slope).sqrt();
    let x_axis = Vec3::new(1.0, 0.0, slope).normalized().unwrap();
    let y_axis = Vec3::new(0.0, 1.0, 0.0);
    let frame = Frame::new(Point3::new(0.0, 0.0, 0.0), x_axis.cross(y_axis), x_axis).unwrap();
    let ellipse = Ellipse::new(frame, major, 1.0).unwrap();
    let hit = intersect_bounded_ellipse_cylinder(
        &ellipse,
        ellipse.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start[0], 0.0);
    assert!((hit.overlaps[0].uv_start[1] - slope).abs() < 1e-12);
    assert!(hit.overlaps[0].uv_end[0].abs() < 1e-12);
    assert!((hit.overlaps[0].uv_end[1] - slope).abs() < 1e-12);

    let clipped = intersect_bounded_ellipse_cylinder(
        &ellipse,
        ellipse.param_range(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(clipped.points.is_empty());
    assert_eq!(clipped.overlaps.len(), 1);
    assert!(clipped.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((clipped.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_start[0], 0.0);
    assert!((clipped.overlaps[0].uv_start[1] - slope).abs() < 1e-12);
    assert!((clipped.overlaps[0].uv_end[0] - core::f64::consts::PI).abs() < 1e-12);
    assert!((clipped.overlaps[0].uv_end[1] + slope).abs() < 1e-12);

    let miss = intersect_bounded_ellipse_cylinder(
        &ellipse,
        ellipse.param_range(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.75, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn circle_cone_secant_tangent_and_surface_window_filtering() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let secant = Circle::new(horizontal_frame([0.5, 0.0, 0.0]), 1.0).unwrap();
    let hit = intersect_bounded_circle_cone(
        &secant,
        secant.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();

    let y = 15.0_f64.sqrt() / 4.0;
    let t = math::atan2(y, -0.25);
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(0.25, y, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - t).abs() < 1e-12);
    assert!((hit.points[0].uv_surface[0] - math::atan2(y, 0.25)).abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface[1], 0.0);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(0.25, -y, 0.0)) < 1e-12);
    assert!((hit.points[1].t_curve - (core::f64::consts::TAU - t)).abs() < 1e-12);

    let filtered = intersect_bounded_circle_cone(
        &secant,
        secant.param_range(),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(0.25, y, 0.0)) < 1e-12);

    let tangent = Circle::new(horizontal_frame([1.5, 0.0, 0.0]), 0.5).unwrap();
    let hit = intersect_bounded_circle_cone(
        &tangent,
        tangent.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn circle_on_cone_clips_overlap_to_surface_window() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_circle_cone(
        &circle,
        circle.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start, [0.0, 0.0]);
    assert_eq!(hit.overlaps[0].uv_end, [0.0, 0.0]);

    let clipped = intersect_bounded_circle_cone(
        &circle,
        circle.param_range(),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(clipped.points.is_empty());
    assert_eq!(clipped.overlaps.len(), 1);
    assert!(clipped.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((clipped.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_start, [0.0, 0.0]);
    assert!((clipped.overlaps[0].uv_end[0] - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_end[1], 0.0);

    let miss = intersect_bounded_circle_cone(
        &circle,
        circle.param_range(),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.5, 1.5),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn ellipse_cone_secant_tangent_and_surface_window_filtering() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let ellipse = Ellipse::new(horizontal_frame([0.5, 0.0, 0.0]), 1.0, 0.5).unwrap();
    let hit = intersect_bounded_ellipse_cone(
        &ellipse,
        ellipse.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();

    let cos_t = (-2.0 + 10.0_f64.sqrt()) / 3.0;
    let sin_t = (1.0 - cos_t * cos_t).sqrt();
    let x = 0.5 + cos_t;
    let y = 0.5 * sin_t;
    let t = math::atan2(sin_t, cos_t);
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(x, y, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - t).abs() < 1e-12);
    assert!((hit.points[0].uv_surface[0] - math::atan2(y, x)).abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface[1], 0.0);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(x, -y, 0.0)) < 1e-12);
    assert!((hit.points[1].t_curve - (core::f64::consts::TAU - t)).abs() < 1e-12);

    let filtered = intersect_bounded_ellipse_cone(
        &ellipse,
        ellipse.param_range(),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(x, y, 0.0)) < 1e-12);

    let tangent = Ellipse::new(horizontal_frame([1.5, 0.0, 0.0]), 0.5, 0.25).unwrap();
    let hit = intersect_bounded_ellipse_cone(
        &tangent,
        tangent.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn ellipse_on_cone_clips_oblique_overlap_to_surface_window() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let slope = 0.5;
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let tan_a = sin_a / cos_a;
    let k = slope * tan_a;
    let axial = 1.0 - k * k;
    let center_x = cone.radius() * k / axial;
    let x_radius = cone.radius() / axial;
    let y_radius = cone.radius() / axial.sqrt();
    let major = x_radius * (1.0_f64 + slope * slope).sqrt();
    let x_axis = Vec3::new(1.0, 0.0, slope).normalized().unwrap();
    let y_axis = Vec3::new(0.0, 1.0, 0.0);
    let frame = Frame::new(
        Point3::new(center_x, 0.0, slope * center_x),
        x_axis.cross(y_axis),
        x_axis,
    )
    .unwrap();
    let ellipse = Ellipse::new(frame, major, y_radius).unwrap();
    let hit = intersect_bounded_ellipse_cone(
        &ellipse,
        ellipse.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();

    let v_start = slope * (center_x + x_radius) / cos_a;
    let v_mid = slope * (center_x - x_radius) / cos_a;
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start[0], 0.0);
    assert!((hit.overlaps[0].uv_start[1] - v_start).abs() < 1e-12);
    assert!(hit.overlaps[0].uv_end[0].abs() < 1e-12);
    assert!((hit.overlaps[0].uv_end[1] - v_start).abs() < 1e-12);

    let clipped = intersect_bounded_ellipse_cone(
        &ellipse,
        ellipse.param_range(),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(clipped.points.is_empty());
    assert_eq!(clipped.overlaps.len(), 1);
    assert!(clipped.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((clipped.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_start[0], 0.0);
    assert!((clipped.overlaps[0].uv_start[1] - v_start).abs() < 1e-12);
    assert!((clipped.overlaps[0].uv_end[0] - core::f64::consts::PI).abs() < 1e-12);
    assert!((clipped.overlaps[0].uv_end[1] - v_mid).abs() < 1e-12);

    let miss = intersect_bounded_ellipse_cone(
        &ellipse,
        ellipse.param_range(),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.9, 1.1),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn line_cone_secant_tangent_and_apex_contacts() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let secant = make_line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_cone(
        &secant,
        ParamRange::new(0.0, 4.0),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(-1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[0].t_curve - 1.0).abs() < 1e-12);
    assert!((hit.points[0].uv_surface[0] - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface[1], 0.0);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[1].t_curve - 3.0).abs() < 1e-12);
    assert_eq!(hit.points[1].uv_surface, [0.0, 0.0]);

    for (height, tolerances) in [
        (1.0, Tolerances::default()),
        (1.0 + 5e-7, Tolerances::with_linear(1e-6).unwrap()),
    ] {
        let tangent = make_line([-2.0, height, 0.0], [1.0, 0.0, 0.0]);
        let hit = intersect_bounded_line_cone(
            &tangent,
            ParamRange::new(0.0, 4.0),
            &cone,
            cone_window(),
            tolerances,
        )
        .unwrap();
        assert_eq!(hit.points.len(), 1);
        assert_eq!(hit.points[0].kind, ContactKind::Tangent);
        assert!((hit.points[0].t_curve - 2.0).abs() < 1e-12);
        assert!(hit.points[0].residual <= tolerances.linear());
    }

    let apex_line = make_line(cone.apex().to_array(), [0.0, 0.0, 1.0]);
    let apex = intersect_bounded_line_cone(
        &apex_line,
        ParamRange::new(0.0, 0.0),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(cone.apex_v(), cone.apex_v()),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(apex.points.len(), 1);
    assert_eq!(apex.points[0].kind, ContactKind::Singular);
    assert_eq!(apex.points[0].uv_surface, [0.0, cone.apex_v()]);
}

#[test]
fn line_cone_surface_range_filters_contacts() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let line = make_line([-2.0, 0.0, 0.25], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_cone(
        &line,
        ParamRange::new(0.0, 4.0),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    let expected_x = 1.0 + 0.25 / 3.0_f64.sqrt();
    assert!(hit.points[0].point.dist(Point3::new(expected_x, 0.0, 0.25)) < 1e-12);
    assert!((hit.points[0].t_curve - (2.0 + expected_x)).abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface[0], 0.0);
}

#[test]
fn line_on_cone_ruling_clips_to_surface_window_overlap() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let ruling = make_line(cone.apex().to_array(), [0.5, 0.0, 3.0_f64.sqrt() / 2.0]);
    let hit = intersect_bounded_line_cone(
        &ruling,
        ParamRange::new(0.0, 4.0),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-0.5, 0.75),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!((hit.overlaps[0].curve.lo - 1.5).abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - 2.75).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start[0], 0.0);
    assert!((hit.overlaps[0].uv_start[1] + 0.5).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_end[0], 0.0);
    assert!((hit.overlaps[0].uv_end[1] - 0.75).abs() < 1e-12);

    let apex = intersect_bounded_line_cone(
        &ruling,
        ParamRange::new(0.0, 0.0),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(cone.apex_v(), cone.apex_v()),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(apex.points.len(), 1);
    assert_eq!(apex.points[0].kind, ContactKind::Singular);
    assert!(apex.overlaps.is_empty());
}

#[test]
fn line_torus_equatorial_secant_returns_four_contacts() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let secant = make_line([-3.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_torus(
        &secant,
        ParamRange::new(0.0, 6.0),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 4);
    for (point, expected_t, expected_x) in [
        (&hit.points[0], 0.5, -2.5),
        (&hit.points[1], 1.5, -1.5),
        (&hit.points[2], 4.5, 1.5),
        (&hit.points[3], 5.5, 2.5),
    ] {
        assert_eq!(point.kind, ContactKind::Transverse);
        assert!((point.t_curve - expected_t).abs() < 1e-9);
        assert!(point.point.dist(Point3::new(expected_x, 0.0, 0.0)) < 1e-9);
    }
    assert!((hit.points[0].uv_surface[0] - core::f64::consts::PI).abs() < 1e-9);
    assert!((hit.points[0].uv_surface[1]).abs() < 1e-9);
    assert!((hit.points[1].uv_surface[0] - core::f64::consts::PI).abs() < 1e-9);
    assert!((hit.points[1].uv_surface[1] - core::f64::consts::PI).abs() < 1e-9);
    assert_eq!(hit.points[2].uv_surface[0], 0.0);
    assert!((hit.points[2].uv_surface[1] - core::f64::consts::PI).abs() < 1e-9);
    assert_eq!(hit.points[3].uv_surface, [0.0, 0.0]);
}

#[test]
fn line_torus_tangent_and_near_tangent_contacts() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    for (height, tolerances) in [
        (2.5, Tolerances::default()),
        (2.5 + 5e-7, Tolerances::with_linear(1e-6).unwrap()),
    ] {
        let tangent = make_line([-3.0, height, 0.0], [1.0, 0.0, 0.0]);
        let hit = intersect_bounded_line_torus(
            &tangent,
            ParamRange::new(0.0, 6.0),
            &torus,
            torus_window(),
            tolerances,
        )
        .unwrap();

        assert_eq!(hit.points.len(), 1);
        assert_eq!(hit.points[0].kind, ContactKind::Tangent);
        assert!((hit.points[0].t_curve - 3.0).abs() < 1e-9);
        assert!(hit.points[0].point.dist(Point3::new(0.0, 2.5, 0.0)) <= tolerances.linear());
        assert!((hit.points[0].uv_surface[0] - core::f64::consts::FRAC_PI_2).abs() < 1e-9);
        assert_eq!(hit.points[0].uv_surface[1], 0.0);
    }
}

#[test]
fn line_torus_surface_range_filters_contacts() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let line = make_line([-3.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let hit = intersect_bounded_line_torus(
        &line,
        ParamRange::new(0.0, 6.0),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 1);
    assert!(hit.points[0].point.dist(Point3::new(2.5, 0.0, 0.0)) < 1e-9);
    assert!((hit.points[0].t_curve - 5.5).abs() < 1e-9);
    assert_eq!(hit.points[0].uv_surface, [0.0, 0.0]);
}

#[test]
fn circle_torus_secant_tangent_and_surface_window_filtering() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let secant = Circle::new(horizontal_frame([1.0, 0.0, 0.0]), 1.0).unwrap();
    let hit = intersect_bounded_circle_torus(
        &secant,
        secant.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    let x = 9.0 / 8.0;
    let y = 63.0_f64.sqrt() / 8.0;
    let t = math::atan2(y, 1.0 / 8.0);
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(x, y, 0.0)) < 1e-9);
    assert!((hit.points[0].t_curve - t).abs() < 1e-9);
    assert!((hit.points[0].uv_surface[0] - math::atan2(y, x)).abs() < 1e-9);
    assert!((hit.points[0].uv_surface[1] - core::f64::consts::PI).abs() < 1e-9);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(x, -y, 0.0)) < 1e-9);
    assert!((hit.points[1].t_curve - (core::f64::consts::TAU - t)).abs() < 1e-9);

    let filtered = intersect_bounded_circle_torus(
        &secant,
        secant.param_range(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(x, y, 0.0)) < 1e-9);

    let tangent = Circle::new(horizontal_frame([3.0, 0.0, 0.0]), 0.5).unwrap();
    let hit = intersect_bounded_circle_torus(
        &tangent,
        tangent.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(2.5, 0.0, 0.0)) < 1e-9);
    assert!((hit.points[0].t_curve - core::f64::consts::PI).abs() < 1e-9);
    assert!(hit.points[0].uv_surface[0].abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface[1], 0.0);
}

#[test]
fn circle_on_torus_clips_latitude_and_tube_overlaps() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let latitude = Circle::new(Frame::world(), 2.5).unwrap();
    let hit = intersect_bounded_circle_torus(
        &latitude,
        latitude.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start, [0.0, 0.0]);
    assert_eq!(hit.overlaps[0].uv_end, [0.0, 0.0]);

    let clipped = intersect_bounded_circle_torus(
        &latitude,
        latitude.param_range(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(clipped.points.is_empty());
    assert_eq!(clipped.overlaps.len(), 1);
    assert!(clipped.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((clipped.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_start, [0.0, 0.0]);
    assert_eq!(clipped.overlaps[0].uv_end, [core::f64::consts::PI, 0.0]);

    let tube_frame = Frame::new(
        Point3::new(2.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let tube = Circle::new(tube_frame, 0.5).unwrap();
    let clipped = intersect_bounded_circle_torus(
        &tube,
        tube.param_range(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.0, core::f64::consts::PI),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(clipped.points.is_empty());
    assert_eq!(clipped.overlaps.len(), 1);
    assert!(clipped.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((clipped.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_start, [0.0, 0.0]);
    assert_eq!(clipped.overlaps[0].uv_end, [0.0, core::f64::consts::PI]);

    let miss = intersect_bounded_circle_torus(
        &latitude,
        latitude.param_range(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(core::f64::consts::FRAC_PI_2, core::f64::consts::PI),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn ellipse_torus_secant_tangent_and_surface_window_filtering() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let ellipse = Ellipse::new(horizontal_frame([1.0, 0.0, 0.0]), 1.0, 0.5).unwrap();
    let hit = intersect_bounded_ellipse_torus(
        &ellipse,
        ellipse.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    let cos_t = (-4.0 + 2.0 * 7.0_f64.sqrt()) / 3.0;
    let sin_t = (1.0 - cos_t * cos_t).sqrt();
    let x = 1.0 + cos_t;
    let y = 0.5 * sin_t;
    let t = math::atan2(sin_t, cos_t);
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(x, y, 0.0)) < 1e-9);
    assert!((hit.points[0].t_curve - t).abs() < 1e-9);
    assert!((hit.points[0].uv_surface[0] - math::atan2(y, x)).abs() < 1e-9);
    assert!((hit.points[0].uv_surface[1] - core::f64::consts::PI).abs() < 1e-9);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(x, -y, 0.0)) < 1e-9);
    assert!((hit.points[1].t_curve - (core::f64::consts::TAU - t)).abs() < 1e-9);

    let filtered = intersect_bounded_ellipse_torus(
        &ellipse,
        ellipse.param_range(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(x, y, 0.0)) < 1e-9);

    let tangent = Ellipse::new(horizontal_frame([3.0, 0.0, 0.0]), 0.5, 0.25).unwrap();
    let hit = intersect_bounded_ellipse_torus(
        &tangent,
        tangent.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(2.5, 0.0, 0.0)) < 1e-9);
    assert!((hit.points[0].t_curve - core::f64::consts::PI).abs() < 1e-9);
    assert!(hit.points[0].uv_surface[0].abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface[1], 0.0);
}

#[test]
fn circle_as_ellipse_on_torus_clips_latitude_and_tube_overlaps() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let latitude = Ellipse::new(Frame::world(), 2.5, 2.5).unwrap();
    let hit = intersect_bounded_ellipse_torus(
        &latitude,
        latitude.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start, [0.0, 0.0]);
    assert_eq!(hit.overlaps[0].uv_end, [0.0, 0.0]);

    let clipped = intersect_bounded_ellipse_torus(
        &latitude,
        latitude.param_range(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(clipped.points.is_empty());
    assert_eq!(clipped.overlaps.len(), 1);
    assert!(clipped.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((clipped.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_start, [0.0, 0.0]);
    assert_eq!(clipped.overlaps[0].uv_end, [core::f64::consts::PI, 0.0]);

    let tube_frame = Frame::new(
        Point3::new(2.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let tube = Ellipse::new(tube_frame, 0.5, 0.5).unwrap();
    let clipped = intersect_bounded_ellipse_torus(
        &tube,
        tube.param_range(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.0, core::f64::consts::PI),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(clipped.points.is_empty());
    assert_eq!(clipped.overlaps.len(), 1);
    assert!(clipped.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((clipped.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(clipped.overlaps[0].uv_start, [0.0, 0.0]);
    assert_eq!(clipped.overlaps[0].uv_end, [0.0, core::f64::consts::PI]);

    let miss = intersect_bounded_ellipse_torus(
        &latitude,
        latitude.param_range(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(core::f64::consts::FRAC_PI_2, core::f64::consts::PI),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn circle_plane_crossing_and_surface_window_filtering() {
    let circle = Circle::new(vertical_conic_frame(), 1.0).unwrap();
    let plane = Plane::new(Frame::world());
    let hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &plane,
        [ParamRange::new(-2.0, 2.0), ParamRange::new(-0.25, 0.25)],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!(hit.points[0].t_curve.abs() < 1e-12);
    assert_eq!(hit.points[0].uv_surface, [1.0, 0.0]);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(-1.0, 0.0, 0.0)) < 1e-12);
    assert!((hit.points[1].t_curve - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(hit.points[1].uv_surface, [-1.0, 0.0]);

    let filtered = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &plane,
        [ParamRange::new(0.0, 2.0), ParamRange::new(-0.25, 0.25)],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert_eq!(filtered.points[0].uv_surface, [1.0, 0.0]);
}

#[test]
fn ellipse_plane_crossing_and_surface_window_filtering() {
    let ellipse = Ellipse::new(vertical_conic_frame(), 2.0, 1.0).unwrap();
    let plane = Plane::new(Frame::world());
    let hit = intersect_bounded_ellipse_plane(
        &ellipse,
        ellipse.param_range(),
        &plane,
        [ParamRange::new(-3.0, 3.0), ParamRange::new(-0.25, 0.25)],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(2.0, 0.0, 0.0)) < 1e-12);
    assert_eq!(hit.points[0].uv_surface, [2.0, 0.0]);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(-2.0, 0.0, 0.0)) < 1e-12);
    assert_eq!(hit.points[1].uv_surface, [-2.0, 0.0]);

    let filtered = intersect_bounded_ellipse_plane(
        &ellipse,
        ellipse.param_range(),
        &plane,
        [ParamRange::new(0.0, 3.0), ParamRange::new(-0.25, 0.25)],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert_eq!(filtered.points[0].uv_surface, [2.0, 0.0]);
}

#[test]
fn circle_in_plane_clips_overlap_to_surface_window() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let plane = Plane::new(Frame::world());
    let hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &plane,
        [ParamRange::new(-2.0, 2.0), ParamRange::new(0.0, 2.0)],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert!(hit.overlaps[0].curve.lo.abs() < 1e-12);
    assert!((hit.overlaps[0].curve.hi - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(hit.overlaps[0].uv_start, [1.0, 0.0]);
    assert!((hit.overlaps[0].uv_end[0] + 1.0).abs() < 1e-12);
    assert!(hit.overlaps[0].uv_end[1].abs() < 1e-12);

    let offset_circle = Circle::new(
        Frame::new(
            Point3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let miss = intersect_bounded_circle_plane(
        &offset_circle,
        circle.param_range(),
        &plane,
        [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)],
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn curve_surface_dispatches_supported_cases_and_rejects_unsupported() {
    let line = make_line([0.0, 0.0, -1.0], [0.0, 0.0, 1.0]);
    let plane = Plane::new(Frame::world());
    let hit = intersect_bounded_curve_surface(
        &line,
        ParamRange::new(0.0, 2.0),
        &plane,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);

    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_curve_surface(
        &line,
        ParamRange::new(0.0, 2.0),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);

    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_curve_surface(
        &line,
        ParamRange::new(0.0, 2.0),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.is_empty());

    let cone = Cone::new(Frame::world(), 1.0, 0.25).unwrap();
    let hit = intersect_bounded_curve_surface(
        &line,
        ParamRange::new(0.0, 2.0),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.is_empty());

    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_curve_surface(
        &line,
        ParamRange::new(0.0, 2.0),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.is_empty());

    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_curve_surface(
        &circle,
        ParamRange::new(0.0, core::f64::consts::TAU),
        &plane,
        [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.overlaps.len(), 1);

    let ellipse = Ellipse::new(Frame::world(), 2.0, 1.0).unwrap();
    let hit = intersect_bounded_curve_surface(
        &ellipse,
        ParamRange::new(0.0, core::f64::consts::TAU),
        &plane,
        [ParamRange::new(-3.0, 3.0), ParamRange::new(-2.0, 2.0)],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.overlaps.len(), 1);

    let hit = intersect_bounded_curve_surface(
        &circle,
        circle.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.overlaps.len(), 1);

    let hit = intersect_bounded_curve_surface(
        &circle,
        circle.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.overlaps.len(), 1);

    let hit = intersect_bounded_curve_surface(
        &circle,
        circle.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.overlaps.len(), 1);

    let torus_circle = Circle::new(Frame::world(), 2.5).unwrap();
    let hit = intersect_bounded_curve_surface(
        &torus_circle,
        torus_circle.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.overlaps.len(), 1);

    let hit = intersect_bounded_curve_surface(
        &ellipse,
        ellipse.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);

    let hit = intersect_bounded_curve_surface(
        &ellipse,
        ellipse.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);

    let hit = intersect_bounded_curve_surface(
        &ellipse,
        ellipse.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);

    let torus_ellipse = Ellipse::new(Frame::world(), 2.5, 2.5).unwrap();
    let hit = intersect_bounded_curve_surface(
        &torus_ellipse,
        torus_ellipse.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.overlaps.len(), 1);

    let nurbs = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, -1.0), Point3::new(0.0, 0.0, 1.0)],
        None,
    )
    .unwrap();
    let err = intersect_bounded_curve_surface(
        &nurbs,
        nurbs.param_range(),
        &plane,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "unsupported curve/surface intersection class"
        }
    );
}
