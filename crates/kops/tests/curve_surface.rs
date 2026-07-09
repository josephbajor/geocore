//! Bounded analytic curve/surface intersection behavior.

use kcore::error::Error;
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane, Sphere, Surface};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, intersect_bounded_curve_surface, intersect_bounded_line_plane,
    intersect_bounded_line_sphere,
};

fn make_line(origin: [f64; 3], direction: [f64; 3]) -> Line {
    Line::new(Point3::from_array(origin), Vec3::from_array(direction)).unwrap()
}

fn plane_window() -> [ParamRange; 2] {
    [ParamRange::new(-1.0, 1.0), ParamRange::new(-0.5, 0.5)]
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
    let err = intersect_bounded_curve_surface(
        &line,
        ParamRange::new(0.0, 2.0),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-1.0, 1.0),
        ],
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
