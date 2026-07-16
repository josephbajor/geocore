//! Bounded curve/surface intersection behavior.

use std::error::Error as _;

use kcore::error::{ClassifiedError, Error, ErrorClass};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::aabb::Aabb3;
use kgeom::curve::{Circle, Curve, CurveDerivs, Ellipse, Line};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, SurfaceDerivs, Torus};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    CURVE_SURFACE_CLASS_PAIR, ContactKind, CurveClass, IntersectionError, SurfaceClass,
    UNSUPPORTED_CLASS_PAIR, intersect_bounded_circle_cone, intersect_bounded_circle_cylinder,
    intersect_bounded_circle_plane, intersect_bounded_circle_sphere,
    intersect_bounded_circle_torus, intersect_bounded_curve_surface,
    intersect_bounded_ellipse_cone, intersect_bounded_ellipse_cylinder,
    intersect_bounded_ellipse_plane, intersect_bounded_ellipse_sphere,
    intersect_bounded_ellipse_torus, intersect_bounded_line_cone, intersect_bounded_line_cylinder,
    intersect_bounded_line_plane, intersect_bounded_line_sphere, intersect_bounded_line_torus,
    intersect_bounded_nurbs_cone, intersect_bounded_nurbs_cylinder, intersect_bounded_nurbs_plane,
    intersect_bounded_nurbs_sphere, intersect_bounded_nurbs_torus,
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

fn bilinear_nurbs_surface() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        None,
    )
    .unwrap()
}

struct UnsupportedCurve;

impl Curve for UnsupportedCurve {
    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn eval_derivs(&self, _t: f64, _order: usize) -> CurveDerivs {
        CurveDerivs::default()
    }

    fn param_range(&self) -> ParamRange {
        ParamRange::new(0.0, 1.0)
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    fn bounding_box(&self, _range: ParamRange) -> Aabb3 {
        Aabb3::from_points(&[Point3::new(0.0, 0.0, 0.0)])
    }
}

struct UnsupportedSurface;

impl Surface for UnsupportedSurface {
    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn eval_derivs(&self, _uv: [f64; 2], _order: usize) -> SurfaceDerivs {
        SurfaceDerivs::default()
    }

    fn param_range(&self) -> [ParamRange; 2] {
        [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)]
    }

    fn periodicity(&self) -> [Option<f64>; 2] {
        [None, None]
    }

    fn bounding_box(&self, _range: [ParamRange; 2]) -> Aabb3 {
        Aabb3::from_points(&[Point3::new(0.0, 0.0, 0.0)])
    }
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

fn crossing_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 0.0, -1.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
        ],
        None,
    )
    .unwrap()
}

fn tangent_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, -1.0),
            Point3::new(1.0, 0.0, 1.0),
        ],
        None,
    )
    .unwrap()
}

fn contained_quarter_circle_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        Some(vec![1.0, 0.5_f64.sqrt(), 1.0]),
    )
    .unwrap()
}

fn crossing_sphere_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
        None,
    )
    .unwrap()
}

fn tangent_sphere_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-0.37, 1.0, 0.0), Point3::new(0.63, 1.0, 0.0)],
        None,
    )
    .unwrap()
}

fn crossing_cylinder_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
        None,
    )
    .unwrap()
}

fn tangent_cylinder_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-0.37, 1.0, 0.0), Point3::new(0.63, 1.0, 0.0)],
        None,
    )
    .unwrap()
}

fn crossing_cone_nurbs() -> NurbsCurve {
    crossing_cylinder_nurbs()
}

fn tangent_cone_nurbs() -> NurbsCurve {
    tangent_cylinder_nurbs()
}

fn crossing_torus_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-3.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)],
        None,
    )
    .unwrap()
}

fn tangent_torus_nurbs() -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-0.37, 2.5, 0.0), Point3::new(0.63, 2.5, 0.0)],
        None,
    )
    .unwrap()
}

fn outer_torus_quarter_circle_nurbs(radius: f64) -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(radius, 0.0, 0.0),
            Point3::new(radius, radius, 0.0),
            Point3::new(0.0, radius, 0.0),
        ],
        Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
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
    assert_eq!(
        intersect_bounded_circle_sphere(
            &secant,
            secant.param_range(),
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        )
        .unwrap(),
        hit
    );

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
    assert_eq!(
        intersect_bounded_circle_sphere(
            &secant,
            secant.param_range(),
            &sphere,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2,),
            ],
            Tolerances::default(),
        )
        .unwrap(),
        filtered
    );
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
fn near_concentric_circle_sphere_is_secant_not_tolerant_overlap() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let circle = Circle::new(horizontal_frame([5.0e-9, 0.0, 0.0]), 1.0).unwrap();
    let hit = intersect_bounded_circle_sphere(
        &circle,
        circle.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(
        intersect_bounded_circle_sphere(
            &circle,
            circle.param_range(),
            &sphere,
            sphere.param_range(),
            Tolerances::default(),
        )
        .unwrap(),
        hit
    );

    assert!(hit.is_complete());
    assert!(hit.overlaps.is_empty());
    assert_eq!(hit.points.len(), 2);
    assert!(
        hit.points
            .iter()
            .all(|point| point.kind == ContactKind::Transverse)
    );
    assert!(hit.points[0].point.dist(hit.points[1].point) > 1.9);
    for point in &hit.points {
        assert!(point.point.x.abs() < Tolerances::default().linear());
        assert!((point.point.norm() - 1.0).abs() < Tolerances::default().linear());
        assert!(point.residual < Tolerances::default().linear());
    }

    let opposite = Circle::new(horizontal_frame([-5.0e-9, 0.0, 0.0]), 1.0).unwrap();
    let opposite_hit = intersect_bounded_circle_sphere(
        &opposite,
        opposite.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(opposite_hit.is_complete());
    assert_eq!(opposite_hit.points.len(), 2);
    assert!(opposite_hit.overlaps.is_empty());
}

#[test]
fn axially_offset_circle_sphere_is_an_exact_constant_miss() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let circle = Circle::new(horizontal_frame([0.0, 0.0, 5.0e-9]), 1.0).unwrap();
    let hit = intersect_bounded_circle_sphere(
        &circle,
        circle.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.is_complete());
    assert!(hit.is_empty());
    assert!(hit.overlaps.is_empty());
}

#[test]
fn offset_circle_sphere_uses_exact_pythagorean_identity() {
    let sphere = Sphere::new(Frame::world(), 5.0).unwrap();
    let circle = Circle::new(horizontal_frame([0.0, 0.0, 3.0]), 4.0).unwrap();
    let hit = intersect_bounded_circle_sphere(
        &circle,
        circle.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.is_complete());
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].curve, circle.param_range());
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
    assert_eq!(
        intersect_bounded_circle_plane(
            &circle,
            circle.param_range(),
            &plane,
            [ParamRange::new(-2.0, 2.0), ParamRange::new(-0.25, 0.25)],
            Tolerances::default(),
        )
        .unwrap(),
        hit
    );

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
    assert_eq!(
        intersect_bounded_circle_plane(
            &circle,
            circle.param_range(),
            &plane,
            [ParamRange::new(0.0, 2.0), ParamRange::new(-0.25, 0.25)],
            Tolerances::default(),
        )
        .unwrap(),
        filtered
    );
    assert_eq!(filtered.points.len(), 1);
    assert_eq!(filtered.points[0].uv_surface, [1.0, 0.0]);
}

#[test]
fn near_coplanar_circle_plane_is_secant_not_tolerant_overlap() {
    let circle = Circle::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 5.0e-9, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let plane = Plane::new(Frame::world());
    let plane_range = [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)];
    let hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &plane,
        plane_range,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(
        intersect_bounded_circle_plane(
            &circle,
            circle.param_range(),
            &plane,
            plane_range,
            Tolerances::default(),
        )
        .unwrap(),
        hit
    );

    assert!(hit.is_complete());
    assert!(hit.overlaps.is_empty());
    assert_eq!(hit.points.len(), 2);
    assert!(
        hit.points
            .iter()
            .all(|point| point.kind == ContactKind::Transverse)
    );
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1.0e-12);
    assert!(hit.points[0].t_curve.abs() < 1.0e-12);
    assert!(hit.points[1].point.dist(Point3::new(-1.0, 0.0, 0.0)) < 1.0e-12);
    assert!((hit.points[1].t_curve - core::f64::consts::PI).abs() < 1.0e-12);

    let reversed = Circle::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, -5.0e-9, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let reversed_hit = intersect_bounded_circle_plane(
        &reversed,
        reversed.param_range(),
        &plane,
        plane_range,
        Tolerances::default(),
    )
    .unwrap();
    assert!(reversed_hit.is_complete());
    assert_eq!(reversed_hit.points.len(), 2);
    assert!(reversed_hit.overlaps.is_empty());
}

#[test]
fn parallel_offset_circle_plane_is_an_exact_constant_miss() {
    let circle = Circle::new(horizontal_frame([0.0, 0.0, 5.0e-9]), 1.0).unwrap();
    let plane = Plane::new(Frame::world());
    let hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &plane,
        [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.is_complete());
    assert!(hit.is_empty());
    assert!(hit.overlaps.is_empty());
}

#[test]
fn shared_tilted_frame_retains_semantic_planar_identity() {
    let frame = Frame::new(
        Point3::new(2.0, -3.0, 5.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let circle = Circle::new(frame, 1.0).unwrap();
    let plane = Plane::new(frame);
    let plane_range = [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)];
    let contained = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &plane,
        plane_range,
        Tolerances::default(),
    )
    .unwrap();

    assert!(contained.is_complete());
    assert!(contained.points.is_empty());
    assert_eq!(contained.overlaps.len(), 1);
    assert_eq!(contained.overlaps[0].curve, circle.param_range());

    let offset_circle =
        Circle::new(frame.with_origin(frame.origin() + frame.z() * 5.0e-9), 1.0).unwrap();
    let miss = intersect_bounded_circle_plane(
        &offset_circle,
        offset_circle.param_range(),
        &plane,
        plane_range,
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_complete());
    assert!(miss.is_empty());

    let reversed_plane = Plane::new(
        Frame::new(frame.origin(), -frame.z(), frame.x())
            .expect("the opposite normal retains the same semantic plane"),
    );
    let reversed_contained = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &reversed_plane,
        plane_range,
        Tolerances::default(),
    )
    .unwrap();
    assert!(reversed_contained.is_complete());
    assert!(reversed_contained.points.is_empty());
    assert_eq!(reversed_contained.overlaps.len(), 1);
    assert_eq!(reversed_contained.overlaps[0].curve, circle.param_range());
    let reversed_miss = intersect_bounded_circle_plane(
        &offset_circle,
        offset_circle.param_range(),
        &reversed_plane,
        plane_range,
        Tolerances::default(),
    )
    .unwrap();
    assert!(reversed_miss.is_complete());
    assert!(reversed_miss.is_empty());
}

#[test]
fn public_harmonic_parameter_collision_is_indeterminate() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let plane = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
    );
    let hit = intersect_bounded_circle_plane(
        &circle,
        ParamRange::new(0.0, 0.0),
        &plane,
        [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)],
        Tolerances::with_linear(core::f64::consts::TAU).unwrap(),
    )
    .unwrap();

    assert!(!hit.is_complete());
    assert!(hit.is_empty());
}

#[test]
fn planar_window_retains_sub_tolerance_overlap_between_distinct_roots() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let plane = Plane::new(Frame::world());
    let upper_x = -0.999_996_875;
    let tolerances = Tolerances::with_linear(0.01).unwrap();
    let plane_range = [ParamRange::new(-2.0, upper_x), ParamRange::new(-2.0, 2.0)];

    let hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &plane,
        plane_range,
        tolerances,
    )
    .unwrap();
    assert_eq!(
        intersect_bounded_circle_plane(
            &circle,
            circle.param_range(),
            &plane,
            plane_range,
            tolerances,
        )
        .unwrap(),
        hit
    );

    assert!(hit.is_complete());
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    let overlap = hit.overlaps[0].curve;
    assert!(overlap.width() > 0.0);
    assert!(
        overlap.width() < tolerances.linear(),
        "the retained chart interval must stay narrower than the requested parameter tolerance"
    );
    assert!(overlap.lo < core::f64::consts::PI);
    assert!(overlap.hi > core::f64::consts::PI);
}

#[test]
fn planar_window_retains_positive_input_range_below_parameter_tolerance() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let plane = Plane::new(Frame::world());
    let tolerances = Tolerances::with_linear(0.01).unwrap();
    let curve_range = ParamRange::new(1.0, 1.005);
    let plane_range = [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)];

    let hit = intersect_bounded_circle_plane(&circle, curve_range, &plane, plane_range, tolerances)
        .unwrap();
    assert_eq!(
        intersect_bounded_circle_plane(&circle, curve_range, &plane, plane_range, tolerances)
            .unwrap(),
        hit
    );

    assert!(hit.is_complete());
    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].curve, curve_range);
    assert!(hit.overlaps[0].curve.width() < tolerances.linear());
}

#[test]
fn planar_window_distinct_root_parameter_collision_is_indeterminate() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let plane = Plane::new(Frame::world());
    let upper_x = -0.999_996_875;
    let tolerances = Tolerances::with_linear(0.01).unwrap();
    let curve_range = ParamRange::new(0.0, core::f64::consts::PI - 3.0e-3);
    let plane_range = [ParamRange::new(-2.0, upper_x), ParamRange::new(-2.0, 2.0)];

    let hit = intersect_bounded_circle_plane(&circle, curve_range, &plane, plane_range, tolerances)
        .unwrap();
    assert_eq!(
        intersect_bounded_circle_plane(&circle, curve_range, &plane, plane_range, tolerances)
            .unwrap(),
        hit
    );
    assert!(!hit.is_complete());
    assert!(hit.is_empty());
}

#[test]
fn circle_plane_near_tangent_keeps_two_exactly_transverse_roots() {
    let circle = Circle::new(Frame::world(), 1.0).unwrap();
    let plane = Plane::new(
        Frame::new(
            Point3::new(-0.991, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
    );
    let hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &plane,
        [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)],
        Tolerances::with_linear(0.01).unwrap(),
    )
    .unwrap();

    assert_eq!(
        hit.points.len(),
        2,
        "near-tangent secant must not collapse to pi"
    );
    assert!(
        hit.points
            .iter()
            .all(|point| point.kind == ContactKind::Transverse)
    );
    assert!(hit.points[1].point.dist(hit.points[0].point) > 0.25);
    for point in &hit.points {
        assert!((point.point.x + 0.991).abs() < 2.0e-15);
        assert!(point.residual < 2.0e-15);
    }
}

#[test]
fn near_coplanar_nonidentity_without_roots_is_a_complete_miss() {
    let circle = Circle::new(
        Frame::new(
            Point3::new(0.0, 0.0, 7.5e-9),
            Vec3::new(0.0, 5.0e-9, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let plane = Plane::new(Frame::world());
    let hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &plane,
        [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.is_complete());
    assert!(hit.is_empty());
    assert!(hit.overlaps.is_empty());
}

#[test]
fn large_circle_near_tangent_roots_are_not_deduplicated_by_parameter_tolerance() {
    let circle = Circle::new(Frame::world(), 100.0).unwrap();
    let plane = Plane::new(
        Frame::new(
            Point3::new(-99.999_687_5, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
    );
    let plane_range = [ParamRange::new(-101.0, 101.0), ParamRange::new(-1.0, 1.0)];
    let tolerances = Tolerances::with_linear(0.01).unwrap();
    let hit = intersect_bounded_circle_plane(
        &circle,
        circle.param_range(),
        &plane,
        plane_range,
        tolerances,
    )
    .unwrap();
    assert_eq!(
        intersect_bounded_circle_plane(
            &circle,
            circle.param_range(),
            &plane,
            plane_range,
            tolerances,
        )
        .unwrap(),
        hit
    );

    assert!(hit.is_complete());
    assert!(hit.overlaps.is_empty());
    assert_eq!(hit.points.len(), 2);
    assert!(
        hit.points
            .iter()
            .all(|point| point.kind == ContactKind::Transverse)
    );
    let parameter_separation = hit.points[1].t_curve - hit.points[0].t_curve;
    assert!(parameter_separation > 0.0);
    assert!(parameter_separation < tolerances.linear());
    assert!(hit.points[0].point.dist(hit.points[1].point) > 0.4);
    for point in &hit.points {
        assert!((point.point.x + 99.999_687_5).abs() < 2.0e-12);
        assert!(point.residual < 2.0e-12);
    }
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
fn nurbs_plane_crossing_tangent_and_range_filtering() {
    let plane = Plane::new(Frame::world());
    let wide_plane = [ParamRange::new(-2.0, 2.0), ParamRange::new(-2.0, 2.0)];

    let crossing = crossing_nurbs();
    let hit = intersect_bounded_nurbs_plane(
        &crossing,
        crossing.param_range(),
        &plane,
        wide_plane,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert_eq!(hit.points[0].t_curve.to_bits(), 0.5_f64.to_bits());
    assert!((hit.points[0].t_curve - 0.5).abs() < 1e-8);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 0.0, 0.0)) < 1e-8);
    assert_eq!(hit.points[0].uv_surface, [0.0, 0.0]);
    assert_eq!(
        intersect_bounded_nurbs_plane(
            &crossing,
            crossing.param_range(),
            &plane,
            wide_plane,
            Tolerances::default(),
        )
        .unwrap(),
        hit,
    );

    let range_miss = intersect_bounded_nurbs_plane(
        &crossing,
        ParamRange::new(0.0, 0.25),
        &plane,
        wide_plane,
        Tolerances::default(),
    )
    .unwrap();
    assert!(range_miss.is_empty());

    let window_miss = intersect_bounded_nurbs_plane(
        &crossing,
        crossing.param_range(),
        &plane,
        [ParamRange::new(0.25, 2.0), ParamRange::new(-1.0, 1.0)],
        Tolerances::default(),
    )
    .unwrap();
    assert!(window_miss.is_empty());

    let tangent = tangent_nurbs();
    let hit = intersect_bounded_nurbs_plane(
        &tangent,
        tangent.param_range(),
        &plane,
        wide_plane,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!((hit.points[0].t_curve - 0.5).abs() < 1e-8);
    assert!(hit.points[0].point.dist(Point3::new(0.0, 0.0, 0.0)) < 1e-8);
}

#[test]
fn nurbs_plane_exact_source_signs_prevent_oblique_false_overlap() {
    let normal = Vec3::new(0.6, 0.8, 0.0);
    let positive = Point3::new(-2_863_298_200.0, 2_147_473_650.0, 0.0);
    assert_eq!(normal.dot(positive), 0.0);

    let make_curve = |first, second| {
        NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![first, second], None).unwrap()
    };
    let curve = make_curve(positive, -positive);
    let plane =
        Plane::new(Frame::new(Point3::default(), normal, Vec3::new(0.0, 0.0, 1.0)).unwrap());
    let window = [
        ParamRange::new(-5.0e9, 5.0e9),
        ParamRange::new(-5.0e9, 5.0e9),
    ];

    let hit = intersect_bounded_nurbs_plane(
        &curve,
        curve.param_range(),
        &plane,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert!(hit.overlaps.is_empty());
    assert_eq!(hit.points[0].t_curve.to_bits(), 0.5_f64.to_bits());
    assert_eq!(hit.points[0].point, Point3::default());
    assert!(!hit.is_complete());
    assert_eq!(
        intersect_bounded_nurbs_plane(
            &curve,
            curve.param_range(),
            &plane,
            window,
            Tolerances::default(),
        )
        .unwrap(),
        hit,
    );

    let point_query = intersect_bounded_nurbs_plane(
        &curve,
        ParamRange::new(0.0, 0.0),
        &plane,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert!(point_query.is_empty());
    assert!(point_query.overlaps.is_empty());
    assert!(!point_query.is_complete());

    let reversed = make_curve(-positive, positive);
    let reversed_hit = intersect_bounded_nurbs_plane(
        &reversed,
        reversed.param_range(),
        &plane,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(reversed_hit.points.len(), 1);
    assert!(reversed_hit.overlaps.is_empty());
    assert_eq!(reversed_hit.points[0].t_curve.to_bits(), 0.5_f64.to_bits());
    assert_eq!(reversed_hit.points[0].point, Point3::default());

    let reversed_normal_plane =
        Plane::new(Frame::new(Point3::default(), -normal, Vec3::new(0.0, 0.0, 1.0)).unwrap());
    let reversed_normal_hit = intersect_bounded_nurbs_plane(
        &curve,
        curve.param_range(),
        &reversed_normal_plane,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(reversed_normal_hit.points.len(), 1);
    assert!(reversed_normal_hit.overlaps.is_empty());
    assert_eq!(
        reversed_normal_hit.points[0].t_curve.to_bits(),
        0.5_f64.to_bits()
    );
    assert_eq!(reversed_normal_hit.points[0].point, Point3::default());
}

#[test]
fn nurbs_plane_uses_source_range_when_split_controls_lose_midpoint_contact() {
    let contact_z = 9_007_199_254_740_991.0;
    let curve = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(-1.0, 0.0, 9_007_199_254_740_360.0),
            Point3::new(-1.0 / 3.0, 0.0, 9_007_199_254_740_978.0),
            Point3::new(1.0 / 3.0, 0.0, 9_007_199_254_741_648.0),
            Point3::new(1.0, 0.0, 9_007_199_254_739_690.0),
        ],
        None,
    )
    .unwrap();
    let (left, right) = curve.split_at(0.5).unwrap();
    assert!(left.points().iter().all(|point| point.z < contact_z));
    assert!(right.points().iter().all(|point| point.z < contact_z));
    assert_eq!(curve.eval(0.5).z.to_bits(), contact_z.to_bits());

    let plane = Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, contact_z)));
    let window = [ParamRange::new(-2.0, 2.0), ParamRange::new(-1.0, 1.0)];
    let hit = intersect_bounded_nurbs_plane(
        &curve,
        curve.param_range(),
        &plane,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert!(hit.overlaps.is_empty());
    assert_eq!(hit.points[0].t_curve.to_bits(), 0.5_f64.to_bits());
    assert_eq!(hit.points[0].point.z.to_bits(), contact_z.to_bits());
    assert_eq!(
        intersect_bounded_nurbs_plane(
            &curve,
            curve.param_range(),
            &plane,
            window,
            Tolerances::default(),
        )
        .unwrap(),
        hit,
    );
}

#[test]
fn nurbs_plane_window_overlap_extents_require_source_band_proof() {
    let outside = 9_007_199_254_740_991.0;
    let lower = outside - 2_000.0;
    let upper = outside - 1.0;
    let curve = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(9_007_199_254_740_360.0, 0.0, 0.0),
            Point3::new(9_007_199_254_740_978.0, 0.0, 0.0),
            Point3::new(9_007_199_254_741_648.0, 0.0, 0.0),
            Point3::new(9_007_199_254_739_690.0, 0.0, 0.0),
        ],
        None,
    )
    .unwrap();
    let (left, right) = curve.split_at(0.5).unwrap();
    for derived in [&left, &right] {
        assert!(
            derived
                .points()
                .iter()
                .all(|point| lower <= point.x && point.x <= upper)
        );
    }
    assert_eq!(curve.eval(0.5).x.to_bits(), outside.to_bits());

    let plane = Plane::new(Frame::world());
    let window = [ParamRange::new(lower, upper), ParamRange::new(-1.0, 1.0)];
    let hit = intersect_bounded_nurbs_plane(
        &curve,
        curve.param_range(),
        &plane,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.points.is_empty());
    assert!(
        hit.overlaps
            .iter()
            .all(|overlap| !overlap.curve.contains(0.5)),
        "an overlap range must not include the source midpoint outside the plane window",
    );
    assert!(!hit.is_complete());
    assert_eq!(
        intersect_bounded_nurbs_plane(
            &curve,
            curve.param_range(),
            &plane,
            window,
            Tolerances::default(),
        )
        .unwrap(),
        hit,
    );
}

#[test]
fn nurbs_contained_in_plane_reports_overlap() {
    let curve = contained_quarter_circle_nurbs();
    let plane = Plane::new(Frame::world());
    let hit = intersect_bounded_nurbs_plane(
        &curve,
        curve.param_range(),
        &plane,
        [ParamRange::new(-0.25, 1.25), ParamRange::new(-0.25, 1.25)],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].curve, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].uv_start, [1.0, 0.0]);
    assert!(hit.overlaps[0].uv_end[0].abs() < 1e-12);
    assert!((hit.overlaps[0].uv_end[1] - 1.0).abs() < 1e-12);
}

#[test]
fn nurbs_sphere_crossing_tangent_and_range_filtering() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let crossing = crossing_sphere_nurbs();
    let hit = intersect_bounded_nurbs_sphere(
        &crossing,
        crossing.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(-1.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[0].t_curve - 0.25).abs() < 1e-8);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[1].t_curve - 0.75).abs() < 1e-8);

    let range_miss = intersect_bounded_nurbs_sphere(
        &crossing,
        ParamRange::new(0.0, 0.2),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(range_miss.is_empty());

    let filtered = intersect_bounded_nurbs_sphere(
        &crossing,
        crossing.param_range(),
        &sphere,
        [
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-8);

    let tangent = tangent_sphere_nurbs();
    let hit = intersect_bounded_nurbs_sphere(
        &tangent,
        tangent.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(
        hit.points[0].point.dist(Point3::new(0.0, 1.0, 0.0)) < 2e-8,
        "{:?}",
        hit.points[0]
    );
    assert!((hit.points[0].t_curve - 0.37).abs() < 2e-8);
}

#[test]
fn nurbs_contained_in_sphere_reports_overlap() {
    let curve = contained_quarter_circle_nurbs();
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_nurbs_sphere(
        &curve,
        curve.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].curve, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].uv_start, [0.0, 0.0]);
    assert!((hit.overlaps[0].uv_end[0] - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
    assert!(hit.overlaps[0].uv_end[1].abs() < 1e-12);
}

#[test]
fn nurbs_cylinder_crossing_tangent_and_range_filtering() {
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let crossing = crossing_cylinder_nurbs();
    let hit = intersect_bounded_nurbs_cylinder(
        &crossing,
        crossing.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(-1.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[0].t_curve - 0.25).abs() < 1e-8);
    assert!((hit.points[0].uv_surface[0] - core::f64::consts::PI).abs() < 1e-8);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[1].t_curve - 0.75).abs() < 1e-8);
    assert!(hit.points[1].uv_surface[0].abs() < 1e-8);

    let range_miss = intersect_bounded_nurbs_cylinder(
        &crossing,
        ParamRange::new(0.0, 0.2),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(range_miss.is_empty());

    let filtered = intersect_bounded_nurbs_cylinder(
        &crossing,
        crossing.param_range(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-8);

    let z_filtered = intersect_bounded_nurbs_cylinder(
        &crossing,
        crossing.param_range(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.25, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(z_filtered.is_empty());

    let tangent = tangent_cylinder_nurbs();
    let hit = intersect_bounded_nurbs_cylinder(
        &tangent,
        tangent.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(
        hit.points[0].point.dist(Point3::new(0.0, 1.0, 0.0)) < 2e-8,
        "{:?}",
        hit.points[0]
    );
    assert!((hit.points[0].t_curve - 0.37).abs() < 2e-8);
}

#[test]
fn nurbs_contained_in_cylinder_reports_overlap() {
    let curve = contained_quarter_circle_nurbs();
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_nurbs_cylinder(
        &curve,
        curve.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].curve, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].uv_start, [0.0, 0.0]);
    assert!((hit.overlaps[0].uv_end[0] - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
    assert!(hit.overlaps[0].uv_end[1].abs() < 1e-12);
}

#[test]
fn nurbs_cone_crossing_tangent_and_range_filtering() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::FRAC_PI_4).unwrap();
    let crossing = crossing_cone_nurbs();
    let hit = intersect_bounded_nurbs_cone(
        &crossing,
        crossing.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!(hit.points[0].point.dist(Point3::new(-1.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[0].t_curve - 0.25).abs() < 1e-8);
    assert!((hit.points[0].uv_surface[0] - core::f64::consts::PI).abs() < 1e-8);
    assert!(hit.points[0].uv_surface[1].abs() < 1e-8);
    assert_eq!(hit.points[1].kind, ContactKind::Transverse);
    assert!(hit.points[1].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-8);
    assert!((hit.points[1].t_curve - 0.75).abs() < 1e-8);
    assert!(hit.points[1].uv_surface[0].abs() < 1e-8);
    assert!(hit.points[1].uv_surface[1].abs() < 1e-8);

    let range_miss = intersect_bounded_nurbs_cone(
        &crossing,
        ParamRange::new(0.0, 0.2),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(range_miss.is_empty());

    let filtered = intersect_bounded_nurbs_cone(
        &crossing,
        crossing.param_range(),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(filtered.points.len(), 1);
    assert!(filtered.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) < 1e-8);

    let v_filtered = intersect_bounded_nurbs_cone(
        &crossing,
        crossing.param_range(),
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.25, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(v_filtered.is_empty());

    let tangent = tangent_cone_nurbs();
    let hit = intersect_bounded_nurbs_cone(
        &tangent,
        tangent.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(
        hit.points[0].point.dist(Point3::new(0.0, 1.0, 0.0)) < 2e-8,
        "{:?}",
        hit.points[0]
    );
    assert!((hit.points[0].t_curve - 0.37).abs() < 2e-8);
}

#[test]
fn nurbs_contained_in_cone_reports_overlap() {
    let curve = contained_quarter_circle_nurbs();
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::FRAC_PI_4).unwrap();
    let hit = intersect_bounded_nurbs_cone(
        &curve,
        curve.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].curve, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].uv_start, [0.0, 0.0]);
    assert!((hit.overlaps[0].uv_end[0] - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
    assert!(hit.overlaps[0].uv_end[1].abs() < 1e-12);
}

#[test]
fn nurbs_torus_crossing_tangent_and_range_filtering() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let crossing = crossing_torus_nurbs();
    let hit = intersect_bounded_nurbs_torus(
        &crossing,
        crossing.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 4);
    let expected = [
        (
            Point3::new(-2.5, 0.0, 0.0),
            1.0 / 12.0,
            core::f64::consts::PI,
            0.0,
        ),
        (
            Point3::new(-1.5, 0.0, 0.0),
            0.25,
            core::f64::consts::PI,
            core::f64::consts::PI,
        ),
        (Point3::new(1.5, 0.0, 0.0), 0.75, 0.0, core::f64::consts::PI),
        (Point3::new(2.5, 0.0, 0.0), 11.0 / 12.0, 0.0, 0.0),
    ];
    for (point, (expected_point, expected_t, expected_u, expected_v)) in
        hit.points.iter().zip(expected)
    {
        assert_eq!(point.kind, ContactKind::Transverse);
        assert!(point.point.dist(expected_point) < 1e-8);
        assert!((point.t_curve - expected_t).abs() < 1e-8);
        assert!((point.uv_surface[0] - expected_u).abs() < 1e-8);
        assert!((point.uv_surface[1] - expected_v).abs() < 1e-8);
    }

    let range_miss = intersect_bounded_nurbs_torus(
        &crossing,
        ParamRange::new(0.0, 0.05),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(range_miss.is_empty());

    let u_filtered = intersect_bounded_nurbs_torus(
        &crossing,
        crossing.param_range(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(u_filtered.points.len(), 2);
    assert!(u_filtered.points[0].point.dist(Point3::new(1.5, 0.0, 0.0)) < 1e-8);
    assert!(u_filtered.points[1].point.dist(Point3::new(2.5, 0.0, 0.0)) < 1e-8);

    let v_filtered = intersect_bounded_nurbs_torus(
        &crossing,
        crossing.param_range(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(v_filtered.points.len(), 2);
    assert!(v_filtered.points[0].point.dist(Point3::new(-2.5, 0.0, 0.0)) < 1e-8);
    assert!(v_filtered.points[1].point.dist(Point3::new(2.5, 0.0, 0.0)) < 1e-8);

    let tangent = tangent_torus_nurbs();
    let hit = intersect_bounded_nurbs_torus(
        &tangent,
        tangent.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(
        hit.points[0].point.dist(Point3::new(0.0, 2.5, 0.0)) < 5e-8,
        "{:?}",
        hit.points[0]
    );
    assert!((hit.points[0].t_curve - 0.37).abs() < 5e-8);
    assert!((hit.points[0].uv_surface[0] - core::f64::consts::FRAC_PI_2).abs() < 2e-8);
    assert!(hit.points[0].uv_surface[1].abs() < 2e-8);
}

#[test]
fn nurbs_contained_in_torus_reports_overlap() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let curve = outer_torus_quarter_circle_nurbs(torus.major_radius() + torus.minor_radius());
    let hit = intersect_bounded_nurbs_torus(
        &curve,
        curve.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.overlaps.len(), 1);
    assert_eq!(hit.overlaps[0].curve, ParamRange::new(0.0, 1.0));
    assert_eq!(hit.overlaps[0].uv_start, [0.0, 0.0]);
    assert!((hit.overlaps[0].uv_end[0] - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
    assert!(hit.overlaps[0].uv_end[1].abs() < 1e-12);
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
    let hit = intersect_bounded_curve_surface(
        &nurbs,
        nurbs.param_range(),
        &plane,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Transverse);
    assert!((hit.points[0].t_curve - 0.5).abs() < 1e-8);

    let nurbs = crossing_sphere_nurbs();
    let hit = intersect_bounded_curve_surface(
        &nurbs,
        nurbs.param_range(),
        &sphere,
        sphere.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);

    let nurbs = crossing_cylinder_nurbs();
    let hit = intersect_bounded_curve_surface(
        &nurbs,
        nurbs.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);

    let nurbs = crossing_cone_nurbs();
    let hit = intersect_bounded_curve_surface(
        &nurbs,
        nurbs.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 2);

    let nurbs = crossing_torus_nurbs();
    let hit = intersect_bounded_curve_surface(
        &nurbs,
        nurbs.param_range(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.points.len(), 4);
}

#[test]
fn curve_surface_dispatch_matches_specialized_analytic_and_nurbs_routes() {
    let tolerances = Tolerances::default();
    let plane = Plane::new(Frame::world());
    let surface_range = plane_window();

    let line = make_line([0.0, 0.0, -1.0], [0.0, 0.0, 1.0]);
    let line_range = ParamRange::new(0.0, 2.0);
    let direct =
        intersect_bounded_line_plane(&line, line_range, &plane, surface_range, tolerances).unwrap();
    let dispatched =
        intersect_bounded_curve_surface(&line, line_range, &plane, surface_range, tolerances)
            .unwrap();
    assert_eq!(dispatched, direct);

    let nurbs = crossing_nurbs();
    let direct = intersect_bounded_nurbs_plane(
        &nurbs,
        nurbs.param_range(),
        &plane,
        surface_range,
        tolerances,
    )
    .unwrap();
    let dispatched = intersect_bounded_curve_surface(
        &nurbs,
        nurbs.param_range(),
        &plane,
        surface_range,
        tolerances,
    )
    .unwrap();
    assert_eq!(dispatched, direct);
}

#[test]
fn curve_surface_dispatch_reports_structured_unsupported_pair() {
    let line = make_line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let surface = bilinear_nurbs_surface();
    let err = intersect_bounded_curve_surface(
        &line,
        ParamRange::new(0.0, 1.0),
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap_err();

    assert_eq!(
        err,
        IntersectionError::UnsupportedCurveSurfacePair {
            curve_class: Some(CurveClass::Line.key()),
            surface_class: Some(SurfaceClass::Nurbs.key()),
        }
    );
    assert_eq!(err.class(), ErrorClass::Unsupported);
    assert_eq!(err.code(), UNSUPPORTED_CLASS_PAIR);
    assert_eq!(err.capability(), Some(CURVE_SURFACE_CLASS_PAIR));
    assert!(err.source().is_none());

    let classified: &dyn ClassifiedError = &err;
    assert_eq!(classified.class(), ErrorClass::Unsupported);
}

#[test]
fn curve_surface_dispatch_preserves_unknown_operand_identity() {
    let curve = UnsupportedCurve;
    let plane = Plane::new(Frame::world());
    let err = intersect_bounded_curve_surface(
        &curve,
        curve.param_range(),
        &plane,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        IntersectionError::UnsupportedCurveSurfacePair {
            curve_class: None,
            surface_class: Some(SurfaceClass::Plane.key()),
        }
    );

    let line = make_line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let surface = UnsupportedSurface;
    let err = intersect_bounded_curve_surface(
        &line,
        ParamRange::new(0.0, 1.0),
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        IntersectionError::UnsupportedCurveSurfacePair {
            curve_class: Some(CurveClass::Line.key()),
            surface_class: None,
        }
    );
}

#[test]
fn curve_surface_dispatch_preserves_kernel_error_classification_and_source() {
    let line = make_line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let plane = Plane::new(Frame::world());
    let invalid_range = ParamRange { lo: 1.0, hi: 0.0 };
    let err = intersect_bounded_curve_surface(
        &line,
        invalid_range,
        &plane,
        plane_window(),
        Tolerances::default(),
    )
    .unwrap_err();

    let kernel = Error::InvalidGeometry {
        reason: "line/plane intersection requires a finite non-reversed line range",
    };
    assert_eq!(err, IntersectionError::Kernel(kernel.clone()));
    assert_eq!(err.class(), kernel.class());
    assert_eq!(err.code(), kernel.code());
    assert_eq!(err.capability(), kernel.capability());
    assert_eq!(err.limit(), kernel.limit());
    assert_eq!(
        err.source()
            .and_then(|source| source.downcast_ref::<Error>()),
        Some(&kernel)
    );
}
