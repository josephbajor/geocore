//! Bounded analytic surface/surface intersection behavior.

use kcore::error::Error;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    intersect_bounded_cone_cylinder, intersect_bounded_cone_nurbs_surface,
    intersect_bounded_cone_sphere, intersect_bounded_cone_torus, intersect_bounded_cones,
    intersect_bounded_cylinder_nurbs_surface, intersect_bounded_cylinder_sphere,
    intersect_bounded_cylinder_torus, intersect_bounded_cylinders, intersect_bounded_plane_cone,
    intersect_bounded_plane_cylinder, intersect_bounded_plane_nurbs_surface,
    intersect_bounded_plane_sphere, intersect_bounded_plane_torus, intersect_bounded_planes,
    intersect_bounded_sphere_nurbs_surface, intersect_bounded_sphere_torus,
    intersect_bounded_spheres, intersect_bounded_surfaces, intersect_bounded_tori,
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

fn cone_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-1.0, 1.0),
    ]
}

fn wide_cone_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-4.0, 2.0),
    ]
}

fn sphere_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ]
}

fn torus_window() -> [ParamRange; 2] {
    [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(0.0, core::f64::consts::TAU),
    ]
}

fn torus_plane_window() -> [ParamRange; 2] {
    [ParamRange::new(-3.0, 3.0), ParamRange::new(-3.0, 3.0)]
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

fn cone_plane_with_slope(slope: f64) -> Plane {
    let x_axis = Vec3::new(1.0, 0.0, slope).normalized().unwrap();
    let y_axis = Vec3::new(0.0, 1.0, 0.0);
    Plane::new(Frame::new(Point3::new(0.0, 0.0, 0.0), x_axis.cross(y_axis), x_axis).unwrap())
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

fn assert_cylinder_sphere_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    cylinder: &Cylinder,
    sphere: &Sphere,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(cylinder.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(cylinder.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(sphere.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(sphere.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_cylinder_cylinder_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    a: &Cylinder,
    b: &Cylinder,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(a.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(a.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(b.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(b.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_cylinder_torus_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    cylinder: &Cylinder,
    torus: &Torus,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(cylinder.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(cylinder.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(torus.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(torus.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_plane_torus_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    plane: &Plane,
    torus: &Torus,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(plane.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(plane.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(torus.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(torus.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_sphere_torus_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    sphere: &Sphere,
    torus: &Torus,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(sphere.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(sphere.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(torus.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(torus.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_torus_torus_branch_endpoints(hit: &SurfaceSurfaceIntersections, a: &Torus, b: &Torus) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(a.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(a.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(b.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(b.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_plane_cone_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    plane: &Plane,
    cone: &Cone,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(plane.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(plane.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(cone.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(cone.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_cone_sphere_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    cone: &Cone,
    sphere: &Sphere,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(cone.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(cone.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(sphere.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(sphere.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_cone_cylinder_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    cone: &Cone,
    cylinder: &Cylinder,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(cone.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(cone.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(cylinder.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(cylinder.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_cone_cone_branch_endpoints(hit: &SurfaceSurfaceIntersections, a: &Cone, b: &Cone) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(a.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(a.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(b.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(b.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

fn assert_cone_torus_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    cone: &Cone,
    torus: &Torus,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(cone.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(cone.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(torus.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(torus.eval(branch.uv_b_end).dist(end) < 1e-12);
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

fn assert_plane_nurbs_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    plane: &Plane,
    surface: &NurbsSurface,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(plane.eval(branch.uv_a_start).dist(start) < 1e-7);
        assert!(plane.eval(branch.uv_a_end).dist(end) < 1e-7);
        assert!(surface.eval(branch.uv_b_start).dist(start) < 1e-7);
        assert!(surface.eval(branch.uv_b_end).dist(end) < 1e-7);
    }
}

fn assert_sphere_nurbs_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    sphere: &Sphere,
    surface: &NurbsSurface,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(sphere.eval(branch.uv_a_start).dist(start) < 1e-7);
        assert!(sphere.eval(branch.uv_a_end).dist(end) < 1e-7);
        assert!(surface.eval(branch.uv_b_start).dist(start) < 1e-7);
        assert!(surface.eval(branch.uv_b_end).dist(end) < 1e-7);
    }
}

fn assert_cylinder_nurbs_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    cylinder: &Cylinder,
    surface: &NurbsSurface,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(cylinder.eval(branch.uv_a_start).dist(start) < 1e-7);
        assert!(cylinder.eval(branch.uv_a_end).dist(end) < 1e-7);
        assert!(surface.eval(branch.uv_b_start).dist(start) < 1e-7);
        assert!(surface.eval(branch.uv_b_end).dist(end) < 1e-7);
    }
}

fn assert_cone_nurbs_branch_endpoints(
    hit: &SurfaceSurfaceIntersections,
    cone: &Cone,
    surface: &NurbsSurface,
) {
    for branch in &hit.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(cone.eval(branch.uv_a_start).dist(start) < 1e-7);
        assert!(cone.eval(branch.uv_a_end).dist(end) < 1e-7);
        assert!(surface.eval(branch.uv_b_start).dist(start) < 1e-7);
        assert!(surface.eval(branch.uv_b_end).dist(end) < 1e-7);
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

fn bilinear_nurbs_surface() -> NurbsSurface {
    bilinear_nurbs_surface_at_z(0.0)
}

fn bilinear_nurbs_surface_at_z(z: f64) -> NurbsSurface {
    bilinear_nurbs_surface_rect(0.0, 1.0, 0.0, 1.0, z)
}

fn bilinear_nurbs_surface_rect(x0: f64, x1: f64, y0: f64, y1: f64, z: f64) -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(x0, y0, z),
            Point3::new(x0, y1, z),
            Point3::new(x1, y0, z),
            Point3::new(x1, y1, z),
        ],
        None,
    )
    .unwrap()
}

fn quarter_cylinder_nurbs_surface(radius: f64, z0: f64, z1: f64) -> NurbsSurface {
    let weight = core::f64::consts::FRAC_1_SQRT_2;
    NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(radius, 0.0, z0),
            Point3::new(radius, 0.0, z1),
            Point3::new(radius, radius, z0),
            Point3::new(radius, radius, z1),
            Point3::new(0.0, radius, z0),
            Point3::new(0.0, radius, z1),
        ],
        Some(vec![1.0, 1.0, weight, weight, 1.0, 1.0]),
    )
    .unwrap()
}

fn quarter_cone_nurbs_surface(cone: &Cone, v0: f64, v1: f64) -> NurbsSurface {
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let rho0 = cone.radius() + v0 * sin_a;
    let rho1 = cone.radius() + v1 * sin_a;
    let z0 = v0 * cos_a;
    let z1 = v1 * cos_a;
    let weight = core::f64::consts::FRAC_1_SQRT_2;
    NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(rho0, 0.0, z0),
            Point3::new(rho1, 0.0, z1),
            Point3::new(rho0, rho0, z0),
            Point3::new(rho1, rho1, z1),
            Point3::new(0.0, rho0, z0),
            Point3::new(0.0, rho1, z1),
        ],
        Some(vec![1.0, 1.0, weight, weight, 1.0, 1.0]),
    )
    .unwrap()
}

fn sloped_bilinear_nurbs_surface() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, -0.4),
            Point3::new(0.0, 1.0, -0.4),
            Point3::new(1.0, 0.0, 0.6),
            Point3::new(1.0, 1.0, 0.6),
        ],
        None,
    )
    .unwrap()
}

#[test]
fn surface_intersection_curve_carries_nurbs_branch() {
    let nurbs = quarter_circle_nurbs();
    let branch = SurfaceIntersectionCurve::Nurbs(nurbs.clone());
    assert_eq!(branch.param_range(), ParamRange::new(0.0, 1.0));
    assert!(branch.eval(0.0).dist(Point3::new(1.0, 0.0, 0.0)) < 1e-12);
    assert!(branch.eval(1.0).dist(Point3::new(0.0, 1.0, 0.0)) < 1e-12);
    assert!(
        branch.eval(0.5).dist(Point3::new(
            core::f64::consts::FRAC_1_SQRT_2,
            core::f64::consts::FRAC_1_SQRT_2,
            0.0
        )) < 1e-12
    );

    let intersections = SurfaceSurfaceIntersections::canonicalized(
        Vec::new(),
        vec![SurfaceSurfaceCurve {
            curve: SurfaceIntersectionCurve::Nurbs(nurbs),
            curve_range: ParamRange::new(0.0, 1.0),
            uv_a_start: [0.0, 0.0],
            uv_a_end: [1.0, 0.0],
            uv_b_start: [0.0, 1.0],
            uv_b_end: [1.0, 1.0],
            kind: ContactKind::Transverse,
        }],
    )
    .unwrap();
    assert_eq!(intersections.curves.len(), 1);
    assert!(matches!(
        intersections.curves[0].curve,
        SurfaceIntersectionCurve::Nurbs(_)
    ));
}

#[test]
fn plane_nurbs_surface_marches_transverse_branch() {
    let plane = horizontal_plane(0.0);
    let surface = sloped_bilinear_nurbs_surface();
    let hit = intersect_bounded_plane_nurbs_surface(
        &plane,
        [ParamRange::new(0.0, 1.0), ParamRange::new(-0.1, 1.1)],
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert_eq!(hit.curves[0].kind, ContactKind::Transverse);
    assert_plane_nurbs_branch_endpoints(&hit, &plane, &surface);

    let SurfaceIntersectionCurve::Nurbs(curve) = &hit.curves[0].curve else {
        panic!("marched plane/NURBS-surface cut should be carried by a NURBS polyline");
    };
    assert_eq!(curve.degree(), 1);
    assert!(curve.points().len() >= 2);

    let branch = &hit.curves[0];
    assert!((branch.uv_b_start[0] - 0.4).abs() < 1e-7);
    assert!((branch.uv_b_end[0] - 0.4).abs() < 1e-7);
    let v_min = branch.uv_b_start[1].min(branch.uv_b_end[1]);
    let v_max = branch.uv_b_start[1].max(branch.uv_b_end[1]);
    assert!(v_min.abs() < 1e-7);
    assert!((v_max - 1.0).abs() < 1e-7);
}

#[test]
fn plane_nurbs_surface_dispatches_both_orders_and_rejects_overlap() {
    let plane = horizontal_plane(0.0);
    let surface = sloped_bilinear_nurbs_surface();
    let plane_range = [ParamRange::new(0.0, 1.0), ParamRange::new(-0.1, 1.1)];
    let hit = intersect_bounded_surfaces(
        &plane,
        plane_range,
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 1);
    assert_plane_nurbs_branch_endpoints(&hit, &plane, &surface);

    let swapped = intersect_bounded_surfaces(
        &surface,
        surface.param_range(),
        &plane,
        plane_range,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), 1);
    assert_eq!(swapped.curves[0].uv_a_start, hit.curves[0].uv_b_start);
    assert_eq!(swapped.curves[0].uv_b_start, hit.curves[0].uv_a_start);

    let coincident = bilinear_nurbs_surface();
    let err = intersect_bounded_plane_nurbs_surface(
        &plane,
        [ParamRange::new(-0.25, 1.25), ParamRange::new(-0.25, 1.25)],
        &coincident,
        coincident.param_range(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "coincident plane/nurbs-surface intersection is a surface overlap"
        }
    );
}

#[test]
fn sphere_nurbs_surface_marches_planar_patch_arc() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let surface = bilinear_nurbs_surface_at_z(0.5);
    let hit = intersect_bounded_sphere_nurbs_surface(
        &sphere,
        sphere_window(),
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert_eq!(hit.curves[0].kind, ContactKind::Transverse);
    assert_sphere_nurbs_branch_endpoints(&hit, &sphere, &surface);

    let SurfaceIntersectionCurve::Nurbs(curve) = &hit.curves[0].curve else {
        panic!("marched sphere/NURBS-surface cut should be carried by a NURBS polyline");
    };
    assert_eq!(curve.degree(), 1);
    assert!(curve.points().len() >= 2);
    for point in curve.points() {
        assert!((point.dist(sphere.frame().origin()) - sphere.radius()).abs() < 1e-7);
        assert!((point.z - 0.5).abs() < 1e-12);
    }

    let branch = &hit.curves[0];
    assert!((branch.uv_a_start[1] - core::f64::consts::FRAC_PI_6).abs() < 1e-7);
    assert!((branch.uv_a_end[1] - core::f64::consts::FRAC_PI_6).abs() < 1e-7);
    let u_min = branch.uv_a_start[0].min(branch.uv_a_end[0]);
    let u_max = branch.uv_a_start[0].max(branch.uv_a_end[0]);
    assert!(u_min.abs() < 1e-7);
    assert!((u_max - core::f64::consts::FRAC_PI_2).abs() < 1e-7);

    let miss_surface = bilinear_nurbs_surface_at_z(2.0);
    let miss = intersect_bounded_sphere_nurbs_surface(
        &sphere,
        sphere_window(),
        &miss_surface,
        miss_surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn sphere_nurbs_surface_dispatches_both_orders() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let surface = bilinear_nurbs_surface_at_z(0.5);
    let hit = intersect_bounded_surfaces(
        &sphere,
        sphere_window(),
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 1);
    assert_sphere_nurbs_branch_endpoints(&hit, &sphere, &surface);

    let swapped = intersect_bounded_surfaces(
        &surface,
        surface.param_range(),
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), 1);
    assert_eq!(swapped.curves[0].uv_a_start, hit.curves[0].uv_b_start);
    assert_eq!(swapped.curves[0].uv_b_start, hit.curves[0].uv_a_start);
}

#[test]
fn cylinder_nurbs_surface_marches_planar_patch_arc() {
    let radius = 0.75;
    let cylinder = Cylinder::new(Frame::world(), radius).unwrap();
    let surface = bilinear_nurbs_surface_at_z(0.5);
    let hit = intersect_bounded_cylinder_nurbs_surface(
        &cylinder,
        cylinder_window(),
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert_eq!(hit.curves[0].kind, ContactKind::Transverse);
    assert_cylinder_nurbs_branch_endpoints(&hit, &cylinder, &surface);

    let SurfaceIntersectionCurve::Nurbs(curve) = &hit.curves[0].curve else {
        panic!("marched cylinder/NURBS-surface cut should be carried by a NURBS polyline");
    };
    assert_eq!(curve.degree(), 1);
    assert!(curve.points().len() >= 2);
    for point in curve.points() {
        assert!(((point.x * point.x + point.y * point.y).sqrt() - radius).abs() < 1e-7);
        assert!((point.z - 0.5).abs() < 1e-12);
    }

    let branch = &hit.curves[0];
    assert!((branch.uv_a_start[1] - 0.5).abs() < 1e-7);
    assert!((branch.uv_a_end[1] - 0.5).abs() < 1e-7);
    let u_min = branch.uv_a_start[0].min(branch.uv_a_end[0]);
    let u_max = branch.uv_a_start[0].max(branch.uv_a_end[0]);
    assert!(u_min.abs() < 1e-7);
    assert!((u_max - core::f64::consts::FRAC_PI_2).abs() < 1e-7);

    let miss_surface = bilinear_nurbs_surface_rect(0.0, 0.25, 0.0, 0.25, 0.5);
    let miss = intersect_bounded_cylinder_nurbs_surface(
        &cylinder,
        cylinder_window(),
        &miss_surface,
        miss_surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let coincident = quarter_cylinder_nurbs_surface(radius, -0.25, 0.75);
    let err = intersect_bounded_cylinder_nurbs_surface(
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            ParamRange::new(-0.25, 0.75),
        ],
        &coincident,
        coincident.param_range(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "coincident cylinder/nurbs-surface intersection is a surface overlap"
        }
    );
}

#[test]
fn cylinder_nurbs_surface_dispatches_both_orders() {
    let cylinder = Cylinder::new(Frame::world(), 0.75).unwrap();
    let surface = bilinear_nurbs_surface_at_z(0.5);
    let hit = intersect_bounded_surfaces(
        &cylinder,
        cylinder_window(),
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 1);
    assert_cylinder_nurbs_branch_endpoints(&hit, &cylinder, &surface);

    let swapped = intersect_bounded_surfaces(
        &surface,
        surface.param_range(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), 1);
    assert_eq!(swapped.curves[0].uv_a_start, hit.curves[0].uv_b_start);
    assert_eq!(swapped.curves[0].uv_b_start, hit.curves[0].uv_a_start);
}

#[test]
fn cone_nurbs_surface_marches_planar_patch_arc() {
    let cone = Cone::new(Frame::world(), 0.5, core::f64::consts::FRAC_PI_4).unwrap();
    let z = 0.5;
    let expected_v = z / core::f64::consts::FRAC_1_SQRT_2;
    let expected_radius = 1.0;
    let surface = bilinear_nurbs_surface_at_z(z);
    let hit = intersect_bounded_cone_nurbs_surface(
        &cone,
        cone_window(),
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert_eq!(hit.curves[0].kind, ContactKind::Transverse);
    assert_cone_nurbs_branch_endpoints(&hit, &cone, &surface);

    let SurfaceIntersectionCurve::Nurbs(curve) = &hit.curves[0].curve else {
        panic!("marched cone/NURBS-surface cut should be carried by a NURBS polyline");
    };
    assert_eq!(curve.degree(), 1);
    assert!(curve.points().len() >= 2);
    for point in curve.points() {
        assert!(((point.x * point.x + point.y * point.y).sqrt() - expected_radius).abs() < 1e-7);
        assert!((point.z - z).abs() < 1e-12);
    }

    let branch = &hit.curves[0];
    assert!((branch.uv_a_start[1] - expected_v).abs() < 1e-7);
    assert!((branch.uv_a_end[1] - expected_v).abs() < 1e-7);
    let u_min = branch.uv_a_start[0].min(branch.uv_a_end[0]);
    let u_max = branch.uv_a_start[0].max(branch.uv_a_end[0]);
    assert!(u_min.abs() < 1e-7);
    assert!((u_max - core::f64::consts::FRAC_PI_2).abs() < 1e-7);

    let miss_surface = bilinear_nurbs_surface_rect(0.0, 0.25, 0.0, 0.25, z);
    let miss = intersect_bounded_cone_nurbs_surface(
        &cone,
        cone_window(),
        &miss_surface,
        miss_surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let coincident = quarter_cone_nurbs_surface(&cone, 0.0, 0.75);
    let err = intersect_bounded_cone_nurbs_surface(
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            ParamRange::new(0.0, 0.75),
        ],
        &coincident,
        coincident.param_range(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "coincident cone/nurbs-surface intersection is a surface overlap"
        }
    );
}

#[test]
fn cone_nurbs_surface_dispatches_both_orders() {
    let cone = Cone::new(Frame::world(), 0.5, core::f64::consts::FRAC_PI_4).unwrap();
    let surface = bilinear_nurbs_surface_at_z(0.5);
    let hit = intersect_bounded_surfaces(
        &cone,
        cone_window(),
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 1);
    assert_cone_nurbs_branch_endpoints(&hit, &cone, &surface);

    let swapped = intersect_bounded_surfaces(
        &surface,
        surface.param_range(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), 1);
    assert_eq!(swapped.curves[0].uv_a_start, hit.curves[0].uv_b_start);
    assert_eq!(swapped.curves[0].uv_b_start, hit.curves[0].uv_a_start);
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

    let nurbs = bilinear_nurbs_surface();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let err = intersect_bounded_surfaces(
        &nurbs,
        nurbs.param_range(),
        &torus,
        torus_window(),
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
fn plane_torus_axis_normal_secant_returns_latitude_circles() {
    let plane = horizontal_plane(0.25);
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_plane_torus(
        &plane,
        torus_plane_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_plane_torus_branch_endpoints(&hit, &plane, &torus);

    let radial_delta = 3.0_f64.sqrt() / 4.0;
    let mut radii = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("axis-normal plane/torus cut should be carried by latitude circles");
        };
        assert!(circle.frame().origin().dist(Point3::new(0.0, 0.0, 0.25)) < 1e-12);
        radii.push(circle.radius());
    }
    radii.sort_by(f64::total_cmp);
    assert!((radii[0] - (2.0 - radial_delta)).abs() < 1e-12);
    assert!((radii[1] - (2.0 + radial_delta)).abs() < 1e-12);
}

#[test]
fn plane_torus_surface_windows_clip_latitude_circles() {
    let plane = horizontal_plane(0.0);
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_plane_torus(
        &plane,
        torus_plane_window(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::PI).abs() < 1e-12);
    assert_plane_torus_branch_endpoints(&hit, &plane, &torus);
    for branch in &hit.curves {
        assert!(branch.curve_range.lo.abs() < 1e-12);
        assert!((branch.curve_range.hi - core::f64::consts::PI).abs() < 1e-12);
    }
}

#[test]
fn plane_torus_meridian_returns_tube_circles() {
    let plane = vertical_plane_y(0.0);
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_plane_torus(
        &plane,
        torus_plane_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_plane_torus_branch_endpoints(&hit, &plane, &torus);

    let mut centers = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("meridian plane/torus cut should be carried by tube circles");
        };
        assert!((circle.radius() - 0.5).abs() < 1e-12);
        centers.push(circle.frame().origin());
    }
    centers.sort_by(|lhs, rhs| lhs.x.total_cmp(&rhs.x));
    assert!(centers[0].dist(Point3::new(-2.0, 0.0, 0.0)) < 1e-12);
    assert!(centers[1].dist(Point3::new(2.0, 0.0, 0.0)) < 1e-12);
}

#[test]
fn plane_torus_tangent_miss_and_unsupported_cases() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let tangent_plane = horizontal_plane(0.5);
    let tangent = intersect_bounded_plane_torus(
        &tangent_plane,
        torus_plane_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(tangent.points.is_empty());
    assert_eq!(tangent.curves.len(), 1);
    assert_eq!(tangent.curves[0].kind, ContactKind::Tangent);
    assert_plane_torus_branch_endpoints(&tangent, &tangent_plane, &torus);

    let miss = intersect_bounded_plane_torus(
        &horizontal_plane(0.75),
        torus_plane_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let offset_meridian = vertical_plane_y(0.25);
    let err = intersect_bounded_plane_torus(
        &offset_meridian,
        torus_plane_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "plane/torus intersection currently supports only axis-normal latitude circles or axis-containing meridian circles"
        }
    );

    let oblique = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.5, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let err = intersect_bounded_plane_torus(
        &oblique,
        torus_plane_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "plane/torus intersection currently supports only axis-normal latitude circles or axis-containing meridian circles"
        }
    );
}

#[test]
fn surface_surface_dispatches_plane_torus_both_orders() {
    let plane = horizontal_plane(0.0);
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_surfaces(
        &plane,
        torus_plane_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 2);
    assert_plane_torus_branch_endpoints(&hit, &plane, &torus);

    let swapped = intersect_bounded_surfaces(
        &torus,
        torus_window(),
        &plane,
        torus_plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), hit.curves.len());
    assert_eq!(swapped.curves[0].uv_a_start, hit.curves[0].uv_b_start);
    assert_eq!(swapped.curves[0].uv_b_start, hit.curves[0].uv_a_start);
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
fn cylinder_cylinder_parallel_secant_returns_ruling_lines() {
    let a = Cylinder::new(Frame::world(), 1.0).unwrap();
    let b = Cylinder::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let hit = intersect_bounded_cylinders(
        &a,
        cylinder_window(),
        &b,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 4.0).abs() < 1e-12);
    assert_cylinder_cylinder_branch_endpoints(&hit, &a, &b);

    let mut origins = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Line(line) = &branch.curve else {
            panic!("parallel cylinder/cylinder cut should be carried by ruling lines");
        };
        assert!(line.dir().cross(Vec3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
        origins.push(line.origin());
    }
    origins.sort_by(|lhs, rhs| lhs.y.total_cmp(&rhs.y));
    let h = 3.0_f64.sqrt() / 2.0;
    assert!(origins[0].dist(Point3::new(0.5, -h, 0.0)) < 1e-12);
    assert!(origins[1].dist(Point3::new(0.5, h, 0.0)) < 1e-12);
}

#[test]
fn cylinder_cylinder_surface_windows_clip_ruling_lines() {
    let a = Cylinder::new(Frame::world(), 1.0).unwrap();
    let b = Cylinder::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let hit = intersect_bounded_cylinders(
        &a,
        cylinder_window(),
        &b,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-0.75, -0.25),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 1.0).abs() < 1e-12);
    assert_cylinder_cylinder_branch_endpoints(&hit, &a, &b);
    for branch in &hit.curves {
        assert!((branch.curve_range.lo - 0.25).abs() < 1e-12);
        assert!((branch.curve_range.hi - 0.75).abs() < 1e-12);
    }
}

#[test]
fn cylinder_cylinder_window_boundary_contact_returns_points() {
    let a = Cylinder::new(Frame::world(), 1.0).unwrap();
    let b = Cylinder::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let hit = intersect_bounded_cylinders(
        &a,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.0, 1.0),
        ],
        &b,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(1.0, 2.0),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.curves.is_empty());
    assert_eq!(hit.points.len(), 2);
    for point in &hit.points {
        assert_eq!(point.kind, ContactKind::Transverse);
        assert!((point.point.z - 1.0).abs() < 1e-12);
        assert!(a.eval(point.uv_a).dist(point.point) < 1e-12);
        assert!(b.eval(point.uv_b).dist(point.point) < 1e-12);
    }
}

#[test]
fn cylinder_cylinder_tangent_miss_coincident_and_unsupported_cases() {
    let a = Cylinder::new(Frame::world(), 1.0).unwrap();
    let tangent_b = Cylinder::new(
        Frame::new(
            Point3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let tangent = intersect_bounded_cylinders(
        &a,
        cylinder_window(),
        &tangent_b,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(tangent.points.is_empty());
    assert_eq!(tangent.curves.len(), 1);
    assert_eq!(tangent.curves[0].kind, ContactKind::Tangent);
    assert_cylinder_cylinder_branch_endpoints(&tangent, &a, &tangent_b);

    let miss_b = Cylinder::new(
        Frame::new(
            Point3::new(2.5, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let miss = intersect_bounded_cylinders(
        &a,
        cylinder_window(),
        &miss_b,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let concentric_b = Cylinder::new(Frame::world(), 0.5).unwrap();
    let concentric = intersect_bounded_cylinders(
        &a,
        cylinder_window(),
        &concentric_b,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(concentric.is_empty());

    let err = intersect_bounded_cylinders(
        &a,
        cylinder_window(),
        &a,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "coincident cylinder/cylinder intersection is a surface overlap"
        }
    );

    let skew_b = Cylinder::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.5, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let err = intersect_bounded_cylinders(
        &a,
        cylinder_window(),
        &skew_b,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cylinder/cylinder intersection currently supports only parallel-axis ruling cuts"
        }
    );
}

#[test]
fn surface_surface_dispatches_cylinder_cylinder_both_orders() {
    let a = Cylinder::new(Frame::world(), 1.0).unwrap();
    let b = Cylinder::new(
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
        cylinder_window(),
        &b,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 2);
    assert_cylinder_cylinder_branch_endpoints(&hit, &a, &b);

    let swapped = intersect_bounded_surfaces(
        &b,
        cylinder_window(),
        &a,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), 2);
    assert_cylinder_cylinder_branch_endpoints(&swapped, &b, &a);
}

#[test]
fn cylinder_torus_coaxial_secant_returns_latitude_circles() {
    let cylinder = Cylinder::new(Frame::world(), 2.25).unwrap();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_cylinder_torus(
        &cylinder,
        cylinder_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_cylinder_torus_branch_endpoints(&hit, &cylinder, &torus);

    let mut centers = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("coaxial cylinder/torus cut should be carried by latitude circles");
        };
        assert!((circle.radius() - 2.25).abs() < 1e-12);
        centers.push(circle.frame().origin().z);
    }
    centers.sort_by(f64::total_cmp);
    let h = 3.0_f64.sqrt() / 4.0;
    assert!((centers[0] + h).abs() < 1e-12);
    assert!((centers[1] - h).abs() < 1e-12);
}

#[test]
fn cylinder_torus_surface_windows_clip_latitude_circles() {
    let cylinder = Cylinder::new(Frame::world(), 2.25).unwrap();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_cylinder_torus(
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, 1.0),
        ],
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!((total_curve_width(&hit) - core::f64::consts::PI).abs() < 1e-12);
    assert_cylinder_torus_branch_endpoints(&hit, &cylinder, &torus);
    assert!(hit.curves[0].curve_range.lo.abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn cylinder_torus_tangent_miss_and_unsupported_cases() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let tangent_cylinder = Cylinder::new(Frame::world(), 2.5).unwrap();
    let tangent = intersect_bounded_cylinder_torus(
        &tangent_cylinder,
        cylinder_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(tangent.points.is_empty());
    assert_eq!(tangent.curves.len(), 1);
    assert_eq!(tangent.curves[0].kind, ContactKind::Tangent);
    assert_cylinder_torus_branch_endpoints(&tangent, &tangent_cylinder, &torus);

    let miss_cylinder = Cylinder::new(Frame::world(), 3.0).unwrap();
    let miss = intersect_bounded_cylinder_torus(
        &miss_cylinder,
        cylinder_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let shifted_cylinder = Cylinder::new(
        Frame::new(
            Point3::new(0.25, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.25,
    )
    .unwrap();
    let err = intersect_bounded_cylinder_torus(
        &shifted_cylinder,
        cylinder_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cylinder/torus intersection currently supports only coaxial circular cuts"
        }
    );

    let tilted_cylinder = Cylinder::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.5, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.25,
    )
    .unwrap();
    let err = intersect_bounded_cylinder_torus(
        &tilted_cylinder,
        cylinder_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cylinder/torus intersection currently supports only coaxial circular cuts"
        }
    );
}

#[test]
fn surface_surface_dispatches_cylinder_torus_both_orders() {
    let cylinder = Cylinder::new(Frame::world(), 2.25).unwrap();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_surfaces(
        &cylinder,
        cylinder_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 2);
    assert_cylinder_torus_branch_endpoints(&hit, &cylinder, &torus);

    let swapped = intersect_bounded_surfaces(
        &torus,
        torus_window(),
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), hit.curves.len());
    for branch in &swapped.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(torus.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(torus.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(cylinder.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(cylinder.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

#[test]
fn cone_cylinder_coaxial_secant_returns_circle_branches() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let cylinder = Cylinder::new(Frame::world(), 0.5).unwrap();
    let cone_range = [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-4.0, 0.0),
    ];
    let cylinder_range = [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-3.0, 0.0),
    ];
    let hit = intersect_bounded_cone_cylinder(
        &cone,
        cone_range,
        &cylinder,
        cylinder_range,
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 3);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_cone_cylinder_branch_endpoints(&hit, &cone, &cylinder);

    let mut centers = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("coaxial cone/cylinder cut should be carried by circles");
        };
        assert!((circle.radius() - 0.5).abs() < 1e-12);
        if !centers
            .iter()
            .any(|z: &f64| (*z - circle.frame().origin().z).abs() < 1e-12)
        {
            centers.push(circle.frame().origin().z);
        }
    }
    centers.sort_by(f64::total_cmp);
    assert_eq!(centers.len(), 2);
    let c = cos_pi_over_six();
    assert!((centers[0] + 3.0 * c).abs() < 1e-12);
    assert!((centers[1] + c).abs() < 1e-12);
}

#[test]
fn cone_cylinder_surface_windows_clip_circle_branch() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let cylinder = Cylinder::new(Frame::world(), 0.5).unwrap();
    let hit = intersect_bounded_cone_cylinder(
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-1.5, -0.5),
        ],
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-1.0, 0.0),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!((total_curve_width(&hit) - core::f64::consts::PI).abs() < 1e-12);
    assert_cone_cylinder_branch_endpoints(&hit, &cone, &cylinder);
    assert!(hit.curves[0].curve_range.lo.abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn cone_cylinder_window_miss_and_unsupported_cases() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let cylinder = Cylinder::new(Frame::world(), 0.5).unwrap();
    let miss = intersect_bounded_cone_cylinder(
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.0, 1.0),
        ],
        &cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let shifted_cylinder = Cylinder::new(
        Frame::new(
            Point3::new(0.25, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        0.5,
    )
    .unwrap();
    let err = intersect_bounded_cone_cylinder(
        &cone,
        cone_window(),
        &shifted_cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cone/cylinder intersection currently supports only coaxial circular cuts"
        }
    );

    let tilted_cylinder = Cylinder::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.5, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        0.5,
    )
    .unwrap();
    let err = intersect_bounded_cone_cylinder(
        &cone,
        cone_window(),
        &tilted_cylinder,
        cylinder_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cone/cylinder intersection currently supports only coaxial circular cuts"
        }
    );
}

#[test]
fn surface_surface_dispatches_cone_cylinder_both_orders() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let cylinder = Cylinder::new(Frame::world(), 0.5).unwrap();
    let cone_range = [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-4.0, 0.0),
    ];
    let cylinder_range = [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(-3.0, 0.0),
    ];
    let hit = intersect_bounded_surfaces(
        &cone,
        cone_range,
        &cylinder,
        cylinder_range,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 3);
    assert_cone_cylinder_branch_endpoints(&hit, &cone, &cylinder);

    let swapped = intersect_bounded_surfaces(
        &cylinder,
        cylinder_range,
        &cone,
        cone_range,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), hit.curves.len());
    for branch in &swapped.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(cylinder.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(cylinder.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(cone.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(cone.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

#[test]
fn cone_cone_coaxial_secant_returns_circle_branches() {
    let a = Cone::new(Frame::world(), 2.0, core::f64::consts::FRAC_PI_4).unwrap();
    let b = Cone::new(Frame::world(), 2.0, math::atan2(1.0, 2.0)).unwrap();
    let hit = intersect_bounded_cones(
        &a,
        wide_cone_window(),
        &b,
        wide_cone_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 3);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_cone_cone_branch_endpoints(&hit, &a, &b);

    let mut centers = Vec::new();
    let mut radii = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("coaxial cone/cone cut should be carried by latitude circles");
        };
        if !centers
            .iter()
            .any(|z: &f64| (*z - circle.frame().origin().z).abs() < 1e-12)
        {
            centers.push(circle.frame().origin().z);
        }
        if !radii
            .iter()
            .any(|radius: &f64| (*radius - circle.radius()).abs() < 1e-12)
        {
            radii.push(circle.radius());
        }
    }
    centers.sort_by(f64::total_cmp);
    radii.sort_by(f64::total_cmp);
    assert_eq!(centers.len(), 2);
    assert_eq!(radii.len(), 2);
    assert!((centers[0] + 8.0 / 3.0).abs() < 1e-12);
    assert!(centers[1].abs() < 1e-12);
    assert!((radii[0] - 2.0 / 3.0).abs() < 1e-12);
    assert!((radii[1] - 2.0).abs() < 1e-12);
}

#[test]
fn cone_cone_surface_windows_clip_circle_branch() {
    let a = Cone::new(Frame::world(), 2.0, core::f64::consts::FRAC_PI_4).unwrap();
    let b = Cone::new(Frame::world(), 2.0, math::atan2(1.0, 2.0)).unwrap();
    let hit = intersect_bounded_cones(
        &a,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(-0.25, 0.25),
        ],
        &b,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-0.25, 0.25),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!((total_curve_width(&hit) - core::f64::consts::PI).abs() < 1e-12);
    assert_cone_cone_branch_endpoints(&hit, &a, &b);
    assert!(hit.curves[0].curve_range.lo.abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn cone_cone_shared_apex_window_miss_overlap_and_unsupported_cases() {
    let a = Cone::new(Frame::world(), 1.0, core::f64::consts::FRAC_PI_4).unwrap();
    let b = Cone::new(Frame::world(), 2.0, math::atan2(2.0, 1.0)).unwrap();
    let apex = intersect_bounded_cones(
        &a,
        wide_cone_window(),
        &b,
        wide_cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(apex.curves.is_empty());
    assert_eq!(apex.points.len(), 1);
    assert_eq!(apex.points[0].kind, ContactKind::Singular);
    assert!(apex.points[0].point.dist(Point3::new(0.0, 0.0, -1.0)) < 1e-12);

    let miss = intersect_bounded_cones(
        &a,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.25, 0.75),
        ],
        &b,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.25, 0.75),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let err = intersect_bounded_cones(
        &a,
        wide_cone_window(),
        &a,
        wide_cone_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "coincident cone/cone intersection is a surface overlap"
        }
    );

    let shifted = Cone::new(
        Frame::new(
            Point3::new(0.25, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        math::atan2(1.0, 2.0),
    )
    .unwrap();
    let err = intersect_bounded_cones(
        &a,
        wide_cone_window(),
        &shifted,
        wide_cone_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cone/cone intersection currently supports only coaxial circular cuts"
        }
    );

    let tilted = Cone::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.5, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        math::atan2(1.0, 2.0),
    )
    .unwrap();
    let err = intersect_bounded_cones(
        &a,
        wide_cone_window(),
        &tilted,
        wide_cone_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cone/cone intersection currently supports only coaxial circular cuts"
        }
    );
}

#[test]
fn cone_cone_accepts_antiparallel_coaxial_axes() {
    let a = Cone::new(Frame::world(), 2.0, core::f64::consts::FRAC_PI_4).unwrap();
    let b = Cone::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        math::atan2(1.0, 2.0),
    )
    .unwrap();
    let hit = intersect_bounded_cones(&a, cone_window(), &b, cone_window(), Tolerances::default())
        .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!((total_curve_width(&hit) - core::f64::consts::TAU).abs() < 1e-12);
    assert_cone_cone_branch_endpoints(&hit, &a, &b);
}

#[test]
fn surface_surface_dispatches_cone_cone_both_orders() {
    let a = Cone::new(Frame::world(), 2.0, core::f64::consts::FRAC_PI_4).unwrap();
    let b = Cone::new(Frame::world(), 2.0, math::atan2(1.0, 2.0)).unwrap();
    let hit = intersect_bounded_surfaces(
        &a,
        wide_cone_window(),
        &b,
        wide_cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 3);
    assert_cone_cone_branch_endpoints(&hit, &a, &b);

    let swapped = intersect_bounded_surfaces(
        &b,
        wide_cone_window(),
        &a,
        wide_cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), hit.curves.len());
    assert_cone_cone_branch_endpoints(&swapped, &b, &a);
}

#[test]
fn cone_torus_coaxial_secant_returns_latitude_circles() {
    let cone = Cone::new(Frame::world(), 2.0, core::f64::consts::FRAC_PI_4).unwrap();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_cone_torus(
        &cone,
        cone_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_cone_torus_branch_endpoints(&hit, &cone, &torus);

    let h = core::f64::consts::FRAC_1_SQRT_2 / 2.0;
    let mut centers = Vec::new();
    let mut radii = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("coaxial cone/torus cut should be carried by latitude circles");
        };
        centers.push(circle.frame().origin().z);
        radii.push(circle.radius());
    }
    centers.sort_by(f64::total_cmp);
    radii.sort_by(f64::total_cmp);
    assert!((centers[0] + h).abs() < 1e-12);
    assert!((centers[1] - h).abs() < 1e-12);
    assert!((radii[0] - (2.0 - h)).abs() < 1e-12);
    assert!((radii[1] - (2.0 + h)).abs() < 1e-12);
}

#[test]
fn cone_torus_surface_windows_clip_latitude_circle() {
    let cone = Cone::new(Frame::world(), 2.0, core::f64::consts::FRAC_PI_4).unwrap();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_cone_torus(
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, 1.0),
        ],
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!((total_curve_width(&hit) - core::f64::consts::PI).abs() < 1e-12);
    assert_cone_torus_branch_endpoints(&hit, &cone, &torus);
    assert!(hit.curves[0].curve_range.lo.abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn cone_torus_accepts_antiparallel_coaxial_axes() {
    let cone = Cone::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        core::f64::consts::FRAC_PI_4,
    )
    .unwrap();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_cone_torus(
        &cone,
        cone_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_cone_torus_branch_endpoints(&hit, &cone, &torus);
}

#[test]
fn cone_torus_tangent_miss_and_unsupported_cases() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let tangent_cone = Cone::new(
        Frame::world(),
        2.0 + core::f64::consts::FRAC_1_SQRT_2,
        core::f64::consts::FRAC_PI_4,
    )
    .unwrap();
    let tangent = intersect_bounded_cone_torus(
        &tangent_cone,
        cone_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(tangent.points.is_empty());
    assert_eq!(tangent.curves.len(), 1);
    assert_eq!(tangent.curves[0].kind, ContactKind::Tangent);
    assert_cone_torus_branch_endpoints(&tangent, &tangent_cone, &torus);

    let miss_cone = Cone::new(Frame::world(), 3.0, core::f64::consts::FRAC_PI_4).unwrap();
    let miss = intersect_bounded_cone_torus(
        &miss_cone,
        cone_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let shifted_cone = Cone::new(
        Frame::new(
            Point3::new(0.25, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        core::f64::consts::FRAC_PI_4,
    )
    .unwrap();
    let err = intersect_bounded_cone_torus(
        &shifted_cone,
        cone_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cone/torus intersection currently supports only coaxial circular cuts"
        }
    );

    let tilted_cone = Cone::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.5, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        core::f64::consts::FRAC_PI_4,
    )
    .unwrap();
    let err = intersect_bounded_cone_torus(
        &tilted_cone,
        cone_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cone/torus intersection currently supports only coaxial circular cuts"
        }
    );
}

#[test]
fn surface_surface_dispatches_cone_torus_both_orders() {
    let cone = Cone::new(Frame::world(), 2.0, core::f64::consts::FRAC_PI_4).unwrap();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_surfaces(
        &cone,
        cone_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 2);
    assert_cone_torus_branch_endpoints(&hit, &cone, &torus);

    let swapped = intersect_bounded_surfaces(
        &torus,
        torus_window(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), hit.curves.len());
    for branch in &swapped.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(torus.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(torus.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(cone.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(cone.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

#[test]
fn plane_cone_perpendicular_cut_returns_circle_branch() {
    let plane = horizontal_plane(0.0);
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let hit = intersect_bounded_plane_cone(
        &plane,
        wide_plane_window(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert_eq!(hit.curves[0].kind, ContactKind::Transverse);
    assert!(hit.curves[0].curve_range.lo.abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - core::f64::consts::TAU).abs() < 1e-12);
    assert_plane_cone_branch_endpoints(&hit, &plane, &cone);

    let SurfaceIntersectionCurve::Circle(circle) = &hit.curves[0].curve else {
        panic!("axis-perpendicular plane/cone cut should be a circle");
    };
    assert!(circle.frame().origin().dist(Point3::new(0.0, 0.0, 0.0)) < 1e-12);
    assert!((circle.radius() - 1.0).abs() < 1e-12);
}

#[test]
fn plane_cone_surface_windows_clip_circle_branch() {
    let plane = horizontal_plane(0.0);
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let hit = intersect_bounded_plane_cone(
        &plane,
        wide_plane_window(),
        &cone,
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
    assert_plane_cone_branch_endpoints(&hit, &plane, &cone);
    assert_eq!(hit.curves[0].uv_b_start, [0.0, 0.0]);
    assert!((hit.curves[0].uv_b_end[0] - core::f64::consts::PI).abs() < 1e-12);
    assert_eq!(hit.curves[0].uv_b_end[1], 0.0);
}

#[test]
fn plane_cone_oblique_elliptic_cut_returns_ellipse_branch() {
    let slope = 0.5_f64;
    let plane = cone_plane_with_slope(slope);
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let hit = intersect_bounded_plane_cone(
        &plane,
        wide_plane_window(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();

    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let tan_a = sin_a / cos_a;
    let k = slope * tan_a;
    let axial = 1.0 - k * k;
    let center_x = cone.radius() * k / axial;
    let expected_major = cone.radius() * (1.0 + slope * slope).sqrt() / axial;
    let expected_minor = cone.radius() / axial.sqrt();

    assert!(hit.points.is_empty());
    assert!(!hit.curves.is_empty());
    assert!((total_curve_width(&hit) - core::f64::consts::TAU).abs() < 1e-12);
    assert_plane_cone_branch_endpoints(&hit, &plane, &cone);

    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Ellipse(ellipse) = &branch.curve else {
            panic!("oblique elliptic plane/cone cut should be an ellipse");
        };
        assert!(
            ellipse
                .frame()
                .origin()
                .dist(Point3::new(center_x, 0.0, slope * center_x))
                < 1e-12
        );
        assert!((ellipse.major_radius() - expected_major).abs() < 1e-12);
        assert!((ellipse.minor_radius() - expected_minor).abs() < 1e-12);
    }
}

#[test]
fn plane_cone_apex_contact_and_v_window_filtering() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let plane = horizontal_plane(cone.apex().z);
    let apex_window = [
        ParamRange::new(0.0, core::f64::consts::TAU),
        ParamRange::new(cone.apex_v(), cone.apex_v()),
    ];
    let hit = intersect_bounded_plane_cone(
        &plane,
        plane_window(),
        &cone,
        apex_window,
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.curves.is_empty());
    assert_eq!(hit.points.len(), 1);
    assert_eq!(hit.points[0].kind, ContactKind::Singular);
    assert!(hit.points[0].point.dist(cone.apex()) < 1e-12);
    assert_eq!(hit.points[0].uv_a, [0.0, 0.0]);
    assert_eq!(hit.points[0].uv_b, [0.0, cone.apex_v()]);

    let miss = intersect_bounded_plane_cone(
        &plane,
        plane_window(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());
}

#[test]
fn surface_surface_dispatches_plane_cone_circles_and_ellipses() {
    let plane = horizontal_plane(0.0);
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let hit = intersect_bounded_surfaces(
        &plane,
        wide_plane_window(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 1);

    let swapped = intersect_bounded_surfaces(
        &cone,
        cone_window(),
        &plane,
        wide_plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), 1);
    assert_eq!(swapped.curves[0].uv_a_start, hit.curves[0].uv_b_start);
    assert_eq!(swapped.curves[0].uv_b_start, hit.curves[0].uv_a_start);

    let elliptic = intersect_bounded_surfaces(
        &cone_plane_with_slope(0.5),
        wide_plane_window(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(!elliptic.curves.is_empty());
    assert!((total_curve_width(&elliptic) - core::f64::consts::TAU).abs() < 1e-12);
    assert!(
        elliptic
            .curves
            .iter()
            .all(|branch| matches!(branch.curve, SurfaceIntersectionCurve::Ellipse(_)))
    );

    let swapped_elliptic = intersect_bounded_surfaces(
        &cone,
        cone_window(),
        &cone_plane_with_slope(0.5),
        wide_plane_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped_elliptic.curves.len(), elliptic.curves.len());
    assert_eq!(
        swapped_elliptic.curves[0].uv_a_start,
        elliptic.curves[0].uv_b_start
    );
    assert_eq!(
        swapped_elliptic.curves[0].uv_b_start,
        elliptic.curves[0].uv_a_start
    );
}

#[test]
fn plane_cone_rejects_unsupported_parabolic_and_hyperbolic_sections() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    for slope in [cos_a / sin_a, 3.0] {
        let err = intersect_bounded_surfaces(
            &cone_plane_with_slope(slope),
            wide_plane_window(),
            &cone,
            cone_window(),
            Tolerances::default(),
        )
        .unwrap_err();
        assert_eq!(
            err,
            Error::InvalidGeometry {
                reason: "plane/cone intersection currently supports only circular and elliptic cuts"
            }
        );
    }
}

#[test]
fn cone_sphere_coaxial_secant_returns_circle_branches() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let sphere = Sphere::new(
        Frame::new(
            cone.apex(),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
    )
    .unwrap();
    let hit = intersect_bounded_cone_sphere(
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(cone.apex_v() - 2.1, cone.apex_v() + 2.1),
        ],
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();

    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let radius = 2.0 * sin_a;
    let z_offset = 2.0 * cos_a;
    assert!(hit.points.is_empty());
    assert!(!hit.curves.is_empty());
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_cone_sphere_branch_endpoints(&hit, &cone, &sphere);

    let mut centers = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("coaxial cone/sphere secant should be carried by circles");
        };
        assert!((circle.radius() - radius).abs() < 1e-12);
        centers.push(circle.frame().origin().z);
    }
    centers.sort_by(f64::total_cmp);
    centers.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
    assert_eq!(centers.len(), 2);
    assert!((centers[0] - (cone.apex().z - z_offset)).abs() < 1e-12);
    assert!((centers[1] - (cone.apex().z + z_offset)).abs() < 1e-12);
}

#[test]
fn cone_sphere_surface_windows_clip_circle_branches() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let sphere = Sphere::new(
        Frame::new(
            cone.apex(),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
    )
    .unwrap();
    let hit = intersect_bounded_cone_sphere(
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(cone.apex_v(), cone.apex_v() + 2.1),
        ],
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!((total_curve_width(&hit) - core::f64::consts::PI).abs() < 1e-12);
    assert_cone_sphere_branch_endpoints(&hit, &cone, &sphere);

    let SurfaceIntersectionCurve::Circle(circle) = &hit.curves[0].curve else {
        panic!("coaxial cone/sphere secant should be carried by circle segments");
    };
    assert!(circle.frame().origin().z > cone.apex().z);
}

#[test]
fn cone_sphere_tangent_apex_miss_and_unsupported_cases() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let (sin_a, cos_a) = math::sincos(cone.half_angle());

    let tangent_sphere = Sphere::new(Frame::world(), cos_a).unwrap();
    let tangent = intersect_bounded_cone_sphere(
        &cone,
        cone_window(),
        &tangent_sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(tangent.points.is_empty());
    assert_eq!(tangent.curves.len(), 1);
    assert_eq!(tangent.curves[0].kind, ContactKind::Tangent);
    assert_cone_sphere_branch_endpoints(&tangent, &cone, &tangent_sphere);

    let apex_sphere = Sphere::new(Frame::world(), cos_a / sin_a).unwrap();
    let apex = intersect_bounded_cone_sphere(
        &cone,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(cone.apex_v(), 1.0),
        ],
        &apex_sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(apex.points.len(), 1);
    assert_eq!(apex.points[0].kind, ContactKind::Singular);
    assert!(apex.points[0].point.dist(cone.apex()) < 1e-12);
    assert_eq!(apex.curves.len(), 1);
    assert_cone_sphere_branch_endpoints(&apex, &cone, &apex_sphere);

    let miss_sphere = Sphere::new(Frame::world(), 0.1).unwrap();
    let miss = intersect_bounded_cone_sphere(
        &cone,
        cone_window(),
        &miss_sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let shifted_sphere = Sphere::new(
        Frame::new(
            Point3::new(0.25, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let err = intersect_bounded_cone_sphere(
        &cone,
        cone_window(),
        &shifted_sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cone/sphere intersection currently supports only coaxial circular cuts"
        }
    );
}

#[test]
fn surface_surface_dispatches_cone_sphere_both_orders() {
    let cone = Cone::new(Frame::world(), 1.0, core::f64::consts::PI / 6.0).unwrap();
    let sphere = Sphere::new(Frame::world(), cos_pi_over_six()).unwrap();
    let hit = intersect_bounded_surfaces(
        &cone,
        cone_window(),
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 1);

    let swapped = intersect_bounded_surfaces(
        &sphere,
        sphere_window(),
        &cone,
        cone_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), hit.curves.len());
    assert_eq!(swapped.curves[0].uv_a_start, hit.curves[0].uv_b_start);
    assert_eq!(swapped.curves[0].uv_b_start, hit.curves[0].uv_a_start);
}

fn cos_pi_over_six() -> f64 {
    let (_, cos_a) = math::sincos(core::f64::consts::PI / 6.0);
    cos_a
}

#[test]
fn cylinder_sphere_coaxial_secant_returns_circle_branches() {
    let cylinder = Cylinder::new(Frame::world(), 0.5).unwrap();
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_cylinder_sphere(
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-1.0, 1.0),
        ],
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();

    let h = 3.0_f64.sqrt() / 2.0;
    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_cylinder_sphere_branch_endpoints(&hit, &cylinder, &sphere);

    let mut centers = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("coaxial cylinder/sphere secant should be carried by circles");
        };
        assert!((circle.radius() - 0.5).abs() < 1e-12);
        centers.push(circle.frame().origin().z);
    }
    centers.sort_by(f64::total_cmp);
    assert!((centers[0] + h).abs() < 1e-12);
    assert!((centers[1] - h).abs() < 1e-12);
}

#[test]
fn cylinder_sphere_surface_windows_clip_circle_branches() {
    let cylinder = Cylinder::new(Frame::world(), 0.5).unwrap();
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_cylinder_sphere(
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, 1.0),
        ],
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();

    let h = 3.0_f64.sqrt() / 2.0;
    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!((total_curve_width(&hit) - core::f64::consts::PI).abs() < 1e-12);
    assert_cylinder_sphere_branch_endpoints(&hit, &cylinder, &sphere);

    let SurfaceIntersectionCurve::Circle(circle) = &hit.curves[0].curve else {
        panic!("coaxial cylinder/sphere secant should be carried by circle segments");
    };
    assert!((circle.frame().origin().z - h).abs() < 1e-12);
}

#[test]
fn cylinder_sphere_tangent_miss_and_unsupported_cases() {
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let tangent_cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    let tangent = intersect_bounded_cylinder_sphere(
        &tangent_cylinder,
        cylinder_window(),
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(tangent.points.is_empty());
    assert_eq!(tangent.curves.len(), 1);
    assert_eq!(tangent.curves[0].kind, ContactKind::Tangent);
    assert_cylinder_sphere_branch_endpoints(&tangent, &tangent_cylinder, &sphere);

    let miss_cylinder = Cylinder::new(Frame::world(), 1.5).unwrap();
    let miss = intersect_bounded_cylinder_sphere(
        &miss_cylinder,
        cylinder_window(),
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let shifted_sphere = Sphere::new(
        Frame::new(
            Point3::new(0.25, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let err = intersect_bounded_cylinder_sphere(
        &tangent_cylinder,
        cylinder_window(),
        &shifted_sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "cylinder/sphere intersection currently supports only coaxial circular cuts"
        }
    );
}

#[test]
fn surface_surface_dispatches_cylinder_sphere_both_orders() {
    let cylinder = Cylinder::new(Frame::world(), 0.5).unwrap();
    let sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let hit = intersect_bounded_surfaces(
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-1.0, 1.0),
        ],
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 2);

    let swapped = intersect_bounded_surfaces(
        &sphere,
        sphere_window(),
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-1.0, 1.0),
        ],
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), hit.curves.len());
    assert_eq!(swapped.curves[0].uv_a_start, hit.curves[0].uv_b_start);
    assert_eq!(swapped.curves[0].uv_b_start, hit.curves[0].uv_a_start);
}

#[test]
fn sphere_torus_coaxial_secant_returns_circle_branches() {
    let sphere = Sphere::new(Frame::world(), 2.0).unwrap();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_sphere_torus(
        &sphere,
        sphere_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_sphere_torus_branch_endpoints(&hit, &sphere, &torus);

    let mut centers = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("coaxial sphere/torus secant should be carried by latitude circles");
        };
        assert!((circle.radius() - 31.0 / 16.0).abs() < 1e-12);
        centers.push(circle.frame().origin().z);
    }
    centers.sort_by(f64::total_cmp);
    let h = 63.0_f64.sqrt() / 16.0;
    assert!((centers[0] + h).abs() < 1e-12);
    assert!((centers[1] - h).abs() < 1e-12);
}

#[test]
fn sphere_torus_surface_windows_clip_circle_branches() {
    let sphere = Sphere::new(Frame::world(), 2.0).unwrap();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_sphere_torus(
        &sphere,
        sphere_window(),
        &torus,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ],
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::PI).abs() < 1e-12);
    assert_sphere_torus_branch_endpoints(&hit, &sphere, &torus);
    for branch in &hit.curves {
        assert!(branch.curve_range.lo.abs() < 1e-12);
        assert!((branch.curve_range.hi - core::f64::consts::PI).abs() < 1e-12);
    }
}

#[test]
fn sphere_torus_tangent_miss_and_unsupported_cases() {
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let tangent_sphere = Sphere::new(Frame::world(), 1.5).unwrap();
    let tangent = intersect_bounded_sphere_torus(
        &tangent_sphere,
        sphere_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(tangent.points.is_empty());
    assert_eq!(tangent.curves.len(), 1);
    assert_eq!(tangent.curves[0].kind, ContactKind::Tangent);
    assert_sphere_torus_branch_endpoints(&tangent, &tangent_sphere, &torus);

    let miss_sphere = Sphere::new(Frame::world(), 1.0).unwrap();
    let miss = intersect_bounded_sphere_torus(
        &miss_sphere,
        sphere_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let shifted_sphere = Sphere::new(
        Frame::new(
            Point3::new(0.25, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
    )
    .unwrap();
    let err = intersect_bounded_sphere_torus(
        &shifted_sphere,
        sphere_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "sphere/torus intersection currently supports only coaxial circular cuts"
        }
    );
}

#[test]
fn surface_surface_dispatches_sphere_torus_both_orders() {
    let sphere = Sphere::new(Frame::world(), 2.0).unwrap();
    let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let hit = intersect_bounded_surfaces(
        &sphere,
        sphere_window(),
        &torus,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 2);
    assert_sphere_torus_branch_endpoints(&hit, &sphere, &torus);

    let swapped = intersect_bounded_surfaces(
        &torus,
        torus_window(),
        &sphere,
        sphere_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(swapped.curves.len(), hit.curves.len());
    for branch in &swapped.curves {
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(torus.eval(branch.uv_a_start).dist(start) < 1e-12);
        assert!(torus.eval(branch.uv_a_end).dist(end) < 1e-12);
        assert!(sphere.eval(branch.uv_b_start).dist(start) < 1e-12);
        assert!(sphere.eval(branch.uv_b_end).dist(end) < 1e-12);
    }
}

#[test]
fn torus_torus_coaxial_secant_returns_latitude_circles() {
    let a = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let b = Torus::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let hit = intersect_bounded_tori(
        &a,
        torus_window(),
        &b,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_torus_torus_branch_endpoints(&hit, &a, &b);

    let radial_delta = 3.0_f64.sqrt() / 4.0;
    let mut radii = Vec::new();
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Transverse);
        let SurfaceIntersectionCurve::Circle(circle) = &branch.curve else {
            panic!("coaxial torus/torus secant should be carried by latitude circles");
        };
        assert!((circle.frame().origin().z - 0.25).abs() < 1e-12);
        radii.push(circle.radius());
    }
    radii.sort_by(f64::total_cmp);
    assert!((radii[0] - (2.0 - radial_delta)).abs() < 1e-12);
    assert!((radii[1] - (2.0 + radial_delta)).abs() < 1e-12);
}

#[test]
fn torus_torus_surface_windows_clip_latitude_circle() {
    let a = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let b = Torus::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let hit = intersect_bounded_tori(
        &a,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
        ],
        &b,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert!((total_curve_width(&hit) - core::f64::consts::PI).abs() < 1e-12);
    assert_torus_torus_branch_endpoints(&hit, &a, &b);
    assert!(hit.curves[0].curve_range.lo.abs() < 1e-12);
    assert!((hit.curves[0].curve_range.hi - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn torus_torus_accepts_antiparallel_coaxial_axes() {
    let a = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let b = Torus::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let hit = intersect_bounded_tori(
        &a,
        torus_window(),
        &b,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();

    assert!(hit.points.is_empty());
    assert_eq!(hit.curves.len(), 2);
    assert!((total_curve_width(&hit) - 2.0 * core::f64::consts::TAU).abs() < 1e-12);
    assert_torus_torus_branch_endpoints(&hit, &a, &b);
}

#[test]
fn torus_torus_tangent_miss_overlap_and_unsupported_cases() {
    let a = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let tangent_b = Torus::new(
        Frame::new(
            Point3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let tangent = intersect_bounded_tori(
        &a,
        torus_window(),
        &tangent_b,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(tangent.points.is_empty());
    assert_eq!(tangent.curves.len(), 1);
    assert_eq!(tangent.curves[0].kind, ContactKind::Tangent);
    assert_torus_torus_branch_endpoints(&tangent, &a, &tangent_b);

    let miss_b = Torus::new(
        Frame::new(
            Point3::new(0.0, 0.0, 1.25),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let miss = intersect_bounded_tori(
        &a,
        torus_window(),
        &miss_b,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_empty());

    let err = intersect_bounded_tori(
        &a,
        torus_window(),
        &a,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "coincident torus/torus intersection is a surface overlap"
        }
    );

    let shifted = Torus::new(
        Frame::new(
            Point3::new(0.25, 0.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let err = intersect_bounded_tori(
        &a,
        torus_window(),
        &shifted,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "torus/torus intersection currently supports only coaxial circular cuts"
        }
    );

    let tilted = Torus::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.5, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let err = intersect_bounded_tori(
        &a,
        torus_window(),
        &tilted,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        err,
        Error::InvalidGeometry {
            reason: "torus/torus intersection currently supports only coaxial circular cuts"
        }
    );
}

#[test]
fn surface_surface_dispatches_torus_torus() {
    let a = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
    let b = Torus::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let hit = intersect_bounded_surfaces(
        &a,
        torus_window(),
        &b,
        torus_window(),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(hit.curves.len(), 2);
    assert_torus_torus_branch_endpoints(&hit, &a, &b);
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
