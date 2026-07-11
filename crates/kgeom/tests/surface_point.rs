//! Focused coverage for shared point-to-surface services.

use core::f64::consts::{FRAC_PI_2, TAU};

use kgeom::frame::Frame;
use kgeom::nurbs::NurbsSurface;
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};
use kgeom::surface_point::{
    SurfacePointMethod, distance_to_surface, invert_surface_point, normalize_surface_uv,
};
use kgeom::vec::{Point3, Vec3};

fn tilted_frame() -> Frame {
    Frame::new(
        Point3::new(0.3, -1.2, 2.1),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap()
}

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:?} differs from {expected:?} by more than {tolerance:?}"
    );
}

fn assert_analytic_surface(
    surface: &dyn Surface,
    uv: [f64; 2],
    expected_raw: [f64; 2],
    expected_base: [f64; 2],
) {
    let point = surface.eval(uv);
    let mapped = invert_surface_point(surface, point).unwrap();
    assert_eq!(mapped.method, SurfacePointMethod::Analytic);
    assert_close(mapped.uv[0], expected_raw[0], 2.0e-12);
    assert_close(mapped.uv[1], expected_raw[1], 2.0e-12);

    let base = normalize_surface_uv(surface, mapped.uv);
    assert_close(base[0], expected_base[0], 2.0e-12);
    assert_close(base[1], expected_base[1], 2.0e-12);

    let on_surface = distance_to_surface(surface, point).unwrap();
    assert_eq!(on_surface.method, SurfacePointMethod::Analytic);
    assert_close(on_surface.distance, 0.0, 2.0e-12);

    let normal = surface.normal(uv).unwrap();
    let offset = distance_to_surface(surface, point + normal * 0.25).unwrap();
    assert_eq!(offset.method, SurfacePointMethod::Analytic);
    assert_close(offset.distance, 0.25, 2.0e-12);
}

#[test]
fn analytic_classes_share_raw_inversion_normalization_and_distance_semantics() {
    let frame = tilted_frame();
    let plane = Plane::new(frame);
    assert_analytic_surface(&plane, [-1.2, 2.3], [-1.2, 2.3], [-1.2, 2.3]);

    let cylinder = Cylinder::new(frame, 1.7).unwrap();
    assert_analytic_surface(&cylinder, [-0.25, 1.1], [-0.25, 1.1], [TAU - 0.25, 1.1]);

    let cone = Cone::new(frame, 1.4, 0.35).unwrap();
    assert_analytic_surface(&cone, [-0.4, 0.7], [-0.4, 0.7], [TAU - 0.4, 0.7]);

    let sphere = Sphere::new(frame, 2.2).unwrap();
    assert_analytic_surface(&sphere, [-0.3, 0.4], [-0.3, 0.4], [TAU - 0.3, 0.4]);

    let torus = Torus::new(frame, 3.0, 0.8).unwrap();
    assert_analytic_surface(&torus, [-0.2, -0.6], [-0.2, -0.6], [TAU - 0.2, TAU - 0.6]);
}

#[test]
fn periodic_seams_and_analytic_singularities_are_deterministic() {
    let frame = Frame::world();
    let cylinder = Cylinder::new(frame, 2.0).unwrap();
    let near_seam = invert_surface_point(&cylinder, cylinder.eval([-1.0e-8, 0.75])).unwrap();
    assert!(near_seam.uv[0] < 0.0);
    let normalized = normalize_surface_uv(&cylinder, near_seam.uv);
    assert_close(normalized[0], TAU - 1.0e-8, 2.0e-12);

    let sphere = Sphere::new(frame, 1.5).unwrap();
    let north = frame.origin() + frame.z() * sphere.radius();
    let north_uv = invert_surface_point(&sphere, north).unwrap();
    assert_eq!(north_uv.method, SurfacePointMethod::Analytic);
    assert_close(north_uv.uv[1], FRAC_PI_2, 1.0e-15);
    assert_eq!(normalize_surface_uv(&sphere, north_uv.uv), north_uv.uv);

    let cone = Cone::new(frame, 1.25, 0.4).unwrap();
    let apex_uv = invert_surface_point(&cone, cone.apex()).unwrap();
    assert_eq!(apex_uv.method, SurfacePointMethod::Analytic);
    assert_close(apex_uv.uv[1], cone.apex_v(), 2.0e-12);
    let reconstructed = cone.eval(normalize_surface_uv(&cone, apex_uv.uv));
    assert_close(reconstructed.dist(cone.apex()), 0.0, 2.0e-12);
}

#[test]
fn nurbs_uses_finite_domain_projection_for_uv_and_distance() {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let surface = NurbsSurface::new(
        1,
        1,
        knots.clone(),
        knots,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 2.0, 0.0),
        ],
        None,
    )
    .unwrap();

    let point = surface.eval([0.3, 0.7]);
    let mapped = invert_surface_point(&surface, point).unwrap();
    assert_eq!(mapped.method, SurfacePointMethod::Projected);
    assert_close(mapped.uv[0], 0.3, 2.0e-10);
    assert_close(mapped.uv[1], 0.7, 2.0e-10);
    assert_eq!(normalize_surface_uv(&surface, mapped.uv), mapped.uv);

    let distance = distance_to_surface(&surface, point + Vec3::new(0.0, 0.0, 0.5)).unwrap();
    assert_eq!(distance.method, SurfacePointMethod::Projected);
    assert_close(distance.distance, 0.5, 2.0e-10);
}
