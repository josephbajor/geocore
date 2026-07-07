//! Determinism harness for the geometry layer.
//!
//! Same contract as kcore's: a fixed batch of geometry results — evaluators
//! for every class, projections, and a full tessellation — folded bit-by-bit
//! into an FNV-1a hash pinned to a golden constant, run by CI on all three
//! platforms in debug and release. Changing the golden value is a reviewed,
//! intentional event.

use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, interpolate};
use kgeom::param::ParamRange;
use kgeom::project::{project_to_curve, project_to_surface};
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};
use kgeom::tess::{TessOptions, TrimmedSurface, tessellate};
use kgeom::vec::{Point3, Vec3};

struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Fnv(0xcbf2_9ce4_8422_2325)
    }
    fn write_u64(&mut self, v: u64) {
        for byte in v.to_le_bytes() {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    fn write_f64(&mut self, v: f64) {
        self.write_u64(v.to_bits());
    }
    fn write_vec3(&mut self, v: Vec3) {
        self.write_f64(v.x);
        self.write_f64(v.y);
        self.write_f64(v.z);
    }
}

fn tilted() -> Frame {
    Frame::new(
        Point3::new(0.3, -1.2, 2.1),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap()
}

#[test]
fn golden_hash_of_geometry_results() {
    let mut hash = Fnv::new();
    let f = tilted();

    // Every curve class, all derivative orders, across a parameter sweep.
    let line = Line::new(f.origin(), f.z()).unwrap();
    let circle = Circle::new(f, 1.7).unwrap();
    let ellipse = Ellipse::new(f, 2.4, 1.1).unwrap();
    let ncurve = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.5),
            Point3::new(2.0, -1.0, 1.5),
            Point3::new(3.5, 0.5, -0.5),
            Point3::new(4.0, 1.0, 2.0),
        ],
        Some(vec![1.0, 0.8, 1.5, 0.9, 1.2]),
    )
    .unwrap();
    let interp = interpolate(
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.2),
            Point3::new(2.0, 0.5, 1.0),
            Point3::new(3.0, -0.5, 0.6),
            Point3::new(4.0, 0.0, -0.4),
            Point3::new(5.0, 1.5, 0.0),
        ],
        3,
    )
    .unwrap();
    let curves: [&dyn Curve; 5] = [&line, &circle, &ellipse, &ncurve, &interp];
    for curve in curves {
        let range = curve.param_range().clamped(ParamRange::new(-5.0, 5.0));
        for i in 0..=32 {
            let t = range.lerp(f64::from(i) / 32.0);
            let d = curve.eval_derivs(t, 3);
            for k in 0..4 {
                hash.write_vec3(d.d[k]);
            }
        }
        let bb = curve.bounding_box(range);
        hash.write_vec3(bb.min);
        hash.write_vec3(bb.max);
    }

    // Every surface class across a (u, v) sweep.
    let plane = Plane::new(f);
    let cylinder = Cylinder::new(f, 1.3).unwrap();
    let cone = Cone::new(f, 1.0, 0.35).unwrap();
    let sphere = Sphere::new(f, 2.2).unwrap();
    let torus = Torus::new(f, 3.0, 0.8).unwrap();
    let surfaces: [&dyn Surface; 5] = [&plane, &cylinder, &cone, &sphere, &torus];
    for surface in surfaces {
        let [ur, vr] = surface.param_range();
        let ur = ur.clamped(ParamRange::new(-4.0, 4.0));
        let vr = vr.clamped(ParamRange::new(-4.0, 4.0));
        for i in 0..=16 {
            for j in 0..=16 {
                let uv = [ur.lerp(f64::from(i) / 16.0), vr.lerp(f64::from(j) / 16.0)];
                let d = surface.eval_derivs(uv, 2);
                hash.write_vec3(d.p);
                hash.write_vec3(d.du);
                hash.write_vec3(d.dv);
                hash.write_vec3(d.duu);
                hash.write_vec3(d.duv);
                hash.write_vec3(d.dvv);
                hash.write_u64(surface.normal(uv).map_or(0, |n| {
                    let mut h = Fnv::new();
                    h.write_vec3(n);
                    h.0
                }));
            }
        }
    }

    // Projection results.
    for i in 0..64 {
        let s = f64::from(i) / 63.0;
        let p = Point3::new(4.0 * s - 2.0, 3.0 * (1.0 - s) - 1.0, 2.0 * s);
        let cp = project_to_curve(&circle, p, circle.param_range()).unwrap();
        hash.write_f64(cp.t);
        hash.write_vec3(cp.point);
        hash.write_f64(cp.dist);
        let sp = project_to_surface(
            &sphere,
            p,
            [sphere.param_range()[0], sphere.param_range()[1]],
        )
        .unwrap();
        hash.write_f64(sp.uv[0]);
        hash.write_f64(sp.uv[1]);
        hash.write_vec3(sp.point);
        hash.write_f64(sp.dist);
    }

    // A full tessellation of a cylinder patch.
    let face = TrimmedSurface::rectangle(
        &cylinder,
        [
            ParamRange::new(0.0, core::f64::consts::PI),
            ParamRange::new(0.0, 2.0),
        ],
    )
    .unwrap();
    let mesh = tessellate(
        &face,
        &TessOptions {
            chord_tol: 1e-3,
            max_edge_len: None,
        },
    )
    .unwrap();
    hash.write_u64(mesh.positions.len() as u64);
    hash.write_u64(mesh.triangles.len() as u64);
    for p in &mesh.positions {
        hash.write_vec3(*p);
    }
    for t in &mesh.triangles {
        for &i in t {
            hash.write_u64(u64::from(i));
        }
    }

    // Golden value. Changing it is a reviewed, intentional event.
    assert_eq!(hash.0, 0x7688_519F_33AC_EECA, "geometry results drifted");
}
