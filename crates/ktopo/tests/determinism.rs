//! Determinism harness for the topology layer.
//!
//! Same contract as kcore's and kgeom's: a fixed batch of results — here,
//! full watertight tessellations of all five primitive bodies on a tilted
//! frame — folded bit-by-bit into an FNV-1a hash pinned to a golden
//! constant, run by CI on all three platforms in debug and release.
//! Changing the golden value is a reviewed, intentional event.

use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};
use ktopo::btess::{TessOptions, tessellate_body};
use ktopo::check::check_body;
use ktopo::make;
use ktopo::store::Store;

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
}

#[test]
fn golden_hash_of_topology_results() {
    let f = Frame::new(
        Point3::new(0.3, -1.2, 2.1),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();

    let mut store = Store::new();
    let bodies = [
        make::block(&mut store, &f, [2.0, 3.0, 4.0]).unwrap(),
        make::cylinder(&mut store, &f, 1.3, 2.0).unwrap(),
        make::cone(&mut store, &f, 1.5, 0.6, 2.0).unwrap(),
        make::sphere(&mut store, &f, 1.1).unwrap(),
        make::torus(&mut store, &f, 2.0, 0.7).unwrap(),
    ];

    let mut hash = Fnv::new();
    for body in bodies {
        // Fold in the checker verdict (must stay clean) …
        let faults = check_body(&store, body).unwrap();
        assert!(faults.is_empty(), "primitive not checker-clean: {faults:?}");
        hash.write_u64(faults.len() as u64);

        // … and every bit of the tessellation.
        for chord_tol in [1e-2, 1e-3] {
            let mesh = tessellate_body(
                &store,
                body,
                &TessOptions {
                    chord_tol,
                    max_edge_len: None,
                },
            )
            .unwrap();
            hash.write_u64(mesh.positions.len() as u64);
            hash.write_u64(mesh.triangles.len() as u64);
            for p in &mesh.positions {
                hash.write_f64(p.x);
                hash.write_f64(p.y);
                hash.write_f64(p.z);
            }
            for t in &mesh.triangles {
                for &i in t {
                    hash.write_u64(u64::from(i));
                }
            }
            for (_, range) in &mesh.face_ranges {
                hash.write_u64(range.start as u64);
                hash.write_u64(range.end as u64);
            }
            for (_, poly) in &mesh.edge_polylines {
                hash.write_u64(poly.len() as u64);
                for &i in poly {
                    hash.write_u64(u64::from(i));
                }
            }
        }
    }

    // Golden value. Changing it is a reviewed, intentional event.
    // Updated when boundary UV construction switched from approximate 3D
    // inversion to the fin's exact retained edge parameters and pcurves.
    assert_eq!(hash.0, 0x1329_1F63_B68C_9B19, "topology results drifted");
}
