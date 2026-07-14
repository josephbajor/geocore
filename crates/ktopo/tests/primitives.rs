#![allow(
    deprecated,
    reason = "compatibility coverage retains the deprecated v1 tessellation wrapper"
)]

//! Cross-module integration: every primitive constructor must be
//! checker-clean and tessellate watertight with the right volume — the M2
//! exit criteria, exercised end-to-end (make → check → btess).

use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};
use ktopo::btess::{TessOptions, check_watertight, signed_volume, tessellate_body};
use ktopo::check::check_body;
use ktopo::entity::BodyId;
use ktopo::make;
use ktopo::store::Store;

const PI: f64 = core::f64::consts::PI;

fn tilted() -> Frame {
    Frame::new(
        Point3::new(0.3, -1.2, 2.1),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap()
}

/// Checker-clean, watertight at two tolerances, and mesh volume within
/// `rel_tol` of (and never exceeding, up to float slack) the exact volume.
fn assert_solid(store: &Store, body: BodyId, exact_volume: f64, rel_tol: f64) {
    let faults = check_body(store, body).unwrap();
    assert!(faults.is_empty(), "checker faults: {faults:?}");
    for chord_tol in [1e-2, 1e-3] {
        let mesh = tessellate_body(
            store,
            body,
            &TessOptions {
                chord_tol,
                max_edge_len: None,
            },
        )
        .unwrap();
        let problems = check_watertight(&mesh);
        assert!(problems.is_empty(), "not watertight: {problems:?}");
        let vol = signed_volume(&mesh);
        assert!(
            vol > exact_volume * (1.0 - rel_tol) && vol < exact_volume * (1.0 + 1e-9),
            "volume {vol} vs exact {exact_volume} (chord_tol {chord_tol})"
        );
    }
}

#[test]
fn block_is_clean_watertight_and_exact() {
    let mut store = Store::new();
    let body = make::block(&mut store, &tilted(), [2.0, 3.0, 4.0]).unwrap();
    assert_solid(&store, body, 24.0, 1e-12);
}

#[test]
fn cylinder_is_clean_watertight_and_accurate() {
    let mut store = Store::new();
    let body = make::cylinder(&mut store, &tilted(), 1.3, 2.0).unwrap();
    assert_solid(&store, body, PI * 1.3 * 1.3 * 2.0, 0.02);
}

#[test]
fn cone_frustum_is_clean_watertight_and_accurate() {
    let mut store = Store::new();
    let body = make::cone(&mut store, &tilted(), 1.5, 0.6, 2.0).unwrap();
    let exact = PI * 2.0 * (1.5 * 1.5 + 1.5 * 0.6 + 0.6 * 0.6) / 3.0;
    assert_solid(&store, body, exact, 0.02);
}

#[test]
fn sphere_is_clean_watertight_and_accurate() {
    let mut store = Store::new();
    let body = make::sphere(&mut store, &tilted(), 1.1).unwrap();
    let exact = 4.0 / 3.0 * PI * 1.1_f64.powi(3);
    assert_solid(&store, body, exact, 0.02);
}

#[test]
fn torus_is_clean_watertight_and_accurate() {
    let mut store = Store::new();
    let body = make::torus(&mut store, &tilted(), 2.0, 0.7).unwrap();
    let exact = 2.0 * PI * PI * 2.0 * 0.7 * 0.7;
    assert_solid(&store, body, exact, 0.02);
}

#[test]
fn shrinking_frustum_also_passes() {
    // top radius larger than base: exercises the flipped-surface-frame path.
    let mut store = Store::new();
    let body = make::cone(&mut store, &tilted(), 0.6, 1.5, 2.0).unwrap();
    let exact = PI * 2.0 * (1.5 * 1.5 + 1.5 * 0.6 + 0.6 * 0.6) / 3.0;
    assert_solid(&store, body, exact, 0.02);
}
