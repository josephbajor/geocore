//! M3a exit criterion, end to end: real-world transmit files parse,
//! reconstruct checker-clean, and tessellate into watertight meshes.

use ktopo::btess::{TessOptions, check_watertight, signed_volume, tessellate_body};
use ktopo::check::check_body;
use ktopo::entity::BodyKind;
use ktopo::store::Store;

fn import(name: &str) -> (Store, kxt::Reconstruction) {
    let path = format!("{}/tests/fixtures/{name}", env!("CARGO_MANIFEST_DIR"));
    let bytes = std::fs::read(path).unwrap();
    let mut store = Store::new();
    let recon = kxt::import(&bytes, &mut store).unwrap();
    (store, recon)
}

/// Every reconstructed body is checker-clean and tessellates; solids must
/// be watertight with positive enclosed volume.
fn assert_imports_tessellate(name: &str) {
    let (store, recon) = import(name);
    assert!(!recon.bodies.is_empty());
    for &body in &recon.bodies {
        let faults = check_body(&store, body).unwrap();
        assert!(faults.is_empty(), "{name}: checker faults: {faults:?}");
        let mesh = tessellate_body(
            &store,
            body,
            &TessOptions {
                chord_tol: 1e-3,
                max_edge_len: None,
            },
        )
        .unwrap();
        assert!(!mesh.triangles.is_empty(), "{name}: empty tessellation");
        if store.get(body).unwrap().kind == BodyKind::Solid {
            let problems = check_watertight(&mesh);
            assert!(problems.is_empty(), "{name}: not watertight: {problems:?}");
            assert!(signed_volume(&mesh) > 0.0, "{name}: non-positive volume");
        }
    }
}

#[test]
fn hand_authored_block_tessellates_watertight() {
    assert_imports_tessellate("block.x_t");
}

#[test]
fn real_world_plate_tessellates_watertight() {
    assert_imports_tessellate("plate.x_t");
}

#[test]
fn real_world_cut_sphere_tessellates_watertight() {
    assert_imports_tessellate("sphere.x_t");
}

#[test]
fn real_world_sheet_disk_tessellates() {
    assert_imports_tessellate("disk_nat.x_t");
}
