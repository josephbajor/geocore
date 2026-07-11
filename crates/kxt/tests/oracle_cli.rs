//! CLI contract for the M3b external-oracle harness (`xt_oracle`).

use std::path::{Path, PathBuf};
use std::process::Command;

fn export_into(dir: &Path) {
    let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
        .arg("export")
        .arg(dir)
        .output()
        .expect("running xt_oracle export");
    assert!(
        output.status.success(),
        "export failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
}

fn bundle_dir(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(name);
    if dir.exists() {
        std::fs::remove_dir_all(&dir).expect("clearing previous bundle dir");
    }
    dir
}

/// The declared bundle contents. Growing this list is a writer-capability
/// event; shrinking it is a regression.
const EXPECTED_FILES: &[&str] = &[
    "solid_block.x_t",
    "solid_cylinder.x_t",
    "solid_cone.x_t",
    "solid_sphere.x_t",
    "solid_torus.x_t",
    "solid_block_nurbs_edge.x_t",
    "solid_block_nurbs_face.x_t",
    "solid_block_tolerant_edge.x_t",
    "sheet_cylinder_seam.x_t",
    "sheet_plane_polygon.x_t",
    "wire_polyline_open.x_t",
    "wire_polyline_closed.x_t",
    "acorn_point.x_t",
];

#[test]
fn export_is_complete_deterministic_and_self_importable() {
    let first = bundle_dir("oracle_bundle_a");
    let second = bundle_dir("oracle_bundle_b");
    export_into(&first);
    export_into(&second);

    let mut produced: Vec<String> = std::fs::read_dir(&first)
        .expect("reading bundle dir")
        .map(|entry| entry.expect("dir entry").file_name().into_string().unwrap())
        .collect();
    produced.sort();
    let mut expected: Vec<String> = EXPECTED_FILES
        .iter()
        .map(|name| name.to_string())
        .chain(std::iter::once("manifest.tsv".to_string()))
        .collect();
    expected.sort();
    assert_eq!(produced, expected, "bundle file set changed");

    for name in &expected {
        let a = std::fs::read(first.join(name)).expect("first bundle file");
        let b = std::fs::read(second.join(name)).expect("second bundle file");
        assert_eq!(a, b, "{name} is not byte-deterministic across exports");
    }

    // The manifest declares one row per fixture in bundle order.
    let manifest = std::fs::read_to_string(first.join("manifest.tsv")).expect("manifest");
    let rows: Vec<&str> = manifest.lines().collect();
    assert_eq!(rows.len(), EXPECTED_FILES.len() + 1, "manifest row count");
    assert!(rows[0].starts_with("file\tbody_kind\tprobe\t"));

    // Every transport file re-imports checker-clean here. The generator
    // already enforces this before writing; the test pins it as a contract.
    for name in EXPECTED_FILES {
        let bytes = std::fs::read(first.join(name)).expect("bundle file");
        let mut store = ktopo::store::Store::new();
        let recon = kxt::import(&bytes, &mut store)
            .unwrap_or_else(|error| panic!("{name}: import failed: {error:?}"));
        assert_eq!(recon.bodies.len(), 1, "{name}: expected exactly one body");
        let faults = ktopo::check::check_body(&store, recon.bodies[0]).expect("check");
        assert!(faults.is_empty(), "{name}: checker faults: {faults:?}");
    }
}

#[test]
fn compare_accepts_identity_and_rejects_a_different_body() {
    let dir = bundle_dir("oracle_bundle_compare");
    export_into(&dir);

    // Identity comparison passes for an exact solid and for the tolerant
    // SP-curve fixture (the hardest writer path).
    for name in ["solid_block.x_t", "solid_block_tolerant_edge.x_t"] {
        let file = dir.join(name);
        let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
            .arg("compare")
            .arg(&file)
            .arg(&file)
            .output()
            .expect("running xt_oracle compare");
        assert!(
            output.status.success(),
            "{name}: identity compare failed:\n{}",
            String::from_utf8_lossy(&output.stdout),
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(stdout.contains("COMPARE OK"), "{name}: {stdout}");
    }

    // A genuinely different body must be flagged with exit code 1.
    let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
        .arg("compare")
        .arg(dir.join("solid_block.x_t"))
        .arg(dir.join("solid_sphere.x_t"))
        .output()
        .expect("running xt_oracle compare");
    assert_eq!(output.status.code(), Some(1), "mismatch must exit 1");
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("COMPARE FAILED"), "{stdout}");
    assert!(stdout.contains("FAIL  surface_classes"), "{stdout}");

    // An unreadable input is an operational error, not a mismatch.
    let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
        .arg("compare")
        .arg(dir.join("solid_block.x_t"))
        .arg(dir.join("does_not_exist.x_t"))
        .output()
        .expect("running xt_oracle compare");
    assert_eq!(output.status.code(), Some(2), "IO failure must exit 2");
}

#[test]
fn compare_reports_offset_as_a_stable_surface_class() {
    let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/offset_plane.x_t");
    let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
        .arg("compare")
        .arg(&fixture)
        .arg(&fixture)
        .output()
        .expect("running xt_oracle offset identity comparison");
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("PASS  surface_classes: offset:1"),
        "{stdout}"
    );
}
