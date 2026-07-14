//! Executed and structural contracts for the facade-only lifecycle client.

use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

fn output_path(label: &str) -> std::path::PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    std::env::temp_dir().join(format!(
        "kernel-lifecycle-{label}-{}-{nonce}.x_t",
        std::process::id()
    ))
}

fn run(output_path: &std::path::Path) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_kernel-lifecycle"))
        .arg(output_path)
        .output()
        .unwrap()
}

#[test]
fn facade_only_client_completes_the_supported_lifecycle() {
    let first_path = output_path("first");
    let second_path = output_path("second");
    let first = run(&first_path);
    let second = run(&second_path);
    for output in [&first, &second] {
        assert!(
            output.status.success(),
            "stderr: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    let summary = String::from_utf8(first.stdout).unwrap();
    assert!(summary.contains("kind=Solid"));
    assert!(summary.contains("faces=6"));
    assert!(summary.contains("edges=12"));
    assert!(summary.contains("vertices=8"));
    assert!(summary.contains("mesh_vertices=8"));
    assert!(summary.contains("mesh_triangles=12"));
    assert!(summary.contains("check=Valid"));
    assert!(summary.contains("surface=kernel.surface.plane.v1"));
    assert!(summary.contains("imported_bodies=1"));
    assert!(summary.contains("byte_stable=true"));
    assert!(summary.contains("original_live=true"));
    assert_eq!(summary.as_bytes(), second.stdout.as_slice());

    let first_xt = std::fs::read(&first_path).unwrap();
    let second_xt = std::fs::read(&second_path).unwrap();
    std::fs::remove_file(first_path).unwrap();
    std::fs::remove_file(second_path).unwrap();
    assert!(first_xt.starts_with(b"**ABCDEFGHIJKLMNOPQRSTUVWXYZ"));
    assert_eq!(first_xt, second_xt);
}

#[test]
fn manifest_has_only_the_supported_facade_as_a_direct_dependency() {
    let manifest = include_str!("../Cargo.toml");
    let dependency_lines = manifest
        .split("[dependencies]")
        .nth(1)
        .unwrap()
        .split('\n')
        .skip(1)
        .take_while(|line| !line.trim_start().starts_with('['))
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .collect::<Vec<_>>();

    assert_eq!(
        dependency_lines,
        ["kernel = { path = \"../../crates/kernel\" }"]
    );
}
