//! End-to-end coverage for the JSON-lines XT corpus inspector.

use std::process::Command;

fn fixture(name: &str) -> String {
    format!("{}/tests/fixtures/{name}", env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn inspector_reports_all_success_stages_for_supported_fixture() {
    let output = Command::new(env!("CARGO_BIN_EXE_xt_inspect"))
        .arg(fixture("block.x_t"))
        .output()
        .unwrap();
    assert!(output.status.success());
    let row = String::from_utf8(output.stdout).unwrap();
    assert!(row.starts_with('{') && row.ends_with("}\n"));
    assert!(row.contains("\"parse\":\"pass\""));
    assert!(row.contains("\"reconstruct\":\"pass\""));
    assert!(row.contains("\"checker\":\"pass\""));
    assert!(row.contains("\"tessellate\":\"pass\""));
    assert!(row.contains("\"null_curve_tolerant_edges\":0"));
    assert!(row.contains("\"trimmed_sp_fin_curves\":0"));
}

#[test]
fn inspector_keeps_scanning_and_fails_process_for_unsupported_fixture() {
    let output = Command::new(env!("CARGO_BIN_EXE_xt_inspect"))
        .args([fixture("block.x_t"), fixture("longbar.x_t")])
        .output()
        .unwrap();
    assert!(!output.status.success());
    let rows = String::from_utf8(output.stdout).unwrap();
    assert_eq!(rows.lines().count(), 2);
    assert!(rows.lines().next().unwrap().contains("\"parse\":\"pass\""));
    assert!(
        rows.lines()
            .nth(1)
            .unwrap()
            .contains("\"parse\":\"unsupported\"")
    );
}
