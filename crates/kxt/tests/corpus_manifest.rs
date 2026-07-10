//! Ratcheted stage expectations for every committed XT fixture.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;

const COMMITTED_CORPUS_FLOOR: usize = 6;
const DISCOVERY_FLOOR: usize = 8;

#[derive(Debug)]
struct Entry {
    file: String,
    schema: String,
    bytes: usize,
    nodes: usize,
    parse: String,
    reconstruct: String,
    checker: String,
    tessellate: String,
    checker_faults: usize,
}

fn fixture_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn manifest() -> Vec<Entry> {
    let text = std::fs::read_to_string(fixture_dir().join("manifest.tsv")).unwrap();
    let mut lines = text.lines();
    let header = lines.next().unwrap();
    assert_eq!(header.split('\t').count(), 15);
    lines
        .enumerate()
        .map(|(line_number, line)| {
            let columns: Vec<_> = line.split('\t').collect();
            assert_eq!(
                columns.len(),
                15,
                "manifest line {} has wrong column count",
                line_number + 2
            );
            for (column, value) in columns.iter().enumerate() {
                assert!(
                    !value.is_empty(),
                    "manifest line {}, column {column} is empty",
                    line_number + 2
                );
            }
            Entry {
                file: columns[0].to_owned(),
                schema: columns[6].to_owned(),
                bytes: columns[7].parse().unwrap(),
                nodes: columns[8].parse().unwrap(),
                parse: columns[9].to_owned(),
                reconstruct: columns[10].to_owned(),
                checker: columns[11].to_owned(),
                tessellate: columns[12].to_owned(),
                checker_faults: columns[13].parse().unwrap(),
            }
        })
        .collect()
}

#[test]
fn manifest_covers_every_transmit_fixture_and_matches_file_sizes() {
    let entries = manifest();
    assert!(
        entries.len() >= COMMITTED_CORPUS_FLOOR,
        "committed corpus shrank below its ratcheted floor"
    );
    let declared: BTreeSet<_> = entries.iter().map(|entry| entry.file.clone()).collect();
    assert_eq!(declared.len(), entries.len(), "duplicate manifest file");
    let present: BTreeSet<_> = std::fs::read_dir(fixture_dir())
        .unwrap()
        .filter_map(|entry| {
            let entry = entry.unwrap();
            let name = entry.file_name().into_string().ok()?;
            (name.ends_with(".x_t") || name.ends_with(".x_b")).then_some(name)
        })
        .collect();
    assert_eq!(declared, present);
    for entry in entries {
        assert_eq!(
            std::fs::metadata(fixture_dir().join(&entry.file))
                .unwrap()
                .len() as usize,
            entry.bytes,
            "{} byte count changed without a manifest update",
            entry.file
        );
    }
}

#[test]
fn unlicensed_discovery_catalog_is_metadata_only() {
    let text = std::fs::read_to_string(fixture_dir().join("discovery.tsv")).unwrap();
    let mut lines = text.lines();
    assert_eq!(lines.next().unwrap().split('\t').count(), 15);
    let rows: Vec<_> = lines.collect();
    assert!(
        rows.len() >= DISCOVERY_FLOOR,
        "discovery catalog shrank below its ratcheted floor"
    );
    for (line_number, row) in rows.iter().enumerate() {
        let columns: Vec<_> = row.split('\t').collect();
        assert_eq!(columns.len(), 15, "discovery line {}", line_number + 2);
        assert_eq!(columns[1], "external-discovery-only");
        assert_eq!(columns[4], "NOT-REDISTRIBUTABLE-no-license-found");
        assert!(columns[2].starts_with("https://github.com/"));
        assert!(
            !fixture_dir().join(columns[0]).exists(),
            "unlicensed discovery {} must not be copied into fixtures",
            columns[0]
        );
    }
}

#[test]
fn observed_corpus_stages_match_the_non_shrinking_manifest() {
    let entries = manifest();
    let paths: Vec<_> = entries
        .iter()
        .map(|entry| fixture_dir().join(&entry.file))
        .collect();
    let output = Command::new(env!("CARGO_BIN_EXE_xt_inspect"))
        .args(&paths)
        .output()
        .unwrap();
    assert!(
        !output.status.success(),
        "manifest deliberately includes an unsupported fixture"
    );
    let stdout = String::from_utf8(output.stdout).unwrap();
    let rows: Vec<_> = stdout.lines().collect();
    assert_eq!(rows.len(), entries.len());
    for (entry, row) in entries.iter().zip(rows) {
        assert!(
            row.contains(&format!("\"bytes\":{}", entry.bytes)),
            "{}",
            entry.file
        );
        assert!(
            row.contains(&format!("\"nodes\":{}", entry.nodes)),
            "{}",
            entry.file
        );
        assert!(
            row.contains(&format!("\"parse\":\"{}\"", entry.parse)),
            "{}: {row}",
            entry.file
        );
        assert!(
            row.contains(&format!("\"reconstruct\":\"{}\"", entry.reconstruct)),
            "{}: {row}",
            entry.file
        );
        assert!(
            row.contains(&format!("\"checker\":\"{}\"", entry.checker)),
            "{}: {row}",
            entry.file
        );
        assert!(
            row.contains(&format!("\"tessellate\":\"{}\"", entry.tessellate)),
            "{}: {row}",
            entry.file
        );
        assert!(
            row.contains(&format!("\"checker_faults\":{}", entry.checker_faults)),
            "{}: {row}",
            entry.file
        );
        if entry.parse == "pass" {
            assert!(
                row.contains(&format!("\"schema\":\"{}\"", entry.schema)),
                "{}: {row}",
                entry.file
            );
        }
    }
}
