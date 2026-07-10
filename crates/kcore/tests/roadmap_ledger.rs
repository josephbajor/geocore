//! Structural guard for the machine-readable kernel capability ledger.

use std::collections::BTreeSet;
use std::path::Path;

const LEDGER: &str = include_str!("../../../docs/kernel-support.tsv");

#[test]
fn capability_ledger_is_well_formed_and_evidence_paths_exist() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut lines = LEDGER.lines();
    assert_eq!(
        lines.next(),
        Some("id\tarea\tcapability\tstatus\tfirst_exit_gate\tevidence\tnext_required_step")
    );

    let statuses = [
        "implemented_slice",
        "in_progress",
        "provisional_gated",
        "not_started",
        "conformant",
    ];
    let gates = ["M0", "M1", "M2", "M2.5", "M3", "M4", "M5", "M6", "M7", "M8"];
    let mut ids = BTreeSet::new();

    for (index, line) in lines.enumerate() {
        let line_number = index + 2;
        let fields: Vec<_> = line.split('\t').collect();
        assert_eq!(
            fields.len(),
            7,
            "ledger line {line_number} must contain exactly seven fields"
        );
        assert!(
            fields.iter().all(|field| !field.trim().is_empty()),
            "ledger line {line_number} contains an empty field"
        );
        assert!(
            ids.insert(fields[0]),
            "duplicate capability id {}",
            fields[0]
        );
        assert!(
            statuses.contains(&fields[3]),
            "unknown status {} on ledger line {line_number}",
            fields[3]
        );
        assert!(
            gates.contains(&fields[4]),
            "unknown exit gate {} on ledger line {line_number}",
            fields[4]
        );
        if fields[3] != "not_started" {
            assert_ne!(
                fields[5], "-",
                "started capability {} must name evidence",
                fields[0]
            );
        }
        if fields[5] != "-" {
            for evidence in fields[5].split(';') {
                assert!(
                    root.join(evidence).exists(),
                    "evidence path {evidence} for {} does not exist",
                    fields[0]
                );
            }
        }
    }

    assert!(
        ids.len() >= 20,
        "ledger unexpectedly lost capability coverage"
    );
}
