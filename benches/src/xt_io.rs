//! Deterministic Q5 X_T parse/read/write/round-trip fixtures and evidence.

use ktopo::benchmark::{StoreSnapshot, store_snapshot};
use ktopo::check::check_body;
use ktopo::store::Store;
use kxt::parse::{Value, XtFile};
use kxt::{export_text, import, read_xt};
use std::collections::BTreeMap;

const BLOCK_TEXT: &[u8] = include_bytes!("../../crates/kxt/tests/fixtures/block.x_t");
const OFFSET_PLANE: &[u8] = include_bytes!("../../crates/kxt/tests/fixtures/offset_plane.x_t");

/// Deterministic fixture seed (fixtures themselves contain no randomness).
pub const FIXTURE_SEED: u64 = 0x5154_5854_494f_0005;

/// Redistributable checked-in source fixture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XtFixtureKind {
    /// Hand-authored schema-13006 solid block.
    BlockText,
    /// Hand-authored canonical planar offset sheet with pcurves.
    OffsetPlane,
}

impl XtFixtureKind {
    /// Versioned registry identity.
    pub const fn version(self) -> &'static str {
        match self {
            Self::BlockText => "xt-io.block-text.v1",
            Self::OffsetPlane => "xt-io.offset-plane.v2",
        }
    }
}

/// Truthfully isolated public X_T operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XtPhase {
    /// Public parse-to-records API, including tokenization and record validation.
    ParseRecords,
    /// Complete parse plus atomic reconstruction into a fresh store.
    FullRead,
    /// Complete public writer, including its mandatory body check, planning,
    /// and text emission.
    WriteText,
    /// Complete read, write, and reread pipeline.
    RoundTrip,
}

impl XtPhase {
    /// Stable registry spelling.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::ParseRecords => "parse-records",
            Self::FullRead => "full-read",
            Self::WriteText => "write-text",
            Self::RoundTrip => "round-trip",
        }
    }
}

/// Stable Q5 case definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XtIoCase {
    /// Five-segment benchmark path.
    pub path: &'static str,
    /// Source fixture.
    pub fixture: XtFixtureKind,
    /// Timed phase.
    pub phase: XtPhase,
    /// Reviewed source byte count.
    pub expected_input_bytes: usize,
    /// Reviewed source record count.
    pub expected_input_records: usize,
    /// Reviewed emitted byte count, or zero for non-writer phases.
    pub expected_output_bytes: usize,
    /// Reviewed complete semantic evidence digest.
    pub expected_output_digest: u64,
}

/// Eight cases: two redistributable fixtures across four public phase boundaries.
pub const CASES: [XtIoCase; 8] = [
    case(
        "interchange/xt-parse/block-text-v1/3647/default-v1",
        XtFixtureKind::BlockText,
        XtPhase::ParseRecords,
        3_647,
        87,
        0,
        0x6e33_08a4_4d89_85eb,
    ),
    case(
        "interchange/xt-read/block-text-v1/3647/default-v1",
        XtFixtureKind::BlockText,
        XtPhase::FullRead,
        3_647,
        87,
        0,
        0x02ba_1b31_f2b5_03b6,
    ),
    case(
        "interchange/xt-write/block-text-v1/3647/default-v1",
        XtFixtureKind::BlockText,
        XtPhase::WriteText,
        3_647,
        87,
        3_960,
        0x93fd_80f8_8a32_c52a,
    ),
    case(
        "interchange/xt-roundtrip/block-text-v1/3647/default-v1",
        XtFixtureKind::BlockText,
        XtPhase::RoundTrip,
        3_647,
        87,
        3_960,
        0x6981_e9fa_592d_6834,
    ),
    case(
        "interchange/xt-parse/offset-plane-v2/2974/default-v1",
        XtFixtureKind::OffsetPlane,
        XtPhase::ParseRecords,
        2_974,
        72,
        0,
        0x797e_d4e3_4f77_e356,
    ),
    case(
        "interchange/xt-read/offset-plane-v2/2974/default-v1",
        XtFixtureKind::OffsetPlane,
        XtPhase::FullRead,
        2_974,
        72,
        0,
        0xf5dd_feb8_88e4_e0ed,
    ),
    case(
        "interchange/xt-write/offset-plane-v2/2974/default-v1",
        XtFixtureKind::OffsetPlane,
        XtPhase::WriteText,
        2_974,
        72,
        2_974,
        0x0c16_f8f8_c1f5_b8f5,
    ),
    case(
        "interchange/xt-roundtrip/offset-plane-v2/2974/default-v1",
        XtFixtureKind::OffsetPlane,
        XtPhase::RoundTrip,
        2_974,
        72,
        2_974,
        0x3d73_d288_9330_044b,
    ),
];

const fn case(
    path: &'static str,
    fixture: XtFixtureKind,
    phase: XtPhase,
    expected_input_bytes: usize,
    expected_input_records: usize,
    expected_output_bytes: usize,
    expected_output_digest: u64,
) -> XtIoCase {
    XtIoCase {
        path,
        fixture,
        phase,
        expected_input_bytes,
        expected_input_records,
        expected_output_bytes,
        expected_output_digest,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ParsedEvidence {
    records: usize,
    classes: usize,
    class_digest: u64,
    parse_digest: u64,
}

#[derive(Clone, Copy)]
struct PhaseArtifacts<'a> {
    input: ParsedEvidence,
    store: Option<&'a Store>,
    text: Option<&'a str>,
    journal_mutations: usize,
    skipped_records: usize,
    roundtrip_equal: bool,
}

/// Immutable prepared input. File I/O, source import, checks, and canonical write are setup.
pub struct XtIoFixture {
    bytes: &'static [u8],
    input: ParsedEvidence,
    input_byte_digest: u64,
    source_store: Store,
    source_body: ktopo::entity::BodyId,
    canonical_text: String,
}

impl XtIoFixture {
    /// Execute one case while timing only its declared public API phase.
    pub fn measure_once(&self, case: XtIoCase) -> (core::time::Duration, XtIoEvidence) {
        match case.phase {
            XtPhase::ParseRecords => {
                let started = std::time::Instant::now();
                let file = read_xt(self.bytes).expect("reviewed Q5 fixture must parse");
                let elapsed = started.elapsed();
                let evidence = self.evidence(
                    case,
                    PhaseArtifacts {
                        input: parsed_evidence(&file),
                        store: None,
                        text: None,
                        journal_mutations: 0,
                        skipped_records: 0,
                        roundtrip_equal: false,
                    },
                );
                (elapsed, evidence)
            }
            XtPhase::FullRead => {
                let mut store = Store::new();
                let started = std::time::Instant::now();
                let reconstruction =
                    import(self.bytes, &mut store).expect("reviewed Q5 fixture must import");
                let elapsed = started.elapsed();
                let mutations = reconstruction.journal.mutations().len();
                let skipped = reconstruction.skipped.iter().map(|(_, count)| count).sum();
                let evidence = self.evidence(
                    case,
                    PhaseArtifacts {
                        input: self.input,
                        store: Some(&store),
                        text: None,
                        journal_mutations: mutations,
                        skipped_records: skipped,
                        roundtrip_equal: false,
                    },
                );
                (elapsed, evidence)
            }
            XtPhase::WriteText => {
                let started = std::time::Instant::now();
                let text = export_text(&self.source_store, self.source_body)
                    .expect("reviewed Q5 fixture must export");
                let elapsed = started.elapsed();
                let evidence = self.evidence(
                    case,
                    PhaseArtifacts {
                        input: self.input,
                        store: Some(&self.source_store),
                        text: Some(&text),
                        journal_mutations: 0,
                        skipped_records: 0,
                        roundtrip_equal: false,
                    },
                );
                (elapsed, evidence)
            }
            XtPhase::RoundTrip => {
                let mut source = Store::new();
                let mut reread = Store::new();
                let started = std::time::Instant::now();
                let first =
                    import(self.bytes, &mut source).expect("reviewed Q5 fixture must import");
                let text = export_text(&source, first.bodies[0])
                    .expect("reviewed Q5 imported fixture must export");
                let second = import(text.as_bytes(), &mut reread)
                    .expect("reviewed Q5 writer output must reread");
                let elapsed = started.elapsed();
                let mutations = first.journal.mutations().len() + second.journal.mutations().len();
                let skipped = first.skipped.iter().map(|(_, count)| count).sum::<usize>()
                    + second.skipped.iter().map(|(_, count)| count).sum::<usize>();
                let equivalent = store_snapshot(&source) == store_snapshot(&reread);
                let evidence = self.evidence(
                    case,
                    PhaseArtifacts {
                        input: self.input,
                        store: Some(&reread),
                        text: Some(&text),
                        journal_mutations: mutations,
                        skipped_records: skipped,
                        roundtrip_equal: equivalent,
                    },
                );
                (elapsed, evidence)
            }
        }
    }

    fn evidence(&self, case: XtIoCase, artifacts: PhaseArtifacts<'_>) -> XtIoEvidence {
        let output = artifacts.text.map(|text| {
            let parsed = read_xt(text.as_bytes()).expect("writer output must parse");
            (
                text.len(),
                byte_digest(text.as_bytes()),
                parsed_evidence(&parsed),
            )
        });
        let snapshot = artifacts.store.map(store_snapshot);
        let checker_ran = artifacts.store.is_some();
        let checker_clean = artifacts.store.is_some_and(|store| {
            store
                .iter::<ktopo::entity::Body>()
                .all(|(body, _)| check_body(store, body).is_ok_and(|faults| faults.is_empty()))
        });
        let snapshot = snapshot.unwrap_or(StoreSnapshot {
            bodies: 0,
            regions: 0,
            shells: 0,
            faces: 0,
            loops: 0,
            fins: 0,
            edges: 0,
            vertices: 0,
            points: 0,
            curves: 0,
            surfaces: 0,
            pcurves: 0,
            digest: 0,
        });
        let (output_bytes, output_byte_digest, output) = output.map_or(
            (
                0,
                0,
                ParsedEvidence {
                    records: 0,
                    classes: 0,
                    class_digest: 0,
                    parse_digest: 0,
                },
            ),
            |(bytes, digest, parsed)| (bytes, digest, parsed),
        );
        let deterministic_write = artifacts
            .text
            .is_some_and(|text| text == self.canonical_text);
        let mut evidence = XtIoEvidence {
            input_bytes: self.bytes.len(),
            input_records: artifacts.input.records,
            input_classes: artifacts.input.classes,
            input_class_digest: artifacts.input.class_digest,
            input_parse_digest: artifacts.input.parse_digest,
            input_byte_digest: self.input_byte_digest,
            output_bytes,
            output_records: output.records,
            output_classes: output.classes,
            output_class_digest: output.class_digest,
            output_parse_digest: output.parse_digest,
            output_byte_digest,
            phase_imports: usize::from(matches!(case.phase, XtPhase::FullRead))
                + 2 * usize::from(matches!(case.phase, XtPhase::RoundTrip)),
            phase_exports: usize::from(matches!(
                case.phase,
                XtPhase::WriteText | XtPhase::RoundTrip
            )),
            journal_mutations: artifacts.journal_mutations,
            skipped_records: artifacts.skipped_records,
            unsupported_capabilities: 0,
            checker_ran,
            checker_clean,
            deterministic_write,
            roundtrip_equal: artifacts.roundtrip_equal,
            bodies: snapshot.bodies,
            regions: snapshot.regions,
            shells: snapshot.shells,
            faces: snapshot.faces,
            loops: snapshot.loops,
            fins: snapshot.fins,
            edges: snapshot.edges,
            vertices: snapshot.vertices,
            points: snapshot.points,
            curves: snapshot.curves,
            surfaces: snapshot.surfaces,
            pcurves: snapshot.pcurves,
            semantic_digest: snapshot.digest,
            output_digest: 0,
        };
        evidence.output_digest = evidence.digest(case.phase);
        evidence
    }
}

/// Stable Q5 byte, record, model, and semantic evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XtIoEvidence {
    /// Source byte count.
    pub input_bytes: usize,
    /// Source node-record count.
    pub input_records: usize,
    /// Distinct source node-class count.
    pub input_classes: usize,
    /// Stable source node-class histogram digest.
    pub input_class_digest: u64,
    /// Stable complete parsed-record digest.
    pub input_parse_digest: u64,
    /// Stable source byte digest.
    pub input_byte_digest: u64,
    /// Emitted byte count, if the phase writes.
    pub output_bytes: usize,
    /// Emitted node-record count.
    pub output_records: usize,
    /// Distinct emitted node-class count.
    pub output_classes: usize,
    /// Stable emitted node-class histogram digest.
    pub output_class_digest: u64,
    /// Stable complete emitted parsed-record digest.
    pub output_parse_digest: u64,
    /// Stable emitted byte digest.
    pub output_byte_digest: u64,
    /// Imports performed by the timed phase.
    pub phase_imports: usize,
    /// Exports performed by the timed phase.
    pub phase_exports: usize,
    /// Atomic reconstruction journal mutations produced by the phase.
    pub journal_mutations: usize,
    /// Parsed records intentionally skipped during reconstruction.
    pub skipped_records: usize,
    /// Unsupported capability outcomes; successful Q5 baselines require zero.
    pub unsupported_capabilities: usize,
    /// Whether checker verification was applicable and executed.
    pub checker_ran: bool,
    /// Whether every reconstructed/source body was checker-clean.
    pub checker_clean: bool,
    /// Whether emitted text exactly matches the prepared canonical write.
    pub deterministic_write: bool,
    /// Whether read and reread stores have identical semantic snapshots.
    pub roundtrip_equal: bool,
    /// Live body count.
    pub bodies: usize,
    /// Live region count.
    pub regions: usize,
    /// Live shell count.
    pub shells: usize,
    /// Live face count.
    pub faces: usize,
    /// Live loop count.
    pub loops: usize,
    /// Live fin count.
    pub fins: usize,
    /// Live edge count.
    pub edges: usize,
    /// Live vertex count.
    pub vertices: usize,
    /// Live point count.
    pub points: usize,
    /// Live curve count.
    pub curves: usize,
    /// Live surface count.
    pub surfaces: usize,
    /// Live pcurve count.
    pub pcurves: usize,
    /// Stable semantic store digest.
    pub semantic_digest: u64,
    /// Stable digest of all phase evidence.
    pub output_digest: u64,
}

impl XtIoEvidence {
    fn digest(self, phase: XtPhase) -> u64 {
        let mut digest = StableHasher::new();
        digest.tag(0xa1);
        digest.tag(match phase {
            XtPhase::ParseRecords => 0,
            XtPhase::FullRead => 1,
            XtPhase::WriteText => 2,
            XtPhase::RoundTrip => 3,
        });
        for value in [
            self.input_bytes,
            self.input_records,
            self.input_classes,
            self.output_bytes,
            self.output_records,
            self.output_classes,
            self.phase_imports,
            self.phase_exports,
            self.journal_mutations,
            self.skipped_records,
            self.unsupported_capabilities,
            self.bodies,
            self.regions,
            self.shells,
            self.faces,
            self.loops,
            self.fins,
            self.edges,
            self.vertices,
            self.points,
            self.curves,
            self.surfaces,
            self.pcurves,
        ] {
            digest.count(value);
        }
        for value in [
            self.input_class_digest,
            self.input_parse_digest,
            self.input_byte_digest,
            self.output_class_digest,
            self.output_parse_digest,
            self.output_byte_digest,
            self.semantic_digest,
        ] {
            digest.u64(value);
        }
        for value in [
            self.checker_ran,
            self.checker_clean,
            self.deterministic_write,
            self.roundtrip_equal,
        ] {
            digest.boolean(value);
        }
        digest.finish()
    }
}

/// Prepare and fully verify one immutable source fixture.
pub fn fixture(case: XtIoCase) -> XtIoFixture {
    let bytes = match case.fixture {
        XtFixtureKind::BlockText => BLOCK_TEXT,
        XtFixtureKind::OffsetPlane => OFFSET_PLANE,
    };
    let file = read_xt(bytes).expect("reviewed Q5 source fixture must parse");
    let input = parsed_evidence(&file);
    let input_byte_digest = byte_digest(bytes);
    let mut source_store = Store::new();
    let reconstruction = import(bytes, &mut source_store).expect("reviewed Q5 source must import");
    assert_eq!(reconstruction.bodies.len(), 1);
    let source_body = reconstruction.bodies[0];
    assert!(check_body(&source_store, source_body).is_ok_and(|faults| faults.is_empty()));
    let canonical_text =
        export_text(&source_store, source_body).expect("reviewed source must write");
    assert_eq!(
        canonical_text,
        export_text(&source_store, source_body).expect("repeated write must succeed")
    );
    XtIoFixture {
        bytes,
        input,
        input_byte_digest,
        source_store,
        source_body,
        canonical_text,
    }
}

/// Verify exact reviewed phase evidence.
pub fn verify(case: XtIoCase, evidence: XtIoEvidence) {
    assert_eq!(evidence.input_bytes, case.expected_input_bytes);
    assert_eq!(evidence.input_records, case.expected_input_records);
    assert_eq!(evidence.output_bytes, case.expected_output_bytes);
    assert_eq!(evidence.unsupported_capabilities, 0);
    match case.phase {
        XtPhase::ParseRecords => {
            assert!(!evidence.checker_ran);
            assert!(!evidence.deterministic_write);
            assert!(!evidence.roundtrip_equal);
        }
        XtPhase::FullRead => {
            assert!(evidence.checker_ran && evidence.checker_clean);
            assert!(!evidence.deterministic_write);
            assert!(!evidence.roundtrip_equal);
        }
        XtPhase::WriteText => {
            assert!(evidence.checker_ran && evidence.checker_clean);
            assert!(evidence.deterministic_write);
            assert!(!evidence.roundtrip_equal);
        }
        XtPhase::RoundTrip => {
            assert!(evidence.checker_ran && evidence.checker_clean);
            assert!(evidence.deterministic_write);
            assert!(evidence.roundtrip_equal);
        }
    }
    assert_ne!(case.expected_output_digest, 0);
    assert_eq!(evidence.output_digest, case.expected_output_digest);
}

fn parsed_evidence(file: &XtFile) -> ParsedEvidence {
    let classes = class_histogram(file);
    let mut class_digest = StableHasher::new();
    class_digest.tag(0xa0);
    class_digest.count(classes.len());
    for (code, count) in classes {
        class_digest.u64(u64::from(code));
        class_digest.count(count);
    }
    ParsedEvidence {
        records: file.nodes.len(),
        classes: file
            .nodes
            .values()
            .map(|node| node.code)
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        class_digest: class_digest.finish(),
        parse_digest: parsed_digest(file),
    }
}

fn class_histogram(file: &XtFile) -> BTreeMap<u16, usize> {
    let mut classes = BTreeMap::new();
    for node in file.nodes.values() {
        *classes.entry(node.code).or_default() += 1;
    }
    classes
}

fn parsed_digest(file: &XtFile) -> u64 {
    let mut digest = StableHasher::new();
    digest.tag(0xa2);
    digest.bytes_with_len(file.schema.as_bytes());
    digest.count(file.usfld_size);
    digest.count(file.header.pairs.len());
    for (key, value) in &file.header.pairs {
        digest.bytes_with_len(key.as_bytes());
        digest.bytes_with_len(value.as_bytes());
    }
    digest.count(file.nodes.len());
    for (&index, node) in &file.nodes {
        digest.u64(u64::from(index));
        digest.u64(u64::from(node.code));
        digest.count(node.values.len());
        for value in &node.values {
            digest_value(&mut digest, value);
        }
    }
    digest.count(file.foreign_codes.len());
    for &code in &file.foreign_codes {
        digest.u64(u64::from(code));
    }
    digest.finish()
}

fn digest_value(digest: &mut StableHasher, value: &Value) {
    match value {
        Value::Null => digest.tag(0),
        Value::Int(value) => {
            digest.tag(1);
            digest.u64(*value as u64);
        }
        Value::Double(value) => {
            digest.tag(2);
            digest.f64(*value);
        }
        Value::Char(value) => {
            digest.tag(3);
            digest.u64(u64::from(u32::from(*value)));
        }
        Value::Logical(value) => {
            digest.tag(4);
            digest.boolean(*value);
        }
        Value::Ptr(value) => {
            digest.tag(5);
            digest.u64(u64::from(*value));
        }
        Value::Vector(value) => {
            digest.tag(6);
            digest.optional_f64s(value.as_ref().map(|value| value.as_slice()));
        }
        Value::Interval(value) => {
            digest.tag(7);
            digest.optional_f64s(value.as_ref().map(|value| value.as_slice()));
        }
        Value::Str(value) => {
            digest.tag(8);
            digest.bytes_with_len(value.as_bytes());
        }
        Value::Arr(values) => {
            digest.tag(9);
            digest.count(values.len());
            for value in values {
                digest_value(digest, value);
            }
        }
    }
}

fn byte_digest(bytes: &[u8]) -> u64 {
    let mut digest = StableHasher::new();
    digest.tag(0xa3);
    digest.bytes_with_len(bytes);
    digest.finish()
}

struct StableHasher(u64);

impl StableHasher {
    const fn new() -> Self {
        Self(14_695_981_039_346_656_037)
    }

    fn bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 = (self.0 ^ u64::from(byte)).wrapping_mul(1_099_511_628_211);
        }
    }

    fn tag(&mut self, value: u8) {
        self.bytes(&[value]);
    }

    fn boolean(&mut self, value: bool) {
        self.tag(u8::from(value));
    }

    fn u64(&mut self, value: u64) {
        self.bytes(&value.to_le_bytes());
    }

    fn count(&mut self, value: usize) {
        self.u64(value as u64);
    }

    fn f64(&mut self, value: f64) {
        self.u64(value.to_bits());
    }

    fn bytes_with_len(&mut self, bytes: &[u8]) {
        self.count(bytes.len());
        self.bytes(bytes);
    }

    fn optional_f64s(&mut self, values: Option<&[f64]>) {
        if let Some(values) = values {
            self.tag(1);
            self.count(values.len());
            for &value in values {
                self.f64(value);
            }
        } else {
            self.tag(0);
        }
    }

    const fn finish(self) -> u64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    #[test]
    fn registry_contains_exactly_eight_unique_canonical_cases() {
        let unique: BTreeSet<_> = CASES.iter().map(|case| case.path).collect();
        assert_eq!(unique.len(), CASES.len());
        for case in CASES {
            crate::validate_case_path(case.path).unwrap();
            assert_eq!(case.expected_input_bytes, fixture(case).bytes.len());
        }
    }

    #[test]
    fn json_registry_matches_every_rust_case_and_counter() {
        fn assert_histogram(value: &serde_json::Value, expected: &BTreeMap<u16, usize>) {
            let value = value
                .as_object()
                .expect("class histogram must be an object");
            assert_eq!(value.len(), expected.len());
            for (code, count) in expected {
                assert_eq!(
                    value
                        .get(&code.to_string())
                        .and_then(serde_json::Value::as_u64),
                    Some(*count as u64),
                    "record class {code}"
                );
            }
        }

        let manifest: serde_json::Value =
            serde_json::from_str(include_str!("../cases.json")).unwrap();
        let entries = manifest["cases"].as_array().unwrap();
        let entries: Vec<_> = entries
            .iter()
            .filter(|entry| entry["benchmark_target"] == "xt_io")
            .collect();
        assert_eq!(entries.len(), CASES.len());
        for case in CASES {
            let matches: Vec<_> = entries
                .iter()
                .copied()
                .filter(|entry| entry["path"] == case.path)
                .collect();
            assert_eq!(matches.len(), 1, "registry mismatch for {}", case.path);
            let entry = matches[0];
            assert_eq!(entry["fixture_version"], case.fixture.version());
            assert_eq!(entry["deterministic_seed"].as_u64(), Some(FIXTURE_SEED));
            assert_eq!(
                entry["size_parameters"]["elements"].as_u64(),
                Some(case.expected_input_bytes as u64)
            );
            assert_eq!(entry["policy_values"]["phase"], case.phase.as_str());
            assert_eq!(entry["policy_values"]["wire_encoding"], "text");
            assert_eq!(entry["policy_values"]["schema_base"].as_u64(), Some(13_006));
            let api = match case.phase {
                XtPhase::ParseRecords => "read_xt",
                XtPhase::FullRead => "import",
                XtPhase::WriteText => "export_text",
                XtPhase::RoundTrip => "import-export-import",
            };
            assert_eq!(entry["policy_values"]["public_api"], api);

            let fixture = fixture(case);
            let evidence = fixture.measure_once(case).1;
            verify(case, evidence);
            let counters = &entry["expected_result_counters"];
            let input_file = read_xt(fixture.bytes).unwrap();
            assert_histogram(
                &counters["input_record_classes"],
                &class_histogram(&input_file),
            );
            if evidence.output_records == 0 {
                assert_histogram(&counters["output_record_classes"], &BTreeMap::new());
            } else {
                let output_file = read_xt(fixture.canonical_text.as_bytes()).unwrap();
                assert_histogram(
                    &counters["output_record_classes"],
                    &class_histogram(&output_file),
                );
            }
            for (field, actual) in [
                ("input_bytes", evidence.input_bytes as u64),
                ("input_records", evidence.input_records as u64),
                ("input_classes", evidence.input_classes as u64),
                ("output_bytes", evidence.output_bytes as u64),
                ("output_records", evidence.output_records as u64),
                ("output_classes", evidence.output_classes as u64),
                ("phase_imports", evidence.phase_imports as u64),
                ("phase_exports", evidence.phase_exports as u64),
                ("journal_mutations", evidence.journal_mutations as u64),
                ("skipped_records", evidence.skipped_records as u64),
                (
                    "unsupported_capabilities",
                    evidence.unsupported_capabilities as u64,
                ),
                ("bodies", evidence.bodies as u64),
                ("regions", evidence.regions as u64),
                ("shells", evidence.shells as u64),
                ("faces", evidence.faces as u64),
                ("loops", evidence.loops as u64),
                ("fins", evidence.fins as u64),
                ("edges", evidence.edges as u64),
                ("vertices", evidence.vertices as u64),
                ("points", evidence.points as u64),
                ("curves", evidence.curves as u64),
                ("surfaces", evidence.surfaces as u64),
                ("pcurves", evidence.pcurves as u64),
            ] {
                assert_eq!(counters[field].as_u64(), Some(actual), "{field}");
            }
            for (field, actual) in [
                ("checker_ran", evidence.checker_ran),
                ("checker_clean", evidence.checker_clean),
                ("deterministic_write", evidence.deterministic_write),
                ("roundtrip_equal", evidence.roundtrip_equal),
            ] {
                assert_eq!(counters[field].as_bool(), Some(actual), "{field}");
            }
            for (field, actual) in [
                ("input_class_digest", evidence.input_class_digest),
                ("input_parse_digest", evidence.input_parse_digest),
                ("input_byte_digest", evidence.input_byte_digest),
                ("output_class_digest", evidence.output_class_digest),
                ("output_parse_digest", evidence.output_parse_digest),
                ("output_byte_digest", evidence.output_byte_digest),
                ("semantic_digest", evidence.semantic_digest),
                ("output_digest", evidence.output_digest),
            ] {
                assert_eq!(
                    counters[field].as_str(),
                    Some(format!("{actual:016x}").as_str()),
                    "{field}"
                );
            }
        }
    }

    #[test]
    fn every_phase_is_repeatable_and_reports_reviewed_evidence() {
        for case in CASES {
            let fixture = fixture(case);
            let first = fixture.measure_once(case).1;
            let second = fixture.measure_once(case).1;
            assert_eq!(first, second, "repeatability drift for {}", case.path);
            verify(case, first);
        }
    }
}
