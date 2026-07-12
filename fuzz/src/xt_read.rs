//! Bounded X_T parser/import robustness contract.

use kcore::error::ErrorCode;
use kgeom::vec::Point3;
use ktopo::check::check_body;
use ktopo::entity::{Body, Edge, Face, Fin, Loop, Region, Shell, Vertex};
use ktopo::store::Store;
use kxt::{XtError, import, read_xt};

/// Maximum X_T payload admitted by the target.
pub const MAX_PAYLOAD_BYTES: usize = 256 * 1024;

/// Maximum parsed-record count admitted to reconstruction.
pub const MAX_IMPORT_RECORDS: usize = 4_096;

/// Parser/read operation selected by the leading byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadMode {
    /// Parse the complete transport stream into records.
    Parse,
    /// Parse and atomically reconstruct a bounded record graph.
    Import,
}

impl ReadMode {
    const fn from_selector(selector: u8) -> Self {
        if selector & 1 == 0 {
            Self::Parse
        } else {
            Self::Import
        }
    }
}

/// Exercise one selector-plus-payload input.
///
/// Inputs without a selector or above the payload cap are intentionally
/// rejected by the harness. Kernel panics remain visible to libFuzzer. A
/// successful import is checked and every failure retains a known stable error
/// code.
pub fn exercise(input: &[u8]) {
    let Some((&selector, payload)) = input.split_first() else {
        return;
    };
    if payload.len() > MAX_PAYLOAD_BYTES {
        return;
    }

    match read_xt(payload) {
        Ok(file) => {
            if ReadMode::from_selector(selector) == ReadMode::Import
                && file.nodes.len() <= MAX_IMPORT_RECORDS
            {
                exercise_import(payload);
            }
        }
        Err(error) => assert_stable_error(&error),
    }
}

fn assert_failed_import_is_atomic(store: &mut Store) {
    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<Region>(), 0);
    assert_eq!(store.count::<Shell>(), 0);
    assert_eq!(store.count::<Face>(), 0);
    assert_eq!(store.count::<Loop>(), 0);
    assert_eq!(store.count::<Fin>(), 0);
    assert_eq!(store.count::<Edge>(), 0);
    assert_eq!(store.count::<Vertex>(), 0);
    assert_eq!(store.count::<Point3>(), 0);
    assert_eq!(store.geometry().len(), 0);
    store
        .transaction()
        .expect("failed X_T import must leave the store transaction-reusable")
        .rollback()
        .expect("empty transaction rollback must succeed after failed X_T import");
}

fn exercise_import(payload: &[u8]) {
    let mut store = Store::new();
    match import(payload, &mut store) {
        Ok(reconstruction) => {
            for body in reconstruction.bodies {
                let faults = check_body(&store, body)
                    .expect("successful X_T import must leave readable body topology");
                assert!(
                    faults.is_empty(),
                    "successful X_T import produced checker faults: {faults:?}"
                );
            }
        }
        Err(error) => {
            assert_failed_import_is_atomic(&mut store);
            assert_stable_error(&error);
        }
    }
}

fn assert_stable_error(error: &XtError) {
    let code = error.code();
    assert_eq!(ErrorCode::new(code.as_str()), Ok(code));
    assert!(
        known_error_code(code),
        "X_T returned an error outside the reviewed stable inventories: {code}"
    );
    if error.class() == kcore::error::ErrorClass::Unsupported {
        assert!(
            error.capability_id().is_some(),
            "unsupported X_T error omitted its stable capability identity"
        );
    }
}

fn known_error_code(code: ErrorCode) -> bool {
    kxt::error::code::ALL.contains(&code)
        || kcore::error::code::ALL.contains(&code)
        || kcore::operation::code::ALL.contains(&code)
        || kgraph::eval_error_code::ALL.contains(&code)
}

#[cfg(test)]
mod tests {
    use super::*;

    const CORPUS: &[&[u8]] = &[
        include_bytes!("../corpus/xt_read/parse-minimal-valid.xtseed"),
        include_bytes!("../corpus/xt_read/parse-block.xtseed"),
        include_bytes!("../corpus/xt_read/import-block.xtseed"),
        include_bytes!("../corpus/xt_read/import-offset-plane.xtseed"),
        include_bytes!("../corpus/xt_read/parse-header-boundary.xtseed"),
        include_bytes!("../corpus/xt_read/parse-token-boundary.xtseed"),
        include_bytes!("../corpus/xt_read/import-record-boundary.xtseed"),
    ];

    #[test]
    fn checked_corpus_replays_without_panics() {
        for seed in CORPUS {
            exercise(seed);
        }
    }

    #[test]
    fn valid_seeds_reach_the_reviewed_success_paths() {
        let minimal = include_bytes!("../corpus/xt_read/parse-minimal-valid.xtseed");
        let parsed = read_xt(&minimal[1..]).expect("minimal seed must remain parser-valid");
        assert!(parsed.nodes.is_empty());

        for seed in [
            include_bytes!("../corpus/xt_read/import-block.xtseed").as_slice(),
            include_bytes!("../corpus/xt_read/import-offset-plane.xtseed").as_slice(),
        ] {
            assert_eq!(ReadMode::from_selector(seed[0]), ReadMode::Import);
            let mut store = Store::new();
            let reconstruction =
                import(&seed[1..], &mut store).expect("reviewed import seed must remain supported");
            assert!(!reconstruction.bodies.is_empty());
            for body in reconstruction.bodies {
                assert!(check_body(&store, body).is_ok_and(|faults| faults.is_empty()));
            }
        }
    }

    #[test]
    fn boundary_truncation_seeds_reach_classified_rejections() {
        for seed in [
            include_bytes!("../corpus/xt_read/parse-header-boundary.xtseed").as_slice(),
            include_bytes!("../corpus/xt_read/parse-token-boundary.xtseed").as_slice(),
            include_bytes!("../corpus/xt_read/import-record-boundary.xtseed").as_slice(),
        ] {
            let error = match read_xt(&seed[1..]) {
                Ok(_) => panic!("truncation seed must remain invalid"),
                Err(error) => error,
            };
            assert_stable_error(&error);
        }
    }

    #[test]
    fn selectors_cover_parse_and_import_without_changing_payload() {
        let payload = include_bytes!("../../crates/kxt/tests/fixtures/block.x_t");
        for (selector, expected) in [(0, ReadMode::Parse), (1, ReadMode::Import)] {
            assert_eq!(ReadMode::from_selector(selector), expected);
            let mut input = Vec::with_capacity(payload.len() + 1);
            input.push(selector);
            input.extend_from_slice(payload);
            assert_eq!(&input[1..], payload);
            exercise(&input);
        }
    }

    #[test]
    fn empty_and_oversized_harness_inputs_are_bounded_rejections() {
        exercise(&[]);
        exercise(&vec![0; MAX_PAYLOAD_BYTES + 2]);
    }

    #[test]
    fn inventories_are_nonempty_unique_and_namespaced() {
        let all = kxt::error::code::ALL
            .iter()
            .chain(kcore::error::code::ALL)
            .chain(kcore::operation::code::OWNED)
            .chain(kgraph::eval_error_code::ALL)
            .copied()
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(
            all.len(),
            kxt::error::code::ALL.len()
                + kcore::error::code::ALL.len()
                + kcore::operation::code::OWNED.len()
                + kgraph::eval_error_code::ALL.len()
        );
        assert!(all.iter().all(|code| code.as_str().contains('.')));
    }

    #[test]
    fn parse_valid_reconstruction_failure_is_atomic() {
        let seed = include_bytes!("../corpus/xt_read/parse-minimal-valid.xtseed");
        assert!(read_xt(&seed[1..]).is_ok());
        exercise_import(&seed[1..]);
    }
}
