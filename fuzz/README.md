# Kernel fuzz contracts

This package is an isolated, non-networked robustness harness. It is excluded
from the kernel workspace so the fuzz-only dependency and nightly toolchain do
not affect the kernel MSRV or root lockfile.

The first target, `xt_read`, accepts exactly one selector byte followed by an
arbitrary X_T payload. An even selector parses the payload; an odd selector
parses and, for inputs with at most 4,096 records, attempts atomic import.
Payloads over 256 KiB are rejected by the harness before entering `kxt`.
Successful imports must produce checker-clean bodies. Every returned error must
belong to the stable `kxt`, `kcore`, or `kgraph` code inventory. Panics, aborts,
timeouts, and RSS-limit crossings are failures reported by libFuzzer.

The checked corpus is generated deterministically from repository-owned
Apache-2.0 fixtures. Regenerate and validate it without a fuzz run:

```sh
python3 fuzz/scripts/generate_xt_read_corpus.py
python3 -m unittest scripts.tests.test_fuzz_contract -v
cargo test --manifest-path fuzz/Cargo.toml --locked --no-default-features
```

Install and run the exact fuzz runner with the pinned nightly:

```sh
cargo install cargo-fuzz --version 0.13.2 --locked
cd fuzz
cargo fuzz run xt_read --features fuzzing corpus/xt_read -- -seed=5860406134146269190 -max_len=262145 -timeout=5 -rss_limit_mb=2048 -max_total_time=20 -artifact_prefix=artifacts/xt_read/
```

The 20-second duration is a smoke budget, not a correctness threshold. Generated
crashes and coverage data remain ignored. A minimized, understood failure is
promoted later under the Q7 regression contract rather than committed under its
runner-generated hash.
