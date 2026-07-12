# Kernel fuzz contracts

This package is an isolated, non-networked robustness harness. It is excluded
from the kernel workspace so the fuzz-only dependency and nightly toolchain do
not affect the kernel MSRV or root lockfile.

The `xt_read` target accepts exactly one selector byte followed by an
arbitrary X_T payload. An even selector parses the payload; an odd selector
parses and, for inputs with at most 4,096 records, attempts atomic import.
Payloads over 256 KiB are rejected by the harness before entering `kxt`.
Successful imports must produce checker-clean bodies. Every returned error must
belong to the stable `kxt`, `kcore`, or `kgraph` code inventory. Panics, aborts,
timeouts, and RSS-limit crossings are failures reported by libFuzzer.

The `nurbs_constructors` target uses a seven-byte structured header followed by
raw little-endian `f64` bit patterns. The header selects curves or surfaces,
polynomial or rational construction, derivative order, projection, and split
direction, while declaring degrees and the knot, point, and weight counts. The
harness rejects inputs over 4 KiB or above fixed degree/count caps before
allocating descriptor vectors. Accepted constructors must satisfy knot,
control-net, and weight invariants; bounded evaluations, derivatives, splits,
restrictions, optional fixed-work projections, and depth-2 implicit isolation
with a 32-cell soft candidate budget must be bitwise repeatable. Curve and
surface seeds remain in one target-specific corpus and are distinct from the
X_T parser/import corpus.

The checked corpus is generated deterministically from repository-owned
Apache-2.0 fixtures. Regenerate and validate it without a fuzz run:

```sh
python3 fuzz/scripts/generate_xt_read_corpus.py
python3 fuzz/scripts/generate_nurbs_constructors_corpus.py
python3 -m unittest scripts.tests.test_fuzz_contract -v
cargo test --manifest-path fuzz/Cargo.toml --locked --no-default-features
```

Install the exact fuzz runner with the pinned nightly, then use the shared
bounded runner. It copies the checked seeds into a fresh disposable corpus and
enforces the same 45-second process deadline as CI, because libFuzzer may grow
its input corpus or fail to honor its own 20-second exploration deadline. The
runner targets the repository's supported POSIX development/CI hosts (Linux and
macOS) so it can terminate the complete cargo-fuzz process group.

```sh
cargo install cargo-fuzz --version 0.13.2 --locked
python3 scripts/fuzz_smoke.py xt_read
python3 scripts/fuzz_smoke.py nurbs_constructors
```

The 20-second duration is a smoke budget, not a correctness threshold. Generated
crashes and coverage data remain ignored. A minimized, understood failure is
promoted later under the Q7 regression contract rather than committed under its
runner-generated hash.
