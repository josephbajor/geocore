# Test-throughput foundation subproject

Status: initial blocking checkpoint implemented and verified; continuous measured optimization remains

## Outcome

Keep correctness feedback short enough that core kernel development can proceed
without normalizing skipped tests. The developer runner uses only the Python
standard library and Cargo, adds no dependency to a kernel crate, and reports
elapsed wall time for every stage and for the complete lane. Timing is
diagnostic evidence, never a correctness threshold.

The initial checkpoint no longer blocks further core-system work: the
integrated full lane passed after the lane split and first corpus
consolidation. `full` remains the required pre-merge/handoff gate; the shorter
lanes change scheduling, not the definition of complete correctness evidence.

## Commands

Run these from the repository root:

```sh
# Inspect the exact reviewed target classification without running tests.
python3 scripts/test_lanes.py list

# Normal edit/commit gate: unit/binary tests plus a curated integration smoke set.
python3 scripts/test_lanes.py fast

# Broad local gate: all non-corpus integration targets and tooling.
python3 scripts/test_lanes.py standard

# Workspace doctests, including compile-fail architectural boundaries.
python3 scripts/test_lanes.py docs

# Tight inner loop for one integration binary or one package library.
python3 scripts/test_lanes.py focused -p kxt -t read
python3 scripts/test_lanes.py focused -p kgeom --lib --filter surface_point

# Pre-merge/handoff gate: every workspace target, every doc test, and tooling contracts.
python3 scripts/test_lanes.py full
```

Each executable lane accepts `--dry-run` to show its exact Cargo/Python
commands and `--release` to select Cargo's release profile. The focused lane
also accepts `--filter`, `--exact`, and `--nocapture`.

The runner requires Python 3.11+ for standard-library TOML parsing.

The runner prints a start line, exact shell-quoted command, pass/fail result,
and elapsed seconds for every stage. It fails at the first unsuccessful stage
and preserves that subprocess's exit status.

## Lane contract

The `fast` lane runs every workspace library and binary test, then 13 curated
integration targets spanning determinism, roadmap-ledger shape, intersection
certificates, kernel lifecycle, topology transactions, operation completion,
X_T read/write/charts, and the facade-only example. Its X_T smoke set is:

- `intersection_chart`;
- `read`; and
- `write`.

It intentionally omits broad doc/tooling stages and the remaining integration
binaries so ordinary edit/commit feedback stays bounded, while retaining its
own lane-classification/command contract suite as a final stage.

The `standard` lane adds every one of the 79 current non-corpus integration
targets and the Python tooling contracts, but not documentation tests. It
retains all seven current lightweight `kxt` integration binaries:

- `import_tess`;
- `inspect_cli`;
- `intersection_chart`;
- `offset_surface`;
- `oracle_cli`;
- `read`; and
- `write`.

Only these 14 reviewed production-corpus ratchets are excluded from
`standard`:

- `corpus_manifest`;
- `equal_limit_intersection`;
- `finite_open_cubic_dual_offset`;
- `finite_open_five_sample_dual_offset`;
- `finite_open_nurbs_endpoint_roundoff`;
- `finite_open_plane_nurbs_data`;
- `finite_open_plane_offset_nurbs_data`;
- `finite_open_seven_sample_dual_offset`;
- `finite_open_two_sample_dual_offset`;
- `offset_nurbs_intersection`;
- `periodic_nurbs`;
- `plane_sp_curve`;
- `terminated_intersection`; and
- `zero_multiplicity_knot_padding`.

Thirteen binaries are classified by a concrete source boundary: each names the
908 KiB, 7,423-node `exemplar.x_t` production fixture. `corpus_manifest` is the
fourteenth because its observed-corpus-stage test reaches the production
fixture through `manifest.tsv` and performs the same production-scale
reconstruction work. All 14 remain mandatory in `full`. Cargo metadata is the
authority for integration-target names and source paths; nonstandard explicit
targets therefore cannot silently fall outside `standard`. The runner also
validates workspace/package identity, smoke membership, direct fixture
references, and the exact 93/79/14 total/standard/corpus counts before every
listing or run. Drift fails closed until the reviewed inventory is updated.

The `docs` lane runs `cargo test --workspace --doc` explicitly. Its executable
and `no_run` examples check documented use, while its compile-fail examples
enforce architectural boundaries such as facade opacity, topology mutation
authority, and checked transaction use. Separating this compiler-intensive
stage from `standard` shortens broad local feedback; it does not weaken or
remove those contracts. `full` still runs every workspace target, every
doctest, and the Python tooling contracts as the required pre-merge/handoff
evidence.

The contract tests live in `scripts/tests/test_test_lanes.py`. They run in
`fast` directly, within the `standard` and `full` tooling stages, and can be
invoked independently during runner work:

```sh
python3 -m unittest scripts.tests.test_test_lanes -v
```

## Timing evidence

Measurements below were taken on 2026-07-14 on a MacBook Pro `Mac16,7` with an
Apple M4 Pro (14 cores) and 48 GB RAM, Darwin 25.5.0 arm64, rustc 1.93.0, and
Python 3.14.0. Commands used Cargo's debug profile and redirected output to a
file. Build artifacts were incremental; the final `fast` run was warm after
the integrated `full` run. Timing remains diagnostic and is not a test
threshold.

The first broad non-corpus precursor passed in 1,432.237 seconds (23m52s), but
it is not a reproducible performance baseline. Cargo was warm, test harnesses
reported only about 107 seconds of work, and about 1,322 seconds remained as
unexplained idle/wait time. Host sleep was ruled out. The retained record is a
contaminated-run diagnostic, not evidence that ordinary integration binaries
normally require 23 minutes.

A clean warm rerun of the same pre-split shape completed in 270.517 seconds:
59.467 seconds for workspace library/binary and all 79 non-corpus integration
tests, 207.720 seconds for documentation tests, and 3.327 seconds for Python
tooling contracts. Documentation compilation therefore accounted for 76.8%
of that reproducible broad lane. Moving it to the explicit `docs` lane leaves
the new `standard` contract near one minute. Direct post-split runs passed in
62.900 seconds for `standard` and 176.581 seconds for `docs`. The independent
lane measurements do not sum to the pre-split total because rustdoc compilation
and process-startup costs vary between runs.

| Lane or observation | Total wall time | Role |
| --- | ---: | --- |
| contaminated broad precursor | 1,432.237 s | retained anomaly; about 1,322s was unexplained idle/wait time |
| reproducible warm pre-split broad lane | 270.517 s | 59.467s ordinary Rust, 207.720s docs, 3.327s tooling |
| post-split `standard` | 62.900 s | ordinary Rust plus tooling |
| explicit `docs` | 176.581 s | workspace doctests and compile-fail boundaries |
| final warm `fast` | 14.231 s | edit/commit gate with unit/binary tests, 13 integration targets, and self-contract |
| integrated `full` | 1,726.501 s | every workspace target, docs, tooling |

The post-documentation run that rebuilt the ledger-bearing `kcore` target took
44.484s; its immediate warm repeat took 14.430s. The full result breaks down
into 1,556.453s for all workspace targets, 166.816s for documentation tests,
and 3.231s for 87 Python tooling contracts.
The longest remaining individual X_T suites were endpoint roundoff at 294.22s
and finite-open Plane/Offset(NURBS) data at 236.18s. The retained historical
v12 seven-sample frontier passed in 166.46s; the later v13 five-sample
production frontier passed in 183.38s.

Before X_T test consolidation, this machine measured the representative
seven-sample ratchet at 172.58 seconds of test time and 185.92 seconds of wall
time:

```sh
cargo test -p kxt --test finite_open_seven_sample_dual_offset
```

The manifest-driven observed-corpus-stage test was independently observed at
approximately 169 seconds, which is why `corpus_manifest` is in the
production-corpus group and excluded from `standard` despite lacking a direct
fixture reference. These are diagnostic baselines, not pass/fail thresholds.

The first audit removed two historical full-exemplar replays whose accepted
prefixes are subsumed by the retained v13 record-3609 frontier. Each suite keeps
its exact aggregate profile values and accounting modes, source-payload pins,
isolated certificate and N/N-1 resource crossings, malformed-input evidence,
and rollback assertions:

| Integration target | Before | After | Saved |
| --- | ---: | ---: | ---: |
| `finite_open_cubic_dual_offset` | 145.25 s | 6.22 s | 139.03 s |
| `zero_multiplicity_knot_padding` | 159.50 s | 3.79 s | 155.71 s |
| **Combined cargo-reported test time** | **304.75 s** | **10.01 s** | **294.74 s** |

The seven-sample suite remains the authoritative end-to-end historical v12
record-4230 boundary. Its 166.46s rerun passed in the integrated checkpoint
gate; the five-sample suite now adds the v13 production traversal through
record 4230 and the exact record-3609 resource stop. Neither was replaced with
profile arithmetic.

## Follow-on measured work

1. Audit endpoint-roundoff's remaining multi-profile replays first, because the
   full-lane timing identifies it as the largest production-corpus target. Do
   not remove distinct frontier or rollback evidence merely because it is
   expensive.
2. Measure the implemented parallel debug/release CI profiles and rolling
   caches, then consider target-level sharding only if corpus consolidation
   leaves a material critical-path imbalance.
3. Consolidate documentation compile-fail examples only if the explicit docs
   lane becomes a material bottleneck; retain every facade and checked-mutation
   boundary rather than optimizing compiler invocations by weakening evidence.
4. Investigate operation-scoped certificate caching only after target-level
   timing shows proof computation remains the dominant cost. Logical
   Work/Items/Depth accounting must remain unchanged by physical caching.

`cargo test --workspace` remains valid and complete. The lane runner is a
developer scheduling surface, not a replacement for Cargo's test semantics.
