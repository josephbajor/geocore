# Blocking test-throughput foundation subproject

Status: developer fast/full/focused lanes and CI scheduling implemented; corpus consolidation remains blocking

## Outcome

Keep correctness feedback short enough that core kernel development can proceed
without normalizing skipped tests. The developer runner uses only the Python
standard library and Cargo, adds no dependency to a kernel crate, and reports
elapsed wall time for every stage and for the complete lane. Timing is
diagnostic evidence, never a correctness threshold.

This subproject remains blocking for further core-system breadth until the
production-corpus duplication is consolidated. The landed developer lanes and
CI scheduling make the boundary explicit and measurable in the meantime.

## Commands

Run these from the repository root:

```sh
# Inspect the exact reviewed target classification without running tests.
python3 scripts/test_lanes.py list

# Normal development gate: unit, binary, doc, tooling, and ordinary integration tests.
python3 scripts/test_lanes.py fast

# Tight inner loop for one integration binary or one package library.
python3 scripts/test_lanes.py focused -p kxt -t read
python3 scripts/test_lanes.py focused -p kgeom --lib --filter surface_point

# Pre-merge/handoff gate: every workspace target, every doc test, and tooling contracts.
python3 scripts/test_lanes.py full
```

Each executable lane accepts `--dry-run` to show its exact Cargo/Python
commands and `--release` to select Cargo's release profile. The focused lane
also accepts `--filter`, `--exact`, and `--nocapture`.

The runner prints a start line, exact shell-quoted command, pass/fail result,
and elapsed seconds for every stage. It fails at the first unsuccessful stage
and preserves that subprocess's exit status.

## Lane contract

The fast lane is not `cargo test --workspace --exclude kxt`. It retains all
seven current lightweight `kxt` integration binaries:

- `import_tess`;
- `inspect_cli`;
- `intersection_chart`;
- `offset_surface`;
- `oracle_cli`;
- `read`; and
- `write`.

Only these 13 reviewed production-corpus ratchets are excluded from fast:

- `corpus_manifest`;
- `equal_limit_intersection`;
- `finite_open_cubic_dual_offset`;
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

Twelve binaries are classified by a concrete source boundary: each embeds the
908 KiB, 7,423-node `exemplar.x_t` production fixture. `corpus_manifest` is the
thirteenth because its observed-corpus-stage test reaches the production
fixture through `manifest.tsv` and performs the same production-scale
reconstruction work. All 13 remain mandatory in `full`. The runner validates
the Cargo workspace, package identities, and source tree before every listing
or run. A new workspace package, embedded-exemplar consumer, or renamed/removed
ratchet fails closed until the reviewed inventory is updated.

The contract tests live in `scripts/tests/test_test_lanes.py` and run as part
of both developer lanes through the existing standard-library tooling suite:

```sh
python3 -m unittest scripts.tests.test_test_lanes -v
```

## Initial timing evidence

Before X_T test consolidation, this machine measured the representative
seven-sample ratchet at 172.58 seconds of test time and 185.92 seconds of wall
time:

```sh
cargo test -p kxt --test finite_open_seven_sample_dual_offset
```

The manifest-driven observed-corpus-stage test was independently observed at
approximately 169 seconds, which is why `corpus_manifest` is not in the fast
lane despite lacking the embedded-fixture marker. These are diagnostic
baselines, not pass/fail thresholds.

## Remaining blocking work

1. Consolidate historical v1-v12 production traversal assertions around one
   current end-to-end corpus ratchet plus small certifier-level fixtures.
2. Record before/after fast and full lane timings on the named development
   host; use the measurements to find any remaining unexpected hot target.
3. Measure the implemented parallel debug/release CI profiles and rolling
   caches, then consider target-level sharding only if corpus consolidation
   leaves a material critical-path imbalance.
4. Investigate operation-scoped certificate caching only after target-level
   timing shows proof computation remains the dominant cost. Logical
   Work/Items/Depth accounting must remain unchanged by physical caching.

`cargo test --workspace` remains valid and complete. The lane runner is a
developer scheduling surface, not a replacement for Cargo's test semantics.
