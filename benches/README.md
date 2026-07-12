# Kernel benchmark harness

This isolated package owns benchmark-only dependencies and deterministic
fixtures. It is excluded from the normal Cargo workspace so `cargo test
--workspace` and the kernel MSRV contract do not depend on the benchmark
runner lifecycle.

## Contract

Every case is registered in `cases.json` with a five-segment path:

```text
<subsystem>/<operation>/<fixture>/<scale>/<policy>
```

Fixture construction, seeds, expected counters, and invariant checks are
deterministic. Criterion measures elapsed time, but timing never establishes
correctness. The runner always marks measurements advisory.

Criterion is pinned to `0.8.2`. Machine-readable measured runs use
`cargo-criterion 1.1.0` because its JSON-lines output is a documented external
tool contract. Install that exact runner when recording a real measurement:

```sh
cargo install --locked --version 1.1.0 cargo-criterion
```

The package has its own reviewed `Cargo.lock`; do not add benchmark
dependencies to the root workspace lockfile.

## Offline validation

These commands require no benchmark execution or network:

```sh
python3 scripts/benchmark_baseline.py smoke
python3 scripts/benchmark_baseline.py validate
python3 -m unittest discover -s scripts/tests -p 'test_benchmark_baseline.py' -v
```

They validate the case registry, JSON Schema root contract, the complete
checked-in synthetic report, strict cargo-criterion parsing, runner-output
round trips, and fail-closed behavior for missing identity fields or format
drift.

The checked-in `example.synthetic.v1.json` and synthetic JSON-lines fixture
are schema/parser examples only. They are explicitly comparison-ineligible and
contain no performance claim.

## Compile and smoke the Rust target

```sh
cargo test --manifest-path benches/Cargo.toml
cargo bench --manifest-path benches/Cargo.toml --bench benchmark_contract --no-run
cargo bench --manifest-path benches/Cargo.toml --bench topology_commit --no-run
cargo bench --manifest-path benches/Cargo.toml --bench graph_build --no-run
cargo bench --manifest-path benches/Cargo.toml --bench graph_traversal --no-run
cargo bench --manifest-path benches/Cargo.toml --bench body_tessellation --no-run
cargo bench --manifest-path benches/Cargo.toml --bench nurbs_isolation --no-run
cargo bench --manifest-path benches/Cargo.toml --bench curve_pair_isolation --no-run
cargo bench --manifest-path benches/Cargo.toml --bench curve_pair_solve --no-run
cargo bench --manifest-path benches/Cargo.toml --bench xt_io --no-run
```

The curve-pair solve ladder pins ordered structured incomplete-proof evidence
separately from emitted contact geometry, then folds both digests into its
semantic output contract.

The Q1 target verifies the result digest before measurement and again in every
timed iteration. The Q2 target provides the 21 checked-commit, incremental
index-refresh, rejection, and full-rebuild cases in the quality contract. It
times only the transaction edit and ordinary `commit_checked`, except that the
full-rebuild ladder times the independent reference-index rebuild itself.
Fixture cloning, store/index snapshots, rollback probes, and correctness checks
run outside the accumulated duration. Read-only full-rebuild samples reuse one
verified prepared fixture so Criterion calibration does not repeat excluded
cloning and snapshot work. Set `KERNEL_BENCH_SMOKE=1` and pass one full case
path after `--` for a bounded local smoke run.

The Q2a target registers 17 graph-construction cases: independent plane nodes
at 1 through 10,000 nodes, offset dependency chains and shared-basis fanout at
1 through 1,000 edges, and rejected transient chains with exact undo rollback
at 1 through 1,000 nodes. Descriptor policy and rollback control state are
prepared outside timing. Only graph insertion, dependency maintenance, and
rollback are timed; evaluation is deliberately absent. Every sample verifies
node/edge counts, actual reverse-index registrations, zero complete-order
rebuilds, stable iteration order, graph and reverse-index digests, validation,
and rollback equality. The indexed replacement retains insertion-ordered
adjacency and uses hash storage only for non-observable lookup/membership. The
planned diamond row is deferred because current
descriptors expose at most one dependency; the harness does not invent a
benchmark-only graph shape.

The Q2b target registers eight prepared-chain traversal cases. Dependency-
first closure and a deterministic missing-path search each run at 1, 10, 100,
and 1,000 dependency edges. Graph construction, validation, and stable ordinal
indexing stay outside timing; every sample pins result presence, exact returned
node count and order digest, and repeatability. Traversal output and cycle paths
remain vector-ordered, while active/completed membership uses hash lookup that
is never iterated. The smallest closure row runs in CI.

The Q3 v2 contract registers twenty closed-solid cases: ten analytic block,
cylinder, cone, sphere, and torus rows; two mixed-store target-cylinder rows;
four certified imported NURBS-face and tolerant-edge rows at `1e-2` and
`1e-3`; and a four-point certified imported-cylinder ladder at `1e-2`,
`3e-3`, `1e-3`, and `3e-4`. Cylinder, sphere, and torus exercise periodic
seam/pole assembly. The NURBS fixture is an
exact benchmark-owned copy of `solid_block_nurbs_face.x_t`; setup asserts its
6,488-byte identity, portable digest, one B-surface, and absence of pcurves.
The tolerant fixture references the current certified oracle outbox and asserts
its 7,172-byte identity, portable digest, one curve-less tolerant edge, two
NURBS pcurve uses, and four intentionally skipped geometric-owner records. The
Python contract checks all three imported SHA-256 values against
`docs/oracle-certification.json`. One immutable fixture and one Serial
compatibility-v1 operation context are prepared per case. Import, context
construction, outcome unpacking, finite/range, watertightness, orientation,
volume, and exact mesh/report repeatability checks run outside timing. Only the
`tessellate_body_with_context` call is measured.

`usage_consumed` follows the closed `q3-usage.v1` order:

1. surface projection halvings, candidates, Newton iterations, queries, samples;
2. face boundary depth, boundary splits, interior passes, triangles, vertices;
3. graph dependency depth and node visits; and
4. body edge depth, edge splits, edge-storage items, iso depth, iso splits,
   mesh vertices, prepared-patch items, retained triangles, and structural items.

Every case pins all 21 values, a profile/policy/stage digest, contextual API and
execution identity, and zero completion-event counts. Existing mesh bits and
digests remain unchanged. The NURBS-face rows activate projection candidates,
Newton depth, queries, and samples; the tolerant rows prove the explicit
SP-curve/NURBS-pcurve path remains projection-free and account for its graph
queries. These measurements do not justify finite `bounded_v1` caps:
projection backtracking remains zero, and genuinely curved NURBS, more imported
representations, and four-point ladders across additional representations are
still required. The
mixed-store rows prepare a block, the target cylinder, and a sphere; they pin
three stored bodies, a shifted target identity, and exact equality with the
standalone cylinder's normalized output and complete report.

The 2,309-byte imported cylinder pins the certified SHA-256, analytic curved
surface class, three faces, two vertex-less ring edges, and scale-sensitive
mesh/accounting evidence. Its coarse and fine volume ratios are approximately
`0.94968`, `0.98487`, `0.99436`, and `0.99857`; the reviewed lower floors are
explicitly `0.94`, `0.98`, `0.99`, and `0.998` because the coarsest chord
request is about 7.7% of the fixture radius. Face passes progress `3 → 4 → 5 →
6`, edge depth `2 → 2 → 3 → 4`, and target vertices `202 → 540 → 2,320 →
12,248`, making the non-smooth tolerance-tier transitions part of the contract.

The standalone Q3 face ladder separately measures a half-cylinder trimmed
surface at chord tolerances `1e-2` and `1e-3` through
`tessellate_with_context`. This split is intentional: whole-body tessellation
pre-refines every shared edge with a safety margin and then freezes it, so a
nested face-boundary split would be a crack-prevention failure. The standalone
rows activate all five face stages and pin boundary depth/splits, interior
passes, mesh triangles/vertices, complete reports, and mesh bits. Fixture,
trim, session, context, and verification remain outside timing.

The first Q4 slice registers six contextual implicit-isolation cases. It varies
polynomial versus rational single patches, one versus four extracted Bezier
patches, retained candidates versus a certified separated miss, and exact
`N-1` work/candidate budget crossings. Surface construction, Bezier extraction,
BVH construction, operation-context setup, reports, digests, and conservative
cover verification are excluded from the measured duration. Degree and larger
control-net/patch-count ladders remain deferred.

The curve-pair Q4 slice adds six deterministic exact-subdivision cases. It
varies polynomial/rational curves, retained endpoint contacts versus a hidden
separated miss, and independent work, candidate-high-water, and depth stops.
Every limited result must retain its conservative parent cover and remain
indeterminate. Curve and policy construction, operation-context setup, report
extraction, and evidence verification remain outside the measured duration.

The solve-level Q4 slice adds six contextual cases over the exact-cell-driven
NURBS/NURBS path: polynomial and rational transverse contacts, tangency, two
roots, a subdivision-proven hidden miss, and zero seed admission. It pins all
four report stages, ordered contact/output digests, exact in-cell witness
re-evaluation, and the seed-limit crossing. Geometry, session, request
overrides, and evidence verification are outside timing; the public contextual
solve, including profile composition and report finalization, is timed.

The first Q5 slice uses only the repository's Apache-2.0 hand-authored
`block.x_t` and `offset_plane.x_t` fixtures. Eight cases time the public
parse-to-records API, complete import/reconstruction, combined writer
validation/planning/text emission, and read-write-read round trip separately.
The writer phase includes the mandatory body check performed by `export_text`;
the current API does not expose that check, writer planning, serialization, or
the byte sink as independent phases, so Q5 does not claim those boundaries.
Fixture loading, prepared source import, additional benchmark checker
verification, record/store/byte digests, and round-trip comparison remain
outside measured duration. Neutral binary and larger redistributable corpora
remain follow-ups.

## Record a measured run

Write generated reports outside the repository or below ignored
`benches/results/`:

```sh
python3 scripts/benchmark_baseline.py run \
  --smoke \
  --output benches/results/q1-smoke.json
python3 scripts/benchmark_baseline.py validate benches/results/q1-smoke.json
```

The report includes repository state, compiler/Cargo/target/profile/features,
OS/architecture/CPU/cores/available memory, runner configuration and affinity,
fixture identity and policy, result counters, and Criterion's typical estimate.
A dirty worktree is recorded but is not comparison-eligible.

## Compare identity

```sh
python3 scripts/benchmark_baseline.py compare baseline.json candidate.json
```

At Q1 this command only determines whether comparison identities match. It
refuses compatibility when either input is ineligible or schema, repository,
host, compiler, runner, fixture, policy, result counters, or measurement units
differ. It does not calculate ratios, thresholds, or a performance pass/fail
result.

Compact reviewed baselines belong under `baselines/<host>/<revision>.json`.
Raw cargo-criterion streams and generated reports are artifacts and must not be
committed.
