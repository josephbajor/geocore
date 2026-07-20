# F7 quality, fuzzing, and performance harnesses

Make toolchain compatibility, robustness exploration, and performance trends
repeatable before modeling breadth grows. The harnesses protect observable kernel
contracts and algorithmic scale. Elapsed time is a measurement, never a
correctness oracle.

Status: Q0–Q2b, Q8, and the first Q3–Q6 foundation slices landed (pinned
toolchain; benchmark contract/runner; topology-commit, graph-construction, and
graph-traversal ladders; body/face tessellation, NURBS-isolation, and X_T-I/O
slices; two fuzz targets; bounded CI jobs); broader benchmark families, the
remaining fuzz targets, and corpus expansion remain evidence-driven.

## Contract — pinned toolchain (Q0)

- `rust-toolchain.toml` pins Rust 1.93.0 (minimal profile plus `rustfmt` and
  `clippy`); the workspace declares `rust-version = "1.93.0"`, inherited by every
  member; the three-OS CI matrix installs that exact version, not a floating
  channel, and the explicit workflow value must stay synchronized with the file.
- Pinned version and MSRV are intentionally equal. Lowering the MSRV requires a
  separate change that compiles and tests every target with the proposed version;
  raising either value requires release notes and a CI-green toolchain commit. Do
  not claim compatibility with an untested compiler.
- Benchmark deps live in the excluded `benches/` package; fuzz-only deps and the
  pinned nightly live in the excluded `fuzz/` package with its own lockfile.
  Neither raises the kernel workspace MSRV or enters its root lockfile, and normal
  `cargo test --workspace` must remain sufficient for all promoted regressions.

## Contract — governing rules

1. A benchmark verifies its result before recording a sample. Timing never
   decides whether output is correct.
2. Fixtures, operation sequences, tolerances, work limits, and seeds are named
   and versioned. Random setup is never inside a timed section.
3. Benchmark ladders change one scale dimension at a time and report semantic
   work counts next to elapsed time where the implementation exposes them.
4. CI smoke jobs prove that harnesses compile, terminate within fixed work/time
   bounds, and retain invariants. Shared CI runners do not enforce wall-clock
   regression thresholds.
5. Performance comparisons are made on a named, stable benchmark host. A baseline
   from a different host or compiler is informative, not comparable.
6. Fuzz failures retain the exact input and toolchain identity. Minimized,
   deterministic failures become ordinary regression tests.
7. Fuzz targets may reject malformed input, report unsupported capability, or
   return indeterminate. They must not panic, abort, leak unbounded work, commit
   invalid topology, or claim unsupported completion evidence.
8. Benchmark-only observability must not become a stable public kernel API. A
   doc-hidden seam may exist only behind a `benchmark-internals` feature for
   counters that cannot otherwise be observed; it exposes no mutable index
   representation and is absent without the feature.

## Contract — benchmark case identity and metadata

Every case has a stable five-segment path
`<subsystem>/<operation>/<fixture>/<scale>/<policy>`, registered in the versioned
`benches/cases.json`; `benches/baselines/schema.json` defines the closed report
shape. Each run emits machine-readable metadata: schema version and case path;
repository revision and dirty flag; rustc/cargo versions, target triple, profile,
features; OS, arch, CPU model, core count, memory; runner version, warm-up/sample
config, affinity; fixture version, size parameters, tolerance/policy values, and
deterministic seed; and operation-relevant result counters (affected bodies,
triangles, candidates, parsed records, output bytes). Store compact reviewed
baselines under `benches/baselines/`; large raw sample streams belong in CI
artifacts, not Git. A baseline comparison must refuse a pass/fail judgement when
schema, host identity, compiler, target, profile, fixture version, or case
parameters differ. Fixture construction, cloning, snapshots, and invariant
verification are always excluded from measured duration.

**Verification before recording a sample.** Tessellation/operation succeeds within
configured limits; the mesh is watertight when the fixture promises a closed body;
indices and coordinates are finite and in range; orientation/volume expectations
hold; and repeated runs produce the same output digest. Budget exhaustion must
retain the expected indeterminate reason and is never recorded as a complete miss.
Ladders reveal output-sensitive scaling; different tolerances are never compared
as equal work.

## Contract — regression policy

- Correctness/invariant failures fail everywhere.
- Compile-and-one-iteration smoke runs fail in pull-request CI.
- Elapsed-time and allocation regressions are advisory on shared CI.
- A scheduled or manually triggered stable-host job may enforce reviewed per-case
  thresholds; threshold changes include the before/after baseline and an
  explanation of the intended algorithmic change.

## Contract — fuzz workspace and target properties

Pin the fuzz runner, its package versions, and any required nightly by exact
version/date in `fuzz/`, isolated from the workspace MSRV. Check in seed corpora
and regression inputs; never check in generated crash artifacts with duplicate or
non-minimized data. Every target enforces byte/work/depth caps independently and
obeys governing rule 7. Four targets are contracted:

- **`xt_read`** (arbitrary bytes + a compact parser/read-option selector):
  terminates within byte/work limits; no panic, abort, OOB access, or unbounded
  allocation; successful reads produce a checker-accepted store; classified errors
  have valid stable codes; and writing then reading any successfully imported
  supported model preserves the documented semantic digest. Seed with the smallest
  valid header/model, one fixture per supported record class, truncations at
  token/record boundaries, and each existing malformed-parser regression.
- **`nurbs_constructors`** (bounded encoding of degree, knots, control points,
  optional weights, query params; floats from raw bits but vector sizes capped
  before allocation): constructors either reject the descriptor or establish all
  documented knot/dimension/finiteness/weight invariants; accepted curves/surfaces
  evaluate only within explicit query domains and never panic on
  derivative/split/restriction/projection/isolation calls; equivalent repeated
  calls are bitwise deterministic; limit exhaustion stays classified and bounded.
  Keep separate curve and surface seed families.
- **`intersection_result`** (bounded points, params, overlaps, orientation flags,
  and completion/indeterminate data per result family): canonicalization is
  idempotent; output ordering is total and deterministic with no non-finite public
  values; swapping twice returns the original; swapping preserves completion
  evidence and indeterminate causes; deduplication never converts indeterminate
  evidence into complete evidence; serialization/debug formatting never controls
  equality. Start with curve/curve and surface/surface, then extend to
  curve/surface.
- **`topology_transactions`** (bounded bytecode of primitive creation and
  Euler/transaction ops using indices into prior handles; stale/out-of-range refs
  are expected inputs): after every committed and every rejected step — checked
  commits leave a checker-accepted store or a classified model rejection; a
  rejected op/commit leaves the pre-op store and committed-index digests unchanged;
  transaction state cannot become nested or remain active accidentally; handles
  never alias a different generation after deletion/reuse; and replaying the same
  bytecode yields the same journal, result classes, and final digest. Cap
  operations, live entities, topology depth, checker work, and journal/output
  bytes independently; seed with minimal successful sequences per Euler operator
  plus one failure-atomic sequence per operator.

## Contract — corpus promotion (Q7)

For each discovered failure: retain the raw crash artifact as a temporary CI
artifact; minimize with the pinned runner while preserving the same stable failure
code or violated property; assign a descriptive regression identifier (not a
hash-only name); add the minimized input to `fuzz/regressions/<target>/`; promote
it to an ordinary deterministic unit/integration test once expressible without the
fuzz runtime; and record the fixing revision, target, original/minimized sizes,
stable error or property identity, and whether the seed corpus now covers the
behavior. CI runs promoted tests on all three OSes; the fuzz smoke job may run on
Linux only.

## Contract — bounded CI jobs (Q8) and test lanes

CI provides three bounded surfaces. `benchmark-smoke` (Linux, pinned toolchain):
builds every benchmark and runs the smallest fixtures under Criterion's bounded
100 ms warm-up / 10-sample / 200 ms measurement, verifies invariants and metadata
schema, hard 10-minute timeout, uploads results without enforcing timing
thresholds. `fuzz-smoke` (Linux, exact fuzz toolchain/locked runner): deterministic
seed corpus plus fixed seed, 20 s budget / 256 KiB max input / explicit
RSS/artifact limits per target, each already-built target also under a 45 s OS
deadline, hard 10-minute timeout; crashes/timeouts upload minimized-or-raw
artifacts and fail the job. A scheduled job may use longer budgets and rotating
seeds but must stay bounded and never silently update checked-in corpora/baselines.
The Python contract surface is load-bearing: CI runs
`python -m unittest discover -s scripts/tests` and validates the excluded
`benches/`/`fuzz/` manifests and locks. Non-host oracle-certification checks
regenerate both the declared base and supplemental Boolean bundles and compare
their writer/payload identities with committed licensed-host records; they reject
false freshness and never pretend to perform `docs/oracle-loop.md` validation.

Developer lanes (`scripts/test_lanes.py`, fail-closed): `focused` (one package
target); `fast` (workspace library/binary tests plus reviewed integration smoke
targets); `standard` (all non-corpus integration targets plus tooling contracts);
`docs` (compiler-intensive workspace doctests, enforcing facade opacity and
checked-mutation compile-fail contracts); `full` (all integration targets including
the production-corpus ratchets, plus docs and tooling). The runner validates
workspace membership, package identity, the smoke inventory, and every direct
`exemplar.x_t` consumer before execution. Run `fast` before commit and `standard`
before handoff; `docs` and `full` remain mandatory in CI. Timing evidence and
consolidation policy live in `docs/projects/test-throughput.md`; elapsed time is
diagnostic, not a correctness threshold.

## Evidence

- `benches/benches/*.rs`: benchmark_contract, topology_commit, graph_build,
  graph_traversal, body_tessellation, face_tessellation, nurbs_isolation,
  curve_pair_isolation, curve_pair_solve, xt_io
- `fuzz/fuzz_targets/`: xt_read, nurbs_constructors
- `crates/ktopo/tests/benchmark_observation.rs`
- `scripts/test_lanes.py`, `scripts/benchmark_baseline.py`, `scripts/fuzz_smoke.py`,
  `scripts/package_contract.py`, `scripts/tests/`

## Open items

- `intersection_result` and `topology_transactions` fuzz targets, plus the
  `fuzz/regressions/<target>/` promotion layout, are planned, not landed. Add them
  only after the two existing targets run in CI.
- Land Q7 minimization/promotion tooling and the regression manifest so CI findings
  have one durable path into portable tests.
- Grow the Q3–Q5 tessellation, NURBS-isolation, and X_T size/class matrices only in
  response to an algorithm/adoption question or a measured coverage gap; neutral
  binary, additional size tiers, and larger redistributable corpora remain deferred.
- Allocation counters remain deferred until allocation instrumentation exists.

## Exit criteria

- Compiler/MSRV changes are explicit and exercised across the existing matrix.
- Each benchmark family has named scale ladders, verified outputs, semantic work
  counters, and comparable baseline metadata.
- All four fuzz targets have bounded inputs/work, seed corpora, and portable
  promoted regressions.
- Pull-request CI compiles and smoke-runs every harness within hard limits.
- Stable-host baselines identify algorithmic regressions without treating
  shared-runner timing noise as correctness.
- Normal workspace tests remain independent of the benchmark/fuzz runners.
