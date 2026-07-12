# F7 quality, fuzzing, and performance harnesses

Status: Q0-Q2b, Q8, and the first Q3-Q6 foundation slices implemented; the Q2a diamond row awaits a real multi-dependency descriptor

## Outcome

Make toolchain compatibility, robustness exploration, and performance trends
repeatable before modeling breadth grows. The harnesses protect observable
kernel contracts and algorithmic scale. Elapsed time is a measurement, never a
correctness oracle.

Q0 establishes one explicit Rust contract:

- `rust-toolchain.toml` pins Rust 1.93.0 with the minimal profile plus `rustfmt`
  and `clippy`;
- the workspace declares `rust-version = "1.93.0"`, inherited by every member;
  and
- the three-OS CI matrix installs that exact version instead of a floating
  channel. The explicit workflow value prevents runner-level rustup overrides
  from bypassing the repository file and must remain synchronized with it.

The pinned version and MSRV are intentionally equal today. Lowering the MSRV
requires a separate change that compiles and tests every target with the
proposed version. Raising either value requires release notes and a CI-green
toolchain commit. Do not claim compatibility with an untested compiler.

Benchmark dependencies are isolated in the excluded `benches/` package and do
not enter the kernel workspace dependency graph or root lockfile. No fuzz
dependency has been introduced.

## Governing rules

1. A benchmark verifies its result before recording a sample. Timing never
   decides whether output is correct.
2. Fixtures, operation sequences, tolerances, work limits, and seeds are named
   and versioned. Random setup is never inside a timed section.
3. Benchmark ladders change one scale dimension at a time and report semantic
   work counts next to elapsed time where the implementation exposes them.
4. CI smoke jobs prove that harnesses compile, terminate within fixed work/time
   bounds, and retain invariants. Shared CI runners do not enforce wall-clock
   regression thresholds.
5. Performance comparisons are made on a named, stable benchmark host. A
   baseline from a different host or compiler is informative, not comparable.
6. Fuzz failures retain the exact input and toolchain identity. Minimized,
   deterministic failures become ordinary regression tests.
7. Fuzz targets may reject malformed input, report unsupported capability, or
   return an indeterminate result. They must not panic, abort, leak unbounded
   work, commit invalid topology, or claim unsupported completion evidence.
8. Benchmark-only observability must not become a stable public kernel API.

## Repository layout

Land the following structure incrementally:

```text
benches/
  Cargo.toml
  Cargo.lock
  README.md
  cases.json
  benches/
    benchmark_contract.rs
  baselines/
    schema.json
    <host>/<git-revision>.json
crates/ktopo/benches/
  transaction_commit.rs
  body_tessellation.rs
crates/kgeom/benches/
  nurbs_isolation.rs
crates/kxt/benches/
  xt_io.rs
fuzz/
  Cargo.toml
  rust-toolchain.toml
  fuzz_targets/
    xt_read.rs
    nurbs_constructors.rs
    intersection_result.rs
    topology_transactions.rs
  corpus/<target>/
  regressions/<target>/
```

Use one workspace benchmark dependency and one workspace fuzzing stack rather
than per-crate frameworks. Benchmark and fuzz packages stay outside normal
workspace membership if their toolchain or dependency lifecycle would raise
the kernel's MSRV. Normal `cargo test --workspace` must remain sufficient for
all promoted regressions.

## Stage Q0 — pinned toolchain contract

Implemented by this slice.

### Acceptance

- `cargo metadata` reports Rust 1.93.0 for every workspace package.
- invoking Cargo from the repository selects Rust 1.93.0 and has `rustfmt` and
  `clippy` available;
- CI retains format, warning-denied Clippy, and debug/release tests on Linux,
  macOS, and Windows; and
- no workflow selects `stable`, `beta`, or an unpinned nightly.

## Stage Q1 — benchmark contract and runner

Implemented with Criterion 0.8.2 for established sampling/statistics and
cargo-criterion 1.1.0 for its documented machine-readable JSON-lines output.
The isolated `kernel-benchmarks` package contains shared deterministic fixture
helpers; repository tooling validates and enriches runner output without
implementing custom statistics. Fixture construction and invariant checking
remain ordinary Rust helpers.

Every benchmark case has a stable path:

```text
<subsystem>/<operation>/<fixture>/<scale>/<policy>
```

Each run emits machine-readable metadata and measurements. Required metadata:

- schema version and benchmark case path;
- repository revision and dirty-worktree flag;
- rustc/cargo versions, target triple, profile, and enabled features;
- OS, architecture, CPU model, logical core count, and available memory;
- runner version, warm-up/sample configuration, and process-affinity settings;
- fixture version, size parameters, tolerance/policy values, and deterministic
  seed; and
- result counters relevant to the operation, such as affected bodies, emitted
  triangles, candidates, parsed records, or output bytes.

Store compact reviewed baselines under `benches/baselines/`. Large raw sample
streams belong in CI artifacts, not Git. A baseline comparison must refuse to
produce a pass/fail judgement when schema, host identity, compiler, target,
profile, fixture version, or case parameters differ.

Q1 implementation:

- `benches/cases.json` is the versioned case registry and enforces the stable
  five-segment path;
- `benches/baselines/schema.json` defines the closed v1 report shape;
- `scripts/benchmark/` separates strict contract parsing, environment capture,
  report composition, and CLI orchestration;
- `scripts/benchmark_baseline.py smoke` exercises schema validation, strict
  runner parsing, synthetic report assembly, and output validation without
  timing or network access;
- the checked-in report and runner stream are unmistakably synthetic and
  comparison-ineligible; and
- Q1 comparison reports identity compatibility only. Timing ratios,
  thresholds, and performance pass/fail policy remain outside this stage.

### Regression policy

- correctness/invariant failures fail everywhere;
- compile-and-one-iteration smoke runs fail in pull-request CI;
- elapsed-time and allocation regressions are advisory on shared CI;
- a scheduled or manually triggered stable-host job may enforce reviewed
  per-case thresholds; and
- threshold changes include the before/after baseline and an explanation of
  the intended algorithmic change.

## Stage Q2 — topology commit and index-refresh ladder

Owner: `ktopo`.

Implemented as 21 registered cases in the isolated `benches/` package. The
`benchmark-internals` feature adds only read-only commit counters, stable
ordinal-based digests, store/index snapshots, and a full-rebuild audit; it is
absent from normal `ktopo` builds. Fixture cloning, snapshots, and invariant
verification are excluded from measured duration; the full-rebuild cases time
the independent rebuild itself over one verified prepared fixture. Allocation
counts remain deferred until allocation instrumentation exists.

Measure checked transaction cost through public transaction operations. Since
Criterion lives in the isolated external package, a doc-hidden public seam may
exist only behind `benchmark-internals` for counters that cannot otherwise be
observed. It must not expose mutable index representation and is absent without
the feature.

Fixtures:

- `isolated_acorns`: many independent minimal bodies;
- `primitive_mix`: deterministic box/cylinder/cone/sphere/torus repetitions;
- `shared_geometry_fanout`: bodies that legally depend on the same immutable
  geometry where the model supports it; and
- `rejected_edit`: one deterministic mutation that fails checked commit.

Ladders:

| Case | Scale | Timed work | Required checks/counters |
| --- | --- | --- | --- |
| clean checked commit | 1, 10, 100, 1,000 bodies | begin plus no-op checked commit | body count unchanged; affected-body count |
| local refresh | same store sizes | mutate one body's point/tolerance and commit | exactly expected body scope checked; index entries unchanged outside scope |
| fanout refresh | 1, 10, 100 dependent bodies | mutate one referenced geometry and commit | deterministic affected-body set and order |
| batched refresh | 1, 10, 100 edited bodies | perform deterministic edits in one transaction | one atomic commit; refreshed-body count |
| rejected commit | 1, 10, 100 bodies | commit one invalid mutation | identical pre/post store digest and index digest |
| full rebuild reference | 1, 10, 100, 1,000 bodies | explicitly rebuild/audit index through crate-private seam | rebuilt index equals committed incremental index |

Record entity counts, affected/refreshed body counts, checker obligations, and
allocation counts when allocation instrumentation becomes available. The
rejected-edit case protects failure atomicity, not merely throughput.

## Stage Q2a — geometry graph construction and reverse-dependency ladder

Owner: `kgraph`, with `ktopo` integration cases.

Status: implemented as 17 registered, CI-compiled cases with one bounded smoke
case, and used to land the reverse-index replacement. The diamond row is deliberately deferred because every current
procedural descriptor reports at most one dependency; a benchmark-only fake
descriptor would not measure the production graph.

Capture the current deterministic implementation as the baseline before
optimizing it. Measure graph construction and dependency maintenance separately
from geometry evaluation with these one-dimension-at-a-time ladders:

| Case | Scale | Timed work | Required checks/counters |
| --- | --- | --- | --- |
| independent nodes | 1, 10, 100, 1,000, 10,000 nodes | insert leaf nodes | node count, stable order, graph digest |
| dependency chain | depth 1, 10, 100, 1,000 | insert dependency-first chain | accepted nodes, dependency visits, reverse-index digest |
| shared fanout | 1, 10, 100, 1,000 dependents | insert offsets sharing one basis | exact dependent set and deterministic order |
| diamond graph | deferred until a real descriptor has two or more dependencies | construct shared dependency diamonds | deduplicated traversal and graph digest |
| transactional rollback | same representative scales | insert then reject/rollback a dependent subgraph | identical pre/post graph and reverse-index digests |

Record nodes, dependency edges, reverse-index updates, and full-order rebuilds
when those counters can be exposed behind `benchmark-internals`. Fixture
construction outside the operation under test is excluded from timing. Any
replacement—such as slot-indexed adjacency—must preserve insertion-ordered
determinism, rollback behavior, stale-handle checks, and full-reconstruction
audit equality before performance evidence can justify it.

The implemented target records actual node/edge registrations and full-order
rebuilds through a doc-hidden, read-only `kgraph/benchmark-internals`
observation snapshot; it exposes no mutable index representation. Stable graph
and reverse-index digests, exact dependent order, graph validation, and
rollback pre/post equality are checked outside every accumulated duration.
Geometry evaluation is not called by this target. The registered scale ladders
are complete for independent nodes, chains, fanout, and rollback. The
replacement uses insertion-ordered adjacency vectors with hash-backed key and
membership lookup that is never iterated for observable output. All 17 rows
preserve graph/reverse-index digests and now pin zero full-order rebuilds.
Removed entry slots are reused deterministically instead of accumulating
tombstones.

## Stage Q2b — geometry graph traversal ladder

Owner: `kgraph`.

Status: implemented as eight registered and CI-smoked cases.

The prepared offset-chain fixture measures dependency-first closure and a
deterministic missing dependency-path search at 1, 10, 100, and 1,000 edges.
Construction, graph validation, ordinal indexing, and repeated-result checking
are excluded from timing. Every iteration verifies exact result presence,
returned node count, stable node-order digest, and repeatability.

The ladder protects the traversal scratch replacement: ordered vectors remain
the sole source of closure results and cycle paths, while hash maps/sets provide
active and completed membership without being iterated for observable output.
`EvalContext` uses the same active-position pattern and retains exact node-visit,
dependency-depth, and cycle-path evidence. A local advisory smoke measured the
1,000-edge closure at roughly 120–123 µs; elapsed time becomes comparable only
after it is captured in a named stable-host baseline.

## Stage Q3 — body tessellation ladder

Owner: `ktopo`, consuming `kgeom` tessellation.

Status: the closed-solid ladder uses the contextual v2 contract across twenty
registered cases: ten analytic block, cylinder, cone, sphere, and torus rows;
two mixed-store target-cylinder rows; four certified imported NURBS-face and
tolerant-edge rows at chord tolerances `1e-2` and `1e-3`; and a four-point
certified imported-cylinder ladder at `1e-2`, `3e-3`, `1e-3`, and `3e-4`.
Each case uses a Serial, compatibility-v1 session and measures only
`tessellate_body_with_context`; primitive construction or X_T import,
session/context construction, outcome unpacking, and verification remain
outside timing. Repetition proves both the mesh and complete operation report
are identical. Existing analytic mesh counts, bits, ownership order, volume
evidence, and mesh digests remain pinned unchanged.

The `q3-usage.v1` evidence records all 21 canonical report stages in profile
order, not output-size proxies: five surface-projection stages, five face-
tessellation stages, graph dependency depth and node visits, then nine body-
tessellation stages from edge depth through structural items. Each case pins
the ordered consumed values, stage count, and a portable digest over the
profile/policy identity plus stage, resource, accounting mode, and consumption.
The active platform's allowances are asserted against the profile at runtime
but excluded from the checked-in digest because graph node visits uses
`usize::MAX`. Policy/API/execution identity and zero limit, numeric-stop,
diagnostic, and dropped-diagnostic counts are also explicit.

The benchmark-owned imported fixture is the exact 6,488-byte
`solid_block_nurbs_face.x_t` output accepted by the licensed Onshape host on
2026-07-11. Setup asserts its portable byte digest, one B-surface, and zero
pcurves; the Python contract checks its SHA-256 against the oracle
certification. Its legacy exact fins therefore exercise the legitimate NURBS
projection fallback: candidates, Newton depth, queries, and samples are now
nonzero without changing the 21-stage evidence contract. Projection
backtracking remains zero; the sphere/torus cases also have zero graph use.
The certified tolerant-edge fixture adds one curve-less tolerant edge, two
trimmed SP-curve/NURBS-pcurve uses, and their graph queries while correctly
leaving projection at zero. Its four skipped geometric-owner records are
explicitly pinned rather than silently ignored. The mixed-store rows prepare a
block, the target cylinder, and a sphere in that order, proving the shifted
target identity has exactly the standalone cylinder's normalized mesh and
21-stage report at both tolerances. The 2,309-byte certified cylinder adds the
first broader imported curved solid: setup pins its SHA-256, one analytic
cylinder face, three total faces, and two vertex-less ring edges. Its coarse
`1e-2` chord request is large relative to radius `0.13`. Across the four-point
ladder, reviewed volume ratios are approximately `0.94968`, `0.98487`,
`0.99436`, and `0.99857`; explicit `0.94`, `0.98`, `0.99`, and `0.998` lower
floors admit each row without pretending coarse output has fine-row accuracy.
Face-refinement passes progress `3 → 4 → 5 → 6`, edge depth `2 → 2 → 3 → 4`,
and target vertices `202 → 540 → 2,320 → 12,248`, so finite presets must not
assume smooth scaling between tolerance tiers. Genuinely curved NURBS, more
imported representations, and four-point ladders across additional
representations remain required before corpus-backed finite body `bounded_v1`
allowances are proposed.

Face-profile evidence is owned by a separate standalone ladder. Whole-body
tessellation pre-refines each shared edge against every adjacent surface with a
safety margin and passes the resulting frozen UV boundary into `kgeom`; a
nested face-boundary insertion would be a crack-prevention error, not useful
body-corpus coverage. The standalone ladder therefore measures a half-cylinder
trim through `tessellate_with_context` at `1e-2` and `1e-3`. Both rows activate
all five canonical face stages and pin boundary depth/splits, interior passes,
mesh triangles/vertices, mesh bits, and complete repeatable reports. Cylinder
and trim construction plus session/context setup stay outside timing. Broader
plane/NURBS, hole, and multi-loop rows remain before selecting finite face caps.

Fixtures use existing deterministic primitive constructors first, followed by
trimmed NURBS fixtures promoted from the test corpus:

- box, cylinder, cone, sphere, and torus;
- a mixed multi-body store (landed for a block/cylinder/sphere store with the
  cylinder selected as the operation target);
- a periodic seam/pole case;
- a face with a NURBS pcurve; and
- a bounded trimmed NURBS patch with one and multiple loops.

For each fixture, run chord tolerances `1e-2`, `3e-3`, `1e-3`, and `3e-4`
where deterministic work bounds allow; the certified cylinder now covers all
four. Record source entity counts, boundary
samples, vertices, triangles, refinement steps, and output digest. Before a
sample is accepted, verify:

- tessellation succeeds within configured limits;
- the mesh is watertight when the fixture promises a closed body;
- indices and coordinates are finite and in range;
- orientation/volume expectations hold; and
- repeated runs produce the same output digest.

The ladder should reveal output-sensitive scaling. Do not compare different
tolerances as though they perform the same amount of work.

## Stage Q4 — NURBS isolation ladder

Owner: `kgeom`.

Status: the first contextual implicit-isolation slice is implemented as six
registered cases. It varies polynomial/rational representation, one/four
Bezier patches, retained/separated implicit geometry, and the exact work and
candidate-cover budget boundaries. Limited results must remain indeterminate,
retain a deterministic conservative cover of the corresponding complete
result, and never become a complete miss. Only contextual isolation is timed;
surface/BVH/policy setup and verification are excluded. The broader degree,
control-net, knot-span/patch-count, and deeper subdivision matrix remains
deferred.

Exercise curve and surface subdivision/isolation independently from full
intersection dispatch. Use generated deterministic fixtures whose control
points and knots are checked into helpers, not sampled during the benchmark.

Dimensions, varied one at a time:

- degree: 2, 3, 5;
- control points per direction: 4, 8, 16, 32;
- knot-span/patch count: 1, 4, 16, 64;
- polynomial versus rational weights;
- separated, tangent, and clustered candidate geometry; and
- candidate budgets around the expected completion boundary.

Record extracted patches, BVH nodes/pairs, subdivisions, retained candidates,
proof completion, and consumed/allowed work. Verify candidate covers against
the existing certified classification contract. Budget exhaustion must retain
the expected indeterminate reason and must never be benchmarked as a complete
miss.

## Stage Q5 — X_T I/O ladder

Owner: `kxt`.

Status: the first bounded slice is implemented as eight cases over the
Apache-2.0 hand-authored block and offset-plane text fixtures. It separately
times public parse-to-records, complete import/reconstruction, combined writer
validation/planning/text emission, and read-write-read round trip. The writer
phase includes the mandatory body check performed by `export_text`. The current
public APIs do not isolate lexer-only, writer-check-only, writer-planning-only,
serialization-only, or byte-sink phases, so this slice makes no claims for
them. Neutral binary, additional size tiers, and larger redistributable corpora
remain deferred.

Use only redistributable, versioned corpus fixtures. Define `tiny`, `small`,
`medium`, and `large` by both byte count and record/entity count; never infer
scale solely from filenames.

Benchmark separately:

- lexical parse to records;
- record validation/reconstruction to a checked store;
- complete read;
- deterministic text planning/emission;
- write to a byte sink; and
- read-write-read round trip.

Record input/output bytes, record counts by class, topology/geometry entity
counts, unsupported capabilities, and store/output digests. Verify supported
fixtures reconstruct checker-clean stores, writer output is bitwise stable,
and round trips retain the documented semantic equivalence. Invalid fixtures
belong in parser fuzz/regression suites, not throughput baselines unless a
specific rejection path is named.

## Stage Q6 — fuzz workspace and target contracts

Status: `xt_read` and `nurbs_constructors` foundations are implemented in an
isolated pinned workspace. Direct capped targets check stable errors, import
atomicity, constructor invariants, and deterministic bounded queries across
seven X_T and nine curve/surface polynomial/rational seed cases. The constructor
contract also closed a production curve non-finite-control-point gap. Stable
host gates pass; exact pinned-nightly 20-second smokes, semantic X_T write/read
digest property, broader corpora, and remaining targets are deferred.

Pin the fuzz runner, its package versions, and any required nightly by exact
version/date in `fuzz/`. That toolchain is isolated from the workspace MSRV.
Check in seed corpora and regression inputs; do not check in generated crash
artifacts containing duplicate or non-minimized data.

### `xt_read`

Input: arbitrary bytes plus a compact selector for parser/read options.

Properties:

- termination within the target's byte and work limits;
- no panic, abort, out-of-bounds access, or unbounded allocation;
- successful reads produce a store accepted by the appropriate checker
  contract;
- classified errors have valid stable codes; and
- writing then reading any successfully imported supported model preserves the
  documented semantic digest.

Seed with the smallest valid header/model, one fixture per supported record
class, truncations at token/record boundaries, and each existing malformed
parser regression.

### `nurbs_constructors`

Input: a bounded structured encoding of degree, knots, control points, optional
weights, and query parameters. Decode floats from raw bits but cap vector sizes
before allocation.

Properties:

- constructors either reject the descriptor or establish all documented knot,
  dimension, finiteness, and weight invariants;
- accepted curves/surfaces evaluate only within explicit query domains and do
  not panic on derivative, split, restriction, projection, or isolation calls;
- equivalent repeated calls are bitwise deterministic; and
- limit exhaustion remains classified and bounded.

Keep separate curve and surface seed families even if one target dispatches
between them.

### `intersection_result`

Input: bounded points, parameters, overlaps, orientation flags, and structured
completion/indeterminate data for each result family.

Properties:

- canonicalization is idempotent;
- output ordering is total and deterministic, with no non-finite public values;
- swapping twice returns the original canonical result;
- swapping preserves completion evidence and indeterminate causes;
- deduplication does not convert indeterminate evidence into complete evidence;
  and
- serialization/debug formatting, if exercised, never controls equality.

Start with curve/curve and surface/surface results, then extend the same common
property helpers to curve/surface.

### `topology_transactions`

Input: a bounded bytecode of primitive creation and Euler/transaction
operations using indices into previously created handles. Stale and
out-of-range references are expected inputs, not harness bugs.

Properties checked after every committed step and after every rejected step:

- checked commits leave a checker-accepted store or return a classified model
  rejection;
- a rejected operation/commit leaves the pre-operation store and committed
  index digests unchanged;
- transaction state cannot become nested or remain active accidentally;
- handles never alias a different generation after deletion/reuse; and
- replaying the same bytecode yields the same journal, result classes, and
  final digest.

Cap operations, live entities, topology depth, checker work, and journal/output
bytes independently. Seed with minimal successful sequences for each Euler
operator plus one failure-atomic sequence per operator.

## Stage Q7 — corpus minimization and promotion

For each discovered failure:

1. retain the raw crash artifact as a temporary CI artifact;
2. minimize with the pinned runner while preserving the same stable failure
   code or violated property;
3. assign a descriptive regression identifier rather than a hash-only name;
4. add the minimized input to `fuzz/regressions/<target>/`;
5. promote it to an ordinary deterministic unit/integration test when it can
   be expressed without the fuzz runtime; and
6. record the fixing revision, target, original/minimized sizes, stable error or
   property identity, and whether the seed corpus now covers the behavior.

CI runs promoted tests on all three operating systems. The fuzz smoke job may
run on Linux only because its purpose is bounded exploration, while the
regression remains portable.

## Stage Q8 — bounded CI jobs

Status: implemented. Existing benchmark targets and the two current fuzz
targets run in bounded Linux jobs; more matrices and targets remain behind the
post-Q8 landing order.

CI has three bounded jobs/surfaces for the existing Q1 and Q6 assets. The
Python contract surface is load-bearing: CI runs
`python -m unittest discover -s scripts/tests` and validates the excluded
`benches/` and `fuzz/` manifests/locks rather than assuming root-workspace CI
covers them.

`benchmark-smoke`:

- pinned workspace toolchain;
- Linux only;
- build every benchmark and run the smallest fixture with Criterion's bounded
  100 ms warm-up, 10-sample, 200 ms measurement smoke;
- verify invariants and metadata schema;
- hard job timeout of 10 minutes; and
- upload results for inspection without enforcing timing thresholds.

`fuzz-smoke`:

- exact fuzz toolchain and locked fuzz runner;
- Linux only;
- deterministic seed corpus plus a fixed seed;
- each target gets a 20-second exploration budget, 256 KiB maximum input, and
  explicit RSS/artifact limits;
- each already-built target also runs beneath a 45-second OS deadline so a
  runner-level timeout defect cannot consume the job or skip its sibling;
- hard job timeout of 10 minutes; and
- crashes/timeouts upload minimized-or-raw artifacts and fail the job.

A scheduled job may use longer budgets, rotating deterministic seeds, and a
corpus cache. It must remain bounded and must not silently update checked-in
corpora or baselines.

CI includes a non-host oracle-certification status check in the tooling
contracts. It compares the current writer/bundle identity with the last
committed licensed-host identity. CI must reject a falsely “current” status, report an explicitly
acknowledged stale status prominently, and reserve a failing stale-evidence
gate for writer-conformance/release claims. It does not pretend to perform the
licensed-host validation described in `docs/oracle-loop.md`.

## Revised landing sequence from the current state

1. **Q7:** land minimization/promotion tooling and the regression manifest so
   CI findings have one durable path into portable tests.
2. **Q6 expansion:** add result-canonicalization and transaction/Euler targets
   only after the two existing targets run in CI.
3. **Q3-Q5 expansion:** grow tessellation, NURBS isolation, and X_T size/class
   matrices only in response to an algorithm/adoption question or measured
   coverage gap.

Q0-Q2b, Q8, and the current Q3-Q6 foundation slices are completed milestones,
apart from Q2a's explicitly descriptor-blocked diamond row. Additional fuzz
targets, benchmark families, and broad corpus expansion remain evidence-driven.

## Exit criteria

- compiler/MSRV changes are explicit and exercised across the existing matrix;
- each benchmark family has named scale ladders, verified outputs, semantic
  work counters, and comparable baseline metadata;
- all four initial fuzz targets have bounded inputs/work, seed corpora, and
  portable promoted regressions;
- pull-request CI compiles and smoke-runs every harness within hard limits;
- stable-host baselines can identify algorithmic regressions without treating
  shared-runner timing noise as correctness; and
- normal workspace tests remain independent of the benchmark/fuzz runners.
