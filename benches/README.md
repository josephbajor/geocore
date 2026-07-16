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
The manifest currently registers 167 cases, including the 32-row Q2 topology
matrix and the 32-row Q3 `body-tessellation.v3` matrix.

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
and unique-root certificate counts/digests separately from emitted contact
geometry, then folds all of them into its semantic output contract. Joined
candidate components collapse duplicate boundary leaves to one proof region;
polynomial and rational transverse cases plus separated two-root cases are
complete only when every component has a certificate and verified
representative, while tangency remains indeterminate. Exact and sampled
overlap controls separately pin complete representation proof versus
provisional tolerance containment. The v18 solve fixture also pins a
clipped common-refinement overlap, exact overlap-equivalence Work/Items
accounting, its `N-1` work crossing, and an ordered range/orientation digest
whose endpoints are re-evaluated against both source curves. Version 18 also
pins source-range root-certificate digests so rounded restricted controls
cannot substitute for original-curve proof evidence.
The curve-pair isolation ladder separately pins unique-root certificate counts
and ordered certificate digests. Its positive-weight rational case now proves
both retained cells through interval quotient-rule derivative bounds, while
the tilted-plane case pins exact affine coplanarity and injective projection.
The diagonal-separation case pins Euclidean interval exclusion beyond the
axis-wise inflated-box test. Resource-limited controls remain zero-certificate
cases.

The Q1 target verifies the result digest before measurement and again in every
timed iteration. The Q2 target provides 32 checked-commit, incremental
index-refresh, rejection, and full-rebuild cases in the quality contract. It
times only the transaction edit and ordinary `commit_checked`, except that the
full-rebuild ladder times the independent reference-index rebuild itself.
Fixture cloning, store/index snapshots, rollback probes, and correctness checks
run outside the accumulated duration. Read-only full-rebuild samples reuse one
verified prepared fixture so Criterion calibration does not repeat excluded
cloning and snapshot work. Seven v2 mixed-store cohort rows add a two-axis
affected-root scaling matrix. One axis holds four shared-point dependents while
total bodies grow through 4, 16, 64, and 256; the other holds 64 total bodies
while dependents grow through 1, 4, 16, and 64. Unaffected bodies cycle through
the closed block, cylinder, cone, sphere, and torus fixtures. Every row pins one
mutation and exact equality of affected, refreshed, and checked body counts to
the dependent cohort, plus stable affected-order and full-output digests.
These counters, not elapsed time, establish affected-root scope. The timed
ordinary commit still includes full geometry-graph validation, cloning the
committed index, and refreshing its store body order, so this baseline does not
claim total-store-independent end-to-end latency. The affected cohort remains
the minimal one-vertex body needed to isolate dependency-index scope.

Four additional rows hold `primitive_mix` at 64 production solids and sweep
1, 4, 16, and 64 affected roots. One ordinary checked transaction grows the
first face of each selected solid to `2e-8` under an exact `N × 1e-8`
operation-owned budget. Every row pins exactly N modified Face mutations and
ordered tolerance events, affected/refreshed/checked/mutation equality, stable
affected-order and full-output digests, and committed-index equality. Elapsed
time remains advisory; global ordinary-commit cost, broader footprint scaling,
and production-assembly behavior remain open. Set
`KERNEL_BENCH_SMOKE=1` and pass one full case path after `--` for a bounded
local smoke run.

The Q2a v2 target registers 21 graph-construction cases: independent plane nodes
at 1 through 10,000 nodes, offset dependency chains and shared-basis fanout at
1 through 1,000 edges, verified intersection diamonds at 1 through 1,000 merge
descriptors, and rejected transient chains with exact undo rollback at 1
through 1,000 nodes. Diamond branches are two equal offset planes sharing one
basis; the certified merge curve retains both source and pcurve dependencies,
and dependency-first closure pins that the shared basis is visited once.
Descriptor policy, certificates, and rollback control state are
prepared outside timing. Only graph insertion, dependency maintenance, and
rollback are timed; evaluation is deliberately absent. Every sample verifies
node/edge counts, actual reverse-index registrations, zero complete-order
rebuilds, stable iteration order, graph and reverse-index digests, validation,
deduplicated diamond traversal, and rollback equality. The indexed replacement
retains insertion-ordered adjacency and uses hash storage only for
non-observable lookup/membership.

The Q2b v2 target registers ten prepared traversal cases. Dependency-
first closure and a deterministic missing-path search each run at 1, 10, 100,
and 1,000 chain edges; a production verified-intersection diamond adds both
closure and missing-path rows. Its two equal offset branches share one basis,
so the six-node closure proves timed traversal deduplicates that basis. Graph
construction, validation, certificate minting, and stable ordinal indexing stay
outside timing; every sample pins result presence, exact returned node count and
order digest, and repeatability. Traversal output and cycle paths remain
vector-ordered, while active/completed membership uses hash lookup that is never
iterated. The smallest chain closure row runs in CI.

The Q3 `body-tessellation.v3` contract registers 32 cases: the twenty existing
closed-solid rows plus four tiers each for a locally verified genuinely curved
NURBS block, a historically host-certified plane sheet, and a historically
host-certified full-period cylinder sheet. The complete matrix has 24 solids
and eight sheets. Its existing rows comprise ten analytic block, cylinder,
cone, sphere, and torus rows; two mixed-store target-cylinder rows; four
certified imported NURBS-face and tolerant-edge rows at `1e-2` and `1e-3`;
and a four-point certified imported-cylinder ladder at `1e-2`, `3e-3`,
`1e-3`, and `3e-4`. Cylinder, sphere, and torus exercise periodic seam/pole
assembly. The legacy NURBS fixture is an
exact benchmark-owned copy of `solid_block_nurbs_face.x_t`; setup asserts its
6,488-byte identity, portable digest, one B-surface, and absence of pcurves.
The tolerant fixture references the current certified oracle outbox and asserts
its 7,172-byte identity, portable digest, one curve-less tolerant edge, two
NURBS pcurve uses, and four intentionally skipped geometric-owner records. The
Python contract checks all three imported SHA-256 values against
`docs/oracle-certification.json`. One immutable fixture and one Serial
compatibility-v1 operation context are prepared per case. Import, context
construction, outcome unpacking, finite/range, exact directed incidence,
topological-boundary, face-sense orientation, measure, and exact mesh/report
repeatability checks run outside timing. Solids prove a closed two-fin
incidence and signed volume; sheets prove their one-fin boundary, exclude
two-fin seams from it, and measure faceted area. Only the
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
queries. The 12 new representation rows close the named body evidence gate.
The curved block runs at `1e-2`, `3e-3`, `1e-3`, and `5e-4`; `3e-4` is
deliberately rejected because it consumes 25 face-refinement passes against
compatibility v1's limit of 24. The two sheets run at `1e-2`, `3e-3`, `1e-3`,
and `3e-4`. The curved payload is locally import-verified, while the expanded
15-file licensed-host certification remains pending and the historical
14-file record is correctly stale. The reviewed finite presets now use the
next power of two at or above twice each measured nonzero maximum, preserve a
measured zero as zero, and retain existing smaller algorithm ceilings. The
compatibility benchmarks still record `v1_defaults`; separate matrix tests run
every row under `bounded_v1` and pin the actual root-work crossings at
`222/221` for faces and `2,822/2,821` for bodies. The
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

The full-period cylinder sheet is recognized only after its single trim loop
is proven to be the complete rectangular parameter window. Tessellation then
uses four quarter-period patches with shared iso columns, preserves the frozen
topological ring boundaries, and excludes the two-fin seam from the sheet
boundary. All four tiers meet their reviewed faceted-area bands without a
special area exception; non-rectangular cases retain the generic path.

The standalone Q3 face matrix measures plane, analytic half-cylinder, and exact
rational-quadratic NURBS representations across outer-only, one-hole, and
three-hole trims at chord tolerances `1e-2` and `1e-3` through
`tessellate_with_context`. The 18 rows pin every matrix combination. The NURBS
fixture is an exact quarter-cylinder chart whose non-uniform weights,
non-coplanar control net, and nonzero normal curvature prevent planar
canonicalization. This split is intentional: whole-body tessellation
pre-refines every shared edge with a safety margin and then freezes it, so a
nested face-boundary split would be a crack-prevention failure. Evidence pins
all five face stages, per-loop refined counts, source-trim and output-boundary
digests, retained trim vertices, exact surface re-evaluation, UV area,
face-surface orientation, model area, complete reports, and mesh bits. Fixture,
trim, session, context, and verification remain outside timing.

The Q4 implicit-isolation slice registers eight contextual cases. It varies
polynomial versus rational single patches, one versus four extracted Bezier
patches, retained candidates versus a certified separated miss, and exact
`N-1` work/candidate budget crossings. Fixture v3 adds a repeated/multi-span
source-rectangle Work crossing and retains the exact cubic-extrusion
adversary whose rounded children lose a real plane contact while the
source-provenanced cover remains nonempty. Surface construction, Bezier extraction,
BVH construction, operation-context setup, reports, digests, and conservative
cover verification are excluded from the measured duration. Degree and larger
control-net/patch-count ladders remain deferred.

The curve-pair Q4 slice has nine deterministic subdivision cases. It varies
polynomial/rational/tilted curves, retained contacts versus axis and
Euclidean-distance separated misses, and independent work,
candidate-high-water, and depth stops. Fixture version 4 pins source-range
position enclosures and root certificates rather than exclusion/proof evidence
transferred from rounded restricted controls. Its cubic/line case retains an
exact midpoint contact that rounded child hulls would incorrectly erase.
Work evidence now includes every inspected original-source knot-span slot for
the initial and child range boxes before evaluation.
Every limited result must retain its conservative parent cover and remain
indeterminate. Curve and policy construction, operation-context setup, report
extraction, and evidence verification remain outside the measured duration.

The solve-level Q4 v18 slice adds twenty-eight contextual cases over the exact-cell-driven
NURBS/NURBS path: polynomial and rational transverse contacts, an algebraic
noncoplanar root at normalized `1/3`, a broader signed-linear-form root with no
shared coordinate scalar, a primitive magnitude-two form outside the unit
coefficient family, and magnitude-three, magnitude-four, magnitude-five, magnitude-six, magnitude-seven, magnitude-eight, magnitude-nine, magnitude-ten, magnitude-eleven, and magnitude-twelve
forms that each escape the entire previous family,
tangency, two
roots, a subdivision-proven hidden miss, exact and sampled overlaps, clipped
common-refinement overlap, checked recovery from different knot-insertion
histories, rejection of an altered history, zero seed admission, and exact
overlap Work and Items denial for both common-refinement and inverse-history
search.
It pins all five report stages and both overlap-stage resources, ordered
point/overlap/proof/output digests, endpoint and in-cell witness
re-evaluation, and exact seed/overlap limit crossings. Geometry, session,
request overrides, and evidence verification are outside timing; the public
contextual solve, including profile composition and report finalization, is
timed.
The magnitude-twelve proof enumerates 182 canonical primitive two-axis carriers
per projection plane and 6,153 canonical residuals per omitted axis while
retaining the correlated homogeneous derivative numerator.

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
