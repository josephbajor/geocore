# Kernel foundation project portfolio

Status: active implementation portfolio; convergence phase

This portfolio turns the current foundation review into bounded projects with
explicit dependencies and exit criteria. Projects should preserve the kernel's
existing determinism, failure atomicity, completion evidence, and checked
topology boundaries.

## Planning authority and handoff rule

This file is the authoritative source for **current foundation priority,
project status, and handoff order**. `docs/kernel-roadmap.md` remains
authoritative for milestone contracts, dependency rationale, and long-horizon
exit criteria, but its milestone ordering is not a second execution queue.
Project-specific design files remain authoritative for their local contracts.

At handoff, update this portfolio first. A project file may refine its own next
slice, and the milestone roadmap may record changed evidence, but neither may
silently reorder the portfolio. If evidence changes the order, revise this
section and link the reason from the affected project file in the same change.

## Progress

| Project | Current state |
| --- | --- |
| F0 | Implemented: curve/curve operand swapping preserves completion evidence and canonical order. |
| F1 | G1-G4a plus the F2 graph-budget adapter are implemented: `kgraph` and `ktopo::Store` own one transactional geometry graph; exact offsets evaluate through accepted/attempted node/depth accounting, check, tessellate, and round-trip through the declared X_T subset without basis duplication. Reverse dependencies use deterministic insertion-ordered adjacency with direct key/membership lookup and no full-order rebuilds; traversal keeps vector-defined output/path order with indexed active/completed membership. Q2a/Q2b preserve exact graph, index, traversal, rollback, and accounting evidence. The 14-file writer bundle is host-recertified and machine-fingerprinted; broader G4 corpus coverage and G5 remain. |
| F2 | Stage 1, Stage 1b composition, the bounded NURBS/NURBS Stage 3 scale gate, two Stage 2 pilots, and contextual face/body-tessellation, projection, and checker entries are implemented. `OperationContext` owns family-default < session < request budget precedence for graph evaluation, Full checking, tessellation, and projection, including canonical root stops and accounting-mode validation. Whole-body tessellation owns one scope across graph evaluation, projection fallback, refinement/storage, per-patch work, and retained output. X_T reconstruction now composes graph work with exact NURBS-edge endpoint projection accounting under one owner profile; its compatibility aggregate query allowance remains accounting-only pending broader corpus evidence. Body/standalone-face tessellation and standalone surface projection are closed to new production callers by the CI retirement ratchet. NURBS/NURBS scale guards are normalized while residuals retain sole acceptance authority. Ellipse-intersection curve-projection adoption, segment conditioning, overlap/input/dedup slack, structured numeric stops, other contextual caller adoption/ratcheting, corpus-backed bounded tessellation presets, and broader migrations remain. |
| F3 | Two slices implemented: centralized class dispatch plus shared periodic/range and first-wins candidate emission migrated through line/circle and plane/sphere. Broader driver migration and generic fallback remain. |
| F4 | Phase 1, representative Phase 2 slices, and two Phase 3 pilots are implemented: graph evaluation owns stable classification, and one surface-intersection family retains ordered structured incomplete evidence through limits, numeric stops, canonicalization, and swapping. Broader result-family and legacy migrations remain. |
| F5 | K1-K3, typed K4 interchange, and K5 adoption are implemented: the `kernel` facade owns lifecycle, opaque IDs, classified sources, one-scope outcomes, safe checker subjects, opaque journals, child-accounted procedural evaluation, and atomic typed X_T import/export. The standalone `kernel-lifecycle` client depends directly only on `kernel` and proves construction, semantic inspection, Full checking, surface evaluation, and byte-stable X_T export/import/re-export. Graph-aware intersection and semantic edit/journal iteration remain. |
| F6 | First slice implemented: shared surface inversion, chart normalization, and distance services consumed by checker and tessellation. Module splits remain. |
| F7 | Q0-Q2b, Q8, and the first Q3-Q6 slices are implemented: CI now enforces Python/oracle freshness, compiles and smoke-runs the excluded benchmark package including graph construction/traversal plus contextual body and standalone face tessellation, and runs both pinned fuzz targets within fixed limits. Q2a drove the reverse-index replacement and pins zero full-order rebuilds without graph/index digest drift; Q2b pins deterministic closure/path evidence through 1,000 edges after traversal membership indexing. Q2a's diamond row awaits a real multi-dependency descriptor. Q3's body ladder pins all 21 composed counters; certified B-surface rows activate projection work and tolerant-edge rows cover the explicit SP-curve/NURBS-pcurve path. A separate half-cylinder ladder activates all five face stages because body tessellation deliberately freezes pre-refined boundaries. Broader representation/corpus measurements still gate finite presets. Q3-Q5 expansion, more Q6 targets/corpora, and Q7 remain. |

## Current direction and handoff order

The foundation has enough vertical proof. The current phase prioritizes
convergence, adoption, and continuous enforcement over adding more parallel
surface area:

### External-evidence lane — current

The exact 14-file bundle, including `offset_plane.x_t`, is current against
Onshape. `docs/oracle-certification.json` fingerprints the certified writer
inputs and every host payload; Q8 regenerates the bundle and rejects a falsely
current record. Host findings remain ratcheted in `docs/oracle-results.tsv`.

### Ordered code queue

1. **Adopt and ratchet the completed contextual paths.** X_T reconstruction
   and checked-commit Fast validation share one facade-owned scope and
   cumulative graph allowance. Whole-body tessellation now has equivalent
   contextual and shared-scope entries, composes its projection fallbacks, and
   its remaining `ktopo`/`kxt` production clients now use one contextual
   operation per body. The enforced legacy-API source audit closes new
   production calls to the body wrapper while preserving compatibility tests.
   Standalone surface projection is now closed to new production callers;
   X_T now owns a composed graph/projection profile and charges both endpoint
   projections for each untrimmed NURBS edge. Ellipse intersection remains the
   final production curve-projector owner before its ratchet. Public
   body-tessellation deprecation still waits for an adopted facade replacement.
2. **Finish hostile-input tessellation policy.** Exact per-face split/vertex/
   triangle admission and body-wide edge/iso split, prepared-patch, and retained-
   triangle stages have landed, including physical representability checks,
   atomic rejection, deterministic diagnostics, and composition evidence.
   Prepared UV/patch copies and final nondegenerate triangles are admitted
   before their first body-owned allocation; later moves do not recharge them.
   Pre-UV edge face-use, seed, recursive-interior, retained-sample, and record
   slots plus final edge-polyline records and indices now share one exact
   `Items/Cumulative` stage, including pre-allocation arithmetic and atomic
   N/N+1 evidence. The compatibility-v1 preparation, edge-storage, structural,
   and body-triangle totals intentionally remain accounting-only at `u64::MAX`
   because no truthful finite legacy cap exists. A distinct structural-items
   stage now admits the single first-seen topology plan, deterministic
   membership scratch, `vgids`, `face_ranges`, outer loop/chain and patch-hole
   collections, `trim_loops`, and torus arc-row holders. The reviewed block total is 84, and
   closed-surface, multi-hole, atomic N/N+1, shared-scope, overflow, diagnostic,
   legacy, and execution-policy evidence has landed. Q3's contextual analytic
   ladder now records all 21 aggregate stages and preserves the reviewed mesh
   bits. Certified imported B-surface rows exercise projection candidates,
   Newton depth, queries, and samples; tolerant-edge rows cover two explicit
   NURBS pcurve uses without projection fallback. Mixed/genuinely-curved-NURBS/
   broader-import measurement remains and must precede a reviewed opt-in body
   `bounded_v1` preset. The standalone half-cylinder face ladder now measures
   every face stage at two tolerances; expand its representation/trim matrix
   independently before proposing the face preset. In the body ladder, zero
   face-boundary use is the required frozen-boundary invariant, not missing
   evidence.
   Do not describe product-facing tessellation as hostile-input bounded, use
   allocator-dependent byte counts, or silently tune the legacy v1 wrapper.
3. **Resume algorithm/API expansion behind the completed gates.** F3's generic fallback
   and facade graph-aware intersection follow the F2 scale/context work;
   semantic K4 edit transactions follow the K5 adoption pass. F6 splits and F4
   legacy cleanup land only with an owner-level behavioral migration. The Q2a/
   Q2b ladders are executable in CI; any graph-index/traversal representation change
   still requires a recorded stable-host before/after comparison.

No C ABI, plugin ABI, broad topology privacy break, speculative facade family,
or file-size-only module split is part of this convergence phase.

## Dependency outline

```text
F0 Completion-preserving result symmetry        (independent corrective fix)
F1 Procedural geometry graph                    (blocks procedural geometry)
F2 Operation context and numerical policy       (blocks generic solver growth)
F3 Intersection engine consolidation            (after F2 foundations; uses F1 types later)
F4 Kernel error and capability taxonomy         (independent, coordinate with F2/F3)
F5 Kernel facade and topology encapsulation     (after F1, F2, and F4 contracts)
F6 Shared surface services/module decomposition (independent first slice)
F7 Quality and performance harnesses             (independent and continuous)
```

The original independent foundations have landed. Work is no longer scheduled
as broad parallel expansion: Q8 made the harness protective; K5 tested the
facade against a consumer; the completed F2 profile/scale gates make bounded F3
fallback work eligible; X_T reconstruction and checked-commit Fast checking now
share one graph child in one scope. Contextual body tessellation now composes
projection and sequential graph/face work in one scope; its `ktopo`/`kxt`
production callers are contextual and its internal legacy-use ratchet is
enforced. Exact body edge-line and remaining structural-holder admission have
landed. Ellipse-intersection curve-projection adoption/ratcheting and corpus-
backed bounded tessellation presets remain.
The Q2a/Q2b ladders now protect graph construction, reverse indexing, and
dependency traversal through the current 1,000-edge procedural scale.

### Standing handoff ratchets

- Writer-reachable byte changes invalidate the affected licensed-host evidence;
  local read/write round-trip does not restore it.
- A proven contextual replacement closes the door to new crate-internal legacy
  calls even while source-compatible public wrappers remain.
- Excluded benchmark, fuzz, and Python tooling is protective only when its
  contracts run in CI.
- The facade-only lifecycle client keeps exactly `kernel` as its direct
  dependency, and the reviewed `kernel` package inventory stays enforced in
  CI.
- Large-import work exercises the graph-construction ladder; representation
  optimization includes a stable-host before/after measurement and preserves
  deterministic ordering.

## Reconciled F1/F2/F4 boundary

The geometry-graph and operation-context projects use one normative ownership
model:

- `kgeom` keeps total, context-free leaf evaluators for analytic and NURBS
  values.
- `kgraph` owns geometry handles, descriptors, dependency traversal, cycle
  detection, and a fallible per-query `EvalContext`.
- `OperationContext` owns immutable session/numerical/execution policy;
  `OperationScope` owns the top-level deterministic work ledger and ordered
  diagnostics.
- An operation scope deterministically reserves graph node-visit/depth work,
  then constructs a graph evaluator with that `EvalLimits` reservation and a
  copy of the operation's model-acceptance `Tolerances`.
- The graph evaluator owns no session policy, executor, cancellation contract,
  topology state, or operation diagnostic buffer. The operation context owns no
  graph handles, caches, cycle stack, or descriptor knowledge.
- F1 and F2 may introduce typed local evaluation/limit data. F4 standardizes
  stable capability, stage, and public error identifiers without erasing those
  distinctions or introducing graph types into `kcore`.

This contract is the integration gate for implementing either design. Changes
that create a second session/context abstraction require an explicit portfolio
revision.

## F0 — Completion-preserving result symmetry

**Purpose:** prevent operand-order normalization from weakening proof evidence.

**Scope:** add first-class curve/curve result swapping that preserves points,
overlaps, ordering, orientation, and completion; route reversed dispatch through
it; add symmetry regressions.

**Exit criteria:** complete hits and misses remain complete in either operand
order; indeterminate reasons survive swapping; all `kops` tests pass.

## F1 — Procedural geometry graph

**Purpose:** represent offset, intersection, swept, spun, and blend geometry as
exact dependent geometry without duplicating owned basis objects or introducing
topology-to-geometry dependency cycles.

**Scope:** define graph ownership and handles, serializable descriptors, a
fallible evaluation context, dependency traversal and cycle rejection, class
identity, and integration boundaries for `ktopo`, `kops`, and `kxt`. Prove the
design with the narrow offset-surface import/evaluation slice.

**Non-goals:** general caching, concurrency optimization, every procedural
class, or a public plugin ABI.

**Exit criteria:** an imported offset surface references its basis surface by
handle, evaluates position/derivatives through a typed context, rejects cycles
deterministically, remains exactly classifiable for X_T, and is consumable by a
topology face without owned surface duplication.

## F2 — Operation context and numerical policy

**Purpose:** stop model tolerances, solver conditioning thresholds, proof limits,
and fixed work caps from becoming unrelated per-module policy.

**Scope:** define the context and policy types, ownership/lifetime rules,
deterministic work accounting, structured limit diagnostics, and a staged
migration for intersections, checker proofs, construction, projection, and
tessellation.

**Non-goals:** making the Parasolid model-space regime arbitrarily configurable,
introducing nondeterministic cancellation behavior, or tuning all algorithms in
the first change.

**Exit criteria:** one representative intersection and one refinement/checking
algorithm consume explicit policy; defaults reproduce existing golden results;
limits are test-overridable and failures report stage plus consumed/allowed
work.

**Current convergence gate:** operation-family composition and the
scale-sensitive contact/minimizer gate are complete. Contextual graph
evaluation and checked commit use the same scope/child-reservation model.
Projection's standalone contextual entries have landed and body tessellation
now consumes them in one shared scope. Body production callers and its
internal-use ratchet are complete. Projection caller adoption/ratcheting,
hostile-input tessellation allocation bounds, and facade construction
composition remain.

## F3 — Intersection engine consolidation

**Purpose:** keep analytic special cases while preventing quadratic dispatch and
helper duplication from becoming the architecture.

**Scope:** introduce stable geometry-class inspection, centralized pair
normalization/swapping, shared range and periodic-parameter utilities, shared
candidate deduplication/emission, and one generic certified fallback contract.
Migrate one curve/curve family and one surface/surface family before expanding.

**Non-goals:** rewriting correct closed-form solvers or completing every NURBS
case in the same project.

**Dependencies:** F2 policy types; coordinate descriptor identity with F1.

**Exit criteria:** adding a new geometry class does not require hand-writing both
operand orders; specialized and fallback paths return the same result contract;
completion and structured limits survive dispatch transformations.

## F4 — Kernel error and capability taxonomy

**Purpose:** let callers and metrics distinguish invalid input, unsupported valid
input, incomplete proof, exhausted resources, and violated invariants without
parsing diagnostic strings.

**Scope:** define stable capability/stage identifiers, structured algorithm-limit
data, and layer-appropriate error/outcome boundaries. Migrate intersection
dispatch and one topology/checking path. Retain human-readable context.

**Exit criteria:** unsupported geometry is not `InvalidGeometry`; limit telemetry
is machine-readable; X_T wrapping retains kernel classifications; C-ABI mapping
can be defined without inspecting strings.

## F5 — Kernel facade and topology encapsulation

**Purpose:** give future application, bindings, and feature-history clients a
stable conceptual API without exposing arena layout and backlink vectors.

**Scope:** introduce a thin `Kernel` or `Session` facade, read-only entity views
and deterministic iterators, operation request/result types, and an explicitly
unstable low-level assembly boundary for interchange. Gradually privatize raw
topology fields where cross-crate construction no longer requires them.

**Dependencies:** stable first versions of F1, F2, and F4.

**Exit criteria:** ordinary clients can construct, query, mutate transactionally,
and export a body without importing raw entity structs; `kxt` still reconstructs
atomically; compile-fail tests protect raw mutation boundaries.

## F6 — Shared surface services and responsibility splits

**Purpose:** remove semantic drift before splitting large modules for size alone.

**First slice:** centralize analytic surface inversion/projection, periodic base
chart normalization, and point-to-surface distance in `kgeom`; migrate checker
and body tessellation to it.

**Later slices:** separate structural/incidence/domain/shell checking;
boundary/chart/triangulation tessellation; and X_T
planning/emission/serialization only when the corresponding contextual or
adoption work establishes a tested seam. File size alone is not a split
criterion.

**Exit criteria:** checker and tessellator share one inversion implementation and
the same class coverage; focused tests cover seams, singularities, and NURBS
projection; later moves are behavior-preserving.

## F7 — Quality, fuzzing, and performance harnesses

**Purpose:** make robustness and asymptotic expectations executable before broad
modeling operations land.

**Scope:** pin the Rust toolchain/MSRV; add benchmark ladders for checked commit,
index refresh, tessellation, NURBS isolation, and X_T I/O; add initial fuzz
targets for X_T parsing, NURBS constructors, result canonicalization, and
transaction/Euler sequences; retain minimized regressions.

**Exit criteria:** benchmarks have named fixtures and recorded baselines; fuzz
targets run locally and in bounded CI smoke jobs; toolchain changes are explicit;
no benchmark depends on wall-clock ordering for correctness.

## Integration rules

Each project must state which capability changes, whether results are complete
or indeterminate, which tolerances and work budgets apply, how failure atomicity
is verified, what journal/checker evidence is produced, and which deterministic
or performance regression protects it. Cross-project shared types should land
in small contract commits before broad migrations.

During convergence, new production code must use the F2/F4 contracts, but F4
does not run a repository-wide cleanup campaign. Remaining legacy call sites
migrate opportunistically with their owning behavior change.
