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
| F1 | G1-G4a plus the F2 graph-budget adapter are implemented: `kgraph` and `ktopo::Store` own one transactional geometry graph; exact offsets evaluate through accepted/attempted node/depth accounting, check, tessellate, and round-trip through the declared X_T subset without basis duplication. The 14-file writer bundle is host-recertified and machine-fingerprinted; broader G4 corpus coverage and G5 remain. |
| F2 | Stage 1, Stage 1b composition, two Stage 2 pilots, three bounded Stage 3 pilots, and the first Stage 4 migration are implemented. `OperationContext` now owns family-default < session < request budget precedence for graph evaluation, Full checking, and face tessellation, including canonical root stops and accounting-mode validation. Scoped NURBS proof/march and NURBS/NURBS Newton conditioning/progress/gradient stationarity retain compatibility, scale, and swap evidence. Minimizer/contact-classification/legacy-slack guards, contextual projection, body tessellation, and broader migrations remain. |
| F3 | Two slices implemented: centralized class dispatch plus shared periodic/range and first-wins candidate emission migrated through line/circle and plane/sphere. Broader driver migration and generic fallback remain. |
| F4 | Phase 1, representative Phase 2 slices, and two Phase 3 pilots are implemented: graph evaluation owns stable classification, and one surface-intersection family retains ordered structured incomplete evidence through limits, numeric stops, canonicalization, and swapping. Broader result-family and legacy migrations remain. |
| F5 | K1-K3, typed K4 interchange, and K5 adoption are implemented: the `kernel` facade owns lifecycle, opaque IDs, classified sources, one-scope outcomes, safe checker subjects, opaque journals, child-accounted procedural evaluation, and atomic typed X_T import/export. The standalone `kernel-lifecycle` client depends directly only on `kernel` and proves construction, semantic inspection, Full checking, surface evaluation, and byte-stable X_T export/import/re-export. Graph-aware intersection and semantic edit/journal iteration remain. |
| F6 | First slice implemented: shared surface inversion, chart normalization, and distance services consumed by checker and tessellation. Module splits remain. |
| F7 | Q0-Q2, Q8, and the first Q3-Q6 slices are implemented: CI now enforces Python/oracle freshness, compiles and smoke-runs the excluded benchmark package, and runs both pinned fuzz targets within fixed limits. The offset-writer change that root CI missed is captured as the first Q8 registry regression. Q2a, Q3/Q4/Q5 expansion, more Q6 targets/corpora, and Q7 remain. |

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

1. **Finish F2 numerical convergence.** Land the bounded NURBS/NURBS
   contact-classification and minimizer/local-search scale guards that gate the
   future generic intersection fallback. Stage 1b profile composition is now
   implemented and remains the mandatory rule for each later contextual
   family; no owner-local merge recipe may reappear.
2. **Contextualize and ratchet the remaining foundation paths.** Migrate body
   tessellation,
   projection, and X_T reconstruction's nested graph evaluation onto shared
   scopes and child reservations before expanding their facade APIs. As each
   contextual path proves legacy equivalence, forbid new crate-internal calls
   to its legacy wrapper. K5 has now exercised the public compatibility
   surface, so proven legacy wrappers may be deprecated when their owning
   migration lands.
3. **Measure graph construction before large imports.** Land the Q2a
   graph-build/reverse-dependency ladder before production-scale imports or a
   reverse-index representation change. Optimize the current deterministic
   linear index only against that measured baseline.
4. **Resume algorithm/API expansion behind those gates.** F3's generic fallback
   and facade graph-aware intersection follow the F2 scale/context work;
   semantic K4 edit transactions follow the K5 adoption pass. F6 splits and F4
   legacy cleanup land only with an owner-level behavioral migration.

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
facade against a consumer; F2 profile/scale convergence gates F3; contextual
X_T/body-tessellation work gates the corresponding facade operations; and the
Q2a graph-build baseline gates large imports and reverse-index optimization.

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
- Large import claims wait for a graph-construction baseline; representation
  optimization waits for measurements and preserves deterministic ordering.

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

**Current convergence gate:** move operation-family default/session/request
composition into one `kcore` contract, remove owner-local overlay recipes, and
finish the scale-sensitive contact/minimizer guards required by the generic
intersection fallback. Contextual X_T graph evaluation, projection, and body
tessellation must use the same scope/child-reservation model.

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
