# Kernel foundation project portfolio

Status: proposed execution plan

This portfolio turns the current foundation review into bounded projects with
explicit dependencies and exit criteria. Projects should preserve the kernel's
existing determinism, failure atomicity, completion evidence, and checked
topology boundaries.

## Progress

| Project | Current state |
| --- | --- |
| F0 | Implemented: curve/curve operand swapping preserves completion evidence and canonical order. |
| F1 | G1-G4a implemented: `kgraph` and `ktopo::Store` own one transactional geometry graph; exact offsets evaluate, check, tessellate, and round-trip through the declared X_T subset without basis duplication. Broader G4 corpus coverage and G5 remain. |
| F2 | Stage 1, two Stage 2 pilots, and the first Stage 4 migration are implemented: scoped NURBS proof/march, Full face-domain checking, and face tessellation share deterministic ledgers while retaining accepted usage, attempted limit events, numeric-resolution stops, structured gaps, compatibility parity, and exact legacy precedence. Contextual projection, body tessellation, and broader migrations remain. |
| F3 | Two slices implemented: centralized class dispatch plus shared periodic/range and first-wins candidate emission migrated through line/circle and plane/sphere. Broader driver migration and generic fallback remain. |
| F4 | Phase 1, representative Phase 2 slices, and two Phase 3 pilots are implemented: graph evaluation owns stable classification, and one surface-intersection family retains ordered structured incomplete evidence through limits, numeric stops, canonicalization, and swapping. Broader result-family and legacy migrations remain. |
| F5 | K1 and the K3 geometry-identity/read-view slice are implemented: the `kernel` facade owns session/part lifecycle, opaque part-qualified topology and graph IDs, classified source-preserving errors, deterministic views, and shared offset-basis identity without descriptor leakage. K2, operation-scoped K3 calls, K4, and K5 remain. |
| F6 | First slice implemented: shared surface inversion, chart normalization, and distance services consumed by checker and tessellation. Module splits remain. |
| F7 | Q0-Q2, the first Q3 analytic slice, and the first Q4 NURBS-isolation slice are implemented: the isolated runner covers topology commits/indexes, ten exact body-tessellation cases, and six representation/patch/relation/budget isolation cases with semantic evidence and bounded smokes. Mixed/NURBS Q3 expansion, the broader Q4 matrix, and Q5-Q8 remain. |

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

F0, F1, F2, the first slice of F6, and F7 can proceed concurrently. F3 should
not perform a broad pair-file rewrite until F2 defines solver policy. F5 should
not freeze a facade until graph handles, operation context, and error outcomes
have stable shapes.

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
boundary/chart/triangulation tessellation; and X_T planning/emission/serialization.

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
