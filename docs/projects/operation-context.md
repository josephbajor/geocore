# Operation context and numerical policy

Status: Stage 1b composition, the Stage 3 NURBS contact/minimizer scale gate, and representative Stage 2–5 pilots implemented; projection and whole-body tessellation are contextually adopted with their standalone-wrapper ratchets closed and ellipse/ellipse preserves typed projection failures; broader operation-family migration (Stage 6) remains.

## Purpose

Introduce one explicit, deterministic policy boundary for kernel operations before general
intersections, procedural geometry, booleans, and healing multiply local numerical constants and
work caps. It separates five concerns that are easy to conflate: (1) the fixed Parasolid-compatible
session precision regime; (2) model-space acceptance tolerances requested by an operation;
(3) scale-aware parameter/rounding/conditioning guards used by algorithms; (4) deterministic
proof/work/memory/depth/output limits; and (5) execution controls and structured diagnostics. The
design preserves the existing bit-determinism, completion-evidence, failure-atomicity, and
tolerance-provenance contracts, and lets the kernel add policy without adding an argument to every
public function.

## Contract

### Four distinct tolerance concepts

These values remain different types/fields with different rules:

| Concept | Meaning | Caller-loosened? | May prove model acceptance? | Owner |
| --- | --- | --- | --- | --- |
| `SessionPrecision` | Linear/angular resolution and size box of the file/model regime. | No in v1. | Yes, where the spec names session resolution. | Immutable session policy. |
| `Tolerances` | Requested model-space acceptance for an operation, validated at/above session resolution. | Linear already may loosen; angular only with validated semantics. | Yes, for that operation's residual/proximity contract. | Operation context. |
| Entity tolerance | Persisted per-face/edge/vertex model allowance with provenance. | Only through checked operation rules. | Yes, for obligations on that entity. | Topology + transaction journal. |
| Numerical guard | Parameter progress, rounding slack, scaled-zero, or conditioning threshold. | Only through a validated numerical profile, kernel-owned initially. | No. May stop refinement or flag ill-conditioning; cannot certify incidence, coincidence, containment, or a topological sign. | Session numerical policy + local scale. |

`TessOptions::chord_tol`, angular faceting tolerance, and approximation bounds are requested output
quality — neither session resolution nor work limits. `Tolerances` stays source-compatible, is
documented as "model acceptance tolerances", and must not grow fields for iterations, sampling, or
conditioning. `SessionPrecision::parasolid()` is the only production v1 regime and exposes the
current linear, angular, and half-size values as data.

### Immutable session policy, fresh per-operation scope

`kcore::operation` owns `SessionPolicy` (validated, immutable, cheap to share, owning no
model/graph/topology state), a borrowed `OperationContext` snapshot (no mutable counters; shareable
for planned parallel work), and one `OperationScope` per top-level call (owning all mutable work
usage + diagnostic buffers). Nested algorithms borrow the same scope or deterministic child scopes;
they never create fresh default budgets. A context is never stored in geometry/topology entities.
There are no process-global mutable defaults; changing policy means constructing a new validated
`SessionPolicy`/context.

### Numerical policy: scale-aware and proof-ineligible

`NumericalPolicy` centralizes recipes over named documented factors times `f64::EPSILON` and
actual coefficient/window/Jacobian/accounting magnitudes — not unqualified epsilon constants;
absolute floors survive only where the quantity has a fixed documented normalization, else they
are bugs to classify. Consuming rules: a parameter-progress threshold may terminate an iteration
but the candidate still needs an independent model-space residual check before acceptance; if the
rounding floor exceeds the metric-driven step and the residual is not independently certified the
result is numerically stopped/indeterminate, never silently accepted; an ill-conditioned Jacobian
selects a safeguarded fallback/subdivision/conditioning diagnostic, never converting near-contact
into contact; exact predicates and interval-certified signs continue to decide topology and a
numeric guard never replaces them; call sites name the semantic `NumericGuardKind`, with raw
`EPSILON` multipliers limited to `kcore` policy and narrowly justified exact-arithmetic modules.
The default profile is versioned (`PolicyVersion::V1`) because changing a factor can change output
bits/completion; policy versions live in corpus/benchmark metadata, never in persisted B-rep
entities.

### Budgets: deterministic plan and ledger

Budgets use stable namespaced `StageId` constants defined by the owning crate (e.g.
`kops.intersect.ssi-proof-candidates`) plus generic resource accounting
(`ResourceKind::{Work,Items,Bytes,Depth}`, `AccountingMode::{Cumulative,HighWater}`, `LimitSpec`,
`LimitSnapshot`). Prose messages attach separately and may change; identifiers may not. Higher-layer
crates define typed default profile constructors returning a `BudgetPlan`, avoiding a `kcore`→higher
dependency. A "work" unit is not a time unit (wall-clock deadlines are excluded so explored geometry
stays machine/schedule-independent).

The ledger supports `charge(stage, amount)` for cumulative work, `observe(stage, value)` for
high-water resources, deterministic child reservation by stable work-item ordinal, a root
total-work ceiling in addition to per-stage limits, and accepted-usage plus first-crossing/
numeric-resolution evidence on success and failure. With a root ceiling, every child reserves root
as well as stage capacity; a child plan omitting an explicit root ceiling gets the checked sum of
its cumulative Work allowances (a stricter explicit child ceiling is preserved) so a valid child
stays mergeable. Strictly sequential nested algorithms use `SequentialWorkLedger`: accepted units
stream into parent stage/root totals immediately, local limits win ties, a rejected unit mutates
neither view. Budget exhaustion follows the proof contract: return verified partial evidence with
`Completion::Indeterminate` (or the checker's verification gap) and a structured `LimitSnapshot`,
else `Error::AlgorithmLimit` during the compatibility period with the structured limit in the
report. Never discard candidates to fit a budget and report `Complete`; never turn a work limit
into `InvalidGeometry`. F4 may later replace `Error::AlgorithmLimit`; F2 owns the stable
stage/resource/usage/limit data.

### Reports without breaking `Result<T>`

Context-aware entry points return `OperationOutcome<T, E>` (result + `OperationReport`), preserving
diagnostics after failure. `OperationReport` carries `policy_version`, accepted `usage`,
`limit_events` (first attempted crossing per configured stage/resource), `numeric_resolution_stages`,
and `diagnostics`. The two machine-readable records are independent of diagnostic level; optional
diagnostics add bounded human context but are never the only explanation for incomplete work, and
reports are assembled only after child work is merged in deterministic ordinal order.
`OperationScope::finish` stays fixed to `kcore::Error` (inference-safe legacy `finish(Ok(value))`);
`finish_typed` builds a layer-owned-error outcome. Existing public functions remain compatibility
wrappers using exact v1 defaults and discarding only the new report; the additive `_with_context`
form's internal helpers take `&mut OperationScope` so nested work cannot reset usage.

### Diagnostics: structured, bounded, observational

Diagnostics (`DiagnosticCode`, `DiagnosticKind`, `OperationDiagnostic`) are semantic summaries, not
logging: codes/stages are stable and machine-readable; messages give human context but are not
control-flow contracts; repeats are deduplicated/capped by a documented budget; subject-specific
detail stays in the owning result type (no `kcore`-upward dependency); enabling diagnostics cannot
change branch selection, work accounting, output order, or mutation; callbacks are never invoked
from parallel workers (child buffers merge by stable ordinal first). Wall time, thread count, and
OS telemetry are non-semantic and must not enter the deterministic `OperationReport`.

### Execution policy: concurrency, never ordering

`ExecutionPolicy` supports `Serial`, `AtMost(NonZeroUsize)`, and `Available`. Work items get
stable ordinals before parallel execution; result/diagnostic/limit/numeric-resolution merging is
ordinal-ordered; floating reductions use a prescribed index order/deterministic tree, never
completion order; budget allocation is never an atomic race (planned serially or via deterministic
per-child reservation); unused reservation returns only at a deterministic join and cannot be
stolen on timing; serial and every permitted thread count produce bit-identical results,
completion, diagnostics, and usage for an uncancelled operation. External cancellation is a
read-only token checked at documented safe points: a cancelled run need not reproduce its usage
report, must return no successful partial mutation, must roll back any active transaction, and must
not expose timing-based partial geometry as complete.

### Relationship to procedural geometry `EvalContext`

F2 must not create a second graph context. Ownership nests: `SessionPolicy` → `OperationContext` →
`OperationScope`/global ledger → child reservation per geometry query → graph `EvalContext` +
`EvalLimits`. The operation layer builds an `EvalContext` from F1-owned inputs only (graph borrow, query-local
cache/cycle state, a copy of `Tolerances`, a deterministically reserved node-visit/depth allowance
as `EvalLimits`, and a stable child work ordinal). F2 maps F1's typed
`DependencyDepthExceeded`/`NodeVisitLimitExceeded` into the parent ledger/diagnostics without
changing the graph error. `EvalContext` owns no `SessionPolicy`, executor, cancellation, topology,
or diagnostic buffer; `OperationContext` knows no graph descriptors/handles/caches/cycle stacks.
Direct `kgraph` clients may use standalone `EvalLimits` + default `Tolerances`; higher-level
`ktopo`/`kops`/`kxt` clients must derive it from their active operation scope so nested evaluation
is charged to the caller. Graph-owned proofs (the plane/sphere adapter's bounded `kops` proof, the
scoped NURBS marcher, X_T intersection-chart import) compose their own budget profiles outside
`EvalContext`, charging named `kops.intersect.*` stages while graph node-visit/dependency-depth
stages account graph work; exact predicates pin every crossing independently. X_T chart import
preflights Work and observes Items/Depth before retaining position/UV arrays, charges Work only
after both whole-range lifts certify, admits only seam-safe clamped periodic proof rectangles, and
relies on the enclosing reconstruction transaction for rollback.

### Dispatch and proof authority (`kops`)

- Curve/curve, curve/surface, and surface/surface each normalize the full runtime class matrix to
  one dispatch arm per unordered pair; a reversed call swaps the canonical result, preserving
  completion and first-operand ordering with no second path. Surface/surface canonical rank is
  Plane<Cone<Cylinder<Sphere<Torus<NURBS (unlike the public enum order, to keep the cone-first
  contract). Unsupported known/custom classes return a structured `*.class-pair` capability error
  retaining both operands; typed routing never upgrades discovery to a proof (NURBS bridges stay
  `Indeterminate`).
- Exclusion/existence proofs consume only original-source interval enclosures (outward interval de
  Boor tightened by conservative positive-weight control hulls, failing open when inconclusive);
  generated/rounded controls only partition and seed, never proving exclusion, containment, or a
  topological sign. Exactly one box is outward-inflated by model tolerance, so strict separation
  suffices while boundary contact stays indeterminate; only an empty complete cover proves a miss.
- The certified fallback stays indeterminate until complete-domain exclusion is proven; witnesses
  prove only their emitted contact, not root uniqueness/complete discovery/coincident extent. Exact
  overlap equivalence is a separate pre-discovery admission stage, so distinct representations cannot
  return `Complete` with zero unbounded work. Root-existence certificates use exact predicates and
  fail closed on unsupported/tangent/multi-root/non-monotone inputs; the NURBS/NURBS coverage gap
  retires only when isolation completed and every component has both a unique-root certificate and a
  verified representative.
- Newton termination is typed and reported (parameter-resolution stops stay in always-on
  numeric-resolution evidence); accepted residual witnesses stay authoritative regardless of the
  terminal state, and tolerance only selects/clamps a representative — distinct classified roots are
  never collapsed by parameter tolerance. Analytic containment keeps metric tolerance out of
  harmonic-identity decisions (exact source signs classify identity / nonzero-constant / general).

### Layer consumption

- **`kcore`:** add `operation` types, policy validation, stable IDs, budget plans/ledgers, reports,
  deterministic child reservation, and `ExecutionPolicy`-aware parallel primitives; keep current
  constants and the `Tolerances` compatibility API; add `SessionPrecision` and scale-aware helpers.
  No geometry/topology/intersection/checker/tessellation-specific fields in `kcore`.
- **`kgeom`:** projection gains contextual fallible entry points; fixed samples/candidates/
  iterations/halvings move to named budget profiles; NURBS isolation accepts a child scope and
  `ImplicitIsolationLimits` becomes a view over common limit/numeric events; parameter/conditioning
  checks call `NumericalPolicy` with local scales.
- **`ktopo` checker:** add `check_body_report_with_context` and retain wrappers; Fast structural
  checks use fixed session precision and entity tolerance (a looser operation tolerance must not
  make an invalid body pass); Fast samples are fault-detection only, never proof; Full adaptive
  checks charge subdivisions/segments/evaluations/candidate pairs and record exhaustion as a
  structured verification gap (not a fault, not `Valid`). Loop orientation uses no sampled/magnitude
  authority — only whole-interval-certified planar straight loops with nonzero exact polygon signs
  and a containment-certified unique outer emit `WrongLoopOrientation`; other loops become per-loop
  Full `LoopOrientation` gaps, unresolved outer/hole roles a separate `LoopContainment` gap. A
  `Valid` Full result means all obligations completed within limits; raising a budget discharges
  gaps but never erases faults.
- **`ktopo::make` + checked transactions:** add contextual variants of the checked-creation driver
  first, then public constructors; one scope spans input validation, assembly, affected-body
  checking, and commit; limit/cancellation/checker failure rolls back like current checked commit;
  committed journals contain only successful changes. Transaction-owned tolerance-growth budgets
  stay separate — a context may impose a policy ceiling, but growth is still declared, charged,
  rolled back, and journaled by the transaction API. Exact primitive construction must not start
  depending on model tolerance; sub-resolution rejection follows the session/model contract.
- **`kops`:** add context-aware curve/curve, curve/surface, and surface/surface dispatch sharing the
  caller's scope; move repeated sample/bisection/polish/minimize/proof caps into named profiles;
  replace repeated parameter-tolerance helpers with scale-aware recipes while preserving independent
  model-space residual acceptance; report conditioning/fallback/exhaustion/partial evidence
  structurally; do not force analytic closed forms through iterative infrastructure.
- **Tessellation (`kgeom` + `ktopo::btess`):** `TessOptions` is the shared quality request; a body
  tessellation owns one scope with named child stages and one aggregate output/scratch budget;
  shared edges are discretized once; per-face parallelism reserves deterministic child budgets by
  face order and splices in that order; a cap hit without meeting chord/edge/angle quality is a
  limit failure — never a lower-quality mesh labeled successful.

### Compatibility and migration

Additive and staged; no repository-wide signature rewrite. Keep every existing public function and
its defaults; add `_with_context` at top-level seams only, with internal helpers taking
`&mut OperationScope`; implement legacy wrappers via contextual paths under exact v1 policy,
preserving return values, errors, and golden bits; move constants in behavior-preserving pilots
before tuning any value (default changes need their own evidence, corpus update, and policy-version
decision); do not require context on leaf evaluators or simple accessors. Once a contextual path
proves bit/result/error/report equivalence, close the old entry point to new crate-internal
production callers via Clippy `disallowed-methods` or a targeted source audit; public `#[deprecated]`
follows only after K5 proves the replacement against a real consumer; removal is a separately
announced compatibility decision.

**Legacy API retirement ratchet** — every migrated entry point moves monotonically: (1) contextual
alternative available; (2) equivalence proven (outputs, errors, completion, reports, journals,
rollback, determinism); (3) internal legacy use closed (production code cannot call the wrapper;
focused equivalence tests keep it exercised); (4) publicly deprecated (only after facade
adoption); (5) removed (only under explicit compatibility policy). The owner records the state on
migration; "opportunistic migration" never licenses new legacy callers.
`scripts/legacy_api_contract.py` audits `kernel` and lower production trees and rejects new refs.

### Rollout invariants

Rollout is staged 0–6 (audit → inert `kcore` infra → proof/refinement pilots → numerical pilot →
projection+tessellation → checker/make → broad intersection migration); each stage's exit gate is a
subset of the acceptance criteria below. Two invariants are unique to staging: owner-level profile
composition fills family defaults, then session overrides, then explicit request overrides (the only
later override), preserving the root total-work ceiling and canonical stop across all three layers
with no crate-local merge semantics; and bounded `bounded_v1` presets take the next power of two ≥
twice each measured nonzero maximum, preserve zero, and are never implicitly promoted by a later
policy version. Stage 6 starts only after Stage 1b composition and the Stage 3 scale gate.

### Non-goals

Configurable linear/angular resolution or size box in v1; replacing journaled tolerance-growth
budgets with ephemeral work accounting; treating tessellation/approximation quality as a resource
limit; bit-identical progress usage for cancelled runs; wall-clock deadlines, randomization,
completion-order reductions, or a user-supplied executor; moving every test epsilon, security cap,
arena capacity, or schema limit into `OperationContext`; tuning every solver while introducing the
types (moving vs changing policy are separate reviews); requiring context on total bounded leaf
evaluators; owning procedural-graph handles/caches/cycle detection; freezing the C ABI, `Kernel`
facade, or complete error taxonomy.

### Acceptance criteria

F2 is complete when: `kcore` provides validated immutable session/numerical/execution policy,
per-operation context/scope, deterministic budget accounting, structured diagnostics, and reports;
the four tolerance concepts are separately represented and documented; at least one `kops`
intersection, one `kgeom` refinement, and one Full `ktopo` proof consume explicit policy and a
shared root ledger; the graph `EvalContext` consumes a deterministic child reservation without
misplacing policy/state; defaults reproduce legacy bits/completion/errors/journals/rollback for
migrated paths; limits are test-overridable and report stable stage/resource/consumed/high-water/
allowed; budget or numeric-resolution exhaustion cannot yield false completeness, false checker
validity, a degraded mesh, or committed partial topology; serial and all thread counts produce
bit-identical uncancelled results/diagnostics/reports; transaction tolerance-growth stays
transaction-owned and journaled; no migrated module has an unexplained epsilon or work-cap literal
deciding a model/topology fact; the test plan passes in debug/release across the platform matrix.

## Evidence

Boundary tests run each pilot stage at `N-1`/`N`/`N+1` asserting the stable limiting stage/resource;
adversarial tests repeat problems under in-box translation, rescaling, reversed ranges, and
regime-spanning scales; determinism tests randomize worker delays and vary diagnostic level.

- F2 types + policy/budget/report/determinism: `crates/kcore/src/operation/{mod,policy,context,budget,id,tests}.rs`, `crates/kcore/tests/operation_context.rs`, `crates/kcore/tests/determinism.rs`.
- Projection / tessellation / trim contextual paths: `crates/kgeom/tests/surface_point.rs`, `crates/kgeom/tests/trim_orientation.rs`, `crates/kernel/src/tessellation.rs`, `crates/kxt/tests/import_tess.rs`.
- Intersection contextual + numerical scale gate: `crates/kops/tests/{curve_curve,nurbs_nurbs,ellipse_ellipse,operation_intersection,completion}.rs`, `crates/kops/tests/graph_surface*.rs`.
- Checker / make / transaction accounting: `crates/ktopo/tests/{builders,transactions,tolerance_budgets,benchmark_observation}.rs`.
- Legacy retirement + lane enforcement: `scripts/legacy_api_contract.py`, `scripts/test_lanes.py`.

## Open items

- Stage 6: migrate remaining iterative intersection paths and shared drivers; add the review
  lint/audit rejecting new unclassified epsilon/work-cap constants; publish policy-version/limit
  metrics. Segment conditioning, legacy overlap/input and parameter-dedup slack, and the other
  intersection-family minimizers remain unmigrated.
- Standalone projection/tessellation wrappers not yet publicly deprecated (no adopted
  standalone-face path in `kernel`); solver-local error collapses beyond the ellipse/ellipse
  exception remain migration debt.
- NURBS/plane arm: root isolation incomplete; root/window node/depth caps still static and
  non-contextual; unresolved UV boundary cells indeterminate; affine exact fallback outside the
  reviewed envelope and general-NURBS / higher-polynomial / broader topology-decision audits open.
- Design decisions to validate in pilots: work-unit granularity (per-call vs batched); child-budget
  reservation (deterministic frontier vs ordinal); numerical-policy surface area; report allocation
  on large bodies; legacy error-mapping richness (pending F4); checker acceptance-tolerance
  isolation; cross-layer scope lifetime under borrow pressure.
- Coordination: F1 child-budget/usage adapters (do not merge contexts); F3 shared drivers reuse the
  common scope/stage IDs; F4 consumes `StageId`/`DiagnosticCode`/`LimitSnapshot`; F5 owns/shares
  `SessionPolicy`; F7 records policy version + budget profile and fuzzes limit boundaries.
