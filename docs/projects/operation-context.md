# Operation context and numerical policy

Status: Stage 1, intersection and checker Stage 2 pilots, the first conditioning-only Stage 3 pilot, and the first contextual Stage 4 face-tessellation migration implemented

## Purpose

Introduce one explicit, deterministic policy boundary for kernel operations before
general intersections, procedural geometry, booleans, and healing multiply the current
set of local numerical constants and work caps.

This project separates five concerns that are currently easy to conflate:

1. the fixed Parasolid-compatible session precision regime;
2. model-space acceptance tolerances requested by an operation;
3. scale-aware parameter, rounding, and conditioning guards used by algorithms;
4. deterministic proof, work, memory, depth, and output limits; and
5. execution controls and structured diagnostics.

The design must preserve the existing bit-determinism, completion-evidence,
failure-atomicity, and tolerance-provenance contracts. It must also let the kernel add
new policy without adding another argument to every public function each time.

## Current evidence and pressure

The current foundation has the right individual ideas, but no common ownership model
for them:

| Area | Current state | Pressure created by growth |
| --- | --- | --- |
| Session/model tolerance | `kcore::tolerance::Tolerances` carries linear and angular values; `LINEAR_RESOLUTION`, `ANGULAR_RESOLUTION`, and `SIZE_BOX_HALF` define the fixed numeric regime. | The same `Tolerances` value is also used to derive solver stopping and parameter thresholds, obscuring whether a comparison is a model acceptance decision or an implementation guard. |
| Intersections | Public entry points take `Tolerances` directly. NURBS paths contain local sample, bisection, projection, minimization, proof-depth, and candidate caps. | Adding solver controls directly to these signatures would cause repeated churn; leaving them local prevents controlled robustness experiments and structured telemetry. |
| Numerical guards | Intersection and projection code contains absolute values such as `1e-12`, `1e-18`, `1e-24`, and `1e-30`; several modules independently derive parameter tolerance as a fraction of range width. | Absolute thresholds are not consistently scale-aware and their semantic role is unclear. Some are legitimate arithmetic guards, but none should silently enlarge model tolerance or decide topology. |
| Geometry proof/refinement | NURBS implicit isolation already reports candidate-budget and parameter-resolution stops in `ImplicitIsolationLimits`. | This is a useful local precedent, but it does not compose with a parent operation budget or a common diagnostic record. |
| Projection | `kgeom::project` owns fixed sampling, candidate, Newton, and line-search caps and currently panics for a non-finite public window. | General solvers will call projection as a nested stage and need shared budgets, typed stops, and panic-free input handling. |
| Tessellation | `TessOptions` correctly represents requested output quality. Refinement passes, boundary depth, and triangle caps are module constants reported through `Error::AlgorithmLimit`. | Output quality and resource policy must remain separate, while limits become configurable and report stage, observed/consumed work, and allowed work. |
| Checker | `check_body_report` constructs `Tolerances::default()` internally. Sampling counts and adaptive depth/segment caps are local constants. Full checking already represents missing proof as gaps. | A caller cannot budget a Full proof, and a stopped proof needs a structured gap rather than being confused with invalid topology or a clean result. |
| Construction | `ktopo::make` wraps mutation in checked transactions and calls the checker through checked commit. | Construction needs one scope spanning validation, mutation, checking, and rollback so nested work is accounted once and cancellation/limits cannot leave committed partial state. |
| Tolerance growth | Transactions own explicit aggregate tolerance-growth budgets and journal their use. | This stateful model-edit budget is correct and must not be replaced by an ephemeral algorithm work budget. The two need a clear relationship. |
| Parallelism | `kcore::parallel` deterministically assembles index-ordered results, but chooses hardware parallelism globally. | Callers need serial/fixed/available execution controls for testing and deployment without making result selection or budget exhaustion schedule-dependent. |
| Limits and diagnostics | `Error::AlgorithmLimit` carries an operation string and configured limit; completion reasons are static prose; some result types carry richer local limit state. | Metrics and callers need stable stage/resource identifiers and consumed/allowed values without parsing messages. F4 may later refine the shared error taxonomy, but F2 must define the underlying data. |

Representative constants to migrate include:

- `kgeom::project`: curve/surface samples, candidate counts, Newton iterations,
  and backtracking halvings;
- `kgeom::tess`: refinement passes, triangle count, and boundary depth;
- `kgeom::nurbs::patch_bvh`: candidate cells and requested subdivision depth;
- `ktopo::domain`: containment depth and segment count;
- `ktopo::check` and `ktopo::incidence`: deterministic sample counts;
- `ktopo::btess`: edge-refinement depth;
- `kops::intersect`: repeated grid/sample, bisection, polishing, minimization,
  proof-depth, and proof-candidate caps.

Test-only assertion tolerances and schema/security limits such as X_T maximum input node
counts are not automatically operation-policy candidates. The migration audit must
classify each constant by semantics rather than moving every numeric literal.

## Architectural decisions

### 1. Preserve four distinct tolerance concepts

The following values must remain different types or fields with different rules:

| Concept | Meaning | May be caller-loosened? | May prove model acceptance? | Owner |
| --- | --- | --- | --- | --- |
| `SessionPrecision` | Linear/angular resolution and size box of the file/model regime. | No in v1. | Yes, where the kernel specification names session resolution. | Immutable session policy. |
| `Tolerances` | Requested model-space acceptance for an operation, validated at or above session resolution. | Linear tolerance already may be loosened. Angular customization can be added only with validated semantics. | Yes, for that operation's documented residual/proximity contract. | Operation context. |
| Entity tolerance | Persisted per-face/edge/vertex model allowance with provenance. | Only through checked operation rules. | Yes, for obligations involving that entity. | Topology plus transaction journal. |
| Numerical guard | Parameter progress, rounding slack, scaled-zero, or conditioning threshold. | Only through a validated numerical profile, initially kernel-owned. | No. It may stop refinement or classify a solve as ill-conditioned, but it cannot certify incidence, coincidence, containment, or a topological sign. | Session numerical policy, applied with local scale data. |

`TessOptions::chord_tol`, future angular faceting tolerance, approximation error bounds,
and similar requested output quality remain operation request data. They are neither
session resolution nor work limits.

The existing `Tolerances` type and constructors remain source-compatible. Its
documentation should be narrowed to "model acceptance tolerances" during rollout; it
must not grow fields for iterations, sampling, or solver conditioning.

### 2. Use immutable session policy and a fresh per-operation scope

`kcore` should add an `operation` module with these conceptual types:

```rust
pub struct SessionPolicy {
    precision: SessionPrecision,
    numerical: NumericalPolicy,
    execution: ExecutionPolicy,
    default_budget: BudgetPlan,
    policy_version: PolicyVersion,
}

pub struct OperationContext<'session> {
    session: &'session SessionPolicy,
    tolerances: Tolerances,
    budget_overrides: BudgetPlan,
    diagnostic_level: DiagnosticLevel,
    cancellation: Option<&'session dyn CancellationToken>,
}

pub struct OperationScope<'context, 'session> {
    context: &'context OperationContext<'session>,
    ledger: WorkLedger,
    diagnostics: Vec<OperationDiagnostic>,
    next_diagnostic_ordinal: u64,
}
```

Names may be adjusted to Rust lifetime constraints during implementation, but the
ownership boundary is normative:

- `SessionPolicy` is validated, immutable, cheap to share, and owns no model, graph, or
  topology state. A future `Kernel`/`Session` facade may own an `Arc<SessionPolicy>`;
  F2 does not depend on that facade.
- `OperationContext` is a cheap borrowed configuration snapshot. It does not contain
  mutable counters and can be shared when planning deterministic parallel work.
- `OperationScope` is created once for a top-level call and owns all mutable work usage
  and diagnostic buffers. Nested algorithms borrow the same scope or deterministic
  child scopes; they do not create fresh default budgets.
- A context is never stored in geometry or topology entities. Persisted entities retain
  only their existing exact data, entity tolerances, and provenance.
- There are no process-global mutable defaults. Changing policy means creating a new
  validated `SessionPolicy` or a new operation context.

`SessionPrecision::parasolid()` is the only production v1 precision regime and exposes
the current `1e-8 m`, `1e-11 rad`, and `500 m` half-size values. Keeping this as data
rather than scattered constants makes dependencies explicit without promising that
arbitrary regimes are supported.

### 3. Make numerical policy scale-aware and proof-ineligible

`NumericalPolicy` centralizes recipes, not unqualified epsilon constants. Its public
surface should accept the scale information needed by the decision:

```rust
pub enum NumericGuardKind {
    ParameterProgress,
    CoefficientCancellation,
    LinearSolve,
    PeriodicNormalization,
    BudgetAccounting,
}

pub struct ParameterScale {
    pub coordinate_magnitude: f64,
    pub span: f64,
    pub output_rate_upper: Option<f64>,
}

pub struct ParameterTolerance {
    pub termination_step: f64,
    pub rounding_floor: f64,
    pub metric_driven_step: Option<f64>,
}

impl NumericalPolicy {
    pub fn rounding_guard(&self, kind: NumericGuardKind, scale: f64) -> f64;
    pub fn parameter_tolerance(
        &self,
        scale: ParameterScale,
        output_tolerance: f64,
    ) -> Result<ParameterTolerance>;
    pub fn reciprocal_condition_is_usable(&self, rcond: f64) -> bool;
}
```

The first implementation should use named, documented factors over `f64::EPSILON` and
the magnitude of the actual coefficients, parameter window, Jacobian, or accounting
values. Absolute floors may remain only where the represented quantity has a fixed,
documented normalization; otherwise they are bugs to classify, not defaults to copy.

Rules for consuming these values:

- A parameter-progress threshold can terminate an iteration. The candidate still needs
  an independent model-space residual check before it is accepted.
- If the rounding floor is larger than the metric-driven step and the residual is not
  independently certified, the result is numerically stopped/indeterminate rather than
  silently accepted.
- An ill-conditioned Jacobian selects a safeguarded fallback, subdivision, or an
  explicit conditioning diagnostic. It does not convert a near-contact into contact.
- Exact predicates and interval-certified signs continue to decide topology. A numeric
  guard must never replace them.
- Call sites name the semantic `NumericGuardKind`; raw `EPSILON` multipliers should be
  limited to `kcore` policy implementations and narrowly justified exact-arithmetic
  modules.

The default numerical profile is versioned (`PolicyVersion::V1`) because changing a
factor can change output bits or completion. Policy versions belong in corpus and
benchmark metadata, not in persisted B-rep entities.

### 4. Represent budgets as a deterministic plan and ledger

One monolithic struct with a field for every future algorithm would make `kcore` depend
on higher layers. An untyped map of prose strings would lose compile-time discipline.
Use stable stage constants defined by the owning crate and generic resource accounting:

```rust
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct StageId(&'static str); // validated namespaced identifier

pub enum ResourceKind {
    Work,
    Items,
    Bytes,
    Depth,
}

pub enum AccountingMode {
    Cumulative,
    HighWater,
}

pub struct LimitSpec {
    pub stage: StageId,
    pub resource: ResourceKind,
    pub mode: AccountingMode,
    pub allowed: u64,
}

pub struct LimitSnapshot {
    pub stage: StageId,
    pub resource: ResourceKind,
    pub consumed: u64,
    pub allowed: u64,
}
```

Every stage identifier is a public constant at the layer that defines the operation,
for example `kgeom.tess.boundary-depth`, `kgeom.project.curve-newton`,
`ktopo.check.domain-segments`, or `kops.intersect.ssi-proof-candidates`. Prose messages
are attached separately and may change; identifiers may not.

Higher-layer crates define typed default profile constructors that produce a
`BudgetPlan`. This avoids a dependency from `kcore` to intersection or tessellation
types while grouping related caps coherently:

```rust
impl IntersectionBudgetProfile {
    pub fn v1_defaults() -> BudgetPlan;
}

impl TessellationBudgetProfile {
    pub fn v1_defaults() -> BudgetPlan;
}
```

The ledger supports:

- `charge(stage, amount)` for evaluations, samples, iterations, subdivisions, and other
  cumulative work;
- `observe(stage, value)` for recursion depth, retained candidates, triangle/output
  count, and scratch-memory high-water marks;
- deterministic child reservation by stable work-item ordinal;
- a root total-work ceiling in addition to stage-specific limits; and
- accepted-usage snapshots plus first-crossing and numeric-resolution evidence on
  both success and failure.

When a parent has a root total-work ceiling, every child reserves root capacity
as well as stage capacity. If a child plan omits an explicit root ceiling, the
ledger infers the checked sum of its cumulative Work allowances; an explicit
stricter child ceiling is preserved. This makes a valid completed child
mergeable instead of allowing parent work to consume capacity already promised
implicitly to the child.

The unit charged at each stage is part of that stage's documentation and tests. A
"work" unit is not a time unit. Wall-clock deadlines are deliberately excluded because
they make the amount of explored geometry machine- and scheduling-dependent.

Budget exhaustion follows the existing proof contract:

- If the API can retain individually verified partial evidence, return it with
  `Completion::Indeterminate` (or the checker's corresponding verification gap) and a
  structured `LimitSnapshot`.
- If no sound partial-result contract exists, return `Error::AlgorithmLimit` during the
  compatibility period and include the structured limit in the operation report.
- Never discard candidates merely to fit a budget and then report a complete result.
- Never turn a configured work limit into `InvalidGeometry`.

F4 may replace `Error::AlgorithmLimit` with a richer shared error variant. F2 owns the
stable `StageId`, resource, usage, and limit data so that migration does not duplicate
concepts.

### 5. Return a report without breaking existing `Result<T>` APIs

Context-aware entry points return an outcome that preserves diagnostics even when the
operation fails:

```rust
pub struct OperationOutcome<T, E = kcore::error::Error> {
    result: core::result::Result<T, E>,
    report: OperationReport,
}

pub struct OperationReport {
    policy_version: PolicyVersion,
    usage: Vec<LimitSnapshot>,
    limit_events: Vec<LimitSnapshot>,
    numeric_resolution_stages: Vec<StageId>,
    diagnostics: Vec<OperationDiagnostic>,
}

impl<T, E> OperationOutcome<T, E> {
    pub fn result(&self) -> core::result::Result<&T, &E>;
    pub fn report(&self) -> &OperationReport;
    pub fn into_result(self) -> core::result::Result<T, E>;
    pub fn into_parts(self) -> (core::result::Result<T, E>, OperationReport);
    pub fn map<U>(self, op: impl FnOnce(T) -> U) -> OperationOutcome<U, E>;
    pub fn map_err<F>(self, op: impl FnOnce(E) -> F) -> OperationOutcome<T, F>;
}
```

This shape avoids putting mutable output sinks in the context, preserves reports after
errors, and lets each layer retain its classified error without copying report machinery.
`OperationScope::finish` remains fixed to `kcore::Error` so legacy
`finish(Ok(value))` calls stay inference-safe; `finish_typed` constructs an outcome for a
layer-owned error. Reports are assembled only after child work is merged in deterministic
ordinal order.

`usage` records accepted accounting. `limit_events` separately retains the first
attempted crossing for each configured stage/resource pair, and
`numeric_resolution_stages` retains arithmetic-resolution stops. These two semantic
records are independent of diagnostic level; optional diagnostics add bounded human
context but are never the only machine-readable explanation for incomplete work.

Existing public functions remain and become compatibility wrappers:

```rust
pub fn intersect_bounded_curves(/* current args */, tolerances: Tolerances)
    -> Result<CurveCurveIntersections>
{
    let context = OperationContext::legacy(tolerances);
    intersect_bounded_curves_with_context(/* inputs */, &context).into_result()
}
```

The contextual form is additive:

```rust
pub fn intersect_bounded_curves_with_context(
    /* geometry and ranges */,
    context: &OperationContext<'_>,
) -> OperationOutcome<CurveCurveIntersections>;
```

Internal helpers take `&mut OperationScope`, not another `OperationContext`, so nested
work cannot accidentally reset usage. The legacy adapter uses the exact v1 defaults and
discards only the new report. Existing result contents, completion status, error variant,
and golden bits must remain unchanged during the compatibility rollout.

### 6. Keep diagnostics structured, bounded, and observational

```rust
pub struct DiagnosticCode(&'static str); // stable namespaced identifier

pub enum DiagnosticKind {
    LimitReached(LimitSnapshot),
    NumericResolution,
    IllConditioned,
    FallbackSelected,
    ProofIncomplete,
    Cancelled,
}

pub struct OperationDiagnostic {
    pub ordinal: u64,
    pub stage: StageId,
    pub code: DiagnosticCode,
    pub kind: DiagnosticKind,
    pub message: &'static str,
}
```

Diagnostics are semantic summaries, not arbitrary logging:

- codes and stage identifiers are stable and machine-readable;
- messages provide human context but are not control-flow contracts;
- repeated diagnostics are deduplicated or capped by a documented diagnostic budget;
- subject-specific details remain in the owning result type when they require topology
  handles or geometry classes, avoiding a dependency from `kcore` upward;
- enabling diagnostics cannot change branch selection, work accounting, output order,
  or model mutation; and
- callbacks are not invoked from parallel workers. Child buffers are merged by stable
  work ordinal and only then exposed in the report.

Low-level performance tracing, wall time, thread count, and OS telemetry are explicitly
non-semantic instrumentation and must not be mixed into the deterministic
`OperationReport`. They may be collected by benchmark tooling outside the kernel.

### 7. Execution policy controls concurrency, never ordering

`ExecutionPolicy` supports `Serial`, `AtMost(NonZeroUsize)`, and `Available` modes.
`kcore::parallel` gains context-aware map/reduce primitives while retaining its existing
wrappers for compatibility.

Normative rules:

- work items receive stable ordinals before parallel execution;
- result and diagnostic merging is ordinal-ordered;
- limit and numeric-resolution evidence from child ledgers is merged in that same
  ordinal order;
- floating reductions use a prescribed index order or a prescribed deterministic tree,
  never completion order;
- budget allocation cannot be an atomic race. A frontier is planned serially or each
  child receives a deterministic reservation before it runs;
- unused child reservation is returned only at a deterministic join point and cannot be
  stolen based on completion timing; and
- serial and every permitted thread count produce bit-identical results, completion,
  semantic diagnostics, and budget usage for an uncancelled operation.

External cancellation is allowed only through a read-only token checked at documented
safe points. Timing determines when cancellation is observed, so a cancelled operation
does not promise the same usage report across runs. It must return no successful partial
model mutation, must roll back any active transaction, and must not expose timing-based
partial geometry as a complete result. Uncancelled runs retain the full determinism
contract. Cancellation can land after F4 supplies its stable error category; the types
reserve the boundary now, but the initial F2 slice need not enable it.

## Relationship to procedural geometry `EvalContext`

The procedural geometry graph project owns a narrow per-query `EvalContext` and
`EvalLimits` for handle resolution, dependency traversal, cache use, cycle detection,
node visits, and dependency depth. F2 must not create a second graph context.

The ownership relationship is:

```text
SessionPolicy
    └── OperationContext
          └── OperationScope / global deterministic ledger
                └── child reservation for one geometry query
                      └── graph EvalContext + EvalLimits
```

The operation layer constructs an `EvalContext` with exactly the inputs F1 owns:

- a borrow of the graph and the graph project's query-local cache/cycle state;
- a copy of the operation's existing `Tolerances` value;
- a deterministically reserved node-visit/depth allowance represented as `EvalLimits`;
  and
- an operation-side stable child work ordinal used only to merge the query outcome.

F2 maps F1's typed `DependencyDepthExceeded` and `NodeVisitLimitExceeded` errors into
the parent ledger/diagnostics without changing the graph error. If exact successful-query
usage is needed, F1 may expose a small `last_node_visits()`/query-usage accessor; that
does not move the ledger into the graph. `EvalContext` does not own `SessionPolicy`,
`NumericalPolicy`, a parallel executor, cancellation semantics, topology, or the
operation's diagnostic buffer. Conversely, `OperationContext` does not know graph
descriptors, handle resolution, caches, or cycle stacks.

Direct low-level `kgraph` clients may construct an `EvalContext` with standalone
`EvalLimits` and default validated `Tolerances`. Higher-level `ktopo`, `kops`, and `kxt`
clients must derive it from their active operation scope so nested evaluation is charged
to the caller's operation rather than an independent default budget.

## Layer consumption

### `kcore`

- Add `operation` types, policy validation, stable IDs, budget plans/ledgers, reports,
  and deterministic child reservation.
- Extend deterministic parallel primitives to accept `ExecutionPolicy`.
- Keep the current constants and `Tolerances` compatibility API; add
  `SessionPrecision` and scale-aware numerical helpers alongside them.
- Do not add geometry-, topology-, intersection-, checker-, or tessellation-specific
  fields to `kcore`.

### `kgeom`

- Projection gains contextual, fallible entry points. Fixed samples, candidates,
  iterations, and halvings move to a named projection budget profile.
- NURBS isolation accepts a child scope/reservation. Its existing
  `ImplicitIsolationLimits` becomes an algorithm-specific view over common limit and
  numeric-resolution events; candidate covers remain conservative.
- Tessellation keeps `TessOptions` for requested quality. Resource caps move to a named
  tessellation budget profile and limit failures include stage/usage data.
- Parameter and conditioning checks call `NumericalPolicy` with local range, derivative,
  coefficient, and Jacobian scales.
- Pure evaluator methods that are total, bounded, and allocation-free do not need a
  context solely for uniformity.

### `ktopo` checker

- Add `check_body_report_with_context(store, body, level, context)` and retain current
  wrappers.
- Structural Fast checks use fixed session precision and applicable entity tolerance;
  a caller's looser intersection tolerance must not make an invalid body pass.
- Deterministic Fast samples may use a named profile but remain fault-detection only,
  never proof evidence.
- Full adaptive checks charge subdivisions, segments, evaluations, and candidate pairs.
  Exhaustion adds a structured verification gap linked to a limit diagnostic, not a
  fault and not `Valid`.
- A `Valid` Full result means all obligations completed inside their limits. Raising a
  budget may discharge gaps; it may not erase proven faults.

### `ktopo::make` and checked transactions

- Add contextual variants for the internal checked-creation driver first, then public
  constructors as needed. The one scope spans input validation, topology assembly,
  affected-body checking, and commit.
- Limit, cancellation, or checker failure rolls back exactly as current checked commit
  failures do. Reports can describe attempted work, but committed journals contain only
  successful model changes.
- Transaction-owned tolerance-growth budgets remain separate. An operation context may
  impose a policy ceiling on what a constructor is allowed to declare, but actual
  entity growth is still declared, charged, rolled back, and journaled by the
  transaction API.
- Exact primitive construction should not start depending on model tolerance simply
  because a context exists. Sub-resolution input rejection continues to follow the
  documented session/model contract.

### `kops`

- Add context-aware top-level curve/curve, curve/surface, and surface/surface dispatch.
  Specialized pair algorithms share the caller's scope.
- Move repeated sample/bisection/polish/minimize/proof limits into named intersection
  profiles. A specialized solver can add stage-specific entries without changing the
  public function signature.
- Replace repeated parameter-tolerance helpers with common scale-aware recipes, while
  preserving independent model-space residual acceptance.
- Report conditioning, fallback selection, proof budget exhaustion, and retained partial
  evidence structurally. Completion survives dispatch normalization and nested calls.
- Analytic closed forms still charge bounded work only if useful for aggregate metrics;
  context introduction must not force them through iterative infrastructure.

### Tessellation across `kgeom` and `ktopo::btess`

- `TessOptions` remains the quality request shared by face/body tessellation.
- A body tessellation owns one scope. Boundary discretization, per-face tessellation,
  stitching, and output assembly use named child stages and one aggregate output/scratch
  budget.
- Shared edges are discretized once as today. Per-face parallelism reserves deterministic
  child budgets by face order, and face meshes are spliced in that same order.
- Hitting a cap without meeting chord/edge/angle quality is a limit failure; the kernel
  never returns a lower-quality mesh labeled successful.

## Compatibility and migration strategy

The project is additive and staged. There is no repository-wide signature rewrite.

1. Keep every existing public function and its existing defaults.
2. Add `_with_context` entry points only at top-level API seams. Internal pair/helper
   functions accept `&mut OperationScope` as they migrate.
3. Implement legacy wrappers in terms of contextual entry points with an exact v1
   compatibility policy.
4. Preserve current return values and errors in wrappers. New reports are opt-in until
   the facade/C API chooses a stable public representation.
5. Move constants in behavior-preserving pilots before tuning any value. Default-value
   changes require their own evidence, corpus update, and policy-version decision.
6. Do not require leaf analytic evaluators or simple arena accessors to accept context.
   Context appears where work can be iterative, recursive, procedural, parallel,
   diagnostic-bearing, or state-changing.
7. Deprecation of tolerance-only overloads is a later facade/API decision, not an F2
   exit criterion.

## Rollout stages

### Stage 0 — Audit and vocabulary lock

- Classify production constants as model acceptance, numerical guard, requested output
  quality, proof/work limit, security/input limit, or test-only assertion tolerance.
- Assign stable namespaced stage and diagnostic IDs for the pilot paths.
- Record current defaults and golden outputs before moving code.

Exit: every pilot constant has one documented category; no behavior changes.

### Stage 1 — Land inert `kcore` infrastructure

- Add validated `SessionPrecision`, `NumericalPolicy`, `SessionPolicy`,
  `ExecutionPolicy`, `OperationContext`, `OperationScope`, budget/ledger/report types,
  and deterministic child reservation.
- Add unit tests for validation, accounting boundaries, report ordering, and policy
  versioning.
- Add context-aware parallel map helpers.

Exit: types are usable without any production operation depending on them; existing
tests and determinism hashes are unchanged.

### Stage 2 — Behavior-preserving proof/refinement pilots

- Migrate `NurbsSurfaceBvh::isolate_implicit_candidates` as the geometry pilot.
- Migrate the NURBS surface/implicit intersection proof path as the `kops` pilot.
- Migrate face-domain containment or one Full checker proof as the `ktopo` pilot.
- Adapt graph `EvalContext` construction when F1 lands, using child budget reservation
  rather than duplicating policy.

Exit: default results are bit-identical; candidate/depth exhaustion is visible as
structured data; checker exhaustion remains indeterminate.

### Stage 3 — Numerical-policy pilot

Status: the NURBS/NURBS Newton symmetric 2×2 conditioning decision now uses
the shared numerical policy and normalized extreme-scale fallback while model
residuals retain sole contact authority. Absolute gradient, parameter-progress,
minimizer, segment-conditioning, and repeated parameter-tolerance guards remain
separate migrations.

- Replace the repeated NURBS intersection parameter-tolerance helpers with the
  scale-aware policy API.
- Replace the absolute determinant/gradient/progress guards in the NURBS/NURBS solver
  with normalized conditioning and scaled-zero checks.
- Keep old v1 numerical behavior where it is semantically valid; where an old absolute
  threshold is unsound, fix it in a separately reviewable change with adversarial scale
  tests.

Exit: candidate acceptance still depends on model residuals; parameter/conditioning
guards have no direct proof authority; scale tests pass.

### Stage 4 — Projection and tessellation

Status: face tessellation now has contextual and shared-scope entry points,
truthful boundary-depth and completed-pass accounting, exact structured limit
evidence, and a bit-compatible legacy adapter. Contextual projection, body
tessellation, and execution-policy equivalence remain.

- Add fallible contextual projection APIs and remove public panic behavior through the
  new path.
- Migrate face and body tessellation resource limits while retaining `TessOptions`.
- Verify serial/fixed/available parallel equivalence before enabling new face-level
  parallel execution.

Exit: quality failures are never silent, reports identify the limiting stage, and all
thread-count variants produce identical mesh bits and semantic reports.

### Stage 5 — Checker/make integration

- Route checked construction through one scope, including affected-body checking.
- Add contextual checker APIs and structured Full verification gaps.
- Define the policy-ceiling relationship to transaction tolerance-growth budgets without
  moving consumption out of the transaction.

Exit: limit/cancellation/check failure is rollback-clean; successful journals are
unchanged under the compatibility policy.

### Stage 6 — Broad intersection migration and enforcement

- Migrate remaining iterative intersection paths and shared drivers as F3 consolidates
  them.
- Add a review lint or targeted source audit that rejects new unclassified production
  epsilon/work-limit constants in migrated modules.
- Publish policy version and limit metrics in the corpus tooling.

Exit: adding a solver stage requires a named profile entry and structured stop; migrated
modules contain no unexplained numerical or work-cap literals.

## Test plan

### Policy and type tests

- Reject non-finite, negative, zero where forbidden, duplicate stage/resource, and
  overflow-prone budget specifications.
- Verify the production session regime exactly matches the existing constants.
- Verify all default profiles are stable for `PolicyVersion::V1`.
- Verify `Tolerances::with_linear` compatibility and that numerical guards cannot be
  converted into entity/model tolerance accidentally.

### Budget boundary tests

- For each pilot stage, run at allowed work `N - 1`, `N`, and `N + 1`.
- Assert exact consumed/high-water values and the stable limiting stage/resource.
- Assert nested work is charged once and cannot reset by constructing a nested context.
- Assert child reservations and unused-work reconciliation are independent of worker
  completion order.
- Assert limit exhaustion retains a conservative candidate cover or verified partial
  evidence and never claims `Complete` incorrectly.

### Numerical/adversarial tests

- Repeat equivalent problems under translations within the size box, parameter-domain
  rescaling, reversed ranges, and geometry scales spanning the supported regime.
- Exercise nearly singular Jacobians, tangent contacts, multiple roots, collapsed
  parameter progress, large coefficients with small normalized determinants, and
  rounding at periodic seams.
- Assert a looser model tolerance changes only documented acceptance decisions, while a
  numerical-policy change cannot turn an unverified candidate into a certified one.
- Assert numeric-resolution stops are distinct from configured-budget stops and invalid
  input.

### Compatibility tests

- Compare every legacy wrapper with its contextual v1-default equivalent, including
  output bits, ordering, completion, error variants, topology journals, and rollback
  state.
- Keep current debug/release/platform determinism hashes unchanged for behavior-preserving
  stages.
- Add compile tests showing ordinary legacy call sites continue to build.

### Layer-specific tests

- Intersection: complete/indeterminate status, retained points/curves, residual bounds,
  and structured solver/proof limits.
- Checker: raising a Full proof budget can discharge a gap; lowering it cannot hide a
  fault; looser operation tolerance cannot weaken structural validity.
- Make: limit and cancellation at validation, assembly, and checked-commit boundaries
  leave the store and journal exactly as before the call.
- Tessellation: requested quality is met or a typed limit is returned; output count/depth
  high-water data is exact.
- Procedural evaluation: child `EvalLimits` are charged to the parent operation once;
  cycle/depth/node stops remain graph-specific and surface as structured operation
  diagnostics.

### Determinism tests

- Run representative intersection, Full checker, graph evaluation, and body tessellation
  with serial, two-thread, and available-thread policies.
- Hash result bits, completion, semantic diagnostics, and usage reports; all must match
  for uncancelled operations.
- Randomize worker delays in tests to prove budget allocation and merge order do not
  depend on scheduling.
- Verify diagnostic level `Off`, `Summary`, and `Verbose` does not change result or work
  selection; only permitted report detail differs.

## Non-goals

- Making linear/angular resolution or the size box arbitrarily configurable in v1.
- Replacing transaction-owned, journaled tolerance-growth budgets with ephemeral work
  accounting.
- Treating tessellation/approximation quality as a resource limit.
- Guaranteeing bit-identical progress usage for externally cancelled runs.
- Adding wall-clock deadlines, randomization, completion-order reductions, or a
  user-supplied arbitrary executor.
- Moving every test epsilon, file-format security cap, arena capacity, or schema limit
  into `OperationContext`.
- Tuning every solver while introducing the types. Moving policy and changing policy are
  separate reviews.
- Requiring context parameters on total, bounded leaf evaluation and trivial query APIs.
- Owning procedural-graph handles, caches, dependency stacks, or cycle detection; those
  remain the geometry graph `EvalContext`'s responsibility.
- Freezing the final C ABI, public `Kernel` facade, or complete error taxonomy. F2
  provides data those projects can expose.

## Dependencies and coordination

- **F1 procedural geometry graph:** agree on child-budget/usage adapters between
  `OperationScope` and graph `EvalContext`; do not merge the contexts.
- **F3 intersection consolidation:** use the common scope and stage IDs in shared drivers;
  avoid broad solver migration before F2 Stage 2/3 types stabilize.
- **F4 error/capability taxonomy:** consume `StageId`, `DiagnosticCode`, and
  `LimitSnapshot`; decide the final error mapping without redefining work data.
- **F5 kernel facade:** eventually owns/shares `SessionPolicy` and exposes contextual
  request/result APIs. F2 remains usable without it.
- **F7 benchmarks/fuzzing:** record policy version and budget profile with results; fuzz
  policy validation and limit boundaries.

## Open risks and decisions to validate in pilots

1. **Work-unit granularity.** Charging every evaluator call is clear but may add overhead;
   batching charges reduces overhead but makes exact stop boundaries coarser. Pilot both
   and benchmark before freezing a convention.
2. **Child budget reservation.** Static equal partitions can strand work while dynamic
   stealing risks schedule dependence. Prefer deterministic frontier rounds or
   ordinal-based reservations; validate with realistic SSI and tessellation workloads.
3. **Policy surface area.** Exposing every numerical factor invites unsupported tuning.
   Keep `NumericalPolicy` constructors versioned and validated initially; expose only
   settings with a documented correctness range.
4. **Report allocation.** Verbose per-stage diagnostics could become material on large
   bodies. Keep summaries bounded, deduplicate repeated events, and make detailed traces
   explicitly diagnostic-level controlled.
5. **Legacy error mapping.** Contextual structured limits and current
   `Error::AlgorithmLimit` differ in richness. Preserve legacy behavior until F4 chooses
   the final variant and add mapping tests.
6. **Checker semantics.** The checker must not inherit a loose operation acceptance
   tolerance accidentally. Pilot APIs should make fixed session/entity tolerance use
   obvious in types and tests.
7. **Cross-layer context lifetime.** A scope spanning a mutable topology transaction and
   nested immutable geometry evaluation can encounter borrow pressure. Prototype the
   checked-construction pilot before freezing exact Rust lifetime signatures; do not
   solve it with global mutable state or interior-mutability races.

## Acceptance criteria

F2 is complete when all of the following hold:

- `kcore` provides validated immutable session/numerical/execution policy, per-operation
  context/scope, deterministic budget accounting, structured diagnostics, and reports.
- Fixed session precision, model acceptance tolerance, entity tolerance, output quality,
  and numerical guards are separately represented and documented.
- One representative `kops` intersection, one `kgeom` refinement path, and one Full
  `ktopo` proof consume explicit policy and a shared root ledger.
- The geometry graph's `EvalContext` consumes a deterministic child reservation without
  duplicating operation/session policy or graph state in the wrong layer.
- Defaults reproduce the legacy APIs' result bits, completion, errors, journals, and
  rollback behavior for migrated paths.
- Limits are test-overridable and report stable stage, resource, consumed/high-water, and
  allowed values.
- Budget or numeric-resolution exhaustion cannot yield false completeness, false checker
  validity, a silently degraded mesh, or committed partial topology.
- Serial and all supported thread counts produce bit-identical uncancelled results,
  completion, semantic diagnostics, and usage reports.
- Transaction tolerance-growth accounting remains transaction-owned and journaled.
- No migrated production module contains an unexplained absolute epsilon or work-cap
  literal that makes a model/topology decision.
- The test plan above passes in debug/release and on the supported platform matrix, and
  pilot benchmarks show that accounting overhead is measured and acceptable.
