# F4 kernel error and capability taxonomy

Status: Phase 1, representative Phase 2 slices, graph classification, one structured-incompleteness pilot, and the first solver-local source-identity migration implemented

## Outcome

Give every public failure, unsupported feature, incomplete proof, and checker
finding a stable machine-readable identity without making human-readable text an
API. Preserve the distinctions already present in X_T capability codes and
checker reports, and make the distinctions missing from `kcore::Error` and
`Completion` explicit.

This project defines contracts and a staged migration. It does not require a
flag-day conversion of every existing `InvalidGeometry { reason }` call site.

The central rule is:

> An error says that the requested call did not produce a usable value. An
> outcome says what was proven. A diagnostic explains how that outcome was
> reached. These are related, but they are not interchangeable.

## Current evidence

The repository already contains useful pieces, but their classifications are
not yet consistent:

- `kcore::Error` is typed at the variant level, but `InvalidGeometry` and
  `AlgorithmLimit` place control-flow distinctions in prose. `AlgorithmLimit`
  reports an operation name and allowed count, but not a stable stage, resource,
  or consumed count.
- intersection dispatch has typed unsupported curve/surface class-pair errors,
  while remaining solver-local catch-all mappings are being migrated by owner;
- ellipse/ellipse is the first solver-local source-identity slice: all five
  `ProjectionError` variants survive as `IntersectionError::Projection` through
  generic and facade adapters without changing class, code, limit, capability,
  or source identity;
- intersection result types correctly distinguish complete evidence from
  verified partial evidence, but `Completion::Indeterminate` identifies the
  missing proof only with a static message;
- the topology checker correctly keeps proven `Fault`s separate from
  `VerificationGap`s and reports `Valid`, `Invalid`, or `Indeterminate`;
- `XtCapability` already provides unique stable dotted codes and `XtError`
  exposes valid-but-unsupported content without parsing display text; and
- F1 requires a detailed local `kgraph::EvalError`, while F2 defines
  deterministic `StageId`, `ResourceKind`, and `LimitSnapshot` data.

F4 standardizes how these pieces meet. It does not flatten them into one giant
enum.

## Normative classification

Every public non-success condition belongs to exactly one of the following
semantic classes. The same underlying numerical observation can have different
classes at different API boundaries, so classification is made by the owner of
the public contract, not by matching its message.

| Class | Meaning | Normal representation |
| --- | --- | --- |
| Invalid input | The caller supplied a value or request that violates a documented precondition: non-finite parameter, invalid range, degenerate constructor input, or stale handle where a live handle is required. | `Err`, with a stable error code and structured subject data where useful. |
| Unsupported capability | The request and model are valid, but this kernel version does not implement the requested representation, class pair, derivative order, schema feature, or proof method. | `Err` when no useful result contract exists; otherwise verified partial evidence with an indeterminate cause naming the capability. |
| Indeterminate outcome | No contradiction was proven, but one or more required proof obligations remain unresolved. This is neither success nor invalid input. | A successful result/report carrying partial evidence and structured incomplete-proof data. |
| Resource limit | A deterministic configured work, item, byte, output, or depth allowance was reached. | `LimitSnapshot`; it causes either an indeterminate partial outcome or `Err` according to the API's partial-result contract. |
| Model invariant violation | A checker proved that topology or geometry in a model violates a declared modeling invariant. | A checker `Fault` and `CheckOutcome::Invalid`; a checked transaction may surface a model-rejected error that points to the retained report. |
| Invalid operation state | The request is meaningful in general but illegal in the current session/transaction state, such as nested or inactive transactions. | `Err`, distinct from invalid model data. |
| Internal invariant violation | Kernel-owned state or an algorithm-produced value violates an invariant that callers could not have been required to establish. This indicates a defect or corrupt persisted state, not unsupported input. | Typed `Err` at safe Rust boundaries; never a panic across the facade or C ABI. |

Cancellation is an operation stop, not invalid input or a resource limit. F2 may
represent deterministic cancellation in diagnostics and the public facade may
add a `Cancelled` error class. It must not be used to claim a complete result.

### Boundary examples

- A public `Line::new` call with a zero direction is invalid input. A solver
  internally constructing that same zero direction from supposedly regular
  intermediate data is an internal invariant failure or an explicitly handled
  singular numerical outcome.
- A valid curve/curve class pair with no registered solver is unsupported, not
  `InvalidGeometry`.
- A solver that found verified contacts before exhausting candidate work
  returns those contacts as indeterminate with a `LimitSnapshot`. A solver whose
  API cannot safely expose partial state returns a limit error with the same
  snapshot.
- An open loop discovered by `check_body_report` is a model fault. Inability to
  certify that a loop is simple is a verification gap. Neither is an internal
  kernel failure.
- Failure to evaluate an offset at a singular basis point is an evaluation
  failure with graph-local subject data. It is not proof that the offset
  descriptor itself is invalid everywhere.

## Stable identifiers

F2's `StageId` remains the canonical stage identity and `DiagnosticCode` remains
the identity for observational diagnostics. F4 adds two equally small shared
identifier wrappers in `kcore`:

```rust
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ErrorCode(&'static str);

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CapabilityId(&'static str);
```

Construction is `const` and validates literals in tests. These wrappers carry
no registry and no knowledge of higher-layer enums. They let an owning crate
define constants without adding an upward dependency to `kcore`.

Identifiers obey these rules:

1. They are lower-case ASCII dotted paths using digits and hyphens within a
   segment.
2. The first segment is the owning crate/layer: `kcore`, `kgeom`, `kgraph`,
   `ktopo`, `kops`, or `xt`.
3. An identifier is permanent once shipped. Its message and Rust variant name
   may change; its meaning may only be narrowed compatibly.
4. A behaviorally incompatible replacement gets a new identifier. A version
   suffix is used only when two semantic versions must coexist, not for ordinary
   releases.
5. Codes identify a branch callers may act on. They must not encode arena slot
   numbers, geometry handles, parameter values, or arbitrary source text.
6. Display messages are not stable and must never be parsed.

The three namespaces have different jobs:

- `ErrorCode` answers **why did this call fail?**
- `CapabilityId` answers **which valid feature is unavailable?**
- `StageId` answers **where was work consumed or stopped?**

One error may have all three. For example, an intersection may have error code
`kops.intersect.unsupported-class-pair`, capability
`kops.intersect.curve-curve.class-pair`, and stage
`kops.intersect.curve-curve.dispatch`.

## Ownership by layer

Identifier constants live with the implementation and contract that owns their
meaning. `kcore` owns only wrappers, broad classes, and identifiers for its own
preconditions/state.

| Layer | Owns | Examples |
| --- | --- | --- |
| `kcore` | Identifier wrappers, broad error classes, F2 limit structures, arena/session/transaction-neutral base errors. | `kcore.input.invalid-tolerance`, `kcore.handle.stale` |
| `kgeom` | Leaf constructor, projection, tessellation, and numerical-stage codes/capabilities. | `kgeom.project.invalid-query-point`, `kgeom.project.no-candidate`, `kgeom.tess.triangle-output` |
| `kgraph` | Geometry dependency, graph build, descriptor, and evaluation codes/capabilities. | `kgraph.eval.dependency-cycle`, `kgraph.eval.derivative-order` |
| `ktopo` | Topology operation codes, checker fault/gap codes, model-invariant identities. | `ktopo.check.fault.open-loop`, `ktopo.check.gap.shell-self-intersection` |
| `kops` | Modeling-operation and intersection dispatch/solver codes and capabilities. | `kops.intersect.unsupported-class-pair`, `kops.intersect.ssi-proof-candidates` |
| `kxt` | X_T parsing, schema, reconstruction, and emission codes and support-matrix capabilities. Existing external capability strings keep the `xt.` prefix. | `xt.read.general-bodies`, `xt.parse.bad-field` |

Higher layers may delegate a lower-layer classification unchanged. They add a
new code only when their public contract adds meaning. Wrapping an invalid
tolerance during X_T reconstruction does not turn it into `xt.read.invalid-*`;
the X_T error delegates the `kcore` code and retains reconstruction context for
display. Conversely, malformed node fields are owned by `kxt`, even when the
bad field would eventually have been passed to a geometry constructor.

### Capability granularity

A capability is the smallest stable support-matrix unit useful for feature
queries, retry/fallback selection, or corpus metrics. It is not one capability
per error occurrence and not one catch-all per crate.

Class-pair capabilities should use stable geometry class keys from F1/F3 as
structured subject data rather than dynamically manufacturing identifiers.
For example, the fixed capability is
`kops.intersect.curve-curve.class-pair`; the requested class keys are separate
fields. This keeps the capability inventory finite and enumerable.

Each owning crate exposes its known capabilities in deterministic code order.
`XtCapability::ALL` is the precedent. Enum discriminants and Rust debug strings
are never persistence or ABI contracts.

## Public Rust failure contract

Do not replace every layer-local error with a monolithic `kcore::Error`.
Instead, expose a small common classification view:

```rust
#[derive(Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ErrorClass {
    InvalidInput,
    Unsupported,
    ResourceLimit,
    InvalidState,
    ModelRejected,
    InternalInvariant,
    Cancelled,
}

pub trait ClassifiedError {
    fn class(&self) -> ErrorClass;
    fn code(&self) -> ErrorCode;
    fn capability(&self) -> Option<CapabilityId> { None }
    fn limit(&self) -> Option<LimitSnapshot> { None }
}
```

`ClassifiedError` is intended for in-repository generic reporting and ABI
mapping; object safety is desirable but not required for the first migration.
Layer-local enums retain their useful payloads, source variants, and subject
types. `std::error::Error::source` is implemented when an error wraps another
layer.

`kcore::Error` remains `#[non_exhaustive]`. The first implementation adds
classification/code accessors to existing variants and replaces
`AlgorithmLimit` additively with a structured limit variant before deprecating
the old shape. Existing specific payloads such as offending coordinates and
tolerance growth remain available. Compatibility constructors can preserve old
variant behavior until downstream callers migrate.

The implemented ellipse/ellipse slice demonstrates the required wrapper shape.
`ProjectionError::{InvalidQueryPoint, InvalidWindow, NoCandidate,
NonFiniteEvaluation, Policy}` is retained as `IntersectionError::Projection`.
The concrete solver, generic intersection adapter, `GeometryIntersectionError`,
and `KernelError` delegate class, code, limit, and `capability() == None`, and
each exposes the exact lower error through `std::error::Error::source`.
`ProjectionError::Policy` additionally retains its `OperationPolicyError`
source. The direct `intersect_bounded_ellipses` API now returns
`IntersectionResult`; retaining the former `kcore::Result` shape would require a
lossy conversion for the four non-policy variants. This is a bounded migration,
not evidence that other solver-local `InvalidGeometry` collapses are closed.

New public code must not add another prose-only `InvalidGeometry` or
`AlgorithmLimit` site. During migration, old sites are catalogued and assigned
to a class before their representation changes.

## Outcomes, incompleteness, and limits

An indeterminate result is not an error merely because it is not complete.
Result types that can retain verified evidence expose structured incomplete
evidence:

```rust
#[derive(Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum IncompleteCause {
    Unsupported { capability: CapabilityId },
    Limit { snapshot: LimitSnapshot },
    NumericResolution,
    Cancelled,
    ProofMethodUnavailable { capability: CapabilityId },
}

pub struct IncompleteEvidence {
    pub code: DiagnosticCode,
    pub stage: StageId,
    pub cause: IncompleteCause,
    pub message: &'static str,
}
```

`Completion` remains the compact complete/indeterminate summary. Migration is
additive: intersection and other proof-bearing results gain
`incomplete_evidence()`, while `Completion::indeterminate_reason()` remains a
compatibility display accessor. Once all result owners carry evidence, the
prose field in `Completion::Indeterminate` can be deprecated in a major API
change. Operand swapping and canonicalization must preserve incomplete evidence
exactly, just as F0 now preserves completion.

F2's `LimitSnapshot` is used without duplication:

```rust
pub struct LimitSnapshot {
    pub stage: StageId,
    pub resource: ResourceKind,
    pub consumed: u64,
    pub allowed: u64,
}
```

F4 does not add another limit enum. A limit error contains the snapshot; an
indeterminate result's evidence refers to the same value; `OperationReport`
retains the complete deterministic usage ledger and diagnostics. The owning
stage documentation defines what one unit means. `consumed` is the attempted or
observed value that crossed the contract and may be greater than `allowed` by
the documented atomic charge size.

Unsupported and limit are therefore causes, while indeterminate is an outcome.
This distinction permits both of these valid APIs:

- dispatch with no applicable solver: `Err(Unsupported { ... })`;
- a fallback that verifies some branches but lacks a proof method for the rest:
  `Ok(partial evidence, Indeterminate(Unsupported { ... }))`.

No path may return `Completion::Complete` after silently dropping work because
of unsupported capability, cancellation, numerical resolution, or a limit.

## Checker and checked-transaction contract

The checker keeps its current report-first design:

- `FaultKind` identifies a proven model invariant violation;
- `VerificationGapKind` identifies the proof obligation;
- the gap additionally identifies why it remains unresolved; and
- `CheckOutcome` remains `Valid`, `Invalid`, or `Indeterminate`.

Add stable accessors rather than replacing the existing enums:

```rust
impl FaultKind {
    pub const fn code(self) -> ErrorCode;
}

impl VerificationGapKind {
    pub const fn code(self) -> DiagnosticCode;
}

pub enum VerificationGapCause {
    Capability(CapabilityId),
    Limit(LimitSnapshot),
    NumericResolution { stage: StageId },
}
```

`FaultKind::OpenLoop`, for example, maps to
`ktopo.check.fault.open-loop`. Fault codes use `ErrorCode` because they identify
proven violations, although a `Fault` is report data rather than an immediate
Rust `Err`.

Checked commit remains failure-atomic. A rejected candidate returns
`ErrorClass::ModelRejected` with code `ktopo.transaction.check-failed`; the
context-aware operation report or a transaction-specific error payload exposes
the deterministic checker findings. `fault_count` remains a compatibility
summary, not the only machine-readable evidence. Ordinary checking does not
turn every fault into a separate call failure.

An internal checker traversal error is not a model fault. For example, the root
body handle being stale is invalid input; a stale reference stored inside the
body is a model fault under the current checker contract; corruption of a
checker-owned index that disagrees after validated commit is an internal
invariant failure.

## Reconciliation with `kgraph::EvalError`

F1's `kgraph::EvalError` remains in `kgraph`. It may contain graph-only types
such as `GeometryRef`, `SurfaceHandle`, UV coordinates, class keys, and a cycle
path. None of those types move into `kcore`.

`EvalError` implements the common classification view:

| `EvalError` case | Class/cause | Shared data |
| --- | --- | --- |
| `StaleGeometryHandle`, `InvalidParameter`, `ParameterOutsideDomain` | Invalid input | graph-owned `ErrorCode`; subject remains local |
| `DependencyCycle` found in supplied/persisted graph | Invalid input or model-rejected at the graph-build boundary | graph-owned code; path remains local |
| `DependencyCycle` found after validated insertion | Internal invariant | same local detail, different boundary code |
| `DependencyDepthExceeded`, `NodeVisitLimitExceeded` | Resource limit | F2 `LimitSnapshot` with graph-owned `StageId` |
| `SingularSurface`, `IllConditionedSurface`, `NonFiniteResult` | Evaluation failure; usually an indeterminate numerical cause in a proof-bearing caller | graph-owned code/stage; surface/UV remain local |
| `DerivativeUnavailable` | Unsupported capability | `CapabilityId` plus local class and requested-order fields |

The two graph recursion limits use the shared resources and stable stages, for
example `kgraph.eval.dependency-depth` with `ResourceKind::Depth` and
`kgraph.eval.node-visits` with `ResourceKind::Work`. `EvalLimits` remains the
query-local reservation described by F1; the values are charged/reserved from
F2's operation scope. F4 standardizes the snapshot, not ownership of the graph
ledger or evaluator.

When `kops`, `ktopo`, or `kxt` wraps an `EvalError`, it retains the source and
delegates class, code, capability, and limit unless the higher-level boundary
changes the semantics. A proof-bearing operation may convert a singular local
evaluation into structured indeterminate evidence, but must retain its graph
code/stage in the operation report. No mapping is allowed to collapse
`EvalError` into `InvalidGeometry { reason }`.

## X_T mapping

`XtError` remains a layer-local enum because parse offsets, node indexes,
schema strings, and writer context do not belong in `kcore`.

Migration rules:

1. Existing `XtCapability::code()` strings are frozen. `XtCapability` gains a
   conversion to the shared `CapabilityId` and keeps `ALL` as the deterministic
   support inventory.
2. `BadHeader`, `Parse`, `MissingNode`, and `BadField` classify as invalid
   interchange input with stable `xt.*` error codes and retain their offsets or
   node indexes.
3. `UnsupportedSchema`, `UnknownNodeType`, and `Unsupported` classify as
   unsupported and expose their existing capability codes.
4. `InvalidModel` classifies as model-rejected for export and receives a stable
   code; its future detailed writer findings remain X_T-owned.
5. `Kernel(kcore::Error)` delegates all common classification accessors to the
   source. Add explicit source variants for graph/topology errors when needed;
   do not stringify or remap them to `InvalidModel`.
6. X_T reconstruction remains failure-atomic. Parse/reconstruction context may
   be attached to display and diagnostics without replacing the source code.

Corpus manifests record capability ID, error code, and stage/limit when
present. They do not record a display message as an expected classification.

## Eventual C ABI mapping

The C ABI uses a small fixed status enum for broad control flow and string IDs
for extensible detail. It must not assign one ABI enum member to every Rust
error or capability.

```c
typedef enum kernel_status_t {
    KERNEL_STATUS_OK = 0,
    KERNEL_STATUS_INVALID_INPUT,
    KERNEL_STATUS_UNSUPPORTED,
    KERNEL_STATUS_LIMIT_REACHED,
    KERNEL_STATUS_INVALID_STATE,
    KERNEL_STATUS_MODEL_REJECTED,
    KERNEL_STATUS_INTERNAL_ERROR,
    KERNEL_STATUS_CANCELLED
} kernel_status_t;
```

An operation-owned error/result record exposes:

- stable error-code UTF-8 string;
- optional capability-ID string;
- optional stage-ID string;
- optional resource kind, consumed, and allowed values;
- a non-stable human message;
- operation-specific subject data through typed result/report queries, not a
  generic pointer; and
- the full checker report or incomplete-evidence list when applicable.

Adding a new `ErrorCode` or `CapabilityId` does not change the ABI enum. Unknown
IDs remain safely classifiable by the broad status. Rust enum discriminants are
never cast into C values. The record's ownership/lifetime is explicit; the
design must not rely on parsing a process-global last-error string.

`KERNEL_STATUS_OK` does not imply proof completeness. Result APIs expose
completion/check outcome separately, so a successful call may return verified
partial evidence with an indeterminate status in its result record.

## Migration plan

### Phase 0 — Inventory and freeze

- Freeze current `XtCapability` strings and add uniqueness/prefix tests.
- Inventory every `InvalidGeometry` and `AlgorithmLimit` site by owning layer.
- Classify each site before changing representation. Ambiguous internal/public
  helpers are split or given boundary-specific adapters.
- Reserve initial code, capability, and stage constants in owner modules.

### Phase 1 — Shared identity and classification

- Land `ErrorCode`, `CapabilityId`, `ErrorClass`, and the classification view in
  `kcore` alongside F2's `StageId`/`LimitSnapshot` types.
- Add `class()` and `code()` to every existing `kcore::Error` variant without
  removing variants or payloads.
- Adapt `XtCapability`/`XtError` and add source delegation tests.

### Phase 2 — Representative vertical migrations

Status: typed unsupported class-pair dispatch and the ellipse/ellipse
projection-source vertical slice are implemented. Remaining solver-local and
legacy wrappers migrate only with owner tests that pin their exact public error
contract.

- Change unsupported curve/curve dispatch from `InvalidGeometry` to a typed
  unsupported error that carries the fixed pair capability and structured
  curve class keys from F3.
- Migrate one tessellation or refinement `AlgorithmLimit` path to a structured
  `LimitSnapshot` using the F2 stage constant.
- Give checker fault/gap kinds stable codes and migrate one bounded Full-check
  stop to `VerificationGapCause::Limit`.
- Preserve legacy wrappers and golden result bits.

### Phase 3 — Proof-bearing results and graph integration

Status: SSI and NURBS curve/curve solving carry structured incomplete evidence
through canonicalization, swapping, generic dispatch, and the kernel facade.
Curve-pair evidence retains exact isolation/seed limit snapshots, numeric and
method stops, and the stable complete-coverage capability in proof-pipeline
order. NURBS curve-pair polishing owns a stable six-code diagnostic
inventory for stationary, ill-conditioned, no-descent, parameter-resolution,
iteration-bound, and fallback observations. Its nested fallback minimizers own
a separate stable three-code inventory for parameter-resolution,
invalid-objective, and iteration-bound termination. Diagnostics remain bounded
and opt-in, while parameter-resolution stages remain always-on report evidence.
Other provisional result families still need the same migration before prose
completion reasons can retire.

- Add structured incomplete evidence to intersection result types and verify
  it survives swapping, canonicalization, and fallback routing.
- Implement classification for `kgraph::EvalError`; retain it as an error
  source through `kops`, `ktopo`, and `kxt`.
- Make context-aware operation reports the source of complete limit usage and
  deterministic diagnostics.

### Phase 4 — Owner-driven cleanup and facade enforcement

- Require all new production paths to use the stable taxonomy and migrate
  remaining prose-only invalid/limit call sites only with bounded owner-sized
  behavior changes.
- Deprecate legacy `AlgorithmLimit { operation, limit }` and prose-only
  completion reasons after all public consumers have structured accessors.
- Keep the implemented F5 facade error as an adapter over classified source
  errors; do not duplicate layer payloads.
- Defer the C record and ABI project until K5 adoption validates the native
  facade and a separately approved ABI contract exists.

Phase 4 is not a repository-wide cleanup campaign. Stable identifiers are
enforced for new work; legacy migrations are opportunistic and owner-driven.

## Required tests

Identifier tests:

- all known identifiers validate, are unique within their namespace, and are
  emitted in deterministic order;
- existing `xt.*` capability strings remain byte-for-byte unchanged;
- display-message changes do not affect code/class/capability accessors.

Classification tests:

- unsupported intersection class pairs are `Unsupported`, never
  `InvalidInput`;
- malformed ranges remain invalid input;
- a limit reports exact stage, resource, consumed, and allowed values;
- layer wrappers preserve source classification and structured data;
- all five ellipse/ellipse projection failures retain
  `IntersectionError::Projection`, exact class/code/limit, no capability, and
  the direct `ProjectionError` source through `GeometryIntersectionError` and
  `KernelError`; the policy case also retains `OperationPolicyError`;
- an internal invariant has no unsupported capability and is never reported as
  a model fault.

Outcome tests:

- complete empty remains distinct from indeterminate empty;
- swapping/canonicalization preserves structured incomplete evidence;
- budget exhaustion with verified partial evidence returns `Ok` plus
  indeterminate evidence, while an API without a partial contract returns a
  limit error;
- no limited, cancelled, unsupported, or numerically unresolved path reports
  `Completion::Complete`.

Checker/transaction tests:

- proven faults take precedence over gaps in `CheckOutcome` without deleting
  gap evidence already collected;
- capability, numerical, and limit causes can refer to the same gap kind;
- checked-commit rejection rolls back all mutations and exposes stable fault
  identities;
- stale root input and stale internal model reference retain their deliberately
  different classifications.

X_T and ABI adapter tests:

- unsupported schema and general-body content preserve existing capabilities;
- wrapped kernel and graph errors retain code/class/capability/limit data;
- every Rust class maps to the fixed C status, and unknown future codes map
  without ABI failure;
- a successful indeterminate result maps to `KERNEL_STATUS_OK` while exposing
  incomplete evidence separately.

## Non-goals

- One global enum containing every graph, topology, operation, and interchange
  failure.
- Stable display strings, localization, stack traces, or general logging.
- Encoding handles, UV values, node indexes, or other subject data into dotted
  identifiers.
- Treating unsupported work, missing proof, or deterministic limits as invalid
  geometry.
- Replacing checker reports with exceptions or returning one error per model
  fault.
- Moving `GeometryRef`, graph handles, checker entities, or X_T positions into
  `kcore`.
- Defining the final F5 facade or C memory-management API in this project.
- Changing numerical policy, default budgets, solver selection, or result bits.

## Acceptance criteria

The contract is ready to implement when all of the following are true:

1. Every public failure can be assigned a broad class and stable owner-defined
   code without parsing text.
2. Valid-but-unsupported input has a capability identity and is never mapped to
   invalid geometry.
3. F2's `StageId`, `ResourceKind`, and `LimitSnapshot` are the only shared limit
   vocabulary; reports and errors do not invent parallel structures.
4. Proof-bearing APIs keep verified partial evidence as indeterminate outcomes,
   and APIs without a sound partial contract return typed errors.
5. Checker faults, verification gaps, operation errors, and internal invariants
   remain four distinct concepts.
6. `kgraph::EvalError` retains graph-specific payloads in `kgraph` while
   exposing stable common classification through wrappers and source chains.
7. X_T preserves its current capability strings and delegates wrapped kernel
   classifications without loss.
8. The proposed C mapping needs no message parsing and can accept future codes
   without changing its broad status enum.
9. Representative intersection, limit, checker, X_T, and graph mapping tests
   cover the vertical contract before broad migration begins.

## Principal risks

- **Over-granular identifiers.** Generating a code for every pair or message
  creates an unmaintainable public registry. Fixed capabilities plus structured
  class keys keep the inventory bounded.
- **Double classification in wrappers.** Higher layers may accidentally replace
  a useful source classification with a generic local error. Delegation is the
  default; semantic translation must be explicit and tested.
- **Confusing stop cause with outcome.** Unsupported work and limits can cause
  either an error or an indeterminate result. The API's partial-evidence
  contract, not the cause alone, chooses the representation.
- **`kcore` dependency creep.** Shared wrappers must remain payload-agnostic.
  Graph paths, topology entities, class keys, and interchange locations stay in
  their owner crates.
- **Premature ABI freezing.** Only broad status values and dotted identifier
  semantics are fixed here. Operation-specific record layouts wait for F5 and
  the C API project.
