# F4 kernel error and capability taxonomy

Give every public failure, unsupported feature, incomplete proof, and checker
finding a stable machine-readable identity without making human-readable text an
API. Preserve the distinctions already present in X_T capability codes and checker
reports, and make the ones missing from `kcore::Error` and `Completion` explicit.
This project defines contracts and a staged migration; it does not require a
flag-day conversion of every `InvalidGeometry { reason }` site, and it does not
flatten the layer-local enums into one giant enum. The central rule:

> An error says the requested call did not produce a usable value. An outcome says
> what was proven. A diagnostic explains how that outcome was reached. These are
> related but not interchangeable.

Status: Phase 1, representative Phase 2 slices, graph-evaluation and
intersection-certificate classification, one structured-incompleteness pilot, and
the projection and rigid-copy source-identity migrations are implemented;
remaining solver-local/legacy wrappers and proof-bearing families migrate only
with owner tests pinning their exact public error contract.

## Contract — normative classification

Every public non-success condition belongs to exactly one semantic class.
Classification is made by the owner of the public contract, not by matching a
message; the same numerical observation can have different classes at different
boundaries.

| Class | Meaning | Normal representation |
| --- | --- | --- |
| Invalid input | Caller supplied a value/request violating a documented precondition: non-finite parameter, invalid range, degenerate constructor input, or a stale handle where a live one is required. | `Err` with a stable error code and structured subject data where useful. |
| Unsupported capability | Request and model are valid, but this version does not implement the requested representation, class pair, derivative order, schema feature, or proof method. | `Err` when no useful result contract exists; otherwise verified partial evidence with an indeterminate cause naming the capability. |
| Indeterminate outcome | No contradiction was proven, but required proof obligations remain unresolved. Neither success nor invalid input. | A successful result/report carrying partial evidence and structured incomplete-proof data. |
| Resource limit | A deterministic configured work, item, byte, output, or depth allowance was reached. | `LimitSnapshot`; causes either an indeterminate partial outcome or `Err` per the API's partial-result contract. |
| Model invariant violation | A checker proved topology/geometry in a model violates a declared modeling invariant. | A checker `Fault` and `CheckOutcome::Invalid`; a checked transaction may surface a model-rejected error pointing to the retained report. |
| Invalid operation state | Request is meaningful in general but illegal in the current session/transaction state (nested or inactive transactions). | `Err`, distinct from invalid model data. |
| Internal invariant violation | Kernel-owned state or an algorithm-produced value violates an invariant callers could not be required to establish — a defect or corrupt persisted state. | Typed `Err` at safe Rust boundaries; never a panic across the facade or C ABI. |

Cancellation is an operation stop, not invalid input or a resource limit. F2 may
represent deterministic cancellation in diagnostics and the facade may add a
`Cancelled` class; it must never claim a complete result. Boundary examples: a
public `Line::new` with a zero direction is invalid input, but a solver
internally deriving a zero direction from supposedly regular data is an internal
invariant failure or an explicitly handled singular outcome; a valid class pair
with no registered solver is unsupported, not `InvalidGeometry`; a solver that
found verified contacts before exhausting candidate work returns them as
indeterminate with a `LimitSnapshot` (or a limit `Err` if it cannot expose partial
state); an open loop from `check_body_report` is a model fault, while inability to
certify a loop simple is a verification gap; a singular offset evaluation is an
evaluation failure with graph-local subject data, not proof the descriptor is
invalid everywhere.

## Contract — stable identifiers

F2's `StageId` remains the canonical stage identity and `DiagnosticCode` the
identity for observational diagnostics. F4 adds two payload-agnostic wrappers in
`kcore`, `const`-constructed and validated in tests, carrying no registry:

```rust
pub struct ErrorCode(&'static str);
pub struct CapabilityId(&'static str);
```

Rules: (1) lower-case ASCII dotted paths with digits/hyphens within a segment;
(2) first segment is the owning crate/layer (`kcore`, `kgeom`, `kgraph`, `ktopo`,
`kops`, `xt`); (3) an identifier is permanent once shipped — its message and Rust
variant name may change, its meaning may only narrow compatibly; (4) a
behaviorally incompatible replacement gets a new identifier (version suffix only
when two semantic versions must coexist, not for ordinary releases); (5) codes
identify a branch callers may act on and must not encode arena slots, handles,
parameter values, or source text; (6) display messages are not stable and must
never be parsed. The three namespaces answer different questions: `ErrorCode` —
why did this call fail; `CapabilityId` — which valid feature is unavailable;
`StageId` — where was work consumed or stopped. One error may carry all three
(e.g. code `kops.intersect.unsupported-class-pair`, capability
`kops.intersect.curve-curve.class-pair`, stage `kops.intersect.curve-curve.dispatch`).

## Contract — ownership by layer

Identifier constants live with the implementation and contract that owns their
meaning. `kcore` owns only wrappers, broad classes, and its own precondition/state
identifiers. Higher layers may delegate a lower-layer classification unchanged and
add a code only when their public contract adds meaning (wrapping an invalid
tolerance during X_T reconstruction keeps the `kcore` code; malformed node fields
are owned by `kxt`).

| Layer | Owns | Examples |
| --- | --- | --- |
| `kcore` | Identifier wrappers, broad error classes, F2 limit structures, arena/session/transaction-neutral base errors. | `kcore.input.invalid-tolerance`, `kcore.handle.stale` |
| `kgeom` | Leaf constructor, projection, tessellation, and numerical-stage codes/capabilities. | `kgeom.project.invalid-query-point`, `kgeom.project.no-candidate`, `kgeom.tess.triangle-output` |
| `kgraph` | Geometry dependency, graph build, descriptor, and evaluation codes/capabilities. | `kgraph.eval.dependency-cycle`, `kgraph.eval.derivative-order` |
| `ktopo` | Topology operation codes, checker fault/gap codes, model-invariant identities. | `ktopo.check.fault.open-loop`, `ktopo.check.gap.shell-self-intersection` |
| `kops` | Modeling-operation and intersection dispatch/solver codes and capabilities. | `kops.intersect.unsupported-class-pair`, `kops.intersect.ssi-proof-candidates` |
| `kxt` | X_T parsing, schema, reconstruction, emission codes and support-matrix capabilities; external capability strings keep the `xt.` prefix. | `xt.read.general-bodies`, `xt.parse.bad-field` |

**Capability granularity.** A capability is the smallest stable support-matrix
unit useful for feature queries, retry/fallback selection, or corpus metrics — not
one per error occurrence and not one catch-all per crate. Class-pair capabilities
use stable geometry class keys from F1/F3 as structured subject data (fixed
capability `kops.intersect.curve-curve.class-pair`; requested keys are separate
fields), keeping the inventory finite. Each crate exposes its capabilities in
deterministic code order (`XtCapability::ALL` is the precedent); enum discriminants
and debug strings are never persistence or ABI contracts.

## Contract — public Rust failure contract

Do not replace layer-local errors with a monolithic `kcore::Error`. Expose a small
common classification view; layer enums keep their payloads, source variants, and
subject types, and implement `std::error::Error::source` when wrapping another
layer.

```rust
#[non_exhaustive]
pub enum ErrorClass {
    InvalidInput, Unsupported, ResourceLimit, InvalidState, ModelRejected,
    InternalInvariant, Cancelled,
}
pub trait ClassifiedError {
    fn class(&self) -> ErrorClass;
    fn code(&self) -> ErrorCode;
    fn capability(&self) -> Option<CapabilityId> { None }
    fn limit(&self) -> Option<LimitSnapshot> { None }
}
```

`kcore::Error` stays `#[non_exhaustive]`; the migration adds classification/code
accessors to existing variants and replaces `AlgorithmLimit` additively with a
structured limit variant before deprecating the old shape, preserving specific
payloads (offending coordinates, tolerance growth). New public code must not add
another prose-only `InvalidGeometry` or `AlgorithmLimit` site; during migration,
old sites are catalogued and assigned a class before their representation changes.

## Contract — outcomes, incompleteness, and limits

An indeterminate result is not an error merely because it is incomplete. Result
types that retain verified evidence expose structured incomplete evidence:

```rust
#[non_exhaustive]
pub enum IncompleteCause {
    Unsupported { capability: CapabilityId },
    Limit { snapshot: LimitSnapshot },
    NumericResolution,
    Cancelled,
    ProofMethodUnavailable { capability: CapabilityId },
}
pub struct IncompleteEvidence { pub code: DiagnosticCode, pub stage: StageId, pub cause: IncompleteCause, pub message: &'static str }
```

`Completion` remains the compact complete/indeterminate summary; migration is
additive (`incomplete_evidence()` added, `indeterminate_reason()` kept as a
compatibility display accessor until all owners carry evidence). Operand swapping
and canonicalization must preserve incomplete evidence exactly. F2's
`LimitSnapshot { stage, resource, consumed, allowed }` is reused without a parallel
limit enum: a limit error contains the snapshot, an indeterminate result's evidence
refers to the same value, and `OperationReport` holds the complete usage ledger.
`consumed` is the attempted/observed value that crossed the contract and may exceed
`allowed` by the documented atomic charge size. Unsupported and limit are causes;
indeterminate is an outcome — so both `Err(Unsupported)` (no applicable solver) and
`Ok(partial evidence, Indeterminate(Unsupported))` (some branches verified, no
proof method for the rest) are valid. No path may return `Completion::Complete`
after silently dropping work for unsupported capability, cancellation, numerical
resolution, or a limit.

## Contract — checker and checked transactions

The checker keeps its report-first design: `FaultKind` identifies a proven
invariant violation, `VerificationGapKind` the proof obligation and why it is
unresolved, and `CheckOutcome` remains `Valid`/`Invalid`/`Indeterminate`. Add
stable accessors rather than replacing enums:

```rust
impl FaultKind { pub const fn code(self) -> ErrorCode; }            // e.g. ktopo.check.fault.open-loop
impl VerificationGapKind { pub const fn code(self) -> DiagnosticCode; }
pub enum VerificationGapCause { Capability(CapabilityId), Limit(LimitSnapshot), NumericResolution { stage: StageId } }
```

Fault codes use `ErrorCode` (they identify proven violations) even though a `Fault`
is report data, not an immediate `Err`. Checked commit stays failure-atomic: a
rejected candidate returns `ErrorClass::ModelRejected` with code
`ktopo.transaction.check-failed`, and the operation report or a transaction error
payload exposes deterministic findings; `fault_count` is a compatibility summary,
not the only machine-readable evidence. A stale root handle is invalid input; a
stale reference stored inside the body is a model fault; corruption of a
checker-owned index that disagrees after validated commit is an internal invariant
failure.

## Contract — kgraph::EvalError reconciliation

`kgraph::EvalError` stays in `kgraph` with graph-only types (`GeometryRef`,
`SurfaceHandle`, UV, class keys, cycle path); none move into `kcore`. It implements
the common classification view:

| `EvalError` case | Class/cause | Shared data |
| --- | --- | --- |
| `StaleGeometryHandle`, `InvalidParameter`, `ParameterOutsideDomain` | Invalid input | graph-owned `ErrorCode`; subject local |
| `DependencyCycle` in supplied/persisted graph | Invalid input or model-rejected at the graph-build boundary | graph-owned code; path local |
| `DependencyCycle` after validated insertion | Internal invariant | same detail, different boundary code |
| `DependencyDepthExceeded`, `NodeVisitLimitExceeded` | Resource limit | F2 `LimitSnapshot` with graph-owned `StageId` |
| `SingularSurface`, `IllConditionedSurface`, `NonFiniteResult` | Evaluation failure; usually an indeterminate numerical cause in a proof-bearing caller | graph-owned code/stage; surface/UV local |
| `DerivativeUnavailable` | Unsupported capability | `CapabilityId` plus local class/requested-order fields |

The two recursion limits use shared resources and stable stages
(`kgraph.eval.dependency-depth` with `ResourceKind::Depth`,
`kgraph.eval.node-visits` with `ResourceKind::Work`). When `kops`/`ktopo`/`kxt`
wraps an `EvalError` it retains the source and delegates class/code/capability/limit
unless the boundary changes semantics; a proof-bearing op may convert a singular
local evaluation into indeterminate evidence but must retain the graph code/stage
in the report. No mapping may collapse `EvalError` into `InvalidGeometry { reason }`.

## Contract — X_T and C ABI mapping

`XtError` stays a layer-local enum (parse offsets, node indexes, schema strings,
writer context do not belong in `kcore`). Rules: `XtCapability::code()` strings are
frozen and gain a conversion to `CapabilityId` while keeping `ALL` as the
deterministic inventory; `BadHeader`/`Parse`/`MissingNode`/`BadField` are invalid
interchange input with stable `xt.*` codes and retain offsets/node indexes;
`UnsupportedSchema`/`UnknownNodeType`/`Unsupported` are unsupported and expose their
capability codes; `InvalidModel` is model-rejected for export with a stable code;
`Kernel(kcore::Error)` delegates all classification accessors to the source (add
explicit graph/topology source variants when needed, never stringify or remap to
`InvalidModel`); reconstruction stays failure-atomic. Corpus manifests record
capability ID, error code, and stage/limit — never a display message.

The eventual C ABI uses a small fixed status enum for broad control flow plus
string IDs for extensible detail; it must not assign one ABI member per Rust error:

```c
typedef enum kernel_status_t {
    KERNEL_STATUS_OK = 0, KERNEL_STATUS_INVALID_INPUT, KERNEL_STATUS_UNSUPPORTED,
    KERNEL_STATUS_LIMIT_REACHED, KERNEL_STATUS_INVALID_STATE,
    KERNEL_STATUS_MODEL_REJECTED, KERNEL_STATUS_INTERNAL_ERROR, KERNEL_STATUS_CANCELLED
} kernel_status_t;
```

An operation-owned error/result record exposes a stable error-code string, optional
capability/stage-ID strings, optional resource kind/consumed/allowed, a non-stable
human message, operation-specific subject data through typed queries (not a generic
pointer), and the full checker report or incomplete-evidence list when applicable.
Adding a new `ErrorCode`/`CapabilityId` never changes the enum; unknown IDs stay
classifiable by broad status; Rust discriminants are never cast into C values; and
`KERNEL_STATUS_OK` does not imply proof completeness (a successful call may return
verified partial evidence with an indeterminate status in its record).

## Non-goals

- One global enum for every graph/topology/operation/interchange failure.
- Stable display strings, localization, stack traces, or general logging.
- Encoding handles, UV values, node indexes, or other subject data into identifiers.
- Treating unsupported work, missing proof, or deterministic limits as invalid
  geometry; replacing checker reports with exceptions or one error per model fault.
- Moving `GeometryRef`, graph handles, checker entities, or X_T positions into
  `kcore`.
- Defining the final F5 facade or C memory-management API here; changing numerical
  policy, default budgets, solver selection, or result bits.

## Acceptance

Every public failure has a broad class and stable owner-defined code without text
parsing; valid-but-unsupported input has a capability identity and is never mapped
to invalid geometry; F2's `StageId`/`ResourceKind`/`LimitSnapshot` are the only
shared limit vocabulary; proof-bearing APIs keep verified partial evidence as
indeterminate outcomes while APIs without a sound partial contract return typed
errors; checker faults, verification gaps, operation errors, and internal
invariants remain four distinct concepts; `kgraph::EvalError` keeps graph payloads
in `kgraph` while exposing common classification through wrappers/source chains;
X_T preserves its capability strings and delegates wrapped kernel classifications
without loss; and the C mapping needs no message parsing and accepts future codes
without changing its status enum.

## Evidence

- `crates/kops/tests/ellipse_ellipse.rs` (projection source-identity slice),
  `completion.rs`, `curve_curve.rs`, `nurbs_nurbs.rs`, `surface_surface.rs`
- `crates/kgraph/tests/intersection_curve_certificate.rs` (graph certificate leaf
  class/code/capability)
- `crates/ktopo/tests/body_copy.rs`, `transactions.rs`; `crates/kernel/tests/lifecycle.rs`
  (rigid-copy source chain through `BodyCopyError`/`KernelError::BodyCopy`)
- `crates/kxt/tests/read.rs`, `write.rs`, `corpus_manifest.rs`, `inspect_cli.rs`
  (frozen `xt.*` capabilities, wrapped-kernel delegation)
- `crates/kcore/tests/operation_context.rs`, `determinism.rs` (limit snapshots,
  identifier validation)

## Open items

- Migrate remaining solver-local `InvalidGeometry`/`AlgorithmLimit` sites (Phase 2)
  and add structured incomplete evidence to the remaining provisional result
  families (Phase 3) only with owner tests pinning the exact public contract and
  preserving legacy wrappers and golden bits.
- Phase 4 (owner-driven, not a repo-wide campaign): require new production paths to
  use the taxonomy; deprecate legacy `AlgorithmLimit { operation, limit }` and
  prose-only completion reasons only after all public consumers have structured
  accessors; keep the F5 facade error as an adapter over classified sources.
- Defer the C record and ABI project until K5 adoption validates the native facade
  and a separately approved ABI contract exists.
- **Risks:** over-granular identifiers (bound with fixed capabilities + class
  keys); double classification in wrappers (delegation is default; translation must
  be explicit and tested); confusing stop cause with outcome (the API's
  partial-evidence contract chooses the representation); `kcore` dependency creep
  (wrappers stay payload-agnostic); premature ABI freezing (only broad status and
  dotted-identifier semantics are fixed here).
