# F5 kernel facade and topology encapsulation

Status: K1-K3, typed K4 interchange and journal views, the first checked semantic K4 transaction, K5 adoption, and facade body tessellation implemented; broader K4 edits remain

## Outcome

Add one native Rust facade crate, `kernel`, as the supported application-facing
entry point. It owns session and part lifecycle, exposes typed operation
requests and outcomes, and provides read-only topology views without exposing
arena layout, entity structs, backlink vectors, graph descriptors, or mutable
assembly.

The facade is additive. Existing `kgeom`, `kgraph`, `ktopo`, `kops`, and `kxt`
entry points remain available to in-repository kernel work while clients
migrate. Those lower crates retain their layer-specific contracts; the facade
adapts them and does not become a second implementation of geometry,
transactions, numerical policy, errors, or interchange.

F5 deliberately does not define a C ABI. It establishes the semantic and
ownership seam from which a later ABI can be generated without exposing Rust
enum layout, arena slots, borrowed references, or a process-global last error.

## Current boundary and pressure

The existing low-level APIs already protect several important invariants:

- `Store` does not expose generic insertion or mutable entity borrows;
- `Transaction` rolls back on drop and only checked commits are public;
- `AssemblyStore` is scoped to an active transaction and cannot retain an
  unchecked candidate;
- primitive constructors and X_T reconstruction are failure-atomic and return
  deterministic journals; and
- generational handles make stale identity observable rather than aliasing a
  replacement entity.

The remaining coupling is conceptual and representational:

- public topology structs expose their parent links and stored child `Vec`s;
- generic `Store::get`, `iter`, `count`, and the `Entity` trait make arena
  membership part of a client's source-level vocabulary;
- applications must choose directly among `ktopo::make`, `Transaction`,
  checker, tessellation, `kops`, and `kxt` contracts;
- operation policy is currently passed piecemeal, and future contextual
  outcomes need a single top-level home;
- `AssemblyStore` is public because `kxt` is a sibling crate, even though it is
  not an ordinary modeling API; and
- F1 moves geometry ownership from topology arenas into `GeometryGraph`, which
  must not force application clients through a second identity migration.

The facade solves these problems by stabilizing concepts and behavior, not the
current storage representation.

## Crate placement and dependency rule

Add a workspace crate named `kernel` at the native API layer:

```text
kcore  kgeom
   \    /
    kgraph          geometry identity/evaluation (F1)
       | \
     ktopo kops     topology and operations
        \   /
         kxt        X_T interchange
          \ \
          kernel    supported native Rust facade
             |
          future kcapi / product / bindings
```

The exact lower-layer arrows continue to follow their existing Cargo rules;
the important F5 rule is that `kernel` is a terminal adapter:

- `kernel` may depend on `kcore`, `kgeom`, `kgraph`, `ktopo`, `kops`, and
  `kxt` as their relevant contracts land;
- no lower crate depends on `kernel`;
- operation implementations stay in their owning lower crate;
- X_T parse/reconstruction/emission stays in `kxt`;
- topology checking and transactions stay in `ktopo`; and
- F2 policy/report and F4 identifier/classification types are re-exported or
  adapted, never copied into new facade-owned equivalents.

The package name `kernel` is intentional: ordinary Rust clients should need
one direct dependency and should not need to understand the internal layer
names. Lower crates remain usable for kernel development and trusted adapters,
but the project makes no facade-level compatibility promise for their raw
entity layout.

## Lifecycle and ownership

The facade has three ownership levels:

```text
Kernel
  owns validated defaults and exposes an aggregated capability view
    |
    +-- creates Session
          owns Arc<SessionPolicy>
          owns zero or more independent PartState values
            |
            +-- each PartState owns exactly one ktopo::Store
                  and, after F1 G2, that Store owns its GeometryGraph
```

Conceptual types:

```rust
pub struct Kernel {
    default_policy: Arc<SessionPolicy>,
}

pub struct Session {
    policy: Arc<SessionPolicy>,
    parts: PartArena,
}

pub struct PartId {
    // private facade identity; no public slot/generation representation
}

struct PartState {
    store: ktopo::store::Store,
}

pub struct Part<'session> {
    session: &'session Session,
    id: PartId,
}

pub struct PartEdit<'session> {
    policy: &'session SessionPolicy,
    id: PartId,
    state: &'session mut PartState,
}
```

Normative ownership rules:

1. `Kernel` is a cheap factory/configuration root. It owns no model, graph,
   transaction, executor thread, or process-global mutable state.
2. `Session` is not `Clone`. It owns one immutable F2 `SessionPolicy`, shared
   by reference with all operations in that session.
3. A `PartState` is the unit of model ownership and failure-atomic mutation.
   Parts do not share topology or geometry handles. Cross-part references are
   rejected before reaching a lower-layer arena lookup.
4. `Part` and `PartEdit` are borrowed capabilities, not independently owned
   models. Dropping a session invalidates its parts naturally at the Rust
   boundary.
5. Only one mutable borrow of a part exists at a time. F5 does not add interior
   mutability or a global lock to bypass this rule.
6. Independent parts may later be operated on concurrently through a scoped
   session API, provided F2 deterministic execution and operation ordering are
   preserved. That concurrency API is not required for the first slice.
7. A part cannot be removed while borrowed. Part removal and future partition
   history get explicit operations; they are not side effects of dropping a
   view.

Initial construction is narrow:

```rust
impl Kernel {
    pub fn new() -> Self;
    pub fn with_default_policy(policy: SessionPolicy) -> Self;
    pub fn create_session(&self) -> Session;
}

impl Session {
    pub fn create_part(&mut self) -> PartId;
    pub fn part(&self, id: PartId) -> Result<Part<'_>>;
    pub fn edit_part(&mut self, id: PartId) -> Result<PartEdit<'_>>;
    pub fn policy(&self) -> &SessionPolicy;
}
```

`Kernel::new` uses F2's validated production v1 policy and Parasolid precision
regime. It does not hide arbitrary mutable defaults. A different policy means
constructing a different `Kernel`/`Session`, subject to F2 validation.
Capability enumeration delegates to the deterministic inventories owned by F4's
individual layers; `Kernel` may concatenate those views in documented layer
order but does not reassign or duplicate their identifiers.

## Facade identities

The facade exposes opaque, typed, part-qualified identities:

```rust
pub struct BodyId { /* private */ }
pub struct RegionId { /* private */ }
pub struct ShellId { /* private */ }
pub struct FaceId { /* private */ }
pub struct LoopId { /* private */ }
pub struct FinId { /* private */ }
pub struct EdgeId { /* private */ }
pub struct VertexId { /* private */ }

pub struct CurveId { /* private */ }
pub struct SurfaceId { /* private */ }
pub struct Curve2dId { /* private */ }
```

Each value carries enough private identity to reject use with the wrong part
and to resolve the corresponding lower-layer generational handle. Public
traits may include `Copy`, `Eq`, `Ord`, `Hash`, and `Debug`; the debug text,
size, field order, integer width, and any internal arena slot/generation are
not persistence or ABI contracts.

Facade IDs have no public constructor and no `From<u64>`. They are produced by
facade operations and views. Native persistence uses a future document-local
identity format, not the in-memory representation. A later C layer owns its
own tag registry and may map its tags to these IDs; it must not transmute them.

### Geometry and F1 G2

Raw `kgraph::CurveHandle`, `SurfaceHandle`, and `Curve2dHandle` are not
re-exported from `kernel`. The facade's opaque `CurveId`, `SurfaceId`, and
`Curve2dId` expose geometry identity without exposing graph storage.

After F1 G2:

- `PartState::store` remains the sole owner visible to F5;
- the store's embedded `GeometryGraph` resolves facade geometry IDs;
- topology faces/edges/fins and facade geometry views refer to the same graph
  nodes, so no facade mirror or copied leaf geometry exists;
- procedural dependencies remain graph-internal;
- `SurfaceView::class_key()` and analogous accessors expose F1 stable class
  keys, not Rust descriptor discriminants; and
- evaluation is a facade operation that derives a graph `EvalContext` from the
  active F2 operation scope. A view never constructs an uncharged evaluator.

Trusted lower-layer code may use graph handles directly. Ordinary facade
clients can create an offset by passing a facade `SurfaceId` in a typed request,
inspect its class/dependency summary through views, and evaluate it through an
operation, but cannot mutate a descriptor or walk arena internals.

## Read-only part and entity views

No facade method returns `&ktopo::entity::Body`, another raw entity reference,
`&Vec<_>`, `&Store`, `&GeometryGraph`, or a generic `get<T>` result. It returns
small read-only views whose methods express model semantics:

```rust
impl Part<'_> {
    pub fn bodies(&self) -> impl ExactSizeIterator<Item = BodyId> + '_;
    pub fn body(&self, id: BodyId) -> Result<BodyView<'_>>;
    pub fn face(&self, id: FaceId) -> Result<FaceView<'_>>;
    pub fn edge(&self, id: EdgeId) -> Result<EdgeView<'_>>;
    pub fn vertex(&self, id: VertexId) -> Result<VertexView<'_>>;
    pub fn surface(&self, id: SurfaceId) -> Result<SurfaceView<'_>>;
}

pub struct BodyView<'part> { /* private part borrow + id */ }

impl BodyView<'_> {
    pub fn id(&self) -> BodyId;
    pub fn kind(&self) -> BodyKind;
    pub fn regions(&self) -> impl ExactSizeIterator<Item = RegionId> + '_;
    pub fn faces(&self) -> Result<impl Iterator<Item = FaceId> + '_>;
    pub fn edges(&self) -> Result<impl Iterator<Item = EdgeId> + '_>;
    pub fn vertices(&self) -> Result<impl Iterator<Item = VertexId> + '_>;
}

impl FaceView<'_> {
    pub fn id(&self) -> FaceId;
    pub fn shell(&self) -> ShellId;
    pub fn loops(&self) -> impl ExactSizeIterator<Item = LoopId> + '_;
    pub fn surface(&self) -> SurfaceId;
    pub fn sense(&self) -> Sense;
    pub fn domain(&self) -> Option<FaceDomain>;
    pub fn tolerance(&self) -> Option<EntityTolerance>;
}
```

The exact return syntax may use named iterator structs where `impl Trait`
cannot satisfy public API or future binding needs. The semantic rules are:

- all collection traversal is deterministic and documented: part/entity slot
  order for global iteration, stored topological order for ownership, and
  current first-traversal order for deduplicated body edges/vertices;
- views expose IDs and immutable value objects, never mutable collection
  references;
- a stale, wrong-part, or wrong-kind ID is a classified error rather than
  `None`;
- empty topology is represented by an empty iterator, not a null view;
- optional modeled attributes remain `Option` only where absence is valid;
- a view's lifetime prevents it from surviving a mutable borrow of its part;
  and
- adding a cached index or changing an arena does not change view behavior or
  ordering.

Use named domain enums (`BodyKind`, `Sense`, `RegionKind`) and validated value
objects where they are already stable concepts. Do not expose facade copies of
raw entity structs merely to avoid writing accessors.

## Policy and operation-context construction

F5 consumes the F2 ownership model exactly:

```text
Session owns Arc<SessionPolicy>
  PartEdit receives typed request + operation settings
    constructs one borrowed OperationContext
      constructs one fresh OperationScope
        reserves child graph work
          constructs kgraph::EvalContext for that query
```

The facade re-exports the F2 types clients must configure, such as
`SessionPolicy`, `Tolerances`, `BudgetPlan`, `DiagnosticLevel`, and supported
execution settings. It does not define `FacadeTolerances`, `KernelBudget`, or
another numerical profile.

Each request contains or borrows an `OperationSettings` value whose fields are
inputs to F2's `OperationContext`:

```rust
pub struct OperationSettings<'a> {
    pub tolerances: Tolerances,
    pub budget_overrides: BudgetPlan,
    pub diagnostic_level: DiagnosticLevel,
    pub cancellation: Option<&'a dyn CancellationToken>,
}
```

If F2 lands an equivalent configuration type, `kernel` re-exports it under the
facade module instead of defining the sketch above. `SessionPolicy` supplies
precision, numerical recipes, execution policy, default budgets, and policy
version. Request settings cannot replace the session precision or introduce
proof-ineligible epsilons.

A top-level facade call creates exactly one `OperationScope`. Nested topology
checking, graph evaluation, tessellation, and intersections borrow that scope
or receive deterministic child reservations. They must not call a legacy
wrapper that creates a new default context and resets work accounting.

Simple, total reads such as `FaceView::sense()` do not create a context.
Iterative, recursive, procedural, diagnostic-bearing, or state-changing calls
do. This preserves F2's boundary without adding context parameters to trivial
queries.

## Typed requests and outcomes

Facade methods use named request and result types at operation seams. They do
not reproduce every internal pair-specific solver as a top-level method.

Representative construction API:

```rust
pub struct BlockRequest<'a> {
    pub frame: Frame,
    pub extents: [f64; 3],
    pub settings: OperationSettings<'a>,
}

pub struct BodyCreated {
    body: BodyId,
    journal: ChangeJournal,
}

impl PartEdit<'_> {
    pub fn create_block(&mut self, request: BlockRequest<'_>)
        -> OperationOutcome<BodyCreated>;
}
```

Representative query/check/intersection API:

```rust
pub struct CheckBodyRequest<'a> {
    pub body: BodyId,
    pub level: CheckLevel,
    pub settings: OperationSettings<'a>,
}

pub struct IntersectCurvesRequest {
    // private facade BoundedCurve operands and OperationSettings
}

impl Part<'_> {
    pub fn check_body(&self, request: CheckBodyRequest<'_>)
        -> OperationOutcome<CheckReport>;
    pub fn intersect_curves(&self, request: IntersectCurvesRequest)
        -> Result<OperationOutcome<CurveCurveIntersections>>;
}
```

`BoundedCurve` contains a facade `CurveId` and `ParamRange`; it never contains
a `&dyn Curve`, raw descriptor, or topology entity reference. Equivalent
bounded-surface requests use `SurfaceId` and explicit UV ranges.

Normative request/result rules:

1. Required inputs are fields, not positional boolean flags.
2. Extensible option groups are `#[non_exhaustive]` builders or validated
   values. Callers do not construct a struct with an open-ended collection of
   string settings.
3. Result identity is facade identity. Lower-layer handles are converted once
   at the boundary and remain part-qualified.
4. Proof-bearing lower-layer results retain `Completion` and F4 structured
   incomplete evidence. The facade never turns an indeterminate success into
   an error solely for convenience, and never turns it into a complete miss.
5. State-changing success returns its committed `ChangeJournal`; failure
   returns no candidate body or mutable partial state.
6. Default convenience methods may be added only as wrappers over the typed
   request with the versioned F2 v1 settings.
7. Pair-specific accelerators remain callable inside `kops`, but the facade
   exposes operation-family contracts so F3 can change dispatch without
   application API growth.

## Transactions, edits, and journals

Ordinary one-shot facade operations own their lower-layer transaction from
start through checked commit. Callers receive only the committed result and
journal.

F5 also exposes a narrow semantic edit transaction for workflows that must
compose multiple already-defined topology edits:

```rust
pub struct EditTransaction<'part> {
    inner: ktopo::transaction::Transaction<'part>,
    // facade part identity and validated OperationContext
}

impl PartEdit<'_> {
    pub fn begin_edit(
        &mut self,
        settings: OperationSettings,
    ) -> Result<EditTransaction<'_>>;
}

impl EditTransaction<'_> {
    pub fn split_face(&mut self, request: SplitFaceRequest) -> Result<SplitFaceResult>;
    pub fn merge_faces(&mut self, request: MergeFacesRequest) -> Result<()>;
    pub fn commit(self, roots: &[BodyId]) -> Result<OperationOutcome<ChangeJournal>>;
    pub fn rollback(self) -> Result<()>;
}
```

This wrapper exposes only semantic, pcurve-aware, journal-producing methods.
It does not expose `Store`, `Store::get_mut`, `AssemblyStore`, raw Euler
functions, or unchecked commit. Dropping it rolls back. Nested edit
transactions remain rejected until journal composition and partition history
have their own contract.

The implemented slice accepts existing bounded curve and pcurve IDs. Identity
mapping remains the constructor default, while a facade-owned validated affine
map supports shifted, scaled, and reversed authored pcurve parameters.
Facade-owned incidence values additionally carry integer-period chart shifts,
increasing-edge-parameter endpoint singularity markers, optional closed-use
winding, and an explicit lower/upper periodic seam role. Numeric chart input
must be finite, integral, and representable as `i32`; contextual validation
then rejects shifts on nonperiodic surfaces, invalid singular endpoints,
nonclosed winding, and seam roles without a full-period face boundary. The
committed range, affine map, and incidence metadata are inspectable through
`FinView`. The transaction validates part ownership and liveness before
mutation, inherits the source face carrier and sense, and commits only through
contextual affected-body validation. Drop, explicit rollback, and a denied
checked commit restore both candidate topology and exact future opaque
identities.

`ChangeJournal` is an owning adapter over `ktopo::transaction::Journal`, not a
copied facade journal schema:

```rust
pub struct ChangeJournal {
    inner: ktopo::transaction::Journal,
    part: PartId,
}

impl ChangeJournal {
    pub fn mutations(&self) -> impl ExactSizeIterator<Item = MutationView> + '_;
    pub fn lineage(&self) -> impl ExactSizeIterator<Item = LineageView<'_>> + '_;
    pub fn tolerance_budgets(&self)
        -> impl ExactSizeIterator<Item = ToleranceBudgetView> + '_;
    pub fn tolerance_events(&self)
        -> impl ExactSizeIterator<Item = ToleranceEventView> + '_;
}
```

Views map `EntityRef` to part-qualified facade IDs while preserving the
existing deterministic order and semantic split/merge/derived/replaced/deleted
events. Deleted IDs remain valid journal identities even though resolving them
as live entity views returns stale-handle. The adapter does not reinterpret
tolerance growth as F2 work usage; transaction-owned tolerance budgets remain
distinct and journaled as required by F2.

This adapter is implemented. Mutations retain arena-type/slot order, lineage
retains operation order and ordered split/merge members, and tolerance events
refer to declaration-ordered facade budget IDs. Topology and graph geometry use
their normal part-qualified IDs. Stored points remain outside the ordinary
geometry view surface, so journals use an opaque `JournalPointId` solely to
retain exact created/deleted identity without exposing its arena handle.

F5 does not add undo/redo, rollback marks, journal persistence, attributes, or
persistent naming. The journal adapter is deliberately sufficient for those
future layers to consume without learning arena layout.

## Interchange and raw assembly boundary

Ordinary interchange is a typed facade operation on a part:

```rust
pub struct ImportXtRequest<'a> {
    pub bytes: &'a [u8],
    pub settings: OperationSettings<'a>,
}

pub struct ImportXtResult {
    bodies: Vec<BodyId>,
    skipped: Vec<XtSkippedNode>,
    journal: ChangeJournal,
}

pub struct ExportXtRequest<'a> {
    pub body: BodyId,
    pub settings: OperationSettings<'a>,
}

impl PartEdit<'_> {
    pub fn import_xt(&mut self, request: ImportXtRequest<'_>)
        -> OperationOutcome<ImportXtResult>;
}

impl Part<'_> {
    pub fn export_xt(&self, request: ExportXtRequest<'_>)
        -> OperationOutcome<String>;
}
```

Import delegates to `kxt` and preserves its atomic reconstruction and journal.
Export delegates to the class-preserving writer. X_T parse offsets, node
indexes, skipped-node data, and capability subjects remain X_T-owned result or
error detail.

`ktopo::transaction::AssemblyStore` remains a lower-layer reconstruction tool
because sibling interchange crates need it. F5 applies these boundaries:

- `kernel` does not re-export `AssemblyStore`, `Store`, `Entity`, or raw entity
  constructors;
- facade transactions have no `assembly()` method;
- package/module documentation labels raw assembly as unstable and trusted
  interchange/kernel-builder infrastructure;
- all in-repository raw assembly clients must end in the existing checked
  commit and rollback tests; and
- new interchange formats may use raw assembly only in their lower-layer
  adapter, never through an application-provided callback.

Rust visibility across sibling crates means `AssemblyStore` may remain
technically public in `ktopo` during F5. This is not a promise to freeze it.
Moving it behind a sealed reconstruction protocol or a dedicated internal
crate is a later cleanup after all interchange consumers are known. Feature
gating alone is not treated as an access-control boundary because Cargo feature
unification can expose it.

## Error and report adaptation

F5 consumes F4's `ClassifiedError` vocabulary and F2's `OperationOutcome` and
`OperationReport`. It does not flatten layer errors into a string or define new
copies of `ErrorClass`, `ErrorCode`, `CapabilityId`, `StageId`, or
`LimitSnapshot`.

The facade error is an adapter that retains its source:

```rust
#[non_exhaustive]
pub enum ErrorSource {
    Core(kcore::error::Error),
    Graph(kgraph::EvalError),
    Interchange(kxt::XtError),
    // Add a topology/operation source only when that layer owns a distinct
    // public error type under F4.
}

pub struct KernelError {
    source: ErrorSource,
    context: ErrorContext,
}

impl ClassifiedError for KernelError {
    // Delegate class, code, capability, and limit to source unless the
    // facade itself rejected lifecycle/part identity before dispatch.
}
```

`ErrorContext` contains bounded facade-owned context such as operation family
and part identity. It does not replace the source's graph path, X_T offset,
topology report, or structured limit. `std::error::Error::source` retains the
chain. Facade-owned errors are limited to genuine facade contracts such as an
unknown part ID, a cross-part entity ID, or an operation attempted while the
part is already mutably borrowed through an external registry.

Contextual facade calls return F2's outcome shape, with the lower-layer result
mapped to facade IDs and errors mapped to `KernelError` while retaining the
same `OperationReport`:

```rust
pub type OperationOutcome<T> = kcore::operation::OperationOutcome<T, KernelError>;
```

The generic F2 outcome is the only report implementation. Lower-layer outcomes
map values and errors into facade identities with `map`/`map_err`, preserving
the exact report. `kernel` owns no ledger, report clone, diagnostic buffer, or
fallback report for failures that occur before an operation scope exists.

Rules:

- unsupported valid input remains `Unsupported`, not `InvalidInput`;
- checker faults remain a `CheckReport`; checked-commit rejection is
  `ModelRejected` and retains the report where the lower contract supplies it;
- graph evaluation detail remains `kgraph::EvalError` source data;
- partial verified intersections remain successful outcomes with structured
  incompleteness;
- operation and graph limits reuse F2 `LimitSnapshot` exactly;
- display text is non-stable and never drives facade branching; and
- no panic or internal invariant crosses the facade as unwinding behavior.
  Safe Rust entry points return the F4 classified internal error. The later C
  boundary additionally catches any forbidden unwind as a last-resort defect
  barrier.

## Future C ABI seam, without starting the ABI

The later C API should live in a separate terminal crate such as `kcapi` that
depends on `kernel`. F5 fixes only the semantic mapping:

- `Kernel`, `Session`, and part/entity IDs become opaque registry-owned C tags;
- typed request/result Rust structs inform versioned C option/result records,
  but their Rust memory layout is never reused;
- F4 broad `ErrorClass` maps to the fixed status enum, with stable string IDs
  and structured limit fields retained separately;
- successful calls expose completion/check outcome separately from status;
- result, report, journal, and error record ownership is per operation, never a
  process-global last-error string;
- iterators become snapshot/count/index or callback APIs only after their
  lifetime and reentrancy rules are designed; and
- C handles are validated for session, part, kind, generation, and liveness
  before lower-layer dispatch.

F5 does not choose C integer widths, allocation callbacks, string ownership,
thread-local behavior, option struct layouts, symbol names, tag reuse rules, or
calling convention. No F5 Rust type receives `repr(C)` merely because an ABI is
planned.

## Migration plan

Every phase is reviewable, behavior-preserving for existing lower-layer entry
points, and keeps the workspace green.

### K0 — Contract and API inventory

- Land this design and classify existing public entry points as facade,
  supported lower-level, or raw assembly/internal.
- Record deterministic ordering for every proposed view iterator.
- Inventory every place a future facade operation would accidentally call a
  legacy F2 wrapper and reset its operation scope.
- Define facade-owned codes only for session/part/wrong-part identity errors.

Exit: ownership, ordering, and error ownership are unambiguous; no source API
changes.

### Completed convergence gate — adoption before expansion

The facade was validated against a real consumer before graph-aware
intersection was added. The same gate remains mandatory before semantic edit
transactions, more operation families, or any ABI layer:

- migrate one in-repository tool/example to depend only on `kernel`;
- audit every lower-crate import or raw-field access that migration still
  requires;
- add semantic accessors in the owner crate rather than exposing raw facade
  escape hatches;
- verify the `kernel` package file list and facade-only examples are
  self-contained; and
- record API friction before stabilizing additional request/result types.

Compile-fail leakage guards and existing facade behavior remain frozen during
this adoption pass. Necessary additive accessors are allowed; speculative
operation families are not.

### K1 — Lifecycle, opaque IDs, and read views

- Add the `kernel` crate, `Kernel`, `Session`, `PartId`, `Part`, and `PartEdit`.
- Add private ID conversions and wrong-part validation.
- Add body/region/shell/face/loop/fin/edge/vertex read-only views and named
  deterministic iterators.
- Use current `Store` reads internally; do not re-export `Store` or raw entity
  types.
- Add facade-only compile-pass and compile-fail tests.

Exit: a client with only a `kernel` dependency can enumerate and inspect a
part, but cannot obtain or mutate a stored raw entity.

### K2 — F2/F4 contextual operation pilot

Status: implemented. Typed block construction, Fast/Full body checking, and
whole-body tessellation create one facade-owned context and scope, retain exact
reports and classified sources, adapt topology identities to opaque IDs, and
expose committed journals or immutable mesh values without raw topology.
Tessellation reuses the lower `TessOptions` quality contract, installs the
complete body family profile, calls the shared-scope entry directly, and maps
ordered face ranges and edge polylines to part-qualified facade identities.
The facade-only lifecycle consumer explicitly selects the corpus-backed
`BodyTessellationBudgetProfile::bounded_v1()` request override. Direct/facade
mesh bits and reports, repeated output, exact structural-item N/N-1 limits,
bounded-profile classification, invalid options, wrong-part precedence, and
private lower sources are pinned. Compatibility defaults remain unchanged.
That adoption advances only the legacy whole-body compatibility wrapper to
public retirement state 4; lower contextual and shared-scope integration APIs
remain supported implementation seams.

- Re-export the landed F2 configuration types and construct one scope per
  facade call.
- Add the F4-delegating `KernelError` adapter.
- Wrap one primitive constructor, body checker, and tessellation/query path in
  typed request/outcome APIs.
- Compare facade outputs, reports, journals, errors, and rollback state against
  their direct lower-layer equivalents.

Exit: one read operation and one state-changing operation prove the complete
policy/report/error adapter chain without duplicated context or taxonomy.

### K3 — F1 G2 geometry identity integration

Status: implemented. Opaque part-qualified curve, surface, and pcurve IDs; deterministic
geometry views; topology attachments; class metadata; and shared offset-basis
identity are implemented. Operation-scoped surface evaluation now reserves one
F2 child ledger, retains accepted/attempted graph work and classified sources,
and keeps graph limits, handles, evaluators, and descriptors private.
`Part::intersect_curves` resolves opaque graph-owned leaf curves without
copying descriptors, rejects wrong-part/stale identity before scope creation,
and delegates to the contextual generic `kops` dispatcher through one scope.
Facade-owned points, overlaps, contact/orientation values, operand identities,
and `Complete`/`Indeterminate` evidence prevent raw result leakage. Direct
ellipse/ellipse parity pins the exact report and smallest projection-limit
crossing. The composed curve/curve profile also carries certified NURBS pair
isolation work without exposing its stages as facade configuration types; the
facade preserves an adaptively proven empty result. An internal graph-owned
facade-boundary test also pins checked-ancestor clipped reversed overlap,
operand identity, exact Work/Items N admission, and isolated per-resource N-1
evidence. It uses internal construction and lower-layer stage identities; the
facade-only lifecycle client still exercises only an analytic graph-owned edge
pair.
Future procedural curve descriptors must add truthful graph child accounting
instead of retroactively charging today's leaf borrow.

This phase lands after F1 G2 or in the same integration window:

- add facade curve/surface/pcurve IDs backed by graph handles;
- add geometry class/metadata views over stable F1 class keys;
- adapt topology views to return facade geometry IDs;
- add operation-scoped evaluation and a graph-aware intersection request;
- verify that direct topology attachment and procedural dependencies resolve
  one graph identity with no facade copy.

Exit: moving geometry arenas into `GeometryGraph` causes no ordinary facade API
change, and an offset basis remains shared and hidden behind opaque IDs.

### K4 — Transactions, journals, and interchange

Status: typed X_T import/export now returns opaque body IDs, skipped-schema
summaries, deterministic text, classified source chains, and the exact opaque
commit journal. Import into populated parts is rollback- and allocator-clean.
`ChangeJournal` now exposes exact-size facade-ID iterators for net mutations,
all five semantic lineage forms, tolerance budgets, and tolerance events while
retaining deleted and point identities without raw handles. The first semantic
transaction composes pcurve-aware face split/merge with validated affine
parameter maps and facade-owned periodic-chart, seam, closed-use, and
singular-endpoint metadata through checked contextual commit without exposing
raw assembly.

Broader semantic edit surfaces resume after the K5 adoption pass. The
interchange facade stays thin: `kxt` reconstruction and checked-commit Fast
validation share one contextual graph child and return one truthful
facade report.

- Add the semantic `EditTransaction` wrapper over currently public checked
  transaction methods. **First canonical split/merge slice implemented.**
- Add `ChangeJournal` and its facade-ID iterators. **Implemented.**
- Add typed X_T import/export requests and preserve source error detail.
- Ensure raw assembly is not reachable through the facade.
- Add rollback tests at parse, graph construction, topology assembly, checking,
  and facade adaptation boundaries.

Exit: ordinary clients can construct, query, semantically edit, import, and
export a body without importing raw topology structs.

### K5 — Adoption and lower-layer encapsulation preparation

Status: implemented. This adoption gate remains the prerequisite evidence for
the remaining K3/K4 API expansion.

- Migrate at least one real product/tool/example path to a `kernel`-only direct
  dependency before attempting broad adoption.
- Add semantic accessors to `ktopo` wherever facade or interchange still reads
  public fields directly.
- Mark raw topology layout and assembly modules explicitly unstable in package
  documentation and stop adding external-style examples that construct entity
  structs.
- Audit public fields, generic `Entity` access, and raw assembly consumers.
- Make `cargo package --list` self-contained for facade tests/examples and
  inventory the pre-existing path-dependency version work required for full
  package verification.

Exit: the adopted path has only `kernel` as a direct kernel dependency and
completes a supported application lifecycle across construction or import,
semantic inspection, checking, surface evaluation, and X_T export. Every
lower-crate import discovered during migration is either removed through an
owner-provided semantic accessor or recorded as a named facade gap; it is not
papered over with a raw escape hatch. Record the friction before adding another
public operation family.

F5 does not remove or privatize existing public fields in this additive phase.
Once all in-repository consumers use accessors, a separately announced
low-level breaking release may make entity fields crate-private and replace
cross-crate raw assembly with a sealed reconstruction seam. That change is not
allowed to alter the `kernel` API or behavior. This sequencing preserves
existing APIs during F5 without accidentally promising that raw topology
layout is stable forever.

#### K5 adoption evidence and recorded friction

`examples/kernel-lifecycle` is a standalone, publish-disabled workspace
package whose only direct dependency is `kernel`. Its executed path constructs
a block, retains its committed-journal summary, traverses semantic topology,
performs a budgeted Full check, evaluates a supporting surface at the center of
its finite face domain, tessellates the body through the facade, intersects two
adjacent graph-owned edge curves,
exports X_T, imports into another part, checks and re-exports that body, proves
byte stability, and resolves the original opaque body ID after the unrelated
import. The example's structural test and
`scripts/package_contract.py` reject any new direct dependency, including a
development-only lower-layer dependency.

The adoption pass added only semantic polish exposed by that path:
`FaceDomain::center`, `SurfaceEvaluation::position`, and checker-finding
accessors. It did not add a facade operation family or a raw escape hatch.
Topology-owned getters now cover every production read of raw Body-through-
Vertex fields in the facade views and X_T writer. Public fields remain source
compatible during this additive phase.

The remaining pressure is explicit:

- `xt_inspect` intentionally mines transport nodes/schema and `xt_oracle`
  intentionally authors conformance fixtures, so neither is an ordinary
  application client;
- X_T reconstruction and oracle fixture authoring remain reviewed trusted raw
  assembly seams pending a separately announced sealed-reconstruction change;
- broader semantic edit families remain K4 work;
  facade journal iteration is implemented; and
- `cargo package -p kernel --list` is now an exact CI-reviewed inventory with
  the facade README and lifecycle tests, while full package creation remains
  blocked by the five versionless direct path dependencies (19 internal path
  edges across current workspace members after adding the publish-disabled
  example). Registry versioning/publication is a separate workstream.

The facade still re-exports selected F2/F1 value types such as derivative order
and derivative values rather than copying them into facade-owned mirrors. The
client audit found that acceptable for the current Rust boundary;
classification and opaque identity prevent storage or evaluator leakage.
Revisit those value types only with concrete semver pressure.

## Required tests

### Compile-pass facade tests

External test crates with only `kernel` as a direct dependency must compile
examples that:

- create `Kernel`, `Session`, and a part;
- create a block through a typed request and retain its journal;
- enumerate body faces/edges/vertices through deterministic views;
- intersect bounded graph-owned curves through opaque curve IDs;
- check and tessellate/export the body through operation outcomes;
- import X_T into an empty part and inspect returned bodies; and
- retain an opaque body ID across unrelated successful operations in the same
  part.

### Compile-fail boundary tests

Use `trybuild` or rustdoc `compile_fail` cases to prove that a facade-only
client cannot:

- construct or destructure a facade entity/geometry ID;
- obtain `&Store`, `&GeometryGraph`, `&Body`, or `&mut Face` from a part;
- mutate a body's regions, face loops, edge fins, or parent backlinks;
- call `AssemblyStore::add`, raw Euler operations, or unchecked commit through
  a facade transaction;
- keep a `BodyView` while acquiring `PartEdit` for the same session borrow;
- use an entity ID from one part in an operation on another; where Rust's type
  system cannot distinguish the runtime part, a runtime classified-error test
  covers it;
- construct an independent graph `EvalContext` from a facade part; or
- access the private lower-layer handle embedded by a facade ID.

Lower-layer tests also retain the existing compile-fail guarantees against
`Store::add`, `Store::get_mut`, and unchecked commit.

### Behavioral parity and atomicity tests

- Direct lower-layer and facade primitive construction produce identical
  topology, handle allocation order inside the part, journal events, and
  checker outcome under the F2 compatibility policy.
- Failed construction/import/edit leaves counts, live IDs, future allocation,
  graph dependencies, journal state, and operation report consistent with
  rollback.
- A wrong-part ID returns the stable facade code without consulting the other
  part's arena.
- Deleted journal identities remain reportable but cannot resolve as live
  views.
- View iteration is stable across repeated runs and unaffected by cached index
  rebuilds.
- F1 leaf and offset identities map one-to-one through topology and geometry
  views; no descriptor is copied into the facade.
- Contextual graph evaluation is charged once to the parent operation report.
- Contextual curve intersection preserves the direct lower-layer result and
  exact projection report, including classified limit snapshots.
- X_T wrapped errors retain class, code, capability, node/offset context, and
  limit data.
- Complete and indeterminate proof results survive facade mapping unchanged.

### API-shape regression tests

- Keep a reviewed `cargo public-api` or rustdoc-JSON snapshot for the `kernel`
  crate only. Lower-layer snapshots are informative, not the application
  stability contract.
- Reject new public facade items whose signatures mention `Store`,
  `AssemblyStore`, `Entity`, raw topology structs, graph handles/descriptors,
  `OperationScope`, or `EvalContext`.
- Reject public `repr(C)` facade types until the C ABI project explicitly owns
  them.
- Add uniqueness tests for facade-owned F4 codes and ensure lower-layer codes
  are delegated rather than renamed.

## Non-goals

- Freezing arena storage, handle bit layout, entity struct fields, backlink
  vectors, graph descriptors, or iterator implementation types.
- Replacing `ktopo::Store`, `Transaction`, or `GeometryGraph` with facade-owned
  mirrors.
- Moving F2 session/numerical/execution policy, ledger, diagnostics, or reports
  into `kernel`.
- Moving F4 layer-local errors into one monolithic facade enum or assigning one
  facade code to every lower-layer failure.
- Exposing raw graph handles, descriptor mutation, evaluator caches, dependency
  stacks, or topology consumers.
- Starting the C ABI, choosing its memory model, or adding unsafe code.
- General undo/redo, rollback marks, partition history, attribute propagation,
  persistent naming, or feature history.
- Cross-part topology/geometry references, assemblies, or automatic body moves.
- A plugin/custom geometry API or runtime class registry.
- Deprecating every pair-specific `kops` function or every current `ktopo` API
  during the facade rollout.
- Privatizing public topology fields before all in-repository raw consumers
  have semantic replacements and the low-level breaking change is explicit.

## Principal risks and mitigations

- **Facade becomes a second kernel.** Keep implementations in lower crates and
  require parity tests; facade code performs validation, context construction,
  identity mapping, and adaptation only.
- **Opaque IDs merely leak arena layout indirectly.** Keep fields/private
  representation unobservable, part-qualify every ID, and forbid numeric
  conversion or durable serialization of in-memory IDs.
- **Views calcify current backlinks.** Expose semantic ownership/traversal and
  documented order, not borrowed vectors or a generic entity getter.
- **Requests grow into option bags.** Use operation-family request types and
  validated shared F2 settings; stable named capability subjects replace
  arbitrary strings.
- **Nested calls reset budgets.** Construct one scope at the facade boundary
  and audit that all nested contextual paths borrow it rather than invoking
  legacy wrappers.
- **Error adaptation loses source detail.** Delegate F4 classification and
  retain source enums; never stringify a source into `InvalidGeometry`.
- **Assembly remains technically public.** Treat lower crates as unstable
  implementation APIs, do not re-export assembly, and require checked-commit
  rollback tests for every trusted consumer.
- **Session-owned parts complicate borrowing/concurrency.** Start with explicit
  `part`/`edit_part` borrows. Add scoped disjoint-part execution only after a
  measured use case and F2 deterministic parallel contract exist.
- **Field privacy breaks current consumers.** Land accessors and facade
  adoption first; schedule actual privacy as a separate low-level break that
  cannot affect the facade.
- **A premature ABI inference freezes Rust types.** Keep the C layer separate
  and prohibit `repr(C)` or tag transmutation in F5.

## Acceptance criteria

F5 is complete when all of the following hold:

1. A terminal `kernel` crate exists, and no lower crate depends on it.
2. `Kernel`, `Session`, and session-owned parts have explicit validated policy
   and model ownership with no process-global mutable state.
3. Public facade topology and geometry IDs are opaque, typed, part-qualified,
   stale-safe, and independent of arena/graph representation.
4. Ordinary clients can create, inspect, check, tessellate/interrogate, edit,
   import, and export a body with only the facade API.
5. Read-only views and iterators expose documented deterministic semantic
   order without returning raw entity structs or collection references.
6. Every contextual call constructs one F2 operation scope; nested graph,
   solver, checker, and tessellation work is charged to it exactly once.
7. Graph handles/descriptors stay F1-owned; topology and facade geometry IDs
   resolve the same graph nodes after G2 without copies or a second evaluator.
8. Facade errors delegate F4 class/code/capability/limit data and retain
   layer-local source detail; proof incompleteness remains a result property.
9. State-changing success returns a deterministic facade journal, and every
   failure path leaves the part, graph, future handle allocation, and journal
   state rollback-clean.
10. Facade edit transactions expose semantic checked operations only; raw
    assembly, unchecked mutation, and unchecked commit are unreachable.
11. Existing lower-layer public entry points and behavior remain available
    throughout the additive F5 migration; raw layout is documented as unstable
    and is not re-exported by the facade.
12. Compile-pass, compile-fail, parity, wrong-part, determinism, report/error
    retention, X_T atomicity, and graph identity tests described above pass.
13. A reviewed public-API guard rejects accidental exposure of `Store`, raw
    entities, assembly, graph handles, `OperationScope`, or `EvalContext`.
14. No C ABI, `repr(C)` facade representation, unsafe tag conversion, or
    process-global last-error mechanism is introduced by F5.
