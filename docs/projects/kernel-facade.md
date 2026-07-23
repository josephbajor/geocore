# F5 kernel facade and topology encapsulation

Status: K1–K3, K5 adoption, and the landed K4 slices (typed X_T import/export, `ChangeJournal`,
checked semantic edits — MVFS/KVFS, MEV/KEV, MEF/KEF, KFMRH/MFKRH, face split/merge, bridge/ring —
rigid body copy with certificate reissuance, polygonal-profile extrusion, operation-owned tolerance
batching, block/cylinder construction, the opt-in Full-assurance commit gate, facade body tessellation, public typed block/block Boolean outcomes plus axial intersection, cylinder-minus-block remainder bands, ordered disconnected planar-minus-cylinder, and five-portal cap-retaining Unite/cylinder-left Subtract, zero-cut
truth-selected whole-source union/subtraction copies and both containment cavity orientations, one-ring axial cap-overlap connected union, one-ring axial block-minus-cylinder blind pockets, two-port axial through-holes, two-ring two-sided unions, support-separated exact-contact empty intersections, certified flush axial cap-contact connected unions, and exact zero-gap cylinder/cylinder Intersect/Subtract plus strict-internal and coincident-disk Unite) are implemented;
broader semantic edit families, transmitted-proof reissuance beyond the named families, partition
history, and the C ABI remain.

## Outcome

Add one native Rust facade crate, `kernel`, as the supported application-facing entry point. It owns
session and part lifecycle, exposes typed operation requests and outcomes, and provides read-only
topology views without exposing arena layout, entity structs, backlink vectors, graph descriptors,
or mutable assembly. The facade is additive: existing `kgeom`/`kgraph`/`ktopo`/`kops`/`kxt` entry
points remain for in-repo kernel work while clients migrate, and it adapts those crates rather than
reimplementing geometry, transactions, numerical policy, errors, or interchange. F5 deliberately
does not define a C ABI; it establishes the semantic/ownership seam from which a later ABI can be
generated without exposing Rust enum layout, arena slots, borrowed references, or a process-global
last error.

## Contract

### Crate placement and dependency rule

`kernel` is a terminal adapter workspace crate at the native API layer. It may depend on `kcore`,
`kgeom`, `kgraph`, `ktopo`, `kops`, and `kxt`; no lower crate depends on `kernel`. Operation
implementations stay in their owning lower crate; X_T parse/reconstruction/emission stays in `kxt`;
checking and transactions stay in `ktopo`; F2 policy/report and F4 identifier/classification types
are re-exported or adapted, never copied into facade-owned equivalents. Ordinary Rust clients need
one direct dependency and should not learn internal layer names; lower crates stay usable for kernel
development and trusted adapters but carry no facade-level compatibility promise for raw layout.

### Lifecycle and ownership

Three ownership levels: `Kernel` (cheap factory owning validated defaults; no model/graph/executor/
global mutable state) creates `Session` (not `Clone`; owns one immutable `Arc<SessionPolicy>`),
which owns zero or more independent `PartState` values (each owns exactly one `ktopo::Store`, and
after F1 G2 that Store owns its `GeometryGraph`). Rules: a `PartState` is the unit of model ownership
and failure-atomic mutation; parts share no handles and cross-part references are rejected before any
arena lookup; `Part`/`PartEdit` are borrowed capabilities, not owned models; only one mutable borrow
of a part exists at a time (no interior mutability or global lock added); a part cannot be removed
while borrowed, and removal/partition history are explicit operations, not drop side effects.
`Kernel::new` uses F2's validated production v1 policy and Parasolid regime; a different policy means
a different `Kernel`/`Session` subject to F2 validation. Capability enumeration concatenates F4
layers' deterministic inventories in documented order without reassigning identifiers.

### Facade identities

Body/Region/Shell/Face/Loop/Fin/Edge/Vertex and Curve/Surface/Curve2d IDs are opaque, typed, and
part-qualified. Each carries enough private identity to reject wrong-part use and resolve the lower
generational handle. Public traits may include `Copy`/`Eq`/`Ord`/`Hash`/`Debug`, but debug text,
size, field order, integer width, and any arena slot/generation are not persistence or ABI
contracts. IDs have no public constructor and no `From<u64>`; they are produced only by facade
operations/views. Native persistence uses a future document-local format, not the in-memory
representation; a later C layer maps its own tags to these IDs and must not transmute them. Raw
`kgraph` handles are not re-exported; `SurfaceView::class_key()` and peers expose stable F1 class
keys, not Rust discriminants; evaluation is a facade operation deriving a graph `EvalContext` from
the active F2 scope, never an uncharged evaluator.

### Read-only views

No facade method returns a raw entity reference, `&Vec<_>`, `&Store`, `&GeometryGraph`, or a generic
`get<T>`. Small read-only views (`BodyView`, `FaceView`, …) expose IDs and immutable value objects
with semantic accessors. Rules: all traversal is deterministic and documented (part/entity slot
order for global iteration, stored topological order for ownership, first-traversal order for
deduplicated body edges/vertices); a stale/wrong-part/wrong-kind ID is a classified error, not
`None`; empty topology is an empty iterator, not a null view; optional modeled attributes are
`Option` only where absence is valid; a view's lifetime prevents it surviving a mutable borrow of
its part; adding a cached index or changing an arena never changes view behavior or ordering. Use
named domain enums (`BodyKind`, `Sense`, `RegionKind`) and validated value objects; do not expose
facade copies of raw entity structs to avoid writing accessors.

### Policy and operation-context construction

F5 consumes the F2 ownership model exactly: `Session` owns `Arc<SessionPolicy>`; a request
carries/borrows `OperationSettings` (tolerances, budget overrides, diagnostic level, optional
cancellation token) that feed one borrowed `OperationContext`, which constructs one fresh
`OperationScope` per top-level facade call, which reserves child graph work and builds each
`kgraph::EvalContext`. The facade re-exports the F2 config types clients set (`SessionPolicy`,
`Tolerances`, `BudgetPlan`, `DiagnosticLevel`, execution settings) and defines no
`FacadeTolerances`/`KernelBudget`/numerical profile. Request settings cannot replace session
precision or introduce proof-ineligible epsilons. Nested checking, graph evaluation, tessellation,
and intersection borrow the one scope or take deterministic child reservations — never a legacy
wrapper that resets accounting. Simple total reads (e.g. `FaceView::sense()`) create no context;
iterative/recursive/procedural/diagnostic/state-changing calls do.

### Typed requests and outcomes

Facade methods use named request/result types at operation seams and do not reproduce every internal
pair solver as a top-level method. Rules: required inputs are fields, not positional flags;
extensible option groups are `#[non_exhaustive]` builders or validated values, never open string
bags; result identity is facade identity, converted once at the boundary and part-qualified;
proof-bearing results retain `Completion` and F4 structured incompleteness — the facade never turns
an indeterminate success into an error or a complete miss; state-changing success returns its
committed `ChangeJournal` while failure returns no candidate body or partial state; convenience
methods are wrappers over the typed request with versioned F2 v1 settings; pair-specific accelerators
stay inside `kops` while the facade exposes operation-family contracts so F3 can change dispatch
without application API growth. `BoundedCurve` holds a facade `CurveId` + `ParamRange`, never a
`&dyn Curve` or entity reference; bounded-surface requests use `SurfaceId` + explicit UV ranges.

### Transactions, edits, and journals

One-shot facade operations own their lower transaction from start through checked commit and return
only the committed result + journal. `EditTransaction` is a narrow semantic wrapper over currently
public checked transaction methods (grow tolerances, split/merge face, …, `commit`, `commit_full`,
`rollback`) exposing only semantic, pcurve-aware, journal-producing methods — never `Store`,
`Store::get_mut`, `AssemblyStore`, raw Euler functions, or unchecked commit. Dropping it rolls back;
nested edit transactions stay rejected until journal composition and partition history have their own
contract. Invariants for the landed edit families: every edit validates part ownership and liveness
before mutation and commits only through contextual affected-body validation; drop, explicit
rollback, and a denied checked commit restore both candidate topology and exact future opaque
identities. Position-owning seed/strut creation preflights surface/size-box/topology/curve/bounds/
pcurves before allocating its hidden point; facade KVFS/KEV inverses delete and journal a hidden
point only when unshared, while ordinary lower inverses retain external/shared point geometry.
Tolerance growth is one failure-atomic ordered batch: it validates every Face/Edge/Vertex target's
part + liveness before scalar/duplicate checks, requires unique targets and model-resolution-valid
final values, prepares imported-origin-preserving provenance + exact aggregate accounting before an
infallible apply, keeps request order, and returns a journal-local budget identity that is evidence
only (no edit method accepts it). Face split/merge tolerance propagation is journaled separately and
grants no reusable authoring authority: MEF copies the source face's `Option<EntityTolerance>` and
provenance unchanged; KEF selects the larger ordered `[surviving, absorbed]` value, resolving ties
toward the survivor and preserving exactness. `commit_full` is additive (Fast `commit` unchanged):
it validates explicit roots before scope creation, runs Fast graph validation once before borrowing
the same scope for Full proofs, always rejects Full faults, and under `RequireValid` also rejects any
proof gap while `AllowIndeterminate` may persist only a Fast-clean, fault-free candidate and returns
every gap unchanged; a rejection owns the same ordered reports but no journal. `ChangeJournal` is an
owning adapter over `ktopo::transaction::Journal` (not a copied schema) mapping `EntityRef` to
part-qualified facade IDs while preserving deterministic order and the five semantic lineage forms;
deleted IDs remain valid journal identities but resolve as stale when viewed live; stored points use
an opaque `JournalPointId`; the adapter never reinterprets tolerance growth as F2 work. F5 adds no
undo/redo, rollback marks, journal persistence, attributes, or persistent naming.

### Interchange and raw assembly boundary

Import/export are typed facade operations delegating to `kxt` and preserving its atomic
reconstruction, journal, and X_T-owned offset/node/skipped/capability detail. `kernel` does not
re-export `AssemblyStore`, `Store`, `Entity`, or raw entity constructors; facade transactions have
no `assembly()` method; raw assembly stays a lower-layer reconstruction tool documented as unstable
trusted interchange/kernel-builder infrastructure; all in-repo raw assembly clients must end in the
existing checked-commit and rollback tests; new interchange formats may use raw assembly only in
their lower-layer adapter, never through an application callback. `AssemblyStore` may remain
technically public in `ktopo` during F5 (Rust cross-crate visibility) but that is not a promise to
freeze it; feature gating is not treated as an access boundary.

### Error and report adaptation

F5 consumes F4's `ClassifiedError` and F2's `OperationOutcome`/`OperationReport`; it flattens no
layer error into a string and defines no copies of `ErrorClass`/`ErrorCode`/`CapabilityId`/`StageId`/
`LimitSnapshot`. `KernelError` is an adapter wrapping an `ErrorSource` enum (`Core`/`Graph`/`BodyCopy`/
`Interchange`, extended only when a layer owns a distinct public error under F4) plus bounded
`ErrorContext` (operation family, part identity) and delegates class/code/capability/limit to the
source unless the facade itself rejected lifecycle/part identity before dispatch; `std::error::Error::
source` retains the chain. `OperationOutcome<T> = kcore::operation::OperationOutcome<T, KernelError>`
is the only report implementation; lower outcomes `map`/`map_err` into facade identities preserving
the exact report, and `kernel` owns no ledger/report clone/diagnostic buffer/fallback report. Rules:
unsupported valid input stays `Unsupported`, not `InvalidInput`; checker faults stay a `CheckReport`
and checked-commit rejection is `ModelRejected` retaining the report; graph detail stays
`kgraph::EvalError` source data; partial verified intersections stay successful outcomes with
structured incompleteness; operation/graph limits reuse F2 `LimitSnapshot` exactly; display text is
non-stable and never drives branching; no panic or internal invariant crosses the facade as
unwinding — safe entry points return the F4 classified internal error, and the later C boundary
catches any forbidden unwind as a last-resort barrier.

### Future C ABI seam (not started in F5)

The later C API lives in a separate terminal crate (e.g. `kcapi`) depending on `kernel`. F5 fixes
only the semantic mapping: `Kernel`/`Session`/IDs become opaque registry-owned C tags; typed Rust
request/result structs inform versioned C records but their Rust layout is never reused; F4
`ErrorClass` maps to a fixed status enum with stable string IDs and separate structured limit fields;
completion/check outcome is exposed separately from status; result/report/journal/error ownership is
per operation, never a process-global last error; iterators become snapshot/count/index or callback
APIs only after lifetime/reentrancy design; C handles are validated for session/part/kind/generation/
liveness before dispatch. F5 chooses no C integer widths, allocation callbacks, string ownership,
thread-local behavior, struct layouts, symbol names, tag-reuse rules, or calling convention, and
gives no Rust type `repr(C)`.

### Migration phase exit contracts (K0–K5)

Every phase is reviewable, behavior-preserving for existing lower entry points, and keeps the
workspace green.

- **K0 (contract/inventory):** ownership, view-iterator ordering, and error ownership are
  unambiguous; facade-owned codes exist only for session/part/wrong-part identity; no source changes.
- **Convergence gate (adoption before expansion):** before semantic edit transactions, more operation
  families, or any ABI layer — migrate one in-repo tool/example to depend only on `kernel`, audit
  every lower-crate import/raw-field access it needs, add owner-crate semantic accessors rather than
  escape hatches, verify the package file list and facade-only examples are self-contained, and
  record API friction. Compile-fail leakage guards and existing behavior stay frozen; additive
  accessors allowed, speculative families not.
- **K1 (lifecycle/IDs/views):** a `kernel`-only client can enumerate and inspect a part but cannot
  obtain or mutate a stored raw entity.
- **K2 (F2/F4 operation pilot):** one read and one state-changing operation prove the full policy/
  report/error adapter chain without duplicated context or taxonomy.
- **K3 (F1 G2 geometry identity):** moving geometry arenas into `GeometryGraph` causes no ordinary
  facade API change, and an offset basis stays shared and hidden behind opaque IDs.
- **K4 (transactions/journals/interchange):** ordinary clients can construct, query, semantically
  edit, import, and export a body without importing raw topology structs. Rigid copy maps source
  coordinates through one orientation-preserving `Frame`, duplicates the complete topology+geometry
  closure, checked-commits inside the caller's single scope, gives every new identity deterministic
  `DerivedFrom` evidence, keeps pcurves/bounds/tolerances/offset-bases/periodic metadata exact,
  reruns the whole-range family certifier before insertion, protects copied roots and complete basis
  closures from transitive removal, keeps certificate-reissuance failures typed through
  `BodyCopyError`/`KernelError::BodyCopy` (never `Core(InvalidGeometry)`), and rejects unsupported
  proof families as a stable `Unsupported` capability at facade preflight before scope creation.
  Facade preflight admits only the currently verified certificate families (direct/safe-offset
  PlaneLine and PlaneSphereCircle; operation-generated VerifiedNurbsIntersection families; the named
  transmitted charts); broader depth/binding is graph work, not facade-copy preflight. Extrusion
  builders own exact cap/side topology, line pcurves on every use, shared vertical/perimeter edges,
  failure-atomic checked creation, and the exact prism proof consumed by Full checking;
  `extrude_profile_along` classifies the stored-frame scalar triple with `orient3d` (exact
  coplanarity rejects before allocation; the negative path reflects profile chart + reference normal
  together so model-space points/translation stay unchanged and cap/fin orientation stays canonical).
- **K5 (adoption + encapsulation prep):** the adopted path has only `kernel` as a direct kernel
  dependency and completes a supported lifecycle (construct/import, semantic inspection, checking,
  surface evaluation, X_T export); every lower-crate import found during migration is removed via an
  owner semantic accessor or recorded as a named facade gap, never papered over with a raw escape
  hatch. F5 does not privatize existing public fields; a later separately announced low-level break
  may make entity fields crate-private and replace cross-crate raw assembly with a sealed
  reconstruction seam without altering the `kernel` API or behavior.

### Non-goals

Freezing arena storage, handle bit layout, entity fields, backlinks, graph descriptors, or iterator
types; replacing `ktopo::Store`/`Transaction`/`GeometryGraph` with facade mirrors; moving F2 policy/
ledger/diagnostics/reports or F4 layer-local errors into `kernel`; exposing raw graph handles,
descriptor mutation, evaluator caches, dependency stacks, or topology consumers; starting the C ABI,
choosing its memory model, or adding unsafe code; general undo/redo, rollback marks, partition
history, attribute propagation, persistent naming, or feature history; cross-part references,
assemblies, or automatic body moves; a plugin/custom-geometry API or runtime class registry;
deprecating every pair-specific `kops` function or `ktopo` API during rollout; privatizing public
topology fields before all in-repo raw consumers have semantic replacements and the low-level break
is explicit.

### Acceptance criteria

F5 is complete when: a terminal `kernel` crate exists with no lower crate depending on it;
`Kernel`/`Session`/parts have explicit validated policy and model ownership with no global mutable
state; facade topology/geometry IDs are opaque, typed, part-qualified, stale-safe, and
representation-independent; ordinary clients can create, inspect, check, tessellate/interrogate,
edit, import, and export a body with only the facade; read-only views/iterators expose documented
deterministic order without raw entities or collection refs; every contextual call constructs one F2
scope charging nested graph/solver/checker/tessellation work exactly once; graph handles/descriptors
stay F1-owned and topology + facade geometry IDs resolve the same graph nodes after G2 without copies
or a second evaluator; facade errors delegate F4 class/code/capability/limit and retain layer-local
source, with proof incompleteness a result property; state-changing success returns a deterministic
journal and every failure leaves part/graph/future-handle/journal state rollback-clean; edit
transactions expose semantic checked operations only, with raw assembly, unchecked mutation, and
unchecked commit unreachable; existing lower entry points and behavior remain throughout the additive
migration with raw layout documented unstable and not re-exported; the compile-pass/compile-fail/
parity/wrong-part/determinism/report-error-retention/X_T-atomicity/graph-identity tests pass; a
reviewed public-API guard rejects accidental exposure of `Store`, raw entities, assembly, graph
handles, `OperationScope`, or `EvalContext`; and no C ABI, `repr(C)` facade type, unsafe tag
conversion, or process-global last error is introduced.

## Evidence

Compile-fail `trybuild`/rustdoc cases prove a facade-only client cannot destructure an ID, obtain
`&Store`/`&GeometryGraph`/`&Body`/`&mut Face`, mutate backlinks, reach `AssemblyStore::add`/raw
Euler/unchecked commit, hold a `BodyView` while acquiring `PartEdit`, use a cross-part ID, or build
an independent `EvalContext`. A reviewed `cargo public-api`/rustdoc-JSON snapshot rejects new facade
signatures naming `Store`, `AssemblyStore`, `Entity`, raw entities, graph handles/descriptors,
`OperationScope`, `EvalContext`, or `repr(C)`.

- Facade crate + lifecycle/adoption: `crates/kernel/src/{lib,session,operation,primitive,properties,edit,error,id,interchange,intersection,iter,tessellation}.rs`, `crates/kernel/src/view/`, `crates/kernel/tests/lifecycle.rs`, `examples/kernel-lifecycle`, `scripts/package_contract.py`.
- Rigid copy + certificate reissuance: `crates/ktopo/tests/body_copy.rs`, `crates/kgraph/tests/{intersection_curve_certificate,transmitted_plane_offset_nurbs}.rs`.
- Transmitted-proof / interchange atomicity: `crates/kxt/tests/{finite_open_two_sample_dual_offset,finite_open_cubic_dual_offset,finite_open_five_sample_dual_offset,finite_open_seven_sample_dual_offset,offset_nurbs_intersection,intersection_chart}.rs`.
- Edit / journal / tolerance families: `crates/ktopo/tests/{euler_transactions,builders,tolerance_budgets,transactions,pcurves}.rs`.
- Contextual intersection + ellipse source retention: `crates/kops/tests/{ellipse_ellipse,curve_curve}.rs`; legacy-boundary audit `scripts/legacy_api_contract.py`.
- Public Boolean facade + external operation evidence: `crates/kernel/src/boolean/`,
  `crates/kernel/tests/lifecycle.rs`, `crates/kernel/examples/boolean_xt_oracle.rs`,
  `docs/oracle-boolean-certification.json`.
- Exact planar-shell admission is independent of its optional typed convex certificate; the complete
  non-convex ten-support star/cylinder Intersect Full-commits at 17F/45E/30V with literal-derived volume and deterministic X_T/Fast self-import.

## Open items

- Broader semantic edit families beyond the landed MVFS/KVFS, MEV/KEV, MEF/KEF, KFMRH/MFKRH,
  split/merge, bridge/ring, and rigid copy; nested edit transactions pending a journal-composition +
  partition-history contract; tolerance-propagation policies beyond MEF inheritance / KEF ordered max.
- Public `body_properties`, `body_distance`, and `body_clash` retain Full proof for exact Plane/Cylinder solids; parallel-cylinder regularized CSG retains certified internal-tangency families and exact realization/proof/property frontiers. Exact nonparallel Cylinder/Cylinder Section now publishes root-free Whole sheets and bounded non-wrapping Open spans. Each bounded procedural end exposes exact cap-ring/root provenance separately from its guarded residual point; four spans join four topology-clipped cap rulings into two closed components across world/oblique frames, replay, and swap. Graph and Section N/N-1 work and malformed-final-endpoint refusal are atomic. Evidence: `two_axial_bounds_publish_four_nonwrapping_upper_spans`, `bounded_skew_spans_close_with_cap_rulings_and_topology_owned_roots`, `bounded_skew_section_work_accepts_n_and_refuses_n_minus_one_atomically`, and `malformed_final_open_endpoint_refuses_the_face_pair_without_a_prefix`. Contact/corner roots, seam walks, unsafe arithmetic, persistence, Boolean consumption, and pinched containing-minus-contained remain typed gaps. Next: consume bounded procedural components in mixed face arrangements and regularized skew Cylinder/Cylinder CSG.
- Rigid-copy preflight gaps: altered/witness-mismatched higher-order charts, graph-valid shared-basis
  or periodic-source/carrier proofs, other-sample dual offsets, nested five-/seven-sample roots, and
  Offset(Plane)-peer transmitted proofs; graph trace representation/binding beyond four-descriptor
  two-sample/quadratic/cubic chains; broader transmitted-proof reissuance.
- Non-rigid transform families and attribute-carrying copy (needs an authorable storage contract)
  remain follow-on.
- F5 exposes body section graphs, verified Plane/Cylinder circle and finite exact-family transverse ruling
  branches, and exact endpoint-free `SectionRing` components when topology-owned trims retain the
  whole period. `SectionCurveFragment` exposes exact bounded arcs and topology-clipped line segments
  with operation-shared source-root endpoint identity and topology provenance. An admitted exactly parallel or antiparallel strict-secant Cylinder/Cylinder pair with strict positive axial overlap and two uniquely owned ends yields two rulings plus two cap arcs in one topology-owned closed component; nested and partial-height world/oblique replay and swap are green. Public partial-overlap Intersect and both ordered Subtract meanings join the nested slices through shared arrangements, alternating periodic graph authority, and structural shell proofs. Proof-keyed disk-cap chords and one certified cap arc feed exact disk arrangements with dual classification and period-lifted source-arc lineage, including a lifted simple arc crossing the pcurve seam. Certified maximal transverse annulus traces and proper laminar boundary-returning traces retain exact source-loop roots, canonical/lifted UV, integer chart shifts, and universal-cover noncrossing/winding proof, while semantic Plane supports certify rounded-frame source incidence. Exact planar-circle and periodic-cylinder face-region
  partitions now propagate open-cell occupancy over their proof-bearing dual graph. Exact nested line-cycle planar cells and selected disk/trace cells share finite source-arc planning/materialization; count-independent chord-portal proof now Full-certifies cap-crossing Unite/block-minus-cylinder at 9F/18E/12V, while Intersect/cylinder-minus-block commit at 4F/6E/4V across both rigid frames/orders with deterministic X_T/Fast self-import. Exact retained source/carrier scalars, semantic sloped-support recovery, and harmonic support bounds materialize selected convex multi-chart bounded-arc cells through five supports; ordered planar-minus-cylinder atomically Full-commits every disconnected rectangular/three-sided component across four frames. Endpoint-free cap truth/planning/incidence/materialization and count-independent portal-shell proof Full-commit rectangular/five-support cap-retaining Unite and cylinder-left Subtract across those frames with deterministic X_T, exact topology/volume, and N/N-1 shell-work evidence. There is no general surface/surface entry point; future adapters must preserve the
  lower `kops` completion boundary/evidence, not duplicate or widen it.
- Encapsulation follow-on (separate announced low-level break): make entity fields crate-private and
  replace cross-crate raw assembly with a sealed reconstruction seam; `cargo package -p kernel` full
  creation still blocked by versionless direct path dependencies; `xt_inspect`/`xt_oracle` stay
  reviewed trusted raw-assembly seams, not ordinary clients.
- Future C ABI (`kcapi`) and scoped disjoint-part concurrency remain unstarted.
