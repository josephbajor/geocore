# Kernel Construction Roadmap

Companion to [kernel-spec.md](kernel-spec.md). The specification defines the target
contract; this document defines milestone dependency order, current evidence, and the
gates that prevent a locally successful prototype from being mistaken for a conformant
CAD kernel.

For handoff and day-to-day sequencing,
[`projects/foundation-projects.md`](projects/foundation-projects.md) is the
single authoritative current queue. This roadmap owns milestone contracts,
dependency rationale, and long-horizon evidence gates; sections below that
describe milestone backlog do not independently reorder active foundation work.

Effort calibration, stated honestly: Parasolid represents decades of engineering and
millions of lines of code. A small focused team may reach a useful analytic modeling
kernel in roughly a year and a credible NURBS/blend kernel in multiple years. Commit
velocity is not a substitute for corpus coverage, independent-oracle validation, or
end-to-end modeling success.

## Status semantics

Milestone status is evidence-based:

- **IMPLEMENTED SLICE** — the stated subset exists and passes its local tests. It does
  not imply that the corresponding layer in the specification is complete.
- **IN PROGRESS** — implementation is active, but one or more exit criteria are open.
- **GATED** — work must not become the primary implementation path until its named
  prerequisites are complete. Experiments may continue behind explicitly provisional
  APIs.
- **NOT STARTED** — no end-to-end implementation exists.
- **CONFORMANT** — the full milestone contract, corpus gate, independent-oracle gate,
  robustness gate, and applicable performance gate all pass.

No milestone is called complete merely because its happy-path unit tests pass. A
modeling operation is complete only when it is failure-atomic, journaled, checker-clean,
deterministic, corpus-tested, and explicit about unsupported or indeterminate cases.

## Dependency spine

```text
M0–M2 implemented foundation slices
                 │
                 ▼
       M2.5 architecture gate ──────┐
                 │                  │
                 ▼                  ▼
       M4 certified intersections   M3 XT corpus + interop
                 │                  │
                 └────────┬─────────┘
                          ▼
              M5 analytic booleans
                          ▼
       M6 general modeling + sewing/STEP
                          ▼
          M7 blends/offsets/shelling
                          ▼
               M8 API and hardening
```

M3 continues in parallel because real X_T files are test infrastructure. M2.5 is a hard
gate for the *public contracts* of general intersection and modeling operations: more
analytic experiments may land, but they must not lock in topology or SSI representations
that cannot carry pcurves, tolerances, completion evidence, and journals.

## Current status snapshot

| Milestone | Status | What the status means |
|---|---|---|
| M0 Foundations | IMPLEMENTED SLICE | Deterministic math, current predicates, intervals, tolerances, arenas with copy-on-write undo frames, and deterministic map primitives exist; conformance debt remains. |
| M1 Geometry | IMPLEMENTED SLICE | Analytic geometry, clamped NURBS evaluation plus exact curve/surface splitting, restriction, Bezier extraction and active-subrange bounds, projection, and tessellation exist; periodic/procedural and several full NURBS capabilities remain. |
| M2 Topology | IMPLEMENTED SLICE | Core hierarchy, topology-internal Euler operators, transaction-owned public Euler edits, primitives, the structural/sampled Fast checker, checker-v2 Full reporting, watertight body tessellation, checked transaction-scoped assembly, and deterministic journals exist; general bodies and several degenerate topology classes remain. |
| M2.5 Architecture gate | IN PROGRESS / REQUIRED | Per-fin pcurves with integer-period chart shifts, paired seam-edge roles, closed-use winding, and singular endpoint markers; bounded curve-less tolerant edges; typed entity-tolerance origin/growth provenance and transaction-owned aggregate budgets; shared incidence validation; a complete transaction-owned public Euler surface with mandatory pcurve creation and derived/split/merge/delete lineage; private generic Store mutation; transaction-scoped low-level assembly whose only public persistence path uses deterministic mutation preview, incrementally replaced per-body ownership/shared-geometry dependency footprints, affected-root Fast checks, and complete ownership closure; pcurve-driven tessellation; deterministic mutation/lineage/tolerance journals; failure-atomic journaled solid/sheet/wire/acorn constructors; a reusable validated simple-polygon planar profile; checked X_T reconstruction; explicit face metadata; certified imported domains; adaptive full-active-interval analytic/clamped-NURBS face-domain containment; explicit `Fast`/`Full` checker reports with `Valid`/`Invalid`/`Indeterminate` outcomes; whole-interval affine/harmonic incidence certificates; robust planar-segment/simple-ring loop proofs; and convex-planar, whole sphere/torus, sphere-cap, and single-planar-face shell embedding proofs have landed. General NURBS/mixed-parameter incidence, periodic/unclamped and unsupported exact/mixed-boundary containment, profiles with holes/curves, operation-specific tolerance propagation rules, curved-loop/general curved-shell proofs, production seam/singularity interchange fixtures, geometry graph, higher-operation migration, and multi-body performance baselines remain. |
| M3 X_T | IN PROGRESS | The modern-schema subset reads both wire encodings and writes text, including bounded tolerant edges as trimmed SP-curves over finite 2D B-curves; production coverage and external certification remain. |
| M4 Intersections/profile ops | PROVISIONAL / GATED | Broad analytic special cases, explicit `Complete`/`Indeterminate` result evidence, exact NURBS patch subdivision/BVH, analytic implicit-surface exclusion, deterministic recursive candidate covers with structured limits and proof-bearing SSI miss exits, and sampled NURBS experiments exist; certified root discovery and boolean-ready paired-pcurve branches do not. |
| M5–M8 | NOT STARTED | No end-to-end booleans, general modeling, blends, stable API, or production hardening. |

The machine-readable companion [kernel-support.tsv](kernel-support.tsv) is the capability
ledger. A change updates the ledger only when the named exit evidence lands; file count,
test count, or a new special-case solver is not by itself a status change.

## Reconciled critical path

The architecture is directionally correct: pure geometry, handle-based B-rep topology,
per-incidence pcurves, checked Euler edits, transactions/journals, and interchange as a
parallel corpus source are the right layer boundaries. The main risk is breadth outrunning
proof-bearing contracts. Work therefore advances through these gates in order:

| Order | Delivery tranche | Required result | What it unlocks |
|---|---|---|---|
| 1 | Close M2.5 topology contracts | Production seam/pole/apex interchange fixtures; operation-specific rules over the landed tolerance provenance/budgets; multi-body performance baselines for the landed incremental affected-root index; and discharge the remaining checker-v2 `Full` proof gaps with adaptive incidence, curved-loop/multi-loop containment, and shell proofs. Full-active-interval face-domain containment, private/checked mutation, transaction lineage, affected-root selection, and per-body incremental indexing are landed. | A B-rep that intersections and features can modify without inventing representation rules mid-boolean. |
| 2 | Build the M4 proof substrate | Geometry-graph descriptors for procedural/intersection curves; verified root seeds and polishing over the landed exact adaptive NURBS/implicit candidate isolation; extend the landed common `Complete`/`Indeterminate` evidence with paired pcurves and verified residual bounds. | Certified general CC/CS/SSI and trustworthy empty results. |
| 3 | Ship one end-to-end feature ladder | Profile-region builder with holes, deterministic body copy/transform, extrude/revolve, point-on-face and point-in-body classification, then block/block and block/cylinder booleans. Every result is atomic, journaled, checker-v2 clean, and externally X_T checked. | The first honest CAD modeling vertical slice. |
| 4 | Broaden general modeling | Expand analytic booleans, then periodic NURBS booleans, sweep/loft, sewing/healing, and STEP. | General mechanical part construction and imported-body repair. |
| 5 | Add local/advanced features | Fillet/blend, chamfer, offset, shell, draft/taper, replace/delete-and-heal, then production API and performance hardening. | The operation breadth expected by a fully featured CAD application. |

M3 X_T corpus work runs beside every tranche. It supplies hostile geometry and an external
oracle, but parser breadth does not substitute for a modeling milestone. Conversely, a
kernel operation does not enter the supported matrix until its result survives checker,
corpus, determinism, rollback, tolerance, and independent-oracle gates.

The following activities are useful experiments but do **not** advance the critical path
on their own:

- adding sampled pair-specific intersection solvers without completeness evidence;
- accepting another X_T node class without checker/tessellation and declared capability
  outcomes;
- relying on self-round-trip as interchange certification;
- reopening direct `Store` mutation or adding an unchecked assembly persistence path;
- growing happy-path unit tests without adversarial, minimized, and production cases.

---

## M0 — Foundations — IMPLEMENTED SLICE

### Implemented evidence

`crates/kcore` contains adaptive expansion arithmetic, robust `orient2d`/`orient3d`,
interval arithmetic, the session tolerance regime, typed generational arenas with
copy-on-write undo frames,
deterministic index-ordered parallel map primitives, and kernel-owned deterministic
sin/cos/sincos/atan/atan2. CI pins numeric golden hashes across debug/release and the
supported operating-system matrix.

### Conformance debt

- Add and adversarially verify `incircle`; add `insphere` when 3D Delaunay or equivalent
  classification first needs it.
- Audit classification decisions so exact predicates or interval-certified signs govern
  topology, while metric tolerance governs proximity. Raw sign tests and scattered
  working epsilon literals must not silently decide topology.
- Replace the catch-all use of `InvalidGeometry` with stable categories for invalid
  input, unsupported capability, topology precondition, convergence failure,
  indeterminate result, tolerance exhaustion, and resource limit.
- Remove panics from public kernel operations; invalid caller input returns typed errors.
- Add deterministic reduction primitives when the first real parallel consumers land;
  the existing parallel map helper is infrastructure, not evidence of parallel kernel
  performance.

### Conformance exit

Adversarial predicate suites pass; every public operation is panic-free for invalid
inputs; a decision audit finds no uncertified topological sign decisions; error behavior
is stable enough for the eventual C ABI.

## M1 — Geometry core — IMPLEMENTED SLICE

### Implemented evidence

`crates/kgeom` contains line/circle/ellipse curves; plane/cylinder/cone/sphere/torus
surfaces; exact analytic patch boxes; rational and polynomial clamped NURBS evaluation;
homogeneous 2D/3D curve and tensor-product surface knot insertion/refinement/splitting;
exact active-subcurve/sub-surface restriction and conservative control-hull/net boxes;
deterministic curve-segment and surface-patch Bezier extraction; global curve
interpolation; a reusable balanced deterministic AABB hierarchy with conservative
distance queries; interval-certified NURBS patch/plane control-hull and analytic
plane/sphere/cylinder/cone/torus implicit-surface exclusion; deterministic exact
subpatch isolation with structured cell-budget and parameter-resolution limits;
multi-start projection; deterministic trimmed-face
tessellation; and explicit `AlgorithmLimit` failures when refinement cannot meet its
request.

### Debt and delivery point

- **Before M4 certified general intersections:** Turn the landed exact adaptive
  NURBS/analytic-implicit candidate covers into verified root seeds with safeguarded
  polishing, then extend the interval contract to procedural and NURBS targets; add
  bounding/refinement support for periodic and unclamped forms, evaluator
  conditioning/singularity information, and projection APIs that distinguish converged,
  indeterminate, and failed searches.
- **Before M3 production Tier 2 / M6:** periodic NURBS curves and surfaces, collapsed
  patch detection, degree elevation, knot removal, approximation and fitting with
  verified error, and derivative/iso-curve construction.
- **Through M2.5/M6:** intersection, SP, trimmed, degenerate, swept, spun, offset, and
  blend geometry represented as exact procedural classes where X_T requires them.
- Add curvature and conditioning to the common evaluator protocol before blends and
  offset singularity analysis depend on them.
- Tessellation must eventually add angular tolerance, triangle-quality controls,
  incremental invalidation, and actual deterministic per-face parallelism.

### Conformance exit

Every target geometry class passes evaluator, periodicity, singularity, projection, and
bounding tests. NURBS operations pass published-value and randomized invariance tests.
Approximate constructions carry verified error bounds rather than undocumented sample
accuracy.

## M2 — Topology + primitives — IMPLEMENTED SLICE

### Implemented evidence

`crates/ktopo` contains body→region→shell→face→loop→fin→edge→vertex entities over typed
arenas, the ten Euler operators, a randomized Euler–Poincare harness, block/cylinder/
cone-frustum/sphere/torus constructors, a structural/sampled Fast checker plus the
checker-v2 Full reporting foundation, and edge-once whole-body tessellation. Primitive
meshes are Fast-checker-clean, watertight, outward-oriented, and volume-tested.

### Known limits

- Fins retain independent, explicitly parameter-mapped pcurves; authored analytic
  primitives and transaction-owned pcurve-aware Euler edits propagate them. Bounded curve-less
  tolerant edges use a canonical logical domain and require every fin pcurve; the
  checker compares lifted realizations and endpoints within entity tolerance, and body
  tessellation shares one deterministic 3D polyline across all uses. Public Euler
  creation requires independent pcurves; low-level graph assembly is transaction-scoped
  and cannot persist unless every affected body passes Fast checking and the complete
  live topology passes the ownership-closure boundary.
- `General` mixed-dimension bodies, face tolerances/domains, curve-less ring edges,
  isolated loops, and several pole/apex or degenerate topologies are unsupported.
- Entity fields remain readable plain data, but generic Store insertion/mutable borrow/
  removal and unchecked commit are private. External low-level construction is available
  only through transaction-scoped `AssemblyStore` and mandatory checked commit.
- Scoped Store transactions provide rollback-on-drop and deterministic raw mutation plus
  semantic lineage journals. Non-consuming mutation previews resolve old/new topology
  owners and curve/surface/point/pcurve dependents through committed/candidate indexes;
  caller-declared roots and every affected body are checked, while candidate construction
  audits global ownership closure. Invalid unlisted bodies, orphan subgraphs, and
  cross-body topology sharing roll back. X_T reconstruction, the complete public Euler
  edit surface, and every public implemented solid/sheet/wire/acorn constructor use this
  path. Candidate indexes clone the committed map and replace only deterministic affected
  body footprints; full reconstruction is asserted as a debug oracle. Deterministic body
  rank refresh, partition history, attribute propagation, and invalidation records remain.
- Fast checking samples some incidence and supports loop orientation only on a subset of
  surfaces. Full checking now proves the supported analytic incidence, simple planar
  segment/circle/ellipse loops, convex planar and selected closed analytic shells, but
  returns explicit `Indeterminate` gaps for unsupported NURBS/mixed incidence,
  multi-loop containment, general curved loops/shells, and wire self-intersection.

The following milestone closes these gaps before booleans.

## M2.5 — Boolean-ready architecture gate — IN PROGRESS / REQUIRED

### A. Parameter-space incidence

Landed slice:

- `kgeom` has validated line, circle, and NURBS 2D evaluators; `ktopo` stores them by
  `Curve2dId`.
- A fin can carry an independent `FinPcurve` with a finite curve range and an invertible
  affine edge-to-pcurve parameter map whose sign records orientation.
- Block, cylinder, and cone-frustum construction authors explicit pcurves, including
  reversed periodic correspondence on cap uses. Shared edge uses do not share a forced
  UV representation.
- The checker validates pcurve range coverage and samples the full 3D edge → pcurve →
  supporting-surface incidence tuple. Loop orientation consumes pcurves when present.
- Whole-body tessellation retains the edge parameter beside every shared mesh vertex,
  evaluates each fin's line/circle/NURBS pcurve directly, preserves explicit periodic
  branches through loop closure, and uses 3D surface inversion only for legacy fins.
- Pcurve-aware MEV/MEF/MEKR variants preflight both new fin uses before mutation and
  attach them after successful preflight. MEF/KEF/KFMRH/MFKRH preflight existing
  pcurve-bearing fins on a destination surface before moving them; checker and Euler
  validation share one incidence implementation. Full multi-step atomicity remains part
  of the transaction gate below.
- A bounded tolerant edge may omit its 3D curve and use a finite increasing logical edge
  domain (canonically `[0, 1]`). Every real fin must then carry a pcurve whose affine map
  covers that domain. The checker verifies pcurve definitions, endpoint-to-vertex
  tolerance, and agreement among all lifted fin realizations; shared-edge tessellation
  refines their deterministic averaged realization while anchoring topological vertices.
- X_T import/export maps the conforming tolerant-edge representation
  `EDGE.curve = null` plus per-fin `TRIMMED_CURVE → SP_CURVE → 2D B_CURVE`. Exact-edge
  pcurves are intentionally not written into `FIN.curve`. Polynomial/rational finite
  2D B-curves and reversed trim direction round-trip locally. A regression stores a
  B-curve ten times longer than its active SP trim and proves domain reconstruction uses
  only the active subrange.

Remaining before the gate closes:

- Migrate higher modeling/healing operations to the landed transaction-owned Euler
  methods. Pcurves are mandatory on every public MEV/MEF/MEKR face-edge creation; only
  topology-internal unit tests retain pcurve-less helpers.
- Certify the emitted SP-curve chain in a licensed Parasolid host. The corpus has
  an independently authored tolerant-edge exemplar (`exemplar.x_t`, owner-contributed
  Onshape/Parasolid 37.1 export, 2026-07-11): 131 curve-less tolerant edges whose fins
  carry `TRIMMED_CURVE → SP_CURVE` chains matching our emitted shape, and whose
  resolved 37102 layouts confirm the base-13006 133/141 field order. The import leg
  is now host-certified: Onshape accepted `solid_block_tolerant_edge.x_t`
  (2026-07-11) once its pcurve NURBS_CURVE nodes declared knot_type 5 /
  curve_form 1 with CURVE_DATA companions. The re-export/compare leg remains,
  as does extending interchange to periodic/circular pcurves and any
  geometric-owner variants observed in that corpus.
- `FinPcurve` now carries an explicit `PcurveChart` of integer period shifts. Domain
  derivation, incidence, loop orientation, tolerant-edge comparison, and tessellation all
  consume the same charted evaluator; the checker rejects shifts in non-periodic
  directions and domains that miss actual charted pcurve endpoints. Closed uses may also
  declare integer winding in edge-parameter direction; cylinder/cone ring pcurves author
  it, the checker validates it, and tessellation cross-checks it against the realized loop.
  Bounded uses can mark either endpoint as a surface singularity; the checker validates
  the marker against surface degeneracies and X_T SP-curve reconstruction infers it.
  A `PcurveSeam` now explicitly identifies the lower/upper side of a full-period chart
  cut. The checker proves that each marked pcurve lies on the named chart boundary and
  requires a complementary role on another fin of the same edge and face. A synthetic
  cylindrical-sheet fixture is checker-clean and tessellates through that paired seam.
  Still needed: mandatory metadata in all future checked creation paths, production seam/
  pole/apex X_T fixtures, and X_T round-trip for non-identity tolerant-pcurve charts.
- `FaceDomain` now carries an optional finite conservative UV work box and faces carry
  optional tolerance metadata. Analytic primitives author exact boxes; finite natural
  surface ranges initialize imported faces; Euler splits inherit them, merges union them
  only on the same surface (otherwise mark them unknown); the checker validates range/
  period/full-closed-face invariants; and tessellation uses them to anchor periodic
  branches. X_T reconstruction now derives conservative plane/cylinder/cone work boxes
  from each available fin pcurve's analytic or positive-weight NURBS control-hull box.
  Legacy exact fins without pcurves fall back to tolerance-inflated 3D curve boxes and
  analytic projection, so mixed faces are supported. Periodic faces preserve a consistent
  unwrapped pcurve branch; an exact fallback expands that direction to a full period, and
  incompatible pcurve branches yield an explicit unknown domain rather than a sampled or
  invalid box. The checker now verifies actual charted pcurve endpoints against declared
  domains. Positive-weight clamped 2D and 3D NURBS now derive conservative boxes from the
  exact requested subcurve control hull. Full checking adaptively subdivides every
  available charted pcurve interval: complete box coverage certifies containment, an
  evaluated exterior point proves invalidity, and depth/work/floating-progress limits
  preserve an explicit indeterminate result. Still needed: periodic/unclamped NURBS and
  unsupported exact/mixed boundary classes, production seam/pole/apex interchange
  coverage, and operation-specific tolerance propagation rules.
- Upgrade remaining sampled local incidence checking to adaptive verification and charge
  any accepted tolerance growth through the landed provenance/budget contract.
  Endpoint-to-vertex checks now exist, but they are still sampling-based rather than a
  proof over the full interval.

### B. Geometry graph and procedural evaluation

- Keep `kgeom` as pure mathematics, but introduce a geometry graph/evaluation context
  capable of resolving curve/surface handles.
- Define descriptors for intersection and SP curves first, and pull the offset-surface
  descriptor forward as the graph's first import client — M3c drives `exemplar.x_t`
  reconstruction through it; reserve stable extension points for swept, spun, and
  blend surfaces.
- Prevent recursive procedural geometry from requiring duplicated owned surfaces or a
  dependency from pure geometry back into topology.

### C. Transactions and journals

Landed slice:

- Every typed arena supports nested copy-on-write undo frames. A frame snapshots allocator
  metadata and clones only each first-touched pre-existing slot; rollback restores entity
  contents, handle generations, free-list order, and subsequent allocations exactly.
- A scoped Store transaction opens frames on every arena, rolls back on drop, rejects
  underspecified nested modeling transactions, and commits deterministic created/
  modified/deleted mutations in entity-type then slot order.
- Every arena exposes a non-consuming deterministic net-mutation preview that is asserted
  equal to the eventual commit journal. `StoreIndex` maps each live topology entity to
  its sole body and every referenced curve, surface, point, and pcurve to all dependent
  bodies. Committed and candidate indexes resolve old/new dependents for moves, deletion,
  replacement, and shared-geometry edits.
- `Transaction::commit_checked` validates declared result roots first and every
  mutation-affected live body with the Fast checker. Candidate index construction audits
  the complete store ownership closure and rolls the operation back with typed
  `TopologyCheckFailed` evidence for body faults, invalid unlisted bodies, orphan
  topology, or cross-body sharing. Duplicate roots are checked once deterministically;
  successful commit atomically installs the candidate index.
- Each committed body retains a deterministic footprint of owned regions/shells/faces/
  loops/fins/edges/vertices and referenced curves/surfaces/points/pcurves. Normal commits
  remove and rebuild only mutation-affected footprints, including committed-body deletion;
  debug/test builds compare every clean incremental candidate with a full reconstruction.
  A 64-body scope regression proves a one-body metadata edit rebuilds exactly one
  footprint and selects exactly that result root.
- Journals carry semantic `split`, `merge`, `derived_from`, `replaced`, and `deleted`
  events in addition to raw storage mutations. Every public Euler edit—minimal-body,
  edge/vertex, edge/face, edge/ring, and face/ring-hole—runs inside a transaction and
  emits tested deterministic lineage; all public face-edge creation requires pcurves.
- All public block/cylinder/cone/sphere/torus/cylindrical-sheet, simple planar-sheet,
  open/closed line-wire, and acorn constructors are scoped checked transactions and have
  journal-returning variants. Invalid polygon/wire/acorn inputs and partial invalid torus
  creation exercise rollback and preserve future handle identity; deterministic builder
  journals are tested.
- X_T reconstruction and transaction-owned Euler edits use checker-gated commits and expose
  their mutation/lineage journals; the previous full-session staging clone has been
  removed.
- Entity tolerances are validated values retaining their original metric value,
  imported-versus-operation origin, accumulated growth, and last modifying operation.
  A transaction declares aggregate growth budgets, applies face/edge/vertex enlargement
  through checked methods, rejects exhaustion before mutation, and commits deterministic
  budget reports plus per-entity tolerance events. Rollback discards model changes and
  budget usage together.

Remaining before the gate closes:

- Route future modeling and healing paths through checked transaction consumers; decide
  and test journal composition before enabling nested modeling transactions.
- Add partition/rollback marks and a committed undo/redo history above scoped operation
  transactions without weakening handle identity guarantees.
- Add attribute propagation and persistent journal serialization/versioning; define
  composition for tolerance budgets before nested modeling transactions are enabled.

### D. Enforced topology API

Landed slice:

- Checked body creation is now the public path for all implemented analytic primitives
  and the cylindrical sheet; convenience functions discard only the returned journal,
  not transaction/checker enforcement.
- Checked commit is reusable by higher operations and returns a stable topology-check
  error category rather than retaining an invalid result.
- Explicit checked builders now cover one-face planar polygon sheets, open/closed
  line-segment wires, and acorn point bodies. Their common void-region scaffold prevents
  ownership-layout drift, every sheet boundary use has an independent pcurve, and all
  three body kinds round-trip through the X_T writer.
- `PlanarProfile` separates robust input validation from topology mutation. Its first
  slice normalizes one simple polygon with exact-sign orientation/intersection decisions
  and is reusable by future sheet, extrude, revolve, and region-building operations.
- Raw Euler functions are crate-private. The complete public operator surface is exposed
  as transaction methods: minimal-body make/kill, pcurve-bearing edge/vertex make and
  inverse, pcurve-bearing face split/merge, edge↔ring, and face↔ring-hole. Multi-operator
  inverse sequences are checker-gated, rollback/identity tested, and emit deterministic
  derived/split/merge/delete lineage.
- Generic `Store::add`, mutable entity borrowing, removal, and unchecked transaction
  commit are crate-private. Immutable geometry insertion has type-specific entry points;
  interchange and specialized graph reconstruction use `AssemblyStore`, which exists
  only while a transaction owns every arena undo frame and can persist only through
  checked commit. Compile-fail guards lock the public boundary.
- Checked commit covers every affected body and complete topology ownership, not only
  caller-listed roots. Tests with no declared roots prove topology edits and shared
  surface changes select their dependents; an unlisted invalid body and an orphan child
  reject atomically, while dropped assembly restores identity and future allocation
  order. X_T reconstruction and its hand-authored writer fixtures use this path.

Remaining before the gate closes:

- Migrate future higher operations to compose the landed transaction Euler/topology API.
- Add explicit general-body and multi-face/multi-loop sheet builders; extend wire inputs
  beyond line polylines without requiring callers to assemble public vectors and
  back-pointers manually.
- Add large multi-body performance baselines and optimize the remaining deterministic
  body-order/rank refresh if measurements require it. Retain full reconstruction as the
  debug/audit oracle.

### E. Tolerance, errors, and checker v2

Landed slice:

- `check_body_report` accepts explicit `Fast` and `Full` assurance levels and returns
  proven faults plus, after Fast structure is clean, unresolved verification gaps.
  Invalid bodies do not attempt downstream proofs over inconsistent topology. Outcomes
  are three-valued:
  `Invalid` dominates proven faults, `Indeterminate` means no fault was found but a
  required proof is absent, and only an empty Full report is `Valid`.
- Full reports currently enumerate edge/surface and pcurve/surface incidence, face-domain
  containment, loop self-intersection/containment, wire self-intersection, shell
  self-intersection, and solid-shell orientation obligations. Every available pcurve is
  checked over its full active interval using conservative analytic or positive-weight
  clamped-NURBS subrange boxes and adaptive subdivision; any evaluated charted point
  outside the declared domain is a fault, while unsupported boundary uses and proof-limit
  exhaustion remain explicit gaps.
- Whole-interval incidence certification now recognizes exact affine and
  single-frequency harmonic traces. It proves all stored curve classes on planes when
  their analytic/control-hull residual is bounded, lines and harmonic curves on
  cylinders plus harmonic curves on spheres where the implicit residual is bounded, and
  matching line/circle
  pcurve lifts on analytic plane/cylinder/cone/sphere/torus iso-curves. A scale-aware
  rounding guard is charged before accepting the certificate; unsupported NURBS
  compositions and mixed-parameter nonlinear traces remain explicit gaps.
- Whole-loop certification uses robust `orient2d` signs to prove planar straight-segment
  rings pairwise disjoint, detects non-adjacent touches/crossings and adjacent backtracking
  as Full faults, and proves one-fin circle/ellipse loops simple over at most one period.
  Curved multi-fin and nonlinear-chart loops remain explicit gaps.
- Convex planar solid shells are certified as boundaries of their convex hull: each
  strict-convex face loop must contain exactly the vertices on one robustly classified
  supporting plane, and the occupied half-space determines the required outward face
  sense. Sub-resolution plane residuals consume metric tolerance before nonzero side
  signs route through robust `orient3d`. Single-face planar sheet shells are embedded
  when their sole loop is simple. Whole sphere/torus faces are certified from their
  analytic embedding, and a two-face sphere-cap shell is certified when its shared
  circle is the complete sphere/plane intersection and fin traversal agrees with the
  cap material half-space. Every supported positive fixture in the committed X_T corpus
  is now Full `Valid`.
- The X_T JSONL inspector uses every proven Full fault as its checker/tessellation gate
  while unresolved gaps remain non-failing, and separately records the Full outcome, gap
  count, and gap categories. Production
  corpus dashboards can therefore ratchet proof coverage without turning unknowns into
  successes or breaking the existing reconstruction pipeline.
- `EntityTolerance` replaces naked entity `f64` metadata. X_T reconstruction validates
  and stamps `ImportedXt` provenance; export emits the current metric value. The value
  retains origin/original value across later budgeted growth. Transaction journals
  expose declared limits, committed aggregate consumption, and ordered entity changes;
  budget exhaustion and invalid limits have stable typed kernel errors. Adversarial tests
  cover arithmetic-boundary accounting, deterministic journals, imported-origin
  retention, exhaustion before mutation, and checker-triggered rollback.

Remaining before the gate closes:

- Wire `Full` checking into an opt-in commit gate: a commit variant that rejects
  `Invalid` and can require `Valid` (or bounded `Indeterminate`) outcomes for its
  result roots. Today `commit_checked` enforces the Fast (structural/sampled) level
  only and the Full proof apparatus is exercised solely by tests and the corpus
  inspector; the proofs must become load-bearing on a write path before booleans
  consume them as an acceptance oracle.
- Define operation-specific tolerance combination/propagation rules and route every
  future modeling/healing tolerance change through the landed transaction budget API.
- Introduce capability and completion errors rather than treating unsupported or
  indeterminate geometry as invalid input.
- Extend incidence proof to Bezier-extracted NURBS/procedural curves and mixed-parameter
  nonlinear pcurves with conservative subdivision residuals.
- Extend loop proofs to Bezier-extracted curved pcurves and safe periodic/singular charts,
  then prove multi-loop containment, complete unsupported/mixed-boundary face containment, shell
  closure plus curved/non-convex self-intersection/orientation, and tolerance-budget
  validation.

### Exit gate

- A seam-crossing face and a pole/apex-adjacent face round-trip with explicit pcurves and
  pass checker v2 without reconstructing UVs from 3D samples.
- A deliberately failing multi-step topology operation restores bit-identical entities,
  handle validity, and next-allocation behavior. **Landed for scoped Store transactions.**
- A successful split/merge scenario emits deterministic lineage events. **Landed for the
  checked pcurve-aware transaction wrappers.**
- Budget exhaustion and a checker-failing tolerance edit restore the prior entity state;
  successful growth preserves imported origin and emits deterministic usage/events.
  **Landed for transaction-owned face/edge/vertex tolerance growth.**
- External code cannot mutate topology without checked topology APIs. **Landed: generic
  Store mutation and unchecked commit are private; transaction-scoped assembly is
  affected-body checker/whole-store ownership-gated, with compile-fail, dependency, and
  rollback tests.**
- Invalid inputs and unsupported capabilities cannot panic or masquerade as proven
  geometric misses.

No boolean implementation begins before this exit gate passes.

## M3 — X_T interchange and production corpus — IN PROGRESS

M3 is a parallel validation workstream, not a promise that every X_T construct can be
read before its corresponding kernel geometry exists.

### M3a0 — Modern-schema parser/reconstructor — IMPLEMENTED SLICE

- Text and neutral-binary cursors share one schema-driven parser.
- Embedded C/D/I/A/Z edits are applied against base schema 13006.
- The supported body/analytic/non-periodic-NURBS subset reconstructs atomically and can
  be checked and tessellated.
- The supported tolerant subset reconstructs bounded curve-less edges from trimmed
  SP-curves over finite polynomial/rational 2D B-curves, including decreasing parameter
  correspondence. It still lacks an external positive fixture.
- Three committed external positive fixtures pass parse, reconstruction, checker, and
  tessellation. Eight additional public discovery candidates—including multi-loop solids
  and a 1,501-node SolidWorks part—were inspected but are metadata-only because no source
  license was found. These are small smoke/regression results, not production-read
  evidence.

### M3a1 — Corpus and reader observability — IN PROGRESS / PARALLEL

- A committed TSV manifest records provenance, source commit, license status, exporter,
  schema, size, node count, feature tags, and ratcheted parse/reconstruct/checker/
  tessellation outcomes for all 6 committed fixtures. `xt_inspect` emits one JSON Lines record per
  input and continues after failures; tests prevent fixture or stage-result shrinkage.
- The committed set contains 4 externally authored files and 2 hand-authored
  schema-13006 fixtures. Three external files pass every local stage and one old-schema
  file remains an explicit unsupported case. A metadata-only discovery catalog records
  eight more inspected SCOREC files without redistributing them: seven pass locally and
  one general body is unsupported. The largest discovery is only 89 KB / 1,501 nodes, no
  external tolerant-edge positive has been found, and permission or licensed replacements
  are required before these candidates can enter CI.

- Maintain a manifest for every fixture: provenance, license/redistribution status,
  exporter/version, schema family, size, entity counts, geometry/topology features, and
  expected outcome.
- Record separate parse, reconstruct, checker, tessellation, and round-trip outcomes.
  Unsupported parse/reconstruction content now carries a stable `XtCapability` code in
  both the Rust API and JSONL corpus output while retaining human context. Extend the
  same manifest discipline to round-trip and external-host outcomes.
- Track ratcheted rates for the declared support matrix. Adding a new file cannot hide a
  regression by changing the denominator.
- Build a legally usable corpus spanning schema generations, analytic and NURBS parts,
  tolerant entities, seams/poles, sheets/wires/general bodies, voids, multi-body files,
  attributes, transforms, and assemblies. Reach at least 100 independent real files
  before describing the reader as a production subset; grow into the thousands over
  later milestones.
- Add parser mutation/property fuzzing now, not only in M8. Every crash or semantic bug
  becomes a minimized fixture.
- Surface dropped semantic content as explicit per-file diagnostics rather than bare
  skip counts: import currently skips ATTRIBUTE/GROUP/TRANSFORM/GEOMETRIC_OWNER and
  foreign nodes silently, and ignores the GEOMETRIC_OWNER rings this repository's own
  writer emits. Audit whether any skipped class (TRANSFORM in particular) can affect
  reconstructed geometry and reject rather than skip if so.
- Replace the full text-buffer copy and full-session clone path with bounded-memory
  parsing/staging. The reconstruction clone has been removed in favor of Store
  transactions; the full input buffer and parser representation remain. Add a size
  ladder through at least one 50 MB file and one million reconstructed topological
  entities.
- Make tessellation inspection scale-aware and record mesh quality/watertightness, not
  only stage success. The initial inspector intentionally uses one fixed chord tolerance,
  which is deterministic but too coarse to compare differently scaled models.

### M3b — Tier 1 authoring and external certification — IN PROGRESS

- Deterministic schema-13006 text output covers supported self-authored analytic solids,
  sheets, wires, acorns, bounded arcs, shared geometry, exact entity tolerances, and
  bounded curve-less tolerant edges represented by per-fin trimmed SP-curves. Circular
  and periodic pcurves are not yet in the declared writer subset.
- The full-period `cylindrical_sheet` primitive preserves its shared longitudinal seam
  edge and same-face double incidence through text X_T export/import, then checks and
  tessellates. Exact-fin pcurve chart/seam metadata is kernel-side and is not reconstructed
  yet; non-identity chart or seam metadata on emitted tolerant pcurves remains an explicit
  `xt.geometry.periodic-pcurves` unsupported capability.
- The published schema-13006 FACE tolerance field is required to be null. The writer
  therefore rejects a non-null kernel face tolerance with stable capability
  `xt.write.face-tolerances` rather than emitting a nonconforming file; face UV domains
  remain kernel-side because XT represents face bounds through loops, not a UV box field.
- Field-by-field ownership/link audit against real corpus files (plate.x_t,
  exemplar.x_t) landed 2026-07-11: exact bounded edges now reference their basis curve
  directly (every real exact-modeling file does; bounds are implied by vertices and
  recovered by inversion on import), POINT nodes are owned by their VERTEX (modern
  convention; only ancient base-13006 output used the body), and the
  TRIMMED_CURVE/GEOMETRIC_OWNER chain shape our tolerant path emits matches the
  exemplar's real linkage exactly (GO owned by the consumer, self-ring when alone,
  shared_geometry at the basis, basis owned by the body). Host acceptance of the
  reworked emission remains to be recorded.
- Add neutral-binary output after the text semantics are independently certified.
- For every authored capability: import into Solid Edge/another licensed Parasolid host,
  run its checker, re-export, re-import here, and compare topology, geometry class,
  tolerance, orientation, and mass properties.
- Landed harness: `xt_oracle export` deterministically generates the full declared
  Tier 1 authoring bundle (analytic solids, seam sheet, concave planar sheet, wires,
  acorn, B-curve edge, B-surface face, and bounded tolerant SP-curve edge) with a
  manifest of expected topology counts, mass properties, checker outcomes, and content
  hashes; `xt_oracle compare` diffs a host re-export against the original by body kind,
  entity counts, geometry-class histograms, tolerances, checker cleanliness,
  watertightness, and enclosed volume. The human-in-the-loop procedure is
  [oracle-loop.md](oracle-loop.md); outcomes ratchet in
  [oracle-results.tsv](oracle-results.tsv). First host runs (Onshape, 2026-07-11)
  accepted `solid_sphere` and `solid_torus` — the first kernel-authored files accepted
  by a licensed Parasolid — and drove four writer conformance fixes plus the
  bounded-edge emission rework. The second run (2026-07-11, after the
  `cyl.x_t`-driven `EDGE.fin` → positive-fin fix, the NURBS_CURVE
  knot_type/curve_form/CURVE_DATA alignment, and the sheet front-face +
  fins-at-vertex chain fixes) imported 10 of 13 fixtures cleanly: every
  solid (block, cylinder, cone, sphere, torus), the B-surface and B-curve
  blocks, the tolerant SP-curve block, and both sheets. Wire and acorn
  bodies remain rejected as corrupt with no real exemplar to bisect
  against. The third run (2026-07-11, writer=2beb267, fully automated via
  `scripts/oracle_loop.py`) added `offset_plane.x_t` — rejected as corrupt
  until the writer registered each OFFSET_SURF in its basis surface's
  geometric-owner ring (exemplar: 44/44) — and executed the first
  there-and-back leg: 6 of 9 re-exports compare clean (block, cylinder,
  sphere, torus, both sheets — topology, classes, and volume all match).
  The three mismatches are themselves findings: the exactly-analytic NURBS
  fixtures come back host-canonicalized to line/plane (class-preservation
  testing needs genuinely curved B-geometry), and Onshape's cone and
  tolerant-edge re-exports fail our reconstruction with checked-commit
  faults (preserved as `*_onshape_reexport.x_t` reader-gap fixtures).
  The accepted offset sheet materializes no exportable body on the host,
  so its compare leg is unavailable.
- The bundle must include a B-surface part in every host run until the provisional
  v-fastest B-surface pole ordering (`kxt::recon`) is confirmed or corrected by a
  licensed host; self-round-trip structurally cannot detect a transposed convention
  because reader and writer share it.
- Accessible Parasolid-backed hosts: Solid Edge Community Edition (free license,
  Windows) and Onshape (Parasolid-kernel SaaS; free plan documents are public).
  Open-source OCCT cannot read XT (its XT translator is a commercial component), so
  OCCT serves as a differential oracle only at the geometry/mass-property level via a
  host-exported STEP, or after M6 STEP support lands here.

**Exit:** 100% of the declared Tier 1 authoring matrix imports into the Parasolid oracle
with zero checker errors and survives there-and-back comparison. Self-round-trip alone
does not satisfy this gate.

### M3c — Tier 2/3 fidelity — THROUGH M6/M8

Periodic B-geometry, circular/periodic pcurve encodings, intersection curves, procedural
surfaces, curve-less tolerant ring/degenerate cases, general bodies, attributes,
transforms, instances, assemblies, and older schemas land with their kernel
dependencies. Full “any well-formed X_T” Tier 0 is not claimed until this matrix is
closed.

The gate fixture for the import half of this matrix is `exemplar.x_t`
(owner-contributed Onshape/Parasolid 37.1 production part, 2026-07-11: 96 faces
of which 44 sit on offset surfaces and 68 reference B-surfaces, 110 intersection
curves, 131 curve-less tolerant edges, 7,423 nodes). It parses fully today, and
the geometry-graph/offset reconstruction slice is complete. Reconstruction now
stops at periodic/closed B-geometry with the named capability
`xt.geometry.periodic-nurbs-surfaces`; this is step 3 below, not an offset
surface blocker. Its manifest row remains the committed progress meter:
`reconstruct: unsupported → pass`, then `tessellate: pass`, then full-checker
gaps shrinking toward `valid`. Dependency plan:

1. **COMPLETE — geometry graph with the offset-surface evaluator as its first
   import client.** The M2.5-B evaluation context and `OFFSET_SURF`
   reconstruction on it: point/derivative evaluation through the basis
   surface's curvature, validity domains (an offset is singular where the
   distance reaches the basis's minimum concave radius of curvature),
   conservative work boxes, and pcurve-driven tessellation are landed. This is
   the graph substrate shared by exemplar reconstruction and future M4 proofs;
   its descriptors serve both clients without duplicated owned geometry.
2. **Verified import of intersection curves.** Consume stored XT `INTERSECTION`
   geometry by verification, not recomputation: certify the transmitted
   B-spline representation against both owning surfaces with residual bounds
   inside the declared tolerances, following the landed
   certified-imported-domain pattern for pcurves. Re-deriving boolean scars
   through our own surface/surface intersector remains an M4 concern; import
   must not wait on it.
3. **LIVE RECONSTRUCTION BLOCKER — periodic/closed B-geometry.** Extend
   `kgeom` NURBS evaluation,
   reconstruction, and the writer beyond clamped non-periodic forms (the M1
   debt line). The pcurve chart machinery already carries integer period
   shifts, so the work concentrates in evaluation/knot handling, edge-bound
   recovery, and checker acceptance.
4. **Checker-gap ratchet on real parts.** Import does not gate on full-checker
   `valid`: exemplar-class bodies land as `Indeterminate` with named gaps
   (curved-face loop containment, general curved-shell orientation/embedding,
   self-intersection), and the M2.5 proof workstream closes those gaps
   incrementally with the manifest recording each ratchet.

Remaining matrix entries (general bodies, attributes, transforms, instances,
assemblies, older schemas) continue to land opportunistically with their kernel
dependencies.

**Exit evidence for the import half:** `exemplar.x_t` reconstructs and
tessellates deterministically with its mass properties recorded in the manifest
and compared against the originating host's values, making a 96-face production
part a standing differential fixture for every later modeling milestone.

## M4 — Certified intersections + profile operations — PROVISIONAL / GATED

Existing analytic solvers remain valuable exact accelerators. Existing fixed-grid NURBS
curve and surface marchers are experiments: they may discover contacts, but they cannot
label an empty result a proven miss or label an interpolated polyline an exact
intersection curve. The common result types now enforce that distinction: analytic
solvers explicitly construct `Complete` results, while provisional NURBS paths return
verified discoveries with a stable `Indeterminate` reason, and `is_proven_empty()` is
true only for an empty complete result.

### M4a — Common intersection contract and numerical core

- Landed common evidence contract: CC, CS, and SSI results carry shared `Complete` or
  diagnostic `Indeterminate` status; conservative/default construction is indeterminate,
  committed analytic solvers opt into completion, and empty means miss only through
  `is_proven_empty()`. Recursive isolation separately exposes structured cell-budget and
  parameter-resolution limits; intersection results still need structured provenance
  beyond stable reasons.
- An SSI branch carries a 3D curve, pcurve on surface A, pcurve on surface B, parameter
  correspondence, closure/end events, contact character, and a verified residual/error
  bound over the entire active interval—not only endpoint UVs.
- Represent coincident curve intervals and coincident surface regions separately from
  isolated contacts and ordinary branches.
- Landed substrate: NURBS-to-Bezier surface subdivision, positive-weight control-hull
  boxes, a reusable deterministic AABB BVH with outward-rounded separation queries, and
  interval-certified affine-plane patch exclusion, analytic plane/sphere/cylinder/cone/
  torus implicit fields over outward-inflated boxes, exact adaptive U/V subpatch isolation
  with conservative covers and structured limits, proof-bearing analytic/NURBS SSI
  empty exits, and exact restricted NURBS curve control-hull separation for complete
  curve/curve misses. Extend it to adaptive curve-pair subdivision, generic
  curve/surface pairing, verified seed existence,
  safeguarded Newton polishing, conditioning diagnostics, and procedural/NURBS fields.
- Analytic special cases and the generic solver feed the same canonical result type.
- Consolidate the per-pair analytic curve/surface and SSI boilerplate (range
  validation, parameter fitting, dedup, branch emission — currently repeated across
  the conic/quadric pair files) into shared drivers behind the same result contract,
  following the `MarchConfig` pattern already proven by the NURBS-surface marchers,
  before adding further special-case pairs.

### M4b — Curve/curve and curve/surface completion

- Complete analytic pairs, then use subdivision plus Newton as the general NURBS and
  procedural fallback.
- Prove domain exclusion, endpoint ownership, overlap extent, tangency classification,
  and deterministic duplicate merging.
- Test high curvature, multiple roots inside one coarse cell, near coincidence, tiny
  features, singular derivatives, reversed/periodic ranges, and tolerance boundaries.

### M4c — Surface/surface intersection

- Use adaptive predictor-corrector marching after certified seed discovery.
- Discover tangent seeds, singular points, branch junctions, closed small loops, boundary
  contacts, and coincident regions.
- Build a branch graph before curve fitting; fit 3D and both pcurves together and verify
  them against both surfaces.
- A branch is accepted as edge geometry only when checker v2 validates its complete
  incidence at the requested tolerance.

### M4d — First modeling consumers: profiles, transforms, extrude, and revolve

- Extend the landed validated simple-polygon `PlanarProfile` to curve loops, holes,
  nesting/arrangements, and explicit pcurves without weakening its exact-sign rejection
  of degenerate or self-intersecting boundaries.
- Add deterministic checked copy/transform for geometry and complete bodies, preserving
  incidence, attributes, tolerances, and lineage without aliasing mutable ownership.
- Extrude and revolve them through the transaction/journal/topology APIs.
- Exercise seams, axis contacts, caps, inner loops, and full/partial revolutions.

### Exit gate

The adversarial CC/CS/SSI battery includes tangencies, near-coincidence, small loops,
singularities, and NURBS-vs-NURBS cases. Every `Complete` result is independently
verifiable; unresolved cases return `Indeterminate` or a typed limit. Extrude/revolve
outputs are checker-v2 clean, journaled, watertight, and externally X_T validated.

## M5 — Analytic booleans + interrogation — NOT STARTED

Do not wait for an exhaustive analytic pair table before testing the end-to-end boolean
architecture. Begin with a deliberately narrow vertical slice as soon as the required
M4 cases are certified.

### M5a — Vertical boolean slice

- Start with a small named matrix such as block/block and block/cylinder
  unite/subtract/intersect.
- Implement the real pipeline: face-pair broad phase → intersection → imprint pcurves →
  split faces → classify fragments → assemble shells → tolerant stitch → full checker.
- Every phase runs inside one transaction and emits lineage/attribute propagation
  events.
- Add point-on-face and point-in-body classification before fragment classification;
  classifier uncertainty must propagate rather than become an arbitrary inside/outside
  choice.

### M5b — Broad analytic booleans and interrogation

- Expand to the supported analytic primitive/transform matrix, regularized and
  non-regular cases, disjoint/contained/coincident inputs, voids, sheets, and
  sheet-splits-solid.
- Add certified area, volume, centroid, and inertia; body/face bounding hierarchies;
  point/ray classification, section curves, minimum distance, and clash detection.
- Run differential tests against OCCT and Parasolid. Every disagreement is classified
  and retained as a regression case.

### Exit gate

- At least 99.5% success on a versioned, non-shrinking analytic corpus with a published
  denominator and zero checker-failing “successes.”
- Volume conservation and boolean identities pass within certified error bounds.
- Failure is atomic; journals and tolerance growth satisfy their contracts.
- Successful results import into the Parasolid oracle with zero checker errors.

## M6 — General booleans, sweeps, lofts, sewing, STEP — NOT STARTED

- Extend intersections and booleans to periodic NURBS and procedural surfaces.
- Implement sweep along curve and loft, including compatibility operations such as
  degree elevation and knot refinement/removal.
- Implement tolerant sewing with explicit gap/tolerance budgets, non-manifold
  diagnostics, and healing journals.
- Add STEP AP242 read/write after sewing is independently robust.
- Close the M3 Tier 2 geometry matrix for B-curves/B-surfaces, SP/intersection curves,
  swept/spun/offset geometry, and their X_T representations.

**Exit:** the imported NURBS boolean corpus satisfies the success-rate ratchet; sewing
closes the versioned torture corpus without exceeding tolerance budgets; STEP and X_T
round-trips preserve geometry class and topology.

## M7 — Blends, offsets, shelling, and local operations — NOT STARTED

- Constant-radius rolling-ball edge blends and chamfers with exact procedural
  representation.
- Variable radius, tangent chains, setbacks, and corner patches.
- Face/body offset, hollow/shell, taper, tweak/replace surface, and delete-and-heal.
- Detect offset singularities, blend overrun, vanishing faces, and topology changes;
  report them through typed outcomes and journals.

**Exit:** a versioned torture corpus covers converging edges, tangent chains, blend
overruns, corner interactions, thin regions, and shell self-intersections. Procedural
blend/offset classes survive Parasolid round-trip rather than silently becoming NURBS.

## M8 — API stabilization and production hardening — NOT STARTED

- Freeze a versioned native API, then a PK-style C ABI with opaque handles, typed error
  codes, and versioned option structs; add Python bindings for corpus automation.
- Finalize partition/session semantics and entity-id stability guarantees using the
  M2.5 transaction/journal foundation.
- Continue—not begin—fuzzing across parsers, topology operations, intersections, and
  modeling pipelines; run sustained campaigns and retain minimized regressions.
- Profile and optimize against the specification’s size/performance ladder without
  weakening determinism or correctness gates.
- Add compatibility/version policy, migration tests, documentation, and long-running
  stress suites.

**Exit:** API semantics are frozen; fuzzers and stress suites run clean for sustained
CPU-days; supported performance targets pass on named hardware; no robustness metric
regresses.

---

## Cross-cutting implementation controls

Every future pull request that adds a kernel capability should answer:

1. **Capability:** Which support-matrix cell changed? What remains explicitly
   unsupported?
2. **Completion:** Can the algorithm prove completeness? If not, how is
   `Indeterminate` represented?
3. **Tolerance:** Which tolerances were consumed/produced, and can they grow?
4. **Atomicity:** What state changes, and how is rollback verified?
5. **Journal:** Which lineage and attribute events are emitted?
6. **Checker:** Which fast/full checker rules validate the result?
7. **Corpus:** Which adversarial, production, and minimized-regression cases were added?
8. **Oracle:** What independent implementation or mathematical property validates it?
9. **Determinism:** Are ordering, reductions, IDs, and output bits stable?
10. **Performance:** What benchmark prevents an accidental asymptotic regression?

Metrics are versioned artifacts, not prose claims. Maintain machine-readable support
matrices and dashboards for X_T stage rates, checker failures, boolean outcomes,
tolerance growth, algorithm limits, fuzz regressions, and performance percentiles.
The committed X_T manifest therefore records both the Fast checker gate and the expected
Full outcome/gap count for each fixture. A checker-v2 change advances the roadmap only
when it either discharges a ratcheted gap with a conservative proof or adds a previously
missing obligation explicitly; deleting, weakening, or silently reclassifying an
obligation is not progress. Any intentional baseline change must update the capability
ledger and include an adversarial regression that distinguishes `Invalid`,
`Indeterminate`, and `Valid`.

## Milestone dependency backlog

This is not an execution-priority list. The authoritative handoff queue is the
foundation portfolio; this section preserves the larger milestone obligations
that queue must eventually discharge.

- M3b: keep the exact 14-file oracle bundle and its writer-byte invalidation
  rule current. The 2026-07-11 Onshape run is machine-fingerprinted; remaining
  evidence is the named wire/acorn rejection, host-canonicalized analytic NURBS
  fixtures, offset-sheet re-export gap, and two preserved host re-export reader
  gaps.
- M3c: implement periodic/closed B-geometry, the live exemplar reconstruction
  blocker, and independently verify imported intersection curves.
- M2.5: finish parameter-space incidence and ratcheted Full-checker proofs for
  periodic/mixed boundaries, multi-loop containment, and curved shells.
- M4: extend completion evidence with paired pcurves, coincident regions,
  singular events, structured limits, and verified residual bounds; turn
  retained NURBS cells into verified root seeds only after the F2 scale gates.
- M5: grow planar profiles and booleans only after facade adoption and the
  checker, rollback, lineage, tolerance, determinism, corpus, performance, and
  independent-oracle gates.
- Production-scale and assembly imports must exercise the Q2a construction and
  Q2b traversal ladders. Reverse-index and traversal membership indexing have
  landed against deterministic audit/digest evidence; future representation or
  body-order optimization still follows named-host measurements and retains
  full-reconstruction audits.

## After the kernel

1. **Constraint solver:** 2D sketch solver first, then 3D constraints.
2. **Parametric feature framework:** feature tree, persistent naming consuming kernel
   lineage journals, and deterministic regeneration.
3. Application/UI layer.

## Standing risks

| Risk | Mitigation |
|---|---|
| Pair-specific intersection growth hides the absence of a complete generic solver | M4 common contract, certified subdivision fallback, and `Indeterminate` semantics precede further breadth. |
| Boolean work hardens an insufficient B-rep | M2.5 pcurve, transaction, geometry-graph, and checker gates are mandatory. |
| Self-round-trip validates shared X_T bugs | Parasolid and OCCT independent-oracle gates; production corpus stage metrics. |
| Tolerant modeling is retrofitted too late | Per-incidence pcurves, entity tolerance provenance, operation budgets, and checker v2 land before booleans. |
| Corpus size grows without useful coverage | Feature manifests, versioned support matrix, stage-specific rates, and non-shrinking regression sets. |
| Atomicity becomes full-store cloning | Arena checkpoints and mutation logs in M2.5; X_T migrates to the same transaction path. |
| Determinism prevents practical parallelism | Deterministic work ordering and reduction trees, verified at each real parallel consumer. |
| Performance targets are deferred until redesign is expensive | Size ladders and benchmarks begin with X_T, BVHs, tessellation, and the first boolean slice. |
| Blend/offset complexity is underestimated | Dedicated M7, procedural exactness, and torture corpora remain explicit. |
