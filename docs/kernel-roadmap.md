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
| M0 Foundations | IMPLEMENTED SLICE | Deterministic math, robust exact-fallback `orient2d`/`orient3d`/`incircle`, exact cyclic `polygon_orientation2d` plus its non-copying streaming companion `polygon_orientation2d_iter`, intervals, tolerances, arenas with copy-on-write undo frames, deterministic map primitives, exact-sign consumers in strict first-chart SSI polygon convexity, coincident Plane/Plane monotone-hull construction, and oblique-extrusion direction, exact streaming `kgeom` trim cleaning and outer/hole winding, exact paired-chart `kops` polygonal-region winding, exact `ktopo` ordinary-face outer-loop selection, and the first source-preserving solver-error path exist; rounded trim signed area is reporting only. `insphere`, an `incircle` production decision consumer when required, checker sampled-loop winding/outer selection, conic discriminant and NURBS-plane sign certification, other solver-local error migrations, the broader topological-decision audit, and broader conformance debt remain. |
| M1 Geometry | IMPLEMENTED SLICE | Analytic geometry, clamped NURBS evaluation plus exact curve/surface splitting, restriction, Bezier extraction and active-subrange bounds, projection, and tessellation exist; periodic/procedural and several full NURBS capabilities remain. |
| M2 Topology | IMPLEMENTED SLICE | Core hierarchy, topology-internal Euler operators, transaction-owned public Euler edits, primitives, the structural/sampled Fast checker, checker-v2 Full reporting, watertight body tessellation, checked transaction-scoped assembly, and deterministic journals exist; general bodies and several degenerate topology classes remain. |
| M2.5 Architecture gate | IN PROGRESS / REQUIRED | Per-fin pcurves with integer-period chart shifts, paired seam-edge roles, closed-use winding, and singular endpoint markers; bounded curve-less tolerant edges; typed entity-tolerance origin/growth provenance, transaction-owned aggregate budgets, one checked facade batch for operation-owned Face/Edge/Vertex tolerance growth, and descriptive MEF inheritance plus KEF ordered-max face-tolerance journals; shared incidence validation; a complete transaction-owned public Euler surface with position-owning transient MVFS/KVFS, mandatory pcurve creation, hidden-point cleanup, and derived/split/merge/delete lineage; private generic Store mutation; transaction-scoped low-level assembly whose only public persistence path uses deterministic mutation preview, incrementally replaced per-body ownership/shared-geometry dependency footprints, affected-root Fast checks, complete ownership closure, and an opt-in evidence-bearing Full-assurance commit gate; pcurve-driven tessellation; deterministic mutation/lineage/tolerance journals; failure-atomic journaled solid/sheet/wire/acorn constructors; reusable validated polygonal planar profiles with strictly contained pairwise-disjoint holes, checked holed-sheet construction, and checked nonzero-normal oblique extrusion; checked X_T reconstruction; explicit face metadata; certified imported domains; adaptive full-active-interval analytic/clamped-NURBS face-domain containment; explicit `Fast`/`Full` checker reports with `Valid`/`Invalid`/`Indeterminate` outcomes; whole-interval affine/harmonic incidence certificates; robust planar-segment/simple-ring and strict outer/hole containment proofs; convex-planar, whole sphere/torus, sphere-cap, single-planar-face, and exact polygonal-prism shell embedding proofs; the unchanged seven-row crossed affected-production-solid `primitive_mix` grid and distinct four-row fixed-64 block-cohort ladder retain exact scoped edit evidence, and a distinct four-row production-clean ladder over unchanged `primitive_mix` totals 4/16/64/256 has landed with zero-edit commit, store/index/output ratchet, and repeat evidence for the combined current validation/index-clone/body-order path, not phase isolation. General NURBS/mixed-parameter incidence, periodic/unclamped and unsupported exact/mixed-boundary containment, curved or nested-island profiles, operation-specific tolerance combination/propagation rules beyond the landed MEF/KEF policy and generic batch, curved-loop/general curved-shell proofs, production seam/singularity interchange fixtures, broader higher-operation migration, and remaining global ordinary-commit phase counters and optimization, broader heterogeneous production-edit-footprint, and production-assembly performance baselines remain. |
| M3 X_T | IN PROGRESS | The modern-schema subset reads both wire encodings and writes text, including bounded tolerant edges as trimmed SP-curves over finite 2D B-curves; production coverage and external certification remain. |
| M4 Intersections/profile ops | PROVISIONAL / GATED | Broad analytic special cases, explicit `Complete`/`Indeterminate` result evidence, exact NURBS patch subdivision/BVH, analytic implicit-surface exclusion, deterministic recursive candidate covers with structured limits and proof-bearing miss exits, source-range-certified curve-pair exclusion bounds, bounded cell-local curve-pair polishing, several exact-cell interval-certified root/overlap slices, rigid complete-body copy, and the first checked polygonal-profile extrusion exist; general root discovery and boolean-ready paired-pcurve branches do not. |
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
| 1 | Close M2.5 topology contracts | Production seam/pole/apex interchange fixtures; operation-specific combination/propagation rules beyond landed MEF inheritance and KEF ordered maximum over the tolerance provenance, budgets, and checked facade batch; production-assembly and broader heterogeneous production-edit-footprint baselines plus global ordinary-commit phase counters and optimization beyond the landed seven-row mixed-store affected-root cohort, unchanged seven-row crossed `primitive_mix` affected-solid grid, four-row fixed-64 mixed-entity block cohort, and four-row production-clean global-path ladder; and discharge the remaining checker-v2 `Full` proof gaps with adaptive incidence, curved-loop/multi-loop containment, and shell proofs. Full-active-interval face-domain containment, private/checked mutation, transaction lineage, affected-root selection, per-body incremental indexing, failure-atomic facade tolerance batching, the opt-in Full-assurance write gate, and the crossed affected-production-solid count/total-size grid are landed; the distinct block cohort pins first Face-to-Edge-to-Vertex mutation at 1/4/8/13 affected blocks with exact `3N` event/accounting, scoped-counter, digest, installed-index, and repeat evidence, while the production-clean ladder pins one zero-edit ordinary commit, equal store/index snapshots, stable store/index/output ratchets, and repeat equality at 4/16/64/256 bodies without claiming phase isolation. | A B-rep that intersections and features can modify without inventing representation rules mid-boolean. |
| 2 | Build the M4 proof substrate | Geometry-graph descriptors for procedural/intersection curves; generalize the landed verified seed/polish path and exact-cell root/overlap certificates over adaptive NURBS/implicit candidate isolation; extend the landed common `Complete`/`Indeterminate` evidence with paired pcurves and verified residual bounds. | Certified general CC/CS/SSI and trustworthy empty results. |
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

`crates/kcore` contains adaptive expansion arithmetic; robust deterministic
`orient2d`, `orient3d`, and `incircle`; interval arithmetic; the session
tolerance regime; typed generational arenas with copy-on-write undo frames;
deterministic index-ordered parallel map primitives; and kernel-owned deterministic
sin/cos/sincos/atan/atan2. `incircle` uses Shewchuk's conservative stage-A
filter and an exact expansion fallback without tolerance: for counterclockwise
`a,b,c` it is positive inside, negative outside, exactly zero for cocircular
inputs, and orientation reversal flips the sign. Its evidence includes 20,000
random integer cases against an `i128` oracle, all six defining-point
permutations, exact and one-unit near-cocircular fixtures that demonstrably
force the fallback, degenerate/non-finite behavior, and the deterministic
numeric golden. CI pins numeric golden hashes across debug/release and the
supported operating-system matrix.

`incircle` remains a public predicate with conformance evidence but has no
production topology decision consumer yet. `insphere` remains deferred until a
3D Delaunay or equivalent classification consumer needs it.

The public `polygon_orientation2d` slice and its streaming companion
`polygon_orientation2d_iter` evaluate the exact cyclic shoelace expansion. The
streaming form does not retain or allocate a coordinate copy. Fewer than three
vertices, any non-finite coordinate, and an exactly zero expansion return
`Orientation::Zero`; cyclic
rotation preserves the sign, reversal flips every nonzero result, and repeated
vertices are accepted with algebraic-area semantics. Its evidence includes
20,000 random integer polygons checked against an `i128` oracle, the
`2^52`-translated unit square whose naive floating shoelace sum is zero, and
the cross-platform numeric determinism golden.

`kgeom` now routes `TrimLoop::cleaned_point_count` degeneracy and
`TrimmedSurface` outer/hole winding through the streaming exact sign without a
coordinate-copy allocation. Consecutive and closing duplicate cleanup retains
the same exact decision, and rounded `TrimLoop::signed_area` remains available
for reporting and magnitude estimates only. Public trim conformance rotates and
reverses the large-cancellation square, accepts it as an outer loop or reversed
hole as appropriate, and rejects exact-zero and non-finite inputs.

`kops` polygonal-region canonicalization now evaluates the streaming exact
polygon sign in both parameter charts. Equal signs derive
`SurfaceRegionOrientation::Same`, opposite signs derive `Reversed`, either zero
sign fails closed, a contradictory declared relation is rejected, and a
negative first chart is reversed into the canonical positive winding. The
focused integer-source boundary has exact
`i128` doubled area `+2` where the former origin-relative floating sum is zero,
and pins repeat, rotation, reversal, and both chart relations.

`ktopo` ordinary-face body tessellation now selects exactly one
`Orientation::Positive` outer loop with the streaming predicate. Duplicate
positive outers, missing positive outers, exact-zero loops, and non-finite loops
fail closed. The `2^52`-translated unit square has exact doubled area `+2` while
the former naive shoelace sum is zero. Selection precedes and preserves the
existing periodic hole anchoring and outer-first loop order; public planar-sheet
tessellation pins deterministic output.

The first audited exact-sign consumer is the strict first-chart polygon
convexity gate used by SSI region consolidation. It rejects boundaries with
fewer than three vertices or any non-finite first-chart coordinate, and
requires `orient2d(a, b, c) == Orientation::Positive` at every consecutive
turn; exact collinearity and every nonpositive turn therefore fail closed. A
public region-consolidation fixture supplies a determinant of exactly `+1` at
approximately `2^52`, where the previous rounded cross product became zero,
and pins repeat, rotation, and reversal canonicalization. It remains a bounded
consumer migration alongside the separately landed general polygon-orientation
primitive.

Coincident bounded Plane/Plane window construction is another landed exact-sign
consumer. Both lower and upper monotone hull chains retain only an exact
`Orientation::Positive` `orient2d` turn; exact `Zero` and `Negative` turns pop,
matching the former nonpositive policy without rounded classification. A direct
private production-helper fixture uses integer coordinates near `2^52` with an
exact `i128` determinant of `+1` where the legacy floating turn is zero. It
retains the middle hull vertex under repeat, input rotation, and reversal, and
removes an exactly collinear middle vertex. This adversary cannot be expressed
as three consecutive exact public overlap edges: the intersection of two
rectangles draws its boundary directions from two axis pairs, with same-frame
directions perpendicular. The public seam therefore uses an ordinary
eight-vertex coincident overlap to pin a `Complete` Region, cyclic exact-positive
turns, repeatability, operand-swap semantic parity, and non-finite range
rejection.

The oblique-profile extrusion direction gate is also migrated.
`extrude_profile_along_in` evaluates the exact stored-frame scalar triple as
`orient3d(frame.x(), frame.y(), translation, 0)`, preserving the ordinary
positive/negative convention because the frame is right-handed. Exact zero is
rejected before topology allocation; an exact negative result reflects the
profile chart and reference normal before reusing the positive branch. The
focused integer-source adversary has normal dot `+3` while the former
normalized `translation.dot(frame.z())` rounds to zero. Both translation
directions construct deterministic closed topology, and ordinary oblique
fixtures remain Full-valid.

The first solver-local F4 identity migration is also landed. Ellipse/ellipse
closest-point failures retain all five `ProjectionError` variants as
`IntersectionError::Projection`: `InvalidQueryPoint`, `InvalidWindow`,
`NoCandidate`, `NonFiniteEvaluation`, and `Policy`. Their class, stable code,
structured limit, `capability() == None`, and direct `std::error::Error` source
survive the concrete solver, generic intersection adapter,
`GeometryIntersectionError`, and `KernelError`; `Policy` additionally retains
its exact `OperationPolicyError` source. The direct
`intersect_bounded_ellipses` entry returns `IntersectionResult` so this path
cannot be flattened back into `kcore::Error`.

### Conformance debt

- Add `insphere` when a 3D Delaunay or equivalent classification consumer first
  needs it.
- Audit classification decisions so exact predicates or interval-certified signs govern
  topology, while metric tolerance governs proximity. Raw sign tests and scattered
  working epsilon literals must not silently decide topology. The first-chart SSI
  convexity, coincident Plane/Plane monotone hull, and oblique-extrusion direction
  gates are migrated. The `kgeom` trim-loop, `kops` paired-chart
  polygonal-region, and `ktopo` ordinary-face body-tessellation shoelace
  decisions are also migrated. Concrete remaining targets include checker
  sampled-loop winding and outer-loop selection, conic discriminant root-count
  classification, NURBS-plane control-distance and bracketing signs, and other
  raw classification branches. Continue the broader topological-decision audit
  without treating these bounded consumers as audit completion.
- Continue replacing catch-all `InvalidGeometry` mappings with stable categories
  for invalid input, unsupported capability, topology precondition, convergence
  failure, indeterminate result, tolerance exhaustion, and resource limit. The
  ellipse/ellipse projection path is migrated; other solver-local collapses and
  legacy wrappers remain explicit debt.
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
  proves strict containment and embedding for one exact polygonal outer loop
  with pairwise-disjoint unnested polygonal holes, and returns explicit
  `Indeterminate` gaps for unsupported NURBS/mixed incidence, curved or nested
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
- Position-owning MVFS validates the supporting surface and finite size-box
  seed position before allocating its hidden point, returns every created
  topology identity opaquely, and remains a transient candidate that checked
  commit rejects until later Euler edits complete it or facade KVFS removes it.
  The facade KVFS inverse deletes and journals the hidden point only when
  unshared; ordinary lower KVFS retains external/shared geometry.
- Pcurve-aware MEV/MEF/MEKR variants preflight both new fin uses before mutation and
  attach them after successful preflight. MEF/KEF/KFMRH/MFKRH preflight existing
  pcurve-bearing fins on a destination surface before moving them; checker and Euler
  validation share one incidence implementation. The facade now exposes checked
  position-owning MEV/KEV and KFMRH/MFKRH requests with exact lineage, rollback
  identity, and pcurve metadata transport. MEV preflights the position, topology,
  curve, bounds, and both pcurves before allocating its hidden point; its facade
  inverse removes and journals that point only when no live vertex shares it, while
  ordinary lower KEV retains externally owned point geometry. The structural
  face/hole operators do not pre-certify geometric hole containment; supported Fast
  checks gate persistence while operation-specific unsupported containment remains an
  explicit caller proof obligation.
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
  budget reports plus per-entity tolerance events. The facade exposes one ordered batch
  rather than a reusable transaction budget capability: every target is part-qualified
  and live before value or duplicate validation, targets are unique, requested final
  values meet the model-resolution floor, and exact aggregate accounting plus
  imported-origin-preserving provenance complete before an infallible apply. Events retain
  request order; the returned opaque budget identity resolves only in the committed journal
  and no edit method accepts it. Rollback and checked-commit denial discard model changes
  and transaction-local budget usage together.
- MEF copies the source face's complete optional tolerance provenance to the new
  face without growth. KEF selects the larger ordered `[surviving, absorbed]`
  input, resolves equal values toward the survivor, and preserves exactness
  when both are exact. The committed journal describes the inputs, result,
  selected source, and provenance without creating budget or authoring
  authority. Checked/resource denial restores the values and exact future
  face/edge/loop/fin identities.

Remaining before the gate closes:

- Route future modeling and healing paths through checked transaction consumers; decide
  and test journal composition before enabling nested modeling transactions.
- Define and test propagation/combination policies for edit families beyond
  the landed MEF inheritance and KEF ordered maximum.
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
- `PlanarProfile` separates robust input validation from topology mutation. It
  normalizes one simple outer polygon and strictly contained pairwise-disjoint,
  unnested polygonal holes with exact-sign orientation/intersection decisions.
  The checked sheet and first positive-axis prism extrusion consume the same
  profile without revalidating it through weaker predicates.
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
- The seven-row mixed-store affected-root baseline holds a four-body dependent
  cohort across 4/16/64/256 total bodies and independently sweeps 1/4/16/64
  affected roots at 64 total bodies. The existing seven affected-production-solid
  `primitive_mix` rows remain unchanged and form a crossed grid: 1/4/16/64
  affected roots at 64 total bodies and one affected root across
  4/16/64/256 total bodies, sharing the 64/1 row. The total-size ladder uses
  one ordinary checked operation scope
  to grow exactly the first Face to `2e-8` under an exact `1e-8` budget. It
  pins one modified Face net mutation and one ordered tolerance event,
  affected/refreshed/checked/mutations = 1, a stable affected digest across
  totals, before/after store and output-digest ratchets, and installed-index
  equality. The fixed-total ladder retains exact `N × 1e-8`, N-face/event,
  scope, digest, and index-equality evidence. A distinct fixed-total-64
  four-row block-cohort ladder selects `primitive_mix` body ordinals divisible
  by five at 1/4/8/13 affected blocks (all 13 eligible blocks), because the
  cylinder and cone ring solids have no vertices. One ordinary checked scope
  grows the deterministic first Face, then first Edge, then first Vertex of
  every selected block to `2e-8` under a deterministic `3N`-event budget at
  `1e-8` per entity. It pins N modified mutations of each entity type, exact
  ordered events, affected/refreshed/checked = N, affected/store/output digest
  ratchets, installed-index equality, and repeat parity. These prior 39 Q2 rows
  remain unchanged. A distinct four-row production-clean ladder holds an
  unchanged `primitive_mix` at 4/16/64/256 total bodies. Each timed sample
  executes exactly one ordinary `commit_checked(&[])` scope with zero edits,
  affected, refreshed, checked, or mutated entities; commits; pins equal
  before/after store and installed-index snapshots plus stable
  store/index/output ratchets; and requires repeat equality. This is the first
  production-solid global ordinary-commit baseline over the combined current
  graph-validation, committed-index-clone, and body-order path, not phase
  isolation. Elapsed time remains advisory. Add phase counters and optimize
  where measurements require it; add production-assembly and broader
  heterogeneous production-edit-footprint ladders; and retain full
  reconstruction as the debug/audit oracle.

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
- Opt-in Full-assurance commits preserve the Fast compatibility path, run Fast graph
  validation exactly once, and then make Full proofs load-bearing before persistence.
  Explicit roots precede affected roots and store-wide audit roots, with duplicates checked
  once. `RequireValid` rolls back on any fault or proof gap; `AllowIndeterminate` accepts
  only Fast-clean, Full-fault-free candidates and retains every gap. Both committed and
  rejected decisions own ordered per-body Full reports, while rejected decisions carry no
  journal. Wrong-part/stale facade roots fail before scope creation; proof or accounting
  denial restores model/tolerance state, the committed dependency index, and future IDs.

Remaining before the gate closes:

- Define operation-specific tolerance combination/propagation rules beyond the
  landed MEF inheritance and KEF ordered maximum, and route every future
  modeling/healing tolerance change through the landed transaction budget API.
- Introduce capability and completion errors rather than treating unsupported or
  indeterminate geometry as invalid input.
- Extend incidence proof to Bezier-extracted NURBS/procedural curves and mixed-parameter
  nonlinear pcurves with conservative subdivision residuals.
- Extend loop proofs to Bezier-extracted curved pcurves and safe periodic/singular charts,
  then broaden the landed exact polygonal outer/hole containment proof to curved and nested
  arrangements, complete unsupported/mixed-boundary face containment, shell
  closure plus curved/non-convex self-intersection/orientation, and tolerance-budget
  validation.

### Exit gate

- A seam-crossing face and a pole/apex-adjacent face round-trip with explicit pcurves and
  pass checker v2 without reconstructing UVs from 3D samples.
- A deliberately failing multi-step topology operation restores bit-identical entities,
  handle validity, and next-allocation behavior. **Landed for scoped Store transactions.**
- Successful split/merge scenarios emit deterministic lineage events. **Landed for the
  checked pcurve-aware face wrappers and the facade-owned bridge-edge removal/ring-join
  plus face-as-hole merge/split wrappers, including exact rollback identity, face
  merge/split lineage, and affine pcurve metadata transport.**
- Position-owning MEV/KEV emits deterministic edge/vertex derivation and deletion
  lineage, consumes no point identity on preflight failure, restores future identities
  on rollback, and removes its hidden point only when it is unshared. **Landed for the
  facade-owned strut wrappers without exposing `PointId` or raw assembly.**
- Position-owning MVFS/KVFS emits deterministic vertex-from-point and body/point
  deletion lineage, consumes no point identity on preflight failure, restores all
  future topology and point identities on rollback, and retains a shared point.
  **Landed for the explicitly transient facade seed-body wrappers without exposing
  `PointId` or raw assembly.**
- Budget exhaustion and a checker-failing tolerance edit restore the prior entity and
  budget state; successful growth preserves imported origin and emits deterministic
  request-order usage/events. **Landed for transaction-owned face/edge/vertex tolerance
  growth and the facade batch, including identity/liveness-before-values, unique targets,
  model-resolution validation, complete-preflight/infallible-apply, exact aggregate N/N-1
  accounting, and journal-local non-authoring budget identity.**
- An opt-in commit can require Full `Valid` or accept explicit `Indeterminate` evidence;
  Full faults always reject. **Landed with exact explicit/affected-root ordering,
  duplicate suppression, one-scope Fast-plus-Full accounting, owned per-body reports,
  rollback-clean rejected decisions without journals, tolerance/index/future-ID restore,
  and exact graph-work N/N-1 denial. The existing Fast commit remains unchanged.**
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

Arbitrary cyclic B-geometry beyond the certified clamped-seam slice,
circular/periodic pcurve encodings, intersection curves, procedural surfaces,
curve-less tolerant ring/degenerate cases, general bodies, attributes,
transforms, instances, assemblies, and older schemas land with their kernel
dependencies. Full “any well-formed X_T” Tier 0 is not claimed until this matrix is
closed.

The gate fixture for the import half of this matrix is `exemplar.x_t`
(owner-contributed Onshape/Parasolid 37.1 production part, 2026-07-11: 96 faces
of which 44 sit on offset surfaces and 68 reference B-surfaces, 110 intersection
curves, 131 curve-less tolerant edges, 7,423 nodes). It parses fully today, and
the geometry-graph/offset reconstruction slice is complete. The three production
periodic/closed B-surface leaves now reconstruct through a certified clamped
position/C1 seam contract. The equal-limit form at records 1828 and 2008 is
certified when one shared or two distinct `H/?` limits identify the same
spatial point and the chart closes on exactly one periodic NURBS axis. Every
coordinate on that periodic trace axis is preserved modulo
at most one exact declared period; a unique predecessor-relative lift may
normalize interior aliases, while the result must traverse exactly one period
and pass the unchanged whole-range original-source certificate. The finite-open/end-
terminated `T/F` singular form at records
1671 and 1678 is also certified with one appended final span. Production
v5 additionally certifies finite-open direct B-surface/Plane record 1252,
recovering its paired-null interior Plane UVs by exact frame inversion while
requiring numeric endpoints and NURBS UVs. Production v6 then lifts native
Plane `SP_CURVE` node 30 exactly from its open, nonperiodic, nonrational 2D
B-curve controls. The affine Plane map preserves degree, knots, and
parameterization without approximation. The equal-limit proof now promotes
its closed carrier to periodic semantics only after one complete certified
NURBS seam crossing, so vertex-less EDGE 2210 remains a topology ring and FACE
1195 derives its finite domain. Production v7 then certifies finite-open
Plane/Offset(B-surface) `INTERSECTION` 5089 by recovering sample 2 operand 0's
paired-null interior Plane UV through exact frame inversion and rerunning the
whole-carrier Plane/Offset-NURBS proof. The same canonical family now permits
paired-null Plane UVs at endpoints: a synthetic record-5089 variant preserves
the exact v7 report, while endpoint displacement still fails that unchanged
proof. A separate exact variant retains record 778's finite positive
noncanonical `base_parameter`/`base_scale` metadata while keeping the shared
carrier/pcurves on their canonical sample-index basis; it pins cumulative
`139792442/4/10` Work/Items/Depth with exact N/N-1 rollback. Corpus records 778
and 3620 themselves remain rejected because their NURBS pcurves materially
leave the original source domains. V1-v7 remain unchanged; v7 pins its
next denied attempt at 285,283,414 Work. Production v8 certifies nonperiodic
NURBS/Offset(B-surface) `INTERSECTION` 1984 after snapping only the final first-
trace `u` value from `-2.02217766823431e-15` to its exact source-domain lower
bound, then rerunning the unchanged whole-carrier proof. V8 admits exact
`315245660/22/10` Work/Items/Depth and preserves its atomic stop at
`INTERSECTION` 5945's 323,814,492-Work proof preflight. Production v9 admits
that record at exact `323814492/22/10`. Its ordered Offset(B-surface) roots
`[3338, 773]`, three model-space positions, and twelve paired UV values define
one common degree-2 clamped carrier/pcurve basis; independent depth-10 interval
proofs against both original NURBS bases, distances, and unit-normal fields are
the acceptance evidence. Production v10 then admits four-sample cubic
dual-offset record 3819 at exact `336759900/22/10`. Its ordered roots
`[3370, 773]`, four positions, and sixteen paired UV values determine the
unique common degree-3 clamped interpolants, while two independent depth-10
original-source proofs remain authoritative. Historical v1-v9 profiles retain
exact parity. Production v11 normalizes only null or finite-numeric
zero-multiplicity nonperiodic source-knot padding. It admits quadratic
dual-offset record 3790 at isolated `8593408/3/10`, exposes and certifies the
existing 11-sample Plane/Offset(B-surface) record 3745 at isolated
`42772491/11/10`, and reaches exact `388125799/22/10` with historical v1-v10
parity. Production v12 admits seven-sample dual-offset record 3615 at isolated
`26443776/7/10`. Its ordered roots `[3374, 773]` retain bases `[3730, 1186]`;
the exact transmitted positions and paired UVs define a common degree-1
open-clamped polyline, while both original offset-NURBS proofs remain
authoritative. The corpus reaches exact `414569575/22/10` with historical
v1-v11 parity. Two-sample dual-offset record 3595, roots `[783, 773]`, now
certifies independently as the canonical open-clamped line at isolated
`4352000/2/10`; residual bounds
`[3.468467250779673e-5, 3.384554176162513e-5]` remain below its
`1.817100064117055e-4` chordal tolerance. It is not reached by production
traversal. Five-sample dual-offset record 4230, roots `[3320, 773]`, chart 4231,
independently certifies as a canonical degree-1 open-clamped polyline at
isolated `17285120/5/10`. Production v13 admits it at exact
`431854695/22/10` with historical v1-v12 parity. Production v14 admits
two-sample direct Plane/Offset(B-surface) record 3609, chart 3607, at isolated
`4277250` Work and exact cumulative `436131945/22/10`, preserving historical
v1-v13 parity. Production v15 admits two-sample dual-offset record 6044, chart
6043, at isolated `4352000/2/10` and exact cumulative `440483945/22/10`,
preserving historical v1-v14 parity. It then stops atomically before
four-sample dual-offset record 5921, chart 6027, whose isolated
`13774848/4/10` proof would request cumulative `454258793` Work. At that exact
attempted budget, the canonical cubic first pcurve materially exits its
original open nonperiodic source domain; certification fails atomically and
retains the v15 report. The exemplar
manifest row is therefore `reconstruct: fail` with no
unsupported capability and remains
the committed progress meter: `reconstruct: fail → pass`, then
`tessellate: pass`, then full-checker gaps shrinking toward `valid`. Dependency
plan:

1. **COMPLETE — geometry graph with the offset-surface evaluator as its first
   import client.** The M2.5-B evaluation context and `OFFSET_SURF`
   reconstruction on it: point/derivative evaluation through the basis
   surface's curvature, validity domains (an offset is singular where the
   distance reaches the basis's minimum concave radius of curvature),
   conservative work boxes, and pcurve-driven tessellation are landed. This is
   the graph substrate shared by exemplar reconstruction and future M4 proofs;
   its descriptors serve both clients without duplicated owned geometry.
2. **VERIFIED TRANSMITTED SLICES IN PROGRESS — import intersection curves without
   recomputation.** X_T transmits `CHART` model-space positions rather than an
   owned spatial B-spline. The landed exact-plane-field slices retain those
   positions and the modern `INTERSECTION_DATA(204)` paired UV tuples as one
   shared, polynomial degree-1 basis, then proves both lifted pcurves over every
   complete knot span with outward-rounded control residuals inside the owning
   body/chart tolerance. Ordered source and pcurve dependencies, affine chart
   metadata, and the proof persist in the geometry graph; natural edge bounds
   come from the retained carrier. Each source may be a direct plane or a safe
   finite constant-offset chain terminating at a plane, so Plane/Plane,
   Plane/Offset, and Offset/Offset orderings share one proof contract.
   Certificates bind the effective plane fields while retaining the actual
   ordered source handles and protecting every complete basis chain. Two
   Offset/Offset roots must have independent chains; shared or cross-linked
   chains fail closed. The accepted boundary remains intentionally narrow:
   two distinct nonparallel effective planes, at most two offset operands,
   finite open `L/?` limits plus the bounded end `T/F` singular terminator,
   `uv_type=4` UVs with paired-null recovery only for an exact Plane trace, and
   the canonical affine chart recurrence. A bounded finite-open two- through
   five-sample direct-Plane/B-surface, safe-Offset(Plane)/B-surface, direct-
   Plane/Offset(B-surface), direct constant-normal Offset(B-surface)/direct
   B-surface, independent direct one-descriptor Offset(B-surface)/Offset(B-surface),
   or direct-B-surface/B-surface slice may instead
   retain finite positive affine chart metadata while
   canonicalizing only the common sample-index basis. Direct B/B, direct
   Offset(B)/B, and independent direct Offset(B)/Offset(B) pin exact 2–5 sample
   Work/Items/Depth at `14336/2/10`, `28672/3/10`, `43008/4/10`, and
   `57344/5/10`. The offset slices cover source swap and polynomial or rational
   bases; the dual-offset arm retains both live roots, signed distances,
   independent direct original bases, paired UVs, and the unchanged two-source
   proof. Nested, shared-basis, multi-offset, null/mixed, and out-of-range
   noncanonical forms remain unsupported.
   Canonical
   Plane/B-surface, safe-Offset(Plane)/B-surface,
   B-surface/B-surface, direct constant-normal Offset(B-surface)/B-surface,
   and every applicable reversed operand order now retain
   the same degree-1 carrier/pcurves while a separate certificate proves each
   polynomial or rational source NURBS trace. The additional canonical
   Offset(B-surface)/Offset(B-surface) arms remain bounded to the
   finite-open two-sample degree-1 line, three-sample degree-2 quadratic,
   four-sample degree-3 cubic, five-sample degree-1 polyline, or seven-sample
   degree-1 polyline forms. Their
   model-space and canonicalized UV tuples share one exact open-clamped basis.
   Those retained/interpolated controls define only the candidate; both original
   offset-NURBS sources must independently pass the whole-range interval proof.
   A plane trace may bind either a direct plane or a safe finite offset chain
   resolving to an exact plane. A direct Offset(B-surface) trace instead binds
   the live offset root, signed distance, and original NURBS basis; each proof
   box outwardly encloses `du x dv`, proves a positive normal-length lower
   bound, and divides to the complete unit-normal field before applying the
   signed displacement. The persistent descriptor retains the actual ordered
   roots and protects every transitive basis. Every carrier span is
   subdivided to binary depth 10; original-source homogeneous point/partial
   intervals and a centered mean-value residual provide whole-range evidence
   without samples or spatial-intersection recomputation. For `N` chart
   positions, `P` exact-plane traces, and each NURBS trace span count
   `R_i=(nu_i-pu_i)(nv_i-pv_i)`, the NURBS descriptor pre-admits
   `P*N+(N-1)*2^10*sum_i(6R_i+1)` Work, `N` Items, and Depth 10, with exact
   N/N-1 rollback coverage. The canonical two-position, one-span B/B fixture
   consumes `14336/2/10`; the historical Plane/B fixture remains
   `7170/2/10`; the synthetic Offset(B)/B fixtures consume `14336/2/10`,
   `28672/3/10`, `43008/4/10`, and `57344/5/10` in both operand orders and
   polynomial/rational variants. Historical import profile v1
   remains capped at 131,072 Work and v2 retains its 81,267,732 Work
   compatibility boundary. Production v3 admits the exemplar through record
   1828 and all later equal-limit charts at exact `115485725/20/10`.
   Production v4 admits the first end terminator at exact `116396069/20/10`;
   the independently transplanted record-2008 and record-1678 payloads pin
   `124040223/22/10` and `116413476` Work. Production v5 then admits record
   1252 at exact `117478445/20/10`; its six interior Plane pairs are the only
   omitted UVs and are recovered through the exact direct-plane frame. V5
   remains capped there and pins the later attempted 118,406,196-Work chart
   crossing. Production v6 admits the native direct-Plane SP-curve lift and
   every later already-supported chart it exposes at exact
   `208228426/22/10` Work/Items/Depth, derives FACE 1195's vertex-less ring
   domain from certificate-owned periodic carrier semantics, then pins its next
   denied chart proof at 221,060,174 Work. Production v7 admits finite-open
   Plane/Offset(B-surface) record 5089 at exact `272430166/22/10`: only
   paired-null Plane samples may use exact frame inversion, with endpoints
   admitted only for this canonical Plane/Offset family; the existing
   whole-carrier certificate remains authoritative. A synthetic endpoint-null
   variant preserves the exact v7 report and next crossing. Production v8
   then admits `INTERSECTION` 1984 at exact `315245660/22/10`: only first/last
   NURBS coordinates may snap across a source-domain boundary within
   `16384 * EPSILON * domain-scale`, and the original-source whole-carrier
   certificate remains authoritative. Material or interior overhangs and
   displaced carriers remain typed and atomic. V8 still stops before
   `INTERSECTION` 5945's attempted 323,814,492-Work proof. Production v9 admits
   its bounded quadratic dual-offset certificate at exact
   `323814492/22/10`, with exact N/N-1 crossings and historical v1-v8 profile
   parity. Production v10 admits four-sample cubic record 3819 at exact
   `336759900/22/10`, with isolated `12945408/4/10` accounting, exact N/N-1
   crossings, and historical v1-v9 parity. Production v11 accepts only null or
   finite-numeric zero-multiplicity source-knot padding, certifies quadratic
   record 3790 and the exposed 11-sample Plane/Offset record 3745 at exact
   `388125799/22/10`, and preserves v1-v10 parity; seven-sample dual-offset
   record 3615 is admitted by v12 at exact `414569575/22/10`, with isolated
   `26443776/7/10` accounting and historical v1-v11 parity. Two-sample
   dual-offset record 3595 independently certifies at isolated
   `4352000/2/10` with exact per-resource N/N-1 rollback evidence. The
   five-sample record 4230 independently certifies at `17285120/5/10` and is
   admitted by production v13 at exact `431854695/22/10`. Production v14
   admits two-sample Plane/Offset record 3609 at isolated `4277250/2/10` and
   reaches exact `436131945/22/10` with historical v1-v13 parity. Production
   v15 admits two-sample dual-offset record 6044 at isolated `4352000/2/10`,
   reaches exact `440483945/22/10` with historical v1-v14 parity, and next
   stops before four-sample dual-offset record 5921's cumulative
   `454258793`-Work request. With that exact budget admitted, record 5921's
   canonical cubic first pcurve reaches `-0.08141266222011943` outside its
   original `[-0.01, 1.0]` source domain; certification retains the v15 report
   and the store remains empty.
   The corpus contains no NURBS-side paired-null UVs, and noncanonical records
   778 and 3620 materially exit their original NURBS domains. Both
   original B-surface identities and their paired pcurves are graph-protected
   in source order. Other parameter conventions, limits,
   nullable data, periodic/closed transmitted trace ranges, arbitrary unclamped
   cyclic bases, and unsupported closed forms
   fail with typed capabilities rather than being silently reparameterized. The
   production exemplar's three clamped periodic/closed B-surface leaves now
   reconstruct, wrap evaluation, bound seam-crossing ranges, and write matching
   periodic/closed flags. Its first Offset(B-surface)/B-surface chart now
   certifies. The one-period equal-limit records 1828 and 2008, including an
   exact synthetic non-endpoint interior alias range, and the
   end-terminated records 1671 and 1678, finite-open records 1252 and 5089, and
   native Plane SP-curve node 30 now certify; FACE 1195's vertex-less ring
   domain also derives. Nonperiodic NURBS endpoint-roundoff record 1984 and the
   canonical finite-open three-sample dual-offset record 5945 and four-sample
   cubic dual-offset record 3819, zero-padded quadratic record 3790, exposed
   11-sample Plane/Offset record 3745, seven-sample dual-offset record 3615, and
   the independently transplanted two-sample record 3595 and five-sample record
   4230 now certify. Production v14 additionally admits record 3609 at exact
   `436131945/22/10`; production v15 admits record 6044 at exact
   `440483945/22/10` and stops before record 5921's `454258793`-Work request;
   an exact-budget regression pins the subsequent original-domain certificate
   rejection and atomic v15-report rollback.
   Original-backed, tolerance-qualified, non-Plane, reversed-basis, periodic,
   closed, rational, or non-2D SP-curves, foreign curves, null/general
   closed-limit, endpoint or NURBS-trace omissions, other nullable
   chart-data, production-profile admission past the v15 record-5921 frontier,
   ambiguous or multi-period trace aliases, and broader noncanonical chart
   conventions remain. Re-deriving
   boolean scars through our own surface/surface intersector remains an M4
   concern; import must not wait on it.
3. **COMPLETE VERTICAL SLICE — certified clamped periodic/closed B-surfaces.**
   `kgeom` certifies polynomial seams against the owning BODY linear tolerance
   and rational seams by exact homogeneous position/first-partial equality;
   evaluation wraps certified directions, bounding boxes split seam-crossing
   ranges, and partitioning retains only untouched periodic directions.
   Reconstruction validates matching periodic/closed flags and knot-implied
   control counts, while the writer preserves the certified flags. Arbitrary
   unclamped cyclic bases, periodic B-curves, and broader closed-form encodings
   remain explicit typed boundaries.
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
- Target common SSI branch contract: a 3D curve, pcurve on surface A, pcurve on
  surface B, parameter correspondence, closure/end events, contact character,
  and a verified residual/error bound over the entire active interval—not only
  endpoint UVs. Narrow graph-aware Plane/Plane and Plane/Sphere branches have
  landed paired pcurves and whole-interval proofs; the common analytic result
  families have not yet adopted that complete branch contract.
- Represent coincident curve intervals and coincident surface regions separately from
  isolated contacts and ordinary branches.
- Landed substrate: NURBS-to-Bezier surface subdivision, positive-weight control-hull
  boxes, a reusable deterministic AABB BVH with outward-rounded separation queries, and
  interval-certified affine-plane patch exclusion, analytic plane/sphere/cylinder/cone/
  torus implicit fields over outward-inflated boxes, exact adaptive U/V subpatch isolation
  with conservative covers and structured limits, proof-bearing analytic/NURBS SSI
  empty exits, and deterministic NURBS curve-pair subdivision whose exclusions
  use original-source outward interval range boxes, with conservative covers
  and structured limits for complete curve/curve misses.
  Retained curve-pair cells now feed deterministic chord/midpoint seeds into
  safeguarded local Newton polishing; a finite seed-attempt stage bounds the
  work, and only re-evaluated in-cell residual witnesses are emitted. These
  discoveries remain indeterminate. Newton stops and fallback selection now
  have stable bounded diagnostics, with parameter-resolution retained even
  when diagnostics are off. Nested fallback minimizers now retain typed
  parameter-resolution, invalid-objective, and iteration-bound termination.
  Ordered incomplete proof obligations survive canonicalization, swapping,
  generic dispatch, and facade adaptation, and Q4 pins their semantic digest.
  Exact retained regions can now certify unique transverse roots for
  polynomial and positive-weight rational subcurves in arbitrary exact affine
  planes using robust coplanarity, Poincaré–Miranda face signs, and interval
  P-matrix univalence; Q4 pins polynomial, rational, and tilted proof digests.
  Deterministic joined-region ownership groups exact shared grid vertices and
  requires one certificate/witness per component, completing polynomial and
  rational boundary roots plus separated multi-root components. Partial
  evidence cannot upgrade completion. An outward-safe Euclidean distance lower
  bound between source-range position enclosures now excludes diagonal
  tolerance-empty cells beyond the axis-wise broad phase while retaining the
  inclusive boundary. Exact affine
  parameterizations with matching normalized knots, Euclidean control points,
  and globally proportional rational weights now supply complete clipped
  same/reversed overlap extents. Exact shared 3D corners extend unique-root
  proof to noncoplanar pairs whenever an interval P-matrix certifies a globally
  injective coordinate projection. Sampled near-coincidence remains
  indeterminate. Deterministic exact knot-insertion descendants now inherit
  complete clipped overlap evidence when reconstruction to a common knot
  multiset yields an exact normalized representation match. Different rounded
  insertion histories are now compared through bounded inverse candidates;
  every accepted predecessor must reproduce its descendant exactly through
  production reinsertion, so a common checked ancestor makes equivalence
  history-independent while altered data stays indeterminate. Full-
  multiplicity interior knots whose stored Euclidean points agree exactly add
  a noncoplanar interior existence witness before the same global injectivity
  proof. Candidate cells retain shared original-source provenance so rounded
  split controls cannot establish source roots or exclude source geometry.
  Exclusion boxes come from outward interval evaluation over original-source
  child ranges and fail open to the whole-source hull. A cubic/line adversary
  whose exact midpoint rounds outside every generated child hull remains
  nonempty, while the separate `2^-53` adversary guards against false
  certificates. On partial ranges, direct
  outward interval de Boor bounds over each original knot span enclose
  homogeneous positions and derivative B-splines; source-range Poincare signs
  and P-matrix bounds certify coplanar roots, while bounded exact
  `{mid,lo,hi}` source samples and in-range full-multiplicity knots provide
  noncoplanar existence. Exact normalized same/reversed parameter
  correspondence now adds a non-sampled algebraic route: proportional
  positive rational weights, a strictly monotone shared carrier, and exact
  omitted-coordinate controls lift a projected Poincare/P-matrix root to 3D.
  Canonical primitive integer carriers and residuals with coefficient magnitude
  at most two extend the exact lift to sources with no corresponding coordinate
  scalar or unit-coefficient form. Global-sign and scalar-multiple duplicates
  are removed by positive leading coefficients and gcd normalization. Direct
  homogeneous integer-form derivatives preserve correlation; swap, reversed
  affine domains, positive rational weights, and broken gates are covered.
  Both Q4 roots lie at normalized `1/3` inside partial ranges. The `2^-53` rounded-restriction adversary now stays
  inconclusive, while rational boundary and separated multi-root completion
  remain source-valid. Exact overlap scans, reconstruction, and inverse-state bounds now
  pre-admit conservative Work and Items; exhaustion returns structured
  indeterminate evidence before proof allocation. Curve-pair range enclosures
  also pre-admit every inspected original-source knot-span slot before
  evaluation; a denied child scan retains its parent. Surface-patch BVH,
  bounding-box, plane/implicit, and adaptive-child exclusion now use outward
  tensor interval bounds over original source rectangles, tightened by active
  source-support hulls and centered derivative bounds. An exact cubic extrusion
  whose rounded children lose a real contact remains nonempty. For
  `R=(nu-pu)(nv-pv)` source tensor slots, including repeated/empty slots, one
  range bound admits `6R+1` Work; contextual BVH build admits `R*(6R+1)` and
  each parent admits `1+4*(6R+1)` before any child scan. Exact repeated-knot,
  roundoff, and composed-marcher N/N-1 tests retain the source cover. Internal graph-owned facade
  evidence pins exact Work/Items N and isolated N-1 propagation for distinct
  checked-ancestor curves. Q4 isolation fixture v4, implicit-isolation v3, and
  solve fixture v18 pin resource
  usage/allowances, common-refinement and checked inverse-history success,
  altered-history rejection, ordered overlap extents and orientation,
  primitive magnitude-two through compatibility-default magnitude-twelve
  algebraic completion, and
  independent overlap Work/Items N-1 denial. G5a now adds invertible affine
  carrier/pcurve maps and whole-interval paired residual certificates. The
  graph-aware adapter builds deterministic verified branches for exact
  plane/plane lines and both common-axis and genuinely oblique plane/sphere
  circles while preserving source identity and raw-result parity. The
  common-axis fast path accepts either sphere-axis orientation, arbitrary plane
  rotation about that axis, and shifted, seam-crossing, full-turn, or overwide
  longitude windows through exact `t`/`-t` plane maps. Other finite regular
  secants use a private-field nonperiodic inverse sphere-chart pcurve with
  continuous seam unwrapping, analytic derivatives through order three,
  conservative bounds, and whole-branch pole/window proof. Each retained
  oblique branch pre-admits exactly 128 proof-subdivision Work units with pinned
  N/N-1 evidence. Direct fields and context-accounted plane/sphere offset chains
  share both arms; sphere chains fail closed if any effective radius is
  nonpositive or non-finite. Certified finite line/circle branches persist
  atomically with ordered source/pcurve dependencies and their proof. Tangencies
  stay vertex-only, misses preserve completion, and pole-crossing or out-of-window
  charts fail with precise typed errors. Exact direct/safe-Offset(Plane) and
  direct/safe-Offset(Sphere) fields against genuinely non-planar direct NURBS now run
  the fixed-grid marcher in that same owner scope, retain
  its degree-1 carrier and paired pcurves, and mint a separate non-transmitted
  whole-range certificate before atomic persistence. Exact certificate Work is
  `C + S*2^10*(6T+1)`; the curved one-segment fixture pins 7,170/7,169, and
  failed residual proofs retain attempted Work. Exact Sphere lifts add an
  outward centered mean-value interval per depth-10 cell; the one-segment
  paired proof pins 8,192/8,191 Work, 1,024/1,023 Items, and 10/9 Depth. Lower raw/report evidence,
  complete misses, indeterminate completion, canonical swap, and positional
  branch/trace identity remain unchanged. Safe sphere-offset chains retain the
  live root identity, protect every basis transitively, and pin direct-root
  graph visits and dependency depth at exact 2/1 boundaries; nested safe chains
  preserve the same effective-field proof. Compatible direct NURBS/NURBS fields
  now add one exact shared-chart arm: both genuinely non-planar sources must use
  the same finite-open quadratic-linear unit chart, identical constant weights,
  and finite requested windows with a positive-area axiswise overlap. Discovery
  is clipped to that exact shared rectangle. Its rounded scalar difference guides
  discovery only; outward original-control differences own complete misses and
  both original lifts are independently certified. The one-span paired fixture
  pins 14,336/14,335 Work, 1,024/1,023 Items, and 10/9 Depth; equal windows remain
  supported while disjoint or boundary-only windows fail closed. A capped
  Offset(NURBS)/direct-NURBS arm admits the exact constant-+Z-normal unit-chart
  terminal basis against a genuinely non-planar compatible peer. At most four
  offset descriptors are accumulated inner-to-outer, with every
  partial distance and outward basis lift required finite. Distinct finite
  operand
  windows are accepted only when their exact axiswise overlap has positive
  area, and discovery is clipped to that shared rectangle. It retains and
  validates the live outer root, accumulated signed distance, terminal original
  basis, direct source, and paired pcurves;
  outward original controls own misses while the rounded displaced surface is
  discovery-only. The paired proof keeps the same exact Work/Items/Depth
  boundaries; one, two, three, or four offset descriptors use exact 2/depth-2,
  3/depth-3, 4/depth-4, or 5/depth-5 graph traversal with matching N/N-1
  admission.
  The first varying-normal operation-generated arm accepts one offset
  descriptor over an exact rational quarter-cylinder extrusion and either a
  canonical bilinear planar direct-NURBS peer, direct analytic Plane, or one
  safe Offset(Plane) descriptor over a direct Plane basis normal to the global
  X, Y, or Z axis. The direct analytic Plane arm alone accepts the complete
  one- through four-descriptor family, retains every outer-to-inner signed
  distance, and proves every intermediate and final cylinder radius finite and
  positive. An original-derivative
  interval
  enclosure proves a nonzero normal over the complete positive operand window
  before the true rational parallel surface guides discovery; orientation-
  selected original control intervals, radially scaled only for X/Y, alone own
  complete misses. The normal proof costs
  7 Work, 1 Item, and Depth 1. Planar-NURBS X/Y generators pin combined
  14,343/14,342 Work and 1,024/1,023 Items, while their Z-normal peer pins
  573,447/573,446 Work and 40,960/40,959 Items on a certified 40-span,
  41-control horizontal quarter-circle chordal carrier. Analytic-Plane X/Y
  peers instead pin combined 7,177/7,176 Work and 1,024/1,023 Items; their
  40-span Z-normal peer pins 286,768/286,767 Work and 40,960/40,959 Items. All
  orientations and chain lengths retain 10/9 certificate Depth and unchanged
  certificate Work/Items budgets. One through four direct-Plane descriptors
  consume exact graph Work/depth 2, 3, 4, and 5 with N/N-1 admission; an
  Offset(Plane) peer pins 4/3 graph Work and 2/1 dependency
  Depth because both live roots traverse their direct bases. The paired trace
  retains the live NURBS root, original basis, peer root and direct Plane basis
  when present, and both pcurves. The original rational-quarter-cylinder basis
  remains proof authority; the derived rational effective sheet is discovery-
  only. Swap, persistence, and complete misses cover every admitted chain.
  Persistence rejects each altered descriptor, including a same-sum mutation,
  and stale roots or peers atomically. Singular or inconclusive normal fields,
  descriptor-chain depth five or greater, multi-descriptor planar-NURBS or
  Offset(Plane) peers, and incompatible sources fail closed and persist
  nothing.
  Two independent one- through four-descriptor Offset(NURBS) roots now share a
  compatible planar constant-normal unit-chart arm. Intersecting pairs cover
  the complete 4×4 chain matrix, retain both ordered live roots, both transitive
  basis chains, accumulated distances, terminal original bases, and paired
  pcurves, and certify one branch against both original sources; the normalized
  effective charts remain discovery-only. Every pair pins exact
  14,336/14,335 Work, 1,024/1,023 Items, and 10/9 certificate Depth. Graph
  traversal is exactly `A+B+2` Work and `max(A+1,B+1)` dependency depth, with
  the 4×4 maximum at 10/9 Work and 5/4 Depth admission. The strict-separated
  complete-miss path remains at those graph ceilings with zero certificate use
  or persistence allocation and preserved operand order.
  Incompatible planar or unaligned peers, disjoint or boundary-only ranges,
  unequal weights,
  collapsed or non-finite sphere-offset fields, positive
  Offset(NURBS)/direct-NURBS chains of five or more offset descriptors,
  other varying-normal Offset(NURBS) families, descriptor-chain depth five or
  greater, multi-descriptor peers outside direct analytic Plane,
  incompatible, coincident, five-or-more-descriptor, altered, or stale dual
  Offset(NURBS), broader NURBS/NURBS, and
  other procedural pairs remain typed unsupported.
  The compatibility magnitude-twelve rung runs the complete historical
  magnitude-eleven family first so all prior evidence remains stable, then
  admits only new carrier/residual pairs that reach twelve. An explicit
  validated search configuration can add the magnitude-thirteen or magnitude-
  fourteen shell while leaving twelve as the default and exact prefix. Every
  through-thirteen enumeration and certificate golden remains unchanged. The
  fourteen ceiling owns exactly 254 carrier forms and 9,825 residual forms,
  adding 24 and 1,704 respectively over thirteen. Its genuinely noncoplanar
  normalized-`1/3` fixture remains uncertifiable at every explicit ceiling
  through thirteen and certifies at fourteen, with standalone/candidate-cell,
  polynomial/rational, swap, and reversal parity. Direct correlated homogeneous
  derivative bounds accept coefficients only through the reviewed magnitude-
  fourteen corridor; invalid ceilings and broken, overflowing, non-finite, or
  out-of-range forms fail closed. Next, extend the adapter to further contextual
  non-plane fields and verified carrier families; coefficients above fourteen
  remain unsupported.
- Analytic special cases and the generic solver feed the same canonical result type.
- Consolidate the per-pair analytic curve/surface and SSI boilerplate behind
  shared drivers with the same result contract. The complete-support-curve
  emitter now owns clipping, periodic/nonperiodic membership, candidate
  reacceptance, and first-wins point/endpoint-aware branch dedup for
  cylinder/cylinder, cone/cylinder, cone/cone, cone/sphere, cylinder/sphere,
  and sphere/sphere. Pair-owned apex/tangent paths and sphere tolerance remain
  unchanged. The broader curve/surface rungs now route circle/ellipse ×
  cylinder, circle/ellipse × cone, circle/ellipse × torus, and circle/ellipse ×
  sphere through one config driver per surface family for validation, roots,
  tangency, contained overlap, window clipping, candidate admission, ordering,
  and completion; pre-refactor debug/release bit signatures and exact
  diagnostics are pinned. The analytic primitive-surface family is now
  consolidated. Circle/ellipse × NURBS now shares one marcher with explicit
  radial-circle and closest-projection ellipse strategies while preserving
  bit-exact diagnostics, completion, clipping, ordering, and classification.
  Circle/circle, circle/ellipse, and ellipse/ellipse now share one bounded
  conic-pair driver for validation, plane routing, inverse periodic fitting,
  contact classification, first-wins deduplication, and coincident periodic
  overlap construction. Their distinct quadratic/quartic root and projection
  arithmetic remains strategy-local, with pre-refactor debug/release result and
  operation-report bit signatures pinned. Ellipse/ellipse projection failures
  retain every typed `ProjectionError` as `IntersectionError::Projection`
  through generic dispatch and facade adaptation, including exact
  class/code/limit/source delegation and the nested policy source; its direct
  entry now returns `IntersectionResult` rather than performing a lossy core
  error conversion. Other solver-local error collapses remain migration debt.
  Bounded coincident plane/plane,
  exact coincident cylinder/cylinder, exact common-axis sphere/sphere windows,
  exact signed-coordinate-permutation sphere octants, and arbitrary-frame
  sphere octants now return dimensionally truthful complete evidence. Plane
  regions retain paired convex chart boundaries. Their strict first-chart
  convexity gate requires at least three finite vertices and an exact positive
  `orient2d` result at every turn, rejecting exact collinearity and all
  nonpositive turns. Cylinder and common-axis sphere regions retain paired
  seam-aware chart rectangles. Signed-coordinate-
  permutation octants retain a certifier-minted nonlinear bidirectional chart
  correspondence, three exact physical boundary anchors, and an outward
  operation-count/periodic-phase residual bound. Arbitrary-frame octants use
  robust six-halfspace topology, deterministic spherical-polygon anchors, a
  certifier-minted nonlinear bidirectional chart correspondence, and an outward
  whole-region residual bound; angularly ill-conditioned boundary planes fail
  closed rather than changing intersection dimension. Finite periodic
  representatives are admitted only while an outward endpoint phase bound
  remains inside the active angular-identity tolerance; more remote
  representatives fail closed before phase drift can change intersection
  dimension. Shared sphere-octant edges and vertices collapse to tangent branches
  or isolated contacts, poles are singular point evidence, and disjoint octants
  are proven empty. Exact coincident coaxial cone charts now retain paired affine
  longitude/slant correspondence across shifted reference origins and radii,
  rotated transverse frames, and reversed axes. Finite overlap regions split at
  the apex; collapsed latitudes and longitudes become tangent circles or
  rulings, the isolated apex is singular, and disjoint windows are proven
  empty. Whole-region residual bounds are outward, and overwide, noncanonical,
  near-coincident, or roundoff-unsafe chart families fail closed. Exact
  coincident torus charts independently split longitude and latitude seams;
  positive-area overlaps retain paired polygonal regions, while collapsed axes
  become exact latitude or meridian circle branches and collapsed rectangles
  become tangent points. Antiparallel frames preserve the signed two-axis chart
  map, and overwide, near-coincident, or roundoff-unsafe windows fail closed.
  These residual and singular-contact claims apply to the named landed slices,
  not to SSI families that still lack whole-branch certificates. A first
  certified general-window sphere arm now accepts exact coincident,
  arbitrary-axis, positive-area windows whose longitude spans are below π and
  whose latitude bounds clear both poles. It interval-classifies all 28 pairs
  of the eight window halfspaces, requires one connected degree-2 boundary
  cycle plus a strict interior witness before `Complete`, and retains source
  rectangles with nonlinear bidirectional correspondence rather than treating
  anchors as polygon interpolation. Containment, seam crossing, swap, the exact
  28/27 proof bound, and corrupt-anchor rejection are pinned. A second bounded
  proof scans at most 112 open arrangement arcs after those pair checks and
  returns `Complete` empty only when every boundary component is excluded; its
  disjoint exemplar pins exact 96/95 Work-style witness evidence and swap
  parity. Bit-exact equality locks additionally collapse to tangent circle arcs
  or points. A bounded polar arm accepts exactly one sub-π source window ending
  at one bit-exact natural pole while the other source window remains pole-clear
  and sub-π. It splits the polar latitude range into one closed pole-clear cell
  and one closed cap, requires both cells to certify empty or exactly one cell
  to own all retained evidence while its sibling certifies empty, and restores
  the parent correspondence only after sibling emptiness excludes the
  artificial latitude seam. The cap omits the collapsed pole-latitude plane as
  a redundant inequality; its longitude planes meet at one canonical singular
  pole anchor, and exact source-pole aliases map identically. The reviewed cap
  produces one three-anchor positive-area region with exact repeat/swap and
  outward residual evidence. Piece/pair/arc admission pins exact 2/1, 49/48,
  and 196/195 ceilings. One-ULP near-poles, two polar sources, wide polar
  sources, tangent boundary circles, and layouts occupying both latitude cells
  remain fail-closed. A broader polar-by-wide arm crosses those two latitude
  cells with the three closed sub-π longitude cells of exactly one pole-clear
  wide peer. It accepts all six cells empty, exactly one occupied child with
  five certified-empty siblings, exactly two edge-adjacent same-row children
  with four certified-empty siblings, an exact same-column vertical pair with
  the other four siblings certified empty, or the exact
  full three-child row in either latitude row with all three opposite-row
  siblings certified empty, one exact mixed-axis three-cell L path with the
  other three siblings certified empty, any exact four-positive grid path,
  exact lower/upper-stem T tree, or exact left/right 2×2 cycle with the other
  two siblings certified empty, or the exact disconnected outer-column
  vertical pairs `[0,0]`/`[0,2]`/`[1,0]`/`[1,2]` with middle-column
  `[0,1]`/`[1,1]` certified empty, or any of the four exact disconnected
  isolated-corner plus three-cell mixed-axis L layouts with the two omitted
  graph-cut siblings certified empty, or exactly five positive cells with the
  sole other sibling certified empty, or all six cells positive with no empty
  sibling. The two reviewed connected L orientations are
  the cap-right `[0,2]`/`[1,1]`/`[1,2]` path and the lower-middle
  `[0,1]`/`[0,2]`/`[1,1]` path.
  The single-child path excludes every
  artificial seam through sibling emptiness. The two-child path additionally
  requires one reverse-oriented, bit-exact shared edge on the applicable regular
  longitude or latitude seam, removes only that edge, rejects any surviving used
  seam or unused-longitude seam, and restores the parent correspondence. The
  vertical slice requires strict bit-exact latitude-seam cancellation; its
  reviewed fixture is `[0,2]`/`[1,2]`. The reviewed `[1,1]` single-cell
  fixture retains the canonical singular pole alias and three-anchor region;
  the reviewed `[1,0]`/`[1,1]` adjacent fixture retains that alias in one
  five-anchor region. A one-turn-shifted `[0,0]`/`[0,1]` lower-row fixture
  retains one six-anchor region without a pole alias. The `[0,2]`/`[1,2]`
  vertical fixture retains one merged region after exact latitude-seam
  cancellation and proves the other four siblings empty. A one-turn-shifted
  `[1,0]`/`[1,1]`/`[1,2]` fixture
  cancels both strict regular-longitude seams and retains the pole alias in one
  11-anchor region after the empty lower row excludes the latitude seam. The
  exact opposite non-cap row instead retains one canonical eight-vertex region
  after the empty cap row excludes the latitude seam. Each mixed-axis L path
  cancels exactly one reverse-oriented longitude seam and one
  reverse-oriented latitude seam, rejects a surviving used seam or an unused
  longitude seam, and restores one parent region. The generic four-positive
  arm admits a three-edge path with exactly two longitude and one latitude
  adjacency, backtracks only across exact association orders, and cancels all
  three artificial seams. Its real cap-row-right
  `[0,2]`/`[1,0]`/`[1,1]`/`[1,2]` and zigzag
  `[0,1]`/`[0,2]`/`[1,0]`/`[1,1]` fixtures pin repeat/swap; one-ULP seam
  mutation and duplicate-edge ambiguity fail closed. The path route stays
  exclusive to degree sequence `2,2,1,1`. A disjoint T route admits only the
  exact lower-stem `[0,0]`/`[0,1]`/`[0,2]`/`[1,1]` and upper-stem
  `[0,1]`/`[1,0]`/`[1,1]`/`[1,2]` trees with the other two siblings certified
  empty. Each has degree sequence `3,1,1,1`, proves all three reverse-oriented
  bit-exact internal adjacencies simultaneously before removing them, requires
  one unambiguous outer cycle with no artificial seam, and restores the parent
  mapping and maximum child/parent residual. Both real fixtures pin repeat/swap;
  one-ULP seam mutation and duplicate-edge ambiguity fail closed. The third
  disjoint route admits only the
  exact left `[0,0]`/`[0,1]`/`[1,0]`/`[1,1]` and right
  `[0,1]`/`[0,2]`/`[1,1]`/`[1,2]` 2x2 cycles with the other two siblings
  certified empty. It proves all four reverse-oriented bit-exact internal
  adjacencies simultaneously before removing them, requires one unambiguous
  outer cycle with no artificial seam, and restores the parent mapping and
  maximum child/parent residual. Both real fixtures pin repeat/swap; one-ULP
  seam mutation and duplicate-edge ambiguity fail closed. A separate
  disconnected route merges both exact outer-column latitude seams into
  exactly two canonical regions, excludes both longitude separators, restores
  the parent maps and maximum child/parent residuals, and pins repeat/swap,
  exact 6/5 piece, 147/146 pair, and 588/587 arc N/N-1 admission plus one-ULP
  and duplicate-edge ambiguity rejection. A second disconnected route admits
  all four isolated-corner plus three-cell mixed-axis L layouts:
  `[0,0]` + `[0,2]`/`[1,1]`/`[1,2]`, `[1,2]` +
  `[0,0]`/`[0,1]`/`[1,0]`, `[1,0]` + `[0,1]`/`[0,2]`/`[1,2]`, and
  `[0,2]` + `[0,0]`/`[1,0]`/`[1,1]`; respectively the omitted graph-cut
  sibling pairs `[0,1]`/`[1,0]`, `[0,2]`/`[1,1]`, `[0,0]`/`[1,1]`, and
  `[0,1]`/`[1,2]` must certify empty. It proves and removes both reverse-
  oriented bit-exact L seams, requires zero occupied-boundary contact with every
  empty cut separator and no bit-exact contact between the singleton and merged
  component, and returns exactly two canonical regions with restored parent
  maps and maximum child/parent residuals. The four real fixtures pin
  repeat/swap, the same exact 6/5 piece, 147/146 pair, and 588/587 arc N/N-1
  admission, plus one-ULP and duplicate-edge ambiguity rejection. Together
  with the outer-column vertical-pair route, these exhaust the exact
  disconnected four-positive graph layouts in the polar-by-wide 2×3
  decomposition. The
  five-positive arm simultaneously removes all internal reverse-oriented bit-
  exact seams and requires one unambiguous outer cycle. Its `[0,0]` corner-
  empty fixture is a 2×2 cycle plus tail; its `[1,1]` edge-middle-empty fixture
  is a tree. Both pin repeat/swap, restored parent correspondence, outward
  residuals, and complete seam removal; one-ULP and duplicate-edge ambiguity
  mutations fail closed. The all-six-positive arm admits the complete 2×3 grid
  with no empty sibling only after all four longitude and three latitude
  adjacencies prove reverse-oriented and bit-exact simultaneously. It removes
  all seven internal edges, requires one unambiguous outer cycle with no
  artificial seam, and restores the parent mapping and child/parent residual.
  Its real `0.14716980102990423`-tilt fixture pins repeat/swap plus one-ULP and
  duplicate-edge ambiguity rejection. All admitted layouts retain exact 6/5
  piece, 147/146 pair, and 588/587 arc admission. The path (`2,2,1,1`), T
  (`3,1,1,1`), and cycle (`2,2,2,2`) routes remain disjoint. Other polar
  layouts outside the admitted exact same-row, same-column, full-row,
  mixed-axis three-cell-path, connected four-positive path/T/cycle,
  disconnected four-positive graph, exactly-five-positive, and all-six-positive
  families remain fail-closed. A first wide
  arm splits exactly
  one pole-clear wide operand into
  three closed sub-π cells and returns `Complete` only for three certified-empty
  cells or one positive region with two certified-empty siblings; sibling
  emptiness cancels the artificial seams before parent correspondence is
  restored. Its piece/pair/arc ceilings pin 3/2, 84/83, and 336/335 evidence.
  When both operands are wide, a bounded arm checks the Cartesian 3×3 grid and
  returns `Complete` after all nine closed child intersections certify empty,
  when exactly one child owns one positive region and its eight siblings
  certify empty, when exactly two children own one positive region each and
  the other seven certify empty, when exactly three positive children are
  pairwise non-edge-adjacent and all six siblings certify empty, when exactly
  three positive children comprise one exact adjacent pair plus one isolated
  component and all six siblings certify empty, when exactly three positive
  children form a two-edge grid path and their six siblings certify empty, or
  when exactly four positive children form a three-edge grid path and their five siblings
  certify empty, or when exactly five positive children form a four-edge grid
  path and their four siblings certify empty, when four, five, or six positive
  children form an exact connected shared-seam union, when seven positive
  children form an exact connected shared-seam union and the other two
  siblings certify empty, when seven positive children instead comprise one
  occupied corner singleton, its two orthogonal neighbor cells certified
  empty, and one exact connected six-cell component in any of the four grid
  rotations, or when eight
  positive children form an exact connected shared-seam union and the sole
  remaining sibling certifies empty, or when all nine children are positive
  and their exhaustive closed decomposition cancels every internal seam into
  one unambiguous outer cycle. Multi-cell parents
  must remain below a full turn. Two or three pairwise non-edge-adjacent
  children stay as separate components after closed sibling ownership excludes
  every artificial seam, including diagonal corner contact through both
  certified-empty orthogonal owners. In the mixed three-cell case, the sole
  adjacent pair merges while the separated singleton retains canonical grid
  component order. Edge-adjacent children merge
  only when each shared seam is one reverse-oriented boundary edge with exactly
  two consecutive, bit-identical endpoint records; each edge is removed and
  the complementary paths are spliced before parent correspondence is restored.
  Three- through six-cell paths recheck every remaining seam against the current
  merged boundary after each earlier splice. Connected four- through nine-cell
  non-path unions prove all internal seams together. Paired owners require
  reverse-oriented bit-exact edge records; one exact owner may canonicalize a
  unique reverse consecutive edge only when its endpoints are bit-identical in
  the unchanged pole-clear complementary chart and are not a non-exact
  multi-owner grid corner. The remaining edges must trace one unambiguous outer
  boundary cycle. The certified
  five-cell 2×2-cycle-plus-tail fixture has one 12-edge boundary, and the exact
  connected six-cell fixture has one 14-edge boundary; bit-mismatched
  central seam records remain indeterminate even when their physical points
  differ by only a few machine epsilons.
  Exactly five positive cells may also remain as multiple canonical components
  when every cross-component pair has certified sibling emptiness, including
  both orthogonal empty owners at each diagonal corner. The pinned singleton
  plus four-cell fixture retains two regions with 3- and 8-edge boundaries;
  removing either separator owner fails closed.
  The exact disconnected seven-positive family covers all four rotations:
  singleton `[0,0]` with cuts `[0,1]`/`[1,0]`, singleton `[0,2]` with cuts
  `[0,1]`/`[1,2]`, singleton `[2,0]` with cuts `[1,0]`/`[2,1]`, and singleton
  `[2,2]` with cuts `[1,2]`/`[2,1]`; every non-cut cell is occupied and the
  other six cells form one component. The existing connected-seven merger has
  precedence. The disconnected route requires both cuts certified empty and
  zero occupied-boundary contact at their separators, proves and cancels all
  six internal seams of the six-cell component simultaneously, and rejects any
  surviving artificial seam edge or bit-exact contact between components. It
  returns exactly two canonical parent-mapped regions with maximum child/parent
  residual propagation. Real fixtures for all rotations pin repeat/swap, exact
  9/8 piece, 252/251 pair, and 1,008/1,007 arc N/N-1 admission; one-ULP seam
  mutation and duplicate-edge ambiguity fail closed.
  The reviewed seven-cell 2×3-block-plus-tail fixture now returns one canonical
  15-edge region: seven adjacencies own reverse-oriented bit-exact seam pairs,
  while `[1,0]/[1,1]` uses the one-owner proof above. Closed-cell containment
  and pole-clear constraint topology prove the whole shared arc, and operand
  swap is bit-exact. Approximate complementary endpoints, ambiguity, and the
  existing one-ULP central-grid adversary remain fail-closed.
  A second reviewed seven-cell fixture leaves the opposite corner cells
  `[0,2]` and `[2,0]` certified empty. Its two nonadjacent notches give it a
  different adjacency graph from the 2×3-block-plus-tail fixture; the unchanged
  generic merger proves every current internal seam and returns one canonical
  17-edge outer cycle with exact repeat/swap and parent correspondence.
  The reviewed eight-cell fixture occupies every grid cell except certified-
  empty `[2,1]` and returns one canonical 18-edge outer cycle. All nine occupied
  adjacencies own reverse-oriented bit-exact seam pairs; repeat and operand swap
  are bit-exact, while the general merger retains the bounded one-owner rule.
  A second reviewed eight-cell fixture leaves the corner cell `[2,0]`
  certified empty rather than an edge-middle cell. Its distinct corner-notch
  adjacency topology passes the same generic exact seam proof and returns one
  canonical 17-edge outer cycle with exact repeat/swap, proving the arm is not
  limited to the original all-but-`[2,1]` layout.
  The reviewed nine-cell fixture needs no empty sibling because its closed 3×3
  decomposition exhausts the parent. All twelve internal adjacencies certify
  and cancel, leaving one canonical 17-edge outer cycle with exact repeat/swap;
  no artificial seam edge survives.
  Exact 9/8 piece-pair, 252/251 boundary-pair, and 1,008/1,007 arc-witness
  admission remain pinned. Other seven-positive unions outside the exact
  connected and four corner-singleton-plus-six-component families, other eight-
  or nine-positive unions, disconnected five-cell
  layouts without exact sibling separation, non-exact or otherwise ambiguous
  multi-edge shared seams, full-turn
  aliases, other polar, non-exact tangent, ambiguous multiple-cycle,
  and near-coincident
  non-identical cases remain `Indeterminate` or on their existing typed
  failure boundary.

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

- Extend the landed polygon-with-holes `PlanarProfile` and checked planar-sheet builder
  to curve loops, nesting/material islands, and arrangements without weakening their
  exact-sign rejection of degenerate, intersecting, touching, or unnested boundaries.
- The first deterministic complete-body transform slice is implemented: an
  orientation-preserving rigid placement duplicates the full topology and geometry
  ownership closure, including offset bases and pcurves, preserves bounds, tolerances,
  and periodic-chart metadata, and records `DerivedFrom` lineage for every new identity
  before checked atomic commit. Plane/Plane verified lines copy direct Plane
  sources or safe finite Offset(Plane) chains, while Plane/Sphere latitude or
  oblique circles copy direct Plane or safe finite Offset(Plane) sources plus
  direct Sphere or safe finite Offset(Sphere) sources whose effective Sphere
  radius is positive and finite, in either order. Both paths transform
  the carrier, copy aligned Plane and latitude pcurves into the copied effective
  surface frames or regenerate the oblique spherical pcurve, and reissue the
  whole-range certificate before graph insertion. The copied proof closure is
  leaf-inclusive with depth at most 64. Facade preflight additionally admits
  every current operation-generated `VerifiedNurbsIntersection` family:
  Plane/NURBS, Sphere/NURBS, direct NURBS/NURBS, one- through four-descriptor
  Offset(NURBS)/NURBS, compatible one- through four by one- through four dual
  Offset(NURBS), and retained Offset(NURBS)/Plane variants in either supported
  order, including polynomial/rational traces and oblique frames. Copy retains
  ordered roots and every transitive basis, transforms the carrier and both
  original analytic/NURBS proof fields, preserves range, degree, knots,
  weights, periodicity, paired pcurves and tolerance, and reissues the
  whole-range family certificate before insertion. The transmitted tranche
  reissues Plane/Plane charts over direct or safe nested exact-plane roots,
  direct Plane/NURBS in both orders, direct NURBS/NURBS, direct one-descriptor
  Offset(NURBS)/NURBS in both orders, and exactly one-descriptor
  Offset(NURBS)/direct-Plane charts in both orders, plus only the canonical
  finite-open two-sample degree-1, witnessed three-sample quadratic, witnessed
  four-sample cubic, canonical five-sample degree-1, and canonical seven-sample
  degree-1 dual Offset(NURBS) charts in either ordered-root arrangement, through
  the public original-source recertifier. These complete the existing canonical
  2/3/4/5/7-sample set. The two-sample line, witnessed three-sample quadratic,
  and witnessed four-sample cubic each admit independent exact ordered chains
  of one through four descriptors per root across a full 4×4 matrix and both
  trace orders; the five- and seven-sample families remain exactly one
  descriptor per root. All five require distinct ordered roots with distinct
  direct nonperiodic terminal NURBS basis handles. The line uses unweighted
  two-control carrier/pcurves on
  `[0,0,1,1]` over `[0,1]` without witnesses; the quadratic uses unweighted
  degree-2 three-control carrier/pcurves on `[0,0,0,2,2,2]` over `[0,2]`; and
  the cubic uses unweighted degree-3 four-control carrier/pcurves on
  `[0,0,0,0,3,3,3,3]` over `[0,3]`. Both witnessed higher-order families retain
  exact position and paired-UV interpolation witnesses. The five-sample family
  uses unweighted degree-1 five-control carrier/pcurves on
  `[0,0,1,2,3,4,4]` over `[0,4]`; the seven-sample family uses unweighted
  degree-1 seven-control carrier/pcurves on `[0,0,1,2,3,4,5,6,6]` over
  `[0,6]`. Neither polyline family has interpolation witnesses or a carrier
  period. Generic graph persistence walks each dual root to its direct NURBS
  terminal and binds its complete outer-to-inner distance order bit-for-bit; it
  atomically rejects reordered same-total, extra, missing, or stale chains. The
  graph trace constructor retains at most four exact ordered descriptors per
  trace and rejects a fifth; a live depth-five source paired to the maximum-
  depth trace therefore fails atomically at graph insertion. Extending beyond
  four remains graph representation/binding work, not facade-copy preflight.
  Copy transforms both distinct terminal bases and the line, five-sample, or
  seven-sample carrier; for either witnessed higher-order family it transforms
  the exact positions and
  rebuilds carrier controls by the public interpolation formula. It copies both
  ordered roots and full offset/basis and pcurve chains; preserves the exact
  distance order, terminal source binding, chart metadata, tolerance, and exact
  UV witnesses; publicly recertifies—including every witnessed quadratic and
  cubic 4×4 pair—and protects copied roots and complete basis closures
  transitively. The facade validates the live roots and this proof/source
  contract before scope creation; graph-valid shared-basis or
  periodic charts, nested five-/seven-sample roots, Offset(Plane) peer roots,
  altered higher-order witnesses, other sample counts, nonpositive or nonfinite
  effective spheres, and other unsupported proofs remain typed unsupported. Lower
  transaction rejection restores every Body/Region/Shell/Edge/Vertex and
  Curve/Surface/Pcurve/Point count plus future point identity reuse.
  Attributes remain blocked because no authorable storage contract exists;
  non-rigid transform families also remain. Extend this seam only after those
  explicit contracts land.
- The first checked extrusion slice is implemented for one validated polygonal
  profile with holes along any finite translation having a nonzero component
  on the profile-frame normal. `extrude_profile_along_in` classifies the exact
  stored-frame `(x, y, translation)` scalar triple with `orient3d`: exact
  coplanarity fails before allocation, while a negative sign reflects the
  profile chart and reference normal together before using the same certified
  branch without changing model-space points or translation. The exact-sign
  fixture has integer-source normal dot `+3` although the old normalized dot
  rounds to zero; both directions construct deterministically, and the ordinary
  oblique fixture remains Full-valid. The builder creates exact planar cap
  regions plus one planar parallelogram per boundary segment, shares every
  perimeter and sweep edge, authors a line pcurve on every fin from the actual
  cap/side frames, and commits atomically with a deterministic creation journal. Full checking
  proves the translated cap bijection and affine side-ring embedding; the
  oblique holed fixture is watertight and has signed volume 24 within `1e-9`.
  The typed facade exposes both the height wrapper and translation request
  without a lower-layer type. Extend this to curved profiles, zero-normal
  degenerate sweeps, axis contacts, and revolve.
- Exercise seams, axis contacts, caps, inner loops, and full/partial revolutions.

### Exit gate

The adversarial CC/CS/SSI battery includes tangencies, near-coincidence, small loops,
singularities, and NURBS-vs-NURBS cases. Every `Complete` result is independently
verifiable; unresolved cases return `Indeterminate` or a typed limit. Extrude/revolve
outputs are checker-v2 clean, journaled, watertight, and externally X_T validated.
The landed polygonal extrusion already satisfies the checker, journal, and
watertight portions; external X_T validation remains part of the broader gate.

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

- M3b: keep the historical 14-file host certification and current declared
  15-file oracle bundle distinct while preserving writer-byte invalidation.
  The curved B-surface fixture remains locally verified and explicitly pending
  host certification. The 2026-07-11 Onshape run is machine-fingerprinted; remaining
  evidence is the named wire/acorn rejection, host-canonicalized analytic NURBS
  fixtures, offset-sheet re-export gap, and two preserved host re-export reader
  gaps.
- M3c: broaden the verified transmitted-chart import beyond the landed
  certified clamped periodic/closed B-surface reconstruction and the
  landed canonical Plane/Plane, Plane/Offset, Offset/Offset, Plane/B-surface,
  safe-Offset(Plane)/B-surface, B-surface/B-surface, and direct
  Offset(B-surface)/B-surface and native direct-Plane SP-curve slices past the
  exemplar's now-certified vertex-less ring-domain boundary: broader
  SP/foreign curves, null, mixed/non-`H`, or broader closed limits, remaining
  nullable chart data including NURBS-side omissions, ambiguous or multi-
  period trace aliases, and noncanonical chart variants outside the bounded
  direct-Plane/B-surface, safe-Offset(Plane)/B-surface, direct-
  Plane/Offset(B-surface), direct Offset(B-surface)/direct B-surface, and
  independent direct one-descriptor Offset(B-surface)/Offset(B-surface), and
  direct-B-surface/B-surface affine slices; nested, shared-basis, multi-offset,
  null/mixed, and out-of-range noncanonical forms remain outside the landed slice.
- M2.5: finish parameter-space incidence and ratcheted Full-checker proofs for
  periodic/mixed boundaries, multi-loop containment, and curved shells; define
  operation-specific tolerance combination/propagation policies beyond the
  landed MEF inheritance, KEF ordered maximum, and generic failure-atomic
  facade batch. The opt-in Full-assurance write gate is landed.
- M4: extend the narrow landed graph-aware Plane/Plane and Plane/Sphere paired-
  pcurve and whole-interval residual proofs across the common SSI branch
  contract; extend bounded Plane/Plane, cylinder/cylinder, common-axis
  sphere/sphere, exact signed-axis and arbitrary-frame sphere-octant, coaxial
  cone, and torus region/contact evidence to general non-octant arbitrary-axis
  sphere and remaining coincident or singular families; and generalize the
  landed exact-cell root/overlap certificates, bounded in-cell tolerance
  witnesses, and typed local-solver stops to complete solver-integrated
  coverage. Extend the landed nonzero-normal oblique polygonal-profile
  extrusion to curved profiles, zero-normal degenerate sweeps,
  revolve, and external X_T validation while preserving its atomic journal,
  pcurve, and Full-proof contracts.
- M5: grow planar profiles and booleans only after facade adoption and the
  checker, rollback, lineage, tolerance, determinism, corpus, performance, and
  independent-oracle gates.
- Production-scale and assembly imports must exercise the Q2a construction and
  Q2b traversal ladders. Reverse-index and traversal membership indexing have
  landed against deterministic audit/digest evidence; future representation or
  body-order optimization still follows named-host measurements and retains
  full-reconstruction audits.
- The Q2 mixed-store cohort matrix separates total bodies from affected roots
  with exact scope counters through 256 total and 64 affected bodies. Its first
  production-solid-footprint matrix remains the unchanged seven-row crossed
  `primitive_mix` grid: 1/4/16/64 affected roots at 64 total bodies and one
  affected root at 4/16/64/256 total bodies. It pins exact ordinary-operation-scope,
  tolerance-journal, affected/order, before/after-store/output, and
  installed-index evidence. The distinct four-row fixed-64 block-cohort ladder
  selects body ordinals divisible by five at 1/4/8/13 affected blocks (all 13),
  because cylinder and cone ring solids have no vertices. One ordinary checked
  scope grows each selected block's first Face, Edge, and Vertex in that order
  to `2e-8` under the exact `3N × 1e-8` budget. It pins N mutations of each
  entity type, all ordered events, affected/refreshed/checked = N,
  affected/store/output ratchets, installed-index equality, and repeat parity.
  Those prior 39 Q2 rows remain unchanged. The distinct production-clean
  ladder holds unchanged `primitive_mix` totals of 4/16/64/256 and times one
  ordinary `commit_checked(&[])` scope with zero edits or scoped bodies. It
  pins committed success, equal before/after store and index snapshots, stable
  store/index/output ratchets, and repeat equality. This first production-solid
  global ordinary-commit baseline covers the combined current validation,
  index-clone, and body-order path; it does not isolate phases. Phase counters
  and optimization, broader heterogeneous production edit footprints, and
  production assemblies are still explicit performance boundaries.

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
