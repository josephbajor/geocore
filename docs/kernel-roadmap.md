# Kernel Construction Roadmap

Companion to [kernel-spec.md](kernel-spec.md). The specification defines the target
contract; this document defines milestone dependency order and the gates that prevent
a locally successful prototype from being mistaken for a conformant CAD kernel.

[`../ORCHESTRATION.md`](../ORCHESTRATION.md) owns the standing rules, the queue head,
and success metrics. [`projects/foundation-projects.md`](projects/foundation-projects.md)
owns per-project contracts and open items. This roadmap owns milestone contracts,
dependency rationale, and long-horizon exit gates; nothing here independently reorders
the active queue. Detailed implementation history lives in git, and per-capability
status lives in the machine-readable ledger
[kernel-support.tsv](kernel-support.tsv), which changes only when named exit evidence
lands.

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
modeling operation is complete only when it is failure-atomic, journaled,
checker-clean, deterministic, corpus-tested, and explicit about unsupported or
indeterminate cases.

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

M3 continues in parallel because real X_T files are test infrastructure. M2.5 is a
hard gate for the *public contracts* of general intersection and modeling operations:
analytic experiments may land, but they must not lock in topology or SSI
representations that cannot carry pcurves, tolerances, completion evidence, and
journals.

## Current status snapshot

| Milestone | Status | One-line meaning |
|---|---|---|
| M0 Foundations | IMPLEMENTED SLICE | Exact predicates, intervals, tolerances, arenas, deterministic math and journals exist; the broader topological-decision audit and several fallback envelopes remain open. |
| M1 Geometry | IMPLEMENTED SLICE | Analytic + clamped-NURBS geometry with exact splitting/restriction and certified boxes; periodic/procedural NURBS remain. |
| M2 Topology | IMPLEMENTED SLICE | Full B-rep hierarchy, Euler operators, checked transactions, Fast/Full checker, watertight tessellation; general bodies and degenerate classes remain. |
| M2.5 Architecture gate | IN PROGRESS / REQUIRED | Pcurves, tolerant edges, tolerance provenance, checked assembly, journals landed; seam/pole interchange fixtures, remaining Full-proof gaps, and tolerance-propagation policies remain. |
| M3 X_T | IN PROGRESS | Modern-schema subset reads/writes with host-certified conventions; production coverage and re-certification remain (see [oracle-loop.md](oracle-loop.md)). |
| M4 Intersections/profile ops | PROVISIONAL / GATED | Broad analytic special cases with explicit Complete/Indeterminate evidence plus first modeling consumers; general certified CC/CS/SSI does not exist yet. |
| M5 Analytic booleans | IMPLEMENTED SLICE | Public block/block CSG is atomic, Full-checked, deterministic, host-validated. Axial rings drive block/cylinder intersections, bands, host features, two-ring unions, and both containment cavity orientations; certified support relations prove exact-contact intersections empty and realize flush cap-contact unions. Bounded-arc contacts, broader layouts, and curved external evidence remain. |
| M6–M8 | NOT STARTED | General modeling, blends, stable API, and production hardening remain. |

## Reconciled critical path

The architecture is directionally correct: pure geometry, handle-based B-rep topology,
per-incidence pcurves, checked Euler edits, transactions/journals, and interchange as
a parallel corpus source are the right layer boundaries. The main risk is breadth
outrunning proof-bearing contracts. Work advances through these gates in order:

| Order | Delivery tranche | Required result | What it unlocks |
|---|---|---|---|
| 1 | Close M2.5 topology contracts | Production seam/pole/apex interchange fixtures; tolerance combination/propagation policies beyond MEF/KEF; discharge remaining checker-v2 Full proof gaps. | A B-rep that intersections and features can modify without inventing representation rules mid-boolean. |
| 2 | Build the M4 proof substrate | Geometry-graph descriptors for procedural/intersection curves; generalize verified seed/polish and exact-cell certificates over adaptive NURBS/implicit isolation; paired pcurves and verified residual bounds on the common branch contract. | Certified general CC/CS/SSI and trustworthy empty results. |
| 3 | Ship one end-to-end feature ladder (**queue head — see ORCHESTRATION.md**) | Point classification, then block/block and block/cylinder booleans: atomic, journaled, checker-v2 clean, externally X_T checked. | The first honest CAD modeling vertical slice. |
| 4 | Broaden general modeling | Analytic booleans, periodic NURBS booleans, sweep/loft, sewing/healing, STEP. | General mechanical part construction and imported-body repair. |
| 5 | Add local/advanced features | Fillet/blend, chamfer, offset, shell, draft, replace/delete-and-heal, production API and performance hardening. | The operation breadth expected by a full CAD application. |

Activities that do **not** advance the critical path on their own: sampled
pair-specific intersection solvers without completeness evidence; another X_T node
class without checker/tessellation and declared capability outcomes; self-round-trip
as interchange certification; reopening direct `Store` mutation; happy-path unit
tests without adversarial, minimized, and production cases; and — per ORCHESTRATION
R1 — any hand-enumerated case taxonomy presented as a general algorithm
(`sphere_sphere.rs` reached 9,400 lines that way before being replaced by the general
seam-cancelling merger; do not recreate that pattern).

---

## M0 — Foundations — IMPLEMENTED SLICE

`crates/kcore`: Shewchuk-style exact predicates (`orient2d`/`orient3d`/`incircle`,
polygon orientation, quadratic/harmonic root classification, exact affine and
squared-distance signs) with conservative interval filters and exact expansion
fallbacks; interval arithmetic; the session tolerance regime; typed generational
arenas with copy-on-write undo frames; deterministic index-ordered parallel map
primitives; kernel-owned deterministic sin/cos/atan2 (musl port — platform libm is
banned via clippy `disallowed-methods`). Cross-platform bit-determinism is enforced
by golden-hash suites in CI (`crates/kcore/tests/determinism.rs`).

Exact-sign consumers migrated so far include trim-loop winding, polygonal-region
canonicalization, ordinary-face outer-loop selection, SSI polygon convexity,
coincident Plane/Plane hulls, oblique-extrusion direction, conic/primitive identity
gates, periodic side-loop ordering, the bounded line/Torus quartic, NURBS/Plane sign
bands, and stable extreme-scale vector/direction normalization. Evidence: the kcore
and kgeom unit/integration suites; per-decision fixtures cited in git history.

Conformance debt:

- Add `insphere` when a 3D Delaunay or equivalent consumer first needs it.
- Continue the repository-wide decision audit: exact predicates or certified
  intervals govern topology; metric tolerance governs proximity only. Remaining
  targets include curved-pcurve signed line integrals, curved/periodic containment,
  amplitude policy in higher conic/primitive families, fallbacks outside reviewed
  exponent envelopes, general NURBS root classification, and raw extreme-scale
  vector arithmetic.
- Continue replacing catch-all `InvalidGeometry` with stable typed categories;
  solver-local collapses and legacy compatibility wrappers remain.
- Remove panics from public kernel operations; invalid caller input returns typed
  errors.
- Add deterministic reduction primitives when the first real parallel consumers land.

**Exit:** adversarial predicate suites pass; every public operation is panic-free for
invalid inputs; a decision audit finds no uncertified topological sign decisions;
error behavior is stable enough for the eventual C ABI.

## M1 — Geometry core — IMPLEMENTED SLICE

`crates/kgeom`: analytic curves/surfaces with exact patch boxes; clamped rational and
polynomial NURBS evaluation, knot insertion/refinement/splitting, exact
restriction/Bezier extraction, conservative control-hull boxes; interval-certified
patch/plane and analytic implicit-surface exclusion; deterministic exact subpatch
isolation with structured limits; multi-start projection; deterministic trimmed-face
tessellation with explicit refinement-limit errors; evaluator conformance harness.

Debt and delivery points:

- **Before M4 certified general intersections:** verified root seeds with safeguarded
  polishing over adaptive candidate covers; interval contracts for procedural and
  NURBS targets; periodic/unclamped bounding and refinement; evaluator conditioning
  and singularity information; projection APIs distinguishing converged,
  indeterminate, and failed searches.
- **Before M3 production Tier 2 / M6:** periodic NURBS curves/surfaces, collapsed
  patch detection, degree elevation, knot removal, fitting with verified error.
- **Through M2.5/M6:** intersection, SP, trimmed, swept, spun, offset, and blend
  geometry as exact procedural classes where X_T requires them.
- Curvature/conditioning in the evaluator protocol before blends and offsets.
- Tessellation: angular tolerance, triangle quality, incremental invalidation,
  deterministic per-face parallelism.

**Exit:** every target geometry class passes evaluator, periodicity, singularity,
projection, and bounding tests; NURBS operations pass published-value and randomized
invariance tests; approximate constructions carry verified error bounds.

## M2 — Topology + primitives — IMPLEMENTED SLICE

`crates/ktopo`: body→region→shell→face→loop→fin→edge→vertex entities over typed
arenas; the ten Euler operators with inverses and a randomized Euler–Poincaré
harness; block/cylinder/cone/sphere/torus constructors; the structural/sampled Fast
checker plus checker-v2 Full reporting with explicit `Valid`/`Invalid`/`Indeterminate`
outcomes and typed verification gaps; edge-once watertight body tessellation.

Known limits: `General` mixed-dimension bodies, face tolerances/domains beyond the
landed metadata, curve-less ring edges, isolated loops, and several pole/apex
topologies are unsupported. Fast checking samples some incidence; Full checking
proves the supported analytic incidence and polygonal containment classes and returns
explicit gaps elsewhere. M2.5 closes these before booleans.

## M2.5 — Boolean-ready architecture gate — IN PROGRESS / REQUIRED

### A. Parameter-space incidence — landed contract

- Every fin may carry an independent `FinPcurve` with a finite range, an invertible
  affine edge-to-pcurve map, an explicit `PcurveChart` of integer period shifts,
  optional closed-use winding, singular endpoint markers, and explicit
  `PcurveSeam` roles for full-period chart cuts. Public MEV/MEF/MEKR face-edge
  creation requires pcurves; checker and Euler validation share one incidence
  implementation.
- Bounded tolerant edges may omit the 3D curve over a canonical logical domain;
  every fin then carries a covering pcurve; the checker compares lifted realizations
  within entity tolerance; tessellation shares one deterministic polyline.
- `FaceDomain` carries an optional finite conservative UV work box with authored,
  imported, inherited, and checked variants; Full checking adaptively certifies
  containment or returns explicit indeterminate gaps.
- X_T maps the conforming tolerant-edge representation
  (`TRIMMED_CURVE → SP_CURVE → 2D B_CURVE`); the import leg is host-certified
  (Onshape, 2026-07-11).

Remaining: production seam/pole/apex interchange fixtures; X_T round-trip for
non-identity tolerant-pcurve charts; re-export/compare host certification; upgrade
sampled incidence checks to whole-interval proofs.

### B. Geometry graph and procedural evaluation

`crates/kgraph` owns immutable analytic, NURBS, and procedural nodes with typed
dependencies, deterministic identity, bounded evaluation, and persistent verified
intersection-curve descriptors carrying proof certificates. Contract details:
[projects/geometry-graph.md](projects/geometry-graph.md). Remaining: descriptor
breadth (swept/spun/blend), broader chart classes, and recursive-procedural limits.

### C. Transactions and journals — landed contract

- Copy-on-write arena frames; scoped Store transactions with rollback-on-drop;
  deterministic mutation previews asserted equal to commit journals; checked commits
  validating declared roots and every affected body with complete ownership-closure
  audit; per-body incremental dependency footprints with debug full-reconstruction
  oracles; semantic lineage events; journal-returning checked constructors.
- Entity tolerances retain value, origin, growth, and modifying operation;
  transactions declare aggregate growth budgets; MEF inherits, KEF takes the ordered
  maximum; one failure-atomic facade batch applies operation-owned growth.

Remaining before the gate closes: route higher modeling/healing operations through
the landed checked-transaction consumers; tolerance combination/propagation policies
for further edit families; partition/rollback marks and committed undo/redo history;
persistent journal serialization/versioning; nested modeling-transaction composition.

## M3 — X_T interchange — IN PROGRESS

- **M3a0 (implemented slice):** modern-schema text/binary parser and checked
  reconstructor for the supported node subset; unsupported classes are explicit
  typed rejections, not silent skips.
- **M3a1 (parallel):** corpus observability — the committed fixture corpus with its
  non-shrinking stage manifest (`crates/kxt/tests/fixtures/manifest.tsv`) recording
  parse/reconstruct/check/tessellate outcomes and expected Full-checker gap
  baselines per fixture.
- **M3b (in progress):** Tier 1 authoring and external certification through the
  automated licensed-host loop ([oracle-loop.md](oracle-loop.md)). Writer-byte
  changes stale the certification record
  ([oracle-certification.json](oracle-certification.json)); CI enforces freshness
  bookkeeping. Remaining: 15-fixture re-certification, wire/acorn acceptance,
  re-export/compare closure, curved-fixture class preservation.
- **M3c (through M6/M8):** Tier 2/3 fidelity — procedural surfaces, periodic
  geometry, broader SP/intersection-curve chart classes, assemblies, and general
  bodies. Current landed slices cover bounded transmitted intersection charts over
  plane/offset/B-surface families; everything else remains explicitly unsupported.

**Exit (M3b):** 100% of the declared Tier 1 matrix imports into a licensed Parasolid
host with zero checker errors and survives there-and-back comparison on the
non-shrinking [oracle-results.tsv](oracle-results.tsv) record.

## M4 — Certified intersections + profile operations — PROVISIONAL / GATED

Analytic solvers are exact accelerators; fixed-grid NURBS marchers are experiments
that may discover contacts but can never label an empty result a proven miss. The
common result types enforce that distinction: analytic solvers explicitly construct
`Complete` results; provisional paths return verified discoveries with stable
`Indeterminate` reasons; `is_proven_empty()` is true only for an empty complete
result.

- **M4a — common contract and numerical core.** Landed: shared
  `Complete`/`Indeterminate` evidence across CC/CS/SSI; structured cell-budget and
  parameter-resolution limits; exact NURBS patch subdivision/BVH and analytic
  implicit-surface exclusion; deterministic recursive candidate covers with
  proof-bearing miss exits; bounded cell-local polishing with re-evaluated
  witnesses; exact-cell root/overlap certificates; source-provenanced classifiers
  where derived controls are numeric guidance only; coincident-region results
  (including the general seam-cancelling coincident-sphere merger) separated from
  isolated contacts. Target: the common SSI branch contract — 3D curve, pcurves on
  both surfaces, parameter correspondence, closure events, contact character, and a
  verified whole-interval residual bound. Narrow graph-aware Plane/Plane and
  Plane/Sphere branches have it; the common analytic result families do not yet.
- **M4b — curve/curve and curve/surface completion.** Generalize verified seeds and
  interval certificates to complete root discovery for NURBS pairs.
- **M4c — surface/surface.** Certified general SSI with paired pcurves and
  boolean-ready branch data.
- **M4d — first modeling consumers.** Landed: validated planar polygonal profiles
  with holes, checked oblique prism extrusion, deterministic rigid body copy with
  certificate reissue. Remaining: curved profiles, revolve, degenerate sweeps,
  external X_T validation of operation outputs.

**Exit gate:** the adversarial CC/CS/SSI battery includes tangencies,
near-coincidence, small loops, singularities, and NURBS-vs-NURBS; every `Complete`
result is independently verifiable; unresolved cases return `Indeterminate` or a
typed limit; extrude/revolve outputs are checker-v2 clean, journaled, watertight, and
externally X_T validated.

## M5 — Analytic booleans + interrogation — IN PROGRESS

Status: rungs 1–3 are implemented slices. `kernel::classify` certifies planar
point classification; `kernel::section` produces certified planar section
graphs; the public typed block/block Boolean facade applies all three CSG truth
tables and atomically Full-commits connected, proven-empty, multi-body, and
one-cavity results while exact contact and incomplete proof fail closed.
Deterministic operation outputs passed the supplemental Onshape oracle 6/6 on
import and 6/6 there-and-back (`docs/oracle-boolean-certification.json`). The
active queue head is rung 4 block/cylinder. Public finite-cylinder construction,
manifold finite-cylinder classification, topology-owned conic/ring trim proofs,
deterministic closed-fragment stitching, exact Plane/Cylinder rings,
bounded-arc endpoint topology, finite exact-family transverse ruling carriers
with paired whole-range residual proof, payload-agnostic exact boundary truth selection,
finite-cylinder Full proof, proof-bearing planar-circle/periodic-band partitions,
and exact dual-cell classification now feed the public intersection facade. It
atomically assembles and Full-checks strict axial slab-through-cylinder intersection
bands and the two positive remainder bands for axial finite-cylinder-minus-block
subtraction, copies every zero-cut truth-selected complete source boundary for
union/subtraction, creates zero-cut contained finite-cylinder cavities, and
assembles one-ring axial cap-overlap unions and blind pockets, two-port axial
through-holes, and two-ring two-sided unions as Full-certified cylindrical host
features. It proves zero-cut disjoint and support-separated axial exact-contact
intersections empty, assembles zero-cut finite-cylinder outers with negative
convex-planar cavity shells, and refuses unsupported curved truth/topology
classes before allocation. The results export deterministically and self-import
Fast-valid. Certified flush axial cap-contact unions also remove their full-disk
interface and Full-commit one connected boss. Topology-owned ruling trims,
mixed arc/ruling cycles, bounded-arc contacts/layouts, and the curved licensed-host
evidence gate remain.

Do not wait for an exhaustive analytic pair table before testing the end-to-end
boolean architecture. Begin with a deliberately narrow vertical slice as soon as the
required M4 cases are certified. **This is the active queue head; the rung
decomposition lives in ORCHESTRATION.md.**

- **M5a — vertical boolean slice.** Block/block and block/cylinder
  unite/subtract/intersect through the real pipeline: face-pair broad phase →
  intersection → imprint pcurves → split faces → classify fragments → assemble
  shells → tolerant stitch → full checker. Every phase inside one transaction with
  lineage events. Point-on-face and point-in-body classification precede fragment
  classification; classifier uncertainty propagates rather than becoming an
  arbitrary choice.
- **M5b — broad analytic booleans and interrogation.** The supported
  primitive/transform matrix; regularized and non-regular cases; voids, sheets,
  sheet-splits-solid; certified mass properties; point/ray classification, sections,
  minimum distance, clash. Differential tests against OCCT and Parasolid with every
  disagreement classified and retained.

**Exit gate:** ≥99.5% success on a versioned non-shrinking analytic corpus with a
published denominator and zero checker-failing "successes"; volume conservation and
boolean identities within certified bounds; atomic failure; journals and tolerance
budgets honored; successful results import into the Parasolid oracle with zero
checker errors.

## M6 — General booleans, sweeps, lofts, sewing, STEP — NOT STARTED

Periodic NURBS and procedural-surface intersections and booleans; sweep/loft with
compatibility operations; tolerant sewing with explicit gap budgets and healing
journals; STEP AP242 after sewing is independently robust; close the M3 Tier 2
geometry matrix.

**Exit:** imported NURBS boolean corpus satisfies the success-rate ratchet; sewing
closes the versioned torture corpus within tolerance budgets; STEP and X_T
round-trips preserve geometry class and topology.

## M7 — Blends, offsets, shelling, local operations — NOT STARTED

Constant-radius rolling-ball blends and chamfers with exact procedural
representation; variable radius, tangent chains, setbacks, corner patches; face/body
offset, hollow/shell, taper, tweak/replace, delete-and-heal; typed detection of
offset singularities, blend overrun, vanishing faces.

**Exit:** a versioned torture corpus covers converging edges, tangent chains,
overruns, corner interactions, thin regions, and shell self-intersections; procedural
blend/offset classes survive Parasolid round-trip rather than silently becoming
NURBS.

## M8 — API stabilization and production hardening — NOT STARTED

Frozen versioned native API, then a PK-style C ABI with opaque handles, typed error
codes, versioned option structs, Python bindings; finalized partition/session and
entity-id stability semantics; sustained fuzzing campaigns; profiling against the
specification's size/performance ladder; compatibility/version policy and migration
tests.

**Exit:** API semantics frozen; fuzzers and stress suites run clean for sustained
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
7. **Corpus:** Which adversarial, production, and minimized-regression cases were
   added?
8. **Oracle:** What independent implementation or mathematical property validates it?
9. **Determinism:** Are ordering, reductions, IDs, and output bits stable?
10. **Performance:** What benchmark prevents an accidental asymptotic regression?

Metrics are versioned artifacts, not prose claims. The committed X_T manifest records
both the Fast checker gate and the expected Full outcome/gap count for each fixture.
A checker-v2 change advances the roadmap only when it either discharges a ratcheted
gap with a conservative proof or adds a previously missing obligation explicitly;
deleting, weakening, or silently reclassifying an obligation is not progress. Any
intentional baseline change must update the capability ledger and the manifest in the
same change, and include an adversarial regression distinguishing `Invalid`,
`Indeterminate`, and `Valid`.

## Milestone dependency backlog

Not an execution queue — larger milestone obligations the queue must eventually
discharge:

- **M3b:** keep the historical 14-file host certification distinct from the declared
  15-file bundle; wire/acorn rejection, host-canonicalized analytic NURBS fixtures,
  offset-sheet re-export gap, and two preserved host re-export reader gaps remain.
- **M3c:** broaden verified transmitted-chart import beyond the landed
  plane/offset/B-surface slices: broader SP/foreign curves, null and mixed limits,
  multi-period trace aliases, nested/shared-basis/multi-offset forms.
- **M2.5:** parameter-space incidence completion; ratcheted Full-checker proofs for
  periodic/mixed boundaries, multi-loop containment, curved shells; tolerance
  policies beyond MEF/KEF.
- **M4:** adopt the common branch contract across analytic families; extend
  region/contact evidence to the remaining coincident and singular families;
  generalize exact-cell certificates to complete solver-integrated coverage;
  curved profiles, revolve, external X_T validation of operation outputs.
  (Done 2026-07-17: single-polar multi-occupied routed through the general
  merger; `kgeom::project` polisher twins merged.)
- **M5:** grow planar profiles and booleans only behind the checker, rollback,
  lineage, tolerance, determinism, corpus, performance, and independent-oracle
  gates.
- **Performance evidence:** production-scale imports exercise the Q2 construction
  and traversal ladders in `benches/` (`cases.json` is the registry); phase
  optimization, full-rebuild instrumentation, heterogeneous production edit
  footprints, and production assemblies remain explicit boundaries.

## After the kernel

1. **Constraint solver:** 2D sketch solver first, then 3D constraints.
2. **Parametric feature framework:** feature tree, persistent naming consuming
   kernel lineage journals, deterministic regeneration.
3. Application/UI layer.

## Standing risks

| Risk | Mitigation |
|---|---|
| Pair-specific intersection growth hides the absence of a complete generic solver | M4 common contract, certified subdivision fallback, and `Indeterminate` semantics precede further breadth. |
| Hand-enumerated case taxonomies masquerade as general algorithms | ORCHESTRATION R1: general algorithm or honest `Indeterminate`; solver file size caps; review rejection. |
| Boolean work hardens an insufficient B-rep | M2.5 pcurve, transaction, geometry-graph, and checker gates are mandatory. |
| Self-round-trip validates shared X_T bugs | Parasolid and OCCT independent-oracle gates; production corpus stage metrics. |
| Tolerant modeling is retrofitted too late | Per-incidence pcurves, tolerance provenance, operation budgets, and checker v2 land before booleans. |
| Corpus size grows without useful coverage | Feature manifests, versioned support matrix, stage-specific rates, non-shrinking regression sets. |
| Atomicity becomes full-store cloning | Arena checkpoints and mutation logs in M2.5; X_T uses the same transaction path. |
| Determinism prevents practical parallelism | Deterministic work ordering and reduction trees, verified at each real parallel consumer. |
| Performance targets deferred until redesign is expensive | Size ladders and benchmarks begin with X_T, BVHs, tessellation, and the first boolean slice. |
| Blend/offset complexity underestimated | Dedicated M7, procedural exactness, torture corpora. |
