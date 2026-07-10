# Kernel Construction Roadmap

Companion to [kernel-spec.md](kernel-spec.md). The specification defines the target
contract; this document defines implementation order, current evidence, and the gates
that prevent a locally successful prototype from being mistaken for a conformant CAD
kernel.

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
| M1 Geometry | IMPLEMENTED SLICE | Analytic geometry, clamped NURBS basics, projection, and tessellation exist; periodic/procedural and several full NURBS capabilities remain. |
| M2 Topology | IMPLEMENTED SLICE | Core hierarchy, Euler operators, primitives, checker v1, watertight body tessellation, and the transaction/journal foundation exist; boolean-ready incidence and operation-wide checked mutation do not. |
| M2.5 Architecture gate | IN PROGRESS / REQUIRED | Per-fin pcurves, bounded curve-less tolerant edges, shared incidence validation, pcurve-aware Euler creation, pcurve-driven tessellation, copy-on-write transactions, deterministic raw/semantic journals, and explicit face-domain/tolerance metadata have landed; geometry graph, operation migration, mutation encapsulation, certified imported trim domains, and checker upgrades remain. |
| M3 X_T | IN PROGRESS | The modern-schema subset reads both wire encodings and writes text, including bounded tolerant edges as trimmed SP-curves over finite 2D B-curves; production coverage and external certification remain. |
| M4 Intersections/profile ops | PROVISIONAL / GATED | Broad analytic special cases and sampled NURBS experiments exist; certified generic discovery and boolean-ready branches do not. |
| M5–M8 | NOT STARTED | No end-to-end booleans, general modeling, blends, stable API, or production hardening. |

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
curve knot insertion/refinement/splitting/Bezier extraction; surface knot insertion;
global curve interpolation; multi-start projection; deterministic trimmed-face
tessellation; and explicit `AlgorithmLimit` failures when refinement cannot meet its
request.

### Debt and delivery point

- **Before M4 certified general intersections:** Bezier patch extraction/subdivision for
  NURBS surfaces, tight subrange boxes, evaluator conditioning/singularity information,
  and projection APIs that distinguish converged, indeterminate, and failed searches.
- **Before M3 production Tier 2 / M6:** periodic NURBS curves and surfaces, collapsed
  patch detection, surface splitting, degree elevation, knot removal, approximation and
  fitting with verified error, and derivative/iso-curve construction.
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
cone-frustum/sphere/torus constructors, checker v1, and edge-once whole-body
tessellation. Primitive meshes are checker-clean, watertight, outward-oriented, and
volume-tested.

### Known limits

- Fins retain independent, explicitly parameter-mapped pcurves; authored analytic
  primitives and pcurve-aware Euler variants propagate them. Bounded curve-less
  tolerant edges use a canonical logical domain and require every fin pcurve; the
  checker compares lifted realizations and endpoints within entity tolerance, and body
  tessellation shares one deterministic 3D polyline across all uses. Legacy Euler entry
  points and generic mutation still permit missing pcurves, so incidence is not yet
  boolean-ready by construction.
- `General` mixed-dimension bodies, face tolerances/domains, curve-less ring edges,
  isolated loops, and several pole/apex or degenerate topologies are unsupported.
- Entity fields and generic mutable store access allow callers to bypass Euler
  invariants. “Euler operators only” is a convention rather than an enforced boundary.
- Scoped Store transactions now provide rollback-on-drop and deterministic raw mutation
  plus semantic lineage journals. Only checked face split/merge consumers and X_T
  reconstruction use the foundation so far; most public constructors/Euler callers can
  still mutate outside a transaction, and there is no partition history, attribute
  propagation mechanism, or incremental invalidation record.
- Checker v1 samples incidence, supports loop orientation only on a subset of surfaces,
  and does not yet prove loop self-intersection/containment, face containment, shell
  self-intersection, or full body orientation.

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
  2D B-curves and reversed trim direction round-trip locally.

Remaining before the gate closes:

- Migrate higher operations to the pcurve-aware Euler variants and make pcurves mandatory
  for face-edge uses created through the future checked topology API; the legacy Euler
  entry points still intentionally create `None` during migration.
- Add an independently sourced, redistributable Parasolid fixture containing a true
  tolerant edge and certify the emitted SP-curve chain in a licensed Parasolid host.
  Current evidence is standards-derived and self-round-trip only. Extend interchange to
  periodic/circular pcurves and any geometric-owner variants observed in that corpus.
- Add explicit seam-branch metadata where a periodic pcurve range alone is insufficient,
  plus pole/apex-degenerate pcurve fixtures.
- `FaceDomain` now carries an optional finite conservative UV work box and faces carry
  optional tolerance metadata. Analytic primitives author exact boxes; finite natural
  surface ranges initialize imported faces; Euler splits inherit them, merges union them
  only on the same surface (otherwise mark them unknown); the checker validates range/
  period/full-closed-face invariants; and tessellation uses them to anchor periodic
  branches. Still needed: certified tight trim-domain construction for imported plane/
  cylinder/cone faces, boundary-containment proof, seam-branch metadata beyond a box,
  and tolerance provenance/budgets.
- Upgrade sampled local incidence checking to adaptive verification and add explicit
  tolerance provenance/budget tracking. Endpoint-to-vertex checks now exist, but they
  are still sampling-based rather than a proof over the full interval.

### B. Geometry graph and procedural evaluation

- Keep `kgeom` as pure mathematics, but introduce a geometry graph/evaluation context
  capable of resolving curve/surface handles.
- Define descriptors for intersection and SP curves first; reserve stable extension
  points for swept, spun, offset, and blend surfaces.
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
- Journals carry semantic `split`, `merge`, `derived_from`, and `replaced` events in
  addition to raw storage mutations. Checked pcurve-aware face split/merge wrappers emit
  tested deterministic lineage.
- X_T reconstruction uses the same transaction path and exposes its mutation journal;
  the previous full-session staging clone has been removed.

Remaining before the gate closes:

- Route all higher modeling operations and import/healing paths through checked
  transaction consumers; decide and test journal composition before enabling nested
  modeling transactions.
- Add partition/rollback marks and a committed undo/redo history above scoped operation
  transactions without weakening handle identity guarantees.
- Add attribute propagation and tolerance-budget entries to semantic journals, plus
  persistent serialization/versioning once the feature layer consumes them.

### D. Enforced topology API

- Make entity mutation private to topology internals and expose checked read views.
- Higher operations compose Euler/topology primitives inside transactions.
- Add explicit builders for sheet, wire, acorn, and general bodies rather than requiring
  callers to assemble public vectors and back-pointers manually.

### E. Tolerance, errors, and checker v2

- Define tolerance combination/propagation rules and a tolerance-growth budget for each
  operation. Preserve provenance of imported entity tolerances.
- Introduce capability and completion errors rather than treating unsupported or
  indeterminate geometry as invalid input.
- Checker v2 adds pcurve/3D incidence, loop self-intersection and containment, face
  containment, shell closure/orientation, adaptive incidence verification, tolerance
  validation, and `Fast` versus `Full` levels.

### Exit gate

- A seam-crossing face and a pole/apex-adjacent face round-trip with explicit pcurves and
  pass checker v2 without reconstructing UVs from 3D samples.
- A deliberately failing multi-step topology operation restores bit-identical entities,
  handle validity, and next-allocation behavior. **Landed for scoped Store transactions.**
- A successful split/merge scenario emits deterministic lineage events. **Landed for the
  checked pcurve-aware transaction wrappers.**
- External code cannot mutate topology without checked topology APIs.
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
- The published schema-13006 FACE tolerance field is required to be null. The writer
  therefore rejects a non-null kernel face tolerance with stable capability
  `xt.write.face-tolerances` rather than emitting a nonconforming file; face UV domains
  remain kernel-side because XT represents face bounds through loops, not a UV box field.
- Complete a field-by-field ownership/link audit for every declared writer cell,
  including directly shared untrimmed curves and shared point ownership. The tolerant
  SP path now emits its boundary-curve chain and geometric-owner rings, but acceptance
  by this repository's permissive reader is not evidence that all older writer paths
  satisfy Parasolid's relationship invariants.
- Add neutral-binary output after the text semantics are independently certified.
- For every authored capability: import into Solid Edge/another licensed Parasolid host,
  run its checker, re-export, re-import here, and compare topology, geometry class,
  tolerance, orientation, and mass properties.
- Use OCCT as a second, independent differential oracle where its semantics apply.

**Exit:** 100% of the declared Tier 1 authoring matrix imports into the Parasolid oracle
with zero checker errors and survives there-and-back comparison. Self-round-trip alone
does not satisfy this gate.

### M3c — Tier 2/3 fidelity — THROUGH M6/M8

Periodic B-geometry, circular/periodic pcurve encodings, intersection curves, procedural
surfaces, curve-less tolerant ring/degenerate cases, general bodies, attributes,
transforms, instances, assemblies, and older schemas land with their kernel
dependencies. Full “any well-formed X_T” Tier 0 is not claimed until this matrix is
closed.

## M4 — Certified intersections + profile operations — PROVISIONAL / GATED

Existing analytic solvers remain valuable exact accelerators. Existing fixed-grid NURBS
curve and surface marchers are experiments: they may discover contacts, but they cannot
label an empty result a proven miss or label an interpolated polyline an exact
intersection curve.

### M4a — Common intersection contract and numerical core

- Replace implicit success/empty semantics with `Complete`, `Indeterminate`, and typed
  failure/limit outcomes. Empty means miss only when exclusion evidence covers the full
  requested domain.
- An SSI branch carries a 3D curve, pcurve on surface A, pcurve on surface B, parameter
  correspondence, closure/end events, contact character, and a verified residual/error
  bound over the entire active interval—not only endpoint UVs.
- Represent coincident curve intervals and coincident surface regions separately from
  isolated contacts and ordinary branches.
- Add NURBS-to-Bezier subdivision, convex-hull/AABB and interval exclusion, deterministic
  BVHs, candidate isolation, safeguarded Newton polishing, and conditioning diagnostics.
- Analytic special cases and the generic solver feed the same canonical result type.

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

### M4d — First modeling consumers: extrude and revolve

- Construct planar wire profiles with holes and explicit pcurves.
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
  minimum distance; and clash detection.
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

- Constant-radius rolling-ball edge blends with exact procedural representation.
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

## Immediate implementation queue

1. Finish the landed pcurve/coedge slice: migrate operation callers to pcurve-aware Euler,
   complete certified imported trim-domain/tolerance provenance, expand seam/pole/apex
   fixtures, and externally certify the bounded tolerant-edge X_T SP-curve subset.
2. Migrate every higher operation to the landed transaction/journal foundation; add
   partition history and journal composition semantics.
3. Encapsulate topology mutation and introduce checker v2 foundations.
4. Redesign intersection results around completion evidence and paired pcurves.
5. Expand the landed X_T manifest/stage-rate harness to licensed production-scale,
   tolerant, NURBS, transformed, multi-body, and assembly corpora; complete external M3b
   validation in parallel.
6. Replace fixed-grid “general” intersections with Bezier subdivision and certified
   exclusion, keeping analytic cases as accelerators.
7. Ship extrude/revolve, then the narrow M5a boolean vertical slice before expanding the
   pair-specific solver catalogue further.

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
