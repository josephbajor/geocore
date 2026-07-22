# Kernel foundation project portfolio

Status: active implementation portfolio; convergence phase.

Bounded foundation projects F0–F7, each with the forward contract it established, a one-line status,
evidence pointers, and open items. All work preserves the kernel's determinism, failure atomicity,
completion evidence, and checked topology boundaries (ORCHESTRATION.md R6).

## Planning authority and handoff rule

`ORCHESTRATION.md` owns the queue head, the hard rules (R1–R7), and success metrics.
`docs/kernel-roadmap.md` owns milestone contracts and the ordered queue detail. **This file owns
per-project contracts and open items only**; the per-project design files (`geometry-graph.md`,
`operation-context.md`, `error-capability-taxonomy.md`, `kernel-facade.md`, `quality-harness.md`)
remain authoritative for their local contracts.

At handoff, update a project's Status/Open items only when its exit evidence actually changed. A
project file may refine its own next slice but may not reorder the queue — ORCHESTRATION.md and the
roadmap own ordering; if evidence changes priority, revise it there and link the reason here.

## Current direction and handoff order

The independent foundations have landed; the phase prioritizes convergence, adoption, and continuous
enforcement over new parallel surface area. Read the queue literally.

1. **Adopt and ratchet the completed contextual paths — CLOSED.** X_T reconstruction, checked-commit
   Fast validation, and whole-body tessellation share facade-owned scopes; both standalone projectors
   and the `ktopo::btess::tessellate_body` wrapper are closed to new production callers by the
   source-audit ratchet (state-4 deprecated, v1 behavior still pinned).
2. **Finish hostile-input tessellation policy — CLOSED.** Per-face and body-wide split/vertex/triangle
   admission, structural-holder items, atomic N/N+1 evidence, and the body/face representation gates
   landed. Compatibility-v1 preparation/edge/structural/body totals remain accounting-only at
   `u64::MAX` (no truthful finite legacy cap exists).
3. **Resume algorithm/API expansion behind the completed gates — OPEN (current head).** Block/block
   CSG is public and externally validated. The next consumer is block/cylinder Boolean support:
   public cylinder construction, finite-cylinder classification, topology-owned curved trim,
   closed-fragment stitching, exact rings and bounded arcs, residual-certified finite exact-family
   transverse Plane/Cylinder rulings, generic exact boundary selection, and
   finite-cylinder Full proof landed. Exact curved face partition/classification and failure-atomic,
   Full-checked axial intersection, cylinder-minus-block remainder bands, and zero-cut
   truth-selected whole-source union/subtraction copies and contained-cylinder cavities, one-ring
   axial cap-overlap connected union, one-ring axial block-minus-cylinder blind pockets, two-port
   axial through-holes, two-ring two-sided connected unions, and support-separated axial
   exact-contact empty intersections plus inverse-containment convex-planar cavities now run through
   the public facade and export deterministically; certified flush axial cap-contact connected
   unions also land; topology-owned ruling trims with operation-shared source-edge root identity
   now publish bounded line fragments and closed mixed arc/ruling cycles across shared translated, permuted, and all-nonzero oblique exact frames. Semantic rounded-frame Plane incidence and sloped-support ruling recovery, proof-bearing periodic Section embeddings, general disk/annulus arrangements and adapters, exact bounded analytic loop proof, exact source-root/carrier trim scalars, and correlation-preserving multi-chart clipped-cylinder Full proof landed. Rectangular/three-sided/five-support bounded-arc intersection and every ordered rectangular/three-sided planar-minus-cylinder component Full-commit across four frames. Endpoint-free cap planning/incidence/materialization plus count-independent portal-shell proof now let rectangular/five-support cap-retaining Unite and cylinder-left Subtract Full-commit with deterministic X_T, exact topology/volume, and N/N-1 evidence.
   Cycle-wide proof-local periodic lifting now certifies seam-crossing bounded loops. Exact planar-shell admission is independent of its optional typed convex certificate: complete non-convex ten-support star/cylinder Intersect Full-commits at 17F/45E/30V. Disk-cap chords, transverse annulus traces, and exact nested line-cycle planar cells lower through shared finite source arcs; count-independent chord-portal proof Full-certifies cap-crossing Unite/block-minus-cylinder at 9F/18E/12V and both 4F/6E/4V complement meanings across rigid frames/orders. Shifted endpoint-free chart authority plus one structural circular-attachment theorem now Full-certify both radial sides of same-signed nested-height parallel-cylinder shells: 6F/8E/4V Unite and outer-minus-inner Subtract join the existing lens/inner-crescent slices. Public Full-gated Plane/Cylinder interrogation certifies outward volume/centroid/area/inertia, whole-material distance, and same-scope clearance/clash verdicts with retained witness, refusal, deterministic, Boolean/cavity, and exact N/N-1 evidence; zero lower distance alone never implies interference. The historical Boolean identity is Onshape 15/15; the sixteen-payload bundle is stale pending one host replay. Same-signed strict-secant partial-overlap Intersect is next through a two-overlap-end relation.

**External-evidence lane.** Onshape records pin exact historical bytes: the 15-file base record is
stale; the fifteen-file Boolean bundle is current and passes 15/15 import and comparison. Fingerprints: `docs/oracle-certification.json` and
`docs/oracle-boolean-certification.json`; findings: `docs/oracle-results.tsv` (ORCHESTRATION R5).
CI checks those identities offline; manual catch-up batches own host evidence. Wire/acorn and base-reader gaps remain open.

## F0 — Completion-preserving result symmetry

Mission: prevent operand-order normalization from weakening proof evidence.

**Contract**
- Curve/curve and surface/surface result swapping preserves points, overlaps, ordering, orientation,
  and completion; all reversed-operand dispatch routes through it and restores canonical first-operand
  order. Complete hits and misses stay complete in either order; indeterminate reasons survive swapping.

**Status:** Implemented — swapping preserves completion evidence and canonical order.
**Evidence:** `kops` symmetry regressions (all `kops` tests pass).
**Open items:** none (independent corrective fix, complete).

## F1 — Procedural geometry graph

Mission: represent offset/intersection/swept/spun/blend geometry as exact dependent geometry without
duplicating owned basis or creating topology→geometry cycles. Design: `geometry-graph.md`.

**Contract**
- `kgraph` owns geometry handles, serializable descriptors, dependency traversal, cycle detection, class identity, and a fallible per-query `EvalContext`; `kgeom` keeps total, context-free leaf evaluators; the graph evaluator owns no session policy, executor, topology state, or diagnostics.
- Imported procedural geometry references its basis by handle (no owned-basis duplication), evaluates position/derivatives through a typed context, rejects cycles deterministically, stays exactly X_T-classifiable, and is consumable by a topology face without surface duplication.
- Reverse dependencies use deterministic insertion-ordered adjacency with direct key/membership lookup and no full-order rebuilds; traversal keeps vector-defined output/path order.
- Derived/rounded scalar surfaces are discovery-only; the original source owns proof and completion, and outward original-control differences own complete misses. Every effective sphere/cylinder radius in an offset chain must be proved finite and positive before use.
- Verified line/circle/NURBS-intersection descriptors persist atomically with ordered source/pcurve dependencies and their paired proof; altered/stale sources fail before allocation or roll the batch back. A trace retains at most four ordered offset descriptors (a fifth fails at graph insertion).
- Non-goals: general caching, concurrency optimization, every procedural class, a public plugin ABI.

**Status:** Implemented through G1–G4a, the F2 graph-budget adapter, G5a certified plane/plane-line and
plane/sphere-circle fields, the verified-NURBS descriptor families (Plane/NURBS, Sphere/NURBS, direct
NURBS/NURBS, 1–4 Offset(NURBS)/NURBS, dual Offset(NURBS), varying-normal quarter-cylinder), and M3c
transmitted-chart import (Plane/Plane, Plane/B-surface, offset chains, B/B, dual-offset families) via
corpus profile v15.
**Evidence:** `Q2a`/`Q2b` graph-index and traversal ladders; import-profile v1–v15 corpus fixtures
(per-arm certificate budgets: see git history).

**Open items**
- Broader G4 corpus; descriptor-chain depth ≥5; multi-descriptor planar-NURBS/Offset(Plane) peers;
  nested-dual and other varying-normal Offset(NURBS)/NURBS; broader NURBS/NURBS charts; further
  procedural G5 arms and carrier/descriptor families.
- Null, mixed/non-`H`, or broader closed limits; nullable chart data; ambiguous/multi-period trace
  aliases; noncanonical transmitted-chart variants outside the bounded affine slices;
  nested/shared-basis/multi-offset/out-of-range forms; arbitrary unclamped cyclic B-geometry.

## F2 — Operation context and numerical policy

Mission: stop model tolerances, conditioning thresholds, proof limits, and work caps from becoming
unrelated per-module policy. Design: `operation-context.md`.

**Contract**
- `OperationContext` owns immutable session/numerical/execution policy with family-default < session <
  request budget precedence; `OperationScope` owns the deterministic work ledger and ordered
  diagnostics. A scope deterministically reserves graph node-visit/depth work, then builds a graph
  evaluator with that `EvalLimits` reservation and a copy of the operation's `Tolerances`; the
  evaluator owns no session policy.
- Graph evaluation, Full checking, tessellation, projection, and exact curve/surface isolation account
  owned iterative work through one scope; limits are test-overridable; failures report stage plus
  consumed/allowed work; defaults reproduce existing golden results. Whole-body tessellation owns one
  scope across graph evaluation, projection fallback, refinement/storage, per-patch work, and output.
- Non-goals: making the Parasolid model-space regime arbitrarily configurable, nondeterministic
  cancellation, or tuning all algorithms in the first change.

**Status:** Implemented — Stage 1, Stage 1b composition, the bounded NURBS/NURBS Stage 3 scale gate,
two Stage 2 pilots, and contextual tessellation/projection/checker/generic-curve entries; body
production callers and the internal-use ratchet complete.
**Evidence:** `Q2` topology ladder (43 rows); `Q3 body-tessellation.v3` (32 rows) + face v2; `Q4`
curve-pair isolation v4 / implicit isolation v3 / solve v18; `bounded_v1` face/body presets.

**Open items**
- Projection caller adoption/ratcheting; other intersection-family incomplete-evidence and minimizer
  migrations; hostile-input tessellation allocation bounds; facade construction composition.
- Segment conditioning, input/dedup slack, broader migrations.

## F3 — Intersection engine consolidation

Mission: keep analytic special cases while preventing quadratic dispatch and helper duplication from
becoming the architecture.

**Contract**
- Adding a geometry class must not require hand-writing both operand orders; specialized and fallback paths return the same result contract; completion and structured limits survive dispatch transforms. One shared certified-fallback contract; centralized class inspection, pair normalization/swapping, shared range/periodic utilities, and shared candidate dedup/emission.
- Exact predicates (or certified intervals) own every topological decision; rounded/restricted/Bezier/split controls and derived effective surfaces are partition/sign-variation guidance only, never proof authority. Rounded signed-area is reporting-only; robust strict containment and streaming exact orientation own degeneracy, winding, and outer/hole roles.
- Shared M0 predicates (`harmonic_half_angle_roots`, `affine_dot3`, `quadratic_discriminant`, `squared_distance_difference3`, `polygon_orientation2d`/`_iter`, `orient2d`/`orient3d`, `incircle`) classify from the original identity; unrepresentable cases return `Indeterminate` (or typed `HarmonicRootClassification`/`SingularSphereChart`), never a flattened miss.
- Candidate cells retain shared original-source provenance; exclusion/overlap bounds come from outward interval evaluation of the original source, failed open to the whole-source hull; only exact parameter contact merges overlaps.
- Region emitters return general canonical paired regions with chart orientation and outward whole-region residuals; collapsed/ambiguous/non-exact layouts stay `Indeterminate` rather than enumerating case taxonomies (ORCHESTRATION R1).
- Non-goals: rewriting correct closed-form solvers; completing every NURBS case in one project.

**Status:** Implemented — centralized class inspection, shared fitting/normalization/dispatch, NURBS
curve-pair isolation with bounded verified polishing, shared conic/SSI emitters, coincident-window
region arms (Plane/Plane, Cylinder/Cylinder, Sphere/Sphere octants + bounded general sphere windows,
Cone/Cone, Torus/Torus), exact algebraic carriers/residuals through magnitude-twelve (opt-in 13/14),
the line/Torus quartic slice, and operation-local closed Plane/Cylinder circle branches with paired
whole-period pcurve evidence. Kernel Section now consumes those branches through topology-owned
polygon/ring clipping and emits exact endpoint-free rings for the strict whole-period class.
**Evidence:** SSI/conic bit-signature regressions; sphere-window arm exemplars (piece/pair/arc N/N-1
ceilings); magnitude-12→14 enumeration goldens; `kops` `Indeterminate` / `kgraph` typed-error
propagation tests (per-arm fixtures: see git history).

**Open items**
- Broader NURBS/NURBS and offset/varying-normal fields; further verified carrier/descriptor families; coefficient forms beyond magnitude-fourteen. Remaining sphere-window layouts (other seven/eight/nine-positive two-wide, disconnected-without-exact-separation, other polar, non-exact tangent/collapsed) stay Indeterminate.
- Pcurve classes beyond bounded Line2d/Circle2d and curved/periodic containment; general quartic/higher-degree isolation; other higher conic/primitive containment and window-partition families; full source-exact harmonic discriminant;
  affine-dot fallbacks outside the reviewed envelope; complete NURBS/plane root isolation; contextual replacement of the static depth-72 / 65,536-node caps; proof-bearing finite-window boundary cells.
- Repository-wide topological-decision audit (bounded migrations landed: SSI convexity, oblique-extrusion `orient3d`, coincident Plane/Plane hull); `incircle` has conformance evidence but no production consumer, `insphere` deferred until a 3D Delaunay/equivalent consumer exists.
- Raw extreme-scale `Vec3`/`Vec2` `norm`/`norm_sq`/dot/cross/distance/subtraction overflow.

## F4 — Kernel error and capability taxonomy

Mission: let callers/metrics distinguish invalid input, unsupported valid input, incomplete proof,
exhausted resources, and violated invariants without parsing diagnostic strings. Design:
`error-capability-taxonomy.md`.

**Contract**
- Unsupported geometry is not `InvalidGeometry`; limit telemetry is machine-readable; X_T wrapping
  retains kernel classifications; a C-ABI mapping can be defined without inspecting strings.
- Each error/outcome layer delegates class, code, limit, capability, and the exact `source()`,
  retaining human-readable context; graph certificate variants own their class/code/capability.
- Non-goal: a repository-wide cleanup campaign — legacy call sites migrate opportunistically with their
  owning behavior change.

**Status:** Implemented — Phase 1, representative Phase 2 slices, three Phase 3 pilots, and two
source-identity migrations (ellipse/ellipse `ProjectionError` chain; graph intersection-certificate
inventory; typed rigid-copy `ktopo::BodyCopyError` → facade `BodyCopyError` → `KernelError::BodyCopy`).
**Evidence:** ellipse/ellipse projection-error retention tests; graph certificate
class/code/capability tests; rigid-copy recertification typed-chain tests (see git history).

**Open items**
- Legacy compatibility/transaction wrappers and other solver-local `InvalidGeometry` collapses;
  broader result-family migrations.

## F5 — Kernel facade and topology encapsulation

Mission: give application/bindings/feature-history clients a stable conceptual API without exposing
arena layout or backlink vectors. Design: `kernel-facade.md`.

**Contract**
- Ordinary clients construct, query, mutate transactionally, and export a body without importing raw
  entity structs; `kxt` still reconstructs atomically; compile-fail tests protect raw mutation
  boundaries. Entity views (`FinView`, …) expose facade-owned values without leaking lower types.
- The `kernel` facade owns lifecycle, opaque IDs, classified sources, one-scope outcomes, safe checker
  subjects, child-accounted procedural evaluation, atomic typed X_T import/export, graph-owned bounded
  intersection with facade-owned proof results, immutable conforming body meshes with facade-safe
  face/edge identities, failure-atomic checked construction, and deterministic complete-body rigid copy.
- Semantic edits validate part-qualified live geometry and all shape/chart/incidence preconditions
  before checked mutation, return opaque results plus the committed facade journal, and restore
  topology + future identities on rollback or proof denial. Position-owning scaffolds stay transient
  until Euler composition completes them; facade inverses delete hidden points only when unshared. The
  additive Full-assurance gate keeps Fast commit unchanged (`RequireValid` rejects any gap,
  `AllowIndeterminate` retains gaps, Full faults always reject, rejected decisions carry no journal).
- Rigid copy reissues every operation-generated certificate family with leaf-inclusive proof depth ≤64,
  copying ordered roots and full transitive basis chains and rerunning the whole-range certifier;
  graph-valid shared-basis/periodic charts, nested-deep roots, altered/overdeep bindings fail closed.

**Status:** Implemented — K1–K3, typed K4 interchange + journal views, checked semantic K4 edits
(MVFS/KVFS, MEV/KEV, MEF/KEF, KFMRH/MFKRH, strut/face-split/bridge-removal/ring-join/face-as-hole
composition), tolerance-batch growth, the Full-assurance commit gate, K5 adoption, facade body
tessellation, block and polygonal-profile extrusion, and rigid copy with certificate reissuance.
**Evidence:** standalone `kernel-lifecycle` client (sole direct dep `kernel`; exercises construction,
semantic inspection, Full checking, edit commit/rollback, tessellation, curve intersection, X_T
round-trip, tolerance-batch journaling, journal traversal); compile-fail raw-mutation-boundary tests;
CI-enforced `kernel` package inventory.

**Open items**
- Attributes (blocked on an authorable storage contract); non-rigid transforms; broader semantic-edit
  families and partition-history composition.

## F6 — Shared surface services and responsibility splits

Mission: remove semantic drift before splitting large modules for size alone.

**Contract**
- Checker and tessellator share one analytic surface inversion/projection, periodic base-chart
  normalization, and point-to-surface distance implementation in `kgeom` with the same class coverage;
  focused tests cover seams, singularities, and NURBS projection.
- Later structural/incidence/domain/shell checking, boundary/chart/triangulation tessellation, and X_T
  planning/emission/serialization splits land only when a contextual or adoption seam is tested and the
  move is behavior-preserving. File size alone is not a split criterion (subject to ORCHESTRATION R2);
  F6 splits and F4 legacy cleanup land only with an owner-level behavioral migration.

**Status:** First slice implemented — shared surface inversion, chart normalization, and distance
services consumed by checker and tessellation.
**Evidence:** checker/tessellation shared-inversion focused tests (see git history).

**Open items**
- Module/responsibility splits (structural/incidence/domain/shell checking; boundary/chart/
  triangulation tessellation; X_T planning/emission/serialization).
- Broader pcurve signed-integral classes and curved/periodic containment checker proofs.

## F7 — Quality, fuzzing, and performance harnesses

Mission: make robustness and asymptotic expectations executable before broad modeling operations land.
Design: `quality-harness.md`, [`test-throughput.md`](test-throughput.md).

**Contract**
- Benchmarks have named fixtures and recorded baselines; fuzz targets run locally and in bounded CI
  smoke jobs; toolchain/MSRV changes are explicit; no benchmark depends on wall-clock ordering for
  correctness. CI runs offline Python/oracle-record checks, compiles and smoke-runs the excluded
  benchmark/fuzz/tooling package (protective only when its contracts run in CI), and runs the pinned
  fuzz targets within fixed limits; it never performs licensed-host validation.
- Any graph-index/traversal representation change requires a recorded stable-host before/after
  comparison; counters measure invocation boundaries/cardinalities, not elapsed work. New integration
  test targets state a wall-time budget (ORCHESTRATION R7).

**Status:** Implemented — Q0–Q2b, Q8, and the first Q3–Q6 slices; developer lanes
(`focused`/`fast`/`standard`/`docs`/`full`) and 14-target production-corpus classification landed.
**Evidence:** `Q2` (43 rows), `Q2a` v2 (21 rows), `Q2b` v2 (10 rows), `Q3 body-tessellation.v3` (32
rows) + face v2 (18 rows), `Q4` implicit isolation v3 (8), curve-pair isolation v4 (9), solve v18 (28);
benchmark manifest (178 cases); two pinned fuzz targets; [`test-throughput.md`](test-throughput.md).

**Open items**
- Phase optimization, full-rebuild phase instrumentation, broader heterogeneous production edit
  footprints, production assembly.
- Q3–Q5 expansion, exact coefficient forms beyond twelve, more Q6 targets/corpora, Q7.

## Reconciled F1/F2/F4 boundary (integration gate)

One normative ownership model (ownership triple in the F1/F2 contracts); a second session/context
abstraction requires an explicit portfolio revision. The graph evaluator owns no session policy,
executor, cancellation, topology state, or diagnostics; the operation context owns no graph handles,
caches, cycle stack, or descriptor knowledge. F1/F2 may introduce typed local evaluation/limit data;
F4 standardizes stable capability/stage/error identifiers without erasing those distinctions or
introducing graph types into `kcore`.

## Dependency outline

```text
F0 Completion-preserving result symmetry        (independent corrective fix)
F1 Procedural geometry graph                    (blocks procedural geometry)
F2 Operation context and numerical policy       (blocks generic solver growth)
F3 Intersection engine consolidation            (after F2; uses F1 types later)
F4 Kernel error and capability taxonomy         (independent; coordinate w/ F2/F3)
F5 Kernel facade and topology encapsulation     (after F1, F2, F4 contracts)
F6 Shared surface services/module decomposition (independent first slice)
F7 Quality and performance harnesses            (independent and continuous)
```

## Standing ratchets

- Writer-reachable byte changes invalidate the affected licensed-host evidence; a local read/write round-trip does not restore it.
- A proven contextual replacement closes the door to new crate-internal legacy calls even while source-compatible public wrappers remain.
- Excluded benchmark, fuzz, and Python tooling is protective only when its contracts run in CI.
- The facade-only `kernel-lifecycle` client keeps exactly `kernel` as its direct dependency; the reviewed `kernel` inventory stays CI-enforced.
- Large-import work exercises the graph-construction ladder; representation optimization includes a stable-host before/after measurement and preserves deterministic ordering.

## Integration rules

Each project states which capabilities change, result completeness, applicable tolerances/work budgets,
how failure atomicity is verified, journal/checker evidence produced, and the regression that protects
it. Cross-project shared types land in small contract commits before broad migrations; new production
code uses the F2/F4 contracts; F4 runs no repository-wide cleanup campaign.
