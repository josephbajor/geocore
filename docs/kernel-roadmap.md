# Kernel Construction Roadmap

Companion to [kernel-spec.md](kernel-spec.md). Milestones are sequential where they
share a dependency spine (M0→M2→M4→M5) but M3 (XT) deliberately starts early and runs
in parallel, because the XT corpus is the test infrastructure for everything after it.

Effort calibration, stated honestly: Parasolid is ~35 years and millions of lines.
A small team reaches *useful* (Tier 1 interop + analytic booleans) in roughly a year of
focused work, and *credible* (NURBS booleans + blends, Tier 2) in two to three. The
roadmap is scoped so every milestone ships something independently testable and demoable.

---

## M0 — Foundations (≈ 4–6 weeks) — ✅ COMPLETE
Repo, CI, license. L0 in full: scalar policy, robust predicates (port/verify Shewchuk),
interval arithmetic, tolerance policy module, typed error model, arena/handle storage,
deterministic parallel primitives.

**Exit:** predicate test suite passes including adversarial near-degenerate cases;
determinism harness proves bit-identical results across platforms/thread counts.

**As built (`crates/kcore`):** all of the above, plus one scope addition forced by
evidence — `kcore::math`, a musl/fdlibm port of sin/cos/sincos/atan/atan2. The
golden-hash harness caught platform libm producing different bits in debug vs
release builds (const-folding vs runtime libcall); kernel code now uses only
kernel-owned transcendentals, enforced by clippy `disallowed-methods`, making
results bit-identical across platforms and build modes by construction.

## M1 — Geometry core (≈ 8–10 weeks) — ✅ COMPLETE
Analytic curves/surfaces with full evaluator protocol. NURBS engine (Piegl & Tiller
algorithm set) with exhaustive unit tests against known-good values. Closest-point
projection. Bounding volumes. Single-face tessellation.

**Exit:** every geometry class passes evaluator conformance tests (derivative checks
vs finite differences, periodicity, degeneracy reporting); NURBS ops verified against
published worked examples; a trimmed NURBS face tessellates crack-free.

**As built (`crates/kgeom`):** line/circle/ellipse curves; plane/cylinder/cone/
sphere/torus surfaces with *exact* patch bounding boxes (surface-of-revolution
extremes analysis — stronger than the planned conservative boxes); NURBS engine
(basis + derivatives, rational curve/surface evaluation to order 3/2, knot
insertion/refinement, splitting, Bezier extraction, global interpolation; exact
rational-Bezier circles verified to 1e-12); closest-point projection (multi-start
damped Newton); trimmed-face tessellation (boundary chordal refinement, hole
bridging + ear clipping over robust predicates, conforming interior edge-split
refinement, watertight-with-boundary invariant tested). Tessellation returns a
typed `AlgorithmLimit` error rather than an under-refined mesh when boundary
depth, interior pass, or triangle-count limits prevent satisfying the request.
Deferred, first needed in M6: knot removal and degree elevation; periodic NURBS
remain part of M3c/Tier-2 fidelity, and degenerate-patch reporting remains checker
work. Triangle quality optimization is also deferred. Determinism goldens are
pinned for both `kcore` and `kgeom`.

## M2 — Topology + primitives (≈ 6–8 weeks) — ✅ COMPLETE
Full Parasolid entity hierarchy, Euler operators, body types, tolerant entity plumbing.
The checker, v1. Primitive body constructors. Whole-body watertight tessellation
(cross-face crack elimination at shared edges).

**Exit:** primitives construct checker-clean; Euler invariant property tests pass;
a primitive body renders watertight in a throwaway viewer.

**As built (`crates/ktopo`):** entity hierarchy over generational arenas with a
uniform typed store; ring-edge and zero-loop-face conventions fixed in the data
model (revolved primitives carry no artificial seam edges, matching Parasolid);
the full ten-operator Euler set with a randomized 300-step Euler–Poincaré
property harness (which caught a real `mef` fin-leak during development);
primitives block/cylinder/cone-frustum/sphere/torus, all checker-clean; checker
v1 with 24 fault kinds — structural back-pointer/ring/pairing/kind checks, a
per-shell Euler identity that counts ring edges and zero-loop faces correctly,
and geometric checks (vertex-on-curve, edge-on-surface by exact analytic
distances, size box, loop orientation); whole-body tessellation with edge-once
discretization, seam-cutting for periodic faces, pole collapse for spheres, and
index-mapped assembly — watertightness (every mesh edge in exactly two opposed
triangles) and enclosed-volume accuracy verified per primitive, plus OBJ export
in lieu of a viewer. Deferred: full cones with apex vertices, tolerant
curve-less edges (M3c), Euler identity for shells with unclassifiable faces.
Failure-atomicity hardening has begun: `kev` preflights its known late-failure
path before mutation. Complete transactional modeling operations and journaling
remain M5/M8 work.

## M3 — XT interchange (starts with M2, runs long) — IN PROGRESS
- **M3a Read (Tier 0):** text + neutral-binary parser, schema versioning, topology and
  Tier-0 geometry reconstruction, tessellate-and-view arbitrary real-world XT files.
  Begin harvesting the corpus (GrabCAD, vendor samples, Solid Edge CE exports).

  **Modern-schema subset implemented (`crates/kxt`), as built:** both wire
  encodings behind one cursor abstraction; the embedded-schema mechanism decodes
  C/D/I/A/Z edit scripts against base schema 13006 without external schema files.
  Reconstruction maps the supported topology and geometry subset into
  checker-clean `ktopo` bodies, including sense encodings, ring edges, and
  void-exterior-shell dropping; the corpus includes a hand-authored block in both
  encodings + real-world V27/V28 parts, provenance in
  `crates/kxt/tests/fixtures/README.md`); end-to-end tests import real files
  and tessellate them watertight. Reconstruction is atomic with respect to the
  caller's `Store`: failure leaves existing entities, handles, and subsequent
  handle allocation unchanged. This does not yet satisfy the broader Tier-0
  "any well-formed XT" criterion because pre-13006 schemas, procedural
  geometry, SP-curves, tolerant edges, and assemblies remain unsupported.
  Deferred to M3b/M3c: broader writing and procedural
  geometry, SP-curves, tolerant edges, assemblies, pre-13006 schemas;
  B-surface pole ordering is flagged provisional until the M3b round-trip.
- **M3b Write (Tier 1):** emit XT for self-authored analytic bodies; round-trip
  ourselves; validate empirically in Solid Edge CE (import, checker, re-export, diff).

  **IN PROGRESS:** deterministic base-schema-13006 text output now covers every
  self-authored analytic primitive (block, cylinder, expanding/shrinking cone
  frustum, sphere, torus). Each case round-trips through our reader, checker, and
  watertight tessellator. Neutral-binary output and the required Solid Edge CE
  checker/re-export validation remain before M3b can be marked complete.
- **M3c Tier 2/3 fidelity:** procedural surfaces, SP-curves, intersection curves,
  attributes, assemblies — extends through M6.

**Exit (M3b):** 100% of self-authored analytic bodies import into Solid Edge with zero
errors and survive a there-and-back round trip.

## M4 — Intersections (≈ 10–14 weeks, hardest math) — STARTED
Curve/curve, curve/surface, then SSI: quadric/quadric closed forms, marching with
adaptive stepping, subdivision start-point discovery, tangent/singular handling,
small-loop detection. Extrude/revolve land here too (they need edge geometry but not
booleans) — first "real modeling" demo.

**STARTED (`crates/kops`):** parameter-rich curve/curve result contracts distinguish
proven misses, isolated contacts, and coincident intervals. Bounded line/line handles
transverse contacts, endpoint touches, parallel misses, and same/reversed overlaps.
Bounded 3D line/circle and line/ellipse handle coplanar secants and tangencies,
transverse plane crossings, periodic arc filtering, and tolerance-aware near
contacts. Bounded 3D circle/circle, circle/ellipse, and ellipse/ellipse handle
coplanar secants/tangencies, skew-plane contacts, periodic arc filtering,
tolerance-aware near tangencies, and coincident overlaps where applicable.
The public curve/curve dispatcher now routes every line/circle/ellipse ordered
pair to those exact solvers and rejects unsupported NURBS/procedural curves
explicitly. Curve/surface has begun with bounded line/plane, line/cylinder,
line/cone, line/sphere, line/torus, circle/plane, ellipse/plane, and
circle/sphere over finite surface windows, including contained line-on-plane,
conic-on-plane, circle-on-sphere, and line-on-ruling intervals, cone apex
singular contacts, and torus quartic contacts. General NURBS/procedural
curve/curve, broader curve/surface, and SSI remain.

**Exit:** SSI test battery including tangent cylinders, near-tangent tori, and
NURBS-vs-quadric cases; every intersection curve usable as edge geometry and
checker-clean when imprinted.

## M5 — Booleans v1 + interrogation (≈ 10–12 weeks)
Analytic-geometry booleans end-to-end (intersect → imprint → split → classify →
assemble → stitch), atomic failure, journaling. Point classification, mass properties,
distance/clash. Differential-test harness vs OCCT and Solid Edge goes live with the
boolean success-rate ratchet.

**Exit:** ≥ 99.5% success on the analytic boolean corpus; volume-conservation property
tests pass; results export to SolidWorks/Solid Edge cleanly (Tier 1 sustained).

## M6 — General booleans, sweeps, sewing (≈ 12–16 weeks)
Booleans over NURBS and procedural surfaces. Sweep along curve, loft. Tolerant sewing
(unlocks STEP import). STEP AP242 read as the second format.

**Exit:** boolean corpus extended with imported real-world NURBS models; sewing closes
the standard STEP torture-test set; success-rate ratchet holds.

## M7 — Blends, offsets, shelling (≈ 12–16 weeks)
Constant-radius rolling-ball edge blends (exact `blended_edge` surfaces for XT
fidelity), then variable radius and corner patches. Face/body offset, hollow/shell,
tweak, delete-and-heal.

**Exit:** blend torture tests (converging edges, blend-overruns-face, tangent chains);
blended bodies round-trip to SolidWorks as procedural blends, not NURBS approximations.

## M8 — Hardening + API freeze (≈ 8 weeks, overlaps M7)
PK-style C API stabilization and versioning policy, Python bindings, fuzzing campaign,
performance pass against §4 targets, docs. Partition/rollback session model finalized
for the parametric layer.

**Exit:** public API frozen at 1.0 semantics; fuzzers run clean for sustained CPU-days;
perf targets met.

---

## After the kernel (sequenced, not scheduled)
1. **Constraint solver** (D-Cubed DCM analog): 2D sketch solver first (points/lines/
   arcs; coincident/parallel/tangent/dimension; graph decomposition + Newton), then 3D.
2. **Parametric feature framework**: feature tree, persistent naming built on the
   kernel's entity journaling, regeneration engine.
3. Application layer.

## Standing risks

| Risk | Mitigation |
|---|---|
| SSI/boolean robustness swallows the schedule | Analytic-first staging; ratcheted corpus metric makes regressions visible immediately; tolerant modeling from day one, not retrofitted. |
| XT semantic gaps the spec document doesn't capture | Empirical loop against Solid Edge CE from M3a onward; every discrepancy becomes a corpus case. |
| Blend surfaces underestimated | Own milestone (M7); exact procedural blend representation chosen early so XT fidelity doesn't require rework. |
| Determinism vs parallelism tension | Deterministic reduction ordering mandated in M0 primitives; CI determinism harness runs on every merge. |
| Team knowledge ramp | Canonical references: Piegl & Tiller (NURBS), Hoffmann *Geometric & Solid Modeling*, Mäntylä *Solid Modeling*, Patrikalakis & Maekawa (intersections); study OCCT/truck code for prior art. |
