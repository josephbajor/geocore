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
refinement, watertight-with-boundary invariant tested). Deferred, first needed
in M6: knot removal, degree elevation, periodic NURBS (M3), degenerate-patch
reporting (M2 checker). Deferred within tessellation: triangle quality
optimization. All 123 tests green in debug and release; determinism goldens
pinned for both crates.

## M2 — Topology + primitives (≈ 6–8 weeks)
Full Parasolid entity hierarchy, Euler operators, body types, tolerant entity plumbing.
The checker, v1. Primitive body constructors. Whole-body watertight tessellation
(cross-face crack elimination at shared edges).

**Exit:** primitives construct checker-clean; Euler invariant property tests pass;
a primitive body renders watertight in a throwaway viewer.

## M3 — XT interchange (starts with M2, runs long)
- **M3a Read (Tier 0):** text + neutral-binary parser, schema versioning, topology and
  Tier-0 geometry reconstruction, tessellate-and-view arbitrary real-world XT files.
  Begin harvesting the corpus (GrabCAD, vendor samples, Solid Edge CE exports).
- **M3b Write (Tier 1):** emit XT for self-authored analytic bodies; round-trip
  ourselves; validate empirically in Solid Edge CE (import, checker, re-export, diff).
- **M3c Tier 2/3 fidelity:** procedural surfaces, SP-curves, intersection curves,
  attributes, assemblies — extends through M6.

**Exit (M3b):** 100% of self-authored analytic bodies import into Solid Edge with zero
errors and survive a there-and-back round trip.

## M4 — Intersections (≈ 10–14 weeks, hardest math)
Curve/curve, curve/surface, then SSI: quadric/quadric closed forms, marching with
adaptive stepping, subdivision start-point discovery, tangent/singular handling,
small-loop detection. Extrude/revolve land here too (they need edge geometry but not
booleans) — first "real modeling" demo.

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
