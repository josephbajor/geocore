# ORCHESTRATION.md — read this first

This file is the standing contract for every agent and human working on this
repository. It defines the goal, the rules that keep autonomous work pointed
at the goal, and how success is measured. It overrides habit, precedent in
old commits, and any doc that contradicts it. Keep it under 200 lines; it is
read at every handoff.

## North star

An open-source, Parasolid-interoperable B-rep modeling kernel that a real CAD
application can build on. The unit of progress is a **modeling capability an
application could not use yesterday and can use today** — failure-atomic,
journaled, checker-clean, deterministic, and externally validated through the
X_T oracle loop.

Certification breadth, robustness hardening, and documentation are support
work. They matter exactly as far as a capability on the critical path needs
them, and no further.

## Critical path

The queue head is always the next unfinished rung of the current vertical
slice. The active slice is the **first boolean ladder** (roadmap tranche 3):

1. Point-on-face and point-in-body classification for analytic solids.
   *(planar-face slice landed — `kernel::classify`; curved-face classes ride
   with rung 4)*
2. Face/face intersection curves stitched into edge graphs on two block
   bodies (planar/planar only). *(landed — `kernel::section` certified
   planar section graphs, ledger `modeling.sectioning`)*
3. `unite`/`subtract`/`intersect` for block/block — atomic, journaled,
   checker-clean, X_T-exported, and imported into a licensed host. *(landed —
   public typed facade; connected, proven-empty, atomic multi-body, and
   two-shell cavity results; Full proof; deterministic X_T; Onshape 6/6 import
   and 6/6 there-and-back at writer `b596027`)*
4. Extend the same ladder to block/cylinder (introduces curved SSI pcurves).
   *(active queue head — public cylinder construction, certified finite-cylinder
   classification, topology-owned conic/ring trim proofs, deterministic closed
   fragment stitching, exact Plane/Cylinder rings and bounded-arc endpoint
   topology, finite exact-family transverse ruling carriers with paired
   whole-range residual proof, topology-owned ruling trims with operation-shared source-edge root
   identity, deterministic closed mixed arc/ruling cycles across shared translated,
   permuted, and all-nonzero oblique exact frames,
   semantic Plane incidence for rounded frames, exact disk/annulus arrangement
   verification, bounded Line2d/Circle2d loop proof, and a failure-atomic
   analytic Plane/Cylinder shell assembler with clipped-cylinder Full proof,
   payload-agnostic exact boundary truth selection, and semantic finite-cylinder
   Full proof across general authored frames landed; proof-bearing curved face partition/classification and the
   first Full-checked axial intersection, cylinder-minus-block remainder bands,
   zero-cut truth-selected whole-source union/subtraction copies, one-ring
   axial cap-overlap connected union, and one-ring axial block-minus-cylinder
   blind pockets, two-port axial through-holes, zero-cut contained
   finite-cylinder cavities, two-ring two-sided connected unions, and
   support-separated axial exact-contact empty intersections, inverse-
   containment mixed-shell cavities, and certified flush axial cap-contact
   connected unions now land through the public facade with deterministic X_T
   emission and Fast self-import; proof-bearing periodic Section embeddings,
   internal disk/annulus arrangement adoption, exact source-root and carrier
   trim-scalar materialization, semantic sloped-support ruling recovery, and
   convex multi-chart clipped-cylinder proof now let bounded-arc intersections
   Full-commit across the authored frame/operand-order matrix for rectangular,
   three-sided, and five-support layouts, emit deterministic X_T, and Fast self-import;
   ordered planar-minus-cylinder atomically Full-commits every deterministic
   disconnected rectangular/three-sided component across four authored frames
   with deterministic X_T, Fast self-import, and N-1 batch refusal;
   endpoint-free cap edges/planning/incidence/materialization, multi-loop
   face proof, and the count-independent portal-cylinder shell theorem now let
   rectangular and five-support cap-retaining Unite/cylinder-left Subtract
   atomically Full-commit across the same four frames. The five-portal slice has
   exact topology/analytic-volume evidence, deterministic X_T, Fast self-import,
   and exact 14,966,784/1,095,237 N/N-1 shell-work refusal under the finite
   16,777,216 default. Cycle-wide certified integer-period lifting now admits
   the seam-crossing radius-1.7 cylinder-left Subtract across all four frames
   with 10/32/20 topology, literal-derived volume, deterministic X_T, and Fast
   self-import. Exact planar-shell admission is now independent of the optional
   typed convex half-space certificate, so a complete non-convex ten-support
   star/cylinder Intersect reaches general mixed planning and Full-commits at
   17F/45E/30V with literal-derived volume, deterministic X_T, and Fast
   self-import; pure planar BSP and convex curved shortcuts still require that
   certificate. The historical fifteen-payload Boolean identity imports and
   compares 15/15 in Onshape at writer `fedf1ab`; its rows remain byte-identical
   in the stale sixteen-payload bundle pending one cap-crossing host replay.
   Proof-keyed disk-cap chords
   now feed count-independent exact disk arrangements with source-arc lineage
   and period-lifted realization. Certified maximal transverse annulus traces
   and exact nested line-cycle planar cells now lower through shared finite
   source arcs. A count-independent chord-portal theorem now Full-certifies
   cap-crossing Unite and both ordered Subtract meanings alongside Intersect:
   the 4F/6E/4V and 9F/18E/12V results commit across both rigid frames/orders
   with analytic volume, deterministic X_T, and Fast self-import. Public
   `body_properties` certifies volume/centroid/area/inertia; `body_distance`
   Full-validates exact Plane/Cylinder solid pairs and encloses whole-material
   distance with a retained feasible witness;
   `body_clash` derives Clear/Clashing/Indeterminate clearance verdicts in the
   same scope without treating zero lower bounds as interference. Parallel-cylinder CSG now certifies strict overlaps, coincident caps, strict radial/axial separation, exact zero-gap cap contact, exact external radial tangency, exact common-cylinder support, and the manifold subset of exact unequal-radius internal radial tangency through positive axial overlap. Common support uses one certified four-endpoint total preorder and open-cell sweep to atomically Full-commit zero, one, or two canonical 3F/2E/0V bands with complete DerivedFrom/Merge/Face-Split lineage. Internal tangency retains directed containment and the same endpoint contract: Intersect returns the contained-radius overlap band or whole contained copy; contained-minus-containing returns regularized empty or one/two contained-radius bands; Unite returns a whole containing copy for zero contained-only tails, a compact 5F/4E/1V tangent shoulder for one tail, or a canonical 7F/6E/2V three-band/two-shoulder chain for two tails. Each shoulder's bounded full-period contact rings share one exact tangent vertex in one two-fin planar loop; the chain vertices are distinct. All created results retain complete ordered lineage, deterministic reports, independent properties, X_T/Fast self-import, rollback, and exact 64/63 relation, 420/419 per-band, 1092/1091 shoulder and 2052/2051 chain realization, 1113/1112 shoulder and 1860/1859 chain proof, 26/25 source-copy, 3953/3952 band-property, 7881/7880 shoulder-property, and 11809/11808 chain-property evidence across the rigid-frame/order/axis matrix; one-ULP neighbors, loose tolerance, and unsafe arithmetic fail closed. Exact nonparallel Cylinder/Cylinder SSI certifies strict full-cycle misses and exact topology for all four finite axial bounds. Root-free windows atomically publish zero, one, or two complete operation-local graph sheets only after exact-source and stored radicand, axial-window, common-chart, separation, and paired-residual proofs. Public Section adapts every contained sheet into a canonically oriented closed branch with source-ordered nonlinear pcurves, endpoint-free Whole fragments/components/rings, typed nonlinear periodic-embedding evidence, and read-only rigid/swap/replay coverage; it can therefore complete as empty or with one or two whole sheets. Perpendicular/non-right oracles, mixed-order canonicalization, projection-fold reverse retry, exact Bernstein fallback, boundary refusal, 128/127 classification, 512/511 two-sheet work, and 256/255 four-bound occupancy work land. A window with any boundary root remains one typed clipped-topology gap with no partial publication; contact roots, seam-wrapping open spans, unsafe cases, and persistence remain gaps. Containing-minus-contained remains a pinched non-manifold refusal. Next: certify non-wrapping open skew spans; keep the pinched difference typed non-manifold.)*

Work that does not advance the queue head needs an explicit justification
linking it to a rung ("rung 2 needs curve/curve overlap dedup because …").
"The foundation could be stronger" is not a justification; the boolean rungs
decide which foundation gaps are real.

## Hard rules

**R1 — General algorithms only.** The enumeration test: *if supporting a new
input configuration requires code that names that configuration — a new
function per layout, a new match arm over case shapes, a new "family" or
"sample-count" variant — you are enumerating an infinite space, not solving
it.* Implement the general algorithm, or return an honest
`Indeterminate`/`Unsupported` and record the gap. Hand-enumerated case
taxonomies will not be accepted in review, no matter how well-tested or
well-documented. (History: `sphere_sphere.rs` reached 9,400 lines enumerating
coincident-window region layouts before being rewritten; do not recreate
that.)

**R2 — Size is a smell.** A solver file above ~1,500 lines or a function
above ~150 lines requires a written decomposition rationale in the PR/commit
body. No file may exceed 3,000 lines.

**R3 — Red main blocks new work.** If CI on `main` is failing, the only
permitted work is fixing it. CI evidence claims in docs must reference an
actually-green run. Fuzz findings are real bugs; fix them before resuming.

**R4 — Docs are contracts and pointers, not narration.** Implementation
history lives in git; evidence lives in test names. Budgets (enforced by
`scripts/doc_budget.py` in CI):
- `ORCHESTRATION.md` ≤ 200 lines; `docs/kernel-roadmap.md` ≤ 500;
  each `docs/projects/*.md` ≤ 300; `README.md` ≤ 120.
- No table cell over 400 characters anywhere in `docs/`.
- A doc entry states: contract, status (one line), evidence pointer (test
  target names), open items. Nothing else. If you are pasting an
  accomplishment summary into a doc, delete it and improve the commit
  message instead.

**R5 — External validation on cadence.** Licensed-host runs are manually
dispatched, API-assisted catch-up batches; CI checks bundle/record identity
offline only. Mark affected evidence stale and queue it whenever writer bytes
change; certify the final batch, not superseded revisions, only after material
changes accumulate or before a release/conformance claim. Age alone does not
spend the host request budget. Self-round-trip is never certification.

**R6 — Preserve the non-negotiables.** Bit-exact cross-platform determinism
(kernel-owned math, no fast-math, golden hashes); exact predicates or
certified intervals for every topological decision; fail-closed
`Indeterminate` over guessed answers; typed errors, no panics on invalid
input; checked transactions for all topology mutation; `unsafe_code = forbid`.
These came from the project's first principles and are why the codebase is
sound. Nothing in R1–R5 licenses weakening them.

**R7 — Tests must pull their weight.** New tests assert values against
independent oracles or exact invariants, not the implementation's own output.
No template-stamped test-file families (one table-driven test instead). Every
new integration test target must state its wall-time budget; anything over
60s needs the corpus-ratchet justification used by the existing 14.

## Work-selection protocol (each handoff)

1. Read this file, then `docs/kernel-roadmap.md` (contracts + queue), then
   the specific project doc for your item.
2. Take the queue head unless it is blocked; if blocked, take the blocker.
3. Before writing code, state in one sentence which boolean-ladder rung (or
   explicitly-queued support item) the work advances.
4. Land the smallest slice that moves the rung: code + tests + one-line doc
   status update. Run `python3 scripts/test_lanes.py fast` before commit and
   `standard` before handoff.
5. Update the ledger row only when its exit evidence actually changed.

## Success metrics (reviewed at each orchestrator checkpoint)

- **Primary:** boolean-ladder rung count; new externally-validated modeling
  capabilities per week (ledger rows moving to `implemented_slice` or
  `conformant` with oracle evidence).
- **Health:** CI green streak on `main`; offline oracle-record freshness,
  queued recertification debt, and annual host request budget; `full` lane wall
  time (budget: 30 min, do not grow it); doc budget compliance.
- **Anti-metrics (high values are a warning, not progress):** commits/day,
  lines/day, count of `Certify`/`Document`/`Reconcile` commits between
  capability commits. Three consecutive days without a primary-metric change
  triggers a replan, not more hardening.

## Environment notes

- macOS Gatekeeper (`syspolicyd`) malware-scans every freshly built test
  binary on first run and can inflate test lanes ~75× (measured: `standard`
  63s → 4,700s). Exempt the terminal running agents via System Settings →
  Privacy & Security → Developer Tools, or run tests in a Linux container.
  Do not write timing docs from a contaminated host.
- Test lanes need Python ≥ 3.11 (`mise` shim: `python3.14`).
- The GitHub repository is `josephbajor/geocore`; CI is
  `.github/workflows/ci.yml`.

## Document map

- `ORCHESTRATION.md` (this file) — standing rules, queue head, metrics.
- `docs/kernel-spec.md` — target contract (stable).
- `docs/kernel-roadmap.md` — milestone contracts, gates, and the ordered
  queue detail.
- `docs/kernel-support.tsv` — capability ledger (status + evidence pointers).
- `docs/projects/*.md` — per-project contracts and open items.
- `docs/oracle-loop.md` — external certification procedure.
