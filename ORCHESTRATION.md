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
   topology, payload-agnostic exact boundary truth selection, and finite-cylinder
   Full proof landed; proof-bearing curved face partition/classification and the
   first Full-checked axial intersection, cylinder-minus-block remainder bands,
   zero-cut truth-selected whole-source union/subtraction copies, one-ring
   axial cap-overlap connected union, and one-ring axial block-minus-cylinder
   blind pockets, two-port axial through-holes, zero-cut contained
   finite-cylinder cavities, two-ring two-sided connected unions, and
   support-separated axial exact-contact empty intersections now land through
   the public facade with deterministic X_T emission and Fast self-import;
   inverse-containment mixed-shell cavities, constructive/full-ring and
   bounded-arc contacts/layouts, and curved licensed-host evidence remain)*

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

**R5 — External validation on cadence.** The oracle loop
(`docs/oracle-loop.md`) is automated; run it whenever writer bytes change,
and treat a `stale` certification older than 3 days of active interchange
work as queue-blocking. Self-round-trip is never certification.

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
- **Health:** CI green streak on `main`; oracle certification freshness;
  `full` lane wall time (budget: 30 min, do not grow it); doc budget
  compliance.
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
