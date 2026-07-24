# Spine Consolidation — cylinder-pair boolean paths

**Status: DORMANT — operator-directed only.** This document is not a queue
source. Swarm agents must not schedule, start, or extend work from this file,
and must not link it from `ORCHESTRATION.md` or `docs/kernel-roadmap.md`. It
activates only when the operator explicitly assigns a work package that cites
it by name. Any commit touching the modules listed here without such an
assignment is out of contract.

## Why this project exists

A 2026-07-24 dependency trace of the boolean paths found three layers:

- **Layer A — config-agnostic infrastructure (the real kernel).** Exact
  algebra (`kgraph::intersection`, `kgraph::exact`) → section layer
  (`kernel/src/section`, `BodySectionGraph`) → generic arrangement
  (`face_arrangement.rs:676` and adapters) → `plan_mixed_shell` →
  `mixed_shell_materialize` → `ktopo` assembly floor (`analytic_shell`,
  `transaction`, `euler`) → `check_body`. Clean walls: no `boolean/` file
  imports raw intersection algebra; everything flows through the section
  product.
- **Layer B — config-specific adapters built ON the spine.** Relation
  recognizers and plan grafts that classify a pair, then feed the generic
  spine. The lens, coincident-caps, and skew paths already work this way.
- **Layer C — config-specific bypasses.** Planners that hand-assemble
  `AnalyticShellInput` and skip arrangement/plan/materialize entirely, each
  pair-bonded to a bespoke `ktopo` shell certifier that exists only to bless
  that planner's output. Plus a duplicate engine: the ring-cut path in
  `curved_realize.rs` solves the same single-cylinder mixed case that the
  bounded-arc spine path solves generically.

The growth law this project breaks: `ktopo/src/shell_proof.rs:182-274` is a
16-step per-shape certifier cascade, and every Layer C planner ships with a
matching cascade entry. Each new configuration currently costs a planner + a
proof + pinned tests (~33k lines per configuration). That is rule R1
enumeration one level up, and it regrows unless the pairing law is broken.

## Objective

Route every cylinder-pair boolean configuration through the general
arrangement → plan → materialize spine, then delete the Layer C planners and
their paired certifiers. Every work package must land net-negative in lines
outside `tests/`.

## Non-goals

- No new configurations while this project is active. Rung queue is frozen.
- The general curved-shell embedding certifier (replacing the cascade) is
  explicitly deferred; WP5 only freezes the cascade, it does not replace it.
- No solver/feature-tree work. No changes to Layer A algebra semantics.

## Acceptance gate (built 2026-07-24)

The lifecycle corpus under `crates/kernel/tests/` is the parity oracle. Its
implementation-derived work-count pins were removed (budgets are now derived
at runtime: a stage must meter nonzero work, admit at measured N, refuse
atomically at N−1). Geometric, topological, mass-property, and X_T
assertions are ground truth and must never be edited to make a migration
pass. A migration is complete only when the corpus is green before and after
with the same fixtures, and `check_body` full validation still passes.

## Work packages

### WP1 — parallel-cylinder contact onto the spine

Replace the hand-assembly in `boolean/parallel_cylinder_contact.rs` (+
`contact/secant.rs`) with a boundary-prep adapter feeding `plan_mixed_shell`,
patterned on the two working precedents:
`mixed_shell_plan/parallel_cylinder_lens.rs` (graft emitting a generic plan)
and `mixed_shell_plan/cylinder_pair.rs` (skew: prep → `plan_mixed_shell:98` →
generic materialize). Keep `parallel_cylinder_relation.rs` as the recognizer;
its `CertifiedAxialContact` arm re-routes in
`parallel_cylinder_pipeline.rs:157`. Delete
`ktopo/src/parallel_cylinder_contact_shell_proof.rs` and its cascade entry
(`shell_proof.rs:199`) in the same commit as the planner deletion.

### WP2 — common-support interval path onto the spine

`boolean/parallel_cylinder_interval.rs` + `axial_interval_sweep.rs` build
shells from a private 1-D interval algebra. The spine's periodic arrangement
already represents axial cells; the adapter maps the certified
common-support relation into section evidence for
`arrange_mixed_periodic_face`. The 1-D sweep survives only if the interval
evidence genuinely cannot be expressed as arrangement input — if kept, it
becomes an internal helper of the adapter, not a planner.

### WP3 — internal tangency onto the spine (hardest)

`boolean/parallel_cylinder_internal_tangency.rs` hand-assembles the
tangent-contact shells. Migration requires the periodic arrangement to admit
tangency-degenerate cells (contact points where two cell boundaries meet
without crossing). This is the one package with real new math in the
arrangement core; scope that extension first, land it with its own unit
proofs, then the adapter. If the degeneracy extension stalls, the honest
fallback is refusal (`Indeterminate`), not retention of the bypass.

### WP4 — retire the ring-cut engine

`curved_realize.rs::realize_selected_result` and satellites
(`curved_host_bands.rs`, `curved_cavity.rs`, `curved_contact.rs`,
`curved_support_separation.rs`, `curved_source.rs`, `convex_containment.rs`)
duplicate the bounded-arc spine path (`execute_mixed_bounded_arc`,
`curved_pipeline.rs:538`) for single-cylinder mixed booleans. Route all such
ops through bounded-arc, verify corpus parity, delete the ring-cut branch and
whichever portal/chord/cap-reaching/two-host/prism certifiers lose their
last producer. `realize_analytic_shell_inputs` and the disjoint rigid-copy
path stay (shared floor, not bypass).

### WP5 — freeze the certifier cascade

After WP1–WP4, `shell_proof.rs` keeps only certifiers with a live producer.
Rule (mechanically checkable): no new `certify_*_shell` cascade entries and
no new files matching `*_shell_proof.rs`, `parallel_cylinder_*.rs`,
`*_cylinder_pipeline.rs` may be added. A configuration that the spine cannot
express is refused, not special-cased.

## Per-package landing checklist

1. Adapter lands behind the existing dispatch arm; old planner still active.
2. Parity corpus green on the adapter path (flip dispatch, run corpus).
3. Planner + paired certifier + their `tests.rs` deleted in one commit;
   cascade entry removed; `scripts/package_contract.py` inventory updated in
   the same commit (that update is the review event — no orphan entries).
4. Commit is net-negative in non-test lines; state the delta in the message.
5. `python3.14 scripts/test_lanes.py standard` green; clippy `-D warnings`
   green; push the same day — unpushed work is unreviewed work.

## Invariants while the project is active

- `AnalyticShellInput::new` may appear only in `mixed_shell_materialize`,
  `curved_realize.rs`, and `ktopo` itself. Grep is the check; a new call
  site outside those files is a bypass being reborn.
- Planner and certifier die together; neither is ever orphaned.
- No assertion in the parity corpus may be weakened or re-pinned to make a
  migration pass. Fixture changes require operator sign-off.
- Refusal is always an acceptable outcome; a bypass is never one.

## Definition of done

Dispatch in `curved_pipeline.rs` routes parallel and skew cylinder pairs
into recognizer → spine with no hand-assembled shells; the ring-cut engine
is gone; the cascade has no entry without a live producer; the parity corpus
and the full test lanes are green; total non-test line count in
`crates/kernel/src/boolean` + `crates/ktopo/src` is lower than at project
start (baseline: commit `70fa59d`, 49,511 + 74,853 lines).
