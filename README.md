# cad_prototype

An open, performant B-rep modeling kernel built for interoperability with
Parasolid-based CAD systems (SolidWorks, Solid Edge, NX, Onshape) via XT
round-trip. It is the geometry and topology foundation for an eventual full
parametric CAD application; feature history and regeneration are later layers.

- **Specification:** [docs/kernel-spec.md](docs/kernel-spec.md)
- **Construction roadmap:** [docs/kernel-roadmap.md](docs/kernel-roadmap.md)
- **Machine-readable capability ledger:** [docs/kernel-support.tsv](docs/kernel-support.tsv)
- **Licensed-host certification loop:** [docs/oracle-loop.md](docs/oracle-loop.md) — automated via `scripts/oracle_loop.py`

## Layout

Application and product code should start at the `kernel` facade. The numbered
lower layers remain available for kernel implementation and trusted adapters,
but their raw storage and assembly APIs are not the application compatibility
boundary.

| Crate | Layer | Contents |
|---|---|---|
| [`crates/kcore`](crates/kcore) | L0 foundations | Deterministic robust `orient2d`, `orient3d`, positive-inside-CCW `incircle`, and exact cyclic `polygon_orientation2d` predicates with conservative floating filters or exact expansion evaluation, interval filters, tolerance policy (Parasolid numeric regime), typed errors, generational entity arenas with copy-on-write undo frames, deterministic parallel primitives, deterministic transcendental math (musl port — platform libm is banned in kernel code via clippy `disallowed-methods`) |
| [`crates/kgeom`](crates/kgeom) | L1 geometry | Analytic curves (line/circle/ellipse), true 2D line/circle/NURBS pcurve evaluators, and analytic surfaces (plane/cylinder/cone/sphere/torus) with exact bounding boxes, NURBS engine (Piegl & Tiller) with homogeneous 2D/3D knot operations and conservative active-subrange control-hull boxes, closest-point projection, deterministic trimmed-face tessellation with exact streaming trim-loop winding and explicit refinement-limit errors, evaluator conformance harness |
| [`crates/kgraph`](crates/kgraph) | L1.5 geometry graph | Immutable analytic, NURBS, and procedural geometry nodes with typed dependencies, deterministic identity, and bounded evaluation |
| [`crates/ktopo`](crates/ktopo) | L2 topology | Parasolid entity hierarchy (body→region→shell→face→loop→fin→edge→vertex), finite conservative face UV domains, typed entity-tolerance provenance and transaction-owned growth budgets, independent per-fin pcurves, bounded curve-less tolerant edges, reusable validated polygon-with-holes profiles and checked prism extrusion, transaction-owned pcurve-aware Euler edits, private generic Store mutation with transaction-scoped checked assembly, deterministic mutation/lineage/tolerance journals, journal-returning checked solid/sheet/wire/acorn constructors, shared incidence validation, and pcurve-driven watertight tessellation |
| [`crates/kops`](crates/kops) | L3 operations | Provisional M4 intersection foundation: exact analytic special cases plus early sampled NURBS curve/curve, curve/surface, and surface/surface experiments; generic completeness and boolean-ready pcurve results remain gated |
| [`crates/kxt`](crates/kxt) | L5 interchange | Atomic modern-schema Parasolid XT (`.x_t`/`.x_b`) import for the supported geometry subset, including zero-multiplicity null-knot normalization and proof-bearing transmitted intersection charts through bounded two-sample line, three-sample quadratic, four-sample cubic, five-sample polyline, and seven-sample polyline dual-offset NURBS slices, plus a deterministic schema-13006 text writer for self-authored analytic solids, sheets, wires, acorns, and bounded tolerant edges encoded as trimmed SP-curves over 2D B-curves (clean-room from the published XT Format Reference) |
| [`crates/kernel`](crates/kernel) | Supported native facade | Session/part lifecycle, opaque semantic IDs and views, contextual block/profile-extrusion construction, deterministic rigid body copy, checking/evaluation/tessellation, and typed X_T import/export without raw topology or graph leakage |
| [`examples/kernel-lifecycle`](examples/kernel-lifecycle) | Facade-only client | Executable application lifecycle with `kernel` as its only direct kernel dependency |

### Facade quickstart

Run the facade-only lifecycle client and write its deterministic X_T result:

```sh
cargo run -p kernel-lifecycle -- target/kernel-lifecycle.x_t
```

The client constructs a block, traverses semantic faces/edges/vertices, checks
and tessellates the body, evaluates one supporting surface, and exports through
the `kernel` API. Lower-layer X_T reconstruction and oracle tools remain
trusted adapter and conformance seams; they are not examples of the ordinary
application boundary.

## Current Status

- M0 foundations now include deterministic exact-fallback `orient2d`, `orient3d`,
  and `incircle`; M1 geometry and M2 topology/primitives also have implemented
  alpha slices. The public `polygon_orientation2d` slice and its non-copying
  streaming companion, `polygon_orientation2d_iter`, compute the exact cyclic
  shoelace expansion. Fewer than three vertices, any non-finite coordinate,
  and an exactly zero expansion
  return `Orientation::Zero`; cyclic rotation preserves the result, reversal
  flips every nonzero result, and repeated vertices are allowed under algebraic
  area semantics. Evidence includes 20,000 random integer polygons against an
  `i128` oracle, a `2^52`-translated unit square whose naive shoelace sum is
  zero, and the cross-platform determinism golden. `incircle` remains a public
  predicate with conformance evidence but has no production topology decision
  consumer yet. Strict first-chart SSI
  polygon convexity is the first audited
  exact decision consumer: it accepts only at least three finite vertices with
  `orient2d(...) == Orientation::Positive` at every turn, so exact collinear
  and all other nonpositive turns fail closed. Oblique profile extrusion is the
  next migrated consumer: `extrude_profile_along_in` classifies the stored-frame
  `(x, y, translation)` scalar triple with exact `orient3d`, rejects exact
  coplanarity before allocation, and reflects the chart for a negative result.
  Its integer-source adversary has normal dot `+3` while the former normalized
  `translation.dot(frame.z())` rounds to zero; both directions now construct
  deterministically, while ordinary oblique extrusion remains Full-valid.
  Coincident bounded Plane/Plane window construction now also uses exact
  `orient2d` in both monotone hull chains: only exact `Positive` turns are
  retained, while `Zero` and `Negative` turns pop. A private production-helper
  fixture near `2^52` has exact `i128` determinant `+1` where the former rounded
  turn is zero, and pins middle-vertex retention, repeat, input
  rotation/reversal, and exact-collinear removal. Rectangle-overlap boundaries
  draw consecutive directions from two axis pairs, so that cancellation triple
  cannot be encoded as consecutive exact public window edges; the public seam
  instead pins an eight-vertex `Complete` Region, cyclic exact-positive turns,
  repeatability, operand-swap semantic parity, and non-finite range rejection.
  `kgeom` trim cleaning plus `TrimmedSurface` outer/hole winding now consume the
  streaming exact sign without allocating a coordinate copy; rounded
  `TrimLoop::signed_area` is reporting only. `kops` polygonal-region
  canonicalization now compares exact winding in both parameter charts to
  derive `Same` or `Reversed`, rejects zero or a contradictory declared chart
  relation, and normalizes a negative first chart by reversal. Its
  integer-source adversary has exact doubled area `+2` where the former
  origin-relative floating sum is zero. `ktopo`
  `face_case_a` now requires exactly one exact-positive outer loop; exact-zero
  and non-finite loops fail closed before the existing periodic anchoring and
  outer-first ordering. Its `2^52`-translated unit-square adversary also has
  exact doubled area `+2` while naive shoelace summation is zero. Remaining
  concrete decision-audit debt includes checker sampled-loop winding and
  outer-loop selection, conic discriminant root-count classification,
  NURBS-plane sign certification, and other raw topological sign branches.
  `insphere`, an `incircle` production decision consumer when required, the
  broader topological-decision audit, and full conformance remain ahead.
- M2.5 is in progress and remains the architecture gate. Transaction-owned checked
  topology, pcurve-aware Euler edits, deterministic journals, tolerance provenance,
  bounded operation contexts, `Fast`/`Full` checking, adaptive face-domain proofs, and
  the immutable geometry graph with persistent verified intersection descriptors have
  landed. General NURBS/mixed-parameter incidence, periodic and unsupported mixed-boundary
  containment, curved-loop/shell proofs, operation-specific tolerance rules, and
  production-scale ownership/dependency benchmarks remain.
- M3 X_T interchange is in progress. Modern text and neutral-binary reading, atomic
  reconstruction, deterministic analytic text writing, bounded tolerant-edge SP-curves,
  safe offset surfaces, certified clamped periodic/closed B-surfaces, canonical
  transmitted Plane/Offset/NURBS intersection-chart slices, and bounded
  noncanonical affine direct-Plane/B-surface, safe-Offset(Plane)/B-surface,
  direct-Plane/Offset(B-surface), direct constant-normal Offset(B-surface)/direct
  B-surface, independent direct one-descriptor Offset(B-surface)/Offset(B-surface),
  and direct-B-surface/B-surface slices are implemented. The noncanonical direct
  offset slices are limited to two through five finite-open samples, preserve
  ordered roots and paired UVs, and cover polynomial or rational bases and
  operand swap at exact `14336/28672/43008/57344` Work, `N` Items, and Depth
  10. Nested, shared-basis, multi-offset, null/mixed, and out-of-range
  noncanonical forms remain unsupported. The committed
  corpus includes a production 7,423-node Onshape part and reconstructs under the exact v15
  profile at `440483945/22/10` Work/Items/Depth. Two-sample dual-offset record 3595
  certifies independently; v13 admits five-sample record 4230 at isolated
  `17285120/5/10`, v14 admits two-sample Plane/Offset record 3609 at isolated
  `4277250/2/10`, and v15 admits two-sample dual-offset record 6044 at isolated
  `4352000/2/10`. Production next stops atomically before four-sample dual-offset
  record 5921 at its exact `454258793`-Work request. At that attempted budget,
  record 5921 still fails closed: its canonical cubic first pcurve materially
  leaves the original open nonperiodic source domain, so the retained report
  remains the exact v15 prefix with an empty rollback. Broader cyclic
  B-geometry, periodic/circular pcurves,
  remaining intersection/procedural families, assemblies, older schemas, neutral-binary
  writing, and broader external Parasolid certification remain.
- M4 intersections are useful but provisional. Shared `Complete`/`Indeterminate`
  evidence, source-provenanced adaptive NURBS covers, interval implicit exclusion,
  Work-bounded polishing, exact algebraic seed/overlap certificates, selected paired
  pcurves, a first exact varying-normal Offset(NURBS) arm with global-X-,
  global-Y-, and global-Z-normal planar-NURBS, analytic-Plane, or one-descriptor
  safe-Offset(Plane) peers, plus the complete one- through four-descriptor
  rational-quarter-cylinder family against direct global-axis analytic Planes
  only, and bounded
  coincident
  Plane/Cylinder/Sphere/Cone/Torus regions including exact polar-cap and
  same-row adjacent, exact same-column vertical (reviewed at
  `[0,2]`/`[1,2]`), either exact full latitude row, and an exact mixed-axis
  three-cell L path in two real orientations and the generic exact four-
  positive shared-seam path family in two real orientations, plus the disjoint
  exact lower-stem `[0,0]`/`[0,1]`/`[0,2]`/`[1,1]` and upper-stem
  `[0,1]`/`[1,0]`/`[1,1]`/`[1,2]` T trees and the exact left
  `[0,0]`/`[0,1]`/`[1,0]`/`[1,1]` and right
  `[0,1]`/`[0,2]`/`[1,1]`/`[1,2]` four-positive 2×2 cycle family with two
  certified-empty siblings, plus the exact disconnected outer-column vertical
  pairs `[0,0]`/`[0,2]`/`[1,0]`/`[1,2]` with middle-column `[0,1]`/`[1,1]`
  certified empty, all four exact disconnected isolated-corner plus three-cell
  mixed-axis L layouts—`[0,0]` + `[0,2]`/`[1,1]`/`[1,2]`, `[1,2]` +
  `[0,0]`/`[0,1]`/`[1,0]`, `[1,0]` + `[0,1]`/`[0,2]`/`[1,2]`, and
  `[0,2]` + `[0,0]`/`[1,0]`/`[1,1]`—with their respective two omitted
  graph-cut siblings certified empty, and exactly five positive cells with one
  certified-empty sibling, in the polar-by-wide family have landed. The
  connected four-positive routes are disjoint by degree sequence: path
  `2,2,1,1`, T `3,1,1,1`, and cycle `2,2,2,2`. Each T certifies the other two
  siblings empty,
  simultaneously proves and removes exactly three reverse-oriented bit-exact
  seams, requires one outer cycle with no artificial seam, and restores the
  parent map and maximum child/parent residual. Its real fixtures pin
  repeat/swap while one-ULP mutation and duplicate-edge ambiguity fail closed.
  The cycle route simultaneously proves and removes exactly four reverse-
  oriented bit-exact adjacencies, requires one outer cycle with no artificial
  seam, and restores the parent map and maximum child/parent residual. Its real
  fixtures pin repeat/swap while one-ULP mutation and duplicate-edge ambiguity
  fail closed. The outer-column disconnected arm merges both exact latitude
  seams into exactly two canonical regions, excludes both longitude separators,
  restores parent maps and maximum child/parent residuals, and pins repeat/swap
  plus one-ULP/ambiguity rejection. Each singleton-plus-L arm proves and removes
  both reverse-oriented bit-exact L seams, requires zero occupied-boundary contact
  with either empty cut separator and no bit-exact contact between the singleton
  and merged component, and likewise returns exactly two canonical regions with
  restored parent maps and maximum child/parent residuals. Together these
  routes exhaust the exact disconnected four-positive graph layouts in the 2×3
  decomposition, with repeat/swap, exact 6/5 piece, 147/146 pair, and 588/587
  arc N/N-1 evidence plus one-ULP/ambiguity rejection. The five-positive arm
  simultaneously removes every
  internal reverse-oriented
  bit-exact seam and requires one outer cycle; corner-empty cycle-plus-tail and
  edge-middle-empty tree fixtures pin repeat/swap, parent mapping, residuals,
  and one-ULP/ambiguity rejection. The all-six-positive arm now admits the
  complete 2×3 grid with no empty sibling only after all seven reverse bit-
  exact internal adjacencies are removed simultaneously, leaving one
  unambiguous outer cycle and no artificial seam edge. Its real fixture pins
  parent mapping, residuals, repeat/swap, and one-ULP/ambiguity rejection. The
  non-cap row retains eight vertices and every six-cell decomposition keeps the
  exact 6/5, 147/146, and 588/587 piece/pair/arc N/N-1 ceilings; other polar
  layouts remain unsupported.
  The both-wide Cartesian 3×3 arm now also admits all four exact disconnected
  seven-positive rotations in which one occupied corner is a singleton, its
  two orthogonal neighbor cells certify empty, and the other six occupied cells
  form one exact component. The existing connected-seven merger keeps
  precedence. The disconnected proof requires zero occupied-boundary contact
  at both empty separators, cancels the six-cell component's six internal seams
  simultaneously, and rejects any surviving artificial seam or bit-exact
  contact between components. Success returns exactly two canonical parent-
  mapped regions with maximum child/parent residual propagation. Real fixtures
  pin all rotations, repeat/swap, exact 9/8 piece, 252/251 pair, and 1,008/1,007
  arc N/N-1 admission; one-ULP seam and duplicate-edge ambiguity fail closed.
  Other seven-positive 3×3 layouts outside the exact connected and corner-
  singleton-plus-six-component families remain indeterminate.
  Varying-normal chains against a direct analytic Plane retain exact outer-to-
  inner metadata, prove every intermediate and final radius finite and positive
  from the original basis, use the derived sheet only for discovery, preserve
  certificate budgets, and pin exact 2–5 graph Work/depth with N/N-1 evidence;
  other peers remain one descriptor and chain depth five or greater remains
  unsupported. Compatible intersecting
  planar constant-normal dual Offset(NURBS) chains now cover the full 1–4×1–4
  matrix with original-source proof at exact 14,336/1,024/depth-10 certificate
  use and maximum 10 Work/depth-5 graph traversal; the strict-separated
  complete-miss arm remains. Exact algebraic curve-pair search keeps magnitude
  twelve as its compatibility default and stable prefix while explicit
  configuration admits the reviewed thirteen and fourteen shells. Fourteen
  owns exactly 254 carrier forms and 9,825 residual forms—24 and 1,704 more than
  thirteen—and alone certifies the reviewed noncoplanar normalized-`1/3`
  fixture after every ceiling through thirteen returns no certificate. Earlier
  goldens remain unchanged; invalid ceilings and broken, overflowing, non-
  finite, or out-of-range forms fail closed. Polygonal
  profiles with holes, checked complete-body rigid copy with direct Plane or
  safe finite Offset(Plane)-backed Plane/Plane lines, and Plane/Sphere latitude
  or oblique circles backed by direct Plane or safe finite Offset(Plane) sources
  plus direct Sphere or safe finite Offset(Sphere) sources whose effective
  Sphere radius is positive and finite, in either source order and through
  leaf-inclusive depth at most 64, and checked nonzero-normal oblique
  polygonal-profile extrusion are the first M4 modeling consumers. Rigid copy
  now reissues every current operation-generated verified-NURBS intersection
  family, including one- through four-level nested dual offsets and oblique
  frames. Its first transmitted tranche covers Plane/Plane over direct or safe
  nested exact-plane roots, direct Plane/NURBS in both orders, direct
  NURBS/NURBS, direct one-descriptor Offset(NURBS)/NURBS in both orders,
  exactly one-descriptor Offset(NURBS)/direct-Plane charts in both orders, and
  only the canonical finite-open two-sample degree-1, witnessed three-sample
  quadratic, witnessed four-sample cubic, canonical five-sample degree-1, or
  canonical seven-sample degree-1 dual Offset(NURBS) charts in either ordered-
  root arrangement. These complete the existing canonical 2/3/4/5/7-sample set.
  The two-sample line, witnessed three-sample quadratic, and witnessed four-
  sample cubic each admit independent exact ordered chains of one through four
  descriptors per root across a full 4×4 matrix and both trace orders; the five-
  and seven-sample families remain exactly one descriptor per root. All five
  require distinct ordered roots with distinct direct nonperiodic terminal
  NURBS basis handles.
  The line uses unweighted two-
  control carrier/pcurves on `[0,0,1,1]` over `[0,1]` without witnesses; the
  quadratic uses unweighted degree-2 three-control carrier/pcurves on
  `[0,0,0,2,2,2]` over `[0,2]`; and the cubic uses unweighted degree-3 four-
  control carrier/pcurves on `[0,0,0,0,3,3,3,3]` over `[0,3]`. Both witnessed
  higher-order families retain exact position and paired-UV interpolation
  witnesses. The five-sample family uses unweighted degree-1 five-control
  carrier/pcurves on `[0,0,1,2,3,4,4]` over `[0,4]`; the seven-sample family
  uses unweighted degree-1 seven-control carrier/pcurves on
  `[0,0,1,2,3,4,5,6,6]` over `[0,6]`. Neither polyline family has interpolation
  witnesses or a carrier period. Generic graph persistence walks each dual root
  to its direct NURBS terminal and binds its complete outer-to-inner distance
  order bit-for-bit; it atomically rejects reordered same-total, extra, missing,
  or stale chains. The graph trace constructor retains at most four exact
  ordered descriptors per trace and rejects a fifth; a live depth-five source
  paired to the maximum-depth trace therefore fails atomically at graph
  insertion. Broader depth remains graph representation/binding work. Rigid
  copy transforms both distinct terminal bases and the line, five-sample,
  or seven-sample carrier; for either witnessed higher-order family it transforms
  exact positions and rebuilds carrier controls by the public interpolation
  formula. It copies both ordered roots and full offset/basis and pcurve chains,
  preserves exact distance order, terminal source binding, metadata, tolerance,
  and exact UV witnesses, publicly recertifies—including every witnessed
  quadratic and cubic 4×4 pair—and protects the copied roots and complete basis
  closures transitively.
  Facade rejection happens before scope creation, while lower-copy rejection
  restores every Body/Region/Shell/Edge/Vertex and Curve/Surface/Pcurve/Point
  count plus future point identity. Graph-valid shared-basis or periodic charts,
  nested five-/seven-sample roots, Offset(Plane) peers, altered higher-order
  witnesses, and other sample counts remain copy-unsupported;
  attributes are blocked on an authorable storage contract, and non-rigid
  transforms remain.
  General root discovery, complete verified residuals across every result family, and
  boolean-ready paired-pcurve branches remain gated.
- M5-M8 are not started: there are no end-to-end booleans, general sweeps/sewing,
  blends/offsets/shelling, stable C API, or production hardening yet.

The authoritative foundation priority and handoff order live in
[`docs/projects/foundation-projects.md`](docs/projects/foundation-projects.md).
The longer milestone contracts remain in the construction roadmap; they are
not a second execution queue.

## Building

```sh
# Tight inner loop for one package target.
python3 scripts/test_lanes.py focused -p kxt -t read

# Normal edit/commit gate: workspace unit tests plus representative integration
# coverage across determinism, topology, operations, interchange, and facade use.
python3 scripts/test_lanes.py fast

# Broad local gate: every non-corpus integration target plus tooling.
python3 scripts/test_lanes.py standard

# Architectural API-boundary examples and other workspace doctests.
python3 scripts/test_lanes.py docs

# Mandatory pre-merge/handoff gate, including every corpus ratchet.
python3 scripts/test_lanes.py full

cargo clippy --all-targets -- -D warnings
```

Direct `cargo test --workspace` remains complete and supported. The
[test-throughput contract](docs/projects/test-throughput.md) documents the
fail-closed lane inventory, timing evidence, and CI scheduling policy.

Requires stable Rust (1.93+). The lane runner requires Python 3.11+ and uses
only the standard library. The workspace is dependency-free by policy at L0.

## Determinism contract

Same input → bit-identical output on every platform, thread count, and run.
CI enforces this with golden-hash tests across Linux/macOS/Windows in both
debug and release. Changing a golden value is a reviewed, intentional event.
