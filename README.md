# cad_prototype

An open, performant B-rep modeling kernel built for interoperability with
Parasolid-based CAD systems (SolidWorks, Solid Edge, NX, Onshape) via XT
round-trip. It is the geometry and topology foundation for an eventual full
parametric CAD application; feature history and regeneration are later layers.

- **Specification:** [docs/kernel-spec.md](docs/kernel-spec.md)
- **Construction roadmap:** [docs/kernel-roadmap.md](docs/kernel-roadmap.md)
- **Machine-readable capability ledger:** [docs/kernel-support.tsv](docs/kernel-support.tsv)

## Layout

| Crate | Layer | Contents |
|---|---|---|
| [`crates/kcore`](crates/kcore) | L0 foundations | Robust predicates, exact expansion arithmetic, interval filters, tolerance policy (Parasolid numeric regime), typed errors, generational entity arenas with copy-on-write undo frames, deterministic parallel primitives, deterministic transcendental math (musl port — platform libm is banned in kernel code via clippy `disallowed-methods`) |
| [`crates/kgeom`](crates/kgeom) | L1 geometry | Analytic curves (line/circle/ellipse), true 2D line/circle/NURBS pcurve evaluators, and analytic surfaces (plane/cylinder/cone/sphere/torus) with exact bounding boxes, NURBS engine (Piegl & Tiller) with homogeneous 2D/3D knot operations and conservative active-subrange control-hull boxes, closest-point projection, deterministic trimmed-face tessellation with explicit refinement-limit errors, evaluator conformance harness |
| [`crates/ktopo`](crates/ktopo) | L2 topology | Parasolid entity hierarchy (body→region→shell→face→loop→fin→edge→vertex), finite conservative face UV domains, typed entity-tolerance provenance and transaction-owned growth budgets, independent per-fin pcurves, bounded curve-less tolerant edges, reusable validated simple-polygon profiles, transaction-owned pcurve-aware Euler edits, private generic Store mutation with transaction-scoped checked assembly, deterministic mutation/lineage/tolerance journals, journal-returning checked solid/sheet/wire/acorn constructors, shared incidence validation, and pcurve-driven watertight tessellation |
| [`crates/kops`](crates/kops) | L3 operations | Provisional M4 intersection foundation: exact analytic special cases plus early sampled NURBS curve/curve, curve/surface, and surface/surface experiments; generic completeness and boolean-ready pcurve results remain gated |
| [`crates/kxt`](crates/kxt) | L5 interchange | Atomic modern-schema Parasolid XT (`.x_t`/`.x_b`) import for the supported geometry subset, plus a deterministic schema-13006 text writer for self-authored analytic solids, sheets, wires, acorns, and bounded tolerant edges encoded as trimmed SP-curves over 2D B-curves (clean-room from the published XT Format Reference) |

## Current Status

- M0 foundations, M1 geometry, and M2 topology/primitives have implemented alpha
  slices. They are not yet conformant with the full target contract.
- M2.5 is in progress and remains the next architecture gate. The per-fin pcurve storage,
  2D evaluators, analytic primitive authoring, shared incidence checking, pcurve-aware
  Euler creation, bounded curve-less tolerant edges, pcurve-driven body tessellation,
  conforming finite 2D B-curve X_T SP-curve slices, and the first explicit face-domain/
  tolerance propagation slice have landed. X_T import now derives certified conservative
  work domains for exact-edge and pcurve-bounded tolerant plane/cylinder/cone faces
  without sampling, while rejecting incompatible periodic branches as unknown. Per-fin
  integer-period charts now select branches without duplicating pcurve geometry, and the
  checker enforces chart validity plus actual pcurve-endpoint containment. Closed-use
  winding and singular endpoint markers are explicit and checker-validated; X_T SP-curve
  import infers singular markers. Paired lower/upper seam roles are checker-validated and
  exercised by a cylindrical-sheet primitive that remains checker-clean and tessellates
  through X_T round-trip. Exact-fin seam metadata itself is not reconstructed yet.
  The checker now has explicit `Fast` and `Full` reports: `Fast` preserves the current
  structural/sampled gate, while `Full` returns `Valid`, `Invalid`, or `Indeterminate`
  and, for Fast-clean bodies, enumerates every proof obligation the current
  implementation cannot discharge.
  The X_T corpus inspector records proven checker faults plus the Full outcome/gap
  categories.
  A first whole-interval proof slice now certifies exact affine and harmonic incidence:
  all stored curves on planes, cylinder generators/sections, sphere
  sections, and matching analytic pcurve lifts on plane and revolved surfaces.
  Robust-predicate loop proofs now certify planar straight-segment loops and simple
  circle/ellipse rings, while reporting proven crossings or overlaps as Full faults.
  Convex planar solid shells now receive global embedding/outward-orientation proofs,
  and single-face planar sheets inherit embedding from their simple loop. The committed
  block, plate, and disk fixtures consequently reach Full `Valid`. Whole sphere/torus
  shells and an exact sphere-cap-plus-plane shell are also certified, bringing every
  supported positive fixture in the committed X_T corpus to Full `Valid`; general curved
  multi-face shell proofs remain open.
  Face-domain containment now evaluates every available charted pcurve over its full
  active interval. Analytic curves and positive-weight clamped NURBS use conservative
  subrange boxes with deterministic adaptive subdivision; a witnessed exterior point is
  `Invalid`, proof-limit exhaustion remains `Indeterminate`, and only complete box
  coverage is certified. The tolerant-edge X_T round-trip exercises a 2D B-curve whose
  stored extent is ten times its active SP-curve trim, guarding the production path
  against accidental whole-curve bounds.
  Checked transaction commits preview deterministic net mutations and resolve them
  through committed/candidate topology-ownership and shared-geometry dependency indexes.
  Explicit and affected bodies receive the Fast checker while every commit still audits
  store-wide topology ownership closure, rejecting invalid unlisted bodies, orphan
  subgraphs, and cross-body topology sharing with atomic rollback and a typed error.
  Generic `Store` add/mutate/remove and unchecked commit are no longer public; low-level
  X_T reconstruction uses a transaction-scoped assembly facade and the same mandatory
  checked commit as ordinary operations. Compile-fail API guards and rollback/identity/
  ownership tests enforce that boundary. All public analytic primitive, simple planar
  sheet, line-wire, and acorn constructors are failure-atomic and have journal-returning
  variants.
  The planar sheet consumes a reusable robust-predicate-validated profile input and
  round-trips through X_T. Profiles with holes/curves and general-body/multi-face builders
  remain. Raw Euler functions are now
  topology-internal: public MVFS/KVFS, MEV/KEV, MEF/KEF, KEMR/MEKR, and KFMRH/MFKRH
  edits run through transaction methods, require pcurves when creating face-edge uses,
  and emit deterministic derived/split/merge/delete lineage.
  Entity tolerances now retain imported-versus-operation origin, original value,
  accumulated growth, and last operation. Transactions own aggregate tolerance-growth
  budgets, reject exhaustion with a typed error, journal every authorized change, and
  discard usage on rollback; X_T import stamps imported provenance and export preserves
  the metric value. Operation-specific propagation/combination rules and migration of
  every future tolerance-producing operation remain.
  Periodic/unclamped NURBS containment, unsupported exact/mixed boundary classes,
  production seam/pole/apex interchange fixtures, operation caller migration, a
  procedural geometry graph, large multi-body performance baselines for the landed
  incremental ownership/dependency index, partition history, richer errors/remaining
  tolerance rules, and the remaining adaptive incidence/loop/shell proofs behind checker
  v2 must still land before booleans.
- M3 is in progress: modern base-13006 schema edit scripts, text/neutral-binary
  reading, atomic reconstruction, and analytic text writing are implemented.
  X_T reconstruction now uses the same copy-on-write transaction mechanism instead of a
  full-store staging clone. A machine-readable corpus manifest and JSONL stage inspector
  now ratchet six committed fixtures; three external modern-schema files pass parse,
  reconstruction, checking, and tessellation. A separate metadata-only discovery catalog
  records eight more files that exposed real defects but cannot be redistributed because
  no source license was found. This is an observability foothold, not production-read
  evidence. Pre-13006 schemas,
  assemblies, intersection/procedural geometry, the rest of tolerant
  topology, periodic/circular pcurve interchange, neutral-binary writing, a
  production-scale corpus, and external Parasolid
  round-trip certification remain.
- M4 contains useful exact analytic solvers and sampled NURBS experiments, but it is
  provisional: fixed-grid discovery cannot prove misses or reliably recover small
  loops/tangencies, and SSI results do not yet carry paired pcurves with verified
  whole-branch error bounds. Exact homogeneous NURBS surface splitting/restriction,
  deterministic tensor-product Bezier patch extraction, and conservative active-patch
  control-net boxes now feed a reusable deterministic AABB BVH. Outward-rounded distance
  padding, interval-certified plane/control-hull exclusion, and reusable interval implicit
  fields for planes, spheres, cylinders, cones, and tori can prove broad-phase misses
  without sampling. Analytic/NURBS SSI now promotes those certified misses to `Complete`.
  Common CC/CS/SSI results carry explicit `Complete` or diagnostic `Indeterminate`
  evidence: sampled candidate paths preserve discoveries without claiming an unresolved
  empty sample is a miss. Deterministic exact subpatch isolation now refines surviving
  boxes adaptively, reports cell-budget/parameter-resolution limits, and proves additional
  misses hidden by a source control hull. Verified root seeds, paired pcurves,
  coincident-region handling, and verified whole-branch residuals remain.
- M5-M8 are not started: there are no end-to-end booleans, general sweeps/sewing,
  blends/offsets/shelling, stable C API, or production hardening yet.

Immediate work per the roadmap: run the external Parasolid oracle loop
(`xt_oracle` bundle export → licensed-host import → re-export → `xt_oracle compare`;
see [docs/oracle-loop.md](docs/oracle-loop.md)), complete the M2.5 architecture gate,
grow the X_T corpus in parallel, replace sampled general intersections with certified
subdivision and completion semantics, then exercise the architecture through
extrude/revolve and a narrow end-to-end analytic boolean slice.

## Building

```sh
cargo test          # all unit + determinism tests
cargo clippy --all-targets -- -D warnings
```

Requires stable Rust (1.93+). The workspace is dependency-free by policy at L0.

## Determinism contract

Same input → bit-identical output on every platform, thread count, and run.
CI enforces this with golden-hash tests across Linux/macOS/Windows in both
debug and release. Changing a golden value is a reviewed, intentional event.
