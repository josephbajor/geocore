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
| [`crates/kgeom`](crates/kgeom) | L1 geometry | Analytic curves (line/circle/ellipse), true 2D line/circle/NURBS pcurve evaluators, and analytic surfaces (plane/cylinder/cone/sphere/torus) with exact bounding boxes, NURBS engine (Piegl & Tiller), closest-point projection, deterministic trimmed-face tessellation with explicit refinement-limit errors, evaluator conformance harness |
| [`crates/ktopo`](crates/ktopo) | L2 topology | Parasolid entity hierarchy (body→region→shell→face→loop→fin→edge→vertex), finite conservative face UV domains and tolerance metadata, independent per-fin pcurves, bounded curve-less tolerant edges, pcurve-aware Euler variants, scoped failure-atomic transactions, deterministic mutation/lineage journals, shared incidence validation, and pcurve-driven watertight tessellation |
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
  The X_T corpus inspector records both the Fast gate and Full outcome/gap categories.
  A first whole-interval proof slice now certifies exact affine and harmonic incidence:
  all stored curves on planes, cylinder generators/sections, sphere
  sections, and matching analytic pcurve lifts on plane and revolved surfaces.
  Adaptive full-curve containment, production seam/pole/apex interchange fixtures,
  operation caller migration, a procedural geometry graph, operation-wide transaction/
  journal adoption, partition history, enforced topology mutation, richer errors/
  tolerance rules, and the adaptive proofs behind checker v2 must still land before
  booleans.
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
  whole-branch error bounds.
- M5-M8 are not started: there are no end-to-end booleans, general sweeps/sewing,
  blends/offsets/shelling, stable C API, or production hardening yet.

Immediate work per the roadmap: complete the M2.5 architecture gate, build the X_T
corpus/oracle harness in parallel, replace sampled general intersections with certified
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
