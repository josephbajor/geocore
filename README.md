# cad_prototype

An open, performant B-rep modeling kernel built for interoperability with
Parasolid-based CAD systems (SolidWorks, Solid Edge, NX, Onshape) via XT
round-trip. It is the geometry and topology foundation for an eventual full
parametric CAD application; feature history and regeneration are later layers.

- **Specification:** [docs/kernel-spec.md](docs/kernel-spec.md)
- **Construction roadmap:** [docs/kernel-roadmap.md](docs/kernel-roadmap.md)

## Layout

| Crate | Layer | Contents |
|---|---|---|
| [`crates/kcore`](crates/kcore) | L0 foundations | Robust predicates, exact expansion arithmetic, interval filters, tolerance policy (Parasolid numeric regime), typed errors, generational entity arenas, deterministic parallel primitives, deterministic transcendental math (musl port — platform libm is banned in kernel code via clippy `disallowed-methods`) |
| [`crates/kgeom`](crates/kgeom) | L1 geometry | Analytic curves (line/circle/ellipse), true 2D line/circle/NURBS pcurve evaluators, and analytic surfaces (plane/cylinder/cone/sphere/torus) with exact bounding boxes, NURBS engine (Piegl & Tiller), closest-point projection, deterministic trimmed-face tessellation with explicit refinement-limit errors, evaluator conformance harness |
| [`crates/ktopo`](crates/ktopo) | L2 topology | Parasolid entity hierarchy (body→region→shell→face→loop→fin→edge→vertex) over generational arenas, independent per-fin pcurve uses with explicit parameter maps, Euler operators, pcurve-authoring primitive constructors, the body checker (including local edge/pcurve/surface incidence), pcurve-driven whole-body watertight tessellation |
| [`crates/kops`](crates/kops) | L3 operations | Provisional M4 intersection foundation: exact analytic special cases plus early sampled NURBS curve/curve, curve/surface, and surface/surface experiments; generic completeness and boolean-ready pcurve results remain gated |
| [`crates/kxt`](crates/kxt) | L5 interchange | Atomic modern-schema Parasolid XT (`.x_t`/`.x_b`) import for the supported geometry subset, plus a deterministic schema-13006 text writer for self-authored analytic solids, sheet bodies, wire bodies, and acorn bodies (clean-room from the published XT Format Reference) |

## Current Status

- M0 foundations, M1 geometry, and M2 topology/primitives have implemented alpha
  slices. They are not yet conformant with the full target contract.
- M2.5 is in progress and remains the next architecture gate. The per-fin pcurve storage,
  2D evaluators, analytic primitive authoring, local incidence checking, and
  pcurve-driven body tessellation slices have landed. X_T/Euler migration, face
  domains/tolerances, a procedural geometry graph, transactions/rollback, deterministic
  lineage journals, enforced topology mutation, richer errors/tolerance rules, and
  checker v2 must still land before booleans.
- M3 is in progress: modern base-13006 schema edit scripts, text/neutral-binary
  reading, atomic reconstruction, and analytic text writing are implemented.
  Pre-13006 schemas, assemblies, procedural/SP geometry, full tolerant entities,
  neutral-binary writing, a production-scale corpus, and external Parasolid
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
