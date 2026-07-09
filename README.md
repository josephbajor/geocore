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
| [`crates/kgeom`](crates/kgeom) | L1 geometry | Analytic curves (line/circle/ellipse) and surfaces (plane/cylinder/cone/sphere/torus) with exact bounding boxes, NURBS engine (Piegl & Tiller), closest-point projection, deterministic trimmed-face tessellation with explicit refinement-limit errors, evaluator conformance harness |
| [`crates/ktopo`](crates/ktopo) | L2 topology | Parasolid entity hierarchy (body→region→shell→face→loop→fin→edge→vertex) over generational arenas, Euler operators, primitive body constructors, the body checker (structural + geometric invariants), whole-body watertight tessellation |
| [`crates/kops`](crates/kops) | L3 operations | M4 intersection foundation: parameter-rich curve/curve results plus deterministic bounded line/line, 3D line/circle, 3D line/ellipse, 3D circle/circle, 3D circle/ellipse, and 3D ellipse/ellipse intersections behind a general analytic dispatcher |
| [`crates/kxt`](crates/kxt) | L5 interchange | Atomic modern-schema Parasolid XT (`.x_t`/`.x_b`) import for the supported geometry subset, plus a deterministic schema-13006 text writer for self-authored analytic solids (clean-room from the published XT Format Reference) |

## Current Status

- M0 foundations, M1 geometry, and M2 topology/primitives are complete for their
  documented scopes.
- M3 is in progress: modern base-13006 schema edit scripts, text/neutral-binary
  reading, atomic reconstruction, and analytic text writing are implemented.
  Pre-13006 schemas, assemblies, procedural/SP geometry, tolerant entities,
  neutral-binary writing, and external Solid Edge round-trip certification remain.
- M4 has started in `kops` with bounded line/line, 3D line/circle, 3D
  line/ellipse, 3D circle/circle, 3D circle/ellipse, and 3D ellipse/ellipse
  intersections behind a general analytic dispatcher. Curve/surface has begun
  with bounded line/plane, line/cylinder, line/cone, line/sphere, line/torus,
  circle/plane, ellipse/plane, circle/cylinder, circle/cone, circle/sphere, and
  circle/torus plus ellipse/sphere, ellipse/cylinder, ellipse/cone, and
  ellipse/torus. General NURBS/procedural curve/curve cases, broader
  curve/surface, surface/surface intersections, and imprinting remain.

Immediate work per the roadmap: broaden analytic curve/curve intersections,
then curve/surface and SSI; complete M3b external XT validation in parallel.

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
