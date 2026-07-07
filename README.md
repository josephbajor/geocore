# cad_prototype

An open, performant parametric B-rep modeling kernel, built for interoperability
with Parasolid-based CAD systems (SolidWorks, Solid Edge, NX, Onshape) via XT
round-trip — the foundation of an eventual full parametric CAD application.

- **Specification:** [docs/kernel-spec.md](docs/kernel-spec.md)
- **Construction roadmap:** [docs/kernel-roadmap.md](docs/kernel-roadmap.md)

## Layout

| Crate | Layer | Contents |
|---|---|---|
| [`crates/kcore`](crates/kcore) | L0 foundations | Robust predicates, exact expansion arithmetic, interval filters, tolerance policy (Parasolid numeric regime), typed errors, generational entity arenas, deterministic parallel primitives, deterministic transcendental math (musl port — platform libm is banned in kernel code via clippy `disallowed-methods`) |
| [`crates/kgeom`](crates/kgeom) | L1 geometry | Analytic curves (line/circle/ellipse) and surfaces (plane/cylinder/cone/sphere/torus) with exact bounding boxes, NURBS engine (Piegl & Tiller), closest-point projection, trimmed-face tessellation, evaluator conformance harness |

Upcoming per the roadmap: `ktopo` (B-rep topology, Euler ops, checker), `kops`
(modeling operations), `kxt` (XT interchange), `kapi` (PK-style C API).

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
