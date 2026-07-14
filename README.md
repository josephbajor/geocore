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
| [`crates/kcore`](crates/kcore) | L0 foundations | Robust predicates, exact expansion arithmetic, interval filters, tolerance policy (Parasolid numeric regime), typed errors, generational entity arenas with copy-on-write undo frames, deterministic parallel primitives, deterministic transcendental math (musl port — platform libm is banned in kernel code via clippy `disallowed-methods`) |
| [`crates/kgeom`](crates/kgeom) | L1 geometry | Analytic curves (line/circle/ellipse), true 2D line/circle/NURBS pcurve evaluators, and analytic surfaces (plane/cylinder/cone/sphere/torus) with exact bounding boxes, NURBS engine (Piegl & Tiller) with homogeneous 2D/3D knot operations and conservative active-subrange control-hull boxes, closest-point projection, deterministic trimmed-face tessellation with explicit refinement-limit errors, evaluator conformance harness |
| [`crates/kgraph`](crates/kgraph) | L1.5 geometry graph | Immutable analytic, NURBS, and procedural geometry nodes with typed dependencies, deterministic identity, and bounded evaluation |
| [`crates/ktopo`](crates/ktopo) | L2 topology | Parasolid entity hierarchy (body→region→shell→face→loop→fin→edge→vertex), finite conservative face UV domains, typed entity-tolerance provenance and transaction-owned growth budgets, independent per-fin pcurves, bounded curve-less tolerant edges, reusable validated simple-polygon profiles, transaction-owned pcurve-aware Euler edits, private generic Store mutation with transaction-scoped checked assembly, deterministic mutation/lineage/tolerance journals, journal-returning checked solid/sheet/wire/acorn constructors, shared incidence validation, and pcurve-driven watertight tessellation |
| [`crates/kops`](crates/kops) | L3 operations | Provisional M4 intersection foundation: exact analytic special cases plus early sampled NURBS curve/curve, curve/surface, and surface/surface experiments; generic completeness and boolean-ready pcurve results remain gated |
| [`crates/kxt`](crates/kxt) | L5 interchange | Atomic modern-schema Parasolid XT (`.x_t`/`.x_b`) import for the supported geometry subset, including zero-multiplicity null-knot normalization and proof-bearing transmitted intersection charts through the bounded three-sample quadratic and four-sample cubic dual-offset NURBS slices, plus a deterministic schema-13006 text writer for self-authored analytic solids, sheets, wires, acorns, and bounded tolerant edges encoded as trimmed SP-curves over 2D B-curves (clean-room from the published XT Format Reference) |
| [`crates/kernel`](crates/kernel) | Supported native facade | Session/part lifecycle, opaque semantic IDs and views, contextual construction/check/evaluation/tessellation, and typed X_T import/export without raw topology or graph leakage |
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

- M0 foundations, M1 geometry, and M2 topology/primitives have implemented alpha
  slices; full conformance remains ahead.
- M2.5 is in progress and remains the architecture gate. Transaction-owned checked
  topology, pcurve-aware Euler edits, deterministic journals, tolerance provenance,
  bounded operation contexts, `Fast`/`Full` checking, adaptive face-domain proofs, and
  the immutable geometry graph with persistent verified intersection descriptors have
  landed. General NURBS/mixed-parameter incidence, periodic and unsupported mixed-boundary
  containment, curved-loop/shell proofs, operation-specific tolerance rules, and
  production-scale ownership/dependency benchmarks remain.
- M3 X_T interchange is in progress. Modern text and neutral-binary reading, atomic
  reconstruction, deterministic analytic text writing, bounded tolerant-edge SP-curves,
  safe offset surfaces, certified clamped periodic/closed B-surfaces, and canonical
  transmitted Plane/Offset/NURBS intersection-chart slices are implemented. The committed
  corpus includes a production 7,423-node Onshape part; its first direct
  `Offset(B-surface)/B-surface` chart now certifies and the ratchet has advanced to equal
  intersection limits. Broader cyclic B-geometry, periodic/circular pcurves, remaining
  intersection/procedural families, assemblies, older schemas, neutral-binary writing,
  and broader external Parasolid certification remain.
- M4 intersections are useful but provisional. Shared `Complete`/`Indeterminate`
  evidence, source-provenanced adaptive NURBS covers, interval implicit exclusion,
  Work-bounded polishing, exact algebraic seed/overlap certificates, selected paired
  pcurves, and bounded coincident Plane/Cylinder/Sphere/Cone/Torus regions have landed.
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
cargo test          # all unit + determinism tests
cargo clippy --all-targets -- -D warnings
```

Requires stable Rust (1.93+). The workspace is dependency-free by policy at L0.

## Determinism contract

Same input → bit-identical output on every platform, thread count, and run.
CI enforces this with golden-hash tests across Linux/macOS/Windows in both
debug and release. Changing a golden value is a reviewed, intentional event.
