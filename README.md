# cad_prototype

An open, performant B-rep modeling kernel built for interoperability with
Parasolid-based CAD systems (SolidWorks, Solid Edge, NX, Onshape) via XT
round-trip. It is the geometry and topology foundation for an eventual full
parametric CAD application; feature history and regeneration are later layers.

- **Standing rules for all contributors and agents:** [ORCHESTRATION.md](ORCHESTRATION.md)
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
| [`crates/kcore`](crates/kcore) | L0 foundations | Deterministic exact geometric predicates, tolerance policy, typed errors, generational copy-on-write entity arenas, and deterministic parallel and transcendental math (platform libm banned). |
| [`crates/kgeom`](crates/kgeom) | L1 geometry | Analytic curves and surfaces, a Piegl & Tiller NURBS engine with conservative bounds, closest-point projection, and deterministic trimmed-face tessellation with exact trim-loop winding. |
| [`crates/kgraph`](crates/kgraph) | L1.5 geometry graph | Immutable analytic, NURBS, and procedural geometry nodes with typed dependencies, deterministic identity, and bounded evaluation. |
| [`crates/ktopo`](crates/ktopo) | L2 topology | Parasolid entity hierarchy with independent pcurves, tolerant edges, transaction-owned checked Euler edits, deterministic journals, and pcurve-driven watertight tessellation. |
| [`crates/kops`](crates/kops) | L3 operations | Provisional M4 intersection foundation: exact analytic special cases plus early sampled NURBS curve/curve, curve/surface, and surface/surface experiments; boolean-ready results remain gated. |
| [`crates/kxt`](crates/kxt) | L5 interchange | Atomic Parasolid XT (`.x_t`/`.x_b`) import for the supported subset plus a deterministic schema-13006 text writer for self-authored analytic solids, sheets, wires, and acorns. |
| [`crates/kernel`](crates/kernel) | Supported native facade | Session/part lifecycle, opaque semantic IDs and views, block and profile-extrusion construction, rigid copy, checking, tessellation, and typed X_T import/export without raw topology leakage. |
| [`examples/kernel-lifecycle`](examples/kernel-lifecycle) | Facade-only client | Executable application lifecycle with `kernel` as its only direct kernel dependency. |

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
