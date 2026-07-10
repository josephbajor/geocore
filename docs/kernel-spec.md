# Geometric Modeling Kernel — Specification

Status: draft v0.1 (2026-07-06)

This document defines the target contract. Implemented status and known
limitations are tracked in [kernel-roadmap.md](kernel-roadmap.md); a roadmap
milestone is not conformant until its stated exit criterion is met.

## 1. Mission and scope

Build an open, performant boundary-representation (B-rep) geometric modeling kernel
whose data model, numeric regime, and exchange format are compatible with Parasolid,
so that models round-trip cleanly with Parasolid-based systems (SolidWorks, Solid Edge,
NX, Onshape) via XT (`.x_t` / `.x_b`) files.

The kernel is the bottom layer of a larger parametric CAD system:

```
┌─────────────────────────────────────────────┐
│  Application / UI                            │
├─────────────────────────────────────────────┤
│  Parametric feature framework                │   (feature tree, persistent naming,
│                                              │    regeneration — later workstream)
├──────────────────────┬──────────────────────┤
│  Constraint solver   │  THIS KERNEL          │   (solver ≈ D-Cubed DCM — deferred)
│  (2D/3D DCM-like)    │  (geometry + topology │
│                      │   + modeling ops +    │
│                      │   XT interchange)     │
└──────────────────────┴──────────────────────┘
```

In scope for the kernel:
- Exact analytic + NURBS + procedural geometry
- Parasolid-compatible B-rep topology with tolerant entities
- Modeling operations: primitives, sweeps, booleans, offsets, blends, shelling
- Interrogation: classification, mass properties, clash, tessellation
- XT read/write, STEP AP242 as secondary format
- A stable C API in the style of Parasolid's PK interface

Out of scope (separate workstreams): sketching/constraints, feature tree, drawings,
assemblies-as-product-structure (XT assembly *entities* are in scope for I/O fidelity).

## 2. Compatibility contract with Parasolid

Interoperability is defined by three commitments, not by cloning Parasolid internals:

1. **Data-model compatibility.** Our entity classes, topology hierarchy, and geometry
   types map 1:1 onto the published XT schema. Nothing we author is inexpressible in XT.
2. **Numeric-regime compatibility.** SI meters internally; working size box of
   1000 m (coordinates within ±500 m of origin); linear resolution 1e-8 m; angular
   resolution 1e-11 rad. Tolerant edges/vertices carry validated per-entity tolerances ≥
   resolution together with their import/operation origin and accumulated growth,
   exactly as Parasolid's tolerant modeling requires operationally. Geometry we export must satisfy
   Parasolid's checker (verified empirically via Solid Edge import).
3. **Exactness commitment.** Analytic surfaces (plane, cylinder, cone, sphere, torus)
   and procedural surfaces (swept, spun, offset, rolling-ball blend) are represented
   *exactly*, never approximated as NURBS on export. NURBS-ification destroys downstream
   editability and is the classic interop failure mode.

### XT conformance tiers (acceptance criteria per tier)

- **Tier 0 — Read/visualize:** parse any well-formed XT (text + neutral binary),
  build the B-rep, tessellate and render it. No write.
- **Tier 1 — Author/round-trip analytic:** bodies we create using analytic geometry
  export to XT and import into Solid Edge/SolidWorks with zero checker errors, and
  re-import into us bit-faithfully (modulo entity ids).
- **Tier 2 — Full geometry:** B-curves/B-surfaces, SP-curves, intersection curves,
  swept/spun/offset/blend surfaces round-trip.
- **Tier 3 — Parity:** attributes, attribute definitions, assemblies/instances,
  tolerant entities, and mixed-dimension (general) bodies round-trip.

## 3. Architecture: layers

Strict layering; each layer depends only on layers below it.

### L0 — Foundations
- Deterministic scalar math (`f64` everywhere; no fast-math; identical results across
  platforms and thread counts).
- Robust geometric predicates: Shewchuk-style adaptive-precision orient2d/orient3d/
  incircle, plus interval arithmetic for filters. All classification decisions route
  through predicates, never raw float comparisons.
- Tolerance policy object (session precision constants above) threaded through all ops.
- Error model: every public operation returns rich, typed errors (mirroring PK error
  codes) — no panics/exceptions across the API boundary.
- Arena/handle entity storage: entities are integer ids into typed arenas; geometry is
  immutable and shareable; topology references geometry by id. Enables cheap copy,
  undo/rollback (partition-like snapshots), journaling, and parallelism.

### L1 — Geometry
Curve classes (matching XT): line, circle, ellipse, B-curve (NURBS, rational and
polynomial), intersection curve (procedural, lazily evaluated against its two surfaces),
SP-curve (curve in a surface's parameter space), trimmed curve, degenerate curve.

Surface classes (matching XT): plane, cylinder, cone, sphere, torus, B-surface (NURBS),
swept surface, spun surface, offset surface, rolling-ball blend surface (`blended_edge`),
blend bound.

Uniform evaluator protocol for every curve/surface: position, 1st/2nd/3rd derivatives,
parameter bounds and periodicity, singularity/degeneracy description, curvature,
closest-point projection, and bounding volume (exact for analytics, convex-hull-based
for NURBS).

NURBS engine per Piegl & Tiller: knot insertion/removal, degree elevation, splitting,
Bezier extraction, interpolation/approximation fitting, derivative surfaces. This is
pure, heavily unit-tested math with no topology dependencies.

Intersection suite (the kernel's technical core):
- curve/curve, curve/surface: subdivision + Newton polishing, interval-filtered.
- surface/surface (SSI): hybrid of analytic special-casing (quadric/quadric pairs have
  closed forms), marching with adaptive step control, and subdivision fallback for
  start-point discovery; explicit handling of tangent contacts, singular points, and
  small loops. Results are intersection-curve geometry usable directly as edge geometry.
Every bounded intersection result carries explicit completion evidence. `Complete` means
the full requested domains were covered and therefore permits an empty result to prove a
miss; `Indeterminate` may still carry individually verified discoveries but can never be
silently interpreted as complete.

### L2 — Topology
Parasolid's exact hierarchy:

```
BODY → REGION (solid|void) → SHELL → FACE → LOOP → FIN (half-edge) → EDGE → VERTEX
```

Body types: solid, sheet, wire, acorn (minimal), general (mixed-dimension).
Faces reference surfaces with a sense flag, an optional finite conservative UV work
domain, and optional operation/import tolerance metadata; fins reference edges with
sense and may carry an independent pcurve use plus explicit integer-period chart shift,
paired lower/upper seam role, closed-use winding, and endpoint singularity markers; edges
reference curves; vertices reference points. Tolerant edges/vertices store a typed
tolerance overriding session precision, retaining its original value/source and every
budgeted enlargement in the transaction journal. An unknown face domain stays explicit
rather than being replaced by an uncertified sampled bound.

- Euler operators as the only structural mutation primitives (MEV, MEF, KEMR, etc.),
  each preserving the Euler–Poincaré invariant; raw operators are topology-internal and
  all higher ops compose transaction-owned methods with mandatory pcurve-bearing
  creation, rollback, checked result commit, and semantic lineage.
- Generic topology insertion, mutable borrowing, removal, and unchecked commit are not
  public Store operations. Interchange reconstruction and specialized graph builders use
  a transaction-scoped assembly facade whose changes can survive only a checked commit.
  Deterministic pending mutations are resolved through committed and candidate topology
  ownership/shared-geometry dependency indexes; every affected body is checked and every
  commit audits global ownership closure. Candidate indexes replace deterministic
  per-body footprints for affected roots and retain full reconstruction as a debug/audit
  oracle.
- **Checker** (our `PK_BODY_check` equivalent): validates topology (closure, manifold
  conditions per body type, loop orientation), geometry (self-intersection, degeneracy),
  and geometry–topology consistency (face-loop containment, edge-on-surface within
  tolerance). `Fast` checking may use bounded deterministic approximations; `Full`
  checking returns `Valid`, `Invalid`, or `Indeterminate` and must identify every
  unresolved proof obligation rather than treating a clean sample as proof. It runs in
  CI after every modeling op on the test corpus; nothing ships that emits
  checker-failing bodies, and conformance claims require a `Full` `Valid` result.
  Face-domain containment follows the same contract: conservative active-subrange boxes
  and adaptive subdivision may prove a complete pcurve interval inside, evaluated points
  may witness an exterior fault, and unsupported representations or exhausted proof
  limits remain `Indeterminate`.

### L3 — Modeling operations
Ordered by dependency, roughly matching build order in the roadmap:
- Primitives: block, cylinder, cone, sphere, torus, prism; sheet/wire body constructors.
- Profile ops: planar wire profile → extrude, revolve; then sweep along curve, loft.
- **Booleans** (unite/subtract/intersect, plus sheet-splits-solid): intersect all face
  pairs → imprint intersection edges → split faces → classify fragments (in/out/on via
  point classification with robust predicates) → assemble result shells → tolerant-stitch
  seams. Staged rollout: analytic-only first, NURBS later. This op defines the kernel's
  reputation; see robustness strategy §5.
- Local ops: face offset, body offset/hollow (shell), face taper, tweak (replace face
  surface), delete-face-and-heal.
- Blending: constant-radius rolling-ball edge blends first, then variable radius,
  setbacks, and corner patches (hardest sustained effort in the kernel; own milestone).
- Sewing: tolerant stitching of sheet bodies into solids (also the STEP-import workhorse).

Every op supports: atomic failure (input bodies untouched on error), journaling of
created/deleted/split entities (required later for persistent naming in the parametric
layer), and attribute propagation rules.

### L4 — Interrogation & analysis
- Point-in-body / point-on-face classification.
- Mass properties (volume, area, centroid, inertia) via divergence-theorem integration
  with certified error bounds.
- Bounding boxes/hierarchies, minimum distance, clash detection.
- Tessellation: watertight, crack-free faceting with chordal + angular tolerance control,
  per-face parallel, incremental re-tessellation of changed faces (feeds rendering, STL,
  and mesh-based downstream consumers). If configured refinement or mesh-size limits
  prevent satisfying the requested tolerance, return a typed error rather than silently
  emitting an under-refined mesh.
- Ray firing, silhouette computation (needed later for drawings).

### L5 — Interchange
- **XT reader/writer**: text (`.x_t`) and neutral-binary (`.x_b`) per the published
  Parasolid XT Format Reference, including schema-version handling of older files.
  The reader lands *early* (Tier 0) because the world's supply of real Parasolid models
  becomes our test corpus.
- STEP AP242 read/write as second-class citizen (via sewing).
- STL/3MF/glTF export from tessellation.

### L6 — API surface
- **C API modeled on the PK interface**: opaque integer entity handles, `pk_`-style
  namespacing, out-parameters, typed error returns, option structs with versioned
  initializers. This is the stable ABI for bindings (Python for testing, app language
  for the product) and eases the mental mapping for engineers who know Parasolid.
- Native-language API underneath for internal/product use.
- Session model: partitions (independent undo scopes), roll-back marks, entity id
  stability guarantees — the hooks the parametric layer will need.

## 4. Non-functional requirements

- **Determinism:** same input → bit-identical output regardless of platform, thread
  count, or run count. Non-negotiable; parametric regeneration and testing depend on it.
- **Performance targets (v1):** tessellate a 5k-face body < 250 ms on 8 cores; boolean
  of two 1k-face analytic bodies < 100 ms; XT read of a 50 MB assembly < 2 s. Parallelism
  in tessellation, face-pair intersection, and independent-body ops; single-threaded
  determinism preserved by deterministic reduction ordering.
- **Robustness budget:** on the differential-test corpus (§5), boolean success rate is a
  tracked release metric with a ratchet — it may never regress.
- **Memory:** streaming XT parse; arenas sized for 1M+ topological entities per body.

## 5. Robustness strategy (the actual hard part)

Kernels fail on tolerances and intersections, not on data structures. Policy:

1. Robust predicates + interval filters for every sign decision; adaptive exact
   arithmetic as fallback. No epsilon-tuning scattered through op code — tolerance
   decisions live in one policy module.
2. Tolerant modeling as a first-class concept from day one (not retrofitted): edges and
   vertices may carry tolerances; ops must accept and produce tolerant entities exactly
   as Parasolid does, or real-world imported geometry will be unusable. Every operation
   that enlarges tolerance declares an aggregate transaction-owned budget; exhaustion is
   a typed failure, and rollback discards both model changes and recorded consumption.
3. Spatial broad phases are deterministic and conservative: finite primitives enter
   balanced AABB hierarchies, tolerance padding rounds outward, and a candidate is
   excluded only by control-hull, interval, or exact-predicate evidence covering its
   complete domain. A clean sample is never a proven miss.
4. Differential testing: every modeling op runs against oracles — Open CASCADE (open
   source) and actual Parasolid via Solid Edge Community Edition batch import/export —
   over a growing corpus of real XT files. Disagreements are triaged into bugs or
   documented semantic differences.
5. Property-based testing and fuzzing: Euler invariants after every Euler-op sequence;
   volume conservation across boolean partitions (vol(A) = vol(A∩B) + vol(A−B) within
   certified bounds); checker-clean outputs; XT fuzzing on the parser.
6. Every fixed bug becomes a corpus file. The corpus is the kernel's real asset.

## 6. Key design decisions (proposed)

| Decision | Choice | Rationale |
|---|---|---|
| Language | Rust (C ABI exports) | Memory safety in a 500k-LOC-class codebase, fearless parallelism for tessellation/booleans, first-class fuzzing/property-test tooling. C++ remains viable if we want CGAL/OCCT linkage; revisit before M1 ends. |
| License | Apache-2.0 or MPL-2.0 | "Open kernel" mission; avoids OCCT's LGPL friction for embedders. |
| Internal representation | Exact analytics + procedurals, never NURBS-ify | Interop fidelity (§2.3). |
| Storage | Arena + integer handles, immutable shared geometry | Cheap rollback, journaling, parallelism, PK-style handles for free. |
| XT legal posture | Clean-room from the published XT Format Reference only | Siemens publishes the spec explicitly for interoperability; we never disassemble Parasolid. |
| Oracle stack | Solid Edge CE (Parasolid ground truth) + OCCT (open oracle) | Empirical compatibility, not aspirational. |

## 7. Explicitly deferred

- Geometric constraint solver (D-Cubed DCM analog) — separate kernel, separate spec;
  the modeling kernel only promises the entity-id + journaling hooks it will need.
- Feature tree / parametric regeneration / persistent naming — consumes the journaling
  hooks; lives above the kernel.
- Facet/hybrid modeling, convergent modeling, sheet-metal, drawings.
