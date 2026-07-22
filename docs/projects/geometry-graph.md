# F1 procedural geometry graph

Add a layer-1.5 geometry graph (`kgraph`) representing leaf and procedural
geometry as immutable, dependency-bearing nodes. Topology stores typed handles,
not copies of basis geometry. Evaluation of a graph handle is fallible, bounded,
deterministic, and independent of topology. The reference vertical slice is an X_T
`OFFSET_SURF`: import a constant signed offset over a surface node, attach to a
face, evaluate through the basis, and tessellate through pcurves — preserving
`OFFSET_SURF` class identity and failing explicitly at singular/unresolved regions.
This project owns the geometry dependency boundary only; it does not implement
every procedural class or become a second operation-policy project.

Status: G1–G4a plus the G5a operations-adapter slices landed (topology
geometry-ownership migration; offset descriptor/evaluator; X_T `OFFSET_SURF` slice;
F2 evaluation-budget adapter; certified intersection-curve and transmitted-chart/
M3c consumers across the planar and rational-quarter-cylinder families; operation-local exact-parallel/antiparallel strict-secant Cylinder/Cylinder rulings); broader
corpus coverage and further procedural-intersection/descriptor families remain.

## Contract — layer and crate placement

`kgraph` is a workspace crate at L1.5 between geometry and topology:

```text
kcore   L0    deterministic math, arenas, tolerances, base errors
kgeom   L1    independent analytic/NURBS/2D geometry values, leaf evaluators
kgraph  L1.5  immutable geometry nodes, handles, dependencies, evaluation
kops    L3    graph-aware dispatch and procedural/generic algorithms
ktopo   L2    topology holding graph handles; checked ownership integration
kxt     L5    X_T graph reconstruction and class-preserving emission
```

`kgraph` depends only on `kcore`/`kgeom`, never on `ktopo`/`kops`/`kxt`. `ktopo`
depends on `kgraph` (may still use `kgeom` value types); `kops` depends on
`kgraph` for entry points, not `ktopo`; `kxt` may import `kgraph` for
descriptors/class inspection. Procedural descriptors never contain topology
handles, so geometry is shared by many faces/bodies without a geometry→topology
cycle. The graph is not in `kgeom`, which stays pure value math with no session,
arena, handle, recursion guard, or work budget.

## Contract — kgraph types, evaluation, and offsets

**Ownership and public types.** `kgraph` owns three typed arenas (`curves`,
`surfaces`, `curves_2d`) plus a `ReverseDependencyIndex` in one `GeometryGraph`;
points stay in `ktopo::Store` (no procedural dependencies). Node fields are
private; descriptors are immutable after insertion (replace by inserting a new
node and transactionally retargeting consumers — no in-place mutation).

```rust
pub type CurveHandle = Handle<CurveNode>;   // also SurfaceHandle, Curve2dHandle

#[non_exhaustive]
pub enum CurveDescriptor {
    Line(Line), Circle(Circle), Ellipse(Ellipse), Nurbs(NurbsCurve),
    Intersection(IntersectionCurveDescriptor), // added after the offset slice
}
#[non_exhaustive]
pub enum SurfaceDescriptor {
    Plane(Plane), Cylinder(Cylinder), Cone(Cone), Sphere(Sphere),
    Torus(Torus), Nurbs(NurbsSurface), Offset(OffsetSurfaceDescriptor),
}
#[non_exhaustive]
pub enum Curve2dDescriptor { Line(Line2d), Circle(Circle2d), Nurbs(NurbsCurve2d) }

pub struct OffsetSurfaceDescriptor { basis: SurfaceHandle, signed_distance: f64 }
```

`signed_distance` is finite, in model meters, measured along the basis
evaluator's natural unit normal. A zero distance stays an offset node (class
identity must survive round-trip). The published X_T reference marks
`true_offset` unused and `scale` internal-only/nullable, so `kxt` ignores both,
converts the transmitted signed `offset` through the common surface sense to this
canonical definition on read, and emits an equivalent canonical form on write.
`IntersectionCurveDescriptor` is declared only once its verification contract is
implemented (shape: 3D carrier handle, two surface handles, two pcurve handles,
parameter maps, whole-interval residual certificate); uncertified descriptors are
not part of F1.

Topology keeps its vocabulary: `ktopo::entity::{CurveId, SurfaceId, Curve2dId}`
alias/re-export the `kgraph` handles; `ktopo::geom` may temporarily re-export
descriptor names but keeps no second owned enums. `ktopo::Store` embeds one
`GeometryGraph` with explicit access (`store.geometry().surface(id)?`, `.curve(id)?`,
`store.insert_surface(desc)?`, `store.eval_context(limits, tols)`); generic
`Store::get` remains for topology entities and points.

**Class identity.** Inspection must not rely on `Any`, Rust discriminants, or
debug strings. Provide a closed dispatch enum and stable external key:

```rust
#[non_exhaustive]
pub enum SurfaceClass { Plane, Cylinder, Cone, Sphere, Torus, Nurbs, Offset }
pub struct GeometryClassKey(&'static str);
impl SurfaceClass { pub const fn key(self) -> GeometryClassKey; }
```

Keys are namespaced, versioned strings (`kernel.surface.offset.v1`,
`kernel.curve.nurbs.v1`); enum layout is never a serialization contract. A
descriptor exposes `class()`/`class_key()`; leaf accessors (`as_plane()`,
`as_nurbs()`) are explicit and never expose `Any`. X_T emission switches on
`SurfaceClass`; unsupported class versions fail with a machine-readable
unsupported-capability result.

**Dependencies.** Every descriptor reports direct dependencies in deterministic
field order:

```rust
pub enum GeometryRef { Curve(CurveHandle), Surface(SurfaceHandle), Curve2d(Curve2dHandle) }
pub trait GeometryDependencies { fn visit_dependencies(&self, visit: &mut dyn FnMut(GeometryRef)); }
```

An offset's only dependency is `Surface(basis)`; leaves have none; intersection
curves visit carrier, surfaces A/B, then pcurves A/B in that order. Insertion
rules: (1) every referenced handle must already be live in the same graph;
(2) descriptors are immutable after insertion; (3) therefore ordinary insertion
is dependency-before-dependent and acyclic by construction; (4) a deterministic
reverse-dependency index updates atomically with each insertion/rollback, its
values in arena slot order never hash order; (5) public removal is deferred, and
internal removal/GC must reject a node with graph dependents or topology
consumers.

Native deserialization and X_T can carry forward references or corrupt cycles.
Builders build a transport-ID table then resolve with tri-color DFS; re-entering a
`Visiting` node returns `GeometryBuildError::DependencyCycle` with a deterministic
transport-ID path. The evaluator also keeps an active stack (defense in depth);
malformed data must never recurse to stack overflow. Deterministic utilities
expose direct deps, dependency-first transitive traversal (dupes removed),
dependents, reachability, and graph validation over live handles, cycles,
descriptor invariants, and reverse-index agreement.

**Fallible evaluation.** `kgeom` leaf protocols stay infallible; do not make them
graph-aware. `kgraph::EvalContext` is the only entry point for resolving handles
and evaluating nodes — `eval_curve`, `eval_surface` (with `SurfaceDerivativeOrder
{Position, First, Second}`), and `surface_bounds`, each returning an `EvalResult`.
Limits are query-local:

```rust
pub struct EvalLimits { pub max_dependency_depth: usize, pub max_node_visits_per_query: usize }

#[non_exhaustive]
pub enum EvalError {
    StaleGeometryHandle { geometry: GeometryRef }, InvalidParameter, ParameterOutsideDomain,
    DependencyCycle { path: Vec<GeometryRef> },
    DependencyDepthExceeded { consumed: usize, limit: usize },
    NodeVisitLimitExceeded { consumed: usize, limit: usize },
    SingularSurface { .. }, IllConditionedSurface { .. }, // both { surface, uv }
    DerivativeUnavailable { class: GeometryClassKey, requested: usize }, NonFiniteResult { class },
}
```

Each public query resets per-query work accounting; the graph is read-only during
evaluation (F1 adds no shared mutable cache). Evaluation validates finite
parameters/ranges, accounts one visit before descending, checks the active stack,
and checks results for finiteness and the model size box; it never silently clamps
invalid input and never collapses failures into `InvalidGeometry { reason }`.

**Boundary with F2.** `EvalLimits` owns only graph-recursion work (depth, node
visits) — never solver policy, cancellation, trace sink, proof budget, or session
facade; `EvalContext` takes the existing `Tolerances`. F2's `OperationContext`
owns broader solver limits and constructs a graph `EvalContext` per operation (F1
borrows a small policy view rather than duplicating fields).

**Offset evaluation.** For basis `s(u,v)`:

```text
w = s_u x s_v ;  q = |w| ;  n = w / q
w_u = s_uu x s_v + s_u x s_uv ;  w_v = s_uv x s_v + s_u x s_vv
n_u = (w_u - n (n·w_u)) / q ;  n_v = (w_v - n (n·w_v)) / q
offset position = s + d n ;  offset u = s_u + d n_u ;  offset v = s_v + d n_v
```

This gives exact position and first derivatives through second-order basis
derivatives; the offset inherits basis parameter range and periodicity. Basis
degeneracies remain degeneracies; extra offset singularities occur when the offset
Jacobian loses rank (an offset factor reaching a principal radius of curvature).
The node supports `Position` and `First`; `Second` returns `DerivativeUnavailable`
(exact second derivatives need third basis derivatives, beyond `SurfaceDerivs`) —
never zero or finite-difference values.

**Validity.** `surface_validity(surface, uv) -> EvalResult<SurfaceValidity>` with
`SurfaceValidity::{Regular { normalized_jacobian }, Singular, Indeterminate {
reason }}`. Exact-zero/non-finite basis or offset Jacobians are singular; a
normalized Jacobian at or below angular tolerance is ill-conditioned, never
guessed regular; a point is regular only after the offset Jacobian is checked; an
interval/region is certified regular only when a class-specific curvature/Jacobian
bound excludes zero over the whole region; unsupported global certification stays
indeterminate — samples are never global proof.

**Work boxes.** The unit-normal displacement over a finite basis rectangle has
length `abs(d)`, so `offset_box(range) = outward_inflate(basis_box(range),
abs(d))` via the existing outward-rounded AABB machinery. A stale basis,
non-finite distance/range, or unavailable basis bound is an error, not an empty
box.

## Contract — topology, operations, and serialization

**Topology.** `ktopo::Store` owns the `GeometryGraph` alongside topology/point
arenas. A transaction opens graph undo frames; commit/rollback covers all three
geometry arenas and the reverse-index, and a failed checked commit leaves graph
counts, handle validity, free-list order, and traversal exactly as at entry. Body
footprints are transitive (topology-used handles plus their full closure); a
mutation selects bodies whose footprints contain the node; deletion is rejected
while any footprint or graph node reaches it. Faces/edges/fins keep typed IDs;
`FaceDomain::natural` asks the graph for parameter metadata; incidence, checking,
and boundary tessellation evaluate through one borrowed `EvalContext`, not
`SurfaceGeom::as_surface()`; analytic proof accelerators inspect `SurfaceClass` and
borrow the exact leaf, returning an explicit proof gap for unknown/procedural
classes. Pcurve-driven tessellation evaluates every UV vertex through the graph,
checks validity, and returns `Indeterminate`/an error rather than silently
sampling across a singularity.

**Operations.** Keep leaf-specialized `kops` functions during migration; add
graph-aware entry points (e.g. `intersect_graph_surfaces(graph, a, a_range, b,
b_range, context)`). The dispatcher uses `SurfaceClass`: two supported leaf
classes borrow leaf values and call analytic accelerators; leaf/procedural and
procedural/procedural pairs use a generic evaluator path when it lands; lack of a
certified path is unsupported/indeterminate per the common intersection contract
— never `InvalidGeometry` and never a proven empty result. F1 only proves `kops`
can inspect/evaluate an offset, obtain its dependency closure, and preserve its
handle; implementing offset intersections belongs to F3/M4.

**X_T reconstruction** (`kxt::recon::surface`, dependency-aware). For
`OFFSET_SURF`: mark `Visiting`; resolve the basis recursively; accept `check` in
`{U,V}`, reject `I`, ignore `true_offset`/`scale`, validate a nonzero finite
`offset`, require offset and basis senses to agree; convert the signed
displacement to model meters along the basis natural normal (`offset` for basis
sense `+`, `-offset` for `-`); insert `SurfaceDescriptor::Offset`; mark `Complete`
and cache the handle. Only the constant normal-offset form enters the first slice;
invalid check status, mismatched senses, zero-resolution displacement, or
malformed fields return typed errors. Rules follow the published
[*Parasolid XT Format Reference, April 2008*](https://ww3.cad.de/foren/ubb/uploads/Rainer%2BSchulze/XT_Format_April_2008_tcm73-62642.pdf),
`OFFSET_SURF` section. A modern external Parasolid round-trip fixture still gates
any claim of emitting multiple offsets sharing one basis (the older reference
forbids it while the graph supports it internally). An X_T dependency cycle fails
with its deterministic node-index path and rolls back the import.

**X_T writing.** Walk the dependency closure from topology-attached handles and
assign X_T node IDs in stable dependency-first order; shared nodes emit once.
`SurfaceDescriptor::Offset` emits `OFFSET_SURF` referencing the planned basis.
Canonical field values are acceptable; semantic class-preserving round-trip is
required, byte-for-byte retention is not. Planning must not depend on `HashMap`
iteration; duplicate suppression uses handles but never determines output order.

**Native format.** Do not serialize arena indexes/generations as durable identity.
A future native format uses document-local IDs (document id, class key, descriptor
schema version, dependency document ids, class payload), written dependency-first
and rebuilt through the same cycle-checking builder as X_T; unknown class keys or
unsupported schema versions are typed capability failures.

**Committed intersection carriers.** Certified intersection branches may be
committed atomically as `CurveDescriptor::{Intersection, VerifiedNurbsIntersection,
TransmittedIntersection, TransmittedNurbsIntersection}` nodes, each with a stable
class key, ordered source-surface and pcurve dependencies, a finite carrier
interval, and a paired whole-interval residual certificate. Graph validation
recomputes the certified field and rejects any mismatch before allocation; reverse
dependencies protect every transitive basis (including offset chains) while the
proof is live, and stale/altered sources roll the persistence batch back atomically.
Exact-parallel/antiparallel strict-secant Cylinder/Cylinder rulings remain operation-local: their graph proof yields exactly two deterministic branches, tangent/miss/coincident/skew are typed gaps, and persistence refuses unsupported analytic cylinder families.

## Explicit non-goals

- Making leaf `kgeom` traits return `Result`; general surface third derivatives in
  the offset slice; certifying every offset face globally singularity-free.
- Graph memoization/eviction, parallel scheduling, or GPU evaluation; a
  plugin/custom-geometry ABI or dynamic class registry; a durable native file
  format implementation (only its identity rules are fixed).
- Recomputing general operational Offset/Offset or Offset/NURBS intersections, or
  imported intersection curves (verified import of supported transmitted scars is
  in scope); sweeps, spun surfaces, rolling-ball blends/bounds, or foreign
  geometry; moving topological points into the graph.
- Freezing the public `Kernel`/`Session` facade; duplicating F2's solver policy,
  cancellation, diagnostics, or work budgets.

## Evidence

- `crates/kgraph/tests/*.rs`: graph_contract, leaf_parity, offset_surface,
  intersection_curve_certificate, transmitted_plane_offset_nurbs
- `crates/kxt/tests/*.rs`: offset_surface, import_tess, write, read,
  corpus_manifest, intersection_chart, offset_nurbs_intersection,
  equal_limit_intersection, terminated_intersection, periodic_nurbs,
  plane_sp_curve, zero_multiplicity_knot_padding, finite_open_*
- `crates/kops/tests/graph_surface*.rs` (plane/sphere/NURBS, varying-offset arms); `crates/kops/tests/graph_cylinder_cylinder_rulings.rs` (parallel/antiparallel rulings and typed gaps)
- `crates/ktopo/tests/`: assembly_boundary, transactions, body_copy

## Open items

- Broaden the capped four-descriptor Offset(NURBS)/NURBS unit-chart arm beyond
  positive-area finite-window overlap; add broader NURBS/NURBS and other
  exact/procedural families only with contextual accounting and paired trace
  evidence. Add swept, spun, and blend descriptors only with their own evaluator,
  validity, bounds, interchange, and test contracts.
- Broaden the M3c consumer to null/mixed/non-`H`/broader closed limits, other
  nullable chart data, ambiguous/multi-period aliases, noncanonical charts outside
  the bounded affine slices, and further terminator variants — without recomputing
  transmitted scars.
- Exit: F3/M4 can add a procedural fallback without changing topology handles or
  the graph ownership model.
- **X_T shared offset bases.** Require a modern Parasolid oracle before claiming
  shared-basis writer conformance.
- **Third derivatives.** Defer the surface-jet API until an M4 SSI/curvature
  consumer owns the contract. **Global regularity proof.** Bounding principal
  curvature over arbitrary NURBS regions is a later proof extension.
