# F1 procedural geometry graph

Status: G1-G4a and the F2 evaluation-budget adapter implemented; broader corpus coverage and G5 remain

## Outcome

Add a layer-1.5 geometry graph that represents leaf and procedural geometry as
immutable, dependency-bearing nodes. Topology stores typed handles to those
nodes; it does not own copies of basis geometry. Evaluation of a graph handle is
fallible, bounded, deterministic, and independent of topology.

The first vertical slice is an X_T `OFFSET_SURF`: import one constant signed
offset whose basis is an existing surface node, attach it to a face, evaluate it
through the basis surface, and tessellate the face through its pcurves. The slice
must preserve `OFFSET_SURF` class identity and must fail explicitly at singular
or numerically unresolved regions.

This project implements the geometry dependency boundary. It does not try to
implement every procedural class or become a second operation-policy project.

## Why this boundary is needed now

Today `kgeom::Curve` and `kgeom::Surface` are exact, context-free, infallible
leaf evaluators. `ktopo::geom::{CurveGeom, SurfaceGeom}` are closed owned enums,
and `ktopo::Store` keeps those values in topology-owned arenas. This is a good
shape for analytics and NURBS, but an offset surface must refer to a basis
surface, and an intersection curve ultimately refers to two surfaces plus its
paired parameter-space geometry. Storing those dependencies by value would:

- duplicate potentially large NURBS objects;
- make shared identity and X_T class-preserving output ambiguous;
- permit accidental recursive ownership;
- force procedural failure modes into the currently infallible leaf traits;
- hide transitive geometry dependencies from checked topology commits.

The graph makes dependency and identity explicit without weakening the existing
leaf math contracts.

## Layer and crate placement

Create a workspace crate named `kgraph` between pure geometry and topology:

```text
kcore       L0 deterministic math, arenas, tolerance primitives, base errors
  |
kgeom       L1 independent analytic/NURBS/2D geometry values and leaf evaluators
  |
kgraph      L1.5 immutable geometry nodes, handles, dependencies, evaluation
  | \
  |  kops   L3 graph-aware dispatch and procedural/generic algorithms
  |
ktopo       L2 topology holding graph handles; checked ownership integration
  |
kxt         L5 X_T graph reconstruction and class-preserving emission
```

Dependency rules:

- `kgraph` depends only on `kcore` and `kgeom`.
- `kgraph` must not depend on `ktopo`, `kops`, or `kxt`.
- `ktopo` depends on `kgraph` and may continue to use `kgeom` value types such
  as points, parameter ranges, and vectors.
- `kops` depends on `kgraph` for graph-aware entry points but not on `ktopo`.
- `kxt` depends on `kgraph` through its existing `ktopo`/reconstruction role
  and may import `kgraph` directly for descriptors and class inspection.
- Procedural descriptors never contain topology handles. Geometry can therefore
  be shared by many faces/bodies without a geometry-to-topology cycle.

Do not put the graph in `kgeom`. `kgeom` remains pure value math that can be
tested and reused without a session, arena, handle, recursion guard, or work
budget.

## Ownership and public types

`kgraph` owns three typed arenas. Points remain in `ktopo::Store` for this
project because they have no procedural dependencies. Moving points later is a
storage/API decision, not a prerequisite for procedural geometry.

```rust
pub type CurveHandle = Handle<CurveNode>;
pub type SurfaceHandle = Handle<SurfaceNode>;
pub type Curve2dHandle = Handle<Curve2dNode>;

pub struct GeometryGraph {
    curves: Arena<CurveNode>,
    surfaces: Arena<SurfaceNode>,
    curves_2d: Arena<Curve2dNode>,
    reverse_dependencies: ReverseDependencyIndex,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CurveNode {
    descriptor: CurveDescriptor,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceNode {
    descriptor: SurfaceDescriptor,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Curve2dNode {
    descriptor: Curve2dDescriptor,
}
```

Node fields are private. Descriptors are immutable after insertion. Replacing
geometry means inserting a new node and transactionally retargeting consumers;
there is no public in-place descriptor mutation.

The initial descriptors are:

```rust
#[non_exhaustive]
pub enum CurveDescriptor {
    Line(Line),
    Circle(Circle),
    Ellipse(Ellipse),
    Nurbs(NurbsCurve),
    // Added after the offset slice:
    Intersection(IntersectionCurveDescriptor),
}

#[non_exhaustive]
pub enum SurfaceDescriptor {
    Plane(Plane),
    Cylinder(Cylinder),
    Cone(Cone),
    Sphere(Sphere),
    Torus(Torus),
    Nurbs(NurbsSurface),
    Offset(OffsetSurfaceDescriptor),
}

#[non_exhaustive]
pub enum Curve2dDescriptor {
    Line(Line2d),
    Circle(Circle2d),
    Nurbs(NurbsCurve2d),
}

pub struct OffsetSurfaceDescriptor {
    basis: SurfaceHandle,
    signed_distance: f64,
}
```

`signed_distance` is finite, expressed in model meters, and measured along the
basis evaluator's natural unit normal. A zero distance remains an offset node;
it is not canonicalized to its basis because class identity must survive
round-trip. The published April 2008 X_T reference defines `true_offset` as
unused and `scale` as internal-only and nullable. `kxt` therefore ignores those
two fields, converts the transmitted signed `offset` through the common surface
sense to this canonical definition on read, and emits an equivalent canonical
form on write.

`IntersectionCurveDescriptor` is declared only when its verification contract
is implemented. Its intended shape is a handle to the transmitted/generated 3D
carrier curve, two surface handles, two pcurve handles, parameter maps, and a
whole-interval residual certificate. Declaring that dependency shape early is
useful; accepting uncertified intersection descriptors is not part of F1.

### Compatibility names in topology

`ktopo::entity::{CurveId, SurfaceId, Curve2dId}` become aliases or re-exports of
the `kgraph` handles so topology call sites keep their conceptual vocabulary.
`ktopo::geom` temporarily re-exports descriptor names for source migration, but
must not retain a second set of owned enums.

`ktopo::Store` embeds one `GeometryGraph`. Geometry access is explicit:

```rust
store.geometry().surface(id)?
store.geometry().curve(id)?
store.eval_context(limits, tolerances)
store.insert_surface(descriptor)?
```

The generic topology `Store::get` remains for topology entities and points.
Geometry-specific accessors replace relying on the topology `Entity` trait to
reach an arena owned by another crate.

## Class identity

Class inspection must not rely on `Any`, Rust discriminant values, or debug
strings. Provide a closed internal dispatch enum and a stable external key:

```rust
#[non_exhaustive]
pub enum SurfaceClass {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Nurbs,
    Offset,
}

pub struct GeometryClassKey(&'static str);

impl SurfaceClass {
    pub const fn key(self) -> GeometryClassKey;
}
```

Initial stable keys use namespaced, versioned strings such as
`kernel.surface.offset.v1` and `kernel.curve.nurbs.v1`. Rust enum layout is not
a serialization contract. A descriptor provides `class()` and `class_key()`;
leaf accessors such as `as_plane()` and `as_nurbs()` are explicit and do not
expose `Any`.

X_T emission switches on `SurfaceClass`/the descriptor and must write an offset
as `OFFSET_SURF`, never as a sampled or fitted B-surface. Unsupported class
versions fail with a machine-readable unsupported-capability result.

## Dependency contract

Every descriptor reports direct dependencies in deterministic field order:

```rust
pub enum GeometryRef {
    Curve(CurveHandle),
    Surface(SurfaceHandle),
    Curve2d(Curve2dHandle),
}

pub trait GeometryDependencies {
    fn visit_dependencies(&self, visit: &mut dyn FnMut(GeometryRef));
}
```

For an offset the only dependency is `Surface(basis)`. Leaves have none.
Intersection curves later visit carrier, surfaces A/B, then pcurves A/B in that
documented order.

Insertion has these rules:

1. Every referenced handle must already be live in the same graph.
2. Descriptors are immutable after insertion.
3. Therefore ordinary insertion is dependency-before-dependent and acyclic by
   construction.
4. A deterministic reverse-dependency index is updated atomically with each
   insertion/rollback. Its values are kept in arena slot order, not hash order.
5. Public removal is deferred. Internal removal/GC must reject a node with graph
   dependents or topology consumers.

Native deserialization and X_T can contain forward references or corrupt
cycles. Their builders first construct a transport-ID table, then resolve with
tri-color DFS (`Unvisited`, `Visiting`, `Complete`). Re-entering a `Visiting`
node returns `GeometryBuildError::DependencyCycle` with the deterministic path
of transport IDs. The evaluator also keeps an active stack and rejects a cycle
as defense in depth; malformed persisted data must never recurse until stack
overflow.

Expose deterministic utilities needed by writers, checkers, and indexes:

- direct dependencies;
- dependency-first transitive traversal with duplicates removed;
- dependents of a node;
- reachability query;
- graph validation that checks live handles, cycles, descriptor invariants, and
  reverse-index agreement.

## Fallible evaluation

Do not change `kgeom::Curve`, `Surface`, or `Curve2d` into graph-aware traits.
They remain infallible leaf protocols. `kgraph::EvalContext` is the only entry
point for resolving handles and evaluating procedural nodes:

```rust
pub struct EvalLimits {
    pub max_dependency_depth: usize,
    pub max_node_visits_per_query: usize,
}

pub struct EvalContext<'g> {
    graph: &'g GeometryGraph,
    limits: EvalLimits,
    tolerances: Tolerances,
    active: Vec<GeometryRef>,
    node_visits: usize,
}

pub enum SurfaceDerivativeOrder {
    Position,
    First,
    Second,
}

impl EvalContext<'_> {
    pub fn eval_curve(
        &mut self,
        curve: CurveHandle,
        t: f64,
        order: usize,
    ) -> EvalResult<CurveDerivs>;

    pub fn eval_surface(
        &mut self,
        surface: SurfaceHandle,
        uv: [f64; 2],
        order: SurfaceDerivativeOrder,
    ) -> EvalResult<SurfaceDerivs>;

    pub fn surface_bounds(
        &mut self,
        surface: SurfaceHandle,
        range: [ParamRange; 2],
    ) -> EvalResult<Aabb3>;
}
```

Each public query resets per-query work accounting even when the context is
reused. Contexts are cheap, per-thread values. The graph is read-only during an
evaluation. F1 does not add a shared mutable cache; bounded memoization can be
added after profiling without changing descriptors or handles.

Evaluation validates finite parameters and finite bounded ranges, accounts one
visit before descending, checks the active stack, and checks returned values for
finiteness and the model size box where applicable. It never clamps invalid
procedural input silently.

Use a typed `kgraph::EvalError` during this project:

```rust
#[non_exhaustive]
pub enum EvalError {
    StaleGeometryHandle { geometry: GeometryRef },
    InvalidParameter,
    ParameterOutsideDomain,
    DependencyCycle { path: Vec<GeometryRef> },
    DependencyDepthExceeded { consumed: usize, limit: usize },
    NodeVisitLimitExceeded { consumed: usize, limit: usize },
    SingularSurface { surface: SurfaceHandle, uv: [f64; 2] },
    IllConditionedSurface { surface: SurfaceHandle, uv: [f64; 2] },
    DerivativeUnavailable { class: GeometryClassKey, requested: usize },
    NonFiniteResult { class: GeometryClassKey },
}
```

Higher layers retain the distinction when mapping to their public errors.
F4 may later move shared capability/stage identifiers into `kcore`; F1 must not
collapse evaluation failures into `InvalidGeometry { reason }`.

### Boundary with F2 OperationContext

`EvalLimits` owns only graph-recursion work: dependency depth and node visits.
It is not a solver policy, cancellation token, trace sink, proof budget, or
session facade. `EvalContext` accepts the existing `Tolerances` value.

F2's `OperationContext` should own the broader precision/conditioning/solver
limits and construct or contain a graph `EvalContext` for each operation. If F2
lands first, F1 should accept a small borrowed policy view from it rather than
duplicate those fields. The class/descriptor/handle API does not depend on that
choice.

## Offset evaluation

Let the basis surface be `s(u,v)`, with first and second derivatives supplied
by the leaf or recursively evaluated basis. Define:

```text
w   = s_u x s_v
q   = |w|
n   = w / q
w_u = s_uu x s_v + s_u x s_uv
w_v = s_uv x s_v + s_u x s_vv
n_u = (w_u - n (n dot w_u)) / q
n_v = (w_v - n (n dot w_v)) / q

offset position = s + d n
offset u        = s_u + d n_u
offset v        = s_v + d n_v
```

This supplies exact position and first derivatives from basis derivatives
through second order. The offset inherits the basis parameter range and
periodicity. Basis degeneracies remain degeneracies; additional offset
singularities occur when the offset Jacobian loses rank, equivalently when an
offset factor reaches a principal radius of curvature under the chosen sign
convention.

The initial offset node supports `Position` and `First`. A request for `Second`
returns `DerivativeUnavailable`, because exact second offset derivatives need
third basis derivatives and the current `kgeom::SurfaceDerivs` protocol stops at
second order. It must not return zero or finite-difference derivatives. A later
surface-jet extension can add the missing exact order without changing the
descriptor.

### Validity and singularity

Provide a fallible validity query:

```rust
pub enum SurfaceValidity {
    Regular { normalized_jacobian: f64 },
    Singular,
    Indeterminate { reason: ValidityGap },
}

pub fn surface_validity(
    &mut self,
    surface: SurfaceHandle,
    uv: [f64; 2],
) -> EvalResult<SurfaceValidity>;
```

For the vertical slice:

- exact zero or non-finite basis/offset Jacobians are singular;
- a normalized Jacobian at or below the angular tolerance is
  `IllConditioned`/`Indeterminate`, not guessed regular;
- regular evaluated points are accepted only after the offset Jacobian is
  checked;
- an interval or face region is certified regular only when its class-specific
  curvature/Jacobian bound excludes zero over the complete region;
- unsupported global certification remains explicitly indeterminate.

This is enough to import and tessellate regular offset faces while refusing to
bridge a singularity. General global curvature certification is a later proof
extension, not permission to treat samples as proof.

### Conservative work boxes

For any finite basis parameter rectangle, the unit-normal displacement has
length `abs(d)`. Therefore:

```text
offset_box(range) = outward_inflate(basis_box(range), abs(d))
```

This bound is conservative even when curvature is high. It is not necessarily
tight. Inflation must use the existing outward-rounded AABB machinery. A stale
basis, non-finite distance/range, or unavailable basis bound is an error, not an
empty box.

## Topology integration

`ktopo::Store` owns a `GeometryGraph` alongside topology and point arenas.
Starting a topology transaction starts graph undo frames; commit and rollback
include all three geometry arenas and the reverse-dependency index. A failed
checked commit must leave graph counts, handle validity, free-list order, and
dependency traversal exactly as they were at transaction entry.

The existing shared-geometry body index becomes transitive:

- a body's footprint contains the curve/surface/pcurve handles directly used
  by its topology and their complete graph dependency closure;
- a geometry mutation selects bodies whose footprints contain that node;
- checked commit validates every live dependency and reverse-index edge;
- geometry deletion is rejected while any topology footprint or graph node
  reaches it.

Because descriptors are immutable, normal modeling operations do not mutate a
basis under an offset. They create a replacement and retarget affected faces.
The transitive index is nevertheless required for import rollback, future
native loading/GC, diagnostics, and any controlled internal reconstruction.

Update topology consumers as follows:

- `Face`, `Edge`, and `FinPcurve` keep typed IDs with no layout change beyond
  the aliased handle target.
- `FaceDomain::natural` asks the graph for parameter metadata.
- incidence, checking, and boundary tessellation evaluate through one borrowed
  `EvalContext`, rather than calling `SurfaceGeom::as_surface()`.
- analytic proof accelerators inspect `SurfaceClass` and borrow the exact leaf
  descriptor. Unknown/procedural classes return an explicit proof gap and may
  use graph-generic evaluation where their contract permits it.
- pcurve-driven tessellation evaluates every UV vertex through the graph,
  checks surface validity, and returns `Indeterminate`/an evaluation error if a
  cell cannot exclude a singularity. It never silently samples across one.

The offset slice does not require broad checker proof for curved offset faces.
It does require endpoint/incidence checks and tessellation to consume the graph
correctly, while unsupported full proofs remain named `Indeterminate` gaps.

## Operations integration

Keep current leaf-specialized `kops` functions intact during migration. Add
graph-aware entry points alongside them:

```rust
pub fn intersect_graph_surfaces(
    graph: &GeometryGraph,
    a: SurfaceHandle,
    a_range: [ParamRange; 2],
    b: SurfaceHandle,
    b_range: [ParamRange; 2],
    context: &mut OperationContext,
) -> Result<SurfaceSurfaceIntersections>;
```

The graph-aware dispatcher uses `SurfaceClass`:

- two supported leaf classes borrow leaf values and call existing analytic
  accelerators;
- leaf/procedural and procedural/procedural pairs use a generic evaluator-based
  path when that solver lands;
- lack of a certified path is unsupported or indeterminate according to the
  common intersection contract, never `InvalidGeometry` and never a proven
  empty result.

F1 only needs to prove that `kops` can inspect an offset, evaluate it, obtain its
dependency closure, and preserve its handle in a result/request. Implementing
offset intersections belongs to F3/M4. This keeps F1 independent of the
intersection consolidation project.

## X_T and native serialization

### X_T reconstruction

`kxt::recon::surface` becomes dependency-aware. For `OFFSET_SURF` it:

1. marks the X_T node `Visiting`;
2. resolves the referenced basis surface recursively;
3. accepts `check` values `U` and `V`, rejects `I`, ignores the published-unused
   `true_offset` field and internal nullable `scale`, validates a nonzero finite
   `offset`, and requires the offset and basis surface senses to agree;
4. converts the transmitted signed displacement to model meters along the
   graph basis's natural normal: use `offset` for basis sense `+` and `-offset`
   for basis sense `-`;
5. inserts `SurfaceDescriptor::Offset` referencing the basis handle;
6. marks the X_T node `Complete` and caches its graph handle.

Only the constant normal-offset form enters the first slice. Invalid check
status, mismatched senses, a zero-resolution displacement, or malformed field
types return typed reconstruction errors; the unused flag and scale do not
change geometry. These rules follow the published
[*Parasolid XT Format Reference, April 2008*](https://ww3.cad.de/foren/ubb/uploads/Rainer%2BSchulze/XT_Format_April_2008_tcm73-62642.pdf),
`OFFSET_SURF` section. A modern external Parasolid
round-trip fixture still gates claims about emitting multiple offset nodes that
share one basis, because the older reference forbids that sharing while the
graph intentionally supports it internally.

Recursive reconstruction means a basis node is interned once even if it is
used by multiple offsets or directly by another face. An X_T dependency cycle
fails with its deterministic node-index path and rolls back the whole import.

### X_T writing

The writer starts from topology-attached handles, walks the dependency closure,
and assigns X_T node IDs in stable dependency-first order. Shared nodes are
emitted once. `SurfaceDescriptor::Offset` emits `OFFSET_SURF` referencing the
already-planned basis node. The canonical first-slice form uses common sense
`+`, check `U` (the Full regularity proof is still open), `true_offset=F`, null
`scale`, and the graph distance directly. Canonical field values are
acceptable; semantic and class-preserving round-trip is required,
byte-for-byte retention is not. Shared-basis emission remains oracle-gated as
described above.

Writer planning must not depend on `HashMap` iteration. Root bodies retain
their existing deterministic order; direct dependencies retain descriptor
field order; duplicate suppression uses handles but never determines output
order.

### Native graph format

Do not serialize arena indexes/generations as durable identity. A future native
format uses document-local IDs and records:

```text
document id
class key
descriptor schema version
dependency document ids
class payload
```

Nodes are written dependency-first and rebuilt through the same cycle-checking
builder used by X_T. Unknown class keys or unsupported schema versions are
typed capability failures. No trait-object or plugin ABI is implied.

## Migration plan

Each stage is a reviewable commit/PR and keeps the workspace green.

### G1 — Graph contract and leaf parity

- Add `kgraph` to the workspace with node, handle, class, descriptor,
  dependency, error, and `EvalContext` types.
- Store leaf analytic/NURBS/2D descriptors in standalone graph tests.
- Delegate leaf evaluation and metadata to current `kgeom` traits.
- Add insertion validation, traversal, reverse indexing, cycle-safe transport
  builder, and deterministic graph validation.

Exit: every existing geometry class has graph evaluation parity tests, and no
topology source has changed yet.

### G2 — Move topology geometry ownership

- Embed `GeometryGraph` in `ktopo::Store`.
- Re-export handle/descriptor compatibility names.
- Move curve, surface, and pcurve arenas plus transaction undo into the graph.
- Migrate store, assembly, journal, checker, incidence, tessellation, and
  constructors to explicit geometry access/evaluation.
- Extend body footprints through transitive graph dependencies.

Exit: existing topology, transaction, primitive, tessellation, determinism, and
X_T Tier-1 tests pass with leaf geometry stored only in `GeometryGraph`.

### G3 — Offset descriptor and evaluator

- Implement `OffsetSurfaceDescriptor`, first-order exact evaluation, inherited
  metadata, validity query, and conservative inflated bounds.
- Add singular/ill-conditioned errors and limit accounting.
- Teach pcurve-driven topology tessellation to evaluate a regular offset face.

Exit: analytic and NURBS basis unit tests pass; a checked topology face can
share its basis with an offset without geometry duplication.

### G4 — X_T offset vertical slice

- Add tri-color surface reconstruction and `OFFSET_SURF` field handling.
- Add dependency-first writer planning and offset emission.
- Add a small committed synthetic offset X_T fixture; update capability codes.
- Ratchet the production exemplar manifest from reconstruction-blocked only
  when the available fixture actually reconstructs and tessellates.

Exit: synthetic read/evaluate/tessellate/write/read is deterministic and
class-preserving; the external oracle accepts the output. The import rollback
test covers malformed cyclic and singular cases.

### G5 — Operations adapter and follow-on descriptors

Priority gate: F2 owns operation-family profile composition and the NURBS
scale guards required by the generic fallback are complete. X_T
reconstruction-owned graph evaluation now consumes one caller-owned child
reservation across face-domain metadata and SP-curve validation, including
aggregate and root-total limit reconciliation. Checked commit's broader Fast
checker evaluation remains part of F2 Stage 5 rather than this G5 gate. The
existing facade evaluation adapter remains the reference accounting contract.

- Add graph-aware `kops` inspection/evaluation adapters without rewriting
  analytic solvers.
- Reserve the verified intersection descriptor construction path for the M3c
  intersection-import project.
- Add swept, spun, and blend descriptors only with their own evaluator,
  validity, bounds, interchange, and test contracts.

Exit: F3/M4 can add a procedural fallback without changing topology handles or
the graph ownership model.

## Test matrix

### Graph and dependency tests

- leaf nodes have no dependencies; offsets report exactly one basis dependency;
- two offsets share one basis handle and graph node count proves no copy;
- dependency-first traversal is stable and deduplicates a diamond graph;
- stale dependency insertion is rejected;
- transport self-cycle and multi-node cycle report stable paths;
- evaluation's defensive recursion guard rejects a forged cycle;
- depth and node-visit limits report consumed and allowed values;
- graph clone preserves values but has independent undo state;
- graph validation catches reverse-index disagreement in a test-only corruptor.

### Offset evaluator tests

- world plane offset: exact position, first derivatives, normal, range, and
  inflated box for positive, negative, and zero distances;
- cylinder offset: expected radius and derivatives for both signed directions;
- sphere inward offset at its radius reports a singular surface;
- nested regular offsets evaluate deterministically and share their base chain;
- finite-difference checks validate implemented analytic first derivatives, but
  finite differences are test oracles only, never production derivatives;
- second derivative requests return `DerivativeUnavailable`;
- non-finite parameter/distance and out-of-domain parameter are rejected;
- repeated runs and debug/release produce the existing deterministic golden
  representation.

### Topology and transaction tests

- a face references an offset handle and its pcurve tessellation vertices lie
  on the expected offset within declared tolerance;
- checker incidence uses graph evaluation and identifies a deliberately
  displaced pcurve/edge;
- unsupported full offset proof returns a named indeterminate obligation;
- a failed checked transaction that inserted a basis and offset rolls both
  back, including free-list behavior and reverse dependencies;
- a shared basis is retained while any direct face or offset dependency uses
  it;
- a basis dependency is included in every consuming body's footprint.

### Interchange tests

- synthetic X_T offset fixture imports one basis and one offset node;
- multiple offset faces referencing one basis do not duplicate it;
- X_T offset/basis pointer cycle fails deterministically and atomically;
- imported class key is `kernel.surface.offset.v1` before and after write/read;
- writer emits dependency before dependent and produces byte-identical output
  on repeated writes of the same store;
- external Parasolid oracle accepts the canonical output and sampled points and
  normals agree with host evaluation;
- production exemplar reconstruction/tessellation becomes a ratcheted test only
  when its licensed fixture is available in the expected corpus workflow.

## Explicit non-goals

- Changing the leaf `kgeom` evaluator traits to return `Result`.
- Implementing general surface third derivatives in the offset slice.
- Certifying every offset face globally free of singularities.
- General graph memoization, eviction policy, parallel scheduling, or GPU
  evaluation.
- A plugin/custom-geometry ABI or dynamic class registry.
- A durable native file format implementation; only its identity rules are
  fixed here.
- Offset/offset or offset/NURBS intersection completion.
- Recomputing imported intersection curves.
- Sweeps, spun surfaces, rolling-ball blends, blend bounds, or foreign geometry.
- Moving topological points into the graph.
- Freezing the public `Kernel`/`Session` facade.
- Duplicating F2's solver policy, cancellation, diagnostics, or work budgets.

## Acceptance criteria

F1 is complete only when all of the following are true:

1. `kgraph` exists at L1.5 with enforced one-way Cargo dependencies.
2. All existing curves, surfaces, and pcurves have one graph-owned node and
   retain exact leaf class identity and evaluator behavior.
3. `ktopo` faces/edges/fins hold graph handles; no basis surface is stored by
   value inside a procedural descriptor or duplicated in topology.
4. Descriptors are immutable, dependencies are inspectable and deterministic,
   ordinary insertion is acyclic by construction, and transport cycles fail
   with a typed deterministic path.
5. Evaluation is fallible, bounded by dependency depth/node visits, and reports
   stale handles, cycles, unavailable derivatives, singularities, and
   ill-conditioning distinctly.
6. Constant signed offsets evaluate exact positions and first derivatives,
   inherit parameter metadata, and return conservative outward-rounded work
   boxes.
7. Offset validity is never inferred from clean sampling; singular or
   unresolved regions stop tessellation with a typed outcome.
8. Graph insertions participate in topology transaction rollback, checked
   commits, journals, and transitive affected-body indexing.
9. `kxt` reconstructs and writes the supported `OFFSET_SURF` form with shared
   basis identity and class-preserving, deterministic dependency-first output.
10. A committed synthetic fixture passes read/evaluate/tessellate/write/read and
    external-oracle validation; malformed cycle and rollback tests pass.
11. Existing workspace formatting, Clippy, debug/release tests, determinism
    tests, Tier-1 X_T fixtures, and primitive tessellation tests do not regress.
12. `kops` can inspect and evaluate graph surfaces without depending on
    topology; unsupported procedural intersection paths remain explicitly
    unsupported/indeterminate rather than returning a false complete miss.

## Open risks and decisions requiring evidence

- **X_T shared offset bases.** Sign, unused-flag, and nullable-scale semantics
  are resolved by the published reference. The remaining format risk is the
  older restriction against multiple offset nodes sharing one basis. Keep the
  graph representation permissive, but require a modern Parasolid oracle before
  claiming shared-basis writer conformance.
- **Third derivatives.** Exact second derivatives of an offset require a larger
  surface jet. M4 SSI marching and curvature-driven tessellation are the known
  future consumers and must treat `DerivativeUnavailable` as an explicit gate,
  not rediscover it as a numerical failure. Defer the jet API until one of
  those consumers owns the end-to-end contract; never substitute production
  finite differences or zeros.
- **Assembly-scale reverse indexing.** The implemented reverse-dependency index
  and visited/order helpers favor simple deterministic vectors and linear
  scans. Preserve that correctness baseline, but land the F7/Q2a graph-build
  ladder before production-scale imports. A slot-indexed adjacency or other
  replacement is justified only by measurements and must retain deterministic
  insertion order, rollback, stale-handle behavior, and full-index audit
  equality.
- **Global regularity proof.** Bounding principal curvature over arbitrary
  NURBS regions is nontrivial. The first slice is useful with local evaluation
  and explicit indeterminate region proof; it must not overclaim certification.
- **Topology migration size.** Moving three arenas touches many consumers. The
  leaf-parity stage and compatibility re-exports are deliberately separate so
  the storage move remains mechanical and reviewable.
- **Error taxonomy coordination.** F1 needs typed evaluation failures now; F4
  should later standardize shared capability and stage IDs without erasing the
  distinctions or forcing graph types into `kcore`.
- **Operation-context coordination.** F2 owns numerical/solver policy. F1 owns
  only graph recursion limits and consumes existing tolerances, preventing two
  competing context abstractions.
