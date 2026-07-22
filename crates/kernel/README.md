# `kernel`

`kernel` is the supported native Rust facade for this modeling kernel. Ordinary
applications, feature systems, bindings, and product tools should depend on
this crate rather than making topology storage or geometry-graph handles part
of their source-level vocabulary.

The current facade supports:

- kernel/session/part lifecycle with opaque, part-qualified identities;
- deterministic semantic topology and geometry views;
- contextual block and cylinder construction plus Fast/Full body checking;
- operation-accounted conforming solid and sheet tessellation;
- bounded, operation-accounted surface evaluation;
- certified body section graphs exposing Plane/Cylinder rings, exact bounded
  arcs, and topology-clipped transverse ruling fragments with operation-shared
  source-edge root identity plus deterministic closed mixed-family cycles for
  shared translated, permuted, and all-nonzero oblique exact frames, with
  semantic Plane incidence certification for rounded general frames and
  proof-bearing lifted periodic cylinder-face embeddings. For same-signed
  strict-secant nested-height parallel cylinders, public Section returns two
  rulings plus two cap arcs in one topology-owned closed component across world
  and oblique frames, replay, and operand swap; the operation-local graph proof
  also admits exact antiparallel axes, while tangent, miss, coincident, and skew
  pairs remain typed gaps;
- failure-atomic block/block unite/intersect/subtract plus axial
  convex-planar/finite-cylinder intersection, finite-cylinder-minus-planar
  remainder bands, zero-cut truth-selected whole-source union/subtraction
  copies and contained-cylinder cavities, one-ring axial cap-overlap connected
  union, one-ring axial block-minus-cylinder blind pockets, two-port axial
  through-holes, two-ring two-sided connected unions, inverse-containment
  convex-planar cavities, support-separated axial exact-contact empty
  intersections, and certified flush axial cap-contact connected unions, with
  convex bounded-arc Plane/Cylinder intersections with rectangular,
  three-sided, and five-support multi-chart layouts across general authored frames, plus ordered
  planar-minus-cylinder subtraction that Full-commits every disconnected
  rectangular or three-sided component and rectangular/five-support
  cap-retaining mixed Unite and cylinder-left Subtract across world, translated, permuted, and oblique
  frames, plus complete nonconvex ten-support star/cylinder Intersect through
  convex-certificate-independent mixed planning, typed empty/created/refused
  outcomes, and Full validation, plus Full-checked same-signed nested-height
  parallel-cylinder Intersect lens and ordered axial-inner-minus-outer
  Subtract crescent prisms across world/oblique frames;
- atomic typed X_T import and deterministic X_T export; and
- deterministic facade-ID mutation, lineage, and tolerance journal iteration;
  and
- checked, failure-atomic face split/merge transactions using existing curve
  and pcurve identities with validated affine edge-to-pcurve maps plus
  facade-owned periodic-chart, seam, closed-use winding, and singular-endpoint
  metadata.

Broader modeling operations and additional semantic edits remain under active
development. The facade is additive: lower crates remain available to kernel
implementation and trusted adapter work, but their raw layouts are not part of
the facade compatibility promise. Parallel-cylinder Booleans are deliberately
limited to the certified strict-secant nested-height Intersect and ordered
axial-inner-minus-outer Subtract families; Unite and reverse Subtract are not
yet claimed.

The five-portal cap-retaining slice verifies exact face/edge/vertex signatures
(23/47/30 for Unite; 10/32/20 for cylinder-left Subtract), independent analytic
mesh volumes, deterministic X_T, Fast self-import, and exact N/N-1 shell-work atomicity.
Proof-local period lifting also admits a radius-1.7 five-support subtraction across authored `u=0` without mutating pcurves.
Exact planar-shell admission is separate from its optional typed convex certificate:
the nonconvex star result is 17F/45E/30V with literal-derived volume and deterministic
X_T/Fast self-import, while pure planar BSP and convex shortcuts remain certificate-gated.

## Application boundary

A facade client imports only `kernel` concepts:

```rust
use kernel::{
    BlockRequest, BodyTessellationBudgetProfile, CheckBodyRequest, CheckLevel,
    CheckOutcome, ExportXtRequest, Frame, Kernel, OperationSettings, TessOptions,
    TessellateBodyRequest,
};

let mut session = Kernel::new().create_session();
let part_id = session.create_part();
let created = session
    .edit_part(part_id.clone())?
    .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))?
    .into_result()?;
let body = created.body();

let part = session.part(part_id)?;
let checked = part
    .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Fast))?
    .into_result()?;
assert_eq!(checked.outcome(), CheckOutcome::Valid);

let mesh = part
    .tessellate_body(
        TessellateBodyRequest::new(
            body.clone(),
            TessOptions { chord_tol: 1.0e-3, max_edge_len: None },
        )
        .with_settings(OperationSettings::new().with_budget_overrides(
            BodyTessellationBudgetProfile::bounded_v1(),
        )),
    )?
    .into_result()?;
assert!(!mesh.triangles().is_empty());

let exported = part
    .export_xt(ExportXtRequest::new(body))?
    .into_result()?;
assert!(!exported.bytes().is_empty());
# Ok::<(), Box<dyn std::error::Error>>(())
```

Facade IDs cannot be constructed or destructured, and no facade view exposes a
`ktopo::Store`, raw entity struct, `kgraph` handle, or low-level assembly
capability. The crate-level rustdoc contains compile-fail guards for these
boundaries.

The repository's
[`kernel-lifecycle`](https://github.com/josephbajor/cad_prototype/tree/main/examples/kernel-lifecycle)
executable is a real facade-only client. Its manifest has `kernel` as its only
direct dependency and exercises construction, semantic inspection, checking,
body tessellation, surface evaluation, and X_T export:

```sh
cargo run -p kernel-lifecycle -- target/kernel-lifecycle.x_t
```

## Packaging status

`cargo package -p kernel --list` is the reviewed file-inventory boundary for
this crate. Full package verification is not enabled yet: the workspace's
internal path dependencies do not yet declare the registry version requirements
that Cargo requires when it removes their `path` entries from a packaged
manifest. That repository-wide versioning/publication decision is tracked
separately; K5 does not paper over it or claim that the facade is publish-ready.
