# `kernel`

`kernel` is the supported native Rust facade for this modeling kernel. Ordinary
applications, feature systems, bindings, and product tools should depend on
this crate rather than making topology storage or geometry-graph handles part
of their source-level vocabulary.

The current facade supports:

- kernel/session/part lifecycle with opaque, part-qualified identities;
- deterministic semantic topology and geometry views;
- contextual block construction and Fast/Full body checking;
- operation-accounted conforming solid and sheet tessellation;
- bounded, operation-accounted surface evaluation; and
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
the facade compatibility promise.

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
