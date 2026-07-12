//! `kxt` — Parasolid XT transmit-file interchange (spec §L5, roadmap M3).
//!
//! Reads `.x_t` (text) and `.x_b` (neutral binary) transmit files into
//! [`ktopo`] bodies. Implemented clean-room from the openly published
//! *Parasolid XT Format Reference* (Siemens) and inspection of real
//! transmit files — never from Parasolid code.
//!
//! Current M3 scope: both read wire encodings, schema edit decoding for modern
//! files based on schema 13006, and atomic reconstruction of the supported
//! topology and geometry subset. Failed reconstruction leaves the supplied
//! store unchanged. Supported geometry includes point, line, circle, ellipse,
//! B-curve, plane, cylinder, cone, sphere, torus, B-surface, and the G4a
//! single-offset subset. The M3b
//! writer emits deterministic base-schema text XT for checker-clean solids,
//! supported sheet bodies, supported wire bodies, acorn bodies, and bounded
//! curve-less tolerant edges using analytic geometry, non-periodic B-spline/
//! NURBS geometry, dependency-first one-level offsets, and per-fin trimmed
//! SP-curves over finite 2D B-curves.
//! Reconstruction is failure-atomic through `ktopo`'s copy-on-write Store
//! transactions and returns the committed entity mutation journal. Imported
//! entity tolerances are validated and retain explicit XT origin provenance;
//! writing emits their current metric value. Valid
//! content outside the declared subset reports a stable [`XtCapability`]
//! code as well as human-readable context.
//! Intersection and other procedural geometry, nested/shared-basis offset
//! export, broader tolerant topology, periodic or
//! circular pcurve encoding, periodic NURBS, pre-13006 schemas, assemblies,
//! non-null face-tolerance writing, and neutral-binary writing remain deferred. Face UV
//! work domains are kernel-side metadata because XT bounds faces through their loops.
//!
//! Typical use:
//!
//! ```no_run
//! let bytes = std::fs::read("part.x_t").unwrap();
//! let mut store = ktopo::store::Store::new();
//! let recon = kxt::import(&bytes, &mut store).unwrap();
//! for body in recon.bodies {
//!     assert!(ktopo::check::check_body(&store, body).unwrap().is_empty());
//! }
//! ```

pub mod cursor;
pub mod error;
pub mod parse;
pub mod recon;
pub mod schema;
pub mod write;

pub use error::{Result, XtCapability, XtError};
pub use parse::{Header, Node, Value, XtFile, read_xt};
pub use recon::{
    Reconstruction, reconstruct, reconstruct_in_scope, reconstruct_with_context,
    reconstruction_budget_profile,
};
pub use write::export_text;

use kcore::operation::{OperationContext, OperationOutcome, OperationPolicyError, OperationScope};
use ktopo::store::Store;

/// Parse and reconstruct a transmit file atomically. On error, `store` is
/// unchanged.
pub fn import(bytes: &[u8], store: &mut Store) -> Result<Reconstruction> {
    let file = read_xt(bytes)?;
    reconstruct(&file, store)
}

/// Parse and reconstruct with graph and curve-projection work charged to a
/// fresh operation scope.
pub fn import_with_context(
    bytes: &[u8],
    store: &mut Store,
    context: &OperationContext<'_>,
) -> core::result::Result<OperationOutcome<Reconstruction, XtError>, OperationPolicyError> {
    let context = context
        .clone()
        .with_family_budget_defaults(reconstruction_budget_profile());
    kgraph::EvalLimits::from_budget_plan(&context.effective_budget())?;
    let mut scope = OperationScope::new(&context);
    let result = read_xt(bytes).and_then(|file| reconstruct_in_scope(&file, store, &mut scope, 0));
    Ok(scope.finish_typed(result))
}

/// Parse and reconstruct inside an existing caller-owned operation scope.
///
/// The caller supplies the stable ordinal for the reconstruction's one graph
/// child reservation and must have installed the X_T reconstruction profile
/// (graph evaluation plus aggregate curve projection) before creating `scope`.
pub fn import_in_scope(
    bytes: &[u8],
    store: &mut Store,
    scope: &mut OperationScope<'_, '_>,
    child_ordinal: u64,
) -> Result<Reconstruction> {
    let file = read_xt(bytes)?;
    reconstruct_in_scope(&file, store, scope, child_ordinal)
}
