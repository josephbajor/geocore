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
//! B-curve, plane, cylinder, cone, sphere, torus, and B-surface. The initial
//! M3b writer emits deterministic base-schema text XT for self-authored
//! analytic solids. Procedural and SP geometry, tolerant entities, pre-13006
//! schemas, assemblies, and neutral-binary writing remain deferred.
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

pub use error::{Result, XtError};
pub use parse::{Header, Node, Value, XtFile, read_xt};
pub use recon::{Reconstruction, reconstruct};
pub use write::export_text;

use ktopo::store::Store;

/// Parse and reconstruct a transmit file atomically. On error, `store` is
/// unchanged.
pub fn import(bytes: &[u8], store: &mut Store) -> Result<Reconstruction> {
    let file = read_xt(bytes)?;
    reconstruct(&file, store)
}
