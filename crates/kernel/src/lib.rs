//! Supported native Rust façade for kernel lifecycle and topology reads.
//!
//! K1 is deliberately narrow: it owns sessions and independent parts, exposes
//! opaque part-qualified topology identities, and returns immutable semantic
//! views. Modeling operations, journals, geometry identities/evaluation,
//! interchange, and contextual error adaptation land in later façade stages.
//!
//! Raw lower-layer storage is not reachable through this crate:
//!
//! ```compile_fail
//! fn raw_store(part: kernel::Part<'_>) {
//!     let _ = part.store();
//! }
//! ```
//!
//! Opaque identities cannot be constructed or destructured:
//!
//! ```compile_fail
//! fn expose(id: kernel::BodyId) {
//!     let kernel::BodyId { part, raw } = id;
//!     let _ = (part, raw);
//! }
//! ```
//!
//! Views expose no raw fields or mutable backlink collections:
//!
//! ```compile_fail
//! fn mutate(view: kernel::BodyView<'_>) {
//!     view.regions.clear();
//! }
//! ```
//!
//! A read view prevents acquiring a mutable capability for the same session:
//!
//! ```compile_fail
//! fn conflict(
//!     session: &mut kernel::Session,
//!     part_id: kernel::PartId,
//!     body_id: kernel::BodyId,
//! ) {
//!     let part = session.part(part_id.clone()).unwrap();
//!     let body = part.body(body_id).unwrap();
//!     let _edit = session.edit_part(part_id).unwrap();
//!     let _ = body.kind();
//! }
//! ```
//!
//! Mutable part capabilities do not expose raw assembly:
//!
//! ```compile_fail
//! fn assemble(mut edit: kernel::PartEdit<'_>) {
//!     edit.assembly();
//! }
//! ```
//!
//! Sessions uniquely own their parts and therefore are not cloneable:
//!
//! ```compile_fail
//! fn duplicate(session: kernel::Session) {
//!     let _ = session.clone();
//! }
//! ```

mod error;
mod id;
mod iter;
mod session;
mod view;

pub use error::{EntityKind, Error, Result, code as error_code};
pub use id::{BodyId, EdgeId, FaceId, FinId, LoopId, PartId, RegionId, ShellId, VertexId};
pub use iter::{
    BodyIds, EdgeIds, FaceIds, FinIds, LoopIds, PartIds, RegionIds, ShellIds, VertexIds,
};
pub use session::{Kernel, Part, PartEdit, Session};
pub use view::{
    BodyView, EdgeView, FaceView, FinView, LoopView, RegionView, ShellView, VertexView,
};

pub use kcore::operation::SessionPolicy;
pub use kgeom::param::ParamRange;
pub use kgeom::vec::{Point3, Vec3};
pub use ktopo::entity::{BodyKind, FaceDomain, RegionKind, Sense};
pub use ktopo::tolerance::{EntityTolerance, ToleranceOrigin};
