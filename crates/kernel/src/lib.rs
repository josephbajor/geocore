//! Supported native Rust façade for kernel lifecycle, topology, and geometry identity reads.
//!
//! The implemented foundation owns sessions and independent parts, exposes
//! opaque part-qualified topology and geometry identities, and returns
//! immutable semantic views. Modeling operations, journals, operation-scoped
//! geometry evaluation/intersection, interchange, and contextual error
//! adaptation remain later façade stages.
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
//! Geometry identities are equally opaque and expose no graph handle:
//!
//! ```compile_fail
//! fn expose_geometry(id: kernel::SurfaceId) {
//!     let kernel::SurfaceId { part, raw } = id;
//!     let _ = (part, raw);
//! }
//! ```
//!
//! ```compile_fail
//! fn construct_geometry(part: kernel::PartId) {
//!     let _ = kernel::SurfaceId::new(part, todo!());
//! }
//! ```
//!
//! ```compile_fail
//! fn raw_geometry(id: kernel::CurveId) {
//!     let _ = id.raw();
//! }
//! ```
//!
//! Geometry views expose stable metadata, not graph descriptors:
//!
//! ```compile_fail
//! fn descriptor(view: kernel::SurfaceView<'_>) {
//!     let _ = view.descriptor();
//! }
//! ```
//!
//! A part cannot construct an uncharged evaluator:
//!
//! ```compile_fail
//! fn evaluator(part: kernel::Part<'_>) {
//!     let _ = part.eval_context();
//! }
//! ```
//!
//! A part cannot expose its authoritative graph:
//!
//! ```compile_fail
//! fn graph(part: kernel::Part<'_>) {
//!     let _ = part.geometry();
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
pub use id::{
    BodyId, CurveId, EdgeId, FaceId, FinId, LoopId, PartId, PcurveId, RegionId, ShellId, SurfaceId,
    VertexId,
};
pub use iter::{
    BodyIds, CurveIds, EdgeIds, FaceIds, FinIds, LoopIds, PartIds, PcurveIds, RegionIds, ShellIds,
    SurfaceIds, VertexIds,
};
pub use session::{Kernel, Part, PartEdit, Session};
pub use view::{
    BodyView, CurveView, EdgeView, FaceView, FinView, LoopView, PcurveView, RegionView, ShellView,
    SurfaceView, VertexView,
};

pub use kcore::operation::SessionPolicy;
pub use kgeom::param::ParamRange;
pub use kgeom::vec::{Point3, Vec3};
pub use kgraph::GeometryClassKey;
pub use ktopo::entity::{BodyKind, FaceDomain, RegionKind, Sense};
pub use ktopo::tolerance::{EntityTolerance, ToleranceOrigin};
