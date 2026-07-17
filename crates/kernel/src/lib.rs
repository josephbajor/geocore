//! Supported native Rust façade for kernel lifecycle, topology, and geometry identity reads.
//!
//! The implemented foundation owns sessions and independent parts, exposes
//! opaque part-qualified topology and geometry identities, and returns
//! immutable semantic views. Contextual operations cover checked block and
//! polygonal-profile extrusion construction, deterministic complete-body
//! rigid copy, body checking and tessellation, operation-scoped surface
//! evaluation, and typed X_T
//! import/export plus graph-owned bounded curve intersection with F2 reports
//! and delegated classified errors. Rigid copies own disjoint topology and
//! geometry closures and retain exact `DerivedFrom` lineage. Committed
//! mutation, lineage, tolerance-growth, and descriptive face-tolerance
//! propagation evidence is exposed through part-qualified facade journal
//! views. A narrow checked semantic edit transaction composes
//! pcurve-aware face split/merge operations with validated affine
//! edge-to-pcurve maps plus facade-owned periodic-chart, seam, closed-use, and
//! singular-endpoint metadata, without exposing raw assembly.
//! Broader modeling operations remain later façade stages.
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
//! Contextual operations do not expose their active operation scope:
//!
//! ```compile_fail
//! fn scope(part: kernel::Part<'_>) {
//!     let _ = part.operation_scope();
//! }
//! ```
//!
//! Committed journals expose part-qualified semantic views, not a raw
//! transaction journal or its entity handles:
//!
//! ```compile_fail
//! fn raw_journal(created: &kernel::BodyCreated) {
//!     let _ = created.journal().raw();
//! }
//! ```
//!
//! Journal-only point identities expose no stored point handle:
//!
//! ```compile_fail
//! fn raw_journal_point(id: kernel::JournalPointId) {
//!     let _ = id.raw();
//! }
//! ```
//!
//! Semantic edit transactions expose no raw store or unchecked commit seam:
//!
//! ```compile_fail
//! fn raw_edit(mut edit: kernel::EditTransaction<'_>) {
//!     let _ = edit.store();
//! }
//! ```
//!
//! Operation settings are extended through typed builders rather than direct
//! field mutation:
//!
//! ```compile_fail
//! fn mutate_settings(mut settings: kernel::OperationSettings) {
//!     settings.diagnostic_capacity = usize::MAX;
//! }
//! ```
//!
//! Geometry-evaluation errors retain lower sources without exposing their
//! graph-specific payload as a public field:
//!
//! ```compile_fail
//! fn raw_evaluation(error: kernel::GeometryEvaluationError) {
//!     let _ = error.source;
//! }
//! ```
//!
//! Intersection errors retain lower sources without exposing their solver
//! payload as a public field:
//!
//! ```compile_fail
//! fn raw_intersection_error(error: kernel::GeometryIntersectionError) {
//!     let _ = error.source;
//! }
//! ```
//!
//! Tessellation errors likewise preserve lower sources without exposing their
//! implementation payload as a public field:
//!
//! ```compile_fail
//! fn raw_tessellation_error(error: kernel::BodyTessellationError) {
//!     let _ = error.source;
//! }
//! ```
//!
//! Tessellation results expose immutable facade values rather than mutable
//! lower mesh fields:
//!
//! ```compile_fail
//! fn raw_mesh(mut mesh: kernel::BodyMesh) {
//!     mesh.triangles.clear();
//! }
//! ```
//!
//! Intersection results expose facade accessors rather than mutable lower
//! result collections:
//!
//! ```compile_fail
//! fn raw_intersections(mut result: kernel::CurveCurveIntersections) {
//!     result.points.clear();
//! }
//! ```
//!
//! Surface-evaluation results retain opaque facade identity rather than a
//! graph handle:
//!
//! ```compile_fail
//! fn raw_surface(result: kernel::SurfaceEvaluation) {
//!     let _ = result.surface().raw();
//! }
//! ```
//!
//! The facade exposes the typed budget profile, not an independently
//! configurable graph evaluator limit object:
//!
//! ```compile_fail
//! fn graph_limits() {
//!     let _ = kernel::EvalLimits::default();
//! }
//! ```
//!
//! Facade face domains do not accept raw graph descriptors:
//!
//! ```compile_fail
//! fn raw_natural(surface: &kgraph::SurfaceDescriptor) {
//!     let _ = kernel::FaceDomain::natural(surface);
//! }
//! ```
//!
//! X_T results expose opaque bodies and semantic summaries, not the lower
//! reconstruction object or transport node indexes:
//!
//! ```compile_fail
//! fn raw_import(result: &kernel::ImportXtResult) {
//!     let _ = result.reconstruction();
//! }
//! ```
//!
//! ```compile_fail
//! fn transport_index(skipped: kernel::XtSkippedNode) {
//!     let _ = skipped.node_index();
//! }
//! ```
//!
//! Interchange errors retain their exact source without a public raw field:
//!
//! ```compile_fail
//! fn raw_xt_error(error: kernel::XtInterchangeError) {
//!     let _ = error.source;
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

mod classify;
mod edit;
mod error;
mod id;
mod interchange;
mod intersection;
mod iter;
mod operation;
mod session;
mod tessellation;
mod view;

pub use classify::{
    ClassifyPointInBodyRequest, ClassifyPointOnFaceRequest, POINT_CLASSIFICATION_RAYS,
    POINT_CLASSIFICATION_WORK, PointBodyClassification, PointBodyVerdict,
    PointClassificationBudgetProfile, PointFaceClassification, PointFaceSite, PointFaceVerdict,
    RayParityWitness,
};
pub use edit::{
    BoundedPcurve, CreateSeedBodyRequest, CreateSeedBodyResult, CreateStrutRequest,
    CreateStrutResult, EditTransaction, FullCommitRequirement, FullCommitResult,
    GrowTolerancesRequest, GrowTolerancesResult, JoinRingRequest, JoinRingResult,
    MergeFaceAsHoleRequest, MergeFaceAsHoleResult, MergeFacesRequest, PcurveChart,
    PcurveEndpointKind, PcurveMetadata, PcurveParameterMap, PcurveSeam, PcurveSeamSide,
    RemoveBridgeRequest, RemoveBridgeResult, RemoveSeedBodyRequest, RemoveSeedBodyResult,
    RemoveStrutRequest, RemoveStrutResult, SplitFaceRequest, SplitFaceResult,
    SplitHoleAsFaceRequest, SplitHoleAsFaceResult, SurfaceParameter, ToleranceGrowth,
    ToleranceGrowthTarget,
};
pub use error::{
    BodyCopyError, BodyTessellationError, EntityKind, Error, GeometryEvaluationError,
    GeometryIntersectionError, KernelError, Result, XtInterchangeError, code as error_code,
};
pub use id::{
    BodyId, CurveId, EdgeId, FaceId, FinId, JournalPointId, LoopId, PartId, PcurveId, RegionId,
    ShellId, SurfaceId, VertexId,
};
pub use interchange::{
    ExportXtRequest, ExportXtResult, ImportXtRequest, ImportXtResult, XtSkippedNode,
};
pub use iter::{
    BodyIds, CurveIds, EdgeIds, FaceIds, FinIds, LoopIds, PartIds, PcurveIds, RegionIds, ShellIds,
    SurfaceIds, VertexIds,
};
pub use operation::{
    BlockRequest, BodyCheckReport, BodyCreated, BoundedCurve, ChangeJournal, CheckBodyRequest,
    CheckEntity, CheckFault, CheckGap, CheckReport, CopyBodyRequest, CurveContactKind,
    CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint, CurveOverlapOrientation,
    ExtrudeProfileAlongRequest, ExtrudeProfileRequest, FaceTolerancePropagationView,
    IntersectCurvesRequest, IntersectionCompletion, JournalEntities, JournalEntity, LineageView,
    MutationKind, MutationView, OperationOutcome, OperationSettings, SurfaceEvaluation,
    SurfaceEvaluationRequest, ToleranceBudgetId, ToleranceBudgetView, ToleranceEventView,
};
pub use session::{Kernel, Part, PartEdit, Session};
pub use tessellation::{BodyMesh, EdgePolyline, FaceTriangleRange, TessellateBodyRequest};
pub use view::{
    BodyView, CurveView, EdgeView, FaceDomain, FaceView, FinView, LoopView, PcurveView, RegionView,
    ShellView, SurfaceView, VertexView,
};

pub use kcore::error::{CapabilityId, ClassifiedError, ErrorClass, ErrorCode};
pub use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticLevel, ExecutionPolicy, LimitSnapshot, LimitSpec,
    NumericalPolicy, OperationPolicyError, OperationReport, PolicyVersion, ResourceKind,
    SessionPolicy, SessionPrecision, StageId,
};
pub use kcore::proof::{IncompleteCause, IncompleteEvidence};
pub use kcore::tolerance::Tolerances;
pub use kgeom::frame::Frame;
pub use kgeom::nurbs::{CurvePairProjectionPlane, CurvePairRootCertificate};
pub use kgeom::param::ParamRange;
pub use kgeom::surface::SurfaceDerivs;
pub use kgeom::tess::TessOptions;
pub use kgeom::vec::{Point2, Point3, Vec3};
pub use kgraph::{EvalBudgetProfile, GeometryClassKey, SurfaceDerivativeOrder};
pub use ktopo::btess::BodyTessellationBudgetProfile;
pub use ktopo::check::{
    CheckLevel, CheckOutcome, FaultKind, FullCheckBudgetProfile, VerificationGapCause,
    VerificationGapKind,
};
pub use ktopo::entity::{BodyKind, RegionKind, Sense};
pub use ktopo::tolerance::{EntityTolerance, ToleranceOrigin};
