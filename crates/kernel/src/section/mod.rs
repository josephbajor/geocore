//! Certified section evidence between two solid bodies.
//!
//! Second rung of the boolean ladder: `unite`/`subtract`/`intersect` need the
//! curves where the two operand boundaries meet, stitched into a coherent
//! edge graph whose vertices sit on the operands' own edges and vertices.
//! This module computes that graph for the planar slice — every face on a
//! plane, every edge a bounded straight line. It also retains complete-period
//! Plane/Cylinder circle carriers with paired pcurves as verified partial
//! evidence, while refusing to promote them into trimmed edges or a complete
//! body graph until curved clipping and fragment stitching are certified.
//!
//! The algorithm is general over topology (any number of faces, loops,
//! holes, non-convex boundaries). Per candidate face pair it takes the
//! certified plane/plane carrier line from the graph-aware intersection
//! branch (pcurves, parameter maps, and residual certificate included), then
//! clips that line against each face's trim loops using exact
//! `orient3d`/`orient2d` side signs evaluated on stored vertex coordinates —
//! a point of the carrier is inside face A's plane on a given side of the
//! line exactly when it is on that side of face B's plane. Combinatorial
//! stitching keys (which operand edge or vertex produced each segment
//! endpoint) connect segments across face pairs without ever comparing
//! derived floating-point points. Metric orderings along the carrier use
//! conservative intervals; any decision the intervals and exact signs cannot
//! certify becomes a structured gap, never a guess.
//!
//! Determinism: candidate face pairs iterate in stored body-face order
//! (first operand major), segments order along each carrier by certified
//! parameter, graph vertices and edges number in first-appearance order over
//! that global segment order, and loops start from the lowest unused edge
//! index. Serial re-execution reproduces the graph bit-identically.

mod clip;
mod stitch;

#[cfg(test)]
mod tests;

use kcore::interval::Interval;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::vec::{Point2, Point3, Vec2, Vec3};
use kgraph::{AffineParamMap1d, CurveDescriptor};
use kops::intersect::{
    ContactKind, GraphSurfaceIntersectionError, IntersectionBranchEdge, IntersectionBranchTopology,
    IntersectionBranchVertexEvent, intersect_bounded_graph_surfaces_in_scope,
};
use ktopo::entity::{BodyKind, FaceId as RawFaceId, Sense, SurfaceId as RawSurfaceId};
use ktopo::geom::SurfaceGeom;
use ktopo::store::Store;

use crate::error::{Error, Result};
use crate::operation::{OperationOutcome, OperationSettings};
use crate::session::Part;
use crate::{BodyId, EdgeId, EntityKind, FaceId, PartId, VertexId};

/// Cumulative predicate/clip/stitch work performed by one section query.
pub const SECTION_WORK: StageId = known_stage("kernel.section.work");
/// High-water count of candidate face pairs examined by one section query.
pub const SECTION_FACE_PAIRS: StageId = known_stage("kernel.section.face-pairs");

const fn known_stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid built-in section stage identifier"),
    }
}

/// Built-in accounting ceilings for one body/body section query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BodySectionBudgetProfile;

impl BodySectionBudgetProfile {
    /// Returns generous exact ceilings for one section query.
    pub fn v1_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                SECTION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                64_000_000,
            ),
            LimitSpec::new(
                SECTION_FACE_PAIRS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                1_048_576,
            ),
        ])
        .expect("built-in body-section budget is valid")
    }
}

/// Typed request to section one solid body against another.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionBodiesRequest {
    pub(crate) body_a: BodyId,
    pub(crate) body_b: BodyId,
    pub(crate) settings: OperationSettings,
}

impl SectionBodiesRequest {
    /// Construct a request with default operation settings.
    pub fn new(body_a: BodyId, body_b: BodyId) -> Self {
        Self {
            body_a,
            body_b,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// First operand body.
    pub fn body_a(&self) -> BodyId {
        self.body_a.clone()
    }

    /// Second operand body.
    pub fn body_b(&self) -> BodyId {
        self.body_b.clone()
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Where a section-graph vertex sits on one operand body's boundary.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum SectionSite {
    /// Strictly inside the trimmed face carrying the local section curve.
    FaceInterior(FaceId),
    /// On one bounding edge of the operand, away from its vertices.
    EdgeInterior(EdgeId),
    /// At one vertex of the operand.
    AtVertex(VertexId),
}

/// Conservative intrinsic parameter enclosure on a source edge crossed by
/// a section-graph vertex.
///
/// The parameter is in the source edge's supporting-curve direction, not
/// the local fin or section-loop traversal direction. The closed interval
/// therefore remains meaningful when adjacent faces use the edge with
/// opposite fin senses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionEdgeParameterInterval {
    lo: f64,
    hi: f64,
}

impl SectionEdgeParameterInterval {
    fn from_interval(interval: Interval) -> Self {
        Self {
            lo: interval.lo(),
            hi: interval.hi(),
        }
    }

    /// Lower bound of the closed source-edge parameter enclosure.
    pub const fn lo(self) -> f64 {
        self.lo
    }

    /// Upper bound of the closed source-edge parameter enclosure.
    pub const fn hi(self) -> f64 {
        self.hi
    }

    /// Whether this enclosure contains `parameter`.
    pub const fn contains(self, parameter: f64) -> bool {
        self.lo <= parameter && parameter <= self.hi
    }
}

/// One stitched section-graph vertex with its per-operand boundary sites.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionVertex {
    pub(crate) point: Point3,
    pub(crate) sites: [SectionSite; 2],
    pub(crate) edge_parameters: [Option<SectionEdgeParameterInterval>; 2],
}

impl SectionVertex {
    /// Numeric representative location (evidence, not a topological claim).
    pub const fn point(&self) -> Point3 {
        self.point
    }

    /// Boundary sites on the first and second operand, in operand order.
    pub const fn sites(&self) -> &[SectionSite; 2] {
        &self.sites
    }

    /// Conservative intrinsic source-edge parameter enclosures in operand
    /// order. A slot is `Some` exactly when the matching [`SectionSite`] is
    /// [`SectionSite::EdgeInterior`] in a certified graph.
    pub const fn edge_parameters(&self) -> &[Option<SectionEdgeParameterInterval>; 2] {
        &self.edge_parameters
    }
}

/// The carrier pcurve of one section edge in one face's surface parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionUvLine {
    pub(crate) origin: Point2,
    pub(crate) direction: Point2,
}

impl SectionUvLine {
    /// UV location corresponding to carrier parameter zero.
    pub const fn origin(&self) -> Point2 {
        self.origin
    }

    /// UV displacement per unit carrier parameter.
    pub const fn direction(&self) -> Point2 {
        self.direction
    }
}

/// One certified section edge lying on one face of each operand.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionEdge {
    pub(crate) faces: [FaceId; 2],
    pub(crate) origin: Point3,
    pub(crate) direction: Vec3,
    pub(crate) range: ParamRange,
    pub(crate) endpoints: [usize; 2],
    pub(crate) uv_lines: [SectionUvLine; 2],
    pub(crate) residual_bounds: [f64; 2],
}

impl SectionEdge {
    /// Carrier faces on the first and second operand, in operand order.
    pub const fn faces(&self) -> &[FaceId; 2] {
        &self.faces
    }

    /// Carrier line origin (parameter zero).
    pub const fn origin(&self) -> Point3 {
        self.origin
    }

    /// Carrier line direction, oriented so that walking the edge keeps the
    /// section loop's canonical traversal (outward normal of the first
    /// operand crossed with outward normal of the second).
    pub const fn direction(&self) -> Vec3 {
        self.direction
    }

    /// Active finite parameter interval on the carrier line.
    pub const fn range(&self) -> ParamRange {
        self.range
    }

    /// Graph vertex indices at the low/high carrier-range endpoints.
    pub const fn endpoints(&self) -> [usize; 2] {
        self.endpoints
    }

    /// Carrier pcurves in the two faces' surface parameters, operand order.
    pub const fn uv_lines(&self) -> &[SectionUvLine; 2] {
        &self.uv_lines
    }

    /// Conservative model-space residual bounds of the carrier against the
    /// two face surfaces, in operand order.
    pub const fn residual_bounds(&self) -> [f64; 2] {
        self.residual_bounds
    }
}

/// Topology of one verified, not-yet-trimmed section carrier branch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SectionBranchTopology {
    /// Distinct low/high fragment sites bound an open carrier interval.
    Open,
    /// The carrier covers one complete period and both endpoint slots share
    /// one intentional parameter-seam site.
    Closed,
}

/// Kernel-facade carrier geometry for a verified section branch.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum SectionCarrier {
    /// Circular carrier.
    Circle {
        /// Circle center.
        center: Point3,
        /// Unit normal of the circle plane.
        normal: Vec3,
        /// Unit direction from the center at parameter zero.
        x_direction: Vec3,
        /// Positive circle radius.
        radius: f64,
    },
}

/// Circular pcurve composed directly with its carrier parameter map.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionUvCircle {
    center: Point2,
    radius: f64,
    x_direction: Vec2,
    parameter_scale: f64,
    parameter_offset: f64,
}

impl SectionUvCircle {
    /// Circle center in surface parameters.
    pub const fn center(self) -> Point2 {
        self.center
    }

    /// Positive parameter-space radius.
    pub const fn radius(self) -> f64 {
        self.radius
    }

    /// Unit direction from the center at pcurve parameter zero.
    pub const fn x_direction(self) -> Vec2 {
        self.x_direction
    }

    /// Carrier-angle multiplier.
    pub const fn parameter_scale(self) -> f64 {
        self.parameter_scale
    }

    /// Carrier-angle phase offset.
    pub const fn parameter_offset(self) -> f64 {
        self.parameter_offset
    }
}

/// Kernel-facade parameter-space carrier trace.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum SectionUvCurve {
    /// Affine parameter-space line composed with the carrier map.
    Line(SectionUvLine),
    /// Parameter-space circle composed with the carrier angle map.
    Circle(SectionUvCircle),
}

/// Kernel-owned summary of the paired whole-range proof.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionBranchEvidence {
    residual_bounds: [f64; 2],
    tolerance: f64,
}

impl SectionBranchEvidence {
    /// Conservative model-space trace residuals in operand order.
    pub const fn residual_bounds(self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Model-space tolerance used by the graph-owned proof.
    pub const fn tolerance(self) -> f64 {
        self.tolerance
    }
}

/// One graph-owned site retained for future trim-fragment assembly.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionFragmentSite {
    point: Point3,
    surface_parameters: [[f64; 2]; 2],
    surface_window_boundaries: [bool; 2],
}

impl SectionFragmentSite {
    /// Model-space representative on the certified carrier.
    pub const fn point(self) -> Point3 {
        self.point
    }

    /// Parameters on the two source face surfaces, in operand order.
    pub const fn surface_parameters(self) -> [[f64; 2]; 2] {
        self.surface_parameters
    }

    /// Which conservative source surface windows contain this site on their
    /// boundary. This is chart evidence, not a trimmed-face boundary claim.
    pub const fn surface_window_boundaries(self) -> [bool; 2] {
        self.surface_window_boundaries
    }
}

/// One certified Plane/Cylinder circle carrier awaiting exact trim clipping.
///
/// These branches are verified partial evidence. They are deliberately kept
/// separate from [`SectionEdge`], whose endpoints and sites already carry
/// certified trimmed-face topology.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionBranch {
    faces: [FaceId; 2],
    carrier: SectionCarrier,
    range: ParamRange,
    topology: SectionBranchTopology,
    pcurves: [SectionUvCurve; 2],
    fragment_sites: Vec<SectionFragmentSite>,
    endpoint_sites: [usize; 2],
    evidence: SectionBranchEvidence,
}

impl SectionBranch {
    /// Source faces in operand order.
    pub const fn faces(&self) -> &[FaceId; 2] {
        &self.faces
    }

    /// Exact model-space carrier through kernel-owned value types.
    pub const fn carrier(&self) -> SectionCarrier {
        self.carrier
    }

    /// Complete finite carrier interval covered by the paired proof.
    pub const fn range(&self) -> ParamRange {
        self.range
    }

    /// Open/closed topology of the carrier interval.
    pub const fn topology(&self) -> SectionBranchTopology {
        self.topology
    }

    /// Exact paired pcurves in operand order.
    pub const fn pcurves(&self) -> &[SectionUvCurve; 2] {
        &self.pcurves
    }

    /// Graph-owned sites retained for later trim-fragment assembly.
    pub fn fragment_sites(&self) -> &[SectionFragmentSite] {
        &self.fragment_sites
    }

    /// Fragment-site indices at the low/high parameter slots. A closed
    /// branch intentionally returns the same site in both slots.
    pub const fn endpoint_sites(&self) -> [usize; 2] {
        self.endpoint_sites
    }

    /// Kernel-owned summary of the graph-owned paired residual proof.
    pub const fn evidence(&self) -> SectionBranchEvidence {
        self.evidence
    }
}

/// One stitched chain of section edges.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionLoop {
    pub(crate) edges: Vec<usize>,
    pub(crate) closed: bool,
}

impl SectionLoop {
    /// Edge indices in traversal order.
    pub fn edges(&self) -> &[usize] {
        &self.edges
    }

    /// Whether the chain closes back onto its first vertex. Open chains
    /// appear only alongside structured gaps.
    pub const fn closed(&self) -> bool {
        self.closed
    }
}

/// One structured reason the section graph is not certified complete.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionGap {
    pub(crate) reason: &'static str,
    pub(crate) faces: Vec<FaceId>,
}

impl SectionGap {
    /// Stable explanation for the refused portion of the graph.
    pub const fn reason(&self) -> &'static str {
        self.reason
    }

    /// Faces the gap applies to: one for a face admission gap, two (operand
    /// order) for a pair-local gap, none for a graph-global gap.
    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }
}

/// Whether the returned section graph is proven to be the complete
/// boundary/boundary intersection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SectionCompletion {
    /// Every candidate face pair resolved and every chain closed.
    Complete,
    /// The graph is verified evidence but provably incomplete or unresolved;
    /// the gaps list the stable reasons.
    Indeterminate,
}

/// Certified section edge graph between two solid bodies.
#[derive(Debug, Clone, PartialEq)]
pub struct BodySectionGraph {
    pub(crate) bodies: [BodyId; 2],
    pub(crate) vertices: Vec<SectionVertex>,
    pub(crate) edges: Vec<SectionEdge>,
    pub(crate) branches: Vec<SectionBranch>,
    pub(crate) loops: Vec<SectionLoop>,
    pub(crate) gaps: Vec<SectionGap>,
    pub(crate) completion: SectionCompletion,
}

impl BodySectionGraph {
    /// Operand bodies, in request order.
    pub const fn bodies(&self) -> &[BodyId; 2] {
        &self.bodies
    }

    /// Stitched vertices in deterministic first-appearance order.
    pub fn vertices(&self) -> &[SectionVertex] {
        &self.vertices
    }

    /// Certified edges in deterministic pair-major, along-carrier order.
    pub fn edges(&self) -> &[SectionEdge] {
        &self.edges
    }

    /// Certified surface-window branches awaiting exact curved trim clipping
    /// and fragment stitching.
    pub fn branches(&self) -> &[SectionBranch] {
        &self.branches
    }

    /// Stitched chains in deterministic discovery order.
    pub fn loops(&self) -> &[SectionLoop] {
        &self.loops
    }

    /// Structured reasons the graph is not certified complete.
    pub fn gaps(&self) -> &[SectionGap] {
        &self.gaps
    }

    /// Completion status of the whole graph.
    pub const fn completion(&self) -> SectionCompletion {
        self.completion
    }
}

pub(crate) const GAP_PLANAR_ONLY: &str =
    "body sectioning is certified only for faces on planar surfaces in this slice";
pub(crate) const GAP_LINE_EDGES_ONLY: &str =
    "body sectioning is certified only for faces bounded by straight line edges";
pub(crate) const GAP_BOUNDED_EDGES_ONLY: &str =
    "body sectioning requires bounded edges with vertices at both ends";
pub(crate) const GAP_NO_LOOPS: &str = "body sectioning requires at least one bounding loop";
pub(crate) const GAP_SHORT_LOOP: &str = "a face boundary loop has fewer than three vertices";
pub(crate) const GAP_COINCIDENT_FACE_PAIR: &str =
    "a coincident face pair carries a two-dimensional contact this slice does not stitch";
pub(crate) const GAP_TANGENT_CONTACT: &str =
    "a face pair meets in an isolated or tangent contact this slice does not stitch";
pub(crate) const GAP_UNORDERED_CROSSINGS: &str =
    "two boundary crossings along a section carrier could not be certifiably ordered";
pub(crate) const GAP_DEGENERATE_VERTEX: &str = "a section-graph vertex has a degree other than two";
pub(crate) const GAP_OPEN_CHAIN: &str = "a stitched section chain did not close";
pub(crate) const GAP_CARRIER_ORIENTATION: &str =
    "a section carrier's canonical orientation could not be certified";
pub(crate) const GAP_PAIR_UNRESOLVED: &str =
    "a candidate face pair returned an indeterminate intersection result";
pub(crate) const GAP_INCOMPATIBLE_EDGE_PARAMETERS: &str =
    "stitched source-edge parameter enclosures are incompatible";
pub(crate) const GAP_CURVED_TRIM_UNRESOLVED: &str =
    "a certified curved section carrier awaits exact trim clipping and fragment stitching";

impl Part<'_> {
    /// Compute the certified section edge graph between two solid bodies of
    /// this part through one facade-owned operation scope.
    ///
    /// The graph is read-only interrogation evidence: no topology is
    /// created or modified. Wrong-part, stale, and identical operand
    /// identities are rejected before the scope starts. Faces outside the
    /// certified planar slice, coincident or tangent face pairs, and any
    /// metric ordering the conservative intervals cannot certify yield
    /// [`SectionCompletion::Indeterminate`] with structured
    /// [`SectionGap`] reasons instead of a guessed graph.
    pub fn section_bodies(
        &self,
        request: SectionBodiesRequest,
    ) -> Result<OperationOutcome<BodySectionGraph>> {
        let SectionBodiesRequest {
            body_a,
            body_b,
            settings,
        } = request;
        self.body(body_a.clone())?;
        self.body(body_b.clone())?;
        if body_a == body_b {
            return Err(Error::Core {
                source: kcore::error::Error::InvalidGeometry {
                    reason: "body sectioning requires two distinct operand bodies",
                },
            });
        }

        let context = settings.context(self.policy)?.with_family_budget_defaults(
            BodySectionBudgetProfile::v1_defaults()
                .overlaid(&kops::intersect::GraphSurfaceBudgetProfile::v1_defaults()),
        );
        let mut scope = OperationScope::new(&context);
        let linear = settings.tolerances().linear();
        let result = section_impl(self, &body_a, &body_b, linear, &mut scope);
        Ok(scope.finish_typed(result))
    }
}

/// Orchestrate admission, broad phase, per-pair intersection, exact clip,
/// and combinatorial stitching for one section query.
///
/// Both operands must be [`BodyKind::Solid`]; anything else is a typed
/// error, exactly like point/body classification. Every face of both bodies
/// runs slice admission; an inadmissible face records one [`SectionGap`] and
/// excludes only the candidate pairs it participates in, so the returned
/// graph is still verified partial evidence. Candidate pair ordinals are
/// assigned A-major over the original stored face lists and are therefore
/// stable under exclusions. Budget and ledger failures propagate as `Err`;
/// they are never converted into graph content.
fn section_impl(
    part: &Part<'_>,
    body_a: &BodyId,
    body_b: &BodyId,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<BodySectionGraph> {
    let store = &part.state.store;
    let part_id = body_a.part();
    require_solid(store, body_a)?;
    require_solid(store, body_b)?;
    let faces_a = read(store.faces_of_body(body_a.raw()))?;
    let faces_b = read(store.faces_of_body(body_b.raw()))?;
    charge(scope, (faces_a.len() + faces_b.len()) as u64)?;

    let mut acc = SectionAccumulator::default();
    let mut examined: u64 = 0;
    collect_plane_cylinder_branches(
        store,
        part_id,
        &faces_a,
        &faces_b,
        &mut examined,
        scope,
        &mut acc,
    )?;
    let admitted_a = admit_faces(store, part_id, &faces_a, linear, scope, &mut acc)?;
    let admitted_b = admit_faces(store, part_id, &faces_b, linear, scope, &mut acc)?;

    for (a_index, slot_a) in admitted_a.iter().enumerate() {
        let Some(face_a) = slot_a else { continue };
        for (b_index, slot_b) in admitted_b.iter().enumerate() {
            let Some(face_b) = slot_b else { continue };
            let ordinal = pair_ordinal(a_index, admitted_b.len(), b_index);
            examined += 1;
            scope
                .ledger_mut()
                .observe(SECTION_FACE_PAIRS, ResourceKind::Items, examined)
                .map_err(Error::from)?;
            charge(scope, 1)?;
            if clip::boxes_certifiably_disjoint(&face_a.prep, &face_b.prep, linear) {
                continue;
            }
            process_pair(store, face_a, face_b, linear, ordinal, scope, &mut acc)?;
        }
    }
    assemble_graph(part_id, [body_a.clone(), body_b.clone()], acc)
}

fn read<T>(result: kcore::error::Result<T>) -> Result<T> {
    result.map_err(|source| Error::InconsistentTopology { source })
}

fn charge(scope: &mut OperationScope<'_, '_>, amount: u64) -> Result<()> {
    scope
        .ledger_mut()
        .charge(SECTION_WORK, amount)
        .map_err(Error::from)
}

/// Require a live solid operand, mirroring point/body classification's
/// admission of solid bodies only.
fn require_solid(store: &Store, body: &BodyId) -> Result<()> {
    let raw = store.get(body.raw()).map_err(|_| Error::StaleEntity {
        kind: EntityKind::Body,
    })?;
    if raw.kind() != BodyKind::Solid {
        return Err(Error::Core {
            source: kcore::error::Error::InvalidGeometry {
                reason: "body sectioning requires solid operand bodies",
            },
        });
    }
    Ok(())
}

/// Deterministic candidate-pair ordinal: A-major row order over the
/// ORIGINAL stored face lists, so ordinals never shift when a face fails
/// admission and its pairs are excluded.
fn pair_ordinal(a_index: usize, b_count: usize, b_index: usize) -> usize {
    a_index * b_count + b_index
}

/// One face admitted to the certified planar slice, with the exact stored
/// plane frame and sense needed for windows and canonical orientation.
struct AdmittedFace {
    prep: clip::PreparedSectionFace,
    facade: FaceId,
    surface: RawSurfaceId,
    frame: Frame,
    sense: Sense,
    /// Conservative UV superset of the trim region; `None` when no finite
    /// superset could be certified (each affected pair then gaps honestly).
    window: Option<[ParamRange; 2]>,
}

/// Facade-typed evidence carried per certified segment, aligned index-for-
/// index with the stitch segment sequence.
struct SegmentGeometry {
    faces: [FaceId; 2],
    origin: Point3,
    direction: Vec3,
    range: ParamRange,
    uv_lines: [SectionUvLine; 2],
    residual_bounds: [f64; 2],
}

/// Deterministic collectors for one section query.
#[derive(Default)]
struct SectionAccumulator {
    segments: Vec<stitch::StitchSegment>,
    geometry: Vec<SegmentGeometry>,
    branches: Vec<SectionBranch>,
    gaps: Vec<SectionGap>,
}

impl SectionAccumulator {
    fn pair_gap(&mut self, reason: &'static str, a: &AdmittedFace, b: &AdmittedFace) {
        self.gaps.push(SectionGap {
            reason,
            faces: vec![a.facade.clone(), b.facade.clone()],
        });
    }
}

/// Collect proof-bearing Plane/Cylinder circle carriers independently of the
/// planar trim/stitch admission path.
///
/// Face domains are conservative source-owned surface windows. They are
/// sufficient for analytic branch discovery and paired trace proof, but not
/// for claiming membership in the exact trimmed face. Every retained branch
/// therefore carries an explicit curved-trim gap and cannot contribute to a
/// `Complete` body section graph yet.
#[allow(clippy::too_many_arguments)]
fn collect_plane_cylinder_branches(
    store: &Store,
    part_id: &PartId,
    faces_a: &[RawFaceId],
    faces_b: &[RawFaceId],
    examined: &mut u64,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut SectionAccumulator,
) -> Result<()> {
    for &raw_a in faces_a {
        let face_a = read(store.get(raw_a))?;
        let surface_a = read(store.surface(face_a.surface))?;
        for &raw_b in faces_b {
            let face_b = read(store.get(raw_b))?;
            let surface_b = read(store.surface(face_b.surface))?;
            if !matches!(
                (surface_a, surface_b),
                (SurfaceGeom::Plane(_), SurfaceGeom::Cylinder(_))
                    | (SurfaceGeom::Cylinder(_), SurfaceGeom::Plane(_))
            ) {
                continue;
            }
            *examined += 1;
            scope
                .ledger_mut()
                .observe(SECTION_FACE_PAIRS, ResourceKind::Items, *examined)
                .map_err(Error::from)?;
            charge(scope, 1)?;
            let facades = [
                FaceId::new(part_id.clone(), raw_a),
                FaceId::new(part_id.clone(), raw_b),
            ];
            let (Some(domain_a), Some(domain_b)) = (face_a.domain(), face_b.domain()) else {
                acc.gaps.push(SectionGap {
                    reason: GAP_PAIR_UNRESOLVED,
                    faces: facades.to_vec(),
                });
                continue;
            };
            let intersections = match intersect_bounded_graph_surfaces_in_scope(
                store.geometry(),
                face_a.surface,
                [domain_a.u, domain_a.v],
                face_b.surface,
                [domain_b.u, domain_b.v],
                scope,
            ) {
                Ok(intersections) => intersections,
                Err(error) => {
                    if let Some(error) = lift_limit_error(error) {
                        return Err(error);
                    }
                    acc.gaps.push(SectionGap {
                        reason: GAP_PAIR_UNRESOLVED,
                        faces: facades.to_vec(),
                    });
                    continue;
                }
            };
            if !intersections.raw.is_complete()
                || !intersections.raw.points.is_empty()
                || !intersections.raw.regions.is_empty()
            {
                acc.gaps.push(SectionGap {
                    reason: GAP_PAIR_UNRESOLVED,
                    faces: facades.to_vec(),
                });
                continue;
            }
            for edge in &intersections.branch_graph.edges {
                let Some(branch) = adapt_plane_cylinder_branch(
                    &facades,
                    edge,
                    &intersections.branch_graph.vertices,
                ) else {
                    acc.gaps.push(SectionGap {
                        reason: GAP_PAIR_UNRESOLVED,
                        faces: facades.to_vec(),
                    });
                    continue;
                };
                acc.branches.push(branch);
                acc.gaps.push(SectionGap {
                    reason: GAP_CURVED_TRIM_UNRESOLVED,
                    faces: facades.to_vec(),
                });
            }
        }
    }
    Ok(())
}

/// Adapt one whole-period graph branch without turning its chart seam into
/// two physical endpoints.
fn adapt_plane_cylinder_branch(
    faces: &[FaceId; 2],
    edge: &IntersectionBranchEdge,
    vertices: &[kops::intersect::IntersectionBranchVertex],
) -> Option<SectionBranch> {
    if edge.topology != IntersectionBranchTopology::Closed
        || edge.endpoint_vertices[0] != edge.endpoint_vertices[1]
        || edge.kind != ContactKind::Transverse
    {
        return None;
    }
    let CurveDescriptor::Circle(carrier) = edge.carrier else {
        return None;
    };
    let certificate = edge.certificate.as_plane_cylinder_circle()?;
    let pcurves = [
        adapt_branch_pcurve(&edge.pcurves[0], edge.parameter_maps[0])?,
        adapt_branch_pcurve(&edge.pcurves[1], edge.parameter_maps[1])?,
    ];
    let vertex = *vertices.get(edge.endpoint_vertices[0])?;
    let IntersectionBranchVertexEvent::PeriodSeam { surfaces } = vertex.event else {
        return None;
    };
    Some(SectionBranch {
        faces: faces.clone(),
        carrier: SectionCarrier::Circle {
            center: carrier.frame().origin(),
            normal: carrier.frame().z(),
            x_direction: carrier.frame().x(),
            radius: carrier.radius(),
        },
        range: edge.carrier_range,
        topology: SectionBranchTopology::Closed,
        pcurves,
        fragment_sites: vec![SectionFragmentSite {
            point: vertex.point,
            surface_parameters: vertex.surface_parameters,
            surface_window_boundaries: surfaces,
        }],
        endpoint_sites: [0, 0],
        evidence: SectionBranchEvidence {
            residual_bounds: certificate.residual_bounds(),
            tolerance: certificate.tolerance(),
        },
    })
}

/// Compose graph-owned pcurve geometry with its carrier map into facade-owned
/// exact values. Unsupported descriptor families fail closed.
fn adapt_branch_pcurve(
    descriptor: &kgraph::Curve2dDescriptor,
    map: AffineParamMap1d,
) -> Option<SectionUvCurve> {
    if let Some(line) = descriptor.as_line() {
        return Some(SectionUvCurve::Line(compose_uv_line(
            line.origin(),
            line.dir(),
            map,
            false,
        )));
    }
    let circle = descriptor.as_circle()?;
    Some(SectionUvCurve::Circle(SectionUvCircle {
        center: circle.center(),
        radius: circle.radius(),
        x_direction: circle.x_dir(),
        parameter_scale: map.scale(),
        parameter_offset: map.offset(),
    }))
}

/// Run slice admission over one body's stored face list.
///
/// The returned vector is aligned with `faces` so pair ordinals stay stable;
/// an inadmissible face records its gap and leaves `None` in its slot.
fn admit_faces(
    store: &Store,
    part_id: &PartId,
    faces: &[RawFaceId],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut SectionAccumulator,
) -> Result<Vec<Option<AdmittedFace>>> {
    let mut admitted = Vec::with_capacity(faces.len());
    for &raw in faces {
        let facade = FaceId::new(part_id.clone(), raw);
        match clip::prepare_section_face(store, raw, linear, scope)? {
            Err(reason) => {
                acc.gaps.push(SectionGap {
                    reason,
                    faces: vec![facade],
                });
                admitted.push(None);
            }
            Ok(prep) => {
                let face = read(store.get(raw))?;
                let SurfaceGeom::Plane(plane) = read(store.surface(face.surface))? else {
                    acc.gaps.push(SectionGap {
                        reason: GAP_PLANAR_ONLY,
                        faces: vec![facade],
                    });
                    admitted.push(None);
                    continue;
                };
                let frame = *plane.frame();
                let window = face_window(&frame, &prep, linear);
                admitted.push(Some(AdmittedFace {
                    prep,
                    facade,
                    surface: face.surface,
                    frame,
                    sense: face.sense,
                    window,
                }));
            }
        }
    }
    Ok(admitted)
}

/// Conservative inflation of a face's UV window as a fraction of the larger
/// window dimension. The window only needs to be a superset of the trim
/// region — the exact clip decides true topology — so generosity is safe.
const WINDOW_INFLATION: f64 = 1.0 / 1024.0;

/// Conservative UV superset of the face's trim region in its plane frame.
///
/// Every prepared ring vertex projects onto the frame's x/y axes with
/// interval arithmetic; because projection is affine and every boundary edge
/// is a straight line, the outward-rounded hull of the vertex projections
/// contains the whole trimmed face. The hull is then inflated by
/// `max(linear, WINDOW_INFLATION * max dimension)` and never shrunk.
fn face_window(
    frame: &Frame,
    prep: &clip::PreparedSectionFace,
    linear: f64,
) -> Option<[ParamRange; 2]> {
    let origin = frame.origin();
    let axes = [frame.x(), frame.y()];
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for ring in &prep.rings {
        for vertex in &ring.vertices {
            let offset = [
                Interval::point(vertex.point[0]) - Interval::point(origin.x),
                Interval::point(vertex.point[1]) - Interval::point(origin.y),
                Interval::point(vertex.point[2]) - Interval::point(origin.z),
            ];
            for (uv_axis, axis) in axes.iter().enumerate() {
                let along = offset[0] * Interval::point(axis.x)
                    + offset[1] * Interval::point(axis.y)
                    + offset[2] * Interval::point(axis.z);
                lo[uv_axis] = lo[uv_axis].min(along.lo());
                hi[uv_axis] = hi[uv_axis].max(along.hi());
            }
        }
    }
    let diameter = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    let pad = linear.max(diameter * WINDOW_INFLATION);
    let mut window = [ParamRange { lo: 0.0, hi: 0.0 }; 2];
    for uv_axis in 0..2 {
        let low = (lo[uv_axis] - pad).next_down();
        let high = (hi[uv_axis] + pad).next_up();
        if !low.is_finite() || !high.is_finite() || low >= high {
            return None;
        }
        window[uv_axis] = ParamRange { lo: low, hi: high };
    }
    Some(window)
}

/// Exact outward normal of a face on a plane: the stored frame normal kept
/// for `Sense::Forward` and negated (exactly) for `Sense::Reversed`.
fn outward_normal(frame: &Frame, sense: Sense) -> [f64; 3] {
    let z = frame.z();
    let flip = if sense.is_forward() { 1.0 } else { -1.0 };
    [flip * z.x, flip * z.y, flip * z.z]
}

/// Certified sign of `dot(direction, normal_a × normal_b)` under interval
/// arithmetic. `Some(true)`/`Some(false)` are proven positive/negative;
/// `None` means the enclosure straddles zero and no orientation is claimed.
fn certified_carrier_sign(
    direction: [f64; 3],
    normal_a: [f64; 3],
    normal_b: [f64; 3],
) -> Option<bool> {
    let a = normal_a.map(Interval::point);
    let b = normal_b.map(Interval::point);
    let cross = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    let mut dot = Interval::point(0.0);
    for axis in 0..3 {
        dot = dot + Interval::point(direction[axis]) * cross[axis];
    }
    if dot.lo() > 0.0 {
        Some(true)
    } else if dot.hi() < 0.0 {
        Some(false)
    } else {
        None
    }
}

/// Compose one branch pcurve line with its carrier parameter map into the
/// facade UV line over the CANONICAL carrier parameter `t'`.
///
/// The branch supplies `uv(t) = origin + direction * (scale * t + offset)`
/// over the branch parameter `t`. Canonicalization either keeps `t' = t`
/// (`flipped == false`) or negates it, `t' = -t` (`flipped == true`). In
/// both cases the UV origin is the point at `t' = 0` (== `t = 0`), so only
/// the per-unit displacement changes sign:
/// `uv(t') = (origin + direction * offset) + direction * (±scale) * t'`.
fn compose_uv_line(
    origin: Point2,
    direction: Vec2,
    map: AffineParamMap1d,
    flipped: bool,
) -> SectionUvLine {
    let scale = if flipped { -map.scale() } else { map.scale() };
    SectionUvLine {
        origin: origin + direction * map.offset(),
        direction: direction * scale,
    }
}

/// UV line of one operand of a branch, `None` when the pcurve is not the
/// straight line the planar slice requires.
fn branch_uv_line(
    branch: &IntersectionBranchEdge,
    operand: usize,
    flipped: bool,
) -> Option<SectionUvLine> {
    let pcurve = branch.pcurves[operand].as_line()?;
    Some(compose_uv_line(
        pcurve.origin(),
        pcurve.dir(),
        branch.parameter_maps[operand],
        flipped,
    ))
}

/// Lift a graph-surface intersection failure into a kernel error when it is
/// a budget/ledger crossing, which must never become graph content. Every
/// other failure is pair-local evidence and returns `None` so the caller
/// records an honest pair gap instead.
fn lift_limit_error(error: GraphSurfaceIntersectionError) -> Option<Error> {
    match error {
        GraphSurfaceIntersectionError::OperationPolicy(source) => Some(Error::from(source)),
        GraphSurfaceIntersectionError::Intersection(source) => {
            if source.limit().is_some() {
                Some(Error::from_intersection(source))
            } else {
                None
            }
        }
        GraphSurfaceIntersectionError::GeometryEvaluation(source) => {
            if source.limit().is_some() {
                Some(Error::from_graph(source))
            } else {
                None
            }
        }
        _ => None,
    }
}

/// Canonically oriented carrier evidence for one surviving candidate pair.
struct PairCarrier {
    carrier: clip::SectionCarrierLine,
    uv_lines: [SectionUvLine; 2],
    residual_bounds: [f64; 2],
}

/// Outcome of resolving one candidate pair's certified carrier.
enum PairResolution {
    /// Exactly one certified transverse line branch, canonically oriented.
    Carrier(PairCarrier),
    /// A certified complete empty intersection: nothing to record.
    Empty,
    /// The pair cannot be stitched in this slice; stable reason.
    Gap(&'static str),
}

/// Resolve one candidate pair through the certified plane/plane
/// intersection and canonicalize the carrier orientation.
///
/// Canonical convention: the carrier direction satisfies a certified
/// `dot(direction, n_a × n_b) > 0` where `n_a`/`n_b` are the operands'
/// exact outward face normals. A negative certified sign flips the working
/// carrier before clipping (so all clip parameters are already canonical)
/// and negates the UV parameter maps consistently.
fn resolve_pair_carrier(
    store: &Store,
    a: &AdmittedFace,
    b: &AdmittedFace,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PairResolution> {
    let (Some(window_a), Some(window_b)) = (a.window, b.window) else {
        return Ok(PairResolution::Gap(GAP_PAIR_UNRESOLVED));
    };
    let intersections = match intersect_bounded_graph_surfaces_in_scope(
        store.geometry(),
        a.surface,
        window_a,
        b.surface,
        window_b,
        scope,
    ) {
        Ok(value) => value,
        Err(error) => {
            return match lift_limit_error(error) {
                Some(lifted) => Err(lifted),
                None => Ok(PairResolution::Gap(GAP_PAIR_UNRESOLVED)),
            };
        }
    };
    if !intersections.raw.is_complete() {
        return Ok(PairResolution::Gap(GAP_PAIR_UNRESOLVED));
    }
    if !intersections.raw.regions.is_empty() {
        return Ok(PairResolution::Gap(GAP_COINCIDENT_FACE_PAIR));
    }
    let branches = &intersections.branch_graph.edges;
    if !intersections.raw.points.is_empty()
        || branches
            .iter()
            .any(|branch| branch.kind != ContactKind::Transverse)
    {
        return Ok(PairResolution::Gap(GAP_TANGENT_CONTACT));
    }
    if branches.is_empty() {
        return Ok(PairResolution::Empty);
    }
    let [branch] = branches.as_slice() else {
        return Ok(PairResolution::Gap(GAP_PAIR_UNRESOLVED));
    };
    let Some(line) = branch.carrier.as_line() else {
        return Ok(PairResolution::Gap(GAP_PAIR_UNRESOLVED));
    };
    let direction = [line.dir().x, line.dir().y, line.dir().z];
    let positive = match certified_carrier_sign(
        direction,
        outward_normal(&a.frame, a.sense),
        outward_normal(&b.frame, b.sense),
    ) {
        Some(positive) => positive,
        None => return Ok(PairResolution::Gap(GAP_CARRIER_ORIENTATION)),
    };
    let flipped = !positive;
    let carrier_direction = if flipped {
        [-direction[0], -direction[1], -direction[2]]
    } else {
        direction
    };
    let (Some(uv_a), Some(uv_b)) = (
        branch_uv_line(branch, 0, flipped),
        branch_uv_line(branch, 1, flipped),
    ) else {
        return Ok(PairResolution::Gap(GAP_PAIR_UNRESOLVED));
    };
    Ok(PairResolution::Carrier(PairCarrier {
        carrier: clip::SectionCarrierLine {
            origin: [line.origin().x, line.origin().y, line.origin().z],
            direction: carrier_direction,
        },
        uv_lines: [uv_a, uv_b],
        residual_bounds: branch.certificate.residual_bounds(),
    }))
}

/// Numeric representative of the carrier at one canonical parameter.
fn carrier_point(carrier: &clip::SectionCarrierLine, parameter: f64) -> [f64; 3] {
    [
        carrier.origin[0] + carrier.direction[0] * parameter,
        carrier.origin[1] + carrier.direction[1] * parameter,
        carrier.origin[2] + carrier.direction[2] * parameter,
    ]
}

/// Deterministic midpoint of a conservative parameter enclosure.
fn interval_midpoint(interval: Interval) -> f64 {
    0.5 * (interval.lo() + interval.hi())
}

/// Combinatorial stitching key of one merged-span endpoint on one operand:
/// `None` means the carrier stays inside that operand's face there.
fn site_key(site: Option<clip::CrossingSite>, face: RawFaceId) -> stitch::SiteKey {
    match site {
        None => stitch::SiteKey::Face(face),
        Some(clip::CrossingSite::EdgeInterior(edge)) => stitch::SiteKey::Edge(edge),
        Some(clip::CrossingSite::AtVertex(vertex)) => stitch::SiteKey::Vertex(vertex),
    }
}

fn span_vertex_key(
    endpoint: &clip::MergedEndpoint,
    face_a: RawFaceId,
    face_b: RawFaceId,
) -> stitch::VertexKey {
    stitch::VertexKey {
        a: site_key(endpoint.a, face_a),
        b: site_key(endpoint.b, face_b),
    }
}

/// Wrap one stitch site key into a part-qualified facade site.
fn adapt_site(part: &PartId, key: stitch::SiteKey) -> SectionSite {
    match key {
        stitch::SiteKey::Face(face) => SectionSite::FaceInterior(FaceId::new(part.clone(), face)),
        stitch::SiteKey::Edge(edge) => SectionSite::EdgeInterior(EdgeId::new(part.clone(), edge)),
        stitch::SiteKey::Vertex(vertex) => {
            SectionSite::AtVertex(VertexId::new(part.clone(), vertex))
        }
    }
}

/// Intersect, clip, and merge one surviving candidate pair, appending its
/// certified spans as stitch segments in canonical along-carrier order.
fn process_pair(
    store: &Store,
    a: &AdmittedFace,
    b: &AdmittedFace,
    linear: f64,
    ordinal: usize,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut SectionAccumulator,
) -> Result<()> {
    let pair = match resolve_pair_carrier(store, a, b, scope)? {
        PairResolution::Carrier(pair) => pair,
        PairResolution::Empty => return Ok(()),
        PairResolution::Gap(reason) => {
            acc.pair_gap(reason, a, b);
            return Ok(());
        }
    };
    let spans_a =
        match clip::clip_face_with_plane(&a.prep, &pair.carrier, &b.prep.witness, linear, scope)? {
            clip::ClipOutcome::Spans(spans) => spans,
            clip::ClipOutcome::Gap(reason) => {
                acc.pair_gap(reason, a, b);
                return Ok(());
            }
        };
    let spans_b =
        match clip::clip_face_with_plane(&b.prep, &pair.carrier, &a.prep.witness, linear, scope)? {
            clip::ClipOutcome::Spans(spans) => spans,
            clip::ClipOutcome::Gap(reason) => {
                acc.pair_gap(reason, a, b);
                return Ok(());
            }
        };
    let merged = match clip::merge_clip_spans(&spans_a, &spans_b, scope)? {
        clip::MergeOutcome::Spans(spans) => spans,
        clip::MergeOutcome::Gap(reason) => {
            acc.pair_gap(reason, a, b);
            return Ok(());
        }
    };
    for span in &merged {
        let start = interval_midpoint(span.start.parameter);
        let end = interval_midpoint(span.end.parameter);
        if !start.is_finite() || !end.is_finite() || start > end {
            acc.pair_gap(GAP_UNORDERED_CROSSINGS, a, b);
            continue;
        }
        acc.segments.push(stitch::StitchSegment {
            pair: ordinal,
            faces: [a.prep.raw, b.prep.raw],
            start: span_vertex_key(&span.start, a.prep.raw, b.prep.raw),
            end: span_vertex_key(&span.end, a.prep.raw, b.prep.raw),
            start_point: carrier_point(&pair.carrier, start),
            end_point: carrier_point(&pair.carrier, end),
            start_edge_parameters: span.start.edge_parameters,
            end_edge_parameters: span.end.edge_parameters,
        });
        acc.geometry.push(SegmentGeometry {
            faces: [a.facade.clone(), b.facade.clone()],
            origin: Point3::new(
                pair.carrier.origin[0],
                pair.carrier.origin[1],
                pair.carrier.origin[2],
            ),
            direction: Vec3::new(
                pair.carrier.direction[0],
                pair.carrier.direction[1],
                pair.carrier.direction[2],
            ),
            range: ParamRange::new(start, end),
            uv_lines: pair.uv_lines,
            residual_bounds: pair.residual_bounds,
        });
    }
    Ok(())
}

/// Stitch the accumulated segments and wrap the result in facade types.
///
/// Structural stitch defects become graph-global gaps; completion is
/// `Complete` exactly when no gap of any kind was recorded.
fn assemble_graph(
    part_id: &PartId,
    bodies: [BodyId; 2],
    acc: SectionAccumulator,
) -> Result<BodySectionGraph> {
    let SectionAccumulator {
        segments,
        geometry,
        branches,
        mut gaps,
    } = acc;
    let stitched = stitch::stitch_segments(&segments);
    let vertices = stitched
        .vertices
        .iter()
        .map(|vertex| SectionVertex {
            point: Point3::new(vertex.point[0], vertex.point[1], vertex.point[2]),
            sites: [
                adapt_site(part_id, vertex.key.a),
                adapt_site(part_id, vertex.key.b),
            ],
            edge_parameters: vertex
                .edge_parameters
                .map(|parameter| parameter.map(SectionEdgeParameterInterval::from_interval)),
        })
        .collect();
    let mut edges = Vec::with_capacity(stitched.edges.len());
    for edge in &stitched.edges {
        let geom = geometry
            .get(edge.segment)
            .ok_or(Error::InconsistentTopology {
                source: kcore::error::Error::InvalidGeometry {
                    reason: "section stitching referenced an unknown segment index",
                },
            })?;
        edges.push(SectionEdge {
            faces: geom.faces.clone(),
            origin: geom.origin,
            direction: geom.direction,
            range: geom.range,
            endpoints: edge.endpoints,
            uv_lines: geom.uv_lines,
            residual_bounds: geom.residual_bounds,
        });
    }
    let loops = stitched
        .chains
        .iter()
        .map(|chain| SectionLoop {
            edges: chain.edges.clone(),
            closed: chain.closed,
        })
        .collect();
    for defect in &stitched.defects {
        gaps.push(SectionGap {
            reason: stitch_defect_reason(*defect),
            faces: Vec::new(),
        });
    }
    let completion = if gaps.is_empty() {
        SectionCompletion::Complete
    } else {
        SectionCompletion::Indeterminate
    };
    Ok(BodySectionGraph {
        bodies,
        vertices,
        edges,
        branches,
        loops,
        gaps,
        completion,
    })
}

/// Stable public graph-gap reason corresponding to one internal stitch
/// defect. Keeping this total mapping separate makes evidence incompatibility
/// impossible to drop while assembling a partial graph.
fn stitch_defect_reason(defect: stitch::StitchDefect) -> &'static str {
    match defect {
        stitch::StitchDefect::DegreeNotTwo(_) => GAP_DEGENERATE_VERTEX,
        stitch::StitchDefect::OpenChain(_) => GAP_OPEN_CHAIN,
        stitch::StitchDefect::IncompatibleEdgeParameter(_) => GAP_INCOMPATIBLE_EDGE_PARAMETERS,
    }
}

#[cfg(test)]
mod unit_tests {
    use kgeom::frame::Frame;
    use kgeom::vec::Point3;

    use super::*;
    use crate::{Kernel, KernelError, Session};

    fn block_part() -> (Session, PartId, BodyId) {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let raw = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            ktopo::make::block(edit.store_mut_for_test(), &Frame::world(), [2.0, 2.0, 2.0]).unwrap()
        };
        (session, part_id.clone(), BodyId::new(part_id, raw))
    }

    #[test]
    fn incompatible_edge_parameter_stitch_defect_maps_to_stable_graph_gap() {
        assert_eq!(
            stitch_defect_reason(stitch::StitchDefect::IncompatibleEdgeParameter(7)),
            GAP_INCOMPATIBLE_EDGE_PARAMETERS
        );
    }

    #[test]
    fn identical_operand_bodies_are_rejected_before_the_scope_starts() {
        let (session, part_id, body) = block_part();
        let part = session.part(part_id).unwrap();
        let result = part.section_bodies(SectionBodiesRequest::new(body.clone(), body));
        assert!(matches!(result, Err(KernelError::Core { .. })));
    }

    #[test]
    fn wrong_part_operand_bodies_are_rejected_before_the_scope_starts() {
        let mut session = Kernel::new().create_session();
        let part_a = session.create_part();
        let part_b = session.create_part();
        let body_a = {
            let mut edit = session.edit_part(part_a.clone()).unwrap();
            let raw =
                ktopo::make::block(edit.store_mut_for_test(), &Frame::world(), [2.0, 2.0, 2.0])
                    .unwrap();
            BodyId::new(part_a.clone(), raw)
        };
        let body_b = {
            let mut edit = session.edit_part(part_b.clone()).unwrap();
            let raw =
                ktopo::make::block(edit.store_mut_for_test(), &Frame::world(), [2.0, 2.0, 2.0])
                    .unwrap();
            BodyId::new(part_b, raw)
        };
        let part = session.part(part_a).unwrap();
        let result = part.section_bodies(SectionBodiesRequest::new(body_a, body_b));
        assert!(matches!(result, Err(KernelError::WrongPart { .. })));
    }

    #[test]
    fn non_solid_operand_bodies_are_rejected_inside_the_scope() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (solid, acorn) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let store = edit.store_mut_for_test();
            let solid = ktopo::make::block(store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
            let acorn = ktopo::make::acorn(store, Point3::new(5.0, 0.0, 0.0)).unwrap();
            (
                BodyId::new(part_id.clone(), solid),
                BodyId::new(part_id.clone(), acorn),
            )
        };
        let part = session.part(part_id).unwrap();
        let outcome = part
            .section_bodies(SectionBodiesRequest::new(solid, acorn))
            .unwrap();
        assert!(matches!(
            outcome.into_result(),
            Err(KernelError::Core { .. })
        ));
    }

    #[test]
    fn uv_line_flip_composition_matches_hand_computed_affine_cases() {
        // uv(t) = (1, -1) + (0.5, 0.25) * (2t + 3).
        let map = AffineParamMap1d::new(2.0, 3.0).unwrap();
        let kept = compose_uv_line(Point2::new(1.0, -1.0), Vec2::new(0.5, 0.25), map, false);
        assert_eq!(kept.origin(), Point2::new(2.5, -0.25));
        assert_eq!(kept.direction(), Point2::new(1.0, 0.5));

        // Flip (t' = -t): the origin is the same t = 0 point and only the
        // per-unit displacement negates.
        let flipped = compose_uv_line(Point2::new(1.0, -1.0), Vec2::new(0.5, 0.25), map, true);
        assert_eq!(flipped.origin(), Point2::new(2.5, -0.25));
        assert_eq!(flipped.direction(), Point2::new(-1.0, -0.5));

        // Independent oracle: for every t', flipped(t') == kept(-t').
        for parameter in [-2.0, -0.5, 0.0, 1.5, 4.0] {
            let via_flip = flipped.origin() + flipped.direction() * parameter;
            let via_negation = kept.origin() + kept.direction() * (-parameter);
            assert_eq!(via_flip, via_negation);
        }

        // Negative-scale map composed with a flip turns increasing again.
        let reversed = AffineParamMap1d::new(-1.5, 0.5).unwrap();
        let unflipped =
            compose_uv_line(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0), reversed, false);
        let reflipped = compose_uv_line(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0), reversed, true);
        assert_eq!(unflipped.direction(), Point2::new(-1.5, 0.0));
        assert_eq!(reflipped.direction(), Point2::new(1.5, 0.0));
        assert_eq!(unflipped.origin(), reflipped.origin());
    }

    #[test]
    fn stitch_site_keys_adapt_to_part_qualified_facade_ids() {
        let (session, part_id, body) = block_part();
        let part = session.part(part_id.clone()).unwrap();
        let store = &part.state.store;
        let face = store.faces_of_body(body.raw()).unwrap()[0];
        let edge = store.edges_of_body(body.raw()).unwrap()[0];
        let vertex = store.get(edge).unwrap().vertices[0].unwrap();

        assert_eq!(
            adapt_site(&part_id, stitch::SiteKey::Face(face)),
            SectionSite::FaceInterior(FaceId::new(part_id.clone(), face))
        );
        assert_eq!(
            adapt_site(&part_id, stitch::SiteKey::Edge(edge)),
            SectionSite::EdgeInterior(EdgeId::new(part_id.clone(), edge))
        );
        assert_eq!(
            adapt_site(&part_id, stitch::SiteKey::Vertex(vertex)),
            SectionSite::AtVertex(VertexId::new(part_id.clone(), vertex))
        );
    }

    #[test]
    fn pair_ordinals_are_stable_when_faces_are_excluded() {
        // Row-major over the ORIGINAL lists.
        assert_eq!(pair_ordinal(0, 3, 0), 0);
        assert_eq!(pair_ordinal(0, 3, 2), 2);
        assert_eq!(pair_ordinal(1, 3, 0), 3);
        assert_eq!(pair_ordinal(1, 3, 2), 5);

        // Excluding faces skips their pairs without renumbering survivors.
        let admitted_a = [false, true];
        let admitted_b = [true, false, true];
        let mut ordinals = Vec::new();
        for (a_index, &a_ok) in admitted_a.iter().enumerate() {
            if !a_ok {
                continue;
            }
            for (b_index, &b_ok) in admitted_b.iter().enumerate() {
                if !b_ok {
                    continue;
                }
                ordinals.push(pair_ordinal(a_index, admitted_b.len(), b_index));
            }
        }
        assert_eq!(ordinals, vec![3, 5]);
    }

    #[test]
    fn carrier_orientation_sign_is_certified_or_refused() {
        let z = [0.0, 0.0, 1.0];
        let x = [1.0, 0.0, 0.0];
        // z × x = +y: aligned carriers certify positive, opposed negative.
        assert_eq!(certified_carrier_sign([0.0, 1.0, 0.0], z, x), Some(true));
        assert_eq!(certified_carrier_sign([0.0, -1.0, 0.0], z, x), Some(false));
        // Parallel outward normals have a zero cross product: refused.
        assert_eq!(certified_carrier_sign([1.0, 0.0, 0.0], z, z), None);
        assert_eq!(certified_carrier_sign([0.0, 1.0, 0.0], z, z), None);
    }
}
