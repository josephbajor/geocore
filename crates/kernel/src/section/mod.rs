//! Certified section evidence between two solid bodies.
//!
//! Second rung of the boolean ladder: `unite`/`subtract`/`intersect` need the
//! curves where the two operand boundaries meet, stitched into a coherent
//! edge graph whose vertices sit on the operands' own edges and vertices.
//! This module computes that graph for the planar slice — every face on a
//! plane, every edge a bounded straight line. It also clips complete-period
//! Plane/Cylinder circle carriers against topology-owned polygon/ring trims
//! and retains both intact carriers and certified bounded arcs as exact
//! curved fragments. Their endpoint joins use source edge/root identities;
//! carrier points are diagnostic representatives only.
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

mod broad_phase;
mod clip;
mod closed_stitch;
mod curved_clip;
mod stitch;

#[cfg(test)]
mod tests;

use kcore::interval::Interval;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::predicates::{Orientation, affine_dot3};
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
use crate::{BodyId, EdgeId, EntityKind, FaceId, FinId, LoopId, PartId, VertexId};

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

/// Topology of one verified section carrier branch.
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

/// One certified Plane/Cylinder circle carrier.
///
/// These branches are deliberately kept separate from [`SectionEdge`], whose
/// endpoints and sites carry bounded trimmed-face topology. A matching
/// [`SectionRing`] proves that exact trimming retained this complete-period
/// carrier; a branch without a ring remains verified intersection evidence,
/// with unresolved trim/fragment work represented by graph gaps.
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

/// One endpoint-free closed section component carried by a complete-period
/// curved branch.
///
/// The referenced branch retains the exact carrier, paired pcurves, source
/// faces, intentional chart seam, and residual proof. A ring is emitted only
/// after both exact face trims certify the whole branch and the closed
/// fragment stitcher accepts it; the chart seam is never promoted to a
/// physical vertex.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SectionRing {
    branch: usize,
}

impl SectionRing {
    /// Index into [`BodySectionGraph::branches`].
    pub const fn branch(self) -> usize {
        self.branch
    }
}

/// Conservative enclosure of one projective pcurve half-angle.
///
/// The exact curved clipper orders trim roots by disjoint intervals in
/// `y = tan(q / 2)`. This enclosure is therefore topological ordering
/// evidence; a rounded carrier angle is never substituted for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionProjectiveParameterInterval {
    lo: f64,
    hi: f64,
}

impl SectionProjectiveParameterInterval {
    fn from_interval(interval: Interval) -> Self {
        Self {
            lo: interval.lo(),
            hi: interval.hi(),
        }
    }

    /// Lower bound of the closed projective-parameter enclosure.
    pub const fn lo(self) -> f64 {
        self.lo
    }

    /// Upper bound of the closed projective-parameter enclosure.
    pub const fn hi(self) -> f64 {
        self.hi
    }
}

/// Stable proof identity of one isolated root on a source topology edge.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct SectionSourceParameterKey {
    edge: EdgeId,
    root_ordinal: usize,
}

impl SectionSourceParameterKey {
    /// Source edge containing the isolated root.
    pub fn edge(&self) -> EdgeId {
        self.edge.clone()
    }

    /// Ordinal in certified intrinsic source-edge parameter order.
    pub const fn root_ordinal(&self) -> usize {
        self.root_ordinal
    }
}

/// Exact combinatorial identity of one stitched curved-fragment endpoint.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum SectionCurveEndpointTopology {
    /// Physical trim event on one or both operand boundaries.
    Trim {
        /// Operand-local boundary sites.
        sites: [SectionSite; 2],
        /// Isolated source-edge roots exactly where `sites` names an edge.
        source_parameters: [Option<SectionSourceParameterKey>; 2],
    },
    /// Intentional parameter seam of a complete-period carrier.
    ParameterSeam {
        /// Index into [`BodySectionGraph::branches`].
        branch: usize,
        /// Index into [`SectionBranch::fragment_sites`].
        site: usize,
    },
}

/// One proof-keyed vertex shared by stitched curved fragments.
///
/// Equality and joins are owned by [`SectionCurveEndpointTopology`], never
/// by a metric point comparison. Source-edge intervals are intersected only
/// after those exact identities match.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionCurveEndpoint {
    topology: SectionCurveEndpointTopology,
    edge_parameters: [Option<SectionEdgeParameterInterval>; 2],
}

impl SectionCurveEndpoint {
    /// Exact combinatorial endpoint identity.
    pub const fn topology(&self) -> &SectionCurveEndpointTopology {
        &self.topology
    }

    /// Compatible intrinsic source-edge parameter enclosures by operand.
    pub const fn edge_parameters(&self) -> &[Option<SectionEdgeParameterInterval>; 2] {
        &self.edge_parameters
    }
}

/// Topology-owned trim event that bounds one curved fragment occurrence.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionCurveTrimProvenance {
    operand: usize,
    face: FaceId,
    loop_id: LoopId,
    fin: FinId,
    source_parameter: SectionSourceParameterKey,
    edge_parameter: SectionEdgeParameterInterval,
    pcurve_half_angle: SectionProjectiveParameterInterval,
}

impl SectionCurveTrimProvenance {
    /// Operand slot whose face trim contributed this event.
    pub const fn operand(&self) -> usize {
        self.operand
    }

    /// Trimmed source face.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// Source boundary loop.
    pub fn loop_id(&self) -> LoopId {
        self.loop_id.clone()
    }

    /// Source fin whose pcurve supplied the crossing equation.
    pub fn fin(&self) -> FinId {
        self.fin.clone()
    }

    /// Stable source-edge/root identity.
    pub const fn source_parameter(&self) -> &SectionSourceParameterKey {
        &self.source_parameter
    }

    /// Intrinsic source-edge parameter enclosure.
    pub const fn edge_parameter(&self) -> SectionEdgeParameterInterval {
        self.edge_parameter
    }

    /// Projective pcurve parameter enclosure used for cyclic ordering.
    pub const fn pcurve_half_angle(&self) -> SectionProjectiveParameterInterval {
        self.pcurve_half_angle
    }
}

/// One directed occurrence of a stitched curved-fragment endpoint.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionCurveFragmentEnd {
    endpoint: usize,
    point: Point3,
    carrier_parameter: f64,
    trim: SectionCurveTrimProvenance,
}

impl SectionCurveFragmentEnd {
    /// Index into [`BodySectionGraph::curve_endpoints`].
    pub const fn endpoint(&self) -> usize {
        self.endpoint
    }

    /// Numeric model-space representative (evidence, not join authority).
    pub const fn point(&self) -> Point3 {
        self.point
    }

    /// Numeric representative in the source branch's canonical parameter.
    pub const fn carrier_parameter(&self) -> f64 {
        self.carrier_parameter
    }

    /// Exact topology and parameter provenance for this trim event.
    pub const fn trim(&self) -> &SectionCurveTrimProvenance {
        &self.trim
    }
}

/// Exact coverage of one public curved fragment.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum SectionCurveFragmentSpan {
    /// The complete periodic carrier survived both trims without endpoints.
    Whole,
    /// Directed bounded arc between two exact trim events.
    Arc {
        /// Start/end occurrences in canonical carrier orientation.
        endpoints: Box<[SectionCurveFragmentEnd; 2]>,
        /// Whether the arc crosses the plane pcurve's projective chart seam.
        wraps_pcurve_seam: bool,
    },
}

/// One proof-bearing exact curved fragment retained by the section graph.
///
/// [`Self::branch`] links to the carrier, paired pcurves, source faces, and
/// residual certificate. The source ordinal is deterministic within that
/// branch and the span follows its canonical carrier orientation.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionCurveFragment {
    branch: usize,
    source_ordinal: usize,
    span: SectionCurveFragmentSpan,
}

impl SectionCurveFragment {
    /// Index into [`BodySectionGraph::branches`].
    pub const fn branch(&self) -> usize {
        self.branch
    }

    /// Deterministic clipper-owned ordinal within the source branch.
    pub const fn source_ordinal(&self) -> usize {
        self.source_ordinal
    }

    /// Exact whole-period or bounded-arc coverage.
    pub const fn span(&self) -> &SectionCurveFragmentSpan {
        &self.span
    }
}

/// One maximal directed component of exact curved fragments.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SectionCurveComponent {
    fragments: Vec<usize>,
    closed: bool,
}

impl SectionCurveComponent {
    /// Indices into [`BodySectionGraph::curve_fragments`] in traversal order.
    pub fn fragments(&self) -> &[usize] {
        &self.fragments
    }

    /// Whether exact endpoint incidence closes this component.
    pub const fn closed(&self) -> bool {
        self.closed
    }
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
    pub(crate) curve_endpoints: Vec<SectionCurveEndpoint>,
    pub(crate) curve_fragments: Vec<SectionCurveFragment>,
    pub(crate) curve_components: Vec<SectionCurveComponent>,
    pub(crate) loops: Vec<SectionLoop>,
    pub(crate) rings: Vec<SectionRing>,
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

    /// Certified curved carrier branches. [`Self::rings`] identifies the
    /// branches whose exact trims retained a whole closed component.
    pub fn branches(&self) -> &[SectionBranch] {
        &self.branches
    }

    /// Proof-keyed endpoints shared by bounded curved fragments.
    pub fn curve_endpoints(&self) -> &[SectionCurveEndpoint] {
        &self.curve_endpoints
    }

    /// Exact curved fragments in deterministic clipper order.
    pub fn curve_fragments(&self) -> &[SectionCurveFragment] {
        &self.curve_fragments
    }

    /// Maximal directed curved components in deterministic discovery order.
    pub fn curve_components(&self) -> &[SectionCurveComponent] {
        &self.curve_components
    }

    /// Stitched chains in deterministic discovery order.
    pub fn loops(&self) -> &[SectionLoop] {
        &self.loops
    }

    /// Endpoint-free curved components in deterministic discovery order.
    pub fn rings(&self) -> &[SectionRing] {
        &self.rings
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
    "two independently bounded curved trims cannot yet be intersected in cyclic parameter space";
pub(crate) const GAP_CLOSED_STITCH: &str =
    "closed curved section fragments could not be stitched into manifold rings";

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
    let envelopes_a = broad_phase::prepare_face_envelopes(store, &faces_a, scope)?;
    let envelopes_b = broad_phase::prepare_face_envelopes(store, &faces_b, scope)?;

    let mut acc = SectionAccumulator::default();
    let mut examined: u64 = 0;
    collect_plane_cylinder_branches(
        store,
        part_id,
        &faces_a,
        &faces_b,
        &envelopes_a,
        &envelopes_b,
        linear,
        &mut examined,
        scope,
        &mut acc,
    )?;
    let admitted_a = admit_faces(store, part_id, &faces_a, linear, scope)?;
    let admitted_b = admit_faces(store, part_id, &faces_b, linear, scope)?;

    for (a_index, slot_a) in admitted_a.iter().enumerate() {
        for (b_index, slot_b) in admitted_b.iter().enumerate() {
            let envelope_a = envelopes_a[a_index];
            let envelope_b = envelopes_b[b_index];
            if plane_cylinder_pair(envelope_a.class, envelope_b.class) {
                // The curved dispatcher above owns this pair exactly once.
                continue;
            }
            let ordinal = pair_ordinal(a_index, admitted_b.len(), b_index);
            examined += 1;
            scope
                .ledger_mut()
                .observe(SECTION_FACE_PAIRS, ResourceKind::Items, examined)
                .map_err(Error::from)?;
            charge(scope, 1)?;
            if broad_phase::certifiably_disjoint(envelope_a, envelope_b, linear) {
                continue;
            }
            let (PlanarFaceAdmission::Ready(face_a), PlanarFaceAdmission::Ready(face_b)) =
                (slot_a, slot_b)
            else {
                record_admission_gaps(slot_a, slot_b, &mut acc);
                continue;
            };
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

/// Pair-local planar admission.  A face outside the planar trim class is not
/// a graph gap until a non-disjoint pair actually needs that class.
enum PlanarFaceAdmission {
    Ready(Box<AdmittedFace>),
    Gap {
        facade: FaceId,
        reason: &'static str,
    },
}

fn plane_cylinder_pair(a: broad_phase::FaceSurfaceClass, b: broad_phase::FaceSurfaceClass) -> bool {
    matches!(
        (a, b),
        (
            broad_phase::FaceSurfaceClass::Plane,
            broad_phase::FaceSurfaceClass::Cylinder
        ) | (
            broad_phase::FaceSurfaceClass::Cylinder,
            broad_phase::FaceSurfaceClass::Plane
        )
    )
}

fn record_admission_gaps(
    a: &PlanarFaceAdmission,
    b: &PlanarFaceAdmission,
    acc: &mut SectionAccumulator,
) {
    let pair_faces = [admission_facade(a).clone(), admission_facade(b).clone()];
    for admission in [a, b] {
        if let PlanarFaceAdmission::Gap { reason, .. } = admission {
            acc.gaps.push(SectionGap {
                reason,
                faces: pair_faces.to_vec(),
            });
        }
    }
}

fn admission_facade(admission: &PlanarFaceAdmission) -> &FaceId {
    match admission {
        PlanarFaceAdmission::Ready(face) => &face.facade,
        PlanarFaceAdmission::Gap { facade, .. } => facade,
    }
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

/// Facade adaptation evidence aligned index-for-index with the exact closed
/// stitcher's fragment input. The stitcher intentionally owns only proof
/// identities; metric representatives and full trim provenance stay here.
#[derive(Debug, Clone, Copy)]
struct ClosedFragmentEvidence {
    branch: usize,
    ordinal: usize,
    span: ClosedFragmentEvidenceSpan,
}

#[derive(Debug, Clone, Copy)]
enum ClosedFragmentEvidenceSpan {
    Whole,
    Arc {
        ends: [ClosedFragmentEndEvidence; 2],
        wraps_pcurve_seam: bool,
    },
}

#[derive(Debug, Clone, Copy)]
struct ClosedFragmentEndEvidence {
    trim_operand: usize,
    site: curved_clip::ClosedConicTrimSite,
}

/// Deterministic collectors for one section query.
#[derive(Default)]
struct SectionAccumulator {
    segments: Vec<stitch::StitchSegment>,
    geometry: Vec<SegmentGeometry>,
    branches: Vec<SectionBranch>,
    closed_fragments: Vec<closed_stitch::ClosedCurveFragment>,
    closed_fragment_evidence: Vec<ClosedFragmentEvidence>,
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

#[derive(Debug, Clone, PartialEq)]
enum ClosedTrimMerge {
    Empty,
    Fragments(Vec<MergedClosedFragment>),
    UnsupportedIntersection,
    Gap(&'static str),
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct MergedClosedFragment {
    fragment: curved_clip::ClosedConicFragment,
    /// `None` is a whole-period carrier; bounded fragments name the operand
    /// whose exact face trim contributed both endpoints.
    trim_operand: Option<usize>,
}

fn whole_closed_conic_fragment(fragments: &[curved_clip::ClosedConicFragment]) -> bool {
    matches!(
        fragments,
        [curved_clip::ClosedConicFragment {
            start: None,
            end: None,
            wraps_pcurve_seam: true,
        }]
    )
}

/// Intersect two operand-local trim results over the currently admitted exact
/// classes. Empty is absorbing and whole-period is the identity, so any
/// bounded set on the other operand is retained without a layout taxonomy.
/// Two independently bounded sets require cyclic interval-set intersection,
/// which remains a typed gap rather than a numeric merge.
fn merge_closed_trim_outcomes(
    a: &curved_clip::ClosedConicClipOutcome,
    b: &curved_clip::ClosedConicClipOutcome,
) -> ClosedTrimMerge {
    use curved_clip::ClosedConicClipOutcome::{Fragments, Indeterminate};

    if matches!(a, Fragments(fragments) if fragments.is_empty())
        || matches!(b, Fragments(fragments) if fragments.is_empty())
    {
        return ClosedTrimMerge::Empty;
    }
    if let Indeterminate(gap) = a {
        return ClosedTrimMerge::Gap(gap.reason());
    }
    if let Indeterminate(gap) = b {
        return ClosedTrimMerge::Gap(gap.reason());
    }
    let (Fragments(a), Fragments(b)) = (a, b) else {
        unreachable!("indeterminate closed trim outcomes returned above")
    };
    let (a_whole, b_whole) = (
        whole_closed_conic_fragment(a),
        whole_closed_conic_fragment(b),
    );
    if a_whole && b_whole {
        return ClosedTrimMerge::Fragments(vec![MergedClosedFragment {
            fragment: a[0],
            trim_operand: None,
        }]);
    }
    if a_whole {
        return ClosedTrimMerge::Fragments(
            b.iter()
                .copied()
                .map(|fragment| MergedClosedFragment {
                    fragment,
                    trim_operand: Some(1),
                })
                .collect(),
        );
    }
    if b_whole {
        return ClosedTrimMerge::Fragments(
            a.iter()
                .copied()
                .map(|fragment| MergedClosedFragment {
                    fragment,
                    trim_operand: Some(0),
                })
                .collect(),
        );
    }
    ClosedTrimMerge::UnsupportedIntersection
}

fn certified_closed_trim_endpoint(
    source: closed_stitch::ClosedBranchSource,
    trim_operand: usize,
    site: curved_clip::ClosedConicTrimSite,
) -> Option<closed_stitch::CertifiedClosedEndpoint> {
    if trim_operand >= 2 || source.faces[trim_operand] != site.face {
        return None;
    }
    let mut sites = source.faces.map(stitch::SiteKey::Face);
    sites[trim_operand] = stitch::SiteKey::Edge(site.edge);
    let mut keys = [None, None];
    keys[trim_operand] = Some(closed_stitch::CertifiedSourceParameterKey::new(
        site.edge,
        site.root_ordinal,
    ));
    let mut parameters = [None, None];
    parameters[trim_operand] = Some(site.edge_parameter);
    Some(closed_stitch::CertifiedClosedEndpoint::trim_site(
        stitch::VertexKey {
            a: sites[0],
            b: sites[1],
        },
        keys,
        parameters,
    ))
}

/// Adapt exact merged clip fragments into proof-key stitch inputs and retain
/// their richer facade evidence in the same deterministic order.
fn append_closed_fragments(
    branch_index: usize,
    merged: &[MergedClosedFragment],
    acc: &mut SectionAccumulator,
) -> bool {
    let Some(source) = closed_stitch::ClosedBranchSource::from_section_branch(
        branch_index,
        &acc.branches[branch_index],
    ) else {
        return false;
    };
    let mut stitch_inputs = Vec::with_capacity(merged.len());
    let mut facade_evidence = Vec::with_capacity(merged.len());
    for (ordinal, merged) in merged.iter().copied().enumerate() {
        let (span, evidence_span) = match (
            merged.trim_operand,
            merged.fragment.start,
            merged.fragment.end,
        ) {
            (None, None, None) if merged.fragment.wraps_pcurve_seam => (
                closed_stitch::ClosedFragmentSpan::Whole,
                ClosedFragmentEvidenceSpan::Whole,
            ),
            (Some(trim_operand), Some(start), Some(end)) => {
                let (Some(start_key), Some(end_key)) = (
                    certified_closed_trim_endpoint(source, trim_operand, start),
                    certified_closed_trim_endpoint(source, trim_operand, end),
                ) else {
                    return false;
                };
                (
                    closed_stitch::ClosedFragmentSpan::Arc {
                        start: start_key,
                        end: end_key,
                    },
                    ClosedFragmentEvidenceSpan::Arc {
                        ends: [
                            ClosedFragmentEndEvidence {
                                trim_operand,
                                site: start,
                            },
                            ClosedFragmentEndEvidence {
                                trim_operand,
                                site: end,
                            },
                        ],
                        wraps_pcurve_seam: merged.fragment.wraps_pcurve_seam,
                    },
                )
            }
            _ => return false,
        };
        stitch_inputs.push(closed_stitch::ClosedCurveFragment {
            source: source.fragment(ordinal),
            orientation: closed_stitch::ClosedFragmentOrientation::AlongCarrier,
            span,
        });
        facade_evidence.push(ClosedFragmentEvidence {
            branch: branch_index,
            ordinal,
            span: evidence_span,
        });
    }
    acc.closed_fragments.extend(stitch_inputs);
    acc.closed_fragment_evidence.extend(facade_evidence);
    true
}

/// Decide whether the graph-owned cylinder-longitude parameterization must be
/// reversed to follow Section's canonical `n_A × n_B` orientation.
///
/// The exact dyadic sign of the plane-normal/cylinder-axis dot product owns
/// the decision. Operand order and both face senses then contribute only
/// exact sign flips. Parallel-but-indeterminate and perpendicular inputs are
/// refused rather than oriented from a rounded dot product.
fn canonical_plane_cylinder_flip(
    surface_a: &SurfaceGeom,
    sense_a: Sense,
    surface_b: &SurfaceGeom,
    sense_b: Sense,
) -> Option<bool> {
    let (plane_normal, plane_sense, cylinder_axis, cylinder_sense, plane_is_a) =
        match (surface_a, surface_b) {
            (SurfaceGeom::Plane(plane), SurfaceGeom::Cylinder(cylinder)) => (
                plane.frame().z(),
                sense_a,
                cylinder.frame().z(),
                sense_b,
                true,
            ),
            (SurfaceGeom::Cylinder(cylinder), SurfaceGeom::Plane(plane)) => (
                plane.frame().z(),
                sense_b,
                cylinder.frame().z(),
                sense_a,
                false,
            ),
            _ => return None,
        };
    let mut sign = match affine_dot3(
        plane_normal.to_array(),
        cylinder_axis.to_array(),
        [0.0; 3],
        0.0,
    )?
    .sign()
    {
        Orientation::Positive => 1_i8,
        Orientation::Negative => -1_i8,
        Orientation::Zero => return None,
    };
    if !plane_is_a {
        sign = -sign;
    }
    if !plane_sense.is_forward() {
        sign = -sign;
    }
    if !cylinder_sense.is_forward() {
        sign = -sign;
    }
    Some(sign < 0)
}

/// Collect proof-bearing Plane/Cylinder circle carriers independently of the
/// planar trim/stitch admission path.
///
/// Face domains are conservative source-owned surface windows used only for
/// analytic branch discovery and paired trace proof. Exact membership is
/// decided afterward from topology-owned loops, fins, edges, and pcurves.
#[allow(clippy::too_many_arguments)]
fn collect_plane_cylinder_branches(
    store: &Store,
    part_id: &PartId,
    faces_a: &[RawFaceId],
    faces_b: &[RawFaceId],
    envelopes_a: &[broad_phase::FaceEnvelope],
    envelopes_b: &[broad_phase::FaceEnvelope],
    linear: f64,
    examined: &mut u64,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut SectionAccumulator,
) -> Result<()> {
    for (a_index, &raw_a) in faces_a.iter().enumerate() {
        let face_a = read(store.get(raw_a))?;
        let surface_a = read(store.surface(face_a.surface))?;
        for (b_index, &raw_b) in faces_b.iter().enumerate() {
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
            if broad_phase::certifiably_disjoint(envelopes_a[a_index], envelopes_b[b_index], linear)
            {
                continue;
            }
            let facades = [
                FaceId::new(part_id.clone(), raw_a),
                FaceId::new(part_id.clone(), raw_b),
            ];
            let Some(flipped) =
                canonical_plane_cylinder_flip(surface_a, face_a.sense, surface_b, face_b.sense)
            else {
                acc.gaps.push(SectionGap {
                    reason: GAP_CARRIER_ORIENTATION,
                    faces: facades.to_vec(),
                });
                continue;
            };
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
                    flipped,
                ) else {
                    acc.gaps.push(SectionGap {
                        reason: GAP_PAIR_UNRESOLVED,
                        faces: facades.to_vec(),
                    });
                    continue;
                };
                let clipped = [
                    curved_clip::clip_closed_conic_to_face(
                        store,
                        raw_a,
                        branch.pcurves[0],
                        branch.range,
                        scope,
                    )?,
                    curved_clip::clip_closed_conic_to_face(
                        store,
                        raw_b,
                        branch.pcurves[1],
                        branch.range,
                        scope,
                    )?,
                ];
                let trim = merge_closed_trim_outcomes(&clipped[0], &clipped[1]);
                let branch_index = acc.branches.len();
                acc.branches.push(branch);
                match trim {
                    ClosedTrimMerge::Empty => {}
                    ClosedTrimMerge::Fragments(fragments) => {
                        if !append_closed_fragments(branch_index, &fragments, acc) {
                            acc.gaps.push(SectionGap {
                                reason: GAP_CLOSED_STITCH,
                                faces: facades.to_vec(),
                            });
                        }
                    }
                    ClosedTrimMerge::UnsupportedIntersection => acc.gaps.push(SectionGap {
                        reason: GAP_CURVED_TRIM_UNRESOLVED,
                        faces: facades.to_vec(),
                    }),
                    ClosedTrimMerge::Gap(reason) => acc.gaps.push(SectionGap {
                        reason,
                        faces: facades.to_vec(),
                    }),
                }
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
    flipped: bool,
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
        adapt_branch_pcurve(&edge.pcurves[0], edge.parameter_maps[0], flipped)?,
        adapt_branch_pcurve(&edge.pcurves[1], edge.parameter_maps[1], flipped)?,
    ];
    let vertex = *vertices.get(edge.endpoint_vertices[0])?;
    let IntersectionBranchVertexEvent::PeriodSeam { surfaces } = vertex.event else {
        return None;
    };
    Some(SectionBranch {
        faces: faces.clone(),
        carrier: SectionCarrier::Circle {
            center: carrier.frame().origin(),
            normal: if flipped {
                -carrier.frame().z()
            } else {
                carrier.frame().z()
            },
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
    flipped: bool,
) -> Option<SectionUvCurve> {
    if let Some(line) = descriptor.as_line() {
        return Some(SectionUvCurve::Line(compose_uv_line(
            line.origin(),
            line.dir(),
            map,
            flipped,
        )));
    }
    let circle = descriptor.as_circle()?;
    Some(SectionUvCurve::Circle(SectionUvCircle {
        center: circle.center(),
        radius: circle.radius(),
        x_direction: circle.x_dir(),
        parameter_scale: if flipped { -map.scale() } else { map.scale() },
        parameter_offset: map.offset(),
    }))
}

/// Run slice admission over one body's stored face list.
///
/// The returned vector is aligned with `faces` so pair ordinals stay stable.
/// Inadmissibility remains latent until a non-disjoint pair needs the face.
fn admit_faces(
    store: &Store,
    part_id: &PartId,
    faces: &[RawFaceId],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Vec<PlanarFaceAdmission>> {
    let mut admitted = Vec::with_capacity(faces.len());
    for &raw in faces {
        let facade = FaceId::new(part_id.clone(), raw);
        match clip::prepare_section_face(store, raw, linear, scope)? {
            Err(reason) => {
                admitted.push(PlanarFaceAdmission::Gap { facade, reason });
            }
            Ok(prep) => {
                let face = read(store.get(raw))?;
                let SurfaceGeom::Plane(plane) = read(store.surface(face.surface))? else {
                    admitted.push(PlanarFaceAdmission::Gap {
                        facade,
                        reason: GAP_PLANAR_ONLY,
                    });
                    continue;
                };
                let frame = *plane.frame();
                let window = face_window(&frame, &prep, linear);
                admitted.push(PlanarFaceAdmission::Ready(Box::new(AdmittedFace {
                    prep,
                    facade,
                    surface: face.surface,
                    frame,
                    sense: face.sense,
                    window,
                })));
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

fn adapt_closed_endpoint(
    part: &PartId,
    vertex: &closed_stitch::ClosedStitchVertex,
) -> SectionCurveEndpoint {
    let topology = match vertex.key {
        closed_stitch::CertifiedClosedEndpointKey::TrimSite {
            site,
            edge_parameter_keys,
        } => SectionCurveEndpointTopology::Trim {
            sites: [adapt_site(part, site.a), adapt_site(part, site.b)],
            source_parameters: edge_parameter_keys.map(|key| {
                key.map(|key| SectionSourceParameterKey {
                    edge: EdgeId::new(part.clone(), key.edge()),
                    root_ordinal: key.root_ordinal(),
                })
            }),
        },
        closed_stitch::CertifiedClosedEndpointKey::PeriodSeam { branch, site } => {
            SectionCurveEndpointTopology::ParameterSeam {
                branch: branch.index(),
                site,
            }
        }
    };
    SectionCurveEndpoint {
        topology,
        edge_parameters: vertex
            .edge_parameters
            .map(|value| value.map(SectionEdgeParameterInterval::from_interval)),
    }
}

fn curved_carrier_point(branch: &SectionBranch, parameter: f64) -> Option<Point3> {
    let SectionCarrier::Circle {
        center,
        normal,
        x_direction,
        radius,
    } = branch.carrier;
    let (sin, cos) = kcore::math::sincos(parameter);
    let point = center + x_direction * (radius * cos) + normal.cross(x_direction) * (radius * sin);
    [point.x, point.y, point.z]
        .into_iter()
        .all(f64::is_finite)
        .then_some(point)
}

fn adapt_curve_fragment_end(
    part: &PartId,
    branch: &SectionBranch,
    endpoint: usize,
    evidence: ClosedFragmentEndEvidence,
) -> Option<SectionCurveFragmentEnd> {
    let site = evidence.site;
    Some(SectionCurveFragmentEnd {
        endpoint,
        point: curved_carrier_point(branch, site.carrier_parameter)?,
        carrier_parameter: site.carrier_parameter,
        trim: SectionCurveTrimProvenance {
            operand: evidence.trim_operand,
            face: FaceId::new(part.clone(), site.face),
            loop_id: LoopId::new(part.clone(), site.loop_id),
            fin: FinId::new(part.clone(), site.fin),
            source_parameter: SectionSourceParameterKey {
                edge: EdgeId::new(part.clone(), site.edge),
                root_ordinal: site.root_ordinal,
            },
            edge_parameter: SectionEdgeParameterInterval::from_interval(site.edge_parameter),
            pcurve_half_angle: SectionProjectiveParameterInterval::from_interval(
                site.pcurve_half_angle,
            ),
        },
    })
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
        closed_fragments,
        closed_fragment_evidence,
        mut gaps,
    } = acc;
    if closed_fragments.len() != closed_fragment_evidence.len() {
        return Err(Error::InconsistentTopology {
            source: kcore::error::Error::InvalidGeometry {
                reason: "closed section fragment evidence is not index-aligned",
            },
        });
    }
    let stitched = stitch::stitch_segments(&segments);
    let closed_stitched = closed_stitch::stitch_closed_fragments(&closed_fragments);
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
    let curve_endpoints = closed_stitched
        .vertices
        .iter()
        .map(|vertex| adapt_closed_endpoint(part_id, vertex))
        .collect::<Vec<_>>();
    let mut fragment_endpoints = vec![None; closed_fragments.len()];
    for chain in &closed_stitched.chains {
        for fragment in &chain.fragments {
            let Some(slot) = fragment_endpoints.get_mut(fragment.input_fragment) else {
                return Err(Error::InconsistentTopology {
                    source: kcore::error::Error::InvalidGeometry {
                        reason: "closed stitch chain referenced an unknown fragment",
                    },
                });
            };
            *slot = Some(fragment.endpoints);
        }
    }
    let mut curve_fragments = Vec::with_capacity(closed_fragment_evidence.len());
    for (input_index, evidence) in closed_fragment_evidence.iter().copied().enumerate() {
        let Some(branch) = branches.get(evidence.branch) else {
            return Err(Error::InconsistentTopology {
                source: kcore::error::Error::InvalidGeometry {
                    reason: "curved section fragment referenced an unknown branch",
                },
            });
        };
        let span = match evidence.span {
            ClosedFragmentEvidenceSpan::Whole => SectionCurveFragmentSpan::Whole,
            ClosedFragmentEvidenceSpan::Arc {
                ends,
                wraps_pcurve_seam,
            } => {
                let Some(endpoint_indices) = fragment_endpoints
                    .get(input_index)
                    .copied()
                    .flatten()
                    .flatten()
                else {
                    return Err(Error::InconsistentTopology {
                        source: kcore::error::Error::InvalidGeometry {
                            reason: "certified curved arc lacks stitched endpoint indices",
                        },
                    });
                };
                let (Some(start), Some(end)) = (
                    adapt_curve_fragment_end(part_id, branch, endpoint_indices[0], ends[0]),
                    adapt_curve_fragment_end(part_id, branch, endpoint_indices[1], ends[1]),
                ) else {
                    return Err(Error::InconsistentTopology {
                        source: kcore::error::Error::InvalidGeometry {
                            reason: "certified curved endpoint has no finite representative",
                        },
                    });
                };
                SectionCurveFragmentSpan::Arc {
                    endpoints: Box::new([start, end]),
                    wraps_pcurve_seam,
                }
            }
        };
        curve_fragments.push(SectionCurveFragment {
            branch: evidence.branch,
            source_ordinal: evidence.ordinal,
            span,
        });
    }
    let curve_components = closed_stitched
        .chains
        .iter()
        .map(|chain| SectionCurveComponent {
            fragments: chain
                .fragments
                .iter()
                .map(|fragment| fragment.input_fragment)
                .collect(),
            closed: chain.closed,
        })
        .collect::<Vec<_>>();
    let mut rings = Vec::new();
    for chain in &closed_stitched.chains {
        let [fragment] = chain.fragments.as_slice() else {
            continue;
        };
        let Some(input) = closed_fragments.get(fragment.input_fragment) else {
            continue;
        };
        if chain.closed
            && fragment.endpoints.is_none()
            && matches!(input.span, closed_stitch::ClosedFragmentSpan::Whole)
            && fragment.source.branch.index() < branches.len()
        {
            rings.push(SectionRing {
                branch: fragment.source.branch.index(),
            });
        }
    }
    for defect in &stitched.defects {
        gaps.push(SectionGap {
            reason: stitch_defect_reason(*defect),
            faces: Vec::new(),
        });
    }
    for defect in &closed_stitched.defects {
        gaps.push(SectionGap {
            reason: closed_stitch_defect_reason(*defect),
            faces: Vec::new(),
        });
    }
    if closed_stitched.completion == closed_stitch::ClosedStitchCompletion::Indeterminate
        && closed_stitched.defects.is_empty()
    {
        gaps.push(SectionGap {
            reason: GAP_CLOSED_STITCH,
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
        curve_endpoints,
        curve_fragments,
        curve_components,
        loops,
        rings,
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

fn closed_stitch_defect_reason(defect: closed_stitch::ClosedStitchDefect) -> &'static str {
    match defect {
        closed_stitch::ClosedStitchDefect::IncompatibleEndpointParameter(_) => {
            GAP_INCOMPATIBLE_EDGE_PARAMETERS
        }
        _ => GAP_CLOSED_STITCH,
    }
}

#[cfg(test)]
mod unit_tests {
    use kgeom::frame::Frame;
    use kgeom::surface::{Cylinder, Plane};
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

    #[test]
    fn plane_cylinder_orientation_accounts_for_operand_order_and_face_senses() {
        let plane = SurfaceGeom::Plane(Plane::new(Frame::world()));
        let cylinder = SurfaceGeom::Cylinder(Cylinder::new(Frame::world(), 1.0).unwrap());
        let f = Sense::Forward;
        let r = Sense::Reversed;

        assert_eq!(
            canonical_plane_cylinder_flip(&plane, f, &cylinder, f),
            Some(false)
        );
        assert_eq!(
            canonical_plane_cylinder_flip(&plane, r, &cylinder, f),
            Some(true)
        );
        assert_eq!(
            canonical_plane_cylinder_flip(&plane, f, &cylinder, r),
            Some(true)
        );
        assert_eq!(
            canonical_plane_cylinder_flip(&plane, r, &cylinder, r),
            Some(false)
        );

        assert_eq!(
            canonical_plane_cylinder_flip(&cylinder, f, &plane, f),
            Some(true)
        );
        assert_eq!(
            canonical_plane_cylinder_flip(&cylinder, r, &plane, f),
            Some(false)
        );
        assert_eq!(
            canonical_plane_cylinder_flip(&cylinder, f, &plane, r),
            Some(false)
        );
        assert_eq!(
            canonical_plane_cylinder_flip(&cylinder, r, &plane, r),
            Some(true)
        );

        let perpendicular = SurfaceGeom::Cylinder(
            Cylinder::new(
                Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
                1.0,
            )
            .unwrap(),
        );
        assert_eq!(
            canonical_plane_cylinder_flip(&plane, f, &perpendicular, f),
            None
        );
    }
}
