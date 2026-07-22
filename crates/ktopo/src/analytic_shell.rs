//! Preflight contract for bounded analytic Plane/Cylinder shells.
//!
//! This module is deliberately allocation-free.  It accepts stable keyed
//! vertices, bounded line/circle edges, ordered face loops, analytic pcurves,
//! and optional source lineage.  [`prepare_analytic_shell`] validates the
//! complete proposal and returns a canonical immutable plan that a later
//! transaction adapter can realize without revisiting caller-controlled
//! combinatorics.
//!
//! Every edge is proved from its two face uses.  The admitted proof families
//! are Plane/Plane lines, transverse Plane/Cylinder rulings, transverse
//! Cylinder/Cylinder rulings, and transverse Plane/Cylinder circles. These are
//! representation families, not shell layouts: unsupported analytic pairings
//! fail closed with a typed error.

use core::fmt;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

use crate::entity::{
    Body, Edge, EntityRef, Face, FaceDomain, Fin, Loop, PcurveChart, Region, Sense, Shell, Vertex,
};
use crate::store::Store;
use kcore::interval::Interval;
use kcore::tolerance::check_in_size_box;
use kgeom::curve::{Circle, Curve, Line};
use kgeom::curve2d::{Circle2d, Curve2d, Line2d};
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane, Surface};
use kgeom::vec::{Point2, Point3};
use kgraph::{
    AffineParamMap1d, CylinderLongitudeTrace, CylinderRulingTrace, IntersectionCertificateError,
    PairedCylinderCylinderRulingResidualCertificate, PairedPlaneCylinderCircleResidualCertificate,
    PairedPlaneCylinderRulingResidualCertificate, PairedPlaneLineResidualCertificate,
    PlaneCircleTrace, PlaneCylinderCircleTrace, PlaneCylinderRulingTrace, PlaneRulingTrace,
    certify_paired_cylinder_cylinder_ruling_residuals,
    certify_paired_plane_cylinder_circle_residuals, certify_paired_plane_cylinder_ruling_residuals,
    certify_paired_plane_line_residuals,
};

mod assemble;
pub use assemble::{AnalyticShellAssemblyError, AnalyticShellOutput};
mod lineage_ruling;
pub use lineage_ruling::SourceLineagePlaneCylinderRulingResidualCertificate;

#[cfg(test)]
mod assemble_tests;
#[cfg(test)]
pub(crate) mod cylinder_cylinder_tests;

macro_rules! stable_key {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(u64);

        impl $name {
            /// Construct a key. Its numeric value has no geometric meaning.
            pub const fn new(value: u64) -> Self {
                Self(value)
            }

            /// Numeric value supplied by the semantic caller.
            pub const fn value(self) -> u64 {
                self.0
            }
        }
    };
}

stable_key!(
    AnalyticVertexKey,
    "Stable identity of one analytic-shell vertex."
);
stable_key!(
    AnalyticEdgeKey,
    "Stable identity of one analytic-shell edge."
);
stable_key!(
    AnalyticFaceKey,
    "Stable identity of one analytic-shell face."
);

/// Representative model-space point retained for one vertex identity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalyticShellVertex {
    key: AnalyticVertexKey,
    position: Point3,
}

impl AnalyticShellVertex {
    /// Pair a stable key with a finite representative point.
    pub const fn new(key: AnalyticVertexKey, position: Point3) -> Self {
        Self { key, position }
    }

    /// Stable vertex identity.
    pub const fn key(self) -> AnalyticVertexKey {
        self.key
    }

    /// Authoritative model-space position.
    pub const fn position(self) -> Point3 {
        self.position
    }
}

/// Exact analytic surface representation admitted by this plan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnalyticShellSurface {
    /// Unbounded plane restricted by the face domain and loops.
    Plane(Plane),
    /// Infinite cylinder restricted by the face domain and loops.
    Cylinder(Cylinder),
}

impl AnalyticShellSurface {
    fn periodicity(self) -> [Option<f64>; 2] {
        match self {
            Self::Plane(surface) => surface.periodicity(),
            Self::Cylinder(surface) => surface.periodicity(),
        }
    }

    fn eval(self, uv: Point2) -> Point3 {
        match self {
            Self::Plane(surface) => surface.eval([uv.x, uv.y]),
            Self::Cylinder(surface) => surface.eval([uv.x, uv.y]),
        }
    }
}

/// Exact analytic 3D carrier of one bounded topological edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnalyticShellCurve {
    /// Unit-speed line carrier.
    Line(Line),
    /// Periodic circle carrier.
    Circle(Circle),
}

impl AnalyticShellCurve {
    fn eval(self, parameter: f64) -> Point3 {
        match self {
            Self::Line(curve) => curve.eval(parameter),
            Self::Circle(curve) => curve.eval(parameter),
        }
    }
}

/// Exact analytic pcurve representation for one face use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnalyticShellPcurve {
    /// Unit-speed parameter-space line.
    Line(Line2d),
    /// Parameter-space circle.
    Circle(Circle2d),
}

impl AnalyticShellPcurve {
    fn eval(self, parameter: f64) -> Point2 {
        match self {
            Self::Line(curve) => curve.eval(parameter),
            Self::Circle(curve) => curve.eval(parameter),
        }
    }

    fn bounds(self, range: ParamRange) -> (Point2, Point2) {
        let bounds = match self {
            Self::Line(curve) => curve.bounding_box(range),
            Self::Circle(curve) => curve.bounding_box(range),
        };
        (bounds.min, bounds.max)
    }
}

/// One face's exact parameter-space use of an edge carrier.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalyticPcurveUse {
    curve: AnalyticShellPcurve,
    edge_to_pcurve: AffineParamMap1d,
    chart: PcurveChart,
    closure_winding: Option<[i32; 2]>,
}

impl AnalyticPcurveUse {
    /// Construct an authored pcurve on the identity surface chart.
    pub const fn new(curve: AnalyticShellPcurve, edge_to_pcurve: AffineParamMap1d) -> Self {
        Self {
            curve,
            edge_to_pcurve,
            chart: PcurveChart::identity(),
            closure_winding: None,
        }
    }

    /// Select an integer-period surface chart for this use.
    pub const fn with_chart(mut self, chart: PcurveChart) -> Self {
        self.chart = chart;
        self
    }

    /// Authored analytic pcurve.
    pub const fn curve(self) -> AnalyticShellPcurve {
        self.curve
    }

    /// Carrier-to-pcurve affine parameter correspondence.
    pub const fn edge_to_pcurve(self) -> AffineParamMap1d {
        self.edge_to_pcurve
    }

    /// Explicit periodic chart selection.
    pub const fn chart(self) -> PcurveChart {
        self.chart
    }

    /// Declare the whole-period displacement of an endpoint-free closed use.
    pub const fn with_closure_winding(mut self, winding: [i32; 2]) -> Self {
        self.closure_winding = Some(winding);
        self
    }

    /// Explicit whole-period displacement in increasing edge parameter.
    pub const fn closure_winding(self) -> Option<[i32; 2]> {
        self.closure_winding
    }
}

/// One bounded analytic edge shared by exactly two face uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalyticShellEdge {
    key: AnalyticEdgeKey,
    vertices: [AnalyticVertexKey; 2],
    carrier: AnalyticShellCurve,
    range: ParamRange,
    source: Option<EntityRef>,
}

impl AnalyticShellEdge {
    /// Describe an increasing finite portion of an analytic carrier.
    pub const fn new(
        key: AnalyticEdgeKey,
        vertices: [AnalyticVertexKey; 2],
        carrier: AnalyticShellCurve,
        range: ParamRange,
    ) -> Self {
        Self {
            key,
            vertices,
            carrier,
            range,
            source: None,
        }
    }

    /// Retain the source entity for transaction-journal lineage.
    pub const fn with_source(mut self, source: EntityRef) -> Self {
        self.source = Some(source);
        self
    }

    /// Stable edge identity.
    pub const fn key(self) -> AnalyticEdgeKey {
        self.key
    }

    /// Tail/head vertex identities in increasing carrier-parameter order.
    pub const fn vertices(self) -> [AnalyticVertexKey; 2] {
        self.vertices
    }

    /// Exact analytic carrier.
    pub const fn carrier(self) -> AnalyticShellCurve {
        self.carrier
    }

    /// Active finite carrier interval.
    pub const fn range(self) -> ParamRange {
        self.range
    }

    /// Optional source entity.
    pub const fn source(self) -> Option<EntityRef> {
        self.source
    }
}

/// One endpoint-free analytic circle shared by exactly two face uses.
///
/// `logical_range` is certification-only. Assembly emits no edge bounds and
/// no physical vertices; the range must equal the carrier's canonical full
/// period exactly.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalyticShellClosedEdge {
    key: AnalyticEdgeKey,
    carrier: AnalyticShellCurve,
    logical_range: ParamRange,
    source: Option<EntityRef>,
}

impl AnalyticShellClosedEdge {
    /// Describe an endpoint-free candidate over one canonical logical period.
    pub const fn new(
        key: AnalyticEdgeKey,
        carrier: AnalyticShellCurve,
        logical_range: ParamRange,
    ) -> Self {
        Self {
            key,
            carrier,
            logical_range,
            source: None,
        }
    }

    /// Retain the source entity for transaction-journal lineage.
    pub const fn with_source(mut self, source: EntityRef) -> Self {
        self.source = Some(source);
        self
    }

    /// Stable edge identity.
    pub const fn key(self) -> AnalyticEdgeKey {
        self.key
    }

    /// Candidate analytic carrier. Preflight currently admits only circles.
    pub const fn carrier(self) -> AnalyticShellCurve {
        self.carrier
    }

    /// Canonical full-period range used only by analytic certificates.
    pub const fn logical_range(self) -> ParamRange {
        self.logical_range
    }

    /// Optional source entity.
    pub const fn source(self) -> Option<EntityRef> {
        self.source
    }
}

/// Directed face-loop use of one shared analytic edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalyticShellFin {
    edge: AnalyticEdgeKey,
    sense: Sense,
    pcurve: AnalyticPcurveUse,
}

impl AnalyticShellFin {
    /// Pair a shared edge identity with face traversal and pcurve evidence.
    pub const fn new(edge: AnalyticEdgeKey, sense: Sense, pcurve: AnalyticPcurveUse) -> Self {
        Self {
            edge,
            sense,
            pcurve,
        }
    }

    /// Shared edge identity.
    pub const fn edge(self) -> AnalyticEdgeKey {
        self.edge
    }

    /// Face-loop traversal relative to increasing edge parameter.
    pub const fn sense(self) -> Sense {
        self.sense
    }

    /// Exact parameter-space use.
    pub const fn pcurve(self) -> AnalyticPcurveUse {
        self.pcurve
    }
}

/// One closed, ordered face boundary component.
#[derive(Debug, Clone, PartialEq)]
pub struct AnalyticShellLoop {
    fins: Vec<AnalyticShellFin>,
}

impl AnalyticShellLoop {
    /// Retain fins in directed boundary order.
    pub const fn new(fins: Vec<AnalyticShellFin>) -> Self {
        Self { fins }
    }

    /// Directed boundary uses.
    pub fn fins(&self) -> &[AnalyticShellFin] {
        &self.fins
    }
}

/// One analytic face with ordered boundary loops.
#[derive(Debug, Clone, PartialEq)]
pub struct AnalyticShellFace {
    key: AnalyticFaceKey,
    surface: AnalyticShellSurface,
    sense: Sense,
    domain: FaceDomain,
    loops: Vec<AnalyticShellLoop>,
    source: Option<EntityRef>,
}

impl AnalyticShellFace {
    /// Describe a trimmed analytic face.
    pub const fn new(
        key: AnalyticFaceKey,
        surface: AnalyticShellSurface,
        sense: Sense,
        domain: FaceDomain,
        loops: Vec<AnalyticShellLoop>,
    ) -> Self {
        Self {
            key,
            surface,
            sense,
            domain,
            loops,
            source: None,
        }
    }

    /// Retain the source entity for transaction-journal lineage.
    pub const fn with_source(mut self, source: EntityRef) -> Self {
        self.source = Some(source);
        self
    }

    /// Stable face identity.
    pub const fn key(&self) -> AnalyticFaceKey {
        self.key
    }

    /// Exact analytic support.
    pub const fn surface(&self) -> AnalyticShellSurface {
        self.surface
    }

    /// Face orientation against its surface normal.
    pub const fn sense(&self) -> Sense {
        self.sense
    }

    /// Conservative finite chart domain.
    pub const fn domain(&self) -> FaceDomain {
        self.domain
    }

    /// Ordered boundary components.
    pub fn loops(&self) -> &[AnalyticShellLoop] {
        &self.loops
    }

    /// Optional source entity.
    pub const fn source(&self) -> Option<EntityRef> {
        self.source
    }
}

/// Complete caller proposal for one connected closed analytic shell.
#[derive(Debug, Clone, PartialEq)]
pub struct AnalyticShellInput {
    vertices: Vec<AnalyticShellVertex>,
    edges: Vec<AnalyticShellEdge>,
    closed_edges: Vec<AnalyticShellClosedEdge>,
    faces: Vec<AnalyticShellFace>,
}

impl AnalyticShellInput {
    /// Construct a proposal. Complete validation occurs during preflight.
    pub const fn new(
        vertices: Vec<AnalyticShellVertex>,
        edges: Vec<AnalyticShellEdge>,
        faces: Vec<AnalyticShellFace>,
    ) -> Self {
        Self {
            vertices,
            edges,
            closed_edges: Vec::new(),
            faces,
        }
    }

    /// Add endpoint-free closed-circle declarations to this proposal.
    pub fn with_closed_edges(mut self, closed_edges: Vec<AnalyticShellClosedEdge>) -> Self {
        self.closed_edges = closed_edges;
        self
    }

    /// Caller-provided vertices.
    pub fn vertices(&self) -> &[AnalyticShellVertex] {
        &self.vertices
    }

    /// Caller-provided edges.
    pub fn edges(&self) -> &[AnalyticShellEdge] {
        &self.edges
    }

    /// Caller-provided endpoint-free closed edges.
    pub fn closed_edges(&self) -> &[AnalyticShellClosedEdge] {
        &self.closed_edges
    }

    /// Caller-provided faces.
    pub fn faces(&self) -> &[AnalyticShellFace] {
        &self.faces
    }
}

/// Whole-interval proof retained for one pair of analytic face uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnalyticEdgeProof {
    /// A finite line lifted through two planar pcurves.
    PlaneLine(PairedPlaneLineResidualCertificate),
    /// A finite transverse cylinder ruling lifted through Plane/Cylinder pcurves.
    PlaneCylinderRuling(PairedPlaneCylinderRulingResidualCertificate),
    /// A finite transverse cylinder ruling whose plane-family witness comes
    /// from two whole-fin signed-axis lines on the lineaged source face.
    SourceLineagePlaneCylinderRuling(SourceLineagePlaneCylinderRulingResidualCertificate),
    /// A finite strict-secant ruling lifted through two cylindrical pcurves.
    CylinderCylinderRuling(PairedCylinderCylinderRulingResidualCertificate),
    /// A circle whose complete period is proved on a plane and cylinder.
    PlaneCylinderCircle(PairedPlaneCylinderCircleResidualCertificate),
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum AnalyticEdgeDeclaration {
    Bounded(AnalyticShellEdge),
    Closed(AnalyticShellClosedEdge),
}

impl AnalyticEdgeDeclaration {
    const fn key(self) -> AnalyticEdgeKey {
        match self {
            Self::Bounded(edge) => edge.key(),
            Self::Closed(edge) => edge.key(),
        }
    }

    const fn carrier(self) -> AnalyticShellCurve {
        match self {
            Self::Bounded(edge) => edge.carrier(),
            Self::Closed(edge) => edge.carrier(),
        }
    }

    const fn logical_range(self) -> ParamRange {
        match self {
            Self::Bounded(edge) => edge.range(),
            Self::Closed(edge) => edge.logical_range(),
        }
    }

    const fn is_closed(self) -> bool {
        matches!(self, Self::Closed(_))
    }
}

/// One canonical edge-use reference retained by the prepared plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AnalyticEdgeUseRef {
    face: AnalyticFaceKey,
    loop_index: usize,
    fin_index: usize,
    sense: Sense,
}

impl AnalyticEdgeUseRef {
    /// Owning face identity.
    pub const fn face(self) -> AnalyticFaceKey {
        self.face
    }

    /// Owning face-local loop index after loop canonicalization.
    pub const fn loop_index(self) -> usize {
        self.loop_index
    }

    /// Loop-local fin index after cyclic canonicalization.
    pub const fn fin_index(self) -> usize {
        self.fin_index
    }

    /// Directed traversal against increasing edge parameter.
    pub const fn sense(self) -> Sense {
        self.sense
    }
}

/// One canonical edge plus its two exact face-use proofs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PreparedAnalyticEdge {
    edge: AnalyticShellEdge,
    uses: [AnalyticEdgeUseRef; 2],
    proof: AnalyticEdgeProof,
}

impl PreparedAnalyticEdge {
    /// Canonical edge declaration.
    pub const fn edge(self) -> AnalyticShellEdge {
        self.edge
    }

    /// Two uses ordered by face, loop, then fin identity.
    pub const fn uses(self) -> [AnalyticEdgeUseRef; 2] {
        self.uses
    }

    /// Whole-interval pairing certificate.
    pub const fn proof(self) -> AnalyticEdgeProof {
        self.proof
    }
}

/// One canonical endpoint-free edge plus its two exact face-use proofs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PreparedAnalyticClosedEdge {
    edge: AnalyticShellClosedEdge,
    uses: [AnalyticEdgeUseRef; 2],
    proof: AnalyticEdgeProof,
}

impl PreparedAnalyticClosedEdge {
    /// Canonical endpoint-free edge declaration.
    pub const fn edge(self) -> AnalyticShellClosedEdge {
        self.edge
    }

    /// Two uses ordered by face, loop, then fin identity.
    pub const fn uses(self) -> [AnalyticEdgeUseRef; 2] {
        self.uses
    }

    /// Whole-period pairing certificate.
    pub const fn proof(self) -> AnalyticEdgeProof {
        self.proof
    }
}

/// Canonical immutable result of complete allocation-free preflight.
#[derive(Debug, Clone, PartialEq)]
pub struct PreparedAnalyticShell {
    vertices: Vec<AnalyticShellVertex>,
    edges: Vec<PreparedAnalyticEdge>,
    closed_edges: Vec<PreparedAnalyticClosedEdge>,
    edge_order: Vec<AnalyticEdgeKey>,
    faces: Vec<AnalyticShellFace>,
}

impl PreparedAnalyticShell {
    /// Vertices in ascending stable-key order.
    pub fn vertices(&self) -> &[AnalyticShellVertex] {
        &self.vertices
    }

    /// Edges in ascending stable-key order.
    pub fn edges(&self) -> &[PreparedAnalyticEdge] {
        &self.edges
    }

    /// Endpoint-free edges in ascending stable-key order.
    pub fn closed_edges(&self) -> &[PreparedAnalyticClosedEdge] {
        &self.closed_edges
    }

    /// Faces in ascending stable-key order, with canonical loop rotations.
    pub fn faces(&self) -> &[AnalyticShellFace] {
        &self.faces
    }

    fn edge_order(&self) -> &[AnalyticEdgeKey] {
        &self.edge_order
    }

    fn declaration(&self, key: AnalyticEdgeKey) -> Option<AnalyticEdgeDeclaration> {
        if let Ok(index) = self
            .edges
            .binary_search_by_key(&key, |candidate| candidate.edge().key())
        {
            return Some(AnalyticEdgeDeclaration::Bounded(self.edges[index].edge()));
        }
        self.closed_edges
            .binary_search_by_key(&key, |candidate| candidate.edge().key())
            .ok()
            .map(|index| AnalyticEdgeDeclaration::Closed(self.closed_edges[index].edge()))
    }
}

/// Typed, fail-closed analytic-shell preflight failure.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum AnalyticShellPlanError {
    /// The proposal has no vertices, edges, or faces.
    EmptyShell,
    /// A stable vertex identity appeared more than once.
    DuplicateVertex(AnalyticVertexKey),
    /// A stable edge identity appeared more than once.
    DuplicateEdge(AnalyticEdgeKey),
    /// An endpoint-free declaration used a non-circle carrier.
    ClosedEdgeRequiresCircle(AnalyticEdgeKey),
    /// An endpoint-free circle did not declare its exact canonical period.
    ClosedEdgeRequiresCanonicalPeriod(AnalyticEdgeKey),
    /// A stable face identity appeared more than once.
    DuplicateFace(AnalyticFaceKey),
    /// A referenced vertex identity was not declared.
    UnknownVertex {
        /// Referencing edge.
        edge: AnalyticEdgeKey,
        /// Missing vertex.
        vertex: AnalyticVertexKey,
    },
    /// A face loop referenced an undeclared edge.
    UnknownEdge {
        /// Referencing face.
        face: AnalyticFaceKey,
        /// Missing edge.
        edge: AnalyticEdgeKey,
    },
    /// A position or bounded carrier failed finite size-box admission.
    InvalidGeometry {
        /// Stable caller-facing explanation.
        reason: &'static str,
    },
    /// A vertex is not certifiably incident to its carrier endpoint within
    /// the caller's admitted modeling tolerance.
    CarrierEndpointMismatch {
        /// Edge whose endpoint disagrees.
        edge: AnalyticEdgeKey,
        /// Endpoint index in increasing carrier order.
        endpoint: usize,
    },
    /// A face has no boundary or one of its boundary loops is empty.
    EmptyFaceBoundary(AnalyticFaceKey),
    /// Consecutive directed fins do not share the same vertex identity.
    OpenLoop {
        /// Face containing the loop.
        face: AnalyticFaceKey,
        /// Face-local loop index.
        loop_index: usize,
    },
    /// An endpoint-free edge must be the sole fin in its boundary loop.
    ClosedEdgeRequiresSingleFinLoop {
        /// Face containing the malformed loop.
        face: AnalyticFaceKey,
        /// Closed edge referenced by that loop.
        edge: AnalyticEdgeKey,
    },
    /// A bounded edge carried closure metadata reserved for endpoint-free uses.
    BoundedEdgeHasClosureWinding {
        /// Face containing the use.
        face: AnalyticFaceKey,
        /// Bounded edge carrying invalid closure metadata.
        edge: AnalyticEdgeKey,
    },
    /// An endpoint-free use omitted its explicit whole-period displacement.
    MissingClosureWinding {
        /// Face containing the use.
        face: AnalyticFaceKey,
        /// Closed edge missing winding evidence.
        edge: AnalyticEdgeKey,
    },
    /// Closure winding does not match the admitted Plane/Cylinder pcurve family.
    InvalidClosureWinding {
        /// Face containing the use.
        face: AnalyticFaceKey,
        /// Closed edge with contradictory winding evidence.
        edge: AnalyticEdgeKey,
    },
    /// A pcurve chart or active range lies outside the face's admitted domain.
    PcurveOutsideFaceDomain {
        /// Owning face.
        face: AnalyticFaceKey,
        /// Referenced edge.
        edge: AnalyticEdgeKey,
    },
    /// Consecutive pcurve uses do not meet exactly on their selected chart.
    PcurveLoopNotClosed {
        /// Face containing the loop.
        face: AnalyticFaceKey,
        /// Face-local loop index.
        loop_index: usize,
        /// Fin whose directed pcurve endpoint misses its successor.
        fin_index: usize,
    },
    /// A manifold edge did not have exactly two uses.
    EdgeUseCount {
        /// Edge with incomplete or excess incidence.
        edge: AnalyticEdgeKey,
        /// Observed use count.
        uses: usize,
    },
    /// The two uses traverse a shared edge in the same direction.
    EdgeUsesNotOpposed(AnalyticEdgeKey),
    /// The same face occupies both sides of one edge; this plan cannot prove it.
    SelfAdjacentEdge(AnalyticEdgeKey),
    /// Some faces are disconnected in the dual face graph.
    DisconnectedShell,
    /// A declared vertex is not used by any edge.
    UnusedVertex(AnalyticVertexKey),
    /// Optional source lineage refers to an entity absent from the store.
    StaleLineage(EntityRef),
    /// The carrier/surface/pcurve representation family has no exact certifier.
    UnsupportedPairing(AnalyticEdgeKey),
    /// An admitted pairing failed its whole-interval certificate.
    PairingCertification {
        /// Edge being certified.
        edge: AnalyticEdgeKey,
        /// Exact certifier failure.
        source: IntersectionCertificateError,
    },
}

impl fmt::Display for AnalyticShellPlanError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "analytic shell preflight failed: {self:?}")
    }
}

impl std::error::Error for AnalyticShellPlanError {}

#[derive(Debug, Clone, Copy)]
struct UseCandidate {
    face: AnalyticFaceKey,
    loop_index: usize,
    fin_index: usize,
    sense: Sense,
    surface: AnalyticShellSurface,
    pcurve: AnalyticPcurveUse,
    source: Option<EntityRef>,
}

/// Validate and canonicalize one analytic shell without allocating topology.
///
/// `tolerance` is consumed only by conservative whole-interval residual
/// certificates; every combinatorial choice and endpoint identity check is
/// exact.  Optional lineage is checked against `store` before any plan is
/// returned.
pub fn prepare_analytic_shell(
    input: &AnalyticShellInput,
    store: &Store,
    tolerance: f64,
) -> Result<PreparedAnalyticShell, AnalyticShellPlanError> {
    if input.faces.is_empty()
        || input.edges.is_empty() && input.closed_edges.is_empty()
        || input.vertices.is_empty() && input.closed_edges.is_empty()
    {
        return Err(AnalyticShellPlanError::EmptyShell);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(AnalyticShellPlanError::InvalidGeometry {
            reason: "analytic shell tolerance must be finite and nonnegative",
        });
    }

    let vertices = prepare_vertices(&input.vertices)?;
    let bounded_edges = prepare_edges(&input.edges, &vertices, store, tolerance)?;
    let closed_edges = prepare_closed_edges(&input.closed_edges, store)?;
    let mut declarations = BTreeMap::new();
    for (&key, &edge) in &bounded_edges {
        declarations.insert(key, AnalyticEdgeDeclaration::Bounded(edge));
    }
    for (&key, &edge) in &closed_edges {
        if declarations
            .insert(key, AnalyticEdgeDeclaration::Closed(edge))
            .is_some()
        {
            return Err(AnalyticShellPlanError::DuplicateEdge(key));
        }
    }
    let mut faces = prepare_faces(&input.faces, &declarations, store, tolerance)?;
    canonicalize_faces(&mut faces);
    let uses = collect_uses(&faces, &declarations)?;
    certify_connected_faces(&faces, &uses)?;

    let mut prepared_edges = Vec::with_capacity(bounded_edges.len());
    let mut prepared_closed_edges = Vec::with_capacity(closed_edges.len());
    for (&key, &declaration) in &declarations {
        let candidates = uses
            .get(&key)
            .ok_or(AnalyticShellPlanError::EdgeUseCount { edge: key, uses: 0 })?;
        if candidates.len() != 2 {
            return Err(AnalyticShellPlanError::EdgeUseCount {
                edge: key,
                uses: candidates.len(),
            });
        }
        if candidates[0].sense == candidates[1].sense {
            return Err(AnalyticShellPlanError::EdgeUsesNotOpposed(key));
        }
        if candidates[0].face == candidates[1].face {
            return Err(AnalyticShellPlanError::SelfAdjacentEdge(key));
        }
        let proof = certify_edge_pair(key, declaration, *candidates, store, tolerance)?;
        let refs = (*candidates).map(|candidate| AnalyticEdgeUseRef {
            face: candidate.face,
            loop_index: candidate.loop_index,
            fin_index: candidate.fin_index,
            sense: candidate.sense,
        });
        match declaration {
            AnalyticEdgeDeclaration::Bounded(edge) => {
                prepared_edges.push(PreparedAnalyticEdge {
                    edge,
                    uses: refs,
                    proof,
                });
            }
            AnalyticEdgeDeclaration::Closed(edge) => {
                prepared_closed_edges.push(PreparedAnalyticClosedEdge {
                    edge,
                    uses: refs,
                    proof,
                });
            }
        }
    }

    Ok(PreparedAnalyticShell {
        vertices: vertices.into_values().collect(),
        edges: prepared_edges,
        closed_edges: prepared_closed_edges,
        edge_order: declarations.keys().copied().collect(),
        faces,
    })
}

fn prepare_vertices(
    input: &[AnalyticShellVertex],
) -> Result<BTreeMap<AnalyticVertexKey, AnalyticShellVertex>, AnalyticShellPlanError> {
    let mut vertices = BTreeMap::new();
    for &vertex in input {
        check_in_size_box(vertex.position.to_array()).map_err(|_| {
            AnalyticShellPlanError::InvalidGeometry {
                reason: "analytic shell vertex lies outside the finite size box",
            }
        })?;
        if vertices.insert(vertex.key, vertex).is_some() {
            return Err(AnalyticShellPlanError::DuplicateVertex(vertex.key));
        }
    }
    Ok(vertices)
}

fn prepare_edges(
    input: &[AnalyticShellEdge],
    vertices: &BTreeMap<AnalyticVertexKey, AnalyticShellVertex>,
    store: &Store,
    tolerance: f64,
) -> Result<BTreeMap<AnalyticEdgeKey, AnalyticShellEdge>, AnalyticShellPlanError> {
    let mut edges = BTreeMap::new();
    let mut used_vertices = BTreeSet::new();
    for &edge in input {
        if !edge.range.is_finite() || edge.range.lo >= edge.range.hi {
            return Err(AnalyticShellPlanError::InvalidGeometry {
                reason: "analytic shell edge range must be finite and increasing",
            });
        }
        match edge.carrier {
            AnalyticShellCurve::Line(_) if edge.vertices[0] == edge.vertices[1] => {
                return Err(AnalyticShellPlanError::InvalidGeometry {
                    reason: "bounded line edge requires distinct endpoint identities",
                });
            }
            AnalyticShellCurve::Circle(curve)
                if edge.range.width() > curve.param_range().width() =>
            {
                return Err(AnalyticShellPlanError::InvalidGeometry {
                    reason: "bounded circle edge cannot span more than one period",
                });
            }
            _ => {}
        }
        for (endpoint, key) in edge.vertices.into_iter().enumerate() {
            let vertex = vertices
                .get(&key)
                .ok_or(AnalyticShellPlanError::UnknownVertex {
                    edge: edge.key,
                    vertex: key,
                })?;
            let parameter = if endpoint == 0 {
                edge.range.lo
            } else {
                edge.range.hi
            };
            if !certify_endpoint_incidence(vertex.position, edge.carrier.eval(parameter), tolerance)
            {
                return Err(AnalyticShellPlanError::CarrierEndpointMismatch {
                    edge: edge.key,
                    endpoint,
                });
            }
            used_vertices.insert(key);
        }
        if let Some(source) = edge.source
            && !lineage_is_live(store, source)
        {
            return Err(AnalyticShellPlanError::StaleLineage(source));
        }
        if edges.insert(edge.key, edge).is_some() {
            return Err(AnalyticShellPlanError::DuplicateEdge(edge.key));
        }
    }
    if let Some(key) = vertices
        .keys()
        .find(|key| !used_vertices.contains(key))
        .copied()
    {
        return Err(AnalyticShellPlanError::UnusedVertex(key));
    }
    Ok(edges)
}

fn prepare_closed_edges(
    input: &[AnalyticShellClosedEdge],
    store: &Store,
) -> Result<BTreeMap<AnalyticEdgeKey, AnalyticShellClosedEdge>, AnalyticShellPlanError> {
    let mut edges = BTreeMap::new();
    for &edge in input {
        let AnalyticShellCurve::Circle(circle) = edge.carrier else {
            return Err(AnalyticShellPlanError::ClosedEdgeRequiresCircle(edge.key));
        };
        let canonical = circle.param_range();
        if edge.logical_range != canonical {
            return Err(AnalyticShellPlanError::ClosedEdgeRequiresCanonicalPeriod(
                edge.key,
            ));
        }
        if let Some(source) = edge.source
            && !lineage_is_live(store, source)
        {
            return Err(AnalyticShellPlanError::StaleLineage(source));
        }
        if edges.insert(edge.key, edge).is_some() {
            return Err(AnalyticShellPlanError::DuplicateEdge(edge.key));
        }
    }
    Ok(edges)
}

fn prepare_faces(
    input: &[AnalyticShellFace],
    edges: &BTreeMap<AnalyticEdgeKey, AnalyticEdgeDeclaration>,
    store: &Store,
    tolerance: f64,
) -> Result<Vec<AnalyticShellFace>, AnalyticShellPlanError> {
    let mut face_keys = BTreeSet::new();
    let mut faces = Vec::with_capacity(input.len());
    for face in input {
        if !face_keys.insert(face.key) {
            return Err(AnalyticShellPlanError::DuplicateFace(face.key));
        }
        if face.loops.is_empty() || face.loops.iter().any(|loop_| loop_.fins.is_empty()) {
            return Err(AnalyticShellPlanError::EmptyFaceBoundary(face.key));
        }
        if let Some(source) = face.source
            && !lineage_is_live(store, source)
        {
            return Err(AnalyticShellPlanError::StaleLineage(source));
        }
        let mut copy = face.clone();
        for (loop_index, loop_) in copy.loops.iter_mut().enumerate() {
            validate_loop(face, loop_, loop_index, edges, tolerance)?;
            canonicalize_loop(loop_);
        }
        faces.push(copy);
    }
    Ok(faces)
}

fn validate_loop(
    face: &AnalyticShellFace,
    loop_: &AnalyticShellLoop,
    loop_index: usize,
    edges: &BTreeMap<AnalyticEdgeKey, AnalyticEdgeDeclaration>,
    tolerance: f64,
) -> Result<(), AnalyticShellPlanError> {
    for (index, fin) in loop_.fins.iter().enumerate() {
        let declaration = edges
            .get(&fin.edge)
            .ok_or(AnalyticShellPlanError::UnknownEdge {
                face: face.key,
                edge: fin.edge,
            })?;
        validate_pcurve_domain(face, declaration.key(), declaration.logical_range(), fin)?;
        if declaration.is_closed() {
            if loop_.fins.len() != 1 {
                return Err(AnalyticShellPlanError::ClosedEdgeRequiresSingleFinLoop {
                    face: face.key,
                    edge: fin.edge,
                });
            }
            validate_closed_pcurve_use(face, *declaration, fin)?;
            continue;
        }
        if fin.pcurve.closure_winding.is_some() {
            return Err(AnalyticShellPlanError::BoundedEdgeHasClosureWinding {
                face: face.key,
                edge: fin.edge,
            });
        }
        let AnalyticEdgeDeclaration::Bounded(edge) = declaration else {
            unreachable!("closed declaration returned above")
        };
        let next = loop_.fins[(index + 1) % loop_.fins.len()];
        let next_declaration =
            edges
                .get(&next.edge)
                .ok_or(AnalyticShellPlanError::UnknownEdge {
                    face: face.key,
                    edge: next.edge,
                })?;
        let AnalyticEdgeDeclaration::Bounded(next_edge) = next_declaration else {
            return Err(AnalyticShellPlanError::ClosedEdgeRequiresSingleFinLoop {
                face: face.key,
                edge: next.edge,
            });
        };
        if directed_head(*edge, fin.sense) != directed_tail(*next_edge, next.sense) {
            return Err(AnalyticShellPlanError::OpenLoop {
                face: face.key,
                loop_index,
            });
        }
        let current_end = directed_pcurve_endpoints(face, *edge, fin)?.1;
        let next_start = directed_pcurve_endpoints(face, *next_edge, &next)?.0;
        if !point2_bits_equal(current_end, next_start)
            && !certify_endpoint_incidence(
                face.surface.eval(current_end),
                face.surface.eval(next_start),
                tolerance,
            )
        {
            return Err(AnalyticShellPlanError::PcurveLoopNotClosed {
                face: face.key,
                loop_index,
                fin_index: index,
            });
        }
    }
    Ok(())
}

fn validate_closed_pcurve_use(
    face: &AnalyticShellFace,
    edge: AnalyticEdgeDeclaration,
    fin: &AnalyticShellFin,
) -> Result<(), AnalyticShellPlanError> {
    let winding =
        fin.pcurve
            .closure_winding
            .ok_or(AnalyticShellPlanError::MissingClosureWinding {
                face: face.key,
                edge: edge.key(),
            })?;
    let valid = match (face.surface, fin.pcurve.curve) {
        (AnalyticShellSurface::Plane(_), AnalyticShellPcurve::Circle(_)) => winding == [0, 0],
        (AnalyticShellSurface::Cylinder(cylinder), AnalyticShellPcurve::Line(line)) => {
            let range = edge.logical_range();
            let delta = line.dir() * (fin.pcurve.edge_to_pcurve.scale() * range.width());
            let period = cylinder.periodicity()[0].unwrap_or(core::f64::consts::TAU);
            delta.y == 0.0
                && matches!(winding[0], -1 | 1)
                && winding[1] == 0
                && delta.x == f64::from(winding[0]) * period
        }
        _ => false,
    };
    if valid {
        Ok(())
    } else {
        Err(AnalyticShellPlanError::InvalidClosureWinding {
            face: face.key,
            edge: edge.key(),
        })
    }
}

fn directed_pcurve_endpoints(
    face: &AnalyticShellFace,
    edge: AnalyticShellEdge,
    fin: &AnalyticShellFin,
) -> Result<(Point2, Point2), AnalyticShellPlanError> {
    let [start, end] = match fin.sense {
        Sense::Forward => [edge.range.lo, edge.range.hi],
        Sense::Reversed => [edge.range.hi, edge.range.lo],
    };
    let periods = face.surface.periodicity();
    let apply = |parameter| {
        let uv = fin
            .pcurve
            .curve
            .eval(fin.pcurve.edge_to_pcurve.map(parameter));
        fin.pcurve.chart.apply(uv, periods).map_err(|_| {
            AnalyticShellPlanError::PcurveOutsideFaceDomain {
                face: face.key,
                edge: edge.key,
            }
        })
    };
    Ok((apply(start)?, apply(end)?))
}

fn point2_bits_equal(first: Point2, second: Point2) -> bool {
    first.x.to_bits() == second.x.to_bits() && first.y.to_bits() == second.y.to_bits()
}

fn validate_pcurve_domain(
    face: &AnalyticShellFace,
    edge: AnalyticEdgeKey,
    range: ParamRange,
    fin: &AnalyticShellFin,
) -> Result<(), AnalyticShellPlanError> {
    let map = fin.pcurve.edge_to_pcurve;
    let first = map.map(range.lo);
    let second = map.map(range.hi);
    let active = ParamRange::new(first.min(second), first.max(second));
    if !active.is_finite() || active.lo >= active.hi {
        return Err(AnalyticShellPlanError::PcurveOutsideFaceDomain {
            face: face.key,
            edge,
        });
    }
    let periods = face.surface.periodicity();
    let (min, max) = fin.pcurve.curve.bounds(active);
    let min = fin.pcurve.chart.apply(min, periods).map_err(|_| {
        AnalyticShellPlanError::PcurveOutsideFaceDomain {
            face: face.key,
            edge,
        }
    })?;
    let max = fin.pcurve.chart.apply(max, periods).map_err(|_| {
        AnalyticShellPlanError::PcurveOutsideFaceDomain {
            face: face.key,
            edge,
        }
    })?;
    if !face.domain.contains([min.x, min.y]) || !face.domain.contains([max.x, max.y]) {
        return Err(AnalyticShellPlanError::PcurveOutsideFaceDomain {
            face: face.key,
            edge,
        });
    }
    Ok(())
}

fn canonicalize_loop(loop_: &mut AnalyticShellLoop) {
    let best = (0..loop_.fins.len())
        .min_by_key(|&start| {
            (0..loop_.fins.len())
                .map(|offset| {
                    let fin = loop_.fins[(start + offset) % loop_.fins.len()];
                    (fin.edge, sense_rank(fin.sense))
                })
                .collect::<Vec<_>>()
        })
        .unwrap_or(0);
    loop_.fins.rotate_left(best);
}

fn canonicalize_faces(faces: &mut [AnalyticShellFace]) {
    for face in faces.iter_mut() {
        face.loops.sort_by_key(loop_signature);
    }
    faces.sort_by_key(|face| face.key);
}

fn loop_signature(loop_: &AnalyticShellLoop) -> Vec<(AnalyticEdgeKey, u8)> {
    loop_
        .fins
        .iter()
        .map(|fin| (fin.edge, sense_rank(fin.sense)))
        .collect()
}

const fn sense_rank(sense: Sense) -> u8 {
    match sense {
        Sense::Forward => 0,
        Sense::Reversed => 1,
    }
}

fn collect_uses(
    faces: &[AnalyticShellFace],
    edges: &BTreeMap<AnalyticEdgeKey, AnalyticEdgeDeclaration>,
) -> Result<BTreeMap<AnalyticEdgeKey, [UseCandidate; 2]>, AnalyticShellPlanError> {
    let mut uses = BTreeMap::<AnalyticEdgeKey, Vec<UseCandidate>>::new();
    for face in faces {
        for (loop_index, loop_) in face.loops.iter().enumerate() {
            for (fin_index, fin) in loop_.fins.iter().enumerate() {
                uses.entry(fin.edge).or_default().push(UseCandidate {
                    face: face.key,
                    loop_index,
                    fin_index,
                    sense: fin.sense,
                    surface: face.surface,
                    pcurve: fin.pcurve,
                    source: face.source,
                });
            }
        }
    }
    for &key in edges.keys() {
        let count = uses.get(&key).map_or(0, Vec::len);
        if count != 2 {
            return Err(AnalyticShellPlanError::EdgeUseCount {
                edge: key,
                uses: count,
            });
        }
    }
    uses.into_iter()
        .map(|(key, mut values)| {
            values.sort_by_key(|use_| (use_.face, use_.loop_index, use_.fin_index));
            let pair: [UseCandidate; 2] = values.try_into().map_err(|values: Vec<_>| {
                AnalyticShellPlanError::EdgeUseCount {
                    edge: key,
                    uses: values.len(),
                }
            })?;
            Ok((key, pair))
        })
        .collect()
}

fn certify_connected_faces(
    faces: &[AnalyticShellFace],
    uses: &BTreeMap<AnalyticEdgeKey, [UseCandidate; 2]>,
) -> Result<(), AnalyticShellPlanError> {
    let mut adjacency = BTreeMap::<AnalyticFaceKey, BTreeSet<AnalyticFaceKey>>::new();
    for face in faces {
        adjacency.entry(face.key).or_default();
    }
    for pair in uses.values() {
        adjacency
            .entry(pair[0].face)
            .or_default()
            .insert(pair[1].face);
        adjacency
            .entry(pair[1].face)
            .or_default()
            .insert(pair[0].face);
    }
    let Some(&start) = adjacency.keys().next() else {
        return Err(AnalyticShellPlanError::EmptyShell);
    };
    let mut visited = BTreeSet::new();
    let mut queue = VecDeque::from([start]);
    while let Some(face) = queue.pop_front() {
        if visited.insert(face) {
            queue.extend(adjacency[&face].iter().copied());
        }
    }
    if visited.len() != faces.len() {
        return Err(AnalyticShellPlanError::DisconnectedShell);
    }
    Ok(())
}

fn certify_edge_pair(
    key: AnalyticEdgeKey,
    edge: AnalyticEdgeDeclaration,
    uses: [UseCandidate; 2],
    store: &Store,
    tolerance: f64,
) -> Result<AnalyticEdgeProof, AnalyticShellPlanError> {
    let certified = match (edge.carrier(), uses[0].surface, uses[1].surface) {
        (
            AnalyticShellCurve::Line(carrier),
            AnalyticShellSurface::Plane(a),
            AnalyticShellSurface::Plane(b),
        ) => {
            let [AnalyticShellPcurve::Line(pa), AnalyticShellPcurve::Line(pb)] =
                [uses[0].pcurve.curve, uses[1].pcurve.curve]
            else {
                return Err(AnalyticShellPlanError::UnsupportedPairing(key));
            };
            certify_paired_plane_line_residuals(
                carrier,
                edge.logical_range(),
                [a, b],
                [pa, pb],
                [uses[0].pcurve.edge_to_pcurve, uses[1].pcurve.edge_to_pcurve],
                tolerance,
            )
            .map(AnalyticEdgeProof::PlaneLine)
        }
        (
            AnalyticShellCurve::Line(carrier),
            AnalyticShellSurface::Plane(_),
            AnalyticShellSurface::Cylinder(_),
        ) => certify_ruling(carrier, edge.logical_range(), uses, store, tolerance),
        (
            AnalyticShellCurve::Line(carrier),
            AnalyticShellSurface::Cylinder(_),
            AnalyticShellSurface::Plane(_),
        ) => certify_ruling(carrier, edge.logical_range(), uses, store, tolerance),
        (
            AnalyticShellCurve::Line(carrier),
            AnalyticShellSurface::Cylinder(_),
            AnalyticShellSurface::Cylinder(_),
        ) => certify_cylinder_cylinder_ruling(carrier, edge.logical_range(), uses, tolerance),
        (
            AnalyticShellCurve::Circle(carrier),
            AnalyticShellSurface::Plane(plane),
            AnalyticShellSurface::Cylinder(cylinder),
        ) => certify_circle(carrier, uses, plane, cylinder, false, tolerance),
        (
            AnalyticShellCurve::Circle(carrier),
            AnalyticShellSurface::Cylinder(cylinder),
            AnalyticShellSurface::Plane(plane),
        ) => certify_circle(carrier, uses, plane, cylinder, true, tolerance),
        _ => return Err(AnalyticShellPlanError::UnsupportedPairing(key)),
    };
    certified.map_err(|source| AnalyticShellPlanError::PairingCertification { edge: key, source })
}

fn certify_cylinder_cylinder_ruling(
    carrier: Line,
    range: ParamRange,
    uses: [UseCandidate; 2],
    tolerance: f64,
) -> Result<AnalyticEdgeProof, IntersectionCertificateError> {
    let traces = uses.map(|use_| {
        let AnalyticShellSurface::Cylinder(cylinder) = use_.surface else {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        };
        let AnalyticShellPcurve::Line(pcurve) = use_.pcurve.curve else {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        };
        Ok(CylinderRulingTrace::new(
            cylinder,
            pcurve,
            use_.pcurve.edge_to_pcurve,
        ))
    });
    let [first, second] = traces;
    certify_paired_cylinder_cylinder_ruling_residuals(carrier, range, [first?, second?], tolerance)
        .map(AnalyticEdgeProof::CylinderCylinderRuling)
}

fn certify_ruling(
    carrier: Line,
    range: ParamRange,
    uses: [UseCandidate; 2],
    store: &Store,
    tolerance: f64,
) -> Result<AnalyticEdgeProof, IntersectionCertificateError> {
    let (plane_index, plane, cylinder) = match (uses[0].surface, uses[1].surface) {
        (AnalyticShellSurface::Plane(plane), AnalyticShellSurface::Cylinder(cylinder)) => {
            (0, plane, cylinder)
        }
        (AnalyticShellSurface::Cylinder(cylinder), AnalyticShellSurface::Plane(plane)) => {
            (1, plane, cylinder)
        }
        _ => return Err(IntersectionCertificateError::InvalidTraceFamily),
    };
    let cylinder_index = 1 - plane_index;
    let AnalyticShellPcurve::Line(plane_pcurve) = uses[plane_index].pcurve.curve else {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    };
    let AnalyticShellPcurve::Line(cylinder_pcurve) = uses[cylinder_index].pcurve.curve else {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    };
    let plane_trace = PlaneCylinderRulingTrace::Plane(PlaneRulingTrace::new(
        plane,
        plane_pcurve,
        uses[plane_index].pcurve.edge_to_pcurve,
    ));
    let cylinder_trace = PlaneCylinderRulingTrace::Cylinder(CylinderRulingTrace::new(
        cylinder,
        cylinder_pcurve,
        uses[cylinder_index].pcurve.edge_to_pcurve,
    ));
    let traces = if plane_index == 1 {
        [cylinder_trace, plane_trace]
    } else {
        [plane_trace, cylinder_trace]
    };
    match certify_paired_plane_cylinder_ruling_residuals(carrier, range, traces, tolerance) {
        Ok(certificate) => Ok(AnalyticEdgeProof::PlaneCylinderRuling(certificate)),
        Err(error) if lineage_ruling::is_exact_plane_axis_zero_refusal(&error) => {
            match lineage_ruling::certify_source_lineage_ruling_residuals(
                store,
                carrier,
                range,
                traces,
                uses[plane_index].source,
                tolerance,
            ) {
                Some(Ok(certificate)) => Ok(AnalyticEdgeProof::SourceLineagePlaneCylinderRuling(
                    certificate,
                )),
                Some(Err(source)) => Err(source),
                None => Err(error),
            }
        }
        Err(error) => Err(error),
    }
}

fn certify_circle(
    carrier: Circle,
    uses: [UseCandidate; 2],
    plane: Plane,
    cylinder: Cylinder,
    cylinder_first: bool,
    tolerance: f64,
) -> Result<AnalyticEdgeProof, IntersectionCertificateError> {
    let plane_index = usize::from(cylinder_first);
    let cylinder_index = 1 - plane_index;
    let AnalyticShellPcurve::Circle(plane_pcurve) = uses[plane_index].pcurve.curve else {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    };
    let AnalyticShellPcurve::Line(cylinder_pcurve) = uses[cylinder_index].pcurve.curve else {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    };
    let plane_trace = PlaneCylinderCircleTrace::Plane(PlaneCircleTrace::new(
        plane,
        plane_pcurve,
        uses[plane_index].pcurve.edge_to_pcurve,
    ));
    let cylinder_trace = PlaneCylinderCircleTrace::Cylinder(CylinderLongitudeTrace::new(
        cylinder,
        cylinder_pcurve,
        uses[cylinder_index].pcurve.edge_to_pcurve,
    ));
    let traces = if cylinder_first {
        [cylinder_trace, plane_trace]
    } else {
        [plane_trace, cylinder_trace]
    };
    certify_paired_plane_cylinder_circle_residuals(
        carrier,
        carrier.param_range(),
        traces,
        tolerance,
    )
    .map(AnalyticEdgeProof::PlaneCylinderCircle)
}

const fn directed_tail(edge: AnalyticShellEdge, sense: Sense) -> AnalyticVertexKey {
    match sense {
        Sense::Forward => edge.vertices[0],
        Sense::Reversed => edge.vertices[1],
    }
}

const fn directed_head(edge: AnalyticShellEdge, sense: Sense) -> AnalyticVertexKey {
    match sense {
        Sense::Forward => edge.vertices[1],
        Sense::Reversed => edge.vertices[0],
    }
}

fn same_point_bits(a: Point3, b: Point3) -> bool {
    a.x.to_bits() == b.x.to_bits()
        && a.y.to_bits() == b.y.to_bits()
        && a.z.to_bits() == b.z.to_bits()
}

/// Certify metric compatibility only after stable vertex identity has
/// already selected the incidence combinatorially.
///
/// Exact bits remain the zero-cost path. Otherwise outward interval
/// arithmetic must prove the complete squared separation no larger than the
/// admitted tolerance; an inconclusive comparison refuses rather than
/// rounding a near endpoint into incidence.
fn certify_endpoint_incidence(a: Point3, b: Point3, tolerance: f64) -> bool {
    if same_point_bits(a, b) {
        return true;
    }
    let distance_squared = [a.x, a.y, a.z]
        .into_iter()
        .zip([b.x, b.y, b.z])
        .fold(Interval::point(0.0), |sum, (left, right)| {
            sum + (Interval::point(left) - Interval::point(right)).square()
        });
    let allowed_squared = Interval::point(tolerance).square();
    distance_squared.hi().is_finite()
        && allowed_squared.lo().is_finite()
        && distance_squared.hi() <= allowed_squared.lo()
}

fn lineage_is_live(store: &Store, source: EntityRef) -> bool {
    match source {
        EntityRef::Body(id) => store.contains::<Body>(id),
        EntityRef::Region(id) => store.contains::<Region>(id),
        EntityRef::Shell(id) => store.contains::<Shell>(id),
        EntityRef::Face(id) => store.contains::<Face>(id),
        EntityRef::Loop(id) => store.contains::<Loop>(id),
        EntityRef::Fin(id) => store.contains::<Fin>(id),
        EntityRef::Edge(id) => store.contains::<Edge>(id),
        EntityRef::Vertex(id) => store.contains::<Vertex>(id),
        EntityRef::Curve(id) => store.contains::<crate::geom::CurveGeom>(id),
        EntityRef::Surface(id) => store.contains::<crate::geom::SurfaceGeom>(id),
        EntityRef::Point(id) => store.contains::<Point3>(id),
        EntityRef::Curve2d(id) => store.contains::<crate::geom::Curve2dGeom>(id),
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use kgeom::frame::Frame;
    use kgeom::vec::{Vec2, Vec3};

    const V0: AnalyticVertexKey = AnalyticVertexKey::new(0);
    const V1: AnalyticVertexKey = AnalyticVertexKey::new(1);
    const V2: AnalyticVertexKey = AnalyticVertexKey::new(2);
    const V3: AnalyticVertexKey = AnalyticVertexKey::new(3);
    const E0: AnalyticEdgeKey = AnalyticEdgeKey::new(0);
    const E1: AnalyticEdgeKey = AnalyticEdgeKey::new(1);
    const E2: AnalyticEdgeKey = AnalyticEdgeKey::new(2);
    const E3: AnalyticEdgeKey = AnalyticEdgeKey::new(3);
    const E4: AnalyticEdgeKey = AnalyticEdgeKey::new(4);
    const E5: AnalyticEdgeKey = AnalyticEdgeKey::new(5);

    fn map(scale: f64, offset: f64) -> AffineParamMap1d {
        AffineParamMap1d::new(scale, offset).unwrap()
    }

    fn line_use(
        edge: AnalyticEdgeKey,
        sense: Sense,
        origin: Point2,
        dir: Vec2,
    ) -> AnalyticShellFin {
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(Line2d::new(origin, dir).unwrap()),
                map(1.0, 0.0),
            ),
        )
    }

    pub(crate) fn half_cylinder_input() -> AnalyticShellInput {
        let cylinder_frame = Frame::world();
        let cylinder = Cylinder::new(cylinder_frame, 1.0).unwrap();
        let bottom_circle = Circle::new(cylinder_frame, 1.0).unwrap();
        let top_frame = cylinder_frame.with_origin(Point3::new(0.0, 0.0, 1.0));
        let top_circle = Circle::new(top_frame, 1.0).unwrap();
        let arc = ParamRange::new(0.0, core::f64::consts::PI);
        let p0 = bottom_circle.eval(arc.lo);
        let p1 = bottom_circle.eval(arc.hi);
        let p2 = top_circle.eval(arc.lo);
        let p3 = top_circle.eval(arc.hi);
        let vertices = vec![
            AnalyticShellVertex::new(V0, p0),
            AnalyticShellVertex::new(V1, p1),
            AnalyticShellVertex::new(V2, p2),
            AnalyticShellVertex::new(V3, p3),
        ];
        let edges = vec![
            AnalyticShellEdge::new(E0, [V0, V1], AnalyticShellCurve::Circle(bottom_circle), arc),
            AnalyticShellEdge::new(E1, [V2, V3], AnalyticShellCurve::Circle(top_circle), arc),
            AnalyticShellEdge::new(
                E2,
                [V0, V2],
                AnalyticShellCurve::Line(Line::new(p0, Vec3::new(0.0, 0.0, 1.0)).unwrap()),
                ParamRange::new(0.0, 1.0),
            ),
            AnalyticShellEdge::new(
                E3,
                [V1, V3],
                AnalyticShellCurve::Line(Line::new(p1, Vec3::new(0.0, 0.0, 1.0)).unwrap()),
                ParamRange::new(0.0, 1.0),
            ),
            AnalyticShellEdge::new(
                E4,
                [V0, V1],
                AnalyticShellCurve::Line(Line::new(p0, p1 - p0).unwrap()),
                ParamRange::new(0.0, (p1 - p0).norm()),
            ),
            AnalyticShellEdge::new(
                E5,
                [V2, V3],
                AnalyticShellCurve::Line(Line::new(p2, p3 - p2).unwrap()),
                ParamRange::new(0.0, (p3 - p2).norm()),
            ),
        ];

        let bottom_frame = Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let cut_frame = Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let bottom = Plane::new(bottom_frame);
        let top = Plane::new(top_frame);
        let cut = Plane::new(cut_frame);
        let bottom_pcurve_circle =
            Circle2d::new(Point2::new(0.0, 0.0), 1.0, Vec2::new(1.0, 0.0)).unwrap();
        let top_pcurve_circle = bottom_pcurve_circle;
        let bottom_diameter_start = bottom_pcurve_circle.eval(0.0);
        let bottom_diameter_end = bottom_pcurve_circle.eval(-core::f64::consts::PI);
        let top_diameter_start = top_pcurve_circle.eval(0.0);
        let top_diameter_end = top_pcurve_circle.eval(core::f64::consts::PI);

        let cylinder_loop = AnalyticShellLoop::new(vec![
            line_use(
                E2,
                Sense::Reversed,
                Point2::new(0.0, 0.0),
                Vec2::new(0.0, 1.0),
            ),
            AnalyticShellFin::new(
                E0,
                Sense::Forward,
                AnalyticPcurveUse::new(
                    AnalyticShellPcurve::Line(
                        Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
                    ),
                    map(1.0, 0.0),
                ),
            ),
            line_use(
                E3,
                Sense::Forward,
                Point2::new(core::f64::consts::PI, 0.0),
                Vec2::new(0.0, 1.0),
            ),
            AnalyticShellFin::new(
                E1,
                Sense::Reversed,
                AnalyticPcurveUse::new(
                    AnalyticShellPcurve::Line(
                        Line2d::new(Point2::new(0.0, 1.0), Vec2::new(1.0, 0.0)).unwrap(),
                    ),
                    map(1.0, 0.0),
                ),
            ),
        ]);
        let bottom_loop = AnalyticShellLoop::new(vec![
            AnalyticShellFin::new(
                E0,
                Sense::Reversed,
                AnalyticPcurveUse::new(
                    AnalyticShellPcurve::Circle(bottom_pcurve_circle),
                    map(-1.0, 0.0),
                ),
            ),
            line_use(
                E4,
                Sense::Forward,
                bottom_diameter_start,
                bottom_diameter_end - bottom_diameter_start,
            ),
        ]);
        let top_loop = AnalyticShellLoop::new(vec![
            AnalyticShellFin::new(
                E1,
                Sense::Forward,
                AnalyticPcurveUse::new(
                    AnalyticShellPcurve::Circle(top_pcurve_circle),
                    map(1.0, 0.0),
                ),
            ),
            line_use(
                E5,
                Sense::Reversed,
                top_diameter_start,
                top_diameter_end - top_diameter_start,
            ),
        ]);
        let cut_loop = AnalyticShellLoop::new(vec![
            line_use(
                E4,
                Sense::Reversed,
                Point2::new(1.0, 0.0),
                Vec2::new(-1.0, 0.0),
            ),
            line_use(
                E2,
                Sense::Forward,
                Point2::new(1.0, 0.0),
                Vec2::new(0.0, 1.0),
            ),
            line_use(
                E5,
                Sense::Forward,
                Point2::new(1.0, 1.0),
                Vec2::new(-1.0, 0.0),
            ),
            line_use(
                E3,
                Sense::Reversed,
                Point2::new(-1.0, 0.0),
                Vec2::new(0.0, 1.0),
            ),
        ]);
        let faces = vec![
            AnalyticShellFace::new(
                AnalyticFaceKey::new(0),
                AnalyticShellSurface::Cylinder(cylinder),
                Sense::Forward,
                FaceDomain::from_bounds(0.0, core::f64::consts::PI, 0.0, 1.0).unwrap(),
                vec![cylinder_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(1),
                AnalyticShellSurface::Plane(bottom),
                Sense::Forward,
                FaceDomain::from_bounds(-1.0, 1.0, -1.0, 0.0).unwrap(),
                vec![bottom_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(2),
                AnalyticShellSurface::Plane(top),
                Sense::Forward,
                FaceDomain::from_bounds(-1.0, 1.0, 0.0, 1.0).unwrap(),
                vec![top_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(3),
                AnalyticShellSurface::Plane(cut),
                Sense::Forward,
                FaceDomain::from_bounds(-1.0, 1.0, 0.0, 1.0).unwrap(),
                vec![cut_loop],
            ),
        ];
        AnalyticShellInput::new(vertices, edges, faces)
    }

    pub(crate) fn full_cylinder_input() -> AnalyticShellInput {
        let cylinder_frame = Frame::world();
        let cylinder = Cylinder::new(cylinder_frame, 1.0).unwrap();
        let bottom_circle = Circle::new(cylinder_frame, 1.0).unwrap();
        let top_frame = cylinder_frame.with_origin(Point3::new(0.0, 0.0, 1.0));
        let top_circle = Circle::new(top_frame, 1.0).unwrap();
        let full = bottom_circle.param_range();
        let closed_edges = vec![
            AnalyticShellClosedEdge::new(E0, AnalyticShellCurve::Circle(bottom_circle), full),
            AnalyticShellClosedEdge::new(E1, AnalyticShellCurve::Circle(top_circle), full),
        ];

        let side_use = |edge, sense, height| {
            AnalyticShellFin::new(
                edge,
                sense,
                AnalyticPcurveUse::new(
                    AnalyticShellPcurve::Line(
                        Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap(),
                    ),
                    map(1.0, 0.0),
                )
                .with_closure_winding([1, 0]),
            )
        };
        let cap_circle = Circle2d::new(Point2::new(0.0, 0.0), 1.0, Vec2::new(1.0, 0.0)).unwrap();
        let cap_use = |edge, sense, scale, offset| {
            AnalyticShellFin::new(
                edge,
                sense,
                AnalyticPcurveUse::new(AnalyticShellPcurve::Circle(cap_circle), map(scale, offset))
                    .with_closure_winding([0, 0]),
            )
        };
        let bottom_frame = Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let faces = vec![
            AnalyticShellFace::new(
                AnalyticFaceKey::new(0),
                AnalyticShellSurface::Cylinder(cylinder),
                Sense::Forward,
                FaceDomain::from_bounds(0.0, core::f64::consts::TAU, 0.0, 1.0).unwrap(),
                vec![
                    AnalyticShellLoop::new(vec![side_use(E0, Sense::Forward, 0.0)]),
                    AnalyticShellLoop::new(vec![side_use(E1, Sense::Reversed, 1.0)]),
                ],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(1),
                AnalyticShellSurface::Plane(Plane::new(bottom_frame)),
                Sense::Forward,
                FaceDomain::from_bounds(-1.0, 1.0, -1.0, 1.0).unwrap(),
                vec![AnalyticShellLoop::new(vec![cap_use(
                    E0,
                    Sense::Reversed,
                    -1.0,
                    core::f64::consts::TAU,
                )])],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(2),
                AnalyticShellSurface::Plane(Plane::new(top_frame)),
                Sense::Forward,
                FaceDomain::from_bounds(-1.0, 1.0, -1.0, 1.0).unwrap(),
                vec![AnalyticShellLoop::new(vec![cap_use(
                    E1,
                    Sense::Forward,
                    1.0,
                    0.0,
                )])],
            ),
        ];
        AnalyticShellInput::new(Vec::new(), Vec::new(), faces).with_closed_edges(closed_edges)
    }

    #[test]
    fn endpoint_free_full_cylinder_preflights_without_synthetic_vertices() {
        let input = full_cylinder_input();
        let prepared = prepare_analytic_shell(&input, &Store::new(), 1.0e-12).unwrap();
        assert!(prepared.vertices().is_empty());
        assert!(prepared.edges().is_empty());
        assert_eq!(prepared.closed_edges().len(), 2);
        assert!(
            prepared
                .closed_edges()
                .iter()
                .all(|edge| matches!(edge.proof(), AnalyticEdgeProof::PlaneCylinderCircle(_)))
        );
    }

    #[test]
    fn endpoint_free_tampering_fails_closed() {
        let store = Store::new();

        let mut line = full_cylinder_input();
        line.closed_edges[0].carrier = AnalyticShellCurve::Line(
            Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
        );
        assert_eq!(
            prepare_analytic_shell(&line, &store, 1.0e-12),
            Err(AnalyticShellPlanError::ClosedEdgeRequiresCircle(E0))
        );

        let mut partial = full_cylinder_input();
        partial.closed_edges[0].logical_range = ParamRange::new(0.0, core::f64::consts::PI);
        assert_eq!(
            prepare_analytic_shell(&partial, &store, 1.0e-12),
            Err(AnalyticShellPlanError::ClosedEdgeRequiresCanonicalPeriod(
                E0
            ))
        );

        let mut synthetic = full_cylinder_input();
        synthetic
            .vertices
            .push(AnalyticShellVertex::new(V0, Point3::new(1.0, 0.0, 0.0)));
        assert_eq!(
            prepare_analytic_shell(&synthetic, &store, 1.0e-12),
            Err(AnalyticShellPlanError::UnusedVertex(V0))
        );

        let mut duplicate = full_cylinder_input();
        duplicate.closed_edges.push(duplicate.closed_edges[0]);
        assert_eq!(
            prepare_analytic_shell(&duplicate, &store, 1.0e-12),
            Err(AnalyticShellPlanError::DuplicateEdge(E0))
        );

        let mut missing_winding = full_cylinder_input();
        missing_winding.faces[0].loops[0].fins[0]
            .pcurve
            .closure_winding = None;
        assert_eq!(
            prepare_analytic_shell(&missing_winding, &store, 1.0e-12),
            Err(AnalyticShellPlanError::MissingClosureWinding {
                face: AnalyticFaceKey::new(0),
                edge: E0,
            })
        );

        let mut bad_winding = full_cylinder_input();
        bad_winding.faces[0].loops[0].fins[0].pcurve.closure_winding = Some([0, 0]);
        assert_eq!(
            prepare_analytic_shell(&bad_winding, &store, 1.0e-12),
            Err(AnalyticShellPlanError::InvalidClosureWinding {
                face: AnalyticFaceKey::new(0),
                edge: E0,
            })
        );

        let mut bad_chart = full_cylinder_input();
        bad_chart.faces[1].loops[0].fins[0].pcurve.chart = PcurveChart::shifted([1, 0]);
        assert!(matches!(
            prepare_analytic_shell(&bad_chart, &store, 1.0e-12),
            Err(AnalyticShellPlanError::PcurveOutsideFaceDomain { .. })
        ));

        let mut excess_use = full_cylinder_input();
        let duplicate_loop = excess_use.faces[0].loops[0].clone();
        excess_use.faces[0].loops.push(duplicate_loop);
        assert_eq!(
            prepare_analytic_shell(&excess_use, &store, 1.0e-12),
            Err(AnalyticShellPlanError::EdgeUseCount { edge: E0, uses: 3 })
        );

        let mut same_sense = full_cylinder_input();
        same_sense.faces[1].loops[0].fins[0].sense = Sense::Forward;
        assert_eq!(
            prepare_analytic_shell(&same_sense, &store, 1.0e-12),
            Err(AnalyticShellPlanError::EdgeUsesNotOpposed(E0))
        );

        let mut self_adjacent = full_cylinder_input();
        let mut second_use = self_adjacent.faces[1].loops.remove(0);
        second_use.fins[0].pcurve = self_adjacent.faces[0].loops[0].fins[0].pcurve;
        self_adjacent.faces[0].loops.push(second_use);
        self_adjacent.faces.remove(1);
        assert_eq!(
            prepare_analytic_shell(&self_adjacent, &store, 1.0e-12),
            Err(AnalyticShellPlanError::SelfAdjacentEdge(E0))
        );
    }

    #[test]
    fn endpoint_free_source_lineage_must_remain_live() {
        let mut store = Store::new();
        let stale_edge = {
            let mut transaction = store.transaction().unwrap();
            let output = transaction
                .assemble_analytic_shell(&half_cylinder_input(), 1.0e-12)
                .unwrap();
            let edge = output.edges()[0].1;
            transaction.rollback().unwrap();
            edge
        };
        let mut input = full_cylinder_input();
        input.closed_edges[0].source = Some(EntityRef::Edge(stale_edge));
        assert_eq!(
            prepare_analytic_shell(&input, &store, 1.0e-12),
            Err(AnalyticShellPlanError::StaleLineage(EntityRef::Edge(
                stale_edge
            )))
        );
    }

    #[test]
    fn preflight_is_permutation_invariant_and_retains_shared_edge_identity() {
        let store = Store::new();
        let input = half_cylinder_input();
        let first = prepare_analytic_shell(&input, &store, 1.0e-12).unwrap();
        let mut permuted = input.clone();
        permuted.vertices.reverse();
        permuted.edges.rotate_left(2);
        permuted.faces.reverse();
        for face in &mut permuted.faces {
            for loop_ in &mut face.loops {
                loop_.fins.rotate_left(1);
            }
        }
        let second = prepare_analytic_shell(&permuted, &store, 1.0e-12).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.edges.len(), 6);
        for prepared in &first.edges {
            assert_eq!(prepared.uses[0].sense, prepared.uses[1].sense.flipped());
            assert_eq!(prepared.edge.key, prepared.edge().key());
        }
        assert!(matches!(
            first.edges[0].proof,
            AnalyticEdgeProof::PlaneCylinderCircle(_)
        ));
        assert!(matches!(
            first.edges[2].proof,
            AnalyticEdgeProof::PlaneCylinderRuling(_)
        ));
        assert!(matches!(
            first.edges[4].proof,
            AnalyticEdgeProof::PlaneLine(_)
        ));
    }

    #[test]
    fn adversarial_inputs_fail_before_store_mutation() {
        let mut store = Store::new();
        let body = crate::make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let before = counts(&store);

        let mut duplicate = half_cylinder_input();
        duplicate.vertices.push(duplicate.vertices[0]);
        assert_eq!(
            prepare_analytic_shell(&duplicate, &store, 1.0e-12),
            Err(AnalyticShellPlanError::DuplicateVertex(V0))
        );

        let mut open = half_cylinder_input();
        open.faces[0].loops[0].fins.swap(0, 1);
        assert!(matches!(
            prepare_analytic_shell(&open, &store, 1.0e-12),
            Err(AnalyticShellPlanError::OpenLoop { .. })
        ));

        let mut mismatch = half_cylinder_input();
        mismatch.vertices[0].position.x += 0.25;
        assert_eq!(
            prepare_analytic_shell(&mismatch, &store, 1.0e-12),
            Err(AnalyticShellPlanError::CarrierEndpointMismatch {
                edge: E0,
                endpoint: 0,
            })
        );

        let mut open_pcurve = half_cylinder_input();
        open_pcurve.faces[1].loops[0].fins[1].pcurve.curve = AnalyticShellPcurve::Line(
            Line2d::new(Point2::new(1.0, -0.25), Vec2::new(-1.0, 0.0)).unwrap(),
        );
        assert_eq!(
            prepare_analytic_shell(&open_pcurve, &store, 1.0e-12),
            Err(AnalyticShellPlanError::PcurveLoopNotClosed {
                face: AnalyticFaceKey::new(1),
                loop_index: 0,
                fin_index: 0,
            })
        );

        let mut same_sense = half_cylinder_input();
        same_sense.faces[1].loops[0].fins[0].sense = Sense::Forward;
        assert!(matches!(
            prepare_analytic_shell(&same_sense, &store, 1.0e-12),
            Err(AnalyticShellPlanError::OpenLoop { .. })
                | Err(AnalyticShellPlanError::EdgeUsesNotOpposed(_))
        ));

        let mut live_lineage = half_cylinder_input();
        live_lineage.faces[0].source = Some(EntityRef::Body(body));
        assert!(prepare_analytic_shell(&live_lineage, &store, 1.0e-12).is_ok());
        assert_eq!(counts(&store), before);
    }

    #[test]
    fn stable_vertex_identity_still_requires_certified_endpoint_distance() {
        let mut within = half_cylinder_input();
        within.vertices[0].position.x += 5.0e-13;
        let store = Store::new();
        assert!(prepare_analytic_shell(&within, &store, 1.0e-12).is_ok());
        assert_eq!(
            prepare_analytic_shell(&within, &store, 1.0e-14),
            Err(AnalyticShellPlanError::CarrierEndpointMismatch {
                edge: E0,
                endpoint: 0,
            })
        );
    }

    #[test]
    fn stable_loop_identity_uses_surface_lifted_pcurve_distance() {
        let mut within = half_cylinder_input();
        within.faces[1].loops[0].fins[1].pcurve.curve = AnalyticShellPcurve::Line(
            Line2d::new(Point2::new(1.0, -5.0e-13), Vec2::new(-1.0, 0.0)).unwrap(),
        );
        let store = Store::new();
        assert!(prepare_analytic_shell(&within, &store, 1.0e-12).is_ok());
        assert_eq!(
            prepare_analytic_shell(&within, &store, 1.0e-14),
            Err(AnalyticShellPlanError::PcurveLoopNotClosed {
                face: AnalyticFaceKey::new(1),
                loop_index: 0,
                fin_index: 0,
            })
        );
    }

    fn counts(store: &Store) -> [usize; 12] {
        [
            store.count::<Body>(),
            store.count::<Region>(),
            store.count::<Shell>(),
            store.count::<Face>(),
            store.count::<Loop>(),
            store.count::<Fin>(),
            store.count::<Edge>(),
            store.count::<Vertex>(),
            store.count::<crate::geom::CurveGeom>(),
            store.count::<crate::geom::SurfaceGeom>(),
            store.count::<Point3>(),
            store.count::<crate::geom::Curve2dGeom>(),
        ]
    }
}
