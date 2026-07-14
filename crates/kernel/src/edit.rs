//! Checked semantic topology edits at the supported facade boundary.

use kcore::operation::OperationContext;
use kgeom::vec::Point3;
use ktopo::entity::{
    FinPcurve, ParamMap1d, PcurveChart as RawPcurveChart,
    PcurveEndpointKind as RawPcurveEndpointKind, PcurveSeam as RawPcurveSeam,
    SeamSide as RawPcurveSeamSide, SurfaceParameter as RawSurfaceParameter,
};
use ktopo::euler::FinPcurvePair;
use ktopo::transaction::{
    FullCommitRequirement as RawFullCommitRequirement, ToleranceGrowth as RawToleranceGrowth,
    ToleranceGrowthTarget as RawToleranceGrowthTarget, Transaction,
};

use crate::error::{Error, Result};
use crate::operation::adapt_transaction_check;
use crate::session::PartEdit;
use crate::{
    BodyCheckReport, BodyId, BoundedCurve, ChangeJournal, CurveId, EdgeId, EntityKind, FaceId,
    FinId, LoopId, OperationOutcome, OperationSettings, ParamRange, PartId, PcurveId, RegionId,
    Sense, ShellId, SurfaceId, ToleranceBudgetId, VertexId,
};

/// Validated affine correspondence from edge parameter `t` to pcurve
/// parameter `q = scale * t + offset`.
///
/// A nonzero finite scale keeps the map invertible. Negative scale explicitly
/// represents a pcurve authored opposite to increasing edge parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PcurveParameterMap {
    scale: f64,
    offset: f64,
}

impl PcurveParameterMap {
    /// Identity edge-to-pcurve correspondence.
    pub const fn identity() -> Self {
        Self {
            scale: 1.0,
            offset: 0.0,
        }
    }

    /// Construct a finite invertible affine correspondence.
    pub fn affine(scale: f64, offset: f64) -> Result<Self> {
        ParamMap1d::affine(scale, offset)?;
        Ok(Self { scale, offset })
    }

    /// Map an edge parameter to the authored pcurve parameter.
    pub fn map(self, edge_parameter: f64) -> f64 {
        self.scale * edge_parameter + self.offset
    }

    /// Map an authored pcurve parameter back to the edge parameter.
    pub fn inverse(self, pcurve_parameter: f64) -> f64 {
        (pcurve_parameter - self.offset) / self.scale
    }

    /// Affine scale; its sign is the relative parameter orientation.
    pub const fn scale(self) -> f64 {
        self.scale
    }

    /// Affine offset.
    pub const fn offset(self) -> f64 {
        self.offset
    }

    pub(crate) fn from_raw(map: ParamMap1d) -> Self {
        Self {
            scale: map.scale(),
            offset: map.offset(),
        }
    }

    fn into_raw(self) -> ParamMap1d {
        ParamMap1d::affine(self.scale, self.offset)
            .expect("facade pcurve parameter maps are validated at construction")
    }
}

/// Integer-period branch selection for a pcurve on a periodic surface.
///
/// Each component is a whole-period shift in surface `(u, v)`. Whether a
/// nonzero component is meaningful depends on the destination surface and is
/// checked before or during the semantic edit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct PcurveChart {
    period_shifts: [i32; 2],
}

impl PcurveChart {
    /// The authored pcurve branch without a period translation.
    pub const fn identity() -> Self {
        Self {
            period_shifts: [0, 0],
        }
    }

    /// Select branches using exact integer period counts in surface `(u, v)`.
    pub const fn integer(period_shifts: [i32; 2]) -> Self {
        Self { period_shifts }
    }

    /// Validate numeric whole-period branch counts in surface `(u, v)`.
    ///
    /// This constructor is useful at interchange and application-data
    /// boundaries where numeric values have not yet been narrowed to integer
    /// topology metadata. Nonfinite, fractional, and out-of-range counts are
    /// rejected without starting an edit transaction.
    pub fn shifted(period_shifts: [f64; 2]) -> Result<Self> {
        let mut exact = [0; 2];
        for (index, shift) in period_shifts.into_iter().enumerate() {
            if !shift.is_finite()
                || shift.fract() != 0.0
                || shift < f64::from(i32::MIN)
                || shift > f64::from(i32::MAX)
            {
                return Err(kcore::error::Error::InvalidGeometry {
                    reason: "pcurve chart shifts must be finite in-range integers",
                }
                .into());
            }
            exact[index] = shift as i32;
        }
        Ok(Self::integer(exact))
    }

    /// Integer whole-period shifts in surface `(u, v)`.
    pub const fn period_shifts(self) -> [i32; 2] {
        self.period_shifts
    }

    /// Whether the authored pcurve branch is used unchanged.
    pub const fn is_identity(self) -> bool {
        self.period_shifts[0] == 0 && self.period_shifts[1] == 0
    }

    fn from_raw(chart: RawPcurveChart) -> Self {
        Self::integer(chart.period_shifts())
    }

    fn into_raw(self) -> RawPcurveChart {
        RawPcurveChart::shifted(self.period_shifts)
    }
}

/// Topological meaning of one pcurve endpoint in increasing edge-parameter
/// direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PcurveEndpointKind {
    /// An ordinary endpoint where the supporting surface is regular.
    #[default]
    Regular,
    /// The endpoint lies on a degenerate surface iso-line such as a sphere
    /// pole or cone apex.
    SurfaceSingularity,
}

impl PcurveEndpointKind {
    fn from_raw(kind: RawPcurveEndpointKind) -> Self {
        match kind {
            RawPcurveEndpointKind::Regular => Self::Regular,
            RawPcurveEndpointKind::SurfaceSingularity => Self::SurfaceSingularity,
        }
    }

    fn into_raw(self) -> RawPcurveEndpointKind {
        match self {
            Self::Regular => RawPcurveEndpointKind::Regular,
            Self::SurfaceSingularity => RawPcurveEndpointKind::SurfaceSingularity,
        }
    }
}

/// One of the two supporting-surface parameter directions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SurfaceParameter {
    /// First surface parameter (`u`).
    U,
    /// Second surface parameter (`v`).
    V,
}

impl SurfaceParameter {
    fn from_raw(direction: RawSurfaceParameter) -> Self {
        match direction {
            RawSurfaceParameter::U => Self::U,
            RawSurfaceParameter::V => Self::V,
        }
    }

    fn into_raw(self) -> RawSurfaceParameter {
        match self {
            Self::U => RawSurfaceParameter::U,
            Self::V => RawSurfaceParameter::V,
        }
    }
}

/// Which full-period face-domain boundary represents a seam use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PcurveSeamSide {
    /// Lower bound of the face domain in the seam direction.
    Lower,
    /// Upper bound of the face domain in the seam direction.
    Upper,
}

impl PcurveSeamSide {
    fn from_raw(side: RawPcurveSeamSide) -> Self {
        match side {
            RawPcurveSeamSide::Lower => Self::Lower,
            RawPcurveSeamSide::Upper => Self::Upper,
        }
    }

    fn into_raw(self) -> RawPcurveSeamSide {
        match self {
            Self::Lower => RawPcurveSeamSide::Lower,
            Self::Upper => RawPcurveSeamSide::Upper,
        }
    }
}

/// Explicit role of a pcurve lying on a periodic face-chart cut.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PcurveSeam {
    direction: SurfaceParameter,
    side: PcurveSeamSide,
}

impl PcurveSeam {
    /// Declare a lower or upper seam use in one surface direction.
    pub const fn new(direction: SurfaceParameter, side: PcurveSeamSide) -> Self {
        Self { direction, side }
    }

    /// Periodic surface direction containing the chart cut.
    pub const fn direction(self) -> SurfaceParameter {
        self.direction
    }

    /// Lower or upper boundary of the face chart.
    pub const fn side(self) -> PcurveSeamSide {
        self.side
    }

    fn from_raw(seam: RawPcurveSeam) -> Self {
        Self::new(
            SurfaceParameter::from_raw(seam.direction()),
            PcurveSeamSide::from_raw(seam.side()),
        )
    }

    fn into_raw(self) -> RawPcurveSeam {
        RawPcurveSeam::new(self.direction.into_raw(), self.side.into_raw())
    }
}

/// Complete topology metadata attached to one fin's pcurve use.
///
/// Endpoint kinds are ordered by increasing edge parameter, independent of
/// fin traversal sense. Closure winding is a whole-period displacement in
/// surface `(u, v)` and is valid only for a ring or same-vertex closed edge.
/// Seam roles are validated against a full-period face domain. These meanings
/// require edge, face, and surface context, so inconsistent combinations are
/// rejected failure-atomically by checked semantic edits rather than guessed
/// by this context-free value type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct PcurveMetadata {
    chart: PcurveChart,
    endpoint_kinds: [PcurveEndpointKind; 2],
    closure_winding: Option<[i32; 2]>,
    seam: Option<PcurveSeam>,
}

impl PcurveMetadata {
    /// Ordinary non-periodic, open, regular-endpoint metadata.
    pub const fn regular() -> Self {
        Self {
            chart: PcurveChart::identity(),
            endpoint_kinds: [PcurveEndpointKind::Regular; 2],
            closure_winding: None,
            seam: None,
        }
    }

    /// Select an explicit periodic chart branch.
    pub const fn with_chart(mut self, chart: PcurveChart) -> Self {
        self.chart = chart;
        self
    }

    /// Mark endpoint semantics in increasing edge-parameter order.
    pub const fn with_endpoint_kinds(mut self, endpoint_kinds: [PcurveEndpointKind; 2]) -> Self {
        self.endpoint_kinds = endpoint_kinds;
        self
    }

    /// Declare a closed use's whole-period `(u, v)` displacement.
    pub const fn with_closure_winding(mut self, winding: [i32; 2]) -> Self {
        self.closure_winding = Some(winding);
        self
    }

    /// Declare this use to occupy one side of a periodic chart seam.
    pub const fn with_seam(mut self, seam: PcurveSeam) -> Self {
        self.seam = Some(seam);
        self
    }

    /// Explicit periodic chart selection.
    pub const fn chart(self) -> PcurveChart {
        self.chart
    }

    /// Endpoint semantics in increasing edge-parameter order.
    pub const fn endpoint_kinds(self) -> [PcurveEndpointKind; 2] {
        self.endpoint_kinds
    }

    /// Whole-period displacement of a closed use, when declared.
    pub const fn closure_winding(self) -> Option<[i32; 2]> {
        self.closure_winding
    }

    /// Explicit periodic seam role, when declared.
    pub const fn seam(self) -> Option<PcurveSeam> {
        self.seam
    }

    pub(crate) fn from_raw(use_: FinPcurve) -> Self {
        Self {
            chart: PcurveChart::from_raw(use_.chart()),
            endpoint_kinds: use_.endpoint_kinds().map(PcurveEndpointKind::from_raw),
            closure_winding: use_.closure_winding(),
            seam: use_.seam().map(PcurveSeam::from_raw),
        }
    }

    fn apply_to_raw(self, mut use_: FinPcurve) -> FinPcurve {
        use_ = use_
            .with_chart(self.chart.into_raw())
            .with_endpoint_kinds(self.endpoint_kinds.map(PcurveEndpointKind::into_raw));
        if let Some(winding) = self.closure_winding {
            use_ = use_.with_closure_winding(winding);
        }
        if let Some(seam) = self.seam {
            use_ = use_.with_seam(seam.into_raw());
        }
        use_
    }
}

/// One existing pcurve restricted to a finite parameter interval.
///
/// [`Self::new`] uses the identity edge-to-pcurve map. Call
/// [`Self::with_parameter_map`] for a reversed, shifted, or scaled authored
/// parameterization and [`Self::with_metadata`] for periodic charts, singular
/// endpoints, closed uses, and seam roles.
#[derive(Debug, Clone, PartialEq)]
pub struct BoundedPcurve {
    pcurve: PcurveId,
    range: ParamRange,
    parameter_map: PcurveParameterMap,
    metadata: PcurveMetadata,
}

impl BoundedPcurve {
    /// Bind an opaque pcurve identity to its active finite interval.
    pub const fn new(pcurve: PcurveId, range: ParamRange) -> Self {
        Self {
            pcurve,
            range,
            parameter_map: PcurveParameterMap::identity(),
            metadata: PcurveMetadata::regular(),
        }
    }

    /// Replace the identity edge-to-pcurve correspondence.
    pub const fn with_parameter_map(mut self, parameter_map: PcurveParameterMap) -> Self {
        self.parameter_map = parameter_map;
        self
    }

    /// Attach periodic-chart, endpoint, closure, and seam semantics.
    pub const fn with_metadata(mut self, metadata: PcurveMetadata) -> Self {
        self.metadata = metadata;
        self
    }

    /// Exact graph-owned pcurve identity.
    pub fn pcurve(&self) -> PcurveId {
        self.pcurve.clone()
    }

    /// Active pcurve interval.
    pub const fn range(&self) -> ParamRange {
        self.range
    }

    /// Edge-to-pcurve parameter correspondence.
    pub const fn parameter_map(&self) -> PcurveParameterMap {
        self.parameter_map
    }

    /// Topological incidence metadata for this pcurve use.
    pub const fn metadata(&self) -> PcurveMetadata {
        self.metadata
    }

    fn into_raw_use(self) -> Result<FinPcurve> {
        Ok(self.metadata.apply_to_raw(FinPcurve::new(
            self.pcurve.raw(),
            self.range,
            self.parameter_map.into_raw(),
        )?))
    }
}

/// One facade topology identity eligible for metric-tolerance growth.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum ToleranceGrowthTarget {
    /// Face tolerance.
    Face(FaceId),
    /// Edge tolerance.
    Edge(EdgeId),
    /// Vertex tolerance.
    Vertex(VertexId),
}

/// One requested final tolerance in a batch.
#[derive(Debug, Clone, PartialEq)]
pub struct ToleranceGrowth {
    target: ToleranceGrowthTarget,
    requested: f64,
}

impl ToleranceGrowth {
    /// Construct one target-tolerance request.
    pub const fn new(target: ToleranceGrowthTarget, requested: f64) -> Self {
        Self { target, requested }
    }

    /// Entity whose tolerance may grow.
    pub const fn target(&self) -> &ToleranceGrowthTarget {
        &self.target
    }

    /// Requested final tolerance in model units.
    pub const fn requested(&self) -> f64 {
        self.requested
    }
}

/// Failure-atomic operation-owned tolerance-growth batch.
///
/// The operation name is retained as tolerance provenance. `max_total_growth`
/// limits aggregate enlargement above each target's existing tolerance or the
/// model resolution floor. Targets must be unique within the batch.
#[derive(Debug, Clone, PartialEq)]
pub struct GrowTolerancesRequest {
    operation: &'static str,
    max_total_growth: f64,
    growth: Vec<ToleranceGrowth>,
}

impl GrowTolerancesRequest {
    /// Construct one ordered tolerance-growth batch.
    pub fn new(
        operation: &'static str,
        max_total_growth: f64,
        growth: Vec<ToleranceGrowth>,
    ) -> Self {
        Self {
            operation,
            max_total_growth,
            growth,
        }
    }

    /// Stable operation name retained in tolerance provenance.
    pub const fn operation(&self) -> &'static str {
        self.operation
    }

    /// Maximum aggregate enlargement in model units.
    pub const fn max_total_growth(&self) -> f64 {
        self.max_total_growth
    }

    /// Ordered target requests.
    pub fn growth(&self) -> &[ToleranceGrowth] {
        &self.growth
    }
}

/// Journal-local identity of one successfully applied tolerance budget.
///
/// This identity is not an authoring capability and no edit method accepts it.
/// After commit, resolve it only against the resulting [`ChangeJournal`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GrowTolerancesResult {
    budget: ToleranceBudgetId,
}

/// Acceptance rule for an opt-in Full-assurance edit commit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FullCommitRequirement {
    /// Persist only when every selected body is proof-complete.
    RequireValid,
    /// Persist Fast-clean bodies while retaining explicit Full proof gaps.
    /// Proven Full faults still reject the complete transaction.
    AllowIndeterminate,
}

/// Evidence-bearing result of a Full-assurance edit commit attempt.
///
/// A rejected result has no journal because the transaction has already
/// restored its entry state. Operation execution and resource failures remain
/// errors in the enclosing [`OperationOutcome`].
#[derive(Debug)]
pub struct FullCommitResult {
    journal: Option<ChangeJournal>,
    reports: Vec<BodyCheckReport>,
}

impl FullCommitResult {
    /// Whether the candidate was persisted.
    pub fn is_committed(&self) -> bool {
        self.journal.is_some()
    }

    /// Committed journal, absent after proof-policy rejection.
    pub const fn journal(&self) -> Option<&ChangeJournal> {
        self.journal.as_ref()
    }

    /// Full reports in explicit-root, affected-root, then store order.
    pub fn reports(&self) -> &[BodyCheckReport] {
        &self.reports
    }

    /// Consume the result and return its journal when committed.
    pub fn into_journal(self) -> Option<ChangeJournal> {
        self.journal
    }

    /// Consume the result into its optional journal and owned body reports.
    pub fn into_parts(self) -> (Option<ChangeJournal>, Vec<BodyCheckReport>) {
        (self.journal, self.reports)
    }
}

impl GrowTolerancesResult {
    /// Budget identity to resolve in the committed journal.
    pub const fn budget(self) -> ToleranceBudgetId {
        self.budget
    }
}

/// Create the transient seed topology produced by MVFS at one position.
///
/// The result is an intermediate modeling state: checked commit must reject it
/// unless later Euler operations complete it into valid topology or KVFS
/// removes it in the same transaction.
#[derive(Debug, Clone, PartialEq)]
pub struct CreateSeedBodyRequest {
    surface: SurfaceId,
    sense: Sense,
    position: Point3,
}

impl CreateSeedBodyRequest {
    /// Construct a position-owning MVFS request.
    pub const fn new(surface: SurfaceId, sense: Sense, position: Point3) -> Self {
        Self {
            surface,
            sense,
            position,
        }
    }

    /// Supporting surface of the seed face.
    pub fn surface(&self) -> SurfaceId {
        self.surface.clone()
    }

    /// Seed face orientation relative to its surface.
    pub const fn sense(&self) -> Sense {
        self.sense
    }

    /// Model-space seed-vertex position.
    pub const fn position(&self) -> Point3 {
        self.position
    }
}

/// Opaque identities created by one transient MVFS operation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CreateSeedBodyResult {
    body: BodyId,
    void_region: RegionId,
    solid_region: RegionId,
    shell: ShellId,
    face: FaceId,
    loop_id: LoopId,
    vertex: VertexId,
}

impl CreateSeedBodyResult {
    /// New transient body.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Infinite exterior void region.
    pub fn void_region(&self) -> RegionId {
        self.void_region.clone()
    }

    /// Solid region owning the seed shell.
    pub fn solid_region(&self) -> RegionId {
        self.solid_region.clone()
    }

    /// Shell holding the seed vertex in its acorn slot.
    pub fn shell(&self) -> ShellId {
        self.shell.clone()
    }

    /// Single seed face.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// Face's single empty loop.
    pub fn loop_id(&self) -> LoopId {
        self.loop_id.clone()
    }

    /// Seed vertex at the requested position.
    pub fn vertex(&self) -> VertexId {
        self.vertex.clone()
    }
}

/// Remove a body that is still in the exact transient MVFS seed shape.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoveSeedBodyRequest {
    body: BodyId,
}

impl RemoveSeedBodyRequest {
    /// Select the transient seed body to remove.
    pub const fn new(body: BodyId) -> Self {
        Self { body }
    }

    /// Seed body selected for removal.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }
}

/// Identity removed by one successful KVFS operation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoveSeedBodyResult {
    body: BodyId,
}

impl RemoveSeedBodyResult {
    /// Removed body identity, stale in the transaction candidate afterward.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }
}

/// Sprout one pcurve-bearing strut edge and a new vertex into a loop.
///
/// The fin index selects the existing fin whose tail is the sprout vertex.
/// The new edge runs from that vertex to `position`; its forward and reversed
/// pcurve uses are inserted consecutively into the same loop.
#[derive(Debug, Clone, PartialEq)]
pub struct CreateStrutRequest {
    loop_id: LoopId,
    fin_index: usize,
    curve: BoundedCurve,
    position: Point3,
    pcurves: [BoundedPcurve; 2],
}

impl CreateStrutRequest {
    /// Construct one position-owning, pcurve-aware strut request.
    pub const fn new(
        loop_id: LoopId,
        fin_index: usize,
        curve: BoundedCurve,
        position: Point3,
        pcurves: [BoundedPcurve; 2],
    ) -> Self {
        Self {
            loop_id,
            fin_index,
            curve,
            position,
            pcurves,
        }
    }

    /// Loop that receives the two new fins.
    pub fn loop_id(&self) -> LoopId {
        self.loop_id.clone()
    }

    /// Stored fin position whose tail starts the strut.
    pub const fn fin_index(&self) -> usize {
        self.fin_index
    }

    /// Existing 3D edge geometry and active interval.
    pub const fn curve(&self) -> &BoundedCurve {
        &self.curve
    }

    /// Model-space position of the new vertex.
    pub const fn position(&self) -> Point3 {
        self.position
    }

    /// Forward/reversed pcurve uses for the new fins.
    pub const fn pcurves(&self) -> &[BoundedPcurve; 2] {
        &self.pcurves
    }
}

/// Opaque topology identities created by one strut operation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CreateStrutResult {
    edge: EdgeId,
    vertex: VertexId,
    fins: [FinId; 2],
}

impl CreateStrutResult {
    /// New strut edge.
    pub fn edge(&self) -> EdgeId {
        self.edge.clone()
    }

    /// New vertex at the requested position.
    pub fn vertex(&self) -> VertexId {
        self.vertex.clone()
    }

    /// New fins in forward/reversed sense order.
    pub fn fins(&self) -> [FinId; 2] {
        self.fins.clone()
    }
}

/// Remove one live MEV-shaped strut edge and its otherwise-unused vertex.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoveStrutRequest {
    edge: EdgeId,
}

impl RemoveStrutRequest {
    /// Select the strut edge to remove.
    pub const fn new(edge: EdgeId) -> Self {
        Self { edge }
    }

    /// Strut edge to remove.
    pub fn edge(&self) -> EdgeId {
        self.edge.clone()
    }
}

/// Surviving loop after removing one strut.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoveStrutResult {
    loop_id: LoopId,
}

impl RemoveStrutResult {
    /// Loop from which the strut fins were removed.
    pub fn loop_id(&self) -> LoopId {
        self.loop_id.clone()
    }
}

/// Split one loop between two stored fin positions using existing geometry.
///
/// The new face inherits the source face's surface, orientation, domain, and
/// tolerance. The two pcurve uses are ordered by the sense of the new edge's
/// fins: forward first, reversed second.
#[derive(Debug, Clone, PartialEq)]
pub struct SplitFaceRequest {
    loop_id: LoopId,
    fin_indices: [usize; 2],
    curve: BoundedCurve,
    pcurves: [BoundedPcurve; 2],
}

impl SplitFaceRequest {
    /// Construct one affine-map-aware pcurve face split request.
    pub const fn new(
        loop_id: LoopId,
        fin_indices: [usize; 2],
        curve: BoundedCurve,
        pcurves: [BoundedPcurve; 2],
    ) -> Self {
        Self {
            loop_id,
            fin_indices,
            curve,
            pcurves,
        }
    }

    /// Loop that will be split.
    pub fn loop_id(&self) -> LoopId {
        self.loop_id.clone()
    }

    /// Stored loop-fin positions joined by the new edge.
    pub const fn fin_indices(&self) -> [usize; 2] {
        self.fin_indices
    }

    /// Existing 3D edge geometry and active interval.
    pub const fn curve(&self) -> &BoundedCurve {
        &self.curve
    }

    /// Forward/reversed pcurve uses for the new fins.
    pub const fn pcurves(&self) -> &[BoundedPcurve; 2] {
        &self.pcurves
    }
}

/// Opaque identities created by one in-transaction face split.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SplitFaceResult {
    edge: EdgeId,
    face: FaceId,
    loop_id: LoopId,
    fins: [FinId; 2],
}

impl SplitFaceResult {
    /// New separating edge.
    pub fn edge(&self) -> EdgeId {
        self.edge.clone()
    }

    /// New face.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// New face's outer loop.
    pub fn loop_id(&self) -> LoopId {
        self.loop_id.clone()
    }

    /// New edge fins in old-face/new-face order.
    pub fn fins(&self) -> [FinId; 2] {
        self.fins.clone()
    }
}

/// Merge the two faces separated by one live two-fin edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MergeFacesRequest {
    edge: EdgeId,
}

impl MergeFacesRequest {
    /// Construct a semantic face-merge request.
    pub const fn new(edge: EdgeId) -> Self {
        Self { edge }
    }

    /// Edge that separates the faces to merge.
    pub fn edge(&self) -> EdgeId {
        self.edge.clone()
    }
}

/// Remove a live bridge edge whose two fins occur in one loop.
///
/// The surviving loop keeps its identity and the fins between the bridge uses
/// become a new ring on the same face. Both resulting loops must be nonempty.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoveBridgeRequest {
    edge: EdgeId,
}

impl RemoveBridgeRequest {
    /// Select the bridge edge to remove.
    pub const fn new(edge: EdgeId) -> Self {
        Self { edge }
    }

    /// Bridge edge to remove.
    pub fn edge(&self) -> EdgeId {
        self.edge.clone()
    }
}

/// Loop identities produced by removing one bridge edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoveBridgeResult {
    outer: LoopId,
    ring: LoopId,
}

impl RemoveBridgeResult {
    /// Surviving source-loop identity.
    pub fn outer(&self) -> LoopId {
        self.outer.clone()
    }

    /// Newly created inner-ring identity.
    pub fn ring(&self) -> LoopId {
        self.ring.clone()
    }
}

/// Join an outer loop to a ring of the same face with a new bridge edge.
///
/// Fin indices select the tail vertices joined by the new edge. The pcurve
/// uses are ordered by edge sense: forward from outer to ring, then reversed
/// returning from ring to outer.
#[derive(Debug, Clone, PartialEq)]
pub struct JoinRingRequest {
    outer: LoopId,
    outer_fin_index: usize,
    ring: LoopId,
    ring_fin_index: usize,
    curve: BoundedCurve,
    pcurves: [BoundedPcurve; 2],
}

impl JoinRingRequest {
    /// Construct one pcurve-aware ring join request.
    pub const fn new(
        outer: LoopId,
        outer_fin_index: usize,
        ring: LoopId,
        ring_fin_index: usize,
        curve: BoundedCurve,
        pcurves: [BoundedPcurve; 2],
    ) -> Self {
        Self {
            outer,
            outer_fin_index,
            ring,
            ring_fin_index,
            curve,
            pcurves,
        }
    }

    /// Surviving outer loop.
    pub fn outer(&self) -> LoopId {
        self.outer.clone()
    }

    /// Stored outer-loop fin position whose tail starts the bridge.
    pub const fn outer_fin_index(&self) -> usize {
        self.outer_fin_index
    }

    /// Ring dissolved into the outer loop.
    pub fn ring(&self) -> LoopId {
        self.ring.clone()
    }

    /// Stored ring-fin position whose tail ends the bridge.
    pub const fn ring_fin_index(&self) -> usize {
        self.ring_fin_index
    }

    /// Existing 3D bridge geometry and active interval.
    pub const fn curve(&self) -> &BoundedCurve {
        &self.curve
    }

    /// Forward/reversed pcurve uses for the bridge fins.
    pub const fn pcurves(&self) -> &[BoundedPcurve; 2] {
        &self.pcurves
    }
}

/// Opaque identities created by joining one ring into an outer loop.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JoinRingResult {
    loop_id: LoopId,
    edge: EdgeId,
    fins: [FinId; 2],
}

impl JoinRingResult {
    /// Surviving merged-loop identity.
    pub fn loop_id(&self) -> LoopId {
        self.loop_id.clone()
    }

    /// New bridge edge.
    pub fn edge(&self) -> EdgeId {
        self.edge.clone()
    }

    /// Bridge fins in forward/reversed sense order.
    pub fn fins(&self) -> [FinId; 2] {
        self.fins.clone()
    }
}

/// Absorb one single-loop face into another face of the same shell as a ring.
///
/// Topology proves ownership, incidence, and Euler structure, but it does not
/// pre-certify that the moved loop is geometrically contained as a hole in the
/// surviving face. Checked commit is the final persistence authority and
/// applies the supported Fast checks failure-atomically; callers still need
/// operation-specific containment evidence where Fast cannot prove it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MergeFaceAsHoleRequest {
    keep: FaceId,
    remove: FaceId,
}

impl MergeFaceAsHoleRequest {
    /// Select the surviving face and the single-loop face to absorb.
    pub const fn new(keep: FaceId, remove: FaceId) -> Self {
        Self { keep, remove }
    }

    /// Face that survives and receives the ring.
    pub fn keep(&self) -> FaceId {
        self.keep.clone()
    }

    /// Single-loop face removed by the operation.
    pub fn remove(&self) -> FaceId {
        self.remove.clone()
    }
}

/// Identities retained after absorbing one face as a ring hole.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MergeFaceAsHoleResult {
    face: FaceId,
    ring: LoopId,
}

impl MergeFaceAsHoleResult {
    /// Surviving face identity.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// Moved loop, now owned by the surviving face.
    pub fn ring(&self) -> LoopId {
        self.ring.clone()
    }
}

/// Detach one loop of a multi-loop face into a new face.
///
/// The caller supplies the new supporting surface and orientation. Topology
/// does not pre-certify that the selected loop is geometrically an inner hole;
/// checked commit applies the supported Fast checks before persistence, while
/// callers retain any operation-specific containment proof obligation that
/// Fast cannot discharge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SplitHoleAsFaceRequest {
    ring: LoopId,
    surface: SurfaceId,
    sense: Sense,
}

impl SplitHoleAsFaceRequest {
    /// Select a ring plus the new face's supporting carrier and orientation.
    pub const fn new(ring: LoopId, surface: SurfaceId, sense: Sense) -> Self {
        Self {
            ring,
            surface,
            sense,
        }
    }

    /// Loop moved to the new face.
    pub fn ring(&self) -> LoopId {
        self.ring.clone()
    }

    /// Supporting surface of the new face.
    pub fn surface(&self) -> SurfaceId {
        self.surface.clone()
    }

    /// New face orientation relative to its surface.
    pub const fn sense(&self) -> Sense {
        self.sense
    }
}

/// Identities produced by detaching one ring into a new face.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SplitHoleAsFaceResult {
    source_face: FaceId,
    face: FaceId,
    loop_id: LoopId,
}

impl SplitHoleAsFaceResult {
    /// Surviving source face.
    pub fn source_face(&self) -> FaceId {
        self.source_face.clone()
    }

    /// Newly created face.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// Same loop identity moved from source to new face.
    pub fn loop_id(&self) -> LoopId {
        self.loop_id.clone()
    }
}

/// Failure-atomic composition of checked semantic edits on one part.
///
/// Dropping this value rolls every uncommitted mutation back. Only the
/// semantic methods below are exposed; lower storage, assembly, raw Euler
/// functions, and unchecked commit remain unreachable.
pub struct EditTransaction<'part> {
    inner: Transaction<'part>,
    context: OperationContext<'part>,
    part: PartId,
}

impl PartEdit<'_> {
    /// Start one checked semantic edit transaction.
    ///
    /// Settings are validated before the lower transaction begins. Nested
    /// transactions on the same part retain the lower typed rejection.
    pub fn begin_edit(&mut self, settings: OperationSettings) -> Result<EditTransaction<'_>> {
        let context = settings.context(self.policy)?;
        let part = self.id.clone();
        let inner = self.state.store.transaction()?;
        Ok(EditTransaction {
            inner,
            context,
            part,
        })
    }
}

impl EditTransaction<'_> {
    /// Part whose candidate state is exclusively borrowed.
    pub fn part(&self) -> PartId {
        self.part.clone()
    }

    /// Apply one failure-atomic operation-owned tolerance-growth batch.
    ///
    /// Part qualification, liveness, model-valid final tolerances, uniqueness,
    /// provenance construction, and aggregate growth are fully preflighted
    /// before the lower transaction declares its internal budget or mutates
    /// any entity. Wrong-part and stale-identity rejection precede numeric or
    /// duplicate validation, and event order matches request order.
    pub fn grow_tolerances(
        &mut self,
        request: GrowTolerancesRequest,
    ) -> Result<GrowTolerancesResult> {
        let mut qualified = Vec::with_capacity(request.growth.len());
        for growth in &request.growth {
            let target = match &growth.target {
                ToleranceGrowthTarget::Face(face) => {
                    self.validate_part(face.part())?;
                    self.inner
                        .store()
                        .get(face.raw())
                        .map_err(|_| Error::StaleEntity {
                            kind: EntityKind::Face,
                        })?;
                    RawToleranceGrowthTarget::Face(face.raw())
                }
                ToleranceGrowthTarget::Edge(edge) => {
                    self.validate_part(edge.part())?;
                    self.inner
                        .store()
                        .get(edge.raw())
                        .map_err(|_| Error::StaleEntity {
                            kind: EntityKind::Edge,
                        })?;
                    RawToleranceGrowthTarget::Edge(edge.raw())
                }
                ToleranceGrowthTarget::Vertex(vertex) => {
                    self.validate_part(vertex.part())?;
                    self.inner
                        .store()
                        .get(vertex.raw())
                        .map_err(|_| Error::StaleEntity {
                            kind: EntityKind::Vertex,
                        })?;
                    RawToleranceGrowthTarget::Vertex(vertex.raw())
                }
            };
            qualified.push(target);
        }
        if request.operation.is_empty() {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "tolerance operation name must not be empty",
            }
            .into());
        }
        if !request.max_total_growth.is_finite() || request.max_total_growth < 0.0 {
            return Err(kcore::error::Error::InvalidToleranceBudget {
                limit: request.max_total_growth,
            }
            .into());
        }

        let mut seen = Vec::with_capacity(request.growth.len());
        let mut raw = Vec::with_capacity(request.growth.len());
        for (growth, target) in request.growth.iter().zip(qualified) {
            if seen.contains(&growth.target) {
                return Err(kcore::error::Error::InvalidGeometry {
                    reason: "tolerance growth batch contains a duplicate target",
                }
                .into());
            }
            seen.push(growth.target.clone());
            kcore::tolerance::Tolerances::default().entity_tolerance(growth.requested)?;
            raw.push(RawToleranceGrowth::new(target, growth.requested));
        }

        let budget =
            self.inner
                .grow_tolerances(request.operation, request.max_total_growth, &raw)?;
        Ok(GrowTolerancesResult {
            budget: ToleranceBudgetId::from_index(budget.index()),
        })
    }

    /// Create one transient position-owning MVFS seed body.
    ///
    /// The returned topology is intentionally incomplete and cannot pass
    /// checked commit by itself. Compose further Euler edits or call
    /// [`Self::remove_seed_body`] before committing.
    pub fn create_seed_body(
        &mut self,
        request: CreateSeedBodyRequest,
    ) -> Result<CreateSeedBodyResult> {
        self.require_surface(&request.surface)?;
        let CreateSeedBodyRequest {
            surface,
            sense,
            position,
        } = request;
        let made = self
            .inner
            .make_minimal_body_at_position(surface.raw(), sense, position)?;
        Ok(CreateSeedBodyResult {
            body: BodyId::new(self.part.clone(), made.body),
            void_region: RegionId::new(self.part.clone(), made.void_region),
            solid_region: RegionId::new(self.part.clone(), made.solid_region),
            shell: ShellId::new(self.part.clone(), made.shell),
            face: FaceId::new(self.part.clone(), made.face),
            loop_id: LoopId::new(self.part.clone(), made.ring),
            vertex: VertexId::new(self.part.clone(), made.vertex),
        })
    }

    /// Remove exact transient MVFS topology and its unshared hidden point.
    pub fn remove_seed_body(
        &mut self,
        request: RemoveSeedBodyRequest,
    ) -> Result<RemoveSeedBodyResult> {
        self.validate_part(request.body.part())?;
        self.inner
            .store()
            .get(request.body.raw())
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Body,
            })?;
        self.inner
            .kill_position_owned_minimal_body(request.body.raw())?;
        Ok(RemoveSeedBodyResult { body: request.body })
    }

    /// Create one position-owning strut with mandatory independent pcurves.
    ///
    /// All position, identity, incidence, and MEV topology preconditions are
    /// checked before the new point enters the transaction candidate.
    pub fn create_strut(&mut self, request: CreateStrutRequest) -> Result<CreateStrutResult> {
        self.validate_part(request.loop_id.part())?;
        let loop_ =
            self.inner
                .store()
                .get(request.loop_id.raw())
                .map_err(|_| Error::StaleEntity {
                    kind: EntityKind::Loop,
                })?;
        if loop_.fins().is_empty() {
            if request.fin_index != 0 {
                return Err(kcore::error::Error::InvalidGeometry {
                    reason: "strut fin index must be zero on an empty loop",
                }
                .into());
            }
        } else {
            let Some(&fin) = loop_.fins().get(request.fin_index) else {
                return Err(kcore::error::Error::InvalidGeometry {
                    reason: "strut fin index is out of range",
                }
                .into());
            };
            let vertex = self.inner.store().fin_tail(fin)?.ok_or_else(|| {
                Error::from(kcore::error::Error::InvalidGeometry {
                    reason: "strut cannot sprout from a ring-edge fin",
                })
            })?;
            self.inner.store().vertex_position(vertex)?;
        }
        self.require_curve(&request.curve.curve)?;
        for pcurve in &request.pcurves {
            self.require_pcurve(&pcurve.pcurve)?;
        }

        let CreateStrutRequest {
            loop_id,
            fin_index,
            curve,
            position,
            pcurves: [forward, reversed],
        } = request;
        let pcurves = FinPcurvePair::new(forward.into_raw_use()?, reversed.into_raw_use()?);
        let made = self.inner.make_edge_vertex_at_position(
            loop_id.raw(),
            fin_index,
            curve.curve.raw(),
            (curve.range.lo, curve.range.hi),
            position,
            pcurves,
        )?;
        Ok(CreateStrutResult {
            edge: EdgeId::new(self.part.clone(), made.edge),
            vertex: VertexId::new(self.part.clone(), made.vertex),
            fins: [
                FinId::new(self.part.clone(), made.fin_out),
                FinId::new(self.part.clone(), made.fin_back),
            ],
        })
    }

    /// Remove one MEV-shaped strut and its otherwise-unused vertex.
    pub fn remove_strut(&mut self, request: RemoveStrutRequest) -> Result<RemoveStrutResult> {
        self.validate_part(request.edge.part())?;
        let edge = self
            .inner
            .store()
            .get(request.edge.raw())
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Edge,
            })?;
        let [first, _] = edge.fins() else {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "strut removal requires an edge with exactly two fins",
            }
            .into());
        };
        let loop_id = self.inner.store().get(*first)?.parent();
        self.inner
            .kill_position_owned_edge_vertex(request.edge.raw())?;
        Ok(RemoveStrutResult {
            loop_id: LoopId::new(self.part.clone(), loop_id),
        })
    }

    /// Split one face using the transaction's pcurve-aware checked operator.
    ///
    /// The new face inherits the source tolerance and its complete provenance.
    /// Commit journals describe that propagation without exposing a reusable
    /// tolerance budget.
    pub fn split_face(&mut self, request: SplitFaceRequest) -> Result<SplitFaceResult> {
        self.validate_part(request.loop_id.part())?;
        self.inner
            .store()
            .get(request.loop_id.raw())
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Loop,
            })?;
        self.require_curve(&request.curve.curve)?;
        for pcurve in &request.pcurves {
            self.require_pcurve(&pcurve.pcurve)?;
        }

        let source_face = self.inner.store().get(request.loop_id.raw())?.face();
        let source = self.inner.store().get(source_face)?;
        let surface = source.surface();
        let sense = source.sense();
        let [forward, reversed] = request.pcurves;
        let pcurves = FinPcurvePair::new(forward.into_raw_use()?, reversed.into_raw_use()?);
        let made = self.inner.split_face(
            request.loop_id.raw(),
            request.fin_indices[0],
            request.fin_indices[1],
            request.curve.curve.raw(),
            (request.curve.range.lo, request.curve.range.hi),
            surface,
            sense,
            pcurves,
        )?;
        Ok(SplitFaceResult {
            edge: EdgeId::new(self.part.clone(), made.edge),
            face: FaceId::new(self.part.clone(), made.face),
            loop_id: LoopId::new(self.part.clone(), made.ring),
            fins: [
                FinId::new(self.part.clone(), made.fin_old),
                FinId::new(self.part.clone(), made.fin_new),
            ],
        })
    }

    /// Merge the two faces separated by one live edge.
    ///
    /// The surviving face selects the larger input tolerance, retaining the
    /// selected origin/growth provenance. Equal values select the survivor;
    /// commit journals describe the ordered inputs and selection.
    pub fn merge_faces(&mut self, request: MergeFacesRequest) -> Result<()> {
        self.validate_part(request.edge.part())?;
        self.inner
            .store()
            .get(request.edge.raw())
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Edge,
            })?;
        self.inner
            .merge_faces(request.edge.raw())
            .map_err(Error::from)
    }

    /// Remove one bridge edge and split its loop into outer and ring loops.
    pub fn remove_bridge(&mut self, request: RemoveBridgeRequest) -> Result<RemoveBridgeResult> {
        self.validate_part(request.edge.part())?;
        let edge = self
            .inner
            .store()
            .get(request.edge.raw())
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Edge,
            })?;
        let [first, _] = edge.fins() else {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "bridge removal requires an edge with exactly two fins",
            }
            .into());
        };
        let outer = self.inner.store().get(*first)?.parent();
        let ring = self
            .inner
            .kill_edge_make_ring(request.edge.raw())
            .map_err(Error::from)?;
        Ok(RemoveBridgeResult {
            outer: LoopId::new(self.part.clone(), outer),
            ring: LoopId::new(self.part.clone(), ring),
        })
    }

    /// Join an outer loop to a ring with a pcurve-bearing bridge edge.
    pub fn join_ring(&mut self, request: JoinRingRequest) -> Result<JoinRingResult> {
        self.validate_part(request.outer.part())?;
        self.validate_part(request.ring.part())?;
        let outer =
            self.inner
                .store()
                .get(request.outer.raw())
                .map_err(|_| Error::StaleEntity {
                    kind: EntityKind::Loop,
                })?;
        let ring = self
            .inner
            .store()
            .get(request.ring.raw())
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Loop,
            })?;
        if request.outer == request.ring {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "ring join requires two distinct loops",
            }
            .into());
        }
        if outer.face() != ring.face() {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "ring join requires loops owned by one face",
            }
            .into());
        }
        let selected = [
            outer.fins().get(request.outer_fin_index),
            ring.fins().get(request.ring_fin_index),
        ];
        for fin in selected {
            let Some(&fin) = fin else {
                return Err(kcore::error::Error::InvalidGeometry {
                    reason: "ring join fin index is out of range",
                }
                .into());
            };
            let vertex = self.inner.store().fin_tail(fin)?.ok_or_else(|| {
                Error::from(kcore::error::Error::InvalidGeometry {
                    reason: "ring join cannot select a ring-edge fin",
                })
            })?;
            self.inner.store().vertex_position(vertex)?;
        }
        self.require_curve(&request.curve.curve)?;
        for pcurve in &request.pcurves {
            self.require_pcurve(&pcurve.pcurve)?;
        }

        let JoinRingRequest {
            outer,
            outer_fin_index,
            ring,
            ring_fin_index,
            curve,
            pcurves: [forward, reversed],
        } = request;
        let pcurves = FinPcurvePair::new(forward.into_raw_use()?, reversed.into_raw_use()?);
        let made = self.inner.make_edge_kill_ring(
            outer.raw(),
            outer_fin_index,
            ring.raw(),
            ring_fin_index,
            curve.curve.raw(),
            (curve.range.lo, curve.range.hi),
            pcurves,
        )?;
        Ok(JoinRingResult {
            loop_id: outer,
            edge: EdgeId::new(self.part.clone(), made.edge),
            fins: [
                FinId::new(self.part.clone(), made.fin_out),
                FinId::new(self.part.clone(), made.fin_back),
            ],
        })
    }

    /// Absorb a single-loop face as a ring of another face in the same shell.
    ///
    /// This preflights structural ownership and moved pcurve incidence. It
    /// does not claim geometric hole containment. Checked commit is the final
    /// persistence authority for supported Fast checks and restores failures
    /// atomically; unsupported containment still needs caller evidence.
    pub fn merge_face_as_hole(
        &mut self,
        request: MergeFaceAsHoleRequest,
    ) -> Result<MergeFaceAsHoleResult> {
        self.validate_part(request.keep.part())?;
        self.validate_part(request.remove.part())?;
        let keep = self
            .inner
            .store()
            .get(request.keep.raw())
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Face,
            })?;
        let remove =
            self.inner
                .store()
                .get(request.remove.raw())
                .map_err(|_| Error::StaleEntity {
                    kind: EntityKind::Face,
                })?;
        if request.keep == request.remove {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "face-as-hole merge requires two distinct faces",
            }
            .into());
        }
        if keep.shell() != remove.shell() {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "face-as-hole merge requires one owning shell",
            }
            .into());
        }
        self.inner
            .store()
            .get(keep.shell())
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Shell,
            })?;
        for surface in [keep.surface(), remove.surface()] {
            self.inner
                .store()
                .geometry()
                .surface(surface)
                .ok_or(Error::StaleEntity {
                    kind: EntityKind::Surface,
                })?;
        }
        let [ring] = remove.loops() else {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "face-as-hole merge requires the removed face to own exactly one loop",
            }
            .into());
        };
        self.inner
            .store()
            .get(*ring)
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Loop,
            })?;

        let ring = self
            .inner
            .merge_face_as_hole(request.keep.raw(), request.remove.raw())?;
        Ok(MergeFaceAsHoleResult {
            face: request.keep,
            ring: LoopId::new(self.part.clone(), ring),
        })
    }

    /// Detach one loop of a multi-loop face into a new face.
    ///
    /// The selected loop's geometric hole role is not pre-certified. Carrier
    /// incidence is preflighted; checked commit gates persistence with
    /// supported Fast checks, while unproved containment remains a caller
    /// obligation.
    pub fn split_hole_as_face(
        &mut self,
        request: SplitHoleAsFaceRequest,
    ) -> Result<SplitHoleAsFaceResult> {
        self.validate_part(request.ring.part())?;
        self.validate_part(request.surface.part())?;
        let ring = self
            .inner
            .store()
            .get(request.ring.raw())
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Loop,
            })?;
        self.require_surface(&request.surface)?;
        let source_face = ring.face();
        let source = self
            .inner
            .store()
            .get(source_face)
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Face,
            })?;
        self.inner
            .store()
            .get(source.shell())
            .map_err(|_| Error::StaleEntity {
                kind: EntityKind::Shell,
            })?;
        if source.loops().len() < 2 {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "hole split requires another loop to remain on the source face",
            }
            .into());
        }

        let face = self.inner.split_hole_as_face(
            request.ring.raw(),
            request.surface.raw(),
            request.sense,
        )?;
        Ok(SplitHoleAsFaceResult {
            source_face: FaceId::new(self.part.clone(), source_face),
            face: FaceId::new(self.part.clone(), face),
            loop_id: request.ring,
        })
    }

    /// Fast-check every affected body and commit one journal atomically.
    ///
    /// `roots` supplies preferred result-body validation order. Wrong-part or
    /// stale roots are rejected before scope creation; consuming this method
    /// then drops and rolls back the lower transaction.
    pub fn commit(self, roots: &[BodyId]) -> Result<OperationOutcome<ChangeJournal>> {
        for root in roots {
            self.validate_part(root.part())?;
            self.inner
                .store()
                .get(root.raw())
                .map_err(|_| Error::StaleEntity {
                    kind: EntityKind::Body,
                })?;
        }
        let raw_roots = roots.iter().map(BodyId::raw).collect::<Vec<_>>();
        let part = self.part.clone();
        let outcome = self
            .inner
            .commit_checked_with_context(&raw_roots, &self.context)?;
        Ok(outcome
            .map(|journal| ChangeJournal::from_raw(part, journal))
            .map_err(Error::from))
    }

    /// Full-check every selected body and decide whether to commit atomically.
    ///
    /// Wrong-part and stale explicit roots are rejected before the operation
    /// scope starts. Fast faults and execution failures remain errors. Full
    /// proof faults produce a rollback-clean rejected result, while proof gaps
    /// are accepted only under [`FullCommitRequirement::AllowIndeterminate`].
    /// Every returned checker subject is adapted while retaining candidate
    /// point values even when rejection has already rolled the store back.
    pub fn commit_full(
        self,
        roots: &[BodyId],
        requirement: FullCommitRequirement,
    ) -> Result<OperationOutcome<FullCommitResult>> {
        for root in roots {
            self.validate_part(root.part())?;
            self.inner
                .store()
                .get(root.raw())
                .map_err(|_| Error::StaleEntity {
                    kind: EntityKind::Body,
                })?;
        }
        let raw_roots = roots.iter().map(BodyId::raw).collect::<Vec<_>>();
        let raw_requirement = match requirement {
            FullCommitRequirement::RequireValid => RawFullCommitRequirement::RequireValid,
            FullCommitRequirement::AllowIndeterminate => {
                RawFullCommitRequirement::AllowIndeterminate
            }
        };
        let part = self.part.clone();
        let outcome =
            self.inner
                .commit_full_with_context(&raw_roots, raw_requirement, &self.context)?;
        Ok(outcome.map_err(Error::from).map(|decision| {
            let (journal, checks) = decision.into_parts();
            let reports = checks
                .iter()
                .map(|check| adapt_transaction_check(&part, check))
                .collect::<Vec<_>>();
            FullCommitResult {
                journal: journal.map(|journal| ChangeJournal::from_raw(part, journal)),
                reports,
            }
        }))
    }

    /// Explicitly restore the transaction's entry state.
    ///
    /// Dropping without commit is equivalent.
    pub fn rollback(self) -> Result<()> {
        self.inner.rollback().map_err(Error::from)
    }

    fn validate_part(&self, actual: &PartId) -> Result<()> {
        if actual != &self.part {
            return Err(Error::WrongPart {
                expected: self.part.clone(),
                actual: actual.clone(),
            });
        }
        Ok(())
    }

    fn require_curve(&self, curve: &CurveId) -> Result<()> {
        self.validate_part(curve.part())?;
        self.inner
            .store()
            .geometry()
            .curve(curve.raw())
            .map(|_| ())
            .ok_or(Error::StaleEntity {
                kind: EntityKind::Curve,
            })
    }

    fn require_pcurve(&self, pcurve: &PcurveId) -> Result<()> {
        self.validate_part(pcurve.part())?;
        self.inner
            .store()
            .geometry()
            .curve2d(pcurve.raw())
            .map(|_| ())
            .ok_or(Error::StaleEntity {
                kind: EntityKind::Pcurve,
            })
    }

    fn require_surface(&self, surface: &SurfaceId) -> Result<()> {
        self.validate_part(surface.part())?;
        self.inner
            .store()
            .geometry()
            .surface(surface.raw())
            .map(|_| ())
            .ok_or(Error::StaleEntity {
                kind: EntityKind::Surface,
            })
    }
}

#[cfg(test)]
mod tests {
    use kcore::operation::{AccountingMode, BudgetPlan, LimitSpec, ResourceKind};
    use kcore::tolerance::LINEAR_RESOLUTION;
    use kgeom::curve::{Circle, Line};
    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::surface::Sphere;
    use kgeom::vec::{Point2, Point3, Vec2, Vec3};
    use kgraph::eval_stage;
    use ktopo::entity::{
        Edge as RawEdge, FaceDomain as RawFaceDomain, Fin as RawFin, FinPcurve, Loop as RawLoop,
        ParamMap1d, PcurveEndpointKind as RawPcurveEndpointKind, Sense as RawSense,
        Vertex as RawVertex,
    };
    use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};

    use super::*;
    use crate::{
        BlockRequest, CheckOutcome, FaceTolerancePropagationView, JournalEntity, Kernel,
        LineageView, MutationKind, ToleranceOrigin,
    };

    fn block(edit: &mut PartEdit<'_>) -> BodyId {
        edit.create_block(BlockRequest::new(Frame::world(), [2.0, 2.0, 2.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body()
    }

    fn split_request(edit: &mut PartEdit<'_>, body: &BodyId) -> SplitFaceRequest {
        split_request_with_parameterization(edit, body, false)
    }

    fn reversed_split_request(edit: &mut PartEdit<'_>, body: &BodyId) -> SplitFaceRequest {
        split_request_with_parameterization(edit, body, true)
    }

    fn seed_body_request(edit: &mut PartEdit<'_>, body: &BodyId) -> CreateSeedBodyRequest {
        let part = edit.id();
        let store = edit.store_mut_for_test();
        let raw_face = store.faces_of_body(body.raw()).unwrap()[0];
        let face = store.get(raw_face).unwrap();
        let loop_id = face.loops()[0];
        let fin = store.get(loop_id).unwrap().fins()[0];
        let vertex = store.fin_tail(fin).unwrap().unwrap();
        CreateSeedBodyRequest::new(
            SurfaceId::new(part, face.surface()),
            face.sense(),
            store.vertex_position(vertex).unwrap(),
        )
    }

    fn tolerance_targets(edit: &mut PartEdit<'_>, body: &BodyId) -> (FaceId, EdgeId, VertexId) {
        let part = edit.id();
        let store = edit.store_mut_for_test();
        let face = store.faces_of_body(body.raw()).unwrap()[0];
        let edge = store.edges_of_body(body.raw()).unwrap()[0];
        let vertex = store.get(edge).unwrap().vertices()[0].unwrap();
        (
            FaceId::new(part.clone(), face),
            EdgeId::new(part.clone(), edge),
            VertexId::new(part, vertex),
        )
    }

    fn planar_strut_request(
        edit: &mut PartEdit<'_>,
    ) -> (BodyId, LoopId, EdgeId, CreateStrutRequest, PcurveId) {
        let part = edit.id();
        let store = edit.store_mut_for_test();
        let raw_body = ktopo::make::planar_sheet(
            store,
            &Frame::world(),
            &[
                Point2::new(-1.0, -1.0),
                Point2::new(1.0, -1.0),
                Point2::new(1.0, 1.0),
                Point2::new(-1.0, 1.0),
            ],
        )
        .unwrap();
        let face = store.faces_of_body(raw_body).unwrap()[0];
        let loop_id = store.get(face).unwrap().loops()[0];
        let first_fin = store.get(loop_id).unwrap().fins()[0];
        let ordinary_edge = store.get(first_fin).unwrap().edge();
        let sprout = store.fin_tail(first_fin).unwrap().unwrap();
        let start = store.vertex_position(sprout).unwrap();
        let position = Point3::new(0.0, 0.0, 0.0);
        let length = (position - start).norm();
        let curve = store
            .insert_curve(CurveGeom::Line(Line::new(start, position - start).unwrap()))
            .unwrap();
        let start_uv = Point2::new(start.x, start.y);
        let position_uv = Point2::new(position.x, position.y);
        let make_pcurve = |store: &mut ktopo::store::Store| {
            store
                .insert_pcurve(Curve2dGeom::Line(
                    Line2d::new(position_uv, start_uv - position_uv).unwrap(),
                ))
                .unwrap()
        };
        let forward = make_pcurve(store);
        let reversed = make_pcurve(store);
        let off_curve = store
            .insert_pcurve(Curve2dGeom::Line(
                Line2d::new(Point2::new(10.0, 10.0), Vec2::new(1.0, 1.0)).unwrap(),
            ))
            .unwrap();
        let range = ParamRange::new(0.0, length);
        let map = PcurveParameterMap::affine(-1.0, length).unwrap();
        let bounded_pcurve = |raw| {
            BoundedPcurve::new(PcurveId::new(part.clone(), raw), range)
                .with_parameter_map(map)
                .with_metadata(PcurveMetadata::regular())
        };
        let request = CreateStrutRequest::new(
            LoopId::new(part.clone(), loop_id),
            0,
            BoundedCurve::new(CurveId::new(part.clone(), curve), range),
            position,
            [bounded_pcurve(forward), bounded_pcurve(reversed)],
        );
        (
            BodyId::new(part.clone(), raw_body),
            LoopId::new(part.clone(), loop_id),
            EdgeId::new(part.clone(), ordinary_edge),
            request,
            PcurveId::new(part, off_curve),
        )
    }

    fn spherical_sector_split_request(edit: &mut PartEdit<'_>) -> (BodyId, SplitFaceRequest) {
        let part = edit.id();
        let store = edit.store_mut_for_test();
        let raw_body = ktopo::make::planar_sheet(
            store,
            &Frame::world(),
            &[
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(0.0, 1.0),
            ],
        )
        .unwrap();
        let face = store.faces_of_body(raw_body).unwrap()[0];
        let surface = store.get(face).unwrap().surface();
        let loop_id = store.get(face).unwrap().loops()[0];
        let fins = store.get(loop_id).unwrap().fins().to_vec();
        let edges = fins
            .iter()
            .map(|&fin| store.get(fin).unwrap().edge())
            .collect::<Vec<_>>();
        let vertices = fins
            .iter()
            .map(|&fin| store.fin_tail(fin).unwrap().unwrap())
            .collect::<Vec<_>>();
        let curve_ids = edges
            .iter()
            .map(|&edge| store.get(edge).unwrap().curve().unwrap())
            .collect::<Vec<_>>();
        let pcurve_ids = fins
            .iter()
            .map(|&fin| store.get(fin).unwrap().pcurve().unwrap().curve())
            .collect::<Vec<_>>();

        let quarter = core::f64::consts::FRAC_PI_4;
        let half = core::f64::consts::FRAC_PI_2;
        let diagonal = core::f64::consts::FRAC_1_SQRT_2;
        let positions = [
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(diagonal, diagonal, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let curve_geometries = [
            CurveGeom::Circle(Circle::new(Frame::world(), 1.0).unwrap()),
            CurveGeom::Circle(Circle::new(Frame::world(), 1.0).unwrap()),
            CurveGeom::Circle(
                Circle::new(
                    Frame::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Vec3::new(1.0, 0.0, 0.0),
                        Vec3::new(0.0, 1.0, 0.0),
                    )
                    .unwrap(),
                    1.0,
                )
                .unwrap(),
            ),
            CurveGeom::Circle(
                Circle::new(
                    Frame::new(
                        Point3::new(0.0, 0.0, 0.0),
                        Vec3::new(0.0, 1.0, 0.0),
                        Vec3::new(0.0, 0.0, 1.0),
                    )
                    .unwrap(),
                    1.0,
                )
                .unwrap(),
            ),
        ];
        let bounds = [(0.0, quarter), (quarter, half), (0.0, half), (0.0, half)];
        let pcurve_geometries = [
            Curve2dGeom::Line(Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
            Curve2dGeom::Line(Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
            Curve2dGeom::Line(Line2d::new(Point2::new(half, 0.0), Vec2::new(0.0, 1.0)).unwrap()),
            Curve2dGeom::Line(Line2d::new(Point2::new(0.0, half), Vec2::new(0.0, -1.0)).unwrap()),
        ];
        let endpoint_kinds = [
            [RawPcurveEndpointKind::Regular; 2],
            [RawPcurveEndpointKind::Regular; 2],
            [
                RawPcurveEndpointKind::Regular,
                RawPcurveEndpointKind::SurfaceSingularity,
            ],
            [
                RawPcurveEndpointKind::SurfaceSingularity,
                RawPcurveEndpointKind::Regular,
            ],
        ];

        let mut transform = store.transaction().unwrap();
        {
            let mut assembly = transform.assembly();
            assembly
                .replace_surface(
                    surface,
                    SurfaceGeom::Sphere(Sphere::new(Frame::world(), 1.0).unwrap()),
                )
                .unwrap();
            let raw_face = assembly.get_mut(face).unwrap();
            raw_face.domain = Some(RawFaceDomain::from_bounds(0.0, half, 0.0, half).unwrap());
            for index in 0..4 {
                let point = assembly.add(positions[index]);
                assembly.get_mut(vertices[index]).unwrap().point = point;
                assembly
                    .replace_curve(curve_ids[index], curve_geometries[index].clone())
                    .unwrap();
                assembly
                    .replace_pcurve(pcurve_ids[index], pcurve_geometries[index].clone())
                    .unwrap();
                assembly.get_mut(edges[index]).unwrap().bounds = Some(bounds[index]);
                assembly.get_mut(fins[index]).unwrap().pcurve = Some(
                    FinPcurve::new(
                        pcurve_ids[index],
                        ParamRange::new(bounds[index].0, bounds[index].1),
                        ParamMap1d::identity(),
                    )
                    .unwrap()
                    .with_endpoint_kinds(endpoint_kinds[index]),
                );
            }
        }
        transform.commit_checked_body(raw_body).unwrap();

        let split_frame = Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(diagonal, -diagonal, 0.0),
            Vec3::new(diagonal, diagonal, 0.0),
        )
        .unwrap();
        let curve = store
            .insert_curve(CurveGeom::Circle(Circle::new(split_frame, 1.0).unwrap()))
            .unwrap();
        let make_pcurve = |store: &mut ktopo::store::Store| {
            store
                .insert_pcurve(Curve2dGeom::Line(
                    Line2d::new(Point2::new(quarter, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
                ))
                .unwrap()
        };
        let forward = make_pcurve(store);
        let reversed = make_pcurve(store);
        let range = ParamRange::new(0.0, half);
        let metadata = PcurveMetadata::regular().with_endpoint_kinds([
            PcurveEndpointKind::Regular,
            PcurveEndpointKind::SurfaceSingularity,
        ]);
        let body = BodyId::new(part.clone(), raw_body);
        let request = SplitFaceRequest::new(
            LoopId::new(part.clone(), loop_id),
            [1, 3],
            BoundedCurve::new(CurveId::new(part.clone(), curve), range),
            [
                BoundedPcurve::new(PcurveId::new(part.clone(), forward), range)
                    .with_metadata(metadata),
                BoundedPcurve::new(PcurveId::new(part, reversed), range).with_metadata(metadata),
            ],
        );
        (body, request)
    }

    fn planar_annulus_join_request(edit: &mut PartEdit<'_>) -> (BodyId, JoinRingRequest) {
        let part = edit.id();
        let store = edit.store_mut_for_test();
        let raw_body = ktopo::make::planar_sheet(
            store,
            &Frame::world(),
            &[
                Point2::new(-2.0, -2.0),
                Point2::new(2.0, -2.0),
                Point2::new(2.0, 2.0),
                Point2::new(-2.0, 2.0),
            ],
        )
        .unwrap();
        let face = store.faces_of_body(raw_body).unwrap()[0];
        let outer = store.get(face).unwrap().loops()[0];
        let inner_points = [
            Point2::new(-1.0, -1.0),
            Point2::new(-1.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, -1.0),
        ];

        let mut add_ring = store.transaction().unwrap();
        let ring;
        {
            let mut assembly = add_ring.assembly();
            ring = assembly.add(RawLoop {
                face,
                fins: Vec::new(),
            });
            assembly.get_mut(face).unwrap().loops.push(ring);
            let vertices = inner_points.map(|point| {
                let point = assembly.add(Point3::new(point.x, point.y, 0.0));
                assembly.add(RawVertex {
                    point,
                    tolerance: None,
                })
            });
            for index in 0..inner_points.len() {
                let next = (index + 1) % inner_points.len();
                let start = inner_points[index];
                let end = inner_points[next];
                let delta = end - start;
                let length = delta.norm();
                let curve = assembly
                    .insert_curve(CurveGeom::Line(
                        Line::new(
                            Point3::new(start.x, start.y, 0.0),
                            Vec3::new(delta.x, delta.y, 0.0),
                        )
                        .unwrap(),
                    ))
                    .unwrap();
                let pcurve = assembly
                    .insert_pcurve(Curve2dGeom::Line(Line2d::new(start, delta).unwrap()))
                    .unwrap();
                let edge = assembly.add(RawEdge {
                    curve: Some(curve),
                    vertices: [Some(vertices[index]), Some(vertices[next])],
                    bounds: Some((0.0, length)),
                    fins: Vec::new(),
                    tolerance: None,
                });
                let fin = assembly.add(RawFin {
                    parent: ring,
                    edge,
                    sense: RawSense::Forward,
                    pcurve: Some(
                        FinPcurve::new(
                            pcurve,
                            ParamRange::new(0.0, length),
                            ParamMap1d::identity(),
                        )
                        .unwrap(),
                    ),
                });
                assembly.get_mut(edge).unwrap().fins.push(fin);
                assembly.get_mut(ring).unwrap().fins.push(fin);
            }
        }
        add_ring.commit_checked_body(raw_body).unwrap();

        let start = Point3::new(-2.0, -2.0, 0.0);
        let end = Point3::new(-1.0, -1.0, 0.0);
        let delta = end - start;
        let length = delta.norm();
        let curve = store
            .insert_curve(CurveGeom::Line(Line::new(start, delta).unwrap()))
            .unwrap();
        let make_reversed_pcurve = |store: &mut ktopo::store::Store| {
            store
                .insert_pcurve(Curve2dGeom::Line(
                    Line2d::new(
                        Point2::new(end.x, end.y),
                        Vec2::new(start.x - end.x, start.y - end.y),
                    )
                    .unwrap(),
                ))
                .unwrap()
        };
        let forward = make_reversed_pcurve(store);
        let reversed = make_reversed_pcurve(store);
        let range = ParamRange::new(0.0, length);
        let map = PcurveParameterMap::affine(-1.0, length).unwrap();
        let request = JoinRingRequest::new(
            LoopId::new(part.clone(), outer),
            0,
            LoopId::new(part.clone(), ring),
            0,
            BoundedCurve::new(CurveId::new(part.clone(), curve), range),
            [
                BoundedPcurve::new(PcurveId::new(part.clone(), forward), range)
                    .with_parameter_map(map),
                BoundedPcurve::new(PcurveId::new(part.clone(), reversed), range)
                    .with_parameter_map(map),
            ],
        );
        (BodyId::new(part, raw_body), request)
    }

    fn split_request_with_parameterization(
        edit: &mut PartEdit<'_>,
        body: &BodyId,
        reversed_parameterization: bool,
    ) -> SplitFaceRequest {
        let part = edit.id();
        let store = edit.store_mut_for_test();
        let face = store.faces_of_body(body.raw()).unwrap()[0];
        let face_data = store.get(face).unwrap();
        let loop_id = face_data.loops()[0];
        let surface = face_data.surface();
        let fins = store.get(loop_id).unwrap().fins().to_vec();
        let start = store
            .vertex_position(store.fin_tail(fins[0]).unwrap().unwrap())
            .unwrap();
        let end = store
            .vertex_position(store.fin_tail(fins[2]).unwrap().unwrap())
            .unwrap();
        let delta = end - start;
        let length = delta.norm();
        let curve = store
            .insert_curve(CurveGeom::Line(Line::new(start, delta).unwrap()))
            .unwrap();
        let plane = match store.get(surface).unwrap() {
            SurfaceGeom::Plane(plane) => *plane,
            _ => panic!("block face must be planar"),
        };
        let local_start = plane.frame().to_local(start);
        let local_end = plane.frame().to_local(end);
        let uv_start = Point2::new(local_start.x, local_start.y);
        let uv_end = Point2::new(local_end.x, local_end.y);
        let range = ParamRange::new(0.0, length);
        let (pcurve_start, pcurve_delta, parameter_map) = if reversed_parameterization {
            (
                uv_end,
                uv_start - uv_end,
                PcurveParameterMap::affine(-1.0, length).unwrap(),
            )
        } else {
            (uv_start, uv_end - uv_start, PcurveParameterMap::identity())
        };
        let mut make_pcurve = || {
            store
                .insert_pcurve(Curve2dGeom::Line(
                    Line2d::new(pcurve_start, pcurve_delta).unwrap(),
                ))
                .unwrap()
        };
        let forward = make_pcurve();
        let reversed = make_pcurve();
        SplitFaceRequest::new(
            LoopId::new(part.clone(), loop_id),
            [0, 2],
            BoundedCurve::new(CurveId::new(part.clone(), curve), range),
            [
                BoundedPcurve::new(PcurveId::new(part.clone(), forward), range)
                    .with_parameter_map(parameter_map),
                BoundedPcurve::new(PcurveId::new(part, reversed), range)
                    .with_parameter_map(parameter_map),
            ],
        )
    }

    fn node_visits(outcome: &OperationOutcome<ChangeJournal>) -> u64 {
        outcome
            .report()
            .usage()
            .iter()
            .find(|usage| {
                usage.stage == eval_stage::NODE_VISITS && usage.resource == ResourceKind::Work
            })
            .unwrap()
            .consumed
    }

    #[test]
    fn semantic_split_and_merge_commit_facade_lineage_and_checked_state() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let body = block(&mut edit);
        let request = split_request(&mut edit, &body);
        let source_face = {
            let source = edit
                .store_mut_for_test()
                .get(request.loop_id.raw())
                .unwrap()
                .face();
            FaceId::new(part_id.clone(), source)
        };
        let imported = crate::EntityTolerance::imported_xt(2.0 * LINEAR_RESOLUTION).unwrap();
        {
            let store = edit.store_mut_for_test();
            let mut setup = store.transaction().unwrap();
            setup
                .assembly()
                .get_mut(source_face.raw())
                .unwrap()
                .tolerance = Some(imported);
            setup.commit_checked_body(body.raw()).unwrap();
        }
        let original_face_count = edit.as_part().faces().len();

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        assert_eq!(transaction.part(), part_id);
        let split = transaction.split_face(request).unwrap();
        let outcome = transaction.commit(core::slice::from_ref(&body)).unwrap();
        let journal = outcome.result().unwrap();
        assert!(
            journal
                .mutations()
                .any(|mutation| mutation.kind() == MutationKind::Created)
        );
        let mut lineage = journal.lineage();
        let LineageView::Split { source, pieces } = lineage.next().unwrap() else {
            panic!("split must retain semantic lineage");
        };
        assert_eq!(source, pieces.clone().next().unwrap());
        assert_eq!(pieces.len(), 2);
        assert!(lineage.next().is_none());
        assert_eq!(journal.face_tolerance_propagation_count(), 1);
        assert!(matches!(
            journal.face_tolerance_propagations().next(),
            Some(FaceTolerancePropagationView::Inherited {
                source,
                result,
                tolerance: Some(value),
            }) if source == source_face && result == split.face() && value == imported
        ));
        assert_eq!(journal.tolerance_budget_count(), 0);
        assert_eq!(journal.tolerance_event_count(), 0);
        assert_eq!(edit.as_part().faces().len(), original_face_count + 1);
        edit.as_part().face(split.face()).unwrap();
        edit.as_part().edge(split.edge()).unwrap();

        // Simulate a later exact child so the merge-side growth introduces a
        // distinct operation origin; facade adaptation must retain both input
        // provenances and the selected result without lower-layer identities.
        {
            let store = edit.store_mut_for_test();
            let mut reset_child = store.transaction().unwrap();
            reset_child
                .assembly()
                .get_mut(split.face().raw())
                .unwrap()
                .tolerance = None;
            reset_child.commit_checked_body(body.raw()).unwrap();
        }

        let mut merge = edit.begin_edit(OperationSettings::default()).unwrap();
        merge
            .grow_tolerances(GrowTolerancesRequest::new(
                "facade-merge-face-maximum",
                4.0 * LINEAR_RESOLUTION,
                vec![ToleranceGrowth::new(
                    ToleranceGrowthTarget::Face(split.face()),
                    5.0 * LINEAR_RESOLUTION,
                )],
            ))
            .unwrap();
        merge
            .merge_faces(MergeFacesRequest::new(split.edge()))
            .unwrap();
        let outcome = merge.commit(core::slice::from_ref(&body)).unwrap();
        let journal = outcome.result().unwrap();
        assert!(matches!(
            journal.lineage().next(),
            Some(LineageView::Merge { .. })
        ));
        let inherited = journal.tolerance_events().next().unwrap().current();
        assert_eq!(
            inherited.origin(),
            ToleranceOrigin::Operation("facade-merge-face-maximum")
        );
        assert_eq!(
            inherited.last_operation(),
            Some("facade-merge-face-maximum")
        );
        assert!(matches!(
            journal.face_tolerance_propagations().next(),
            Some(FaceTolerancePropagationView::CombinedMax {
                sources,
                source_tolerances,
                result,
                selected_source: Some(selected),
                tolerance: Some(value),
            }) if sources == [source_face.clone(), split.face()]
                && source_tolerances == [Some(imported), Some(inherited)]
                && result == source_face
                && selected == split.face()
                && value == inherited
        ));
        assert_eq!(edit.as_part().faces().len(), original_face_count);
        assert!(matches!(
            edit.as_part().face(split.face()),
            Err(Error::StaleEntity {
                kind: EntityKind::Face
            })
        ));
        assert!(matches!(
            edit.as_part().edge(split.edge()),
            Err(Error::StaleEntity {
                kind: EntityKind::Edge
            })
        ));
    }

    #[test]
    fn tolerance_batch_is_atomic_ordered_provenanced_and_journal_scoped() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let foreign_part = session.create_part();
        let foreign_body = session
            .edit_part(foreign_part.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let foreign_face = {
            let part = session.part(foreign_part).unwrap();
            part.body(foreign_body)
                .unwrap()
                .faces()
                .unwrap()
                .next()
                .unwrap()
        };

        let mut edit = session.edit_part(part_id).unwrap();
        let body = block(&mut edit);
        let (face, edge, vertex) = tolerance_targets(&mut edit, &body);
        let seed_request = seed_body_request(&mut edit, &body);
        let imported = crate::EntityTolerance::imported_xt(2.0 * LINEAR_RESOLUTION).unwrap();
        {
            let store = edit.store_mut_for_test();
            let mut setup = store.transaction().unwrap();
            setup.assembly().get_mut(edge.raw()).unwrap().tolerance = Some(imported);
            setup.commit_checked_body(body.raw()).unwrap();
        }
        let stale_vertex = {
            let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
            let seed = transaction.create_seed_body(seed_request.clone()).unwrap();
            transaction.rollback().unwrap();
            seed.vertex()
        };
        let batch = |limit| {
            GrowTolerancesRequest::new(
                "facade-heal",
                limit,
                vec![
                    ToleranceGrowth::new(
                        ToleranceGrowthTarget::Vertex(vertex.clone()),
                        4.0 * LINEAR_RESOLUTION,
                    ),
                    ToleranceGrowth::new(
                        ToleranceGrowthTarget::Face(face.clone()),
                        3.0 * LINEAR_RESOLUTION,
                    ),
                    ToleranceGrowth::new(
                        ToleranceGrowthTarget::Edge(edge.clone()),
                        5.0 * LINEAR_RESOLUTION,
                    ),
                ],
            )
        };

        let rolled_back = {
            let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
            let result = transaction
                .grow_tolerances(batch(8.0 * LINEAR_RESOLUTION))
                .unwrap();
            assert_eq!(result.budget().index(), 0);
            transaction.rollback().unwrap();
            result
        };
        assert_eq!(edit.as_part().face(face.clone()).unwrap().tolerance(), None);
        assert_eq!(
            edit.as_part().edge(edge.clone()).unwrap().tolerance(),
            Some(imported)
        );
        assert_eq!(
            edit.as_part().vertex(vertex.clone()).unwrap().tolerance(),
            None
        );

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        let wrong_part_and_invalid = GrowTolerancesRequest::new(
            "facade-heal",
            f64::NAN,
            vec![ToleranceGrowth::new(
                ToleranceGrowthTarget::Face(foreign_face),
                f64::NAN,
            )],
        );
        assert!(matches!(
            transaction.grow_tolerances(wrong_part_and_invalid),
            Err(Error::WrongPart { .. })
        ));
        let stale_and_invalid = GrowTolerancesRequest::new(
            "facade-heal",
            f64::NAN,
            vec![ToleranceGrowth::new(
                ToleranceGrowthTarget::Vertex(stale_vertex),
                f64::NAN,
            )],
        );
        assert!(matches!(
            transaction.grow_tolerances(stale_and_invalid),
            Err(Error::StaleEntity {
                kind: EntityKind::Vertex
            })
        ));
        assert!(
            transaction
                .grow_tolerances(GrowTolerancesRequest::new(
                    "facade-heal",
                    -1.0,
                    vec![ToleranceGrowth::new(
                        ToleranceGrowthTarget::Face(face.clone()),
                        3.0 * LINEAR_RESOLUTION,
                    )],
                ))
                .is_err()
        );
        assert!(
            transaction
                .grow_tolerances(GrowTolerancesRequest::new(
                    "facade-heal",
                    LINEAR_RESOLUTION,
                    vec![ToleranceGrowth::new(
                        ToleranceGrowthTarget::Face(face.clone()),
                        -1.0,
                    )],
                ))
                .is_err()
        );
        assert!(
            transaction
                .grow_tolerances(GrowTolerancesRequest::new(
                    "facade-heal",
                    LINEAR_RESOLUTION,
                    vec![ToleranceGrowth::new(
                        ToleranceGrowthTarget::Face(face.clone()),
                        0.5 * LINEAR_RESOLUTION,
                    )],
                ))
                .is_err()
        );
        assert!(
            transaction
                .grow_tolerances(GrowTolerancesRequest::new(
                    "facade-heal",
                    8.0 * LINEAR_RESOLUTION,
                    vec![
                        ToleranceGrowth::new(
                            ToleranceGrowthTarget::Face(face.clone()),
                            3.0 * LINEAR_RESOLUTION,
                        ),
                        ToleranceGrowth::new(
                            ToleranceGrowthTarget::Face(face.clone()),
                            4.0 * LINEAR_RESOLUTION,
                        ),
                    ],
                ))
                .is_err()
        );
        assert!(
            transaction
                .grow_tolerances(batch(7.0 * LINEAR_RESOLUTION))
                .is_err()
        );
        assert_eq!(
            transaction
                .inner
                .store()
                .get(face.raw())
                .unwrap()
                .tolerance(),
            None
        );
        assert_eq!(
            transaction
                .inner
                .store()
                .get(edge.raw())
                .unwrap()
                .tolerance(),
            Some(imported)
        );
        assert_eq!(
            transaction
                .inner
                .store()
                .get(vertex.raw())
                .unwrap()
                .tolerance(),
            None
        );

        let result = transaction
            .grow_tolerances(batch(8.0 * LINEAR_RESOLUTION))
            .unwrap();
        assert_eq!(result, rolled_back);
        let outcome = transaction.commit(core::slice::from_ref(&body)).unwrap();
        let journal = outcome.result().unwrap();
        let budget = journal.tolerance_budget(result.budget()).unwrap();
        assert_eq!(budget.operation(), "facade-heal");
        assert_eq!(budget.limit(), 8.0 * LINEAR_RESOLUTION);
        assert_eq!(budget.consumed(), 8.0 * LINEAR_RESOLUTION);
        assert_eq!(budget.remaining(), 0.0);
        let events = journal.tolerance_events().collect::<Vec<_>>();
        assert_eq!(events.len(), 3);
        assert_eq!(events[0].entity(), &JournalEntity::Vertex(vertex.clone()));
        assert_eq!(events[1].entity(), &JournalEntity::Face(face.clone()));
        assert_eq!(events[2].entity(), &JournalEntity::Edge(edge.clone()));
        assert!(events.iter().all(|event| event.budget() == result.budget()));
        assert_eq!(events[2].previous(), Some(imported));
        assert_eq!(events[2].current().origin(), ToleranceOrigin::ImportedXt);
        assert_eq!(events[2].current().origin_value(), imported.value());
        assert_eq!(events[2].current().last_operation(), Some("facade-heal"));

        let committed_face = edit
            .as_part()
            .face(face.clone())
            .unwrap()
            .tolerance()
            .unwrap();
        let mut denied = edit.begin_edit(OperationSettings::default()).unwrap();
        denied
            .grow_tolerances(GrowTolerancesRequest::new(
                "denied-heal",
                LINEAR_RESOLUTION,
                vec![ToleranceGrowth::new(
                    ToleranceGrowthTarget::Face(face.clone()),
                    4.0 * LINEAR_RESOLUTION,
                )],
            ))
            .unwrap();
        denied.create_seed_body(seed_request).unwrap();
        let outcome = denied.commit(core::slice::from_ref(&body)).unwrap();
        assert!(outcome.result().is_err());
        assert_eq!(
            edit.as_part().face(face).unwrap().tolerance(),
            Some(committed_face)
        );
    }

    #[test]
    fn full_commit_adapts_evidence_and_restores_rejected_candidates() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let body = block(&mut edit);
        let raw_face = edit.store_mut_for_test().faces_of_body(body.raw()).unwrap()[0];
        let original_domain = edit.store_mut_for_test().get(raw_face).unwrap().domain();

        let mut rejected = edit.begin_edit(OperationSettings::default()).unwrap();
        rejected.inner.assembly().get_mut(raw_face).unwrap().domain = None;
        let rejected = rejected
            .commit_full(
                core::slice::from_ref(&body),
                FullCommitRequirement::RequireValid,
            )
            .unwrap();
        let rejected = rejected.result().unwrap();
        assert!(!rejected.is_committed());
        assert!(rejected.journal().is_none());
        assert_eq!(rejected.reports().len(), 1);
        assert_eq!(rejected.reports()[0].body(), body);
        assert_eq!(
            rejected.reports()[0].report().outcome(),
            CheckOutcome::Indeterminate
        );
        assert_eq!(
            edit.store_mut_for_test().get(raw_face).unwrap().domain(),
            original_domain
        );

        let mut allowed = edit.begin_edit(OperationSettings::default()).unwrap();
        allowed.inner.assembly().get_mut(raw_face).unwrap().domain = None;
        let allowed = allowed
            .commit_full(
                core::slice::from_ref(&body),
                FullCommitRequirement::AllowIndeterminate,
            )
            .unwrap();
        assert!(!allowed.report().usage().is_empty());
        let allowed = allowed.result().unwrap();
        assert!(allowed.is_committed());
        assert!(allowed.journal().is_some());
        assert_eq!(
            allowed.reports()[0].report().outcome(),
            CheckOutcome::Indeterminate
        );

        let raw_sphere =
            ktopo::make::sphere(edit.store_mut_for_test(), &Frame::world(), 2.0).unwrap();
        let sphere = BodyId::new(part_id.clone(), raw_sphere);
        let raw_sphere_face = edit.store_mut_for_test().faces_of_body(raw_sphere).unwrap()[0];
        let sphere_face = FaceId::new(part_id, raw_sphere_face);
        let original_sense = edit
            .store_mut_for_test()
            .get(raw_sphere_face)
            .unwrap()
            .sense();
        let mut invalid = edit.begin_edit(OperationSettings::default()).unwrap();
        invalid
            .grow_tolerances(GrowTolerancesRequest::new(
                "full-facade-rejected",
                LINEAR_RESOLUTION,
                vec![ToleranceGrowth::new(
                    ToleranceGrowthTarget::Face(sphere_face.clone()),
                    2.0 * LINEAR_RESOLUTION,
                )],
            ))
            .unwrap();
        invalid
            .inner
            .assembly()
            .get_mut(raw_sphere_face)
            .unwrap()
            .sense = match original_sense {
            RawSense::Forward => RawSense::Reversed,
            RawSense::Reversed => RawSense::Forward,
        };
        let invalid = invalid
            .commit_full(&[], FullCommitRequirement::RequireValid)
            .unwrap();
        let invalid = invalid.result().unwrap();
        assert!(!invalid.is_committed());
        assert_eq!(invalid.reports().len(), 1);
        assert_eq!(invalid.reports()[0].body(), sphere);
        assert_eq!(
            invalid.reports()[0].report().outcome(),
            CheckOutcome::Invalid
        );
        assert_eq!(edit.as_part().face(sphere_face).unwrap().tolerance(), None);
        assert_eq!(
            edit.store_mut_for_test()
                .get(raw_sphere_face)
                .unwrap()
                .sense(),
            original_sense
        );
    }

    #[test]
    fn full_commit_preflights_wrong_part_and_stale_explicit_roots() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let foreign_part = session.create_part();
        let foreign_body = session
            .edit_part(foreign_part)
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();

        let mut edit = session.edit_part(part_id).unwrap();
        let body = block(&mut edit);
        let wrong_part = edit
            .begin_edit(OperationSettings::default())
            .unwrap()
            .commit_full(
                core::slice::from_ref(&foreign_body),
                FullCommitRequirement::RequireValid,
            )
            .unwrap_err();
        assert!(matches!(wrong_part, Error::WrongPart { .. }));

        let request = seed_body_request(&mut edit, &body);
        let stale_body = {
            let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
            let seed = transaction.create_seed_body(request).unwrap();
            transaction.rollback().unwrap();
            seed.body()
        };
        let stale = edit
            .begin_edit(OperationSettings::default())
            .unwrap()
            .commit_full(
                core::slice::from_ref(&stale_body),
                FullCommitRequirement::RequireValid,
            )
            .unwrap_err();
        assert!(matches!(
            stale,
            Error::StaleEntity {
                kind: EntityKind::Body
            }
        ));
        edit.as_part().body(body).unwrap();
    }

    #[test]
    fn full_commit_graph_limit_is_structured_and_atomic_at_n_minus_one() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id).unwrap();
        let body = block(&mut edit);
        let (face, _, _) = tolerance_targets(&mut edit, &body);
        let settings = |allowed| {
            OperationSettings::new()
                .with_budget_overrides(kgraph::EvalBudgetProfile::for_limits(64, allowed))
        };
        let growth = || {
            GrowTolerancesRequest::new(
                "full-limit",
                LINEAR_RESOLUTION,
                vec![ToleranceGrowth::new(
                    ToleranceGrowthTarget::Face(face.clone()),
                    2.0 * LINEAR_RESOLUTION,
                )],
            )
        };

        let mut denied = edit.begin_edit(settings(305)).unwrap();
        denied.grow_tolerances(growth()).unwrap();
        let denied = denied
            .commit_full(
                core::slice::from_ref(&body),
                FullCommitRequirement::RequireValid,
            )
            .unwrap();
        assert_eq!(denied.report().limit_events().len(), 1);
        let crossing = denied.report().limit_events()[0];
        assert_eq!(crossing.stage, eval_stage::NODE_VISITS);
        assert_eq!(crossing.resource, ResourceKind::Work);
        assert_eq!((crossing.consumed, crossing.allowed), (306, 305));
        assert_eq!(denied.result().unwrap_err().limit(), Some(crossing));
        assert_eq!(edit.as_part().face(face.clone()).unwrap().tolerance(), None);

        let mut admitted = edit.begin_edit(settings(306)).unwrap();
        admitted.grow_tolerances(growth()).unwrap();
        let admitted = admitted
            .commit_full(
                core::slice::from_ref(&body),
                FullCommitRequirement::RequireValid,
            )
            .unwrap();
        assert!(admitted.report().limit_events().is_empty());
        let admitted = admitted.result().unwrap();
        assert!(admitted.is_committed());
        assert_eq!(
            admitted.reports()[0].report().outcome(),
            CheckOutcome::Valid
        );
        assert_eq!(
            edit.as_part()
                .face(face)
                .unwrap()
                .tolerance()
                .unwrap()
                .value(),
            2.0 * LINEAR_RESOLUTION
        );
    }

    #[test]
    fn checked_seed_body_round_trip_is_transient_atomic_and_identity_exact() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let foreign_part = session.create_part();
        let foreign_body = session
            .edit_part(foreign_part.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let foreign_surface = {
            let part = session.part(foreign_part).unwrap();
            let face = part
                .body(foreign_body)
                .unwrap()
                .faces()
                .unwrap()
                .next()
                .unwrap();
            part.face(face).unwrap().surface()
        };

        let mut edit = session.edit_part(part_id).unwrap();
        let body = block(&mut edit);
        let request = seed_body_request(&mut edit, &body);
        let point_count = edit.store_mut_for_test().count::<Point3>();
        let (rolled_back, rolled_back_point) = {
            let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
            let made = transaction.create_seed_body(request.clone()).unwrap();
            let point = transaction
                .inner
                .store()
                .get(made.vertex.raw())
                .unwrap()
                .point();
            transaction.rollback().unwrap();
            (made, point)
        };
        for entity_is_stale in [
            edit.as_part().body(rolled_back.body()).is_err(),
            edit.as_part().region(rolled_back.void_region()).is_err(),
            edit.as_part().region(rolled_back.solid_region()).is_err(),
            edit.as_part().shell(rolled_back.shell()).is_err(),
            edit.as_part().face(rolled_back.face()).is_err(),
            edit.as_part().loop_(rolled_back.loop_id()).is_err(),
            edit.as_part().vertex(rolled_back.vertex()).is_err(),
        ] {
            assert!(entity_is_stale);
        }

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        assert!(
            transaction
                .remove_seed_body(RemoveSeedBodyRequest::new(rolled_back.body()))
                .is_err()
        );
        assert!(
            transaction
                .remove_seed_body(RemoveSeedBodyRequest::new(body.clone()))
                .is_err()
        );
        let wrong_part =
            CreateSeedBodyRequest::new(foreign_surface, request.sense(), request.position());
        assert!(matches!(
            transaction.create_seed_body(wrong_part),
            Err(Error::WrongPart { .. })
        ));
        let bad_position = CreateSeedBodyRequest::new(
            request.surface(),
            request.sense(),
            Point3::new(f64::NAN, 0.0, 0.0),
        );
        assert!(transaction.create_seed_body(bad_position).is_err());

        let made = transaction.create_seed_body(request.clone()).unwrap();
        assert_eq!(made, rolled_back);
        assert_eq!(
            transaction
                .inner
                .store()
                .get(made.vertex.raw())
                .unwrap()
                .point(),
            rolled_back_point
        );
        let removed = transaction
            .remove_seed_body(RemoveSeedBodyRequest::new(made.body()))
            .unwrap();
        assert_eq!(removed.body(), made.body());
        let outcome = transaction.commit(core::slice::from_ref(&body)).unwrap();
        let journal = outcome.result().unwrap();
        let lineage = journal.lineage().collect::<Vec<_>>();
        assert_eq!(lineage.len(), 3);
        let LineageView::DerivedFrom { derived, source } = &lineage[0] else {
            panic!("position-owning MVFS must derive the seed vertex from its point");
        };
        assert_eq!(*derived, JournalEntity::Vertex(made.vertex()));
        assert!(matches!(source, JournalEntity::Point(_)));
        let hidden_point = source.clone();
        let LineageView::Deleted { entity } = &lineage[1] else {
            panic!("KVFS must record the deleted seed body");
        };
        assert_eq!(*entity, JournalEntity::Body(made.body()));
        let LineageView::Deleted { entity } = &lineage[2] else {
            panic!("position-owning KVFS must record the deleted hidden point");
        };
        assert_eq!(*entity, hidden_point);
        assert_eq!(edit.store_mut_for_test().count::<Point3>(), point_count);
        assert!(edit.as_part().body(body.clone()).is_ok());

        let mut transient = edit.begin_edit(OperationSettings::default()).unwrap();
        let incomplete = transient.create_seed_body(request).unwrap();
        let denied = transient.commit(core::slice::from_ref(&body)).unwrap();
        assert!(denied.result().is_err());
        assert!(matches!(
            edit.as_part().body(incomplete.body()),
            Err(Error::StaleEntity {
                kind: EntityKind::Body
            })
        ));
        assert_eq!(edit.store_mut_for_test().count::<Point3>(), point_count);
    }

    #[test]
    fn checked_strut_round_trip_preserves_metadata_lineage_and_future_ids() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let foreign_part = session.create_part();
        let foreign_body = session
            .edit_part(foreign_part.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let foreign_loop = {
            let part = session.part(foreign_part).unwrap();
            let face = part
                .body(foreign_body)
                .unwrap()
                .faces()
                .unwrap()
                .next()
                .unwrap();
            part.face(face).unwrap().loops().next().unwrap()
        };
        let mut edit = session.edit_part(part_id).unwrap();
        let (body, loop_id, ordinary_edge, request, off_curve) = planar_strut_request(&mut edit);
        let point_count = edit.store_mut_for_test().count::<Point3>();
        let expected_map = PcurveParameterMap::affine(-1.0, core::f64::consts::SQRT_2).unwrap();

        let (rolled_back, rolled_back_point) = {
            let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
            let created = transaction.create_strut(request.clone()).unwrap();
            let point = transaction
                .inner
                .store()
                .get(created.vertex.raw())
                .unwrap()
                .point();
            for fin in created.fins() {
                let use_ = transaction
                    .inner
                    .store()
                    .get(fin.raw())
                    .unwrap()
                    .pcurve()
                    .unwrap();
                assert_eq!(
                    PcurveParameterMap::from_raw(use_.edge_to_pcurve()),
                    expected_map
                );
                assert_eq!(PcurveMetadata::from_raw(use_), PcurveMetadata::regular());
            }
            transaction.rollback().unwrap();
            (created, point)
        };
        assert!(matches!(
            edit.as_part().edge(rolled_back.edge()),
            Err(Error::StaleEntity {
                kind: EntityKind::Edge
            })
        ));
        assert!(matches!(
            edit.as_part().vertex(rolled_back.vertex()),
            Err(Error::StaleEntity {
                kind: EntityKind::Vertex
            })
        ));

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        assert!(
            transaction
                .remove_strut(RemoveStrutRequest::new(rolled_back.edge()))
                .is_err()
        );
        assert!(
            transaction
                .remove_strut(RemoveStrutRequest::new(ordinary_edge))
                .is_err()
        );
        let mut wrong_part = request.clone();
        wrong_part.loop_id = foreign_loop;
        assert!(matches!(
            transaction.create_strut(wrong_part),
            Err(Error::WrongPart { .. })
        ));
        let mut bad_position = request.clone();
        bad_position.position = Point3::new(f64::NAN, 0.0, 0.0);
        assert!(transaction.create_strut(bad_position).is_err());
        let mut bad_index = request.clone();
        bad_index.fin_index = usize::MAX;
        assert!(transaction.create_strut(bad_index).is_err());
        let mut bad_pcurve = request.clone();
        bad_pcurve.pcurves[0] = BoundedPcurve::new(off_curve, ParamRange::new(0.0, 1.0));
        assert!(transaction.create_strut(bad_pcurve).is_err());

        let created = transaction.create_strut(request).unwrap();
        assert_eq!(created, rolled_back);
        assert_eq!(
            transaction
                .inner
                .store()
                .get(created.vertex.raw())
                .unwrap()
                .point(),
            rolled_back_point
        );
        assert_eq!(
            transaction
                .remove_strut(RemoveStrutRequest::new(created.edge()))
                .unwrap()
                .loop_id(),
            loop_id
        );
        let outcome = transaction.commit(core::slice::from_ref(&body)).unwrap();
        let journal = outcome.result().unwrap();
        let lineage = journal.lineage().collect::<Vec<_>>();
        assert_eq!(lineage.len(), 5);
        let LineageView::DerivedFrom { derived, source } = &lineage[0] else {
            panic!("MEV must derive its edge from the destination loop");
        };
        assert_eq!(*derived, JournalEntity::Edge(created.edge()));
        assert_eq!(*source, JournalEntity::Loop(loop_id.clone()));
        let LineageView::DerivedFrom { derived, source } = &lineage[1] else {
            panic!("MEV must derive its vertex from the inserted point");
        };
        assert_eq!(*derived, JournalEntity::Vertex(created.vertex()));
        assert!(matches!(source, JournalEntity::Point(_)));
        let inserted_point = source.clone();
        let LineageView::Deleted { entity } = &lineage[2] else {
            panic!("KEV must record the deleted edge");
        };
        assert_eq!(*entity, JournalEntity::Edge(created.edge()));
        let LineageView::Deleted { entity } = &lineage[3] else {
            panic!("KEV must record the deleted vertex");
        };
        assert_eq!(*entity, JournalEntity::Vertex(created.vertex()));
        let LineageView::Deleted { entity } = &lineage[4] else {
            panic!("position-owning KEV must record the deleted point");
        };
        assert_eq!(*entity, inserted_point);
        assert_eq!(edit.store_mut_for_test().count::<Point3>(), point_count);
        assert!(edit.as_part().loop_(loop_id).is_ok());
        assert!(matches!(
            edit.as_part().edge(created.edge()),
            Err(Error::StaleEntity {
                kind: EntityKind::Edge
            })
        ));
        assert!(matches!(
            edit.as_part().vertex(created.vertex()),
            Err(Error::StaleEntity {
                kind: EntityKind::Vertex
            })
        ));
    }

    #[test]
    fn checked_join_remove_ring_round_trip_preserves_metadata_lineage_and_identity() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id).unwrap();
        let (body, request) = planar_annulus_join_request(&mut edit);
        let original_ring = request.ring();
        let outer = request.outer();
        let expected_map = request.pcurves()[0].parameter_map();
        let expected_metadata = request.pcurves()[0].metadata();
        let original_loop_count = edit
            .as_part()
            .face(edit.as_part().loop_(outer.clone()).unwrap().face())
            .unwrap()
            .loops()
            .len();
        assert_eq!(original_loop_count, 2);

        let rolled_back = {
            let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
            let joined = transaction.join_ring(request.clone()).unwrap();
            assert_eq!(joined.loop_id(), outer);
            for fin in joined.fins() {
                let raw_use = transaction
                    .inner
                    .store()
                    .get(fin.raw())
                    .unwrap()
                    .pcurve()
                    .unwrap();
                assert_eq!(
                    PcurveParameterMap::from_raw(raw_use.edge_to_pcurve()),
                    expected_map
                );
                assert_eq!(PcurveMetadata::from_raw(raw_use), expected_metadata);
            }
            transaction.rollback().unwrap();
            joined
        };
        assert!(edit.as_part().loop_(original_ring.clone()).is_ok());
        assert!(matches!(
            edit.as_part().edge(rolled_back.edge()),
            Err(Error::StaleEntity {
                kind: EntityKind::Edge
            })
        ));

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        let joined = transaction.join_ring(request).unwrap();
        assert_eq!(joined, rolled_back);
        let split = transaction
            .remove_bridge(RemoveBridgeRequest::new(joined.edge()))
            .unwrap();
        assert_eq!(split.outer(), outer);
        let outcome = transaction.commit(core::slice::from_ref(&body)).unwrap();
        let journal = outcome.result().unwrap();
        let lineage = journal.lineage().collect::<Vec<_>>();
        assert_eq!(lineage.len(), 3);
        let LineageView::DerivedFrom { derived, source } = &lineage[0] else {
            panic!("ring join must derive the bridge from the ring");
        };
        assert_eq!(*derived, JournalEntity::Edge(joined.edge()));
        assert_eq!(*source, JournalEntity::Loop(original_ring.clone()));
        let LineageView::Merge { sources, result } = &lineage[1] else {
            panic!("ring join must merge the ring into the outer loop");
        };
        assert_eq!(
            sources.clone().collect::<Vec<_>>(),
            vec![
                JournalEntity::Loop(outer.clone()),
                JournalEntity::Loop(original_ring.clone())
            ]
        );
        assert_eq!(*result, JournalEntity::Loop(outer.clone()));
        let LineageView::Split { source, pieces } = &lineage[2] else {
            panic!("bridge removal must split the merged loop");
        };
        assert_eq!(*source, JournalEntity::Loop(outer.clone()));
        assert_eq!(
            pieces.clone().collect::<Vec<_>>(),
            vec![
                JournalEntity::Loop(outer.clone()),
                JournalEntity::Loop(split.ring())
            ]
        );
        assert!(edit.as_part().loop_(split.outer()).is_ok());
        assert!(edit.as_part().loop_(split.ring()).is_ok());
        assert!(matches!(
            edit.as_part().loop_(original_ring),
            Err(Error::StaleEntity {
                kind: EntityKind::Loop
            })
        ));
        assert!(matches!(
            edit.as_part().edge(joined.edge()),
            Err(Error::StaleEntity {
                kind: EntityKind::Edge
            })
        ));
    }

    #[test]
    fn ring_join_preflight_rejects_positions_and_pcurves_without_mutation() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id).unwrap();
        let (body, request) = planar_annulus_join_request(&mut edit);
        let ring = request.ring();
        let loop_count = edit.as_part().loops().len();

        let mut bad_index = request.clone();
        bad_index.outer_fin_index = usize::MAX;
        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        assert!(transaction.join_ring(bad_index).is_err());
        assert!(transaction.inner.store().get(ring.raw()).is_ok());
        transaction.rollback().unwrap();

        let mut bad_range = request.clone();
        let range = bad_range.pcurves[0].range;
        bad_range.pcurves[0].range = ParamRange::new(range.lo, range.lerp(0.5));
        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        assert!(transaction.join_ring(bad_range).is_err());
        assert!(transaction.inner.store().get(ring.raw()).is_ok());
        transaction.rollback().unwrap();

        let mut bad_chart = request.clone();
        bad_chart.pcurves[0].metadata =
            PcurveMetadata::regular().with_chart(PcurveChart::shifted([1.0, 0.0]).unwrap());
        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        assert!(transaction.join_ring(bad_chart).is_err());
        assert!(transaction.inner.store().get(ring.raw()).is_ok());
        transaction.rollback().unwrap();

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        let joined = transaction.join_ring(request).unwrap();
        let split = transaction
            .remove_bridge(RemoveBridgeRequest::new(joined.edge()))
            .unwrap();
        transaction
            .commit(core::slice::from_ref(&body))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(edit.as_part().loops().len(), loop_count);
        assert!(edit.as_part().loop_(split.ring()).is_ok());
    }

    #[test]
    fn ring_edits_reject_wrong_part_and_stale_identities_before_mutation() {
        let mut session = Kernel::new().create_session();
        let first_part = session.create_part();
        let second_part = session.create_part();
        let (first_body, first_request) = {
            let mut edit = session.edit_part(first_part.clone()).unwrap();
            planar_annulus_join_request(&mut edit)
        };
        let (second_request, foreign_bridge) = {
            let mut edit = session.edit_part(second_part.clone()).unwrap();
            let (_, request) = planar_annulus_join_request(&mut edit);
            let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
            let bridge = transaction.join_ring(request.clone()).unwrap().edge();
            transaction.rollback().unwrap();
            (request, bridge)
        };

        let mut edit = session.edit_part(first_part.clone()).unwrap();
        let loop_count = edit.as_part().loops().len();
        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        assert!(matches!(
            transaction.join_ring(second_request),
            Err(Error::WrongPart { expected, actual })
                if expected == first_part && actual == second_part
        ));
        assert!(matches!(
            transaction.remove_bridge(RemoveBridgeRequest::new(foreign_bridge)),
            Err(Error::WrongPart { expected, actual })
                if expected == first_part && actual == second_part
        ));
        transaction.rollback().unwrap();
        assert_eq!(edit.as_part().loops().len(), loop_count);

        let stale_ring = first_request.ring();
        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        let joined = transaction.join_ring(first_request.clone()).unwrap();
        let stale_edge = joined.edge();
        transaction
            .remove_bridge(RemoveBridgeRequest::new(stale_edge.clone()))
            .unwrap();
        transaction
            .commit(core::slice::from_ref(&first_body))
            .unwrap()
            .into_result()
            .unwrap();

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        assert!(matches!(
            transaction.join_ring(first_request),
            Err(Error::StaleEntity {
                kind: EntityKind::Loop
            })
        ));
        assert!(matches!(
            transaction.remove_bridge(RemoveBridgeRequest::new(stale_edge)),
            Err(Error::StaleEntity {
                kind: EntityKind::Edge
            })
        ));
        transaction.rollback().unwrap();
        assert!(matches!(
            edit.as_part().loop_(stale_ring),
            Err(Error::StaleEntity {
                kind: EntityKind::Loop
            })
        ));
        assert_eq!(edit.as_part().loops().len(), loop_count);
    }

    #[test]
    fn face_hole_merge_split_round_trip_preserves_pcurves_lineage_and_identity() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let body = block(&mut edit);
        let split_request = split_request(&mut edit, &body);
        let keep = edit
            .as_part()
            .loop_(split_request.loop_id())
            .unwrap()
            .face();
        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        let split = transaction.split_face(split_request).unwrap();
        transaction
            .commit(core::slice::from_ref(&body))
            .unwrap()
            .into_result()
            .unwrap();
        let remove = split.face();
        let ring = split.loop_id();

        let (equivalent_surface, remove_sense) = {
            let store = edit.store_mut_for_test();
            let remove_face = store.get(remove.raw()).unwrap();
            let remove_sense = remove_face.sense();
            let original_surface = remove_face.surface();
            let plane = match store.get(original_surface).unwrap() {
                SurfaceGeom::Plane(plane) => *plane,
                _ => panic!("split block face must remain planar"),
            };
            let equivalent = store.insert_surface(SurfaceGeom::Plane(plane)).unwrap();
            let mut transaction = store.transaction().unwrap();
            transaction
                .assembly()
                .get_mut(remove.raw())
                .unwrap()
                .surface = equivalent;
            transaction.commit_checked_body(body.raw()).unwrap();
            (SurfaceId::new(part_id.clone(), equivalent), remove_sense)
        };
        assert_ne!(
            edit.as_part().face(keep.clone()).unwrap().surface(),
            equivalent_surface
        );

        let expected_pcurves = edit
            .as_part()
            .loop_(ring.clone())
            .unwrap()
            .fins()
            .map(|fin| {
                let part = edit.as_part();
                let fin = part.fin(fin).unwrap();
                (
                    fin.pcurve(),
                    fin.pcurve_range(),
                    fin.pcurve_parameter_map(),
                    fin.pcurve_metadata(),
                )
            })
            .collect::<Vec<_>>();
        let merge_request = MergeFaceAsHoleRequest::new(keep.clone(), remove.clone());
        let split_request =
            SplitHoleAsFaceRequest::new(ring.clone(), equivalent_surface.clone(), remove_sense);

        let rolled_back = {
            let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
            let merged = transaction
                .merge_face_as_hole(merge_request.clone())
                .unwrap();
            assert_eq!(merged.face(), keep);
            assert_eq!(merged.ring(), ring);
            let detached = transaction
                .split_hole_as_face(split_request.clone())
                .unwrap();
            assert_eq!(detached.source_face(), keep);
            assert_eq!(detached.loop_id(), ring);
            let actual_pcurves = transaction
                .inner
                .store()
                .get(ring.raw())
                .unwrap()
                .fins()
                .iter()
                .map(|&fin| {
                    let use_ = transaction
                        .inner
                        .store()
                        .get(fin)
                        .unwrap()
                        .pcurve()
                        .unwrap();
                    (
                        Some(PcurveId::new(part_id.clone(), use_.curve())),
                        Some(use_.range()),
                        Some(PcurveParameterMap::from_raw(use_.edge_to_pcurve())),
                        Some(PcurveMetadata::from_raw(use_)),
                    )
                })
                .collect::<Vec<_>>();
            assert_eq!(actual_pcurves, expected_pcurves);
            transaction.rollback().unwrap();
            (merged, detached)
        };
        assert!(edit.as_part().face(remove.clone()).is_ok());
        assert!(matches!(
            edit.as_part().face(rolled_back.1.face()),
            Err(Error::StaleEntity {
                kind: EntityKind::Face
            })
        ));

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        let merged = transaction.merge_face_as_hole(merge_request).unwrap();
        let detached = transaction.split_hole_as_face(split_request).unwrap();
        assert_eq!((merged.clone(), detached.clone()), rolled_back);
        let outcome = transaction.commit(core::slice::from_ref(&body)).unwrap();
        let journal = outcome.result().unwrap();
        let lineage = journal.lineage().collect::<Vec<_>>();
        assert_eq!(lineage.len(), 2);
        let LineageView::Merge { sources, result } = &lineage[0] else {
            panic!("face-as-hole must journal a face merge");
        };
        assert_eq!(
            sources.clone().collect::<Vec<_>>(),
            vec![
                JournalEntity::Face(keep.clone()),
                JournalEntity::Face(remove.clone())
            ]
        );
        assert_eq!(*result, JournalEntity::Face(keep.clone()));
        let LineageView::Split { source, pieces } = &lineage[1] else {
            panic!("hole detachment must journal a face split");
        };
        assert_eq!(*source, JournalEntity::Face(keep.clone()));
        assert_eq!(
            pieces.clone().collect::<Vec<_>>(),
            vec![
                JournalEntity::Face(keep.clone()),
                JournalEntity::Face(detached.face())
            ]
        );
        assert!(edit.as_part().face(keep).is_ok());
        assert!(edit.as_part().face(detached.face()).is_ok());
        assert_eq!(edit.as_part().loop_(ring).unwrap().face(), detached.face());
        assert!(matches!(
            edit.as_part().face(remove),
            Err(Error::StaleEntity {
                kind: EntityKind::Face
            })
        ));
        edit.as_part().body(body).unwrap();
    }

    #[test]
    fn face_hole_preflight_rejects_invalid_ownership_and_incidence_atomically() {
        let mut session = Kernel::new().create_session();
        let first_part = session.create_part();
        let second_part = session.create_part();
        let (body, faces, loop_id, surface, sense) = {
            let mut edit = session.edit_part(first_part.clone()).unwrap();
            let body = block(&mut edit);
            let part = edit.as_part();
            let faces = part
                .body(body.clone())
                .unwrap()
                .faces()
                .unwrap()
                .take(2)
                .collect::<Vec<_>>();
            let face = part.face(faces[0].clone()).unwrap();
            let loop_id = face.loops().next().unwrap();
            (body, faces, loop_id, face.surface(), face.sense())
        };
        let foreign_face = {
            let mut edit = session.edit_part(second_part.clone()).unwrap();
            let body = block(&mut edit);
            edit.as_part()
                .body(body)
                .unwrap()
                .faces()
                .unwrap()
                .next()
                .unwrap()
        };

        let mut edit = session.edit_part(first_part.clone()).unwrap();
        let original_face_count = edit.as_part().faces().len();
        let original_loop_count = edit.as_part().loops().len();
        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        assert!(
            transaction
                .merge_face_as_hole(MergeFaceAsHoleRequest::new(
                    faces[0].clone(),
                    faces[0].clone(),
                ))
                .is_err()
        );
        assert!(
            transaction
                .merge_face_as_hole(MergeFaceAsHoleRequest::new(
                    faces[0].clone(),
                    faces[1].clone(),
                ))
                .is_err()
        );
        assert!(
            transaction
                .split_hole_as_face(SplitHoleAsFaceRequest::new(loop_id, surface, sense,))
                .is_err()
        );
        assert!(matches!(
            transaction.merge_face_as_hole(MergeFaceAsHoleRequest::new(
                faces[0].clone(),
                foreign_face,
            )),
            Err(Error::WrongPart { expected, actual })
                if expected == first_part && actual == second_part
        ));
        transaction.rollback().unwrap();
        assert_eq!(edit.as_part().faces().len(), original_face_count);
        assert_eq!(edit.as_part().loops().len(), original_loop_count);
        edit.as_part().body(body).unwrap();
    }

    #[test]
    fn affine_pcurve_maps_validate_and_round_trip() {
        let identity = PcurveParameterMap::identity();
        assert_eq!((identity.scale(), identity.offset()), (1.0, 0.0));
        assert_eq!(identity.map(2.5), 2.5);
        assert_eq!(identity.inverse(2.5), 2.5);

        let reversed = PcurveParameterMap::affine(-2.0, 7.0).unwrap();
        assert_eq!((reversed.scale(), reversed.offset()), (-2.0, 7.0));
        assert_eq!(reversed.map(1.5), 4.0);
        assert_eq!(reversed.inverse(4.0), 1.5);

        for (scale, offset) in [
            (0.0, 0.0),
            (f64::NAN, 0.0),
            (f64::INFINITY, 0.0),
            (1.0, f64::NEG_INFINITY),
        ] {
            assert!(PcurveParameterMap::affine(scale, offset).is_err());
        }
    }

    #[test]
    fn facade_pcurve_metadata_values_validate_and_round_trip() {
        let chart = PcurveChart::shifted([1.0, -2.0]).unwrap();
        assert_eq!(chart.period_shifts(), [1, -2]);
        assert!(!chart.is_identity());
        assert!(PcurveChart::identity().is_identity());
        assert_eq!(PcurveChart::integer([3, -4]).period_shifts(), [3, -4]);
        for invalid in [
            [f64::NAN, 0.0],
            [f64::INFINITY, 0.0],
            [0.5, 0.0],
            [f64::from(i32::MAX) + 1.0, 0.0],
        ] {
            assert!(PcurveChart::shifted(invalid).is_err());
        }

        let seam = PcurveSeam::new(SurfaceParameter::U, PcurveSeamSide::Upper);
        let metadata = PcurveMetadata::regular()
            .with_chart(chart)
            .with_endpoint_kinds([
                PcurveEndpointKind::Regular,
                PcurveEndpointKind::SurfaceSingularity,
            ])
            .with_closure_winding([1, 0])
            .with_seam(seam);
        assert_eq!(metadata.chart(), chart);
        assert_eq!(
            metadata.endpoint_kinds(),
            [
                PcurveEndpointKind::Regular,
                PcurveEndpointKind::SurfaceSingularity
            ]
        );
        assert_eq!(metadata.closure_winding(), Some([1, 0]));
        assert_eq!(metadata.seam(), Some(seam));
        assert_eq!(seam.direction(), SurfaceParameter::U);
        assert_eq!(seam.side(), PcurveSeamSide::Upper);
    }

    #[test]
    fn periodic_seam_and_closed_uses_are_facade_visible() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let raw = ktopo::make::cylindrical_sheet(
                edit.store_mut_for_test(),
                &Frame::world(),
                1.25,
                2.5,
            )
            .unwrap();
            BodyId::new(part_id.clone(), raw)
        };
        let part = session.part(part_id).unwrap();
        let fins = part
            .body(body)
            .unwrap()
            .edges()
            .unwrap()
            .flat_map(|edge| part.edge(edge).unwrap().fins().collect::<Vec<_>>())
            .collect::<Vec<_>>();
        let metadata = fins
            .into_iter()
            .map(|fin| part.fin(fin).unwrap().pcurve_metadata().unwrap())
            .collect::<Vec<_>>();
        assert!(metadata.iter().any(|value| {
            value.chart() == PcurveChart::integer([1, 0])
                && value.seam() == Some(PcurveSeam::new(SurfaceParameter::U, PcurveSeamSide::Upper))
        }));
        assert!(metadata.iter().any(|value| {
            value.chart().is_identity()
                && value.seam() == Some(PcurveSeam::new(SurfaceParameter::U, PcurveSeamSide::Lower))
        }));
        assert_eq!(
            metadata
                .iter()
                .filter(|value| value.closure_winding() == Some([1, 0]))
                .count(),
            2
        );
    }

    #[test]
    fn singular_endpoint_metadata_survives_checked_split_merge_and_rollback() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id).unwrap();
        let (body, request) = spherical_sector_split_request(&mut edit);
        let expected = request.pcurves()[0].metadata();

        let rolled_back = {
            let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
            let made = transaction.split_face(request.clone()).unwrap();
            transaction.rollback().unwrap();
            made
        };
        assert!(matches!(
            edit.as_part().edge(rolled_back.edge()),
            Err(Error::StaleEntity {
                kind: EntityKind::Edge
            })
        ));

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        let split = transaction.split_face(request).unwrap();
        assert_eq!(split, rolled_back);
        transaction
            .commit(core::slice::from_ref(&body))
            .unwrap()
            .into_result()
            .unwrap();
        let part = edit.as_part();
        for fin in split.fins() {
            let view = part.fin(fin).unwrap();
            assert_eq!(view.pcurve_metadata(), Some(expected));
            assert_eq!(
                view.pcurve_endpoint_kinds(),
                Some([
                    PcurveEndpointKind::Regular,
                    PcurveEndpointKind::SurfaceSingularity
                ])
            );
        }

        let mut merge = edit.begin_edit(OperationSettings::default()).unwrap();
        merge
            .merge_faces(MergeFacesRequest::new(split.edge()))
            .unwrap();
        merge
            .commit(core::slice::from_ref(&body))
            .unwrap()
            .into_result()
            .unwrap();
        edit.as_part().body(body).unwrap();
    }

    #[test]
    fn invalid_chart_and_inconsistent_metadata_leave_split_atomic() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id).unwrap();
        let body = block(&mut edit);
        let original_face_count = edit.as_part().faces().len();

        let mut invalid_chart = split_request(&mut edit, &body);
        let shifted =
            PcurveMetadata::regular().with_chart(PcurveChart::shifted([1.0, 0.0]).unwrap());
        invalid_chart.pcurves[0].metadata = shifted;
        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        assert!(transaction.split_face(invalid_chart).is_err());
        transaction.rollback().unwrap();
        assert_eq!(edit.as_part().faces().len(), original_face_count);

        let mut inconsistent = split_request(&mut edit, &body);
        let singular = PcurveMetadata::regular().with_endpoint_kinds([
            PcurveEndpointKind::SurfaceSingularity,
            PcurveEndpointKind::Regular,
        ]);
        inconsistent.pcurves[0].metadata = singular;
        inconsistent.pcurves[1].metadata = singular;
        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        let made = transaction.split_face(inconsistent).unwrap();
        let outcome = transaction.commit(core::slice::from_ref(&body)).unwrap();
        assert!(outcome.result().is_err());
        assert_eq!(edit.as_part().faces().len(), original_face_count);
        assert!(matches!(
            edit.as_part().edge(made.edge()),
            Err(Error::StaleEntity {
                kind: EntityKind::Edge
            })
        ));
    }

    #[test]
    fn reversed_pcurve_maps_commit_through_facade_views_and_merge() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id).unwrap();
        let body = block(&mut edit);
        let request = reversed_split_request(&mut edit, &body);
        let range = request.pcurves()[0].range();
        let map = request.pcurves()[0].parameter_map();
        assert!(map.scale() < 0.0);
        assert_eq!(map.map(range.lo), range.hi);
        assert_eq!(map.map(range.hi), range.lo);

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        let split = transaction.split_face(request).unwrap();
        transaction
            .commit(core::slice::from_ref(&body))
            .unwrap()
            .into_result()
            .unwrap();

        let part = edit.as_part();
        for fin in split.fins() {
            let view = part.fin(fin).unwrap();
            assert_eq!(view.pcurve_range(), Some(range));
            assert_eq!(view.pcurve_parameter_map(), Some(map));
            assert!(view.pcurve().is_some());
        }

        let mut merge = edit.begin_edit(OperationSettings::default()).unwrap();
        merge
            .merge_faces(MergeFacesRequest::new(split.edge()))
            .unwrap();
        merge
            .commit(core::slice::from_ref(&body))
            .unwrap()
            .into_result()
            .unwrap();
        edit.as_part().body(body).unwrap();
    }

    #[test]
    fn rollback_and_failed_commit_restore_identity_and_candidate_topology() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id).unwrap();
        let body = block(&mut edit);
        let request = split_request(&mut edit, &body);
        let original_face_count = edit.as_part().faces().len();

        let first = {
            let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
            let made = transaction.split_face(request.clone()).unwrap();
            transaction.rollback().unwrap();
            made
        };
        assert_eq!(edit.as_part().faces().len(), original_face_count);
        assert!(matches!(
            edit.as_part().face(first.face()),
            Err(Error::StaleEntity {
                kind: EntityKind::Face
            })
        ));

        let mut transaction = edit.begin_edit(OperationSettings::default()).unwrap();
        let repeated = transaction.split_face(request.clone()).unwrap();
        assert_eq!(repeated, first);
        drop(transaction);
        assert_eq!(edit.as_part().faces().len(), original_face_count);

        let mut success = edit.begin_edit(OperationSettings::default()).unwrap();
        let success_split = success.split_face(request.clone()).unwrap();
        let success = success.commit(core::slice::from_ref(&body)).unwrap();
        let visits = node_visits(&success);
        assert!(visits > 0);
        assert!(matches!(
            success
                .result()
                .as_ref()
                .unwrap()
                .face_tolerance_propagations()
                .next(),
            Some(FaceTolerancePropagationView::Inherited {
                source: _,
                result,
                tolerance: None,
            }) if result == success_split.face()
        ));

        let mut merge = edit.begin_edit(OperationSettings::default()).unwrap();
        merge
            .merge_faces(MergeFacesRequest::new(success_split.edge()))
            .unwrap();
        let merge_journal = merge
            .commit(core::slice::from_ref(&body))
            .unwrap()
            .into_result()
            .unwrap();
        assert!(matches!(
            merge_journal.face_tolerance_propagations().next(),
            Some(FaceTolerancePropagationView::CombinedMax {
                source_tolerances: [None, None],
                selected_source: None,
                tolerance: None,
                ..
            })
        ));

        let denied_settings = OperationSettings::default().with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                eval_stage::NODE_VISITS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                visits - 1,
            )])
            .unwrap(),
        );
        let mut denied = edit.begin_edit(denied_settings).unwrap();
        let denied_split = denied.split_face(request.clone()).unwrap();
        let outcome = denied.commit(core::slice::from_ref(&body)).unwrap();
        let error = outcome.result().unwrap_err();
        let crossing = error.limit().unwrap();
        assert_eq!(crossing.stage, eval_stage::NODE_VISITS);
        assert_eq!((crossing.consumed, crossing.allowed), (visits, visits - 1));
        assert_eq!(edit.as_part().faces().len(), original_face_count);
        assert!(matches!(
            edit.as_part().face(denied_split.face()),
            Err(Error::StaleEntity {
                kind: EntityKind::Face
            })
        ));

        let mut repeated = edit.begin_edit(OperationSettings::default()).unwrap();
        let repeated_split = repeated.split_face(request).unwrap();
        assert_eq!(repeated_split, denied_split);
        repeated.rollback().unwrap();
        assert_eq!(edit.as_part().faces().len(), original_face_count);
    }

    #[test]
    fn wrong_part_is_rejected_before_equal_raw_edit_identities() {
        let mut session = Kernel::new().create_session();
        let first_part = session.create_part();
        let second_part = session.create_part();
        let (first_body, first_request) = {
            let mut first = session.edit_part(first_part.clone()).unwrap();
            let body = block(&mut first);
            let request = split_request(&mut first, &body);
            (body, request)
        };
        let second_request = {
            let mut second = session.edit_part(second_part.clone()).unwrap();
            let body = block(&mut second);
            split_request(&mut second, &body)
        };
        assert_eq!(first_request.loop_id.raw(), second_request.loop_id.raw());

        let mut first = session.edit_part(first_part.clone()).unwrap();
        let original_face_count = first.as_part().faces().len();
        let mut transaction = first.begin_edit(OperationSettings::default()).unwrap();
        assert!(matches!(
            transaction.split_face(second_request),
            Err(Error::WrongPart { expected, actual })
                if expected == first_part && actual == second_part
        ));
        drop(transaction);
        assert_eq!(first.as_part().faces().len(), original_face_count);
        first.as_part().body(first_body).unwrap();
    }
}
