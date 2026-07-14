//! Checked semantic topology edits at the supported facade boundary.

use kcore::operation::OperationContext;
use ktopo::entity::{
    FinPcurve, ParamMap1d, PcurveChart as RawPcurveChart,
    PcurveEndpointKind as RawPcurveEndpointKind, PcurveSeam as RawPcurveSeam,
    SeamSide as RawPcurveSeamSide, SurfaceParameter as RawSurfaceParameter,
};
use ktopo::euler::FinPcurvePair;
use ktopo::transaction::Transaction;

use crate::error::{Error, Result};
use crate::session::PartEdit;
use crate::{
    BodyId, BoundedCurve, ChangeJournal, CurveId, EdgeId, EntityKind, FaceId, FinId, LoopId,
    OperationOutcome, OperationSettings, ParamRange, PartId, PcurveId,
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

    /// Split one face using the transaction's pcurve-aware checked operator.
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
        let pcurves = FinPcurvePair::new(
            forward.metadata.apply_to_raw(FinPcurve::new(
                forward.pcurve.raw(),
                forward.range,
                forward.parameter_map.into_raw(),
            )?),
            reversed.metadata.apply_to_raw(FinPcurve::new(
                reversed.pcurve.raw(),
                reversed.range,
                reversed.parameter_map.into_raw(),
            )?),
        );
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
}

#[cfg(test)]
mod tests {
    use kcore::operation::{AccountingMode, BudgetPlan, LimitSpec, ResourceKind};
    use kgeom::curve::{Circle, Line};
    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::surface::Sphere;
    use kgeom::vec::{Point2, Point3, Vec2, Vec3};
    use kgraph::eval_stage;
    use ktopo::entity::{
        FaceDomain as RawFaceDomain, FinPcurve, ParamMap1d,
        PcurveEndpointKind as RawPcurveEndpointKind,
    };
    use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};

    use super::*;
    use crate::{BlockRequest, Kernel, LineageView, MutationKind};

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
        assert_eq!(edit.as_part().faces().len(), original_face_count + 1);
        edit.as_part().face(split.face()).unwrap();
        edit.as_part().edge(split.edge()).unwrap();

        let mut merge = edit.begin_edit(OperationSettings::default()).unwrap();
        merge
            .merge_faces(MergeFacesRequest::new(split.edge()))
            .unwrap();
        let outcome = merge.commit(core::slice::from_ref(&body)).unwrap();
        let journal = outcome.result().unwrap();
        assert!(matches!(
            journal.lineage().next(),
            Some(LineageView::Merge { .. })
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

        let mut merge = edit.begin_edit(OperationSettings::default()).unwrap();
        merge
            .merge_faces(MergeFacesRequest::new(success_split.edge()))
            .unwrap();
        merge
            .commit(core::slice::from_ref(&body))
            .unwrap()
            .into_result()
            .unwrap();

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
        let denied_split = denied.split_face(request).unwrap();
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
