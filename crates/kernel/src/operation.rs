//! Typed contextual operations at the supported façade boundary.

use core::fmt;

use kcore::operation::{ChildWorkLedger, OperationContext, OperationPolicyError, OperationScope};
use kgeom::param::ParamRange;
use kgraph::{EvalBudgetProfile, EvalContext, EvalLimits, EvalUsage};
#[cfg(test)]
use ktopo::check::FullCheckBudgetProfile;
use ktopo::entity::EntityRef as RawEntityRef;
use ktopo::transaction::{
    FaceTolerancePropagation as RawFaceTolerancePropagation, LineageEvent as RawLineageEvent,
    MutationKind as RawMutationKind,
};

use crate::error::{Error, Result};
use crate::session::{Part, PartEdit};
use crate::{
    BodyId, BudgetPlan, CheckLevel, CheckOutcome, CurveId, DiagnosticLevel, EdgeId, EntityKind,
    FaceId, FaultKind, FinId, Frame, JournalPointId, LoopId, PartId, PcurveId, Point3, RegionId,
    SessionPolicy, ShellId, SurfaceId, Tolerances, Vec3, VerificationGapCause, VerificationGapKind,
    VertexId,
};

/// F2 settings used to construct one operation context at a façade call.
///
/// The fields reuse the shared F2 configuration types. Session precision,
/// numerical policy, execution policy, and policy version remain fixed by the
/// owning [`crate::Session`].
#[derive(Debug, Clone, PartialEq)]
pub struct OperationSettings {
    tolerances: Tolerances,
    budget_overrides: BudgetPlan,
    diagnostic_level: DiagnosticLevel,
    diagnostic_capacity: usize,
}

impl OperationSettings {
    /// Settings at the Parasolid-compatible model tolerance, with no budget
    /// overrides or retained diagnostics.
    pub fn new() -> Self {
        Self::default()
    }

    /// Replace model-space operation tolerances.
    pub fn with_tolerances(mut self, tolerances: Tolerances) -> Self {
        self.tolerances = tolerances;
        self
    }

    /// Overlay operation-local limits on the session's default budget.
    pub fn with_budget_overrides(mut self, budget_overrides: BudgetPlan) -> Self {
        self.budget_overrides = budget_overrides;
        self
    }

    /// Retain at most `capacity` semantic diagnostics at the requested level.
    pub fn with_diagnostics(mut self, level: DiagnosticLevel, capacity: usize) -> Self {
        self.diagnostic_level = level;
        self.diagnostic_capacity = capacity;
        self
    }

    /// Configured model-space tolerances.
    pub const fn tolerances(&self) -> Tolerances {
        self.tolerances
    }

    /// Operation-local budget overrides.
    pub const fn budget_overrides(&self) -> &BudgetPlan {
        &self.budget_overrides
    }

    /// Diagnostic retention level.
    pub const fn diagnostic_level(&self) -> DiagnosticLevel {
        self.diagnostic_level
    }

    /// Maximum retained diagnostic count.
    pub const fn diagnostic_capacity(&self) -> usize {
        self.diagnostic_capacity
    }

    pub(crate) fn context<'session>(
        &self,
        policy: &'session SessionPolicy,
    ) -> Result<OperationContext<'session>> {
        Ok(OperationContext::new(policy, self.tolerances)?
            .with_budget_overrides(self.budget_overrides.clone())
            .with_diagnostics(self.diagnostic_level, self.diagnostic_capacity))
    }
}

impl Default for OperationSettings {
    fn default() -> Self {
        Self {
            tolerances: Tolerances::default(),
            budget_overrides: BudgetPlan::empty(),
            diagnostic_level: DiagnosticLevel::Off,
            diagnostic_capacity: 0,
        }
    }
}

/// Typed request to construct one checked solid block.
#[derive(Debug, Clone, PartialEq)]
pub struct BlockRequest {
    frame: Frame,
    extents: [f64; 3],
    settings: OperationSettings,
}

impl BlockRequest {
    /// Construct a block request using default operation settings.
    pub fn new(frame: Frame, extents: [f64; 3]) -> Self {
        Self {
            frame,
            extents,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Requested placement frame.
    pub const fn frame(&self) -> Frame {
        self.frame
    }

    /// Requested side lengths along the frame axes.
    pub const fn extents(&self) -> [f64; 3] {
        self.extents
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Typed request to duplicate one complete body under an
/// orientation-preserving rigid placement.
///
/// Source model coordinates are interpreted in the placement frame: a source
/// point `(x, y, z)` becomes `placement.point_at(x, y, z)`. Topology,
/// supporting geometry, pcurves, bounds, tolerances, and periodic chart
/// metadata are independently owned by the result.
#[derive(Debug, Clone, PartialEq)]
pub struct CopyBodyRequest {
    body: BodyId,
    placement: Frame,
    settings: OperationSettings,
}

impl CopyBodyRequest {
    /// Construct a rigid body-copy request using default operation settings.
    pub fn new(body: BodyId, placement: Frame) -> Self {
        Self {
            body,
            placement,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Source body identity.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Rigid placement applied to source model coordinates.
    pub const fn placement(&self) -> Frame {
        self.placement
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Typed request to extrude one polygonal planar profile into a checked solid.
///
/// The outer boundary and each hole omit a repeated closing point. Validation
/// normalizes the outer boundary counterclockwise and holes clockwise, rejects
/// degenerate/intersecting/nested inputs, and extrudes along the positive
/// `frame.z()` direction.
#[derive(Debug, Clone, PartialEq)]
pub struct ExtrudeProfileRequest {
    frame: Frame,
    outer: Vec<kgeom::vec::Point2>,
    holes: Vec<Vec<kgeom::vec::Point2>>,
    height: f64,
    settings: OperationSettings,
}

impl ExtrudeProfileRequest {
    /// Construct a polygonal-profile extrusion using default operation settings.
    pub fn new(
        frame: Frame,
        outer: Vec<kgeom::vec::Point2>,
        holes: Vec<Vec<kgeom::vec::Point2>>,
        height: f64,
    ) -> Self {
        Self {
            frame,
            outer,
            holes,
            height,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Positioned profile plane and positive extrusion axis.
    pub const fn frame(&self) -> Frame {
        self.frame
    }

    /// Outer polygon without a repeated closing point.
    pub fn outer(&self) -> &[kgeom::vec::Point2] {
        &self.outer
    }

    /// Hole polygons in deterministic request order.
    pub fn holes(&self) -> &[Vec<kgeom::vec::Point2>] {
        &self.holes
    }

    /// Positive extrusion distance along `frame.z()`.
    pub const fn height(&self) -> f64 {
        self.height
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Typed request to extrude one polygonal profile by an oblique translation.
///
/// The translation may contain an in-plane component but must have a finite,
/// nonzero component along `frame.z()`. Boundary normalization and rejection
/// rules match [`ExtrudeProfileRequest`].
#[derive(Debug, Clone, PartialEq)]
pub struct ExtrudeProfileAlongRequest {
    frame: Frame,
    outer: Vec<kgeom::vec::Point2>,
    holes: Vec<Vec<kgeom::vec::Point2>>,
    translation: Vec3,
    settings: OperationSettings,
}

impl ExtrudeProfileAlongRequest {
    /// Construct an oblique polygonal-profile extrusion with default settings.
    pub fn new(
        frame: Frame,
        outer: Vec<kgeom::vec::Point2>,
        holes: Vec<Vec<kgeom::vec::Point2>>,
        translation: Vec3,
    ) -> Self {
        Self {
            frame,
            outer,
            holes,
            translation,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Positioned profile plane and reference normal orientation.
    pub const fn frame(&self) -> Frame {
        self.frame
    }

    /// Outer polygon without a repeated closing point.
    pub fn outer(&self) -> &[kgeom::vec::Point2] {
        &self.outer
    }

    /// Hole polygons in deterministic request order.
    pub fn holes(&self) -> &[Vec<kgeom::vec::Point2>] {
        &self.holes
    }

    /// Complete model-space translation from the base cap to the top cap.
    pub const fn translation(&self) -> Vec3 {
        self.translation
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Facade-safe identity retained by a committed journal.
///
/// Every variant is part-qualified. A deleted topology or geometry identity
/// remains comparable in journal evidence even though resolving it as a live
/// part view would fail. Point geometry has no ordinary facade view, so its
/// opaque identity is intentionally useful only as journal evidence.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum JournalEntity {
    /// Body identity.
    Body(BodyId),
    /// Region identity.
    Region(RegionId),
    /// Shell identity.
    Shell(ShellId),
    /// Face identity.
    Face(FaceId),
    /// Loop identity.
    Loop(LoopId),
    /// Fin identity.
    Fin(FinId),
    /// Edge identity.
    Edge(EdgeId),
    /// Vertex identity.
    Vertex(VertexId),
    /// Three-dimensional curve geometry identity.
    Curve(CurveId),
    /// Supporting-surface geometry identity.
    Surface(SurfaceId),
    /// Point geometry identity retained only for journal tracking.
    Point(JournalPointId),
    /// Parameter-space curve geometry identity.
    Pcurve(PcurveId),
}

impl JournalEntity {
    /// Part whose committed journal owns this identity.
    pub fn part(&self) -> PartId {
        match self {
            Self::Body(id) => id.part().clone(),
            Self::Region(id) => id.part().clone(),
            Self::Shell(id) => id.part().clone(),
            Self::Face(id) => id.part().clone(),
            Self::Loop(id) => id.part().clone(),
            Self::Fin(id) => id.part().clone(),
            Self::Edge(id) => id.part().clone(),
            Self::Vertex(id) => id.part().clone(),
            Self::Curve(id) => id.part().clone(),
            Self::Surface(id) => id.part().clone(),
            Self::Point(id) => id.part().clone(),
            Self::Pcurve(id) => id.part().clone(),
        }
    }

    /// Stable semantic kind without exposing a lower-layer handle type.
    pub const fn kind(&self) -> EntityKind {
        match self {
            Self::Body(_) => EntityKind::Body,
            Self::Region(_) => EntityKind::Region,
            Self::Shell(_) => EntityKind::Shell,
            Self::Face(_) => EntityKind::Face,
            Self::Loop(_) => EntityKind::Loop,
            Self::Fin(_) => EntityKind::Fin,
            Self::Edge(_) => EntityKind::Edge,
            Self::Vertex(_) => EntityKind::Vertex,
            Self::Curve(_) => EntityKind::Curve,
            Self::Surface(_) => EntityKind::Surface,
            Self::Point(_) => EntityKind::Point,
            Self::Pcurve(_) => EntityKind::Pcurve,
        }
    }
}

/// Net kind of one committed facade mutation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum MutationKind {
    /// An identity became live.
    Created,
    /// A pre-existing identity changed and remains live.
    Modified,
    /// A pre-existing identity was removed.
    Deleted,
}

/// One deterministic committed net mutation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MutationView {
    entity: JournalEntity,
    kind: MutationKind,
}

impl MutationView {
    /// Affected part-qualified identity.
    pub const fn entity(&self) -> &JournalEntity {
        &self.entity
    }

    /// Net mutation kind.
    pub const fn kind(&self) -> MutationKind {
        self.kind
    }
}

/// Deterministically ordered facade journal identities.
#[derive(Clone)]
pub struct JournalEntities<'journal> {
    part: PartId,
    inner: core::slice::Iter<'journal, RawEntityRef>,
}

impl<'journal> JournalEntities<'journal> {
    fn new(part: PartId, entities: &'journal [RawEntityRef]) -> Self {
        Self {
            part,
            inner: entities.iter(),
        }
    }
}

impl Iterator for JournalEntities<'_> {
    type Item = JournalEntity;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner
            .next()
            .map(|&entity| adapt_journal_entity(&self.part, entity))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.inner.size_hint()
    }
}

impl ExactSizeIterator for JournalEntities<'_> {}
impl core::iter::FusedIterator for JournalEntities<'_> {}

impl fmt::Debug for JournalEntities<'_> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("JournalEntities")
            .field("remaining", &self.len())
            .finish_non_exhaustive()
    }
}

/// One semantic identity relationship in committed operation order.
#[derive(Debug)]
#[non_exhaustive]
pub enum LineageView<'journal> {
    /// `derived` was constructed from `source` without replacing it.
    DerivedFrom {
        /// New or changed identity.
        derived: JournalEntity,
        /// Source identity.
        source: JournalEntity,
    },
    /// One identity was divided into ordered result pieces.
    Split {
        /// Identity that was split.
        source: JournalEntity,
        /// Deterministically ordered result identities.
        pieces: JournalEntities<'journal>,
    },
    /// Ordered source identities were combined into one result.
    Merge {
        /// Deterministically ordered source identities.
        sources: JournalEntities<'journal>,
        /// Combined result identity.
        result: JournalEntity,
    },
    /// One identity was superseded by another.
    Replaced {
        /// Superseded identity.
        old: JournalEntity,
        /// Replacement identity.
        new: JournalEntity,
    },
    /// One semantic identity was intentionally removed.
    Deleted {
        /// Removed identity, intentionally stale after commit.
        entity: JournalEntity,
    },
}

/// Opaque declaration-order identity of a transaction-owned tolerance budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ToleranceBudgetId(usize);

impl ToleranceBudgetId {
    pub(crate) const fn from_index(index: usize) -> Self {
        Self(index)
    }

    /// Stable declaration-order index within this journal.
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Final committed usage of one transaction-owned tolerance budget.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToleranceBudgetView {
    id: ToleranceBudgetId,
    operation: &'static str,
    limit: f64,
    consumed: f64,
}

impl ToleranceBudgetView {
    /// Journal-local declaration identity.
    pub const fn id(self) -> ToleranceBudgetId {
        self.id
    }

    /// Stable semantic operation name supplied at declaration.
    pub const fn operation(self) -> &'static str {
        self.operation
    }

    /// Maximum aggregate model-unit growth permitted.
    pub const fn limit(self) -> f64 {
        self.limit
    }

    /// Aggregate model-unit growth committed.
    pub const fn consumed(self) -> f64 {
        self.consumed
    }

    /// Unspent growth at commit time.
    pub fn remaining(self) -> f64 {
        (self.limit - self.consumed).max(0.0)
    }
}

/// One committed entity-tolerance change.
#[derive(Debug, Clone, PartialEq)]
pub struct ToleranceEventView {
    entity: JournalEntity,
    previous: Option<crate::EntityTolerance>,
    current: crate::EntityTolerance,
    budget: ToleranceBudgetId,
}

impl ToleranceEventView {
    /// Identity whose metric tolerance changed.
    pub const fn entity(&self) -> &JournalEntity {
        &self.entity
    }

    /// Prior tolerance, or `None` when an exact entity became tolerant.
    pub const fn previous(&self) -> Option<crate::EntityTolerance> {
        self.previous
    }

    /// Committed tolerance and its retained provenance.
    pub const fn current(&self) -> crate::EntityTolerance {
        self.current
    }

    /// Journal-local budget that authorized this change.
    pub const fn budget(&self) -> ToleranceBudgetId {
        self.budget
    }
}

/// Descriptive face-tolerance inheritance/combination evidence.
///
/// This journal view carries no budget identity or authoring capability.
/// Complete imported/operation provenance remains inside each tolerance value.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum FaceTolerancePropagationView {
    /// A face split copied the source tolerance to the new result face.
    Inherited {
        /// Existing source face.
        source: FaceId,
        /// New result face.
        result: FaceId,
        /// Copied tolerance, or `None` when the source was exact.
        tolerance: Option<crate::EntityTolerance>,
    },
    /// A face merge selected the larger input tolerance.
    CombinedMax {
        /// Ordered `[surviving, absorbed]` input faces.
        sources: [FaceId; 2],
        /// Input values in the same order.
        source_tolerances: [Option<crate::EntityTolerance>; 2],
        /// Surviving result face.
        result: FaceId,
        /// Input whose complete provenance was retained. Equal values select
        /// the surviving first source; two exact inputs select `None`.
        selected_source: Option<FaceId>,
        /// Selected result tolerance.
        tolerance: Option<crate::EntityTolerance>,
    },
}

/// Opaque owning adapter over one committed lower-layer journal.
pub struct ChangeJournal {
    part: PartId,
    inner: ktopo::transaction::Journal,
}

impl ChangeJournal {
    pub(crate) const fn from_raw(part: PartId, inner: ktopo::transaction::Journal) -> Self {
        Self { part, inner }
    }

    /// Part whose state was changed.
    pub fn part(&self) -> PartId {
        self.part.clone()
    }

    /// Net mutations in deterministic arena-type and slot order.
    pub fn mutations(&self) -> impl ExactSizeIterator<Item = MutationView> + '_ {
        self.inner.mutations().iter().map(|mutation| MutationView {
            entity: adapt_journal_entity(&self.part, mutation.entity),
            kind: adapt_mutation_kind(mutation.kind),
        })
    }

    /// Semantic lineage in caller-recorded operation order.
    pub fn lineage(&self) -> impl ExactSizeIterator<Item = LineageView<'_>> + '_ {
        self.inner
            .lineage()
            .iter()
            .map(|event| adapt_lineage_event(&self.part, event))
    }

    /// Transaction-owned tolerance budgets in declaration order.
    pub fn tolerance_budgets(&self) -> impl ExactSizeIterator<Item = ToleranceBudgetView> + '_ {
        self.inner
            .tolerance_budgets()
            .iter()
            .enumerate()
            .map(|(index, budget)| ToleranceBudgetView {
                id: ToleranceBudgetId(index),
                operation: budget.operation(),
                limit: budget.limit(),
                consumed: budget.consumed(),
            })
    }

    /// Resolve a journal-local budget identity returned by the edit that
    /// produced this committed journal.
    pub fn tolerance_budget(&self, id: ToleranceBudgetId) -> Option<ToleranceBudgetView> {
        self.tolerance_budgets().nth(id.index())
    }

    /// Entity-tolerance changes in semantic operation order.
    pub fn tolerance_events(&self) -> impl ExactSizeIterator<Item = ToleranceEventView> + '_ {
        self.inner
            .tolerance_events()
            .iter()
            .map(|event| ToleranceEventView {
                entity: adapt_journal_entity(&self.part, event.entity()),
                previous: event.previous(),
                current: event.current(),
                budget: ToleranceBudgetId(event.budget().index()),
            })
    }

    /// Face split/merge tolerance propagation in semantic operation order.
    pub fn face_tolerance_propagations(
        &self,
    ) -> impl ExactSizeIterator<Item = FaceTolerancePropagationView> + '_ {
        self.inner
            .face_tolerance_propagations()
            .iter()
            .map(|event| adapt_face_tolerance_propagation(&self.part, event))
    }

    /// Number of committed net mutations.
    pub fn mutation_count(&self) -> usize {
        self.inner.mutations().len()
    }

    /// Number of semantic lineage events.
    pub fn lineage_count(&self) -> usize {
        self.inner.lineage().len()
    }

    /// Number of committed transaction-owned tolerance budgets.
    pub fn tolerance_budget_count(&self) -> usize {
        self.inner.tolerance_budgets().len()
    }

    /// Number of committed entity-tolerance changes.
    pub fn tolerance_event_count(&self) -> usize {
        self.inner.tolerance_events().len()
    }

    /// Number of descriptive face-tolerance propagation records.
    pub fn face_tolerance_propagation_count(&self) -> usize {
        self.inner.face_tolerance_propagations().len()
    }

    #[cfg(test)]
    pub(crate) const fn raw_for_test(&self) -> &ktopo::transaction::Journal {
        &self.inner
    }
}

fn adapt_journal_entity(part: &PartId, entity: RawEntityRef) -> JournalEntity {
    let part = part.clone();
    match entity {
        RawEntityRef::Body(raw) => JournalEntity::Body(BodyId::new(part, raw)),
        RawEntityRef::Region(raw) => JournalEntity::Region(RegionId::new(part, raw)),
        RawEntityRef::Shell(raw) => JournalEntity::Shell(ShellId::new(part, raw)),
        RawEntityRef::Face(raw) => JournalEntity::Face(FaceId::new(part, raw)),
        RawEntityRef::Loop(raw) => JournalEntity::Loop(LoopId::new(part, raw)),
        RawEntityRef::Fin(raw) => JournalEntity::Fin(FinId::new(part, raw)),
        RawEntityRef::Edge(raw) => JournalEntity::Edge(EdgeId::new(part, raw)),
        RawEntityRef::Vertex(raw) => JournalEntity::Vertex(VertexId::new(part, raw)),
        RawEntityRef::Curve(raw) => JournalEntity::Curve(CurveId::new(part, raw)),
        RawEntityRef::Surface(raw) => JournalEntity::Surface(SurfaceId::new(part, raw)),
        RawEntityRef::Point(raw) => JournalEntity::Point(JournalPointId::new(part, raw)),
        RawEntityRef::Curve2d(raw) => JournalEntity::Pcurve(PcurveId::new(part, raw)),
    }
}

const fn adapt_mutation_kind(kind: RawMutationKind) -> MutationKind {
    match kind {
        RawMutationKind::Created => MutationKind::Created,
        RawMutationKind::Modified => MutationKind::Modified,
        RawMutationKind::Deleted => MutationKind::Deleted,
    }
}

fn adapt_lineage_event<'journal>(
    part: &PartId,
    event: &'journal RawLineageEvent,
) -> LineageView<'journal> {
    match event {
        RawLineageEvent::DerivedFrom { derived, source } => LineageView::DerivedFrom {
            derived: adapt_journal_entity(part, *derived),
            source: adapt_journal_entity(part, *source),
        },
        RawLineageEvent::Split { source, pieces } => LineageView::Split {
            source: adapt_journal_entity(part, *source),
            pieces: JournalEntities::new(part.clone(), pieces),
        },
        RawLineageEvent::Merge { sources, result } => LineageView::Merge {
            sources: JournalEntities::new(part.clone(), sources),
            result: adapt_journal_entity(part, *result),
        },
        RawLineageEvent::Replaced { old, new } => LineageView::Replaced {
            old: adapt_journal_entity(part, *old),
            new: adapt_journal_entity(part, *new),
        },
        RawLineageEvent::Deleted { entity } => LineageView::Deleted {
            entity: adapt_journal_entity(part, *entity),
        },
        _ => unreachable!("unadapted lower-layer lineage variant reached the facade"),
    }
}

fn adapt_face_tolerance_propagation(
    part: &PartId,
    event: &RawFaceTolerancePropagation,
) -> FaceTolerancePropagationView {
    match *event {
        RawFaceTolerancePropagation::Inherited {
            source,
            result,
            tolerance,
        } => FaceTolerancePropagationView::Inherited {
            source: FaceId::new(part.clone(), source),
            result: FaceId::new(part.clone(), result),
            tolerance,
        },
        RawFaceTolerancePropagation::CombinedMax {
            sources,
            source_tolerances,
            result,
            selected_source,
            tolerance,
        } => FaceTolerancePropagationView::CombinedMax {
            sources: sources.map(|face| FaceId::new(part.clone(), face)),
            source_tolerances,
            result: FaceId::new(part.clone(), result),
            selected_source: selected_source.map(|face| FaceId::new(part.clone(), face)),
            tolerance,
        },
        _ => unreachable!("unadapted lower-layer face-tolerance propagation reached the facade"),
    }
}

impl fmt::Debug for ChangeJournal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ChangeJournal")
            .field("part", &self.part)
            .field("mutation_count", &self.mutation_count())
            .field("lineage_count", &self.lineage_count())
            .field("tolerance_budget_count", &self.tolerance_budget_count())
            .field("tolerance_event_count", &self.tolerance_event_count())
            .field(
                "face_tolerance_propagation_count",
                &self.face_tolerance_propagation_count(),
            )
            .finish()
    }
}

/// Successfully committed primitive construction.
#[derive(Debug)]
pub struct BodyCreated {
    body: BodyId,
    journal: ChangeJournal,
}

impl BodyCreated {
    /// Created body identity.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Deterministic committed mutation evidence.
    pub const fn journal(&self) -> &ChangeJournal {
        &self.journal
    }

    /// Consume this result into its identity and journal.
    pub fn into_parts(self) -> (BodyId, ChangeJournal) {
        (self.body, self.journal)
    }
}

/// Typed request for a contextual body check.
#[derive(Debug, Clone, PartialEq)]
pub struct CheckBodyRequest {
    body: BodyId,
    level: CheckLevel,
    settings: OperationSettings,
}

/// Typed request for one bounded surface evaluation.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceEvaluationRequest {
    surface: SurfaceId,
    uv: [f64; 2],
    order: kgraph::SurfaceDerivativeOrder,
    settings: OperationSettings,
}

/// One facade curve identity restricted to a finite parameter interval.
#[derive(Debug, Clone, PartialEq)]
pub struct BoundedCurve {
    pub(crate) curve: CurveId,
    pub(crate) range: ParamRange,
}

impl BoundedCurve {
    /// Bind a facade curve identity to the requested parameter interval.
    pub const fn new(curve: CurveId, range: ParamRange) -> Self {
        Self { curve, range }
    }

    /// Exact graph-owned curve identity.
    pub fn curve(&self) -> CurveId {
        self.curve.clone()
    }

    /// Requested parameter interval.
    pub const fn range(&self) -> ParamRange {
        self.range
    }
}

/// Typed request for one graph-aware bounded curve/curve intersection.
#[derive(Debug, Clone, PartialEq)]
pub struct IntersectCurvesRequest {
    pub(crate) first: BoundedCurve,
    pub(crate) second: BoundedCurve,
    pub(crate) settings: OperationSettings,
}

impl IntersectCurvesRequest {
    /// Construct a request with default operation settings.
    pub fn new(first: BoundedCurve, second: BoundedCurve) -> Self {
        Self {
            first,
            second,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// First bounded facade curve.
    pub const fn first(&self) -> &BoundedCurve {
        &self.first
    }

    /// Second bounded facade curve.
    pub const fn second(&self) -> &BoundedCurve {
        &self.second
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Local character of one isolated curve/curve contact.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum CurveContactKind {
    /// Curve tangents are independent at the contact.
    Transverse,
    /// Curves touch without crossing, including overlap endpoints.
    Tangent,
    /// At least one curve is singular at the contact.
    Singular,
    /// A newer lower-layer contact classification is not yet named here.
    Unclassified,
}

/// One isolated facade curve/curve intersection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveCurvePoint {
    pub(crate) point: Point3,
    pub(crate) first_parameter: f64,
    pub(crate) second_parameter: f64,
    pub(crate) residual: f64,
    pub(crate) kind: CurveContactKind,
}

impl CurveCurvePoint {
    /// Symmetric model-space representative point.
    pub const fn point(&self) -> Point3 {
        self.point
    }
    /// Parameter on the first requested curve.
    pub const fn first_parameter(&self) -> f64 {
        self.first_parameter
    }
    /// Parameter on the second requested curve.
    pub const fn second_parameter(&self) -> f64 {
        self.second_parameter
    }
    /// Distance between the two evaluated curve points.
    pub const fn residual(&self) -> f64 {
        self.residual
    }
    /// Local contact character.
    pub const fn kind(&self) -> CurveContactKind {
        self.kind
    }
}

/// Direction correspondence between coincident parameter intervals.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum CurveOverlapOrientation {
    /// Low parameter corresponds to low parameter.
    Same,
    /// Low parameter corresponds to high parameter.
    Reversed,
}

/// One positive-length coincident interval between facade curves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveCurveOverlap {
    pub(crate) first_range: ParamRange,
    pub(crate) second_range: ParamRange,
    pub(crate) orientation: CurveOverlapOrientation,
}

impl CurveCurveOverlap {
    /// Coincident interval on the first requested curve.
    pub const fn first_range(&self) -> ParamRange {
        self.first_range
    }
    /// Coincident interval on the second requested curve.
    pub const fn second_range(&self) -> ParamRange {
        self.second_range
    }
    /// Parameter direction correspondence.
    pub const fn orientation(&self) -> CurveOverlapOrientation {
        self.orientation
    }
}

/// Proof status over both complete requested parameter intervals.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum IntersectionCompletion {
    /// All obligations over the requested intervals were discharged.
    Complete,
    /// Returned contacts are verified, but exclusion evidence is incomplete.
    Indeterminate {
        /// Stable lower-layer explanation for the missing exclusion proof.
        reason: &'static str,
    },
}

/// Curve/curve intersection evidence tied to exact facade identity.
#[derive(Debug, Clone, PartialEq)]
pub struct CurveCurveIntersections {
    pub(crate) first: CurveId,
    pub(crate) second: CurveId,
    pub(crate) points: Vec<CurveCurvePoint>,
    pub(crate) overlaps: Vec<CurveCurveOverlap>,
    pub(crate) completion: IntersectionCompletion,
    pub(crate) root_certificates: Vec<crate::CurvePairRootCertificate>,
    pub(crate) incomplete_evidence: Vec<crate::IncompleteEvidence>,
}

impl CurveCurveIntersections {
    /// First requested curve identity.
    pub fn first(&self) -> CurveId {
        self.first.clone()
    }
    /// Second requested curve identity.
    pub fn second(&self) -> CurveId {
        self.second.clone()
    }
    /// Isolated contacts in deterministic first-curve parameter order.
    pub fn points(&self) -> &[CurveCurvePoint] {
        &self.points
    }
    /// Coincident intervals in deterministic first-curve parameter order.
    pub fn overlaps(&self) -> &[CurveCurveOverlap] {
        &self.overlaps
    }
    /// Complete-domain proof status.
    pub const fn completion(&self) -> IntersectionCompletion {
        self.completion
    }
    /// Structured lower-layer reasons why complete-domain proof remains unavailable.
    pub fn incomplete_evidence(&self) -> &[crate::IncompleteEvidence] {
        &self.incomplete_evidence
    }
    /// Exact unique-root certificates in deterministic parameter-region order.
    pub fn root_certificates(&self) -> &[crate::CurvePairRootCertificate] {
        &self.root_certificates
    }
    /// True only when both requested intervals were completely covered.
    pub fn is_complete(&self) -> bool {
        matches!(self.completion, IntersectionCompletion::Complete)
    }
    /// True when no contacts or overlaps were discovered.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.overlaps.is_empty()
    }
    /// True only for an empty result backed by complete-domain proof.
    pub fn is_proven_empty(&self) -> bool {
        self.is_complete() && self.is_empty()
    }
}

impl SurfaceEvaluationRequest {
    /// Construct a request using graph evaluation's version-1 defaults for
    /// stages omitted by the session policy. Existing session limits remain
    /// authoritative and are never widened by this constructor.
    pub fn new(surface: SurfaceId, uv: [f64; 2], order: kgraph::SurfaceDerivativeOrder) -> Self {
        Self {
            surface,
            uv,
            order,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Surface identity to evaluate.
    pub fn surface(&self) -> SurfaceId {
        self.surface.clone()
    }

    /// Surface parameter pair.
    pub const fn uv(&self) -> [f64; 2] {
        self.uv
    }

    /// Requested exact derivative order.
    pub const fn order(&self) -> kgraph::SurfaceDerivativeOrder {
        self.order
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Successful bounded evaluation of one exact facade surface identity.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceEvaluation {
    surface: SurfaceId,
    derivatives: kgeom::surface::SurfaceDerivs,
}

impl SurfaceEvaluation {
    /// Exact queried identity, including for procedural offset surfaces.
    pub fn surface(&self) -> SurfaceId {
        self.surface.clone()
    }

    /// Evaluated model-space position.
    pub const fn position(&self) -> Point3 {
        self.derivatives.p
    }

    /// Position and requested partial derivatives.
    pub const fn derivatives(&self) -> kgeom::surface::SurfaceDerivs {
        self.derivatives
    }
}

impl CheckBodyRequest {
    /// Construct a check request using default operation settings.
    pub fn new(body: BodyId, level: CheckLevel) -> Self {
        Self {
            body,
            level,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Body being checked.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Requested checker assurance.
    pub const fn level(&self) -> CheckLevel {
        self.level
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Facade-safe subject attached to a checker finding.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum CheckEntity {
    /// Body identity.
    Body(BodyId),
    /// Region identity.
    Region(RegionId),
    /// Shell identity.
    Shell(ShellId),
    /// Face identity.
    Face(FaceId),
    /// Loop identity.
    Loop(LoopId),
    /// Fin identity.
    Fin(FinId),
    /// Edge identity.
    Edge(EdgeId),
    /// Vertex identity.
    Vertex(VertexId),
    /// Three-dimensional geometry identity.
    Curve(CurveId),
    /// Supporting-surface geometry identity.
    Surface(SurfaceId),
    /// Parameter-space geometry identity.
    Pcurve(PcurveId),
    /// Point value. Stored point handles remain an implementation detail.
    Point(Point3),
}

/// One proven body-check fault with a facade-safe subject.
#[derive(Debug, Clone, PartialEq)]
pub struct CheckFault {
    /// Smallest entity or value carrying the fault.
    pub entity: CheckEntity,
    /// Proven invariant violation.
    pub kind: FaultKind,
}

impl CheckFault {
    /// Smallest facade-safe entity or value carrying the fault.
    pub const fn entity(&self) -> &CheckEntity {
        &self.entity
    }

    /// Proven invariant violation.
    pub const fn kind(&self) -> FaultKind {
        self.kind
    }
}

/// One unresolved Full-check proof obligation.
#[derive(Debug, Clone, PartialEq)]
pub struct CheckGap {
    /// Smallest entity or value carrying the proof obligation.
    pub entity: CheckEntity,
    /// Proof category.
    pub kind: VerificationGapKind,
    /// Structured stop or unsupported cause, when supplied by the checker.
    pub cause: Option<VerificationGapCause>,
}

impl CheckGap {
    /// Smallest facade-safe entity or value carrying the proof obligation.
    pub const fn entity(&self) -> &CheckEntity {
        &self.entity
    }

    /// Proof category.
    pub const fn kind(&self) -> VerificationGapKind {
        self.kind
    }

    /// Structured stop or unsupported cause, when supplied by the checker.
    pub const fn cause(&self) -> Option<VerificationGapCause> {
        self.cause
    }
}

/// Checker report with lower raw entity references adapted to facade identity.
#[derive(Debug, Clone, PartialEq)]
pub struct CheckReport {
    level: CheckLevel,
    faults: Vec<CheckFault>,
    gaps: Vec<CheckGap>,
}

impl CheckReport {
    /// Requested assurance level.
    pub const fn level(&self) -> CheckLevel {
        self.level
    }

    /// Proven invariant violations in deterministic checker order.
    pub fn faults(&self) -> &[CheckFault] {
        &self.faults
    }

    /// Unresolved proof obligations in deterministic checker order.
    pub fn gaps(&self) -> &[CheckGap] {
        &self.gaps
    }

    /// Overall checker result without conflating proof gaps with validity.
    pub fn outcome(&self) -> CheckOutcome {
        if !self.faults.is_empty() {
            CheckOutcome::Invalid
        } else if !self.gaps.is_empty() {
            CheckOutcome::Indeterminate
        } else {
            CheckOutcome::Valid
        }
    }
}

/// One body and its Full checker evidence from an edit commit decision.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyCheckReport {
    body: BodyId,
    report: CheckReport,
}

impl BodyCheckReport {
    /// Part-qualified body checked before commit or rollback.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Exact Full report retained for this body.
    pub const fn report(&self) -> &CheckReport {
        &self.report
    }
}

/// F2 outcome retaining one operation report and a classified facade error.
pub type OperationOutcome<T> = kcore::operation::OperationOutcome<T, Error>;

impl PartEdit<'_> {
    /// Construct and checked-commit one block through a single facade-owned
    /// operation context and scope.
    ///
    /// Context-construction failures are returned before a scope exists.
    /// Once started, success or failure is paired with the exact F2 report.
    pub fn create_block(&mut self, request: BlockRequest) -> Result<OperationOutcome<BodyCreated>> {
        let BlockRequest {
            frame,
            extents,
            settings,
        } = request;
        let context = settings.context(self.policy)?;
        let scope = OperationScope::new(&context);
        let part = self.id.clone();
        let result = ktopo::make::block_with_journal(&mut self.state.store, &frame, extents)
            .map(|creation| {
                let (raw_body, inner) = creation.into_parts();
                BodyCreated {
                    body: BodyId::new(part.clone(), raw_body),
                    journal: ChangeJournal::from_raw(part.clone(), inner),
                }
            })
            .map_err(Error::from);
        Ok(scope.finish_typed(result))
    }

    /// Duplicate one complete body under a rigid placement and checked-commit
    /// it atomically.
    ///
    /// The result owns a disjoint topology and geometry closure and the
    /// journal records `DerivedFrom` lineage for every copied identity.
    /// Wrong-part, stale, and unsupported proof-bearing geometry are rejected
    /// before an operation scope starts. Plane/Plane line and Plane/Sphere
    /// circle descriptors backed by direct fields or safe finite constant-
    /// normal offset chains are admitted because the lower copy transaction
    /// reissues their whole-range certificates. Operation-generated verified
    /// analytic/NURBS and transmitted descriptors are admitted under their
    /// graph-validated direct and bounded offset-source contracts when the
    /// lower layer can rerun their public original-source certifier.
    pub fn copy_body_rigid(
        &mut self,
        request: CopyBodyRequest,
    ) -> Result<OperationOutcome<BodyCreated>> {
        let CopyBodyRequest {
            body,
            placement,
            settings,
        } = request;
        self.as_part().body(body.clone())?;

        for edge in self.state.store.edges_of_body(body.raw())? {
            let Some(curve) = self.state.store.get(edge)?.curve() else {
                continue;
            };
            if !rigid_copy_curve_is_reissuable(&self.state.store, curve)? {
                return Err(Error::UnsupportedBodyCopyGeometry {
                    capability: crate::error::capability::RIGID_COPY_VERIFIED_INTERSECTION,
                });
            }
        }

        let context = settings
            .context(self.policy)?
            .with_family_budget_defaults(EvalBudgetProfile::v1_defaults());
        EvalLimits::from_budget_plan(&context.effective_budget())?;
        let mut scope = OperationScope::new(&context);
        let part = self.id.clone();
        let result = (|| -> core::result::Result<BodyCreated, kcore::error::Error> {
            let mut transaction = self.state.store.transaction()?;
            let copied = transaction.copy_body_rigid(body.raw(), placement)?;
            let raw_journal = transaction.commit_checked_body_in_scope(copied, &mut scope, 0)?;
            Ok(BodyCreated {
                body: BodyId::new(part.clone(), copied),
                journal: ChangeJournal::from_raw(part, raw_journal),
            })
        })()
        .map_err(Error::from);
        Ok(scope.finish_typed(result))
    }

    /// Validate, construct, and checked-commit one polygonal-profile prism.
    ///
    /// Every cap and side use receives an exact line pcurve. Profile holes
    /// become material voids bounded by inward-facing side rings. Input,
    /// topology, geometry, checking, and journal failures are atomic and are
    /// paired with the single facade operation report once the scope starts.
    pub fn extrude_profile(
        &mut self,
        request: ExtrudeProfileRequest,
    ) -> Result<OperationOutcome<BodyCreated>> {
        let ExtrudeProfileRequest {
            frame,
            outer,
            holes,
            height,
            settings,
        } = request;
        let context = settings.context(self.policy)?;
        let scope = OperationScope::new(&context);
        let part = self.id.clone();
        let hole_slices = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
        let result =
            ktopo::profile::PlanarProfile::from_polygon_with_holes(frame, &outer, &hole_slices)
                .and_then(|profile| {
                    ktopo::make::extrude_profile_with_journal(
                        &mut self.state.store,
                        &profile,
                        height,
                    )
                })
                .map(|creation| {
                    let (raw_body, inner) = creation.into_parts();
                    BodyCreated {
                        body: BodyId::new(part.clone(), raw_body),
                        journal: ChangeJournal::from_raw(part, inner),
                    }
                })
                .map_err(Error::from);
        Ok(scope.finish_typed(result))
    }

    /// Validate, construct, and checked-commit one oblique polygonal prism.
    ///
    /// The complete translation is retained by shared sweep edges and exact
    /// side-plane pcurves. Failures are atomic and paired with the single
    /// facade operation report once the scope starts.
    pub fn extrude_profile_along(
        &mut self,
        request: ExtrudeProfileAlongRequest,
    ) -> Result<OperationOutcome<BodyCreated>> {
        let ExtrudeProfileAlongRequest {
            frame,
            outer,
            holes,
            translation,
            settings,
        } = request;
        let context = settings.context(self.policy)?;
        let scope = OperationScope::new(&context);
        let part = self.id.clone();
        let hole_slices = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
        let result =
            ktopo::profile::PlanarProfile::from_polygon_with_holes(frame, &outer, &hole_slices)
                .and_then(|profile| {
                    ktopo::make::extrude_profile_along_with_journal(
                        &mut self.state.store,
                        &profile,
                        translation,
                    )
                })
                .map(|creation| {
                    let (raw_body, inner) = creation.into_parts();
                    BodyCreated {
                        body: BodyId::new(part.clone(), raw_body),
                        journal: ChangeJournal::from_raw(part, inner),
                    }
                })
                .map_err(Error::from);
        Ok(scope.finish_typed(result))
    }
}

fn rigid_copy_curve_is_reissuable(
    store: &ktopo::store::Store,
    curve: ktopo::entity::CurveId,
) -> kcore::error::Result<bool> {
    let descriptor = store.curve(curve)?;
    if !descriptor.is_verified_intersection() {
        return Ok(true);
    }
    let Some(intersection) = descriptor.as_intersection() else {
        return Ok(descriptor.as_verified_nurbs_intersection().is_some()
            || descriptor.as_transmitted_intersection().is_some()
            || descriptor
                .as_transmitted_nurbs_intersection()
                .is_some_and(|intersection| {
                    kgraph::transmitted_nurbs_intersection_has_rigid_copy_recertifier(
                        intersection.certificate(),
                    )
                }));
    };
    let [first, second] = intersection.source_surfaces();
    let exact_field = |surface| {
        let mut evaluator = store.eval_context(EvalLimits::default(), Tolerances::default());
        evaluator.surface_exact_field(surface).ok().flatten()
    };
    let fields = [exact_field(first), exact_field(second)];
    Ok(match intersection.certificate() {
        kgraph::VerifiedIntersectionCertificate::PlaneLine(_) => fields
            .into_iter()
            .all(|field| matches!(field, Some(kgraph::ExactSurfaceField::Plane(_)))),
        kgraph::VerifiedIntersectionCertificate::PlaneSphereCircle(_) => {
            matches!(
                fields,
                [
                    Some(kgraph::ExactSurfaceField::Plane(_)),
                    Some(kgraph::ExactSurfaceField::Sphere(_))
                ] | [
                    Some(kgraph::ExactSurfaceField::Sphere(_)),
                    Some(kgraph::ExactSurfaceField::Plane(_))
                ]
            )
        }
    })
}

impl Part<'_> {
    /// Check one body through a single facade-owned operation context and
    /// scope. Full-check proof work borrows that scope directly.
    ///
    /// Wrong-part/stale identity and invalid or incomplete policy
    /// configuration are rejected before the scope starts. Once started,
    /// checker results and failures retain the exact F2 operation report.
    pub fn check_body(&self, request: CheckBodyRequest) -> Result<OperationOutcome<CheckReport>> {
        let CheckBodyRequest {
            body,
            level,
            settings,
        } = request;
        self.body(body.clone())?;
        let context = settings
            .context(self.policy)?
            .with_family_budget_defaults(ktopo::check::CheckBudgetProfile::v1_defaults(level));
        {
            let effective = context.effective_budget();
            for required in ktopo::check::CheckBudgetProfile::v1_defaults(level).limits() {
                effective.require_limit(required.stage, required.resource, required.mode)?;
            }
        }
        let mut scope = OperationScope::new(&context);
        let lower = ktopo::check::check_body_report_in_scope(
            &self.state.store,
            body.raw(),
            level,
            &mut scope,
        );
        let result = match lower {
            Ok(report) => adapt_check_report(&self.id, &self.state.store, report),
            Err(source) => Err(Error::from(source)),
        };
        Ok(scope.finish_typed(result))
    }

    /// Evaluate one surface through a facade-owned operation scope and one
    /// deterministically reserved graph child ledger.
    ///
    /// Wrong-part and stale identities, invalid settings, and incompatible
    /// graph budget modes are rejected before the operation scope starts.
    /// Graph evaluation's v1 profile supplies only budget stages omitted by
    /// the session; session limits and explicit request overrides retain F2
    /// precedence.
    /// Once started, accepted graph work and any typed limit crossing are
    /// merged into the exact returned F2 report.
    pub fn evaluate_surface(
        &self,
        request: SurfaceEvaluationRequest,
    ) -> Result<OperationOutcome<SurfaceEvaluation>> {
        let SurfaceEvaluationRequest {
            surface,
            uv,
            order,
            settings,
        } = request;
        self.surface(surface.clone())?;

        let context = settings
            .context(self.policy)?
            .with_family_budget_defaults(EvalBudgetProfile::v1_defaults());
        let effective = context.effective_budget();
        let limits = EvalLimits::from_budget_plan(&effective)?;

        let mut child_plan = limits.budget_plan();
        if let Some(total_work) = effective.total_work_limit() {
            let node_visits = u64::try_from(limits.max_node_visits_per_query).map_err(|_| {
                OperationPolicyError::AccountingOverflow {
                    stage: kgraph::eval_stage::NODE_VISITS,
                    resource: crate::ResourceKind::Work,
                }
            })?;
            child_plan = child_plan.with_total_work_limit(total_work.min(node_visits));
        }

        // EvalContext can stop only on graph-owned recursion limits. Clamp
        // its visit allowance to the reserved child root ceiling so a strict
        // TOTAL_WORK limit stops before accepting excess graph work. The
        // child ledger still owns canonical precedence and translates that
        // synthetic visit stop back to TOTAL_WORK during reconciliation.
        let execution_node_visits = match child_plan.total_work_limit() {
            Some(allowed) => {
                usize::try_from(allowed).map_err(|_| OperationPolicyError::AccountingOverflow {
                    stage: kgraph::eval_stage::NODE_VISITS,
                    resource: crate::ResourceKind::Work,
                })?
            }
            None => limits.max_node_visits_per_query,
        };
        let execution_limits = EvalLimits {
            max_dependency_depth: limits.max_dependency_depth,
            max_node_visits_per_query: execution_node_visits,
        };

        let mut scope = OperationScope::new(&context);
        let mut child = scope.ledger_mut().reserve_child(0, child_plan)?;
        let mut evaluator = EvalContext::new(
            self.state.store.geometry(),
            execution_limits,
            context.tolerances(),
        );
        let lower = evaluator.eval_surface(surface.raw(), uv, order);
        let usage = evaluator.last_query_usage();
        let accounting = account_graph_query(&mut child, usage, lower.as_ref().err());
        let merge = scope.ledger_mut().merge_children(vec![child]);
        let result = match accounting.and(merge) {
            Ok(()) => lower
                .map(|derivatives| SurfaceEvaluation {
                    surface,
                    derivatives,
                })
                .map_err(Error::from_graph),
            Err(source) => Err(Error::from(source)),
        };
        Ok(scope.finish_typed(result))
    }
}

fn account_graph_query(
    child: &mut ChildWorkLedger,
    usage: EvalUsage,
    failure: Option<&kgraph::EvalError>,
) -> core::result::Result<(), OperationPolicyError> {
    let visits = u64::try_from(usage.node_visits()).map_err(|_| {
        OperationPolicyError::AccountingOverflow {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: crate::ResourceKind::Work,
        }
    })?;
    let depth = u64::try_from(usage.dependency_depth()).map_err(|_| {
        OperationPolicyError::AccountingOverflow {
            stage: kgraph::eval_stage::DEPENDENCY_DEPTH,
            resource: crate::ResourceKind::Depth,
        }
    })?;
    child
        .ledger_mut()
        .charge(kgraph::eval_stage::NODE_VISITS, visits)?;
    child.ledger_mut().observe(
        kgraph::eval_stage::DEPENDENCY_DEPTH,
        crate::ResourceKind::Depth,
        depth,
    )?;

    let Some(snapshot) = failure.and_then(kgraph::EvalError::limit) else {
        return Ok(());
    };
    let crossing = match snapshot.resource {
        crate::ResourceKind::Work => child.ledger_mut().charge_resource(
            snapshot.stage,
            snapshot.resource,
            snapshot.consumed.saturating_sub(visits),
        ),
        crate::ResourceKind::Depth => {
            child
                .ledger_mut()
                .observe(snapshot.stage, snapshot.resource, snapshot.consumed)
        }
        _ => {
            return Err(OperationPolicyError::UnknownLimit {
                stage: snapshot.stage,
                resource: snapshot.resource,
            });
        }
    };
    match crossing {
        Err(OperationPolicyError::LimitReached(actual)) if actual == snapshot => Ok(()),
        Err(other) => Err(other),
        Ok(()) => Err(OperationPolicyError::UnknownLimit {
            stage: snapshot.stage,
            resource: snapshot.resource,
        }),
    }
}

fn adapt_check_report(
    part: &PartId,
    store: &ktopo::store::Store,
    report: ktopo::check::CheckReport,
) -> Result<CheckReport> {
    adapt_check_report_with_points(part, report, |point| {
        store
            .get(point)
            .copied()
            .map_err(|source| Error::InconsistentTopology { source })
    })
}

pub(crate) fn adapt_transaction_check(
    part: &PartId,
    check: &ktopo::transaction::FullBodyCheck,
) -> BodyCheckReport {
    let report = adapt_check_report_with_points(part, check.report().clone(), |point| {
        Ok(check
            .point_value(point)
            .expect("FullBodyCheck snapshots every point-valued report subject"))
    })
    .expect("transaction check adaptation is total over captured evidence");
    BodyCheckReport {
        body: BodyId::new(part.clone(), check.body()),
        report,
    }
}

fn adapt_check_report_with_points(
    part: &PartId,
    report: ktopo::check::CheckReport,
    mut point_value: impl FnMut(ktopo::entity::PointId) -> Result<Point3>,
) -> Result<CheckReport> {
    let faults = report
        .faults
        .into_iter()
        .map(|fault| {
            Ok(CheckFault {
                entity: adapt_check_entity(part, fault.entity, &mut point_value)?,
                kind: fault.kind,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let gaps = report
        .gaps
        .into_iter()
        .map(|gap| {
            Ok(CheckGap {
                entity: adapt_check_entity(part, gap.entity, &mut point_value)?,
                kind: gap.kind,
                cause: gap.cause,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    Ok(CheckReport {
        level: report.level,
        faults,
        gaps,
    })
}

fn adapt_check_entity(
    part: &PartId,
    entity: RawEntityRef,
    point_value: &mut impl FnMut(ktopo::entity::PointId) -> Result<Point3>,
) -> Result<CheckEntity> {
    let part = part.clone();
    Ok(match entity {
        RawEntityRef::Body(raw) => CheckEntity::Body(BodyId::new(part, raw)),
        RawEntityRef::Region(raw) => CheckEntity::Region(RegionId::new(part, raw)),
        RawEntityRef::Shell(raw) => CheckEntity::Shell(ShellId::new(part, raw)),
        RawEntityRef::Face(raw) => CheckEntity::Face(FaceId::new(part, raw)),
        RawEntityRef::Loop(raw) => CheckEntity::Loop(LoopId::new(part, raw)),
        RawEntityRef::Fin(raw) => CheckEntity::Fin(FinId::new(part, raw)),
        RawEntityRef::Edge(raw) => CheckEntity::Edge(EdgeId::new(part, raw)),
        RawEntityRef::Vertex(raw) => CheckEntity::Vertex(VertexId::new(part, raw)),
        RawEntityRef::Curve(raw) => CheckEntity::Curve(CurveId::new(part, raw)),
        RawEntityRef::Surface(raw) => CheckEntity::Surface(SurfaceId::new(part, raw)),
        RawEntityRef::Curve2d(raw) => CheckEntity::Pcurve(PcurveId::new(part, raw)),
        RawEntityRef::Point(raw) => CheckEntity::Point(point_value(raw)?),
    })
}

#[cfg(test)]
mod tests {
    use std::error::Error as _;

    use kcore::error::ErrorClass;
    use kcore::operation::{
        AccountingMode, ExecutionPolicy, LimitSpec, NumericalPolicy, PolicyVersion, ResourceKind,
        SessionPrecision, TOTAL_WORK_STAGE,
    };
    use kgeom::curve::{Circle, Line};
    use kgeom::curve2d::{Circle2d, Line2d, NurbsCurve2d};
    use kgeom::nurbs::{NurbsCurve, NurbsSurface};
    use kgeom::param::ParamRange;
    use kgeom::surface::{Plane, Sphere};
    use kgeom::vec::{Point2, Point3, Vec2, Vec3};
    use kgraph::{
        AffineParamMap1d, EvalBudgetProfile, EvalError, NurbsIntersectionTrace,
        OffsetSurfaceDescriptor, PlaneCircleTrace, PlaneSphereCircleTrace, SphereLatitudeTrace,
        TransmittedIntersectionChartMetadata, certify_paired_plane_line_residuals,
        certify_paired_plane_sphere_circle_residuals,
        certify_transmitted_plane_intersection_residuals,
        certify_verified_plane_nurbs_intersection_residuals,
    };
    use ktopo::check::VerificationGapCause;
    use ktopo::entity::{
        Body as RawBody, BodyKind, Edge as RawEdge, Face as RawFace, Region as RawRegion,
        RegionKind, Shell as RawShell, Vertex as RawVertex,
    };
    use ktopo::geom::{Curve2dGeom, SurfaceGeom};
    use ktopo::store::Store;

    use super::*;
    use crate::{GeometryEvaluationError, Kernel, KernelError};

    fn full_check_policy() -> SessionPolicy {
        SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            FullCheckBudgetProfile::v1_defaults(),
            PolicyVersion::V1,
        )
    }

    fn transmitted_metadata() -> TransmittedIntersectionChartMetadata {
        TransmittedIntersectionChartMetadata::new(0.0, 1.0, 0.0, 0.0, [None, None]).unwrap()
    }

    fn transmitted_plane_curve(store: &mut Store) -> ktopo::entity::CurveId {
        let plane = Plane::new(Frame::world());
        let surfaces = [
            store.insert_surface(SurfaceGeom::Plane(plane)).unwrap(),
            store.insert_surface(SurfaceGeom::Plane(plane)).unwrap(),
        ];
        let knots = vec![0.0, 0.0, 1.0, 1.0];
        let carrier = NurbsCurve::new(
            1,
            knots.clone(),
            vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
            None,
        )
        .unwrap();
        let pcurves = [
            NurbsCurve2d::new(
                1,
                knots.clone(),
                vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
                None,
            )
            .unwrap(),
            NurbsCurve2d::new(
                1,
                knots,
                vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
                None,
            )
            .unwrap(),
        ];
        let certificate = certify_transmitted_plane_intersection_residuals(
            carrier,
            [plane; 2],
            pcurves.clone(),
            transmitted_metadata(),
            1.0e-12,
        )
        .unwrap();
        let pcurves =
            pcurves.map(|pcurve| store.insert_pcurve(Curve2dGeom::Nurbs(pcurve)).unwrap());
        store
            .insert_verified_transmitted_plane_intersection_curve(surfaces, pcurves, certificate)
            .unwrap()
    }

    fn transmitted_plane_wire(store: &mut Store) -> ktopo::entity::BodyId {
        let curve = transmitted_plane_curve(store);
        let mut transaction = store.transaction().unwrap();
        let body = {
            let mut assembly = transaction.assembly();
            let body = assembly.add(RawBody {
                kind: BodyKind::Wire,
                regions: Vec::new(),
            });
            let region = assembly.add(RawRegion {
                body,
                kind: RegionKind::Void,
                shells: Vec::new(),
            });
            let shell = assembly.add(RawShell {
                region,
                faces: Vec::new(),
                edges: Vec::new(),
                vertex: None,
            });
            let points = [
                assembly.add(Point3::new(0.0, 0.0, 0.0)),
                assembly.add(Point3::new(1.0, 0.0, 0.0)),
            ];
            let vertices = points.map(|point| {
                assembly.add(RawVertex {
                    point,
                    tolerance: None,
                })
            });
            let edge = assembly.add(RawEdge {
                curve: Some(curve),
                vertices: vertices.map(Some),
                bounds: Some((0.0, 1.0)),
                fins: Vec::new(),
                tolerance: None,
            });
            assembly.get_mut(shell).unwrap().edges.push(edge);
            assembly.get_mut(region).unwrap().shells.push(shell);
            assembly.get_mut(body).unwrap().regions.push(region);
            body
        };
        transaction.commit_checked_body(body).unwrap();
        body
    }

    fn verified_nurbs_wire(store: &mut Store) -> ktopo::entity::BodyId {
        let plane = Plane::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        );
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 1.0),
            ],
            Some(vec![2.0; 4]),
        )
        .unwrap();
        let surfaces = [
            store.insert_surface(SurfaceGeom::Plane(plane)).unwrap(),
            store
                .insert_surface(SurfaceGeom::Nurbs(surface.clone()))
                .unwrap(),
        ];
        let knots = vec![0.0, 0.0, 1.0, 1.0];
        let carrier = NurbsCurve::new(
            1,
            knots.clone(),
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            None,
        )
        .unwrap();
        let pcurve = NurbsCurve2d::new(
            1,
            knots,
            vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
            None,
        )
        .unwrap();
        let certificate = certify_verified_plane_nurbs_intersection_residuals(
            carrier,
            [
                NurbsIntersectionTrace::Plane(plane),
                NurbsIntersectionTrace::Nurbs(surface),
            ],
            [pcurve.clone(), pcurve.clone()],
            1.0e-10,
        )
        .unwrap();
        let pcurves = [pcurve.clone(), pcurve]
            .map(|pcurve| store.insert_pcurve(Curve2dGeom::Nurbs(pcurve)).unwrap());
        let curve = store
            .insert_verified_nurbs_intersection_curve(surfaces, pcurves, certificate)
            .unwrap();

        let mut transaction = store.transaction().unwrap();
        let body = {
            let mut assembly = transaction.assembly();
            let body = assembly.add(RawBody {
                kind: BodyKind::Wire,
                regions: Vec::new(),
            });
            let region = assembly.add(RawRegion {
                body,
                kind: RegionKind::Void,
                shells: Vec::new(),
            });
            let shell = assembly.add(RawShell {
                region,
                faces: Vec::new(),
                edges: Vec::new(),
                vertex: None,
            });
            let points = [
                assembly.add(Point3::new(0.0, 0.0, 0.0)),
                assembly.add(Point3::new(1.0, 0.0, 0.0)),
            ];
            let vertices = points.map(|point| {
                assembly.add(RawVertex {
                    point,
                    tolerance: None,
                })
            });
            let edge = assembly.add(RawEdge {
                curve: Some(curve),
                vertices: vertices.map(Some),
                bounds: Some((0.0, 1.0)),
                fins: Vec::new(),
                tolerance: None,
            });
            assembly.get_mut(shell).unwrap().edges.push(edge);
            assembly.get_mut(region).unwrap().shells.push(shell);
            assembly.get_mut(body).unwrap().regions.push(region);
            body
        };
        transaction.commit_checked_body(body).unwrap();
        body
    }

    fn add_surface_chain(
        session: &mut crate::Session,
        part: &PartId,
        offsets: &[f64],
    ) -> Vec<SurfaceId> {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let store = edit.store_mut_for_test();
        let basis = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        let mut handles = vec![basis];
        for &distance in offsets {
            let next = store
                .insert_surface(
                    OffsetSurfaceDescriptor::new(*handles.last().unwrap(), distance).into(),
                )
                .unwrap();
            handles.push(next);
        }
        handles
            .into_iter()
            .map(|raw| SurfaceId::new(part.clone(), raw))
            .collect()
    }

    fn graph_settings(depth: usize, visits: usize) -> OperationSettings {
        OperationSettings::new().with_budget_overrides(EvalBudgetProfile::for_limits(depth, visits))
    }

    fn report_usage(
        report: &kcore::operation::OperationReport,
        stage: crate::StageId,
        resource: ResourceKind,
    ) -> kcore::operation::LimitSnapshot {
        *report
            .usage()
            .iter()
            .find(|snapshot| snapshot.stage == stage && snapshot.resource == resource)
            .unwrap()
    }

    #[test]
    fn block_and_fast_check_match_direct_topology_journal_and_reports() {
        let mut direct_store = Store::new();
        let direct =
            ktopo::make::block_with_journal(&mut direct_store, &Frame::world(), [2.0, 3.0, 4.0])
                .unwrap();
        let direct_check =
            ktopo::check::check_body_report(&direct_store, direct.body(), CheckLevel::Fast)
                .unwrap();

        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let facade = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))
            .unwrap();
        assert!(facade.report().usage().is_empty());
        assert!(facade.report().limit_events().is_empty());
        let created = facade.into_result().unwrap();
        assert_eq!(created.body.raw(), direct.body());
        assert_eq!(created.journal.inner, *direct.journal());
        assert_eq!(created.journal.part(), part_id);

        let part = session.part(part_id.clone()).unwrap();
        assert_eq!(part.bodies().len(), direct_store.count::<RawBody>());
        assert_eq!(part.faces().len(), direct_store.count::<RawFace>());
        assert_eq!(part.edges().len(), direct_store.count::<RawEdge>());
        assert_eq!(part.vertices().len(), direct_store.count::<RawVertex>());
        let facade_check = part
            .check_body(CheckBodyRequest::new(created.body(), CheckLevel::Fast))
            .unwrap();
        assert_eq!(
            report_usage(
                facade_check.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            )
            .consumed,
            306
        );
        assert_eq!(
            report_usage(
                facade_check.report(),
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
            )
            .consumed,
            1
        );
        assert!(facade_check.report().limit_events().is_empty());
        let expected = adapt_check_report(&part_id, &direct_store, direct_check).unwrap();
        assert_eq!(facade_check.result(), Ok(&expected));
    }

    #[test]
    fn polygonal_profile_extrusion_is_full_valid_and_failure_atomic() {
        let outer = vec![
            Point2::new(-2.0, -2.0),
            Point2::new(2.0, -2.0),
            Point2::new(2.0, 2.0),
            Point2::new(-2.0, 2.0),
        ];
        let hole = vec![
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
            Point2::new(-1.0, 1.0),
        ];
        let request =
            ExtrudeProfileRequest::new(Frame::world(), outer.clone(), vec![hole.clone()], 2.0);
        assert_eq!(request.frame(), Frame::world());
        assert_eq!(request.outer(), outer);
        assert_eq!(request.holes(), &[hole]);
        assert_eq!(request.height(), 2.0);

        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let rejected = session
            .edit_part(part_id.clone())
            .unwrap()
            .extrude_profile(ExtrudeProfileRequest::new(
                Frame::world(),
                outer.clone(),
                Vec::new(),
                -1.0,
            ))
            .unwrap();
        assert!(matches!(
            rejected.into_result(),
            Err(KernelError::Core {
                source: kcore::error::Error::InvalidGeometry { .. }
            })
        ));
        assert_eq!(session.part(part_id.clone()).unwrap().bodies().len(), 0);

        let outcome = session
            .edit_part(part_id.clone())
            .unwrap()
            .extrude_profile(request)
            .unwrap();
        assert!(outcome.report().usage().is_empty());
        let created = outcome.into_result().unwrap();
        assert!(
            created
                .journal()
                .mutations()
                .all(|mutation| mutation.kind() == MutationKind::Created)
        );
        assert_eq!(created.journal().lineage_count(), 0);

        let part = session.part(part_id).unwrap();
        assert_eq!(part.bodies().len(), 1);
        assert_eq!(part.faces().len(), 10);
        assert_eq!(part.loops().len(), 12);
        assert_eq!(part.edges().len(), 24);
        assert_eq!(part.vertices().len(), 16);
        let full = part
            .check_body(CheckBodyRequest::new(created.body(), CheckLevel::Full))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Valid);
    }

    #[test]
    fn oblique_profile_extrusion_is_full_valid_and_failure_atomic() {
        let outer = vec![
            Point2::new(-2.0, -1.0),
            Point2::new(2.0, -1.0),
            Point2::new(2.0, 3.0),
            Point2::new(-2.0, 3.0),
        ];
        let hole = vec![
            Point2::new(-1.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 2.0),
            Point2::new(-1.0, 2.0),
        ];
        let translation = Vec3::new(0.75, -0.5, -2.0);
        let request = ExtrudeProfileAlongRequest::new(
            Frame::world(),
            outer.clone(),
            vec![hole.clone()],
            translation,
        );
        assert_eq!(request.frame(), Frame::world());
        assert_eq!(request.outer(), outer);
        assert_eq!(request.holes(), &[hole]);
        assert_eq!(request.translation(), translation);

        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let rejected = session
            .edit_part(part_id.clone())
            .unwrap()
            .extrude_profile_along(ExtrudeProfileAlongRequest::new(
                Frame::world(),
                outer,
                Vec::new(),
                Vec3::new(1.0, 0.0, 0.0),
            ))
            .unwrap();
        assert!(matches!(
            rejected.into_result(),
            Err(KernelError::Core {
                source: kcore::error::Error::InvalidGeometry { .. }
            })
        ));
        assert_eq!(session.part(part_id.clone()).unwrap().bodies().len(), 0);

        let created = session
            .edit_part(part_id.clone())
            .unwrap()
            .extrude_profile_along(request)
            .unwrap()
            .into_result()
            .unwrap();
        assert!(
            created
                .journal()
                .mutations()
                .all(|mutation| mutation.kind() == MutationKind::Created)
        );
        let full = session
            .part(part_id)
            .unwrap()
            .check_body(CheckBodyRequest::new(created.body(), CheckLevel::Full))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Valid);
    }

    #[test]
    fn rigid_body_copy_is_disjoint_checked_and_fully_lineaged() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let source = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let before = {
            let part = session.part(part_id.clone()).unwrap();
            (
                part.bodies().len(),
                part.faces().len(),
                part.edges().len(),
                part.vertices().len(),
                part.curves().len(),
                part.surfaces().len(),
                part.pcurves().len(),
            )
        };
        let placement = Frame::new(
            Point3::new(4.0, -3.0, 2.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let outcome = session
            .edit_part(part_id.clone())
            .unwrap()
            .copy_body_rigid(CopyBodyRequest::new(source.clone(), placement))
            .unwrap();
        let created = outcome.into_result().unwrap();
        assert_ne!(created.body(), source);
        assert!(
            created
                .journal()
                .mutations()
                .all(|mutation| mutation.kind() == MutationKind::Created)
        );
        assert_eq!(
            created.journal().lineage_count(),
            created.journal().mutation_count()
        );
        assert!(created.journal().lineage().any(|lineage| matches!(
            lineage,
            LineageView::DerivedFrom {
                derived: JournalEntity::Body(derived),
                source: JournalEntity::Body(original),
            } if derived == created.body() && original == source
        )));

        let part = session.part(part_id.clone()).unwrap();
        assert_eq!(
            (
                part.bodies().len(),
                part.faces().len(),
                part.edges().len(),
                part.vertices().len(),
                part.curves().len(),
                part.surfaces().len(),
                part.pcurves().len(),
            ),
            (
                before.0 * 2,
                before.1 * 2,
                before.2 * 2,
                before.3 * 2,
                before.4 * 2,
                before.5 * 2,
                before.6 * 2,
            )
        );
        let full = part
            .check_body(CheckBodyRequest::new(created.body(), CheckLevel::Full))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Valid);
    }

    #[test]
    fn rigid_copy_preflight_admits_safe_exact_offset_field_proof_families() {
        let mut store = Store::new();
        let planes = [
            Plane::new(Frame::world()),
            Plane::new(
                Frame::new(
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            ),
        ];
        let line_pcurves = [
            Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
            Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
        ];
        let plane_line_certificate = certify_paired_plane_line_residuals(
            Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
            ParamRange::new(-1.0, 1.0),
            planes,
            line_pcurves,
            [AffineParamMap1d::new(1.0, 0.0).unwrap(); 2],
            1.0e-12,
        )
        .unwrap();
        let plane_handles =
            planes.map(|plane| store.insert_surface(SurfaceGeom::Plane(plane)).unwrap());
        let line_pcurve_handles =
            line_pcurves.map(|pcurve| store.insert_pcurve(Curve2dGeom::Line(pcurve)).unwrap());
        let plane_line = store
            .insert_verified_plane_intersection_curve(
                plane_handles,
                line_pcurve_handles,
                plane_line_certificate,
            )
            .unwrap();
        assert!(rigid_copy_curve_is_reissuable(&store, plane_line).unwrap());

        let offset_plane = store
            .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                plane_handles[0],
                0.0,
            )))
            .unwrap();
        let offset_plane_line = store
            .insert_verified_plane_intersection_curve(
                [offset_plane, plane_handles[1]],
                line_pcurve_handles,
                plane_line_certificate,
            )
            .unwrap();
        assert!(rigid_copy_curve_is_reissuable(&store, offset_plane_line).unwrap());

        let mut overdeep_plane = plane_handles[0];
        for _ in 0..EvalLimits::default().max_dependency_depth {
            overdeep_plane = store
                .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                    overdeep_plane,
                    0.0,
                )))
                .unwrap();
        }
        let overdeep_plane_line = store
            .insert_verified_plane_intersection_curve(
                [overdeep_plane, plane_handles[1]],
                line_pcurve_handles,
                plane_line_certificate,
            )
            .unwrap();
        assert!(!rigid_copy_curve_is_reissuable(&store, overdeep_plane_line).unwrap());

        let height = 0.5;
        let plane = Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, height)));
        let sphere = Sphere::new(Frame::world(), 2.0).unwrap();
        let radius = (sphere.radius() * sphere.radius() - height * height).sqrt();
        let carrier = Circle::new(
            Frame::world().with_origin(Point3::new(0.0, 0.0, height)),
            radius,
        )
        .unwrap();
        let plane_pcurve =
            Circle2d::new(Point2::new(0.0, 0.0), radius, Vec2::new(1.0, 0.0)).unwrap();
        let sphere_pcurve = Line2d::new(
            Point2::new(0.0, kcore::math::atan2(height, radius)),
            Vec2::new(1.0, 0.0),
        )
        .unwrap();
        let identity = AffineParamMap1d::new(1.0, 0.0).unwrap();
        let plane_sphere_certificate = certify_paired_plane_sphere_circle_residuals(
            carrier,
            ParamRange::new(0.25, 4.75),
            [
                PlaneSphereCircleTrace::Plane(PlaneCircleTrace::new(plane, plane_pcurve, identity)),
                PlaneSphereCircleTrace::Sphere(SphereLatitudeTrace::new(
                    sphere,
                    sphere_pcurve,
                    identity,
                )),
            ],
            1.0e-10,
        )
        .unwrap();
        let plane = store.insert_surface(SurfaceGeom::Plane(plane)).unwrap();
        let sphere = store.insert_surface(SurfaceGeom::Sphere(sphere)).unwrap();
        let plane_pcurve = store
            .insert_pcurve(Curve2dGeom::Circle(plane_pcurve))
            .unwrap();
        let sphere_pcurve = store
            .insert_pcurve(Curve2dGeom::Line(sphere_pcurve))
            .unwrap();
        let plane_sphere = store
            .insert_verified_plane_sphere_intersection_curve(
                [plane, sphere],
                [plane_pcurve, sphere_pcurve],
                plane_sphere_certificate,
            )
            .unwrap();
        assert!(rigid_copy_curve_is_reissuable(&store, plane_sphere).unwrap());

        let offset_plane = store
            .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                plane, 0.0,
            )))
            .unwrap();
        let sphere_basis = store
            .insert_surface(SurfaceGeom::Sphere(
                Sphere::new(Frame::world(), 1.5).unwrap(),
            ))
            .unwrap();
        let offset_sphere = store
            .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                sphere_basis,
                0.5,
            )))
            .unwrap();
        let offset_plane_sphere = store
            .insert_verified_plane_sphere_intersection_curve(
                [offset_plane, offset_sphere],
                [plane_pcurve, sphere_pcurve],
                plane_sphere_certificate,
            )
            .unwrap();
        assert!(rigid_copy_curve_is_reissuable(&store, offset_plane_sphere).unwrap());

        let collapsed_sphere_basis = store
            .insert_surface(SurfaceGeom::Sphere(
                Sphere::new(Frame::world(), 0.5).unwrap(),
            ))
            .unwrap();
        let collapsed_sphere = store
            .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                collapsed_sphere_basis,
                -0.5,
            )))
            .unwrap();
        assert!(
            store
                .insert_verified_plane_sphere_intersection_curve(
                    [plane, collapsed_sphere],
                    [plane_pcurve, sphere_pcurve],
                    plane_sphere_certificate,
                )
                .is_err(),
            "the graph must reject a collapsed effective sphere before copy preflight"
        );

        let transmitted = transmitted_plane_curve(&mut store);
        assert!(rigid_copy_curve_is_reissuable(&store, transmitted).unwrap());
    }

    #[test]
    fn rigid_body_copy_facade_admits_and_reissues_transmitted_plane_proofs() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let source = BodyId::new(part_id, transmitted_plane_wire(&mut edit.state.store));
        let placement = Frame::new(
            Point3::new(2.0, -1.0, 3.0),
            Vec3::new(1.0, 2.0, 3.0).normalized().unwrap(),
            Vec3::new(2.0, -1.0, 0.0).normalized().unwrap(),
        )
        .unwrap();
        let created = edit
            .copy_body_rigid(CopyBodyRequest::new(source, placement))
            .unwrap()
            .into_result()
            .unwrap();
        let copied_edge = edit
            .state
            .store
            .edges_of_body(created.body().raw())
            .unwrap()[0];
        let copied_curve = edit.state.store.get(copied_edge).unwrap().curve.unwrap();
        let copied = edit
            .state
            .store
            .get(copied_curve)
            .unwrap()
            .as_transmitted_intersection()
            .unwrap();
        assert_eq!(
            copied.certificate().carrier().points(),
            &[
                placement.point_at(0.0, 0.0, 0.0),
                placement.point_at(1.0, 0.0, 0.0),
            ]
        );
        assert_eq!(copied.certificate().metadata(), transmitted_metadata());
    }

    #[test]
    fn rigid_body_copy_facade_admits_and_reissues_verified_nurbs_proofs() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let raw_source = verified_nurbs_wire(&mut edit.state.store);
        let source = BodyId::new(part_id, raw_source);
        let placement = Frame::new(
            Point3::new(2.0, -1.0, 3.0),
            Vec3::new(1.0, 2.0, 3.0).normalized().unwrap(),
            Vec3::new(2.0, -1.0, 0.0).normalized().unwrap(),
        )
        .unwrap();

        let created = edit
            .copy_body_rigid(CopyBodyRequest::new(source, placement))
            .unwrap()
            .into_result()
            .unwrap();
        let copied_edge = edit
            .state
            .store
            .edges_of_body(created.body().raw())
            .unwrap()[0];
        let copied_curve = edit.state.store.get(copied_edge).unwrap().curve.unwrap();
        let copied = edit
            .state
            .store
            .get(copied_curve)
            .unwrap()
            .as_verified_nurbs_intersection()
            .unwrap();
        assert_eq!(
            copied.certificate().carrier_range(),
            ParamRange::new(0.0, 1.0)
        );
        assert_eq!(
            copied.certificate().carrier().points(),
            &[
                placement.point_at(0.0, 0.0, 0.0),
                placement.point_at(1.0, 0.0, 0.0),
            ]
        );
        edit.state.store.geometry().validate().unwrap();
    }

    #[test]
    fn rigid_body_copy_rejects_wrong_part_before_starting_an_operation() {
        let mut session = Kernel::new().create_session();
        let source_part = session.create_part();
        let receiving_part = session.create_part();
        let source = session
            .edit_part(source_part)
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let error = session
            .edit_part(receiving_part.clone())
            .unwrap()
            .copy_body_rigid(CopyBodyRequest::new(source.clone(), Frame::world()))
            .unwrap_err();
        assert!(matches!(
            error,
            KernelError::WrongPart { expected, actual }
                if expected == receiving_part && actual == source.part().clone()
        ));
        assert_eq!(session.part(receiving_part).unwrap().bodies().len(), 0);
    }

    #[test]
    fn journal_adapters_preserve_every_lineage_shape_and_ordered_identity() {
        let mut store = Store::new();
        let creation =
            ktopo::make::block_with_journal(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let body = creation.body();
        let face = store.faces_of_body(body).unwrap()[0];
        let edge = store.edges_of_body(body).unwrap()[0];
        let vertex = store.vertices_of_body(body).unwrap()[0];
        let point = store.get(vertex).unwrap().point();

        let mut session = Kernel::new().create_session();
        let part = session.create_part();

        let derived = RawLineageEvent::DerivedFrom {
            derived: RawEntityRef::Edge(edge),
            source: RawEntityRef::Point(point),
        };
        let LineageView::DerivedFrom { derived, source } = adapt_lineage_event(&part, &derived)
        else {
            panic!("expected derived-from lineage");
        };
        assert_eq!(derived.kind(), EntityKind::Edge);
        assert_eq!(source.kind(), EntityKind::Point);

        let split = RawLineageEvent::Split {
            source: RawEntityRef::Face(face),
            pieces: vec![RawEntityRef::Face(face), RawEntityRef::Face(face)],
        };
        let LineageView::Split { source, pieces } = adapt_lineage_event(&part, &split) else {
            panic!("expected split lineage");
        };
        assert_eq!(source.kind(), EntityKind::Face);
        assert_eq!(
            pieces.map(|piece| piece.kind()).collect::<Vec<_>>(),
            vec![EntityKind::Face, EntityKind::Face]
        );

        let merge = RawLineageEvent::Merge {
            sources: vec![RawEntityRef::Face(face), RawEntityRef::Face(face)],
            result: RawEntityRef::Face(face),
        };
        let LineageView::Merge { sources, result } = adapt_lineage_event(&part, &merge) else {
            panic!("expected merge lineage");
        };
        assert_eq!(
            sources.map(|source| source.kind()).collect::<Vec<_>>(),
            vec![EntityKind::Face, EntityKind::Face]
        );
        assert_eq!(result.kind(), EntityKind::Face);

        let replaced = RawLineageEvent::Replaced {
            old: RawEntityRef::Edge(edge),
            new: RawEntityRef::Edge(edge),
        };
        let LineageView::Replaced { old, new } = adapt_lineage_event(&part, &replaced) else {
            panic!("expected replaced lineage");
        };
        assert_eq!(old.kind(), EntityKind::Edge);
        assert_eq!(new.kind(), EntityKind::Edge);

        let deleted = RawLineageEvent::Deleted {
            entity: RawEntityRef::Body(body),
        };
        let LineageView::Deleted { entity } = adapt_lineage_event(&part, &deleted) else {
            panic!("expected deleted lineage");
        };
        assert_eq!(entity.kind(), EntityKind::Body);
        assert_eq!(entity.part(), part);
    }

    #[test]
    fn deleted_lineage_identity_remains_reportable_but_stale_in_the_part() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let journal = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let mut transaction = edit.store_mut_for_test().transaction().unwrap();
            let (surface, point) = {
                let mut assembly = transaction.assembly();
                (
                    assembly
                        .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
                        .unwrap(),
                    assembly.add(Point3::new(0.0, 0.0, 0.0)),
                )
            };
            let transient = transaction
                .make_minimal_body(surface, crate::Sense::Forward, point)
                .unwrap();
            transaction.kill_minimal_body(transient.body).unwrap();
            ChangeJournal::from_raw(part_id.clone(), transaction.commit_checked(&[]).unwrap())
        };

        let mut lineage = journal.lineage();
        let LineageView::DerivedFrom { derived, source } = lineage.next().unwrap() else {
            panic!("expected vertex derivation");
        };
        assert_eq!(derived.kind(), EntityKind::Vertex);
        assert_eq!(source.kind(), EntityKind::Point);
        let LineageView::Deleted { entity } = lineage.next().unwrap() else {
            panic!("expected body deletion");
        };
        assert_eq!(lineage.len(), 0);
        let JournalEntity::Body(deleted_body) = entity else {
            panic!("expected deleted body identity");
        };
        assert!(matches!(
            session.part(part_id).unwrap().body(deleted_body),
            Err(crate::Error::StaleEntity {
                kind: EntityKind::Body
            })
        ));
    }

    #[test]
    fn journal_adapters_keep_tolerance_budgets_distinct_from_operation_work() {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        let edge = store.edges_of_body(body).unwrap()[0];
        let vertex = store.vertices_of_body(body).unwrap()[0];
        let requested = 3.0 * kcore::tolerance::LINEAR_RESOLUTION;
        let growth = requested - kcore::tolerance::LINEAR_RESOLUTION;

        let mut transaction = store.transaction().unwrap();
        let budget = transaction
            .declare_tolerance_budget("facade-journal-test", 3.0 * growth)
            .unwrap();
        transaction
            .grow_face_tolerance(budget, face, requested)
            .unwrap();
        transaction
            .grow_edge_tolerance(budget, edge, requested)
            .unwrap();
        transaction
            .grow_vertex_tolerance(budget, vertex, requested)
            .unwrap();
        let raw = transaction.commit_checked_body(body).unwrap();

        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let journal = ChangeJournal::from_raw(part.clone(), raw);
        let budgets = journal.tolerance_budgets().collect::<Vec<_>>();
        assert_eq!(budgets.len(), 1);
        assert_eq!(budgets[0].id().index(), 0);
        assert_eq!(budgets[0].operation(), "facade-journal-test");
        assert_eq!(budgets[0].limit(), 3.0 * growth);
        assert_eq!(budgets[0].consumed(), 3.0 * growth);
        assert_eq!(budgets[0].remaining(), 0.0);

        let events = journal.tolerance_events().collect::<Vec<_>>();
        assert_eq!(events.len(), 3);
        assert_eq!(
            events
                .iter()
                .map(|event| event.entity().kind())
                .collect::<Vec<_>>(),
            vec![EntityKind::Face, EntityKind::Edge, EntityKind::Vertex]
        );
        assert!(events.iter().all(|event| {
            event.entity().part() == part
                && event.previous().is_none()
                && event.current().value() == requested
                && event.budget() == budgets[0].id()
        }));
        assert!(
            journal
                .mutations()
                .all(|mutation| mutation.kind() == MutationKind::Modified)
        );
    }

    #[test]
    fn checker_finding_accessors_preserve_semantic_values() {
        let entity = CheckEntity::Point(Point3::new(1.0, 2.0, 3.0));
        let fault = CheckFault {
            entity: entity.clone(),
            kind: FaultKind::OutsideSizeBox,
        };
        assert_eq!(fault.entity(), &entity);
        assert_eq!(fault.kind(), FaultKind::OutsideSizeBox);

        let gap = CheckGap {
            entity: entity.clone(),
            kind: VerificationGapKind::LoopSelfIntersection,
            cause: None,
        };
        assert_eq!(gap.entity(), &entity);
        assert_eq!(gap.kind(), VerificationGapKind::LoopSelfIntersection);
        assert_eq!(gap.cause(), None);
    }

    #[test]
    fn full_check_matches_direct_contextual_result_and_exact_report() {
        let policy = full_check_policy();
        let mut direct_store = Store::new();
        let direct_body =
            ktopo::make::block(&mut direct_store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let direct_context = OperationContext::new(&policy, Tolerances::default()).unwrap();
        let direct = ktopo::check::check_body_report_with_context(
            &direct_store,
            direct_body,
            CheckLevel::Full,
            &direct_context,
        )
        .unwrap();

        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let created = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap();
        let facade = session
            .part(part_id.clone())
            .unwrap()
            .check_body(CheckBodyRequest::new(created.body(), CheckLevel::Full))
            .unwrap();
        let (direct_result, direct_report) = direct.into_parts();
        let expected = adapt_check_report(&part_id, &direct_store, direct_result.unwrap()).unwrap();
        assert_eq!(facade.result(), Ok(&expected));
        assert_eq!(facade.report(), &direct_report);
    }

    #[test]
    fn full_check_family_defaults_fill_an_empty_session_budget() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let outcome = session
            .part(part_id)
            .unwrap()
            .check_body(CheckBodyRequest::new(body, CheckLevel::Full))
            .unwrap();
        assert_eq!(outcome.result().unwrap().outcome(), CheckOutcome::Valid);
        assert_eq!(
            report_usage(
                outcome.report(),
                ktopo::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
                ResourceKind::Items,
            )
            .allowed,
            ktopo::domain::FaceDomainContainmentBudgetProfile::v1_defaults().limits()[0].allowed,
        );
        assert!(outcome.report().limit_events().is_empty());
    }

    #[test]
    fn stricter_session_full_check_stage_overrides_the_family_default() {
        let session_budget = BudgetPlan::new([LimitSpec::new(
            ktopo::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
            ResourceKind::Items,
            AccountingMode::HighWater,
            0,
        )])
        .unwrap();
        let policy = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            session_budget,
            PolicyVersion::V1,
        );
        let mut session = Kernel::with_default_policy(policy).create_session();
        let part_id = session.create_part();
        let body = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let outcome = session
            .part(part_id)
            .unwrap()
            .check_body(CheckBodyRequest::new(body, CheckLevel::Full))
            .unwrap();
        let snapshot = kcore::operation::LimitSnapshot {
            stage: ktopo::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
            resource: ResourceKind::Items,
            consumed: 1,
            allowed: 0,
        };
        assert_eq!(outcome.report().limit_events(), &[snapshot]);
        assert_eq!(
            report_usage(
                outcome.report(),
                ktopo::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
                ResourceKind::Items,
            )
            .allowed,
            0,
        );
    }

    #[test]
    fn explicit_full_check_override_wins_over_a_stricter_session_stage() {
        let session_budget = BudgetPlan::new([LimitSpec::new(
            ktopo::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
            ResourceKind::Items,
            AccountingMode::HighWater,
            0,
        )])
        .unwrap();
        let policy = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            session_budget,
            PolicyVersion::V1,
        );
        let mut session = Kernel::with_default_policy(policy).create_session();
        let part_id = session.create_part();
        let body = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let request = CheckBodyRequest::new(body, CheckLevel::Full).with_settings(
            OperationSettings::new().with_budget_overrides(FullCheckBudgetProfile::v1_defaults()),
        );
        let outcome = session.part(part_id).unwrap().check_body(request).unwrap();
        assert_eq!(outcome.result().unwrap().outcome(), CheckOutcome::Valid);
        assert!(outcome.report().limit_events().is_empty());
        assert_eq!(
            report_usage(
                outcome.report(),
                ktopo::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
                ResourceKind::Items,
            )
            .allowed,
            FullCheckBudgetProfile::v1_defaults().limits()[0].allowed,
        );
    }

    #[test]
    fn full_check_limit_event_survives_a_successful_checker_fallback() {
        let mut session = Kernel::with_default_policy(full_check_policy()).create_session();
        let part_id = session.create_part();
        let body = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let override_plan = BudgetPlan::new([LimitSpec::new(
            ktopo::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
            ResourceKind::Items,
            AccountingMode::HighWater,
            0,
        )])
        .unwrap();
        let request = CheckBodyRequest::new(body, CheckLevel::Full)
            .with_settings(OperationSettings::new().with_budget_overrides(override_plan));
        let outcome = session.part(part_id).unwrap().check_body(request).unwrap();
        let report = outcome.result().unwrap();
        assert_eq!(
            report.outcome(),
            CheckOutcome::Valid,
            "the checker can still prove this block through its conservative domain fallback"
        );
        let snapshot = kcore::operation::LimitSnapshot {
            stage: ktopo::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
            resource: ResourceKind::Items,
            consumed: 1,
            allowed: 0,
        };
        assert_eq!(outcome.report().limit_events(), &[snapshot]);
        assert!(
            report
                .gaps()
                .iter()
                .all(|gap| gap.cause != Some(VerificationGapCause::Limit(snapshot)))
        );
    }

    #[test]
    fn failed_block_is_atomic_and_preserves_future_identity_and_journal() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let failed = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, -1.0, 1.0]))
            .unwrap();
        let error = failed.result().unwrap_err();
        assert_eq!(error.class(), ErrorClass::InvalidInput);
        assert_eq!(error.code(), kcore::error::code::INVALID_GEOMETRY);
        assert!(failed.report().usage().is_empty());
        assert_eq!(session.part(part_id.clone()).unwrap().bodies().len(), 0);

        let created = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap();
        let mut direct = Store::new();
        assert!(
            ktopo::make::block_with_journal(&mut direct, &Frame::world(), [1.0, -1.0, 1.0])
                .is_err()
        );
        let expected =
            ktopo::make::block_with_journal(&mut direct, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        assert_eq!(created.body.raw(), expected.body());
        assert_eq!(created.journal.inner, *expected.journal());
    }

    #[test]
    fn body_check_rejects_wrong_part_before_lower_resolution() {
        let mut session = Kernel::new().create_session();
        let first = session.create_part();
        let second = session.create_part();
        let body = session
            .edit_part(first)
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 1.0, 1.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        assert!(matches!(
            session
                .part(second)
                .unwrap()
                .check_body(CheckBodyRequest::new(body, CheckLevel::Fast)),
            Err(KernelError::WrongPart { .. })
        ));
    }

    #[test]
    fn default_surface_evaluation_matches_direct_graph_bits_and_accounting() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let surface = add_surface_chain(&mut session, &part_id, &[])
            .pop()
            .unwrap();
        let expected = {
            let part = session.part(part_id.clone()).unwrap();
            let mut direct = EvalContext::new(
                part.state.store.geometry(),
                EvalLimits::default(),
                Tolerances::default(),
            );
            direct
                .eval_surface(
                    surface.raw(),
                    [2.0, -3.0],
                    kgraph::SurfaceDerivativeOrder::First,
                )
                .unwrap()
        };

        let outcome = session
            .part(part_id)
            .unwrap()
            .evaluate_surface(SurfaceEvaluationRequest::new(
                surface.clone(),
                [2.0, -3.0],
                kgraph::SurfaceDerivativeOrder::First,
            ))
            .unwrap();
        let value = outcome.result().unwrap();
        assert_eq!(value.surface(), surface);
        assert_eq!(value.position(), expected.p);
        assert_eq!(value.derivatives(), expected);
        assert_eq!(
            report_usage(
                outcome.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            ),
            kcore::operation::LimitSnapshot {
                stage: kgraph::eval_stage::NODE_VISITS,
                resource: ResourceKind::Work,
                consumed: 1,
                allowed: EvalLimits::default().max_node_visits_per_query as u64,
            }
        );
        assert_eq!(
            report_usage(
                outcome.report(),
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
            )
            .consumed,
            1
        );
        assert!(outcome.report().limit_events().is_empty());
    }

    #[test]
    fn nested_offset_evaluation_retains_identity_and_charges_each_dependency_once() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let surfaces = add_surface_chain(&mut session, &part_id, &[0.25, 0.5]);
        let basis = surfaces[0].clone();
        let first = surfaces[1].clone();
        let nested = surfaces[2].clone();
        let part = session.part(part_id).unwrap();
        assert_eq!(
            part.surface(nested.clone()).unwrap().offset_basis(),
            Some(first.clone())
        );
        assert_eq!(part.surface(first).unwrap().offset_basis(), Some(basis));

        let outcome = part
            .evaluate_surface(SurfaceEvaluationRequest::new(
                nested.clone(),
                [1.0, -1.0],
                kgraph::SurfaceDerivativeOrder::First,
            ))
            .unwrap();
        assert_eq!(outcome.result().unwrap().surface(), nested);
        assert_eq!(outcome.result().unwrap().derivatives().p.z, 0.75);
        assert_eq!(
            report_usage(
                outcome.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            )
            .consumed,
            3
        );
        assert_eq!(
            report_usage(
                outcome.report(),
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
            )
            .consumed,
            3
        );
    }

    #[test]
    fn graph_child_limits_have_exact_n_minus_one_and_n_boundaries() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let nested = add_surface_chain(&mut session, &part_id, &[0.25, 0.5])
            .pop()
            .unwrap();
        let part = session.part(part_id).unwrap();

        let node_failure = part
            .evaluate_surface(
                SurfaceEvaluationRequest::new(
                    nested.clone(),
                    [0.0, 0.0],
                    kgraph::SurfaceDerivativeOrder::First,
                )
                .with_settings(graph_settings(8, 2)),
            )
            .unwrap();
        let node_snapshot = kcore::operation::LimitSnapshot {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: ResourceKind::Work,
            consumed: 3,
            allowed: 2,
        };
        assert_eq!(
            node_failure.result().unwrap_err().limit(),
            Some(node_snapshot)
        );
        assert_eq!(node_failure.report().limit_events(), &[node_snapshot]);
        assert_eq!(
            report_usage(
                node_failure.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            )
            .consumed,
            2
        );

        let depth_failure = part
            .evaluate_surface(
                SurfaceEvaluationRequest::new(
                    nested.clone(),
                    [0.0, 0.0],
                    kgraph::SurfaceDerivativeOrder::First,
                )
                .with_settings(graph_settings(2, 8)),
            )
            .unwrap();
        let depth_snapshot = kcore::operation::LimitSnapshot {
            stage: kgraph::eval_stage::DEPENDENCY_DEPTH,
            resource: ResourceKind::Depth,
            consumed: 3,
            allowed: 2,
        };
        assert_eq!(
            depth_failure.result().unwrap_err().limit(),
            Some(depth_snapshot)
        );
        assert_eq!(depth_failure.report().limit_events(), &[depth_snapshot]);
        assert_eq!(
            report_usage(
                depth_failure.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            )
            .consumed,
            3
        );
        assert_eq!(
            report_usage(
                depth_failure.report(),
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
            )
            .consumed,
            2
        );

        let exact = part
            .evaluate_surface(
                SurfaceEvaluationRequest::new(
                    nested,
                    [0.0, 0.0],
                    kgraph::SurfaceDerivativeOrder::First,
                )
                .with_settings(graph_settings(3, 3)),
            )
            .unwrap();
        assert!(exact.result().is_ok());
        assert_eq!(
            report_usage(
                exact.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            )
            .consumed,
            3
        );
        assert_eq!(
            report_usage(
                exact.report(),
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
            )
            .consumed,
            3
        );
    }

    #[test]
    fn root_total_work_precedes_a_more_permissive_graph_stage() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let nested = add_surface_chain(&mut session, &part_id, &[0.25, 0.5])
            .pop()
            .unwrap();
        let budget = EvalBudgetProfile::for_limits(8, 8).with_total_work_limit(2);
        let outcome = session
            .part(part_id)
            .unwrap()
            .evaluate_surface(
                SurfaceEvaluationRequest::new(
                    nested,
                    [0.0, 0.0],
                    kgraph::SurfaceDerivativeOrder::First,
                )
                .with_settings(OperationSettings::new().with_budget_overrides(budget)),
            )
            .unwrap();
        let snapshot = kcore::operation::LimitSnapshot {
            stage: TOTAL_WORK_STAGE,
            resource: ResourceKind::Work,
            consumed: 3,
            allowed: 2,
        };
        assert_eq!(outcome.result().unwrap_err().limit(), Some(snapshot));
        assert_eq!(outcome.report().limit_events(), &[snapshot]);
        assert_eq!(
            report_usage(
                outcome.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            )
            .consumed,
            2,
            "accepted graph work up to the root ceiling remains accounted"
        );
        let error = outcome.result().unwrap_err();
        assert_eq!(error.class(), ErrorClass::ResourceLimit);
        assert_eq!(error.code(), kcore::error::code::RESOURCE_LIMIT);
        assert!(matches!(
            error.source().and_then(|source| source.downcast_ref()),
            Some(kcore::error::Error::OperationPolicy {
                source: OperationPolicyError::LimitReached(actual)
            }) if actual == &snapshot
        ));
    }

    #[test]
    fn evaluation_failure_retains_report_classification_and_source_chain() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let surface = add_surface_chain(&mut session, &part_id, &[])
            .pop()
            .unwrap();
        let outcome = session
            .part(part_id)
            .unwrap()
            .evaluate_surface(SurfaceEvaluationRequest::new(
                surface,
                [f64::NAN, 0.0],
                kgraph::SurfaceDerivativeOrder::Position,
            ))
            .unwrap();
        let error = outcome.result().unwrap_err();
        assert_eq!(error.class(), ErrorClass::InvalidInput);
        assert_eq!(error.code(), kgraph::eval_error_code::INVALID_PARAMETER);
        let facade_source = error
            .source()
            .and_then(|source| source.downcast_ref::<GeometryEvaluationError>())
            .unwrap();
        assert!(matches!(
            facade_source
                .source()
                .and_then(|source| source.downcast_ref::<EvalError>()),
            Some(EvalError::InvalidParameter)
        ));
        assert!(outcome.report().limit_events().is_empty());
        assert_eq!(
            report_usage(
                outcome.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            )
            .consumed,
            0
        );
    }

    #[test]
    fn surface_identity_precedes_invalid_operation_settings_for_wrong_and_stale_ids() {
        let strict_policy = SessionPolicy::new(
            SessionPrecision::try_new(1.0e-6, 1.0e-11, 500.0).unwrap(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            BudgetPlan::empty(),
            PolicyVersion::V1,
        );
        let mut session = Kernel::with_default_policy(strict_policy).create_session();
        let first = session.create_part();
        let second = session.create_part();
        let first_surface = add_surface_chain(&mut session, &first, &[]).pop().unwrap();
        let second_surface = add_surface_chain(&mut session, &second, &[]).pop().unwrap();
        assert_eq!(first_surface.raw(), second_surface.raw());

        assert!(matches!(
            session
                .part(second)
                .unwrap()
                .evaluate_surface(SurfaceEvaluationRequest::new(
                    first_surface.clone(),
                    [0.0, 0.0],
                    kgraph::SurfaceDerivativeOrder::Position,
                )),
            Err(KernelError::WrongPart { .. })
        ));

        {
            let mut edit = session.edit_part(first.clone()).unwrap();
            let mut transaction = edit.store_mut_for_test().transaction().unwrap();
            transaction
                .assembly()
                .remove_surface(first_surface.raw())
                .unwrap();
            transaction.commit_checked(&[]).unwrap();
        }
        assert!(matches!(
            session
                .part(first)
                .unwrap()
                .evaluate_surface(SurfaceEvaluationRequest::new(
                    first_surface,
                    [0.0, 0.0],
                    kgraph::SurfaceDerivativeOrder::Position,
                )),
            Err(KernelError::StaleEntity {
                kind: crate::EntityKind::Surface
            })
        ));
    }

    #[test]
    fn request_defaults_do_not_widen_a_stricter_session_graph_limit() {
        let session_budget = BudgetPlan::new([LimitSpec::new(
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            1,
        )])
        .unwrap();
        let policy = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            session_budget,
            PolicyVersion::V1,
        );
        let mut session = Kernel::with_default_policy(policy).create_session();
        let part_id = session.create_part();
        let surface = add_surface_chain(&mut session, &part_id, &[0.25])
            .pop()
            .unwrap();
        let outcome = session
            .part(part_id)
            .unwrap()
            .evaluate_surface(SurfaceEvaluationRequest::new(
                surface,
                [0.0, 0.0],
                kgraph::SurfaceDerivativeOrder::First,
            ))
            .unwrap();
        let snapshot = kcore::operation::LimitSnapshot {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: ResourceKind::Work,
            consumed: 2,
            allowed: 1,
        };
        assert_eq!(outcome.result().unwrap_err().limit(), Some(snapshot));
        assert_eq!(outcome.report().limit_events(), &[snapshot]);
        assert_eq!(
            report_usage(
                outcome.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            ),
            kcore::operation::LimitSnapshot {
                stage: kgraph::eval_stage::NODE_VISITS,
                resource: ResourceKind::Work,
                consumed: 1,
                allowed: 1,
            }
        );
        assert_eq!(
            report_usage(
                outcome.report(),
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
            )
            .allowed,
            EvalLimits::default().max_dependency_depth as u64,
            "the graph default fills only the session's missing depth stage"
        );
    }

    #[test]
    fn explicit_graph_override_has_normal_precedence_over_a_strict_session_stage() {
        let session_budget = BudgetPlan::new([LimitSpec::new(
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            1,
        )])
        .unwrap();
        let policy = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            session_budget,
            PolicyVersion::V1,
        );
        let mut session = Kernel::with_default_policy(policy).create_session();
        let part_id = session.create_part();
        let surface = add_surface_chain(&mut session, &part_id, &[0.25])
            .pop()
            .unwrap();
        let outcome = session
            .part(part_id)
            .unwrap()
            .evaluate_surface(
                SurfaceEvaluationRequest::new(
                    surface,
                    [0.0, 0.0],
                    kgraph::SurfaceDerivativeOrder::First,
                )
                .with_settings(graph_settings(8, 2)),
            )
            .unwrap();
        assert!(outcome.result().is_ok());
        assert_eq!(
            report_usage(
                outcome.report(),
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            ),
            kcore::operation::LimitSnapshot {
                stage: kgraph::eval_stage::NODE_VISITS,
                resource: ResourceKind::Work,
                consumed: 2,
                allowed: 2,
            }
        );
    }
}
