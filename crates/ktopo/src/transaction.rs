//! Failure-atomic modeling transactions and deterministic journals.
//!
//! A [`Transaction`] opens copy-on-write undo frames on every [`Store`]
//! arena. Its public methods own Euler mutation, pcurve preflight, semantic
//! lineage, and tolerance growth without exposing raw operators. Committing
//! produces raw net mutations in a stable entity-type/slot order plus semantic
//! operation evidence. Journals also retain declared tolerance-growth
//! budgets, ordered per-entity changes, and descriptive face-tolerance
//! inheritance/combination evidence. Dropping or explicitly rolling back
//! restores entity contents, handle generations, free-list order, and future
//! allocations while discarding uncommitted budget usage.

use crate::entity::{
    BodyId, CurveId, EdgeId, EntityRef, FaceId, LoopId, PointId, Sense, SurfaceId, VertexId,
};
use crate::euler::{FinPcurvePair, Mef, Mekr, Mev, Mvfs};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::graph_work::GraphQueryWork;
use crate::store::{ArenaEntity, MutableEntity, Store};
use crate::tolerance::EntityTolerance;
use core::ops::Deref;
use kcore::arena::Handle;
use kcore::error::{Error, Result};
use kcore::operation::{OperationContext, OperationOutcome, OperationPolicyError, OperationScope};
use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances};
use kgeom::vec::Point3;
use kgraph::{
    Curve2dHandle, EvalBudgetProfile, EvalLimits, SurfaceHandle,
    TransmittedNurbsIntersectionCertificate, TransmittedPlaneIntersectionCertificate,
};

/// Net kind of one committed entity mutation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MutationKind {
    /// An entity identity became live.
    Created,
    /// A pre-existing entity was mutably accessed and remains live.
    Modified,
    /// A pre-existing entity identity was removed.
    Deleted,
}

/// One raw net entity mutation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mutation {
    /// Affected entity. For [`MutationKind::Deleted`], the handle is
    /// intentionally stale after commit but preserves the deleted identity.
    pub entity: EntityRef,
    /// Net mutation kind.
    pub kind: MutationKind,
}

/// Opaque identifier for a tolerance-growth budget declared on one
/// transaction. Budget identifiers are meaningful only to that transaction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ToleranceBudgetId(usize);

impl ToleranceBudgetId {
    /// Stable declaration-order index into [`Journal::tolerance_budgets`].
    pub fn index(self) -> usize {
        self.0
    }
}

/// Final usage of one transaction-owned tolerance-growth budget.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToleranceBudgetReport {
    operation: &'static str,
    limit: f64,
    consumed: f64,
}

impl ToleranceBudgetReport {
    /// Stable operation name supplied when the budget was declared.
    pub fn operation(&self) -> &'static str {
        self.operation
    }

    /// Maximum aggregate growth permitted for this operation.
    pub fn limit(&self) -> f64 {
        self.limit
    }

    /// Aggregate tolerance growth actually committed.
    pub fn consumed(&self) -> f64 {
        self.consumed
    }

    /// Unspent growth at commit time.
    pub fn remaining(&self) -> f64 {
        (self.limit - self.consumed).max(0.0)
    }
}

/// One entity tolerance introduced or enlarged under a declared budget.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToleranceEvent {
    entity: EntityRef,
    previous: Option<EntityTolerance>,
    current: EntityTolerance,
    budget: ToleranceBudgetId,
}

/// One topology target eligible for operation-owned tolerance growth.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ToleranceGrowthTarget {
    /// Face metric tolerance.
    Face(FaceId),
    /// Edge metric tolerance.
    Edge(EdgeId),
    /// Vertex metric tolerance.
    Vertex(VertexId),
}

/// One requested final tolerance in a failure-atomic growth batch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToleranceGrowth {
    /// Entity whose tolerance may grow.
    pub target: ToleranceGrowthTarget,
    /// Requested final tolerance value.
    pub requested: f64,
}

impl ToleranceGrowth {
    /// Construct one target-tolerance request.
    pub const fn new(target: ToleranceGrowthTarget, requested: f64) -> Self {
        Self { target, requested }
    }
}

impl ToleranceEvent {
    /// Entity whose metric tolerance changed.
    pub fn entity(&self) -> EntityRef {
        self.entity
    }

    /// Prior tolerance, or `None` when an exact entity became tolerant.
    pub fn previous(&self) -> Option<EntityTolerance> {
        self.previous
    }

    /// Committed tolerance including retained origin and growth provenance.
    pub fn current(&self) -> EntityTolerance {
        self.current
    }

    /// Budget that authorized this change, indexing [`Journal::tolerance_budgets`].
    pub fn budget(&self) -> ToleranceBudgetId {
        self.budget
    }
}

/// Descriptive tolerance propagation performed by a semantic face edit.
///
/// These records preserve inherited tolerance origin and growth provenance;
/// they do not declare growth, consume a budget, or grant reusable authoring
/// authority. Exact faces are represented explicitly by `None`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum FaceTolerancePropagation {
    /// MEF copied the source face tolerance to the newly divided face.
    Inherited {
        /// Existing face whose complete tolerance value was copied.
        source: FaceId,
        /// Newly created face receiving the same tolerance.
        result: FaceId,
        /// Exact copied value, including `None` for an exact face.
        tolerance: Option<EntityTolerance>,
    },
    /// KEF selected the larger input tolerance for the surviving face.
    CombinedMax {
        /// Ordered `[surviving, absorbed]` input identities.
        sources: [FaceId; 2],
        /// Input values in the same order, retained even though the absorbed
        /// face is stale after commit.
        source_tolerances: [Option<EntityTolerance>; 2],
        /// Surviving face identity.
        result: FaceId,
        /// Source whose complete tolerance provenance was retained. Equal
        /// values deterministically select the surviving first source; both
        /// exact inputs select `None`.
        selected_source: Option<FaceId>,
        /// Selected result value, including `None` when both inputs were exact.
        tolerance: Option<EntityTolerance>,
    },
}

/// Semantic identity relationship emitted by a modeling operation.
///
/// Raw mutation lists describe storage changes; lineage describes why model
/// identities changed and is the input required by future persistent naming
/// and feature regeneration.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum LineageEvent {
    /// `derived` was constructed from `source` without replacing it.
    DerivedFrom {
        /// New or changed entity.
        derived: EntityRef,
        /// Source entity.
        source: EntityRef,
    },
    /// One source entity was divided into ordered result pieces.
    Split {
        /// Identity that was split.
        source: EntityRef,
        /// Deterministically ordered result identities.
        pieces: Vec<EntityRef>,
    },
    /// Ordered source entities were combined into one result.
    Merge {
        /// Deterministically ordered source identities.
        sources: Vec<EntityRef>,
        /// Combined result identity.
        result: EntityRef,
    },
    /// One identity was superseded by another.
    Replaced {
        /// Superseded identity.
        old: EntityRef,
        /// Replacement identity.
        new: EntityRef,
    },
    /// One semantic identity was intentionally removed without a replacement.
    Deleted {
        /// Removed identity; intentionally stale after commit.
        entity: EntityRef,
    },
}

/// Deterministic committed transaction record.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Journal {
    mutations: Vec<Mutation>,
    lineage: Vec<LineageEvent>,
    tolerance_budgets: Vec<ToleranceBudgetReport>,
    tolerance_events: Vec<ToleranceEvent>,
    face_tolerance_propagations: Vec<FaceTolerancePropagation>,
}

/// Acceptance rule for an opt-in Full-assurance transaction commit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FullCommitRequirement {
    /// Commit only when every checked body is proof-complete.
    RequireValid,
    /// Commit a Fast-clean body even when Full checking retains explicit proof
    /// gaps. Proven Full faults still reject the transaction.
    AllowIndeterminate,
}

/// Full-check evidence for one deterministically selected body.
#[derive(Debug, Clone, PartialEq)]
pub struct FullBodyCheck {
    body: BodyId,
    report: crate::check::CheckReport,
    point_values: Vec<(PointId, Point3)>,
}

impl FullBodyCheck {
    fn new(store: &Store, body: BodyId, report: crate::check::CheckReport) -> Result<Self> {
        let mut point_values = Vec::new();
        for entity in report
            .faults
            .iter()
            .map(|fault| fault.entity)
            .chain(report.gaps.iter().map(|gap| gap.entity))
        {
            let EntityRef::Point(point) = entity else {
                continue;
            };
            if point_values
                .iter()
                .any(|(candidate, _)| *candidate == point)
            {
                continue;
            }
            point_values.push((point, *store.get(point)?));
        }
        Ok(Self {
            body,
            report,
            point_values,
        })
    }

    /// Body checked in explicit-root, affected-root, then store order.
    pub fn body(&self) -> BodyId {
        self.body
    }

    /// Exact Full checker report captured before commit or rollback.
    pub fn report(&self) -> &crate::check::CheckReport {
        &self.report
    }

    /// Snapshot one point-valued checker subject while candidate state was live.
    pub fn point_value(&self, point: PointId) -> Option<Point3> {
        self.point_values
            .iter()
            .find_map(|(candidate, value)| (*candidate == point).then_some(*value))
    }
}

/// Evidence-bearing outcome of an opt-in Full-assurance commit attempt.
///
/// A rejected decision has already restored the transaction entry state and
/// therefore carries no journal. Execution and resource failures remain
/// ordinary errors rather than decisions.
#[derive(Debug, Clone, PartialEq)]
pub struct FullCommitDecision {
    journal: Option<Journal>,
    checks: Vec<FullBodyCheck>,
}

impl FullCommitDecision {
    fn committed(journal: Journal, checks: Vec<FullBodyCheck>) -> Self {
        Self {
            journal: Some(journal),
            checks,
        }
    }

    fn rejected(checks: Vec<FullBodyCheck>) -> Self {
        Self {
            journal: None,
            checks,
        }
    }

    /// Whether the candidate was persisted.
    pub fn is_committed(&self) -> bool {
        self.journal.is_some()
    }

    /// Committed journal, absent after proof-policy rejection.
    pub fn journal(&self) -> Option<&Journal> {
        self.journal.as_ref()
    }

    /// Full reports in deterministic checked-body order.
    pub fn checks(&self) -> &[FullBodyCheck] {
        &self.checks
    }

    /// Consume the decision into its optional journal and owned reports.
    pub fn into_parts(self) -> (Option<Journal>, Vec<FullBodyCheck>) {
        (self.journal, self.checks)
    }
}

/// Low-level entity assembly available only while a transaction owns every
/// arena's undo frame.
///
/// This is the bridge for interchange reconstruction and specialized kernel
/// builders that must materialize an already-defined entity graph. Ordinary
/// modeling should prefer the semantic [`Transaction`] methods. Reads are
/// inherited from [`Store`] through [`Deref`]; all writes remain scoped to the
/// transaction and are therefore rollback-safe and journaled. Public callers
/// can retain assembly changes only through a checked commit, which validates
/// every affected body and the store's complete topology ownership closure.
///
/// # Stability
///
/// `AssemblyStore` is an unstable trusted-adapter seam, not an application
/// modeling API. Its reviewed cross-crate users are X_T reconstruction and the
/// X_T external-oracle fixture generator. New interchange formats must keep raw
/// assembly inside their lower-layer adapters. A future breaking release may
/// seal or replace this type after those consumers migrate; ordinary facade
/// clients must not depend on its methods or raw entity layout.
pub struct AssemblyStore<'a> {
    store: &'a mut Store,
}

impl AssemblyStore<'_> {
    /// Insert any geometry or topology entity into the active transaction.
    pub fn add<T: ArenaEntity>(&mut self, entity: T) -> Handle<T> {
        self.store.add(entity)
    }

    /// Insert a validated 3D curve descriptor into the graph.
    pub fn insert_curve(&mut self, curve: CurveGeom) -> Result<CurveId> {
        self.store.insert_curve(curve)
    }

    /// Insert a certified transmitted exact-plane-field intersection and
    /// retain its ordered source and pcurve dependencies in the geometry graph.
    pub fn insert_verified_transmitted_plane_intersection_curve(
        &mut self,
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: TransmittedPlaneIntersectionCertificate,
    ) -> Result<CurveId> {
        self.store
            .insert_verified_transmitted_plane_intersection_curve(
                source_surfaces,
                pcurves,
                certificate,
            )
    }

    /// Insert a certified transmitted chart containing one or two original
    /// NURBS traces and retain its ordered source/pcurve dependencies.
    pub fn insert_verified_transmitted_nurbs_intersection_curve(
        &mut self,
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: TransmittedNurbsIntersectionCertificate,
    ) -> Result<CurveId> {
        self.store
            .insert_verified_transmitted_nurbs_intersection_curve(
                source_surfaces,
                pcurves,
                certificate,
            )
    }

    /// Compatibility insertion name for the original mixed Plane/NURBS arm.
    pub fn insert_verified_transmitted_plane_nurbs_intersection_curve(
        &mut self,
        source_surfaces: [SurfaceHandle; 2],
        pcurves: [Curve2dHandle; 2],
        certificate: TransmittedNurbsIntersectionCertificate,
    ) -> Result<CurveId> {
        self.insert_verified_transmitted_nurbs_intersection_curve(
            source_surfaces,
            pcurves,
            certificate,
        )
    }

    /// Insert a validated surface descriptor into the graph.
    pub fn insert_surface(&mut self, surface: SurfaceGeom) -> Result<SurfaceId> {
        self.store.insert_surface(surface)
    }

    /// Insert a validated pcurve descriptor into the graph.
    pub fn insert_pcurve(&mut self, curve: Curve2dGeom) -> Result<crate::entity::Curve2dId> {
        self.store.insert_pcurve(curve)
    }

    /// Mutably borrow an entity inside the active transaction.
    pub fn get_mut<T: MutableEntity>(&mut self, handle: Handle<T>) -> Result<&mut T> {
        self.store.get_mut(handle)
    }

    /// Replace a 3D curve descriptor atomically while preserving its handle.
    pub fn replace_curve(&mut self, handle: CurveId, curve: CurveGeom) -> Result<CurveGeom> {
        self.store.replace_curve(handle, curve)
    }

    /// Replace a surface descriptor atomically while preserving its handle.
    pub fn replace_surface(
        &mut self,
        handle: SurfaceId,
        surface: SurfaceGeom,
    ) -> Result<SurfaceGeom> {
        self.store.replace_surface(handle, surface)
    }

    /// Replace a pcurve descriptor atomically while preserving its handle.
    pub fn replace_pcurve(
        &mut self,
        handle: crate::entity::Curve2dId,
        curve: Curve2dGeom,
    ) -> Result<Curve2dGeom> {
        self.store.replace_pcurve(handle, curve)
    }

    /// Remove an entity inside the active transaction.
    pub fn remove<T: ArenaEntity>(&mut self, handle: Handle<T>) -> Result<T> {
        self.store.remove(handle)
    }

    /// Remove an unreferenced graph curve.
    pub fn remove_curve(&mut self, handle: CurveId) -> Result<CurveGeom> {
        self.store.remove_curve(handle)
    }

    /// Remove an unreferenced graph surface.
    pub fn remove_surface(&mut self, handle: SurfaceId) -> Result<SurfaceGeom> {
        self.store.remove_surface(handle)
    }

    /// Remove an unreferenced graph pcurve.
    pub fn remove_pcurve(&mut self, handle: crate::entity::Curve2dId) -> Result<Curve2dGeom> {
        self.store.remove_pcurve(handle)
    }
}

impl Deref for AssemblyStore<'_> {
    type Target = Store;

    fn deref(&self) -> &Self::Target {
        self.store
    }
}

impl Journal {
    /// Raw net mutations, ordered by arena type then slot.
    pub fn mutations(&self) -> &[Mutation] {
        &self.mutations
    }

    /// Semantic lineage in caller-recorded operation order.
    pub fn lineage(&self) -> &[LineageEvent] {
        &self.lineage
    }

    /// Declared tolerance-growth budgets in deterministic declaration order.
    pub fn tolerance_budgets(&self) -> &[ToleranceBudgetReport] {
        &self.tolerance_budgets
    }

    /// Tolerance changes in deterministic operation order.
    pub fn tolerance_events(&self) -> &[ToleranceEvent] {
        &self.tolerance_events
    }

    /// Face split/merge tolerance policy evidence in semantic operation order.
    pub fn face_tolerance_propagations(&self) -> &[FaceTolerancePropagation] {
        &self.face_tolerance_propagations
    }

    pub(crate) fn new(
        mutations: Vec<Mutation>,
        lineage: Vec<LineageEvent>,
        tolerance_budgets: Vec<ToleranceBudgetReport>,
        tolerance_events: Vec<ToleranceEvent>,
        face_tolerance_propagations: Vec<FaceTolerancePropagation>,
    ) -> Self {
        Self {
            mutations,
            lineage,
            tolerance_budgets,
            tolerance_events,
            face_tolerance_propagations,
        }
    }
}

#[cfg(feature = "benchmark-internals")]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct CommitPhaseObservation {
    index: crate::index::CandidateIndexObservation,
    geometry_graph_validation_starts: usize,
    geometry_graph_validation_primary_node_starts: usize,
    fast_body_check_starts: usize,
}

#[cfg(feature = "benchmark-internals")]
impl CommitPhaseObservation {
    fn finish(
        self,
        store: &Store,
        committed: bool,
        affected: &[BodyId],
        refreshed_bodies: usize,
        checked_bodies: usize,
        mutations: usize,
    ) -> crate::benchmark::CommitObservation {
        crate::benchmark::CommitObservation {
            committed,
            body_count: store.count::<crate::entity::Body>(),
            affected_bodies: affected.len(),
            refreshed_bodies,
            checked_bodies,
            mutations,
            affected_order_digest: crate::benchmark::affected_digest(store, affected),
            geometry_graph_validation_starts: self.geometry_graph_validation_starts,
            geometry_graph_validation_primary_node_starts: self
                .geometry_graph_validation_primary_node_starts,
            candidate_index_clone_starts: self.index.clone_starts,
            candidate_index_cloned_body_footprints: self.index.cloned_body_footprints,
            candidate_index_cloned_body_order_entries: self.index.cloned_body_order_entries,
            candidate_index_refresh_body_starts: self.index.refresh_body_starts,
            candidate_index_body_order_refresh_entries: self.index.body_order_refresh_entries,
            affected_root_selection_starts: self.index.affected_selection_starts,
            affected_root_selection_mutation_items: self.index.affected_selection_mutation_items,
            fast_body_check_starts: self.fast_body_check_starts,
        }
    }

    fn observe_geometry_validation(&mut self, observation: kgraph::GraphValidationObservation) {
        self.geometry_graph_validation_starts += observation.validation_starts();
        self.geometry_graph_validation_primary_node_starts += observation.primary_node_starts();
    }
}

/// Scoped failure-atomic mutation of one [`Store`].
///
/// Transactions are rollback-on-drop. Call [`Self::commit_checked`] exactly
/// once to retain changes and obtain their journal. Nested transactions on one
/// store are currently rejected so semantic journal composition cannot be
/// accidentally underspecified.
pub struct Transaction<'a> {
    store: &'a mut Store,
    lineage: Vec<LineageEvent>,
    tolerance_budgets: Vec<ToleranceBudgetReport>,
    tolerance_events: Vec<ToleranceEvent>,
    face_tolerance_propagations: Vec<FaceTolerancePropagation>,
    finished: bool,
}

impl<'a> Transaction<'a> {
    pub(crate) fn new(store: &'a mut Store) -> Self {
        Self {
            store,
            lineage: Vec::new(),
            tolerance_budgets: Vec::new(),
            tolerance_events: Vec::new(),
            face_tolerance_propagations: Vec::new(),
            finished: false,
        }
    }

    /// Read the in-progress store state.
    pub fn store(&self) -> &Store {
        self.store
    }

    /// Topology-internal mutable Store access for implemented constructors.
    pub(crate) fn store_mut(&mut self) -> &mut Store {
        self.store
    }

    /// Record one semantic derivation produced by an in-crate assembler.
    pub(crate) fn record_derived_from(&mut self, derived: EntityRef, source: EntityRef) {
        self.lineage
            .push(LineageEvent::DerivedFrom { derived, source });
    }

    /// Open the unstable low-level reconstruction/assembly seam for this
    /// transaction.
    ///
    /// This is reserved for reviewed kernel builders and trusted interchange
    /// adapters. Ordinary modeling uses semantic transaction operations.
    pub fn assembly(&mut self) -> AssemblyStore<'_> {
        AssemblyStore { store: self.store }
    }

    /// Duplicate one complete body under an orientation-preserving rigid
    /// placement while retaining deterministic DerivedFrom lineage.
    ///
    /// Every owned topology identity, point, curve, surface, offset basis,
    /// and pcurve is newly allocated. Surface/curve parameterization,
    /// pcurve maps, charts, tolerances, domains, and ownership order are
    /// preserved exactly. Supported verified intersection descriptors are
    /// recertified over transformed source geometry.
    ///
    /// This compatibility entry retains the historical [`kcore::error::Result`]
    /// boundary. Use [`Self::copy_body_rigid_with_source`] when the exact graph
    /// certificate failure must remain available.
    pub fn copy_body_rigid(
        &mut self,
        source: BodyId,
        placement: kgeom::frame::Frame,
    ) -> Result<BodyId> {
        self.copy_body_rigid_with_source(source, placement)
            .map_err(crate::BodyCopyError::into_legacy)
    }

    /// Duplicate a body while retaining typed certificate-reissuance errors.
    ///
    /// This has the same copy, lineage, allocation, and rollback semantics as
    /// [`Self::copy_body_rigid`], but returns [`crate::BodyCopyError`] instead
    /// of flattening certificate failures into the legacy shared error.
    pub fn copy_body_rigid_with_source(
        &mut self,
        source: BodyId,
        placement: kgeom::frame::Frame,
    ) -> crate::BodyCopyResult<BodyId> {
        let copied = crate::body_copy::copy_body_rigid(self.store, source, placement)?;
        self.lineage.extend(
            copied
                .lineage
                .into_iter()
                .map(|(derived, source)| LineageEvent::DerivedFrom { derived, source }),
        );
        Ok(copied.body)
    }

    /// Declare the aggregate tolerance growth available to one operation.
    ///
    /// The budget is transaction-owned: rollback discards both its usage and
    /// events. A successful commit records the final usage in the journal.
    /// Growth is charged above the session linear-resolution floor.
    pub fn declare_tolerance_budget(
        &mut self,
        operation: &'static str,
        max_total_growth: f64,
    ) -> Result<ToleranceBudgetId> {
        if operation.is_empty() {
            return Err(Error::InvalidGeometry {
                reason: "tolerance operation name must not be empty",
            });
        }
        if !max_total_growth.is_finite() || max_total_growth < 0.0 {
            return Err(Error::InvalidToleranceBudget {
                limit: max_total_growth,
            });
        }
        let id = ToleranceBudgetId(self.tolerance_budgets.len());
        self.tolerance_budgets.push(ToleranceBudgetReport {
            operation,
            limit: max_total_growth,
            consumed: 0.0,
        });
        Ok(id)
    }

    fn prepare_tolerance_growth(
        &mut self,
        budget: ToleranceBudgetId,
        entity: EntityRef,
        previous: Option<EntityTolerance>,
        requested: f64,
    ) -> Result<Option<EntityTolerance>> {
        Tolerances::default().entity_tolerance(requested)?;
        if previous.is_some_and(|tolerance| requested <= tolerance.value()) {
            return Ok(None);
        }
        let report = self
            .tolerance_budgets
            .get_mut(budget.0)
            .ok_or(Error::InvalidGeometry {
                reason: "tolerance budget does not belong to this transaction",
            })?;
        let old_value = previous
            .map(EntityTolerance::value)
            .unwrap_or(LINEAR_RESOLUTION);
        let growth = (requested - old_value).max(0.0);
        let remaining = (report.limit - report.consumed).max(0.0);
        let new_consumed = report.consumed + growth;
        // Budget accounting combines independently rounded model-unit
        // values. Permit only a small scale-relative arithmetic guard, then
        // clamp to the declared limit so repeated operations cannot exploit
        // that guard as real tolerance growth.
        let accounting_guard = 32.0
            * f64::EPSILON
            * report
                .limit
                .abs()
                .max(new_consumed.abs())
                .max(f64::MIN_POSITIVE);
        if new_consumed > report.limit + accounting_guard {
            return Err(Error::ToleranceBudgetExceeded {
                requested_growth: growth,
                remaining_growth: remaining,
            });
        }
        let current = match previous {
            Some(tolerance) => tolerance.grown_to(requested, report.operation)?,
            None => EntityTolerance::operation(requested, report.operation)?,
        };
        report.consumed = new_consumed.min(report.limit);
        self.tolerance_events.push(ToleranceEvent {
            entity,
            previous,
            current,
            budget,
        });
        Ok(Some(current))
    }

    /// Introduce or enlarge a face tolerance under a declared budget.
    pub fn grow_face_tolerance(
        &mut self,
        budget: ToleranceBudgetId,
        face: FaceId,
        requested: f64,
    ) -> Result<()> {
        let previous = self.store.get(face)?.tolerance;
        if let Some(current) =
            self.prepare_tolerance_growth(budget, EntityRef::Face(face), previous, requested)?
        {
            self.store.get_mut(face)?.tolerance = Some(current);
        }
        Ok(())
    }

    /// Introduce or enlarge an edge tolerance under a declared budget.
    pub fn grow_edge_tolerance(
        &mut self,
        budget: ToleranceBudgetId,
        edge: EdgeId,
        requested: f64,
    ) -> Result<()> {
        let previous = self.store.get(edge)?.tolerance;
        if let Some(current) =
            self.prepare_tolerance_growth(budget, EntityRef::Edge(edge), previous, requested)?
        {
            self.store.get_mut(edge)?.tolerance = Some(current);
        }
        Ok(())
    }

    /// Introduce or enlarge a vertex tolerance under a declared budget.
    pub fn grow_vertex_tolerance(
        &mut self,
        budget: ToleranceBudgetId,
        vertex: VertexId,
        requested: f64,
    ) -> Result<()> {
        let previous = self.store.get(vertex)?.tolerance;
        if let Some(current) =
            self.prepare_tolerance_growth(budget, EntityRef::Vertex(vertex), previous, requested)?
        {
            self.store.get_mut(vertex)?.tolerance = Some(current);
        }
        Ok(())
    }

    /// Apply one fully preflighted operation-owned tolerance-growth batch.
    ///
    /// Targets must be distinct and every requested final tolerance must meet
    /// the model resolution floor. Requests at or below an existing tolerance
    /// are deterministic no-ops. Every liveness, value, provenance, and
    /// aggregate-budget check completes before the budget is declared or any
    /// entity is mutated, so rejection leaves the transaction state unchanged.
    pub fn grow_tolerances(
        &mut self,
        operation: &'static str,
        max_total_growth: f64,
        requests: &[ToleranceGrowth],
    ) -> Result<ToleranceBudgetId> {
        if operation.is_empty() {
            return Err(Error::InvalidGeometry {
                reason: "tolerance operation name must not be empty",
            });
        }
        if !max_total_growth.is_finite() || max_total_growth < 0.0 {
            return Err(Error::InvalidToleranceBudget {
                limit: max_total_growth,
            });
        }

        let mut targets = Vec::with_capacity(requests.len());
        let mut prepared = Vec::with_capacity(requests.len());
        let mut consumed: f64 = 0.0;
        for request in requests {
            if targets.contains(&request.target) {
                return Err(Error::InvalidGeometry {
                    reason: "tolerance growth batch contains a duplicate target",
                });
            }
            targets.push(request.target);
            Tolerances::default().entity_tolerance(request.requested)?;
            let previous = match request.target {
                ToleranceGrowthTarget::Face(face) => self.store.get(face)?.tolerance,
                ToleranceGrowthTarget::Edge(edge) => self.store.get(edge)?.tolerance,
                ToleranceGrowthTarget::Vertex(vertex) => self.store.get(vertex)?.tolerance,
            };
            let old_value = previous
                .map(EntityTolerance::value)
                .unwrap_or(LINEAR_RESOLUTION);
            if request.requested <= old_value {
                continue;
            }
            let growth = request.requested - old_value;
            let new_consumed = consumed + growth;
            let accounting_guard = 32.0
                * f64::EPSILON
                * max_total_growth
                    .abs()
                    .max(new_consumed.abs())
                    .max(f64::MIN_POSITIVE);
            if new_consumed > max_total_growth + accounting_guard {
                return Err(Error::ToleranceBudgetExceeded {
                    requested_growth: growth,
                    remaining_growth: (max_total_growth - consumed).max(0.0),
                });
            }
            consumed = new_consumed.min(max_total_growth);
            let current = match previous {
                Some(tolerance) => tolerance.grown_to(request.requested, operation)?,
                None => EntityTolerance::operation(request.requested, operation)?,
            };
            prepared.push((request.target, previous, current));
        }

        // Everything that can fail has completed. Install the validated
        // budget and prepared values directly so the apply phase cannot leave
        // a partially mutated candidate.
        let budget = ToleranceBudgetId(self.tolerance_budgets.len());
        self.tolerance_budgets.push(ToleranceBudgetReport {
            operation,
            limit: max_total_growth,
            consumed,
        });
        for (target, previous, current) in prepared {
            let entity = match target {
                ToleranceGrowthTarget::Face(face) => {
                    self.store
                        .get_mut(face)
                        .expect("tolerance batch preflight keeps the face live")
                        .tolerance = Some(current);
                    EntityRef::Face(face)
                }
                ToleranceGrowthTarget::Edge(edge) => {
                    self.store
                        .get_mut(edge)
                        .expect("tolerance batch preflight keeps the edge live")
                        .tolerance = Some(current);
                    EntityRef::Edge(edge)
                }
                ToleranceGrowthTarget::Vertex(vertex) => {
                    self.store
                        .get_mut(vertex)
                        .expect("tolerance batch preflight keeps the vertex live")
                        .tolerance = Some(current);
                    EntityRef::Vertex(vertex)
                }
            };
            self.tolerance_events.push(ToleranceEvent {
                entity,
                previous,
                current,
                budget,
            });
        }
        Ok(budget)
    }

    /// MVFS: create the transient minimal solid-body topology.
    ///
    /// Higher construction must complete the body before checked commit.
    pub fn make_minimal_body(
        &mut self,
        surface: SurfaceId,
        sense: Sense,
        point: PointId,
    ) -> Result<Mvfs> {
        let made = crate::euler::mvfs(self.store, surface, sense, point)?;
        self.lineage.push(LineageEvent::DerivedFrom {
            derived: EntityRef::Vertex(made.vertex),
            source: EntityRef::Point(point),
        });
        Ok(made)
    }

    /// MVFS: create transient minimal topology at a validated position.
    ///
    /// The position and supporting surface are validated before the point is
    /// inserted. This candidate cannot pass checked commit until later Euler
    /// operations complete it into checker-valid topology, or KVFS removes it.
    pub fn make_minimal_body_at_position(
        &mut self,
        surface: SurfaceId,
        sense: Sense,
        position: Point3,
    ) -> Result<Mvfs> {
        let (made, point) = crate::euler::mvfs_at_position(self.store, surface, sense, position)?;
        self.lineage.push(LineageEvent::DerivedFrom {
            derived: EntityRef::Vertex(made.vertex),
            source: EntityRef::Point(point),
        });
        Ok(made)
    }

    /// KVFS: remove a body that is still in minimal MVFS form.
    pub fn kill_minimal_body(&mut self, body: BodyId) -> Result<()> {
        let _deleted = crate::euler::kvfs(self.store, body)?;
        self.lineage.push(LineageEvent::Deleted {
            entity: EntityRef::Body(body),
        });
        Ok(())
    }

    /// KVFS inverse for a position-owning MVFS created through the facade.
    ///
    /// The hidden seed point is removed when no live vertex still references
    /// it. Ordinary [`Self::kill_minimal_body`] retains point geometry for
    /// handle-owning callers that may have authored or shared it separately.
    pub fn kill_position_owned_minimal_body(&mut self, body: BodyId) -> Result<()> {
        let deleted = crate::euler::kvfs(self.store, body)?;
        self.lineage.push(LineageEvent::Deleted {
            entity: EntityRef::Body(body),
        });
        let point_in_use = self
            .store
            .iter::<crate::entity::Vertex>()
            .any(|(_, vertex)| vertex.point == deleted.point);
        if !point_in_use {
            self.store.remove(deleted.point)?;
            self.lineage.push(LineageEvent::Deleted {
                entity: EntityRef::Point(deleted.point),
            });
        }
        Ok(())
    }

    /// MEV: sprout an edge and new vertex with mandatory independent pcurves.
    #[allow(clippy::too_many_arguments)]
    pub fn make_edge_vertex(
        &mut self,
        lp: LoopId,
        at: usize,
        curve: CurveId,
        bounds: (f64, f64),
        point: PointId,
        pcurves: FinPcurvePair,
    ) -> Result<Mev> {
        let made =
            crate::euler::mev_with_pcurves(self.store, lp, at, curve, bounds, point, pcurves)?;
        self.lineage.push(LineageEvent::DerivedFrom {
            derived: EntityRef::Edge(made.edge),
            source: EntityRef::Loop(lp),
        });
        self.lineage.push(LineageEvent::DerivedFrom {
            derived: EntityRef::Vertex(made.vertex),
            source: EntityRef::Point(point),
        });
        Ok(made)
    }

    /// MEV: sprout an edge and new vertex at a validated position.
    ///
    /// The position and every topology/pcurve precondition are validated
    /// before the point is inserted, so rejection does not consume a point
    /// identity. The inserted point is retained as the new vertex's lineage
    /// source just like the lower-level handle-taking form.
    #[allow(clippy::too_many_arguments)]
    pub fn make_edge_vertex_at_position(
        &mut self,
        lp: LoopId,
        at: usize,
        curve: CurveId,
        bounds: (f64, f64),
        position: Point3,
        pcurves: FinPcurvePair,
    ) -> Result<Mev> {
        let (made, point) = crate::euler::mev_at_position_with_pcurves(
            self.store, lp, at, curve, bounds, position, pcurves,
        )?;
        self.lineage.push(LineageEvent::DerivedFrom {
            derived: EntityRef::Edge(made.edge),
            source: EntityRef::Loop(lp),
        });
        self.lineage.push(LineageEvent::DerivedFrom {
            derived: EntityRef::Vertex(made.vertex),
            source: EntityRef::Point(point),
        });
        Ok(made)
    }

    /// KEV: remove a strut edge and its otherwise-unused vertex.
    pub fn kill_edge_vertex(&mut self, edge: EdgeId) -> Result<()> {
        let deleted = crate::euler::kev(self.store, edge)?;
        self.lineage.push(LineageEvent::Deleted {
            entity: EntityRef::Edge(edge),
        });
        self.lineage.push(LineageEvent::Deleted {
            entity: EntityRef::Vertex(deleted.vertex),
        });
        Ok(())
    }

    /// KEV inverse for a position-owning MEV created through the facade.
    ///
    /// Besides removing the strut topology, this removes the detached
    /// vertex's point when no live vertex still references it. Ordinary
    /// [`Self::kill_edge_vertex`] intentionally retains point geometry for
    /// handle-owning callers that may have authored or shared it separately.
    pub fn kill_position_owned_edge_vertex(&mut self, edge: EdgeId) -> Result<()> {
        let deleted = crate::euler::kev(self.store, edge)?;
        self.lineage.push(LineageEvent::Deleted {
            entity: EntityRef::Edge(edge),
        });
        self.lineage.push(LineageEvent::Deleted {
            entity: EntityRef::Vertex(deleted.vertex),
        });
        let point_in_use = self
            .store
            .iter::<crate::entity::Vertex>()
            .any(|(_, vertex)| vertex.point == deleted.point);
        if !point_in_use {
            self.store.remove(deleted.point)?;
            self.lineage.push(LineageEvent::Deleted {
                entity: EntityRef::Point(deleted.point),
            });
        }
        Ok(())
    }

    /// KEMR: remove an edge and split its loop into outer and ring loops.
    pub fn kill_edge_make_ring(&mut self, edge: EdgeId) -> Result<LoopId> {
        let [first, _] = self.store.get(edge)?.fins[..] else {
            return Err(Error::InvalidGeometry {
                reason: "KEMR requires an edge with two fins",
            });
        };
        let source = self.store.get(first)?.parent;
        let ring = crate::euler::kemr(self.store, edge)?;
        self.lineage.push(LineageEvent::Split {
            source: EntityRef::Loop(source),
            pieces: vec![EntityRef::Loop(source), EntityRef::Loop(ring)],
        });
        Ok(ring)
    }

    /// MEKR: join an inner ring to another loop with a pcurve-bearing edge.
    #[allow(clippy::too_many_arguments)]
    pub fn make_edge_kill_ring(
        &mut self,
        outer: LoopId,
        i: usize,
        ring: LoopId,
        j: usize,
        curve: CurveId,
        bounds: (f64, f64),
        pcurves: FinPcurvePair,
    ) -> Result<Mekr> {
        let made =
            crate::euler::mekr_with_pcurves(self.store, outer, i, ring, j, curve, bounds, pcurves)?;
        self.lineage.push(LineageEvent::DerivedFrom {
            derived: EntityRef::Edge(made.edge),
            source: EntityRef::Loop(ring),
        });
        self.lineage.push(LineageEvent::Merge {
            sources: vec![EntityRef::Loop(outer), EntityRef::Loop(ring)],
            result: EntityRef::Loop(outer),
        });
        Ok(made)
    }

    /// KFMRH: absorb a one-loop face into another face as a ring hole.
    pub fn merge_face_as_hole(&mut self, keep: FaceId, kill: FaceId) -> Result<LoopId> {
        let ring = crate::euler::kfmrh(self.store, keep, kill)?;
        self.lineage.push(LineageEvent::Merge {
            sources: vec![EntityRef::Face(keep), EntityRef::Face(kill)],
            result: EntityRef::Face(keep),
        });
        Ok(ring)
    }

    /// MFKRH: detach an inner ring into a new face.
    pub fn split_hole_as_face(
        &mut self,
        ring: LoopId,
        surface: SurfaceId,
        sense: Sense,
    ) -> Result<FaceId> {
        let source = self.store.get(ring)?.face;
        let face = crate::euler::mfkrh(self.store, ring, surface, sense)?;
        self.lineage.push(LineageEvent::Split {
            source: EntityRef::Face(source),
            pieces: vec![EntityRef::Face(source), EntityRef::Face(face)],
        });
        Ok(face)
    }

    /// Split a face through the pcurve-aware MEF operator and record the
    /// persistent-naming relationship between the old face identity and
    /// its ordered old/new result pieces. The new face inherits the source's
    /// complete tolerance provenance without declaring growth, and the
    /// descriptive policy result is retained in the journal.
    #[allow(clippy::too_many_arguments)]
    pub fn split_face(
        &mut self,
        lp: LoopId,
        i: usize,
        j: usize,
        curve: CurveId,
        bounds: (f64, f64),
        surface: SurfaceId,
        sense: Sense,
        pcurves: FinPcurvePair,
    ) -> Result<Mef> {
        let source = self.store.get(lp)?.face;
        let tolerance = self.store.get(source)?.tolerance;
        let made = crate::euler::mef_with_pcurves(
            self.store, lp, i, j, curve, bounds, surface, sense, pcurves,
        )?;
        debug_assert_eq!(
            self.store
                .get(made.face)
                .expect("MEF result face remains live")
                .tolerance,
            tolerance
        );
        self.lineage.push(LineageEvent::Split {
            source: EntityRef::Face(source),
            pieces: vec![EntityRef::Face(source), EntityRef::Face(made.face)],
        });
        self.face_tolerance_propagations
            .push(FaceTolerancePropagation::Inherited {
                source,
                result: made.face,
                tolerance,
            });
        Ok(made)
    }

    /// Merge the two faces separated by `edge` through KEF and record the
    /// ordered absorbed/surviving identity relationship. The survivor keeps
    /// the larger input face tolerance (first-source on equal values) with
    /// its complete origin/growth provenance, recorded descriptively without
    /// a tolerance-growth budget.
    pub fn merge_faces(&mut self, edge: EdgeId) -> Result<()> {
        let e = self.store.get(edge)?;
        let [fin_a, fin_b] = e.fins[..] else {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "transaction merge_faces requires an edge with two fins",
            });
        };
        let face_a = self.store.get(self.store.get(fin_a)?.parent)?.face;
        let face_b = self.store.get(self.store.get(fin_b)?.parent)?.face;
        let source_tolerances = [
            self.store.get(face_a)?.tolerance,
            self.store.get(face_b)?.tolerance,
        ];
        let (selected_index, tolerance) =
            EntityTolerance::inherited_max_with_source(source_tolerances);
        let selected_source = selected_index.map(|index| [face_a, face_b][index]);
        crate::euler::kef(self.store, edge)?;
        debug_assert_eq!(
            self.store
                .get(face_a)
                .expect("KEF surviving face remains live")
                .tolerance,
            tolerance
        );
        self.lineage.push(LineageEvent::Merge {
            sources: vec![EntityRef::Face(face_a), EntityRef::Face(face_b)],
            result: EntityRef::Face(face_a),
        });
        self.face_tolerance_propagations
            .push(FaceTolerancePropagation::CombinedMax {
                sources: [face_a, face_b],
                source_tolerances,
                result: face_a,
                selected_source,
                tolerance,
            });
        Ok(())
    }

    /// Full-check every selected body and decide whether to commit atomically.
    ///
    /// This additive gate preserves [`Self::commit_checked`] as the Fast
    /// compatibility path. Full reports are returned for proof-policy
    /// acceptance or rejection; proven Fast faults and execution failures
    /// remain errors. Rejection restores model state, journals, tolerance
    /// usage, the committed dependency index, and future handle allocation.
    pub fn commit_full(
        self,
        bodies: &[BodyId],
        requirement: FullCommitRequirement,
    ) -> Result<FullCommitDecision> {
        let session = kcore::operation::SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default())
            .expect("built-in Full-commit context is valid");
        self.commit_full_with_context(bodies, requirement, &context)
            .expect("built-in Full-check budget is valid")
            .into_result()
    }

    /// Contextual Full-check commit retaining exact operation accounting.
    pub fn commit_full_with_context(
        self,
        bodies: &[BodyId],
        requirement: FullCommitRequirement,
        context: &OperationContext<'_>,
    ) -> core::result::Result<OperationOutcome<FullCommitDecision>, OperationPolicyError> {
        let context = context.clone().with_family_budget_defaults(
            crate::check::CheckBudgetProfile::v1_defaults(crate::check::CheckLevel::Full),
        );
        EvalLimits::from_budget_plan(&context.effective_budget())?;
        crate::check::validate_full_check_budget(&context)?;
        let mut scope = OperationScope::new(&context);
        let result = self.commit_full_in_scope(bodies, requirement, &mut scope, 0);
        Ok(scope.finish(result))
    }

    /// Full-assurance commit inside one caller-owned operation scope.
    ///
    /// Fast graph validation and every Full proof borrow the same scope. The
    /// graph child is merged before persistence, so accounting denial cannot
    /// follow a committed model change.
    pub fn commit_full_in_scope(
        mut self,
        bodies: &[BodyId],
        requirement: FullCommitRequirement,
        scope: &mut OperationScope<'_, '_>,
        child_ordinal: u64,
    ) -> Result<FullCommitDecision> {
        let pending = self.store.pending_transaction_mutations()?;
        let validate_all = self.store.full_validation_required();
        #[cfg(feature = "benchmark-internals")]
        let mut phase_observation = CommitPhaseObservation::default();
        #[cfg(feature = "benchmark-internals")]
        let (candidate_index, refreshed_bodies) = if validate_all {
            (
                crate::index::StoreIndex::build(self.store),
                self.store.count::<crate::entity::Body>(),
            )
        } else {
            let (candidate, refreshed, observation) =
                crate::index::StoreIndex::candidate_with_benchmark_observation(
                    self.store,
                    self.store.committed_index(),
                    &pending,
                );
            phase_observation.index = observation;
            (candidate, refreshed)
        };
        #[cfg(not(feature = "benchmark-internals"))]
        let candidate_index = if validate_all {
            crate::index::StoreIndex::build(self.store)
        } else {
            crate::index::StoreIndex::candidate(self.store, self.store.committed_index(), &pending)
        };
        candidate_index.debug_assert_full_rebuild_parity(self.store);
        #[cfg(feature = "benchmark-internals")]
        phase_observation.index.observe_affected_selection(&pending);
        let affected = candidate_index.affected_bodies(self.store.committed_index(), &pending);

        #[cfg(feature = "benchmark-internals")]
        {
            let (validation, observation) = self.store.validate_geometry_with_observation();
            phase_observation.observe_geometry_validation(observation);
            validation?;
        }
        #[cfg(not(feature = "benchmark-internals"))]
        self.store.validate_geometry()?;
        if candidate_index.ownership_fault_count() != 0 {
            return Err(Error::TopologyCheckFailed {
                fault_count: candidate_index.ownership_fault_count(),
            });
        }

        let mut checked = Vec::new();
        for &body in bodies {
            if !checked.contains(&body) {
                checked.push(body);
            }
        }
        for body in affected.iter().copied() {
            if !checked.contains(&body) && self.store.contains(body) {
                checked.push(body);
            }
        }
        if validate_all {
            for (body, _) in self.store.iter::<crate::entity::Body>() {
                if !checked.contains(&body) {
                    checked.push(body);
                }
            }
        }

        let mut graph = GraphQueryWork::reserve(scope, child_ordinal).map_err(Error::from)?;
        let mut fast_reports = Vec::with_capacity(checked.len());
        let mut fault_count = 0;
        let fast_result: Result<()> = (|| {
            for &body in &checked {
                let body_value = self.store.get(body)?;
                #[cfg(feature = "benchmark-internals")]
                {
                    phase_observation.fast_body_check_starts += 1;
                }
                let report = crate::check::check_body_fast_report_with_graph(
                    self.store, body, body_value, &mut graph,
                )?;
                fault_count += report.faults.len();
                fast_reports.push((body, report));
            }
            Ok(())
        })();
        let accounting = graph.merge(scope).map_err(Error::from);
        fast_result?;
        accounting?;
        if fault_count != 0 {
            return Err(Error::TopologyCheckFailed { fault_count });
        }

        let mut checks = Vec::with_capacity(fast_reports.len());
        for (body, fast_report) in fast_reports {
            let body_value = self.store.get(body)?;
            let report = crate::check::complete_full_report_in_scope(
                self.store,
                body,
                body_value,
                fast_report,
                scope,
            )?;
            checks.push(FullBodyCheck::new(self.store, body, report)?);
        }

        let rejected = checks.iter().any(|check| {
            !check.report.faults.is_empty()
                || (requirement == FullCommitRequirement::RequireValid
                    && !check.report.gaps.is_empty())
        });
        if rejected {
            self.store.rollback_transaction()?;
            #[cfg(feature = "benchmark-internals")]
            self.store
                .set_benchmark_observation(phase_observation.finish(
                    self.store,
                    false,
                    &affected,
                    refreshed_bodies,
                    checked.len(),
                    pending.len(),
                ));
            self.finished = true;
            return Ok(FullCommitDecision::rejected(checks));
        }

        let mutations = self.store.commit_transaction()?;
        debug_assert_eq!(mutations, pending);
        self.store.install_committed_index(candidate_index);
        #[cfg(feature = "benchmark-internals")]
        self.store
            .set_benchmark_observation(phase_observation.finish(
                self.store,
                true,
                &affected,
                refreshed_bodies,
                checked.len(),
                pending.len(),
            ));
        self.finished = true;
        let journal = Journal::new(
            mutations,
            core::mem::take(&mut self.lineage),
            core::mem::take(&mut self.tolerance_budgets),
            core::mem::take(&mut self.tolerance_events),
            core::mem::take(&mut self.face_tolerance_propagations),
        );
        Ok(FullCommitDecision::committed(journal, checks))
    }

    /// Validate every affected body and the complete topology ownership
    /// closure, then commit.
    ///
    /// `bodies` supplies the expected result roots and their preferred
    /// validation order. Deterministic pending mutations are resolved through
    /// both the committed and candidate ownership/dependency indexes, so moved
    /// topology and shared geometry validate every old and new dependent body.
    /// The candidate index also audits the complete store-wide ownership
    /// closure, so low-level assembly cannot hide an invalid unlisted body or
    /// orphan topology. Any fault or validation error rolls the entire
    /// transaction back.
    pub fn commit_checked(self, bodies: &[BodyId]) -> Result<Journal> {
        let session = kcore::operation::SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default())
            .expect("built-in checked-commit context is valid")
            .with_budget_overrides(EvalBudgetProfile::for_limits(
                EvalLimits::default().max_dependency_depth,
                usize::MAX,
            ));
        self.commit_checked_with_context(bodies, &context)
            .expect("built-in checked-commit graph budget is valid")
            .into_result()
    }

    /// Validate and commit while retaining graph-query accounting.
    pub fn commit_checked_with_context(
        self,
        bodies: &[BodyId],
        context: &OperationContext<'_>,
    ) -> core::result::Result<OperationOutcome<Journal>, OperationPolicyError> {
        let context = context
            .clone()
            .with_family_budget_defaults(EvalBudgetProfile::v1_defaults());
        EvalLimits::from_budget_plan(&context.effective_budget())?;
        let mut scope = OperationScope::new(&context);
        let result = self.commit_checked_in_scope(bodies, &mut scope, 0);
        Ok(scope.finish(result))
    }

    /// Validate and commit inside one caller-owned operation scope.
    ///
    /// `child_ordinal` is stable within the caller's current reservation set.
    /// This nested seam never installs defaults or resets accounting.
    pub fn commit_checked_in_scope(
        self,
        bodies: &[BodyId],
        scope: &mut OperationScope<'_, '_>,
        child_ordinal: u64,
    ) -> Result<Journal> {
        let mut graph = GraphQueryWork::reserve(scope, child_ordinal).map_err(Error::from)?;
        let result = self.commit_checked_with_graph(bodies, &mut graph);
        let accounting = graph.merge(scope).map_err(Error::from);
        match (result, accounting) {
            (Err(error), _) => Err(error),
            (Ok(_), Err(error)) => Err(error),
            (Ok(journal), Ok(())) => Ok(journal),
        }
    }

    /// Checked commit using an already-reserved graph child.
    ///
    /// Reviewed compound operations use this seam when graph work before and
    /// during Fast validation must share one indivisible aggregate allowance.
    #[doc(hidden)]
    pub fn commit_checked_with_graph(
        mut self,
        bodies: &[BodyId],
        graph: &mut GraphQueryWork,
    ) -> Result<Journal> {
        let pending = match self.store.pending_transaction_mutations() {
            Ok(pending) => pending,
            Err(error) => {
                self.store.rollback_transaction()?;
                self.finished = true;
                return Err(error);
            }
        };
        let validate_all = self.store.full_validation_required();
        #[cfg(feature = "benchmark-internals")]
        let mut phase_observation = CommitPhaseObservation::default();
        #[cfg(feature = "benchmark-internals")]
        let (candidate_index, refreshed_bodies) = if validate_all {
            (
                crate::index::StoreIndex::build(self.store),
                self.store.count::<crate::entity::Body>(),
            )
        } else {
            let (candidate, refreshed, observation) =
                crate::index::StoreIndex::candidate_with_benchmark_observation(
                    self.store,
                    self.store.committed_index(),
                    &pending,
                );
            phase_observation.index = observation;
            (candidate, refreshed)
        };
        #[cfg(not(feature = "benchmark-internals"))]
        let candidate_index = if validate_all {
            crate::index::StoreIndex::build(self.store)
        } else {
            crate::index::StoreIndex::candidate(self.store, self.store.committed_index(), &pending)
        };
        candidate_index.debug_assert_full_rebuild_parity(self.store);
        #[cfg(feature = "benchmark-internals")]
        phase_observation.index.observe_affected_selection(&pending);
        let affected = candidate_index.affected_bodies(self.store.committed_index(), &pending);
        let mut checked = Vec::new();
        let validation = (|| {
            #[cfg(feature = "benchmark-internals")]
            {
                let (validation, observation) = self.store.validate_geometry_with_observation();
                phase_observation.observe_geometry_validation(observation);
                validation?;
            }
            #[cfg(not(feature = "benchmark-internals"))]
            self.store.validate_geometry()?;
            let mut fault_count = candidate_index.ownership_fault_count();
            for &body in bodies {
                if checked.contains(&body) {
                    continue;
                }
                checked.push(body);
                let body_value = self.store.get(body)?;
                #[cfg(feature = "benchmark-internals")]
                {
                    phase_observation.fast_body_check_starts += 1;
                }
                fault_count += crate::check::check_body_fast_report_with_graph(
                    self.store, body, body_value, graph,
                )?
                .faults
                .len();
            }
            for body in affected.iter().copied() {
                if checked.contains(&body) || !self.store.contains(body) {
                    continue;
                }
                checked.push(body);
                let body_value = self.store.get(body)?;
                #[cfg(feature = "benchmark-internals")]
                {
                    phase_observation.fast_body_check_starts += 1;
                }
                fault_count += crate::check::check_body_fast_report_with_graph(
                    self.store, body, body_value, graph,
                )?
                .faults
                .len();
            }
            if validate_all {
                for (body, _) in self.store.iter::<crate::entity::Body>() {
                    if checked.contains(&body) {
                        continue;
                    }
                    checked.push(body);
                    #[cfg(feature = "benchmark-internals")]
                    {
                        phase_observation.fast_body_check_starts += 1;
                    }
                    fault_count += crate::check::check_body_fast_report_with_graph(
                        self.store,
                        body,
                        self.store.get(body)?,
                        graph,
                    )?
                    .faults
                    .len();
                }
            }
            if fault_count == 0 {
                Ok(())
            } else {
                Err(kcore::error::Error::TopologyCheckFailed { fault_count })
            }
        })();
        if let Err(error) = validation {
            self.store.rollback_transaction()?;
            #[cfg(feature = "benchmark-internals")]
            self.store
                .set_benchmark_observation(phase_observation.finish(
                    self.store,
                    false,
                    &affected,
                    refreshed_bodies,
                    checked.len(),
                    pending.len(),
                ));
            self.finished = true;
            return Err(error);
        }
        let mutations = self.store.commit_transaction()?;
        debug_assert_eq!(mutations, pending);
        self.store.install_committed_index(candidate_index);
        #[cfg(feature = "benchmark-internals")]
        self.store
            .set_benchmark_observation(phase_observation.finish(
                self.store,
                true,
                &affected,
                refreshed_bodies,
                checked.len(),
                pending.len(),
            ));
        self.finished = true;
        Ok(Journal::new(
            mutations,
            core::mem::take(&mut self.lineage),
            core::mem::take(&mut self.tolerance_budgets),
            core::mem::take(&mut self.tolerance_events),
            core::mem::take(&mut self.face_tolerance_propagations),
        ))
    }

    /// Validate one result body and commit atomically.
    pub fn commit_checked_body(self, body: BodyId) -> Result<Journal> {
        self.commit_checked(&[body])
    }

    /// Contextual one-body checked commit retaining the operation report.
    pub fn commit_checked_body_with_context(
        self,
        body: BodyId,
        context: &OperationContext<'_>,
    ) -> core::result::Result<OperationOutcome<Journal>, OperationPolicyError> {
        self.commit_checked_with_context(&[body], context)
    }

    /// Nested contextual one-body checked commit.
    pub fn commit_checked_body_in_scope(
        self,
        body: BodyId,
        scope: &mut OperationScope<'_, '_>,
        child_ordinal: u64,
    ) -> Result<Journal> {
        self.commit_checked_in_scope(&[body], scope, child_ordinal)
    }

    /// Explicitly restore the transaction entry state. Dropping without
    /// commit has the same effect.
    pub fn rollback(mut self) -> Result<()> {
        self.store.rollback_transaction()?;
        self.finished = true;
        Ok(())
    }
}

impl Drop for Transaction<'_> {
    fn drop(&mut self) {
        if !self.finished {
            let _ = self.store.rollback_transaction();
            self.finished = true;
        }
    }
}
