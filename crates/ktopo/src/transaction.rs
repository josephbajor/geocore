//! Failure-atomic modeling transactions and deterministic journals.
//!
//! A [`Transaction`] opens copy-on-write undo frames on every [`Store`]
//! arena. Its public methods own Euler mutation, pcurve preflight, semantic
//! lineage, and tolerance growth without exposing raw operators. Committing
//! produces raw net mutations in a stable entity-type/slot order plus semantic
//! operation evidence. Journals also retain declared tolerance-growth
//! budgets and ordered per-entity changes. Dropping or explicitly rolling back
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

    pub(crate) fn new(
        mutations: Vec<Mutation>,
        lineage: Vec<LineageEvent>,
        tolerance_budgets: Vec<ToleranceBudgetReport>,
        tolerance_events: Vec<ToleranceEvent>,
    ) -> Self {
        Self {
            mutations,
            lineage,
            tolerance_budgets,
            tolerance_events,
        }
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
    finished: bool,
}

impl<'a> Transaction<'a> {
    pub(crate) fn new(store: &'a mut Store) -> Self {
        Self {
            store,
            lineage: Vec::new(),
            tolerance_budgets: Vec::new(),
            tolerance_events: Vec::new(),
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

    /// Open the unstable low-level reconstruction/assembly seam for this
    /// transaction.
    ///
    /// This is reserved for reviewed kernel builders and trusted interchange
    /// adapters. Ordinary modeling uses semantic transaction operations.
    pub fn assembly(&mut self) -> AssemblyStore<'_> {
        AssemblyStore { store: self.store }
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

    /// KVFS: remove a body that is still in minimal MVFS form.
    pub fn kill_minimal_body(&mut self, body: BodyId) -> Result<()> {
        crate::euler::kvfs(self.store, body)?;
        self.lineage.push(LineageEvent::Deleted {
            entity: EntityRef::Body(body),
        });
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

    /// KEV: remove a strut edge and its otherwise-unused vertex.
    pub fn kill_edge_vertex(&mut self, edge: EdgeId) -> Result<()> {
        let deleted_vertex = crate::euler::kev(self.store, edge)?;
        self.lineage.push(LineageEvent::Deleted {
            entity: EntityRef::Edge(edge),
        });
        self.lineage.push(LineageEvent::Deleted {
            entity: EntityRef::Vertex(deleted_vertex),
        });
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
    /// its ordered old/new result pieces.
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
        let made = crate::euler::mef_with_pcurves(
            self.store, lp, i, j, curve, bounds, surface, sense, pcurves,
        )?;
        self.lineage.push(LineageEvent::Split {
            source: EntityRef::Face(source),
            pieces: vec![EntityRef::Face(source), EntityRef::Face(made.face)],
        });
        Ok(made)
    }

    /// Merge the two faces separated by `edge` through KEF and record the
    /// ordered absorbed/surviving identity relationship.
    pub fn merge_faces(&mut self, edge: EdgeId) -> Result<()> {
        let e = self.store.get(edge)?;
        let [fin_a, fin_b] = e.fins[..] else {
            return Err(kcore::error::Error::InvalidGeometry {
                reason: "transaction merge_faces requires an edge with two fins",
            });
        };
        let face_a = self.store.get(self.store.get(fin_a)?.parent)?.face;
        let face_b = self.store.get(self.store.get(fin_b)?.parent)?.face;
        crate::euler::kef(self.store, edge)?;
        self.lineage.push(LineageEvent::Merge {
            sources: vec![EntityRef::Face(face_a), EntityRef::Face(face_b)],
            result: EntityRef::Face(face_a),
        });
        Ok(())
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
        let (candidate_index, refreshed_bodies) = if validate_all {
            (
                crate::index::StoreIndex::build(self.store),
                self.store.count::<crate::entity::Body>(),
            )
        } else {
            crate::index::StoreIndex::candidate_with_stats(
                self.store,
                self.store.committed_index(),
                &pending,
            )
        };
        #[cfg(not(feature = "benchmark-internals"))]
        let candidate_index = if validate_all {
            crate::index::StoreIndex::build(self.store)
        } else {
            crate::index::StoreIndex::candidate(self.store, self.store.committed_index(), &pending)
        };
        candidate_index.debug_assert_full_rebuild_parity(self.store);
        let affected = candidate_index.affected_bodies(self.store.committed_index(), &pending);
        let mut checked = Vec::new();
        let validation = (|| {
            self.store.validate_geometry()?;
            let mut fault_count = candidate_index.ownership_fault_count();
            for &body in bodies {
                if checked.contains(&body) {
                    continue;
                }
                checked.push(body);
                let body_value = self.store.get(body)?;
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
                .set_benchmark_observation(crate::benchmark::CommitObservation {
                    committed: false,
                    body_count: self.store.count::<crate::entity::Body>(),
                    affected_bodies: affected.len(),
                    refreshed_bodies,
                    checked_bodies: checked.len(),
                    mutations: pending.len(),
                    affected_order_digest: crate::benchmark::affected_digest(self.store, &affected),
                });
            self.finished = true;
            return Err(error);
        }
        let mutations = self.store.commit_transaction()?;
        debug_assert_eq!(mutations, pending);
        self.store.install_committed_index(candidate_index);
        #[cfg(feature = "benchmark-internals")]
        self.store
            .set_benchmark_observation(crate::benchmark::CommitObservation {
                committed: true,
                body_count: self.store.count::<crate::entity::Body>(),
                affected_bodies: affected.len(),
                refreshed_bodies,
                checked_bodies: checked.len(),
                mutations: pending.len(),
                affected_order_digest: crate::benchmark::affected_digest(self.store, &affected),
            });
        self.finished = true;
        Ok(Journal::new(
            mutations,
            core::mem::take(&mut self.lineage),
            core::mem::take(&mut self.tolerance_budgets),
            core::mem::take(&mut self.tolerance_events),
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
