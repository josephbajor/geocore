//! Failure-atomic modeling transactions and deterministic journals.
//!
//! A [`Transaction`] opens copy-on-write undo frames on every [`Store`]
//! arena. Existing Euler operations can run against [`Transaction::store_mut`]
//! without learning rollback mechanics. Committing produces raw net mutations
//! in a stable entity-type/slot order plus semantic lineage recorded by the
//! higher-level operation. Journals also retain declared tolerance-growth
//! budgets and ordered per-entity changes. Dropping or explicitly rolling back
//! restores entity contents, handle generations, free-list order, and future
//! allocations while discarding uncommitted budget usage.

use crate::entity::{
    BodyId, CurveId, EdgeId, EntityRef, FaceId, LoopId, Sense, SurfaceId, VertexId,
};
use crate::euler::{FinPcurvePair, Mef};
use crate::store::Store;
use crate::tolerance::EntityTolerance;
use kcore::error::{Error, Result};
use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances};

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
}

/// Deterministic committed transaction record.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Journal {
    mutations: Vec<Mutation>,
    lineage: Vec<LineageEvent>,
    tolerance_budgets: Vec<ToleranceBudgetReport>,
    tolerance_events: Vec<ToleranceEvent>,
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
/// Transactions are rollback-on-drop. Call [`Self::commit`] exactly once to
/// retain changes and obtain their journal. Nested transactions on one store
/// are currently rejected so semantic journal composition cannot be
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

    /// Mutate the in-progress store using checked topology/Euler operations.
    pub fn store_mut(&mut self) -> &mut Store {
        self.store
    }

    /// Record a semantic lineage event in deterministic operation order.
    pub fn record_lineage(&mut self, event: LineageEvent) {
        self.lineage.push(event);
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

    /// Commit all changes and return their raw and semantic journal.
    pub fn commit(mut self) -> Result<Journal> {
        let mutations = self.store.commit_transaction()?;
        self.finished = true;
        Ok(Journal::new(
            mutations,
            core::mem::take(&mut self.lineage),
            core::mem::take(&mut self.tolerance_budgets),
            core::mem::take(&mut self.tolerance_events),
        ))
    }

    /// Validate every listed body with the Fast checker, then commit.
    ///
    /// Any checker fault or validation error rolls the entire transaction
    /// back before returning. Duplicate body handles are checked once in
    /// first-occurrence order.
    pub fn commit_checked(mut self, bodies: &[BodyId]) -> Result<Journal> {
        let mut checked = Vec::new();
        let validation = (|| {
            let mut fault_count = 0usize;
            for &body in bodies {
                if checked.contains(&body) {
                    continue;
                }
                checked.push(body);
                fault_count += crate::check::check_body(self.store, body)?.len();
            }
            if fault_count == 0 {
                Ok(())
            } else {
                Err(kcore::error::Error::TopologyCheckFailed { fault_count })
            }
        })();
        if let Err(error) = validation {
            self.store.rollback_transaction()?;
            self.finished = true;
            return Err(error);
        }
        let mutations = self.store.commit_transaction()?;
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
