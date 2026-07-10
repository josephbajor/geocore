//! Failure-atomic modeling transactions and deterministic journals.
//!
//! A [`Transaction`] opens copy-on-write undo frames on every [`Store`]
//! arena. Existing Euler operations can run against [`Transaction::store_mut`]
//! without learning rollback mechanics. Committing produces raw net mutations
//! in a stable entity-type/slot order plus semantic lineage recorded by the
//! higher-level operation. Dropping or explicitly rolling back restores entity
//! contents, handle generations, free-list order, and future allocations.

use crate::entity::{BodyId, CurveId, EdgeId, EntityRef, LoopId, Sense, SurfaceId};
use crate::euler::{FinPcurvePair, Mef};
use crate::store::Store;
use kcore::error::Result;

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
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Journal {
    mutations: Vec<Mutation>,
    lineage: Vec<LineageEvent>,
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

    pub(crate) fn new(mutations: Vec<Mutation>, lineage: Vec<LineageEvent>) -> Self {
        Self { mutations, lineage }
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
    finished: bool,
}

impl<'a> Transaction<'a> {
    pub(crate) fn new(store: &'a mut Store) -> Self {
        Self {
            store,
            lineage: Vec::new(),
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
        Ok(Journal::new(mutations, core::mem::take(&mut self.lineage)))
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
        Ok(Journal::new(mutations, core::mem::take(&mut self.lineage)))
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
