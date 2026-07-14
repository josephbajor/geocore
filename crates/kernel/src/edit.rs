//! Checked semantic topology edits at the supported facade boundary.

use kcore::operation::OperationContext;
use ktopo::entity::{FinPcurve, ParamMap1d};
use ktopo::euler::FinPcurvePair;
use ktopo::transaction::Transaction;

use crate::error::{Error, Result};
use crate::session::PartEdit;
use crate::{
    BodyId, BoundedCurve, ChangeJournal, CurveId, EdgeId, EntityKind, FaceId, FinId, LoopId,
    OperationOutcome, OperationSettings, ParamRange, PartId, PcurveId,
};

/// One existing pcurve restricted to a finite parameter interval.
///
/// The first semantic split slice requires the edge-to-pcurve map to be the
/// identity. Period-shifted, reversed, singular-endpoint, and closed pcurve
/// uses remain lower-layer operations until facade-owned value contracts are
/// added for those meanings.
#[derive(Debug, Clone, PartialEq)]
pub struct BoundedPcurve {
    pcurve: PcurveId,
    range: ParamRange,
}

impl BoundedPcurve {
    /// Bind an opaque pcurve identity to its active finite interval.
    pub const fn new(pcurve: PcurveId, range: ParamRange) -> Self {
        Self { pcurve, range }
    }

    /// Exact graph-owned pcurve identity.
    pub fn pcurve(&self) -> PcurveId {
        self.pcurve.clone()
    }

    /// Active pcurve interval.
    pub const fn range(&self) -> ParamRange {
        self.range
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
    /// Construct one canonical pcurve-aware face split request.
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
            FinPcurve::new(forward.pcurve.raw(), forward.range, ParamMap1d::identity())?,
            FinPcurve::new(
                reversed.pcurve.raw(),
                reversed.range,
                ParamMap1d::identity(),
            )?,
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
    use kgeom::curve::Line;
    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::vec::Point2;
    use kgraph::eval_stage;
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
        let mut make_pcurve = || {
            store
                .insert_pcurve(Curve2dGeom::Line(
                    Line2d::new(uv_start, uv_end - uv_start).unwrap(),
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
                BoundedPcurve::new(PcurveId::new(part.clone(), forward), range),
                BoundedPcurve::new(PcurveId::new(part, reversed), range),
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
