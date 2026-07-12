//! Typed X_T interchange at the supported facade boundary.

use kcore::operation::OperationScope;
use kgraph::EvalLimits;

use crate::error::{Error, Result};
use crate::session::{Part, PartEdit};
use crate::{BodyId, ChangeJournal, OperationOutcome, OperationSettings, PartId};

/// Request to parse and atomically reconstruct one X_T transmit file.
#[derive(Debug, Clone, PartialEq)]
pub struct ImportXtRequest<'bytes> {
    bytes: &'bytes [u8],
    settings: OperationSettings,
}

impl<'bytes> ImportXtRequest<'bytes> {
    /// Construct an import request with default operation settings.
    pub fn new(bytes: &'bytes [u8]) -> Self {
        Self {
            bytes,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Source transmit bytes.
    pub const fn bytes(&self) -> &'bytes [u8] {
        self.bytes
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// One intentionally ignored X_T schema node type and its occurrence count.
///
/// `node_type_code` is a schema type code, not a transport node index.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XtSkippedNode {
    node_type_code: u16,
    count: usize,
}

impl XtSkippedNode {
    /// X_T schema node-type code.
    pub const fn node_type_code(self) -> u16 {
        self.node_type_code
    }

    /// Number of deliberately skipped nodes of this type.
    pub const fn count(self) -> usize {
        self.count
    }
}

/// Successful atomic X_T reconstruction.
#[derive(Debug)]
pub struct ImportXtResult {
    bodies: Vec<BodyId>,
    skipped: Vec<XtSkippedNode>,
    journal: ChangeJournal,
}

impl ImportXtResult {
    /// Bodies created by the file in deterministic file order.
    pub fn bodies(&self) -> &[BodyId] {
        &self.bodies
    }

    /// Intentionally ignored non-model schema nodes, ordered by type code.
    pub fn skipped(&self) -> &[XtSkippedNode] {
        &self.skipped
    }

    /// Exact committed reconstruction journal behind a semantic facade view.
    pub const fn journal(&self) -> &ChangeJournal {
        &self.journal
    }

    /// Consume the result into its opaque identities, skipped summaries, and
    /// committed journal.
    pub fn into_parts(self) -> (Vec<BodyId>, Vec<XtSkippedNode>, ChangeJournal) {
        (self.bodies, self.skipped, self.journal)
    }
}

/// Request to deterministically emit one body as text X_T.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportXtRequest {
    body: BodyId,
    settings: OperationSettings,
}

impl ExportXtRequest {
    /// Construct an export request with default operation settings.
    pub fn new(body: BodyId) -> Self {
        Self {
            body,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Body to export.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Deterministic text X_T emission.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExportXtResult {
    text: String,
}

impl ExportXtResult {
    /// Borrow the emitted text.
    pub fn text(&self) -> &str {
        &self.text
    }

    /// Borrow the exact emitted UTF-8 bytes.
    pub fn bytes(&self) -> &[u8] {
        self.text.as_bytes()
    }

    /// Consume the result into the emitted text.
    pub fn into_text(self) -> String {
        self.text
    }
}

impl PartEdit<'_> {
    /// Parse and checked-commit X_T content atomically into this part.
    ///
    /// Context construction is completed before lower work. Once dispatched,
    /// `kxt` owns one checked transaction; every parse, graph, topology, or
    /// checking failure rolls that transaction back before the error and exact
    /// facade operation report are returned.
    pub fn import_xt(
        &mut self,
        request: ImportXtRequest<'_>,
    ) -> Result<OperationOutcome<ImportXtResult>> {
        let ImportXtRequest { bytes, settings } = request;
        let context = settings
            .context(self.policy)?
            .with_family_budget_defaults(kxt::reconstruction_budget_profile());
        EvalLimits::from_budget_plan(&context.effective_budget())?;
        let mut scope = OperationScope::new(&context);
        let part = self.id.clone();
        let result = kxt::import_in_scope(bytes, &mut self.state.store, &mut scope, 0)
            .map(|reconstruction| adapt_import(part, reconstruction))
            .map_err(Error::from_xt);
        Ok(scope.finish_typed(result))
    }
}

impl Part<'_> {
    /// Deterministically export one live body as text X_T.
    ///
    /// Wrong-part and stale identities are rejected before contextual policy
    /// construction or writer work.
    pub fn export_xt(&self, request: ExportXtRequest) -> Result<OperationOutcome<ExportXtResult>> {
        let ExportXtRequest { body, settings } = request;
        self.body(body.clone())?;
        let context = settings.context(self.policy)?;
        let scope = OperationScope::new(&context);
        let result = kxt::export_text(&self.state.store, body.raw())
            .map(|text| ExportXtResult { text })
            .map_err(Error::from_xt);
        Ok(scope.finish_typed(result))
    }
}

fn adapt_import(part: PartId, reconstruction: kxt::Reconstruction) -> ImportXtResult {
    let kxt::Reconstruction {
        bodies,
        skipped,
        journal,
    } = reconstruction;
    ImportXtResult {
        bodies: bodies
            .into_iter()
            .map(|body| BodyId::new(part.clone(), body))
            .collect(),
        skipped: skipped
            .into_iter()
            .map(|(node_type_code, count)| XtSkippedNode {
                node_type_code,
                count,
            })
            .collect(),
        journal: ChangeJournal::from_raw(part, journal),
    }
}

#[cfg(test)]
mod tests {
    use std::error::Error as _;

    use kcore::error::ErrorClass;
    use kcore::operation::{
        ExecutionPolicy, LimitSnapshot, NumericalPolicy, PolicyVersion, ResourceKind,
        SessionPrecision, TOTAL_WORK_STAGE,
    };
    use kgeom::frame::Frame;
    use kgeom::surface::Plane;
    use kgeom::vec::Point3;
    use ktopo::check::CheckLevel;
    use ktopo::entity::{Body as RawBody, Edge as RawEdge, Face as RawFace, Vertex as RawVertex};
    use ktopo::geom::SurfaceGeom;
    use ktopo::store::Store;

    use super::*;
    use crate::{
        BlockRequest, CheckBodyRequest, EntityKind, Kernel, KernelError, SessionPolicy,
        XtInterchangeError,
    };

    const OLD_SCHEMA_XT: &[u8] = b"\
**ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz**************************\n\
**PARASOLID !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~0123456789**************************\n\
**PART1;FORMAT=text;\n\
**PART2;SCH=SCH_1000230_10004;USFLD_SIZE=0;\n\
**PART3;\n\
**END_OF_HEADER*****************************************************************\n\
T51 : TRANSMIT FILE created by modeller version 100023017 SCH_1000230_100040";

    fn block_xt() -> String {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [0.1, 0.15, 0.2]).unwrap();
        kxt::export_text(&store, body).unwrap()
    }

    fn invalid_late_block() -> Vec<u8> {
        let mut text = block_xt();
        let start = text.rfind("-0.05").expect("last block point is stable");
        text.replace_range(start..start + "-0.05".len(), "501.0");
        text.into_bytes()
    }

    fn assert_store_shape(part: &Part<'_>, direct: &Store) {
        assert_eq!(part.bodies().len(), direct.count::<RawBody>());
        assert_eq!(part.faces().len(), direct.count::<RawFace>());
        assert_eq!(part.edges().len(), direct.count::<RawEdge>());
        assert_eq!(part.vertices().len(), direct.count::<RawVertex>());
        assert_eq!(part.curves().len(), direct.geometry().curve_count());
        assert_eq!(part.surfaces().len(), direct.geometry().surface_count());
        assert_eq!(part.pcurves().len(), direct.geometry().curve2d_count());
    }

    fn report_snapshot(
        outcome: &OperationOutcome<ImportXtResult>,
        stage: crate::StageId,
        resource: ResourceKind,
    ) -> LimitSnapshot {
        outcome
            .report()
            .usage()
            .iter()
            .copied()
            .find(|snapshot| snapshot.stage == stage && snapshot.resource == resource)
            .expect("configured usage snapshot")
    }

    #[test]
    fn import_into_existing_part_matches_direct_bodies_journal_and_semantics() {
        let block_xt = block_xt();
        let mut direct = Store::new();
        let direct_existing =
            ktopo::make::block(&mut direct, &Frame::world(), [1.0, 2.0, 3.0]).unwrap();
        let direct_policy = kcore::operation::SessionPolicy::v1();
        let direct_context = kcore::operation::OperationContext::new(
            &direct_policy,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap();
        let direct_outcome =
            kxt::import_with_context(block_xt.as_bytes(), &mut direct, &direct_context).unwrap();
        let direct_report = direct_outcome.report().clone();
        let direct_import = direct_outcome.into_result().unwrap();

        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let existing = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 2.0, 3.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        assert_eq!(existing.raw(), direct_existing);
        let facade = session
            .edit_part(part_id.clone())
            .unwrap()
            .import_xt(ImportXtRequest::new(block_xt.as_bytes()))
            .unwrap();
        assert_eq!(facade.report(), &direct_report);
        assert_eq!(
            report_snapshot(&facade, kgraph::eval_stage::NODE_VISITS, ResourceKind::Work,),
            LimitSnapshot {
                stage: kgraph::eval_stage::NODE_VISITS,
                resource: ResourceKind::Work,
                consumed: 30,
                allowed: 4_096,
            }
        );
        assert_eq!(
            report_snapshot(
                &facade,
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
            )
            .consumed,
            1
        );
        let imported = facade.result().unwrap();
        assert_eq!(imported.bodies().len(), direct_import.bodies.len());
        for (facade, direct) in imported.bodies().iter().zip(&direct_import.bodies) {
            assert_eq!(facade.raw(), *direct);
        }
        assert_eq!(imported.journal().raw_for_test(), &direct_import.journal);
        assert_eq!(
            imported
                .skipped()
                .iter()
                .map(|entry| (entry.node_type_code(), entry.count()))
                .collect::<Vec<_>>(),
            direct_import.skipped
        );
        assert_store_shape(&session.part(part_id).unwrap(), &direct);
    }

    #[test]
    fn export_is_directly_equal_byte_stable_and_round_trips_through_the_facade() {
        let block_xt = block_xt();
        let mut session = Kernel::new().create_session();
        let source_part = session.create_part();
        let body = session
            .edit_part(source_part.clone())
            .unwrap()
            .import_xt(ImportXtRequest::new(block_xt.as_bytes()))
            .unwrap()
            .into_result()
            .unwrap()
            .bodies()[0]
            .clone();
        let direct = {
            let part = session.part(source_part.clone()).unwrap();
            kxt::export_text(&part.state.store, body.raw()).unwrap()
        };
        let first = session
            .part(source_part.clone())
            .unwrap()
            .export_xt(ExportXtRequest::new(body.clone()))
            .unwrap()
            .into_result()
            .unwrap();
        let second = session
            .part(source_part)
            .unwrap()
            .export_xt(ExportXtRequest::new(body))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(first.text(), direct);
        assert_eq!(first.bytes(), second.bytes());

        let destination = session.create_part();
        let roundtrip = session
            .edit_part(destination.clone())
            .unwrap()
            .import_xt(ImportXtRequest::new(first.bytes()))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(roundtrip.bodies().len(), 1);
        let check = session
            .part(destination)
            .unwrap()
            .check_body(CheckBodyRequest::new(
                roundtrip.bodies()[0].clone(),
                CheckLevel::Fast,
            ))
            .unwrap();
        assert!(check.result().unwrap().faults().is_empty());
    }

    #[test]
    fn skipped_nodes_and_unsupported_capabilities_remain_facade_safe() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let imported = adapt_import(
            part_id.clone(),
            kxt::Reconstruction {
                bodies: Vec::new(),
                skipped: vec![(30, 2), (88, 1)],
                journal: ktopo::transaction::Journal::default(),
            },
        );
        assert!(imported.bodies().is_empty());
        assert_eq!(imported.journal().mutation_count(), 0);
        assert_eq!(
            imported
                .skipped()
                .iter()
                .map(|entry| (entry.node_type_code(), entry.count()))
                .collect::<Vec<_>>(),
            vec![(30, 2), (88, 1)]
        );

        let bodies_before = session.part(part_id.clone()).unwrap().bodies().len();
        let failure = session
            .edit_part(part_id)
            .unwrap()
            .import_xt(ImportXtRequest::new(OLD_SCHEMA_XT))
            .unwrap();
        let error = failure.result().unwrap_err();
        assert_eq!(error.class(), ErrorClass::Unsupported);
        assert_eq!(error.code(), kxt::error::code::UNSUPPORTED_SCHEMA);
        assert_eq!(
            error.capability(),
            Some(kxt::XtCapability::SchemaBase13006.id())
        );
        assert_eq!(
            session.parts().len(),
            1,
            "parse failure must not alter part ownership"
        );
        let only_part = session.parts().next().unwrap();
        assert_eq!(
            session.part(only_part).unwrap().bodies().len(),
            bodies_before
        );
        let facade_source = error
            .source()
            .and_then(|source| source.downcast_ref::<XtInterchangeError>())
            .unwrap();
        assert!(matches!(
            facade_source
                .source()
                .and_then(|source| source.downcast_ref::<kxt::XtError>()),
            Some(kxt::XtError::UnsupportedSchema { schema })
                if schema == "SCH_1000230_10004"
        ));
    }

    #[test]
    fn late_reconstruction_failure_restores_existing_state_and_future_identity() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let existing = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 2.0, 3.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let mut control = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            edit.store_mut_for_test().clone()
        };
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

        let failed = session
            .edit_part(part_id.clone())
            .unwrap()
            .import_xt(ImportXtRequest::new(&invalid_late_block()))
            .unwrap();
        assert_eq!(
            failed.result().unwrap_err().code(),
            kxt::error::code::OUTSIDE_SIZE_BOX
        );
        assert_eq!(failed.report().usage().len(), 7);
        let part = session.part(part_id.clone()).unwrap();
        assert_eq!(
            before,
            (
                part.bodies().len(),
                part.faces().len(),
                part.edges().len(),
                part.vertices().len(),
                part.curves().len(),
                part.surfaces().len(),
                part.pcurves().len(),
            )
        );
        assert_eq!(part.body(existing).unwrap().kind(), crate::BodyKind::Solid);

        let future = {
            let mut edit = session.edit_part(part_id).unwrap();
            ktopo::make::block_with_journal(
                edit.store_mut_for_test(),
                &Frame::world(),
                [4.0, 5.0, 6.0],
            )
            .unwrap()
        };
        let expected =
            ktopo::make::block_with_journal(&mut control, &Frame::world(), [4.0, 5.0, 6.0])
                .unwrap();
        assert_eq!(future.body(), expected.body());
        assert_eq!(future.journal(), expected.journal());
    }

    #[test]
    fn import_aggregate_graph_limit_counts_prior_queries_and_rolls_back() {
        let bytes = block_xt();
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let budget = kgraph::EvalBudgetProfile::for_limits(64, 1);
        let outcome = session
            .edit_part(part_id.clone())
            .unwrap()
            .import_xt(
                ImportXtRequest::new(bytes.as_bytes())
                    .with_settings(OperationSettings::new().with_budget_overrides(budget)),
            )
            .unwrap();
        let expected = LimitSnapshot {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: ResourceKind::Work,
            consumed: 2,
            allowed: 1,
        };
        assert_eq!(outcome.result().unwrap_err().limit(), Some(expected));
        assert_eq!(outcome.report().limit_events(), &[expected]);
        assert_eq!(
            report_snapshot(
                &outcome,
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
            )
            .consumed,
            1
        );
        assert_eq!(session.part(part_id).unwrap().bodies().len(), 0);
    }

    #[test]
    fn checked_commit_graph_limit_matches_direct_import_and_rolls_back_populated_store() {
        let bytes = block_xt();
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let existing = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [1.0, 2.0, 3.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let mut control = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            edit.store_mut_for_test().clone()
        };
        let mut direct = control.clone();
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
        let budget = kgraph::EvalBudgetProfile::for_limits(64, 12);

        let direct_policy = kcore::operation::SessionPolicy::v1();
        let direct_context = kcore::operation::OperationContext::new(
            &direct_policy,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap()
        .with_budget_overrides(budget.clone());
        let direct_outcome =
            kxt::import_with_context(bytes.as_bytes(), &mut direct, &direct_context).unwrap();
        let facade = session
            .edit_part(part_id.clone())
            .unwrap()
            .import_xt(
                ImportXtRequest::new(bytes.as_bytes())
                    .with_settings(OperationSettings::new().with_budget_overrides(budget)),
            )
            .unwrap();
        let expected = LimitSnapshot {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: ResourceKind::Work,
            consumed: 13,
            allowed: 12,
        };
        assert_eq!(direct_outcome.result().unwrap_err().limit(), Some(expected));
        assert_eq!(facade.result().unwrap_err().limit(), Some(expected));
        assert_eq!(facade.report(), direct_outcome.report());
        assert_eq!(facade.report().limit_events(), &[expected]);

        let part = session.part(part_id.clone()).unwrap();
        assert_eq!(
            before,
            (
                part.bodies().len(),
                part.faces().len(),
                part.edges().len(),
                part.vertices().len(),
                part.curves().len(),
                part.surfaces().len(),
                part.pcurves().len(),
            )
        );
        assert_eq!(part.body(existing).unwrap().kind(), crate::BodyKind::Solid);
        assert_store_shape(&part, &direct);

        let future = {
            let mut edit = session.edit_part(part_id).unwrap();
            ktopo::make::block_with_journal(
                edit.store_mut_for_test(),
                &Frame::world(),
                [4.0, 5.0, 6.0],
            )
            .unwrap()
        };
        let expected_future =
            ktopo::make::block_with_journal(&mut control, &Frame::world(), [4.0, 5.0, 6.0])
                .unwrap();
        assert_eq!(future.body(), expected_future.body());
        assert_eq!(future.journal(), expected_future.journal());
    }

    #[test]
    fn import_total_work_precedes_graph_stage_and_matches_error_report() {
        let bytes = block_xt();
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let budget = kgraph::EvalBudgetProfile::for_limits(64, 64).with_total_work_limit(1);
        let outcome = session
            .edit_part(part_id.clone())
            .unwrap()
            .import_xt(
                ImportXtRequest::new(bytes.as_bytes())
                    .with_settings(OperationSettings::new().with_budget_overrides(budget)),
            )
            .unwrap();
        let expected = LimitSnapshot {
            stage: TOTAL_WORK_STAGE,
            resource: ResourceKind::Work,
            consumed: 2,
            allowed: 1,
        };
        assert_eq!(outcome.result().unwrap_err().limit(), Some(expected));
        assert_eq!(outcome.report().limit_events(), &[expected]);
        assert_eq!(
            report_snapshot(&outcome, TOTAL_WORK_STAGE, ResourceKind::Work).consumed,
            1
        );
        assert_eq!(session.part(part_id).unwrap().bodies().len(), 0);
    }

    fn add_raw_block(session: &mut crate::Session, part: &PartId) -> BodyId {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let raw = ktopo::make::block(edit.store_mut_for_test(), &Frame::world(), [1.0, 1.0, 1.0])
            .unwrap();
        BodyId::new(part.clone(), raw)
    }

    fn make_stale_body(session: &mut crate::Session, part: &PartId) -> BodyId {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let store = edit.store_mut_for_test();
        let surface = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        let point = store.insert_point(Point3::new(0.0, 0.0, 0.0)).unwrap();
        let mut transaction = store.transaction().unwrap();
        let made = transaction
            .make_minimal_body(surface, crate::Sense::Forward, point)
            .unwrap();
        let stale = BodyId::new(part.clone(), made.body);
        transaction.kill_minimal_body(made.body).unwrap();
        transaction.commit_checked(&[]).unwrap();
        stale
    }

    #[test]
    fn export_rejects_wrong_and_stale_body_before_invalid_context() {
        let strict = SessionPolicy::new(
            SessionPrecision::try_new(1.0e-6, 1.0e-11, 500.0).unwrap(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            crate::BudgetPlan::empty(),
            PolicyVersion::V1,
        );
        let mut session = Kernel::with_default_policy(strict).create_session();
        let first = session.create_part();
        let second = session.create_part();
        let body = add_raw_block(&mut session, &first);
        let second_body = add_raw_block(&mut session, &second);
        assert_eq!(body.raw(), second_body.raw());
        assert!(matches!(
            session
                .part(second)
                .unwrap()
                .export_xt(ExportXtRequest::new(body.clone())),
            Err(KernelError::WrongPart { .. })
        ));

        let stale = make_stale_body(&mut session, &first);
        assert!(matches!(
            session
                .part(first)
                .unwrap()
                .export_xt(ExportXtRequest::new(stale)),
            Err(KernelError::StaleEntity {
                kind: EntityKind::Body
            })
        ));
    }
}
