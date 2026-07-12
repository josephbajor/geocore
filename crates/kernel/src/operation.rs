//! Typed contextual operations at the supported façade boundary.

use core::fmt;

use kcore::operation::{OperationContext, OperationScope};
use ktopo::check::FullCheckBudgetProfile;
use ktopo::entity::EntityRef;

use crate::error::{Error, Result};
use crate::session::{Part, PartEdit};
use crate::{
    BodyId, BudgetPlan, CheckLevel, CheckOutcome, CurveId, DiagnosticLevel, EdgeId, FaceId,
    FaultKind, FinId, Frame, LoopId, PartId, PcurveId, Point3, RegionId, SessionPolicy, ShellId,
    SurfaceId, Tolerances, VerificationGapCause, VerificationGapKind, VertexId,
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

    fn context<'session>(
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

/// Opaque owning adapter over one committed lower-layer journal.
pub struct ChangeJournal {
    part: PartId,
    inner: ktopo::transaction::Journal,
}

impl ChangeJournal {
    /// Part whose state was changed.
    pub fn part(&self) -> PartId {
        self.part.clone()
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
}

impl fmt::Debug for ChangeJournal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ChangeJournal")
            .field("part", &self.part)
            .field("mutation_count", &self.mutation_count())
            .field("lineage_count", &self.lineage_count())
            .field("tolerance_budget_count", &self.tolerance_budget_count())
            .field("tolerance_event_count", &self.tolerance_event_count())
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
                    journal: ChangeJournal {
                        part: part.clone(),
                        inner,
                    },
                }
            })
            .map_err(Error::from);
        Ok(scope.finish_typed(result))
    }
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
        let context = settings.context(self.policy)?;
        if level == CheckLevel::Full {
            let effective = context.effective_budget();
            for required in FullCheckBudgetProfile::v1_defaults().limits() {
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
}

fn adapt_check_report(
    part: &PartId,
    store: &ktopo::store::Store,
    report: ktopo::check::CheckReport,
) -> Result<CheckReport> {
    let faults = report
        .faults
        .into_iter()
        .map(|fault| {
            Ok(CheckFault {
                entity: adapt_check_entity(part, store, fault.entity)?,
                kind: fault.kind,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let gaps = report
        .gaps
        .into_iter()
        .map(|gap| {
            Ok(CheckGap {
                entity: adapt_check_entity(part, store, gap.entity)?,
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
    store: &ktopo::store::Store,
    entity: EntityRef,
) -> Result<CheckEntity> {
    let part = part.clone();
    Ok(match entity {
        EntityRef::Body(raw) => CheckEntity::Body(BodyId::new(part, raw)),
        EntityRef::Region(raw) => CheckEntity::Region(RegionId::new(part, raw)),
        EntityRef::Shell(raw) => CheckEntity::Shell(ShellId::new(part, raw)),
        EntityRef::Face(raw) => CheckEntity::Face(FaceId::new(part, raw)),
        EntityRef::Loop(raw) => CheckEntity::Loop(LoopId::new(part, raw)),
        EntityRef::Fin(raw) => CheckEntity::Fin(FinId::new(part, raw)),
        EntityRef::Edge(raw) => CheckEntity::Edge(EdgeId::new(part, raw)),
        EntityRef::Vertex(raw) => CheckEntity::Vertex(VertexId::new(part, raw)),
        EntityRef::Curve(raw) => CheckEntity::Curve(CurveId::new(part, raw)),
        EntityRef::Surface(raw) => CheckEntity::Surface(SurfaceId::new(part, raw)),
        EntityRef::Curve2d(raw) => CheckEntity::Pcurve(PcurveId::new(part, raw)),
        EntityRef::Point(raw) => CheckEntity::Point(
            *store
                .get(raw)
                .map_err(|source| Error::InconsistentTopology { source })?,
        ),
    })
}

#[cfg(test)]
mod tests {
    use std::error::Error as _;

    use kcore::error::ErrorClass;
    use kcore::operation::{
        AccountingMode, ExecutionPolicy, LimitSpec, NumericalPolicy, PolicyVersion, ResourceKind,
        SessionPrecision,
    };
    use ktopo::check::VerificationGapCause;
    use ktopo::entity::{Body as RawBody, Edge as RawEdge, Face as RawFace, Vertex as RawVertex};
    use ktopo::store::Store;

    use super::*;
    use crate::{Kernel, KernelError};

    fn full_check_policy() -> SessionPolicy {
        SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            FullCheckBudgetProfile::v1_defaults(),
            PolicyVersion::V1,
        )
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
        assert!(facade_check.report().usage().is_empty());
        let expected = adapt_check_report(&part_id, &direct_store, direct_check).unwrap();
        assert_eq!(facade_check.result(), Ok(&expected));
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

        let mut session = Kernel::with_default_policy(policy).create_session();
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
    fn missing_full_check_policy_is_a_delegated_pre_scope_failure() {
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
        let error = session
            .part(part_id)
            .unwrap()
            .check_body(CheckBodyRequest::new(body, CheckLevel::Full))
            .unwrap_err();
        let expected = kcore::operation::OperationPolicyError::UnknownLimit {
            stage: ktopo::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
            resource: ResourceKind::Items,
        };
        assert_eq!(error.class(), ErrorClass::InvalidInput);
        assert_eq!(error.code(), expected.code());
        assert!(matches!(
            error.source().and_then(|source| source.downcast_ref()),
            Some(kcore::error::Error::OperationPolicy { source }) if source == &expected
        ));
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
}
