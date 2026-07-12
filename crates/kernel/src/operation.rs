//! Typed contextual operations at the supported façade boundary.

use core::fmt;

use kcore::operation::{ChildWorkLedger, OperationContext, OperationPolicyError, OperationScope};
use kgraph::{EvalBudgetProfile, EvalContext, EvalLimits, EvalUsage};
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

    #[cfg(test)]
    pub(crate) const fn raw_for_test(&self) -> &ktopo::transaction::Journal {
        &self.inner
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

/// Typed request for one bounded surface evaluation.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceEvaluationRequest {
    surface: SurfaceId,
    uv: [f64; 2],
    order: kgraph::SurfaceDerivativeOrder,
    settings: OperationSettings,
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
        let mut context = settings.context(self.policy)?;
        if level == CheckLevel::Full {
            context = context.with_family_budget_defaults(FullCheckBudgetProfile::v1_defaults());
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
        SessionPrecision, TOTAL_WORK_STAGE,
    };
    use kgeom::surface::Plane;
    use kgraph::{EvalBudgetProfile, EvalError, OffsetSurfaceDescriptor};
    use ktopo::check::VerificationGapCause;
    use ktopo::entity::{Body as RawBody, Edge as RawEdge, Face as RawFace, Vertex as RawVertex};
    use ktopo::geom::SurfaceGeom;
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
        assert!(facade_check.report().usage().is_empty());
        let expected = adapt_check_report(&part_id, &direct_store, direct_check).unwrap();
        assert_eq!(facade_check.result(), Ok(&expected));
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
