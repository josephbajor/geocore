//! Failure-atomic internal pipeline for the first convex planar Boolean seam.
//!
//! This module deliberately stops short of a public facade operation.  It
//! composes the proof-bearing source extractor, symbolic BSP, conservative
//! plane-triple realization, keyed planar assembler, and Full checked commit
//! without granting floating-point arithmetic topological authority.

use std::collections::{BTreeMap, BTreeSet};

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use ktopo::check::{CheckLevel, CheckOutcome, CheckReport};
use ktopo::entity::{EntityRef, FaceId as RawFaceId, SurfaceId as RawSurfaceId};
use ktopo::planar::{
    PlanarFacePlaneBinding, PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey,
};
use ktopo::transaction::{FullCommitRequirement, Journal};

use super::extract::{
    ExtractedPlanarSourceBody, PlanarSourceExtractionBudgetProfile, PlanarSourceExtractionError,
    PlanarSourceGap, PlanarSourceProofFailure, extract_planar_source_body,
};
use super::planar_bsp::{PlaneTripleVertexKey, SourcePlane};
use super::realize::{RealizationError, realize_symbolic_vertex};
use super::select::{
    PlanarBooleanOperation, SelectedPlanarFragment, SelectionError, select_boolean_fragments,
};
use crate::error::{Error, Result};
use crate::operation::{OperationOutcome, OperationSettings};
use crate::session::PartEdit;
use crate::{BodyId, EntityKind};

/// Input-size-exact upper-bound work charged before symbolic partitioning.
pub(crate) const PLANAR_BOOLEAN_BSP_WORK: StageId = known_stage("kernel.boolean.planar-bsp-work");
/// Maximum symbolic fragments whose allocation may be attempted.
pub(crate) const PLANAR_BOOLEAN_BSP_FRAGMENTS: StageId =
    known_stage("kernel.boolean.planar-bsp-fragments");
/// Input-size-exact upper-bound work charged before vertex-key discovery.
pub(crate) const PLANAR_BOOLEAN_REALIZATION_WORK: StageId =
    known_stage("kernel.boolean.planar-realization-work");
/// Maximum unique plane triples whose realization may be attempted.
pub(crate) const PLANAR_BOOLEAN_REALIZED_VERTICES: StageId =
    known_stage("kernel.boolean.planar-realized-vertices");

const fn known_stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid built-in planar Boolean pipeline stage identifier"),
    }
}

/// Version-1 allowances for the internal convex planar Boolean pipeline.
pub(crate) struct PlanarBooleanPipelineBudgetProfile;

impl PlanarBooleanPipelineBudgetProfile {
    pub(crate) fn v1_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                PLANAR_BOOLEAN_BSP_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                100_000_000,
            ),
            LimitSpec::new(
                PLANAR_BOOLEAN_BSP_FRAGMENTS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                1_000_000,
            ),
            LimitSpec::new(
                PLANAR_BOOLEAN_REALIZATION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                100_000_000,
            ),
            LimitSpec::new(
                PLANAR_BOOLEAN_REALIZED_VERTICES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                1_000_000,
            ),
        ])
        .expect("built-in planar Boolean pipeline budget is valid")
    }
}

/// One internal Boolean result that survived Full checking and was committed.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CommittedPlanarBoolean {
    body: BodyId,
    journal: Journal,
    full_outcomes: Vec<CheckOutcome>,
}

impl CommittedPlanarBoolean {
    pub(crate) fn body(&self) -> BodyId {
        self.body.clone()
    }

    pub(crate) const fn journal(&self) -> &Journal {
        &self.journal
    }

    pub(crate) fn full_outcomes(&self) -> &[CheckOutcome] {
        &self.full_outcomes
    }
}

/// Typed, non-persistent refusal from the bounded pipeline.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum PlanarBooleanPipelineRefusal {
    SourceNotFastValid {
        operand: u8,
        report: CheckReport,
    },
    UnsupportedSource {
        operand: u8,
        gap: PlanarSourceGap,
    },
    UncertifiedSource {
        operand: u8,
        failure: PlanarSourceProofFailure,
    },
    Symbolic(SelectionError),
    Realization(RealizationError),
    /// Source plane identity could not be bound uniquely to live source geometry.
    PlaneBindingContract(&'static str),
    /// The symbolic boundary did not satisfy the general planar assembler.
    AssemblyContract(&'static str),
    /// The Full checker proved a candidate fault before journal commit.
    FullTopologyFault {
        fault_count: usize,
    },
    /// RequireValid rejected one or more proof-incomplete candidate reports.
    FullProofRejected(Vec<CheckReport>),
    /// A deterministic upper-bound calculation did not fit in `u64`.
    WorkCountOverflow,
}

/// Complete typed outcome of the internal operation.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum PlanarBooleanPipelineOutcome {
    /// Truth selection proved that the regularized result has no boundary.
    ProvenEmpty,
    /// One result body and its deterministic journal were persisted.
    Committed(CommittedPlanarBoolean),
    /// No result topology survived; the exact gap remains explicit.
    Refused(PlanarBooleanPipelineRefusal),
}

/// Run one bounded planar Boolean through a single checked topology transaction.
///
/// Facade identity and settings failures precede the operation scope.  Once a
/// scope starts, execution/resource errors retain its report, while valid but
/// unsupported proof cases are values.  Source bodies are retained unchanged.
pub(crate) fn execute_planar_boolean(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    left: BodyId,
    right: BodyId,
    settings: OperationSettings,
) -> Result<OperationOutcome<PlanarBooleanPipelineOutcome>> {
    validate_operand(edit, &left)?;
    validate_operand(edit, &right)?;

    let defaults = PlanarBooleanPipelineBudgetProfile::v1_defaults()
        .overlaid(&PlanarSourceExtractionBudgetProfile::v1_defaults())
        .overlaid(&ktopo::check::CheckBudgetProfile::v1_defaults(
            CheckLevel::Full,
        ));
    let context = settings
        .context(edit.policy)?
        .with_family_budget_defaults(defaults);
    let mut scope = OperationScope::new(&context);
    let result = execute_in_scope(edit, operation, left, right, &mut scope);
    Ok(scope.finish_typed(result))
}

fn validate_operand(edit: &PartEdit<'_>, body: &BodyId) -> Result<()> {
    if body.part() != &edit.id {
        return Err(Error::WrongPart {
            expected: edit.id.clone(),
            actual: body.part().clone(),
        });
    }
    edit.state
        .store
        .get(body.raw())
        .map(|_| ())
        .map_err(|_| Error::StaleEntity {
            kind: EntityKind::Body,
        })
}

fn execute_in_scope(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    left: BodyId,
    right: BodyId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PlanarBooleanPipelineOutcome> {
    match execute_stages(edit, operation, left, right, scope) {
        Ok(outcome) => Ok(outcome),
        Err(PipelineFailure::Execution(error)) => Err(error),
        Err(PipelineFailure::Refused(refusal)) => {
            Ok(PlanarBooleanPipelineOutcome::Refused(refusal))
        }
    }
}

#[derive(Debug)]
enum PipelineFailure {
    Execution(Error),
    Refused(PlanarBooleanPipelineRefusal),
}

impl From<Error> for PipelineFailure {
    fn from(error: Error) -> Self {
        Self::Execution(error)
    }
}

impl From<kcore::error::Error> for PipelineFailure {
    fn from(error: kcore::error::Error) -> Self {
        Self::Execution(error.into())
    }
}

type StageResult<T> = core::result::Result<T, PipelineFailure>;

fn execute_stages(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    left: BodyId,
    right: BodyId,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<PlanarBooleanPipelineOutcome> {
    validate_pipeline_budget(scope)?;
    let left_source = extract_operand(edit, left, 0, scope)?;
    let right_source = extract_operand(edit, right, 1, scope)?;
    let plane_registry = build_plane_registry(edit, &left_source, &right_source)?;

    let selected = select_with_precharge(&left_source, &right_source, operation, scope)?;
    if selected.is_empty() {
        return Ok(PlanarBooleanPipelineOutcome::ProvenEmpty);
    }
    let planes = combined_planes(&left_source, &right_source);
    let (vertices, vertex_keys) = realize_vertices(&selected, &planes, scope)?;
    let faces = prepare_faces(&selected, &vertex_keys, &plane_registry)?;
    let input = PlanarSolidInput::new(vertices, faces);

    let mut transaction = edit.state.store.transaction().map_err(Error::from)?;
    let output = match transaction.assemble_planar_solid(&input) {
        Ok(output) => output,
        Err(kcore::error::Error::InvalidGeometry { reason }) => {
            return Err(PipelineFailure::Refused(
                PlanarBooleanPipelineRefusal::AssemblyContract(reason),
            ));
        }
        Err(source) => return Err(source.into()),
    };
    let raw_body = output.body();
    let decision = match transaction.commit_full_in_scope(
        &[raw_body],
        FullCommitRequirement::RequireValid,
        scope,
        0,
    ) {
        Ok(decision) => decision,
        Err(kcore::error::Error::TopologyCheckFailed { fault_count }) => {
            return Err(PipelineFailure::Refused(
                PlanarBooleanPipelineRefusal::FullTopologyFault { fault_count },
            ));
        }
        Err(source) => return Err(source.into()),
    };
    let outcomes = decision
        .checks()
        .iter()
        .map(|check| check.report().outcome())
        .collect::<Vec<_>>();
    let reports = decision
        .checks()
        .iter()
        .map(|check| check.report().clone())
        .collect::<Vec<_>>();
    let (journal, _) = decision.into_parts();
    let Some(journal) = journal else {
        return Ok(PlanarBooleanPipelineOutcome::Refused(
            PlanarBooleanPipelineRefusal::FullProofRejected(reports),
        ));
    };
    Ok(PlanarBooleanPipelineOutcome::Committed(
        CommittedPlanarBoolean {
            body: BodyId::new(edit.id.clone(), raw_body),
            journal,
            full_outcomes: outcomes,
        },
    ))
}

fn validate_pipeline_budget(scope: &OperationScope<'_, '_>) -> StageResult<()> {
    for (stage, resource, mode) in [
        (
            PLANAR_BOOLEAN_BSP_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        ),
        (
            PLANAR_BOOLEAN_BSP_FRAGMENTS,
            ResourceKind::Items,
            AccountingMode::HighWater,
        ),
        (
            PLANAR_BOOLEAN_REALIZATION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        ),
        (
            PLANAR_BOOLEAN_REALIZED_VERTICES,
            ResourceKind::Items,
            AccountingMode::HighWater,
        ),
    ] {
        scope
            .ledger()
            .require_limit(stage, resource, mode)
            .map_err(Error::from)?;
    }
    Ok(())
}

fn extract_operand(
    edit: &PartEdit<'_>,
    body: BodyId,
    operand: u8,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<ExtractedPlanarSourceBody> {
    let part = edit.as_part();
    match extract_planar_source_body(&part, body, operand, scope) {
        Ok(source) => Ok(source),
        Err(PlanarSourceExtractionError::NotFastValid(report)) => Err(PipelineFailure::Refused(
            PlanarBooleanPipelineRefusal::SourceNotFastValid { operand, report },
        )),
        Err(PlanarSourceExtractionError::Unsupported(gap)) => Err(PipelineFailure::Refused(
            PlanarBooleanPipelineRefusal::UnsupportedSource { operand, gap },
        )),
        Err(PlanarSourceExtractionError::Uncertified(failure)) => Err(PipelineFailure::Refused(
            PlanarBooleanPipelineRefusal::UncertifiedSource { operand, failure },
        )),
        Err(PlanarSourceExtractionError::Topology(source)) => Err(source.into()),
        Err(PlanarSourceExtractionError::WrongPart) => Err(kcore::error::Error::InvalidGeometry {
            reason: "prevalidated planar Boolean operand changed part",
        }
        .into()),
        Err(PlanarSourceExtractionError::InvalidOperand) => {
            Err(kcore::error::Error::InvalidGeometry {
                reason: "internal planar Boolean operand index is invalid",
            }
            .into())
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct BspPrecharge {
    work: u64,
    fragments: u64,
}

fn select_with_precharge(
    left: &ExtractedPlanarSourceBody,
    right: &ExtractedPlanarSourceBody,
    operation: PlanarBooleanOperation,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<Vec<SelectedPlanarFragment>> {
    let precharge = bsp_precharge(
        left.fragments().len(),
        left.planes().len(),
        right.fragments().len(),
        right.planes().len(),
    )
    .ok_or_else(work_count_overflow)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, precharge.work)
        .map_err(Error::from)?;
    scope
        .ledger_mut()
        .observe(
            PLANAR_BOOLEAN_BSP_FRAGMENTS,
            ResourceKind::Items,
            precharge.fragments,
        )
        .map_err(Error::from)?;

    let planes = combined_planes(left, right);
    let left_plane_ids = left.plane_ids().collect::<Vec<_>>();
    let right_plane_ids = right.plane_ids().collect::<Vec<_>>();
    select_boolean_fragments(
        operation,
        &planes,
        left.fragments().to_vec(),
        &left_plane_ids,
        right.fragments().to_vec(),
        &right_plane_ids,
    )
    .map_err(|error| PipelineFailure::Refused(PlanarBooleanPipelineRefusal::Symbolic(error)))
}

fn bsp_precharge(
    left_seeds: usize,
    left_cutters: usize,
    right_seeds: usize,
    right_cutters: usize,
) -> Option<BspPrecharge> {
    let left_seeds = u64::try_from(left_seeds).ok()?;
    let left_cutters = u64::try_from(left_cutters).ok()?;
    let right_seeds = u64::try_from(right_seeds).ok()?;
    let right_cutters = u64::try_from(right_cutters).ok()?;

    // A deliberately geometry-independent ceiling: every fragment may split
    // at every cutter.  It is conservative but exactly determined by the
    // admitted input sizes, so refusal does not depend on early-exit geometry.
    let left_factor = power_of_two(right_cutters)?;
    let right_factor = power_of_two(left_cutters)?;
    let left_output = left_seeds.checked_mul(left_factor)?;
    let right_output = right_seeds.checked_mul(right_factor)?;
    let final_fragments = left_output.checked_add(right_output)?;
    // Three complete final-size cohorts conservatively cover retained input,
    // old/new partition vectors, classification transfer, and truth-selection
    // storage for either asymmetric operand order.
    let fragments = final_fragments.checked_mul(3)?;
    let split_visits = left_seeds
        .checked_mul(left_factor.checked_sub(1)?)?
        .checked_add(right_seeds.checked_mul(right_factor.checked_sub(1)?)?)?;
    let classification = left_output
        .checked_mul(right_cutters)?
        .checked_add(right_output.checked_mul(left_cutters)?)?;
    let work = split_visits
        .checked_add(classification)?
        .checked_add(final_fragments)?;
    Some(BspPrecharge { work, fragments })
}

fn power_of_two(exponent: u64) -> Option<u64> {
    u32::try_from(exponent)
        .ok()
        .filter(|&shift| shift < u64::BITS)
        .and_then(|shift| 1_u64.checked_shl(shift))
}

fn combined_planes(
    left: &ExtractedPlanarSourceBody,
    right: &ExtractedPlanarSourceBody,
) -> Vec<SourcePlane> {
    let mut planes = left.planes().to_vec();
    planes.extend_from_slice(right.planes());
    planes.sort_by_key(|plane| plane.id());
    planes
}

fn realize_vertices(
    selected: &[SelectedPlanarFragment],
    planes: &[SourcePlane],
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<(
    Vec<PlanarSolidVertex>,
    BTreeMap<PlaneTripleVertexKey, PlanarVertexKey>,
)> {
    let ring_uses = selected.iter().try_fold(0_u64, |total, selected| {
        total.checked_add(u64::try_from(selected.fragment().vertices().len()).ok()?)
    });
    let Some(ring_uses) = ring_uses else {
        return Err(work_count_overflow());
    };
    let plane_count = u64::try_from(planes.len()).map_err(|_| work_count_overflow())?;
    let work = ring_uses
        .checked_mul(plane_count.checked_add(2).ok_or_else(work_count_overflow)?)
        .ok_or_else(work_count_overflow)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_REALIZATION_WORK, work)
        .map_err(Error::from)?;
    scope
        .ledger_mut()
        .observe(
            PLANAR_BOOLEAN_REALIZED_VERTICES,
            ResourceKind::Items,
            ring_uses,
        )
        .map_err(Error::from)?;

    let mut unique = BTreeSet::new();
    for selected in selected {
        unique.extend(selected.fragment().vertices().iter().copied());
    }
    let all_plane_ids = planes.iter().map(|plane| plane.id()).collect::<Vec<_>>();
    let mut vertices = Vec::with_capacity(unique.len());
    let mut keys = BTreeMap::new();
    for (index, triple) in unique.into_iter().enumerate() {
        let key = PlanarVertexKey::new(u64::try_from(index).map_err(|_| work_count_overflow())?);
        let defining = triple.planes();
        let strict_side_planes = all_plane_ids
            .iter()
            .copied()
            .filter(|plane| !defining.contains(plane))
            .collect::<Vec<_>>();
        let realized =
            match realize_symbolic_vertex(scope.context(), planes, triple, &strict_side_planes) {
                Ok(realized) => realized,
                Err(error) => {
                    return Err(PipelineFailure::Refused(
                        PlanarBooleanPipelineRefusal::Realization(error),
                    ));
                }
            };
        vertices.push(PlanarSolidVertex::new(key, realized.point()));
        keys.insert(triple, key);
    }
    Ok((vertices, keys))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SourcePlaneBinding {
    face: RawFaceId,
    surface: RawSurfaceId,
}

fn build_plane_registry(
    edit: &PartEdit<'_>,
    left: &ExtractedPlanarSourceBody,
    right: &ExtractedPlanarSourceBody,
) -> StageResult<BTreeMap<super::planar_bsp::SourcePlaneRef, SourcePlaneBinding>> {
    let mut registry = BTreeMap::new();
    for face in left.faces().iter().chain(right.faces()) {
        let binding = SourcePlaneBinding {
            face: face.face().raw(),
            surface: face.surface(),
        };
        if registry.insert(face.plane(), binding).is_some() {
            return Err(plane_binding_refusal(
                "planar Boolean source-plane identities must be unique",
            ));
        }
        let Ok(source_face) = edit.state.store.get(binding.face) else {
            return Err(plane_binding_refusal(
                "planar Boolean source-plane registry contains a stale face",
            ));
        };
        if source_face.surface() != binding.surface
            || edit
                .state
                .store
                .geometry()
                .surface(binding.surface)
                .and_then(|surface| surface.as_plane())
                .is_none()
        {
            return Err(plane_binding_refusal(
                "planar Boolean source face and Plane surface are mismatched",
            ));
        }
    }

    let mut witnesses = BTreeSet::new();
    for plane in left.planes().iter().chain(right.planes()) {
        if !witnesses.insert(plane.id()) {
            return Err(plane_binding_refusal(
                "planar Boolean source-plane witnesses must be unique",
            ));
        }
        if !registry.contains_key(&plane.id()) {
            return Err(plane_binding_refusal(
                "planar Boolean source plane has no face/surface binding",
            ));
        }
    }
    if registry.len() != witnesses.len() {
        return Err(plane_binding_refusal(
            "planar Boolean face/surface registry has no plane witness",
        ));
    }
    Ok(registry)
}

fn prepare_faces(
    selected: &[SelectedPlanarFragment],
    keys: &BTreeMap<PlaneTripleVertexKey, PlanarVertexKey>,
    registry: &BTreeMap<super::planar_bsp::SourcePlaneRef, SourcePlaneBinding>,
) -> StageResult<Vec<PlanarSolidFace>> {
    selected
        .iter()
        .map(|selected| {
            let support = registry
                .get(&selected.fragment().source_face())
                .copied()
                .ok_or_else(|| {
                    plane_binding_refusal(
                        "selected planar Boolean face has no source plane binding",
                    )
                })?;
            let boundary = selected.oriented_boundary();
            let mut ring = Vec::with_capacity(boundary.len());
            let mut carriers = Vec::with_capacity(boundary.len());
            for (triple, carrier) in boundary {
                ring.push(keys.get(&triple).copied().ok_or_else(|| {
                    plane_binding_refusal("selected planar Boolean face has an unrealized vertex")
                })?);
                let carrier = registry.get(&carrier).ok_or_else(|| {
                    plane_binding_refusal(
                        "selected planar Boolean edge carrier has no source plane binding",
                    )
                })?;
                carriers.push(carrier.surface);
            }
            Ok(PlanarSolidFace::new(ring)
                .with_source(EntityRef::Face(support.face))
                .with_plane_binding(PlanarFacePlaneBinding::new(support.surface, carriers)))
        })
        .collect()
}

fn plane_binding_refusal(reason: &'static str) -> PipelineFailure {
    PipelineFailure::Refused(PlanarBooleanPipelineRefusal::PlaneBindingContract(reason))
}

fn work_count_overflow() -> PipelineFailure {
    PipelineFailure::Refused(PlanarBooleanPipelineRefusal::WorkCountOverflow)
}

#[cfg(test)]
mod tests {
    use kcore::operation::{LimitSnapshot, OperationReport};
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};
    use ktopo::entity::{Body as RawBody, Edge as RawEdge, Face as RawFace, Vertex as RawVertex};
    use ktopo::transaction::{LineageEvent, MutationKind};

    use super::*;
    use crate::{BlockRequest, Kernel, PartId, Session};

    struct Fixture {
        session: Session,
        part: PartId,
        left: BodyId,
        right: BodyId,
    }

    fn frame(origin: [f64; 3], z: [f64; 3], x: [f64; 3]) -> Frame {
        Frame::new(
            Point3::from_array(origin),
            Vec3::from_array(z),
            Vec3::from_array(x),
        )
        .unwrap()
    }

    fn rotated_fixture() -> Fixture {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (left, right) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let left = edit
                .create_block(BlockRequest::new(
                    frame([1.25, -0.75, 0.5], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),
                    [3.5, 2.75, 2.5],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let right = edit
                .create_block(BlockRequest::new(
                    frame([1.75, -0.25, 0.75], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
                    [3.0, 2.0, 2.75],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (left, right)
        };
        Fixture {
            session,
            part,
            left,
            right,
        }
    }

    fn arbitrary_rotated_fixture() -> Fixture {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (sin, cos) = kcore::math::sincos(0.47);
        let (left, right) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let left = edit
                .create_block(BlockRequest::new(
                    frame([-0.3, 0.2, -0.1], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
                    [4.0, 3.5, 3.0],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let right = edit
                .create_block(BlockRequest::new(
                    frame([0.5, 0.1, 0.25], [0.0, 0.0, 1.0], [cos, sin, 0.0]),
                    [3.25, 2.75, 3.5],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (left, right)
        };
        Fixture {
            session,
            part,
            left,
            right,
        }
    }

    fn run_operation(
        fixture: &mut Fixture,
        operation: PlanarBooleanOperation,
        settings: OperationSettings,
    ) -> OperationOutcome<PlanarBooleanPipelineOutcome> {
        let left = fixture.left.clone();
        let right = fixture.right.clone();
        let mut edit = fixture.session.edit_part(fixture.part.clone()).unwrap();
        execute_planar_boolean(&mut edit, operation, left, right, settings).unwrap()
    }

    fn run_intersection(
        fixture: &mut Fixture,
        settings: OperationSettings,
    ) -> OperationOutcome<PlanarBooleanPipelineOutcome> {
        run_operation(fixture, PlanarBooleanOperation::Intersect, settings)
    }

    fn committed(
        outcome: OperationOutcome<PlanarBooleanPipelineOutcome>,
    ) -> CommittedPlanarBoolean {
        match outcome.into_result().unwrap() {
            PlanarBooleanPipelineOutcome::Committed(committed) => committed,
            other => panic!("expected committed planar Boolean, got {other:?}"),
        }
    }

    #[test]
    fn rotated_off_origin_intersection_is_full_valid_journaled_and_deterministic() {
        let mut first_fixture = rotated_fixture();
        let mut second_fixture = rotated_fixture();
        let first = committed(run_intersection(
            &mut first_fixture,
            OperationSettings::new(),
        ));
        let second = committed(run_intersection(
            &mut second_fixture,
            OperationSettings::new(),
        ));

        assert_eq!(first.body().raw(), second.body().raw());
        assert_eq!(first.journal(), second.journal());
        assert!(!first.journal().mutations().is_empty());
        assert!(
            first
                .full_outcomes()
                .iter()
                .all(|outcome| *outcome == CheckOutcome::Valid)
        );
        let lineage = first.journal().lineage();
        assert!(!lineage.is_empty());
        let created_faces = first
            .journal()
            .mutations()
            .iter()
            .filter(|mutation| {
                mutation.kind == MutationKind::Created
                    && matches!(mutation.entity, EntityRef::Face(_))
            })
            .count();
        assert_eq!(lineage.len(), created_faces);
        assert!(lineage.iter().all(|event| matches!(
            event,
            LineageEvent::DerivedFrom {
                derived: EntityRef::Face(_),
                source: EntityRef::Face(_),
            }
        )));
        assert_bound_output_geometry(&first_fixture, &first);
        assert_bound_output_geometry(&second_fixture, &second);
    }

    fn assert_bound_output_geometry(fixture: &Fixture, committed: &CommittedPlanarBoolean) {
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        let store = &part.state.store;
        let source_surfaces = [fixture.left.raw(), fixture.right.raw()]
            .into_iter()
            .flat_map(|body| store.faces_of_body(body).unwrap())
            .map(|face| store.get(face).unwrap().surface())
            .collect::<Vec<_>>();

        for event in committed.journal().lineage() {
            let LineageEvent::DerivedFrom {
                derived: EntityRef::Face(derived),
                source: EntityRef::Face(source),
            } = event
            else {
                continue;
            };
            let derived_surface = store.get(*derived).unwrap().surface();
            let source_surface = store.get(*source).unwrap().surface();
            assert_eq!(derived_surface, source_surface);
            assert!(source_surfaces.contains(&derived_surface));
            assert!(store.get(derived_surface).unwrap().as_plane().is_some());
        }

        for edge in store.edges_of_body(committed.body().raw()).unwrap() {
            let edge = store.get(edge).unwrap();
            let descriptor = store
                .get(edge.curve().unwrap())
                .unwrap()
                .as_intersection()
                .expect("Boolean boundary edges retain verified intersections");
            let bound = descriptor.source_surfaces();
            assert!(bound.iter().all(|surface| {
                source_surfaces.contains(surface)
                    && store.get(*surface).unwrap().as_plane().is_some()
            }));
            assert!(descriptor.certificate().as_plane_line().is_some());

            let adjacent = edge
                .fins()
                .iter()
                .map(|fin| {
                    let fin = store.get(*fin).unwrap();
                    let loop_ = store.get(fin.parent()).unwrap();
                    store.get(loop_.face()).unwrap().surface()
                })
                .collect::<Vec<_>>();
            assert_eq!(adjacent.len(), 2);
            assert_ne!(bound[0], bound[1]);
            assert!(adjacent.iter().all(|support| bound.contains(support)));
            if adjacent[0] != adjacent[1] {
                assert!(
                    bound == [adjacent[0], adjacent[1]] || bound == [adjacent[1], adjacent[0]],
                    "verified edge sources differ from adjacent bound face supports"
                );
            }
        }
    }

    #[test]
    fn overlapping_unite_and_subtract_are_full_valid_journaled_and_deterministic() {
        for operation in [
            PlanarBooleanOperation::Unite,
            PlanarBooleanOperation::Subtract,
        ] {
            let mut first_fixture = rotated_fixture();
            let mut second_fixture = rotated_fixture();
            let first = committed(run_operation(
                &mut first_fixture,
                operation,
                OperationSettings::new(),
            ));
            let second = committed(run_operation(
                &mut second_fixture,
                operation,
                OperationSettings::new(),
            ));
            assert_eq!(first.body().raw(), second.body().raw());
            assert_eq!(first.journal(), second.journal());
            assert!(!first.journal().mutations().is_empty());
            assert!(!first.full_outcomes().is_empty());
            assert!(
                first
                    .full_outcomes()
                    .iter()
                    .all(|outcome| *outcome == CheckOutcome::Valid)
            );
            assert_eq!(first.full_outcomes(), second.full_outcomes());
            assert_bound_output_geometry(&first_fixture, &first);
            assert_bound_output_geometry(&second_fixture, &second);
        }
    }

    #[test]
    fn arbitrary_angle_overlap_is_full_valid_for_all_truth_tables_and_operand_orders() {
        for operation in [
            PlanarBooleanOperation::Unite,
            PlanarBooleanOperation::Subtract,
            PlanarBooleanOperation::Intersect,
        ] {
            for swapped in [false, true] {
                let mut first_fixture = arbitrary_rotated_fixture();
                let mut second_fixture = arbitrary_rotated_fixture();
                if swapped {
                    core::mem::swap(&mut first_fixture.left, &mut first_fixture.right);
                    core::mem::swap(&mut second_fixture.left, &mut second_fixture.right);
                }
                let first = committed(run_operation(
                    &mut first_fixture,
                    operation,
                    OperationSettings::new(),
                ));
                let second = committed(run_operation(
                    &mut second_fixture,
                    operation,
                    OperationSettings::new(),
                ));
                assert_eq!(first.body().raw(), second.body().raw());
                assert_eq!(first.journal(), second.journal());
                assert!(!first.journal().mutations().is_empty());
                assert!(!first.full_outcomes().is_empty());
                assert!(
                    first
                        .full_outcomes()
                        .iter()
                        .all(|outcome| *outcome == CheckOutcome::Valid)
                );
                assert_bound_output_geometry(&first_fixture, &first);
                assert_bound_output_geometry(&second_fixture, &second);
            }
        }
    }

    fn touching_fixture() -> Fixture {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (left, right) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let left = edit
                .create_block(BlockRequest::new(Frame::world(), [2.0, 2.0, 2.0]))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let right = edit
                .create_block(BlockRequest::new(
                    Frame::world().with_origin(Point3::new(2.0, 0.0, 0.0)),
                    [2.0, 2.0, 2.0],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (left, right)
        };
        Fixture {
            session,
            part,
            left,
            right,
        }
    }

    fn store_counts(fixture: &Fixture) -> (usize, usize, usize, usize) {
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        (
            part.state.store.count::<RawBody>(),
            part.state.store.count::<RawFace>(),
            part.state.store.count::<RawEdge>(),
            part.state.store.count::<RawVertex>(),
        )
    }

    fn add_probe(fixture: &mut Fixture) -> BodyId {
        fixture
            .session
            .edit_part(fixture.part.clone())
            .unwrap()
            .create_block(BlockRequest::new(
                Frame::world().with_origin(Point3::new(20.0, -7.0, 3.0)),
                [1.5, 1.25, 0.75],
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body()
    }

    fn disjoint_fixture() -> Fixture {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (left, right) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let left = edit
                .create_block(BlockRequest::new(
                    Frame::world().with_origin(Point3::new(-8.0, 1.0, 0.5)),
                    [2.0, 1.5, 2.5],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let right = edit
                .create_block(BlockRequest::new(
                    frame([8.0, -1.0, -0.5], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),
                    [1.75, 2.25, 1.25],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (left, right)
        };
        Fixture {
            session,
            part,
            left,
            right,
        }
    }

    #[test]
    fn disjoint_intersection_is_proven_empty_without_topology_allocation() {
        let mut attempted = disjoint_fixture();
        let mut control = disjoint_fixture();
        let before = store_counts(&attempted);
        assert!(matches!(
            run_intersection(&mut attempted, OperationSettings::new())
                .into_result()
                .unwrap(),
            PlanarBooleanPipelineOutcome::ProvenEmpty
        ));
        assert_eq!(store_counts(&attempted), before);
        assert_eq!(
            add_probe(&mut attempted).raw(),
            add_probe(&mut control).raw()
        );
        assert_eq!(store_counts(&attempted), store_counts(&control));
    }

    #[test]
    fn exact_contact_refusal_leaves_store_and_future_ids_unchanged() {
        let mut attempted = touching_fixture();
        let mut control = touching_fixture();
        let before = store_counts(&attempted);
        let outcome = run_intersection(&mut attempted, OperationSettings::new());
        assert!(matches!(
            outcome.into_result().unwrap(),
            PlanarBooleanPipelineOutcome::Refused(PlanarBooleanPipelineRefusal::Symbolic(
                SelectionError::Fragment(super::super::planar_bsp::FragmentError::BoundaryContact)
            ))
        ));
        assert_eq!(store_counts(&attempted), before);

        let attempted_probe = add_probe(&mut attempted);
        let control_probe = add_probe(&mut control);
        assert_eq!(attempted_probe.raw(), control_probe.raw());
        assert_eq!(store_counts(&attempted), store_counts(&control));
    }

    fn pipeline_usage(report: &OperationReport) -> Vec<LimitSnapshot> {
        report
            .usage()
            .iter()
            .copied()
            .filter(|snapshot| {
                [
                    PLANAR_BOOLEAN_BSP_WORK,
                    PLANAR_BOOLEAN_BSP_FRAGMENTS,
                    PLANAR_BOOLEAN_REALIZATION_WORK,
                    PLANAR_BOOLEAN_REALIZED_VERTICES,
                ]
                .contains(&snapshot.stage)
            })
            .collect()
    }

    fn override_at(snapshot: LimitSnapshot, allowed: u64) -> OperationSettings {
        let mode = if snapshot.resource == ResourceKind::Work {
            AccountingMode::Cumulative
        } else {
            AccountingMode::HighWater
        };
        OperationSettings::new().with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                snapshot.stage,
                snapshot.resource,
                mode,
                allowed,
            )])
            .unwrap(),
        )
    }

    #[test]
    fn every_pipeline_work_and_count_limit_accepts_n_and_refuses_n_minus_one() {
        let baseline = run_intersection(&mut rotated_fixture(), OperationSettings::new());
        assert!(baseline.result().is_ok());
        let usage = pipeline_usage(baseline.report());
        assert_eq!(usage.len(), 4);
        assert!(usage.iter().all(|snapshot| snapshot.consumed > 0));

        for snapshot in usage {
            let exact = run_intersection(
                &mut rotated_fixture(),
                override_at(snapshot, snapshot.consumed),
            );
            assert!(
                matches!(
                    exact.into_result().unwrap(),
                    PlanarBooleanPipelineOutcome::Committed(_)
                ),
                "exact boundary failed for {snapshot:?}"
            );

            let denied = run_intersection(
                &mut rotated_fixture(),
                override_at(snapshot, snapshot.consumed - 1),
            );
            let error = denied.into_result().unwrap_err();
            assert_eq!(
                error.limit(),
                Some(LimitSnapshot {
                    consumed: snapshot.consumed,
                    allowed: snapshot.consumed - 1,
                    ..snapshot
                }),
                "N-1 refusal lost exact evidence for {snapshot:?}"
            );
        }
    }
}
