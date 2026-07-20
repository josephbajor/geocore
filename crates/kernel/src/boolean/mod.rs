//! Failure-atomic Boolean operations over the certified convex-planar slice.
//!
//! The public facade is deliberately one operation family. Its implementation
//! remains a symbolic exact-plane BSP followed by conservative realization,
//! deterministic shell grouping, and one Full-checked topology transaction.
//! Unsupported geometry and incomplete proof are returned as typed refusal
//! values; no partial topology or lower-layer symbolic identity escapes.

use kcore::operation::{BudgetPlan, StageId};

use crate::operation::{
    BodyCheckReport, ChangeJournal, OperationOutcome, OperationSettings, adapt_transaction_check,
};
use crate::{BodyId, PartEdit};

#[allow(dead_code)]
mod component_layout;
#[allow(dead_code)]
mod components;
#[allow(dead_code)]
mod extract;
#[allow(dead_code)]
mod pipeline;
#[allow(dead_code)]
mod planar_bsp;
#[allow(dead_code)]
mod realize;
#[allow(dead_code)]
mod select;

/// Boolean-specific source-extraction work.
pub const BOOLEAN_SOURCE_EXTRACTION_WORK: StageId = extract::PLANAR_SOURCE_EXTRACTION_WORK;
/// Symbolic face-fragment partitioning and truth-selection work.
pub const BOOLEAN_BSP_WORK: StageId = pipeline::PLANAR_BOOLEAN_BSP_WORK;
/// High-water symbolic fragment allocation bound.
pub const BOOLEAN_BSP_FRAGMENTS: StageId = pipeline::PLANAR_BOOLEAN_BSP_FRAGMENTS;
/// Post-selection component, containment, and realization work.
pub const BOOLEAN_POST_SELECTION_WORK: StageId = pipeline::PLANAR_BOOLEAN_REALIZATION_WORK;
/// High-water plane-triple realization bound.
pub const BOOLEAN_REALIZED_VERTICES: StageId = pipeline::PLANAR_BOOLEAN_REALIZED_VERTICES;

/// Complete built-in accounting ceilings for one body/body Boolean.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BooleanBudgetProfile;

impl BooleanBudgetProfile {
    /// Return version-1 source, symbolic, realization, and Full-check defaults.
    pub fn v1_defaults() -> BudgetPlan {
        pipeline::PlanarBooleanPipelineBudgetProfile::v1_defaults()
            .overlaid(&extract::PlanarSourceExtractionBudgetProfile::v1_defaults())
            .overlaid(&ktopo::check::CheckBudgetProfile::v1_defaults(
                ktopo::check::CheckLevel::Full,
            ))
    }
}

/// Regularized CSG operation applied to the ordered operands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BooleanOperation {
    /// Material contained in either operand.
    Unite,
    /// Material contained in both operands.
    Intersect,
    /// Material in the left operand but not the right operand.
    Subtract,
}

/// Typed request for one body/body Boolean.
#[derive(Debug, Clone, PartialEq)]
pub struct BooleanBodiesRequest {
    operation: BooleanOperation,
    left: BodyId,
    right: BodyId,
    settings: OperationSettings,
}

impl BooleanBodiesRequest {
    /// Construct a request with default operation settings.
    pub fn new(operation: BooleanOperation, left: BodyId, right: BodyId) -> Self {
        Self {
            operation,
            left,
            right,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Requested regularized CSG operation.
    pub const fn operation(&self) -> BooleanOperation {
        self.operation
    }

    /// Left operand, retained unchanged by the operation.
    pub fn left(&self) -> BodyId {
        self.left.clone()
    }

    /// Right operand, retained unchanged by the operation.
    pub fn right(&self) -> BodyId {
        self.right.clone()
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Stable operand position used by public refusal evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BooleanOperand {
    /// Left request operand.
    Left,
    /// Right request operand.
    Right,
}

/// Valid operand geometry outside the first certified Boolean slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BooleanOperandUnsupportedReason {
    /// The operand is not a solid body.
    NonSolidBody,
    /// The body region ownership layout is unsupported.
    RegionLayout,
    /// The solid shell ownership layout is unsupported.
    ShellLayout,
    /// Tolerant topology is not admitted by this slice.
    TolerantEntity,
    /// A face is not planar.
    NonPlanarFace,
    /// A face does not have the required single-loop layout.
    FaceLoopLayout,
    /// An edge is not carried by a straight line.
    NonLineEdge,
    /// A source face is already split into coplanar facets.
    CoplanarFacetPartition,
    /// A source vertex is not a simple intersection of three planes.
    NonSimpleVertex,
}

/// Exact source-body obligation that could not be certified.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BooleanOperandProofFailure {
    /// A finite interior witness was unavailable.
    NonFiniteInteriorSample,
    /// A supporting plane was degenerate.
    DegenerateSupportingPlane,
    /// Stored boundary points were not certified on their support plane.
    NonPlanarBoundary,
    /// A face was not certified convex.
    NonConvexFace,
    /// The body was not certified convex.
    NonConvexBody,
    /// Source face/edge/vertex incidence was inconsistent.
    InconsistentAdjacency,
    /// The extracted symbolic face contract was not certified.
    FragmentContract,
    /// A checked source-work upper bound overflowed.
    WorkCountOverflow,
}

/// Stable, non-persistent refusal from the bounded Boolean slice.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum BooleanRefusal {
    /// Fast checking found the source body invalid.
    OperandNotFastValid {
        /// Operand that failed preflight.
        operand: BooleanOperand,
        /// Complete facade-safe Fast evidence for the still-live source body.
        report: BodyCheckReport,
    },
    /// The operand is valid but outside the supported source class.
    UnsupportedOperand {
        /// Operand outside the slice.
        operand: BooleanOperand,
        /// Exact facade-owned unsupported category.
        reason: BooleanOperandUnsupportedReason,
    },
    /// Required exact source evidence was incomplete.
    UncertifiedOperand {
        /// Operand whose proof was incomplete.
        operand: BooleanOperand,
        /// Exact facade-owned proof category.
        reason: BooleanOperandProofFailure,
    },
    /// The operands have boundary contact not admitted by this slice.
    BoundaryContact,
    /// Exact symbolic boundary classification was incomplete.
    BoundaryProofIncomplete,
    /// Symbolic boundary evidence violated the admitted contract.
    BoundaryContractViolation,
    /// Selected faces could not be certified as closed shell components.
    ShellPartitionIncomplete,
    /// A shell's exact interval winding did not exclude zero.
    ShellWindingIncomplete,
    /// A negative shell did not have one uniquely certified convex owner.
    ShellContainmentIncomplete,
    /// More than one cavity targeted a result body beyond the checker slice.
    MultipleCavitiesUnsupported,
    /// A symbolic plane-triple vertex could not be conservatively realized.
    VertexRealizationIncomplete,
    /// The certified symbolic boundary could not satisfy topology assembly.
    AssemblyRejected,
    /// Candidate topology failed before Full proof reports were available.
    CandidateTopologyInvalid {
        /// Proven topology fault count.
        fault_count: usize,
    },
    /// Full validation rejected the candidate atomically.
    FullValidationRejected {
        /// Deterministic Full evidence captured before rollback.
        ///
        /// The report body identities name rolled-back candidates and are
        /// therefore stale after this refusal.
        reports: Vec<BodyCheckReport>,
    },
    /// A deterministic work/count upper-bound calculation overflowed.
    WorkCountOverflow,
}

/// Successful Boolean result with an explicit empty/created contract.
#[derive(Debug)]
#[non_exhaustive]
pub enum BooleanResult {
    /// Exact truth selection proved the regularized result empty.
    ProvenEmpty,
    /// One or more result bodies committed atomically.
    Created(BooleanCreatedResult),
}

impl BooleanResult {
    /// Result bodies in deterministic symbolic-component order.
    pub fn bodies(&self) -> &[BodyId] {
        match self {
            Self::ProvenEmpty => &[],
            Self::Created(created) => created.bodies(),
        }
    }

    /// Whether exact truth selection proved the regularized result empty.
    pub fn is_empty(&self) -> bool {
        matches!(self, Self::ProvenEmpty)
    }

    /// Committed result and evidence, absent for a proven empty result.
    pub const fn created(&self) -> Option<&BooleanCreatedResult> {
        match self {
            Self::ProvenEmpty => None,
            Self::Created(created) => Some(created),
        }
    }

    /// Consume the result into its committed value, if nonempty.
    pub fn into_created(self) -> Option<BooleanCreatedResult> {
        match self {
            Self::ProvenEmpty => None,
            Self::Created(created) => Some(created),
        }
    }
}

/// Atomically committed Boolean bodies and their exact validation evidence.
#[derive(Debug)]
pub struct BooleanCreatedResult {
    bodies: Vec<BodyId>,
    journal: ChangeJournal,
    reports: Vec<BodyCheckReport>,
}

impl BooleanCreatedResult {
    /// One or more bodies in deterministic symbolic-component order.
    pub fn bodies(&self) -> &[BodyId] {
        &self.bodies
    }

    /// Atomic journal that persisted every result body together.
    pub const fn journal(&self) -> &ChangeJournal {
        &self.journal
    }

    /// Full reports in the same order as [`Self::bodies`].
    pub fn reports(&self) -> &[BodyCheckReport] {
        &self.reports
    }

    /// Consume the committed result into bodies, journal, and Full reports.
    pub fn into_parts(self) -> (Vec<BodyId>, ChangeJournal, Vec<BodyCheckReport>) {
        (self.bodies, self.journal, self.reports)
    }
}

/// Complete value outcome of a body/body Boolean operation.
#[derive(Debug)]
#[non_exhaustive]
pub enum BooleanOutcome {
    /// The Boolean succeeded, possibly with a proven empty result.
    Success(BooleanResult),
    /// No result topology was committed; the refusal remains explicit.
    Refused(BooleanRefusal),
}

impl PartEdit<'_> {
    /// Execute one certified body/body Boolean through a single operation scope.
    ///
    /// Wrong-part and stale operands are rejected before operation settings are
    /// validated. Source bodies remain live. Nonempty success is allocated and
    /// Full-committed atomically; refusal returns no live candidate identity.
    pub fn boolean_bodies(
        &mut self,
        request: BooleanBodiesRequest,
    ) -> crate::Result<OperationOutcome<BooleanOutcome>> {
        let BooleanBodiesRequest {
            operation,
            left,
            right,
            settings,
        } = request;
        let operation = match operation {
            BooleanOperation::Unite => select::PlanarBooleanOperation::Unite,
            BooleanOperation::Intersect => select::PlanarBooleanOperation::Intersect,
            BooleanOperation::Subtract => select::PlanarBooleanOperation::Subtract,
        };
        let part = self.id.clone();
        Ok(
            pipeline::execute_planar_boolean(self, operation, left, right, settings)?
                .map(|outcome| adapt_outcome(part, outcome)),
        )
    }
}

fn adapt_outcome(
    part: crate::PartId,
    outcome: pipeline::PlanarBooleanPipelineOutcome,
) -> BooleanOutcome {
    match outcome {
        pipeline::PlanarBooleanPipelineOutcome::ProvenEmpty => {
            BooleanOutcome::Success(BooleanResult::ProvenEmpty)
        }
        pipeline::PlanarBooleanPipelineOutcome::Committed(committed) => {
            let (bodies, journal, full_checks) = committed.into_parts();
            let reports = full_checks
                .iter()
                .map(|check| adapt_transaction_check(&part, check))
                .collect::<Vec<_>>();
            debug_assert!(
                reports
                    .iter()
                    .all(|report| report.report().outcome() == ktopo::check::CheckOutcome::Valid)
            );
            debug_assert_eq!(reports.len(), bodies.len());
            debug_assert!(
                reports
                    .iter()
                    .zip(&bodies)
                    .all(|(report, body)| report.body() == *body)
            );
            BooleanOutcome::Success(BooleanResult::Created(BooleanCreatedResult {
                bodies,
                journal: ChangeJournal::from_raw(part, journal),
                reports,
            }))
        }
        pipeline::PlanarBooleanPipelineOutcome::Refused(refusal) => {
            BooleanOutcome::Refused(adapt_refusal(&part, refusal))
        }
    }
}

fn adapt_refusal(
    part: &crate::PartId,
    refusal: pipeline::PlanarBooleanPipelineRefusal,
) -> BooleanRefusal {
    use component_layout::ComponentLayoutError;
    use pipeline::PlanarBooleanPipelineRefusal;
    use planar_bsp::FragmentError;
    use realize::RealizationError;
    use select::SelectionError;

    match refusal {
        PlanarBooleanPipelineRefusal::SourceNotFastValid { operand, report } => {
            BooleanRefusal::OperandNotFastValid {
                operand: adapt_operand(operand),
                report,
            }
        }
        PlanarBooleanPipelineRefusal::UnsupportedSource { operand, gap } => {
            BooleanRefusal::UnsupportedOperand {
                operand: adapt_operand(operand),
                reason: adapt_unsupported_operand(gap),
            }
        }
        PlanarBooleanPipelineRefusal::UncertifiedSource { operand, failure } => {
            BooleanRefusal::UncertifiedOperand {
                operand: adapt_operand(operand),
                reason: adapt_operand_proof(failure),
            }
        }
        PlanarBooleanPipelineRefusal::Symbolic(
            SelectionError::Fragment(FragmentError::BoundaryContact)
            | SelectionError::OverlappingOperandPlane,
        )
        | PlanarBooleanPipelineRefusal::ComponentLayout(ComponentLayoutError::SharedVertex)
        | PlanarBooleanPipelineRefusal::Realization(RealizationError::BoundaryContact) => {
            BooleanRefusal::BoundaryContact
        }
        PlanarBooleanPipelineRefusal::Symbolic(SelectionError::Fragment(
            FragmentError::UncertifiedPredicate | FragmentError::MissingClassification,
        )) => BooleanRefusal::BoundaryProofIncomplete,
        PlanarBooleanPipelineRefusal::Symbolic(_) => BooleanRefusal::BoundaryContractViolation,
        PlanarBooleanPipelineRefusal::ComponentPartition(_) => {
            BooleanRefusal::ShellPartitionIncomplete
        }
        PlanarBooleanPipelineRefusal::ComponentLayout(
            ComponentLayoutError::IndeterminateWinding,
        ) => BooleanRefusal::ShellWindingIncomplete,
        PlanarBooleanPipelineRefusal::ComponentLayout(
            ComponentLayoutError::MissingPositiveShell
            | ComponentLayoutError::UncertifiedContainment,
        ) => BooleanRefusal::ShellContainmentIncomplete,
        PlanarBooleanPipelineRefusal::ComponentLayout(ComponentLayoutError::MultipleCavities) => {
            BooleanRefusal::MultipleCavitiesUnsupported
        }
        PlanarBooleanPipelineRefusal::Realization(
            RealizationError::UnknownPlane
            | RealizationError::InvalidPlane
            | RealizationError::InvalidSideSet,
        ) => BooleanRefusal::BoundaryContractViolation,
        PlanarBooleanPipelineRefusal::Realization(_) => BooleanRefusal::VertexRealizationIncomplete,
        PlanarBooleanPipelineRefusal::PlaneBindingContract(_)
        | PlanarBooleanPipelineRefusal::AssemblyContract(_) => BooleanRefusal::AssemblyRejected,
        PlanarBooleanPipelineRefusal::FullTopologyFault { fault_count } => {
            BooleanRefusal::CandidateTopologyInvalid { fault_count }
        }
        PlanarBooleanPipelineRefusal::FullProofRejected(checks) => {
            let reports = checks
                .iter()
                .map(|check| adapt_transaction_check(part, check))
                .collect();
            BooleanRefusal::FullValidationRejected { reports }
        }
        PlanarBooleanPipelineRefusal::WorkCountOverflow => BooleanRefusal::WorkCountOverflow,
    }
}

fn adapt_operand(operand: u8) -> BooleanOperand {
    if operand == 0 {
        BooleanOperand::Left
    } else {
        BooleanOperand::Right
    }
}

fn adapt_unsupported_operand(reason: extract::PlanarSourceGap) -> BooleanOperandUnsupportedReason {
    use extract::PlanarSourceGap;

    match reason {
        PlanarSourceGap::NonSolidBody => BooleanOperandUnsupportedReason::NonSolidBody,
        PlanarSourceGap::RegionLayout => BooleanOperandUnsupportedReason::RegionLayout,
        PlanarSourceGap::ShellLayout => BooleanOperandUnsupportedReason::ShellLayout,
        PlanarSourceGap::TolerantEntity => BooleanOperandUnsupportedReason::TolerantEntity,
        PlanarSourceGap::NonPlanarFace => BooleanOperandUnsupportedReason::NonPlanarFace,
        PlanarSourceGap::FaceLoopLayout => BooleanOperandUnsupportedReason::FaceLoopLayout,
        PlanarSourceGap::NonLineEdge => BooleanOperandUnsupportedReason::NonLineEdge,
        PlanarSourceGap::CoplanarFacetPartition => {
            BooleanOperandUnsupportedReason::CoplanarFacetPartition
        }
        PlanarSourceGap::NonSimpleVertex => BooleanOperandUnsupportedReason::NonSimpleVertex,
    }
}

fn adapt_operand_proof(reason: extract::PlanarSourceProofFailure) -> BooleanOperandProofFailure {
    use extract::PlanarSourceProofFailure;

    match reason {
        PlanarSourceProofFailure::NonFiniteInteriorSample => {
            BooleanOperandProofFailure::NonFiniteInteriorSample
        }
        PlanarSourceProofFailure::DegenerateSupportingPlane => {
            BooleanOperandProofFailure::DegenerateSupportingPlane
        }
        PlanarSourceProofFailure::NonPlanarBoundary => {
            BooleanOperandProofFailure::NonPlanarBoundary
        }
        PlanarSourceProofFailure::NonConvexFace => BooleanOperandProofFailure::NonConvexFace,
        PlanarSourceProofFailure::NonConvexBody => BooleanOperandProofFailure::NonConvexBody,
        PlanarSourceProofFailure::InconsistentAdjacency => {
            BooleanOperandProofFailure::InconsistentAdjacency
        }
        PlanarSourceProofFailure::FragmentContract => BooleanOperandProofFailure::FragmentContract,
        PlanarSourceProofFailure::WorkCountOverflow => {
            BooleanOperandProofFailure::WorkCountOverflow
        }
    }
}
