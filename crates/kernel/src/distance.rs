//! Facade-safe certified distance interrogation for two solid bodies.
//!
//! A successful certificate encloses the Euclidean distance between the two
//! closed material sets, not merely between selected boundary entities.  In
//! particular, an enclosure `[0, upper]` does not distinguish overlap,
//! containment, contact, or an unresolved near separation.  Callers that need
//! that distinction must compose this query with classification or clash
//! evidence. Enclosures include the fixed incidence envelope accepted by Full
//! validation, even for topology without explicit tolerances. [`Part::body_clash`]
//! derives a clearance-threshold verdict from the same certificate and report;
//! it is not an overlap classifier.

use kcore::operation::OperationScope;

use crate::error::{Error, Result, capability};
use crate::operation::{
    BodyCheckReport, OperationOutcome, OperationSettings, adapt_live_body_check,
};
use crate::session::Part;
use crate::{
    BodyId, CapabilityId, EdgeId, FaceId, FinId, PartId, Point3Enclosure, ScalarEnclosure, VertexId,
};

/// Typed request to enclose the distance between two closed material sets.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyDistanceRequest {
    body_a: BodyId,
    body_b: BodyId,
    settings: OperationSettings,
}

impl BodyDistanceRequest {
    /// Construct a request using default operation settings.
    pub fn new(body_a: BodyId, body_b: BodyId) -> Self {
        Self {
            body_a,
            body_b,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// First operand body.
    pub fn body_a(&self) -> BodyId {
        self.body_a.clone()
    }

    /// Second operand body.
    pub fn body_b(&self) -> BodyId {
        self.body_b.clone()
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Typed request to assess a certified material-set clearance threshold.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyClashRequest {
    body_a: BodyId,
    body_b: BodyId,
    clearance: f64,
    settings: OperationSettings,
}

impl BodyClashRequest {
    /// Construct a request using default operation settings.
    pub fn new(body_a: BodyId, body_b: BodyId, clearance: f64) -> Self {
        Self {
            body_a,
            body_b,
            clearance,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// First operand body.
    pub fn body_a(&self) -> BodyId {
        self.body_a.clone()
    }

    /// Second operand body.
    pub fn body_b(&self) -> BodyId {
        self.body_b.clone()
    }

    /// Requested material-distance threshold, validated by [`Part::body_clash`].
    pub const fn clearance(&self) -> f64 {
        self.clearance
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Request-relative operand named by body-distance refusal evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum BodyDistanceOperand {
    /// First request operand.
    A,
    /// Second request operand.
    B,
}

/// Certified relationship between material-set distance and a clearance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum BodyClashVerdict {
    /// The certified distance lower bound is strictly above the clearance.
    Clear,
    /// The certified distance upper bound is at or below the clearance.
    ///
    /// This proves only a threshold violation. It includes contact and near
    /// clearance and does not by itself prove overlap or interference.
    Clashing,
    /// The certified distance enclosure straddles the clearance decision.
    Indeterminate,
}

/// One topology-owned feasible boundary point in an upper-bound proof.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyDistanceBoundaryWitness {
    operand: BodyDistanceOperand,
    face: FaceId,
    fin: FinId,
    edge: EdgeId,
    pcurve_parameter: ScalarEnclosure,
    point: Point3Enclosure,
}

impl BodyDistanceBoundaryWitness {
    /// Request operand owning the boundary point.
    pub const fn operand(&self) -> BodyDistanceOperand {
        self.operand
    }

    /// Opaque owning face identity.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// Opaque fin identity selecting the exact pcurve use.
    pub fn fin(&self) -> FinId {
        self.fin.clone()
    }

    /// Opaque boundary-edge identity.
    pub fn edge(&self) -> EdgeId {
        self.edge.clone()
    }

    /// Certified parameter enclosure on the selected fin pcurve.
    pub const fn pcurve_parameter(&self) -> ScalarEnclosure {
        self.pcurve_parameter
    }

    /// Certified model-space enclosure of the lifted pcurve point, including
    /// the fixed Full-validation edge-incidence envelope.
    pub const fn point(&self) -> Point3Enclosure {
        self.point
    }
}

/// Feasible point pair proving a certified body-distance upper endpoint.
///
/// This is checkable upper-bound evidence, not a closest-point result.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyDistanceUpperWitness {
    points: [BodyDistanceBoundaryWitness; 2],
    distance: ScalarEnclosure,
}

impl BodyDistanceUpperWitness {
    /// Boundary points in `(A, B)` request order.
    pub const fn points(&self) -> &[BodyDistanceBoundaryWitness; 2] {
        &self.points
    }

    /// Enclosure of the Euclidean distance between the feasible points.
    pub const fn distance(&self) -> ScalarEnclosure {
        self.distance
    }
}

/// Certified distance enclosure tied to the exact ordered request identities.
#[derive(Debug, Clone, PartialEq)]
pub struct CertifiedBodyDistance {
    bodies: [BodyId; 2],
    distance: ScalarEnclosure,
    upper_witness: BodyDistanceUpperWitness,
}

impl CertifiedBodyDistance {
    /// Exact operand identities in `(A, B)` request order.
    pub fn bodies(&self) -> [BodyId; 2] {
        self.bodies.clone()
    }

    /// First request operand.
    pub fn body_a(&self) -> BodyId {
        self.bodies[0].clone()
    }

    /// Second request operand.
    pub fn body_b(&self) -> BodyId {
        self.bodies[1].clone()
    }

    /// Body named by a request-relative operand.
    pub fn body(&self, operand: BodyDistanceOperand) -> BodyId {
        match operand {
            BodyDistanceOperand::A => self.body_a(),
            BodyDistanceOperand::B => self.body_b(),
        }
    }

    /// Certified enclosure of the distance between the closed material sets.
    ///
    /// A zero lower bound alone does not certify why zero remains possible:
    /// the bodies may overlap, contain one another, touch, or have a near
    /// separation that the current arithmetic did not resolve.
    pub const fn distance(&self) -> ScalarEnclosure {
        self.distance
    }

    /// Feasible topology-owned point pair proving the upper endpoint.
    ///
    /// The pair is not asserted to minimize distance.
    pub const fn upper_witness(&self) -> &BodyDistanceUpperWitness {
        &self.upper_witness
    }
}

/// Certified clearance assessment tied to the exact ordered body identities.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyClashAssessment {
    clearance: f64,
    verdict: BodyClashVerdict,
    distance: CertifiedBodyDistance,
}

impl BodyClashAssessment {
    /// Canonical finite nonnegative threshold used by the assessment.
    ///
    /// Authored negative zero is retained by [`BodyClashRequest`] but is
    /// canonicalized to positive zero here.
    pub const fn clearance(&self) -> f64 {
        self.clearance
    }

    /// Certified threshold relationship.
    pub const fn verdict(&self) -> BodyClashVerdict {
        self.verdict
    }

    /// Complete distance certificate consumed by this assessment.
    pub const fn distance(&self) -> &CertifiedBodyDistance {
        &self.distance
    }

    /// Certified material-set distance enclosure.
    pub const fn enclosure(&self) -> ScalarEnclosure {
        self.distance.distance()
    }

    /// Exact operand identities in `(A, B)` request order.
    pub fn bodies(&self) -> [BodyId; 2] {
        self.distance.bodies()
    }

    /// First request operand.
    pub fn body_a(&self) -> BodyId {
        self.distance.body_a()
    }

    /// Second request operand.
    pub fn body_b(&self) -> BodyId {
        self.distance.body_b()
    }

    /// Body named by a request-relative operand.
    pub fn body(&self, operand: BodyDistanceOperand) -> BodyId {
        self.distance.body(operand)
    }
}

/// Why a valid facade request did not produce a certified body distance.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum BodyDistanceRefusal {
    /// Full validation found faults or unresolved proof obligations.
    BodyNotFullValid {
        /// Operand whose Full report was not valid.
        operand: BodyDistanceOperand,
    },
    /// One operand is not a three-dimensional solid.
    NonSolidBody {
        /// Operand outside the solid-body contract.
        operand: BodyDistanceOperand,
    },
    /// Exact distance does not consume a tolerant face in this slice.
    TolerantFace {
        /// Operand owning the face.
        operand: BodyDistanceOperand,
        /// Opaque face identity at the proof boundary.
        face: FaceId,
    },
    /// Exact distance does not consume a tolerant edge in this slice.
    TolerantEdge {
        /// Operand owning the edge.
        operand: BodyDistanceOperand,
        /// Opaque edge identity at the proof boundary.
        edge: EdgeId,
    },
    /// Exact distance does not consume a tolerant vertex in this slice.
    TolerantVertex {
        /// Operand owning the vertex.
        operand: BodyDistanceOperand,
        /// Opaque vertex identity at the proof boundary.
        vertex: VertexId,
    },
    /// A face lacks the trimmed-domain evidence needed by the theorem.
    MissingFaceDomain {
        /// Operand owning the face.
        operand: BodyDistanceOperand,
        /// Opaque face identity at the proof boundary.
        face: FaceId,
    },
    /// A face uses a supporting surface outside the admitted analytic slice.
    UnsupportedSurface {
        /// Operand owning the face.
        operand: BodyDistanceOperand,
        /// Opaque face identity at the proof boundary.
        face: FaceId,
    },
    /// A face boundary uses a pcurve outside the admitted analytic slice.
    UnsupportedPcurve {
        /// Operand owning the face.
        operand: BodyDistanceOperand,
        /// Opaque face identity at the proof boundary.
        face: FaceId,
    },
    /// A nominal solid carries lower-dimensional shell attachments.
    MixedDimensionalBody {
        /// Operand outside the pure solid-material contract.
        operand: BodyDistanceOperand,
    },
    /// The admitted operand did not yield a finite material-set witness from
    /// which to construct a certified upper bound.
    NoUpperWitness {
        /// Operand for which witness construction failed closed.
        operand: BodyDistanceOperand,
    },
    /// Outward arithmetic did not produce a finite ordered enclosure.
    IndeterminateEnclosure,
}

impl BodyDistanceRefusal {
    /// Request-relative operand named by this refusal, when there is one.
    pub const fn operand(&self) -> Option<BodyDistanceOperand> {
        match self {
            Self::BodyNotFullValid { operand }
            | Self::NonSolidBody { operand }
            | Self::TolerantFace { operand, .. }
            | Self::TolerantEdge { operand, .. }
            | Self::TolerantVertex { operand, .. }
            | Self::MissingFaceDomain { operand, .. }
            | Self::UnsupportedSurface { operand, .. }
            | Self::UnsupportedPcurve { operand, .. }
            | Self::MixedDimensionalBody { operand }
            | Self::NoUpperWitness { operand } => Some(*operand),
            Self::IndeterminateEnclosure => None,
        }
    }

    /// Missing finite-support capability, when this is an unsupported case.
    pub const fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::TolerantFace { .. }
            | Self::TolerantEdge { .. }
            | Self::TolerantVertex { .. }
            | Self::MissingFaceDomain { .. }
            | Self::UnsupportedSurface { .. }
            | Self::UnsupportedPcurve { .. }
            | Self::MixedDimensionalBody { .. }
            | Self::NoUpperWitness { .. } => Some(capability::ANALYTIC_BODY_DISTANCE),
            Self::BodyNotFullValid { .. }
            | Self::NonSolidBody { .. }
            | Self::IndeterminateEnclosure => None,
        }
    }
}

/// Full-check evidence paired with a certified distance or a typed refusal.
// The certified distance stays inline so the public outcome hands the
// certificate off by value without indirection.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum BodyDistanceOutcome {
    /// Both Full checks and the whole-body distance theorem certified.
    Certified {
        /// Certified distance tied to the ordered request identities.
        distance: CertifiedBodyDistance,
        /// Full checker evidence in `(A, B)` request order.
        full_checks: [BodyCheckReport; 2],
    },
    /// The request was valid but outside the current proof boundary.
    Refused {
        /// Typed refusal reason.
        reason: BodyDistanceRefusal,
        /// Full checker evidence in `(A, B)` request order.
        full_checks: [BodyCheckReport; 2],
    },
}

impl BodyDistanceOutcome {
    /// Full checker reports retained in `(A, B)` request order.
    pub const fn full_checks(&self) -> &[BodyCheckReport; 2] {
        match self {
            Self::Certified { full_checks, .. } | Self::Refused { full_checks, .. } => full_checks,
        }
    }

    /// Certified distance, if the theorem completed.
    pub const fn distance(&self) -> Option<&CertifiedBodyDistance> {
        match self {
            Self::Certified { distance, .. } => Some(distance),
            Self::Refused { .. } => None,
        }
    }

    /// Typed refusal, if the theorem failed closed.
    pub const fn refusal(&self) -> Option<&BodyDistanceRefusal> {
        match self {
            Self::Certified { .. } => None,
            Self::Refused { reason, .. } => Some(reason),
        }
    }
}

/// Full-check evidence paired with a clearance assessment or distance refusal.
// The certified assessment stays inline so the public outcome hands the
// certificate off by value without indirection.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum BodyClashOutcome {
    /// The distance certificate supports a threshold assessment.
    Assessed {
        /// Certified clearance assessment.
        assessment: BodyClashAssessment,
        /// Full checker evidence in `(A, B)` request order.
        full_checks: [BodyCheckReport; 2],
    },
    /// The distance request was valid but outside the current proof boundary.
    Refused {
        /// Unchanged distance-refusal reason.
        reason: BodyDistanceRefusal,
        /// Full checker evidence in `(A, B)` request order.
        full_checks: [BodyCheckReport; 2],
    },
}

impl BodyClashOutcome {
    /// Full checker reports retained in `(A, B)` request order.
    pub const fn full_checks(&self) -> &[BodyCheckReport; 2] {
        match self {
            Self::Assessed { full_checks, .. } | Self::Refused { full_checks, .. } => full_checks,
        }
    }

    /// Certified clearance assessment, if distance certification completed.
    pub const fn assessment(&self) -> Option<&BodyClashAssessment> {
        match self {
            Self::Assessed { assessment, .. } => Some(assessment),
            Self::Refused { .. } => None,
        }
    }

    /// Typed distance refusal, if the proof failed closed.
    pub const fn refusal(&self) -> Option<&BodyDistanceRefusal> {
        match self {
            Self::Assessed { .. } => None,
            Self::Refused { reason, .. } => Some(reason),
        }
    }

    /// Certified threshold relationship, when assessed.
    pub const fn verdict(&self) -> Option<BodyClashVerdict> {
        match self {
            Self::Assessed { assessment, .. } => Some(assessment.verdict()),
            Self::Refused { .. } => None,
        }
    }

    /// Complete distance certificate consumed by the assessment.
    pub const fn distance(&self) -> Option<&CertifiedBodyDistance> {
        match self {
            Self::Assessed { assessment, .. } => Some(assessment.distance()),
            Self::Refused { .. } => None,
        }
    }
}

impl Part<'_> {
    /// Enclose the distance between two closed solid material sets in one
    /// operation scope.
    ///
    /// Operand A is validated before B, and both identities are validated
    /// before distinctness, settings, or scope construction.  The two Full
    /// checks and analytic distance work share the returned accounting report.
    /// The interval carries outward the Full check's fixed incidence envelope.
    /// A certified enclosure beginning at zero does not by itself distinguish
    /// overlap, containment, contact, or unresolved near separation.
    pub fn body_distance(
        &self,
        request: BodyDistanceRequest,
    ) -> Result<OperationOutcome<BodyDistanceOutcome>> {
        let BodyDistanceRequest {
            body_a,
            body_b,
            settings,
        } = request;
        validate_body_pair(
            self,
            &body_a,
            &body_b,
            "body distance requires two distinct operand bodies",
        )?;
        run_body_distance_operation(self, [body_a, body_b], settings)
    }

    /// Assess a finite nonnegative clearance from one certified material-set
    /// distance operation.
    ///
    /// Operand A is validated before B. Distinctness is then checked before
    /// clearance validity, and clearance validity precedes settings or scope
    /// construction. The query runs exactly the distance theorem's two Full
    /// checks and analytic work in one scope and reuses their reports.
    ///
    /// [`BodyClashVerdict::Clashing`] proves only that the certified distance
    /// is at or below the requested threshold. That includes contact and near
    /// clearance and does not prove overlap or interference. In particular, a
    /// zero lower bound never produces `Clashing` unless the upper bound also
    /// lies at or below the threshold.
    pub fn body_clash(
        &self,
        request: BodyClashRequest,
    ) -> Result<OperationOutcome<BodyClashOutcome>> {
        let BodyClashRequest {
            body_a,
            body_b,
            clearance,
            settings,
        } = request;
        validate_body_pair(
            self,
            &body_a,
            &body_b,
            "body clash requires two distinct operand bodies",
        )?;
        if !(clearance.is_finite() && clearance >= 0.0) {
            return Err(Error::Core {
                source: kcore::error::Error::InvalidGeometry {
                    reason: "body clash clearance must be finite and nonnegative",
                },
            });
        }
        let clearance = if clearance == 0.0 { 0.0 } else { clearance };
        Ok(
            run_body_distance_operation(self, [body_a, body_b], settings)?
                .map(|outcome| assess_clash_outcome(outcome, clearance)),
        )
    }
}

fn validate_body_pair(
    part: &Part<'_>,
    body_a: &BodyId,
    body_b: &BodyId,
    distinct_reason: &'static str,
) -> Result<()> {
    part.body(body_a.clone())?;
    part.body(body_b.clone())?;
    if body_a == body_b {
        return Err(Error::Core {
            source: kcore::error::Error::InvalidGeometry {
                reason: distinct_reason,
            },
        });
    }
    Ok(())
}

fn run_body_distance_operation(
    part: &Part<'_>,
    bodies: [BodyId; 2],
    settings: OperationSettings,
) -> Result<OperationOutcome<BodyDistanceOutcome>> {
    let defaults = ktopo::body_distance::BodyDistanceBudgetProfile::v1_defaults();
    let context = settings
        .context(part.policy)?
        .with_family_budget_defaults(defaults.clone());
    let effective = context.effective_budget();
    for required in defaults.limits() {
        effective.require_limit(required.stage, required.resource, required.mode)?;
    }

    let mut scope = OperationScope::new(&context);
    let lower = ktopo::body_distance::certify_body_distance_in_scope(
        &part.state.store,
        bodies[0].raw(),
        bodies[1].raw(),
        &mut scope,
    );
    let result = lower
        .map_err(Error::from)
        .and_then(|outcome| adapt_outcome(&part.id, &part.state.store, bodies, outcome));
    Ok(scope.finish_typed(result))
}

fn assess_clash_outcome(outcome: BodyDistanceOutcome, clearance: f64) -> BodyClashOutcome {
    match outcome {
        BodyDistanceOutcome::Certified {
            distance,
            full_checks,
        } => {
            let enclosure = distance.distance();
            let verdict = clash_verdict(enclosure.lower(), enclosure.upper(), clearance);
            BodyClashOutcome::Assessed {
                assessment: BodyClashAssessment {
                    clearance,
                    verdict,
                    distance,
                },
                full_checks,
            }
        }
        BodyDistanceOutcome::Refused {
            reason,
            full_checks,
        } => BodyClashOutcome::Refused {
            reason,
            full_checks,
        },
    }
}

const fn clash_verdict(lower: f64, upper: f64, clearance: f64) -> BodyClashVerdict {
    if lower > clearance {
        BodyClashVerdict::Clear
    } else if upper <= clearance {
        BodyClashVerdict::Clashing
    } else {
        BodyClashVerdict::Indeterminate
    }
}

fn adapt_outcome(
    part: &PartId,
    store: &ktopo::store::Store,
    bodies: [BodyId; 2],
    outcome: ktopo::body_distance::BodyDistanceOutcome,
) -> Result<BodyDistanceOutcome> {
    Ok(match outcome {
        ktopo::body_distance::BodyDistanceOutcome::Certified {
            distance,
            upper_witness,
            full_checks,
        } => BodyDistanceOutcome::Certified {
            distance: CertifiedBodyDistance {
                bodies: bodies.clone(),
                distance: ScalarEnclosure::from_lower(distance),
                upper_witness: adapt_upper_witness(part, upper_witness),
            },
            full_checks: adapt_full_checks(part, store, &bodies, full_checks)?,
        },
        ktopo::body_distance::BodyDistanceOutcome::Refused {
            reason,
            full_checks,
        } => BodyDistanceOutcome::Refused {
            reason: adapt_refusal(part, reason),
            full_checks: adapt_full_checks(part, store, &bodies, full_checks)?,
        },
    })
}

fn adapt_upper_witness(
    part: &PartId,
    witness: ktopo::body_distance::BodyDistanceUpperWitness,
) -> BodyDistanceUpperWitness {
    let [first, second] = witness.points();
    let adapt = |operand, point: ktopo::body_distance::BodyDistanceBoundaryWitness| {
        BodyDistanceBoundaryWitness {
            operand,
            face: FaceId::new(part.clone(), point.face()),
            fin: FinId::new(part.clone(), point.fin()),
            edge: EdgeId::new(part.clone(), point.edge()),
            pcurve_parameter: ScalarEnclosure::from_lower(point.pcurve_parameter()),
            point: Point3Enclosure::from_lower(point.point()),
        }
    };
    BodyDistanceUpperWitness {
        points: [
            adapt(BodyDistanceOperand::A, first),
            adapt(BodyDistanceOperand::B, second),
        ],
        distance: ScalarEnclosure::from_lower(witness.distance()),
    }
}

fn adapt_full_checks(
    part: &PartId,
    store: &ktopo::store::Store,
    bodies: &[BodyId; 2],
    full_checks: [ktopo::check::CheckReport; 2],
) -> Result<[BodyCheckReport; 2]> {
    let [check_a, check_b] = full_checks;
    Ok([
        adapt_live_body_check(part, store, bodies[0].raw(), check_a)?,
        adapt_live_body_check(part, store, bodies[1].raw(), check_b)?,
    ])
}

const fn adapt_operand(operand: ktopo::body_distance::BodyDistanceOperand) -> BodyDistanceOperand {
    match operand {
        ktopo::body_distance::BodyDistanceOperand::First => BodyDistanceOperand::A,
        ktopo::body_distance::BodyDistanceOperand::Second => BodyDistanceOperand::B,
    }
}

fn adapt_refusal(
    part: &PartId,
    refusal: ktopo::body_distance::BodyDistanceRefusal,
) -> BodyDistanceRefusal {
    match refusal {
        ktopo::body_distance::BodyDistanceRefusal::BodyNotFullValid { operand } => {
            BodyDistanceRefusal::BodyNotFullValid {
                operand: adapt_operand(operand),
            }
        }
        ktopo::body_distance::BodyDistanceRefusal::NonSolidBody { operand } => {
            BodyDistanceRefusal::NonSolidBody {
                operand: adapt_operand(operand),
            }
        }
        ktopo::body_distance::BodyDistanceRefusal::TolerantFace { operand, face } => {
            BodyDistanceRefusal::TolerantFace {
                operand: adapt_operand(operand),
                face: FaceId::new(part.clone(), face),
            }
        }
        ktopo::body_distance::BodyDistanceRefusal::TolerantEdge { operand, edge } => {
            BodyDistanceRefusal::TolerantEdge {
                operand: adapt_operand(operand),
                edge: EdgeId::new(part.clone(), edge),
            }
        }
        ktopo::body_distance::BodyDistanceRefusal::TolerantVertex { operand, vertex } => {
            BodyDistanceRefusal::TolerantVertex {
                operand: adapt_operand(operand),
                vertex: VertexId::new(part.clone(), vertex),
            }
        }
        ktopo::body_distance::BodyDistanceRefusal::MissingFaceDomain { operand, face } => {
            BodyDistanceRefusal::MissingFaceDomain {
                operand: adapt_operand(operand),
                face: FaceId::new(part.clone(), face),
            }
        }
        ktopo::body_distance::BodyDistanceRefusal::UnsupportedSurface { operand, face } => {
            BodyDistanceRefusal::UnsupportedSurface {
                operand: adapt_operand(operand),
                face: FaceId::new(part.clone(), face),
            }
        }
        ktopo::body_distance::BodyDistanceRefusal::UnsupportedPcurve { operand, face } => {
            BodyDistanceRefusal::UnsupportedPcurve {
                operand: adapt_operand(operand),
                face: FaceId::new(part.clone(), face),
            }
        }
        ktopo::body_distance::BodyDistanceRefusal::MixedDimensionalBody { operand } => {
            BodyDistanceRefusal::MixedDimensionalBody {
                operand: adapt_operand(operand),
            }
        }
        ktopo::body_distance::BodyDistanceRefusal::NoUpperWitness { operand } => {
            BodyDistanceRefusal::NoUpperWitness {
                operand: adapt_operand(operand),
            }
        }
        ktopo::body_distance::BodyDistanceRefusal::IndeterminateEnclosure => {
            BodyDistanceRefusal::IndeterminateEnclosure
        }
        // The lower refusal is intentionally non-exhaustive. A newer lower
        // reason must never be promoted to a certificate by an older facade.
        _ => BodyDistanceRefusal::IndeterminateEnclosure,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AccountingMode, BlockRequest, BudgetPlan, CheckOutcome, DiagnosticLevel, ErrorClass,
        ExecutionPolicy, Frame, Kernel, KernelError, LimitSpec, NumericalPolicy, Point3,
        PolicyVersion, ResourceKind, Session, SessionPolicy, SessionPrecision, Tolerances, Vec3,
    };

    fn add_block(session: &mut Session, part: &PartId, frame: Frame) -> BodyId {
        add_block_with_extents(session, part, frame, [2.0, 2.0, 2.0])
    }

    fn add_block_with_extents(
        session: &mut Session,
        part: &PartId,
        frame: Frame,
        extents: [f64; 3],
    ) -> BodyId {
        session
            .edit_part(part.clone())
            .unwrap()
            .create_block(BlockRequest::new(frame, extents))
            .unwrap()
            .into_result()
            .unwrap()
            .body()
    }

    fn translated_frame(x: f64) -> Frame {
        Frame::new(
            Point3::new(x, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap()
    }

    #[test]
    fn request_identity_order_precedes_distinctness_settings_and_scope() {
        let mut session = Kernel::new().create_session();
        let receiving = session.create_part();
        let first_wrong = session.create_part();
        let second_wrong = session.create_part();
        let body_a = add_block(&mut session, &first_wrong, Frame::world());
        let body_b = add_block(&mut session, &second_wrong, translated_frame(5.0));

        let result = session
            .part(receiving.clone())
            .unwrap()
            .body_distance(BodyDistanceRequest::new(body_a.clone(), body_b.clone()));
        assert!(matches!(
            result,
            Err(KernelError::WrongPart { expected, actual })
                if expected == receiving && actual == first_wrong
        ));

        let valid_a = add_block(&mut session, &receiving, Frame::world());
        let result = session
            .part(receiving.clone())
            .unwrap()
            .body_distance(BodyDistanceRequest::new(valid_a, body_b));
        assert!(matches!(
            result,
            Err(KernelError::WrongPart { expected, actual })
                if expected == receiving && actual == second_wrong
        ));

        let strict_policy = SessionPolicy::new(
            SessionPrecision::try_new(1.0e-6, 1.0e-11, 500.0).unwrap(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            BudgetPlan::empty(),
            PolicyVersion::V1,
        );
        let mut strict = Kernel::with_default_policy(strict_policy).create_session();
        let strict_part = strict.create_part();
        let valid_settings =
            OperationSettings::new().with_tolerances(Tolerances::with_linear(1.0e-6).unwrap());
        let body = strict
            .edit_part(strict_part.clone())
            .unwrap()
            .create_block(
                BlockRequest::new(Frame::world(), [2.0, 2.0, 2.0]).with_settings(valid_settings),
            )
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let result = strict
            .part(strict_part)
            .unwrap()
            .body_distance(BodyDistanceRequest::new(body.clone(), body));
        assert!(matches!(
            result,
            Err(KernelError::Core {
                source: kcore::error::Error::InvalidGeometry { .. }
            })
        ));
    }

    #[test]
    fn repeated_and_swapped_queries_preserve_bits_order_and_accessors() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body_a = add_block(&mut session, &part_id, Frame::world());
        let body_b = add_block(&mut session, &part_id, translated_frame(5.0));
        let settings = OperationSettings::new().with_diagnostics(DiagnosticLevel::Summary, 4);
        let request = BodyDistanceRequest::new(body_a.clone(), body_b.clone())
            .with_settings(settings.clone());
        assert_eq!(request.body_a(), body_a);
        assert_eq!(request.body_b(), body_b);
        assert_eq!(request.settings(), &settings);

        let part = session.part(part_id).unwrap();
        let run = |left: BodyId, right: BodyId| {
            part.body_distance(BodyDistanceRequest::new(left, right))
                .unwrap()
                .into_result()
                .unwrap()
        };
        let first = run(body_a.clone(), body_b.clone());
        let repeated = run(body_a.clone(), body_b.clone());
        let swapped = run(body_b.clone(), body_a.clone());

        let first_distance = first.distance().expect("separated blocks must certify");
        let repeated_distance = repeated.distance().expect("repeat must certify");
        let swapped_distance = swapped.distance().expect("swap must certify");
        assert_eq!(first.refusal(), None);
        assert_eq!(first_distance.bodies(), [body_a.clone(), body_b.clone()]);
        assert_eq!(first_distance.body_a(), body_a);
        assert_eq!(first_distance.body_b(), body_b);
        assert_eq!(first_distance.body(BodyDistanceOperand::A), body_a);
        assert_eq!(first_distance.body(BodyDistanceOperand::B), body_b);
        assert_eq!(swapped_distance.bodies(), [body_b.clone(), body_a.clone()]);
        assert!(first_distance.distance().contains(3.0));
        assert_eq!(
            first_distance.upper_witness().distance().upper(),
            first_distance.distance().upper()
        );
        let [first_a, first_b] = first_distance.upper_witness().points();
        let [swapped_b, swapped_a] = swapped_distance.upper_witness().points();
        assert_eq!(first_a.operand(), BodyDistanceOperand::A);
        assert_eq!(first_b.operand(), BodyDistanceOperand::B);
        assert_eq!(swapped_b.operand(), BodyDistanceOperand::A);
        assert_eq!(swapped_a.operand(), BodyDistanceOperand::B);
        assert_eq!(first_a.face(), swapped_a.face());
        assert_eq!(first_b.face(), swapped_b.face());
        assert_eq!(first_a.fin(), swapped_a.fin());
        assert_eq!(first_b.fin(), swapped_b.fin());
        assert_eq!(first_a.edge(), swapped_a.edge());
        assert_eq!(first_b.edge(), swapped_b.edge());
        assert_eq!(first_a.pcurve_parameter(), swapped_a.pcurve_parameter());
        assert_eq!(first_b.pcurve_parameter(), swapped_b.pcurve_parameter());
        assert_eq!(first_a.point(), swapped_a.point());
        assert_eq!(first_b.point(), swapped_b.point());
        assert_eq!(
            [
                first_distance.distance().lower().to_bits(),
                first_distance.distance().upper().to_bits(),
            ],
            [
                repeated_distance.distance().lower().to_bits(),
                repeated_distance.distance().upper().to_bits(),
            ]
        );
        assert_eq!(
            [
                first_distance.distance().lower().to_bits(),
                first_distance.distance().upper().to_bits(),
            ],
            [
                swapped_distance.distance().lower().to_bits(),
                swapped_distance.distance().upper().to_bits(),
            ]
        );
        assert_eq!(first.full_checks()[0].body(), body_a);
        assert_eq!(first.full_checks()[1].body(), body_b);
        assert!(
            first
                .full_checks()
                .iter()
                .all(|check| check.report().outcome() == CheckOutcome::Valid)
        );

        let refusal = BodyDistanceRefusal::NoUpperWitness {
            operand: BodyDistanceOperand::B,
        };
        assert_eq!(refusal.operand(), Some(BodyDistanceOperand::B));
        assert_eq!(
            refusal.capability(),
            Some(capability::ANALYTIC_BODY_DISTANCE)
        );
        assert_eq!(BodyDistanceRefusal::IndeterminateEnclosure.operand(), None);
    }

    #[test]
    fn clash_verdict_uses_only_the_certified_endpoint_theorem() {
        assert_eq!(clash_verdict(3.0, 4.0, 2.0), BodyClashVerdict::Clear);
        assert_eq!(
            clash_verdict(3.0, 4.0, 3.0),
            BodyClashVerdict::Indeterminate
        );
        assert_eq!(
            clash_verdict(3.0, 4.0, 3.5),
            BodyClashVerdict::Indeterminate
        );
        assert_eq!(clash_verdict(3.0, 4.0, 4.0), BodyClashVerdict::Clashing);
        assert_eq!(
            clash_verdict(0.0, 1.0, 0.0),
            BodyClashVerdict::Indeterminate,
            "a zero lower bound alone must never imply clash"
        );
    }

    #[test]
    fn clash_reuses_one_distance_report_and_is_deterministic_under_swap() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body_a = add_block(&mut session, &part_id, Frame::world());
        let body_b = add_block(&mut session, &part_id, translated_frame(5.0));
        let settings = OperationSettings::new().with_diagnostics(DiagnosticLevel::Summary, 4);
        let authored = BodyClashRequest::new(body_a.clone(), body_b.clone(), -0.0)
            .with_settings(settings.clone());
        assert_eq!(authored.body_a(), body_a);
        assert_eq!(authored.body_b(), body_b);
        assert_eq!(authored.clearance().to_bits(), (-0.0_f64).to_bits());
        assert_eq!(authored.settings(), &settings);

        let part = session.part(part_id).unwrap();
        let direct = part
            .body_distance(
                BodyDistanceRequest::new(body_a.clone(), body_b.clone())
                    .with_settings(settings.clone()),
            )
            .unwrap();
        let clash = part.body_clash(authored).unwrap();
        let repeated = part
            .body_clash(
                BodyClashRequest::new(body_a.clone(), body_b.clone(), -0.0)
                    .with_settings(settings.clone()),
            )
            .unwrap();
        let swapped = part
            .body_clash(
                BodyClashRequest::new(body_b.clone(), body_a.clone(), -0.0).with_settings(settings),
            )
            .unwrap();

        assert_eq!(clash.report(), direct.report());
        assert_eq!(repeated.report(), clash.report());
        assert_eq!(swapped.report(), clash.report());
        let distance_outcome = direct.result().unwrap();
        let distance = distance_outcome.distance().unwrap();
        let assessment = clash.result().unwrap().assessment().unwrap();
        let repeated_assessment = repeated.result().unwrap().assessment().unwrap();
        let swapped_assessment = swapped.result().unwrap().assessment().unwrap();
        assert_eq!(assessment.clearance().to_bits(), 0.0_f64.to_bits());
        assert_eq!(assessment.verdict(), BodyClashVerdict::Clear);
        assert_eq!(assessment.enclosure(), distance.distance());
        assert_eq!(assessment.distance(), distance);
        assert_eq!(assessment.bodies(), [body_a.clone(), body_b.clone()]);
        assert_eq!(assessment.body_a(), body_a);
        assert_eq!(assessment.body_b(), body_b);
        assert_eq!(assessment.body(BodyDistanceOperand::A), body_a);
        assert_eq!(assessment.body(BodyDistanceOperand::B), body_b);
        assert_eq!(
            clash.result().unwrap().verdict(),
            Some(BodyClashVerdict::Clear)
        );
        assert_eq!(clash.result().unwrap().distance(), Some(distance));
        assert_eq!(clash.result().unwrap().refusal(), None);
        assert_eq!(
            clash.result().unwrap().full_checks(),
            distance_outcome.full_checks()
        );
        assert_eq!(repeated_assessment, assessment);
        assert_eq!(swapped_assessment.verdict(), assessment.verdict());
        assert_eq!(swapped_assessment.enclosure(), assessment.enclosure());
        assert_eq!(
            swapped_assessment.bodies(),
            [body_b.clone(), body_a.clone()]
        );
    }

    #[test]
    fn clash_forwards_distance_refusal_and_full_checks_unchanged() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let solid = add_block(&mut session, &part_id, Frame::world());
        let non_solid = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let raw =
                ktopo::make::acorn(edit.store_mut_for_test(), Point3::new(5.0, 0.0, 0.0)).unwrap();
            BodyId::new(part_id.clone(), raw)
        };
        let part = session.part(part_id).unwrap();
        let direct = part
            .body_distance(BodyDistanceRequest::new(solid.clone(), non_solid.clone()))
            .unwrap();
        let clash = part
            .body_clash(BodyClashRequest::new(solid, non_solid, 0.25))
            .unwrap();
        assert_eq!(clash.report(), direct.report());

        let BodyDistanceOutcome::Refused {
            reason: distance_reason,
            full_checks: distance_checks,
        } = direct.result().unwrap()
        else {
            panic!("non-solid distance operand must be refused")
        };
        let BodyClashOutcome::Refused {
            reason: clash_reason,
            full_checks: clash_checks,
        } = clash.result().unwrap()
        else {
            panic!("distance refusal must remain a clash refusal")
        };
        assert_eq!(clash_reason, distance_reason);
        assert_eq!(clash_checks, distance_checks);
        assert_eq!(clash.result().unwrap().refusal(), Some(distance_reason));
        assert_eq!(clash.result().unwrap().assessment(), None);
        assert_eq!(clash.result().unwrap().verdict(), None);
        assert_eq!(clash.result().unwrap().distance(), None);
    }

    #[test]
    fn clash_preserves_distance_exact_n_and_n_minus_one_budget_boundary() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body_a = add_block(&mut session, &part_id, Frame::world());
        let body_b = add_block(&mut session, &part_id, translated_frame(5.0));
        let part = session.part(part_id).unwrap();
        let request = |allowed| {
            BodyClashRequest::new(body_a.clone(), body_b.clone(), 0.0).with_settings(
                OperationSettings::new().with_budget_overrides(
                    BudgetPlan::new([LimitSpec::new(
                        crate::BODY_DISTANCE_ANALYTIC_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        allowed,
                    )])
                    .unwrap(),
                ),
            )
        };

        let baseline = part
            .body_clash(BodyClashRequest::new(body_a.clone(), body_b.clone(), 0.0))
            .unwrap();
        let consumed = baseline
            .report()
            .usage()
            .iter()
            .find(|usage| {
                usage.stage == crate::BODY_DISTANCE_ANALYTIC_WORK
                    && usage.resource == ResourceKind::Work
            })
            .expect("distance analytic stage was not charged")
            .consumed;
        assert!(consumed > 0);

        let exact = part.body_clash(request(consumed)).unwrap();
        assert!(exact.result().is_ok());
        let denied = part.body_clash(request(consumed - 1)).unwrap();
        let error = denied.result().unwrap_err();
        assert_eq!(error.class(), ErrorClass::ResourceLimit);
        let limit = error.limit().expect("resource failure lost its limit");
        assert_eq!(limit.stage, crate::BODY_DISTANCE_ANALYTIC_WORK);
        assert_eq!(limit.consumed, consumed);
        assert_eq!(limit.allowed, consumed - 1);
    }

    #[test]
    fn clash_identity_and_distinctness_precede_threshold_and_settings() {
        let mut session = Kernel::new().create_session();
        let receiving = session.create_part();
        let first_wrong = session.create_part();
        let second_wrong = session.create_part();
        let body_a = add_block(&mut session, &first_wrong, Frame::world());
        let body_b = add_block(&mut session, &second_wrong, translated_frame(5.0));

        let result = session
            .part(receiving.clone())
            .unwrap()
            .body_clash(BodyClashRequest::new(body_a, body_b.clone(), f64::NAN));
        assert!(matches!(
            result,
            Err(KernelError::WrongPart { expected, actual })
                if expected == receiving && actual == first_wrong
        ));
        let valid_a = add_block(&mut session, &receiving, Frame::world());
        let result = session
            .part(receiving.clone())
            .unwrap()
            .body_clash(BodyClashRequest::new(valid_a.clone(), body_b, f64::NAN));
        assert!(matches!(
            result,
            Err(KernelError::WrongPart { expected, actual })
                if expected == receiving && actual == second_wrong
        ));
        let result = session
            .part(receiving.clone())
            .unwrap()
            .body_clash(BodyClashRequest::new(
                valid_a.clone(),
                valid_a.clone(),
                f64::NAN,
            ));
        assert!(matches!(
            result,
            Err(KernelError::Core {
                source: kcore::error::Error::InvalidGeometry {
                    reason: "body clash requires two distinct operand bodies"
                }
            })
        ));

        let valid_b = add_block(&mut session, &receiving, translated_frame(5.0));
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
            let result =
                session
                    .part(receiving.clone())
                    .unwrap()
                    .body_clash(BodyClashRequest::new(
                        valid_b.clone(),
                        valid_a.clone(),
                        invalid,
                    ));
            assert!(matches!(
                result,
                Err(KernelError::Core {
                    source: kcore::error::Error::InvalidGeometry {
                        reason: "body clash clearance must be finite and nonnegative"
                    }
                })
            ));
        }

        let strict_policy = SessionPolicy::new(
            SessionPrecision::try_new(1.0e-6, 1.0e-11, 500.0).unwrap(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            BudgetPlan::empty(),
            PolicyVersion::V1,
        );
        let mut strict = Kernel::with_default_policy(strict_policy).create_session();
        let strict_part = strict.create_part();
        let valid_settings =
            OperationSettings::new().with_tolerances(Tolerances::with_linear(1.0e-6).unwrap());
        let (strict_a, strict_b) = {
            let mut edit = strict.edit_part(strict_part.clone()).unwrap();
            let first = edit
                .create_block(
                    BlockRequest::new(Frame::world(), [2.0, 2.0, 2.0])
                        .with_settings(valid_settings.clone()),
                )
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let second = edit
                .create_block(
                    BlockRequest::new(translated_frame(5.0), [2.0, 2.0, 2.0])
                        .with_settings(valid_settings),
                )
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (first, second)
        };
        let strict_view = strict.part(strict_part).unwrap();
        assert!(matches!(
            strict_view.body_clash(BodyClashRequest::new(
                strict_a.clone(),
                strict_b.clone(),
                f64::NAN,
            )),
            Err(KernelError::Core {
                source: kcore::error::Error::InvalidGeometry {
                    reason: "body clash clearance must be finite and nonnegative"
                }
            })
        ));
        assert!(
            strict_view
                .body_clash(BodyClashRequest::new(strict_a, strict_b, 0.0))
                .is_err()
        );
    }

    #[test]
    fn zero_lower_bound_without_upper_threshold_proof_is_indeterminate() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let outer = add_block_with_extents(&mut session, &part_id, Frame::world(), [4.0, 4.0, 4.0]);
        let inner = add_block_with_extents(&mut session, &part_id, Frame::world(), [2.0, 2.0, 2.0]);
        let outcome = session
            .part(part_id)
            .unwrap()
            .body_clash(BodyClashRequest::new(outer, inner, 0.0))
            .unwrap()
            .into_result()
            .unwrap();
        let assessment = outcome.assessment().unwrap();
        assert_eq!(assessment.enclosure().lower(), 0.0);
        assert!(assessment.enclosure().upper() > 0.0);
        assert_eq!(assessment.verdict(), BodyClashVerdict::Indeterminate);
    }
}
