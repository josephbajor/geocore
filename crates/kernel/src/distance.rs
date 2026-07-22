//! Facade-safe certified distance interrogation for two solid bodies.
//!
//! A successful certificate encloses the Euclidean distance between the two
//! closed material sets, not merely between selected boundary entities.  In
//! particular, an enclosure `[0, upper]` does not distinguish overlap,
//! containment, contact, or an unresolved near separation.  Callers that need
//! that distinction must compose this query with classification or clash
//! evidence. Enclosures include the fixed incidence envelope accepted by Full
//! validation, even for topology without explicit tolerances.

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

/// Request-relative operand named by body-distance refusal evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum BodyDistanceOperand {
    /// First request operand.
    A,
    /// Second request operand.
    B,
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
        self.body(body_a.clone())?;
        self.body(body_b.clone())?;
        if body_a == body_b {
            return Err(Error::Core {
                source: kcore::error::Error::InvalidGeometry {
                    reason: "body distance requires two distinct operand bodies",
                },
            });
        }

        let defaults = ktopo::body_distance::BodyDistanceBudgetProfile::v1_defaults();
        let context = settings
            .context(self.policy)?
            .with_family_budget_defaults(defaults.clone());
        let effective = context.effective_budget();
        for required in defaults.limits() {
            effective.require_limit(required.stage, required.resource, required.mode)?;
        }

        let mut scope = OperationScope::new(&context);
        let lower = ktopo::body_distance::certify_body_distance_in_scope(
            &self.state.store,
            body_a.raw(),
            body_b.raw(),
            &mut scope,
        );
        let result = lower.map_err(Error::from).and_then(|outcome| {
            adapt_outcome(&self.id, &self.state.store, [body_a, body_b], outcome)
        });
        Ok(scope.finish_typed(result))
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
        BlockRequest, BudgetPlan, CheckOutcome, DiagnosticLevel, ExecutionPolicy, Frame, Kernel,
        KernelError, NumericalPolicy, Point3, PolicyVersion, Session, SessionPolicy,
        SessionPrecision, Tolerances, Vec3,
    };

    fn add_block(session: &mut Session, part: &PartId, frame: Frame) -> BodyId {
        session
            .edit_part(part.clone())
            .unwrap()
            .create_block(BlockRequest::new(frame, [2.0, 2.0, 2.0]))
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
}
